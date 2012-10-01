// HOG-Man - Hierarchical Optimization for Pose Graphs on Manifolds
// Copyright (C) 2010 G. Grisetti, R. Kümmerle, C. Stachniss
// 
// HOG-Man is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// HOG-Man is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <iostream>
#include <iomanip>
#include <algorithm>
#include <iterator>
#include <sys/time.h>
#include <stuff/os_specific.h>
#include <stuff/color_macros.h>
#include <assert.h>
#include "csparse_helper.h"

namespace AISNavigation{
  using namespace std;

  template <typename PG>
  bool CholOptimizer<PG>::initialize(int rootNode){
    if (this->verbose())
      cerr << "# init " << rootNode << endl;
    if (this->vertex(rootNode)){
      _rootNode=rootNode;
      return true;
    }
    _rootNode=-1;
    _ivMap.clear();
    _activeEdges.clear();
    return false;
  }
  
  template <typename PG>
  int CholOptimizer<PG>::optimize(int iterations, bool online){
    Graph::VertexSet vset;
    for (Graph::VertexIDMap::const_iterator it=this->vertices().begin(); it!=this->vertices().end(); it++){
      vset.insert(it->second);
    }
    
    typename PG::Vertex* root=dynamic_cast<typename PG::Vertex*>(this->vertex(_rootNode));
    if (! root)
      root=_MY_CAST_<typename PG::Vertex*>(this->vertices().begin()->second);
    if (this->verbose())
      cerr << "# root id " << root->id() << endl;
    bool initFromObservations = _guessOnEdges;
    optimizeSubset(root, vset, iterations, 0., initFromObservations);
    return iterations;
  }

  template <typename PG>
  CholOptimizer<PG>::~CholOptimizer<PG>(){
    if (_sparseB)
      delete [] _sparseB;
    if (_sparseMatrix)
      delete [] _sparseMatrix;
    delete[]_sparseMatrixPtr; _sparseMatrixPtr = 0;
    cs_sfree(_symbolicCholesky); _symbolicCholesky = 0;
    delete[] _csWorkspace; _csWorkspace = 0;
    delete[] _csInvWorkB; _csInvWorkB = 0;
    delete[] _csInvWorkTemp; _csInvWorkTemp = 0;
    delete[] _csIntWorkspace; _csIntWorkspace = 0;
  }

  template <typename PG>
  int CholOptimizer<PG>::optimizeSubset(typename PG::Vertex* rootVertex, Graph::VertexSet& vset, int iterations, double lambda, bool initFromObservations,
      int otherNode, typename PG::InformationType* otherCovariance)
  {
    if (vset.size() <= 1) {
      return 0;
    }
    if (!buildIndexMapping(rootVertex,vset)){
      return 0;
    }
    
    // clean up from last call
    if (_symbolicCholesky) {
      cs_sfree(_symbolicCholesky);
      _symbolicCholesky = 0;
    }

    computeActiveEdges(rootVertex,vset);

    if (initFromObservations){
      initializeActiveSubsetWithObservations(rootVertex);
      if (this->verbose()){
        cerr << "iteration= " << -1 
          << "\t chi2= " << this->chi2() 
          << "\t time= " << 0.0
          << "\t cumTime= " << 0.0
          << endl;
      }
    }

    int dim = PG::TransformationVectorType::TemplateSize;
    int cjIterations=0;
    double cumTime=0;
    for (int i=0; i<iterations; i++){
      struct timeval ts, te;
      gettimeofday(&ts,0);
      buildLinearSystem(rootVertex,lambda);

      if (i == 0) {
        // we have to sort the matrix structure only within the first iteration, it stays the same for the following iterations
        SparseMatrixEntry* entry = _sparseMatrix;
        for (int j = 0; j < _sparseNz; ++j) { // store the pointer to the array
          _sparseMatrixPtr[j] = entry;
          ++entry;
        }
        std::sort(_sparseMatrixPtr, _sparseMatrixPtr + _sparseNz, SparseMatrixEntryPtrCmp());
      }

      if (otherNode==-1 || i!=iterations-1){
	solveAndUpdate();
      } else {
	double** pblock = new double*[dim];
        for (int k = 0; k < dim; ++k)
          pblock[k] = (*otherCovariance)[k];
	typename PG::Vertex* otherVertex=_MY_CAST_<typename PG::Vertex*>(this->vertex(otherNode));
	int j=otherVertex->tempIndex()*dim;
	solveAndUpdate(pblock, j,j,j+dim,j+dim);
        static TransformCovariance<PG> tCov;
        tCov(*otherCovariance, rootVertex->transformation, otherVertex->transformation);
	assert(otherCovariance->det()>0.);
      }
      gettimeofday(&te,0);
      double dts=(te.tv_sec-ts.tv_sec)+1e-6*(te.tv_usec-ts.tv_usec);
      cumTime+=dts;
      if (this->verbose()){
        cerr << "iteration= " << i 
          << "\t chi2= " << this->chi2() 
          << "\t time= " << dts 
          << "\t cumTime= " << cumTime
          << endl;
      }
      if (this->visualizeToStdout())
	this->visualizeToStream(cout);
      ++cjIterations;

    }
    clearIndexMapping();

    return cjIterations;
  }


  template <typename PG>
  CholOptimizer<PG>::CholOptimizer(){
    _sparseB=0;
    _sparseDim=0;
    _sparseNz=0;
    _nBlocks=0;
    _sparseMatrix=0;
    _sparseMatrixPtr = 0;
    _sparseDimMax=0;
    _sparseNzMax=0;
    _symbolicCholesky = 0;
    _csWorkspace = 0;
    _csIntWorkspace = 0;
    _csWorkspaceSize = -1;
    _csInvWorkspaceSize = -1;
    _csInvWorkB = 0;
    _csInvWorkTemp = 0;
    _useRelativeError=true;
    _rootNode=-1;
    _addDuplicateEdgeIterations = 3;
  }

  template <typename PG>
  bool CholOptimizer<PG>::buildIndexMapping(typename PG::Vertex* rootVertex, Graph::VertexSet& vset){
    _ivMap.resize(vset.size());
    int i=0;
    for (Graph::VertexSet::iterator it=vset.begin(); it!=vset.end(); it++){
      typename PG::Vertex* v=_MY_CAST_<typename PG::Vertex*>(*it);
      if (v!=rootVertex && ! v->fixed()){
	v->tempIndex()=i;
	_ivMap[i]=v;
	i++;
      } 
    }
    _ivMap.resize(i);
    return true;
  }

  template <typename PG>
  void CholOptimizer<PG>::clearIndexMapping(){
    for (int i=0; i<(int)_ivMap.size(); i++){
      _ivMap[i]->tempIndex()=-1;
      _ivMap[i]=0;
    }
  }

  template <typename PG>
  void CholOptimizer<PG>::computeActiveEdges(typename PG::Vertex* rootVertex, Graph::VertexSet& vset){
    _activeEdges.clear();
    for (int i=0; i<(int)_ivMap.size(); i++){
      typename PG::Vertex* v=_ivMap[i];
      const typename PG::EdgeSet& vEdges=v->edges();
      for (typename PG::EdgeSet::const_iterator it=vEdges.begin(); it!=vEdges.end(); it++){
	_activeEdges.insert(reinterpret_cast<typename PG::Edge*>(*it));
      }
    }
    const typename PG::EdgeSet& vEdges=rootVertex->edges();
    for (typename PG::EdgeSet::const_iterator it=vEdges.begin(); it!=vEdges.end(); it++){
      _activeEdges.insert(reinterpret_cast<typename PG::Edge*>(*it));
    }
  }


  template <typename PG>
  int CholOptimizer<PG>::linearizeConstraint(const typename PG::Edge* e, double lambda){
      typename PG::TransformationVectorType f;
      typename PG::InformationType A, B;
      if (_useRelativeError){
	static ManifoldGradient<PG> gradient;
	gradient(f,A,B,*e);
      } else {
        static Gradient<PG> gradient;
	gradient(f,A,B,*e);
      }
      typename PG::InformationType omega=e->information();
      typename PG::TransformationVectorType r=f*(-1.);

      const typename PG::Vertex* from=_MY_CAST_<const typename PG::Vertex*>(e->from());
      const typename PG::Vertex* to=_MY_CAST_<const typename PG::Vertex*>(e->to());

      int i=from->tempIndex();
      if(i==-1){
	A = PG::InformationType::eye(1.);
      }

      int j=to->tempIndex();
      if(j==-1){
	B = PG::InformationType::eye(1.);
      }

      if (from->fixed() || to->fixed())
	lambda=1.;
      if (i==-1 || j==-1)
	omega=omega*lambda;
      if (i!=-1){
	typename PG::TransformationVectorType bi=A.transpose()*(omega*r);
	typename PG::InformationType Aii = A.transpose()*omega*A;
	from->b()+=bi;
	from->A()+=Aii;
      }
      if (j!=-1){
	typename PG::TransformationVectorType bj=B.transpose()*(omega*r);      
	typename PG::InformationType Ajj = B.transpose()*omega*B;
	to->b()+=bj;
	to->A()+=Ajj;
      }
      if (i!=-1 && j!=-1){
	typename PG::InformationType Aij = A.transpose()*omega*B;
	e->AFromTo()+=Aij;
	return 2;
      }
      return 0;
  }

  template <typename PG>
  typename PG::Edge* CholOptimizer<PG>::addEdge(typename PG::Vertex* from, typename PG::Vertex* to, const typename PG::TransformationType& mean, const typename PG::InformationType& information){
    Graph::EdgeSet eset1=this->connectingEdges(from, to);
    Graph::EdgeSet eset2=this->connectingEdges(to, from);
    Graph::EdgeSet eset;
    std::set_union(eset1.begin(),
		   eset1.end(),
		   eset2.begin(),
		   eset2.end(), 
		   std::insert_iterator<Graph::EdgeSet>(eset, eset.end()));

    if (eset.empty()){
      typename PG::Edge* e = PG::addEdge(from, to, mean, information);
      if (_guessOnEdges && to->edges().size()==1 && ! to->fixed()){
	to->transformation=from->transformation*mean;
      }
      return e;
    }
    assert(eset.size()==1);

    // least square estimate of the new and the old edge to get one edge between the two nodes
    typename PG::Edge*   origEdge = dynamic_cast<typename PG::Edge*>(*eset.begin());
    typename PG::Vertex* origTo   = dynamic_cast<typename PG::Vertex*>(origEdge->to());
    typename PG::Vertex* origFrom = dynamic_cast<typename PG::Vertex*>(origEdge->from());
    CholEdge auxEdge(from, to, mean, information);
    // backup the two vertices and reset the transformation to the old mean
    origFrom->backup();
    origTo->backup();
    origFrom->transformation = typename PG::TransformationType();
    origTo->transformation = origFrom->transformation * origEdge->mean();

    static ManifoldGradient<PG> gradient;
    typename PG::TransformationVectorType f;
    typename PG::InformationType A, B;
    typename PG::InformationType sysMat;
    typename PG::TransformationVectorType rightHand;
    for (int iteration = 0; iteration < _addDuplicateEdgeIterations; ++iteration) {
      sysMat.fill(0.);
      rightHand.fill(0.);

      // original edge
      gradient(f, A, B, *origEdge); f *= -1;
      rightHand += B.transpose() * (origEdge->information() * f);
      sysMat    += B.transpose() * origEdge->information() * B;

      // new edge
      gradient(f, A, B, auxEdge); f *= -1;
      if (origTo == to) {
        rightHand += B.transpose() * (auxEdge.information() * f);
        sysMat    += B.transpose() * auxEdge.information() * B;
      } else {
        rightHand += A.transpose() * (auxEdge.information() * f);
        sysMat    += A.transpose() * auxEdge.information() * A;
      }
      
      // solving the system and updating
      sysMat = sysMat.inverse(); // sysmat is now the covariance
      rightHand = sysMat * rightHand;
      //cerr << "update is " << rightHand << endl;
      static PoseUpdate<PG> poseUpdate;
      poseUpdate(origTo->transformation, &rightHand[0]);
    }

    typename PG::TransformationType newMean = origFrom->transformation.inverse() * origTo->transformation;
    static TransformCovariance<PG> tCov;
    tCov(sysMat, origFrom->transformation, origTo->transformation);
    typename PG::InformationType newInfo = sysMat.inverse();
    origFrom->restore();
    origTo->restore();

    if (0 && this->verbose()) {
      cerr << CL_GREEN(__PRETTY_FUNCTION__ << ": updating old edge with new measurement") << endl;
      cerr << "old mean: " << origEdge->mean().toVector() << endl;
      cerr << "new mean: " << (to==origTo ? mean.toVector() : mean.inverse().toVector()) << endl;
      cerr << " -> mean: " << newMean.toVector() << endl << endl;
      cerr << "old information:\n" << origEdge->information() << endl;
      cerr << "new information:\n" << information << endl;
      cerr << " -> information:\n" << newInfo << endl;
    }

    this->refineEdge(origEdge, newMean, newInfo);
    return origEdge;

  }


  template <typename PG>
  void CholOptimizer<PG>::buildLinearSystem(typename PG::Vertex* rootVertex, double lambda){
    for (int i=0; i<(int)_ivMap.size(); i++){
      typename PG::Vertex* v=_ivMap[i];
      assert(v);
      assert(v!=rootVertex);
      v->b().fill(0.0);
      v->A().fill(0.0);
    }
    // compute the terms for the pairwise constraints
    for (typename std::set<typename PG::Edge*>::const_iterator it=_activeEdges.begin(); it!=_activeEdges.end(); it++){
      const typename PG::Edge* e=*it;
      e->AFromTo().fill(0.0);
    }
    int blockCount=0;
    for (typename set<typename PG::Edge*>::const_iterator it=_activeEdges.begin(); it!=_activeEdges.end(); it++){
      const typename PG::Edge* e=*it;
      double l=lambda;
      if (e->from()==rootVertex || e->to()==rootVertex)
        l=1;
      blockCount+= linearizeConstraint(e, l);
    }

    int dim = PG::TransformationVectorType::TemplateSize;
    assert(dim > 0);

    blockCount+=_ivMap.size();

    _sparseDim=_ivMap.size()*dim;
    _nBlocks=blockCount;
    _sparseNz=0;

    int sparseNzCurr=_nBlocks*dim*dim;
    if (_sparseDim>_sparseDimMax){
      if (_sparseB ) {
	delete [] _sparseB;
        _sparseB = 0;
      }
      _sparseDimMax=_sparseDim*2;
      _sparseB = new double [_sparseDimMax];
      for (int i = 0; i < _sparseDimMax; ++i)
        _sparseB[i] = 0;
    }
    if (sparseNzCurr>_sparseNzMax){
      if (_sparseMatrix)
        delete [] _sparseMatrix;
      _sparseNzMax=2*sparseNzCurr;
      _sparseMatrix = new SparseMatrixEntry[_sparseNzMax];
      delete[] _sparseMatrixPtr;
      _sparseMatrixPtr = new SparseMatrixEntry*[_sparseNzMax];
    }

    
    SparseMatrixEntry* entry=_sparseMatrix;
    int nz=0;
    for (int i=0; i<(int)_ivMap.size(); i++){
      typename PG::Vertex* v=_ivMap[i];
      assert(v);
      int iBase=i*dim;
      for (int j=0; j<dim; j++)
	_sparseB[iBase+j]=v->b()[j];
      typename PG::InformationType& Ai=v->A();
      for (int j=0; j<dim; j++)
	for (int k=0; k<dim; k++){
	  int r=iBase+j;
	  int c=iBase+k;
	  double v=Ai[j][k];
	  entry->set(r,c,v);
	  nz++;
	  entry++;
	}
    }
    for (typename std::set<typename PG::Edge*>::const_iterator it=_activeEdges.begin();
        it!=_activeEdges.end();
        it++){
      const typename PG::Edge* e=*it;
      const typename PG::Vertex* from=_MY_CAST_<const typename PG::Vertex*>(e->from());
      const typename PG::Vertex* to=_MY_CAST_<const typename PG::Vertex*>(e->to());
      typename PG::InformationType Aij=e->AFromTo();

      int i=from->tempIndex();
      int j=to->tempIndex();
      if (i==-1 || j==-1)
	continue;

      for (int symm=0; symm<2; symm++){
	int iBase=i*dim;
	int jBase=j*dim;
	if (symm){
	  Aij.transposeInPlace();
	  iBase=j*dim;
	  jBase=i*dim;
	}
	for (int q=0; q<dim; q++)
	  for (int k=0; k<dim; k++){
	    int r=iBase+q;
	    int c=jBase+k;
	    double v=Aij[q][k];
	    entry->set(r,c,v);
	    nz++;
	    entry++;
	  }
      }
    }
    _sparseNz=nz;
    //if (_sparseNz!=_nBlocks*dim*dim){
      //cerr << "FATAL: _sparseNz=" <<  _sparseNz << " nBlocks*9=" << _nBlocks*9 << "sparseNzCurr" << sparseNzCurr << endl;
    //}
    assert (_sparseNz==_nBlocks*dim*dim);
  }


  template <typename PG>
  void CholOptimizer<PG>::solveAndUpdate(double** block, int r1, int c1, int r2, int c2){
    //cerr << "csparse_create"<< endl;
    struct cs_sparse *_csA=SparseMatrixEntryPtrVector2CSparse(_sparseMatrixPtr, _sparseDim, _sparseDim, _sparseNz);
    //cs_print(_csA, 0);
    struct cs_sparse *_ccsA=cs_compress(_csA);
    //cerr << "solving";
    
    // perform symbolic cholesky once
    if (_symbolicCholesky == 0) {
      _symbolicCholesky = cs_schol (1, _ccsA) ;
      if (!_symbolicCholesky) {
        cerr << "Symbolic cholesky failed" << endl;
      }
    }
    // re-allocate the temporary workspace for cholesky
    if (_csWorkspaceSize < _ccsA->n) {
      _csWorkspaceSize = 2 * _ccsA->n;
      delete[] _csWorkspace;
      _csWorkspace = new double[_csWorkspaceSize];
      delete[] _csIntWorkspace;
      _csIntWorkspace = new int[2*_csWorkspaceSize];
    }

    int ok=0;
    if (! block){
      ok = cs_cholsolsymb(_ccsA, _sparseB, _symbolicCholesky, _csWorkspace, _csIntWorkspace);
    } else {
      // re-allocate the temporary workspace for cholesky
      if (_csInvWorkspaceSize < _ccsA->n) {
        _csInvWorkspaceSize = 2 * _ccsA->n;
        delete[] _csInvWorkB;
        _csInvWorkB = new double[_csInvWorkspaceSize];
        delete[] _csInvWorkTemp;
        _csInvWorkTemp = new double[_csInvWorkspaceSize];
      }
      ok = cs_cholsolinvblocksymb(_ccsA, block, r1, c1, r2, c2, _sparseB,
          _symbolicCholesky, _csWorkspace, _csInvWorkB, _csInvWorkTemp, _csIntWorkspace);
    }
    
    if (! ok) {
      cerr << "***** FAILURE *****" << endl;
      ofstream failed("failed.graph");
      this->save(failed);
      abort();
    }
    cs_spfree(_ccsA);
    cs_spfree(_csA);

    int dim = PG::TransformationVectorType::TemplateSize;
    int position=0;
    double* update = _sparseB;
    static PoseUpdate<PG> poseUpdate;
    for (int i=0; i<_sparseDim; i += dim) {
      typename PG::Vertex* v= _ivMap[position];
      poseUpdate(v->transformation, update);
      update += dim;
      position++;
    }
  }


  template <typename PG>
  void CholOptimizer<PG>::transformSubset(typename PG::Vertex* rootVertex, Graph::VertexSet& vset, const typename PG::TransformationType& newRootPose){
    typename PG::TransformationType t=newRootPose*rootVertex->transformation.inverse();
    for (Graph::VertexSet::iterator it=vset.begin(); it!=vset.end(); it++){
      typename PG::Vertex* v=_MY_CAST_<typename PG::Vertex*>(*it);
      v->transformation=t*v->transformation;
    }
  }
 

  template <typename PG>
  struct ActivePathUniformCostFunction: public PG::PathLengthCostFunction{
    ActivePathUniformCostFunction(CholOptimizer<PG>* optimizer);
    virtual double operator()(Graph::Edge* edge, Graph::Vertex* from, Graph::Vertex* to);
  protected:
    CholOptimizer<PG>* _optimizer;
  };
  
  template <typename PG>
  ActivePathUniformCostFunction<PG>::ActivePathUniformCostFunction(CholOptimizer<PG>* optimizer){
    _optimizer=optimizer;
  }
    
  template <typename PG>
  double ActivePathUniformCostFunction<PG>::operator()(Graph::Edge* edge, Graph::Vertex* from, Graph::Vertex* to){
    typename PG::Edge* e = dynamic_cast<typename PG::Edge*>(edge);
    typename std::set<typename PG::Edge*>::iterator it=_optimizer->_activeEdges.find(e);
    if (it==_optimizer->_activeEdges.end())
      return std::numeric_limits<double>::max();
    return 1.;
    typename PG::TransformationType::TranslationType t=e->mean().translation();
    return sqrt(t*t);
  }

  template <typename PG>
  void CholOptimizer<PG>::initializeActiveSubsetWithObservations(typename PG::Vertex* root, double maxDistance){
    assert(root);
    Dijkstra dv(this);
    std::map<const typename PG::Vertex*, typename PG::TransformationType> tempT;
    for (typename std::set<typename PG::Edge*>::iterator it=_activeEdges.begin(); it!=_activeEdges.end(); it++){
      typename PG::Vertex* from=static_cast<typename PG::Vertex*>((*it)->from());
      typename PG::Vertex* to=static_cast<typename PG::Vertex*>((*it)->to());
      tempT[from]=from->transformation;
      tempT[to]=to->transformation;
    }
    ActivePathUniformCostFunction<PG> apl(this);
    dv.shortestPaths(root,&apl,maxDistance);
    this->propagateAlongDijkstraTree(root, dv.adjacencyMap(), false, true); 
    for (typename std::set<typename PG::Edge*>::iterator it=_activeEdges.begin(); it!=_activeEdges.end(); it++){
      typename PG::Vertex* from=static_cast<typename PG::Vertex*>((*it)->from());
      typename PG::Vertex* to=static_cast<typename PG::Vertex*>((*it)->to());
      if (from->tempIndex()==-1)
	from->transformation=tempT[from];
      if (to->tempIndex()==-1)
	to->transformation=tempT[to];
    }
  }

template <typename PG>
void CholOptimizer<PG>::storeVertices()
{
  for (typename std::vector<typename PG::Vertex*>::iterator it = _ivMap.begin(); it != _ivMap.end(); ++it) {
    typename PG::Vertex* v = *it;
    v->storedTransformation = v->transformation;
  }
}

template <typename PG>
void CholOptimizer<PG>::restoreVertices()
{
  for (typename std::vector<typename PG::Vertex*>::iterator it = _ivMap.begin(); it != _ivMap.end(); ++it) {
    typename PG::Vertex* v = *it;
    v->transformation = v->storedTransformation;
  }
}

} // end namespace
