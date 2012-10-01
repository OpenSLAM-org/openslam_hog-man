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

#include "graph_optimizer2d_hchol.h"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <iterator>
#include <sys/time.h>
#include <stuff/os_specific.h>
#include <assert.h>
#include <list>

namespace AISNavigation{
  using namespace std;

  template <typename PG>
  bool HCholOptimizer<PG>::optimizePendingIncremental(){
    if (! _lowerOptimizer){
      HVertexSet updatedSet;
      for (typename PG::VertexIDMap::iterator it=this->vertices().begin(); it!=this->vertices().end(); it++){
	HVertex* v=dynamic_cast<HVertex*>(it->second);
	if (! v->_root){
	  updatedSet.insert(v);
	}
      }
      updateStructure(true);
      optimizeLevels(_propagateDown);
      std::set<HVertex*> topLevelSet;
      for (typename HVertexSet::iterator it =updatedSet.begin(); it!=updatedSet.end(); it++){
	HVertex* v=*it;
	postprocessIncremental(v);
	while (v){
	  if (! v->parentVertex())
	    topLevelSet.insert(v);
	  v=v->parentVertex();
	}
      }
      for (typename HVertexSet::iterator it =topLevelSet.begin(); it!=topLevelSet.end(); it++){
	HVertex* v=*it;
	v->_optimizer->propagateDownIncremental(v,1.);
      }

      HCholOptimizer<PG>* opt=this;
      int l=0;
      while(opt){
	opt=opt->_upperOptimizer;
	l++;
      }
      return true;
    }
    return false;
  }
 
  template <typename PG>
  void HCholOptimizer<PG>::postprocessIncremental(HVertex* v){
    //annotateHierarchicalEdgesIncremental(v);
    if (!_lowerOptimizer)
      propagateDownIncremental(v);
  }
  
  template <typename PG>
  void HCholOptimizer<PG>::propagateDownIncremental(HVertex* to, double lambda){
    if (! _lowerOptimizer)
      return;
    this->transformSubset(to->lowerRoot(), *(Graph::VertexSet*)(&to->children()),to->transformation);
    this->optimizeSubset(to->lowerRoot(),  *(Graph::VertexSet*)(&to->children()), _downIncrementalIterations, lambda, false);
    for (typename HVertexSet::iterator it=to->children().begin(); it!=to->children().end(); it++){
      HVertex* v=*it;
      _lowerOptimizer->propagateDownIncremental(v,lambda);
    }
  }

  template <typename PG>
  void HCholOptimizer<PG>::propagateDownIncremental(HVertex* to){
    if (! _upperOptimizer)
      return;
    _upperOptimizer->propagateDownIncremental(to->parentVertex());
    if (_upperOptimizer){
      HVertex* toParent=to->parentVertex();
      std::set<HVertex*> upperRegion;
      for (std::set<Graph::Edge*>::iterator it =toParent->edges().begin(); it!=toParent->edges().end(); it++){
	HVertex* fpv=dynamic_cast<HVertex*>((*it)->from());
	HVertex* tpv=dynamic_cast<HVertex*>((*it)->to());
	upperRegion.insert(fpv);
	upperRegion.insert(tpv);
      }
      Graph::VertexSet region;
      for (typename HVertexSet::iterator it=upperRegion.begin(); it!=upperRegion.end(); it++){
	HVertex* pv=*it;
	for(typename HVertexSet::iterator ft=pv->children().begin(); ft!=pv->children().end(); ft++){
	  region.insert(*ft);
	}
	this->transformSubset(pv->lowerRoot(), *(Graph::VertexSet*)(&pv->children()), pv->transformation);
      }
      this->optimizeSubset(toParent->lowerRoot(), region, _downIncrementalIterations, 1., false);
    }
  }

  template <typename PG>
  bool HCholOptimizer<PG>::optimizeLevels(bool propagateDown){
    if (! _upperOptimizer){

      double tGlobalOptimization=0;
      if (cachedChi()-_lastOptChi>0.){
	struct timeval ts,te;
	gettimeofday(&ts,0);
	if (this->verbose()) cerr <<"o";
	bool v=this->verbose();
	this->verbose()=false;
	CholOptimizer<PG>::optimize(_globalIncrementalIterations,false);
	_lastOptChi=this->chi2();
	_cachedChi=_lastOptChi;
	this->verbose()=v;

	gettimeofday(&te,0);
	tGlobalOptimization=1e-6*(te.tv_usec-ts.tv_usec)+te.tv_sec-ts.tv_sec;
	return true;
      }
      return false;
    }
    bool ok=_upperOptimizer->optimizeLevels(propagateDown);
    if (! ok){
      if (this->verbose()) cerr << "n";
      return false;
    }
    HVertexSet changed;
    int touched=0;
    for (typename PG::VertexIDMap::iterator it=_upperOptimizer->vertices().begin(); it!=_upperOptimizer->vertices().end(); it++){
      HVertex* parentVertex=dynamic_cast<HVertex*>(it->second);
      assert (!parentVertex->_tainted);
      typename PG::TransformationType delta=parentVertex->transformation.inverse()*parentVertex->lowerRoot()->transformation;
      typename PG::TransformationType::TranslationType pdeltaTrans = delta.translation();
      _Vector<PG::TransformationType::RotationType::Angles, double> pdeltaRot = delta.rotation().angles();
      double maxTrans = 0.0;
      for (int i = 0; i < pdeltaTrans.size(); ++i)
        if (fabs(pdeltaTrans[i]) > maxTrans)
          maxTrans = fabs(pdeltaTrans[i]);
      double maxRot = 0.0;
      for (int i = 0; i < pdeltaRot.size(); ++i)
        if (fabs(pdeltaRot[i]) > maxRot)
            maxRot = fabs(pdeltaRot[i]);
      if (maxTrans < _translationalPropagationError && maxRot < _rotationalPropagationError)
	continue;
      this->transformSubset(parentVertex->lowerRoot(), *(Graph::VertexSet*)(&parentVertex->children()), parentVertex->transformation);
      touched+=parentVertex->children().size();
      changed.insert(parentVertex);
    }
    _nVerticesPropagatedDownIncremental=changed.size();
    if (propagateDown){
      for (typename HVertexSet::iterator it=changed.begin(); it!=changed.end(); it++){
	HVertex* parentVertex=*it;
	this->optimizeSubset(parentVertex->lowerRoot(), *(Graph::VertexSet*)(&parentVertex->children()), _downIncrementalIterations, 1., false);
      } 
    }
    return true;
  }


  template <typename PG>
  void HCholOptimizer<PG>::annotateHierarchicalEdgesIncremental(HVertex* v){
    if (! _upperOptimizer || _lowerOptimizer)
      return;
  // update the hierarchy of edges up;
    HVertex* vParent=v->parentVertex();
    HCholOptimizer<PG>* opt=_upperOptimizer;
    while (opt) {
      const typename PG::EdgeSet& eset=vParent->edges();
      for (typename PG::EdgeSet::iterator it=eset.begin(); it!=eset.end(); it++){
	typename PG::Edge* e=dynamic_cast<typename PG::Edge*>(*it);	
	opt->_cachedChi-=chi2(e);
	opt->annotateHiearchicalEdge(e, _edgeAnnotationIncrementalIterations, 0., false);
	opt->_cachedChi+=chi2(e);
      }
      vParent=vParent->parentVertex();
      opt=opt->_upperOptimizer;
    }
  }


  template <typename PG>
  bool HCholOptimizer<PG>::optimizeUpperLevelIncremental(){
    if (! _upperOptimizer || _lowerOptimizer)
      return false;
    HCholOptimizer<PG>* upperOpt=this;
    while (upperOpt->_upperOptimizer){
      upperOpt=upperOpt->_upperOptimizer;
    }
    if (upperOpt->cachedChi()-upperOpt->_lastOptChi>1.){
      if (this->verbose()) cerr <<"o";
      bool v=upperOpt->verbose();
      upperOpt->verbose()=false;
      upperOpt->optimize(_globalIncrementalIterations,false);
      upperOpt->_lastOptChi=upperOpt->chi2();
      upperOpt->_cachedChi=upperOpt->_lastOptChi;
      upperOpt->verbose()=v;
      return true;
    }
    return false;
  }

  template <typename PG>
  void HCholOptimizer<PG>::updateStructure(bool incremental){
    typedef  std::set<HVertex*, Graph::VertexIDCompare> HVertexSetID;

    if (!_lowerOptimizer && _upperOptimizer){ 
      if (! incremental) {
	HCholOptimizer<PG> * opt=_upperOptimizer;
	while (opt){
	  opt->clear();
	  opt=opt->_upperOptimizer;
	}
	for (typename PG::VertexIDMap::iterator it=this->vertices().begin(); it!=this->vertices().end(); it++){
	  HVertex* v=dynamic_cast<HVertex*>(it->second);
	  v->_root=0;
	  v->_parentVertex=0;
	  v->_edgeToRoot=0;
	  v->_distanceToRoot=0;
	  v->_lowerRoot=0;
	}
      }
      cleanupTainted();
      _upperOptimizer->updateStructure(incremental);
      return;
    }
    
    typedef std::multimap<double, HVertex*> ProgressiveMap;
    // phase1 construct an associationMap
    HVertexSetID openSet;
    HVertexSetID openOldRoots;
    if (this->verbose()) cerr << "openSet: ";

    for (typename PG::VertexIDMap::iterator it=_lowerOptimizer->vertices().begin(); it!=_lowerOptimizer->vertices().end(); it++){
      HVertex* v=dynamic_cast<HVertex*>(it->second);
      if (v->_root==0){
	openSet.insert(v);
	if (_rootIDs.find(v->id())!=_rootIDs.end()){
	  //if (v->_wasRoot) {
	  openOldRoots.insert(v);
	  if (this->verbose()) cerr << v->id() << "!";
	}
	if (this->verbose()) cerr << v->id() << " ";
      }
    }


    if (this->verbose()) cerr << endl;
    if (this->verbose()) cerr << "openSet.size=" << openSet.size()<< endl;
    std::list<HVertex*> newVertices;
    if (this->verbose()) cerr << "Island Creation" << endl;
    // create the non-overlapping sets
    Dijkstra dv(_lowerOptimizer);
    UniformCostFunction cost;
    while (! openSet.empty()){
      bool resumedRoot=false;
      typename HVertexSetID::iterator openIt=openSet.begin();
      typename HVertexSetID::iterator openOldRootIt=openOldRoots.begin();
      if (! openOldRoots.empty()){
	openIt=openSet.find(*openOldRootIt);
	resumedRoot=true;
	if (openIt == openSet.end()) {
	  cerr << "fatal, vertex in the openRootSet" << (*openOldRootIt)->id() << " not in the openSet" << endl;
	}
      } else {
	openIt=openSet.begin();
      }
      HVertex* root = *openIt;

      if (root->_root){
	openSet.erase(openIt);
	if (resumedRoot)
	  openOldRoots.erase(openOldRootIt);
	continue;
      }

      HVertex* hv=dynamic_cast<HVertex*>(addVertex(root->id()));
      assert(hv);
      newVertices.push_back(hv);
      hv->_lowerRoot=root;
      hv->transformation=root->transformation;
      hv->covariance=root->covariance;
      hv->_children.insert(root);

      root->_parentVertex=hv;
      root->_distanceToRoot=0;
      root->_edgeToRoot=0;
      root->_root=root;

      openSet.erase(openIt);
      if (resumedRoot)
	openOldRoots.erase(openOldRootIt);

      dv.shortestPaths(root,&cost,_maxDistance);
      ProgressiveMap progressiveMap;
      for (Graph::VertexSet::const_iterator it=dv.visited().begin(); it!=dv.visited().end(); it++){
	HVertex* v=dynamic_cast<HVertex*>(*it);
	double d=dv.adjacencyMap().find(v)->second.distance();
	progressiveMap.insert(make_pair(d,v));
      }

      for (typename ProgressiveMap::const_iterator it=progressiveMap.begin(); it!=progressiveMap.end(); it++){
	HVertex* v=it->second;
	typename PG::Edge* edgeToRoot=dynamic_cast<typename PG::Edge*>(dv.adjacencyMap().find(v)->second.edge());
	HVertex* previousV=dynamic_cast<HVertex*>(dv.adjacencyMap().find(v)->second.parent());
	double d=it->first;
	if ((v->_root==0/* || v->_distanceToRoot>d*/) && previousV && previousV->_root==root){
	  typename HVertexSet::iterator ot=openSet.find(v);

	  assert (ot!=openSet.end());
	  openSet.erase(ot);
	  typename HVertexSet::iterator oldRootIt=openOldRoots.find(v);
	  if (oldRootIt!=openOldRoots.end())
	    openOldRoots.erase(oldRootIt);

	  v->_parentVertex=hv;
	  v->_distanceToRoot=d;
	  v->_edgeToRoot=edgeToRoot;
	  v->_root=root;
	  std::set<int>::iterator v_it=_rootIDs.find(v->id());
	  if (v_it!=_rootIDs.end())
	    _rootIDs.erase(v_it);
	  hv->_children.insert(v);
	  if (this->verbose()) cerr << v->id() << " vparent= " << hv->id() << " vprevious=" << previousV->id() << endl;
	}
      }
    }

#ifndef DNDEBUG
    if (this->verbose()) cerr << "CONSISTENCY_CHECK " << endl;
    // check that every children has a parent and that each parent contains the children in its children set;
    for (typename PG::VertexIDMap::iterator it=_lowerOptimizer->vertices().begin(); it!=_lowerOptimizer->vertices().end(); it++){
      HVertex * lv= dynamic_cast<HVertex*>(it->second);
      if (this->verbose()) cerr << "n:" << lv->id() << " p:" << lv->parentVertex()->id() << endl;
      assert(lv->parentVertex());
      assert(lv->parentVertex()->children().find(lv)!=lv->parentVertex()->children().end());
    }
#endif

    if (this->verbose())  {
      cerr << "Edges Creation" << endl;
      cerr << "newVertices.size=" << newVertices.size()<< endl;
    }

    // connect the sets
    for (typename list<HVertex*>::iterator it=newVertices.begin(); it!=newVertices.end(); it++){
      HVertex* hv=dynamic_cast<HVertex*>(*it);
      for (typename HVertexSet::iterator vt=hv->_children.begin(); vt!=hv->_children.end(); vt++){
	HVertex* cv=dynamic_cast<HVertex*>(*vt);
	for (typename PG::EdgeSet::iterator et=cv->edges().begin(); et!=cv->edges().end(); et++){
	  typename PG::Edge* e=dynamic_cast<typename PG::Edge*>(*et);
	  HVertex* cv1=dynamic_cast<HVertex*>(e->from());
	  HVertex* cv2=dynamic_cast<HVertex*>(e->to());
	  HVertex* pv1=cv1->parentVertex();
	  HVertex* pv2=cv2->parentVertex();
	  assert(pv1 && pv2 && (pv1==hv || pv2==hv) );
	  if (pv1!=pv2 && this->connectingEdges(pv1,pv2).empty() && this->connectingEdges(pv2,pv1).empty()){
	    typename PG::InformationType info = PG::InformationType::eye(1.);
            int rotDim = PG::TransformationType::RotationType::Dimension;
            assert(rotDim + PG::TransformationType::RotationType::Angles == info.rows());
            for (int i = rotDim; i < info.rows(); ++i)
              info[i][i]=5;
	    typename PG::TransformationType mean=pv1->transformation.inverse()*pv2->transformation;
	    typename PG::Edge* en=addEdge( pv1, pv2,  mean, info);
	    annotateHiearchicalEdge(en, _edgeAnnotationIncrementalIterations, 0., false);
	  }
	}
      }
    }
    if (_upperOptimizer)
      _upperOptimizer->updateStructure(incremental);
  }

}
