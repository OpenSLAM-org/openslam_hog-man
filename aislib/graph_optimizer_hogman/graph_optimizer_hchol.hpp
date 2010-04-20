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
#include <stuff/macros.h>
#include <stuff/os_specific.h>
#include <assert.h>


namespace AISNavigation{
  using namespace std;

  template <typename PG>
  HCholOptimizer<PG>::HCholOptimizer(double maxDistance){
    _lowerOptimizer=0;
    _upperOptimizer=0;
    _cachedChi=0.;
    _lastOptChi=0;
    _online=false;
    _maxDistance=_maxDistance;
    _gnuplot=false;

    _translationalPropagationError=0.05;
    _rotationalPropagationError=0.05;

    _globalIncrementalIterations=5;
    _downIncrementalIterations=2;
    _edgeAnnotationIncrementalIterations=5;
    _propagateDown=false;
  }

  template <typename PG>
  HCholOptimizer<PG>::HCholOptimizer(HCholOptimizer<PG>* ll, double maxDistance){
    _upperOptimizer=0;
    _lowerOptimizer=ll;
    _cachedChi=1.;
    _lastOptChi=1.;
    HCholOptimizer<PG> *lho=dynamic_cast<HCholOptimizer<PG> *>(ll);
    if (lho)
      lho->_upperOptimizer=this;
    _maxDistance=maxDistance;
    _gnuplot=false;
    _online=false;

    _translationalPropagationError=0.1;
    _rotationalPropagationError=0.05;
    _globalIncrementalIterations=3;
    _downIncrementalIterations=3;
    _edgeAnnotationIncrementalIterations=5;
    _propagateDown=false;
  }

  template <typename PG>
  HCholOptimizer<PG>::HCholOptimizer(int nLevels, double maxDistance){
    _lowerOptimizer=0;
    _upperOptimizer=0;
    _cachedChi=0.;
    _lastOptChi=0;
    _online=false;
    _maxDistance=_maxDistance;
    _gnuplot=false;
    HCholOptimizer<PG>* lopt=this;
    for (int i=1; i<nLevels; i++){
      lopt=new HCholOptimizer<PG>(lopt, maxDistance);
    }

    _translationalPropagationError=0.1;
    _rotationalPropagationError=0.05;
    _globalIncrementalIterations=3;
    _downIncrementalIterations=3;
    _edgeAnnotationIncrementalIterations=5;
    _propagateDown=false;
  }
  
  template <typename PG>
  HCholOptimizer<PG>::~HCholOptimizer(){
    if (_upperOptimizer){
      delete _upperOptimizer;
      _upperOptimizer=0;
    }
  }

  template <typename PG>
  void HCholOptimizer<PG>::clear(){
    if (_upperOptimizer)
      _upperOptimizer->clear();
    PG::clear();
    _rootIDs.clear();
  }


  template <typename PG>
  int HCholOptimizer<PG>::nLevels() const {
    assert(! _lowerOptimizer);
    const HCholOptimizer<PG>* opt=this;
    int l=0;
    while (opt){
      opt=opt->_upperOptimizer;
      l++;
    }
    return l;
  }
  
  template <typename PG>
  HCholOptimizer<PG>* HCholOptimizer<PG>::level(int i) {
    HCholOptimizer<PG>* opt=this;
    while (i!=0 && opt){
      if (i<0){
	opt=opt->_lowerOptimizer;
	i++;
      } else {
	opt=opt->_upperOptimizer;
	i--;
      }
    }
    return opt;
  }
  
  template <typename PG>
  void HCholOptimizer<PG>::annotateHiearchicalEdge(typename PG::Edge* e, int iterations, double lambda, bool initWithObservations){
    if (!_lowerOptimizer)
      return;
    HVertex* from = dynamic_cast< HVertex* >( e->from() );
    HVertex* to = dynamic_cast< HVertex* >( e->to() );
    assert (from && to);
    Graph::VertexSet jointSet;
    std::set_union(from->children().begin(),
		   from->children().end(),
		   to->children().begin(),
		   to->children().end(), 
		   std::insert_iterator<Graph::VertexSet>(jointSet, jointSet.end()));
    typename PG::InformationType covariance = PG::InformationType::eye(1.0);
    int otherId=to->id();
    _lowerOptimizer->backupSubset(jointSet);
//     cerr << __PRETTY_FUNCTION__ << ": js= "; 
//     for (VertexSet::iterator it=jointSet.begin(); it!=jointSet.end(); it++){
//       cerr << (*it)->id() << " ";
//     }
//     cerr << endl;
//     cerr << __PRETTY_FUNCTION__ << ": root= " << from->lowerRoot()->id() << endl; 
    //_lowerOptimizer->transformSubset(from->lowerRoot(), jointSet, typename PG::TransformationType());
    _lowerOptimizer->optimizeSubset(from->lowerRoot(), jointSet, iterations, lambda, initWithObservations, otherId, &covariance);
    typename PG::TransformationType mean=from->lowerRoot()->transformation.inverse()*to->lowerRoot()->transformation;
    _lowerOptimizer->restoreSubset(jointSet);
    if (this->verbose()){
      cerr << "u[" << jointSet.size() << "] ";
    }

    if (covariance.det()<0){
      cerr << "![" << covariance.det() << "] " << endl;
      covariance = typename PG::InformationType().eye(1.)*1e9;
    } else 
      covariance=covariance.inverse();

    refineEdge(e, mean, covariance);
  }

  template <typename PG>
  void HCholOptimizer<PG>::annotateHiearchicalEdgeOnDenseGraph(typename PG::TransformationType& mean, typename PG::InformationType& info,
      typename PG::Edge* e, int iterations, double lambda, bool initWithObservations){
    if (!_lowerOptimizer)
      return;
    assert (! _upperOptimizer);

    HVertex* from = dynamic_cast< HVertex* >( e->from() );
    HVertex* to = dynamic_cast< HVertex* >( e->to() );

    assert (from && to);
    HVertex* fromAux = dynamic_cast< HVertex* >( e->from() );
    HVertex* toAux = dynamic_cast< HVertex* >( e->to() );
    HCholOptimizer<PG>* optAux=this;

    Graph::VertexSet jointSet;
    jointSet.insert(fromAux);
    jointSet.insert(toAux);
    int level=0;
    while (optAux->_lowerOptimizer) {
      //	cerr << "L=" << level << " V=";
      Graph::VertexSet jointAuxSet;
      for (Graph::VertexSet::iterator it=jointSet.begin(); it!=jointSet.end(); it++){
        HVertex* v = dynamic_cast< HVertex* >(*it);
        for (typename HVertexSet::iterator it=v->children().begin(); it!=v->children().end(); it++){
          //  cerr << (*it)->id() << " ";
          jointAuxSet.insert(*it);
        }
      } 
      //cerr << endl;
      fromAux=dynamic_cast<HVertex*>(fromAux->lowerRoot());
      toAux=dynamic_cast<HVertex*>(toAux->lowerRoot());
      optAux=optAux->_lowerOptimizer;
      jointSet=jointAuxSet;
      level++;
    }
    //cerr << "DONE " << jointSet.size() << ": ";
    for (Graph::VertexSet::iterator it=jointSet.begin(); it!=jointSet.end(); it++){
      //cerr << (*it)->id() << " ";
    }
    //cerr << endl;
    assert (toAux->_optimizer==optAux);

    typename PG::InformationType covariance;
    int otherId=toAux->id();
    optAux->backupSubset(jointSet);
    optAux->optimizeSubset(fromAux, jointSet, iterations, lambda, initWithObservations, otherId, &covariance);
    mean=fromAux->transformation.inverse()*toAux->transformation;
    optAux->restoreSubset(jointSet);
    if (covariance.det()<0){
      cerr << "![" << covariance.det() << "] " << endl;
      info=typename PG::InformationType::eye(1e9);
    } else 
      info=covariance.inverse();
  }
  
  template <typename PG>
  void HCholOptimizer<PG>::computeTopLevelDenseGraph(CholOptimizer<PG>* chol, int iterations, int lambda, int initWithObservations){
    if (_upperOptimizer){
      _upperOptimizer->computeTopLevelDenseGraph(chol, iterations, lambda, initWithObservations);
      return;
    }
    chol->clear();
    for (typename PG::VertexIDMap::iterator it=this->vertices().begin(); it!=this->vertices().end(); it++){
      HVertex* v = dynamic_cast< HVertex* >(it->second);
      typename PG::Vertex* vc= chol->addVertex(v->id());
      vc->transformation=v->transformation;
    }
    for (typename PG::EdgeSet::iterator it=this->edges().begin(); it!=this->edges().end(); it++){
      typename PG::Edge* e=dynamic_cast<typename PG::Edge*>(*it);
      typename PG::TransformationType mean;
      typename PG::InformationType info;
      annotateHiearchicalEdgeOnDenseGraph(mean, info, e, iterations, lambda, initWithObservations);
      typename PG::Vertex* from=dynamic_cast<typename PG::Vertex*>(chol->vertex(e->from()->id()));
      typename PG::Vertex* to=dynamic_cast<typename PG::Vertex*>(chol->vertex(e->to()->id()));
      chol->addEdge(from, to, mean, info);
    } 
//     for (VertexIDMap::iterator it=vertices().begin(); it!=vertices().end(); it++){
//       HVertex* v = dynamic_cast< HVertex* >(it->second);
//       Vertex* vc = dynamic_cast< Vertex* >(chol->vertex(v->id()));
//       vc->transformation=v->transformation;
//     }
 }

  template <typename PG>
  int HCholOptimizer<PG>::optimize(int iterations, bool online){
    if (! _upperOptimizer){
      CholOptimizer<PG>::optimize(iterations, online);
      return iterations;
    }
    assert(!_lowerOptimizer);
    if (! online){
      _online=false;
      updateStructure(false);
      HCholOptimizer<PG>* uopt=this;
      while(uopt->_upperOptimizer){
	uopt=uopt->_upperOptimizer;
      }
      for (int i=0; i<iterations; i++){
	uopt->vCycle(3,0.,1.,false);
      }
    } else {
      optimizePendingIncremental();
    }
    return 1;
  }

}
