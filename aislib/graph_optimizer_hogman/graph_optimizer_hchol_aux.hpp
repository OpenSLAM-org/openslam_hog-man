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


namespace AISNavigation{

  using namespace std;

  template <typename PG>
  HCholOptimizer<PG>::HVertex::HVertex(int id):
    PG::Vertex(id){
    _root=0;
    _lowerRoot=0;
    _parentVertex=0;
    _edgeToRoot=0;
    _distanceToRoot=0;
    _tainted=false;
  }

  template <typename PG>
  HCholOptimizer<PG>::HVertex::~HVertex(){
    if (_parentVertex){
      typename HVertexSet::iterator selfIt=_parentVertex->_children.find(this);
      assert(selfIt!=_parentVertex->_children.end());
      if (selfIt!=_parentVertex->_children.end()){
	_parentVertex->_children.erase(selfIt);
      }
    }    
    detachChildren();
  }

  template <typename PG>
  void HCholOptimizer<PG>::HVertex::detachChildren(){
    for (typename HVertexSet::iterator it=children().begin(); it!=children().end(); it++){
	HVertex* v=*it;
	if (v->_root==v)
	  v->_optimizer->_rootIDs.insert(v->id());
	v->detachChildren();
	v->_root=0;
	v->_lowerRoot=0;
	v->_parentVertex=0;
	v->_edgeToRoot=0;
	v->_distanceToRoot=0;
    }
    _lowerRoot=0;
    _children.clear();
  }

  template <typename PG>
  void HCholOptimizer<PG>::HVertex::taint(){
    // std::cerr << __PRETTY_FUNCTION__ << this << " id:" << _id << " opt:" << _optimizer <<  " parent=" << _parentVertex << endl;
    if (_optimizer->_lowerOptimizer)
      _tainted=true;
    if (_parentVertex)
      _parentVertex->taint();
  }

  template <typename PG>
  typename PG::Vertex* HCholOptimizer<PG>::addVertex(const int& k){
    HVertex* v=new HVertex(k);
    v->_optimizer=this;
    HVertex* vresult=dynamic_cast<HVertex*>(Graph::addVertex(v));
    if (!vresult){
      delete v;
    }
    return vresult;
  }
  
  template <typename PG>
  typename PG::Vertex* HCholOptimizer<PG>::addVertex(int id, const typename PG::TransformationType& pose, const typename PG::InformationType& information){
    typename PG::Vertex* v=new HVertex(id);
    HVertex* vresult=dynamic_cast<HVertex*>(Graph::addVertex(v));
    vresult->_optimizer=this;
    if (!vresult){
      delete v;
      return vresult;
    }
    vresult->transformation=pose;
    vresult->covariance=information.inverse();
    return vresult;
  }

  template <typename PG>
  typename PG::Edge*   HCholOptimizer<PG>::addEdge(typename PG::Vertex* from, typename PG::Vertex* to,
      const typename PG::TransformationType& mean, const typename PG::InformationType& information){
    typename PG::Edge* e=0;
    if (!_lowerOptimizer){
      e=CholOptimizer<PG>::addEdge(from, to, mean, information);
      if (to->edges().size()==1){
	to->transformation=from->transformation*mean;
      }
      HVertex* hFrom=dynamic_cast<HVertex*>(from);
      hFrom->taint();
    } else
      e=PG::addEdge(from, to, mean, information);
    _cachedChi+=chi2(e);
    return e;
  };


  template <typename PG>
  void HCholOptimizer<PG>::refineEdge(typename PG::Edge* e, const typename PG::TransformationType& mean, const typename PG::InformationType& information){
    double derr=-chi2(e);
    PG::refineEdge(e,mean,information);
    derr+=chi2(e);
    _cachedChi+=derr;
  }

  template <typename PG>
  bool HCholOptimizer<PG>::removeEdge(Graph::Edge* e){
    //cerr << __PRETTY_FUNCTION__ << " this:" << this << "edge: " << e->from()->id() << " " << e->to()->id() << endl;
    HVertex* v1=dynamic_cast<HVertex*>(e->from());
    HVertex* v2=dynamic_cast<HVertex*>(e->to());
    assert(this->vertex(e->from()->id())==v1);
    assert(this->vertex(e->to()->id())==v2);
    assert(v1->_optimizer==this);
    assert(v2->_optimizer==this);
    if (!_lowerOptimizer){
      v1->taint();
      v2->taint();
    }
    typename PG::Edge* eAux = reinterpret_cast<typename PG::Edge*>(e);
    _cachedChi-=chi2(eAux);
    _lastOptChi-=chi2(eAux);
    return Graph::removeEdge(e);
  }
  

  template <typename PG>
  bool HCholOptimizer<PG>::removeVertex(Graph::Vertex* _v){
    HVertex* v=dynamic_cast<HVertex*>(_v);
    assert (v->_optimizer==this);
    if (! v) {
      cerr << __PRETTY_FUNCTION__ << ": attempting to remove node " << _v->id()  << " which is not present in the graph" << endl;
      assert(0);
      return false;
    }
    //cerr << __PRETTY_FUNCTION__ << " " << this << ": removing " << _v->id() << endl;
    assert(v);
    if (!_lowerOptimizer)
      v->taint();
    return Graph::removeVertex(v);
  }


  template <typename PG>
  void HCholOptimizer<PG>::addVertexToUpperLevels(HVertex* v){
    if (! _upperOptimizer)
      return;
    assert(v);
    v->_edgeToRoot=0;
    v->_distanceToRoot=0;
    v->_root=v;
    HVertex* vup=dynamic_cast<HVertex*>(_upperOptimizer->addVertex(v->id(), v->transformation, v->covariance, 0));
    v->_parentVertex=vup;
    vup->_children.insert(v);
    vup->_lowerRoot=v;
    _upperOptimizer->addVertexToUpperLevels(vup);
  }

  template <typename PG>
  void HCholOptimizer<PG>::cleanupTainted(){
    if (_upperOptimizer)
      _upperOptimizer->cleanupTainted();
    if (_lowerOptimizer){
      HVertexSet removed;
      for (typename PG::VertexIDMap::iterator it=this->vertices().begin(); it!=this->vertices().end(); it++){
	HVertex* v=dynamic_cast<HVertex*>(it->second);
	if (v->_tainted || ! v->_lowerRoot){
	  removed.insert(v);
	}
      }
      for (typename HVertexSet::iterator it=removed.begin(); it!=removed.end(); it++){
	HVertex* v=*it;
	removeVertex(v);
      }
    }
  }


#if 0
  template <typename PG>
  void HCholOptimizer<PG>::visualizeWithGnuplot(){
    if (_lowerOptimizer)
      return;
    int l=0;
    HCholOptimizer<PG>* opt=this;
    string command="plot ";
    int activeLevels=0;
    bool first=false;
    while (opt){
      if (opt && opt->_gnuplot){
	char chlev='0'+(char)l;
	if (first)
	  command =command+ ", ";
	command = command + "\'-\' w l title \"level"+ chlev + "\"";
	if (opt->_upperOptimizer && opt->_upperOptimizer->_gnuplot)
	activeLevels++;
	first=true;
      }
      opt=opt->_upperOptimizer;
      l++;
    }
    if (activeLevels){
      cout  << command << endl;
    }
    opt=this;
    while (opt){
      if (opt && opt->_gnuplot){
	opt->saveAsGnuplot(cout, !_lowerOptimizer);
	cout << "e" << endl;
      }
      opt=opt->_upperOptimizer;
    }
    if (activeLevels)
      cout << endl;
  }
#endif

} // end namespace
