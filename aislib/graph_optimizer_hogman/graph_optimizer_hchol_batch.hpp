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
  void HCholOptimizer<PG>::annotateHiearchicalEdges(int iterations, double lambda, bool initWithObservations){
    if (this->verbose())
      cerr <<  "refining edges" << endl; 
    for (typename PG::EdgeSet::iterator it=this->edges().begin(); it!=this->edges().end(); it++){
      typename PG::Edge* e=dynamic_cast<typename PG::Edge*>(*it);
      annotateHiearchicalEdge(e,iterations, lambda, initWithObservations);
    }
    if (this->verbose())
      cerr << "done";
  }

  template <typename PG>
  void HCholOptimizer<PG>::bottomToTop(int iterations, double lambda, bool initWithObservations){
    if (! _lowerOptimizer)
      return;
    if (_lowerOptimizer){
      _lowerOptimizer->bottomToTop(iterations, lambda,initWithObservations);
    }
    annotateHiearchicalEdges(iterations, lambda, initWithObservations);
    if (this->verbose())
      cerr << "_upperOptimizer="  << _upperOptimizer << endl;
    if (! _upperOptimizer){
      if (this->verbose())
	cerr << "Optimizing the top level" << endl;
      HVertex* rootVertex=_MY_CAST_<HVertex*>(this->vertices().begin()->second);
      this->initialize(rootVertex->id());
      //ofstream oss("topLevel.graph");
      //save(oss);
      //oss.close();
      Graph::VertexSet vset;
      for (Graph::VertexIDMap::const_iterator it=this->vertices().begin(); it!=this->vertices().end(); it++){
	vset.insert(it->second);
      }
      this->optimizeSubset(rootVertex, vset, iterations*3, lambda, true);
      if (this->verbose()) {
	cerr << "Done";
	//ofstream os("topLevel.dat");
	//saveAsGnuplot(os);
	//os.close();
	cerr << "done" << endl;
      }

    }
  }

  template <typename PG>
  void HCholOptimizer<PG>::topToBottom(int iterations, double lambda){
    if (! _lowerOptimizer)
      return;

    int total=0;
    // first step, project the nodes according to the parent
    for (Graph::VertexIDMap::iterator it=this->vertices().begin(); it!=this->vertices().end(); it++){
      HVertex* v=dynamic_cast<HVertex*>(it->second);
      assert(v);
      _lowerOptimizer->transformSubset(v->lowerRoot(), *(Graph::VertexSet*)(&v->children()), v->transformation);
      _lowerOptimizer->optimizeSubset (v->lowerRoot(), *(Graph::VertexSet*)(&v->children()), 0, 0., true);
      total+=v->children().size();
    }
    if((int)_lowerOptimizer->vertices().size()!=total){
      cerr << "fatal error in partitioning: " << _lowerOptimizer->vertices().size()  << " != " << total << endl;  
    }
    
    // second step, optimize the nodes, by keeping the neighbors fixed (relaxation)
    for (Graph::VertexIDMap::iterator it=this->vertices().begin(); it!=this->vertices().end(); it++){

      HVertex* v=dynamic_cast<HVertex*>(it->second);
      assert(v);
      if (this->verbose()) cerr << "d";
      _lowerOptimizer->optimizeSubset (v->lowerRoot(), *(Graph::VertexSet*)(&v->children()), iterations, lambda, true);
    }

    HCholOptimizer<PG>* lowerHOpt=dynamic_cast<HCholOptimizer<PG>*>(_lowerOptimizer);
    if (lowerHOpt){
      lowerHOpt->topToBottom(iterations, lambda);
    }
   
  }

  template <typename PG>
  void HCholOptimizer<PG>::vCycle(int iterations, double lambdaUp, double lambdaDown, bool initWithObservations){
    if (_upperOptimizer){
      cerr << "# ERROR, a V-Cycle can be only started from the topmost optimizer in the hierarchy" << endl;
      return;
    }
    bottomToTop(iterations, lambdaUp, initWithObservations);
    topToBottom(iterations, lambdaDown);
  }

}
