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

#include <queue>
#include <vector>
#include <assert.h>
#include "dijkstra.h"

namespace AISNavigation{

  using namespace std;

  Dijkstra::AdjacencyMapEntry::AdjacencyMapEntry(Graph::Vertex* child_, 
						 Graph::Vertex* parent_, 
						 Graph::Edge* edge_, 
						 double distance_){
    _child=child_;
    _parent=parent_;
    _edge=edge_;
    _distance=distance_;
    std::set<AdjacencyMapEntry*> choldren;
  }

  Dijkstra::Dijkstra(Graph* g): _graph(g){
    for (Graph::VertexIDMap::const_iterator it=_graph->vertices().begin(); it!=_graph->vertices().end(); it++){
      AdjacencyMapEntry entry(it->second, 0,0,std::numeric_limits< double >::max());
      _adjacencyMap.insert(make_pair(entry.child(), entry));
    }
  }

  void Dijkstra::reset(){
    for (Graph::VertexSet::iterator it=_visited.begin(); it!=_visited.end(); it++){
      AdjacencyMap::iterator at=_adjacencyMap.find(*it);
      assert(at!=_adjacencyMap.end());
      at->second=AdjacencyMapEntry(at->first,0,0,std::numeric_limits< double >::max());
    }
    _visited.clear();
  }


  bool operator<(const Dijkstra::AdjacencyMapEntry& a, const Dijkstra::AdjacencyMapEntry& b){
    return a.distance()>b.distance();
  }


  void Dijkstra::shortestPaths(Graph::Vertex* v, 
			       Dijkstra::CostFunction* cost, 
			       double maxDistance, 
			       double comparisonConditioner, 
			       bool directed){
    reset();
    std::priority_queue< AdjacencyMapEntry > frontier;
    AdjacencyMap::iterator it=_adjacencyMap.find(v);
    assert(it!=_adjacencyMap.end());
    it->second._distance=0.;
    frontier.push(it->second);
    
    while(! frontier.empty()){
      AdjacencyMapEntry entry=frontier.top();
      frontier.pop();
      Graph::Vertex* u=entry.child();
      AdjacencyMap::iterator ut=_adjacencyMap.find(u);
      assert(ut!=_adjacencyMap.end());
      double uDistance=ut->second.distance();
      
      std::pair< Graph::VertexSet::iterator, bool> insertResult=_visited.insert(u);
      Graph::EdgeSet::iterator et=u->edges().begin();	
      while(et!=u->edges().end()){
	Graph::Edge* edge=*et;
	et++;
	
	Graph::Vertex* z=0;
	if (edge->from()==u)
	  z=edge->to();
	else if(edge->to()==u)
	  z=edge->from();
	assert(z);
	
	if (directed && edge->from()!=u)
	  continue;

	double edgeDistance=(*cost)(edge, u, z);
	if (edgeDistance==std::numeric_limits< double >::max())
	  continue;
	double zDistance=uDistance+edgeDistance;
	AdjacencyMap::iterator ot=_adjacencyMap.find(z);
	assert(ot!=_adjacencyMap.end());
	
	if (zDistance+comparisonConditioner<ot->second.distance() && zDistance<maxDistance){
	  ot->second._distance=zDistance;
	  ot->second._parent=u;
	  ot->second._edge=edge;
	  frontier.push(ot->second);
	}
      }
    }
  }

  void Dijkstra::computeTree(Graph::Vertex* v __attribute__((unused)), AdjacencyMap& amap){
   for (AdjacencyMap::iterator it=amap.begin(); it!=amap.end(); it++){
      AdjacencyMapEntry& entry(it->second);
      entry._children.clear();
   }
   for (AdjacencyMap::iterator it=amap.begin(); it!=amap.end(); it++){
      AdjacencyMapEntry& entry(it->second);
      Graph::Vertex* parent=entry.parent();
      if (!parent){
	continue;
      }
      Graph::Vertex* v=entry.child();
      assert (v==it->first);

      AdjacencyMap::iterator pt=amap.find(parent);
      assert(pt!=amap.end());
      pt->second._children.insert(v);
   }
  }


  void Dijkstra::visitAdjacencyMap(Graph::Vertex* v, AdjacencyMap& amap, TreeAction* action){
    typedef std::deque<Graph::Vertex*> Deque;
    Deque q;
    action->perform(v,0,0);
    q.push_back(v);
    int count=0;
    while (! q.empty()){
      Graph::Vertex* parent=q.front();
      q.pop_front();
      count++;
      AdjacencyMap::iterator parentIt=amap.find(parent);
      if (parentIt==amap.end())
	continue;
      Graph::VertexSet& childs(parentIt->second.children());
      for (Graph::VertexSet::iterator childsIt=childs.begin(); childsIt!=childs.end(); childsIt++){
	Graph::Vertex* child=*childsIt;
	AdjacencyMap::iterator adjacencyIt=amap.find(child);
 	assert (adjacencyIt!=amap.end());
	Graph::Edge* edge=adjacencyIt->second.edge();	

	assert(adjacencyIt->first==child);
	assert(adjacencyIt->second.child()==child);
	assert(adjacencyIt->second.parent()==parent);
	action->perform(child, parent, edge);
	q.push_back(child);
      } 
    }
    
  }

  void connectedSubset(Graph::VertexSet& connected, Graph::VertexSet& visited, 
				    Graph::VertexSet& startingSet, 
				    Graph* g, Graph::Vertex* v,
				    Dijkstra::CostFunction* cost, double distance, double comparisonConditioner){
    typedef std::queue<Graph::Vertex*> VertexDeque;
    visited.clear();
    connected.clear();
    VertexDeque frontier;
    Dijkstra dv(g);
    connected.insert(v);
    frontier.push(v);
    while (! frontier.empty()){
      Graph::Vertex* v0=frontier.front();
      frontier.pop();
      dv.shortestPaths(v0,cost,distance,comparisonConditioner);
      for (Graph::VertexSet::iterator it=dv.visited().begin(); it!=dv.visited().end(); it++){
	visited.insert(*it);
	if (startingSet.find(*it)==startingSet.end())
	  continue;
	std::pair<Graph::VertexSet::iterator, bool> insertOutcome=connected.insert(*it);
	if (insertOutcome.second){ // the node was not in the connectedSet;
	  frontier.push(dynamic_cast<Graph::Vertex*>(*it));
	}
      }
    }
  }

  double UniformCostFunction::operator () (Graph::Edge* edge __attribute__((unused)),
      Graph::Vertex* from __attribute__((unused)), Graph::Vertex* to __attribute__((unused))) {
    return 1.;
  }



};
