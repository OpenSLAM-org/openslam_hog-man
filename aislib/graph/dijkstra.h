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

#ifndef _AIS_DIJKSTRA_HH
#define _AIS_DIJKSTRA_HH

#include <map>
#include <set>
#include <limits>
#include "graph.h"

namespace AISNavigation{

  struct Dijkstra{
    struct CostFunction {
      virtual double operator() (Graph::Edge* e, Graph::Vertex* from, Graph::Vertex* to)=0;
    };

    struct TreeAction {
      virtual double perform(Graph::Vertex* v, Graph::Vertex* vParent, Graph::Edge* e)=0;
    };


    struct AdjacencyMapEntry{
      friend struct Dijkstra;
      AdjacencyMapEntry(Graph::Vertex* _child=0, 
			Graph::Vertex* _parent=0, 
			Graph::Edge* _edge=0, 
			double _distance=std::numeric_limits<double>::max());
      inline Graph::Vertex* child() const {return _child;}
      inline Graph::Vertex* parent() const {return _parent;}
      inline Graph::Edge* edge() const {return _edge;}
      inline double      distance() const {return _distance;}
      inline Graph::VertexSet& children() {return _children;}
    protected:
      Graph::Vertex* _child;
      Graph::Vertex* _parent;
      Graph::Edge* _edge;
      double _distance;
      Graph::VertexSet _children;
    };

    typedef std::map<Graph::Vertex*, AdjacencyMapEntry> AdjacencyMap;
    Dijkstra(Graph* g);
    inline Graph::VertexSet& visited() {return _visited; }
    inline AdjacencyMap& adjacencyMap() {return _adjacencyMap; }
    inline Graph* graph() {return _graph;} 
    void shortestPaths(Graph::Vertex* v, 
				 Dijkstra::CostFunction* cost, 
				 double maxDistance=std::numeric_limits< double >::max(), 
				 double comparisonConditioner=1e-3, 
				 bool directed=false);

    static void computeTree(Graph::Vertex* v, AdjacencyMap& amap);
    static void visitAdjacencyMap(Graph::Vertex* v, AdjacencyMap& amap, TreeAction* action);

  protected:
    void reset();

    AdjacencyMap _adjacencyMap;
    Graph::VertexSet _visited;
    Graph* _graph;


  };

  void connectedSubset(Graph::VertexSet& connected, Graph::VertexSet& visited, 
		       Graph::VertexSet& startingSet, 
		       Graph* g, Graph::Vertex* v,
		       Dijkstra::CostFunction* cost, double distance, double comparisonConditioner);


  struct UniformCostFunction: public Dijkstra::CostFunction {
    virtual double operator ()(Graph::Edge* edge, Graph::Vertex* from, Graph::Vertex* to);
  };

  

}
#endif
