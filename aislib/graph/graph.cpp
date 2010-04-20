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

#include <assert.h>
#include <queue>
#include "graph.h"

namespace AISNavigation{
  Graph::Vertex::Vertex(int id){
    _id=id;
  }
  
  Graph::Vertex::~Vertex(){
  }

  Graph::Edge::Edge(Vertex* from_, Vertex* to_){
    _from=from_;
    _to=to_;
  }

  Graph::Edge::~Edge(){
  }

  bool Graph::Edge::revert(){
    Graph::Vertex* ap=_from;
    _from=_to;
    _to=ap;
    return true;
  }

  Graph::Vertex* Graph::vertex(int id) {
    VertexIDMap::iterator it=_vertices.find(id);
    if (it==_vertices.end())
      return 0;
    return it->second;
  }

  const Graph::Vertex* Graph::vertex(int id) const {
    VertexIDMap::const_iterator it=_vertices.find(id);
    if (it==_vertices.end())
      return 0;
    return it->second;
  }
  
  Graph::EdgeSet Graph::connectingEdges(const Graph::Vertex* from, const Graph::Vertex* to){
    EdgeSet eset;
    for (EdgeSet::const_iterator it=from->edges().begin(); 
	 it!=from->edges().end(); 
	 it++){
      Edge* e=*it;
      if (e->from()==from && e->to()==to)
	eset.insert(e);
    }
    return eset;
  }

  Graph::Vertex* Graph::addVertex(Vertex* v){
    Vertex* vn=vertex(v->id());
    if (vn)
      return 0;
    _vertices.insert( std::make_pair(v->id(),v) );
    return v;
  }

  Graph::Edge* Graph::addEdge(Edge* e){
    std::pair<EdgeSet::iterator, bool> result=_edges.insert(e);
    if (! result.second)
      return 0;
    e->from()->edges().insert(e);
    e->to()->edges().insert(e);
    return e;
  }
    
  bool Graph::removeVertex(Vertex* v){
    VertexIDMap::iterator it=_vertices.find(v->id());
    if (it==_vertices.end())
      return false;
    assert(it->second==v);
    //remove all edges which are entering or leaving v;
    EdgeSet tmp=v->edges();
    for (EdgeSet::iterator it=tmp.begin(); it!=tmp.end(); it++){
      if (!removeEdge(*it)){
	assert(0);
      }
    }
    _vertices.erase(it);
    delete v;
    return true;
  }
  
  bool Graph::removeEdge(Edge* e){
    EdgeSet::iterator it=_edges.find(e);
    if (it==_edges.end())
      return false;
    _edges.erase(it);

    it=e->from()->edges().find(e);
    assert(it!=e->from()->edges().end());
    e->from()->edges().erase(it);

    it=e->to()->edges().find(e);
    assert(it!=e->to()->edges().end());
    e->to()->edges().erase(it);

    delete e;
    return true;

  }

  Graph::Graph(){
  }

  void Graph::clear(){
    for (VertexIDMap::iterator it=_vertices.begin(); it!=_vertices.end(); it++){
      delete (it->second);
    }
    for (EdgeSet::iterator it=_edges.begin(); it!=_edges.end(); it++){
      delete (*it);
    }
    _vertices.clear();
    _edges.clear();
  }

  Graph::~Graph(){
    clear();
  }

};
