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

#ifndef _AIS_GRAPH_HH
#define _AIS_GRAPH_HH

#include <map>
#include <set>
#include <vector>
#include <limits>

/** @addtogroup graph */
//@{
namespace AISNavigation{

  
  struct Graph{
    friend class Dijkstra;

    struct Vertex;
    struct Edge;

    struct VertexIDCompare {
      bool operator() (const Vertex* v1, const Vertex* v2) const
      {
        return v1->id()<v2->id();
      }
    };
    
    typedef std::set<Edge*> EdgeSet;
    typedef std::map<int, Vertex*> VertexIDMap;
    typedef std::set<Vertex*> VertexSet;
 
    struct Vertex{
      friend class Dijkstra;
      friend struct Graph;
      friend struct _DijkstraCompare;
      virtual ~Vertex();
      inline int id() const {return _id;}
      inline const EdgeSet& edges() const {return _edges;}
      inline EdgeSet& edges() {return _edges;}

      mutable bool _mark;
    protected:
      Vertex(int id=-1);
      int _id;
      EdgeSet _edges;
    };


    struct Edge{
      friend struct Graph;
      virtual ~Edge();
      virtual bool revert();
      inline const Vertex* from() const {return _from;}
      inline Vertex* from() {return _from;}
      inline const Vertex* to() const {return _to;}
      inline Vertex* to() {return _to;}

      mutable bool _mark;
    protected:
      Edge(Vertex* from=0, Vertex* to=0);
      Vertex* _from;
      Vertex* _to;
    };


    Vertex* vertex(int id);
    const Vertex* vertex(int id) const;
    EdgeSet connectingEdges(const Vertex* v1, const Vertex* v2);
    
    
    virtual bool removeVertex(Vertex* v);
    virtual bool removeEdge(Edge* e);
    Graph();
    virtual ~Graph();
    virtual void clear();
    
    inline const VertexIDMap& vertices() const {return _vertices;}
    inline VertexIDMap& vertices() {return _vertices;}
    inline const EdgeSet& edges() const {return _edges;}
    inline EdgeSet& edges() {return _edges;}

protected:
    Vertex* addVertex(Vertex* v);
    Edge* addEdge(Edge* e);

    VertexIDMap _vertices;
    EdgeSet _edges;
  };
};

//@}

#endif
