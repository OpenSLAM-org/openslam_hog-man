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

#include "posegraph2d.h"

#include <sstream>
using namespace std;

namespace AISNavigation {

  void PoseGraph2D::load(istream& is, bool overrideCovariances, std::vector <PoseGraph2D::Edge*>* orderedEdges){
    clear();
    if (! is)
      return;
    Vertex* previousVertex=0;
    if (orderedEdges)
      orderedEdges->clear();
    while(is){
      char buf[LINESIZE];
      is.getline(buf,LINESIZE);
      istringstream ls(buf);
      string tag;
      ls >> tag;
      if (tag=="VERTEX" || tag=="VERTEX2"){
        int id;
        PoseGraph2D::TransformationVectorType p;
        ls >> id >> p.x() >> p.y() >> p.z();
        PoseGraph2D::TransformationType t=PoseGraph2D::TransformationType::fromVector(p);
        PoseGraph2D::InformationType identity=PoseGraph2D::InformationType::eye(1.);
        PoseGraph2D::Vertex* v=addVertex(id,t,identity);
        if (! v) {
          cerr << "vertex " << id << " is already in the graph, reassigning "<<  endl;
          v=vertex(id);
          assert(v);
        } 
        v->transformation=t;
        v->localTransformation=t;
        previousVertex=v;
      } else if (tag=="EDGE" || tag=="EDGE2"){
        int id1, id2;
        PoseGraph2D::TransformationVectorType p;
        PoseGraph2D::InformationType m;
        ls >> id1 >> id2 >> p.x() >> p.y() >> p.z();
        if (overrideCovariances){
          m=PoseGraph2D::InformationType::eye(1.);
        } else {
          ls >> m[0][0] >> m[0][1] >> m [1][1]
            >> m[2][2] >> m[0][2] >> m [1][2];
          m[1][0]=m[0][1];
          m[2][0]=m[0][2];
          m[2][1]=m[1][2];
        }
        previousVertex=0;
        PoseGraph2D::Vertex* v1=vertex(id1);
        PoseGraph2D::Vertex* v2=vertex(id2);
        if (! v1 ) {
          cerr << "vertex " << id1 << " is not existing, cannot add edge (" << id1 << "," << id2 << ")" << endl; 
          continue;
        }
        if (! v2 ) {
          cerr << "vertex " << id2 << " is not existing, cannot add edge (" << id1 << "," << id2 << ")" << endl; 
          continue;
        }
        PoseGraph2D::TransformationType t=PoseGraph2D::TransformationType::fromVector(p);
        PoseGraph2D::Edge* e=addEdge(v1, v2,t ,m);
        if (! e){
          cerr << "error in adding edge " << id1 << "," << id2 << endl;
        } else {
          if (orderedEdges)
            orderedEdges->push_back(e);
        }
      }
    }
  }

  void PoseGraph2D::save(ostream& os, const PoseGraph2D::TransformationType& offset, int type, bool onlyMarked) const{
    for (VertexIDMap::const_iterator it=_vertices.begin(); it!=_vertices.end(); it++){
      const PoseGraph2D::Vertex* v=dynamic_cast<const PoseGraph2D::Vertex*>(it->second);
      PoseGraph2D::TransformationType t=v->transformation;
      switch(type){
        case 1: t=v->localTransformation; break;
      }

      t=offset*t;
      os << "VERTEX2 " 
        << v->id() << " " 
        << t.translation().x() << " "
        << t.translation().y() << " "
        << t.rotation() << endl;

    }

    for (int l = 0; l <= 1; ++l) { // separate writing of loop edges and sequential edges
      bool writeLoopEdges = l == 1;
      if (writeLoopEdges)
        os << "#LOOP EDGES" << endl;
      else
        os << "#SEQUENTIAL EDGES" << endl;

      for (EdgeSet::const_iterator it=_edges.begin(); it!=_edges.end(); it++){
        const PoseGraph2D::Edge* e=dynamic_cast<const PoseGraph2D::Edge*>(*it);
        if (onlyMarked && !e->_mark)
          continue;
	bool revertOnWrite=false;
	if (e->from()->id()>e->to()->id())
	  revertOnWrite=true;
        const PoseGraph2D::Vertex* v1=dynamic_cast<const PoseGraph2D::Vertex*>(e->from());
        const PoseGraph2D::Vertex* v2=dynamic_cast<const PoseGraph2D::Vertex*>(e->to());
        bool isLoop = abs(v1->id()-v2->id()) != 1;
        if (isLoop == writeLoopEdges) {
	  PoseGraph2D::TransformationVectorType p;
	  if (revertOnWrite){
	    os << "EDGE2 " << v2->id() << " " << v1->id() << " ";
	    p=e->mean().inverse().toVector();
	  } else {
	    os << "EDGE2 " << v1->id() << " " << v2->id() << " ";
	    p=e->mean().toVector();
	  }
          os << p.x() << " " << p.y() << " " << p.z() << " ";
          os << e->information()[0][0] << " "
            << e->information()[0][1] << " "
            << e->information()[1][1] << " "
            << e->information()[2][2] << " "
            << e->information()[0][2] << " "
            << e->information()[1][2] << endl;
        }
      }
    }

  }

  void PoseGraph2D::visualizeToStream(std::ostream& os) const
  {
    os << "set terminal x11 noraise" << endl;
    os << "set size ratio -1" << endl;
    os << "plot '-' w l "<< endl;
    saveAsGnuplot(os);
    os  << "e" << endl;
    os << flush;
  }

  void PoseGraph2D::saveAsGnuplot(ostream& os, bool onlyMarked) const
  {
    for (EdgeSet::const_iterator it=_edges.begin(); it!=_edges.end(); it++){
      const PoseGraph2D::Edge* e = reinterpret_cast<const PoseGraph2D::Edge*>(*it);
      if (onlyMarked && ! e->_mark)
	continue;
      const PoseGraph2D::Vertex* v1 = reinterpret_cast<const PoseGraph2D::Vertex*>(e->from());
      const PoseGraph2D::Vertex* v2 = reinterpret_cast<const PoseGraph2D::Vertex*>(e->to());
      os << v1->transformation.translation().x() << " " << v1->transformation.translation().y() << endl;
      os << v2->transformation.translation().x() << " " << v2->transformation.translation().y() << endl << endl;
    }
  }

} // end namespace
