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

#include "posegraph3d.h"

#include <cassert>
#include <queue>
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <sys/time.h>

#include <sstream>
#include "aislib/stuff/os_specific.h"

using namespace std;

namespace AISNavigation{
  void PoseGraph3D::load(istream& is, bool overrideCovariances, std::vector <PoseGraph3D::Edge*>* orderedEdges)
  {
    clear();
    if (! is)
      return;
    Vertex* previousVertex=0;
    if (orderedEdges)
      orderedEdges->clear();
    string line;
    while (getline(is, line)) {
      if (line.size() == 0 || line[0] == '#') // skip comment lines
        continue;
      istringstream ls(line);
      string tag;
      ls >> tag;
      if (tag == "VERTEX3"){
	int id;
	Vector6 p;
        ls >> id;
        for (int i = 0; i < 6; ++i)
          ls >> p[i];
	Transformation3 t = Transformation3::fromVector(p);
	Matrix6 identity = Matrix6::eye(1.0);
	Vertex* v=addVertex(id, t, identity);
	if (! v) {
	  cerr << "vertex " << id << " is already in the graph, reassigning "<<  endl;
	  v = vertex(id);
	  assert(v);
	} 
	v->transformation = t;
	v->localTransformation = t;
	previousVertex = v;
      } else if (tag == "EDGE3"){
	int id1, id2;
	Vector6 p;
        ls >> id1 >> id2;
        for (int i = 0; i < 6; ++i)
          ls >> p[i];
	Matrix6 m;
	if (overrideCovariances) {
	  m = Matrix6::eye(1.0);
	} else {
          for (int i=0; i<6; i++)
	    for (int j=i; j<6; j++) {
	      ls >> m[i][j];
              if (i != j)
                m[j][i] = m[i][j];
            }
	}
	previousVertex=0;
	Vertex* v1=vertex(id1);
	Vertex* v2=vertex(id2);
	if (! v1 ) {
	  cerr << "vertex " << id1 << " is not existing, cannot add edge (" << id1 << "," << id2 << ")" << endl; 
	  continue;
	}
	if (! v2 ) {
	  cerr << "vertex " << id2 << " is not existing, cannot add edge (" << id1 << "," << id2 << ")" << endl; 
	  continue;
	}
	Transformation3 t = Transformation3::fromVector(p);
	Edge* e = addEdge(v1, v2, t, m);
	if (! e){
	  cerr << "error in adding edge " << id1 << "," << id2 << endl;
	} else {
	  if (orderedEdges)
	    orderedEdges->push_back(e);
	}
      }
    }
  }
  
  void PoseGraph3D::save(ostream& os, const Transformation3& offset, int type, bool onlyMarked) const
  {
    //os << setprecision(3);

    for (VertexIDMap::const_iterator it = _vertices.begin(); it != _vertices.end(); ++it) {
      const Vertex* v=dynamic_cast<const PoseGraph3D::Vertex*>(it->second);
      Transformation3 t = v->transformation;
      switch (type) {
        case 1: t = v->localTransformation; break;
      }

      t=offset*t;
      Vector6 pose =  t.toVector();
      os << "VERTEX3 " << v->id() << " " 
       << pose.x() << " " << pose.y() << " " << pose.z() << " "
       << pose[3] << " " << pose[4] << " " << pose[5] << endl;

    }

    for (int l = 0; l <= 1; ++l) { // separate writing of loop edges and sequential edges
      bool writeLoopEdges = l == 1;
      if (writeLoopEdges)
        os << "#LOOP EDGES" << endl;
      else
        os << "#SEQUENTIAL EDGES" << endl;
      for (EdgeSet::const_iterator it = _edges.begin(); it != _edges.end(); ++it) {
        const Edge* e = dynamic_cast<const PoseGraph3D::Edge*>(*it);
        if (onlyMarked && !e->_mark)
          continue;
        const Vertex* v1 = dynamic_cast<const PoseGraph3D::Vertex*>(e->from());
        const Vertex* v2 = dynamic_cast<const PoseGraph3D::Vertex*>(e->to());
        bool isLoop = abs(v1->id()-v2->id()) != 1;
        if (isLoop == writeLoopEdges) {
          os << "EDGE3 " << v1->id() << " " << v2->id() << " ";
          Vector6 p = e->mean().toVector();
          os << p.x() << " " << p.y() << " " << p.z() << " " << p[3] << " " << p[4] << " " << p[5];
          for (int i=0; i<6; i++)
            for (int j=i; j<6; j++)
              os << " " << e->information()[i][j];
          os << endl;
        }
      }
    }
  }

void PoseGraph3D::saveGnuplot(std::ostream& os, const Transformation3& offset, bool onlyMarked) const
{
  for (EdgeSet::const_iterator it = _edges.begin(); it != _edges.end(); ++it) {
    const Edge* e = dynamic_cast<const PoseGraph3D::Edge*>(*it);
    if (onlyMarked && !e->_mark)
      continue;
    const Vertex* v1 = dynamic_cast<const PoseGraph3D::Vertex*>(e->from());
    const Vertex* v2 = dynamic_cast<const PoseGraph3D::Vertex*>(e->to());
    Vector6 v1p = (offset * v1->transformation).toVector();
    Vector6 v2p = (offset * v2->transformation).toVector();
    os << v1p.x() << " " << v1p.y() << " " << v1p.z() << " "
       << v1p[3] << " " << v1p[4] << " " << v1p[5] <<endl;
    os << v2p.x() << " " << v2p.y() << " " << v2p.z() << " "
       << v2p[3] << " " << v2p[4] << " " << v2p[5] <<endl;
    os << endl << endl;
  }

}

void PoseGraph3D::loadBinary(std::istream& is, bool overrideCovariances, std::vector <PoseGraph3D::Edge*> *orderedEdges)
{
  clear();
  if (! is)
    return;
  if (orderedEdges)
    orderedEdges->clear();
  char c = 0;
  while (is.get(c)) {
    if (c == 'V'){
      int id;
      Transformation3 t;
      Matrix6 identity = Matrix6::eye(1.0);
      is.read((char*)&id, sizeof(int));
      is.read((char*)&t.translation().x(), sizeof(double));
      is.read((char*)&t.translation().y(), sizeof(double));
      is.read((char*)&t.translation().z(), sizeof(double));
      is.read((char*)&t.rotation().w(), sizeof(double));
      is.read((char*)&t.rotation().x(), sizeof(double));
      is.read((char*)&t.rotation().y(), sizeof(double));
      is.read((char*)&t.rotation().z(), sizeof(double));
      Vertex* v=addVertex(id, t, identity);
      if (! v) {
        cerr << "vertex " << id << " is already in the graph, reassigning "<<  endl;
        v = vertex(id);
        assert(v);
      } 
      v->transformation = t;
      v->localTransformation = t;
    } else if (c == 'E'){
      int id1, id2;
      Transformation3 t;
      Matrix6 m(6, 6);
      is.read((char*)&id1, sizeof(int));
      is.read((char*)&id2, sizeof(int));
      is.read((char*)&t.translation().x(), sizeof(double));
      is.read((char*)&t.translation().y(), sizeof(double));
      is.read((char*)&t.translation().z(), sizeof(double));
      is.read((char*)&t.rotation().w(), sizeof(double));
      is.read((char*)&t.rotation().x(), sizeof(double));
      is.read((char*)&t.rotation().y(), sizeof(double));
      is.read((char*)&t.rotation().z(), sizeof(double));
      if (overrideCovariances) {
        double dummy; // just read over the information matrix
        for (int i=0; i<6; i++)
          for (int j=i; j<6; j++)
            is.read((char*)&dummy, sizeof(double));
      } else {
        for (int i=0; i<6; i++)
          for (int j=i; j<6; j++) {
            is.read((char*)&m[i][j], sizeof(double));
            if (i != j)
              m[j][i] = m[i][j];
          }
      }

      Vertex* v1=vertex(id1);
      Vertex* v2=vertex(id2);
      if (! v1 ) {
        cerr << "vertex " << id1 << " is not existing, cannot add edge (" << id1 << "," << id2 << ")" << endl; 
        continue;
      }
      if (! v2 ) {
        cerr << "vertex " << id2 << " is not existing, cannot add edge (" << id1 << "," << id2 << ")" << endl; 
        continue;
      }
      Edge* e = addEdge(v1, v2, t, m);
      if (! e){
        cerr << "error in adding edge " << id1 << "," << id2 << endl;
      } else {
        if (orderedEdges)
          orderedEdges->push_back(e);
      }
    }
  }

}

void PoseGraph3D::saveBinary(std::ostream& os, int type, bool onlyMarked) const
{
  for (VertexIDMap::const_iterator it = _vertices.begin(); it != _vertices.end(); ++it) {
    const Vertex* v=dynamic_cast<const PoseGraph3D::Vertex*>(it->second);
    Transformation3 t = v->transformation;
    switch (type) {
      case 1: t = v->localTransformation; break;
    }

    os.put('V');
    int id = v->id();
    os.write((char*)&id, sizeof(int));
    os.write((char*)&t.translation().x(), sizeof(double));
    os.write((char*)&t.translation().y(), sizeof(double));
    os.write((char*)&t.translation().z(), sizeof(double));
    os.write((char*)&t.rotation().w(), sizeof(double));
    os.write((char*)&t.rotation().x(), sizeof(double));
    os.write((char*)&t.rotation().y(), sizeof(double));
    os.write((char*)&t.rotation().z(), sizeof(double));
  }

  for (EdgeSet::const_iterator it = _edges.begin(); it != _edges.end(); ++it) {
    const Edge* e = dynamic_cast<const PoseGraph3D::Edge*>(*it);
    if (onlyMarked && !e->_mark)
      continue;
    const Vertex* v1 = dynamic_cast<const PoseGraph3D::Vertex*>(e->from());
    const Vertex* v2 = dynamic_cast<const PoseGraph3D::Vertex*>(e->to());
    os.put('E');
    int id1 = v1->id();
    int id2 = v2->id();
    os.write((char*)&id1, sizeof(int));
    os.write((char*)&id2, sizeof(int));
    os.write((char*)&e->mean().translation().x(), sizeof(double));
    os.write((char*)&e->mean().translation().y(), sizeof(double));
    os.write((char*)&e->mean().translation().z(), sizeof(double));
    os.write((char*)&e->mean().rotation().w(), sizeof(double));
    os.write((char*)&e->mean().rotation().x(), sizeof(double));
    os.write((char*)&e->mean().rotation().y(), sizeof(double));
    os.write((char*)&e->mean().rotation().z(), sizeof(double));
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++)
        os.write((char*)&e->information()[i][j], sizeof(double));
  }
}

void PoseGraph3D::visualizeToStream(std::ostream& os) const
{
  struct timeval now;
  gettimeofday(&now, 0);
  double curTime = now.tv_sec + now.tv_usec*1e-6;
  saveBinary(os);
  os << "T"; // write time spend in optimization, used to record video in realtime
  os.write((char*)&curTime, sizeof(double));
  os << "F" << flush;
}

} // end namespace
