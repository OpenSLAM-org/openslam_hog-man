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

#include <sstream>
#include "graph_optimizer2d_aux.h"

namespace AISNavigation{
  using namespace std;


#define LINESIZE 4096

  void loadEdges(LoadedEdgeSet& edges, istream& is, bool overrideCovariances){
    edges.clear();
    if (!is)
      return;
    
    while(is){
      char buf[LINESIZE];
      is.getline(buf,LINESIZE);
      istringstream ls(buf);
      string tag;
      ls >> tag;
      
      if (tag=="EDGE" || tag=="EDGE2"){
	int id1, id2;
	Vector3 p;
	Matrix3 m;
	ls >> id1 >> id2 >> p.x() >> p.y() >> p.z();
	if (id1>id2) { //hack
	  int a=id1; id1=id2; id2=a;
	  p=(Transformation2::fromVector(p).inverse()).toVector();
	}
	
	if (overrideCovariances){
	  m=Matrix3::eye(1.);
	} else {
	  ls >> m[0][0] >> m[0][1] >> m [1][1]
	     >> m[2][2] >> m[0][2] >> m [1][2];
	  m[1][0]=m[0][1];
	  m[2][0]=m[0][2];
	  m[2][1]=m[1][2];
	}
	LoadedEdge e;
	e.id1=id1;
	e.id2=id2;
	e.mean=Transformation2::fromVector(p);
	e.informationMatrix=m;
	edges.insert(e);
      }
    }
  }

}
