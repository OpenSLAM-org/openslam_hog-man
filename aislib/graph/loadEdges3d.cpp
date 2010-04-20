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

#include "loadEdges3d.h"

#include <string>
#include <sstream>
using namespace std;

namespace AISNavigation {

void loadEdges3D(LoadedEdgeSet3D& edges, std::istream& is, bool overrideCovariances)
{
  if (! is)
    return;
  string line;
  while (getline(is, line)) {
    if (line.size() > 0 && line[0] == '#') // skip comment lines
      continue;
    istringstream ls(line);
    string tag;
    ls >> tag;
    if (tag == "EDGE3"){
      int id1, id2;
      Vector6 p;
      Matrix6 m;
      ls >> id1 >> id2 >> p.x() >> p.y() >> p.z() >> p.roll() >> p.pitch() >> p.yaw();
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

      LoadedEdge3D e;
      e.id1 = id1;
      e.id2 = id2;
      e.mean = Transformation3::fromVector(p);
      e.informationMatrix = m;
      edges.insert(e);
    }
    // also load the 2D edges for evaluation purposes
    else if (tag == "EDGE2") {
      int id1, id2;
      Vector6 p;
      Matrix6 m = Matrix6::eye(1.0) * 1e9;
      ls >> id1 >> id2 >> p.x() >> p.y() >> p.yaw();
      ls >> m[0][0] >> m[0][1] >> m[1][1] >> m[5][5] >> m[0][5] >> m[1][5];
      m[1][0] = m[0][1];
      m[5][0] = m[0][5];
      m[5][1] = m[1][5];
      LoadedEdge3D e;
      e.id1 = id1;
      e.id2 = id2;
      e.mean = Transformation3::fromVector(p);
      e.informationMatrix = m;
      edges.insert(e);
    }
  }

}

} // end namespace
