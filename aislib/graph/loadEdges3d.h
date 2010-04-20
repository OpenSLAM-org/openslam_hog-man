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

#ifndef LOADEDGES_3D_H
#define LOADEDGES_3D_H

#include <set>
#include <iostream>

#include "aislib/math/transformation.h"

namespace AISNavigation {

  struct LoadedEdge3D
  {
    int id1, id2;
    Transformation3 mean;
    Matrix6 informationMatrix;
  };

  struct LoadedEdgeComparator3D
  {
    inline bool operator()(const LoadedEdge3D& e1, const LoadedEdge3D& e2){
      int i11=e1.id1, i12=e1.id2;
      if (i11>i12){
	i11=e1.id2;
      i12=e1.id1;
      }
      int i21=e2.id1, i22=e2.id2;
      if (i21>i22){
	i21=e2.id2;
	i22=e2.id1;
      }
      if (i12<i22)
	return true;
      if (i12>i22)
	return false;
      return (i11<i21);
    }
  };

  typedef std::set<LoadedEdge3D, LoadedEdgeComparator3D> LoadedEdgeSet3D;

  void loadEdges3D(LoadedEdgeSet3D& edges, std::istream& is, bool overrideCovariances = false);

}// end namespace

#endif
