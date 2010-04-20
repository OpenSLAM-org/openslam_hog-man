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

#ifndef _GRAPH_OPTIMIZER2D_AUX_HH_
#define _GRAPH_OPTIMIZER2D_AUX_HH_

#include "aislib/math/transformation.h"
#include <set>
#include <iostream>

namespace AISNavigation {
  struct LoadedEdge{
    int id1, id2;
    Transformation2 mean;
    Matrix3 informationMatrix;
  };

  struct LoadedEdgeComparator{
    inline bool operator()(const LoadedEdge& e1, const LoadedEdge& e2){
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

  typedef std::set<LoadedEdge, LoadedEdgeComparator> LoadedEdgeSet;

  void loadEdges(LoadedEdgeSet& edges, std::istream& is, bool overrideCovariances=false);

}
#endif
