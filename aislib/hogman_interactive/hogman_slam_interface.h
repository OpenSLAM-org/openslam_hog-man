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

#ifndef HOGMAN_SLAM_INTERFACE_H
#define HOGMAN_SLAM_INTERFACE_H

#include "slam_parser/interface/abstract_slam_interface.h"

#include "aislib/graph_optimizer_hogman/graph_optimizer2d_hchol.h"
#include "aislib/graph_optimizer_hogman/graph_optimizer3d_hchol.h"

#include <map>
#include <vector>

namespace AISNavigation {

  class HogmanSlamInterface : public SlamParser::AbstractSlamInterface
  {
    public:
      HogmanSlamInterface();

      bool addNode(const std::string& tag, int id, int dimension, const std::vector<double>& values);

      bool addEdge(const std::string& tag, int id, int dimension, int v1, int v2, const std::vector<double>& measurement, const std::vector<double>& information);

      bool fixNode(const std::vector<int>& nodes);

      bool queryState(const std::vector<int>& nodes);

      bool solveState();

    protected:
      Optimizer2D* _optimizer;
      GraphOptimizer3D* _optimizer3D;
      bool _firstOptimization;
      int _nodesAdded;
      int _updateGraphEachN;
      bool _initSolverDone;

      bool printVertex(PoseGraph2D::Vertex* v);
      bool printVertex(PoseGraph3D::Vertex* v);
  };

} // end namespace

#endif
