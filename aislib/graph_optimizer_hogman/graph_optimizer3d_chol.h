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

#ifndef CHOL_OPTIMIZER_3D_H
#define CHOL_OPTIMIZER_3D_H

#include "graph_optimizer/graph_optimizer3d.h"
#include "graph_optimizer_chol.h"

namespace AISNavigation {

  /**
   * \brief the 2D cholesky optimizer
   */
  typedef CholOptimizer<PoseGraph3D> CholOptimizer3D;

} // end namespace

#endif
