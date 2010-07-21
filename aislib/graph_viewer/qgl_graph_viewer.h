// HOG-Man - Hierarchical Optimization for Pose Graphs on Manifolds
// Copyright (C) 2010 G. Grisetti, R. Kümmerle, C. Stachniss
//
// This file is part of HOG-Man.
// 
// HOG-Man is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// HOG-Man is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with HOG-Man.  If not, see <http://www.gnu.org/licenses/>.

#ifndef QGL_GRAPH_VIEWER_H
#define QGL_GRAPH_VIEWER_H

#include "pose_graph_vis3d.h"
#include <qglviewer/qglviewer.h>

namespace AISNavigation {

class QGLGraphViewer : public QGLViewer
{
  public:
    explicit QGLGraphViewer(QWidget* parent=NULL, const QGLWidget* shareWidget=0, Qt::WFlags flags=0);
    ~QGLGraphViewer();
    virtual void draw();
    void init();

  public:
    PoseGraph3DVis graph;
};

} // end namespace

#endif
