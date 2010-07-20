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
