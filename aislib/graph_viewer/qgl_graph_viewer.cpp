#include "qgl_graph_viewer.h"

#include "standard_camera.h"

namespace AISNavigation {

QGLGraphViewer::QGLGraphViewer(QWidget* parent, const QGLWidget* shareWidget, Qt::WFlags flags) :
  QGLViewer(parent, shareWidget, flags)
{
}

QGLGraphViewer::~QGLGraphViewer()
{
}

void QGLGraphViewer::draw()
{
  glPushMatrix();
  graph.draw();
  glPopMatrix();
}

void QGLGraphViewer::init()
{
  QGLViewer::init();

  // some default settings i like
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND); 
  glEnable(GL_DEPTH_TEST);
  glShadeModel(GL_SMOOTH);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  setAxisIsDrawn();

  // don't save state
  setStateFileName(QString::null);

  // mouse bindings
  setMouseBinding(Qt::RightButton, CAMERA, ZOOM);
  setMouseBinding(Qt::MidButton, CAMERA, TRANSLATE);

  // keyboard shortcuts
  setShortcut(CAMERA_MODE, 0);
  setShortcut(EXIT_VIEWER, 0);
  //setShortcut(SAVE_SCREENSHOT, 0);

  // replace camera
  qglviewer::Camera* oldcam = camera();
  qglviewer::Camera* cam = new StandardCamera();
  cam->setPosition(qglviewer::Vec(0., 0., 75.));
  cam->setUpVector(qglviewer::Vec(0., 1., 0.));
  cam->lookAt(qglviewer::Vec(0., 0., 0.));
  setCamera(cam);
  delete oldcam;
}

} // end namespace
