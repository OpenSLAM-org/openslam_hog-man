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

#include "main_widget.h"
#include "moc_main_widget.cpp"

#include <aislib/graph/posegraph3d.h>

#include <QFileDialog>

#include <iostream>
#include <fstream>
using namespace std;
using namespace AISNavigation;

MainWidget::MainWidget(QWidget* parent, Qt::WindowFlags flags) :
  QMainWindow(parent, flags)
{
  setupUi(this);
  setWindowTitle( "GraphViewer");

  // main menu items (just click events)
  connect((const QObject*) actionLoad, SIGNAL( activated() ), 
	  (const QObject*) this, SLOT( loadGraph() ) );
  connect((const QObject*) actionSave, SIGNAL( activated() ), 
	  (const QObject*) this, SLOT( saveGraph() ) );
  connect((const QObject*) actionExit, SIGNAL( activated() ), 
	  (const QObject*) this, SLOT( close() ) );
  connect((const QObject*) actionSave_VRML, SIGNAL( activated() ), 
	  (const QObject*) this, SLOT( saveGraphVrml() ) );

  // toggle actions
  connect((const QObject*) actionGraph, SIGNAL( toggled(bool) ), 
	  (const QObject*) this, SLOT( setDrawGraph(bool) ) );
  connect((const QObject*) actionHirarchy, SIGNAL( toggled(bool) ), 
	  (const QObject*) this, SLOT( setDrawHirarchy(bool) ) );

  {
#include "icon.xpm"
    QPixmap pixIcon(icon);
    setWindowIcon(pixIcon);
  }
}

MainWidget::~MainWidget()
{
}

void MainWidget::loadGraph()
{
  PoseGraph3D* graph = viewer->graph.getGraph();
  if (!graph) {
    cerr << "No graph available" << endl;
  }
  QString fileName = QFileDialog::getOpenFileName(this, tr("Open Graph File"), "", tr("Graph-Files (*.graph)"));
  if (!fileName.isEmpty()) {
    ifstream fin(fileName.toAscii());
    graph->clear();
    graph->load(fin);
    viewer->graph.setGraph(graph);
    viewer->updateGL();
  }
}

void MainWidget::saveGraph()
{
  PoseGraph3D* graph = viewer->graph.getGraph();
  if (!graph) {
    cerr << "No graph available" << endl;
  }
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save Graph File"), "", tr("Graph-Files (*.graph)"));
  if (!fileName.isEmpty()) {
    ofstream fout(fileName.toAscii());
    graph->save(fout);
  }
}

void MainWidget::saveGraphVrml()
{
  PoseGraph3D* graph = viewer->graph.getGraph();
  if (!graph) {
    cerr << "No graph available" << endl;
  }
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save Graph VRML-File"), "", tr("VRML-Files (*.wrl)"));
  if (!fileName.isEmpty()) {
    ofstream fout(fileName.toAscii());
    viewer->graph.writeVrml(fout);
  }
}

void MainWidget::setDrawGraph(bool b)
{
  viewer->graph.setDrawGraph(b);
  viewer->updateGL();
}

void MainWidget::setDrawHirarchy(bool b)
{
  viewer->graph.setDrawHirarchy(b);
  viewer->updateGL();
}
