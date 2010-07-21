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

#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include "ui_base_main_widget.h"

class MainWidget : public QMainWindow, public Ui_MainWindow
{
  Q_OBJECT
  public:
    MainWidget(QWidget* parent = 0, Qt::WindowFlags flags = 0);
    ~MainWidget();

  public slots:
    virtual void loadGraph();
    virtual void saveGraph();
    virtual void saveGraphVrml();

    virtual void setDrawGraph(bool b);
    virtual void setDrawHirarchy(bool b);
};

#endif
