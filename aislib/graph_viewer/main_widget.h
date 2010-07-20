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
