/********************************************************************************
** Form generated from reading ui file 'base_main_widget.ui'
**
** Created: Tue Mar 9 12:02:51 2010
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

#ifndef UI_BASE_MAIN_WIDGET_H
#define UI_BASE_MAIN_WIDGET_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "qgl_graph_viewer.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionLoad;
    QAction *actionSave;
    QAction *actionSave_VRML;
    QAction *actionExit;
    QAction *actionGraph;
    QAction *actionHirarchy;
    QWidget *mainWidget;
    QVBoxLayout *verticalLayout;
    AISNavigation::QGLGraphViewer *viewer;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuView;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(800, 600);
        actionLoad = new QAction(MainWindow);
        actionLoad->setObjectName(QString::fromUtf8("actionLoad"));
        actionSave = new QAction(MainWindow);
        actionSave->setObjectName(QString::fromUtf8("actionSave"));
        actionSave_VRML = new QAction(MainWindow);
        actionSave_VRML->setObjectName(QString::fromUtf8("actionSave_VRML"));
        actionExit = new QAction(MainWindow);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        actionGraph = new QAction(MainWindow);
        actionGraph->setObjectName(QString::fromUtf8("actionGraph"));
        actionGraph->setCheckable(true);
        actionGraph->setChecked(true);
        actionHirarchy = new QAction(MainWindow);
        actionHirarchy->setObjectName(QString::fromUtf8("actionHirarchy"));
        actionHirarchy->setCheckable(true);
        actionHirarchy->setChecked(true);
        mainWidget = new QWidget(MainWindow);
        mainWidget->setObjectName(QString::fromUtf8("mainWidget"));
        verticalLayout = new QVBoxLayout(mainWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        viewer = new AISNavigation::QGLGraphViewer(mainWidget);
        viewer->setObjectName(QString::fromUtf8("viewer"));

        verticalLayout->addWidget(viewer);

        MainWindow->setCentralWidget(mainWidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 24));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuView = new QMenu(menubar);
        menuView->setObjectName(QString::fromUtf8("menuView"));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuView->menuAction());
        menuFile->addAction(actionLoad);
        menuFile->addAction(actionSave);
        menuFile->addAction(actionSave_VRML);
        menuFile->addSeparator();
        menuFile->addAction(actionExit);
        menuView->addAction(actionGraph);
        menuView->addAction(actionHirarchy);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        actionLoad->setText(QApplication::translate("MainWindow", "Load", 0, QApplication::UnicodeUTF8));
        actionSave->setText(QApplication::translate("MainWindow", "Save", 0, QApplication::UnicodeUTF8));
        actionSave_VRML->setText(QApplication::translate("MainWindow", "Save VRML", 0, QApplication::UnicodeUTF8));
        actionExit->setText(QApplication::translate("MainWindow", "Exit", 0, QApplication::UnicodeUTF8));
        actionGraph->setText(QApplication::translate("MainWindow", "Graph", 0, QApplication::UnicodeUTF8));
        actionHirarchy->setText(QApplication::translate("MainWindow", "Hirarchy", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
        menuView->setTitle(QApplication::translate("MainWindow", "View", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BASE_MAIN_WIDGET_H
