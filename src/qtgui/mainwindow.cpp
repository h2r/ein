#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QTest>

#include "ein.h"
#include "windowmanager.h"

int last_key = -1;

MainWindow::MainWindow(QWidget *parent, shared_ptr<MachineState> _ms) :
  QMainWindow(parent),
  ui(new Ui::MainWindow),
  wristView(parent, EIN_WINDOW_KEEPRATIO),
  objectView(parent, EIN_WINDOW_KEEPRATIO),
  objectMapView(parent, EIN_WINDOW_KEEPRATIO)
{
    ui->setupUi(this);

    ui->wristViewFrame->layout()->addWidget(wristView.getWidget());
    ui->objectViewFrame->layout()->addWidget(objectView.getWidget());
    ui->objectMapViewFrame->layout()->addWidget(objectMapView.getWidget());
    ms = _ms;

    stackModel = new StackModel(this);
    stackModel->setMachineState(ms);
    ui->stackTableView->setModel(stackModel);

    windowManager.setMenu(ui->menuWindows);

}
  
MainWindow::~MainWindow()
{
    delete ui;
}

void timercallback1(const ros::TimerEvent&); 

void MainWindow::rosSpin()
{
  ros::TimerEvent e;
  timercallback1(e);
  ros::spinOnce();

}

void MainWindow::update() {
  ui->stateLabel->setText(QString::fromStdString(ms->currentState()));
  stackModel->setMachineState(ms);
  if ( !isSketchyMat(ms->config.wristViewImage)) {
     wristView.updateImage(ms->config.wristViewImage);
  }
  if ( !isSketchyMat(ms->config.objectViewerImage)) {
    objectView.updateImage(ms->config.objectViewerImage);
  }
  if ( !isSketchyMat(ms->config.objectMapViewerImage)) {
    objectMapView.updateImage(ms->config.objectMapViewerImage);
  }
}

void MainWindow::setMouseCallBack(EinMouseCallback m, void* param) {
  wristView.setMouseCallBack(m, param);
}


void MainWindow::keyPressEvent(QKeyEvent *evnt) {
  updateLastKey(evnt);
  QWidget::keyPressEvent(evnt);

}

void updateLastKey(QKeyEvent * evnt) {
  int key = evnt->key();

  Qt::Key qtkey = static_cast<Qt::Key>(key);
  char asciiCode = QTest::keyToAscii(qtkey);
  if (asciiCode != 0)
    key = static_cast<int>(asciiCode);
  else
    key = evnt->nativeVirtualKey(); //same codes as returned by GTK-based backend

  //control plus (Z, +, -, up, down, left, right) are used for zoom/panning functions
  if (evnt->modifiers() != Qt::ControlModifier)
    {
      last_key = key;
    }

}
void MainWindow::addWindow(EinWindow * window) {
windowManager.addWindow(window);
}
