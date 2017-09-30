#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QTest>

#include "config.h"
#include "windowmanager.h"
#include "wordaction.h"


MainWindow::MainWindow(QWidget *parent, MachineState * _right_arm, MachineState * _left_arm) :
  QMainWindow(parent), ui(new Ui::MainWindow), rightArmWidget(this, _right_arm), leftArmWidget(this, _left_arm), right_arm(_right_arm), left_arm(_left_arm), 
  objectMapView(parent, EIN_WINDOW_KEEPRATIO)
{
  ui->setupUi(this);

  windowManager.setMenu(ui->menuWindows);

  ui->objectMapViewFrame->layout()->addWidget(objectMapView.getWidget());
  ui->rightArmFrame->layout()->addWidget(&rightArmWidget);
  ui->leftArmFrame->layout()->addWidget(&leftArmWidget);


  if (right_arm != NULL) {
    WordAction * rightClearStackAction  = new WordAction(ui->menuRightWords, right_arm, name_to_word["clearStack"]);
    ui->menuRightWords->addAction(rightClearStackAction);
  }

  if (left_arm != NULL) {
    WordAction * leftClearStackAction  = new WordAction(ui->menuLeftWords, left_arm, name_to_word["clearStack"]);
    ui->menuLeftWords->addAction(leftClearStackAction);
  }


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::update() {
  if ( right_arm != NULL && !isSketchyMat(right_arm->config.objectMapViewerImage)) {
    objectMapView.updateImage(right_arm->config.objectMapViewerImage);
  } else if (left_arm != NULL && !isSketchyMat(left_arm->config.objectMapViewerImage)) {
    objectMapView.updateImage(left_arm->config.objectMapViewerImage);
  } else {
    // no update. 
  }
}


void MainWindow::setObjectMapViewMouseCallBack(EinMouseCallback m, void* param) {
  objectMapView.setMouseCallBack(m, param);
}


void MainWindow::keyPressEvent(QKeyEvent *event) {
  QWidget::keyPressEvent(event);

}



void MainWindow::addWindow(QMainWindow * window) {
windowManager.addWindow(window);
}
