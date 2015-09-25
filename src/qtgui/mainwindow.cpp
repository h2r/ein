#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QTest>

#include "ein.h"
#include "windowmanager.h"
#include "wordaction.h"

int last_key = -1;

MainWindow::MainWindow(QWidget *parent, shared_ptr<MachineState> _right_arm, shared_ptr<MachineState> _left_arm) :
  QMainWindow(parent), ui(new Ui::MainWindow), rightArmWidget(this, _right_arm), leftArmWidget(this, _left_arm), right_arm(_right_arm), left_arm(_left_arm), 
  objectMapView(parent, EIN_WINDOW_KEEPRATIO)
{
  ui->setupUi(this);

  windowManager.setMenu(ui->menuWindows);

  ui->objectMapViewFrame->layout()->addWidget(objectMapView.getWidget());
  ui->rightArmFrame->layout()->addWidget(&rightArmWidget);
  ui->leftArmFrame->layout()->addWidget(&leftArmWidget);






  
  WordAction * rightClearStackAction  = new WordAction(ui->menuRightWords, right_arm, name_to_word["clearStack"]);
  ui->menuRightWords->addAction(rightClearStackAction);


  WordAction * leftClearStackAction  = new WordAction(ui->menuLeftWords, left_arm, name_to_word["clearStack"]);
  ui->menuLeftWords->addAction(leftClearStackAction);


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::update() {
  if ( !isSketchyMat(right_arm->config.objectMapViewerImage)) {
    objectMapView.updateImage(right_arm->config.objectMapViewerImage);
  }
}


void MainWindow::setObjectMapViewMouseCallBack(EinMouseCallback m, void* param) {
  objectMapView.setMouseCallBack(m, param);
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
