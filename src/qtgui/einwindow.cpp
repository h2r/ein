#include "einwindow.h"
#include "ui_einwindow.h"
#include "mainwindow.h"
#include "config.h"


#include <QTest>
#include <boost/algorithm/string.hpp>
#include "qt_util.h"

EinWindow::EinWindow(QWidget *parent, MachineState * _ms) :
    QMainWindow(parent),
    ui(new Ui::EinWindow),
    myView(parent, EIN_WINDOW_KEEPRATIO)
{
    ui->setupUi(this);
    ms = _ms;
    ui->imageFrame->layout()->addWidget(myView.getWidget());
    //cout << "menu: " << ui->menubar << endl;
    //cout << "menu: " << ui->menubar->isVisible() << endl;
    ui->menubar->setVisible(true);
    connect(ui->actionSaveImage, SIGNAL(triggered()), this, SLOT(saveImage()));
}


void EinWindow::saveImage() {
  doSaveImage(ms, this, myImage);
}

void EinWindow::updateImage(const Mat image)  {
  if ( !isSketchyMat(image)) {
    myView.updateImage(image);
    myImage = image;
  }
}

void EinWindow::setMouseCallBack(EinMouseCallback m, void* param) {
  myView.setMouseCallBack(m, param);
}


void EinWindow::keyPressEvent(QKeyEvent *evnt) {
  QWidget::keyPressEvent(evnt);

}


EinWindow::~EinWindow()
{
    delete ui;
}

void EinWindow::toggleVisible(bool show) {
  setVisible(show);
}

void EinWindow::setVisible(bool show) {
  QMainWindow::setVisible(show);
  emit visibleChanged(show);
}

