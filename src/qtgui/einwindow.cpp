#include "einwindow.h"
#include "ui_einwindow.h"
#include "mainwindow.h"

#include <highgui.h>
#include <QTest>
#include <boost/algorithm/string.hpp>


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
  doSaveImage(this, myImage);
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

void doSaveImage(QMainWindow * parent, const Mat & image) {
  cout << "Saving image." << endl;
  QString qFileName = QFileDialog::getSaveFileName(parent, "Save File",
                                                   ".",
                                                   "Images (*.png *.jpg)");
  string fileName = qFileName.toStdString();
  
  cout << "filename: " << fileName << endl;
  cout << "channels: " << image.channels() << endl;
  cout << "type: " << image.type() << endl;
  Mat dst;
  if (image.channels() == 1) {
    dst = image.clone();
    //dst = dst * 255;
  } else {
    dst = image;
  }

  cout << "dst channels: " << dst.channels() << endl;

  if (boost::algorithm::ends_with(fileName, "jpg") || 
      boost::algorithm::ends_with(fileName, "png") || 
      boost::algorithm::ends_with(fileName, "ppm") || 
      boost::algorithm::ends_with(fileName, "tif") || 
      boost::algorithm::ends_with(fileName, "bmp")) {
    imwrite(fileName, dst);
  } else {
    ROS_ERROR_STREAM("Bad extension for " << fileName << endl);
  }
}
