#include "streamviewerwindow.h"

#include "ui_streamviewerwindow.h"

#include <QTest>
#include <boost/algorithm/string.hpp>

#include "einwindow.h"
#include "camera.h"

StreamViewerWindow::StreamViewerWindow(QWidget *parent, MachineState * _ms) :
    QMainWindow(parent),
    ui(new Ui::StreamViewerWindow),
    streamImageView(parent, EIN_WINDOW_KEEPRATIO)
{
    ui->setupUi(this);
    ms = _ms;

    ui->menubar->setVisible(true);
    connect(ui->actionSaveImage, SIGNAL(triggered()), this, SLOT(saveImage()));
    ui->streamFrame->layout()->addWidget(streamImageView.getWidget());
    
}


StreamViewerWindow::~StreamViewerWindow()
{
    delete ui;
}



void StreamViewerWindow::update()
{
  //widget->updateMap(map);
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  streamImage * tsi = camera->currentImage();
  loadStreamImage(ms, tsi);

  cout << "Current stream image: " << tsi << endl;
  if (tsi != NULL) {
    eePose tArmP, tBaseP;
    int success = getStreamPoseAtTime(ms, tsi->time, &tArmP, &tBaseP);
    cout << "Got pose: " << success << endl;
    if (!isSketchyMat(tsi->image)) {
      cout << "rendering image: " << tsi->image << endl;
      streamImageView.updateImage(tsi->image);
    }
  }
  
}


void StreamViewerWindow::saveImage() {
  //doSaveImage(this, widget->selectedImage());
}
