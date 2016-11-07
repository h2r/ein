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
  stringstream txt;
  txt << "Image: " << camera->sibCurIdx << " of " << camera->streamImageBuffer.size() << endl;




  if (tsi != NULL) {
    eePose tArmP, tBaseP;
    int success = getStreamPoseAtTime(ms, tsi->time, &tArmP, &tBaseP);
    cout << "Got pose: " << success << endl;
    if (!isSketchyMat(tsi->image)) {
      streamImageView.updateImage(tsi->image);
    }
    streamImage * start = &camera->streamImageBuffer[0];
    streamImage * end = &camera->streamImageBuffer[camera->streamImageBuffer.size() - 1];
    tsi->time;
    double length = end->time - start->time;
    double secondOffset = tsi->time - start->time;
    txt << secondOffset << " of " << length << " seconds";
  }

  ui->imageLabel->setText(QString::fromStdString(txt.str()));
}


void StreamViewerWindow::saveImage() {
  //doSaveImage(this, widget->selectedImage());
}
