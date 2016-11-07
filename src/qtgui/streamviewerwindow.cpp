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

    //connect(ui->timeSlider, SIGNAL(valueChanged(int)), this, SLOT(timeValueChanged(int)));
    connect(ui->timeSlider, SIGNAL(actionTriggered(int)), this, SLOT(timeValueChanged(int)));

    
}

void StreamViewerWindow::timeValueChanged(int v) 
{
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  int value = ui->timeSlider->sliderPosition();
  double fraction = value / 100.0;
  int targetIdx = (int) (camera->streamImageBuffer.size() * fraction);
  if (targetIdx >= camera->streamImageBuffer.size()) {
    targetIdx = camera->streamImageBuffer.size() - 1;
  }
  camera->setIsbIdxNoLoadNoKick(targetIdx);
  //ms->evaluateProgram("streamRenderStreamWindow");
  update();
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
  txt << "Image: " << camera->sibCurIdx + 1 << " of " << camera->streamImageBuffer.size() << endl;

  double fraction = (double) camera->sibCurIdx / camera->streamImageBuffer.size();
  int value = (int) (fraction * 100);
  ui->timeSlider->setValue(value);



  if (tsi != NULL) {
    eePose tArmP, tBaseP;
    int success = getStreamPoseAtTime(ms, tsi->time, &tArmP, &tBaseP);

    if (!isSketchyMat(tsi->image)) {
      streamImageView.updateImage(tsi->image);
    }
    streamImage * start = &camera->streamImageBuffer[0];
    streamImage * end = &camera->streamImageBuffer[camera->streamImageBuffer.size() - 1];
    tsi->time;
    double length = end->time - start->time;
    double secondOffset = tsi->time - start->time;
    txt << secondOffset << " of " << length << " seconds" << endl;

    if (success) {
      shared_ptr<EePoseWord> w = make_shared<EePoseWord>(tArmP);
      txt << w->repr() << endl;
    }
  }

  ui->imageLabel->setText(QString::fromStdString(txt.str()));
}


void StreamViewerWindow::saveImage() {
  //doSaveImage(this, widget->selectedImage());
}
