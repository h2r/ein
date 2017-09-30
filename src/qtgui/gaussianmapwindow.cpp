#include "gaussianmapwindow.h"

#include "ui_gaussianmapwindow.h"

#include <QTest>
#include <boost/algorithm/string.hpp>
#include "ein.h"
#include "einwindow.h"

GaussianMapWindow::GaussianMapWindow(QWidget *parent, MachineState * _ms) :
    QMainWindow(parent),
    ui(new Ui::GaussianMapWindow)
{
    ui->setupUi(this);
    ms = _ms;
    widget = new GaussianMapWidget(this, ms);
    ui->frame->layout()->addWidget(widget);
    ui->menubar->setVisible(true);
    connect(ui->actionSaveImage, SIGNAL(triggered()), this, SLOT(saveImage()));


}

void GaussianMapWindow::saveImage() {
  doSaveImage(this, widget->selectedImage());
}


GaussianMapWindow::~GaussianMapWindow()
{
    delete ui;
    delete widget;
}



void GaussianMapWindow::updateMap(shared_ptr<GaussianMap> map) 
{
  widget->updateMap(map);
}


void GaussianMapWindow::toggleVisible(bool show) {
  setVisible(show);
}

void GaussianMapWindow::setVisible(bool show) {
  QMainWindow::setVisible(show);
  emit visibleChanged(show);
}
