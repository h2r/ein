#include "einwindow.h"
#include "ui_einwindow.h"
#include "mainwindow.h"

#include <QTest>

EinWindow::EinWindow(QWidget *parent, shared_ptr<MachineState> _ms) :
    QMainWindow(parent),
    ui(new Ui::EinWindow),
    myView(parent, CV_WINDOW_KEEPRATIO)
{
    ui->setupUi(this);
    ms = _ms;
    ui->imageFrame->layout()->addWidget(myView.getWidget());
}

void EinWindow::updateImage(const Mat image)  {
  myView.updateImage(image);
}

void EinWindow::setMouseCallBack(EinMouseCallback m, void* param) {
  myView.setMouseCallBack(m, param);
}


void EinWindow::keyPressEvent(QKeyEvent *evnt) {
  updateLastKey(evnt);
  QWidget::keyPressEvent(evnt);

}


EinWindow::~EinWindow()
{
    delete ui;
}
