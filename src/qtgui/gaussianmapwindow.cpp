#include "gaussianmapwindow.h"

#include "ui_gaussianmapwindow.h"

#include <QTest>
#include <boost/algorithm/string.hpp>


GaussianMapWindow::GaussianMapWindow(QWidget *parent, MachineState * _ms) :
    QMainWindow(parent),
    ui(new Ui::GaussianMapWindow)
{
    ui->setupUi(this);
    ms = _ms;
    widget = new GaussianMapWidget(this, ms);
    ui->frame->layout()->addWidget(widget);
}


GaussianMapWindow::~GaussianMapWindow()
{
    delete ui;
    delete widget;
}



