#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <opencv2/highgui/highgui_c.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    myView(parent, CV_WINDOW_KEEPRATIO)
{
    ui->setupUi(this);

    ui->imageFrame->layout()->addWidget(myView.getWidget());
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::updateImage(const Mat image) 
{
  myView.updateImage(image);
}
