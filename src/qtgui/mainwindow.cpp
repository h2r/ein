#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    myView(parent, 0)
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
