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
//void MainWindow::updateImage(void * arr) 
{
  //CvMat* mat, stub;
    
  //mat = cvGetMat(arr, &stub);

  //cv::Mat im = cv::cvarrToMat(mat);
  myView.updateImage(image);
  
}

void timercallback1(const ros::TimerEvent&); 

void MainWindow::rosSpin()
{
  ros::TimerEvent e;
  timercallback1(e);
  ros::spinOnce();
}


void MainWindow::setMouseCallBack(EinMouseCallback m, void* param) {
  myView.setMouseCallBack(m, param);
}
