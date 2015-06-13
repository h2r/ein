#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QTest>

#include <opencv2/highgui/highgui_c.h>
int last_key = -1;

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


void MainWindow::keyPressEvent(QKeyEvent *evnt) {

  int key = evnt->key();

  Qt::Key qtkey = static_cast<Qt::Key>(key);
  char asciiCode = QTest::keyToAscii(qtkey);
  if (asciiCode != 0)
    key = static_cast<int>(asciiCode);
  else
    key = evnt->nativeVirtualKey(); //same codes as returned by GTK-based backend

  //control plus (Z, +, -, up, down, left, right) are used for zoom/panning functions
  if (evnt->modifiers() != Qt::ControlModifier)
    {
      last_key = key;
    }

  QWidget::keyPressEvent(evnt);
}
