#ifndef EINWINDOW_H
#define EINWINDOW_H

#include <QMainWindow>

#include "window_QT.h"
class MachineState;

using namespace std;



namespace Ui {
class EinWindow;
}

class EinWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit EinWindow(QWidget *parent, MachineState * _ms);
    ~EinWindow();
  void showImage(cv::Mat mat);
  Q_INVOKABLE void updateImage(const cv::Mat image) ;
    Mat myImage;
    void setMouseCallBack(EinMouseCallback m, void* param);
    void keyPressEvent(QKeyEvent *evnt);

    void setWindowTitle(string s) {
      QMainWindow::setWindowTitle(QString::fromStdString(s));
    }

    void setVisible(bool);


public slots:
    void toggleVisible(bool show);
    void saveImage();

signals: 
  void visibleChanged(bool visible);

private:
    Ui::EinWindow *ui;
    MachineState * ms;
    DefaultEinViewPort myView;
};

#endif // EINWINDOW_H
