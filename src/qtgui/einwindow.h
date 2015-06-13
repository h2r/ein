#ifndef EINWINDOW_H
#define EINWINDOW_H

#include <QMainWindow>
#include <iostream>
#include <cv.h>

#include "ein.h"
#include "window_QT.h"

using namespace std;

namespace Ui {
class EinWindow;
}

class EinWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit EinWindow(QWidget *parent, shared_ptr<MachineState> _ms);
    ~EinWindow();
    void showImage(CvMat mat);
    Q_INVOKABLE void updateImage(const Mat image) ;
    void setMouseCallBack(EinMouseCallback m, void* param);
    void keyPressEvent(QKeyEvent *evnt);

    void setWindowTitle(string s) {
      QMainWindow::setWindowTitle(QString::fromStdString(s));
    }

    void setVisible(bool);

public slots:
    void toggleVisible(bool show);

signals: 
  void visibleChanged(bool visible);

private:
    Ui::EinWindow *ui;
    shared_ptr<MachineState> ms;
    DefaultEinViewPort myView;
};

#endif // EINWINDOW_H
