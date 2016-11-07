#ifndef STREAMVIEWERWINDOW_H
#define STREAMVIEWERWINDOW_H

#include <QMainWindow>
#include <iostream>
#include <cv.h>

#include "ein.h"
#include "window_QT.h"

using namespace std;

namespace Ui {
class StreamViewerWindow;
}

class StreamViewerWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit StreamViewerWindow(QWidget *parent, MachineState * _ms);
    ~StreamViewerWindow();

    void setWindowTitle(string s) {
      QMainWindow::setWindowTitle(QString::fromStdString(s));
    }
    void update();
public slots:
    void saveImage();
    void timeValueChanged(int value);

private:
    Ui::StreamViewerWindow *ui;
    MachineState * ms;

    DefaultEinViewPort streamImageView;
};

#endif // STREAMVIEWERWINDOW_H
