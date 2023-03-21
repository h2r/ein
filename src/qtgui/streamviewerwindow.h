#ifndef STREAMVIEWERWINDOW_H
#define STREAMVIEWERWINDOW_H

#include <QMainWindow>
#include <iostream>

#include "window_QT.h"
class MachineState;

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
    void setVisible(bool);

public slots:
    void saveImage();
    void timeValueChanged(int value);
    void toggleVisible(bool show);

signals: 
  void visibleChanged(bool visible);


private:
    Ui::StreamViewerWindow *ui;
    MachineState * ms;

    DefaultEinViewPort streamImageView;
};

#endif // STREAMVIEWERWINDOW_H
