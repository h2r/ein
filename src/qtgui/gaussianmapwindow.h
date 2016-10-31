#ifndef GAUSSIANMAPWINDOW_H
#define GAUSSIANMAPWINDOW_H

#include "gaussianmapwidget.h"
#include <QMainWindow>
#include <iostream>
#include <cv.h>

#include "ein.h"
#include "window_QT.h"

using namespace std;

namespace Ui {
class GaussianMapWindow;
}

class GaussianMapWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit GaussianMapWindow(QWidget *parent, MachineState * _ms);
    ~GaussianMapWindow();

    void setWindowTitle(string s) {
      QMainWindow::setWindowTitle(QString::fromStdString(s));
    }
    void updateMap(shared_ptr<GaussianMap> map);
public slots:
    void saveImage();


private:
    Ui::GaussianMapWindow *ui;
    MachineState * ms;
    GaussianMapWidget *widget;
};

#endif // GAUSSIANMAPWINDOW_H
