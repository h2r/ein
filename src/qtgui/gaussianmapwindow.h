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

private:
    Ui::GaussianMapWindow *ui;
    MachineState * ms;
    GaussianMapWidget *widget;
};

#endif // GAUSSIANMAPWINDOW_H
