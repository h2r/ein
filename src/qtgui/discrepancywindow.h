#ifndef DISCREPANCYWINDOW_H
#define DISCREPANCYWINDOW_H

#include <QMainWindow>
#include <iostream>
#include <cv.h>

#include "ein.h"
#include "window_QT.h"

using namespace std;

namespace Ui {
class DiscrepancyWindow;
}

class DiscrepancyWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit DiscrepancyWindow(QWidget *parent, MachineState * _ms);
    ~DiscrepancyWindow();

    void setWindowTitle(string s) {
      QMainWindow::setWindowTitle(QString::fromStdString(s));
    }
    void update(Mat & discrepancyImage, Mat & densityImage);
public slots:
    void saveImage();


private:
    Ui::DiscrepancyWindow *ui;
    MachineState * ms;
    DefaultEinViewPort discrepancyView;
    DefaultEinViewPort densityView;

    Mat discrepancyImage;
    Mat densityImage;

};

#endif // DISCREPANCYWINDOW_H
