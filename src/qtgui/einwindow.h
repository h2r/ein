#ifndef EINWINDOW_H
#define EINWINDOW_H

#include <QMainWindow>
#include <iostream>
#include <cv.h>


#include "ein_util.h"

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

private:
    Ui::EinWindow *ui;
    shared_ptr<MachineState> ms;

};

#endif // EINWINDOW_H
