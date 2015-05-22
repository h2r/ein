#ifndef EINWINDOW_H
#define EINWINDOW_H

#include <QMainWindow>
#include <iostream>
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

private:
    Ui::EinWindow *ui;
    shared_ptr<MachineState> ms;
    QImage image;

};

#endif // EINWINDOW_H
