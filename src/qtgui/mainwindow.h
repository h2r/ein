#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <iostream>
#include <ros/ros.h>

#include "ein.h"
#include "window_QT.h"
#include "stackmodel.h"
#include "windowmanager.h"
#include "armwidget.h"

using namespace std;
namespace Ui {
class MainWindow;
}


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0, MachineState * _right_arm = 0, MachineState * _left_arm = 0);
    ~MainWindow();
    void setWristViewMouseCallBack(EinMouseCallback m, void* param);
    void setObjectMapViewMouseCallBack(EinMouseCallback m, void* param);
    void keyPressEvent(QKeyEvent *evnt);
    void addWindow(EinWindow * window);
    void update();

private:
    Ui::MainWindow *ui;
    WindowManager windowManager;
    ArmWidget rightArmWidget;
    ArmWidget leftArmWidget;
    DefaultEinViewPort objectMapView;
    MachineState * right_arm;
    MachineState * left_arm;
};

#endif // MAINWINDOW_H
