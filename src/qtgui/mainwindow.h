#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <iostream>
#include <ros/ros.h>

#include "window_QT.h"

using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    Q_INVOKABLE void updateImage(const Mat image) ;
    void setup();
    void setMouseCallBack(EinMouseCallback m, void* param);
public slots:
    void rosSpin();

private:
    Ui::MainWindow *ui;


    DefaultEinViewPort myView;
};

#endif // MAINWINDOW_H
