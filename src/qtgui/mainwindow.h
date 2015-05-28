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

    void updateImage(const Mat /*arr*/);

private:
    Ui::MainWindow *ui;


    DefaultViewPort myView;
};

#endif // MAINWINDOW_H
