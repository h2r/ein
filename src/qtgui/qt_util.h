#ifndef QTUTIL_H
#define QTUTIL_H

class MachineState;
#include <QMainWindow>
#include <opencv2/opencv.hpp> 
#include "config.h"


void doSaveImage(MachineState * ms, QMainWindow * parent, const Mat & image);

#endif // QTUTIL_H
