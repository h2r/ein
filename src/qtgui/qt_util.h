#ifndef QTUTIL_H
#define QTUTIL_H

class MachineState;
#include <QMainWindow>
#include <cv.h>
#include "config.h"


void doSaveImage(MachineState * ms, QMainWindow * parent, const Mat & image);

#endif // QTUTIL_H
