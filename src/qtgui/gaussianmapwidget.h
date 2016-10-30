#ifndef GAUSSIANMAPWIDGET_H
#define GAUSSIANMAPWIDGET_H

#include <QWidget>
#include "ein.h"
#include "window_QT.h"
#include "stackmodel.h"

#include "ui_gaussianmapwidget.h"

#include "capturelineedit.h"

class GaussianMapWidget : public QWidget
{
  Q_OBJECT
  
public: 
  GaussianMapWidget(QWidget * parent, MachineState * ms);

private:
    Ui::GaussianMapWidget *ui;
    MachineState * ms;
    shared_ptr<GaussianMap> myMap;
    DefaultEinViewPort meanView;
    DefaultEinViewPort stdDevView;
    DefaultEinViewPort heightView;


};


#endif // GAUSSIANMAPWIDGET_H
