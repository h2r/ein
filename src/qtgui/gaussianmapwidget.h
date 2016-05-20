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
  GaussianMapWidget(QWidget * parent, shared_ptr<MachineState> ms);

  void updateMap(shared_ptr<GaussianMap> map);
  void render();
private:
    Ui::GaussianMapWidget *ui;
    shared_ptr<MachineState> ms;
    shared_ptr<GaussianMap> myMap;
    DefaultEinViewPort meanView;
    DefaultEinViewPort stdDevView;
    DefaultEinViewPort heightView;


};


#endif // GAUSSIANMAPWIDGET_H
