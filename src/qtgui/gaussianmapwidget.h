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

  void updateMap(shared_ptr<GaussianMap> map);
  Mat selectedImage();

private:
    Ui::GaussianMapWidget *ui;
    MachineState * ms;
    DefaultEinViewPort meanView;
    DefaultEinViewPort stdDevView;
    DefaultEinViewPort heightView;

    Mat meanImage;
    Mat stdDevImage;
    Mat heightImage;
};


#endif // GAUSSIANMAPWIDGET_H
