#ifndef ARMWIDGET_H
#define ARMWIDGET_H

#include <QWidget>
#include "ein.h"
#include "window_QT.h"
#include "stackmodel.h"

#include "ui_armwidget.h"

#include "capturelineedit.h"

class ArmWidget : public QWidget
{
  Q_OBJECT
  
public: 
  ArmWidget(QWidget * parent, MachineState * ms);
  void update();
  DefaultEinViewPort wristView;

public slots:
  void replReturnPressed();

private:
    Ui::ArmWidget *ui;
    StackModel * dataStackModel;
    StackModel * callStackModel;

    CaptureLineEdit * captureLineEdit;
    MachineState * ms;
    
    QCompleter * completer;

};


#endif // ARMWIDGET_H
