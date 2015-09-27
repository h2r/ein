#ifndef ARMWIDGET_H
#define ARMWIDGET_H

#include <QWidget>
#include "ein.h"
#include "window_QT.h"
#include "stackmodel.h"

#include "ui_armwidget.h"



class ArmWidget : public QWidget
{
  Q_OBJECT
  
public: 
ArmWidget(QWidget * parent, shared_ptr<MachineState> ms);
void update();
    DefaultEinViewPort wristView;

private:
    Ui::ArmWidget *ui;
    StackModel * stackModel;

    shared_ptr<MachineState> ms;

};


#endif // ARMWIDGET_H
