#include "armwidget.h"



ArmWidget::ArmWidget(QWidget * parent, shared_ptr<MachineState> _ms) : QWidget(parent),
								       ms(_ms),
								       ui(new Ui::ArmWidget),
								       wristView(this, EIN_WINDOW_KEEPRATIO)
{
    ui->setupUi(this);
    ui->wristViewFrame->layout()->addWidget(wristView.getWidget());

    if (ms == NULL) {
      return;
    }
    ms->config.armWidget = this;

    callStackModel = new StackModel(this);
    callStackModel->setMachineState(ms);
    ui->callStackTableView->setModel(callStackModel);

    ui->callStackTableView->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);

    wristView.setMouseCallBack(pilotCallbackFunc, ms.get());

}


void ArmWidget::update() {
  if (ms == NULL) {
    return;
  }
  ui->stateLabel->setText(QString::fromStdString(ms->currentState()));
  callStackModel->setMachineState(ms);
  if ( !isSketchyMat(ms->config.wristViewImage)) {
     wristView.updateImage(ms->config.wristViewImage);
  }
}
