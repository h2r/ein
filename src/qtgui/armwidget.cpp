#include "armwidget.h"



ArmWidget::ArmWidget(QWidget * parent, shared_ptr<MachineState> _ms) : QWidget(parent),
								       ms(_ms),
								       ui(new Ui::ArmWidget),
								       wristView(this, EIN_WINDOW_KEEPRATIO)
{
    ui->setupUi(this);
    ui->wristViewFrame->layout()->addWidget(wristView.getWidget());

    captureLineEdit = new CaptureLineEdit();

    ui->captureLineEditFrame->layout()->addWidget(captureLineEdit);

    if (ms == NULL) {
      return;
    }
    ms->config.armWidget = this;

    callStackModel = new StackModel(this);
    callStackModel->setStack(ms->call_stack);
    ui->callStackTableView->setModel(callStackModel);
    ui->callStackTableView->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);

    dataStackModel = new StackModel(this);
    dataStackModel->setStack(ms->data_stack);
    ui->dataStackTableView->setModel(dataStackModel);
    ui->dataStackTableView->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);


    wristView.setMouseCallBack(pilotCallbackFunc, ms.get());

    QStringList wordList;
    for (int i = 0; i < words.size(); i++) {
      wordList.push_back(QString::fromStdString(words[i]->name()));
    }
    completer = new QCompleter(wordList);
    ui->replEdit->setCompleter(completer);
    
    connect(ui->replEdit, SIGNAL(returnPressed()), this, SLOT(replReturnPressed()));

}


void ArmWidget::replReturnPressed() {
  string text = ui->replEdit->text().toStdString();
  cout << "Text: " << text << endl;
  ui->replEdit->setText("");
  ms->evaluateProgram(text);
}


void ArmWidget::update() {
  if (ms == NULL) {
    return;
  }
  if (captureLineEdit->last_key != -1) {
    ms->config.last_key = captureLineEdit->last_key;
    captureLineEdit->last_key = -1;
  }
  ui->stateLabel->setText(QString::fromStdString(ms->currentState()));
  callStackModel->setStack(ms->call_stack);
  dataStackModel->setStack(ms->data_stack);
  if ( !isSketchyMat(ms->config.wristViewImage)) {
     wristView.updateImage(ms->config.wristViewImage);
  }
}
