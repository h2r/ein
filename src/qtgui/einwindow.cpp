#include "einwindow.h"
#include "ui_einwindow.h"


EinWindow::EinWindow(QWidget *parent, shared_ptr<MachineState> _ms) :
    QMainWindow(parent),
    ui(new Ui::EinWindow)
{
    ui->setupUi(this);
    ms = _ms;
}

EinWindow::~EinWindow()
{
    delete ui;
}
