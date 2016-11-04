#include "discrepancywindow.h"

#include "ui_discrepancywindow.h"

#include <QTest>
#include <boost/algorithm/string.hpp>

#include "einwindow.h"

DiscrepancyWindow::DiscrepancyWindow(QWidget *parent, MachineState * _ms) :
    QMainWindow(parent),
    ui(new Ui::DiscrepancyWindow),
    discrepancyView(this, EIN_WINDOW_KEEPRATIO),
    densityView(this, EIN_WINDOW_KEEPRATIO)
{
    ui->setupUi(this);
    ms = _ms;
    ui->menubar->setVisible(true);
    connect(ui->actionSaveImage, SIGNAL(triggered()), this, SLOT(saveImage()));

    ui->tabs->widget(0)->layout()->addWidget(discrepancyView.getWidget());
    ui->tabs->widget(1)->layout()->addWidget(densityView.getWidget());




}


DiscrepancyWindow::~DiscrepancyWindow()
{
    delete ui;
}



void DiscrepancyWindow::update(Mat & _discrepancyImage, Mat & _densityImage)
{
  //widget->updateMap(map);
  discrepancyImage = _discrepancyImage;
  densityImage = _densityImage;
  discrepancyView.updateImage(discrepancyImage);
  densityView.updateImage(densityImage);
}


void DiscrepancyWindow::saveImage() {
  if (ui->tabs->currentIndex() == 0) {
    doSaveImage(this, discrepancyImage);
  } else if (ui->tabs->currentIndex() == 1) {
    doSaveImage(this, densityImage);
  } else {
    cout << "Bad index: " << ui->tabs->currentIndex() << endl;
    assert(0);
  }


}
