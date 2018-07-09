#include "gaussianmapwidget.h"

#include "config.h"
#include "gaussian_map.h"


GaussianMapWidget::GaussianMapWidget(QWidget * parent, MachineState * _ms) : QWidget(parent),
                                                                           ms(_ms),
                                                                           ui(new Ui::GaussianMapWidget),
                                                                           meanView(parent, EIN_WINDOW_KEEPRATIO),
                                                                           stdDevView(parent, EIN_WINDOW_KEEPRATIO),
                                                                           heightView(parent, EIN_WINDOW_KEEPRATIO)
{
    ui->setupUi(this);
    //ui->meanImage->layout()->addWidget(meanView.getWidget());
    ui->tabs->widget(0)->layout()->addWidget(meanView.getWidget());
    ui->tabs->widget(1)->layout()->addWidget(stdDevView.getWidget());
    ui->tabs->widget(2)->layout()->addWidget(heightView.getWidget());
}

Mat GaussianMapWidget::selectedImage() {
  if (ui->tabs->currentIndex() == 0) {
    return meanImage;
  } else if (ui->tabs->currentIndex() == 1) {
    return stdDevImage;
  } else if (ui->tabs->currentIndex() == 2) {
    return heightImage;
  } else {
    cout << "Bad index: " << ui->tabs->currentIndex() << endl;
    assert(0);
  }
     
}
void GaussianMapWidget::updateMap(shared_ptr<GaussianMap> _map) 
{

  _map->rgbMuToBgrMat(meanImage);
  meanView.updateImage(meanImage);

  _map->rgbSigmaToMat(stdDevImage);
  stdDevView.updateImage(stdDevImage);


  _map->zMuToScaledMat(heightImage);
  heightView.updateImage(heightImage);
}
