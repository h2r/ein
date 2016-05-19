#include "gaussianmapwidget.h"



GaussianMapWidget::GaussianMapWidget(QWidget * parent, shared_ptr<MachineState> _ms) : QWidget(parent),
                                                                                       ms(_ms),
                                                                                       ui(new Ui::GaussianMapWidget),
                                                                                       meanView(parent, EIN_WINDOW_KEEPRATIO),
                                                                                       stdDevView(parent, EIN_WINDOW_KEEPRATIO),
                                                                                       heightView(parent, EIN_WINDOW_KEEPRATIO)
{
    ui->setupUi(this);


    ui->tabs->widget(0)->layout()->addWidget(meanView.getWidget());


}

void GaussianMapWidget::updateMap(shared_ptr<GaussianMap> map) {
  myMap = map;
}

void GaussianMapWidget::render() {
  
}

