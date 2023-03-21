#ifndef GAUSSIANMAPWINDOW_H
#define GAUSSIANMAPWINDOW_H

#include "gaussianmapwidget.h"
#include <QMainWindow>

#include "window_QT.h"

using namespace std;

namespace Ui {
class GaussianMapWindow;
}

class GaussianMapWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit GaussianMapWindow(QWidget *parent, MachineState * _ms);
    ~GaussianMapWindow();

    void setWindowTitle(string s) {
      QMainWindow::setWindowTitle(QString::fromStdString(s));
    }
    void updateMap(shared_ptr<GaussianMap> map);
    void setVisible(bool);


public slots:
    void saveImage();
    void toggleVisible(bool show);

signals: 
  void visibleChanged(bool visible);


private:
    Ui::GaussianMapWindow *ui;
    MachineState * ms;
    GaussianMapWidget *widget;
};

#endif // GAUSSIANMAPWINDOW_H
