#include "qt_util.h"

#include <string>
#include <QFileDialog>

#include <highgui.h>

void doSaveImage(MachineState * ms, QMainWindow * parent, const Mat & image) {
  cout << "Saving image." << endl;
  QString qFileName = QFileDialog::getSaveFileName(parent, "Save File",
                                                   ".",
                                                   "Images (*.png *.jpg)");
  string fileName = qFileName.toStdString();
  
  cout << "filename: " << fileName << endl;
  cout << "channels: " << image.channels() << endl;
  cout << "type: " << image.type() << endl;
  Mat dst;
  if (image.channels() == 1) {
    dst = image.clone();
    //dst = dst * 255;
  } else {
    dst = image;
  }

  if (boost::algorithm::ends_with(fileName, "jpg") || 
      boost::algorithm::ends_with(fileName, "png") || 
      boost::algorithm::ends_with(fileName, "ppm") || 
      boost::algorithm::ends_with(fileName, "tif") || 
      boost::algorithm::ends_with(fileName, "bmp")) {
    imwrite(fileName, dst);
  } else {
    CONSOLE_ERROR(ms, "Bad extension for " << fileName << endl);
  }
}
