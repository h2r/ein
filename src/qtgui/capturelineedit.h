#ifndef CAPTURELINEEDIT_H
#define CAPTURELINEEDIT_H

#include <QLineEdit>
#include <QKeyEvent>

class CaptureLineEdit : public QLineEdit
{
  Q_OBJECT

 public:
  int last_key;
  void keyPressEvent(QKeyEvent * event);

};


#endif // CAPTURELINEEDIT_H
