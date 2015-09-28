#include "capturelineedit.h"

#include <QTest>
#include <iostream>
using namespace std;

void CaptureLineEdit::keyPressEvent(QKeyEvent * event)
 {
   QWidget::keyPressEvent(event);

   int key = event->key();
   
   Qt::Key qtkey = static_cast<Qt::Key>(key);
   char asciiCode = QTest::keyToAscii(qtkey);
   if (asciiCode != 0) {
     key = static_cast<int>(asciiCode);
   } else {
     key = event->nativeVirtualKey(); //same codes as returned by GTK-based backend
   }
   key = event->nativeVirtualKey();

   int modifiers = event->nativeModifiers();
   key |= modifiers << 16;

   //control plus (Z, +, -, up, down, left, right) are used for zoom/panning functions
   if (event->modifiers() != Qt::ControlModifier)
    {
      last_key = key;
      cout << "Got key: " << last_key << endl;
    }
 }

