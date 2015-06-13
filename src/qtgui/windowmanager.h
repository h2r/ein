#ifndef WINDOWMANAGER_H
#define WINDOWMANAGER_H

#include "einwindow.h"
#include <vector>
using namespace std;

class WindowManager {

 public:
  WindowManager();
  void addWindow(EinWindow * window);

  void setMenu(QMenu * menu);

 private:
  QMenu * menu;

  vector<EinWindow *> windows;
  
};


#endif // WINDOWMANAGER_H
