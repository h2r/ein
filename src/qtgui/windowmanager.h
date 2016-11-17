#ifndef WINDOWMANAGER_H
#define WINDOWMANAGER_H

#include "einwindow.h"
#include <vector>
using namespace std;

class WindowManager {

 public:
  WindowManager();
  void addWindow(QMainWindow * window);

  void setMenu(QMenu * menu);

 private:
  QMenu * menu;

  vector<QMainWindow *> windows;
  
};


#endif // WINDOWMANAGER_H
