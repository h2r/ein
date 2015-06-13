#include "windowmanager.h"


WindowManager::WindowManager() {
}

void WindowManager::setMenu(QMenu * _menu) {
  menu = _menu;

}

void WindowManager::addWindow(EinWindow * window) {
  windows.push_back(window);
  menu->addAction(window->windowTitle());
}
