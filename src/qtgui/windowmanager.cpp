#include "windowmanager.h"


WindowManager::WindowManager() {
}

void WindowManager::setMenu(QMenu * _menu) {
  menu = _menu;

}

void WindowManager::addWindow(EinWindow * window) {
  windows.push_back(window);
  QAction * windowAction  = new QAction(menu);
  windowAction->setText(window->windowTitle());
  windowAction->setCheckable(true);
  windowAction->setChecked(window->isVisible());

  menu->addAction(windowAction);

  windowAction->connect(windowAction, SIGNAL(toggled(bool)), window, SLOT(toggleVisible(bool)));


  window->connect(window, SIGNAL(visibleChanged(bool)), windowAction, SLOT(setChecked(bool)));

}

