#ifndef WORDACTION_H
#define WORDACTION_H

#include <QAction>


#include "einwindow.h"
#include <memory>

using namespace std;
#include <QAction>
#include <QMenu>
class Word;
class MachineState;

class WordAction: public QAction {

    Q_OBJECT

public:
  WordAction(QMenu*, MachineState * ms, shared_ptr<Word> word);

public slots:
 void pushWord();
 void execute();

 private:
  MachineState * ms;
  shared_ptr<Word> word;
  
};


#endif // WORDACTION_H
