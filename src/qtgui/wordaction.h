#ifndef WORDACTION_H
#define WORDACTION_H

#include <QAction>


#include "einwindow.h"
#include <vector>

using namespace std;
#include <QAction>
#include <QMenu>
#include "word.h"

class WordAction: public QAction {

    Q_OBJECT

public:
  WordAction(QMenu*, shared_ptr<MachineState> ms, shared_ptr<Word> word);

public slots:
 void pushWord();
 void execute();

 private:
  shared_ptr<MachineState> ms;
  shared_ptr<Word> word;
  
};


#endif // WORDACTION_H
