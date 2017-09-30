#include "wordaction.h"

#include "word.h"

WordAction::WordAction(QMenu * _menu, MachineState * _ms, shared_ptr<Word> _word) : QAction(_menu){
  ms = _ms;
  word = _word;
  setText(QString::fromStdString(word->name()));
  connect(this, SIGNAL(triggered()), this, SLOT(execute()));
  //connect(this, SIGNAL(triggered()), this, SLOT(pushWord()));

}

void WordAction::pushWord() {
  ms->pushWord(word);
}

void WordAction::execute() {
  ms->execute(word);
}

