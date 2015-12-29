
#include "ein_words.h"
#include "ein.h"



namespace ein_words {

WORD(PublishState)
virtual void execute(std::shared_ptr<MachineState> ms) {
  {
    EinState state;
    fillEinStateMsg(ms, &state);
    ms->config.einPub.publish(state);
  }
}
END_WORD
REGISTER_WORD(PublishState)

WORD(Drand48)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushData(std::make_shared<DoubleWord>(drand48()));
}
END_WORD
REGISTER_WORD(Drand48)

WORD(PushTime)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushData(std::make_shared<DoubleWord>(ros::Time::now().toSec()));
}
END_WORD
REGISTER_WORD(PushTime)

WORD(Throw)
virtual void execute(std::shared_ptr<MachineState> ms) {
  throw runtime_error("test");
  
}
END_WORD
REGISTER_WORD(Throw)

WORD(ThrowOpenCV)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Mat m;
  m.rowRange(100, 100);
}
END_WORD
REGISTER_WORD(ThrowOpenCV)

WORD(SeeHz)
virtual void execute(std::shared_ptr<MachineState> ms) {

  double its = 0;
  GET_NUMERIC_ARG(ms, its);
  
  double start = ros::Time::now().toSec();
  int i = 0;
  for (i = 0; i < its; ) {
    i++;
  }
  double end = ros::Time::now().toSec();

  double hz = double(i) / (end-start);

  ms->pushData(std::make_shared<DoubleWord>(hz));
}
END_WORD
REGISTER_WORD(SeeHz)

WORD(EndArgs)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // marks the end of a list of arguments 
  ms->pushData("endArgs");
}
END_WORD
REGISTER_WORD(EndArgs)

WORD(UploadObjectToDatabase)
virtual void execute(std::shared_ptr<MachineState> ms) {
  stringstream cmd;
  string className;
  GET_ARG(ms, StringWord, className);

  cmd << "bash -c \"rosrun ein upload_zips.py -u 'maria' -p 'maria' ";
  cmd << ms->config.data_directory << "/objects/" << className << "\"";
  cout << "Running: " << cmd.str() << endl;
  system(cmd.str().c_str());
  
}
END_WORD
REGISTER_WORD(UploadObjectToDatabase)


WORD(ZeroGToggle)
CODE('z')
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.zero_g_toggle = !ms->config.zero_g_toggle;
}
END_WORD
REGISTER_WORD(ZeroGToggle)

WORD(ZeroGOn)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.zero_g_toggle = 1;
}
END_WORD
REGISTER_WORD(ZeroGOn)

WORD(ZeroGOff)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.zero_g_toggle = 0;
}
END_WORD
REGISTER_WORD(ZeroGOff)

WORD(ClearStack)
CODE('r') 
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->clearStack();
}
END_WORD
REGISTER_WORD(ClearStack)

WORD(ClearStacks)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->clearData();
  ms->clearStack();
}
END_WORD
REGISTER_WORD(ClearStacks)

WORD(Beep)
CODE(1245308)     // capslock + numlock + |
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "\a"; cout.flush();
}
END_WORD
REGISTER_WORD(Beep)


WORD(ChangeToCounterTable)
CODE(1179735) // capslock + numlock + w
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentTableZ = ms->config.counterTableZ;
}
END_WORD
REGISTER_WORD(ChangeToCounterTable)
  
WORD(ChangeToPantryTable)
CODE(1179717)    // capslock + numlock + e
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentTableZ = ms->config.pantryTableZ;
}
END_WORD
REGISTER_WORD(ChangeToPantryTable)


WORD(ExecuteStack)
CODE('y')

virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back(";");
  return result;
}

virtual void execute(std::shared_ptr<MachineState> ms) {
  // XXX This word is special cased.  It needs to be special cased,
  // because if the stack is paused, then the word to unpause the
  // stack is never executed, because the stack is paused.  So the
  // code that processes words checks if the top of the stack is
  // executeStack, and always executes it, whether or not the stack is
  // paused.
  ms->execute_stack = 1;
}
END_WORD
REGISTER_WORD(ExecuteStack)

WORD(PauseStackExecution)
CODE('Y') 
virtual void execute(std::shared_ptr<MachineState> ms)  {
  cout << "STACK EXECUTION PAUSED, press 'y' to continue." << endl;
  ms->execute_stack = 0;
  ms->config.endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(PauseStackExecution)


 
WORD(PauseAndReset)
CODE('c') 
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->execute_stack = 0;
  ms->config.lastPtheta = INFINITY;
}
END_WORD
REGISTER_WORD(PauseAndReset)


WORD(PushState)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string state = ms->currentState();
  shared_ptr<StringWord> outword = std::make_shared<StringWord>(state);
  ms->pushData(outword);
}
END_WORD
REGISTER_WORD(PushState)



WORD(PrintState)
CODE('u')
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("print");
  ms->pushWord("pushState");
}
END_WORD
REGISTER_WORD(PrintState)

WORD(DecrementTargetClass)
CODE(196438)     // capslock + pagedown
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "targetClass-- " << endl;
  if (ms->config.numClasses > 0) {
    int newTargetClass = (ms->config.targetClass - 1 + ms->config.numClasses) % ms->config.numClasses;
    changeTargetClass(ms, newTargetClass);
  }
}
END_WORD
REGISTER_WORD(DecrementTargetClass)

WORD(Or)
CODE('|') 
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("||");
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  double v2;
  GET_NUMERIC_ARG(ms, v2);

  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(int(v1) || int(v2));
  ms->pushWord(newWord);

}
END_WORD
REGISTER_WORD(Or)

WORD(And)
CODE('&') 
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("&&");
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  double v2;
  GET_NUMERIC_ARG(ms, v2);

  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(int(v1) && int(v2));
  ms->pushWord(newWord);

}
END_WORD
REGISTER_WORD(And)

WORD(Plus)
CODE('+') 
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("+");
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  //double v1;
  //GET_NUMERIC_ARG(ms, v1);
  //double v2;
  //GET_NUMERIC_ARG(ms, v2);
  //std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(v1 + v2);
  //ms->pushWord(newWord);

  shared_ptr<Word> w1;
  GET_WORD_ARG(ms, Word, w1);
  shared_ptr<Word> w2;
  GET_WORD_ARG(ms, Word, w2);

  std::shared_ptr<StringWord> s1 = std::dynamic_pointer_cast<StringWord>(w1);
  std::shared_ptr<StringWord> s2 = std::dynamic_pointer_cast<StringWord>(w2);
  std::shared_ptr<IntegerWord> i1 = std::dynamic_pointer_cast<IntegerWord>(w1);
  std::shared_ptr<IntegerWord> i2 = std::dynamic_pointer_cast<IntegerWord>(w2);
  std::shared_ptr<DoubleWord> d1 = std::dynamic_pointer_cast<DoubleWord>(w1);
  std::shared_ptr<DoubleWord> d2 = std::dynamic_pointer_cast<DoubleWord>(w2);

  if (i1 != NULL && i2 != NULL) {
    ms->pushWord(make_shared<IntegerWord>(w1->to_int() + w2->to_int()));
  } else if ((i1 != NULL && d2 != NULL) || (d1 != NULL && i2 != NULL) || (d1 != NULL && d2 != NULL)) {
    ms->pushWord(make_shared<DoubleWord>(w1->to_double() + w2->to_double()));
  } else {
    stringstream buf;
    buf << w2->to_string() << w1->to_string();
    ms->pushWord(make_shared<StringWord>(buf.str()));
  }
}
END_WORD
REGISTER_WORD(Plus)

WORD(Langle)
CODE('<') 
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("<");
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  double v2;
  GET_NUMERIC_ARG(ms, v2);

  std::shared_ptr<IntegerWord> newWord = std::make_shared<IntegerWord>(v2 < v1);
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(Langle)

WORD(Rangle)
CODE('>') 
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back(">");
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  double v2;
  GET_NUMERIC_ARG(ms, v2);

  std::shared_ptr<IntegerWord> newWord = std::make_shared<IntegerWord>(v2 > v1);
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(Rangle)

WORD(Times)
CODE('*') 
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("*");
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  double v2;
  GET_NUMERIC_ARG(ms, v2);

  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(v2 * v1);
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(Times)

WORD(Divide)
CODE('/') 
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("/");
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  double v2;
  GET_NUMERIC_ARG(ms, v2);

  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(v2 / v1);
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(Divide)

WORD(Minus)
CODE('-') 
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("-");
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  double v2;
  GET_NUMERIC_ARG(ms, v2);

  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(v2 - v1);
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(Minus)

WORD(Equals)
CODE('=') 
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("=");
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms) {

  std::shared_ptr<Word> p1 = ms->popData();
  std::shared_ptr<Word> p2 = ms->popData();

  if (p1 == NULL || p2 == NULL) {
    cout << "Warning, requires two words on the stack." << endl;
    return;
  }
  int new_value;
  if (p1->equals(p2)) {
    new_value = 1;
  } else {
    new_value = 0;
  }
  std::shared_ptr<IntegerWord> newWord = std::make_shared<IntegerWord>(new_value);
  ms->pushData(newWord);
}
END_WORD
REGISTER_WORD(Equals)


WORD(Not)
CODE('!') 
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("!");
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  bool condition;
  GET_BOOLEAN_ARG(ms, condition);

  if (condition) {
    ms->pushWord(std::make_shared<IntegerWord>(0));
  } else {
    ms->pushWord(std::make_shared<IntegerWord>(1));
  }
}
END_WORD
REGISTER_WORD(Not)



WORD(Ift)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> then;
  bool condition;
  
  GET_WORD_ARG(ms, Word, then);

  GET_BOOLEAN_ARG(ms, condition);

  if (condition) {
    ms->pushWord(then);
  }

}
END_WORD
REGISTER_WORD(Ift)


WORD(Ifte)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> then;
  std::shared_ptr<Word> else_;
  bool condition;
  
  GET_WORD_ARG(ms, Word, else_);
  GET_WORD_ARG(ms, Word, then);

  GET_BOOLEAN_ARG(ms, condition);

  if (condition) {
    ms->pushWord(then);
  } else {
    ms->pushWord(else_);
  }

}
END_WORD
REGISTER_WORD(Ifte)


WORD(Start)
virtual void execute(std::shared_ptr<MachineState> ms) {
}
END_WORD
REGISTER_WORD(Start)

WORD(Next)
virtual void execute(std::shared_ptr<MachineState> ms) {
  vector <std::shared_ptr<Word> > words_in_loop;
  
  while (true) {
    std::shared_ptr<Word> word = ms->popWord();
    if (word == NULL) {
      cout << "Warning, next could not find start, aborting." << endl;
      return;
    }
    if (word->name() == "start") {
      break;
    } 
    words_in_loop.push_back(word);
  }
  
  int to_i = 0;
  int from_i = 0;
  GET_ARG(ms, IntegerWord, to_i);
  GET_ARG(ms, IntegerWord, from_i);

  cout << "looping: " << from_i << " to " << to_i << endl;

  for (int i = from_i; i < to_i; i++) {
    for (int j = 0; j < words_in_loop.size(); j++) {
      ms->pushWord(words_in_loop[j]);
    }
  }
}
END_WORD
REGISTER_WORD(Next)

WORD(Print)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> word = ms->popData();
  if (word != NULL) {
    cout << word->repr() << endl;
  }
}
END_WORD
REGISTER_WORD(Print)

WORD(Dup)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> word = ms->popData();
  if (word == NULL) {
    cout << "Must take an argument." << endl;
  } else {
    ms->pushData(word);
    ms->pushData(word);
  }
}
END_WORD
REGISTER_WORD(Dup)

WORD(Pop)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> word = ms->popData();
}
END_WORD
REGISTER_WORD(Pop)


WORD(Swap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> word1;
  std::shared_ptr<Word> word2;
  GET_WORD_ARG(ms, Word, word1);
  GET_WORD_ARG(ms, Word, word2);
  ms->pushData(word1);
  ms->pushData(word2);

}
END_WORD
REGISTER_WORD(Swap)

WORD(Slide)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> word = ms->popWord();
  if (word == NULL) {
    cout << "Slide Must take an argument from the call stack." << endl;
  } else {
    ms->pushData(word);
  }
}
END_WORD
REGISTER_WORD(Slide)

WORD(Slip)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> word = ms->popData();
  if (word == NULL) {
    cout << "Slide Must take an argument from the call stack." << endl;
  } else {
    ms->pushWord(word);
  }
}
END_WORD
REGISTER_WORD(Slip)




WORD(SetEEPosePX)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double value;
  GET_NUMERIC_ARG(ms, value);

  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  
  eePose newPose = word->value();
  newPose.px = value;

  shared_ptr<EePoseWord> newWord = make_shared<EePoseWord>(newPose);

  ms->pushData(newWord);
}
END_WORD
REGISTER_WORD(SetEEPosePX)

WORD(SetEEPosePY)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double value;
  GET_NUMERIC_ARG(ms, value);

  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  
  eePose newPose = word->value();
  newPose.py = value;

  shared_ptr<EePoseWord> newWord = make_shared<EePoseWord>(newPose);

  ms->pushData(newWord);
}
END_WORD
REGISTER_WORD(SetEEPosePY)


WORD(SetEEPosePZ)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double value;
  GET_NUMERIC_ARG(ms, value);

  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  
  eePose newPose = word->value();
  newPose.pz = value;

  shared_ptr<EePoseWord> newWord = make_shared<EePoseWord>(newPose);

  ms->pushData(newWord);
}
END_WORD
REGISTER_WORD(SetEEPosePZ)



WORD(SetEEPoseQX)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double value;
  GET_NUMERIC_ARG(ms, value);

  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  
  eePose newPose = word->value();
  newPose.qx = value;

  shared_ptr<EePoseWord> newWord = make_shared<EePoseWord>(newPose);

  ms->pushData(newWord);
}
END_WORD
REGISTER_WORD(SetEEPoseQX)

WORD(SetEEPoseQY)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double value;
  GET_NUMERIC_ARG(ms, value);

  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  
  eePose newPose = word->value();
  newPose.qy = value;

  shared_ptr<EePoseWord> newWord = make_shared<EePoseWord>(newPose);

  ms->pushData(newWord);
}
END_WORD
REGISTER_WORD(SetEEPoseQY)

WORD(SetEEPoseQZ)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double value;
  GET_NUMERIC_ARG(ms, value);

  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  
  eePose newPose = word->value();
  newPose.qz = value;

  shared_ptr<EePoseWord> newWord = make_shared<EePoseWord>(newPose);

  ms->pushData(newWord);
}
END_WORD
REGISTER_WORD(SetEEPoseQZ)

WORD(SetEEPoseQW)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double value;
  GET_NUMERIC_ARG(ms, value);

  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  
  eePose newPose = word->value();
  newPose.qw = value;

  shared_ptr<EePoseWord> newWord = make_shared<EePoseWord>(newPose);

  ms->pushData(newWord);
}
END_WORD
REGISTER_WORD(SetEEPoseQW)



WORD(EePosePX)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  shared_ptr<DoubleWord> vword = make_shared<DoubleWord>(word->value().px);
  ms->pushData(vword);
}
END_WORD
REGISTER_WORD(EePosePX)


WORD(EePosePY)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  shared_ptr<DoubleWord> vword = make_shared<DoubleWord>(word->value().py);
  ms->pushData(vword);
}
END_WORD
REGISTER_WORD(EePosePY)


WORD(EePosePZ)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  shared_ptr<DoubleWord> vword = make_shared<DoubleWord>(word->value().pz);
  ms->pushData(vword);
}
END_WORD
REGISTER_WORD(EePosePZ)


WORD(EePoseQX)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  shared_ptr<DoubleWord> vword = make_shared<DoubleWord>(word->value().qx);
  ms->pushData(vword);
}
END_WORD
REGISTER_WORD(EePoseQX)


WORD(EePoseQY)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  shared_ptr<DoubleWord> vword = make_shared<DoubleWord>(word->value().qy);
  ms->pushData(vword);
}
END_WORD
REGISTER_WORD(EePoseQY)


WORD(EePoseQZ)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  shared_ptr<DoubleWord> vword = make_shared<DoubleWord>(word->value().qz);
  ms->pushData(vword);
}
END_WORD
REGISTER_WORD(EePoseQZ)


WORD(EePoseQW)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  shared_ptr<DoubleWord> vword = make_shared<DoubleWord>(word->value().qw);
  ms->pushData(vword);
}
END_WORD
REGISTER_WORD(EePoseQW)




WORD(Store)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> nameword = ms->popData();
  std::shared_ptr<Word> valueword = ms->popData();
  if (nameword == NULL || valueword == NULL) {
    cout << " Store takes two arguments." << endl;
  }

  string name = nameword->to_string();
  //cout << "Storing " << name << " value " << valueword << endl;
  ms->variables[name] = valueword;
}
END_WORD
REGISTER_WORD(Store)

WORD(Expand)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> nameword = ms->popWord();
  string name = nameword->to_string();
  shared_ptr<Word> value = ms->variables[name];
  cout << "Expanding " << nameword << " " << name << " value " << value << endl;
  if (value != NULL) {
    cout << " value: " << value->to_string() << endl;
    std::shared_ptr<CompoundWord> hTypeWord = std::dynamic_pointer_cast<CompoundWord>(value);
    if (hTypeWord == NULL) {
      ms->pushWord(value);
    } else {
      hTypeWord->execute(ms);
    }
  } else {
    cout << "No value for variable" << endl;
  }
}
END_WORD
REGISTER_WORD(Expand)


WORD(Import)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string filename;
  GET_ARG(ms, StringWord, filename);
  std::stringstream fname;
  fname << "src/ein/back/" << filename << ".back";
  cout << "fname: " << fname.str() << endl;
  std::ifstream t(fname.str());
  if (!t.is_open()) {
    cout << "Ooops, import tried to read " << fname.str() << " but it couldn't open..." << endl;
    ms->pushWord("pauseStackExecution");   
    return;
  }
  std::stringstream buffer;
  buffer << t.rdbuf();
  ms->evaluateProgram(buffer.str());
 }
END_WORD
REGISTER_WORD(Import)



WORD(Fetch)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> nameword = ms->popWord();
  string name = nameword->to_string();
  shared_ptr<Word> value = ms->variables[name];
  cout << "Fetching " << nameword << " " << name << " value " << value << endl;
  if (value != NULL) {
    cout << " value: " << value->to_string() << endl;
    ms->pushWord(value);
  } else {
    cout << "No value for variable" << endl;
  }
}
END_WORD
REGISTER_WORD(Fetch)





WORD(IncrementTargetClass)
CODE(196437)// capslock + pageup
virtual void execute(std::shared_ptr<MachineState> ms)
{
  cout << "targetClass++ " << endl;
  if (ms->config.numClasses > 0) {
    int newTargetClass = (ms->config.targetClass + 1) % ms->config.numClasses;
    changeTargetClass(ms, newTargetClass);
  }
}
END_WORD
REGISTER_WORD(IncrementTargetClass)

WORD(ChangeTargetClassToClosestBlueBox)
virtual void execute(std::shared_ptr<MachineState> ms)  {
  if (ms->config.pilotClosestBlueBoxNumber == -1) {
    cout << "Not changing because closest bbox is " << ms->config.pilotClosestBlueBoxNumber << endl;
    return;
  }
  int class_idx = ms->config.bLabels[ms->config.pilotClosestBlueBoxNumber];
  cout << "Changing to closest blue blox target, which is class " << ms->config.classLabels[class_idx] << endl;
  changeTargetClass(ms, class_idx);
}
END_WORD
REGISTER_WORD(ChangeTargetClassToClosestBlueBox)

WORD(Noop)
CODE('C')
virtual void execute(std::shared_ptr<MachineState> ms)
{

}
END_WORD
REGISTER_WORD(Noop)

WORD(EndStackCollapseNoop)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(EndStackCollapseNoop)

WORD(ExportWords)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  string wordFileName = "ein_words.txt";
  cout << "Writing words to " << wordFileName << endl;
  ofstream wordFile;
  wordFile.open(wordFileName);
  for (int i = 0; i < words.size(); i++) {
    wordFile << words[i]->name() << " " << words[i]->character_code() << endl;
  }
  wordFile.close();
}
END_WORD
REGISTER_WORD(ExportWords)


WORD(PixelGlobalTest)
CODE(65609) // I
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.paintEEandReg1OnWrist = !ms->config.paintEEandReg1OnWrist;
}
END_WORD
REGISTER_WORD(PixelGlobalTest)

WORD(IncMx)
CODE(65361) // left arrow 
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.m_x += .01;
  ms->config.m_x_h[ms->config.currentThompsonHeightIdx] = ms->config.m_x;
  cout << "m_x, m_x_h: " << ms->config.m_x << endl;
}
END_WORD
REGISTER_WORD(IncMx)

WORD(DecMx)
CODE(65363) // right arrow 
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.m_x -= .01;
  ms->config.m_x_h[ms->config.currentThompsonHeightIdx] = ms->config.m_x;
  cout << "m_x, m_x_h: " << ms->config.m_x << endl;
}
END_WORD
REGISTER_WORD(DecMx)

WORD(IncMy)
CODE(65362) // up arrow 
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.m_y += .01;
  ms->config.m_y_h[ms->config.currentThompsonHeightIdx] = ms->config.m_y;
  cout << "m_y, m_y_h: " << ms->config.m_y << endl;
}
END_WORD
REGISTER_WORD(IncMy)

WORD(DecMy)
CODE(65364) // down arrow 
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.m_y -= .01;
  ms->config.m_y_h[ms->config.currentThompsonHeightIdx] = ms->config.m_y;
  cout << "m_y, m_y_h: " << ms->config.m_y << endl;
}
END_WORD
REGISTER_WORD(DecMy)

WORD(EndStackCollapse)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.endCollapse = 1;
}
END_WORD
REGISTER_WORD(EndStackCollapse)

WORD(CollapseStack)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.endCollapse = 0;
}
END_WORD
REGISTER_WORD(CollapseStack)

WORD(ShakeHeadPositive)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.currentHeadPanCommand.target = 3.1415926/2.0;
  ms->config.currentHeadPanCommand.speed = 50;
  ms->config.headPub.publish(ms->config.currentHeadPanCommand);
}
END_WORD
REGISTER_WORD(ShakeHeadPositive)


WORD(ShakeHeadNegative)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.currentHeadPanCommand.target = -3.1415926/2.0;
  ms->config.currentHeadPanCommand.speed = 50;
  ms->config.headPub.publish(ms->config.currentHeadPanCommand);
}
END_WORD
REGISTER_WORD(ShakeHeadNegative)

WORD(CenterHead)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.currentHeadPanCommand.target = 0;
  ms->config.currentHeadPanCommand.speed = 50;
  ms->config.headPub.publish(ms->config.currentHeadPanCommand);
}
END_WORD
REGISTER_WORD(CenterHead)

WORD(SetHeadPanTargetSpeed)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  double t_target;
  double t_speed;
  GET_NUMERIC_ARG(ms, t_speed);
  GET_NUMERIC_ARG(ms, t_target);

  cout << "setHeadPanTargetSpeed: " << t_target << " " << t_speed << endl;

  ms->config.currentHeadPanCommand.target = t_target;
  ms->config.currentHeadPanCommand.speed = floor(t_speed);
  ms->config.headPub.publish(ms->config.currentHeadPanCommand);
}
END_WORD
REGISTER_WORD(SetHeadPanTargetSpeed)

WORD(SilenceSonar)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.currentSonarCommand.data = 0;
  ms->config.sonarPub.publish(ms->config.currentSonarCommand);
}
END_WORD
REGISTER_WORD(SilenceSonar)

WORD(UnSilenceSonar)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.currentSonarCommand.data = 1;
  ms->config.sonarPub.publish(ms->config.currentSonarCommand);
}
END_WORD
REGISTER_WORD(UnSilenceSonar)

WORD(Nod)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.currentHeadNodCommand.data = 1;
  ms->config.nodPub.publish(ms->config.currentHeadNodCommand);
}
END_WORD
REGISTER_WORD(Nod)

WORD(ResetAuxiliary)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->pushWord("nod");
  ms->pushWord("centerHead");
  ms->pushWord("shakeHeadPositive");
  ms->pushCopies("noop", 20);
  ms->pushWord("nod");
  ms->pushWord("shakeHeadNegative");
  ms->pushWord("shakeHeadNegative");
  ms->pushCopies("noop", 20);
  ms->pushWord("nod");
  ms->pushWord("shakeHeadPositive");
  ms->pushWord("silenceSonar");
  ms->pushWord("centerHead");
}
END_WORD
REGISTER_WORD(ResetAuxiliary)

WORD(WaitUntilImageCallbackReceived)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.lastImageCallbackRequest = ros::Time::now();
  ms->pushWord("waitUntilImageCallbackReceivedA");
  ms->config.shouldIImageCallback = 1;
  ms->config.endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(WaitUntilImageCallbackReceived)

WORD(WaitUntilImageCallbackReceivedA)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  if (ms->config.lastImageCallbackRequest >= ms->config.lastImageCallbackReceived) {
    ms->pushWord("waitUntilImageCallbackReceivedA");
    ms->config.shouldIImageCallback = 1;
    ms->config.endThisStackCollapse = 1;
  } else {
    ms->config.endThisStackCollapse = ms->config.endCollapse;
  }
}
END_WORD
REGISTER_WORD(WaitUntilImageCallbackReceivedA)

WORD(WaitUntilAccelerometerCallbackReceived)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.lastAccelerometerCallbackRequest = ros::Time::now();
  ms->pushWord("waitUntilAccelerometerCallbackReceivedA");
  ms->config.endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(WaitUntilAccelerometerCallbackReceived)

WORD(WaitUntilAccelerometerCallbackReceivedA)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  if (ms->config.lastAccelerometerCallbackRequest >= ms->config.lastAccelerometerCallbackReceived) {
    ms->pushWord("waitUntilAccelerometerCallbackReceivedA");
    ms->config.endThisStackCollapse = 1;
  } else {
    ms->config.endThisStackCollapse = ms->config.endCollapse;
  }
}
END_WORD
REGISTER_WORD(WaitUntilAccelerometerCallbackReceivedA)

WORD(WaitUntilEndpointCallbackReceived)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.lastEndpointCallbackRequest = ros::Time::now();
  ms->pushWord("waitUntilEndpointCallbackReceivedA");
  ms->config.endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(WaitUntilEndpointCallbackReceived)

WORD(WaitUntilEndpointCallbackReceivedA)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  if (ms->config.lastEndpointCallbackRequest >= ms->config.lastEndpointCallbackReceived) {
    ms->pushWord("waitUntilEndpointCallbackReceivedA");
    ms->config.endThisStackCollapse = 1;
  } else {
    ms->config.endThisStackCollapse = ms->config.endCollapse;
  }
}
END_WORD
REGISTER_WORD(WaitUntilEndpointCallbackReceivedA)

WORD(WriteXMLEnvironment)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  // For Dipendra
  ofstream ofile;
  string fileName = ms->config.data_directory + "/" + ms->config.left_or_right_arm + "_environment.xml";
  cout << "Saving environment to " << fileName << endl;
  ofile.open(fileName, ios::trunc);

  ofile << "<environment>" << endl;
  for (int i = 0; i < ms->config.blueBoxMemories.size(); i++) {
    BoxMemory box = ms->config.blueBoxMemories[i];
    if (box.labeledClassIndex >= 0) {
      ofile << "  <object>" << endl;
      ofile << "    <name>" << ms->config.classLabels[box.labeledClassIndex] << "</name>" << endl;
      ofile << "    <position>" << "( " << box.centroid.px << ", " << box.centroid.py << ", " << box.centroid.pz << ")" << "</position>" << endl;
      ofile << "    <rotation>" << "(0, 0, 0)" << "</rotation>" << endl;
      ofile << "  </object>" << endl;
    }
  }
  ofile << "</environment>" << endl;

  ofile.close();
}
END_WORD
REGISTER_WORD(WriteXMLEnvironment)

WORD(DisableRobot)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  int sis = system("bash -c \"echo -e \'C\003\' | rosrun baxter_tools enable_robot.py -d\"");
}
END_WORD
REGISTER_WORD(DisableRobot)

WORD(EnableRobot)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  int sis = system("bash -c \"echo -e \'C\003\' | rosrun baxter_tools enable_robot.py -e\"");
}
END_WORD
REGISTER_WORD(EnableRobot)

WORD(ReplicateWord)
virtual void execute(std::shared_ptr<MachineState> ms)
{

  double v1;
  GET_NUMERIC_ARG(ms, v1);

  shared_ptr<Word> aWord;
  GET_WORD_ARG(ms, Word, aWord);

  int rTimes = (int) v1;
  std::shared_ptr<Word> repWord = aWord;
  ms->pushCopies(repWord, rTimes);

}
END_WORD
REGISTER_WORD(ReplicateWord)


WORD(Help)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  
  shared_ptr<Word> word;
  GET_WORD_ARG(ms, Word, word);
  
  shared_ptr<StringWord> outword = std::make_shared<StringWord>(word->description());
  ms->pushWord(outword);
}
END_WORD
REGISTER_WORD(Help)


WORD(IkModeService)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.currentIKMode = IKSERVICE;
}
END_WORD
REGISTER_WORD(IkModeService)


WORD(IkModeIkFast)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.currentIKMode = IKFAST;
}
END_WORD
REGISTER_WORD(IkModeIkFast)


WORD(IkModeIkFastDebug)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.currentIKMode = IKFASTDEBUG;
}
END_WORD
REGISTER_WORD(IkModeIkFastDebug)

WORD(ResetAveragedWrenchNorm)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.averagedWrechAcc = 0;
  ms->config.averagedWrechMass = 0;
}
END_WORD
REGISTER_WORD(ResetAveragedWrenchNorm)

WORD(DigitalIOCommand)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  string component;
  int value;
  GET_ARG(ms, StringWord, component);
  GET_ARG(ms, IntegerWord, value);

  baxter_core_msgs::DigitalOutputCommand thisCommand;

  thisCommand.name = component;
  thisCommand.value = value;

  ms->config.digital_io_pub.publish(thisCommand);
}
END_WORD
REGISTER_WORD(DigitalIOCommand)

WORD(SetRedHalo)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  double value;
  GET_NUMERIC_ARG(ms, value);
  ms->config.red_halo_state = value;

  {
    std_msgs::Float32 thisCommand;
    thisCommand.data = ms->config.red_halo_state;
    ms->config.red_halo_pub.publish(thisCommand);
  }
}
END_WORD
REGISTER_WORD(SetRedHalo)

WORD(SetGreenHalo)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  double value;
  GET_NUMERIC_ARG(ms, value);
  ms->config.green_halo_state = value;

  {
    std_msgs::Float32 thisCommand;
    thisCommand.data = ms->config.green_halo_state;
    ms->config.green_halo_pub.publish(thisCommand);
  }
}
END_WORD
REGISTER_WORD(SetGreenHalo)

WORD(SetSonarLed)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  int value;
  GET_ARG(ms, IntegerWord, value);
  ms->config.sonar_led_state = value;

  {
    std_msgs::UInt16 thisCommand;
    thisCommand.data = ms->config.sonar_led_state;
    ms->config.sonar_pub.publish(thisCommand);
  }
}
END_WORD
REGISTER_WORD(SetSonarLed)

WORD(LightsOn)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  std::stringstream program;
  program << "100 setGreenHalo 100 setRedHalo 4095 setSonarLed ";
  program << "1 \"left_itb_light_inner\" digitalIOCommand 1 \"right_itb_light_inner\" digitalIOCommand 1 \"torso_left_itb_light_inner\" digitalIOCommand 1 \"torso_right_itb_light_inner\" digitalIOCommand 1 \"left_itb_light_outer\" digitalIOCommand 1 \"right_itb_light_outer\" digitalIOCommand 1 \"torso_left_itb_light_outer\" digitalIOCommand 1 \"torso_right_itb_light_outer\" digitalIOCommand";
  ms->evaluateProgram(program.str());
}
END_WORD
REGISTER_WORD(LightsOn)

WORD(LightsOff)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  std::stringstream program;
  program << "0 setGreenHalo 0 setRedHalo 32768 setSonarLed ";
  program << "0 \"left_itb_light_inner\" digitalIOCommand 0 \"right_itb_light_inner\" digitalIOCommand 0 \"torso_left_itb_light_inner\" digitalIOCommand 0 \"torso_right_itb_light_inner\" digitalIOCommand 0 \"left_itb_light_outer\" digitalIOCommand 0 \"right_itb_light_outer\" digitalIOCommand 0 \"torso_left_itb_light_outer\" digitalIOCommand 0 \"torso_right_itb_light_outer\" digitalIOCommand";
  ms->evaluateProgram(program.str());
}
END_WORD
REGISTER_WORD(LightsOff)

WORD(SwitchSonarLed)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  int value;
  int led;
  GET_ARG(ms, IntegerWord, led);
  GET_ARG(ms, IntegerWord, value);

  int blanked_sls = (~(1<<led)) & ms->config.sonar_led_state;

  ms->config.sonar_led_state = blanked_sls | (1<<15) | (value * (1<<led));

  {
    std_msgs::UInt16 thisCommand;
    thisCommand.data = ms->config.sonar_led_state;
    ms->config.sonar_pub.publish(thisCommand);
  }
}
END_WORD
REGISTER_WORD(SwitchSonarLed)

WORD(PrintStacks)
virtual void execute(std::shared_ptr<MachineState> ms)
{

  cout << endl << "~~~~~~~~~ printing stacks ~~~~~~~~~" << endl << endl ;

  for (int i = 0; i < ms->data_stack.size(); i++) {
    cout << "  " << ms->data_stack[i]->name() << "  " << i << endl;
  }
  cout << endl << " --- " << endl << endl;
  for (int i = ms->call_stack.size()-1; i >= 0; i--) {
    cout << "  " << ms->call_stack[i]->name() << "  " << i << endl;
  }

  cout << endl << "~~~~~~~~~ ~~~~~~~~~~~~~~~ ~~~~~~~~~" << endl << endl ;

}
END_WORD
REGISTER_WORD(PrintStacks)

WORD(ClearData)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->clearData();
}
END_WORD
REGISTER_WORD(ClearData)

WORD(CP)
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back(")");
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ROS_ERROR_STREAM("Close parenthesis should never execute." << endl);
  ms->pushWord("pauseStackExecution");
}
END_WORD
REGISTER_WORD(CP)

WORD(OP)
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("(");
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms)
{

  //cout << "Open parenthesis executing." << endl;
  ms->pushWord("sP");
  ms->pushWord("1");

  ms->pushData("oP");

  //ms->pushWord("printStacks");

}
END_WORD
REGISTER_WORD(OP)

WORD(SP)
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("|S");
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms)
{
  int scopeLevel = 0;
  GET_ARG(ms, IntegerWord, scopeLevel);

  if (scopeLevel < 0) {
    cout << "sP scope error." << endl;
    ms->pushWord("pauseStackExecution");
    return;
  } else if (scopeLevel == 0) {
    return;
  } else {
    // continue
  }

  std::shared_ptr<Word> word = ms->popWord();

  if (word == NULL) {
    cout << "sP found no word... pausing stack execution." << endl;
    ms->pushWord("pauseStackExecution");
    return;
  } else {
    if (word->is_value()) {
      // data goes to the data stack
      ms->pushData(word);

      ms->pushWord("sP");
      ms->pushWord(std::make_shared<IntegerWord>(scopeLevel));
    } else {
      // handle commands specially
      if(0 == word->name().compare("cP")) {
	if (scopeLevel != 1) {
	  ms->pushData(word);

	  //cout << "sP found cP!!! scopeLevel " << scopeLevel << " " << endl;
	  ms->pushWord("sP");
	  ms->pushWord(std::make_shared<IntegerWord>(scopeLevel-1));
	} else {
	  // unwind until matching oP
	  int open_needed = 1;
	  shared_ptr<CompoundWord> cp = make_shared<CompoundWord>();

	  while (open_needed > 0) {
	    std::shared_ptr<Word> datum = ms->popData();
	    if (datum == NULL) {
	      cout << "sP found no datum... pausing stack execution." << endl;
	      ms->pushWord("pauseStackExecution");
	      return;
	    } else {
	      if (datum->is_value()) {
		// data goes to the data stack
		cp->pushWord(datum);
	      } else {
		if(0 == datum->name().compare("oP")) {
		  open_needed = open_needed-1;
		  //cout << " open needed -1: " << open_needed << endl;
		  if (open_needed > 0) {
		    cp->pushWord(datum);
		  } else {
		  }
		} else if(0 == datum->name().compare("cP")) {
		  open_needed = open_needed+1;
		  cp->pushWord(datum);
		  //cout << " open needed +1: " << open_needed << endl;
		} else {
		  cp->pushWord(datum);
		}
	      }
	    }
	  }
	  ms->pushData(cp);

	}
      } else if (0 == word->name().compare("oP")) {
	ms->pushData(word);

	ms->pushWord("sP");
	ms->pushWord(std::make_shared<IntegerWord>(scopeLevel+1));
      } else {
	// other things go on the data stack
	ms->pushData(word);

	ms->pushWord("sP");
	ms->pushWord(std::make_shared<IntegerWord>(scopeLevel));
      }
    }
  }
  //ms->pushWord("printStacks");
}
END_WORD
REGISTER_WORD(SP)


WORD(ExecutionModeInstant)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->execution_mode = INSTANT;
}
END_WORD
REGISTER_WORD(ExecutionModeInstant)

WORD(ExecutionModeStep)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->execution_mode = STEP;
}
END_WORD
REGISTER_WORD(ExecutionModeStep)

WORD(Exec)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<Word> aWord;
  GET_WORD_ARG(ms, Word, aWord);
  ms->pushWord(aWord);
}
END_WORD
REGISTER_WORD(Exec)

WORD(CastToInteger)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double number = 0.0;
  GET_NUMERIC_ARG(ms, number);
  ms->pushData(std::make_shared<IntegerWord>(number));
}
END_WORD
REGISTER_WORD(CastToInteger)

WORD(Assert)
virtual void execute(std::shared_ptr<MachineState> ms) {
  bool value;
  GET_BOOLEAN_ARG(ms, value);
  if (!value) {
    ROS_ERROR_STREAM("Failed assert. Pausing." << endl);
    ms->pushWord("pauseStackExecution");
  }
}
END_WORD
REGISTER_WORD(Assert)


WORD(AssertNo)
virtual void execute(std::shared_ptr<MachineState> ms) {
  bool value;
  GET_BOOLEAN_ARG(ms, value);
  if (value) {
    ROS_ERROR_STREAM("Failed assertNo. Pausing." << endl);
    ms->pushWord("pauseStackExecution");
  }
}
END_WORD
REGISTER_WORD(AssertNo)


WORD(While)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<CompoundWord> block;
  shared_ptr<CompoundWord> condition;
  GET_WORD_ARG(ms, CompoundWord, block);
  GET_WORD_ARG(ms, CompoundWord, condition);

  shared_ptr<CompoundWord> whileblock = make_shared<CompoundWord>();

  whileblock->pushWord(ms, "while");
  whileblock->pushWord(ms, ")");
  for (int i = 0; i < block->size(); i++) {
    whileblock->pushWord(block->getWord(i));
  }
  whileblock->pushWord(ms, "(");
  whileblock->pushWord(ms, ")");
  for (int i = 0; i < condition->size(); i++) {
    whileblock->pushWord(condition->getWord(i));
  }
  whileblock->pushWord(ms, "(");

  for (int i = 0; i < block->size(); i++) {
    whileblock->pushWord(block->getWord(i));
  }

  ms->pushWord("ift");
  ms->pushWord(")");
  for (int i = 0; i < whileblock->size(); i++) {
    ms->pushWord(whileblock->getWord(i));
  }
  ms->pushWord("(");
  ms->pushWord(condition);
}
END_WORD
REGISTER_WORD(While)


WORD(LeftOrRightArm)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<StringWord> left_or_right = make_shared<StringWord>(ms->config.left_or_right_arm);
  ms->pushWord(left_or_right);
}
END_WORD
REGISTER_WORD(LeftOrRightArm)

WORD(IsGripperGripping)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<IntegerWord> isGripping = make_shared<IntegerWord>(isGripperGripping(ms));
  ms->pushWord(isGripping);
}
END_WORD
REGISTER_WORD(IsGripperGripping)





/*

&|3|&= $  $| @ !! !|
9|4| * * * | * * * |

*/

WORD(NumBlueBoxes)
virtual void execute(std::shared_ptr<MachineState> ms) {
  shared_ptr<IntegerWord> numBlueBoxes = make_shared<IntegerWord>(ms->config.bTops.size());
  ms->pushWord(numBlueBoxes);  
}
END_WORD
REGISTER_WORD(NumBlueBoxes)


WORD(DateString)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ros::Time thisNow = ros::Time::now();
  ms->pushWord(make_shared<StringWord>(formatTime(thisNow)));
}
END_WORD
REGISTER_WORD(DateString)


WORD(OB)
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("{");
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms) {

  ms->pushData("oP");
  ms->pushData("oB");

  /* 
    XXX do this all in [, discard sB?
     push ( onto data stack
     
     slide all data to the data stack, this gives each [] its own stack
      >>> consider calling {} instead, scope
     execute first non-data word on stack 
     slide and parse to the paired ], consume
     push cB 1 sP cP to data to rewind 
  */


  int cb_needed = 1;
  bool can_execute = false;
  while (cb_needed > 0) {

    std::shared_ptr<Word> word = ms->popWord();

    if (word == NULL) {
      cout << "oB found no word... pausing stack execution." << endl;
      ms->pushWord("pauseStackExecution");
      return;
    } else {

      if ( 0 == word->name().compare("cB") ) {
	//cout << " oB: pushing cB " << word->name() << endl;
	ms->pushData(word);
	cb_needed = cb_needed - 1;
      } else if ( 0 == word->name().compare("oB") ) {
	if (can_execute) {
	  //cout << " oB: executing oB " << word->name() << endl;
	  ms->execute(word);
	  ms->pushData("sB");
	  can_execute = false;
	  // this one will account for itself
	} else {
	  //cout << " oB: pushing oB " << word->name() << endl;
	  ms->pushData(word);
	  // this one will NOT account for itself
	  cb_needed = cb_needed + 1;
	}
      } else if ( 0 == word->name().compare("sB") ) {
	if (cb_needed <= 1) {
	  can_execute = true;
	} else {
	  ms->pushData(word);
	}
      } else if (can_execute) {
	//cout << " oB: executing " << word->name() << endl;
	ms->execute(word);
	ms->pushData("sB");
	can_execute = false;
      } else {
	//cout << " oB: cannot execute so pushing " << word->name() << endl;
	ms->pushData(word);
      }
    }
  }

  // if you hit the bottom and can still execute, put sB back at the top
  if (can_execute) {
    //cout << "    ob: hit bottom, reinserting sB" << endl;

    // take the one you just pushed, make sure it is cB
    std::shared_ptr<Word> word = ms->popData();
    if (word == NULL) {
      cout << "oB found no word during sB reinsert a... pausing stack execution." << endl;
      ms->pushWord("pauseStackExecution");
      return;
    } else if ( 0 == word->name().compare("cB") ) {
      // good, should always happen
    } else {
      cout << "oB found no cB during sB reinsert a... pausing stack execution." << endl;
      ms->pushWord("pauseStackExecution");
      return;
    }
    // put it on the stack
    ms->pushWord(word);

    // slip until you find the top and put in an sB
    int ob_needed = 1;
    while (ob_needed > 0) {
      std::shared_ptr<Word> word = ms->popData();

      if (word == NULL) {
	cout << "oB found no word during sB reinsert b... pausing stack execution." << endl;
	ms->pushWord("pauseStackExecution");
	return;
      } else if ( 0 == word->name().compare("cB") ) {
	ob_needed = ob_needed + 1;
	ms->pushWord(word);
      } else if ( 0 == word->name().compare("oB") ) {
	ob_needed = ob_needed - 1;
	if (ob_needed <= 0) {
	  ms->pushData(word);
	  ms->pushData("sB");
	  break;
	} else {
	  ms->pushWord(word);
	}
      } else {
	ms->pushWord(word);
      }
    }

    // slide until you are back down where you were
    int cb_needed = 1;
    while (cb_needed > 0) {
      std::shared_ptr<Word> word = ms->popWord();

      if (word == NULL) {
	cout << "oB found no word during sB reinsert c... pausing stack execution." << endl;
	ms->pushWord("pauseStackExecution");
	return;
      } else {
	if ( 0 == word->name().compare("cB") ) {
	  ms->pushData(word);
	  cb_needed = cb_needed - 1;
	} else if ( 0 == word->name().compare("oB") ) {
	  ms->pushData(word);
	  cb_needed = cb_needed + 1;
	} else {
	  ms->pushData(word);
	}
      }
    }
  } else {
  }

  ms->pushWord("cP");

  // this needs to happen immediately so that cBs from below don't affect above
  ms->pushData(std::make_shared<IntegerWord>(1));
  ms->pushWord("sP");
  std::shared_ptr<Word> word = ms->popWord();
  if (word == NULL) {
    cout << "oB found no word on rewind... pausing stack execution." << endl;
    ms->pushWord("pauseStackExecution");
    return;
  } else {
    ms->execute(word);
  }
}
END_WORD
REGISTER_WORD(OB)

WORD(SB)
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("|B");
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  ROS_ERROR_STREAM("sB should never execute." << endl);
  ms->pushWord("pauseStackExecution");
}
END_WORD
REGISTER_WORD(SB)

WORD(CB)
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("}");
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  ROS_ERROR_STREAM("Close bracket should never execute." << endl);
  ms->pushWord("pauseStackExecution");
}
END_WORD
REGISTER_WORD(CB)

// data stack reserve
WORD(Dsr)
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  int numToReserve = 0;
  GET_INT_ARG(ms, numToReserve);
  ms->pushCopies(std::make_shared<IntegerWord>(0), max(int(numToReserve - ms->data_stack.size()), int(0)));
}
END_WORD
REGISTER_WORD(Dsr)



CONFIG_GETTER_INT(GradientServoSoftMaxIterations, ms->config.softMaxGradientServoIterations)
CONFIG_SETTER_INT(SetGradientServoSoftMaxIterations, ms->config.softMaxGradientServoIterations)

CONFIG_GETTER_INT(GradientServoHardMaxIterations, ms->config.hardMaxGradientServoIterations)
CONFIG_SETTER_INT(SetGradientServoHardMaxIterations, ms->config.hardMaxGradientServoIterations)

CONFIG_GETTER_INT(MappingServoTimeout, ms->config.mappingServoTimeout)
CONFIG_SETTER_INT(SetMappingServoTimeout, ms->config.mappingServoTimeout)

CONFIG_GETTER_INT(RepeatHalo, ms->config.repeat_halo)
CONFIG_SETTER_INT(SetRepeatHalo, ms->config.repeat_halo)

CONFIG_GETTER_DOUBLE(TwistThresh, ms->config.twistThresh)
CONFIG_SETTER_DOUBLE(SetTwistThresh, ms->config.twistThresh)

CONFIG_GETTER_DOUBLE(EffortThresh, ms->config.actual_effort_thresh);
CONFIG_SETTER_DOUBLE(SetEffortThresh, ms->config.actual_effort_thresh);


CONFIG_GETTER_STRING(RobotSerial, ms->config.robot_serial)

CONFIG_GETTER_STRING(ScanGroup, ms->config.scan_group)
CONFIG_SETTER_STRING(SetScanGroup, ms->config.scan_group)

CONFIG_GETTER_DOUBLE(IkMapStartHeight, ms->config.ikMapStartHeight)

CONFIG_GETTER_DOUBLE(IkMapEndHeight, ms->config.ikMapEndHeight)


CONFIG_GETTER_DOUBLE(MapSearchFenceXMin, ms->config.mapSearchFenceXMin)
CONFIG_SETTER_DOUBLE(SetMapSearchFenceXMin, ms->config.mapSearchFenceXMin)

CONFIG_GETTER_DOUBLE(MapSearchFenceYMin, ms->config.mapSearchFenceYMin)
CONFIG_SETTER_DOUBLE(SetMapSearchFenceYMin, ms->config.mapSearchFenceYMin)

CONFIG_GETTER_DOUBLE(MapSearchFenceXMax, ms->config.mapSearchFenceXMax)
CONFIG_SETTER_DOUBLE(SetMapSearchFenceXMax, ms->config.mapSearchFenceXMax)

CONFIG_GETTER_DOUBLE(MapSearchFenceYMax, ms->config.mapSearchFenceYMax)
CONFIG_SETTER_DOUBLE(SetMapSearchFenceYMax, ms->config.mapSearchFenceYMax)

CONFIG_GETTER_DOUBLE(GripperMaskThresh, ms->config.gripperMaskThresh)
CONFIG_SETTER_DOUBLE(SetGripperMaskThresh, ms->config.gripperMaskThresh)


CONFIG_GETTER_DOUBLE(CurrentTableZ, ms->config.currentTableZ)
CONFIG_SETTER_DOUBLE(SetCurrentTableZ, ms->config.currentTableZ)



CONFIG_GETTER_INT(ObservedCameraFlip, ms->config.observedCameraFlip)
CONFIG_GETTER_INT(ObservedCameraMirror, ms->config.observedCameraMirror)

CONFIG_GETTER_INT(ObservedCameraExposure, ms->config.observedCameraExposure)
CONFIG_GETTER_INT(ObservedCameraGain, ms->config.observedCameraGain)
CONFIG_GETTER_INT(ObservedCameraWhiteBalanceRed, ms->config.observedCameraWhiteBalanceRed)
CONFIG_GETTER_INT(ObservedCameraWhiteBalanceGreen, ms->config.observedCameraWhiteBalanceGreen)
CONFIG_GETTER_INT(ObservedCameraWhiteBalanceBlue, ms->config.observedCameraWhiteBalanceBlue)
CONFIG_GETTER_INT(ObservedCameraWindowX, ms->config.observedCameraWindowX)
CONFIG_GETTER_INT(ObservedCameraWindowY, ms->config.observedCameraWindowY)

CONFIG_GETTER_INT(CameraExposure, ms->config.cameraExposure)
CONFIG_GETTER_INT(CameraGain, ms->config.cameraGain)
CONFIG_GETTER_INT(CameraWhiteBalanceRed, ms->config.cameraWhiteBalanceRed)
CONFIG_GETTER_INT(CameraWhiteBalanceGreen, ms->config.cameraWhiteBalanceGreen)
CONFIG_GETTER_INT(CameraWhiteBalanceBlue, ms->config.cameraWhiteBalanceBlue)


CONFIG_GETTER_INT(SceneCellCountThreshold, ms->config.sceneCellCountThreshold)
CONFIG_SETTER_INT(SceneSetCellCountThreshold, ms->config.sceneCellCountThreshold)


CONFIG_GETTER_INT(SceneDiscrepancySearchDepth, ms->config.sceneDiscrepancySearchDepth)
CONFIG_SETTER_INT(SceneSetDiscrepancySearchDepth, ms->config.sceneDiscrepancySearchDepth)

//CONFIG_GETTER_INT(NumIkMapHeights, ms->config.numIkMapHeights)



}
