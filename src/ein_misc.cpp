
#include "ein_words.h"
#include "ein.h"
#include <boost/filesystem.hpp>
using namespace boost::filesystem;
#include "camera.h"

namespace ein_words {


WORD(Mkdir)
virtual void execute(MachineState * ms) {
  {
    string dirname;
    GET_STRING_ARG(ms, dirname);

    int result = mkdir(dirname.c_str(), 0777);
    if (result != 0) {
      stringstream buf;
      buf << "Could not create directory: " << dirname;
      CONSOLE_ERROR(ms, buf.str());
      ms->pushWord("pauseStackExecution"); 
    } else {
      cout << "Created '" << dirname << "'" << endl;
    }
  }
}
END_WORD
REGISTER_WORD(Mkdir)

WORD(Mkdirs)
virtual void execute(MachineState * ms) {
  {
    string dirname;
    GET_STRING_ARG(ms, dirname);

    try {
      boost::filesystem::create_directories(dirname);
    } catch(std::exception const&  ex) {
      CONSOLE_ERROR(ms, "Can't make directory: " << ex.what());
      ms->pushWord("pauseStackExecution"); 
    }
  }
}
END_WORD
REGISTER_WORD(Mkdirs)



WORD(PublishState)
virtual void execute(MachineState * ms) {
  {
    EinState state;
    fillEinStateMsg(ms, &state);
    ms->config.einStatePub.publish(state);
  }
}
END_WORD
REGISTER_WORD(PublishState)

WORD(Drand48)
virtual void execute(MachineState * ms) {
  ms->pushData(std::make_shared<DoubleWord>(drand48()));
}
END_WORD
REGISTER_WORD(Drand48)

WORD(Now)
virtual void execute(MachineState * ms) {
  ms->pushData(std::make_shared<DoubleWord>(ros::Time::now().toSec()));
}
END_WORD
REGISTER_WORD(Now)

WORD(Throw)
virtual void execute(MachineState * ms) {
  throw runtime_error("test");
  
}
END_WORD
REGISTER_WORD(Throw)

WORD(ThrowOpenCV)
virtual void execute(MachineState * ms) {
  Mat m;
  m.rowRange(100, 100);
}
END_WORD
REGISTER_WORD(ThrowOpenCV)

WORD(SeeHz)
virtual void execute(MachineState * ms) {

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
virtual void execute(MachineState * ms) {
  // marks the end of a list of arguments 
  ms->pushData("endArgs");
}
END_WORD
REGISTER_WORD(EndArgs)

WORD(UploadObjectToDatabase)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  ms->config.zero_g_toggle = !ms->config.zero_g_toggle;
}
END_WORD
REGISTER_WORD(ZeroGToggle)

WORD(ZeroGOn)
virtual void execute(MachineState * ms) {
  ms->config.zero_g_toggle = 1;
}
END_WORD
REGISTER_WORD(ZeroGOn)

WORD(ZeroGOff)
virtual void execute(MachineState * ms) {
  ms->config.zero_g_toggle = 0;
}
END_WORD
REGISTER_WORD(ZeroGOff)

WORD(ClearStack)
CODE('r') 
virtual void execute(MachineState * ms) {
  ms->clearStack();
}
END_WORD
REGISTER_WORD(ClearStack)

WORD(ClearStacks)
virtual void execute(MachineState * ms) {
  ms->clearData();
  ms->clearStack();
}
END_WORD
REGISTER_WORD(ClearStacks)

WORD(Beep)
CODE(1245308)     // capslock + numlock + |
virtual void execute(MachineState * ms) {
  cout << "\a"; cout.flush();
}
END_WORD
REGISTER_WORD(Beep)


WORD(ChangeToCounterTable)
CODE(1179735) // capslock + numlock + w
virtual void execute(MachineState * ms) {
  ms->config.currentTableZ = ms->config.counterTableZ;
}
END_WORD
REGISTER_WORD(ChangeToCounterTable)
  
WORD(ChangeToPantryTable)
CODE(1179717)    // capslock + numlock + e
virtual void execute(MachineState * ms) {
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

virtual void execute(MachineState * ms) {
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
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("Pause");
  return result;
}
CODE('Y') 
virtual void execute(MachineState * ms)  {
  CONSOLE(ms, "STACK EXECUTION PAUSED.  Press enter to continue.");
  ms->execute_stack = 0;
  ms->config.endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(PauseStackExecution)


 
WORD(PauseAndReset)
CODE('c') 
virtual void execute(MachineState * ms) {
  ms->execute_stack = 0;
  ms->config.lastPtheta = INFINITY;
}
END_WORD
REGISTER_WORD(PauseAndReset)


WORD(PushState)
virtual void execute(MachineState * ms) {
  string state = ms->currentState();
  shared_ptr<StringWord> outword = std::make_shared<StringWord>(state);
  ms->pushData(outword);
}
END_WORD
REGISTER_WORD(PushState)



WORD(PrintState)
CODE('u')
virtual void execute(MachineState * ms) {
  ms->pushWord("print");
  ms->pushWord("pushState");
}
END_WORD
REGISTER_WORD(PrintState)

WORD(DecrementTargetClass)
CODE(196438)     // capslock + pagedown
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  double v2;
  GET_NUMERIC_ARG(ms, v2);

  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(int(v1) && int(v2));
  ms->pushWord(newWord);

}
END_WORD
REGISTER_WORD(And)

WORD(Cos)
virtual void execute(MachineState * ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(cos(v1));
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(Cos)

WORD(Sin)
virtual void execute(MachineState * ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(sin(v1));
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(Sin)

WORD(Pi)
virtual void execute(MachineState * ms) {
  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(M_PI);
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(Pi)

WORD(Exp)
virtual void execute(MachineState * ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(exp(v1));
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(Exp)

WORD(Pow)
virtual void execute(MachineState * ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  double v2;
  GET_NUMERIC_ARG(ms, v2);

  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(pow(v2,v1));
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(Pow)

WORD(Max)
virtual void execute(MachineState * ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  double v2;
  GET_NUMERIC_ARG(ms, v2);

  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(std::max(v2,v1));
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(Max)

WORD(Min)
virtual void execute(MachineState * ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  double v2;
  GET_NUMERIC_ARG(ms, v2);

  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(std::min(v2,v1));
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(Min)

WORD(Floor)
virtual void execute(MachineState * ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);

  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(floor(v1));
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(Floor)

WORD(Ceil)
virtual void execute(MachineState * ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);

  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(ceil(v1));
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(Ceil)

WORD(Plus)
CODE('+') 
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("+");
  return result;
}
virtual void execute(MachineState * ms) {
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



WORD(Sum)
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("join");
  return result;
}
virtual void execute(MachineState * ms) {
  ms->evaluateProgram(" ( + ) accumulate");
}
END_WORD
REGISTER_WORD(Sum)

WORD(Prod)
virtual void execute(MachineState * ms) {
  ms->evaluateProgram(" ( * ) accumulate");
}
END_WORD
REGISTER_WORD(Prod)


WORD(Langle)
CODE('<') 
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("<");
  return result;
}
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  double v2;
  GET_NUMERIC_ARG(ms, v2);

  std::shared_ptr<IntegerWord> newWord = std::make_shared<IntegerWord>(v2 > v1);
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(Rangle)

WORD(Leq)
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("<=");
  return result;
}
virtual void execute(MachineState * ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  double v2;
  GET_NUMERIC_ARG(ms, v2);

  std::shared_ptr<IntegerWord> newWord = std::make_shared<IntegerWord>(v2 <= v1);
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(Leq)

WORD(Geq)
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back(">=");
  return result;
}
virtual void execute(MachineState * ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  double v2;
  GET_NUMERIC_ARG(ms, v2);

  std::shared_ptr<IntegerWord> newWord = std::make_shared<IntegerWord>(v2 >= v1);
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(Geq)

WORD(Times)
CODE('*') 
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("*");
  return result;
}
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {

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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
}
END_WORD
REGISTER_WORD(Start)

WORD(Next)
virtual void execute(MachineState * ms) {
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
virtual string description() {
  return "Pop a word from the stack and print it to the Ein console.";
}
virtual void execute(MachineState * ms) {
  std::shared_ptr<Word> word = ms->popData();
  std::shared_ptr<StringWord> s = std::dynamic_pointer_cast<StringWord>(word);
  if (s != NULL) {
    CONSOLE(ms, s->value());
  } else if (word != NULL) {
    CONSOLE(ms, word->repr());
  } else {
    CONSOLE(ms, "");
  }
}
END_WORD
REGISTER_WORD(Print)

WORD(Dup)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  std::shared_ptr<Word> word = ms->popData();
}
END_WORD
REGISTER_WORD(Pop)


WORD(Swap)
virtual void execute(MachineState * ms) {
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
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("'");
  return result;
}
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  std::shared_ptr<Word> word = ms->popData();
  if (word == NULL) {
    cout << "Slide Must take an argument from the data stack." << endl;
  } else {
    ms->pushWord(word);
  }
}
END_WORD
REGISTER_WORD(Slip)




WORD(SetEEPosePX)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  shared_ptr<DoubleWord> vword = make_shared<DoubleWord>(word->value().px);
  ms->pushData(vword);
}
END_WORD
REGISTER_WORD(EePosePX)


WORD(EePosePY)
virtual void execute(MachineState * ms) {
  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  shared_ptr<DoubleWord> vword = make_shared<DoubleWord>(word->value().py);
  ms->pushData(vword);
}
END_WORD
REGISTER_WORD(EePosePY)


WORD(EePosePZ)
virtual void execute(MachineState * ms) {
  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  shared_ptr<DoubleWord> vword = make_shared<DoubleWord>(word->value().pz);
  ms->pushData(vword);
}
END_WORD
REGISTER_WORD(EePosePZ)


WORD(EePoseQX)
virtual void execute(MachineState * ms) {
  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  shared_ptr<DoubleWord> vword = make_shared<DoubleWord>(word->value().qx);
  ms->pushData(vword);
}
END_WORD
REGISTER_WORD(EePoseQX)


WORD(EePoseQY)
virtual void execute(MachineState * ms) {
  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  shared_ptr<DoubleWord> vword = make_shared<DoubleWord>(word->value().qy);
  ms->pushData(vword);
}
END_WORD
REGISTER_WORD(EePoseQY)


WORD(EePoseQZ)
virtual void execute(MachineState * ms) {
  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  shared_ptr<DoubleWord> vword = make_shared<DoubleWord>(word->value().qz);
  ms->pushData(vword);
}
END_WORD
REGISTER_WORD(EePoseQZ)


WORD(EePoseQW)
virtual void execute(MachineState * ms) {
  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);
  shared_ptr<DoubleWord> vword = make_shared<DoubleWord>(word->value().qw);
  ms->pushData(vword);
}
END_WORD
REGISTER_WORD(EePoseQW)




WORD(Store)
virtual string description() {
  return "Create a new variable.  Usage  <value> \"<name>\" store.  After that you can say <name> and it will push its contents on the call stack.";
}
virtual void execute(MachineState * ms) {
  std::shared_ptr<Word> nameword = ms->popData();
  std::shared_ptr<Word> valueword = ms->popData();
  if (nameword == NULL || valueword == NULL) {
    CONSOLE_ERROR(ms, "Store takes two arguments.");
    ms->pushWord("pauseStackExecution");   
  } else {
    string name = nameword->to_string();
    //cout << "Storing " << name << " value " << valueword << endl;
    ms->variables[name] = valueword;
  }
}
END_WORD
REGISTER_WORD(Store)

WORD(Expand)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  string filename;
  GET_ARG(ms, StringWord, filename);
  std::stringstream fname;
  fname << "src/ein/back/" << filename << ".back";
  CONSOLE(ms, "Importing: " << fname.str());
  std::ifstream t(fname.str());
  if (!t.is_open()) {
    CONSOLE_ERROR(ms, "Import tried to read " << fname.str() << ", but it couldn't open...");
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
virtual void execute(MachineState * ms) {
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


WORD(IncrementCamera)
virtual void execute(MachineState * ms)
{
  int newCamera = (ms->config.focused_camera + 1) % ms->config.cameras.size();
  changeCamera(ms, newCamera);
}
END_WORD
REGISTER_WORD(IncrementCamera)


WORD(DecrementCamera)
virtual void execute(MachineState * ms)
{
  int newCamera = (ms->config.focused_camera - 1) % ms->config.cameras.size();
  changeCamera(ms, newCamera);
}
END_WORD
REGISTER_WORD(DecrementCamera)




WORD(IncrementTargetClass)
CODE(196437)// capslock + pageup
virtual void execute(MachineState * ms)
{
  if (ms->config.numClasses > 0) {
    int newTargetClass = (ms->config.targetClass + 1) % ms->config.numClasses;
    changeTargetClass(ms, newTargetClass);
  }
}
END_WORD
REGISTER_WORD(IncrementTargetClass)

WORD(ChangeTargetClassToClosestBlueBox)
virtual void execute(MachineState * ms)  {
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
virtual void execute(MachineState * ms)
{

}
END_WORD
REGISTER_WORD(Noop)

WORD(EndStackCollapseNoop)
virtual void execute(MachineState * ms)
{
  ms->config.endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(EndStackCollapseNoop)

WORD(ExportWords)
virtual void execute(MachineState * ms)
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

WORD(ExportDoc)
virtual void execute(MachineState * ms)
{
  string wordFileName = "ein_words.html";
  cout << "Writing words to " << wordFileName << endl;
  ofstream wordFile;
  wordFile.open(wordFileName);
  wordFile << "<table><tr><th>Word</th><th>Description</th></tr>" << endl;

  map<string, shared_ptr<Word> > words = ms->wordsInNamespace();
  std::map<std::string, shared_ptr<Word> >::iterator iter;
  for (iter = words.begin(); iter != words.end(); ++iter) {
    wordFile << "<tr><td>" << xmlEncode(iter->first) << "</td><td>" << xmlEncode(iter->second->description()) << "</td></tr>" << endl;
  }
  wordFile << "</table>" << endl;
  wordFile.close();
}
END_WORD
REGISTER_WORD(ExportDoc)



WORD(PixelGlobalTest)
CODE(65609) // I
virtual void execute(MachineState * ms)
{
  ms->config.paintEEandReg1OnWrist = !ms->config.paintEEandReg1OnWrist;
}
END_WORD
REGISTER_WORD(PixelGlobalTest)

WORD(IncMx)
CODE(65361) // left arrow 
virtual void execute(MachineState * ms)
{
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  camera->m_x += .01;
  camera->m_x_h[ms->config.currentThompsonHeightIdx] = camera->m_x;
  cout << "m_x, m_x_h: " << camera->m_x << endl;
}
END_WORD
REGISTER_WORD(IncMx)

WORD(DecMx)
CODE(65363) // right arrow 
virtual void execute(MachineState * ms)
{
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  camera->m_x -= .01;
  camera->m_x_h[ms->config.currentThompsonHeightIdx] = camera->m_x;
  cout << "m_x, m_x_h: " << camera->m_x << endl;
}
END_WORD
REGISTER_WORD(DecMx)

WORD(IncMy)
CODE(65362) // up arrow 
virtual void execute(MachineState * ms)
{
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  camera->m_y += .01;
  camera->m_y_h[ms->config.currentThompsonHeightIdx] = camera->m_y;
  cout << "m_y, m_y_h: " << camera->m_y << endl;
}
END_WORD
REGISTER_WORD(IncMy)

WORD(DecMy)
CODE(65364) // down arrow 
virtual void execute(MachineState * ms)
{
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  camera->m_y -= .01;
  camera->m_y_h[ms->config.currentThompsonHeightIdx] = camera->m_y;
  cout << "m_y, m_y_h: " << camera->m_y << endl;
}
END_WORD
REGISTER_WORD(DecMy)

WORD(CameraZeroNonLinear)
virtual void execute(MachineState * ms)
{
  // kill quadratic and constant terms
  // or use modes instead
}
END_WORD
REGISTER_WORD(CameraZeroNonLinear)

CONFIG_GETTER_INT(CameraGetCalibrationMode, ms->config.cameras[ms->config.focused_camera]->currentCameraCalibrationMode);
CONFIG_SETTER_ENUM(CameraSetCalibrationMode, ms->config.cameras[ms->config.focused_camera]->currentCameraCalibrationMode, (cameraCalibrationMode));

CONFIG_GETTER_INT(SceneGetFixationMode, ms->config.currentSceneFixationMode);
CONFIG_SETTER_ENUM(SceneSetFixationMode, ms->config.currentSceneFixationMode, (sceneFixationMode));

WORD(CameraFitQuadratic)
virtual void execute(MachineState * ms)
{
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  double bBZ[4];
  bBZ[0] = convertHeightIdxToGlobalZ(ms, 0) + ms->config.currentTableZ;
  bBZ[1] = convertHeightIdxToGlobalZ(ms, 1) + ms->config.currentTableZ;
  bBZ[2] = convertHeightIdxToGlobalZ(ms, 2) + ms->config.currentTableZ;
  bBZ[3] = convertHeightIdxToGlobalZ(ms, 3) + ms->config.currentTableZ;

  cout << "cameraFitQuadratic: " << endl;
  {
    cout << "  running y reticles... 0 1 2 3: " <<
       camera->m_y_h[0] << " " <<
       camera->m_y_h[1] << " " <<
       camera->m_y_h[2] << " " <<
       camera->m_y_h[3] << endl;

    Vector3d beta;
    Vector4d Y;
    Matrix<double, 4, 3> X;

    X << 1 , bBZ[0] , bBZ[0] * bBZ[0] 
       , 1 , bBZ[1] , bBZ[1] * bBZ[1] 
       , 1 , bBZ[2] , bBZ[2] * bBZ[2] 
       , 1 , bBZ[3] , bBZ[3] * bBZ[3];

    Y << camera->m_y_h[0]
       , camera->m_y_h[1]
       , camera->m_y_h[2]
       , camera->m_y_h[3];

    beta = (X.transpose() * X).inverse() * X.transpose() * Y;

    cout << "beta: " << endl << beta << endl << "X: " << endl << X << endl << "Y: " << endl << Y << endl << "X times beta: " << endl << X * beta << endl;

    camera->m_YQ[0] = beta(0);
    camera->m_YQ[1] = beta(1);
    camera->m_YQ[2] = beta(2);
  }
  {
    cout << "  running x reticles... 0 1 2 3: " <<
       camera->m_x_h[0] << " " <<
       camera->m_x_h[1] << " " <<
       camera->m_x_h[2] << " " <<
       camera->m_x_h[3] << endl;

    Vector3d beta;
    Vector4d Y;
    Matrix<double, 4, 3> X;

    X << 1 , bBZ[0] , bBZ[0] * bBZ[0] 
       , 1 , bBZ[1] , bBZ[1] * bBZ[1] 
       , 1 , bBZ[2] , bBZ[2] * bBZ[2] 
       , 1 , bBZ[3] , bBZ[3] * bBZ[3];

    Y << camera->m_x_h[0]
       , camera->m_x_h[1]
       , camera->m_x_h[2]
       , camera->m_x_h[3];

    beta = (X.transpose() * X).inverse() * X.transpose() * Y;

    cout << "beta: " << endl << beta << endl << "X: " << endl << X << endl << "Y: " << endl << Y << endl << "X times beta: " << endl << X * beta << endl;

    camera->m_XQ[0] = beta(0);
    camera->m_XQ[1] = beta(1);
    camera->m_XQ[2] = beta(2);
  }
}
END_WORD
REGISTER_WORD(CameraFitQuadratic)

WORD(CameraFitHyperbolic)
virtual void execute(MachineState * ms)
{
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  double bBZ[4];
  bBZ[0] = convertHeightIdxToGlobalZ(ms, 0) + ms->config.currentTableZ;
  bBZ[1] = convertHeightIdxToGlobalZ(ms, 1) + ms->config.currentTableZ;
  bBZ[2] = convertHeightIdxToGlobalZ(ms, 2) + ms->config.currentTableZ;
  bBZ[3] = convertHeightIdxToGlobalZ(ms, 3) + ms->config.currentTableZ;

  if (	bBZ[0] == 0 || 
	bBZ[1] == 0 || 
	bBZ[2] == 0 || 
	bBZ[3] == 0 ) {
    //cout << "cameraFitHyperbolic: error, bailing" << endl;
  }

  bBZ[0] = 1.0/bBZ[0];     
  bBZ[1] = 1.0/bBZ[1];  
  bBZ[2] = 1.0/bBZ[2];
  bBZ[3] = 1.0/bBZ[3];  


  //cout << "cameraFitHyperbolic: " << endl;
  {
    //cout << "  running y reticles... 0 1 2 3: " <<
    // camera->m_y_h[0] << " " <<
    // camera->m_y_h[1] << " " <<
    // camera->m_y_h[2] << " " <<
    // camera->m_y_h[3] << endl;

    Vector3d beta;
    Vector4d Y;
    Matrix<double, 4, 3> X;

    X << 1 , bBZ[0] , bBZ[0] * bBZ[0] 
       , 1 , bBZ[1] , bBZ[1] * bBZ[1] 
       , 1 , bBZ[2] , bBZ[2] * bBZ[2] 
       , 1 , bBZ[3] , bBZ[3] * bBZ[3];

    Y << camera->m_y_h[0]
       , camera->m_y_h[1]
       , camera->m_y_h[2]
       , camera->m_y_h[3];

    beta = (X.transpose() * X).inverse() * X.transpose() * Y;

    //cout << "beta: " << endl << beta << endl << "X: " << endl << X << endl << "Y: " << endl << Y << endl << "X times beta: " << endl << X * beta << endl;

    camera->m_YQ[0] = beta(0);
    camera->m_YQ[1] = beta(1);
    camera->m_YQ[2] = beta(2);
  }
  {
    //cout << "  running x reticles... 0 1 2 3: " <<
    //camera->m_x_h[0] << " " <<
    //camera->m_x_h[1] << " " <<
    //camera->m_x_h[2] << " " <<
    //camera->m_x_h[3] << endl;

    Vector3d beta;
    Vector4d Y;
    Matrix<double, 4, 3> X;

    X << 1 , bBZ[0] , bBZ[0] * bBZ[0] 
       , 1 , bBZ[1] , bBZ[1] * bBZ[1] 
       , 1 , bBZ[2] , bBZ[2] * bBZ[2] 
       , 1 , bBZ[3] , bBZ[3] * bBZ[3];

    Y << camera->m_x_h[0]
       , camera->m_x_h[1]
       , camera->m_x_h[2]
       , camera->m_x_h[3];

    beta = (X.transpose() * X).inverse() * X.transpose() * Y;

    //cout << "beta: " << endl << beta << endl << "X: " << endl << X << endl << "Y: " << endl << Y << endl << "X times beta: " << endl << X * beta << endl;

    camera->m_XQ[0] = beta(0);
    camera->m_XQ[1] = beta(1);
    camera->m_XQ[2] = beta(2);
  }
}
END_WORD
REGISTER_WORD(CameraFitHyperbolic)

WORD(CameraPrintParams)
virtual void execute(MachineState * ms)
{
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  double gZ = ms->config.currentEEPose.pz;

  int x1 = camera->heightReticles[0].px;
  int x2 = camera->heightReticles[1].px;
  int x3 = camera->heightReticles[2].px;
  int x4 = camera->heightReticles[3].px;

  int y1 = camera->heightReticles[0].py;
  int y2 = camera->heightReticles[1].py;
  int y3 = camera->heightReticles[2].py;
  int y4 = camera->heightReticles[3].py;

  double z1 = convertHeightIdxToGlobalZ(ms, 0) + ms->config.currentTableZ;
  double z2 = convertHeightIdxToGlobalZ(ms, 1) + ms->config.currentTableZ;
  double z3 = convertHeightIdxToGlobalZ(ms, 2) + ms->config.currentTableZ;
  double z4 = convertHeightIdxToGlobalZ(ms, 3) + ms->config.currentTableZ;

  {
    //double d = ms->config.d_x;
    double d = ms->config.d_x/camera->m_x;
    double c = ((z4*x4-z2*x2)*(x3-x1)-(z3*x3-z1*x1)*(x4-x2))/((z1-z3)*(x4-x2)-(z2-z4)*(x3-x1));

    double b42 = (z4*x4-z2*x2+(z2-z4)*c)/(x4-x2);
    double b31 = (z3*x3-z1*x1+(z1-z3)*c)/(x3-x1);

    double bDiff = b42-b31;
    //cout << "x1 x2 x3 x4: " << x1 << " " << x2 << " " << x3 << " " << x4 << endl;
    //cout << "y1 y2 y3 y4: " << y1 << " " << y2 << " " << y3 << " " << y4 << endl;
    //cout << "z1 z2 z3 z4: " << z1 << " " << z2 << " " << z3 << " " << z4 << endl;
    //cout << "bDiff = " << bDiff << ", c = " << c << " b42, b31: " << b42 << " " << b31 << " " << endl;
    double b = (b42+b31)/2.0;

    int x_thisZ = c + ( (x1-c)*(z1-b) )/(gZ-b);
    //int x_thisZ = c + ( camera->m_x*(x1-c)*(z1-b) )/(gZ-b);
    //*pX = c + ( (gX-d)*(x1-c) )/(ms->config.currentEEPose.px-d);
    //*pX = c + ( (gX-d)*(x_thisZ-c) )/(ms->config.currentEEPose.px-d);
    //*pX = c + ( camera->m_x*(gX-ms->config.trueEEPose.position.x+d)*(x_thisZ-c) )/(d);
    // need to set this again so things match up if gX is truEEpose
    //x_thisZ = c + ( camera->m_x*(x1-c)*(z1-b) )/(gZ-b);
    //x_thisZ = c + ( (d)*(x_thisZ-c) )/(d);
    // removed the above correction

    cout << "(x pass) d c b42 b31 bDiff b x_thisZ m_x: " << endl 
	 << d << " " << c << " " << b42 << " " << b31 << " " << bDiff << " " << b << " " << x_thisZ << " "  << camera->m_x << " " << endl;
/*
    cout << "x1 x2 x3 x4: " << x1 << " " << x2 << " " << x3 << " " << x4 << endl;
    cout << "z1 z2 z3 z4: " << z1 << " " << z2 << " " << z3 << " " << z4 << endl;
*/
  }
  {
    //double d = ms->config.d_y;
    double d = ms->config.d_y/camera->m_y;
    double c = ((z4*y4-z2*y2)*(y3-y1)-(z3*y3-z1*y1)*(y4-y2))/((z1-z3)*(y4-y2)-(z2-z4)*(y3-y1));

    double b42 = (z4*y4-z2*y2+(z2-z4)*c)/(y4-y2);
    double b31 = (z3*y3-z1*y1+(z1-z3)*c)/(y3-y1);

    double bDiff = b42-b31;
    //cout << "x1 x2 x3 x4: " << x1 << " " << x2 << " " << x3 << " " << x4 << endl;
    //cout << "y1 y2 y3 y4: " << y1 << " " << y2 << " " << y3 << " " << y4 << endl;
    //cout << "z1 z2 z3 z4: " << z1 << " " << z2 << " " << z3 << " " << z4 << endl;
    //cout << "bDiff = " << bDiff << ", c = " << c << " b42, b31: " << b42 << " " << b31 << " " << endl;
    double b = (b42+b31)/2.0;

    int y_thisZ = c + ( (y1-c)*(z1-b) )/(gZ-b);
    //int y_thisZ = c + ( camera->m_y*(y1-c)*(z1-b) )/(gZ-b);
    //*pY = c + ( (gY-d)*(y1-c) )/(ms->config.currentEEPose.py-d);
    //*pY = c + ( (gY-d)*(y_thisZ-c) )/(ms->config.currentEEPose.py-d);
    //*pY = c + ( camera->m_y*(gY-ms->config.trueEEPose.position.y+d)*(y_thisZ-c) )/(d);
    // need to set this again so things match up if gX is truEEpose
    //y_thisZ = c + ( camera->m_y*(y1-c)*(z1-b) )/(gZ-b);
    //y_thisZ = c + ( (d)*(y_thisZ-c) )/(d);
    // XXX removed the above correction still need to check

    cout << "(y pass) d c b42 b31 bDiff b y_thisZ m_y: " << endl 
	 << d << " " << c << " " << b42 << " " << b31 << " " << bDiff << " " << b << " " << y_thisZ << " "  << camera->m_y << " " << endl;
  }
}
END_WORD
REGISTER_WORD(CameraPrintParams)


WORD(EndStackCollapse)
virtual void execute(MachineState * ms)
{
  ms->config.endCollapse = 1;
}
END_WORD
REGISTER_WORD(EndStackCollapse)

WORD(CollapseStack)
virtual void execute(MachineState * ms)
{
  ms->config.endCollapse = 0;
}
END_WORD
REGISTER_WORD(CollapseStack)

WORD(ShakeHeadPositive)
virtual void execute(MachineState * ms)
{
  ms->config.currentHeadPanCommand.target = 3.1415926/2.0;
#ifdef RETHINK_SDK_1_2_0
  ms->config.currentHeadPanCommand.speed_ratio = 0.5;
#else
  ms->config.currentHeadPanCommand.speed = 50;
#endif
  ms->config.headPub.publish(ms->config.currentHeadPanCommand);
}
END_WORD
REGISTER_WORD(ShakeHeadPositive)


WORD(ShakeHeadNegative)
virtual void execute(MachineState * ms)
{
  ms->config.currentHeadPanCommand.target = -3.1415926/2.0;
#ifdef RETHINK_SDK_1_2_0
  ms->config.currentHeadPanCommand.speed_ratio = 0.5;
#else
  ms->config.currentHeadPanCommand.speed = 50;
#endif
  ms->config.headPub.publish(ms->config.currentHeadPanCommand);
}
END_WORD
REGISTER_WORD(ShakeHeadNegative)

WORD(CenterHead)
virtual void execute(MachineState * ms)
{
  ms->config.currentHeadPanCommand.target = 0;
#ifdef RETHINK_SDK_1_2_0
  ms->config.currentHeadPanCommand.speed_ratio = 0.5;
#else
  ms->config.currentHeadPanCommand.speed = 50;
#endif
  ms->config.headPub.publish(ms->config.currentHeadPanCommand);
}
END_WORD
REGISTER_WORD(CenterHead)

WORD(SetHeadPanTargetSpeed)
virtual void execute(MachineState * ms)
{
  double t_target;
  double t_speed;
  GET_NUMERIC_ARG(ms, t_speed);
  GET_NUMERIC_ARG(ms, t_target);

  cout << "setHeadPanTargetSpeed: " << t_target << " " << t_speed << endl;

  ms->config.currentHeadPanCommand.target = t_target;
#ifdef RETHINK_SDK_1_2_0
  ms->config.currentHeadPanCommand.speed_ratio = floor(t_speed);
#else
  ms->config.currentHeadPanCommand.speed = floor(100 * t_speed);
#endif
  ms->config.headPub.publish(ms->config.currentHeadPanCommand);
}
END_WORD
REGISTER_WORD(SetHeadPanTargetSpeed)

WORD(SilenceSonar)
virtual void execute(MachineState * ms)
{
  ms->config.currentSonarCommand.data = 0;
  ms->config.sonarPub.publish(ms->config.currentSonarCommand);
}
END_WORD
REGISTER_WORD(SilenceSonar)

WORD(UnSilenceSonar)
virtual void execute(MachineState * ms)
{
  ms->config.currentSonarCommand.data = 1;
  ms->config.sonarPub.publish(ms->config.currentSonarCommand);
}
END_WORD
REGISTER_WORD(UnSilenceSonar)

WORD(Nod)
virtual void execute(MachineState * ms)
{
  ms->config.currentHeadNodCommand.data = 1;
  ms->config.nodPub.publish(ms->config.currentHeadNodCommand);
}
END_WORD
REGISTER_WORD(Nod)

WORD(ResetAuxiliary)
virtual void execute(MachineState * ms)
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
virtual void execute(MachineState * ms)
{
  ms->config.lastImageCallbackRequest = ros::Time::now();
  ms->pushWord("waitUntilImageCallbackReceivedA");
  ms->config.shouldIImageCallback = 1;
  ms->config.endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(WaitUntilImageCallbackReceived)

WORD(WaitUntilImageCallbackReceivedA)
virtual void execute(MachineState * ms)
{
  if (ms->config.lastImageCallbackRequest >= ms->config.cameras[ms->config.focused_camera]->lastImageCallbackReceived) {
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
virtual void execute(MachineState * ms)
{
  ms->config.lastAccelerometerCallbackRequest = ros::Time::now();
  ms->pushWord("waitUntilAccelerometerCallbackReceivedA");
  ms->config.endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(WaitUntilAccelerometerCallbackReceived)

WORD(WaitUntilAccelerometerCallbackReceivedA)
virtual void execute(MachineState * ms)
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
virtual void execute(MachineState * ms)
{
  ms->config.lastEndpointCallbackRequest = ros::Time::now();
  ms->pushWord("waitUntilEndpointCallbackReceivedA");
  ms->config.endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(WaitUntilEndpointCallbackReceived)

WORD(WaitUntilEndpointCallbackReceivedA)
virtual void execute(MachineState * ms)
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



WORD(WaitUntilRingBufferImageAtCurrentPosition)
virtual void execute(MachineState * ms)
{

  Mat ringImage;
  eePose thisPose;
  ros::Time time;
  int result = getMostRecentRingImageAndPose(ms, &ringImage, &thisPose, &time, false);
  double distance, angleDistance;
  eePose::distanceXYZAndAngle(ms->config.currentEEPose, thisPose, &distance, &angleDistance);
  if (result != 1) { 
    ROS_ERROR("Warning:  waitUntilRingBufferImageAtCurrentPosition got an error when accessing the ring buffer.");
    ms->pushWord("waitUntilRingBufferImageAtCurrentPosition");
    ms->config.endThisStackCollapse = 1;
    return;
  }
  if ((distance > ms->config.w1GoThresh*ms->config.w1GoThresh) || (angleDistance > ms->config.w1AngleThresh*ms->config.w1AngleThresh)) {
    ms->pushWord("waitUntilRingBufferImageAtCurrentPosition");
    ms->config.endThisStackCollapse = 1;
  } else {
    ms->config.endThisStackCollapse = ms->config.endCollapse;
  }
}
END_WORD
REGISTER_WORD(WaitUntilRingBufferImageAtCurrentPosition)


WORD(WriteXMLEnvironment)
virtual void execute(MachineState * ms)
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
virtual void execute(MachineState * ms)
{
  int sis = system("bash -c \"echo -e \'C\003\' | rosrun baxter_tools enable_robot.py -d\"");
}
END_WORD
REGISTER_WORD(DisableRobot)

WORD(EnableRobot)
virtual void execute(MachineState * ms)
{
  int sis = system("bash -c \"echo -e \'C\003\' | rosrun baxter_tools enable_robot.py -e\"");
}
END_WORD
REGISTER_WORD(EnableRobot)

WORD(ReplicateWord)
virtual void execute(MachineState * ms)
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


WORD(PushHelp)
virtual string description() {
  return "Push help text for the word on the data stack.";
}
virtual void execute(MachineState * ms)
{
  shared_ptr<Word> word;
  GET_WORD_ARG(ms, Word, word);
  shared_ptr<StringWord> outword = std::make_shared<StringWord>(word->description());
  ms->pushWord(outword);
}
END_WORD
REGISTER_WORD(PushHelp)

WORD(Help)
virtual string description() {
  return "Return help text for a word, dereferencing the symbol if necessary.  Usage:   ' < word >   help.";
}
virtual void execute(MachineState * ms)
{
  ms->evaluateProgram("derefToTruth pushHelp print");
}
END_WORD
REGISTER_WORD(Help)



WORD(SetHelp)
virtual string description() {
  return "Make a new compound word with specified description text.  Usage:  <compound word> < help text > setHelp -> < compound word with help text >. ";
}
virtual void execute(MachineState * ms)
{
  string description;
  GET_STRING_ARG(ms, description);
  
  shared_ptr<CompoundWord> cWord;
  GET_WORD_ARG(ms, CompoundWord, cWord);

  shared_ptr<CompoundWord> cp = CompoundWord::copy(cWord);
  cp->setDescription(description);
  ms->pushData(cp);
}

END_WORD
REGISTER_WORD(SetHelp)


WORD(Define)
virtual string description() {
  return "Store a new compound word with specified body, description, and name.  Usage:  <compound word> < help text > < name > define. ";
}
virtual void execute(MachineState * ms)
{
  shared_ptr<StringWord> name;
  GET_WORD_ARG(ms, StringWord, name);

  shared_ptr<StringWord> description;
  GET_WORD_ARG(ms, StringWord, description);

  shared_ptr<CompoundWord> body;
  GET_WORD_ARG(ms, CompoundWord, body);

  // Then set up the store
  ms->pushData(name);
  ms->pushWord("store");
  ms->pushWord("swap");

  // First set the help of the compound word
  ms->pushData(body);
  ms->pushData(description);
  ms->pushWord("setHelp");
}
END_WORD
REGISTER_WORD(Define)





WORD(Map)
virtual void execute(MachineState * ms)
{
  shared_ptr<CompoundWord> lambdaWord;
  GET_WORD_ARG(ms, CompoundWord, lambdaWord);

  shared_ptr<CompoundWord> listWord;
  GET_WORD_ARG(ms, CompoundWord, listWord);

  ms->pushWord("]");

  for (int i = 0; i < listWord->size(); i++) {
    for (int j = 0; j < lambdaWord->size(); j++) {
      ms->pushWord(lambdaWord->getWord(j));
    }
    ms->pushWord(listWord->getWord(i));
  }
  ms->pushWord("[");

}
END_WORD
REGISTER_WORD(Map)


WORD(Accumulate)
virtual void execute(MachineState * ms)
{
  shared_ptr<CompoundWord> lambdaWord;
  GET_WORD_ARG(ms, CompoundWord, lambdaWord);

  shared_ptr<CompoundWord> listWord;
  GET_WORD_ARG(ms, CompoundWord, listWord);

  for (int i = 0; i < listWord->size(); i++) {
    ms->pushData(listWord->getWord(listWord->size() - 1 - i));
  }

  for (int i = 0; i < listWord->size() - 1; i++) {
    for (int j = 0; j < lambdaWord->size(); j++) {
      ms->pushWord(lambdaWord->getWord(j));
    }
  }
}
END_WORD
REGISTER_WORD(Accumulate)


WORD(Car)
virtual void execute(MachineState * ms)
{
  shared_ptr<CompoundWord> word;
  GET_WORD_ARG(ms, CompoundWord, word);

  if (word->size() != 0) {
    ms->pushData(word->getWord(word->size() - 1));
  }
}
END_WORD
REGISTER_WORD(Car)

WORD(Append)
virtual void execute(MachineState * ms)
{
  shared_ptr<CompoundWord> w1;
  GET_WORD_ARG(ms, CompoundWord, w1);

  shared_ptr<CompoundWord> w2;
  GET_WORD_ARG(ms, CompoundWord, w2);

  shared_ptr<CompoundWord> newWord = CompoundWord::copy(w1);
  for (int i = 0; i < w2->size(); i++) {
    newWord->pushWord(w2->getWord(i));
  }
  ms->pushData(newWord);
}
END_WORD
REGISTER_WORD(Append)


WORD(Cdr)
virtual void execute(MachineState * ms)
{
  shared_ptr<CompoundWord> word;
  GET_WORD_ARG(ms, CompoundWord, word);

  shared_ptr<CompoundWord> newWord = CompoundWord::copy(word);
  newWord->popWord();
  ms->pushData(newWord);
}
END_WORD
REGISTER_WORD(Cdr)


WORD(CurrentIKModeString)
virtual void execute(MachineState * ms)
{
  shared_ptr<StringWord> currentIkModeWord = make_shared<StringWord>(ikModeToString(ms->config.currentIKMode));
  ms->pushWord(currentIkModeWord);
}
END_WORD
REGISTER_WORD(CurrentIKModeString)





WORD(IkModeService)
virtual void execute(MachineState * ms)
{
  ms->config.currentIKMode = IKSERVICE;
}
END_WORD
REGISTER_WORD(IkModeService)


WORD(IkModeIkFast)
virtual void execute(MachineState * ms)
{
  ms->config.currentIKMode = IKFAST;
}
END_WORD
REGISTER_WORD(IkModeIkFast)


WORD(IkModeIkFastDebug)
virtual void execute(MachineState * ms)
{
  ms->config.currentIKMode = IKFASTDEBUG;
}
END_WORD
REGISTER_WORD(IkModeIkFastDebug)

WORD(ResetAveragedWrenchNorm)
virtual void execute(MachineState * ms)
{
  ms->config.averagedWrechAcc = 0;
  ms->config.averagedWrechMass = 0;
}
END_WORD
REGISTER_WORD(ResetAveragedWrenchNorm)


WORD(AnalogIOCommand)
virtual void execute(MachineState * ms)
{
  string component;
  double value;
  GET_ARG(ms, StringWord, component);
  GET_NUMERIC_ARG(ms, value);

  baxter_core_msgs::AnalogOutputCommand thisCommand;

  thisCommand.name = component;
  thisCommand.value = value;

  ms->config.analog_io_pub.publish(thisCommand);
}
END_WORD
REGISTER_WORD(AnalogIOCommand)

WORD(TorsoFanOn)
virtual void execute(MachineState * ms)
{
  ms->evaluateProgram("100 \"torso_fan\" analogIOCommand");
}
END_WORD
REGISTER_WORD(TorsoFanOn)

WORD(TorsoFanOff)
virtual void execute(MachineState * ms)
{
  ms->evaluateProgram("1 \"torso_fan\" analogIOCommand");
}
END_WORD
REGISTER_WORD(TorsoFanOff)

WORD(TorsoFanAuto)
virtual void execute(MachineState * ms)
{
  ms->evaluateProgram("0 \"torso_fan\" analogIOCommand");
}
END_WORD
REGISTER_WORD(TorsoFanAuto)


WORD(SetTorsoFanLevel)
virtual void execute(MachineState * ms)
{
  double value;
  GET_NUMERIC_ARG(ms, value);
  std::stringstream program;
  program << value << " \"torso_fan\" analogIOCommand";
  ms->evaluateProgram(program.str());
}
END_WORD
REGISTER_WORD(SetTorsoFanLevel)



WORD(DigitalIOCommand)
virtual void execute(MachineState * ms)
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
virtual void execute(MachineState * ms)
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
virtual void execute(MachineState * ms)
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
virtual void execute(MachineState * ms)
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
virtual void execute(MachineState * ms)
{
  std::stringstream program;
  program << "100 setGreenHalo 100 setRedHalo 4095 setSonarLed ";
  program << "1 \"left_itb_light_inner\" digitalIOCommand 1 \"right_itb_light_inner\" digitalIOCommand 1 \"torso_left_itb_light_inner\" digitalIOCommand 1 \"torso_right_itb_light_inner\" digitalIOCommand 1 \"left_itb_light_outer\" digitalIOCommand 1 \"right_itb_light_outer\" digitalIOCommand 1 \"torso_left_itb_light_outer\" digitalIOCommand 1 \"torso_right_itb_light_outer\" digitalIOCommand";
  ms->evaluateProgram(program.str());
}
END_WORD
REGISTER_WORD(LightsOn)

WORD(LightsOff)
virtual void execute(MachineState * ms)
{
  std::stringstream program;
  program << "0 setGreenHalo 0 setRedHalo 32768 setSonarLed ";
  program << "0 \"left_itb_light_inner\" digitalIOCommand 0 \"right_itb_light_inner\" digitalIOCommand 0 \"torso_left_itb_light_inner\" digitalIOCommand 0 \"torso_right_itb_light_inner\" digitalIOCommand 0 \"left_itb_light_outer\" digitalIOCommand 0 \"right_itb_light_outer\" digitalIOCommand 0 \"torso_left_itb_light_outer\" digitalIOCommand 0 \"torso_right_itb_light_outer\" digitalIOCommand";
  ms->evaluateProgram(program.str());
}
END_WORD
REGISTER_WORD(LightsOff)

WORD(SwitchSonarLed)
virtual void execute(MachineState * ms)
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
virtual void execute(MachineState * ms)
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
virtual void execute(MachineState * ms)
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
virtual void execute(MachineState * ms)
{
  CONSOLE_ERROR(ms, "Close parenthesis should never execute.");
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
virtual void execute(MachineState * ms)
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
virtual void execute(MachineState * ms)
{
  int scopeLevel = 0;
  GET_ARG(ms, IntegerWord, scopeLevel);

  if (scopeLevel < 0) {
    CONSOLE_ERROR(ms, "sP scope error.");
    ms->pushWord("pauseStackExecution");
    return;
  } else if (scopeLevel == 0) {
    return;
  } else {
    // continue
  }

  std::shared_ptr<Word> word = ms->popWord();

  if (word == NULL) {
    CONSOLE_ERROR(ms, "sP found no word...");
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
	      CONSOLE_ERROR(ms, "sP found no datum.");
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
virtual void execute(MachineState * ms) {
  ms->execution_mode = INSTANT;
}
END_WORD
REGISTER_WORD(ExecutionModeInstant)

WORD(ExecutionModeStep)
virtual void execute(MachineState * ms) {
  ms->execution_mode = STEP;
}
END_WORD
REGISTER_WORD(ExecutionModeStep)

WORD(Exec)
virtual void execute(MachineState * ms) {
  shared_ptr<Word> aWord;
  GET_WORD_ARG(ms, Word, aWord);
  ms->pushWord(aWord);
}
END_WORD
REGISTER_WORD(Exec)

WORD(CastToInteger)
virtual void execute(MachineState * ms) {
  double number = 0.0;
  GET_NUMERIC_ARG(ms, number);
  ms->pushData(std::make_shared<IntegerWord>(number));
}
END_WORD
REGISTER_WORD(CastToInteger)

WORD(Assert)
virtual void execute(MachineState * ms) {
  bool value;
  GET_BOOLEAN_ARG(ms, value);
  if (!value) {
    CONSOLE_ERROR(ms, "Failed assert. Pausing.");
    ms->pushWord("pauseStackExecution");
  }
}
END_WORD
REGISTER_WORD(Assert)


WORD(AssertNo)
virtual void execute(MachineState * ms) {
  bool value;
  GET_BOOLEAN_ARG(ms, value);
  if (value) {
    CONSOLE_ERROR(ms, "Failed assertNo. Pausing.");
    ms->pushWord("pauseStackExecution");
  }
}
END_WORD
REGISTER_WORD(AssertNo)


WORD(While)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  shared_ptr<StringWord> left_or_right = make_shared<StringWord>(ms->config.left_or_right_arm);
  ms->pushWord(left_or_right);
}
END_WORD
REGISTER_WORD(LeftOrRightArm)

WORD(IsGripperGripping)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  shared_ptr<IntegerWord> numBlueBoxes = make_shared<IntegerWord>(ms->config.bTops.size());
  ms->pushWord(numBlueBoxes);  
}
END_WORD
REGISTER_WORD(NumBlueBoxes)


WORD(DateString)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {

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
      CONSOLE_ERROR(ms, "oB found no word... pausing stack execution.");
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
      CONSOLE_ERROR(ms, "oB found no word during sB reinsert a... pausing stack execution.");
      ms->pushWord("pauseStackExecution");
      return;
    } else if ( 0 == word->name().compare("cB") ) {
      // good, should always happen
    } else {
      CONSOLE_ERROR(ms, "oB found no cB during sB reinsert a... pausing stack execution.");
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
	CONSOLE_ERROR(ms, "oB found no word during sB reinsert b... pausing stack execution.");
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
	CONSOLE_ERROR(ms, "oB found no word during sB reinsert c... pausing stack execution.");
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
    CONSOLE_ERROR(ms, "oB found no word on rewind... pausing stack execution.");
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
virtual void execute(MachineState * ms) {
  CONSOLE_ERROR(ms, "sB should never execute.");
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
virtual void execute(MachineState * ms) {
  CONSOLE_ERROR(ms, "Close bracket should never execute.");
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
virtual void execute(MachineState * ms) {
  int numToReserve = 0;
  GET_INT_ARG(ms, numToReserve);
  ms->pushCopies(std::make_shared<IntegerWord>(0), max(int(numToReserve - ms->data_stack.size()), int(0)));
}
END_WORD
REGISTER_WORD(Dsr)


WORD(OSB)
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("[");
  return result;
}
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("slide (");
}
END_WORD
REGISTER_WORD(OSB)

WORD(CSB)
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("]");
  return result;
}
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("1 |S )");
}
END_WORD
REGISTER_WORD(CSB)


WORD(Time)
virtual void execute(MachineState * ms) {
  shared_ptr<CompoundWord> block;
  GET_WORD_ARG(ms, CompoundWord, block);

  ms->evaluateProgram("time_endTime time_startTime -");
  ms->evaluateProgram("now \"time_endTime\" store");
  ms->pushWord(block);
  ms->evaluateProgram("now \"time_startTime\" store");
}
END_WORD
REGISTER_WORD(Time)

WORD(CommandOtherArm)
virtual void execute(MachineState * ms) {
  string string_in;
  GET_STRING_ARG(ms, string_in);

  std_msgs::String command;
  command.data = string_in;
  ms->config.forthCommandPublisher.publish(command);
}
END_WORD
REGISTER_WORD(CommandOtherArm)



WORD(Deref)
virtual string description() {
  return "Takes a symbol word argument from the data stack and pushes its current value back onto the data stack.  Usage:  < symbol word > deref -> < value > ";
}
virtual void execute(MachineState * ms) {
  shared_ptr<Word> w;
  GET_WORD_ARG(ms, Word, w);

  std::shared_ptr<SymbolWord> sWord = std::dynamic_pointer_cast<SymbolWord>(w);

  if (sWord != NULL) {
    shared_ptr<Word> outword = sWord->getReferencedWord(ms);
    if (outword != NULL) {
      ms->pushData(outword);
    } else {
      CONSOLE_ERROR(ms, "No reference for symbol " << sWord->name());
      ms->pushWord("pauseStackExecution");
    }
  } else {
    ms->pushData(w);
  }
}
END_WORD
REGISTER_WORD(Deref)


WORD(DerefToTruth)
virtual string description() {
  return "Takes a symbol word argument from the data stack and pushes its current value back onto the data stack.  Usage:  < symbol word > deref -> < value > ";
}
virtual void execute(MachineState * ms) {
  shared_ptr<Word> w;
  GET_WORD_ARG(ms, Word, w);

  std::shared_ptr<SymbolWord> sWord = std::dynamic_pointer_cast<SymbolWord>(w);

  if (sWord != NULL) {
    shared_ptr<Word> outword = sWord->getReferencedWord(ms);
    if (outword != NULL) {
      ms->pushData(outword);
      ms->pushWord("derefToTruth");
    } else {
      CONSOLE_ERROR(ms, "No reference for symbol " << sWord->name());
      ms->pushWord("pauseStackExecution");
    }
  } else {
    ms->pushData(w);
  }
}
END_WORD
REGISTER_WORD(DerefToTruth)


WORD(Repr)
virtual string description() {
  return "Takes an argument from the data stack pushes the string representation onto the data stack.  Usage:  < word > repr -> string";
}
virtual void execute(MachineState * ms) {
  shared_ptr<Word> w;
  GET_WORD_ARG(ms, Word, w);
  shared_ptr<StringWord> outword = std::make_shared<StringWord>(w->repr());
  ms->pushData(outword);
}
END_WORD
REGISTER_WORD(Repr)

WORD(Eval)
virtual string description() {
  return "Takes a string from the data stack; evaluates the string as a back program.  Usage:  < string > eval -> whatever the program does";
}
virtual void execute(MachineState * ms) {
  string program;
  GET_STRING_ARG(ms, program);
  ms->evaluateProgram(program);
}
END_WORD
REGISTER_WORD(Eval)


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


CONFIG_GETTER_STRING(DataDirectory, ms->config.data_directory)

CONFIG_GETTER_STRING(RobotSerial, ms->config.robot_serial)
CONFIG_GETTER_STRING(RobotSoftwareVersion, ms->config.robot_software_version)
CONFIG_GETTER_STRING(EinSoftwareVersion, ms->config.ein_software_version)

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



CONFIG_GETTER_INT(ObservedCameraFlip, ms->config.cameras[ms->config.focused_camera]->observedCameraFlip)
CONFIG_GETTER_INT(ObservedCameraMirror, ms->config.cameras[ms->config.focused_camera]->observedCameraMirror)

CONFIG_GETTER_INT(ObservedCameraExposure, ms->config.cameras[ms->config.focused_camera]->observedCameraExposure)
CONFIG_GETTER_INT(ObservedCameraGain, ms->config.cameras[ms->config.focused_camera]->observedCameraGain)
CONFIG_GETTER_INT(ObservedCameraWhiteBalanceRed, ms->config.cameras[ms->config.focused_camera]->observedCameraWhiteBalanceRed)
CONFIG_GETTER_INT(ObservedCameraWhiteBalanceGreen, ms->config.cameras[ms->config.focused_camera]->observedCameraWhiteBalanceGreen)
CONFIG_GETTER_INT(ObservedCameraWhiteBalanceBlue, ms->config.cameras[ms->config.focused_camera]->observedCameraWhiteBalanceBlue)
CONFIG_GETTER_INT(ObservedCameraWindowX, ms->config.cameras[ms->config.focused_camera]->observedCameraWindowX)
CONFIG_GETTER_INT(ObservedCameraWindowY, ms->config.cameras[ms->config.focused_camera]->observedCameraWindowY)

CONFIG_GETTER_INT(CameraExposure, ms->config.cameras[ms->config.focused_camera]->cameraExposure)
CONFIG_GETTER_INT(CameraGain, ms->config.cameras[ms->config.focused_camera]->cameraGain)
CONFIG_GETTER_INT(CameraWhiteBalanceRed, ms->config.cameras[ms->config.focused_camera]->cameraWhiteBalanceRed)
CONFIG_GETTER_INT(CameraWhiteBalanceGreen, ms->config.cameras[ms->config.focused_camera]->cameraWhiteBalanceGreen)
CONFIG_GETTER_INT(CameraWhiteBalanceBlue, ms->config.cameras[ms->config.focused_camera]->cameraWhiteBalanceBlue)


CONFIG_GETTER_INT(SceneCellCountThreshold, ms->config.sceneCellCountThreshold)
CONFIG_SETTER_INT(SceneSetCellCountThreshold, ms->config.sceneCellCountThreshold)


CONFIG_GETTER_INT(SceneDiscrepancySearchDepth, ms->config.sceneDiscrepancySearchDepth)
CONFIG_SETTER_INT(SceneSetDiscrepancySearchDepth, ms->config.sceneDiscrepancySearchDepth)

CONFIG_GETTER_INT(ArmOkButtonState, ms->config.lastArmOkButtonState)
CONFIG_GETTER_INT(ArmShowButtonState, ms->config.lastArmShowButtonState)
CONFIG_GETTER_INT(ArmBackButtonState, ms->config.lastArmBackButtonState)

CONFIG_GETTER_DOUBLE(TorsoFanState, ms->config.torsoFanState)

CONFIG_GETTER_INT(CurrentIKMode, ms->config.currentIKMode)

CONFIG_GETTER_DOUBLE(EeRange, ms->config.eeRange)
CONFIG_GETTER_DOUBLE(EeRangeMaxValue, ms->config.eeRangeMaxValue)

CONFIG_GETTER_DOUBLE(MostRecentUntabledZ, ms->config.mostRecentUntabledZ)

CONFIG_GETTER_INT(NumCameras, ms->config.cameras.size())

//CONFIG_GETTER_INT(NumIkMapHeights, ms->config.numIkMapHeights)




}
