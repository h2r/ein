
#include "ein_words.h"
#include "ein.h"

#include "ein/msg/ein_state.hpp"
using namespace ein;

#include <boost/filesystem.hpp>
#include <iostream>

#include <sys/stat.h>

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
    ein::msg::EinState state;
    fillEinStateMsg(ms, &state);
    ms->config.einStatePub->publish(state);
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
  ms->pushData(std::make_shared<DoubleWord>(rclcpp::Clock{}.now().seconds()));
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
  
  double start = rclcpp::Clock{}.now().seconds();
  int i = 0;
  for (i = 0; i < its; ) {
    i++;
  }
  double end = rclcpp::Clock{}.now().seconds();

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


WORD(PublishCommandsOn)
virtual string description() {
  return "Turn on publishing movement commands and other changing commands (lights, sonar).";
}
virtual void execute(MachineState * ms) {
  ms->config.publish_commands_mode = 1;
}
END_WORD
REGISTER_WORD(PublishCommandsOn)


WORD(PublishCommandsOff)
virtual string description() {
  return "Do not publish commands to the robot; useful if someone else (like MoveIt) is going to move the robot.";
}
virtual void execute(MachineState * ms) {
  ms->config.publish_commands_mode = 0;
}
END_WORD
REGISTER_WORD(PublishCommandsOff)


WORD(ZeroGToggle)
CODE('z')
virtual string description() {
  return "Toggle zero gravity mode.";
}
virtual void execute(MachineState * ms) {
  ms->config.zero_g_toggle = !ms->config.zero_g_toggle;
}
END_WORD
REGISTER_WORD(ZeroGToggle)

WORD(ZeroGOn)
virtual string description() {
  return "Turns on zero gravity mode, so that you can move the arm where you want and it will stay there.  Ein will publish the true joint position as the desired joint position.";
}
virtual void execute(MachineState * ms) {
  ms->config.zero_g_toggle = 1;
}
END_WORD
REGISTER_WORD(ZeroGOn)

WORD(ZeroGOff)
virtual string description() {
  return "Turns off zero gravity mode, so Ein will publish the current pose as the desired target position.";
}
virtual void execute(MachineState * ms) {
  ms->config.zero_g_toggle = 0;
}
END_WORD
REGISTER_WORD(ZeroGOff)

WORD(ClearStack)
CODE('r') 
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("clearCallStack");
  return result;
}
virtual string description() {
  return "Clear the call stack.";
}
virtual void execute(MachineState * ms) {
  ms->clearStack();
}
END_WORD
REGISTER_WORD(ClearStack)

WORD(ClearStacks)
virtual string description() {
  return "Clear the call stack and the data stack.";
}
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

WORD(Or)
CODE('|') 
virtual string description() {
  return "Returns logical or of its numeric arguments. Back does not have booleans so it treats 0 (and 0.0) as false, and all other numbers as true.";
}
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
virtual string description() {
  return "Returns logical and of its numeric arguments. Back does not have booleans so it treats 0 (and 0.0) as false, and all other numbers as true.";
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
virtual string description() {
  return "Takes two numbers and adds them.  If two ints, returns an int; otherwise returns a double; otherwise does string.";
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


WORD(Mod)
CODE('%') 
virtual string description() {
  return "Takes two ints and pushes the mod of the two ints on the stack.  25 4 % returns 1.";
}
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("%");
  return result;
}
virtual void execute(MachineState * ms) {

  int v1;
  GET_INT_ARG(ms, v1);
  int v2;
  GET_INT_ARG(ms, v2);

  std::shared_ptr<IntegerWord> newWord = std::make_shared<IntegerWord>(v2 % v1);
  ms->pushWord(newWord);

}
END_WORD
REGISTER_WORD(Mod)



WORD(Sum)
virtual string description() {
  return "Pops a compound word; sums the entries; pushes the result.  Usage:  ( 1 1 1 ) sum -> 3.";
}
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

WORD(Abs)
virtual string description() {
  return "Returns the absolute value of its argument.";
}
virtual void execute(MachineState * ms) {
  double val = 0.0;
  GET_NUMERIC_ARG(ms, val);
  ms->pushWord(make_shared<DoubleWord>(fabs(val)));
}
END_WORD
REGISTER_WORD(Abs)


WORD(Prod)
virtual string description() {
  return "Pops a compound word; multiplies the entries; pushes the result.  Usage:  ( 1 1 1 ) prod -> 1.";
}

virtual void execute(MachineState * ms) {
  ms->evaluateProgram(" ( * ) accumulate");
}
END_WORD
REGISTER_WORD(Prod)


WORD(Langle)
CODE('<') 
virtual string description() {
  return "Takes two words and returns 1 if they are less than (by value) and 0 otherwise.  1 3 < returns true, and 3 1 < returns false.";
}
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("<");
  return result;
}
virtual void execute(MachineState * ms) {


  shared_ptr<Word> w1;
  GET_WORD_ARG(ms, Word, w1);
  shared_ptr<Word> w2;
  GET_WORD_ARG(ms, Word, w2);
  
  try {
    int result = w2->compareTo(w1);
   
    std::shared_ptr<IntegerWord> newWord = std::make_shared<IntegerWord>(result < 0);
    ms->pushWord(newWord);
   
  } catch (domain_error& e) {
    CONSOLE_ERROR(ms, "Bad type: " << e.what() << " word " << name());
    ms->pushWord("pauseStackExecution");   
    return;
  }

}
END_WORD
REGISTER_WORD(Langle)

WORD(Rangle)
CODE('>') 
virtual string description() {
  return "Takes two words and returns 1 if they are greater than (by value) and 0 otherwise.  1 3 < returns false, and 3 1 < returns true";
}
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back(">");
  return result;
}
virtual void execute(MachineState * ms) {
  shared_ptr<Word> w1;
  GET_WORD_ARG(ms, Word, w1);
  shared_ptr<Word> w2;
  GET_WORD_ARG(ms, Word, w2);
  
  try {
    int result = w2->compareTo(w1);
   
    std::shared_ptr<IntegerWord> newWord = std::make_shared<IntegerWord>(result > 0);
    ms->pushWord(newWord);
   
  } catch (domain_error& e) {
    CONSOLE_ERROR(ms, "Bad type: " << e.what() << " word " << name());
    ms->pushWord("pauseStackExecution");   
    return;
  }
}
END_WORD
REGISTER_WORD(Rangle)

WORD(Leq)
virtual string description() {
  return "Takes two words and returns 1 if they are less than or equal to (by value) and 0 otherwise.";
}
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("<=");
  return result;
}
virtual void execute(MachineState * ms) {


  shared_ptr<Word> w1;
  GET_WORD_ARG(ms, Word, w1);
  shared_ptr<Word> w2;
  GET_WORD_ARG(ms, Word, w2);
  
  try {
    int result = w2->compareTo(w1);
   
    std::shared_ptr<IntegerWord> newWord = std::make_shared<IntegerWord>(result <= 0);
    ms->pushWord(newWord);
   
  } catch (domain_error& e) {
    CONSOLE_ERROR(ms, "Bad type: " << e.what() << " word " << name());
    ms->pushWord("pauseStackExecution");   
    return;
  }

}
END_WORD
REGISTER_WORD(Leq)

WORD(Geq)
virtual string description() {
  return "Takes two words and returns 1 if they are greater than or equal to (by value) and 0 otherwise.";
}
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back(">=");
  return result;
}
virtual void execute(MachineState * ms) {

  shared_ptr<Word> w1;
  GET_WORD_ARG(ms, Word, w1);
  shared_ptr<Word> w2;
  GET_WORD_ARG(ms, Word, w2);
  
  try {
    int result = w2->compareTo(w1);
   
    std::shared_ptr<IntegerWord> newWord = std::make_shared<IntegerWord>(result >= 0);
    ms->pushWord(newWord);
   
  } catch (domain_error& e) {
    CONSOLE_ERROR(ms, "Bad type: " << e.what() << " word " << name());
    ms->pushWord("pauseStackExecution");   
    return;
  }

}
END_WORD
REGISTER_WORD(Geq)

WORD(Cmp)
virtual string description() {
  return "Takes two words and returns 0 if they are equal, -1 if they are less than, and 1 if they are greater than.  Works on integers, doubles and strings.";
}

virtual void execute(MachineState * ms) {
  shared_ptr<Word> w1;
  GET_WORD_ARG(ms, Word, w1);
  shared_ptr<Word> w2;
  GET_WORD_ARG(ms, Word, w2);
  
  try {
    std::shared_ptr<IntegerWord> newWord = std::make_shared<IntegerWord>(w2->compareTo(w1));
    ms->pushWord(newWord);
  } catch (domain_error& e) {
    CONSOLE_ERROR(ms, "Bad type: " << e.what() << " word " << name());
    ms->pushWord("pauseStackExecution");   
    return;
  }

}
END_WORD
REGISTER_WORD(Cmp)

WORD(Times)
CODE('*') 
virtual string description() {
  return "Multiply two numeric arguments.";
}
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
virtual string description() {
  return "Divide two numeric arguments.";
}
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
virtual string description() {
  return "Takes two numbers and subtracts them.  `2 1 - ` produces 1.  1 2 - produces -1.";
}
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("-");
  return result;
}
virtual void execute(MachineState * ms) {
  shared_ptr<Word> w1;
  GET_WORD_ARG(ms, Word, w1);
  shared_ptr<Word> w2;
  GET_WORD_ARG(ms, Word, w2);

  std::shared_ptr<IntegerWord> i1 = std::dynamic_pointer_cast<IntegerWord>(w1);
  std::shared_ptr<IntegerWord> i2 = std::dynamic_pointer_cast<IntegerWord>(w2);
  std::shared_ptr<DoubleWord> d1 = std::dynamic_pointer_cast<DoubleWord>(w1);
  std::shared_ptr<DoubleWord> d2 = std::dynamic_pointer_cast<DoubleWord>(w2);

  if (i1 != NULL && i2 != NULL) {
    ms->pushWord(make_shared<IntegerWord>(w2->to_int() - w1->to_int()));
  } else if ((i1 != NULL && d2 != NULL) || (d1 != NULL && i2 != NULL) || (d1 != NULL && d2 != NULL)) {
    ms->pushWord(make_shared<DoubleWord>(w2->to_double() - w1->to_double()));
  } else {
    CONSOLE_ERROR(ms, "Minus requires numbers.  Got w1: " << w1->repr() << " and w2 " << w2->repr());
    ms->pushWord("pauseStackExecution");   
  }

}
END_WORD
REGISTER_WORD(Minus)

WORD(Equals)
CODE('=') 
virtual string description() {
  return "Takes two words and returns 1 if they are equal (by value) and 0 otherwise.";
}
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("=");
  return result;
}
virtual void execute(MachineState * ms) {
  shared_ptr<Word> p1;
  GET_WORD_ARG(ms, Word, p1);
  shared_ptr<Word> p2;
  GET_WORD_ARG(ms, Word, p2);

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
virtual string description() {
  return "Pops a word; pushes a 0 or a 1 depending on its truth value.";
}
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



WORD(Inc)
virtual string description() {
  return "Adds one to its argument.  1 inc produces 2.";
}
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("1 +");
}
END_WORD
REGISTER_WORD(Inc)


WORD(Ift)
virtual string description() {
  return "Takes two words on the stack, executes the second word if the first word is true.  For example, \" ( \"hello\" print ) 1 ift\" will print hello, while \" ( \"hello\" print ) 0 ift\" will not do it.";
}
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
virtual string description() {
  return "If then else takes two compound words and a condition and does the first one if the condition is true and the second if it is false.   Usage: 0 ( \"condition was true\" print ) ( \"condition was false\" print )   ifte";
}
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

WORD(ClearConsole)
virtual string description() {
  return "Prints a lot of newlines to the console to clear it.";
}
virtual void execute(MachineState * ms) {
  CONSOLE(ms, "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
}
END_WORD
REGISTER_WORD(ClearConsole)


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
virtual string description() {
  return "Moves a word from the call stack to the data stack.";
}
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
virtual string description() {
  return "Moves a word from the data stack to the call stack.";
}
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


WORD(EePoseGetPoseRelativeTo)
virtual void execute(MachineState * ms) {
/* call with "base_pose to_apply EePoseGetPoseRelativeTo" */
  eePose to_apply;
  GET_ARG(ms, EePoseWord, to_apply);

  eePose base_pose;
  GET_ARG(ms, EePoseWord, base_pose);

  ms->pushWord(make_shared<EePoseWord>(base_pose.getPoseRelativeTo(to_apply)));
}
END_WORD
REGISTER_WORD(EePoseGetPoseRelativeTo)

WORD(EePoseApplyRelativePoseTo)
virtual void execute(MachineState * ms) {
/* call with "to_apply base_pose EePoseApplyRelativePoseTo" */
  eePose base_pose;
  GET_ARG(ms, EePoseWord, base_pose);

  eePose to_apply;
  GET_ARG(ms, EePoseWord, to_apply);

  ms->pushWord(make_shared<EePoseWord>(to_apply.applyAsRelativePoseTo(base_pose)));
}
END_WORD
REGISTER_WORD(EePoseApplyRelativePoseTo)

WORD(EePoseRPYOnQ)
virtual void execute(MachineState * ms) {

  double roll, pitch, yaw;
  GET_NUMERIC_ARG(ms, yaw);
  GET_NUMERIC_ARG(ms, pitch);
  GET_NUMERIC_ARG(ms, roll);

  eePose base_pose;
  GET_ARG(ms, EePoseWord, base_pose);

  ms->pushWord(make_shared<EePoseWord>(base_pose.applyRPYTo(roll, pitch, yaw)));
}
END_WORD
REGISTER_WORD(EePoseRPYOnQ)


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

WORD(CalibrateCameraAToB)
virtual void execute(MachineState * ms)
{
  int cameraA = 0;
  int cameraB = 0;
  GET_NUMERIC_ARG(ms, cameraA);
  GET_NUMERIC_ARG(ms, cameraB);

  if ( cameraA >= ms->config.cameras.size() || cameraB >= ms->config.cameras.size() ) {
    cout << "Invalid cameras, exiting." << endl;
  } else {
    cout << "Camera indeces valid, proceeding." << endl;
  }

  // scan once
  // scan again
  // calculate relative pose
  // update pose of cameraA
}
END_WORD
REGISTER_WORD(CalibrateCameraAToB)



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


WORD(ReloadCamera)
virtual void execute(MachineState * ms)
{
  changeCamera(ms, ms->config.focused_camera);
}
END_WORD
REGISTER_WORD(ReloadCamera)


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
virtual string description() {
  return "Export words to a text file for documentation purposes.";
}
virtual void execute(MachineState * ms)
{
  string wordFileName = "ein_words.txt";
  CONSOLE(ms, "Writing words to " << wordFileName);
  std::ofstream wordFile;
  wordFile.open(wordFileName);

  std::vector<std::shared_ptr<Word> > words = register_word(NULL);
  for (int i = 0; i < words.size(); i++) {
    vector<string> names = words[i]->names();
    for (int j = 0; j < names.size(); j++) {
      wordFile << names[j] << " " << words[i]->character_code() << endl;
    }
  }
  wordFile.close();
}
END_WORD
REGISTER_WORD(ExportWords)

WORD(ExportDoc)
virtual string description() {
  return "Export words to an html file which is displayed on the website.";
}
virtual void execute(MachineState * ms)
{
  string wordFileName = "ein_words.html";
  CONSOLE(ms, "Writing words to " << wordFileName);
  std::ofstream wordFile;
  wordFile.open(wordFileName);
  wordFile << "<table><tr><th>Word</th><th>Description</th></tr>" << endl;

  map<string, shared_ptr<Word> > words = ms->wordsInNamespace();
  std::map<std::string, shared_ptr<Word> >::iterator iter;
  for (iter = words.begin(); iter != words.end(); ++iter) {
    vector<string> names = iter->second->names();
    wordFile << "<tr><td>";
    for (int j = 0; j < names.size(); j++) {
      wordFile << xmlEncode(names[j]);
      wordFile << "<br/>";
    }
    wordFile << "</td><td>" << xmlEncode(iter->second->description()) << "</td></tr>" << endl;

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
    //double d = camera->handCameraOffset.py;
    double d = camera->handCameraOffset.py/camera->m_x;
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
    //double d = -camera->handCameraOffset.px;
    double d = -camera->handCameraOffset.px/camera->m_y;
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
  ms->config.lastImageCallbackRequest = rclcpp::Clock{}.now();
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
  ms->config.lastAccelerometerCallbackRequest = rclcpp::Clock{}.now();
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
  ms->config.lastEndpointCallbackRequest = rclcpp::Clock{}.now();
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
  rclcpp::Time time;
  int result = getMostRecentRingImageAndPose(ms, &ringImage, &thisPose, &time, false);
  double distance, angleDistance;
  eePose::distanceXYZAndAngle(ms->config.currentEEPose, thisPose, &distance, &angleDistance);
  if (result != 1) { 
    CONSOLE_ERROR(ms, "Warning:  waitUntilRingBufferImageAtCurrentPosition got an error when accessing the ring buffer.");
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
  shared_ptr<StringWord> outword = std::make_shared<StringWord>("Word " + word->name() + ": " + word->description());
  ms->pushWord(outword);
}
END_WORD
REGISTER_WORD(PushHelp)

WORD(Help)
virtual string description() {
  return "Return help text for a word.  Takes a compound word as an argument with a single word inside.  Usage:  ( word ) help.";
}
virtual void execute(MachineState * ms)
{
  ms->evaluateProgram("car pushHelp print");
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
virtual string description() {
  return "Accumulate entries in a compound word using an operator word.  Usage:  ( 1 1 1 ) ( + ) accumulate -> 3.";
}
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


WORD(Get)
virtual string description() {
  return "Takes a compound word on the stack and an int.  Returns the ith entry of the compound word.  Uses zero based indexing.";
}
virtual void execute(MachineState * ms)
{
  int idx;
  GET_INT_ARG(ms, idx);
  
  shared_ptr<CompoundWord> compoundWord;
  GET_WORD_ARG(ms, CompoundWord, compoundWord);

  if (idx >= compoundWord->size() || idx < 0) {
    CONSOLE_ERROR(ms, "Out of bounds: " << idx << " in word " << compoundWord->repr() << " with size " << compoundWord->size());
    ms->pushWord("pauseStackExecution");
  } else {
    ms->pushData(compoundWord->getWord(compoundWord->size() - idx - 1));
  }
}
END_WORD
REGISTER_WORD(Get)



WORD(Size)
virtual string description() {
  return "Takes a compound word on the stack and an int.  Returns the ith entry of the compound word.  Uses zero based indexing.";
}
virtual void execute(MachineState * ms)
{
  shared_ptr<CompoundWord> compoundWord;
  GET_WORD_ARG(ms, CompoundWord, compoundWord);
  ms->pushData(make_shared<IntegerWord>(compoundWord->size()));
}
END_WORD
REGISTER_WORD(Size)


WORD(Nil)
virtual string description() {
  return "The empty list.  ( ) .";
}
virtual void execute(MachineState * ms)
{
  ms->pushData(ms->nil);
}
END_WORD
REGISTER_WORD(Nil)



WORD(Car)
virtual string description() {
  return "Takes a compound word on the stack and returns the first word in the list.";
}
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("First");
  result.push_back("Head");
  return result;
}
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
virtual string description() {
  return "Takes a compound word on the stack and returns the rest of the word (as a compound word), minus the first word.";
}
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("Rest");
  result.push_back("Tail");
  return result;
}
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
virtual string description() {
  return "Clear the data stack.";
}
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("clearDataStack");
  return result;
}
virtual void execute(MachineState * ms)
{
  ms->clearData();
}
END_WORD
REGISTER_WORD(ClearData)

WORD(CP)
virtual string description() {
  return "Close paren; end a compound word.";
}
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
virtual string description() {
  return "Open paren; begin a compound word.";
}
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
virtual string description() {
  return "While loop.  Usage:  ( 1 ) ( torsoFanOn 1 waitForSeconds torsoFanOff 1 waitForSeconds )  while";
}
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
  ms->pushWord("endStackCollapseNoop");
  ms->pushWord("(");
  ms->pushWord(condition);
}
END_WORD
REGISTER_WORD(While)



WORD(WhileCollapsed)
virtual string description() {
  return "While loop.  Use as in while, but collaspses the stack.  This means that if you don't exit the while, Ein will also never exit the while, and be in an infinite loop, even if you clear the stacks.";
}
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
REGISTER_WORD(WhileCollapsed)


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

WORD(IsGripperMoving)
virtual void execute(MachineState * ms) {
  shared_ptr<IntegerWord> isMoving= make_shared<IntegerWord>(isGripperMoving(ms));
  ms->pushWord(isMoving);
}
END_WORD
REGISTER_WORD(IsGripperMoving)





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
  rclcpp::Time thisNow = rclcpp::Clock{}.now();
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
virtual string description() {
  return "Send a command to the other arm.  It takes a string on the stack and sends it as a program to be executed on the other arm.";
}
virtual void execute(MachineState * ms) {
  string string_in;
  GET_STRING_ARG(ms, string_in);

  std_msgs::msg::String command;
  command.data = string_in;
  ms->config.forthCommandPublisher->publish(command);
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

WORD(FileOpenOutput)
virtual string description() {
  return "Open an output file for writing; takes a file name as an argument.";
}
virtual void execute(MachineState * ms) {
  string fname;
  GET_STRING_ARG(ms, fname);

  shared_ptr<OutputFileWord> word = make_shared<OutputFileWord>();
  word->open(ms, fname);
  ms->pushData(word);
}
END_WORD
REGISTER_WORD(FileOpenOutput)



WORD(FileOpenInput)
virtual string description() {
  return "Open an input file for reading; takes a file name as an argument.";
}
virtual void execute(MachineState * ms) {
  string fname;
  GET_STRING_ARG(ms, fname);
  shared_ptr<InputFileWord> word = make_shared<InputFileWord>();
  word->open(ms, fname);
  ms->pushData(word);
}
END_WORD
REGISTER_WORD(FileOpenInput)


WORD(FileReadLine)
virtual string description() {
  return "Read one line from the file into a string and leave it on the data stack.";
}
virtual void execute(MachineState * ms) {
  shared_ptr<InputFileWord> fileWord;
  GET_WORD_ARG(ms, InputFileWord, fileWord);

  string line = fileWord->readline();

  shared_ptr<StringWord> outword = std::make_shared<StringWord>(line);
  ms->pushData(outword);

}
END_WORD
REGISTER_WORD(FileReadLine)


WORD(FileReadAll)
virtual string description() {
  return "Read the contents of the file into a string and push it on the data stack.";
}
virtual void execute(MachineState * ms) {
  shared_ptr<InputFileWord> fileWord;
  GET_WORD_ARG(ms, InputFileWord, fileWord);

  string line = fileWord->readfile();

  shared_ptr<StringWord> outword = std::make_shared<StringWord>(line);
  ms->pushData(outword);

}
END_WORD
REGISTER_WORD(FileReadAll)


WORD(FileWrite)
virtual string description() {
  return "Write a string to the file.  Takes a file and a word, which is written.  if it is a string, writes it as-is.  Otherwise writes it with repr.";
}
virtual void execute(MachineState * ms) {

  std::shared_ptr<Word> word = ms->popData();
  std::shared_ptr<StringWord> s = std::dynamic_pointer_cast<StringWord>(word);
  string stuff;
  if (s != NULL) {
    stuff = s->value();
  } else if (word != NULL) {
    stuff = word->repr();
  } else {
    stuff = "";
  }

  shared_ptr<OutputFileWord> fileWord;
  GET_WORD_ARG(ms, OutputFileWord, fileWord);

  bool result = fileWord->write(stuff);

}
END_WORD
REGISTER_WORD(FileWrite)

WORD(FileWriteLine)
virtual string description() {
  return "Write a line to the file.  Takes a file and a word, which is written.  if it is a string, writes it as-is.  Otherwise writes it with repr.";
}
virtual void execute(MachineState * ms) {

  string stuff;
  GET_WORD_AS_STRING(ms, stuff);

  shared_ptr<OutputFileWord> fileWord;
  GET_WORD_ARG(ms, OutputFileWord, fileWord);

  bool result = fileWord->write(stuff);
  result = fileWord->write("\n");

}
END_WORD
REGISTER_WORD(FileWriteLine)



WORD(FileClose)
virtual string description() {
  return "Close the file.  If you forget to do this, it will be closed automatically when the word is deallocated.";
}
virtual void execute(MachineState * ms) {
  shared_ptr<FileWord> fileWord;
  GET_WORD_ARG(ms, FileWord, fileWord);
  fileWord->close();
}
END_WORD
REGISTER_WORD(FileClose)



WORD(SaveConfig)
virtual void execute(MachineState * ms) {
  saveConfig(ms, ms->config.config_filename);
}
END_WORD
REGISTER_WORD(SaveConfig)


WORD(LoadConfig)
virtual void execute(MachineState * ms) {
  loadConfig(ms, ms->config.config_filename);
}
END_WORD
REGISTER_WORD(LoadConfig)


WORD(CameraSetTransformMatrix)
virtual void execute(MachineState * ms)
{
  shared_ptr<CompoundWord> argsWord;
  GET_WORD_ARG(ms, CompoundWord, argsWord);

  if (argsWord->size() != 4) {
    CONSOLE_ERROR(ms, "Must pass a four word compound word to set the matrix.");
  }

  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  for (int i = 0; i < argsWord->size(); i++) {
    shared_ptr<Word> w = argsWord->getWord(i);
    camera->transform_matrix[i] = w->to_double();
  }
}
END_WORD
REGISTER_WORD(CameraSetTransformMatrix)

WORD(CameraGetTransformMatrix)
virtual void execute(MachineState * ms)
{
  shared_ptr<CompoundWord> body = make_shared<CompoundWord>();
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  for (int i = 0; i < 4; i++) {
    body->pushWord(make_shared<DoubleWord>(camera->transform_matrix[i]));
  }

  ms->pushData(body);

}
END_WORD
REGISTER_WORD(CameraGetTransformMatrix)




WORD(CameraCreate)
virtual string description() {
  return "Creates a new camera, usage in kinect2.back";
}
virtual void execute(MachineState * ms)
{

  string link;
  GET_STRING_ARG(ms, link);

  string ee_link;
  GET_STRING_ARG(ms, ee_link);


  string topic;
  GET_STRING_ARG(ms, topic);

  string name;
  GET_STRING_ARG(ms, name);

  Camera * camera  = new Camera(ms, name, topic, ee_link, link);

  bool repeat = false;
  for (int i = 0; i < ms->config.cameras.size(); i++) {
    if (ms->config.cameras[i]->name == camera->name) {
      delete ms->config.cameras[i];
      ms->config.cameras[i] = camera;
      repeat = true;
    }
  }
  if (! repeat) {
    ms->config.cameras.push_back(camera);
  }

}
END_WORD
REGISTER_WORD(CameraCreate)



WORD(SetDefaultHandCameraOffset)
virtual string description() {
  return "Sets the hand camera offset to the default value (obtained for Baxter's RGB wrist camera.";
}
virtual void execute(MachineState * ms) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  camera->setDefaultHandCameraOffset();
}
END_WORD
REGISTER_WORD(SetDefaultHandCameraOffset)

WORD(CameraInitializeConfig)
virtual string description() {
  return "Initialize the configuration of the camera (reticles) with reasonable default values based on image size.";
}
virtual void execute(MachineState * ms)
{
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  if (!isSketchyMat(camera->cam_img)) {
    camera->initializeConfig(camera->cam_img.rows, camera->cam_img.cols);
  } else {
    CONSOLE_ERROR(ms, "No camera image!");
  }
}
END_WORD
REGISTER_WORD(CameraInitializeConfig)




CONFIG_GETTER_INT(GradientServoSoftMaxIterations, ms->config.softMaxGradientServoIterations)
CONFIG_SETTER_INT(SetGradientServoSoftMaxIterations, ms->config.softMaxGradientServoIterations)

CONFIG_GETTER_INT(GradientServoHardMaxIterations, ms->config.hardMaxGradientServoIterations)
CONFIG_SETTER_INT(SetGradientServoHardMaxIterations, ms->config.hardMaxGradientServoIterations)

CONFIG_GETTER_INT(MappingServoTimeout, ms->config.mappingServoTimeout)
CONFIG_SETTER_INT(SetMappingServoTimeout, ms->config.mappingServoTimeout)


CONFIG_GETTER_DOUBLE(TwistThresh, ms->config.twistThresh, "")
CONFIG_SETTER_DOUBLE(SetTwistThresh, ms->config.twistThresh)

CONFIG_GETTER_DOUBLE(EffortThresh, ms->config.actual_effort_thresh, "");
CONFIG_SETTER_DOUBLE(SetEffortThresh, ms->config.actual_effort_thresh);


CONFIG_GETTER_STRING(DataDirectory, ms->config.data_directory, "The directory where data is stored.")

CONFIG_GETTER_STRING(RobotType, ms->config.robot_type, "The type of the robot.")

CONFIG_GETTER_STRING(RobotSerial, ms->config.robot_serial, "The robot serial number, used for naming configuration files uniquely.")
CONFIG_GETTER_STRING(RobotSoftwareVersion, ms->config.robot_software_version, "The robot software version.  used for baxter sdk versions.")
CONFIG_GETTER_STRING(EinSoftwareVersion, ms->config.ein_software_version, "Ein's version.")

CONFIG_GETTER_STRING(ScanGroup, ms->config.scan_group, "The scan group, for saving groups of scans organized by objects.")
CONFIG_SETTER_STRING(SetScanGroup, ms->config.scan_group)

CONFIG_GETTER_DOUBLE(IkMapStartHeight, ms->config.ikMapStartHeight, "Start height at which we make the IK map.")

CONFIG_GETTER_DOUBLE(IkMapEndHeight, ms->config.ikMapEndHeight, "End height for making the IP map.")


CONFIG_GETTER_DOUBLE(MapSearchFenceXMin, ms->config.mapSearchFenceXMin, "Xmin for the map in base.")
CONFIG_SETTER_DOUBLE(SetMapSearchFenceXMin, ms->config.mapSearchFenceXMin)

CONFIG_GETTER_DOUBLE(MapSearchFenceYMin, ms->config.mapSearchFenceYMin, "Ymin for the map in base.")
CONFIG_SETTER_DOUBLE(SetMapSearchFenceYMin, ms->config.mapSearchFenceYMin)

CONFIG_GETTER_DOUBLE(MapSearchFenceXMax, ms->config.mapSearchFenceXMax, "Xmax for the map in base.")
CONFIG_SETTER_DOUBLE(SetMapSearchFenceXMax, ms->config.mapSearchFenceXMax)

CONFIG_GETTER_DOUBLE(MapSearchFenceYMax, ms->config.mapSearchFenceYMax, "YMax for the map in base.")
CONFIG_SETTER_DOUBLE(SetMapSearchFenceYMax, ms->config.mapSearchFenceYMax)

CONFIG_GETTER_DOUBLE(GripperMaskThresh, ms->config.cameras[ms->config.focused_camera]->gripperMaskThresh, "Threshold for the gripper mask bmp.")
CONFIG_SETTER_DOUBLE(SetGripperMaskThresh, ms->config.cameras[ms->config.focused_camera]->gripperMaskThresh)


CONFIG_GETTER_DOUBLE(CurrentTableZ, ms->config.currentTableZ, "Current location of the table at z in base.")
CONFIG_SETTER_DOUBLE(SetCurrentTableZ, ms->config.currentTableZ)


CONFIG_GETTER_STRING(BaseTfFrame, ms->config.baseTfFrame, "The name of the global frame you want to use.");
CONFIG_SETTER_STRING(SetBaseTfFrame, ms->config.baseTfFrame);
  

CONFIG_GETTER_STRING(EeTfFrame, ms->config.eeTfFrame, "The name of the end effector frame you want to use.");
CONFIG_SETTER_STRING(SetEeTfFrame, ms->config.eeTfFrame);    


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

CONFIG_GETTER_POSE(TruePose, ms->config.trueEEPoseEEPose);
CONFIG_GETTER_POSE(TrueCameraPose, ms->config.cameras[ms->config.focused_camera]->truePose);

CONFIG_GETTER_STRING(CameraName, ms->config.cameras[ms->config.focused_camera]->name, "The name of the focused camera.");



CONFIG_GETTER_POSE(HandCameraOffset, ms->config.cameras[ms->config.focused_camera]->handCameraOffset);
CONFIG_SETTER_POSE(SetHandCameraOffset, ms->config.cameras[ms->config.focused_camera]->handCameraOffset);

CONFIG_GETTER_POSE(HandEndEffectorOffset, ms->config.handEndEffectorOffset);
CONFIG_SETTER_POSE(SetHandEndEffectorOffset, ms->config.handEndEffectorOffset);

CONFIG_GETTER_INT(FocusedCamera, ms->config.focused_camera);

  CONFIG_GETTER_DOUBLE(CameraMuX, ms->config.cameras[ms->config.focused_camera]->mu_x, "")
  CONFIG_GETTER_DOUBLE(CameraMuY, ms->config.cameras[ms->config.focused_camera]->mu_y, "")
  CONFIG_GETTER_DOUBLE(CameraKappaX, ms->config.cameras[ms->config.focused_camera]->kappa_x, "")
  CONFIG_GETTER_DOUBLE(CameraKappaY, ms->config.cameras[ms->config.focused_camera]->kappa_y, "")

  CONFIG_GETTER_DOUBLE(CameraR00, ms->config.cameras[ms->config.focused_camera]->r_00, "")
  CONFIG_GETTER_DOUBLE(CameraR01, ms->config.cameras[ms->config.focused_camera]->r_01, "")
  CONFIG_GETTER_DOUBLE(CameraR10, ms->config.cameras[ms->config.focused_camera]->r_10, "")
  CONFIG_GETTER_DOUBLE(CameraR11, ms->config.cameras[ms->config.focused_camera]->r_11, "")

CONFIG_SETTER_DOUBLE(SetCameraMuX, ms->config.cameras[ms->config.focused_camera]->mu_x)
CONFIG_SETTER_DOUBLE(SetCameraMuY, ms->config.cameras[ms->config.focused_camera]->mu_y)
CONFIG_SETTER_DOUBLE(SetCameraKappaX, ms->config.cameras[ms->config.focused_camera]->kappa_x)
CONFIG_SETTER_DOUBLE(SetCameraKappaY, ms->config.cameras[ms->config.focused_camera]->kappa_y)

CONFIG_SETTER_DOUBLE(SetCameraR00, ms->config.cameras[ms->config.focused_camera]->r_00)
CONFIG_SETTER_DOUBLE(SetCameraR01, ms->config.cameras[ms->config.focused_camera]->r_01)
CONFIG_SETTER_DOUBLE(SetCameraR10, ms->config.cameras[ms->config.focused_camera]->r_10)
CONFIG_SETTER_DOUBLE(SetCameraR11, ms->config.cameras[ms->config.focused_camera]->r_11)


CONFIG_GETTER_INT(CameraCenterX, ms->config.cameras[ms->config.focused_camera]->centerX)
CONFIG_GETTER_INT(CameraCenterY, ms->config.cameras[ms->config.focused_camera]->centerY)
CONFIG_SETTER_INT(SetCameraCenterX, ms->config.cameras[ms->config.focused_camera]->centerX)
CONFIG_SETTER_INT(SetCameraCenterY, ms->config.cameras[ms->config.focused_camera]->centerY)

CONFIG_GETTER_INT(CameraCropUpperLeftCornerX, ms->config.cameras[ms->config.focused_camera]->cropUpperLeftCorner.px)
CONFIG_GETTER_INT(CameraCropUpperLeftCornerY, ms->config.cameras[ms->config.focused_camera]->cropUpperLeftCorner.py)
CONFIG_SETTER_INT(SetCameraCropUpperLeftCornerX, ms->config.cameras[ms->config.focused_camera]->cropUpperLeftCorner.px)
CONFIG_SETTER_INT(SetCameraCropUpperLeftCornerY, ms->config.cameras[ms->config.focused_camera]->cropUpperLeftCorner.py)

CONFIG_GETTER_INT(SceneCellCountThreshold, ms->config.sceneCellCountThreshold)
CONFIG_SETTER_INT(SceneSetCellCountThreshold, ms->config.sceneCellCountThreshold)


CONFIG_GETTER_INT(SceneDiscrepancySearchDepth, ms->config.sceneDiscrepancySearchDepth)
CONFIG_SETTER_INT(SceneSetDiscrepancySearchDepth, ms->config.sceneDiscrepancySearchDepth)

CONFIG_GETTER_INT(ArmOkButtonState, ms->config.lastArmOkButtonState)
CONFIG_GETTER_INT(ArmShowButtonState, ms->config.lastArmShowButtonState)
CONFIG_GETTER_INT(ArmBackButtonState, ms->config.lastArmBackButtonState)

CONFIG_GETTER_DOUBLE(TorsoFanState, ms->config.torsoFanState, "")

CONFIG_GETTER_INT(CurrentIKMode, ms->config.currentIKMode)

CONFIG_GETTER_DOUBLE(EeRange, ms->config.eeRange, "Range reading.")
CONFIG_GETTER_DOUBLE(EeRangeMaxValue, ms->config.eeRangeMaxValue, "Range max value.")

  CONFIG_GETTER_DOUBLE(MostRecentUntabledZ, ms->config.mostRecentUntabledZ, "")

CONFIG_GETTER_INT(NumCameras, ms->config.cameras.size())

CONFIG_GETTER_INT(CurrentSceneFixationMode, ms->config.currentSceneFixationMode)

CONFIG_GETTER_INT(ZeroGMode, ms->config.zero_g_toggle)
CONFIG_GETTER_INT(PublishCommandsMode, ms->config.publish_commands_mode)

//CONFIG_GETTER_INT(NumIkMapHeights, ms->config.numIkMapHeights)





}
