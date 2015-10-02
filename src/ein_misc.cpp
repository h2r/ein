
#include "ein_words.h"
#include "ein.h"


namespace ein_words {

WORD(PushTime)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushData(std::make_shared<DoubleWord>(ros::Time::now().toSec()));
}
END_WORD
REGISTER_WORD(PushTime)

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


WORD(Plus)
CODE('+') 
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("+");
  return result;
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);
  double v2;
  GET_NUMERIC_ARG(ms, v2);

  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(v1 + v2);
  ms->pushWord(newWord);

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


WORD(Store)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> nameword = ms->popData();
  std::shared_ptr<Word> valueword = ms->popData();
  if (nameword == NULL || valueword == NULL) {
    cout << " Store takes two arguments." << endl;
  }

  string name = nameword->to_string();
  cout << "Storing " << name << " value " << valueword << endl;
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
    ms->pushWord("pauseStackExecution");	\
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

WORD(PrintWords)
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
REGISTER_WORD(PrintWords)


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


WORD(PushRobotSerial)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  shared_ptr<StringWord> outword = std::make_shared<StringWord>(ms->config.robot_serial);
  ms->pushWord(outword);
}
END_WORD
REGISTER_WORD(PushRobotSerial)


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
  cout << "Close parenthesis should never execute." << endl;
  assert(0);
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

  cout << "Open parenthesis executing." << endl;
  ms->pushWord("sP");
  ms->pushWord("1");

  ms->pushData("oP");

  ms->pushWord("printStacks");

}
END_WORD
REGISTER_WORD(OP)

WORD(SP)
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("|");
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

	  cout << "sP found cP!!! scopeLevel " << scopeLevel << " " << endl;
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
		  cout << " open needed -1: " << open_needed << endl;
		  if (open_needed > 0) {
		    cp->pushWord(datum);
		  } else {
		  }
		} else if(0 == datum->name().compare("cP")) {
		  open_needed = open_needed+1;
		  cp->pushWord(datum);
		  cout << " open needed +1: " << open_needed << endl;
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
  ms->pushWord("printStacks");
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
  double number = 0.0;
  GET_NUMERIC_ARG(ms, number);
  if (number != 0) {
  } else {
    ROS_ERROR_STREAM("Failed assert. Pausing." << endl);
    //ms->pushWord("pauseStackExecution");
  }
}
END_WORD
REGISTER_WORD(Assert)


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



}
