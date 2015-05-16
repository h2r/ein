
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




WORD(AssumeWholeFoodsCounter1)
CODE(196672)  // capslock + @
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose = wholeFoodsCounter1;
}
END_WORD
REGISTER_WORD(AssumeWholeFoodsCounter1)

WORD(AssumeWholeFoodsPantry1)
CODE(196643)   // capslock + #
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentEEPose = wholeFoodsPantry1;
}
END_WORD
REGISTER_WORD(AssumeWholeFoodsPantry1)


WORD(ChangeToCounterTable)
CODE(1179735) // capslock + numlock + w
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentTableZ = counterTableZ;
}
END_WORD
REGISTER_WORD(ChangeToCounterTable)
  
WORD(ChangeToPantryTable)
CODE(1179717)    // capslock + numlock + e
virtual void execute(std::shared_ptr<MachineState> ms) {
  currentTableZ = pantryTableZ;
}
END_WORD
REGISTER_WORD(ChangeToPantryTable)


WORD(ExecuteStack)
CODE('y')
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->execute_stack = 1;
}
END_WORD
REGISTER_WORD(ExecuteStack)

WORD(PauseStackExecution)
CODE('Y') 
virtual void execute(std::shared_ptr<MachineState> ms)  {
  cout << "STACK EXECUTION PAUSED, press 'y' to continue." << endl;
  ms->execute_stack = 0;
}
END_WORD
REGISTER_WORD(PauseStackExecution)


 
WORD(PauseAndReset)
CODE('c') 
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->execute_stack = 0;
  lastPtheta = INFINITY;
}
END_WORD
REGISTER_WORD(PauseAndReset)




WORD(PrintState)
CODE('u')
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << endl;
  cout << "Current EE Position (x,y,z): " << currentEEPose.px << " " << currentEEPose.py << " " << currentEEPose.pz << endl;
  cout << "Current EE Orientation (x,y,z,w): " << currentEEPose.qx << " " << currentEEPose.qy << " " << currentEEPose.qz << " " << currentEEPose.qw << endl;
  cout << "True EE Position (x,y,z): " << ms->config.trueEEPose.position.x << " " << ms->config.trueEEPose.position.y << " " << ms->config.trueEEPose.position.z << endl;
  cout << "True EE Orientation (x,y,z,w): " << ms->config.trueEEPose.orientation.x << " " << ms->config.trueEEPose.orientation.y << " " << ms->config.trueEEPose.orientation.z << " " << ms->config.trueEEPose.orientation.w << endl;
  cout <<
    "eePose = {.px = " << ms->config.trueEEPose.position.x << ", .py = " << ms->config.trueEEPose.position.y << ", .pz = " << ms->config.trueEEPose.position.z << "," << endl <<
    "		      .qx = " << ms->config.trueEEPose.orientation.x << ", .qy = " << ms->config.trueEEPose.orientation.y << ", .qz = " << ms->config.trueEEPose.orientation.z << ", .qw = " << ms->config.trueEEPose.orientation.w << "};" << endl;
  cout << "currentThompsonHeightIdx: " << currentThompsonHeightIdx << endl;
  cout << "mostRecentUntabledZ (remember this is inverted but correct): " << mostRecentUntabledZ << endl;
  cout << "currentPickMode: " << pickModeToString(currentPickMode) << endl;
  cout << "currentBoundingBoxMode: " << pickModeToString(currentBoundingBoxMode) << endl;
  cout << "gradientServoTakeClosest: " << gradientTakeClosest << endl;
  cout << "synchronicTakeClosest: " << synchronicTakeClosest << endl;
  cout << "focusedClass: " << focusedClass;
  if (focusedClass != -1) {
    cout << " " << classLabels[focusedClass];
  }
  cout << endl;
  
  cout << "targetClass: " << targetClass;
  if (targetClass != -1) {
    cout << " " << classLabels[targetClass];
  }

  
  double wrenchNorm = sqrt( squareDistanceEEPose(eePoseZero, ms->config.trueEEWrench) );
  cout << "wrenchNorm: " << wrenchNorm << endl;
  
  cout << endl;
  cout << endl;
}
END_WORD
REGISTER_WORD(PrintState)

WORD(DecrementTargetClass)
CODE(196438)     // capslock + pagedown
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "targetClass-- " << endl;
  if (numClasses > 0) {
    int newTargetClass = (targetClass - 1 + numClasses) % numClasses;
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
  std::shared_ptr<Word> p1 = ms->popWord();
  std::shared_ptr<Word> p2 = ms->popWord();
  if (p1 == NULL || p2 == NULL) {
    cout << "Warning, requires two words on the stack." << endl;
    return;
  }

  std::shared_ptr<IntegerWord> w1 = std::dynamic_pointer_cast<IntegerWord>(p1);
  std::shared_ptr<IntegerWord> w2 = std::dynamic_pointer_cast<IntegerWord>(p2);

  if (w1 == NULL || w2 == NULL) {
    cout << "Warning, requires two integers on the stack." << endl;
    return;
  }

  std::shared_ptr<IntegerWord> newWord = std::make_shared<IntegerWord>(w1->value() + w2->value());
  ms->pushWord(newWord);

}
END_WORD
REGISTER_WORD(Plus)

WORD(Equals)
CODE('=') 
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> p1 = ms->popWord();
  std::shared_ptr<Word> p2 = ms->popWord();
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
  ms->pushWord(newWord);

}
END_WORD
REGISTER_WORD(Equals)



WORD(Ift)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> then = ms->popWord();
  std::shared_ptr<Word> condition = ms->popWord();
  if (then == NULL || condition == NULL) {
    cout << "Warning, requires two words on the stack." << endl;
    return;
  }

  if (condition->to_bool()) {
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
  
  std::shared_ptr<Word> index_to = ms->popWord();
  std::shared_ptr<Word> index_from = ms->popWord();
  if (index_from == NULL || index_to == NULL) {
    cout << "Warning, next requires two words on the stack." << endl;
    return;
  }
  cout << "looping: " << index_from->to_int() << " to " << index_to->to_int() << endl;

  for (int i = index_from->to_int(); i < index_to->to_int(); i++) {
    for (int j = 0; j < words_in_loop.size(); j++) {
      ms->pushWord(words_in_loop[j]);
    }
  }
  
}
END_WORD
REGISTER_WORD(Next)

WORD(Print)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> word = ms->popWord();
  if (word != NULL) {
    cout << word->repr() << endl;
  }
}
END_WORD
REGISTER_WORD(Print)

WORD(Dup)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> word = ms->popWord();
  ms->pushWord(word);
  ms->pushWord(word);
}
END_WORD
REGISTER_WORD(Dup)

WORD(Pop)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> word = ms->popWord();
}
END_WORD
REGISTER_WORD(Pop)


WORD(Store)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::shared_ptr<Word> nameword = ms->popWord();
  std::shared_ptr<Word> valueword = ms->popWord();
  string name = nameword->to_string();
  cout << "Storing " << name << " value " << valueword << endl;
  ms->variables[name] = valueword;
}
END_WORD
REGISTER_WORD(Store)


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
  if (numClasses > 0) {
    int newTargetClass = (targetClass + 1) % numClasses;
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
  int class_idx = bLabels[ms->config.pilotClosestBlueBoxNumber];
  cout << "Changing to closest blue blox target, which is class " << classLabels[class_idx] << endl;
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
  endThisStackCollapse = 1;
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
  paintEEandReg1OnWrist = !paintEEandReg1OnWrist;
}
END_WORD
REGISTER_WORD(PixelGlobalTest)

WORD(IncMx)
CODE(65361) // left arrow 
virtual void execute(std::shared_ptr<MachineState> ms)
{
  m_x += .01;
  m_x_h[currentThompsonHeightIdx] = m_x;
  cout << "m_x, m_x_h: " << m_x << endl;
}
END_WORD
REGISTER_WORD(IncMx)

WORD(DecMx)
CODE(65363) // right arrow 
virtual void execute(std::shared_ptr<MachineState> ms)
{
  m_x -= .01;
  m_x_h[currentThompsonHeightIdx] = m_x;
  cout << "m_x, m_x_h: " << m_x << endl;
}
END_WORD
REGISTER_WORD(DecMx)

WORD(IncMy)
CODE(65362) // up arrow 
virtual void execute(std::shared_ptr<MachineState> ms)
{
  m_y += .01;
  m_y_h[currentThompsonHeightIdx] = m_y;
  cout << "m_y, m_y_h: " << m_y << endl;
}
END_WORD
REGISTER_WORD(IncMy)

WORD(DecMy)
CODE(65364) // down arrow 
virtual void execute(std::shared_ptr<MachineState> ms)
{
  m_y -= .01;
  m_y_h[currentThompsonHeightIdx] = m_y;
  cout << "m_y, m_y_h: " << m_y << endl;
}
END_WORD
REGISTER_WORD(DecMy)

WORD(EndStackCollapse)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  endCollapse = 1;
}
END_WORD
REGISTER_WORD(EndStackCollapse)

WORD(CollapseStack)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  endCollapse = 0;
}
END_WORD
REGISTER_WORD(CollapseStack)

WORD(ShakeHeadPositive)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  currentHeadPanCommand.target = 3.1415926/2.0;
  currentHeadPanCommand.speed = 50;
  headPub.publish(currentHeadPanCommand);
}
END_WORD
REGISTER_WORD(ShakeHeadPositive)


WORD(ShakeHeadNegative)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  currentHeadPanCommand.target = -3.1415926/2.0;
  currentHeadPanCommand.speed = 50;
  headPub.publish(currentHeadPanCommand);
}
END_WORD
REGISTER_WORD(ShakeHeadNegative)

WORD(CenterHead)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  currentHeadPanCommand.target = 0;
  currentHeadPanCommand.speed = 50;
  headPub.publish(currentHeadPanCommand);
}
END_WORD
REGISTER_WORD(CenterHead)

WORD(SilenceSonar)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  sonarPub.publish(currentSonarCommand);
}
END_WORD
REGISTER_WORD(SilenceSonar)

WORD(Nod)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  currentHeadNodCommand.data = 1;
  nodPub.publish(currentHeadNodCommand);
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

WORD(ShutdownAllNonessentialSystems)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.shouldIRender = 0;
  ms->config.shouldIDoIK = 0;
  ms->config.shouldIImageCallback = 0;
  ms->config.shouldIRangeCallback = 0;
  ms->config.shouldIMiscCallback = 0;
  cout << "Shutting down all non-essential systems." << endl;
}
END_WORD
REGISTER_WORD(ShutdownAllNonessentialSystems)

WORD(BringUpAllNonessentialSystems)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.shouldIRender = 1;
  ms->config.shouldIDoIK = 1;
  ms->config.shouldIImageCallback = 1;
  ms->config.shouldIRangeCallback = 1;
  ms->config.shouldIMiscCallback = 1;
  cout << "Bringing up all non-essential systems." << endl;
}
END_WORD
REGISTER_WORD(BringUpAllNonessentialSystems)

WORD(WaitUntilImageCallbackReceived)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  lastImageCallbackRequest = ros::Time::now();
  ms->pushWord("waitUntilImageCallbackReceivedA");
  ms->config.shouldIImageCallback = 1;
  endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(WaitUntilImageCallbackReceived)

WORD(WaitUntilImageCallbackReceivedA)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  if (lastImageCallbackRequest >= lastImageCallbackReceived) {
    ms->pushWord("waitUntilImageCallbackReceivedA");
    ms->config.shouldIImageCallback = 1;
    endThisStackCollapse = 1;
  } else {
    endThisStackCollapse = endCollapse;
  }
}
END_WORD
REGISTER_WORD(WaitUntilImageCallbackReceivedA)

WORD(WaitUntilAccelerometerCallbackReceived)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  lastAccelerometerCallbackRequest = ros::Time::now();
  ms->pushWord("waitUntilAccelerometerCallbackReceivedA");
  endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(WaitUntilAccelerometerCallbackReceived)

WORD(WaitUntilAccelerometerCallbackReceivedA)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  if (lastAccelerometerCallbackRequest >= lastAccelerometerCallbackReceived) {
    ms->pushWord("waitUntilAccelerometerCallbackReceivedA");
    endThisStackCollapse = 1;
  } else {
    endThisStackCollapse = endCollapse;
  }
}
END_WORD
REGISTER_WORD(WaitUntilAccelerometerCallbackReceivedA)

WORD(WaitUntilEndpointCallbackReceived)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  lastEndpointCallbackRequest = ros::Time::now();
  ms->pushWord("waitUntilEndpointCallbackReceivedA");
  endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(WaitUntilEndpointCallbackReceived)

WORD(WaitUntilEndpointCallbackReceivedA)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  if (lastEndpointCallbackRequest >= lastEndpointCallbackReceived) {
    ms->pushWord("waitUntilEndpointCallbackReceivedA");
    endThisStackCollapse = 1;
  } else {
    endThisStackCollapse = endCollapse;
  }
}
END_WORD
REGISTER_WORD(WaitUntilEndpointCallbackReceivedA)

WORD(WriteXMLEnvironment)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  // For Dipendra
  ofstream ofile;
  string fileName = data_directory + "/" + ms->config.left_or_right_arm + "_environment.xml";
  cout << "Saving environment to " << fileName << endl;
  ofile.open(fileName, ios::trunc);

  ofile << "<environment>" << endl;
  for (int i = 0; i < blueBoxMemories.size(); i++) {
    BoxMemory box = blueBoxMemories[i];
    if (box.labeledClassIndex >= 0) {
      ofile << "  <object>" << endl;
      ofile << "    <name>" << classLabels[box.labeledClassIndex] << "</name>" << endl;
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





