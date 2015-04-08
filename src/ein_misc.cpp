
WORD(ZeroGToggle)
CODE('z')
virtual void execute() {
  zero_g_toggle = !zero_g_toggle;
}
END_WORD


WORD(ClearStack)
CODE('r') 
virtual void execute() {
  clearStack();
}
END_WORD



WORD(Beep)
CODE(1245308)     // capslock + numlock + |
virtual void execute() {
  cout << "\a"; cout.flush();
}
END_WORD





WORD(AssumeWholeFoodsCounter1)
CODE(196672)  // capslock + @
virtual void execute() {
  currentEEPose = wholeFoodsCounter1;
}
END_WORD

WORD(AssumeWholeFoodsPantry1)
CODE(196643)   // capslock + #
virtual void execute() {
  currentEEPose = wholeFoodsPantry1;
}
END_WORD


WORD(ChangeToCounterTable)
CODE(1179735) // capslock + numlock + w
virtual void execute() {
  currentTableZ = counterTableZ;
}
END_WORD
  
WORD(ChangeToPantryTable)
CODE(1179717)    // capslock + numlock + e
virtual void execute() {
  currentTableZ = pantryTableZ;
}
END_WORD


WORD(ExecuteStack)
CODE('y')
virtual void execute() {
  execute_stack = 1;
}
END_WORD

WORD(PauseStackExecution)
CODE('Y') 
virtual void execute()  {
  cout << "STACK EXECUTION PAUSED, press 'y' to continue." << endl;
  execute_stack = 0;
}
END_WORD


 
WORD(PauseAndReset)
CODE('c') 
virtual void execute() {
  execute_stack = 0;
  lastPtheta = INFINITY;
}
END_WORD




WORD(PrintState)
CODE('u')
virtual void execute() {
  cout << endl;
  cout << "Current EE Position (x,y,z): " << currentEEPose.px << " " << currentEEPose.py << " " << currentEEPose.pz << endl;
  cout << "Current EE Orientation (x,y,z,w): " << currentEEPose.qx << " " << currentEEPose.qy << " " << currentEEPose.qz << " " << currentEEPose.qw << endl;
  cout << "True EE Position (x,y,z): " << trueEEPose.position.x << " " << trueEEPose.position.y << " " << trueEEPose.position.z << endl;
  cout << "True EE Orientation (x,y,z,w): " << trueEEPose.orientation.x << " " << trueEEPose.orientation.y << " " << trueEEPose.orientation.z << " " << trueEEPose.orientation.w << endl;
  cout <<
    "eePose = {.px = " << trueEEPose.position.x << ", .py = " << trueEEPose.position.y << ", .pz = " << trueEEPose.position.z << "," << endl <<
    "		      .ox = 0, .oy = 0, .oz = 0," << endl <<
    "		      .qx = " << trueEEPose.orientation.x << ", .qy = " << trueEEPose.orientation.y << ", .qz = " << trueEEPose.orientation.z << ", .qw = " << trueEEPose.orientation.w << "};" << endl;
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
  
  cout << endl;
  cout << endl;
}
END_WORD

WORD(DecrementTargetClass)
CODE(196438)     // capslock + pagedown
virtual void execute() {
  cout << "targetClass-- " << endl;
  if (numClasses > 0) {
    int newTargetClass = (targetClass - 1 + numClasses) % numClasses;
    changeTargetClass(newTargetClass);
  }
}
END_WORD


WORD(Plus)
CODE('+') 
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("+");
  return result;
}
virtual void execute() {
  Word * p1 = popWord();
  Word * p2 = popWord();
  if (p1 == NULL || p2 == NULL) {
    cout << "Warning, requires two words on the stack." << endl;
    return;
  }

  IntegerWord * w1 = dynamic_cast<IntegerWord *>(p1);
  IntegerWord * w2 = dynamic_cast<IntegerWord *>(p2);

  if (w1 == NULL || w2 == NULL) {
    cout << "Warning, requires two integers on the stack." << endl;
    return;
  }

  IntegerWord * newWord = new IntegerWord(w1->value() + w2->value());
  pushWord(newWord);

}
END_WORD

WORD(Equals)
CODE('=') 
virtual void execute() {
  Word * p1 = popWord();
  Word * p2 = popWord();
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
  IntegerWord * newWord = new IntegerWord(new_value);
  pushWord(newWord);

}
END_WORD



WORD(Ift)
virtual void execute() {
  Word * then = popWord();
  Word * condition = popWord();
  if (then == NULL || condition == NULL) {
    cout << "Warning, requires two words on the stack." << endl;
    return;
  }

  if (condition->as_bool()) {
    pushWord(then);
  }

}
END_WORD


WORD(Start)
virtual void execute() {
}
END_WORD

WORD(Next)
virtual void execute() {
  vector <Word *> words_in_loop;
  
  while (true) {
    Word * word = popWord();
    if (word == NULL) {
      cout << "Warning, next could not find start, aborting." << endl;
      return;
    }
    if (word->name() == "start") {
      break;
    } 
    words_in_loop.push_back(word);
  }
  
  Word * index_to = popWord();
  Word * index_from = popWord();
  if (index_from == NULL || index_to == NULL) {
    cout << "Warning, next requires two words on the stack." << endl;
    return;
  }
  cout << "looping: " << index_from->as_int() << " to " << index_to->as_int() << endl;

  for (int i = index_from->as_int(); i < index_to->as_int(); i++) {
    for (int j = 0; j < words_in_loop.size(); j++) {
      pushWord(words_in_loop[j]);
    }
  }
  
}
END_WORD

WORD(Print)
virtual void execute() {
  Word * word = popWord();
  if (word != NULL) {
    cout << word->as_string() << endl;
  }
}
END_WORD

WORD(Dup)
virtual void execute() {
  Word * word = popWord();
  pushWord(word);
  pushWord(word);
}
END_WORD

WORD(Pop)
virtual void execute() {
  Word * word = popWord();
}
END_WORD

WORD(IncrementTargetClass)
CODE(196437)// capslock + pageup
virtual void execute()
{
  cout << "targetClass++ " << endl;
  if (numClasses > 0) {
    int newTargetClass = (targetClass + 1) % numClasses;
    changeTargetClass(newTargetClass);
  }
}
END_WORD

WORD(ChangeTargetClassToClosestBlueBox)
virtual void execute()  {
  if (pilotClosestBlueBoxNumber == -1) {
    cout << "Not changing because closest bbox is " << pilotClosestBlueBoxNumber << endl;
    return;
  }
  int class_idx = bLabels[pilotClosestBlueBoxNumber];
  cout << "Changing to closest blue blox target, which is class " << classLabels[class_idx] << endl;
  changeTargetClass(class_idx);
}
END_WORD


WORD(Noop)
CODE('C')
virtual void execute()
{

}
END_WORD

WORD(EndStackCollapseNoop)
virtual void execute()
{
  endThisStackCollapse = 1;
}
END_WORD

WORD(PrintWords)
virtual void execute()
{
  string wordFileName = "ein_words.txt";
  cout << "Writing words to " << wordFileName << endl;
  ofstream wordFile;
  wordFile.open(wordFileName);
  std::vector<Word *> words = create_words();
  for (int i = 0; i < words.size(); i++) {
    wordFile << words[i]->name() << " " << words[i]->character_code() << endl;
  }
  wordFile.close();
}
END_WORD


WORD(PixelGlobalTest)
CODE(65609) // I
virtual void execute()
{
  paintEEandReg1OnWrist = !paintEEandReg1OnWrist;
}
END_WORD

WORD(IncMx)
CODE(65361) // left arrow 
virtual void execute()
{
  m_x += .01;
  m_x_h[currentThompsonHeightIdx] = m_x;
  cout << "m_x, m_x_h: " << m_x << endl;
}
END_WORD

WORD(DecMx)
CODE(65363) // right arrow 
virtual void execute()
{
  m_x -= .01;
  m_x_h[currentThompsonHeightIdx] = m_x;
  cout << "m_x, m_x_h: " << m_x << endl;
}
END_WORD

WORD(IncMy)
CODE(65362) // up arrow 
virtual void execute()
{
  m_y += .01;
  m_y_h[currentThompsonHeightIdx] = m_y;
  cout << "m_y, m_y_h: " << m_y << endl;
}
END_WORD

WORD(DecMy)
CODE(65364) // down arrow 
virtual void execute()
{
  m_y -= .01;
  m_y_h[currentThompsonHeightIdx] = m_y;
  cout << "m_y, m_y_h: " << m_y << endl;
}
END_WORD

WORD(EndStackCollapse)
virtual void execute()
{
  endCollapse = 1;
}
END_WORD

WORD(CollapseStack)
virtual void execute()
{
  endCollapse = 0;
}
END_WORD

WORD(ShakeHeadPositive)
virtual void execute()
{
  currentHeadPanCommand.target = 3.1415926/2.0;
  currentHeadPanCommand.speed = 50;
  headPub.publish(currentHeadPanCommand);
}
END_WORD

WORD(ShakeHeadNegative)
virtual void execute()
{
  currentHeadPanCommand.target = -3.1415926/2.0;
  currentHeadPanCommand.speed = 50;
  headPub.publish(currentHeadPanCommand);
}
END_WORD

WORD(CenterHead)
virtual void execute()
{
  currentHeadPanCommand.target = 0;
  currentHeadPanCommand.speed = 50;
  headPub.publish(currentHeadPanCommand);
}
END_WORD

WORD(SilenceSonar)
virtual void execute()
{
  sonarPub.publish(currentSonarCommand);
}
END_WORD

WORD(Nod)
virtual void execute()
{
  currentHeadNodCommand.data = 1;
  nodPub.publish(currentHeadNodCommand);
}
END_WORD

WORD(ResetAuxiliary)
virtual void execute()
{
  pushWord("nod");
  pushWord("centerHead");
  pushWord("shakeHeadPositive");
  pushCopies("noop", 20);
  pushWord("nod");
  pushWord("shakeHeadNegative");
  pushWord("shakeHeadNegative");
  pushCopies("noop", 20);
  pushWord("nod");
  pushWord("shakeHeadPositive");
  pushWord("silenceSonar");
  pushWord("centerHead");
}
END_WORD

WORD(ShutdownAllNonessentialSystems)
virtual void execute()
{
  shouldIRender = 0;
  shouldIDoIK = 0;
  shouldIImageCallback = 0;
  shouldIRangeCallback = 0;
  shouldIMiscCallback = 0;
  cout << "Shutting down all non-essential systems." << endl;
}
END_WORD

WORD(BringUpAllNonessentialSystems)
virtual void execute()
{
  shouldIRender = 1;
  shouldIDoIK = 1;
  shouldIImageCallback = 1;
  shouldIRangeCallback = 1;
  shouldIMiscCallback = 1;
  cout << "Bringing up all non-essential systems." << endl;
}
END_WORD

WORD(WaitUntilImageCallbackReceived)
virtual void execute()
{
  lastImageCallbackRequest = ros::Time::now();
  pushWord("waitUntilImageCallbackReceivedA");
  shouldIImageCallback = 1;
  endThisStackCollapse = 1;
}
END_WORD

WORD(WaitUntilImageCallbackReceivedA)
virtual void execute()
{
  if (lastImageCallbackRequest >= lastImageCallbackReceived) {
    pushWord("waitUntilImageCallbackReceivedA");
    shouldIImageCallback = 1;
    endThisStackCollapse = 1;
  } else {
    endThisStackCollapse = endCollapse;
  }
}
END_WORD

WORD(WriteXMLEnvironment)
virtual void execute()
{
  // For Dipendra
  ofstream ofile;
  string fileName = data_directory + "/" + left_or_right_arm + "_environment.xml";
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

