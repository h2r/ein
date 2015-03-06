
// Preprocessor macro to automatically create class structure for words
// extracted from the case statement of evil (despair)
// WORD(gName)
// // code here
// CODE(123)

#define WORD(gName) \
class gName: public Word \
{ \
public: \
  virtual string name() { \
    string str = #gName; \
    str[0] = tolower(str[0]); \
    return str; \
  } 

#define CODE(code) \
  virtual int character_code() { \
    return code; \
  } 

#define END_WORD };


void CompoundWord::execute() {
  for (unsigned int i = 0; i < stack.size(); i++) {
    pushWord(stack[i]);
  }
}


namespace ein_words
{

#include "ein_bandit.cpp"
#include "ein_render.cpp"
#include "ein_movement.cpp"
#include "ein_servo.cpp"
#include "ein_vision_cycle.cpp"
#include "ein_scanning.cpp"

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
  cout << "mostRecentUntabledZ: " << mostRecentUntabledZ << endl;
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


WORD(PrintWords)
virtual void execute()
{
  ofstream wordFile;
  wordFile.open("ein_words.txt");
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
  cout << "m_x: " << m_x << endl;
}
END_WORD

WORD(DecMx)
CODE(65363) // right arrow 
virtual void execute()
{
  m_x -= .01;
  cout << "m_x: " << m_x << endl;
}
END_WORD

WORD(IncMy)
CODE(65362) // up arrow 
virtual void execute()
{
  m_y += .01;
  cout << "m_y: " << m_y << endl;
}
END_WORD

WORD(DecMy)
CODE(65364) // down arrow 
virtual void execute()
{
  m_y -= .01;
  cout << "m_y: " << m_y << endl;
}
END_WORD


}

using namespace  ein_words;

std::map<int, Word *> create_character_code_to_word(std::vector<Word *> words) {
  std::map<int, Word *> character_code_to_word;
  for (unsigned int i = 0; i < words.size(); i++) {
    if (words[i]->character_code() != -1) {
      if (character_code_to_word.count(words[i]->character_code()) > 0) {
        cout << "Two words with the same code." << endl;
        cout << "Word 1: " << character_code_to_word[words[i]->character_code()]->name() << endl;
        cout << "Word 2: " << words[i]->name() << endl;
        cout << "Code: " << words[i]->character_code() << endl;
        assert(0);
      } else {
        character_code_to_word[words[i]->character_code()] = words[i];
      }
    }
  }
  return character_code_to_word;
}

std::map<string, Word *> create_name_to_word(std::vector<Word *> words) {
  std::map<string, Word *> name_to_word;
  for (unsigned int i = 0; i < words.size(); i++) {
    vector<string> names = words[i]->names();
    for (unsigned int j = 0; j < names.size(); j++) {
      name_to_word[names[j]] = words[i];      
    }
  }
  return name_to_word;
}




std::vector<Word *> create_words() {
  std::vector<Word *> words;
  words.push_back(new XUp());
  words.push_back(new XDown());
  words.push_back(new YUp());
  words.push_back(new YDown());
  words.push_back(new ZUp());
  words.push_back(new ZDown());
  words.push_back(new Noop());
  words.push_back(new SynchronicServoTakeClosest());
  words.push_back(new GradientServoTakeClosest());
  words.push_back(new IncrementTargetClass());
  words.push_back(new PrintState());
  words.push_back(new SetHeightMemoriesFromClassHeightMemories());
  words.push_back(new SetGraspMemoriesFromClassGraspMemories());
  words.push_back(new DrawMapRegisters());
  words.push_back(new ScanObject());
  words.push_back(new ExecuteStack());
  words.push_back(new PauseStackExecution());
  words.push_back(new PauseAndReset());
  words.push_back(new ClearStack());
  words.push_back(new OpenGripper());
  words.push_back(new CloseGripper());
  words.push_back(new ChangeToCounterTable());
  words.push_back(new ChangeToPantryTable());
  words.push_back(new AssumeWholeFoodsPantry1());
  words.push_back(new AssumeWholeFoodsCounter1());
  words.push_back(new SetMovementSpeedNowThatsFast());
  words.push_back(new SetMovementSpeedMoveEvenFaster());
  words.push_back(new SetMovementSpeedMoveFaster());
  words.push_back(new SetMovementSpeedMoveFast());
  words.push_back(new SetMovementSpeedMoveMedium());
  words.push_back(new SetMovementSpeedMoveSlow());
  words.push_back(new SetMovementSpeedMoveVerySlow());
  words.push_back(new ChangeToHeight0());
  words.push_back(new ChangeToHeight1());
  words.push_back(new ChangeToHeight2());
  words.push_back(new ChangeToHeight3());
  words.push_back(new WaitUntilAtCurrentPosition());
  words.push_back(new Beep());
  words.push_back(new VisionCycle());
  words.push_back(new Density());
  words.push_back(new ResetTemporalMap());
  words.push_back(new GoFindBlueBoxes());
  words.push_back(new GoClassifyBlueBoxes());
  words.push_back(new SynchronicServo());
  words.push_back(new GradientServo());
  words.push_back(new TwoDPatrolStart());
  words.push_back(new TwoDPatrolContinue());
  words.push_back(new SynchronicServoDoNotTakeClosest());
  words.push_back(new InitializeAndFocusOnNewClass());
  words.push_back(new ResetAerialGradientTemporalFrameAverage());
  words.push_back(new SaveAerialGradientMap());
  words.push_back(new NeutralScan());
  words.push_back(new InitDepthScan());
  words.push_back(new TurnOnRecordRangeMap());
  words.push_back(new PrepareForSearch());
  words.push_back(new FullRender());
  words.push_back(new SampleHeight());
  words.push_back(new DownsampleIrScan());
  words.push_back(new PaintReticles());
  words.push_back(new SelectBestAvailableGrasp());
  words.push_back(new SelectMaxTargetNotCumulative());
  words.push_back(new SelectMaxTargetCumulative());
  words.push_back(new ApplyGraspFilter());
  words.push_back(new Blur());
  words.push_back(new ShiftIntoGraspGear1());
  words.push_back(new ShiftIntoGraspGear2());
  words.push_back(new ShiftIntoGraspGear3());
  words.push_back(new ShiftIntoGraspGear4());
  words.push_back(new TurnOffScanning());
  words.push_back(new OXDown());
  words.push_back(new OXUp());
  words.push_back(new OYDown());
  words.push_back(new OYUp());
  words.push_back(new OZDown());
  words.push_back(new OZUp());
  
  words.push_back(new SaveRegister1());
  words.push_back(new SaveRegister2());
  words.push_back(new SaveRegister3());
  words.push_back(new SaveRegister4());

  words.push_back(new MoveToRegister1());
  words.push_back(new MoveToRegister2());
  words.push_back(new MoveToRegister3());
  words.push_back(new MoveToRegister4());
  words.push_back(new MoveToRegister5());
  words.push_back(new MoveToRegister6());
  
  words.push_back(new PrepareToApplyGraspFilterFor1());
  words.push_back(new PrepareToApplyGraspFilterFor2());
  words.push_back(new PrepareToApplyGraspFilterFor3());
  words.push_back(new PrepareToApplyGraspFilterFor4());
  words.push_back(new SetTargetReticleToTheMaxMappedPosition());
  words.push_back(new ShiftGraspGear()); 
  words.push_back(new SaveCurrentClassDepthAndGraspMaps());
  words.push_back(new UniformlySampleHeight());
  words.push_back(new PhotoSpin());
  words.push_back(new RgbScan());
  words.push_back(new IncrementGraspGear());
  words.push_back(new SetRandomOrientationForPhotospin());
  words.push_back(new RecordExampleAsFocusedClass());
  words.push_back(new VisionCycleNoClassify());

  words.push_back(new SetPickModeToStaticPrior());
  words.push_back(new SetPickModeToLearningSampling());
  words.push_back(new SetPickModeToLearningAlgorithmC());
  words.push_back(new SetPickModeToStaticMarginals());
  words.push_back(new SetBoundingBoxModeToStaticPrior());
  words.push_back(new SetBoundingBoxModeToLearningSampling());
  words.push_back(new SetBoundingBoxModeToLearningAlgorithmC());
  words.push_back(new SetBoundingBoxModeToStaticMarginals());
  words.push_back(new PrepareForAndExecuteGraspFromMemory());

  words.push_back(new CountGrasp());
  words.push_back(new CheckGrasp());
  words.push_back(new CheckAndCountGrasp());
  words.push_back(new CalibrateGripper());
  words.push_back(new SetGripperThresh());
  words.push_back(new LoadTargetClassRangeMapIntoRegister1());

  words.push_back(new AssumeWinningGgAndXyInLocalPose());
  words.push_back(new MoveToTargetZAndGrasp());
  words.push_back(new ShakeItUpAndDown());
  words.push_back(new TryToMoveToTheLastPickHeight());
  words.push_back(new AssertYesGrasp());
  words.push_back(new AssertNoGrasp());
  words.push_back(new ShakeItOff1());
  words.push_back(new FindBestOfFourGraspsUsingMemory());
  words.push_back(new LoadSampledGraspMemory());
  words.push_back(new LoadMarginalGraspMemory());
  words.push_back(new LoadPriorGraspMemoryAnalytic());
  words.push_back(new LoadPriorGraspMemoryUniform());
  words.push_back(new LoadSampledHeightMemory());
  words.push_back(new LoadMarginalHeightMemory());
  words.push_back(new LoadPriorHeightMemoryAnalytic());
  words.push_back(new LoadPriorHeightMemoryUniform());
  words.push_back(new PerturbPosition());
  words.push_back(new TrainModels());
  words.push_back(new SetTargetClassToLastLabelLearned());
  words.push_back(new SetLastLabelLearned());

  words.push_back(new BeginHeightLearning());
  words.push_back(new ContinueHeightLearning());
  words.push_back(new RecordHeightLearnTrial());
  words.push_back(new SetRandomPositionAndOrientationForHeightLearning());
  words.push_back(new SaveLearnedModels());
  words.push_back(new DecrementTargetClass());
  words.push_back(new PrintWords());

  words.push_back(new Plus());
  words.push_back(new Equals());
  words.push_back(new Ift());
  words.push_back(new Start());
  words.push_back(new Next());
  words.push_back(new Print());
  words.push_back(new Dup());

  words.push_back(new PixelGlobalTest());
  words.push_back(new MappingPatrol());
  words.push_back(new RecordAllBlueBoxes());
  words.push_back(new ClearBlueBoxMemories());
  words.push_back(new PublishRecognizedObjectArrayFromBlueBoxMemory());
  words.push_back(new ChangeTargetClassToClosestBlueBox());
  words.push_back(new InitializeMap());

  words.push_back(new IncMx());
  words.push_back(new DecMx());
  words.push_back(new IncMy());
  words.push_back(new DecMy());



  return words;
}

