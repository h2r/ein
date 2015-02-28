
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



namespace ein_words
{
WORD(LoadMarginalHeightMemory)
CODE(1179709)  // capslock + numlock + =
virtual void execute()
{
  loadMarginalHeightMemory();
  drawHeightMemorySample();
}
END_WORD


WORD(DrawMapRegisters)
// numlock + a 
CODE(1048673)
virtual void execute()
{
  drawMapRegisters();
}
END_WORD

WORD(SetGraspMemoriesFromClassGraspMemories)  
// capslock + numlock + i
CODE(1179721)
virtual void execute() {
  copyClassGraspMemoryTriesToGraspMemoryTries();
}
END_WORD


WORD(CopyClassGraspMemoryTriesToGraspMemoryTries)    
// capslock + numlock + i
CODE(1179721)
virtual void execute()
{
  copyClassGraspMemoryTriesToGraspMemoryTries();
}
END_WORD



WORD(SetHeightMemoriesFromClassHeightMemories)
// capslock + numlock + I 
CODE(1245289)
virtual void execute()
{
        cout << "Loading height memories." << endl;
        if ((classHeightMemoryTries[targetClass].rows > 1) && (classHeightMemoryPicks[targetClass].cols == 1)) {
          cout << "targetClass: " << targetClass << " " << classLabels[targetClass] << endl;
          for (int i = 0; i < hmWidth; i++) {
            heightMemoryPicks[i] = classHeightMemoryPicks[targetClass].at<double>(i, 0);
            heightMemoryTries[i] = classHeightMemoryTries[targetClass].at<double>(i, 0);
            cout << "picks: " << heightMemoryPicks[i] << endl;
            cout << "tries: " << heightMemoryTries[i] << endl;
          }
        } else {
	  cout << "Whoops, tried to set height memories but they don't exist for this class:" << targetClass << " " << classLabels[targetClass] << endl;
        }

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

WORD(IncrementTargetClass)
// capslock + pageup
CODE(196437)
virtual void execute()
{
    cout << "targetClass++ " << endl;
    if (numClasses > 0) {
      int newTargetClass = (targetClass + 1) % numClasses;
      changeTargetClass(newTargetClass);
    }
}
END_WORD

WORD(GradientServoTakeClosest)
// capslock + numlock + h
CODE(1179720)
  virtual void execute()
{
    gradientTakeClosest = 1;
    cout << "gradientTakeClosest = " << gradientTakeClosest << endl;
}
END_WORD

WORD(SynchronicServoTakeClosest)
// capslock + C
CODE(196707)
virtual void execute()
{
    synchronicTakeClosest = 1;
    cout << "synchronicTakeClosest = 1" << endl;
    synServoLockFrames = 0;
}
END_WORD

WORD(GraspGear1)
CODE(1048625)
virtual void execute()
{
  int thisGraspGear = 0;
  
  //   set drX
  drX = ggX[thisGraspGear];
  drY = ggY[thisGraspGear];
  
  //   rotate
  setGGRotation(thisGraspGear);
  
  //   set currentGraspGear;
  currentGraspGear = thisGraspGear;
  // numlock + 1
}
END_WORD

WORD(Pause)
CODE('C')
virtual void execute()
{
  if (auto_pilot || (holding_pattern != 0)) {
    pilot_call_stack.push_back('C');
  } else {
    holding_pattern = 0;
    auto_pilot = 0;
    go_on_lock = 0;
  }
}
END_WORD

WORD(ZUp)
CODE('w')
virtual void execute()
{
  currentEEPose.pz += bDelta;
}
END_WORD

WORD(ZDown)
CODE('s')
virtual void execute()
{
    currentEEPose.pz -= bDelta;
}
END_WORD

}

using namespace  ein_words;
std::vector<Word *> create_words() {
  std::vector<Word *> words;
  words.push_back(new ZUp());
  words.push_back(new ZDown());
  words.push_back(new Pause());
  words.push_back(new GraspGear1());
  words.push_back(new SynchronicServoTakeClosest());
  words.push_back(new GradientServoTakeClosest());
  words.push_back(new IncrementTargetClass());
  words.push_back(new PrintState());
  words.push_back(new SetHeightMemoriesFromClassHeightMemories());
  words.push_back(new SetGraspMemoriesFromClassGraspMemories());
  words.push_back(new DrawMapRegisters());
  words.push_back(new LoadMarginalHeightMemory());
  return words;
}

std::map<int, Word *> create_character_code_to_word(std::vector<Word *> words) {
  std::map<int, Word *> character_code_to_word;
  for (unsigned int i = 0; i < words.size(); i++) {
    character_code_to_word[words[i]->character_code()] = words[i];
  }
  return character_code_to_word;
}

std::map<string, Word *> create_name_to_word(std::vector<Word *> words) {
  std::map<string, Word *> name_to_word;
  for (unsigned int i = 0; i < words.size(); i++) {
    name_to_word[words[i]->name()] = words[i];
  }
  return name_to_word;
}



