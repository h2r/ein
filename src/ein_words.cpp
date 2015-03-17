
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
#include "ein_misc.cpp"
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

  {
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
    words.push_back(new WaitUntilAtCurrentPositionB());
    words.push_back(new WaitUntilGripperNotMoving());
    words.push_back(new WaitUntilGripperNotMovingB());
    words.push_back(new WaitUntilGripperNotMovingC());
    words.push_back(new AssumeDeliveryPose());
    words.push_back(new Beep());
    words.push_back(new ZeroGToggle());
    words.push_back(new VisionCycle());
    words.push_back(new AccumulatedDensity());
    words.push_back(new ResetAccumulatedDensity());
    words.push_back(new Density());
    words.push_back(new DensityA());
    words.push_back(new ResetTemporalMap());
    words.push_back(new GoFindBlueBoxes());
    words.push_back(new GoClassifyBlueBoxes());
    words.push_back(new SynchronicServo());
    words.push_back(new GradientServoPrep());
    words.push_back(new GradientServo());
    words.push_back(new GradientServoA());
    words.push_back(new GradientServoIfBlueBoxes());
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
    words.push_back(new SetBoundingBoxModeToMapping());
    words.push_back(new SetBoundingBoxModeToStaticPrior());
    words.push_back(new SetBoundingBoxModeToLearningSampling());
    words.push_back(new SetBoundingBoxModeToLearningAlgorithmC());
    words.push_back(new SetBoundingBoxModeToStaticMarginals());
    words.push_back(new PrepareForAndExecuteGraspFromMemory());
    words.push_back(new PrepareForAndExecuteGraspFromMemoryLearning());
    words.push_back(new PrepareForGraspFromMemory());
    words.push_back(new ExecutePreparedGrasp());
    words.push_back(new LockTargetIfBlueBoxes());
    words.push_back(new RecordTargetLock());

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
    words.push_back(new TryToMoveToTheLastPrePickHeight());
    words.push_back(new AssertYesGrasp());
    words.push_back(new AssertNoGrasp());
    words.push_back(new IfGrasp());
    words.push_back(new IfNoGrasp());
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
    words.push_back(new Pop());

    words.push_back(new PixelGlobalTest());
    words.push_back(new ClearStackIntoMappingPatrol());
    words.push_back(new MappingPatrol());
    words.push_back(new RecordAllBlueBoxes());
    words.push_back(new ClearBlueBoxMemories());
    words.push_back(new FilterBoxMemories());
    words.push_back(new PublishRecognizedObjectArrayFromBlueBoxMemory());
    words.push_back(new ChangeTargetClassToClosestBlueBox());
    words.push_back(new InitializeMap());
    words.push_back(new MapClosestBlueBox());
    words.push_back(new MapEmptySpace());
    words.push_back(new MoveToNextMapPosition());
    words.push_back(new DeliverObject());
    words.push_back(new PlaceObjectInDeliveryZone());


    words.push_back(new IncMx());
    words.push_back(new DecMx());
    words.push_back(new IncMy());
    words.push_back(new DecMy());

    words.push_back(new ToggleShouldIDoIK());
    words.push_back(new ToggleShouldIRender());
    words.push_back(new ToggleDrawClearanceMap());
    words.push_back(new ToggleDrawIKMap());
    words.push_back(new ToggleUseGlow());
    words.push_back(new ToggleUseFade());

    words.push_back(new FillClearanceMap());
    words.push_back(new LoadIkMap());
    words.push_back(new SaveIkMap());
    words.push_back(new FillIkMap());

    words.push_back(new HundredthImpulse());
    words.push_back(new TenthImpulse());
    words.push_back(new QuarterImpulse());
    words.push_back(new HalfImpulse());
    words.push_back(new FullImpulse());

    words.push_back(new CruisingSpeed());
    words.push_back(new ApproachSpeed());
    words.push_back(new DepartureSpeed());

    words.push_back(new CollapseStack());
    words.push_back(new EndStackCollapse());

    words.push_back(new ShakeHeadPositive());
    words.push_back(new ShakeHeadNegative());
    words.push_back(new CenterHead());
    words.push_back(new SilenceSonar());
    words.push_back(new Nod());
    words.push_back(new ResetAuxiliary());

    words.push_back(new ShutdownAllNonessentialSystems());
    words.push_back(new BringUpAllNonessentialSystems());

    words.push_back(new WaitUntilImageCallbackReceived());
    words.push_back(new WaitUntilImageCallbackReceivedA());

    words.push_back(new ScanCentered());
    words.push_back(new RecordAllExamplesFocusedClass());
    words.push_back(new RasterScanningSpeed());

    words.push_back(new SetRangeMapCenterFromCurrentEEPose());
  }

  return words;
}

