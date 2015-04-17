
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

std::map<int, std::shared_ptr<Word> > create_character_code_to_word(std::vector<std::shared_ptr<Word> > words) {
  std::map<int, std::shared_ptr<Word> > character_code_to_word;
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

std::map<string, std::shared_ptr<Word> > create_name_to_word(std::vector<std::shared_ptr<Word> > words) {
  std::map<string, std::shared_ptr<Word> > name_to_word;
  for (unsigned int i = 0; i < words.size(); i++) {
    vector<string> names = words[i]->names();
    for (unsigned int j = 0; j < names.size(); j++) {
      name_to_word[names[j]] = words[i];      
    }
  }
  return name_to_word;
}



std::vector<std::shared_ptr<Word> > create_words() {
  std::vector<std::shared_ptr<Word> > words;

  {
    words.push_back(std::make_shared<XUp>());
    words.push_back(std::make_shared<XDown>());
    words.push_back(std::make_shared<YUp>());
    words.push_back(std::make_shared<YDown>());
    words.push_back(std::make_shared<ZUp>());
    words.push_back(std::make_shared<ZDown>());
    words.push_back(std::make_shared<Noop>());
    words.push_back(std::make_shared<EndStackCollapseNoop>());
    words.push_back(std::make_shared<SynchronicServoTakeClosest>());
    words.push_back(std::make_shared<GradientServoTakeClosest>());
    words.push_back(std::make_shared<IncrementTargetClass>());
    words.push_back(std::make_shared<PrintState>());
    words.push_back(std::make_shared<SetHeightMemoriesFromClassHeightMemories>());
    words.push_back(std::make_shared<SetGraspMemoriesFromClassGraspMemories>());
    words.push_back(std::make_shared<DrawMapRegisters>());
    words.push_back(std::make_shared<ScanObject>());
    words.push_back(std::make_shared<ExecuteStack>());
    words.push_back(std::make_shared<PauseStackExecution>());
    words.push_back(std::make_shared<PauseAndReset>());
    words.push_back(std::make_shared<ClearStack>());
    words.push_back(std::make_shared<ClearStackAcceptFetchCommands>());
    words.push_back(std::make_shared<OpenGripper>());
    words.push_back(std::make_shared<CloseGripper>());
    words.push_back(std::make_shared<ChangeToCounterTable>());
    words.push_back(std::make_shared<ChangeToPantryTable>());
    words.push_back(std::make_shared<AssumeWholeFoodsPantry1>());
    words.push_back(std::make_shared<AssumeWholeFoodsCounter1>());
    words.push_back(std::make_shared<SetMovementSpeedNowThatsFast>());
    words.push_back(std::make_shared<SetMovementSpeedMoveEvenFaster>());
    words.push_back(std::make_shared<SetMovementSpeedMoveFaster>());
    words.push_back(std::make_shared<SetMovementSpeedMoveFast>());
    words.push_back(std::make_shared<SetMovementSpeedMoveMedium>());
    words.push_back(std::make_shared<SetMovementSpeedMoveSlow>());
    words.push_back(std::make_shared<SetMovementSpeedMoveVerySlow>());
    words.push_back(std::make_shared<ChangeToHeight0>());
    words.push_back(std::make_shared<ChangeToHeight1>());
    words.push_back(std::make_shared<ChangeToHeight2>());
    words.push_back(std::make_shared<ChangeToHeight3>());
    words.push_back(std::make_shared<WaitUntilAtCurrentPosition>());
    words.push_back(std::make_shared<WaitUntilAtCurrentPositionB>());
    words.push_back(std::make_shared<WaitUntilGripperNotMoving>());
    words.push_back(std::make_shared<WaitUntilGripperNotMovingB>());
    words.push_back(std::make_shared<WaitUntilGripperNotMovingC>());
    words.push_back(std::make_shared<AssumeDeliveryPose>());
    words.push_back(std::make_shared<Beep>());
    words.push_back(std::make_shared<ZeroGToggle>());
    words.push_back(std::make_shared<VisionCycle>());
    words.push_back(std::make_shared<AccumulatedDensity>());
    words.push_back(std::make_shared<ResetAccumulatedDensity>());
    words.push_back(std::make_shared<Density>());
    words.push_back(std::make_shared<DensityA>());
    words.push_back(std::make_shared<ResetTemporalMap>());
    words.push_back(std::make_shared<GoFindBlueBoxes>());
    words.push_back(std::make_shared<GoClassifyBlueBoxes>());
    words.push_back(std::make_shared<SynchronicServo>());
    words.push_back(std::make_shared<GradientServoPrep>());
    words.push_back(std::make_shared<GradientServo>());
    words.push_back(std::make_shared<GradientServoA>());
    words.push_back(std::make_shared<GradientServoIfBlueBoxes>());
    words.push_back(std::make_shared<TwoDPatrolStart>());
    words.push_back(std::make_shared<TwoDPatrolContinue>());
    words.push_back(std::make_shared<SynchronicServoDoNotTakeClosest>());
    words.push_back(std::make_shared<InitializeAndFocusOnNewClass>());
    words.push_back(std::make_shared<ResetAerialGradientTemporalFrameAverage>());
    words.push_back(std::make_shared<SaveAerialGradientMap>());
    words.push_back(std::make_shared<NeutralScan>());
    words.push_back(std::make_shared<InitDepthScan>());
    words.push_back(std::make_shared<TurnOnRecordRangeMap>());
    words.push_back(std::make_shared<PrepareForSearch>());
    words.push_back(std::make_shared<FullRender>());
    words.push_back(std::make_shared<SampleHeight>());
    words.push_back(std::make_shared<DownsampleIrScan>());
    words.push_back(std::make_shared<PaintReticles>());
    words.push_back(std::make_shared<SelectBestAvailableGrasp>());
    words.push_back(std::make_shared<SelectMaxTargetNotCumulative>());
    words.push_back(std::make_shared<SelectMaxTargetCumulative>());
    words.push_back(std::make_shared<ApplyGraspFilter>());
    words.push_back(std::make_shared<Blur>());
    words.push_back(std::make_shared<ShiftIntoGraspGear1>());
    words.push_back(std::make_shared<ShiftIntoGraspGear2>());
    words.push_back(std::make_shared<ShiftIntoGraspGear3>());
    words.push_back(std::make_shared<ShiftIntoGraspGear4>());
    words.push_back(std::make_shared<TurnOffScanning>());
    words.push_back(std::make_shared<OXDown>());
    words.push_back(std::make_shared<OXUp>());
    words.push_back(std::make_shared<OYDown>());
    words.push_back(std::make_shared<OYUp>());
    words.push_back(std::make_shared<OZDown>());
    words.push_back(std::make_shared<OZUp>());
    
    words.push_back(std::make_shared<SaveRegister1>());
    words.push_back(std::make_shared<SaveRegister2>());
    words.push_back(std::make_shared<SaveRegister3>());
    words.push_back(std::make_shared<SaveRegister4>());

    words.push_back(std::make_shared<MoveToRegister1>());
    words.push_back(std::make_shared<MoveToRegister2>());
    words.push_back(std::make_shared<MoveToRegister3>());
    words.push_back(std::make_shared<MoveToRegister4>());
    words.push_back(std::make_shared<MoveToRegister5>());
    words.push_back(std::make_shared<MoveToRegister6>());
    
    words.push_back(std::make_shared<PrepareToApplyGraspFilterFor1>());
    words.push_back(std::make_shared<PrepareToApplyGraspFilterFor2>());
    words.push_back(std::make_shared<PrepareToApplyGraspFilterFor3>());
    words.push_back(std::make_shared<PrepareToApplyGraspFilterFor4>());
    words.push_back(std::make_shared<SetTargetReticleToTheMaxMappedPosition>());
    words.push_back(std::make_shared<ShiftGraspGear>()); 
    words.push_back(std::make_shared<SaveCurrentClassDepthAndGraspMaps>());
    words.push_back(std::make_shared<UniformlySampleHeight>());
    words.push_back(std::make_shared<PhotoSpin>());
    words.push_back(std::make_shared<RgbScan>());
    words.push_back(std::make_shared<IncrementGraspGear>());
    words.push_back(std::make_shared<SetRandomOrientationForPhotospin>());
    words.push_back(std::make_shared<RecordExampleAsFocusedClass>());
    words.push_back(std::make_shared<VisionCycleNoClassify>());

    words.push_back(std::make_shared<SetPickModeToStaticPrior>());
    words.push_back(std::make_shared<SetPickModeToLearningSampling>());
    words.push_back(std::make_shared<SetPickModeToLearningAlgorithmC>());
    words.push_back(std::make_shared<SetPickModeToStaticMarginals>());
    words.push_back(std::make_shared<SetBoundingBoxModeToMapping>());
    words.push_back(std::make_shared<SetBoundingBoxModeToStaticPrior>());
    words.push_back(std::make_shared<SetBoundingBoxModeToLearningSampling>());
    words.push_back(std::make_shared<SetBoundingBoxModeToLearningAlgorithmC>());
    words.push_back(std::make_shared<SetBoundingBoxModeToStaticMarginals>());
    words.push_back(std::make_shared<PrepareForAndExecuteGraspFromMemory>());
    words.push_back(std::make_shared<PrepareForAndExecuteGraspFromMemoryLearning>());
    words.push_back(std::make_shared<PrepareForGraspFromMemory>());
    words.push_back(std::make_shared<ExecutePreparedGrasp>());
    words.push_back(std::make_shared<LockTargetIfBlueBoxes>());
    words.push_back(std::make_shared<RecordTargetLock>());

    words.push_back(std::make_shared<CountGrasp>());
    words.push_back(std::make_shared<CheckGrasp>());
    words.push_back(std::make_shared<CheckAndCountGrasp>());
    words.push_back(std::make_shared<CalibrateGripper>());
    words.push_back(std::make_shared<SetGripperThresh>());
    words.push_back(std::make_shared<LoadTargetClassRangeMapIntoRegister1>());

    words.push_back(std::make_shared<AssumeWinningGgAndXyInLocalPose>());
    words.push_back(std::make_shared<MoveToTargetZAndGrasp>());
    words.push_back(std::make_shared<ShakeItUpAndDown>());
    words.push_back(std::make_shared<TryToMoveToTheLastPickHeight>());
    words.push_back(std::make_shared<TryToMoveToTheLastPrePickHeight>());
    words.push_back(std::make_shared<AssertYesGrasp>());
    words.push_back(std::make_shared<AssertNoGrasp>());
    words.push_back(std::make_shared<IfGrasp>());
    words.push_back(std::make_shared<IfNoGrasp>());
    words.push_back(std::make_shared<ShakeItOff1>());
    words.push_back(std::make_shared<FindBestOfFourGraspsUsingMemory>());
    words.push_back(std::make_shared<LoadSampledGraspMemory>());
    words.push_back(std::make_shared<LoadMarginalGraspMemory>());
    words.push_back(std::make_shared<LoadPriorGraspMemoryAnalytic>());
    words.push_back(std::make_shared<LoadPriorGraspMemoryUniform>());
    words.push_back(std::make_shared<LoadSampledHeightMemory>());
    words.push_back(std::make_shared<LoadMarginalHeightMemory>());
    words.push_back(std::make_shared<LoadPriorHeightMemoryAnalytic>());
    words.push_back(std::make_shared<LoadPriorHeightMemoryUniform>());
    words.push_back(std::make_shared<PerturbPosition>());
    words.push_back(std::make_shared<TrainModels>());
    words.push_back(std::make_shared<SetTargetClassToLastLabelLearned>());
    words.push_back(std::make_shared<SetLastLabelLearned>());

    words.push_back(std::make_shared<BeginHeightLearning>());
    words.push_back(std::make_shared<ContinueHeightLearning>());
    words.push_back(std::make_shared<RecordHeightLearnTrial>());
    words.push_back(std::make_shared<SetRandomPositionAndOrientationForHeightLearning>());
    words.push_back(std::make_shared<SaveLearnedModels>());
    words.push_back(std::make_shared<DecrementTargetClass>());
    words.push_back(std::make_shared<PrintWords>());

    words.push_back(std::make_shared<Plus>());
    words.push_back(std::make_shared<Equals>());
    words.push_back(std::make_shared<Ift>());
    words.push_back(std::make_shared<Start>());
    words.push_back(std::make_shared<Next>());
    words.push_back(std::make_shared<Print>());
    words.push_back(std::make_shared<Dup>());
    words.push_back(std::make_shared<Pop>());

    words.push_back(std::make_shared<PixelGlobalTest>());
    words.push_back(std::make_shared<ClearStackIntoMappingPatrol>());
    words.push_back(std::make_shared<MappingPatrol>());
    words.push_back(std::make_shared<RecordAllBlueBoxes>());
    words.push_back(std::make_shared<ClearBlueBoxMemories>());
    words.push_back(std::make_shared<FilterBoxMemories>());
    words.push_back(std::make_shared<PublishRecognizedObjectArrayFromBlueBoxMemory>());
    words.push_back(std::make_shared<ChangeTargetClassToClosestBlueBox>());
    words.push_back(std::make_shared<InitializeMap>());
    words.push_back(std::make_shared<MapClosestBlueBox>());
    words.push_back(std::make_shared<MapEmptySpace>());
    words.push_back(std::make_shared<MoveToNextMapPosition>());
    words.push_back(std::make_shared<DeliverObject>());
    words.push_back(std::make_shared<PlaceObjectInDeliveryZone>());


    words.push_back(std::make_shared<IncMx>());
    words.push_back(std::make_shared<DecMx>());
    words.push_back(std::make_shared<IncMy>());
    words.push_back(std::make_shared<DecMy>());

    words.push_back(std::make_shared<ToggleShouldIDoIK>());
    words.push_back(std::make_shared<ToggleShouldIRender>());
    words.push_back(std::make_shared<ToggleDrawClearanceMap>());
    words.push_back(std::make_shared<ToggleDrawIKMap>());
    words.push_back(std::make_shared<ToggleUseGlow>());
    words.push_back(std::make_shared<ToggleUseFade>());

    words.push_back(std::make_shared<FillClearanceMap>());
    words.push_back(std::make_shared<LoadIkMap>());
    words.push_back(std::make_shared<SaveIkMap>());
    words.push_back(std::make_shared<FillIkMap>());

    words.push_back(std::make_shared<HundredthImpulse>());
    words.push_back(std::make_shared<TenthImpulse>());
    words.push_back(std::make_shared<QuarterImpulse>());
    words.push_back(std::make_shared<HalfImpulse>());
    words.push_back(std::make_shared<FullImpulse>());

    words.push_back(std::make_shared<CruisingSpeed>());
    words.push_back(std::make_shared<ApproachSpeed>());
    words.push_back(std::make_shared<DepartureSpeed>());

    words.push_back(std::make_shared<CollapseStack>());
    words.push_back(std::make_shared<EndStackCollapse>());

    words.push_back(std::make_shared<ShakeHeadPositive>());
    words.push_back(std::make_shared<ShakeHeadNegative>());
    words.push_back(std::make_shared<CenterHead>());
    words.push_back(std::make_shared<SilenceSonar>());
    words.push_back(std::make_shared<Nod>());
    words.push_back(std::make_shared<ResetAuxiliary>());

    words.push_back(std::make_shared<ShutdownAllNonessentialSystems>());
    words.push_back(std::make_shared<BringUpAllNonessentialSystems>());

    words.push_back(std::make_shared<WaitUntilImageCallbackReceived>());
    words.push_back(std::make_shared<WaitUntilImageCallbackReceivedA>());

    words.push_back(std::make_shared<ScanCentered>());
    words.push_back(std::make_shared<RecordAllExamplesFocusedClass>());
    words.push_back(std::make_shared<RasterScanningSpeed>());

    words.push_back(std::make_shared<SetRangeMapCenterFromCurrentEEPose>());

    words.push_back(std::make_shared<Hover>());
    words.push_back(std::make_shared<HoverA>());

    words.push_back(std::make_shared<SpawnTargetClassAtEndEffector>());
    words.push_back(std::make_shared<DestroyObjectInEndEffector>());
    words.push_back(std::make_shared<PickObjectUnderEndEffector>());
    words.push_back(std::make_shared<PlaceObjectInEndEffector>());
    words.push_back(std::make_shared<WriteXMLEnvironment>());

    words.push_back(std::make_shared<SetCurrentCornellTableToZero>());
    words.push_back(std::make_shared<IncrementCurrentCornellTable>());
    words.push_back(std::make_shared<DecrementCurrentCornellTable>());
    words.push_back(std::make_shared<MoveToCurrentCornellTable>());

    words.push_back(std::make_shared<GuiShowAll>());
    words.push_back(std::make_shared<GuiHideAll>());
    words.push_back(std::make_shared<GuiCustom1>());

    words.push_back(std::make_shared<SpawnTargetMasterSpriteAtEndEffector>());
    words.push_back(std::make_shared<DestroyTargetInstanceSprite>());
    words.push_back(std::make_shared<IncrementTargetInstanceSprite>());
    words.push_back(std::make_shared<DecrementTargetInstanceSprite>());
    words.push_back(std::make_shared<IncrementTargetMasterSprite>());
    words.push_back(std::make_shared<DecrementTargetMasterSprite>());

    words.push_back(std::make_shared<SetTable>());
    words.push_back(std::make_shared<SetTableA>());

    words.push_back(std::make_shared<ComeToStop>());
    words.push_back(std::make_shared<ComeToStopA>());
    words.push_back(std::make_shared<ComeToHover>());
    words.push_back(std::make_shared<ComeToHoverA>());
    words.push_back(std::make_shared<WaitForTugThenOpenGripper>());
    words.push_back(std::make_shared<WaitForTugThenOpenGripperA>());

    words.push_back(std::make_shared<Idler>());
  }

  return words;
}

