#include "ein_words.h"
#include "ein.h"
#include "camera.h"
#include "qtgui/streamviewerwindow.h"

int loadStreamImage(MachineState * ms, streamImage * tsi) {
  if (tsi == NULL) {
    return -1;
  }
  if (tsi->image.data == NULL) {
    tsi->image = imread(tsi->filename);
    if (tsi->image.data == NULL) {
      CONSOLE_ERROR(ms, " Failed to load " << tsi->filename);
      tsi->loaded = 0;
      return -1;
    } else {
      tsi->loaded = 1;
      return 0;
    }
  }
  return -1;
}


namespace ein_words {


WORD(RestoreIkShare)
virtual void execute(MachineState * ms)
{
  ms->config.ikShare = 1.0;
}
END_WORD
REGISTER_WORD(RestoreIkShare)

WORD(ShutdownToSensorsAndMovement)
virtual void execute(MachineState * ms)
{
  ms->config.shouldIRender = 0;
  ms->config.shouldIDoIK = 1;
  ms->config.ikShare = 1.0;//0.1;
  ms->config.shouldIImageCallback = 0;
  ms->config.shouldIRangeCallback = 0;
  ms->config.shouldIMiscCallback = 0;
  CONSOLE(ms, "Shutting down to sensors and movement.");
}
END_WORD
REGISTER_WORD(ShutdownToSensorsAndMovement)

WORD(ShutdownAllNonessentialSystems)
virtual string description() {
  return "Shut down all systems that are not important for streaming data, to maximize the framerate we stream.";
}
virtual void execute(MachineState * ms)
{
  ms->config.shouldIRender = 0;
  ms->config.shouldIDoIK = 0;
  ms->config.shouldIImageCallback = 0;
  ms->config.shouldIRangeCallback = 0;
  ms->config.shouldIMiscCallback = 0;
  CONSOLE_ERROR(ms, "Shutting down all non-essential systems.");
}
END_WORD
REGISTER_WORD(ShutdownAllNonessentialSystems)

WORD(BringUpAllNonessentialSystems)
virtual string description() {
  return "Bring up systems that are not important for streaming data.";
}
virtual void execute(MachineState * ms)
{
  ms->config.shouldIRender = 1;
  ms->config.shouldIDoIK = 1;
  ms->config.ikShare = 1.0;
  ms->config.shouldIImageCallback = 1;
  ms->config.shouldIRangeCallback = 1;
  ms->config.shouldIMiscCallback = 1;
  CONSOLE_ERROR(ms, "Bringing up all non-essential systems.");
}
END_WORD
REGISTER_WORD(BringUpAllNonessentialSystems)


WORD(SetSisFlags)
virtual string description() {
  return "Set whether we should save different sensor streams. <pose> <range> <image> <joints> <word> <label> setSisFlags";
}
virtual void execute(MachineState * ms)
{
  cout << "Setting should I stream flags...";
  int tSisLabel = 0;
  int tSisWord = 0;
  int tSisJoints = 0;
  int tSisImage = 0;
  int tSisRange = 0;
  int tSisPose = 0;
  GET_ARG(ms, IntegerWord, tSisLabel);
  GET_ARG(ms, IntegerWord, tSisWord);
  GET_ARG(ms, IntegerWord, tSisJoints);
  GET_ARG(ms, IntegerWord, tSisImage);
  GET_ARG(ms, IntegerWord, tSisRange);
  GET_ARG(ms, IntegerWord, tSisPose);

  ms->config.sisLabel = tSisLabel;
  ms->config.sisWord = tSisWord;
  ms->config.sisJoints = tSisJoints;
  ms->config.sisImage = tSisImage;
  ms->config.sisRange = tSisRange;
  ms->config.sisPose = tSisPose;
  cout << "setting sis values, Pose Range Image Joints Word Label: " << ms->config.sisPose << " " << ms->config.sisRange << " " << ms->config.sisImage << " " << ms->config.sisJoints<< " " << ms->config.sisWord << " " << ms->config.sisLabel <<endl;
}
END_WORD
REGISTER_WORD(SetSisFlags)


WORD(StreamEnableAllSisFlags)
virtual string description() {
  return "Enable all SIS flags.";
}
virtual void execute(MachineState * ms)
{
  ms->evaluateProgram("1 1 1 1 1 1 setSisFlags");
}
END_WORD
REGISTER_WORD(StreamEnableAllSisFlags)

WORD(StreamDisableAllSisFlags)
virtual string description() {
  return "Enable all SIS flags.";
}
virtual void execute(MachineState * ms)
{
  ms->evaluateProgram("0 0 0 0 0 0 setSisFlags");
}
END_WORD
REGISTER_WORD(StreamDisableAllSisFlags)


CONFIG_GETTER_INT(StreamDiskStreaming, ms->config.diskStreamingEnabled)
CONFIG_SETTER_INT(StreamSetDiskStreaming, ms->config.diskStreamingEnabled)

WORD(DisableDiskStreaming)
virtual void execute(MachineState * ms)
{
  ms->config.diskStreamingEnabled= 0;
}
END_WORD
REGISTER_WORD(DisableDiskStreaming)

WORD(EnableDiskStreaming)
virtual void execute(MachineState * ms)
{
  ms->config.diskStreamingEnabled = 1;
}
END_WORD
REGISTER_WORD(EnableDiskStreaming)

WORD(ClearStreamBuffers)
virtual void execute(MachineState * ms)
{
  for (int i = 0; i < ms->config.cameras.size(); i++) {
    ms->config.cameras[i]->clearStreamBuffer();
  }
  ms->config.streamPoseBuffer.resize(0);
  ms->config.streamRangeBuffer.resize(0);
}
END_WORD
REGISTER_WORD(ClearStreamBuffers)


WORD(RewindImageStreamBuffer)
virtual void execute(MachineState * ms)
{
  cout << "rewindImageStreamBuffer" << endl;
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  camera->setIsbIdx(0);
}
END_WORD
REGISTER_WORD(RewindImageStreamBuffer)

WORD(RewindImageStreamBufferNLNK)
virtual void execute(MachineState * ms)
{
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  cout << "rewindImageStreamBufferNLNK" << endl;
  camera->setIsbIdxNoLoadNoKick(0);
}
END_WORD
REGISTER_WORD(RewindImageStreamBufferNLNK)


WORD(SetExpectedCropsToStream)
virtual void execute(MachineState * ms)
{
  int valToSet = 0;
  GET_ARG(ms, IntegerWord, valToSet);

  cout << "setExpectedCropsToStream: got value " << valToSet << endl;
  ms->config.expectedCropsToStream = valToSet;
}
END_WORD
REGISTER_WORD(SetExpectedCropsToStream)

WORD(IncrementImageStreamBuffer)
virtual string description() {
  return "Increments the current location in the image stream buffer.  Loads it into memory and kicks it out when done.";
}
virtual void execute(MachineState * ms)
{
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  int nextIdx = camera->sibCurIdx + 1;
  cout << "incrementImageStreamBuffer: Incrementing to " << nextIdx << " out of " << camera->streamImageBuffer.size() << endl;
  if ( (nextIdx > -1) && (nextIdx < camera->streamImageBuffer.size()) ) {
    streamImage * result = camera->setIsbIdx(nextIdx);  
    if (result == NULL) {
      cout << "increment failed :(" << endl;
    } else {
    }
  } else {
  }
}
END_WORD
REGISTER_WORD(IncrementImageStreamBuffer)

WORD(IncrementImageStreamBufferNoLoadNoKick)
virtual string description() {
  return "Increments the current location in the image stream buffer.  Does not load the image into memory or kick it out when done.";
}
virtual void execute(MachineState * ms)
{
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  int nextIdx = camera->sibCurIdx + 1;
  //cout << "incrementImageStreamBufferNoLoadNoKick: Incrementing to " << nextIdx << endl;
  if ( (nextIdx > -1) && (nextIdx < camera->streamImageBuffer.size()) ) {
    streamImage * result = camera->setIsbIdxNoLoadNoKick(nextIdx);  
    if (result == NULL) {
      cout << "increment failed :(" << endl;
    } else {
    }
  } else {
  }
}
END_WORD
REGISTER_WORD(IncrementImageStreamBufferNoLoadNoKick)

WORD(IncrementImageStreamBufferNoLoad)
virtual string description() {
  return "Increments the current location in the image stream buffer.  Does not load the image into memory, but kicks it out when done.";
}
virtual void execute(MachineState * ms)
{
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  int nextIdx = camera->sibCurIdx + 1;
  //cout << "incrementImageStreamBufferNoLoad: Incrementing to " << nextIdx << endl;
  if ( (nextIdx > -1) && (nextIdx < camera->streamImageBuffer.size()) ) {
    streamImage * result = camera->setIsbIdxNoLoad(nextIdx);  
    if (result == NULL) {
      cout << "increment failed :(" << endl;
    } else {
    }
  } else {
  }
}
END_WORD
REGISTER_WORD(IncrementImageStreamBufferNoLoad)


WORD(StreamIncrementImageStreamBuffer)
virtual string description() {
  return "Increments the current location in the image stream buffer.  The new default word, which does no load and no kick.";
}
virtual void execute(MachineState * ms)
{
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  int nextIdx = camera->sibCurIdx + 1;
  //cout << "incrementImageStreamBufferNoLoadNoKick: Incrementing to " << nextIdx << endl;
  if ( (nextIdx > -1) && (nextIdx < camera->streamImageBuffer.size()) ) {
    streamImage * result = camera->setIsbIdxNoLoadNoKick(nextIdx);  
    if (result == NULL) {
      cout << "increment failed :(" << endl;
    } else {
    }
  } else {
  }
}
END_WORD
REGISTER_WORD(StreamIncrementImageStreamBuffer)


WORD(ImageStreamBufferLoadCurrent)
virtual void execute(MachineState * ms)
{
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  int thisIdx = camera->sibCurIdx;
  //cout << "imageStreamBufferLoadCurrent: reloading " << thisIdx << endl;
  if ( (thisIdx > -1) && (thisIdx < camera->streamImageBuffer.size()) ) {
    streamImage * result = camera->setIsbIdxYesLoadNoKick(thisIdx);  
    if (result == NULL) {
      cout << "increment failed :(" << endl;
    } else {
    }
  } else {
  }
}
END_WORD
REGISTER_WORD(ImageStreamBufferLoadCurrent)



WORD(IntegrateImageStreamBufferServoImages)
virtual void execute(MachineState * ms)       {

  cout << "Searching image stream for servo images..." << endl;

  
  int tNumHeights = ms->config.hmWidth;
  for (int hIdx = tNumHeights-1; hIdx > -1; hIdx--) {
    ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("saveAccumulatedStreamToServoImage");
    ms->pushWord("streamedAccumulatedDensity");

    ms->pushWord("iterateIsbAndAccumulateHeightImages");
    ms->pushWord(std::make_shared<IntegerWord>(hIdx));


    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight"); 
    ms->pushWord(std::make_shared<IntegerWord>(hIdx));
    ms->pushWord("rewindImageStreamBuffer"); 
    ms->pushWord("resetAccumulatedStreamImage");
  }

  ms->pushWord("departureSpeed"); 
}
END_WORD
REGISTER_WORD(IntegrateImageStreamBufferServoImages)

WORD(ResetAccumulatedStreamImage)
virtual void execute(MachineState * ms)       {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  camera->resetAccumulatedStreamImage();
}
END_WORD
REGISTER_WORD(ResetAccumulatedStreamImage)

WORD(IterateIsbAndAccumulateHeightImages)
virtual void execute(MachineState * ms)       {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  streamImage * tsi = camera->setIsbIdxNoLoad(camera->sibCurIdx);
  if (tsi == NULL) {
    cout << "iterateIsbAndAccumulateHeightImages: setIsbIdxNoLoad returned null. Returning." << endl;
  } else {
  }

  int thisHeightIdx =  0;
  GET_ARG(ms, IntegerWord, thisHeightIdx); int tNumHeights = ms->config.hmWidth;

  double scaledHeight = ms->config.minHeight + ( (double(thisHeightIdx)/double(tNumHeights-1)) * (ms->config.maxHeight - ms->config.minHeight) ) ;

  eePose tArmP, tBaseP;
  int success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
  eePose thisServoPose = tBaseP;
  thisServoPose.pz = tBaseP.pz + scaledHeight;

  double p_dtp = 0.005;
  double dist_to_servo_p = eePose::distance(tArmP, thisServoPose);
  double p_dtq = 0.005;
  double dist_to_servo_q = eePose::distanceQ(tArmP, thisServoPose);

  // only load if we pass the distance test
  if ( success && (dist_to_servo_p < p_dtp) && (dist_to_servo_q < p_dtq)) {
    cout << "Stream servo image test SUCCESS, accepting, dist_to_servo_p, p_dtp, dist_to_servo_q, p_dtq: " << dist_to_servo_p << " " << p_dtp << " " << dist_to_servo_q << " " << p_dtq << endl;

    tsi = camera->setIsbIdx(camera->sibCurIdx);
    if (tsi == NULL) {
      CONSOLE_ERROR(ms, "iterateIsbAndAccumulateHeightImages: setIsbIdx returned null after pose check!");
    }

    Size sz = ms->config.accumulatedStreamImage.size();

    if (sz == tsi->image.size()) {
      int imW = sz.width;
      int imH = sz.height;

      for (int x = 0; x < imW; x++) {
	for (int y = 0; y < imH; y++) {
	  ms->config.accumulatedStreamImage.at<Vec3d>(y,x)[0] = ms->config.accumulatedStreamImage.at<Vec3d>(y,x)[0] + tsi->image.at<Vec3b>(y,x)[0];
	  ms->config.accumulatedStreamImage.at<Vec3d>(y,x)[1] = ms->config.accumulatedStreamImage.at<Vec3d>(y,x)[1] + tsi->image.at<Vec3b>(y,x)[1];
	  ms->config.accumulatedStreamImage.at<Vec3d>(y,x)[2] = ms->config.accumulatedStreamImage.at<Vec3d>(y,x)[2] + tsi->image.at<Vec3b>(y,x)[2];
	  ms->config.accumulatedStreamImageMass.at<double>(y,x) += 1.0;
	}
      }
    } else {
      Size newSz = tsi->image.size(); 
      cout << "iterateIsbAndAccumulateHeightImages resizing and setting accumulatedStreamImage and mass: " << sz << newSz << endl;
      ms->config.accumulatedStreamImage.create(newSz, CV_64FC3);
      ms->config.accumulatedStreamImageMass.create(newSz, CV_64F);
      int imW = newSz.width;
      int imH = newSz.height;
      for (int x = 0; x < imW; x++) {
	for (int y = 0; y < imH; y++) {
	  ms->config.accumulatedStreamImage.at<Vec3d>(y,x)[0] = tsi->image.at<Vec3b>(y,x)[0];
	  ms->config.accumulatedStreamImage.at<Vec3d>(y,x)[1] = tsi->image.at<Vec3b>(y,x)[1];
	  ms->config.accumulatedStreamImage.at<Vec3d>(y,x)[2] = tsi->image.at<Vec3b>(y,x)[2];
	  ms->config.accumulatedStreamImageMass.at<double>(y,x) = 1.0;
	}
      }
    }
  } else {
    //cout << "Stream servo image test FAILURE, skipping but recalling, dist_to_servo, p_dist_thresh: " << dist_to_servo << " " << p_dist_thresh << endl;
  }

  int recall = 1;
  int nextIdx = camera->sibCurIdx + 1;
  cout << "iterateIsbAndAccumulateHeightImages incrementing to " << nextIdx << endl;
  if ( (nextIdx > -1) && (nextIdx < camera->streamImageBuffer.size()) ) {
    streamImage * result = camera->setIsbIdx(nextIdx);  
    if (result == NULL) {
      cout << "iterateIsbAndAccumulateHeightImages increment failed :(, nextIdx: " << nextIdx << endl;
      ms->clearStack();
    } else {
    }
  } else {
    recall = 0;
  }

  if (recall) {
    ms->pushWord("iterateIsbAndAccumulateHeightImages");
    ms->pushWord(std::make_shared<IntegerWord>(thisHeightIdx));

  } else {
    cout << "iterateIsbAndAccumulateHeightImages reached the end of the image stream buffer." << endl;
  }
}
END_WORD
REGISTER_WORD(IterateIsbAndAccumulateHeightImages)

WORD(SetMapAutoPick)
virtual void execute(MachineState * ms)
{
  int valToSet = 0;
  GET_ARG(ms, IntegerWord, valToSet);

  cout << "setMapAutoPick: was " << ms->config.mapAutoPick << " will be " << valToSet << endl;
  ms->config.mapAutoPick = valToSet;
}
END_WORD
REGISTER_WORD(SetMapAutoPick)

WORD(MapAndPickL)
virtual void execute(MachineState * ms)
{
    ms->pushWord("mapAndPickL");
    ms->pushWord("mapAndPick");
    ms->pushWord("clearMapForPatrol");
    ms->pushWord("clearBlueBoxMemories");

	ms->pushWord("setPlaceModeToShake");
    ms->pushWord("setBoundingBoxModeToMapping"); 
    ms->pushWord("setIdleModeToEmpty"); 
}
END_WORD
REGISTER_WORD(MapAndPickL)

WORD(MapAndPick)
virtual void execute(MachineState * ms)
{
    ms->pushWord("pickAllBlueBoxes");
    ms->pushWord("mappingPatrol");
}
END_WORD
REGISTER_WORD(MapAndPick)

WORD(SetRandomPositionAfterPick)
virtual void execute(MachineState * ms)
{
  int valToSet = 0;
  GET_ARG(ms, IntegerWord, valToSet);

  cout << "setRandomPositionAfterPick: got value " << valToSet << endl;
  ms->config.setRandomPositionAfterPick = valToSet;
}
END_WORD
REGISTER_WORD(SetRandomPositionAfterPick)

WORD(SetStreamPicks)
virtual void execute(MachineState * ms)
{
  int valToSet = 0;
  GET_ARG(ms, IntegerWord, valToSet);

  cout << "setStreamPicks: got value " << valToSet << endl;
  ms->config.streamPicks = valToSet;
}
END_WORD
REGISTER_WORD(SetStreamPicks)

WORD(MoveAndStreamAimedShot)
virtual void execute(MachineState * ms)
{
  if (ms->config.streamPicks) {
	cout << "Looking down the barrel." << endl;

    ms->pushWord("bringUpAllNonessentialSystems"); 

    ms->pushWord("deactivateSensorStreaming"); 
    {
      stringstream ss;
      ss << "\"aimed for pick, end\"";
      string result = ss.str();
      ms->pushWord("streamLabel");
      ms->pushWord(result);
    }
    ms->pushWord("activateSensorStreaming"); 

    {
      ms->pushWord("deactivateSensorStreaming"); 

      ms->pushWord("waitForSeconds"); 
      ms->pushWord("0.1"); 

      ms->pushWord("waitUntilImageCallbackReceived"); 
      ms->pushWord("activateSensorStreaming"); 

      ms->pushWord("comeToStop");
      ms->pushWord("setMovementStateToMoving");
      ms->pushWord("comeToStop");
      ms->pushWord("waitUntilAtCurrentPosition");
    }

    ms->pushWord("deactivateSensorStreaming"); 
    {
      stringstream ss;
      ss << "\"aimed for pick, start\"";
      string result = ss.str();
      ms->pushWord("streamLabel");
      ms->pushWord(result);
    }
    ms->pushWord("activateSensorStreaming"); 

    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("shutdownToSensorsAndMovement"); 
    ms->pushWord("setSisFlags"); 
    //ms->pushWord(std::make_shared<IntegerWord>(1));
    //ms->pushWord(std::make_shared<IntegerWord>(0));
    //ms->pushWord(std::make_shared<IntegerWord>(1));
    //ms->pushWord(std::make_shared<IntegerWord>(0));
    //ms->pushWord(std::make_shared<IntegerWord>(0));
    //ms->pushWord(std::make_shared<IntegerWord>(1));

    ms->pushWord(std::make_shared<IntegerWord>(1));
    ms->pushWord(std::make_shared<IntegerWord>(0));
    ms->pushWord(std::make_shared<IntegerWord>(0));
    ms->pushWord(std::make_shared<IntegerWord>(1));
    ms->pushWord(std::make_shared<IntegerWord>(0));
    ms->pushWord(std::make_shared<IntegerWord>(1));


    ms->pushWord("comeToStop");
    ms->pushWord("sampleHeight"); 
    ms->pushWord("assumeAimedPose");

    ms->pushWord("saveCalibrationToClass");
  } else {
  }
}
END_WORD
REGISTER_WORD(MoveAndStreamAimedShot)

WORD(StreamGraspResult)
virtual void execute(MachineState * ms)
{
  cout << "streamGraspResult: " << endl;
  if (ms->config.streamPicks) {
    cout << "  streaming picks: " << endl;

    ms->pushWord("deactivateSensorStreaming"); 
	stringstream ss;
	ss << "\"pickSuccess " << isGripperGripping(ms) << "\"";
	string result = ss.str();

    ms->pushWord("streamLabel");
    ms->pushWord(result);
    ms->pushWord("activateSensorStreaming"); 

    ms->pushWord("setSisFlags"); 
    //ms->pushWord(std::make_shared<IntegerWord>(0));
    //ms->pushWord(std::make_shared<IntegerWord>(0));
    //ms->pushWord(std::make_shared<IntegerWord>(0));
    //ms->pushWord(std::make_shared<IntegerWord>(0));
    //ms->pushWord(std::make_shared<IntegerWord>(0));
    //ms->pushWord(std::make_shared<IntegerWord>(1));

    ms->pushWord(std::make_shared<IntegerWord>(1));
    ms->pushWord(std::make_shared<IntegerWord>(0));
    ms->pushWord(std::make_shared<IntegerWord>(0));
    ms->pushWord(std::make_shared<IntegerWord>(0));
    ms->pushWord(std::make_shared<IntegerWord>(0));
    ms->pushWord(std::make_shared<IntegerWord>(0));

  } else {
	cout << "  not streaming picks: " << endl;
  }
}
END_WORD
REGISTER_WORD(StreamGraspResult)


WORD(StreamEnableSisImageAndPoses)
virtual string description() {
  return "Configure Ein to stream images and poses only.";
}
virtual void execute(MachineState * ms)
{
  ms->evaluateProgram("streamDisableAllSisFlags 1 streamSetSisPose 1 streamSetSisImage");
}
END_WORD
REGISTER_WORD(StreamEnableSisImageAndPoses)

WORD(StreamRenderStreamWindow)
virtual string description() {
  return "Render the stream buffer window.";
}
virtual void execute(MachineState * ms) {
  if (!ms->config.showgui) {
    return;
  }    
  ms->config.streamViewerWindow->update();
}
END_WORD
REGISTER_WORD(StreamRenderStreamWindow)


WORD(StreamPoseForCurrentImage)
virtual string description() {
  return "Push the pose for the current image in the stream buffer.";
}
virtual void execute(MachineState * ms) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  streamImage * tsi = camera->currentImage();
  eePose tArmP, tBaseP;
  int success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
  ms->pushWord(make_shared<EePoseWord>(tArmP));
}
END_WORD
REGISTER_WORD(StreamPoseForCurrentImage)


WORD(StreamBasePoseForCurrentImage)
virtual string description() {
  return "Push the arm pose for the current image in the stream buffer.";
}
virtual void execute(MachineState * ms) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  streamImage * tsi = camera->currentImage();
  eePose tArmP, tBaseP;
  int success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
  ms->pushWord(make_shared<EePoseWord>(tBaseP));
}
END_WORD
REGISTER_WORD(StreamBasePoseForCurrentImage)


WORD(StreamPlayStreamBuffer)
virtual string description() {
  return "Play back the stream buffer.  Plays back at a constant rate; is not careful to wait the 'correct' amount of time between frames.";
}
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("( streamIncrementImageStreamBuffer streamRenderStreamWindow 0.01 waitForSeconds ) streamImageBufferSize replicateWord");
}
END_WORD
REGISTER_WORD(StreamPlayStreamBuffer)


CONFIG_GETTER_INT(StreamRangeBufferSize, ms->config.streamRangeBuffer.size())
CONFIG_GETTER_INT(StreamPoseBufferSize, ms->config.streamPoseBuffer.size())
CONFIG_GETTER_INT(StreamJointBufferSize, ms->config.streamJointsBuffer.size())
CONFIG_GETTER_INT(StreamWordBufferSize, ms->config.streamWordBuffer.size())
CONFIG_GETTER_INT(StreamLabelBufferSize, ms->config.streamLabelBuffer.size())
CONFIG_GETTER_INT(StreamImageBufferSize, ms->config.cameras[ms->config.focused_camera]->streamImageBuffer.size())

CONFIG_GETTER_INT(StreamImageBufferCurrentIdx, ms->config.cameras[ms->config.focused_camera]->sibCurIdx)
CONFIG_SETTER_INT(StreamSetImageBufferCurrentIdx, ms->config.cameras[ms->config.focused_camera]->sibCurIdx)




CONFIG_GETTER_INT(StreamSisLabel, ms->config.sisLabel)
CONFIG_GETTER_INT(StreamSisWord, ms->config.sisWord)
CONFIG_GETTER_INT(StreamSisJoints, ms->config.sisJoints)
CONFIG_GETTER_INT(StreamSisImage, ms->config.sisImage)
CONFIG_GETTER_INT(StreamSisRange, ms->config.sisRange)
CONFIG_GETTER_INT(StreamSisPose, ms->config.sisPose)

CONFIG_SETTER_INT(StreamSetSisLabel, ms->config.sisLabel)
CONFIG_SETTER_INT(StreamSetSisWord, ms->config.sisWord)
CONFIG_SETTER_INT(StreamSetSisJoints, ms->config.sisJoints)
CONFIG_SETTER_INT(StreamSetSisImage, ms->config.sisImage)
CONFIG_SETTER_INT(StreamSetSisRange, ms->config.sisRange)
CONFIG_SETTER_INT(StreamSetSisPose, ms->config.sisPose)


}
