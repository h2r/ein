#include "ein_words.h"
#include "ein.h"
#include "camera.h"

namespace ein_words {

WORD(StreamLabel)
virtual void execute(MachineState * ms)
{
  string thisLabel;
  GET_ARG(ms, StringWord, thisLabel);

  cout << "streamLabel: " << thisLabel << endl;

  int cfClass = ms->config.focusedClass;
  if ((cfClass > -1) && (cfClass < ms->config.classLabels.size()) && (ms->config.sensorStreamOn) && (ms->config.sisLabel)) {
    ros::Time rNow = ros::Time::now();
    double thisNow = rNow.toSec();
    streamLabelAsClass(ms, thisLabel, cfClass, thisNow);

    for (int i = 0; i < ms->config.streamLabelBuffer.size(); i++) {
      cout << "  streamLabelBuffer[" << i << "] = " << ms->config.streamLabelBuffer[i].label << " " << ms->config.streamLabelBuffer[i].time << endl;;
    }
  } else {
    cout << "  streamLabel failed " << thisLabel << endl;
cout << " XXX " << (cfClass > -1)  << (cfClass < ms->config.classLabels.size()) << (ms->config.sensorStreamOn) << (ms->config.sisLabel) << endl;
  } // do nothing
}
END_WORD
REGISTER_WORD(StreamLabel)

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

WORD(ActivateSensorStreaming)
virtual string description() {
  return "Start streaming data; you might want to set the SiS (\"Should I Stream?\") first using setSisFlags or streamSetSis.";
}
virtual void execute(MachineState * ms)
{
  activateSensorStreaming(ms);
}
END_WORD
REGISTER_WORD(ActivateSensorStreaming)

WORD(DeactivateSensorStreaming)
virtual string description() {
  return "Stop streaming data.";
}
virtual void execute(MachineState * ms)
{
  deactivateSensorStreaming(ms);
}
END_WORD
REGISTER_WORD(DeactivateSensorStreaming)

WORD(StreamWriteBuffersToDisk)
virtual string description() {
  return "Write what is in the stream buffer to disk.";
}
virtual void execute(MachineState * ms)
{
  REQUIRE_FOCUSED_CLASS(ms,tfc);

  CONSOLE(ms, "Writing to " << streamDirectory(ms, tfc));

  writeRangeBatchAsClass(ms, tfc);	
  writePoseBatchAsClass(ms, tfc);	
  writeJointsBatchAsClass(ms, tfc);	
  writeWordBatchAsClass(ms, tfc);	
  writeLabelBatchAsClass(ms, tfc);
  for (int i = 0; i < ms->config.cameras.size(); i++) {
    ms->config.cameras[i]->writeImageBatchAsClass(tfc);
  }

}
END_WORD
REGISTER_WORD(StreamWriteBuffersToDisk)




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

WORD(PopulateStreamBuffers)
virtual void execute(MachineState * ms)
{
  for (int i = 0; i < ms->config.cameras.size(); i++) {
    ms->config.cameras[i]->clearStreamBuffer();
    ms->config.cameras[i]->populateStreamImageBuffer();
  }
  ms->config.streamPoseBuffer.resize(0);
  ms->config.streamRangeBuffer.resize(0);


  populateStreamPoseBuffer(ms);
  populateStreamRangeBuffer(ms);
  populateStreamJointsBuffer(ms);
  populateStreamWordBuffer(ms);
  populateStreamLabelBuffer(ms);

  sort(ms->config.streamRangeBuffer.begin(), ms->config.streamRangeBuffer.end(), streamRangeComparator);
  sort(ms->config.streamPoseBuffer.begin(), ms->config.streamPoseBuffer.end(), streamPoseComparator);
  sort(ms->config.streamJointsBuffer.begin(), ms->config.streamJointsBuffer.end(), streamJointsComparator);
  sort(ms->config.streamWordBuffer.begin(), ms->config.streamWordBuffer.end(), streamWordComparator);
  sort(ms->config.streamLabelBuffer.begin(), ms->config.streamLabelBuffer.end(), streamLabelComparator);
}
END_WORD
REGISTER_WORD(PopulateStreamBuffers)

WORD(IntegrateRangeStreamBuffer)
virtual void execute(MachineState * ms)
{
  int thisFc = ms->config.focusedClass;
  if ( (thisFc > -1) && (thisFc < ms->config.classLabels.size()) ) {
  } else {
    cout << "integrateRangeStreamBuffer sees an invalid focused class, clearing call stack." << endl;
    ms->clearStack();
    return;
  }

  initClassFolders(ms, ms->config.data_directory + "/objects/" + ms->config.classLabels[thisFc] + "/");


  int p_printSkip = 1000; 

  for (int i = 0; i < ms->config.streamRangeBuffer.size(); i++) {
    streamRange &tsr = ms->config.streamRangeBuffer[i];
    eePose tArmP, tBaseP, tRelP;
    int success = getStreamPoseAtTime(ms, tsr.time, &tArmP, &tBaseP);
    tRelP = tArmP.getPoseRelativeTo(tBaseP); 
    double tRange = tsr.range;

    //cout << "got stream pose at time " << tsr.time << " " << tArmP << tBaseP << tRelP << endl;

    if (success) {
      ms->config.rmcX = tBaseP.px;
      ms->config.rmcY = tBaseP.py;
      ms->config.rmcZ = tBaseP.pz;

      // this allows us to stitch together readings from different scans
      //cout << "XXX: " << endl << tArmP << tBaseP << tRelP << tRelP.applyAsRelativePoseTo(tBaseP) << "YYY" << endl;
      eePose thisCrane = tBaseP;
      thisCrane.copyQ(ms->config.straightDown); 
      eePose thisCraneRelativeThisBase = thisCrane.getPoseRelativeTo(tBaseP);

      eePose rebasedRelative = tRelP.applyAsRelativePoseTo(thisCraneRelativeThisBase);
      eePose rebasedArm = rebasedRelative.applyAsRelativePoseTo(tBaseP);

      Eigen::Vector3d rayDirection;
      Eigen::Vector3d castPoint;
      castRangeRay(ms, tRange, rebasedArm, &castPoint, &rayDirection);
      update2dRangeMaps(ms, castPoint);
      if ((i % p_printSkip) == 0) {
	cout << "cast rays for measurement " << i << " z: " << castPoint[2] << " range: " << tRange << endl;// << tRelP;// << " " << castPoint << endl;
      } else {
      }
    } else {
      cout << "ray " << i << " failed to get pose, not casting." << endl;
    }
  }

  ms->pushWord("fullRender"); 
  ms->pushWord("paintReticles"); 
  ms->pushWord("shiftIntoGraspGear1"); 
  ms->pushWord("drawMapRegisters"); 
  ms->pushWord("downsampleIrScan"); 
}
END_WORD
REGISTER_WORD(IntegrateRangeStreamBuffer)


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

WORD(IntegrateImageStreamBufferCrops)
virtual void execute(MachineState * ms)
{
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  int thisFc = ms->config.focusedClass;
  if ( (thisFc > -1) && (thisFc < ms->config.classLabels.size()) ) {
  } else {
    cout << "integrateImageStreamBuffer sees an invalid focused class, clearing call stack." << endl;
    ms->clearStack();
    return;
  }

  initClassFolders(ms, ms->config.data_directory + "/objects/" + ms->config.classLabels[thisFc] + "/");

  // backwards because it's a stack
  // XXX this should increment without loading and only load if it passes the test

  int keptSamples = 0;
  for (int i = camera->streamImageBuffer.size()-1; i > -1; i--) {
    // should really check pose here and only process if it is good.
    streamImage * tsi = camera->setIsbIdxNoLoad(i);
    if (tsi == NULL) {
      cout << "streamCropsAsFocusedClass: setIsbIdxNoLoad returned null. Returning." << endl;
    } else {
    }

    eePose tArmP, tBaseP;
    int success = getStreamPoseAtTime(ms, tsi->time, &tArmP, &tBaseP);

    double thisZ = tArmP.pz - tBaseP.pz;
    eePose thisVpBaseheight;
    thisVpBaseheight.pz = tBaseP.pz + (thisZ - convertHeightIdxToLocalZ(ms, ms->config.mappingHeightIdx));
    pixelToGlobal(ms, camera->vanishingPointReticle.px, camera->vanishingPointReticle.py, thisZ, &thisVpBaseheight.px, &thisVpBaseheight.py, tArmP);

    double p_dist_thresh = 0.07;
    double dist_to_base = eePose::distance(tBaseP, thisVpBaseheight);
    // only load if we pass the distance test
	
    if (dist_to_base < p_dist_thresh) {
      cout << "Counting stream crop vanishing point to base test SUCCESS, accepting, dist_to_base, p_dist_thresh: " << dist_to_base << " " << p_dist_thresh << endl;
	  keptSamples++;
    } else {
    }
  }

  for (int i = camera->streamImageBuffer.size()-1; i > -1; i--) {
    //ms->pushWord("endStackCollapseNoop");
    ms->pushWord("incrementImageStreamBuffer");

    // should really check pose here and only process if it is good.
    streamImage * tsi = camera->setIsbIdxNoLoad(i);
    if (tsi == NULL) {
      cout << "streamCropsAsFocusedClass: setIsbIdxNoLoad returned null. Returning." << endl;
    } else {
    }

    eePose tArmP, tBaseP;
    int success = getStreamPoseAtTime(ms, tsi->time, &tArmP, &tBaseP);

    double thisZ = tArmP.pz - tBaseP.pz;
    eePose thisVpBaseheight;
    thisVpBaseheight.pz = tBaseP.pz + (thisZ - convertHeightIdxToLocalZ(ms, ms->config.mappingHeightIdx));
    pixelToGlobal(ms, camera->vanishingPointReticle.px, camera->vanishingPointReticle.py, thisZ, &thisVpBaseheight.px, &thisVpBaseheight.py, tArmP);

    double p_dist_thresh = 0.07;
    double dist_to_base = eePose::distance(tBaseP, thisVpBaseheight);
    // only load if we pass the distance test
	
	double keepFraction = ms->config.expectedCropsToStream / double(keptSamples);
	int keepSample = ( (drand48()) < (keepFraction) );
    if ((dist_to_base < p_dist_thresh) && keepSample) {
      cout << "Stream crop vanishing point to base test SUCCESS, accepting, dist_to_base, p_dist_thresh: " << dist_to_base << " " << p_dist_thresh << endl;
	  cout << keepFraction << " " << ms->config.expectedCropsToStream << " " << keptSamples << endl;

      //ms->pushWord("streamCropsAsFocusedClass");
      ms->pushWord("streamCenterCropAsFocusedClass");
      ms->pushWord("goFindBlueBoxes"); 
      ms->pushWord("streamedDensity"); 
    } else {
    }
  }
  ms->pushWord("rewindImageStreamBuffer"); 
}
END_WORD
REGISTER_WORD(IntegrateImageStreamBufferCrops)

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
  return "Increments the current location in the image stream buffer.";
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

WORD(StreamCropsAsFocusedClass)
virtual void execute(MachineState * ms)       {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  streamImage * tsi = camera->setIsbIdxNoLoad(camera->sibCurIdx);
  if (tsi == NULL) {
    cout << "streamCropsAsFocusedClass: setIsbIdxNoLoad returned null. Returning." << endl;
  } else {
  }

  eePose tArmP, tBaseP;
  int success = getStreamPoseAtTime(ms, tsi->time, &tArmP, &tBaseP);

  double thisZ = tArmP.pz - tBaseP.pz;
  eePose thisVpBaseheight;
  thisVpBaseheight.pz = tBaseP.pz + (thisZ - convertHeightIdxToLocalZ(ms, ms->config.mappingHeightIdx));
  pixelToGlobal(ms, camera->vanishingPointReticle.px, camera->vanishingPointReticle.py, thisZ, &thisVpBaseheight.px, &thisVpBaseheight.py, tArmP);

  double p_dist_thresh = 0.07;
  double dist_to_base = eePose::distance(tBaseP, thisVpBaseheight);
  // only load if we pass the distance test
  if (dist_to_base < p_dist_thresh) {
    cout << "streamCropsAsFocusedClass: vanishing point to base test SUCCESS, accepting, dist_to_base, p_dist_thresh: " << dist_to_base << " " << p_dist_thresh << endl;
    tsi = camera->setIsbIdx(camera->sibCurIdx);
    if (tsi == NULL) {
      cout << "streamCropsAsFocusedClass: setIsbIdx returned null after distance check! Returning." << endl;
    } else {
    }
  } else {
    cout << "streamCropsAsFocusedClass: vanishing point to base test FAILURE, skipping, dist_to_base, p_dist_thresh: " << dist_to_base << " " << p_dist_thresh << endl;
    return;
  }

  if ( ms->config.focusedClass > -1 ) {
    for (int c = 0; c < ms->config.bTops.size(); c++) {
      // XXX TODO want to annotate these crops with a yaml file that includes pose and time
      string thisLabelName = ms->config.focusedClassLabel;
      Mat thisTarget = tsi->image;
      Mat crop = thisTarget(cv::Rect(ms->config.bTops[c].x, ms->config.bTops[c].y, ms->config.bBots[c].x-ms->config.bTops[c].x, ms->config.bBots[c].y-ms->config.bTops[c].y));
      std::stringstream buf;
      string this_crops_path = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/detectionCrops/";

      ros::Time thisNow = ros::Time::now();
      buf << this_crops_path << ms->config.run_prefix << ms->config.robot_serial << ms->config.left_or_right_arm << std::setprecision (std::numeric_limits<double>::digits10 + 1) << thisNow << ".png";
      // no compression!
      std::vector<int> args;
      args.push_back(CV_IMWRITE_PNG_COMPRESSION);
      args.push_back(ms->config.globalPngCompression);
      imwrite(buf.str(), crop, args);
      ms->config.cropCounter++;
    }
  } else {
  }
}
END_WORD
REGISTER_WORD(StreamCropsAsFocusedClass)

WORD(StreamCenterCropAsFocusedClass)
virtual void execute(MachineState * ms)       {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  streamImage * tsi = camera->setIsbIdxNoLoad(camera->sibCurIdx);
  if (tsi == NULL) {
    cout << "streamCenterCropAsFocusedClass: setIsbIdxNoLoad returned null. Returning." << endl;
  } else {
  }

  eePose tArmP, tBaseP;
  int success = getStreamPoseAtTime(ms, tsi->time, &tArmP, &tBaseP);

  double thisZ = tArmP.pz - tBaseP.pz;
  eePose thisVpBaseheight;
  thisVpBaseheight.pz = tBaseP.pz + (thisZ - convertHeightIdxToLocalZ(ms, ms->config.mappingHeightIdx));
  pixelToGlobal(ms, camera->vanishingPointReticle.px, camera->vanishingPointReticle.py, thisZ, &thisVpBaseheight.px, &thisVpBaseheight.py, tArmP);

  double p_dist_thresh = 0.07;
  double dist_to_base = eePose::distance(tBaseP, thisVpBaseheight);
  // only load if we pass the distance test
  if (dist_to_base < p_dist_thresh) {
    cout << "streamCenterCropAsFocusedClass: vanishing point to base test SUCCESS, accepting, dist_to_base, p_dist_thresh: " << dist_to_base << " " << p_dist_thresh << endl;
    tsi = camera->setIsbIdx(camera->sibCurIdx);
    if (tsi == NULL) {
      cout << "streamCenterCropAsFocusedClass: setIsbIdx returned null after distance check! Returning." << endl;
    } else {
    }
  } else {
    cout << "streamCenterCropAsFocusedClass: vanishing point to base test FAILURE, skipping, dist_to_base, p_dist_thresh: " << dist_to_base << " " << p_dist_thresh << endl;
    return;
  }

  if ( ms->config.focusedClass > -1 ) {
    int c = ms->config.pilotClosestBlueBoxNumber;
    if ( (c > -1) && (c < ms->config.bTops.size()) ) {
      // XXX TODO want to annotate these crops with a yaml file that includes pose and time
      string thisLabelName = ms->config.focusedClassLabel;
      Mat thisTarget = tsi->image;
      Mat crop = thisTarget(cv::Rect(ms->config.bTops[c].x, ms->config.bTops[c].y, ms->config.bBots[c].x-ms->config.bTops[c].x, ms->config.bBots[c].y-ms->config.bTops[c].y));
      std::stringstream buf;
      string this_crops_path = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/detectionCrops/";

      ros::Time thisNow = ros::Time::now();
      buf << this_crops_path << ms->config.run_prefix << ms->config.robot_serial << ms->config.left_or_right_arm << std::setprecision (std::numeric_limits<double>::digits10 + 1) << thisNow.toSec() << ".png";
cout << "  saving to " << buf.str() << " with this_crops_path " << this_crops_path;
      // no compression!
      std::vector<int> args;
      args.push_back(CV_IMWRITE_PNG_COMPRESSION);
      args.push_back(ms->config.globalPngCompression);
      imwrite(buf.str(), crop, args);
      ms->config.cropCounter++;
    }
  } else {
  }
}
END_WORD
REGISTER_WORD(StreamCenterCropAsFocusedClass)

WORD(SaveAccumulatedStreamToServoImage)
virtual void execute(MachineState * ms)       {
  cout << "saveAccumulatedStreamToServoImage ";
  if ((ms->config.focusedClass > -1) && (ms->config.accumulatedStreamImageBytes.rows >1) && (ms->config.accumulatedStreamImageBytes.cols > 1)) {
    string thisLabelName = ms->config.classLabels[ms->config.focusedClass];

    string folderPath = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/servoImages/";
    string filePath;

    // ATTN 16
    switch (ms->config.currentThompsonHeightIdx) {
    case 0:
      {
        filePath = "aerialHeight0PreGradients.png";
      }
      break;
    case 1:
      {
        filePath = "aerialHeight1PreGradients.png";
      }
      break;
    case 2:
      {
        filePath = "aerialHeight2PreGradients.png";
      }
      break;
    case 3:
      {
        filePath = "aerialHeight3PreGradients.png";
      }
      break;
    default:
      {
        assert(0);
        break;
      }
    }
    string servoImagePath = folderPath + filePath;
    cout << "saving to " << servoImagePath << endl;
    saveAccumulatedStreamToPath(ms, servoImagePath);
  } else {
    cout << "FAILED." << endl;
  }
}
END_WORD
REGISTER_WORD(SaveAccumulatedStreamToServoImage)

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
  int success = getStreamPoseAtTime(ms, tsi->time, &tArmP, &tBaseP);
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

WORD(PickAllBlueBoxes)
virtual void execute(MachineState * ms)
{
  int foundOne = 0;
  int foundClass = -1;
  cout << "pickAllBlueBoxes: promoting blue boxes" << endl;
  promoteBlueBoxes(ms);

  // loop through blue boxes
  int nc = ms->config.classLabels.size();
  for (int i = 0; i < nc; i++) {
    int idxOfFirst = -1;
    vector<BoxMemory> focusedClassMemories = memoriesForClass(ms, i, &idxOfFirst);

    if (idxOfFirst == -1) {
      cout << "No POSE_REPORTED objects of class" << i << "." << endl;
    } else {
      cout << "Found a POSE_REPORTED object of class" << i << ". Picking it." << endl;

      ms->pushWord("pickAllBlueBoxes");
      ms->pushWord("deliverObject");
      ms->pushWord(std::make_shared<StringWord>(ms->config.classLabels[i]));
      ms->config.endThisStackCollapse = 1;
      return;
    }
  }

  cout << "Found no POSE_REPORTED objects of any class" << endl;
}
END_WORD
REGISTER_WORD(PickAllBlueBoxes)

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


WORD(StreamImageBufferSize)
virtual void execute(MachineState * ms)
{
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  ms->pushData(std::make_shared<IntegerWord>(camera->streamImageBuffer.size()));
}
END_WORD
REGISTER_WORD(StreamImageBufferSize)

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



CONFIG_GETTER_INT(StreamRangeBufferSize, ms->config.streamRangeBuffer.size())
CONFIG_GETTER_INT(StreamPoseBufferSize, ms->config.streamPoseBuffer.size())
CONFIG_GETTER_INT(StreamJointBufferSize, ms->config.streamJointsBuffer.size())
CONFIG_GETTER_INT(StreamWordBufferSize, ms->config.streamWordBuffer.size())
CONFIG_GETTER_INT(StreamLabelBufferSize, ms->config.streamLabelBuffer.size())




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
