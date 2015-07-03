#include "ein_words.h"
#include "ein.h"
namespace ein_words {

WORD(RestoreIkShare)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.ikShare = 1.0;
}
END_WORD
REGISTER_WORD(RestoreIkShare)

WORD(ShutdownToSensorsAndMovement)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.shouldIRender = 0;
  ms->config.shouldIDoIK = 1;
  ms->config.ikShare = 1.0;//0.1;
  ms->config.shouldIImageCallback = 0;
  ms->config.shouldIRangeCallback = 0;
  ms->config.shouldIMiscCallback = 0;
  cout << "Shutting down to sensors and movement." << endl;
}
END_WORD
REGISTER_WORD(ShutdownToSensorsAndMovement)

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
  ms->config.ikShare = 1.0;
  ms->config.shouldIImageCallback = 1;
  ms->config.shouldIRangeCallback = 1;
  ms->config.shouldIMiscCallback = 1;
  cout << "Bringing up all non-essential systems." << endl;
}
END_WORD
REGISTER_WORD(BringUpAllNonessentialSystems)

WORD(ActivateSensorStreaming)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  activateSensorStreaming(ms);
}
END_WORD
REGISTER_WORD(ActivateSensorStreaming)

WORD(DeactivateSensorStreaming)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  deactivateSensorStreaming(ms);
}
END_WORD
REGISTER_WORD(DeactivateSensorStreaming)

WORD(SetSisFlags)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  cout << "Setting should I stream flags...";
  shared_ptr<Word> firstFlagWord = ms->popWord();
  shared_ptr<Word> secondFlagWord = ms->popWord();
  shared_ptr<Word> thirdFlagWord = ms->popWord();
  std::shared_ptr<IntegerWord> fiWord = std::dynamic_pointer_cast<IntegerWord>(firstFlagWord);
  std::shared_ptr<IntegerWord> seWord = std::dynamic_pointer_cast<IntegerWord>(secondFlagWord);
  std::shared_ptr<IntegerWord> thWord = std::dynamic_pointer_cast<IntegerWord>(thirdFlagWord);

  if( (fiWord == NULL) || (seWord == NULL) || (thWord == NULL) ) {
    cout << "not enough words... clearing stack." << endl;
    ms->clearStack();
    return;
  } else {
    ms->config.sisImage = fiWord->value();
    ms->config.sisRange = seWord->value();
    ms->config.sisPose = thWord->value();
    cout << "setting sis values, Pose Range Image: " << ms->config.sisPose << " " << ms->config.sisRange << " " << ms->config.sisImage << endl;
  }
}
END_WORD
REGISTER_WORD(SetSisFlags)

WORD(DisableDiskStreaming)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.diskStreamingEnabled= 0;
}
END_WORD
REGISTER_WORD(DisableDiskStreaming)

WORD(EnableDiskStreaming)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.diskStreamingEnabled = 1;
}
END_WORD
REGISTER_WORD(EnableDiskStreaming)

WORD(ClearStreamBuffers)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.streamPoseBuffer.resize(0);
  ms->config.streamRangeBuffer.resize(0);
}
END_WORD
REGISTER_WORD(ClearStreamBuffers)

WORD(PopulateStreamBuffers)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->config.streamImageBuffer.resize(0);
  ms->config.streamPoseBuffer.resize(0);
  ms->config.streamRangeBuffer.resize(0);

  populateStreamImageBuffer(ms);
  populateStreamPoseBuffer(ms);
  populateStreamRangeBuffer(ms);

  sort(ms->config.streamImageBuffer.begin(), ms->config.streamImageBuffer.end(), streamImageComparator);
  sort(ms->config.streamRangeBuffer.begin(), ms->config.streamRangeBuffer.end(), streamRangeComparator);
  sort(ms->config.streamPoseBuffer.begin(), ms->config.streamPoseBuffer.end(), streamPoseComparator);
}
END_WORD
REGISTER_WORD(PopulateStreamBuffers)

WORD(IntegrateRangeStreamBuffer)
virtual void execute(std::shared_ptr<MachineState> ms)
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
virtual void execute(std::shared_ptr<MachineState> ms)
{
  cout << "RewindImageStreamBuffer" << endl;
  setIsbIdx(ms, 0);
}
END_WORD
REGISTER_WORD(RewindImageStreamBuffer)

WORD(IntegrateImageStreamBufferCrops)
virtual void execute(std::shared_ptr<MachineState> ms)
{
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
  for (int i = ms->config.streamImageBuffer.size()-1; i > -1; i--) {
    //ms->pushWord("endStackCollapseNoop");
    ms->pushWord("incrementImageStreamBuffer");

    // should really check pose here and only process if it is good.
    streamImage * tsi = setIsbIdxNoLoad(ms, i);
    if (tsi == NULL) {
      cout << "streamCropsAsFocusedClass: setIsbIdxNoLoad returned null. Returning." << endl;
    } else {
    }

    eePose tArmP, tBaseP;
    int success = getStreamPoseAtTime(ms, tsi->time, &tArmP, &tBaseP);

    double thisZ = tArmP.pz - tBaseP.pz;
    eePose thisVpBaseheight;
    thisVpBaseheight.pz = tBaseP.pz + (thisZ - convertHeightIdxToLocalZ(ms, ms->config.mappingHeightIdx));
    pixelToGlobal(ms, ms->config.vanishingPointReticle.px, ms->config.vanishingPointReticle.py, thisZ, &thisVpBaseheight.px, &thisVpBaseheight.py, tArmP);

    double p_dist_thresh = 0.07;
    double dist_to_base = eePose::distance(tBaseP, thisVpBaseheight);
    // only load if we pass the distance test
    if (dist_to_base < p_dist_thresh) {
      cout << "Stream crop vanishing point to base test SUCCESS, accepting, dist_to_base, p_dist_thresh: " << dist_to_base << " " << p_dist_thresh << endl;

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

WORD(IncrementImageStreamBuffer)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  int nextIdx = ms->config.sibCurIdx + 1;
  cout << "incrementImageStreamBuffer: Incrementing to " << nextIdx << endl;
  if ( (nextIdx > -1) && (nextIdx < ms->config.streamImageBuffer.size()) ) {
    streamImage * result = setIsbIdx(ms, nextIdx);  
    if (result == NULL) {
      cout << "increment failed :(" << endl;
    } else {
    }
  } else {
  }
}
END_WORD
REGISTER_WORD(IncrementImageStreamBuffer)

WORD(IncrementImageStreamBufferNoLoad)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  int nextIdx = ms->config.sibCurIdx + 1;
  cout << "incrementImageStreamBufferNoLoad: Incrementing to " << nextIdx << endl;
  if ( (nextIdx > -1) && (nextIdx < ms->config.streamImageBuffer.size()) ) {
    streamImage * result = setIsbIdxNoLoad(ms, nextIdx);  
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
virtual void execute(std::shared_ptr<MachineState> ms)
{
  int thisIdx = ms->config.sibCurIdx;
  cout << "imageStreamBufferLoadCurrent: reloading " << thisIdx << endl;
  if ( (thisIdx > -1) && (thisIdx < ms->config.streamImageBuffer.size()) ) {
    streamImage * result = setIsbIdxNoLoad(ms, thisIdx);  
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
virtual void execute(std::shared_ptr<MachineState> ms)       {

  streamImage * tsi = setIsbIdxNoLoad(ms, ms->config.sibCurIdx);
  if (tsi == NULL) {
    cout << "streamCropsAsFocusedClass: setIsbIdxNoLoad returned null. Returning." << endl;
  } else {
  }

  eePose tArmP, tBaseP;
  int success = getStreamPoseAtTime(ms, tsi->time, &tArmP, &tBaseP);

  double thisZ = tArmP.pz - tBaseP.pz;
  eePose thisVpBaseheight;
  thisVpBaseheight.pz = tBaseP.pz + (thisZ - convertHeightIdxToLocalZ(ms, ms->config.mappingHeightIdx));
  pixelToGlobal(ms, ms->config.vanishingPointReticle.px, ms->config.vanishingPointReticle.py, thisZ, &thisVpBaseheight.px, &thisVpBaseheight.py, tArmP);

  double p_dist_thresh = 0.07;
  double dist_to_base = eePose::distance(tBaseP, thisVpBaseheight);
  // only load if we pass the distance test
  if (dist_to_base < p_dist_thresh) {
    cout << "Stream crop vanishing point to base test SUCCESS, accepting, dist_to_base, p_dist_thresh: " << dist_to_base << " " << p_dist_thresh << endl;
    tsi = setIsbIdx(ms, ms->config.sibCurIdx);
    if (tsi == NULL) {
      cout << "streamCropsAsFocusedClass: setIsbIdx returned null after distance check! Returning." << endl;
    } else {
    }
  } else {
    cout << "Stream crop vanishing point to base test FAILURE, skipping, dist_to_base, p_dist_thresh: " << dist_to_base << " " << p_dist_thresh << endl;
    return;
  }

  if ( ms->config.focusedClass > -1 ) {
    for (int c = 0; c < ms->config.bTops.size(); c++) {
      // XXX TODO want to annotate these crops with a yaml file that includes pose and time
      string thisLabelName = ms->config.focusedClassLabel;
      Mat thisTarget = tsi->image;
      Mat crop = thisTarget(cv::Rect(ms->config.bTops[c].x, ms->config.bTops[c].y, ms->config.bBots[c].x-ms->config.bTops[c].x, ms->config.bBots[c].y-ms->config.bTops[c].y));
      char buf[1024];
      string this_crops_path = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/detectionCrops/";

      ros::Time thisNow = ros::Time::now();
      sprintf(buf, "%s%s%s_%f.png", this_crops_path.c_str(), thisLabelName.c_str(), ms->config.run_prefix.c_str(), thisNow.toSec());
      // no compression!
      std::vector<int> args;
      args.push_back(CV_IMWRITE_PNG_COMPRESSION);
      args.push_back(ms->config.globalPngCompression);
      imwrite(buf, crop, args);
      ms->config.cropCounter++;
    }
  } else {
  }
}
END_WORD
REGISTER_WORD(StreamCropsAsFocusedClass)

WORD(StreamCenterCropAsFocusedClass)
virtual void execute(std::shared_ptr<MachineState> ms)       {

  streamImage * tsi = setIsbIdxNoLoad(ms, ms->config.sibCurIdx);
  if (tsi == NULL) {
    cout << "streamCenterCropAsFocusedClass: setIsbIdxNoLoad returned null. Returning." << endl;
  } else {
  }

  eePose tArmP, tBaseP;
  int success = getStreamPoseAtTime(ms, tsi->time, &tArmP, &tBaseP);

  double thisZ = tArmP.pz - tBaseP.pz;
  eePose thisVpBaseheight;
  thisVpBaseheight.pz = tBaseP.pz + (thisZ - convertHeightIdxToLocalZ(ms, ms->config.mappingHeightIdx));
  pixelToGlobal(ms, ms->config.vanishingPointReticle.px, ms->config.vanishingPointReticle.py, thisZ, &thisVpBaseheight.px, &thisVpBaseheight.py, tArmP);

  double p_dist_thresh = 0.07;
  double dist_to_base = eePose::distance(tBaseP, thisVpBaseheight);
  // only load if we pass the distance test
  if (dist_to_base < p_dist_thresh) {
    cout << "Stream crop vanishing point to base test SUCCESS, accepting, dist_to_base, p_dist_thresh: " << dist_to_base << " " << p_dist_thresh << endl;
    tsi = setIsbIdx(ms, ms->config.sibCurIdx);
    if (tsi == NULL) {
      cout << "streamCenterCropAsFocusedClass: setIsbIdx returned null after distance check! Returning." << endl;
    } else {
    }
  } else {
    cout << "Stream crop vanishing point to base test FAILURE, skipping, dist_to_base, p_dist_thresh: " << dist_to_base << " " << p_dist_thresh << endl;
    return;
  }

  if ( ms->config.focusedClass > -1 ) {
    int c = ms->config.pilotClosestBlueBoxNumber;
    if ( (c > -1) && (c < ms->config.bTops.size()) ) {
      // XXX TODO want to annotate these crops with a yaml file that includes pose and time
      string thisLabelName = ms->config.focusedClassLabel;
      Mat thisTarget = tsi->image;
      Mat crop = thisTarget(cv::Rect(ms->config.bTops[c].x, ms->config.bTops[c].y, ms->config.bBots[c].x-ms->config.bTops[c].x, ms->config.bBots[c].y-ms->config.bTops[c].y));
      char buf[1024];
      string this_crops_path = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/detectionCrops/";

      ros::Time thisNow = ros::Time::now();
      sprintf(buf, "%s%s%s_%f.png", this_crops_path.c_str(), thisLabelName.c_str(), ms->config.run_prefix.c_str(), thisNow.toSec());
      // no compression!
      std::vector<int> args;
      args.push_back(CV_IMWRITE_PNG_COMPRESSION);
      args.push_back(ms->config.globalPngCompression);
      imwrite(buf, crop, args);
      ms->config.cropCounter++;
    }
  } else {
  }
}
END_WORD
REGISTER_WORD(StreamCenterCropAsFocusedClass)

WORD(SaveAccumulatedStreamToServoImage)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  cout << "saveAccumulatedStreamToServoImage ";
  if ((ms->config.focusedClass > -1) && (ms->config.accumulatedStreamImageBytes.rows >1) && (ms->config.accumulatedStreamImageBytes.cols > 1)) {
    string thisLabelName = ms->config.classLabels[ms->config.focusedClass];

    char buf[1000];
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
virtual void execute(std::shared_ptr<MachineState> ms)       {

  cout << "Searching image stream for servo images..." << endl;

  int tNumHeights = ms->config.hmWidth;
  for (int hIdx = tNumHeights-1; hIdx > -1; hIdx--) {
    ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("saveAccumulatedStreamToServoImage");
    ms->pushWord("streamedAccumulatedDensity");

    ms->pushWord(std::make_shared<IntegerWord>(hIdx));
    ms->pushWord("iterateIsbAndAccumulateHeightImages");

    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord(std::make_shared<IntegerWord>(hIdx));
    ms->pushWord("changeToHeight"); 
    ms->pushWord("rewindImageStreamBuffer"); 
    ms->pushWord("resetAccumulatedStreamImage");
  }
}
END_WORD
REGISTER_WORD(IntegrateImageStreamBufferServoImages)

WORD(ResetAccumulatedStreamImage)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  resetAccumulatedStreamImage(ms);
}
END_WORD
REGISTER_WORD(ResetAccumulatedStreamImage)

WORD(IterateIsbAndAccumulateHeightImages)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  streamImage * tsi = setIsbIdxNoLoad(ms, ms->config.sibCurIdx);
  if (tsi == NULL) {
    cout << "iterateIsbAndAccumulateHeightImages: setIsbIdxNoLoad returned null. Returning." << endl;
  } else {
  }

  shared_ptr<Word> hWord = ms->popWord();

  if (hWord == NULL) {
    cout << "oops, iterateIsbAndAccumulateHeightImages requires an argument..." << endl;
    ms->clearStack();
  } else {
  }
  std::shared_ptr<IntegerWord> hIntWord = std::dynamic_pointer_cast<IntegerWord>(hWord);


  int tNumHeights = ms->config.hmWidth;
  int thisHeightIdx =  hIntWord->value();
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

    tsi = setIsbIdx(ms, ms->config.sibCurIdx);
    if (tsi == NULL) {
      cout << "iterateIsbAndAccumulateHeightImages: setIsbIdx returned null after pose check! Returning." << endl;
    } else {
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
  int nextIdx = ms->config.sibCurIdx + 1;
  cout << "iterateIsbAndAccumulateHeightImages incrementing to " << nextIdx << endl;
  if ( (nextIdx > -1) && (nextIdx < ms->config.streamImageBuffer.size()) ) {
    streamImage * result = setIsbIdx(ms, nextIdx);  
    if (result == NULL) {
      cout << "iterateIsbAndAccumulateHeightImages increment failed :(, nextIdx: " << nextIdx << endl;
      ms->clearStack();
    } else {
    }
  } else {
    recall = 0;
  }

  if (recall) {
    ms->pushWord(std::make_shared<IntegerWord>(thisHeightIdx));
    ms->pushWord("iterateIsbAndAccumulateHeightImages");
  } else {
    cout << "iterateIsbAndAccumulateHeightImages reached the end of the image stream buffer." << endl;
  }
}
END_WORD
REGISTER_WORD(IterateIsbAndAccumulateHeightImages)


}
