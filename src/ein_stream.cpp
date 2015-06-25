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
  ros::NodeHandle n("~");
  image_transport::ImageTransport it(n);
  int cfClass = ms->config.focusedClass;
  if ((cfClass > -1) && (cfClass < ms->config.classLabels.size())) {
    string this_label_name = ms->config.classLabels[cfClass]; 
    string this_raw_path = ms->config.data_directory + "/objects/" + this_label_name + "/raw/";
    string this_image_path = ms->config.data_directory + "/objects/" + this_label_name + "/raw/images/";
    string this_pose_path = ms->config.data_directory + "/objects/" + this_label_name + "/raw/pose/";
    string this_range_path = ms->config.data_directory + "/objects/" + this_label_name + "/raw/range/";
    mkdir(this_raw_path.c_str(), 0777);
    mkdir(this_image_path.c_str(), 0777);
    mkdir(this_pose_path.c_str(), 0777);
    mkdir(this_range_path.c_str(), 0777);
    ms->config.sensorStreamOn = 1;

    // turn that queue size up!
    ms->config.epState =   n.subscribe("/robot/limb/" + ms->config.left_or_right_arm + "/endpoint_state", 100, endpointCallback);
    ms->config.eeRanger =  n.subscribe("/robot/range/" + ms->config.left_or_right_arm + "_hand_range/state", 100, rangeCallback);
    ms->config.image_sub = it.subscribe(ms->config.image_topic, 30, imageCallback);
    cout << "Activating sensor stream." << endl;
  } else {
    cout << "Cannot activate sensor stream: invalid focused class." << endl;
  } 
}
END_WORD
REGISTER_WORD(ActivateSensorStreaming)

WORD(DeactivateSensorStreaming)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ros::NodeHandle n("~");
  image_transport::ImageTransport it(n);
  ms->config.sensorStreamOn = 0;
  // restore those queue sizes to defaults.
  ms->config.epState =   n.subscribe("/robot/limb/" + ms->config.left_or_right_arm + "/endpoint_state", 1, endpointCallback);
  ms->config.eeRanger =  n.subscribe("/robot/range/" + ms->config.left_or_right_arm + "_hand_range/state", 1, rangeCallback);
  ms->config.image_sub = it.subscribe(ms->config.image_topic, 1, imageCallback);

  cout << "deactivateSensorStreams: About to write batches... ";
  int cfClass = ms->config.focusedClass;
  if ((cfClass > -1) && (cfClass < ms->config.classLabels.size())) {
    writeRangeBatchAsClass(ms, cfClass);	
    writePoseBatchAsClass(ms, cfClass);	
    cout << "Wrote batches." << endl;
  } else {
    cout << "Did not write batches, invalid focused class." << endl;
  } 
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

WORD(IntegrateImageStreamBuffer)
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

  setIsbIdx(ms, 0);

  for (int i = 0; i < ms->config.streamImageBuffer.size(); i++) {
    ms->pushWord("waitUntilImageCallbackReceived");
    ms->pushWord("incrementImageStreamBuffer");
    ms->pushWord("streamCropsAsFocusedClass");
    ms->pushWord("goFindBlueBoxes"); 
    ms->pushWord("streamedDensity"); 
  }
}
END_WORD
REGISTER_WORD(IntegrateImageStreamBuffer)

WORD(IncrementImageStreamBuffer)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  int nextIdx = ms->config.sibCurIdx + 1;
  cout << "Incrementing to " << nextIdx << endl;
  if ( (nextIdx > -1) && (nextIdx < ms->config.streamImageBuffer.size()) ) {
    streamImage * result = setIsbIdx(ms, nextIdx);  
    if (result == NULL) {
      cout << "increment failed :(" << endl;
    } else {
    }
  }
}
END_WORD
REGISTER_WORD(IncrementImageStreamBuffer)

WORD(StreamCropsAsFocusedClass)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  if ( ms->config.focusedClass > -1 ) {
    for (int c = 0; c < ms->config.bTops.size(); c++) {
      // XXX TODO want to annotate these crops with a yaml file that includes pose and time
      string thisLabelName = ms->config.focusedClassLabel;
      Mat thisTarget = ms->config.streamImageBuffer[ms->config.sibCurIdx].image;
      Mat crop = thisTarget(cv::Rect(ms->config.bTops[c].x, ms->config.bTops[c].y, ms->config.bBots[c].x-ms->config.bTops[c].x, ms->config.bBots[c].y-ms->config.bTops[c].y));
      char buf[1024];
      string this_crops_path = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/detectionCrops/";

      ros::Time thisNow = ros::Time::now();
      sprintf(buf, "%s%s%s_%f.png", this_crops_path.c_str(), thisLabelName.c_str(), ms->config.run_prefix.c_str(), thisNow.toSec());
      // no compression!
      std::vector<int> args;
      args.push_back(CV_IMWRITE_PNG_COMPRESSION);
      args.push_back(0);
      imwrite(buf, crop, args);
      ms->config.cropCounter++;
    }
  }
}
END_WORD
REGISTER_WORD(StreamCropsAsFocusedClass)


}
