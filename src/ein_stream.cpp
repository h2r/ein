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
  ms->config.shouldIDoIK = 0;
  ms->config.ikShare = 0.1;
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
  if (ms->config.streamRangeBuffer.size() >= ms->config.streamRangeBatchSize) {
    int cfClass = ms->config.focusedClass;
    if ((cfClass > -1) && (cfClass < ms->config.classLabels.size())) {
      writeRangeBatchAsClass(ms, cfClass);	
      writePoseBatchAsClass(ms, cfClass);	
      cout << "Wrote batches." << endl;
    } else {
      cout << "Did not write batches, invalid focused class." << endl;
    } 
  }
}
END_WORD
REGISTER_WORD(DeactivateSensorStreaming)

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

}
END_WORD
REGISTER_WORD(IntegrateRangeStreamBuffer)

WORD(IntegrateImageStreamBuffer)
virtual void execute(std::shared_ptr<MachineState> ms)
{
    setIsbIdx(ms, 0);

    ms->pushWord("incrementImageStreamBuffer");
    ms->pushWord("streamCropsAsFocusedClass");
    ms->pushWord("goFindBlueBoxes"); 
    ms->pushWord("streamedDensity"); 
}
END_WORD
REGISTER_WORD(IntegrateImageStreamBuffer)

WORD(IncrementImageStreamBuffer)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  int nextIdx = ms->config.sibCurIdx + 1;
  if ( (nextIdx > -1) && (nextIdx < ms->config.streamImageBuffer.size()) ) {
    setIsbIdx(ms, nextIdx);  
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
      Mat crop = ms->config.cam_img(cv::Rect(ms->config.bTops[c].x, ms->config.bTops[c].y, ms->config.bBots[c].x-ms->config.bTops[c].x, ms->config.bBots[c].y-ms->config.bTops[c].y));

      char buf[1000];
      string this_crops_path = ms->config.data_directory + "/objects/" + thisLabelName + "/rgb/";
      sprintf(buf, "%s%s%s_%d.png", this_crops_path.c_str(), thisLabelName.c_str(), ms->config.run_prefix.c_str(), ms->config.cropCounter);
      imwrite(buf, crop);
      ms->config.cropCounter++;
    }
  }
}
END_WORD
REGISTER_WORD(RecordAllExamplesFocusedClass)


