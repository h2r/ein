#include "ein_words.h"
#include "ein.h"

#include "qtgui/einwindow.h"
#include <boost/filesystem.hpp>
using namespace std;
using namespace boost::filesystem;


namespace ein_words {

WORD(SetTargetClass)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string className;
  GET_ARG(ms, StringWord, className);

  int class_idx = classIdxForName(ms, className);
  changeTargetClass(ms, class_idx);
}
END_WORD
REGISTER_WORD(SetTargetClass)

WORD(SetTargetClassToLastLabelLearned)
CODE(1179730)     // capslock + numlock + r
virtual void execute(std::shared_ptr<MachineState> ms) {
  for (int i = 0; i < ms->config.numClasses; i++) {
    if (ms->config.lastLabelLearned.compare(ms->config.classLabels[i]) == 0) {
      ms->config.targetClass = i;
      ms->config.focusedClass = ms->config.targetClass;
      ms->config.focusedClassLabel = ms->config.classLabels[ms->config.focusedClass];
      cout << "lastLabelLearned classLabels[targetClass]: " << ms->config.lastLabelLearned << " " << ms->config.classLabels[ms->config.targetClass] << endl;
      changeTargetClass(ms, ms->config.targetClass);
    }
  }

  ms->pushWord("drawMapRegisters"); // render register 1
  // ATTN 10
  //ms->pushWord(196360); // loadPriorGraspMemory
  //ms->pushWord(1179721); // set graspMemories from classGraspMemories
  switch (ms->config.currentPickMode) {
  case STATIC_PRIOR:
    {
      ms->pushWord(196360); // loadPriorGraspMemory
    }
    return;
  case LEARNING_ALGORITHMC:
  case LEARNING_SAMPLING:
    {
      ms->pushWord(1179721); // set graspMemories from classGraspMemories
      //ms->pushWord(196360); // loadPriorGraspMemory
    }
    break;
  case STATIC_MARGINALS:
    {
      ms->pushWord(1179721); // set graspMemories from classGraspMemories
      //ms->pushWord(196360); // loadPriorGraspMemory
    }
    return;
  default:
    {
      assert(0);
    }
    return;
  }
}
END_WORD
REGISTER_WORD(SetTargetClassToLastLabelLearned)


WORD(SetLastLabelLearned)
CODE(1179732)    // capslock + numlock + t 
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.lastLabelLearned = ms->config.focusedClassLabel;
  cout << "lastLabelLearned: " << ms->config.lastLabelLearned << endl;
}
END_WORD
REGISTER_WORD(SetLastLabelLearned)

WORD(TrainModels)
CODE(131142)     // capslock + f
virtual string description() {
  return "Rebuild the kNN model for the detectors.";
}

virtual void execute(std::shared_ptr<MachineState> ms)       {
  ms->config.classLabels.resize(0);
  ms->config.classPoseModels.resize(0);

  ms->pushWord("clearBlueBoxMemories");


  // snoop folders
  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  char buf[1024];
  sprintf(buf, "%s/objects/", ms->config.data_directory.c_str());
  dpdf = opendir(buf);
  if (dpdf != NULL){
    while (epdf = readdir(dpdf)){
      string thisFileName(epdf->d_name);

      string thisFullFileName(buf);
      thisFullFileName = thisFullFileName + "/" + thisFileName;

      struct stat buf2;
      stat(thisFullFileName.c_str(), &buf2);

      int itIsADir = S_ISDIR(buf2.st_mode);
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && itIsADir) {
        ms->config.classLabels.push_back(thisFileName);
        ms->config.classPoseModels.push_back("B");
      }
    }
  }

  if ((ms->config.classLabels.size() != ms->config.classPoseModels.size()) || (ms->config.classLabels.size() < 1)) {
    cout << "Label and pose model list size problem. Not proceeding to train." << endl;
    return;
  }

  cout << "Reinitializing and retraining. " << endl;
  for (int i = 0; i < ms->config.classLabels.size(); i++) {
    cout << ms->config.classLabels[i] << " " << ms->config.classPoseModels[i] << endl;
  }

  ms->config.rewrite_labels = 1;
  //ms->config.retrain_vocab = 0;
  ms->config.reextract_knn = 1;

  // delete things that will be reallocated
  if (ms->config.bowTrainer)
    delete ms->config.bowTrainer;
  if (ms->config.kNN)
    delete ms->config.kNN;

  for (int i = 0; i < ms->config.classPosekNNs.size(); i++) {
    if (ms->config.classPosekNNs[i])
      delete ms->config.classPosekNNs[i];
  }

  //  detectorsInit() will reset numClasses
  detectorsInit(ms);


  // XXX reset anything else
}
END_WORD
REGISTER_WORD(TrainModels)

CONFIG_GETTER_INT(NumClasses, ms->config.numClasses)
CONFIG_SETTER_INT(SetNumClasses, ms->config.numClasses)

WORD(PrintClassLabels)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  cout << "printClassLabels: " << ms->config.classLabels.size() << endl;

  for (int i = 0; i < ms->config.classLabels.size(); i++) {
    cout << i << ": " << ms->config.classLabels[i] << endl;
  }
}
END_WORD
REGISTER_WORD(PrintClassLabels)


WORD(PushClassLabelsReport)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  stringstream ss;

  ss << "classLabels: " << ms->config.classLabels.size() << endl;

  for (int i = 0; i < ms->config.classLabels.size(); i++) {
    ss << i << ": " << ms->config.classLabels[i] << endl;
  }
  shared_ptr<StringWord> outword = std::make_shared<StringWord>(ss.str());
  ms->pushWord(outword);
}
END_WORD
REGISTER_WORD(PushClassLabelsReport)

CONFIG_GETTER_STRING(FocusedClassLabel, ms->config.focusedClassLabel)

WORD(PushClassLabels)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  for (int i = 0; i < ms->config.classLabels.size(); i++) {
    shared_ptr<StringWord> outword = std::make_shared<StringWord>(ms->config.classLabels[i]);
    ms->pushWord(outword);
  }
}
END_WORD
REGISTER_WORD(PushClassLabels)

WORD(ClearClassLabels)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.classLabels.resize(0);
  ms->config.classPoseModels.resize(0);
  ms->pushWord("clearBlueBoxMemories");
  ms->config.numClasses = ms->config.classLabels.size();
}
END_WORD
REGISTER_WORD(ClearClassLabels)


WORD(SetClassLabels)
virtual void execute(std::shared_ptr<MachineState> ms)       {

  cout << "entering setClassLabels." << endl;

  vector<string> newLabels;

  int more_args = 1;
  while(more_args) {
    shared_ptr<Word> bWord = ms->popData();

    if (bWord == NULL) {
      cout << "oops, setClassLabels requires a number of StringWords followed by endArgs..." << endl;
      ms->clearStack();
      return;
    } else {
    }

    if (bWord->name().compare("endArgs") == 0) {
      cout << " found endArgs" << endl;
      more_args = 0;
      break;
    } else {
    }

    std::shared_ptr<StringWord> bStringWord = std::dynamic_pointer_cast<StringWord>(bWord);

    if (bStringWord == NULL) {
      cout << "rejecting a word, not resetting labels, and pausing the stack. probably forgot endArgs." << endl;
      ms->pushWord("pauseStackExecution");
      return;
    } else {
      string thisLabel = bStringWord->to_string();
      if (thisLabel.length() > 0) {
	cout << "accepting " << thisLabel << endl;
	newLabels.push_back(thisLabel);
      } else {
	cout << "found a string word of length 0, not resetting labels, and pausing the stack." << endl;
	ms->pushWord("pauseStackExecution");
	return;
      }
    }
  }

  if (newLabels.size() > 0) {
    ms->config.classLabels.resize(0);
    ms->config.classPoseModels.resize(0);
    ms->pushWord("clearBlueBoxMemories");
    for (int b = 0; b < newLabels.size(); b++) {
      string thisLabel = newLabels[b];
      ms->config.classLabels.push_back(thisLabel);
      ms->config.classPoseModels.push_back("B");
    }
    ms->config.numClasses = ms->config.classLabels.size();
  } else {
    cout << "didn't get any valid labels, clearing stack." << endl;
    ms->clearStack();
    return;
  }

  initRangeMaps(ms);
}
END_WORD
REGISTER_WORD(SetClassLabels)

WORD(TrainModelsFromLabels)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  ms->config.rewrite_labels = 1;
  //ms->config.retrain_vocab = 0;
  ms->config.reextract_knn = 1;

  // delete things that will be reallocated
  if (ms->config.bowTrainer)
    delete ms->config.bowTrainer;
  if (ms->config.kNN)
    delete ms->config.kNN;

  for (int i = 0; i < ms->config.classPosekNNs.size(); i++) {
    if (ms->config.classPosekNNs[i])
      delete ms->config.classPosekNNs[i];
  }

  //  detectorsInit() will reset numClasses
  detectorsInit(ms);
}
END_WORD
REGISTER_WORD(TrainModelsFromLabels)

WORD(TrainAndWriteFocusedClassKnn)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  int tfc = ms->config.focusedClass;

  if ( (tfc > -1) && (tfc < ms->config.classLabels.size()) ) {
  } else {
    cout << "trainAndWriteFocusedClassKnn: Invalid focused class, clearing stack." << endl;
    ms->clearStack();
    return;
  }

  string thisLabelName = ms->config.classLabels[tfc];
  Mat kNNfeatures;
  Mat kNNlabels;

  kNNGetFeatures(ms, ms->config.class_crops_path, ms->config.classLabels[tfc].c_str(), tfc, ms->config.grayBlur, kNNfeatures, kNNlabels, ms->config.sobel_sigma);
  
  string dir_path = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/knn/";
  mkdir(dir_path.c_str(), 0777);

  // write yaml
  FileStorage fsvO;
  string yaml_path = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/knn/knn.yml";
  cout << "trainAndWriteFocusedClassKnn: Writing: " << yaml_path << endl;
  fsvO.open(yaml_path, FileStorage::WRITE);
  fsvO << "knnFeatures" << kNNfeatures;
  fsvO.release();
}
END_WORD
REGISTER_WORD(TrainAndWriteFocusedClassKnn)

WORD(CreateCachedClassifierFromClassLabels)
virtual void execute(std::shared_ptr<MachineState> ms)       {

  ms->config.numClasses = ms->config.classLabels.size();

  Mat knnFeaturesAll;
  Mat knnLabelsAll;

  for (int idx = 0; idx < ms->config.classLabels.size(); idx++) {
    Mat knnFeaturesThese;
    string thisLabelName = ms->config.classLabels[idx];
    string yaml_path = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/knn/knn.yml";
    cout << "createCachedClassifierFromClassLabels: Reading: " << yaml_path << endl;
    FileStorage fsfI;
    fsfI.open(yaml_path, FileStorage::READ);

    if (fsfI.isOpened()) {
      cout << "opened features for class " << idx << endl;
      fsfI["knnFeatures"] >> knnFeaturesThese;
      knnFeaturesAll.push_back(knnFeaturesThese);
      for (int l = 0; l < knnFeaturesThese.rows; l++) {
	knnLabelsAll.push_back(idx);
      }
    } else {
      cout << "unable to open features for class " << idx << endl;
    }
    
    cout << knnFeaturesAll.size() << knnLabelsAll.size() << endl;
  }
  cout << "knnLabelsAll dimensions: " << knnLabelsAll.size().height << " by " << knnLabelsAll.size().width << endl;
  cout << "knnFeaturesAll dimensions: " << knnFeaturesAll.size().height << " by " << knnFeaturesAll.size().width << endl;

  cout << "Main kNN...";

  if ( (knnFeaturesAll.data == NULL) || (knnFeaturesAll.rows < 1) || (knnFeaturesAll.cols < 1) ) {
    cout << "There is a problem with kNN features, cannot initialize detector and files may be corrupt." << endl;
  } else {
    ms->config.kNN = new CvKNearest(knnFeaturesAll, knnLabelsAll);
    cout << "done." << endl;
    for (int i = 0; i < ms->config.numClasses; i++) {
      if (ms->config.classPoseModels[i].compare("G") == 0) {
	cout << "Class " << i << " kNN..." << ms->config.classPosekNNfeatures[i].size() << ms->config.classPosekNNlabels[i].size() << endl;
	ms->config.classPosekNNs[i] = new CvKNearest(ms->config.classPosekNNfeatures[i], ms->config.classPosekNNlabels[i]);
	cout << "Done" << endl;
      }
    }
  }

  initRangeMaps(ms);

  // save models
  {
    char vocabularyPath[1024];
    char featuresPath[1024];
    char labelsPath[1024];
    sprintf(vocabularyPath, "%s/objects/%s", ms->config.data_directory.c_str(), ms->config.vocab_file.c_str());
    sprintf(featuresPath, "%s/objects/%s", ms->config.data_directory.c_str(), ms->config.knn_file.c_str());
    sprintf(labelsPath, "%s/objects/%s", ms->config.data_directory.c_str(), ms->config.label_file.c_str());
    cout << "vocabularyPath: " << vocabularyPath << endl;
    cout << "featuresPath: " << featuresPath << endl;
    cout << "labelsPath: " << labelsPath << endl;
    {
      FileStorage fsvO;
      cout<<"Writing labels and pose models... " << labelsPath << " ...";
      fsvO.open(labelsPath, FileStorage::WRITE);
      fsvO << "labels" << ms->config.classLabels;
      fsvO << "poseModels" << ms->config.classPoseModels;
      fsvO.release();
      cout << "done." << endl;
    }
    // don't rewrite vocab, we didn't train it
    {
      FileStorage fsfO;
      cout<<"Writing features and labels... " << featuresPath << " ..." << endl;
      fsfO.open(featuresPath, FileStorage::WRITE);
      fsfO << "features" << knnFeaturesAll;
      fsfO << "labels" << knnLabelsAll;
    }
  }
}
END_WORD
REGISTER_WORD(CreateCachedClassifierFromClassLabels)

WORD(VisionCycleNoClassify)
CODE(196721)     // capslock + Q
virtual void execute(std::shared_ptr<MachineState> ms)       {
  ms->pushWord("mapEmptySpace");
  ms->pushWord("goFindBlueBoxes"); // blue boxes
  ms->pushCopies("density", 1); // density
  ms->pushWord("hover"); // blue boxes
}
END_WORD
REGISTER_WORD(VisionCycleNoClassify)

WORD(RecordExampleAsFocusedClass)
CODE(131148)     // capslock + l 
virtual void execute(std::shared_ptr<MachineState> ms)       {
  cout << "recordExamplesFocusedClass is deprecated." << endl;
  if ((ms->config.focusedClass > -1) && (ms->config.bTops.size() == 1)) {
    string thisLabelName = ms->config.focusedClassLabel;
    Mat crop = ms->config.cam_img(cv::Rect(ms->config.bTops[0].x, ms->config.bTops[0].y, ms->config.bBots[0].x-ms->config.bTops[0].x, ms->config.bBots[0].y-ms->config.bTops[0].y));
    char buf[1000];
    string this_crops_path = ms->config.data_directory + "/objects/" + thisLabelName + "/rgb/";

    ros::Time thisNow = ros::Time::now();
    sprintf(buf, "%s%s%s_%f.png", this_crops_path.c_str(), thisLabelName.c_str(), ms->config.run_prefix.c_str(), thisNow.toSec());
    imwrite(buf, crop);
    ms->config.cropCounter++;
  }
}
END_WORD
REGISTER_WORD(RecordExampleAsFocusedClass)

WORD(RecordAllExamplesFocusedClass)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  cout << "recordAllExamplesFocusedClass is deprecated." << endl;
  if ( ms->config.focusedClass > -1 ) {
    for (int c = 0; c < ms->config.bTops.size(); c++) {
      string thisLabelName = ms->config.focusedClassLabel;
      Mat crop = ms->config.cam_img(cv::Rect(ms->config.bTops[c].x, ms->config.bTops[c].y, ms->config.bBots[c].x-ms->config.bTops[c].x, ms->config.bBots[c].y-ms->config.bTops[c].y));
      char buf[1000];
      string this_crops_path = ms->config.data_directory + "/objects/" + thisLabelName + "/rgb/";

      ros::Time thisNow = ros::Time::now();
      sprintf(buf, "%s%s%s_%f.png", this_crops_path.c_str(), thisLabelName.c_str(), ms->config.run_prefix.c_str(), thisNow.toSec());
      imwrite(buf, crop);
      ms->config.cropCounter++;
    }
  }
}
END_WORD
REGISTER_WORD(RecordAllExamplesFocusedClass)

WORD(SetRandomOrientationForPhotospin)
CODE(1310722)     // capslock + numlock + "
virtual void execute(std::shared_ptr<MachineState> ms) {
  // this ensures that we explore randomly within each grasp gear sector
  double arcFraction = 0.125;
  double noTheta = arcFraction * 3.1415926 * ((drand48() - 0.5) * 2.0);
  ms->config.currentEEDeltaRPY.pz += noTheta;
}
END_WORD
REGISTER_WORD(SetRandomOrientationForPhotospin)

WORD(RgbScan)
CODE(131143)      // capslock + g
virtual void execute(std::shared_ptr<MachineState> ms)       {
  // ATTN 16

  ms->pushCopies('e', 5);
  ms->pushCopies('a', 5);
  ms->pushWord(196711); // photospin
  ms->pushCopies('q', 5);
  ms->pushWord(1245246); // uniformly sample height
  ms->pushWord(196711); // photospin
  ms->pushCopies('q', 5);
  ms->pushWord(1245246); // uniformly sample height
  ms->pushWord(196711); // photospin
  ms->pushCopies('d', 5);
  ms->pushWord(1245246); // uniformly sample height
  ms->pushWord(196711); // photospin
  ms->pushCopies('d', 5);
  ms->pushWord(1245246); // uniformly sample height
  ms->pushWord(196711); // photospin
  ms->pushCopies('e', 5);
  ms->pushWord(1245246); // uniformly sample height
  ms->pushWord(196711); // photospin
  ms->pushCopies('e', 5);
  ms->pushWord(1245246); // uniformly sample height
  ms->pushWord(196711); // photospin
  ms->pushCopies('a', 5);
  ms->pushWord(1245246); // uniformly sample height
  ms->pushWord(196711); // photospin
  ms->pushCopies('q', 5);
  ms->pushWord(1245246); // uniformly sample height
  ms->pushWord(196711); // photospin

  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  ms->pushWord(1245246); // uniformly sample height
  pushGridSign(ms, GRID_COARSE);
}
END_WORD
REGISTER_WORD(RgbScan)

WORD(SetPhotoPinHere)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.photoPinPose = pixelToGlobalEEPose(ms, ms->config.vanishingPointReticle.px, ms->config.vanishingPointReticle.py, ms->config.trueEEPose.position.z + ms->config.currentTableZ);
}
END_WORD
REGISTER_WORD(SetPhotoPinHere)

WORD(PutCameraOverPhotoPin)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // XXX TODO avoid two steps by using alternate pixelToGlobal and not waiting between
  eePose underVanishingPointReticle = pixelToGlobalEEPose(ms, ms->config.vanishingPointReticle.px, ms->config.vanishingPointReticle.py, ms->config.trueEEPose.position.z + ms->config.currentTableZ);
  eePose toMove = ms->config.photoPinPose.minusP(underVanishingPointReticle);
  eePose nextCurrentPose = ms->config.currentEEPose.plusP(toMove);
  ms->config.currentEEPose.px = nextCurrentPose.px;
  ms->config.currentEEPose.py = nextCurrentPose.py;
}
END_WORD
REGISTER_WORD(PutCameraOverPhotoPin)

WORD(PhotoSpin)
CODE(196711)      // capslock + G
virtual void execute(std::shared_ptr<MachineState> ms) {
  for (int angleCounter = 0; angleCounter < ms->config.totalGraspGears; angleCounter++) {
    //ms->pushWord(131148); // save crop as focused class if there is only one
    ms->pushWord("recordAllExamplesFocusedClass");
    ms->pushWord(196721); // vision cycle no classify
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord("putCameraOverPhotoPin"); 
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord("setRandomOrientationForPhotospin"); 
    ms->pushWord("incrementGraspGear"); 
  }
  ms->pushWord("shiftIntoGraspGear1"); // change gear to 1
}
END_WORD
REGISTER_WORD(PhotoSpin)

WORD(SetTargetReticleToTheMaxMappedPosition)
CODE(1048678)  // numlock + f
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.trX = ms->config.rmcX + ms->config.rmDelta*(ms->config.maxX-ms->config.rmHalfWidth);
  ms->config.trY = ms->config.rmcY + ms->config.rmDelta*(ms->config.maxY-ms->config.rmHalfWidth);
}
END_WORD
REGISTER_WORD(SetTargetReticleToTheMaxMappedPosition)

WORD(ClassRangeMapFromRegister1)
virtual void execute(std::shared_ptr<MachineState> ms) {
  for (int y = 0; y < ms->config.rmWidth; y++) {
    for (int x = 0; x < ms->config.rmWidth; x++) {
      ms->config.rangeMap[x + y*ms->config.rmWidth] = ms->config.rangeMapReg1[x + y*ms->config.rmWidth];
    } 
  } 

  int tfc = ms->config.focusedClass;
  if ((tfc > -1) && (tfc < ms->config.classRangeMaps.size()) && (ms->config.classRangeMaps[tfc].rows > 1) && (ms->config.classRangeMaps[tfc].cols > 1)) {
    for (int y = 0; y < ms->config.rmWidth; y++) {
      for (int x = 0; x < ms->config.rmWidth; x++) {
	ms->config.classRangeMaps[tfc].at<double>(y,x) = ms->config.rangeMapReg1[x + y*ms->config.rmWidth];
      } 
    } 
    cout << "classRangeMapFromRegister1: focused class inside of bounds, " << tfc << " " << ms->config.classRangeMaps.size() << endl;
  } else {
    cout << "classRangeMapFromRegister1: focused class out of bounds, " << tfc << " " << ms->config.classRangeMaps.size() << endl;
  }
}
END_WORD
REGISTER_WORD(ClassRangeMapFromRegister1)

WORD(DownsampleIrScan)
CODE(1048690) // numlock + r
virtual void execute(std::shared_ptr<MachineState> ms) {
  // replace unsampled regions with the lowest z reading, highest reading in those maps because they are inverted
  // 
  double highestReading = -VERYBIGNUMBER;
  double highestEpsilonMassReading = -VERYBIGNUMBER;
  double readingFloor = -1;
  for (int rx = 0; rx < ms->config.rmWidth; rx++) {
    for (int ry = 0; ry < ms->config.rmWidth; ry++) {
      for (int rrx = rx*10; rrx < (rx+1)*10; rrx++) {
        for (int rry = ry*10; rry < (ry+1)*10; rry++) {
          if (ms->config.hiRangeMapMass[rrx + rry*ms->config.hrmWidth] > 0.0) {
            //if ((hiRangeMap[rrx + rry*ms->config.hrmWidth] > highestReading) && (ms->config.hiRangeMap[rrx + rry*ms->config.hrmWidth] >= readingFloor))
            if ((ms->config.hiRangeMap[rrx + rry*ms->config.hrmWidth] > highestEpsilonMassReading) && (ms->config.hiRangeMapMass[rrx + rry*ms->config.hrmWidth] > EPSILON))
              highestEpsilonMassReading = ms->config.hiRangeMap[rrx + rry*ms->config.hrmWidth];

            if ((ms->config.hiRangeMap[rrx + rry*ms->config.hrmWidth] > highestReading) && (ms->config.hiRangeMapMass[rrx + rry*ms->config.hrmWidth] > 0))
              highestReading = ms->config.hiRangeMap[rrx + rry*ms->config.hrmWidth];
          }
        }
      }
    }
  }


  if (highestReading <= -VERYBIGNUMBER) {
    highestReading = 0;
  }

	
  for (int rx = 0; rx < ms->config.rmWidth; rx++) {
    for (int ry = 0; ry < ms->config.rmWidth; ry++) {
      double thisSum = 0;
      double numSamples = 0;
      for (int rrx = rx*10; rrx < (rx+1)*10; rrx++) {
        for (int rry = ry*10; rry < (ry+1)*10; rry++) {
          numSamples += 1.0;
          if (ms->config.hiRangeMapMass[rrx + rry*ms->config.hrmWidth] > 0.0) {
            thisSum += ms->config.hiRangeMap[rrx + rry*ms->config.hrmWidth];
          } else {
            thisSum += 0;
          }
        }
      }
      ms->config.rangeMapReg1[rx + ry*ms->config.rmWidth] = thisSum/numSamples;
    }
  }
}
END_WORD
REGISTER_WORD(DownsampleIrScan)



WORD(ScanObject)
CODE(196708)     // capslock + D
virtual string description() {
  return "Scans an object, including the IR scan.";
}

virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "ENTERING WHOLE FOODS VIDEO MAIN." << endl;
  cout << "Program will pause shortly. Please adjust height for bounding box servo before unpausing." << endl;
  cout << "Program will pause a second time. Please adjust height for IR scan before unpausing." << endl;
  cout << "Program will pause a third time. Please remove any applied contrast agents." << endl;

  ms->config.eepReg2 = ms->config.beeHome;
  ms->config.eepReg4 = ms->config.beeHome;

  // so that closest servoing doesn't go into gradient servoing.
  ms->config.targetClass = -1;


  // this automatically changes learning mode
          
  if (0) {
    ms->pushWord("beginHeightLearning"); // begin bounding box learning

    ms->pushWord("changeToHeight1"); // change to height 1
    ms->pushWord("shiftIntoGraspGear1"); // change to first gear
  }

  if (0) {
    ms->pushWord("saveCurrentClassDepthAndGraspMaps"); // save current depth map to current class
    ms->pushWord("loadPriorGraspMemoryAnalytic");
    // set target class to the lastLabelLearned 
    ms->pushWord("setTargetClassToLastLabelLearned");
    ms->pushWord("trainModels"); // reinitialize and retrain everything
  }

  // set lastLabelLearned
  ms->pushWord("setLastLabelLearned");

  ms->pushWord("scanCentered"); 
  ms->pushWord("setPhotoPinHere");

  // this is a good time to remove a contrast agent
  //ms->pushWord("pauseStackExecution"); // pause stack execution
  //ms->pushCopies("beep", 15); // beep
	  
  { // do density and gradient, save gradient, do medium scan in two directions, save range map
    pushGridSign(ms, GRID_COARSE);
    ms->pushWord("saveCurrentClassDepthAndGraspMaps"); // save current depth map to current class
    ms->pushWord("neutralScan"); // neutral scan 
    ms->pushWord("pauseStackExecution"); // pause stack execution
    ms->pushCopies("beep", 15); // beep
    pushGridSign(ms, GRID_COARSE);

    ms->pushWord("changeToHeight1"); // change to height 1

    {
      ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
      ms->pushWord("gradientServoPrep");
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      ms->pushWord("changeToHeight3"); // change to height 3
    }
    {
      ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
      ms->pushWord("gradientServoPrep");
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      ms->pushWord("changeToHeight2"); // change to height 2
    }
    {
      ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
      ms->pushWord("gradientServoPrep");
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      ms->pushWord("changeToHeight1"); // change to height 1
    }
    {
      ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
      ms->pushWord("gradientServoPrep");
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      ms->pushWord("changeToHeight0"); // change to height 0
    }
  }

  // ATTN 3
  // start NO bag routine
  ms->pushWord("initializeAndFocusOnNewClass"); //  make a new class

  ms->pushWord("synchronicServo"); // synchronic servo
  ms->pushWord("synchronicServoTakeClosest"); // synchronic servo take closest
  ms->pushWord("sampleHeight"); 

  ms->pushWord("pauseStackExecution"); // pause stack execution
  ms->pushCopies("beep", 15); // beep

  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  ms->pushWord("shiftIntoGraspGear1"); // change to first gear
  ms->pushWord("changeToHeight2"); // change to height 2
  pushGridSign(ms, GRID_COARSE);

  ms->pushWord("changeToCounterTable"); // change to counter table
  ms->pushWord("shiftIntoGraspGear1"); // change to first gear
  ms->pushWord('k'); // open gripper
}
END_WORD
REGISTER_WORD(ScanObject)


WORD(PrepareForSearch)
CODE(1114150)     // numlock + &
virtual void execute(std::shared_ptr<MachineState> ms) {
  // XXX this should be computed here from the ir sensor offset
  ms->config.currentEEPose.px = ms->config.rmcX + ms->config.drX;
  ms->config.currentEEPose.py = ms->config.rmcY + ms->config.drY;
}
END_WORD
REGISTER_WORD(PrepareForSearch)
 

WORD(TurnOnRecordRangeMap)
CODE(1048683) 
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.recordRangeMap = 1;
}
END_WORD
REGISTER_WORD(TurnOnRecordRangeMap)

WORD(TurnOffScanning)
CODE(1048684)     // numlock + l
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.recordRangeMap = 0;
}
END_WORD
REGISTER_WORD(TurnOffScanning)

WORD(SetRangeMapCenterFromCurrentEEPose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Set rmcX and rmcY from ms->config.currentEEPose." << endl;
  ms->config.rmcX = ms->config.currentEEPose.px;
  ms->config.rmcY = ms->config.currentEEPose.py;
  //ms->config.rmcZ = ms->config.currentEEPose.pz - ms->config.eeRange;
}
END_WORD
REGISTER_WORD(SetRangeMapCenterFromCurrentEEPose)

WORD(InitDepthScan)
CODE(1048695) // numlock + w
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Set rmcX and rmcY. Resetting maps. " << ms->config.rmcX << " " << ms->config.trueEEPose.position.x << endl;
  ms->config.rmcX = ms->config.trueEEPose.position.x;
  ms->config.rmcY = ms->config.trueEEPose.position.y;
  ms->config.rmcZ = ms->config.trueEEPose.position.z - ms->config.eeRange;

  clearAllRangeMaps(ms);
}
END_WORD
REGISTER_WORD(InitDepthScan)

WORD(ClearAllRangeMaps)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Clearing all range maps." << endl;
  clearAllRangeMaps(ms);
}
END_WORD
REGISTER_WORD(ClearAllRangeMaps)




WORD(NeutralScan)
CODE(1048622) // numlock + .
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("shiftIntoGraspGear1"); 
  ms->pushWord("cruisingSpeed");
  ms->pushWord("neutralScanA");
  ms->pushWord("rasterScanningSpeed");
}
END_WORD
REGISTER_WORD(NeutralScan)

WORD(NeutralScanA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Entering neutral scan." << endl;
  double lineSpeed = GRID_COARSE;//GRID_MEDIUM;//GRID_COARSE;
  double betweenSpeed = GRID_COARSE;//GRID_MEDIUM;//GRID_COARSE;

  ms->pushWord("turnOffScanning"); // turn off scanning
  scanXdirection(ms, lineSpeed, betweenSpeed); // load scan program
  ms->pushWord("prepareForSearch"); // prepare for search
  ms->pushWord("rasterScanningSpeed"); 

  ms->pushWord("turnOnRecordRangeMap"); // turn on scanning

  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies('s',10);
  ms->pushWord("approachSpeed");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies('a',6);
  ms->pushCopies('q',4);
  ms->pushWord("turnOnRecordRangeMap"); // turn on scanning
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftGraspGear"); 


  ms->pushWord("fullRender"); // full render
  ms->pushWord("paintReticles"); // render reticle
  ms->pushWord("shiftIntoGraspGear1"); // change to first gear
  ms->pushWord("drawMapRegisters"); // render register 1
  ms->pushWord("downsampleIrScan"); // load map to register 1
  {
    ms->pushWord("setTargetReticleToTheMaxMappedPosition"); // target best grasp
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord("shiftIntoGraspGear1"); // change to first gear
  }
  ms->pushWord("selectBestAvailableGrasp"); // find best grasp

  ms->pushWord("cruisingSpeed"); 
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies('w',10);
  ms->pushWord("departureSpeed");

  ms->pushWord("turnOffScanning"); // turn off scanning
  scanXdirection(ms, lineSpeed, betweenSpeed); // load scan program
  ms->pushWord("prepareForSearch"); // prepare for search

  ms->pushWord("turnOnRecordRangeMap"); // turn on scanning
  ms->pushWord("initDepthScan"); // clear scan history
  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("shiftIntoGraspGear1"); 
}
END_WORD
REGISTER_WORD(NeutralScanA)

WORD(NeutralScanB)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Entering neutralScanB." << endl;
  double lineSpeed = ms->config.bDelta;
  double betweenSpeed = ms->config.bDelta;

  ms->pushWord("cruisingSpeed");
  scanXdirection(ms, lineSpeed, betweenSpeed); // load scan program
  ms->pushWord("rasterScanningSpeed");
}
END_WORD
REGISTER_WORD(NeutralScanB)

WORD(NeutralScanH)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Entering HALF neutral scan." << endl;
  double lineSpeed = GRID_COARSE;//GRID_MEDIUM;//GRID_COARSE;
  double betweenSpeed = GRID_COARSE;//GRID_MEDIUM;//GRID_COARSE;

  ms->pushWord("fullRender"); // full render
  ms->pushWord("paintReticles"); // render reticle
  ms->pushWord("shiftIntoGraspGear1"); // change to first gear
  ms->pushWord("drawMapRegisters"); // render register 1
  ms->pushWord("downsampleIrScan"); // load map to register 1
  {
    ms->pushWord("setTargetReticleToTheMaxMappedPosition"); // target best grasp
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord("shiftIntoGraspGear1"); // change to first gear
  }
  ms->pushWord("selectBestAvailableGrasp"); // find best grasp

  ms->pushWord("turnOffScanning"); // turn off scanning
  scanXdirection(ms, lineSpeed, betweenSpeed); // load scan program
  ms->pushWord("prepareForSearch"); // prepare for search

  ms->pushWord("turnOnRecordRangeMap"); // turn on scanning
  ms->pushWord("initDepthScan"); // clear scan history
  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("shiftIntoGraspGear1"); 
}
END_WORD
REGISTER_WORD(NeutralScanH)

WORD(SaveAerialGradientMap)
CODE(196730)      // capslock + Z
virtual void execute(std::shared_ptr<MachineState> ms) {
  Size sz = ms->config.objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;
        
  cout << "save aerial gradient ";
  if ((ms->config.focusedClass > -1) && (ms->config.frameGraySobel.rows >1) && (ms->config.frameGraySobel.cols > 1)) {
    string thisLabelName = ms->config.focusedClassLabel;

    char buf[1000];
    string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/servoCrops/";
    string this_range_path;

    // ATTN 16
    switch (ms->config.currentThompsonHeightIdx) {
    case 0:
      {
        this_range_path = dirToMakePath + "aerialHeight0Gradients.yml";
      }
      break;
    case 1:
      {
        this_range_path = dirToMakePath + "aerialHeight1Gradients.yml";
      }
      break;
    case 2:
      {
        this_range_path = dirToMakePath + "aerialHeight2Gradients.yml";
      }
      break;
    case 3:
      {
        this_range_path = dirToMakePath + "aerialHeight3Gradients.yml";
      }
      break;
    default:
      {
        assert(0);
        break;
      }
    }

    mkdir(dirToMakePath.c_str(), 0777);

    //int hbb = ms->config.pilotTargetBlueBoxNumber;
    //int hbb = 0;

    int topCornerX = ms->config.reticle.px - (ms->config.aerialGradientReticleWidth/2);
    int topCornerY = ms->config.reticle.py - (ms->config.aerialGradientReticleWidth/2);
    int crows = ms->config.aerialGradientReticleWidth;
    int ccols = ms->config.aerialGradientReticleWidth;

    //int crows = ms->config.bBots[hbb].y - ms->config.bTops[hbb].y;
    //int ccols = ms->config.bBots[hbb].x - ms->config.bTops[hbb].x;
    int maxDim = max(crows, ccols);
    int tRy = (maxDim-crows)/2;
    int tRx = (maxDim-ccols)/2;
    Mat gCrop(maxDim, maxDim, ms->config.frameGraySobel.type());

    cout << "crows ccols: " << crows << " " << ccols << " ";

    int nanDisco = 0;

    for (int x = 0; x < maxDim; x++) {
      for (int y = 0; y < maxDim; y++) {
        int tx = x - tRx;
        int ty = y - tRy;
        int tCtx = topCornerX + tx;
        int tCty = topCornerY + ty;
        if ( (tx >= 0 && ty >= 0 && ty < crows && tx < ccols) &&
             (tCtx > 0) && (tCty > 0) && (tCtx < imW) && (tCty < imH) ) {
          //gCrop.at<double>(y, x) = ms->config.frameGraySobel.at<double>(ms->config.bTops[hbb].y + ty, ms->config.bTops[hbb].x + tx);
          gCrop.at<double>(y, x) = ms->config.frameGraySobel.at<double>(tCty, tCtx);
        } else {
          gCrop.at<double>(y, x) = 0.0;
        }

	if (isFiniteNumber(gCrop.at<double>(y, x))) {
	} else {
	  nanDisco = 1;
          gCrop.at<double>(y, x) = 0.0;
	}
      }
    }

    if (nanDisco == 1) {
      ROS_ERROR_STREAM("saveAerialGradientMap: NaN discovered. Setting to 0."); 
    } else {
    }
  

    Size toBecome(ms->config.aerialGradientWidth, ms->config.aerialGradientWidth);
    cout << "about to resize to " << toBecome << endl;

    cv::resize(gCrop, gCrop, toBecome);


    FileStorage fsvO;
    cout << "capslock + Z: Writing: " << this_range_path << endl;

    fsvO.open(this_range_path, FileStorage::WRITE);

    // ATTN 16
    switch (ms->config.currentThompsonHeightIdx) {
    case 0:
      {
        fsvO << "aerialHeight0Gradients" << gCrop;
	ms->config.classHeight0AerialGradients[ms->config.focusedClass] = gCrop.clone();
      }
      break;
    case 1:
      {
        fsvO << "aerialHeight1Gradients" << gCrop;
	ms->config.classHeight1AerialGradients[ms->config.focusedClass] = gCrop.clone();
      }
      break;
    case 2:
      {
        fsvO << "aerialHeight2Gradients" << gCrop;
	ms->config.classHeight2AerialGradients[ms->config.focusedClass] = gCrop.clone();
      }
      break;
    case 3:
      {
        fsvO << "aerialHeight3Gradients" << gCrop;
	ms->config.classHeight3AerialGradients[ms->config.focusedClass] = gCrop.clone();
      }
      break;
    default:
      {
        assert(0);
      }
      break;
    }
    fsvO.release();
  } else {
  } 
}
END_WORD
REGISTER_WORD(SaveAerialGradientMap)

WORD(InitializeAndFocusOnNewClass)
CODE(196720)     // capslock + P
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.focusedClass = ms->config.classLabels.size();
  ms->config.targetClass = ms->config.focusedClass;

  ros::Time thisNow = ros::Time::now();
  string formattedTime = formatTime(thisNow);
  stringstream buf;
  buf << ms->config.scan_group << "autoClass_" << ms->config.robot_serial << "_" <<  ms->config.left_or_right_arm << "_" << formattedTime;
  string thisLabelName = buf.str();

  string thisLabelNameGroup = ms->config.scan_group;

  ms->config.classPoseModels.push_back("B");
  {
    string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelNameGroup + "/";
    if ( boost::filesystem::exists(dirToMakePath) ) {
      cout << "Group folder exists: " << dirToMakePath << endl;
    } else {
      mkdir(dirToMakePath.c_str(), 0777);
    }
  }
  bool collision = 1;
  int suffix_counter = 0;
  string the_suffix = "";
  while (collision) {
    string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + the_suffix + "/";
    if ( boost::filesystem::exists(dirToMakePath) ) {
      cout << "Whole label name already exists: " << dirToMakePath << endl << "Looking for a name that isn't in use..." << endl;
      stringstream buf1;
      buf1 << "." << suffix_counter;
      the_suffix = buf1.str(); 
      suffix_counter++;
      collision = 1;
    } else {
      mkdir(dirToMakePath.c_str(), 0777);
      collision = 0;
      thisLabelName = thisLabelName + the_suffix;  
    }
  }
  ms->config.focusedClassLabel = thisLabelName;
  ms->config.classLabels.push_back(thisLabelName);
  ms->config.numClasses = ms->config.classLabels.size();

  initRangeMaps(ms);
  guardGraspMemory(ms);
  guardHeightMemory(ms);

  int idx = ms->config.focusedClass;
  string folderName = ms->config.data_directory + "/objects/" + ms->config.classLabels[idx] + "/";
  initClassFolders(ms, folderName);
}
END_WORD
REGISTER_WORD(InitializeAndFocusOnNewClass)


WORD(WriteFocusedClass)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int idx = ms->config.focusedClass;

  if ((idx > -1) && (idx < ms->config.classLabels.size())) {
    // do nothing
  } else {
    cout << "writeFocusedClass: invalid idx, not writing." << endl;
    return;
  }

  string outfolder = ms->config.data_directory + "/objects/" + ms->config.classLabels[idx] + "/";
  writeClassToFolder(ms, idx, outfolder);
}
END_WORD
REGISTER_WORD(WriteFocusedClass)

WORD(SaveCurrentClassDepthAndGraspMaps)
CODE(196705) // capslock + A
virtual void execute(std::shared_ptr<MachineState> ms) {
  // XXX TODO is this function even ever used anymore?
  if (ms->config.focusedClass > -1) {
    // initialize this if we need to
    guardGraspMemory(ms);
    guardHeightMemory(ms);

    string thisLabelName = ms->config.focusedClassLabel;

    string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/ir2d/";
    string this_range_path = dirToMakePath + "xyzRange.yml";

    Mat rangeMapTemp(ms->config.rmWidth, ms->config.rmWidth, CV_64F);
    for (int y = 0; y < ms->config.rmWidth; y++) {
      for (int x = 0; x < ms->config.rmWidth; x++) {
	rangeMapTemp.at<double>(y,x) = ms->config.rangeMapReg1[x + y*ms->config.rmWidth];
      } 
    } 

    mkdir(dirToMakePath.c_str(), 0777);

    FileStorage fsvO;
    cout << "capslock + A: Writing: " << this_range_path << endl;
    fsvO.open(this_range_path, FileStorage::WRITE);

    {
      fsvO << "graspZ" << "[" 
	<< ms->config.currentGraspZ 
      << "]";

      if (ms->config.classGraspZs.size() > ms->config.focusedClass) {
	ms->config.classGraspZs[ms->config.focusedClass] = ms->config.currentGraspZ;
      }
      if (ms->config.classGraspZsSet.size() > ms->config.focusedClass) {
	ms->config.classGraspZsSet[ms->config.focusedClass] = 1;
      }
    }

    fsvO << "rangeMap" << rangeMapTemp;
    copyGraspMemoryTriesToClassGraspMemoryTries(ms);
    fsvO << "graspMemoryTries1" << ms->config.classGraspMemoryTries1[ms->config.focusedClass];
    fsvO << "graspMemoryPicks1" << ms->config.classGraspMemoryPicks1[ms->config.focusedClass];
    fsvO << "graspMemoryTries2" << ms->config.classGraspMemoryTries2[ms->config.focusedClass];
    fsvO << "graspMemoryPicks2" << ms->config.classGraspMemoryPicks2[ms->config.focusedClass];
    fsvO << "graspMemoryTries3" << ms->config.classGraspMemoryTries3[ms->config.focusedClass];
    fsvO << "graspMemoryPicks3" << ms->config.classGraspMemoryPicks3[ms->config.focusedClass];
    fsvO << "graspMemoryTries4" << ms->config.classGraspMemoryTries4[ms->config.focusedClass];
    fsvO << "graspMemoryPicks4" << ms->config.classGraspMemoryPicks4[ms->config.focusedClass];

    copyHeightMemoryTriesToClassHeightMemoryTries(ms);
    fsvO << "heightMemoryTries" << ms->config.classHeightMemoryTries[ms->config.focusedClass];
    fsvO << "heightMemoryPicks" << ms->config.classHeightMemoryPicks[ms->config.focusedClass];

    fsvO.release();
  } 
}
END_WORD
REGISTER_WORD(SaveCurrentClassDepthAndGraspMaps)

WORD(ScanCentered)
virtual void execute(std::shared_ptr<MachineState> ms) {
  pushGridSign(ms, GRID_COARSE);
  ms->pushWord("rgbScan");
  ms->pushWord("rgbScan");
  ms->pushWord("rgbScan");
  ms->pushWord("rgbScan");
  ms->pushWord("rgbScan");
  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  ms->pushWord("synchronicServo"); 
  ms->pushWord("synchronicServoTakeClosest");
  ms->pushWord("fillClearanceMap");
  ms->config.bDelta = 0.001;
  ms->pushWord("setBoundingBoxModeToMapping"); 
}
END_WORD
REGISTER_WORD(ScanCentered)

WORD(StreamScanCentered)
virtual void execute(std::shared_ptr<MachineState> ms) {
  pushGridSign(ms, GRID_COARSE);
  ms->pushWord("cruisingSpeed");
  ms->pushWord("streamSpin");
  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  ms->pushWord("streamImageSpeed"); // w1 wait until at current position
  ms->pushWord("setBoundingBoxModeToMapping"); 
}
END_WORD
REGISTER_WORD(StreamScanCentered)

WORD(StreamSpin)
virtual void execute(std::shared_ptr<MachineState> ms) {
  for (int angleCounter = 0; angleCounter < ms->config.totalGraspGears; angleCounter++) {
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord("putCameraOverPhotoPin"); 
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    //ms->pushWord("setRandomOrientationForPhotospin"); 
    ms->pushWord("decrementGraspGear"); 
  }
  ms->pushWord("incrementGraspGear"); 
  ms->pushWord("shiftIntoGraspGear1"); // change gear to 1
}
END_WORD
REGISTER_WORD(StreamSpin)

WORD(SetTable)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.firstTableHeightTime = ros::Time::now();
  ms->config.mostRecentUntabledZLastValue = ms->config.mostRecentUntabledZ;
  ms->pushWord("setTableA");
}
END_WORD
REGISTER_WORD(SetTable)

WORD(SetTableA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ros::Time thisTableHeightTime = ros::Time::now();

  ms->config.mostRecentUntabledZLastValue = ((1.0-ms->config.mostRecentUntabledZDecay)*ms->config.mostRecentUntabledZ) + (ms->config.mostRecentUntabledZDecay*ms->config.mostRecentUntabledZLastValue);

  ms->config.oneTable = ms->config.mostRecentUntabledZLastValue;
  ms->config.rightTableZ = ms->config.oneTable;
  ms->config.leftTableZ = ms->config.oneTable;
  ms->config.bagTableZ = ms->config.oneTable;
  ms->config.counterTableZ = ms->config.oneTable;
  ms->config.pantryTableZ = ms->config.oneTable;
  ms->config.currentTableZ = ms->config.oneTable;

  if ( fabs(thisTableHeightTime.sec - ms->config.firstTableHeightTime.sec) > ms->config.mostRecentUntabledZWait) {
    // do nothing
  } else {
    double utZDelta = fabs(ms->config.mostRecentUntabledZ - ms->config.mostRecentUntabledZLastValue);
    ms->config.endThisStackCollapse = 1;
    ms->pushWord("setTableA");
    cout << "Looks like the table reading hasn't steadied for the wait of " << ms->config.mostRecentUntabledZWait << " ." << endl;
    cout << "  current, last, delta: " << ms->config.mostRecentUntabledZ << " " << ms->config.mostRecentUntabledZLastValue << " " << utZDelta << endl;
  } 
}
END_WORD
REGISTER_WORD(SetTableA)

WORD(SetIROffset)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("setIROffsetA");
  ms->pushWord("cruisingSpeed");

  
  ms->pushWord("integrateRangeStreamBuffer");

  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 

  ms->pushWord("neutralScanH");
  
  ms->pushWord("activateSensorStreaming"); 
  ms->pushWord("clearStreamBuffers"); 
  ms->pushWord("shutdownToSensorsAndMovement"); 
  
  ms->pushWord("setSisFlags"); 
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(1));


  ms->pushWord("iRCalibrationSpeed");
  ms->pushWord("lock3dGraspBase"); 
  cout << "Commencing HALF neutral scan for SetIROffset." << endl;

  ms->pushWord("disableDiskStreaming");
}
END_WORD
REGISTER_WORD(SetIROffset)

WORD(ZeroIROffset)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.gear0offset = Eigen::Quaternionf(0.0, 0.0, 0.0, 0.0);
  Eigen::Quaternionf crane2quat(ms->config.straightDown.qw, ms->config.straightDown.qx, ms->config.straightDown.qy, ms->config.straightDown.qz);
  ms->config.irGlobalPositionEEFrame = crane2quat.conjugate() * ms->config.gear0offset * crane2quat;
}
END_WORD
REGISTER_WORD(ZeroIROffset)

WORD(SetIROffsetA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // find the maximum in the map
  // find the coordinate of the maximum
  // compare the coordinate to the root position
  // adjust offset
  // if adjustment was large, recommend running again
  double minDepth = VERYBIGNUMBER;
  double maxDepth = 0;
  int minX=-1, minY=-1;
  int maxX=-1, maxY=-1;

  for (int rx = 0; rx < ms->config.hrmWidth; rx++) {
    for (int ry = 0; ry < ms->config.hrmWidth; ry++) {
      double thisDepth = ms->config.hiRangeMap[rx + ry*ms->config.hrmWidth];
      if (thisDepth < minDepth) {
	minDepth = thisDepth;
	minX = rx;
	minY = ry;
      }
      if (thisDepth > maxDepth) {
	maxDepth = thisDepth;
	maxX = rx;
	maxY = ry;
      }
    }
  }

  double offByX = ((minX-ms->config.hrmHalfWidth)*ms->config.hrmDelta);
  double offByY = ((minY-ms->config.hrmHalfWidth)*ms->config.hrmDelta);

  cout << "SetIROffsetA, ms->config.hrmHalfWidth minX minY offByX offByY: " << ms->config.hrmHalfWidth << " " << minX << " " << minY << " " << offByX << " " << offByY << endl;

  ms->config.gear0offset = Eigen::Quaternionf(0.0, 
    ms->config.gear0offset.x()+offByX, 
    ms->config.gear0offset.y()+offByY, 
    0.0167228); // z is from TF, good for depth alignment

  Eigen::Quaternionf crane2quat(ms->config.straightDown.qw, ms->config.straightDown.qx, ms->config.straightDown.qy, ms->config.straightDown.qz);
  ms->config.irGlobalPositionEEFrame = crane2quat.conjugate() * ms->config.gear0offset * crane2quat;
}
END_WORD
REGISTER_WORD(SetIROffsetA)

WORD(SetHeightReticles)
virtual void execute(std::shared_ptr<MachineState> ms) {

  int heightWaits = 100;
  int numPause = 4; 
  

  ms->pushWord("setHeightReticlesA");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", heightWaits);
  ms->pushWord("resetAccumulatedDensity");
  for (int pauseCounter = 0; pauseCounter < numPause; pauseCounter++){
    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
  }
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("changeToHeight3");

  ms->pushWord("setHeightReticlesA");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", heightWaits);
  ms->pushWord("resetAccumulatedDensity");
  for (int pauseCounter = 0; pauseCounter < numPause; pauseCounter++){
    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
  }
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("changeToHeight2");

  ms->pushWord("setHeightReticlesA");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", heightWaits);
  ms->pushWord("resetAccumulatedDensity");
  for (int pauseCounter = 0; pauseCounter < numPause; pauseCounter++){
    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
  }
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("changeToHeight1");

  ms->pushWord("setHeightReticlesA");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", heightWaits);
  ms->pushWord("resetAccumulatedDensity");
  for (int pauseCounter = 0; pauseCounter < numPause; pauseCounter++){
    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
  }
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("changeToHeight0");

}
END_WORD
REGISTER_WORD(SetHeightReticles)

WORD(PrintGlobalToPixel)
virtual void execute(std::shared_ptr<MachineState> ms) {
  {
    double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;
    int eX=0, eY=0;
    //globalToPixel(&eX, &eY, zToUse, ms->config.eepReg1.px, ms->config.eepReg1.py);
    globalToPixelPrint(ms, &eX, &eY, zToUse, ms->config.eepReg1.px, ms->config.eepReg1.py);
  }
}
END_WORD
REGISTER_WORD(PrintGlobalToPixel)

WORD(SetHeightReticlesA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int darkX = 0;
  int darkY = 0;
  findDarkness(ms, &darkX, &darkY);

  ms->config.pilotTarget.px = darkX;
  ms->config.pilotTarget.py = darkY;

  ms->config.heightReticles[ms->config.currentThompsonHeightIdx].px = darkX;
  ms->config.heightReticles[ms->config.currentThompsonHeightIdx].py = darkY;

  cout << "setHeightReticles,  currentThompsonHeightIdx: " << ms->config.currentThompsonHeightIdx << endl;
  eePose::print(ms->config.heightReticles[0]); cout << endl;
  eePose::print(ms->config.heightReticles[1]); cout << endl;
  eePose::print(ms->config.heightReticles[2]); cout << endl;
  eePose::print(ms->config.heightReticles[3]); cout << endl;
}
END_WORD
REGISTER_WORD(SetHeightReticlesA)

WORD(MoveCropToCenter)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Size sz = ms->config.accumulatedImage.size();
  int imW = sz.width;
  int imH = sz.height;

  ms->config.cropUpperLeftCorner.px = 320;
  ms->config.cropUpperLeftCorner.py = 200;

  baxter_core_msgs::OpenCamera ocMessage;
  ocMessage.request.name = ms->config.left_or_right_arm + "_hand_camera";
  ocMessage.request.settings.controls.resize(2);
  ocMessage.request.settings.controls[0].id = 105;
  ocMessage.request.settings.controls[0].value = ms->config.cropUpperLeftCorner.px;
  ocMessage.request.settings.controls[1].id = 106;
  ocMessage.request.settings.controls[1].value = ms->config.cropUpperLeftCorner.py;
  int testResult = ms->config.cameraClient.call(ocMessage);
}
END_WORD
REGISTER_WORD(MoveCropToCenter)

WORD(MoveCropToProperValue)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("subscribeCameraParameterTrackerToRosOut 0.25 waitForSeconds moveCropToProperValueNoUpdate 0.25 waitForSeconds unsubscribeCameraParameterTrackerToRosOut");
}
END_WORD
REGISTER_WORD(MoveCropToProperValue)


WORD(MoveCropToProperValueNoUpdate)
virtual void execute(std::shared_ptr<MachineState> ms) {
  baxter_core_msgs::OpenCamera ocMessage;
  ocMessage.request.name = ms->config.left_or_right_arm + "_hand_camera";
  ocMessage.request.settings.controls.resize(4);
  ocMessage.request.settings.controls[0].id = 105;
  ocMessage.request.settings.controls[0].value = ms->config.cropUpperLeftCorner.px;
  ocMessage.request.settings.controls[1].id = 106;
  ocMessage.request.settings.controls[1].value = ms->config.cropUpperLeftCorner.py;
  ocMessage.request.settings.controls[2].id = 100;
  ocMessage.request.settings.controls[2].value = ms->config.cameraExposure;
  ocMessage.request.settings.controls[3].id = 101;
  ocMessage.request.settings.controls[3].value = ms->config.cameraGain;
  int testResult = ms->config.cameraClient.call(ocMessage);
}
END_WORD
REGISTER_WORD(MoveCropToProperValueNoUpdate)

WORD(FixCameraLightingToAutomaticParameters)
virtual void execute(std::shared_ptr<MachineState> ms) {
  stringstream p;
  p << "subscribeCameraParameterTrackerToRosOut 0.25 waitForSeconds ";
  p << "unFixCameraLightingNoUpdate 0.5 waitForSeconds ";
  p << "cameraExposure cameraGain fixCameraLightingExposureGainNoUpdate ";
  p << "0.5 waitForSeconds unsubscribeCameraParameterTrackerToRosOut ";
  ms->evaluateProgram(p.str());
}
END_WORD
REGISTER_WORD(FixCameraLightingToAutomaticParameters)

WORD(FixCameraLightingExposureGainNoUpdate)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "fixCameraLightingExposureGain...";

  int fiWordVal = 0;
  int seWordVal = 0;
  GET_ARG(ms, IntegerWord, fiWordVal);
  GET_ARG(ms, IntegerWord, seWordVal);

  {
    int thisExposure = max(0, min(seWordVal,100));
    int thisGain = max(0, min(fiWordVal,100));
    baxter_core_msgs::OpenCamera ocMessage;
    ocMessage.request.name = ms->config.left_or_right_arm + "_hand_camera";
    ocMessage.request.settings.controls.resize(4);
    ocMessage.request.settings.controls[0].id = 105;
    ocMessage.request.settings.controls[0].value = ms->config.cropUpperLeftCorner.px;
    ocMessage.request.settings.controls[1].id = 106;
    ocMessage.request.settings.controls[1].value = ms->config.cropUpperLeftCorner.py;
    ocMessage.request.settings.controls[2].id = 100;
    ocMessage.request.settings.controls[2].value = thisExposure;
    ocMessage.request.settings.controls[3].id = 101;
    ocMessage.request.settings.controls[3].value = thisGain;
    int testResult = ms->config.cameraClient.call(ocMessage);

    cout << "setting camera values, Exposure: " << thisExposure << " Gain: " << thisGain << " testResult: " << testResult << endl;
  }

}
END_WORD
REGISTER_WORD(FixCameraLightingExposureGainNoUpdate)

WORD(FixCameraLightingExposureGain)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("subscribeCameraParameterTrackerToRosOut 0.5 waitForSeconds fixCameraLightingExposureGainNoUpdate 0.5 waitForSeconds unsubscribeCameraParameterTrackerToRosOut");

}
END_WORD
REGISTER_WORD(FixCameraLightingExposureGain)


WORD(SubscribeCameraParameterTrackerToRosOut)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // there are lots of messages sent, and we're interested in the
  // camera tracker messages.  So we need a big message buffer to make
  // sure we didn't drop any.
  ros::NodeHandle n("~");
  //ms->config.rosout_sub = n.subscribe("/rosout", 100, &MachineState::rosoutCallback, ms.get());
}
END_WORD
REGISTER_WORD(SubscribeCameraParameterTrackerToRosOut)


WORD(UnsubscribeCameraParameterTrackerToRosOut)
virtual void execute(std::shared_ptr<MachineState> ms) {
  //ms->config.rosout_sub.shutdown();
}
END_WORD
REGISTER_WORD(UnsubscribeCameraParameterTrackerToRosOut)



WORD(UnFixCameraLightingNoUpdate)
virtual void execute(std::shared_ptr<MachineState> ms) {
  baxter_core_msgs::OpenCamera ocMessage;
  ocMessage.request.name = ms->config.left_or_right_arm + "_hand_camera";
  ocMessage.request.settings.controls.resize(2);
  ocMessage.request.settings.controls[0].id = 105;
  ocMessage.request.settings.controls[0].value = ms->config.cropUpperLeftCorner.px;
  ocMessage.request.settings.controls[1].id = 106;
  ocMessage.request.settings.controls[1].value = ms->config.cropUpperLeftCorner.py;
  int testResult = ms->config.cameraClient.call(ocMessage);
}
END_WORD
REGISTER_WORD(UnFixCameraLightingNoUpdate)

WORD(UnFixCameraLighting)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("subscribeCameraParameterTrackerToRosOut 0.5 waitForSeconds unFixCameraLightingNoUpdate 0.5 waitForSeconds unsubscribeCameraParameterTrackerToRosOut");

}
END_WORD
REGISTER_WORD(UnFixCameraLighting)



WORD(MoveCropToCenterVanishingPoint)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Size sz = ms->config.accumulatedImage.size();
  int imW = sz.width;
  int imH = sz.height;

  double Vx = ms->config.vanishingPointReticle.px - (imW/2);
  double Vy = ms->config.vanishingPointReticle.py - (imH/2);

  ms->config.cropUpperLeftCorner.px += Vx;
  ms->config.cropUpperLeftCorner.py += Vy;
  ms->config.vanishingPointReticle.px -= Vx;
  ms->config.vanishingPointReticle.py -= Vy;

  cout << "MoveCropToCenterVanishingPoint Vx Vy: " << Vx << " " << Vy << endl;

  baxter_core_msgs::OpenCamera ocMessage;
  ocMessage.request.name = ms->config.left_or_right_arm + "_hand_camera";
  ocMessage.request.settings.controls.resize(2);
  ocMessage.request.settings.controls[0].id = 105;
  ocMessage.request.settings.controls[0].value = ms->config.cropUpperLeftCorner.px;
  ocMessage.request.settings.controls[1].id = 106;
  ocMessage.request.settings.controls[1].value = ms->config.cropUpperLeftCorner.py;
  int testResult = ms->config.cameraClient.call(ocMessage);
  //cout << "centerVanishingPoint testResult: " << testResult << endl;
  //cout << ocMessage.response.name << endl;
  //cout << ocMessage.response.name << " " << ocMessage.response.settings.controls.size() << endl;

  //cout << "MoveCropToCenterVanishingPoint moving region of interest and vanishing point. Recalibrate vanishing point, height reticles, and magnification factors." << endl;
}
END_WORD
REGISTER_WORD(MoveCropToCenterVanishingPoint)

WORD(MoveToSetVanishingPointHeightLow)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentEEPose.pz = ms->config.minHeight - ms->config.currentTableZ;
}
END_WORD
REGISTER_WORD(MoveToSetVanishingPointHeightLow)

WORD(MoveToSetVanishingPointHeightHigh)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentEEPose.pz = ((0.75*ms->config.maxHeight)+(0.25*ms->config.minHeight)) - ms->config.currentTableZ;
}
END_WORD
REGISTER_WORD(MoveToSetVanishingPointHeightHigh)

WORD(SetVanishingPoint)
virtual void execute(std::shared_ptr<MachineState> ms) {

  ms->config.setVanishingPointIterations = 0;
  // go low, wait
  ms->pushWord("setVanishingPointA");
  // is darkest point in current vp? loop here until it is so then rise and go to B
  ms->pushWord("setVanishingPointPrep");
}
END_WORD
REGISTER_WORD(SetVanishingPoint)

WORD(SetVanishingPointPrep)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("darkServo");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("moveToSetVanishingPointHeightLow");
}
END_WORD
REGISTER_WORD(SetVanishingPointPrep)

WORD(SetVanishingPointA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int numPause = 4;

  ms->config.setVanishingPointIterations++;
  ms->pushWord("setVanishingPointB");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", 100);
  ms->pushWord("resetAccumulatedDensity");
  for (int pauseCounter = 0; pauseCounter < numPause; pauseCounter++){
    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
  }
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("moveToSetVanishingPointHeightHigh");
}
END_WORD
REGISTER_WORD(SetVanishingPointA)

WORD(SetVanishingPointB)
virtual void execute(std::shared_ptr<MachineState> ms) {

  // where is the darkest point now? did it move? move vp to darkest point and possibly run again
  int darkX = 0;
  int darkY = 0;
  findDarkness(ms, &darkX, &darkY);

  int Px = darkX - ms->config.vanishingPointReticle.px;
  int Py = darkY - ms->config.vanishingPointReticle.py;

  ms->config.vanishingPointReticle.px = darkX;
  ms->config.vanishingPointReticle.py = darkY;
  
  cout << "setVanishingPoint Px Py: " << Px << " " << Py << endl;

  if (ms->config.setVanishingPointIterations > ms->config.setVanishingPointTimeout) {
    cout << "setVanishingPoint timed out, continuing..." << endl;
  }

  if ((fabs(Px) < ms->config.setVanishingPointPixelThresh) && (fabs(Py) < ms->config.setVanishingPointPixelThresh)) {
    cout << "vanishing point set, continuing." << endl;
  } else {
    cout << "vanishing point not set, adjusting more. " << ms->config.setVanishingPointIterations << " " << ms->config.setVanishingPointTimeout << endl;
    ms->pushWord("setVanishingPointA");
    ms->pushWord("setVanishingPointPrep");
  }
}
END_WORD
REGISTER_WORD(SetVanishingPointB)


WORD(SetMagnification)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int translationSteps = 5;
  int imCallsToWait = 10;

  int nudgeSteps = 4;

  // move back
  // adjust until close	
  // move back over then down 
  // adjust until close	
  // move over 
  // go to height
  translationSteps = 15;
  ms->pushCopies("xDown", translationSteps);
  ms->pushCopies("yDown", nudgeSteps);
  ms->pushWord("setMagnificationA");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", imCallsToWait);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("yUp", nudgeSteps);
  ms->pushCopies("xUp", translationSteps);
  ms->pushCopies("yDown", translationSteps);
  ms->pushCopies("xDown", nudgeSteps);
  ms->pushWord("setMagnificationB");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", imCallsToWait);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("xUp", nudgeSteps);
  ms->pushCopies("yUp", translationSteps);
  ms->pushWord("changeToHeight3");

  translationSteps = 15;
  ms->pushCopies("xDown", translationSteps);
  ms->pushCopies("yDown", nudgeSteps);
  ms->pushWord("setMagnificationA");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", imCallsToWait);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("yUp", nudgeSteps);
  ms->pushCopies("xUp", translationSteps);
  ms->pushCopies("yDown", translationSteps);
  ms->pushCopies("xDown", nudgeSteps);
  ms->pushWord("setMagnificationB");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", imCallsToWait);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("xUp", nudgeSteps);
  ms->pushCopies("yUp", translationSteps);
  ms->pushWord("changeToHeight2");

  translationSteps = 15;
  ms->pushCopies("xDown", translationSteps);
  ms->pushCopies("yDown", nudgeSteps);
  ms->pushWord("setMagnificationA");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", imCallsToWait);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("yUp", nudgeSteps);
  ms->pushCopies("xUp", translationSteps);
  ms->pushCopies("yDown", translationSteps);
  ms->pushCopies("xDown", nudgeSteps);
  ms->pushWord("setMagnificationB");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", imCallsToWait);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("xUp", nudgeSteps);
  ms->pushCopies("yUp", translationSteps);
  ms->pushWord("changeToHeight1");

  translationSteps = 10;
  ms->pushCopies("xDown", translationSteps);
  ms->pushCopies("yDown", nudgeSteps);
  ms->pushWord("setMagnificationA");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", imCallsToWait);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("yUp", nudgeSteps);
  ms->pushCopies("xUp", translationSteps);
  ms->pushCopies("yDown", translationSteps);
  ms->pushCopies("xDown", nudgeSteps);
  ms->pushWord("setMagnificationB");
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", imCallsToWait);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("xUp", nudgeSteps);
  ms->pushCopies("yUp", translationSteps);
  ms->pushWord("changeToHeight0");
}
END_WORD
REGISTER_WORD(SetMagnification)

WORD(SetMagnificationA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // adjust until close	

  int darkX = 0;
  int darkY = 0;
  findDarkness(ms, &darkX, &darkY);

  ms->config.pilotTarget.px = darkX;
  ms->config.pilotTarget.py = darkY;

  int magIters = 2000; 
  double magStep = 0.01;

  for (int i = 0; i < magIters; i++) {
    double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;
    int eX=0, eY=0;
    //globalToPixel(&eX, &eY, zToUse, ms->config.eepReg1.px, ms->config.eepReg1.py);
    globalToPixelPrint(ms, &eX, &eY, zToUse, ms->config.eepReg1.px, ms->config.eepReg1.py);

    // remember this is flipped!
    double Px = darkY - eY;
    double Py = darkX - eX;

    double xFlip = 1.0;
    double yFlip = 1.0;

    // remember x, y are swapped
    eePose thisFlipReticle = ms->config.heightReticles[ms->config.currentThompsonHeightIdx];
    if (darkX < thisFlipReticle.px) {
      yFlip = -1.0;
    }
    if (darkY < thisFlipReticle.py) {
      xFlip = -1.0;
    }

    cout << "about to adjust m_x, darkX eX Px xFlip darkY eY Py yFlip: " << darkX << " " << eX << " " << Px << " " << xFlip << " " << darkY << " " << eY << " " << Py << " " << yFlip << " ";

    // only do x
    if ((Px*xFlip) > 0) {
      ms->config.m_x += .01;
      ms->config.m_x_h[ms->config.currentThompsonHeightIdx] = ms->config.m_x;
      cout << "m_x++ ";
    } else if ((Px*xFlip) < 0) {
      ms->config.m_x -= .01;
      ms->config.m_x_h[ms->config.currentThompsonHeightIdx] = ms->config.m_x;
      cout << "m_x-- ";
    }

    cout << endl;
  }
}
END_WORD
REGISTER_WORD(SetMagnificationA)

WORD(SetMagnificationB)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // adjust until close	

  int darkX = 0;
  int darkY = 0;
  findDarkness(ms, &darkX, &darkY);

  ms->config.pilotTarget.px = darkX;
  ms->config.pilotTarget.py = darkY;

  int magIters = 2000; 
  double magStep = 0.01;

  for (int i = 0; i < magIters; i++) {
    double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;
    int eX=0, eY=0;
    //globalToPixel(&eX, &eY, zToUse, ms->config.eepReg1.px, ms->config.eepReg1.py);
    globalToPixelPrint(ms, &eX, &eY, zToUse, ms->config.eepReg1.px, ms->config.eepReg1.py);

    // remember this is flipped!
    double Px = darkY - eY;
    double Py = darkX - eX;

    double xFlip = 1.0;
    double yFlip = 1.0;

    // remember x, y are swapped
    eePose thisFlipReticle = ms->config.heightReticles[ms->config.currentThompsonHeightIdx];
    if (darkX < thisFlipReticle.px) {
      yFlip = -1.0;
    }
    if (darkY < thisFlipReticle.py) {
      xFlip = -1.0;
    }

    cout << "about to adjust m_y, darkX eX Px xFlip darkY eY Py yFlip: " << darkX << " " << eX << " " << Px << " " << xFlip << " " << darkY << " " << eY << " " << Py << " " << yFlip << " ";

    // only do y
    if ((Py*yFlip) > 0) {
      ms->config.m_y += .01;
      ms->config.m_y_h[ms->config.currentThompsonHeightIdx] = ms->config.m_y;
      cout << "m_y++ ";
    } else if ((Py*yFlip) < 0) {
      ms->config.m_y -= .01;
      ms->config.m_y_h[ms->config.currentThompsonHeightIdx] = ms->config.m_y;
      cout << "m_y-- ";
    }

    cout << endl;
  }
}
END_WORD
REGISTER_WORD(SetMagnificationB)

WORD(SetGripperMaskOnes)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.gripperMask.create(ms->config.gripperMaskFirstContrast.size(), CV_8U);
  ms->config.cumulativeGripperMask.create(ms->config.gripperMaskFirstContrast.size(), CV_8U);

  Size sz = ms->config.gripperMask.size();
  int imW = sz.width;
  int imH = sz.height;


  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      ms->config.gripperMask.at<uchar>(y,x) = 1;
      ms->config.cumulativeGripperMask.at<uchar>(y,x) = 1;
    }
  }
}
END_WORD
REGISTER_WORD(SetGripperMaskOnes)

WORD(SetGripperMask)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Program paused; please present the first contrast medium." << endl;
  ms->pushWord("setGripperMaskA"); 
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", 10);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("pauseStackExecution"); 
}
END_WORD
REGISTER_WORD(SetGripperMask)

WORD(SetGripperMaskAA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.gripperMaskFirstContrast = ms->config.accumulatedImage.clone();
  ms->config.gripperMaskSecondContrast = ms->config.gripperMaskFirstContrast.clone();
  ms->config.gripperMaskMean = ms->config.gripperMaskFirstContrast.clone();
  ms->config.gripperMaskMean = 0.0;
  ms->config.gripperMaskSquares = ms->config.gripperMaskFirstContrast.clone();
  ms->config.gripperMaskSquares = 0.0;
  ms->config.gripperMaskCounts = 0;

  ms->config.gripperMask.create(ms->config.gripperMaskFirstContrast.size(), CV_8U);

  Size sz = ms->config.gripperMask.size();
  int imW = sz.width;
  int imH = sz.height;

  cout << "Updating image" << endl;
  //ms->config.gripperMaskFirstContrastWindow->updateImage(ms->config.wristViewImage);

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      double denom = ms->config.accumulatedImageMass.at<double>(y,x);
      if (denom <= 1.0) {
	denom = 1.0;
      }
      ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[0] = (ms->config.accumulatedImage.at<Vec3d>(y,x)[0] / denom);
      ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[1] = (ms->config.accumulatedImage.at<Vec3d>(y,x)[1] / denom);
      ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[2] = (ms->config.accumulatedImage.at<Vec3d>(y,x)[2] / denom);

      ms->config.gripperMask.at<uchar>(y,x) = 0;
    }
  }
  //ms->config.gripperMaskFirstContrastWindow->updateImage(ms->config.gripperMaskFirstContrast / 255.0);
}
END_WORD
REGISTER_WORD(SetGripperMaskAA)

WORD(InitCumulativeGripperMask)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.cumulativeGripperMask.create(ms->config.accumulatedImage.size(), CV_8U);
  Size sz = ms->config.cumulativeGripperMask.size();
  int imW = sz.width;
  int imH = sz.height;
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      ms->config.cumulativeGripperMask.at<uchar>(y,x) = 0;
    }
  }
}
END_WORD
REGISTER_WORD(InitCumulativeGripperMask)

WORD(SetGripperMaskA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Program paused; please present the second contrast medium." << endl;
  ms->pushWord("setGripperMaskB"); 
  ms->pushWord("setGripperMaskBA"); 
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", 10);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("pauseStackExecution"); 
  ms->pushWord("setGripperMaskAA"); 
  ms->pushWord("initCumulativeGripperMask"); 
}
END_WORD
REGISTER_WORD(SetGripperMaskA)

WORD(SetGripperMaskB)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Thank you. Don't forget to save your mask!" << endl;
}
END_WORD
REGISTER_WORD(SetGripperMaskB)

WORD(SetGripperMaskBA)
virtual void execute(std::shared_ptr<MachineState> ms) {

  Size sz = ms->config.gripperMask.size();
  int imW = sz.width;
  int imH = sz.height;

  int dilationPixels = 10;
  double baseThresh = ms->config.gripperMaskThresh;
  //double multiThresh = 3*baseThresh*baseThresh; // for rgb
  double multiThresh = 2*baseThresh*baseThresh; // for ycbcr

  cout << "  multiThresh dilationPixels: " << multiThresh << " " << dilationPixels << endl;

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      double denom = ms->config.accumulatedImageMass.at<double>(y,x);
      if (denom <= 1.0) {
	denom = 1.0;
      }
      ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[0] = (ms->config.accumulatedImage.at<Vec3d>(y,x)[0] / denom);
      ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[1] = (ms->config.accumulatedImage.at<Vec3d>(y,x)[1] / denom);
      ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[2] = (ms->config.accumulatedImage.at<Vec3d>(y,x)[2] / denom);

      ms->config.gripperMaskMean.at<Vec3d>(y,x)[0] += (ms->config.accumulatedImage.at<Vec3d>(y,x)[0] / denom);
      ms->config.gripperMaskMean.at<Vec3d>(y,x)[1] += (ms->config.accumulatedImage.at<Vec3d>(y,x)[1] / denom);
      ms->config.gripperMaskMean.at<Vec3d>(y,x)[2] += (ms->config.accumulatedImage.at<Vec3d>(y,x)[2] / denom);

      ms->config.gripperMaskSquares.at<Vec3d>(y,x)[0] += pow((ms->config.accumulatedImage.at<Vec3d>(y,x)[0] / denom), 2);
      ms->config.gripperMaskSquares.at<Vec3d>(y,x)[1] += pow((ms->config.accumulatedImage.at<Vec3d>(y,x)[1] / denom), 2);
      ms->config.gripperMaskSquares.at<Vec3d>(y,x)[2] += pow((ms->config.accumulatedImage.at<Vec3d>(y,x)[2] / denom), 2);
    }
  }
  ms->config.gripperMaskCounts += 1;
  //ms->config.gripperMaskSecondContrastWindow->updateImage(ms->config.gripperMaskSecondContrast / 255.0);
  Mat firstFloat; Mat firstYCBCR;  ms->config.gripperMaskFirstContrast.convertTo(firstFloat, CV_32FC3); cvtColor(firstFloat, firstYCBCR, CV_BGR2YCrCb);
  Mat secondFloat; Mat secondYCBCR;  ms->config.gripperMaskSecondContrast.convertTo(secondFloat, CV_32FC3); cvtColor(secondFloat, secondYCBCR, CV_BGR2YCrCb);

  Mat varianceImage = ms->config.gripperMaskFirstContrast.clone();

  Mat differenceImage = ms->config.gripperMaskFirstContrast.clone();

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      //double maskDiff = 
      //((ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[0] - ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[0])*
      //(ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[0] - ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[0])) +
      //((ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[1] - ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[1])*
      //(ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[1] - ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[1])) +
      //((ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[2] - ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[2])*
      //(ms->config.gripperMaskFirstContrast.at<Vec3d>(y,x)[2] - ms->config.gripperMaskSecondContrast.at<Vec3d>(y,x)[2]));

      double maskDiffFromFirst = 
      ((firstYCBCR.at<Vec3f>(y,x)[1] - secondYCBCR.at<Vec3f>(y,x)[1])*
      (firstYCBCR.at<Vec3f>(y,x)[1] - secondYCBCR.at<Vec3f>(y,x)[1])) +
      ((firstYCBCR.at<Vec3f>(y,x)[2] - secondYCBCR.at<Vec3f>(y,x)[2])*
      (firstYCBCR.at<Vec3f>(y,x)[2] - secondYCBCR.at<Vec3f>(y,x)[2])) 
	//+
	//((firstYCBCR.at<Vec3f>(y,x)[0] - secondYCBCR.at<Vec3f>(y,x)[0])*
	//(firstYCBCR.at<Vec3f>(y,x)[0] - secondYCBCR.at<Vec3f>(y,x)[0]))
	;

      differenceImage.at<Vec3d>(y,x)[0] = maskDiffFromFirst;
      differenceImage.at<Vec3d>(y,x)[1] = 0.0;
      differenceImage.at<Vec3d>(y,x)[2] = 0.0;

      varianceImage.at<Vec3d>(y,x)[0] = ms->config.gripperMaskSquares.at<Vec3d>(y, x)[0] / ms->config.gripperMaskCounts - pow(ms->config.gripperMaskMean.at<Vec3d>(y, x)[0] / ms->config.gripperMaskCounts, 2) ;
      varianceImage.at<Vec3d>(y,x)[1] = ms->config.gripperMaskSquares.at<Vec3d>(y, x)[1] / ms->config.gripperMaskCounts - pow(ms->config.gripperMaskMean.at<Vec3d>(y, x)[1] / ms->config.gripperMaskCounts, 2) ;
      varianceImage.at<Vec3d>(y,x)[2] = ms->config.gripperMaskSquares.at<Vec3d>(y, x)[2] / ms->config.gripperMaskCounts - pow(ms->config.gripperMaskMean.at<Vec3d>(y, x)[2] / ms->config.gripperMaskCounts, 2) ;

      varianceImage.at<Vec3d>(y,x)[0] = 0;//varianceImage.at<Vec3d>(y,x)[0] / pow(255.0, 2);
      varianceImage.at<Vec3d>(y,x)[1] = varianceImage.at<Vec3d>(y,x)[1] / pow(255.0, 2);
      varianceImage.at<Vec3d>(y,x)[2] = varianceImage.at<Vec3d>(y,x)[2] / pow(255.0, 2);

      double maskDiffVariance = sqrt( pow(varianceImage.at<Vec3d>(y,x)[1],2) + pow(varianceImage.at<Vec3d>(y,x)[2],2) );
      if (maskDiffVariance > multiThresh) {
	ms->config.gripperMask.at<uchar>(y,x) = 1;
      } else {
	ms->config.gripperMask.at<uchar>(y,x) = 0;
      }
    }
  }

  //ms->config.gripperMaskDifferenceWindow->updateImage(differenceImage / 255.0);
  //ms->config.gripperMaskVarianceWindow->updateImage(varianceImage * 10);
  //ms->config.gripperMaskMeanWindow->updateImage(ms->config.gripperMaskMean /  ms->config.gripperMaskCounts / 255.0);
  //ms->config.gripperMaskSquaresWindow->updateImage(ms->config.gripperMaskSquares /  ms->config.gripperMaskCounts / (255.0 * 255.0));

  Mat tmpMask = ms->config.gripperMask.clone();

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      if (tmpMask.at<uchar>(y,x) == 0) {
	int xmin = max(0, x - dilationPixels);
	int xmax = min(imW-1, x + dilationPixels);
	int ymin = max(0, y - dilationPixels);
	int ymax = min(imH-1, y + dilationPixels);
	for (int xp = xmin; xp < xmax; xp++) {
	  for (int yp = ymin; yp < ymax; yp++) {
	    ms->config.gripperMask.at<uchar>(yp,xp) = 0;
	  }
	}
      }
    }
  }
}
END_WORD
REGISTER_WORD(SetGripperMaskBA)

WORD(SetGripperMaskWithMotion)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("setGripperMaskWithMotionA");
  ms->pushWord("initCumulativeGripperMask");
}
END_WORD
REGISTER_WORD(SetGripperMaskWithMotion)

WORD(SetGripperMaskWithMotionA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int maskMotions = 25;
  cout << "Setting gripper mask with motion, iterations: " << maskMotions << endl;

  for (int m = 0; m < maskMotions; m++) {
    // watch it as it develops
    ms->pushWord("saveGripperMask");
    ms->pushWord("setGripperMaskCB");

    // once observed always observed
    ms->pushWord("setGripperMaskCA");

    ms->pushWord("setGripperMaskBA"); 
    ms->pushWord("accumulatedDensity");
    ms->pushCopies("waitUntilImageCallbackReceived", 10);
    ms->pushWord("resetAccumulatedDensity");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");

    // move speed not set so that you can control for aliasing from repl
    ms->pushWord("yDown");
  }

  ms->pushWord("setGripperMaskAA"); 
  ms->pushWord("accumulatedDensity");
  ms->pushCopies("waitUntilImageCallbackReceived", 10);
  ms->pushWord("resetAccumulatedDensity");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
}
END_WORD
REGISTER_WORD(SetGripperMaskWithMotionA)

WORD(SetGripperMaskCA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.cumulativeGripperMask = max(ms->config.cumulativeGripperMask, ms->config.gripperMask);
}
END_WORD
REGISTER_WORD(SetGripperMaskCA)

WORD(SetGripperMaskCB)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.gripperMask = ms->config.cumulativeGripperMask.clone();
  cout << "Thank you. Don't forget to save your mask!" << endl;
}
END_WORD
REGISTER_WORD(SetGripperMaskCB)

WORD(LoadGripperMask)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string filename = ms->config.data_directory + ms->config.config_directory + ms->config.left_or_right_arm + "GripperMask.bmp";
  cout << "Loading gripper mask from " << filename << endl;
  Mat tmpMask = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
  cout << "  tmpMask.type() tmpMask.size(): " << tmpMask.type() << " " << tmpMask.size() << endl;

  ms->config.gripperMask.create(tmpMask.size(), CV_8U);
  Size sz = ms->config.gripperMask.size();
  int imW = sz.width;
  int imH = sz.height;

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      if (tmpMask.at<uchar>(y,x) > 0) {
	ms->config.gripperMask.at<uchar>(y,x) = 1;
      } else {
	ms->config.gripperMask.at<uchar>(y,x) = 0;
      }
    }
  }
  
}
END_WORD
REGISTER_WORD(LoadGripperMask)

WORD(SaveGripperMask)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string filename = ms->config.data_directory + ms->config.config_directory + ms->config.left_or_right_arm + "GripperMask.bmp";
  cout << "Saving gripper mask to " << filename << endl;
  imwrite(filename, 255*ms->config.gripperMask);
}
END_WORD
REGISTER_WORD(SaveGripperMask)

WORD(CalibrateRGBCameraIntrinsics)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord("setMagnification");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("moveToRegister1");

  ms->pushWord("setHeightReticles");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("moveToRegister1");

  ms->pushWord("setVanishingPoint");
  ms->pushWord("moveCropToCenter");

  //int tablePeek = 5;
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("saveRegister1");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("changeToHeight0"); 
  //ms->pushCopies("xUp", tablePeek);
  ms->pushWord("setTable");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  //ms->pushCopies("xDown", tablePeek);
  ms->pushWord("setGridSizeCoarse");

}
END_WORD
REGISTER_WORD(CalibrateRGBCameraIntrinsics)

WORD(AssumeCalibrationPose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentEEPose = ms->config.calibrationPose;
}
END_WORD
REGISTER_WORD(AssumeCalibrationPose)

WORD(InitializeConfig)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string config_dir = ms->config.data_directory + ms->config.config_directory;
  string default_config_dir = ms->config.data_directory + "/config/";
  if (! exists(config_dir)) {
    bool result = copyDir(default_config_dir, config_dir);
    if (! result) {
      cout << "Couldn't initialize config " << config_dir << endl;
      assert(0);
    }
  }
}
END_WORD
REGISTER_WORD(InitializeConfig)

WORD(LoadCalibration)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string fileName = ms->config.data_directory + ms->config.config_directory + ms->config.left_or_right_arm + "Calibration.yml";
  cout << "Loading calibration file from " << fileName << endl;
  loadCalibration(ms, fileName);
}
END_WORD
REGISTER_WORD(LoadCalibration)

WORD(SaveCalibration)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string fileName = ms->config.data_directory + ms->config.config_directory + ms->config.left_or_right_arm + "Calibration.yml";
  cout << "Saving calibration file from " << fileName << endl;
  saveCalibration(ms, fileName);
}
END_WORD
REGISTER_WORD(SaveCalibration)

WORD(SaveCalibrationToClass)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int tfc = ms->config.focusedClass; 

  if ((tfc > -1) && (tfc < ms->config.classLabels.size())) {
    // do nothing
  } else {
    cout << "saveCalibrationToClass: invalid focused class, not writing." << endl;
    return;
  }

  string thisLabelName = ms->config.classLabels[tfc];
  string this_image_path = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/calibration/";
  ros::Time thisNow = ros::Time::now();
  char buf[1024];
  sprintf(buf, "%s%f_%s_%s.yml", this_image_path.c_str(), thisNow.toSec(), ms->config.robot_serial.c_str(), ms->config.left_or_right_arm.c_str());
  string fileName(buf); 

  cout << "saveCalibrationToClass: Saving calibration file to " << fileName << endl;
  saveCalibration(ms, fileName);
}
END_WORD
REGISTER_WORD(SaveCalibrationToClass)

WORD(SetColorReticles)
virtual void execute(std::shared_ptr<MachineState> ms) {

  ms->config.bDelta = ms->config.cReticleIndexDelta;
  ms->config.currentEEPose.pz = ms->config.firstCReticleIndexDepth;

  // leave it in a canonical state
  ms->pushWord("setGridSizeCoarse");

  int * ii = &(ms->config.scrI);
  (*ii) = 0;

  for (int i = 0; i < ms->config.numCReticleIndeces; i++) {
    ms->pushWord("zUp");
    ms->pushWord("setColorReticlesA");
    ms->pushWord("accumulatedDensity");
    ms->pushCopies("waitUntilImageCallbackReceived", 100);
    ms->pushWord("resetAccumulatedDensity");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
  }
}
END_WORD
REGISTER_WORD(SetColorReticles)

WORD(SetColorReticlesA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int lightX = 0;
  int lightY = 0;
  findLight(ms, &lightX, &lightY);

  ms->config.pilotTarget.px = lightX;
  ms->config.pilotTarget.py = lightY;

  int * ii = &(ms->config.scrI);
  ms->config.xCR[(*ii)] = lightX;
  ms->config.yCR[(*ii)] = lightY;

  (*ii)++;
}
END_WORD
REGISTER_WORD(SetColorReticlesA)

WORD(ScanObjectFast)

virtual string description() {
  return "Scans an object without an IR scan, and with an annotated grasp.";
}

virtual void execute(std::shared_ptr<MachineState> ms) {

  int retractCm = 10;
  
  cout << "BEGINNING SCANOBJECTFAST" << endl;
  cout << "Program will pause shortly. Please adjust height and object so that arm would grip if closed and so that the gripper will clear the object once raised 5cm." << endl;

  ms->config.eepReg2 = ms->config.beeHome;
  ms->config.eepReg4 = ms->config.beeHome;

  // so that closest servoing doesn't go into gradient servoing.
  ms->config.targetClass = -1;

  // set lastLabelLearned
  ms->pushWord("setLastLabelLearned");

  ms->pushWord("setGridSizeCoarse");
  //ms->pushWord("rgbScan");
  //ms->pushWord("rgbScan");
  ms->pushWord("scanCentered");
  ms->pushWord("fullImpulse");
  ms->pushWord("setGridSizeVeryFine");

  ms->pushWord("changeToHeight1"); 
  //ms->pushWord("comeToHover");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("moveToRegister1");



  //ms->pushWord("setGridSizeEvenCoarser");
  //ms->pushWord("fasterRasterScanningSpeed");
  
  //ms->pushWord("comeToStop");
  //ms->pushWord("waitUntilAtCurrentPosition");
  //ms->pushWord("comeToHover");
  //ms->pushWord("waitUntilAtCurrentPosition");
  //ms->pushWord("moveToRegister1");
  //ms->pushWord("quarterImpulse");
  
  {
    ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("gradientServoPrep");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight3"); // change to height 3
  }
  {
    ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("gradientServoPrep");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight2"); // change to height 2
  }
  {
    ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("gradientServoPrep");
    ms->pushWord("waitUntilAtCurrentPosition");
      ms->pushWord("changeToHeight1"); // change to height 1
  }
  {
    ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("gradientServoPrep");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight0"); // change to height 0
  }
  
  
  ms->pushWord("saveCurrentClassDepthAndGraspMaps"); // save current depth map to current class
  ms->pushWord("preAnnotateOffsetGrasp"); 
  ms->pushWord("setPhotoPinHere");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("synchronicServo");
  ms->pushWord("synchronicServoTakeClosest");
  ms->pushWord("sampleHeight"); 

  ms->pushWord("fullImpulse");

  ms->pushWord("saveRegister1");
  ms->pushWord("waitUntilAtCurrentPosition");

  // dislodge. necessary because the robot takes a while to "spin up" at slow speeds, which interferes
 //  with the state machine.
  ms->pushCopies("dislodgeEndEffectorFromTable", retractCm);
  ms->pushWord("setCurrentPoseToTruePose");
  ms->pushWord("setGridSizeCoarse");

  ms->pushWord("recordGraspZ");

  ms->pushWord("hundredthImpulse");
  ms->pushWord("pauseStackExecution"); // pause stack execution
  ms->pushWord("initializeAndFocusOnNewClass"); //  make a new class

  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("changeToHeight0");
  //ms->pushCopies("yDown", 25);
  ms->pushWord("setGridSizeCoarse");
  ms->pushWord("assumeBackScanningPose");
  ms->pushWord("assumeCalibrationPose");
  ms->pushWord("fullImpulse");
  ms->pushWord("setBoundingBoxModeToMapping"); 
}
END_WORD
REGISTER_WORD(ScanObjectFast)

WORD(ScanObjectStream)
virtual string description() {
  return "Scans an object in stream mode with an annotated grasp.";
}
virtual void execute(std::shared_ptr<MachineState> ms) {

  int retractCm = 10;
  
  cout << "BEGINNING SCANOBJECTSTREAM" << endl;
  cout << "Program will pause shortly. Please adjust height and object so that arm would grip if closed and so that the gripper will clear the object once raised 5cm." << endl;

  ms->config.eepReg2 = ms->config.beeHome;
  ms->config.eepReg4 = ms->config.beeHome;

// XXX 
//  save grasp information separately
//  remove grasp annotation
//  reinit range maps for one class at a time

  ms->pushWord("pickFocusedClass");
  ms->pushWord("changeToHeight"); 
  ms->pushWord(std::make_shared<IntegerWord>(1));
  // XXX second writeFocusedClass because aerial gradients aren't loaded until re-init
  //ms->pushWord("writeFocusedClass");
  //ms->pushWord("integrateImageStreamBufferCrops");
  //ms->pushWord("reinitRangeMaps");
  ms->pushWord("writeFocusedClass");
  ms->pushWord("integrateImageStreamBufferServoImages");
  //ms->pushWord("saveCurrentClassDepthAndGraspMaps");
  //ms->pushWord("loadMarginalGraspMemory");
  ms->pushWord("loadPriorGraspMemoryAnalytic");
  //ms->pushWord("resetCurrentFocusedClass");
  ms->pushWord("classRangeMapFromRegister1");
  ms->pushWord("integrateRangeStreamBuffer");
  ms->pushWord("populateStreamBuffers");


  // set lastLabelLearned
  ms->pushWord("setLastLabelLearned");


  ms->pushWord("setGridSizeCoarse");

  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 

  ms->pushWord("streamScanCentered");

  ms->pushWord("activateSensorStreaming"); 
  ms->pushWord("clearStreamBuffers"); 
  ms->pushWord("shutdownToSensorsAndMovement"); 
  ms->pushWord("setSisFlags"); 
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));


  ms->pushWord("fullImpulse");

  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1"); 
  ms->pushWord("changeToHeight1"); 
  //ms->pushWord("comeToHover");
  ms->pushWord("moveToRegister1");


  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("gradientServoPrep");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight3"); // change to height 3
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("gradientServoPrep");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight2"); // change to height 2
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("gradientServoPrep");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight1"); // change to height 1
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("gradientServoPrep");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight0"); // change to height 0
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
  //ms->pushWord(std::make_shared<IntegerWord>(0));

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));

  
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("moveToRegister1");

  //ms->pushWord("saveCurrentClassDepthAndGraspMaps"); // XXX

  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 
  ms->pushWord("neutralScan"); 
  
  ms->pushWord("activateSensorStreaming"); 
  ms->pushWord("clearStreamBuffers"); 
  ms->pushWord("shutdownToSensorsAndMovement"); 

  ms->pushWord("setSisFlags");   
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(1));


  ms->pushWord("lock3dGraspBase"); 
  ms->pushWord("saveRegister1");

  ms->pushWord("pauseStackExecution"); 
  


  //ms->pushWord("preAnnotateOffsetGrasp"); // XXX



  ms->pushWord("setPhotoPinHere");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("synchronicServo");
  ms->pushWord("synchronicServoTakeClosest");
  ms->pushWord("sampleHeight"); 

  ms->pushWord("fullImpulse");

  ms->pushWord("waitUntilAtCurrentPosition");

  // dislodge. necessary because the robot takes a while to "spin up" at slow speeds, which interferes
  //  with the state machine while in contact with the table
  ms->pushCopies("dislodgeEndEffectorFromTable", retractCm);
  ms->pushWord("setCurrentPoseToTruePose");
  ms->pushWord("setGridSizeCoarse");
  //ms->pushWord("recordGraspZ"); // XXX
  ms->pushWord("hundredthImpulse");

  ms->pushWord("pauseStackExecution"); // pause stack execution
  ms->pushWord("initializeAndFocusOnNewClass"); //  make a new class

  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("changeToHeight0");

  ms->pushWord("setGridSizeCoarse");
  ms->pushWord("fullImpulse");
  ms->pushWord("setBoundingBoxModeToMapping"); 

  ms->pushWord("clearBlueBoxMemories"); 
  ms->pushWord("clearMapForPatrol"); 
  ms->pushWord("enableDiskStreaming"); 
}
END_WORD
REGISTER_WORD(ScanObjectStream)

WORD(ScanObjectStreamAnnotated)
virtual string description() {
  return "Scans an object in stream mode with an annotated grasp.";
}
virtual void execute(std::shared_ptr<MachineState> ms) {

  int retractCm = 10;
  
  cout << "BEGINNING SCANOBJECTSTREAM" << endl;
  cout << "Program will pause shortly. Please adjust height and object so that arm would grip if closed and so that the gripper will clear the object once raised 5cm." << endl;

  ms->config.eepReg2 = ms->config.beeHome;
  ms->config.eepReg4 = ms->config.beeHome;

// XXX 
//  save grasp information separately
//  remove grasp annotation
//  reinit range maps for one class at a time

  ms->pushWord("pickFocusedClass");
  ms->pushWord("cruisingSpeed"); 
  ms->pushWord("changeToHeight"); 
  ms->pushWord(std::make_shared<IntegerWord>(1));
  // XXX second writeFocusedClass because aerial gradients aren't loaded until re-init
  //ms->pushWord("writeFocusedClass");
  //ms->pushWord("integrateImageStreamBufferCrops");
  //ms->pushWord("reinitRangeMaps");
  ms->pushWord("writeFocusedClass");
  ms->pushWord("integrateImageStreamBufferServoImages");
  //ms->pushWord("saveCurrentClassDepthAndGraspMaps");
  //ms->pushWord("loadMarginalGraspMemory");
  //ms->pushWord("loadPriorGraspMemoryAnalytic"); // XXX
  //ms->pushWord("resetCurrentFocusedClass");
  //ms->pushWord("classRangeMapFromRegister1"); // XXX
  //ms->pushWord("integrateRangeStreamBuffer"); // XXX
  ms->pushWord("populateStreamBuffers");


  // set lastLabelLearned
  ms->pushWord("setLastLabelLearned");


  ms->pushWord("setGridSizeCoarse");

  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 

  ms->pushWord("streamScanCentered");

  ms->pushWord("activateSensorStreaming"); 
  ms->pushWord("clearStreamBuffers"); 
  ms->pushWord("shutdownToSensorsAndMovement"); 
  ms->pushWord("setSisFlags"); 
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));


  ms->pushWord("fullImpulse");

  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1"); 
  ms->pushWord("changeToHeight1"); 
  //ms->pushWord("comeToHover");
  ms->pushWord("moveToRegister1");


  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 
    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight3"); // change to height 3
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight2"); // change to height 2
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight1"); // change to height 1
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight0"); // change to height 0
  }
  ms->pushWord("departureSpeed");

  ms->pushWord("shutdownToSensorsAndMovement"); 
  ms->pushWord("setSisFlags"); 
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));

  
  //ms->pushWord("waitUntilAtCurrentPosition");
  //ms->pushWord("moveToRegister1");
  //ms->pushWord("saveRegister1");  // this is so we return to a reasonable place 

  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("changeToHeight0"); // change to height 0

  ms->pushWord("departureSpeed");

  ms->pushWord("setPhotoPinHere");
  ms->pushWord("writeFocusedClass");
  ms->pushWord("lock3dGraspBase"); 
  //ms->pushWord("preAnnotateCenterGrasp"); // XXX
  ms->pushWord("preAnnotateOffsetGrasp"); // you need to saveRegister1 where the grasp should be before calling this
  ms->pushWord("setPhotoPinHere");

  //ms->pushWord("pauseStackExecution"); // pause to ensure being centered

  ms->pushWord("comeToStop");
  ms->pushWord("setMovementStateToMoving");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  if (ms->config.currentScanMode == CENTERED) {
    ms->pushWord("synchronicServo");
  } else if (ms->config.currentScanMode == NOT_CENTERED) {
  } else {
    assert(0);
  }
  ms->pushWord("synchronicServoTakeClosest");
  ms->pushWord("sampleHeight"); 

  ms->pushWord("departureSpeed");
  ms->pushWord("waitUntilAtCurrentPosition");
  // dislodge. necessary because the robot takes a while to "spin up" at slow speeds, which interferes
  //  with the state machine while in contact with the table
  ms->pushCopies("dislodgeEndEffectorFromTable", retractCm);
  ms->pushWord("setCurrentPoseToTruePose");
  ms->pushWord("setGridSizeCoarse");
  ms->pushWord("hundredthImpulse");

  ms->pushWord("saveRegister1"); // for preAnnotateOffsetGrasp, which isn't use in this version but could be used where it is.
  ms->pushWord("recordGraspZ"); // XXX

  ms->pushWord("pauseStackExecution"); // pause for annotation positioning

  ms->pushWord("initializeAndFocusOnNewClass"); //  make a new class

  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("changeToHeight0");

  ms->pushWord("setGridSizeCoarse");
  ms->pushWord("fullImpulse");
  ms->pushWord("setBoundingBoxModeToMapping"); 

  ms->pushWord("clearBlueBoxMemories"); 
  ms->pushWord("clearMapForPatrol"); 
  ms->pushWord("enableDiskStreaming"); 
}
END_WORD
REGISTER_WORD(ScanObjectStreamAnnotated)

WORD(ScanObjectStreamWaypoints)
virtual string description() {
  return "Scans a stack of objects in stream mode with an annotated grasps in stack.";
}
virtual void execute(std::shared_ptr<MachineState> ms) {

  cout << "Entering scanObjectStreamWaypoints" << endl;

  eePose destPose;
  CONSUME_EEPOSE(destPose, ms);

  ms->config.currentEEPose.px = destPose.px;
  ms->config.currentEEPose.py = destPose.py;
  double graspZGlobal = destPose.pz;

  {
    double flushZ = -(ms->config.currentTableZ) + ms->config.pickFlushFactor;
    ms->config.currentGraspZ = -(graspZGlobal - (-ms->config.currentTableZ));
    cout << "scanObjectStreamWaypoints flushZ currentGraspZ: " << flushZ << " " << ms->config.currentGraspZ << " " << endl;
  }

  ms->config.eepReg1 = destPose;

  ms->pushWord("scanObjectStreamWaypoints");

  int retractCm = 10;

  ms->config.eepReg2 = ms->config.beeHome;
  ms->config.eepReg4 = ms->config.beeHome;

  ms->pushWord("pickFocusedClass");
  ms->pushWord("cruisingSpeed"); 
  ms->pushWord("changeToHeight"); 
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord("writeFocusedClass");
  ms->pushWord("integrateImageStreamBufferServoImages");
  ms->pushWord("populateStreamBuffers");

  // set lastLabelLearned
  ms->pushWord("setLastLabelLearned");


  ms->pushWord("setGridSizeCoarse");

  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 

  ms->pushWord("streamScanCentered");

  ms->pushWord("activateSensorStreaming"); 
  ms->pushWord("clearStreamBuffers"); 
  ms->pushWord("shutdownToSensorsAndMovement"); 
  ms->pushWord("setSisFlags"); 
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));


  ms->pushWord("fullImpulse");

  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1"); 
  ms->pushWord("changeToHeight1"); 
  //ms->pushWord("comeToHover");
  ms->pushWord("moveToRegister1");


  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight3"); // change to height 3
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 
    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight2"); // change to height 2
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight1"); // change to height 1
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight0"); // change to height 0
  }
  ms->pushWord("departureSpeed");

  ms->pushWord("shutdownToSensorsAndMovement"); 
  ms->pushWord("setSisFlags"); 
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  

  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("changeToHeight0"); // change to height 0

  ms->pushWord("departureSpeed");

  ms->pushWord("setPhotoPinHere");
  ms->pushWord("writeFocusedClass");
  ms->pushWord("lock3dGraspBase"); 

  ms->pushWord("preAnnotateOffsetGrasp"); // you need to saveRegister1 where the grasp should be before calling this
  ms->pushWord("setPhotoPinHere");

  //ms->pushWord("pauseStackExecution"); // pause to ensure being centered

  ms->pushWord("comeToStop");
  ms->pushWord("setMovementStateToMoving");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  if (ms->config.currentScanMode == CENTERED) {
    ms->pushWord("synchronicServo");
  } else if (ms->config.currentScanMode == NOT_CENTERED) {
  } else {
    assert(0);
  }
  ms->pushWord("synchronicServoTakeClosest");
  ms->pushWord("sampleHeight"); 

  ms->pushWord("departureSpeed");
  ms->pushWord("waitUntilAtCurrentPosition");
  // dislodge. necessary because the robot takes a while to "spin up" at slow speeds, which interferes
  //  with the state machine while in contact with the table
  ms->pushCopies("dislodgeEndEffectorFromTable", retractCm);
  ms->pushWord("setCurrentPoseToTruePose");
  ms->pushWord("setGridSizeCoarse");
  ms->pushWord("hundredthImpulse");
  
  // register 1 comes from the stack in this version
  //ms->pushWord("saveRegister1"); // for preAnnotateOffsetGrasp
  // recordGraspZ is replaced with logic at the top
  //ms->pushWord("recordGraspZ"); // XXX

  // don't pause because we're doing this from the stack
  //ms->pushWord("pauseStackExecution"); // pause for annotation positioning

  ms->pushWord("initializeAndFocusOnNewClass"); //  make a new class

  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("changeToHeight0");

  ms->pushWord("setGridSizeCoarse");
  ms->pushWord("fullImpulse");
  ms->pushWord("setBoundingBoxModeToMapping"); 

  ms->pushWord("setIdleModeToEmpty"); 
  ms->pushWord("setPlaceModeToShake"); 
  ms->pushWord("clearBlueBoxMemories"); 
  ms->pushWord("clearMapForPatrol"); 
  ms->pushWord("enableDiskStreaming"); 
  ms->pushWord("zeroGOff"); 
}
END_WORD
REGISTER_WORD(ScanObjectStreamWaypoints)


WORD(ScanObjectStreamWaypointsIR)
virtual string description() {
  return "Scans a stack of objects in stream mode using IR raster scan to infer grasp points.";
}
virtual void execute(std::shared_ptr<MachineState> ms) {

  cout << "Entering scanObjectStreamWaypoints" << endl;

  eePose graspArg;
  CONSUME_EEPOSE(graspArg, ms);

  ms->config.currentEEPose.px = graspArg.px;
  ms->config.currentEEPose.py = graspArg.py;

  ms->config.eepReg1 = graspArg;

  ms->pushWord("scanObjectStreamWaypointsIR");

  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("assumeCrane1");
  ms->pushWord("openGripper");


  int retractCm = 10;

  ms->config.eepReg2 = ms->config.beeHome;
  ms->config.eepReg4 = ms->config.beeHome;

  ms->pushWord("pickFocusedClass");
  ms->pushWord("setGraspModeToCrane");
  ms->pushWord("cruisingSpeed"); 
  ms->pushWord("changeToHeight"); 
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord("writeFocusedClass");

  ms->pushWord("loadPriorGraspMemoryAnalytic");
  ms->pushWord("classRangeMapFromRegister1");
  ms->pushWord("integrateRangeStreamBuffer");

  ms->pushWord("integrateImageStreamBufferServoImages");
  ms->pushWord("populateStreamBuffers");

  // set lastLabelLearned
  ms->pushWord("setLastLabelLearned");


  ms->pushWord("setGridSizeCoarse");

  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 

  ms->pushWord("streamScanCentered");

  ms->pushWord("activateSensorStreaming"); 
  ms->pushWord("clearStreamBuffers"); 
  ms->pushWord("shutdownToSensorsAndMovement"); 
  ms->pushWord("setSisFlags"); 
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));


  ms->pushWord("fullImpulse");

  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1"); 
  ms->pushWord("changeToHeight1"); 
  ms->pushWord("moveToRegister2");


  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight3"); // change to height 3
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight2"); // change to height 2
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight1"); // change to height 1
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight0"); // change to height 0
  }
  // results in lost frames if robot chooses to unwind
  //ms->pushWord("departureSpeed");
  ms->pushWord("quarterImpulse");

  
  // put yourself at the 3dGraspBase again
  ms->pushWord("moveToRegister2");
  ms->pushWord("departureSpeed");

  ms->pushWord("shutdownToSensorsAndMovement"); 
  ms->pushWord("setSisFlags"); 
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));


  ms->pushWord("departureSpeed");

  ms->pushWord("saveCalibrationToClass");
  ms->pushWord("writeFocusedClass");


  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 
  ms->pushWord("neutralScan"); 
  
  ms->pushWord("activateSensorStreaming"); 
  ms->pushWord("clearStreamBuffers"); 
  ms->pushWord("shutdownToSensorsAndMovement"); 
  ms->pushWord("setSisFlags");   
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(1));


  ms->pushWord("waitUntilAtCurrentPosition");

  ms->pushWord("assumeZOfPoseWord"); 
  ms->pushWord(std::make_shared<EePoseWord>(graspArg));

  // IR scan, don't record 3d grasp 
  //ms->pushWord("add3dGraspPoseWord");
  //ms->pushWord(std::make_shared<EePoseWord>(graspArg));
  ms->pushWord("lock3dGraspBase"); 

  // set register 2 to the 3dGraspBase again
  ms->pushWord("saveRegister2");
  ms->pushWord("setPhotoPinHere");

  //ms->pushWord("pauseStackExecution"); // pause to ensure being centered

  ms->pushWord("comeToStop");
  ms->pushWord("setMovementStateToMoving");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  if (ms->config.currentScanMode == CENTERED) {
    ms->pushWord("synchronicServo");
  } else if (ms->config.currentScanMode == NOT_CENTERED) {
  } else {
    assert(0);
  }
  ms->pushWord("synchronicServoTakeClosest");
  ms->pushWord("sampleHeight"); 

  ms->pushWord("departureSpeed");
  ms->pushWord("waitUntilAtCurrentPosition");
  // dislodge. necessary because the robot takes a while to "spin up" at slow speeds, which interferes
  //  with the state machine while in contact with the table
  ms->pushCopies("dislodgeEndEffectorFromTable", retractCm);
  ms->pushWord("setCurrentPoseToTruePose");
  ms->pushWord("setGridSizeCoarse");
  ms->pushWord("hundredthImpulse");
  
  // don't pause because we're doing this from the stack
  //ms->pushWord("pauseStackExecution"); // pause for annotation positioning

  ms->pushWord("initializeAndFocusOnNewClass"); //  make a new class

  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("changeToHeight0");

  ms->pushWord("setGridSizeCoarse");
  ms->pushWord("fullImpulse");
  ms->pushWord("setBoundingBoxModeToMapping"); 

  ms->pushWord("setIdleModeToEmpty"); 
  ms->pushWord("setPlaceModeToShake"); 
  ms->pushWord("clearBlueBoxMemories"); 
  ms->pushWord("clearMapForPatrol"); 
  ms->pushWord("enableDiskStreaming"); 
  ms->pushWord("zeroGOff"); 
}
END_WORD
REGISTER_WORD(ScanObjectStreamWaypointsIR)

WORD(ScanObjectStreamWaypoints3d)
virtual string description() {
  return "Scans a stack of objects in stream mode with an annotated 3d grasps in stack.";
}
virtual void execute(std::shared_ptr<MachineState> ms) {

  cout << "Entering scanObjectStreamWaypoints" << endl;

  eePose graspArg;
  CONSUME_EEPOSE(graspArg, ms);

  ms->config.currentEEPose.px = graspArg.px;
  ms->config.currentEEPose.py = graspArg.py;

  ms->config.eepReg1 = graspArg;

  ms->pushWord("scanObjectStreamWaypoints3d");

  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("assumeCrane1");
  ms->pushWord("openGripper");

  int retractCm = 10;

  ms->config.eepReg2 = ms->config.beeHome;
  ms->config.eepReg4 = ms->config.beeHome;

  ms->pushWord("pickFocusedClass");
  ms->pushWord("setGraspModeTo3D");
  ms->pushWord("cruisingSpeed"); 
  ms->pushWord("changeToHeight"); 
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord("writeFocusedClass");
  ms->pushWord("integrateImageStreamBufferServoImages");
  ms->pushWord("populateStreamBuffers");

  // set lastLabelLearned
  ms->pushWord("setLastLabelLearned");


  ms->pushWord("setGridSizeCoarse");

  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 

  ms->pushWord("streamScanCentered");

  ms->pushWord("activateSensorStreaming"); 
  ms->pushWord("clearStreamBuffers"); 
  ms->pushWord("shutdownToSensorsAndMovement"); 
  ms->pushWord("setSisFlags"); 
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));


  ms->pushWord("fullImpulse");

  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1"); 
  ms->pushWord("changeToHeight1"); 
  //ms->pushWord("comeToHover");
  ms->pushWord("moveToRegister1");


  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight3"); // change to height 3
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight2"); // change to height 2
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight1"); // change to height 1
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight0"); // change to height 0
  }
  // results in lost frames if robot chooses to unwind
  //ms->pushWord("departureSpeed");
  ms->pushWord("quarterImpulse");

  ms->pushWord("shutdownToSensorsAndMovement"); 
  ms->pushWord("setSisFlags"); 
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  

  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("changeToHeight0"); // change to height 0

  ms->pushWord("departureSpeed");

  ms->pushWord("writeFocusedClass");
  ms->pushWord("add3dGraspPoseWord");
  ms->pushWord(std::make_shared<EePoseWord>(graspArg));
  ms->pushWord("lock3dGraspBase"); 

  // unneeded for 3d grasps
  // ms->pushWord("preAnnotateOffsetGrasp"); // you need to saveRegister1 where the grasp should be before calling this
  ms->pushWord("setPhotoPinHere");

  //ms->pushWord("pauseStackExecution"); // pause to ensure being centered

  ms->pushWord("comeToStop");
  ms->pushWord("setMovementStateToMoving");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  if (ms->config.currentScanMode == CENTERED) {
    ms->pushWord("synchronicServo");
  } else if (ms->config.currentScanMode == NOT_CENTERED) {
  } else {
    assert(0);
  }
  ms->pushWord("synchronicServoTakeClosest");
  ms->pushWord("sampleHeight"); 

  ms->pushWord("departureSpeed");
  ms->pushWord("waitUntilAtCurrentPosition");
  // dislodge. necessary because the robot takes a while to "spin up" at slow speeds, which interferes
  //  with the state machine while in contact with the table
  ms->pushCopies("dislodgeEndEffectorFromTable", retractCm);
  ms->pushWord("setCurrentPoseToTruePose");
  ms->pushWord("setGridSizeCoarse");
  ms->pushWord("hundredthImpulse");
  
  // register 1 comes from the stack in this version
  //ms->pushWord("saveRegister1"); // for preAnnotateOffsetGrasp
  // recordGraspZ is replaced with logic at the top
  //ms->pushWord("recordGraspZ"); // XXX

  // don't pause because we're doing this from the stack
  //ms->pushWord("pauseStackExecution"); // pause for annotation positioning

  ms->pushWord("initializeAndFocusOnNewClass"); //  make a new class

  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("changeToHeight0");

  ms->pushWord("setGridSizeCoarse");
  ms->pushWord("fullImpulse");
  ms->pushWord("setBoundingBoxModeToMapping"); 

  ms->pushWord("setIdleModeToEmpty"); 
  ms->pushWord("setPlaceModeToShake"); 
  ms->pushWord("clearBlueBoxMemories"); 
  ms->pushWord("clearMapForPatrol"); 
  ms->pushWord("enableDiskStreaming"); 
  ms->pushWord("zeroGOff"); 
}
END_WORD
REGISTER_WORD(ScanObjectStreamWaypoints3d)

WORD(ScanObjectStreamWaypoints3dNoPick)
virtual string description() {
  return "Scans a stack of objects in stream mode with an annotated 3d grasps in stack.";
}
virtual void execute(std::shared_ptr<MachineState> ms) {

  cout << "Entering scanObjectStreamWaypoints" << endl;

  eePose graspArg;
  CONSUME_EEPOSE(graspArg, ms);

  ms->config.currentEEPose.px = graspArg.px;
  ms->config.currentEEPose.py = graspArg.py;

  ms->config.eepReg1 = graspArg;

  ms->pushWord("scanObjectStreamWaypoints3dNoPick");

  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("assumeCrane1");
  ms->pushWord("openGripper");

  int retractCm = 10;

  ms->config.eepReg2 = ms->config.beeHome;
  ms->config.eepReg4 = ms->config.beeHome;

  //ms->pushWord("pickFocusedClass");
  ms->pushWord("setGraspModeTo3D");
  ms->pushWord("cruisingSpeed"); 
  ms->pushWord("changeToHeight"); 
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord("writeFocusedClass");
  ms->pushWord("integrateImageStreamBufferServoImages");
  ms->pushWord("populateStreamBuffers");

  // set lastLabelLearned
  ms->pushWord("setLastLabelLearned");


  ms->pushWord("setGridSizeCoarse");

  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 

  ms->pushWord("streamScanCentered");

  ms->pushWord("activateSensorStreaming"); 
  ms->pushWord("clearStreamBuffers"); 
  ms->pushWord("shutdownToSensorsAndMovement"); 
  ms->pushWord("setSisFlags"); 
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));


  ms->pushWord("fullImpulse");

  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1"); 
  ms->pushWord("changeToHeight1"); 
  //ms->pushWord("comeToHover");
  ms->pushWord("moveToRegister1");


  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight3"); // change to height 3
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight2"); // change to height 2
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight1"); // change to height 1
  }
  {
    //ms->pushWord("saveAerialGradientMap"); // save aerial gradient map if there is only one blue box
    ms->pushWord("deactivateSensorStreaming"); 

    ms->pushWord("waitForSeconds"); 
    ms->pushWord("4.0"); 

    ms->pushWord("activateSensorStreaming"); 
    ms->pushWord("clearStreamBuffers"); 

    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");
    ms->pushWord("changeToHeight0"); // change to height 0
  }
  // results in lost frames if robot chooses to unwind
  //ms->pushWord("departureSpeed");
  ms->pushWord("quarterImpulse");

  ms->pushWord("shutdownToSensorsAndMovement"); 
  ms->pushWord("setSisFlags"); 
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  

  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("changeToHeight0"); // change to height 0

  ms->pushWord("departureSpeed");

  ms->pushWord("writeFocusedClass");
  ms->pushWord("add3dGraspPoseWord");
  ms->pushWord(std::make_shared<EePoseWord>(graspArg));
  ms->pushWord("lock3dGraspBase"); 

  // unneeded for 3d grasps
  // ms->pushWord("preAnnotateOffsetGrasp"); // you need to saveRegister1 where the grasp should be before calling this
  ms->pushWord("setPhotoPinHere");

  //ms->pushWord("pauseStackExecution"); // pause to ensure being centered

  ms->pushWord("comeToStop");
  ms->pushWord("setMovementStateToMoving");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  if (ms->config.currentScanMode == CENTERED) {
    ms->pushWord("synchronicServo");
  } else if (ms->config.currentScanMode == NOT_CENTERED) {
  } else {
    assert(0);
  }
  ms->pushWord("synchronicServoTakeClosest");
  ms->pushWord("sampleHeight"); 

  ms->pushWord("departureSpeed");
  ms->pushWord("waitUntilAtCurrentPosition");
  // dislodge. necessary because the robot takes a while to "spin up" at slow speeds, which interferes
  //  with the state machine while in contact with the table
  ms->pushCopies("dislodgeEndEffectorFromTable", retractCm);
  ms->pushWord("setCurrentPoseToTruePose");
  ms->pushWord("setGridSizeCoarse");
  ms->pushWord("hundredthImpulse");
  
  // register 1 comes from the stack in this version
  //ms->pushWord("saveRegister1"); // for preAnnotateOffsetGrasp
  // recordGraspZ is replaced with logic at the top
  //ms->pushWord("recordGraspZ"); // XXX

  // don't pause because we're doing this from the stack
  //ms->pushWord("pauseStackExecution"); // pause for annotation positioning

  ms->pushWord("initializeAndFocusOnNewClass"); //  make a new class

  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("changeToHeight0");

  ms->pushWord("setGridSizeCoarse");
  ms->pushWord("fullImpulse");
  ms->pushWord("setBoundingBoxModeToMapping"); 

  ms->pushWord("setIdleModeToEmpty"); 
  ms->pushWord("setPlaceModeToShake"); 
  ms->pushWord("clearBlueBoxMemories"); 
  ms->pushWord("clearMapForPatrol"); 
  ms->pushWord("enableDiskStreaming"); 
  ms->pushWord("zeroGOff"); 
}
END_WORD
REGISTER_WORD(ScanObjectStreamWaypoints3dNoPick)

WORD(CollectMoreStreams)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "About to collect more streams, make sure targetClass is set, i.e. you should have" << endl <<
    " \"<target class name>\" setTargetClass " << endl <<
    " in the repl before running this. 3dGraspAnnotation will run, be sure it succeeds. If it fails to get a good lock, start over. Make sure to adjust to a safe scanning height before unpausing after the lock." << endl;


  ms->pushWord("fullImpulse");
  ms->pushWord("setGridSizeCoarse");

  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 

  ms->pushWord("streamScanCentered");

  ms->pushWord("activateSensorStreaming"); 
  ms->pushWord("clearStreamBuffers"); 
  ms->pushWord("shutdownToSensorsAndMovement"); 

  ms->pushWord("fullImpulse");
  ms->pushWord("setGridSizeVeryFine");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1"); 
  ms->pushWord("changeToHeight1"); 

  ms->pushWord("moveToRegister1");

  ms->pushWord("setSisFlags"); 
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));


  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 
  ms->pushWord("neutralScanB"); 
  
  ms->pushWord("activateSensorStreaming"); 
  ms->pushWord("clearStreamBuffers"); 
  ms->pushWord("shutdownToSensorsAndMovement"); 
  ms->pushWord("setSisFlags");   
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(1));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));
  //ms->pushWord(std::make_shared<IntegerWord>(0));

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(1));


  ms->pushWord("pauseStackExecution"); 
  ms->pushWord("saveRegister1");
  ms->pushWord("start3dGraspAnnotation"); 
    
  ms->pushWord("pauseStackExecution"); 
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("moveToMappingHeight");
  ms->pushWord("setBoundingBoxModeToMapping"); 
}
END_WORD
REGISTER_WORD(CollectMoreStreams)

WORD(RecordGraspZ)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // uses ms->config.currentEEPose instead of ms->config.trueEEPose so that we can set it below the table
  double flushZ = -(ms->config.currentTableZ) + ms->config.pickFlushFactor;
  ms->config.currentGraspZ = -(ms->config.currentEEPose.pz - (-ms->config.currentTableZ));
  cout << "recordGraspZ flushZ currentGraspZ: " << flushZ << " " << ms->config.currentGraspZ << " " << endl;
}
END_WORD
REGISTER_WORD(RecordGraspZ)

WORD(Start3dGraspAnnotation)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Starting 3d Grasp Annotation" << endl;
  ms->config.bailAfterSynchronic = 1;
  ms->config.bailAfterGradient = 1;

  ms->pushWord("lock3dGraspBase");

  ms->pushWord("gradientServoIfBlueBoxes");
  ms->pushWord("mapClosestBlueBox");
  ms->pushWord("goClassifyBlueBoxes"); 
  ms->pushWord("synchronicServo"); 
  ms->pushWord("synchronicServoTakeClosest");
  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("sampleHeight"); 
  ms->pushWord("setBoundingBoxModeToMapping");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("cruisingSpeed");
}
END_WORD
REGISTER_WORD(Start3dGraspAnnotation)

WORD(Start3dGraspAnnotationNoChange)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Starting 3d Grasp Annotation No Change" << endl;
  ms->config.bailAfterSynchronic = 1;
  ms->config.bailAfterGradient = 1;

  ms->pushWord("lock3dGraspBase");

  ms->pushWord("gradientServo");
  ms->pushWord("synchronicServo"); 
  ms->pushWord("synchronicServoTakeClosest");
  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("sampleHeight"); 
  ms->pushWord("setBoundingBoxModeToMapping");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("cruisingSpeed");
}
END_WORD
REGISTER_WORD(Start3dGraspAnnotationNoChange)

WORD(Save3dGrasps)
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (ms->config.focusedClass > -1) {
    guard3dGrasps(ms);
    string thisLabelName = ms->config.focusedClassLabel;
    string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/3dGrasps/";
    string this_grasp_path = dirToMakePath + "3dGrasps.yml";

    mkdir(dirToMakePath.c_str(), 0777);
    write3dGrasps(ms, ms->config.focusedClass, this_grasp_path);
  } 
}
END_WORD
REGISTER_WORD(Save3dGrasps)


CONFIG_GETTER_POSE(C3dPoseBase, ms->config.c3dPoseBase)
CONFIG_SETTER_POSE(SetC3dPoseBase, ms->config.c3dPoseBase)

WORD(Lock3dGraspBase)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.c3dPoseBase = ms->config.currentEEPose;
  ms->config.c3dPoseBase.pz = -ms->config.currentTableZ;
  cout << endl
       << "The base for 3d grasp annotation is now locked. You should now activate zero-G mode." << endl 
       << "Please move the gripper to a valid grasping pose and use \"add3dGrasp\" to record a grasp point." << endl
       << "You can record more than one grasp point in a row." << endl
       << "When you are done, make sure to save to disk and to exit zero-G mode." << endl;

  if ( (ms->config.bLabels.size() > 0) && (ms->config.pilotClosestBlueBoxNumber != -1) ) {
  } else {
    cout << endl << "There don't seem to be detections present." << endl;    
    //cout << "Tried to lock c3dPoseBase but failed. Clearing stack." << endl;
    //ms->clearStack();
  }
}
END_WORD
REGISTER_WORD(Lock3dGraspBase)

WORD(Add3dGrasp)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Adding 3d grasp" << endl;
  eePose thisAbsolute3dGrasp = ms->config.currentEEPose;
  //eePose txQ = ms->config.c3dPoseBase.invQ();
  //txQ = txQ.multQ(thisAbsolute3dGrasp);
  //eePose thisAbsoluteDeltaP = thisAbsolute3dGrasp.minusP(ms->config.c3dPoseBase);
  //eePose thisRelative3dGrasp = ms->config.c3dPoseBase.invQ().applyQTo(thisAbsoluteDeltaP);
  //thisRelative3dGrasp.copyQ(txQ);
  eePose thisRelative3dGrasp = thisAbsolute3dGrasp.getPoseRelativeTo(ms->config.c3dPoseBase);

  int tnc = ms->config.classLabels.size();
  if ( (ms->config.targetClass > -1) && (ms->config.targetClass < tnc) ) {
    ms->config.class3dGrasps[ms->config.targetClass].push_back(thisRelative3dGrasp);
    cout << " added " << thisRelative3dGrasp << endl;
  } else {
    cout << " cannot add 3d grasp, targetClass, tnc: " << ms->config.targetClass << " " << tnc << endl;
  }
}
END_WORD
REGISTER_WORD(Add3dGrasp)

WORD(AssumeZOfPoseWord)
virtual void execute(std::shared_ptr<MachineState> ms) {
  eePose thisAbsolute3dGrasp;
  GET_ARG(ms, EePoseWord, thisAbsolute3dGrasp);

  cout << "assumeZOfPoseWord: " << thisAbsolute3dGrasp << endl;
  
  ms->config.currentEEPose.pz = thisAbsolute3dGrasp.pz;
}
END_WORD
REGISTER_WORD(AssumeZOfPoseWord)

WORD(Add3dGraspPoseWord)
virtual void execute(std::shared_ptr<MachineState> ms) {
  eePose thisAbsolute3dGrasp ;
  GET_ARG(ms, EePoseWord, thisAbsolute3dGrasp);

  cout << "Adding 3d grasp from below, global: " << thisAbsolute3dGrasp << endl;

  eePose thisRelative3dGrasp = thisAbsolute3dGrasp.getPoseRelativeTo(ms->config.c3dPoseBase);

  int tnc = ms->config.classLabels.size();
  if ( (ms->config.targetClass > -1) && (ms->config.targetClass < tnc) ) {
    ms->config.class3dGrasps[ms->config.targetClass].push_back(thisRelative3dGrasp);
    cout << " added relative grasp: " << thisRelative3dGrasp << endl;
  } else {
    cout << " cannot add 3d grasp, targetClass, tnc: " << ms->config.targetClass << " " << tnc << endl;
  }
}
END_WORD
REGISTER_WORD(Add3dGraspPoseWord)

WORD(AddPlaceUnderPoint)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Adding place under point." << endl;
  eePose thisAbsolute3dGrasp = ms->config.currentEEPose;
  //eePose txQ = ms->config.c3dPoseBase.invQ();
  //txQ = txQ.multQ(thisAbsolute3dGrasp);
  //eePose thisAbsoluteDeltaP = thisAbsolute3dGrasp.minusP(ms->config.c3dPoseBase);
  //eePose thisRelative3dGrasp = ms->config.c3dPoseBase.invQ().applyQTo(thisAbsoluteDeltaP);
  //thisRelative3dGrasp.copyQ(txQ);
  eePose thisRelative3dGrasp = thisAbsolute3dGrasp.getPoseRelativeTo(ms->config.c3dPoseBase);

  int tnc = ms->config.classPlaceUnderPoints.size();
  if ( (ms->config.targetClass > -1) && (ms->config.targetClass < tnc) ) {
    ms->config.classPlaceUnderPoints[ms->config.targetClass].push_back(thisRelative3dGrasp);
  }
}
END_WORD
REGISTER_WORD(AddPlaceUnderPoint)

WORD(AddPlaceOverPoint)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Adding place over point." << endl;
  eePose thisAbsolute3dGrasp = ms->config.currentEEPose;
  //eePose txQ = ms->config.c3dPoseBase.invQ();
  //txQ = txQ.multQ(thisAbsolute3dGrasp);
  //eePose thisAbsoluteDeltaP = thisAbsolute3dGrasp.minusP(ms->config.c3dPoseBase);
  //eePose thisRelative3dGrasp = ms->config.c3dPoseBase.invQ().applyQTo(thisAbsoluteDeltaP);
  //thisRelative3dGrasp.copyQ(txQ);
  eePose thisRelative3dGrasp = thisAbsolute3dGrasp.getPoseRelativeTo(ms->config.c3dPoseBase);

  int tnc = ms->config.classPlaceOverPoints.size();
  if ( (ms->config.targetClass > -1) && (ms->config.targetClass < tnc) ) {
    ms->config.classPlaceOverPoints[ms->config.targetClass].push_back(thisRelative3dGrasp);
  }
}
END_WORD
REGISTER_WORD(AddPlaceOverPoint)

WORD(AssumeCurrent3dGrasp)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double p_backoffDistance = 0.10;
  int t3dGraspIndex = ms->config.current3dGraspIndex;
  cout << "assumCurrent3dGrasp, t3dGraspIndex: " << t3dGraspIndex << endl;

  eePose toApply = ms->config.class3dGrasps[ms->config.targetClass][t3dGraspIndex];  

  eePose thisBase = ms->config.lastLockedPose;
  thisBase.pz = -ms->config.currentTableZ;
  eePose graspPose = toApply.applyAsRelativePoseTo(thisBase);

  int increments = floor(p_backoffDistance / GRID_COARSE); 
  Vector3d localUnitX;
  Vector3d localUnitY;
  Vector3d localUnitZ;
  fillLocalUnitBasis(graspPose, &localUnitX, &localUnitY, &localUnitZ);
  eePose retractedGraspPose = graspPose.minusP(p_backoffDistance * localUnitZ);

  ms->config.currentEEPose = retractedGraspPose;
  ms->config.lastPickPose = graspPose;
  int tbb = ms->config.targetBlueBox;
  if (tbb < ms->config.blueBoxMemories.size()) {
    ms->config.blueBoxMemories[tbb].pickedPose = ms->config.lastPickPose;  
  } else {
    assert(0);
  }

  ms->pushWord("comeToStop"); 
  ms->pushWord("waitUntilAtCurrentPosition"); 

  ms->pushCopies("localZUp", increments);
  ms->pushWord("setGridSizeCoarse");
  ms->pushWord("approachSpeed");

  ms->pushWord("waitUntilAtCurrentPosition"); 

}
END_WORD
REGISTER_WORD(AssumeCurrent3dGrasp)

WORD(AssumeAny3dGrasp)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double p_backoffDistance = 0.10;

  for (int tc = 0; tc < ms->config.class3dGrasps[ms->config.targetClass].size(); tc++) {
    eePose toApply = ms->config.class3dGrasps[ms->config.targetClass][tc];  

    eePose thisBase = ms->config.lastLockedPose;
    thisBase.pz = -ms->config.currentTableZ;

    cout << "assumeAny3dGrasp, tc: " << tc << endl;

    // this order is important because quaternion multiplication is not commutative
    //ms->config.currentEEPose = ms->config.currentEEPose.plusP(ms->config.currentEEPose.applyQTo(toApply));
    //ms->config.currentEEPose = ms->config.currentEEPose.multQ(toApply);
    eePose graspPose = toApply.applyAsRelativePoseTo(thisBase);

    int increments = floor(p_backoffDistance / GRID_COARSE); 
    Vector3d localUnitX;
    Vector3d localUnitY;
    Vector3d localUnitZ;
    fillLocalUnitBasis(graspPose, &localUnitX, &localUnitY, &localUnitZ);
    eePose retractedGraspPose = graspPose.minusP(p_backoffDistance * localUnitZ);

    int ikResultPassedBoth = 1;
    {
      cout << "Checking IK for 3D grasp number " << tc << ", "; 
      int ikCallResult = 0;
      baxter_core_msgs::SolvePositionIK thisIkRequest;
      eePose toRequest = graspPose;
      fillIkRequest(toRequest, &thisIkRequest);
      queryIK(ms, &ikCallResult, &thisIkRequest);

      cout << ikCallResult << "." << endl;

      int ikResultFailed = 1;
      if (ikCallResult && thisIkRequest.response.isValid[0]) {
	ikResultFailed = 0;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].position.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough positions." << endl;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].name.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough names." << endl;
      } else if (thisIkRequest.response.joints.size() == 1) {
	if( ms->config.usePotentiallyCollidingIK ) {
	  cout << "WARNING: using ik even though result was invalid under presumption of false collision..." << endl;
	  cout << "Received enough positions and names for ikPose: " << thisIkRequest.request.pose_stamp[0].pose << endl;
	  ikResultFailed = 0;
	} else {
	  ikResultFailed = 1;
	  cout << "ik result was reported as colliding and we are sensibly rejecting it..." << endl;
	}
      } else {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, incorrect joint field." << endl;
      }

      ikResultPassedBoth = ikResultPassedBoth && (!ikResultFailed);
    }
    {
      cout << "Checking IK for 3D pre-grasp number " << tc << ", ";
      int ikCallResult = 0;
      baxter_core_msgs::SolvePositionIK thisIkRequest;
      eePose toRequest = retractedGraspPose;
      fillIkRequest(toRequest, &thisIkRequest);
      queryIK(ms, &ikCallResult, &thisIkRequest);

      cout << ikCallResult << "." << endl;

      int ikResultFailed = 1;
      if (ikCallResult && thisIkRequest.response.isValid[0]) {
	ikResultFailed = 0;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].position.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough positions." << endl;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].name.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough names." << endl;
      } else if (thisIkRequest.response.joints.size() == 1) {
	if( ms->config.usePotentiallyCollidingIK ) {
	  cout << "WARNING: using ik even though result was invalid under presumption of false collision..." << endl;
	  cout << "Received enough positions and names for ikPose: " << thisIkRequest.request.pose_stamp[0].pose << endl;
	  ikResultFailed = 0;
	} else {
	  ikResultFailed = 1;
	  cout << "ik result was reported as colliding and we are sensibly rejecting it..." << endl;
	}
      } else {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, incorrect joint field." << endl;
      }

      ikResultPassedBoth = ikResultPassedBoth && (!ikResultFailed);
    }

    if (ikResultPassedBoth) {
      cout << "Grasp and pre-grasp both passed, accepting." << endl;

      ms->config.currentEEPose = retractedGraspPose;
      ms->config.lastPickPose = graspPose;
      int tbb = ms->config.targetBlueBox;
      if (tbb < ms->config.blueBoxMemories.size()) {
	ms->config.blueBoxMemories[tbb].pickedPose = ms->config.lastPickPose;  
      } else {
	assert(0);
      }

      if (ms->config.snapToFlushGrasp) {
	// using twist and effort
	ms->pushWord("closeGripper");
	ms->pushWord("pressUntilEffortOrTwist");

	ms->pushWord("setTwistThresh");
	ms->pushWord("0.015");

	ms->pushWord("setEffortThresh");
	ms->pushWord("20.0");

	ms->pushWord("setSpeed");
	ms->pushWord("0.03");

	ms->pushWord("pressUntilEffortOrTwistInit");
	ms->pushWord("comeToStop");
	ms->pushWord("setMovementStateToMoving");
	ms->pushWord("comeToStop");
	ms->pushWord("waitUntilAtCurrentPosition");

	ms->pushWord("setSpeed");
	ms->pushWord("0.05");

	ms->pushWord("setGridSizeCoarse");
      } else {
	ms->pushWord("comeToStop"); 
	ms->pushWord("waitUntilAtCurrentPosition"); 

	ms->pushCopies("localZUp", increments);
	ms->pushWord("setGridSizeCoarse");
	ms->pushWord("approachSpeed");
      }

      ms->pushWord("waitUntilAtCurrentPosition"); 
      return;
    }
  }

  cout << "No 3D grasps were feasible. Continuing." << endl;
}
END_WORD
REGISTER_WORD(AssumeAny3dGrasp)


WORD(PreAnnotateCenterGrasp)
virtual void execute(std::shared_ptr<MachineState> ms) {
  zeroGraspMemoryAndRangeMap(ms);
  ms->config.graspMemoryTries[ms->config.rmHalfWidth + ms->config.rmHalfWidth*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*0] = 1;
  ms->config.graspMemoryPicks[ms->config.rmHalfWidth + ms->config.rmHalfWidth*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*0] = 1; 
  ms->config.rangeMap[ms->config.rmHalfWidth + ms->config.rmHalfWidth*ms->config.rmWidth] = ms->config.currentGraspZ;
  ms->config.rangeMapReg1[ms->config.rmHalfWidth + ms->config.rmHalfWidth*ms->config.rmWidth] = ms->config.currentGraspZ;
}
END_WORD
REGISTER_WORD(PreAnnotateCenterGrasp)

WORD(Annotate2dGrasp)
virtual void execute(std::shared_ptr<MachineState> ms) {

  // this is in mm's for now
  int gheight = 0;
  int tgg = 0;
  int Utgg = 0;
  int x = ms->config.rmHalfWidth;
  int y = ms->config.rmHalfWidth;

  GET_ARG(ms, IntegerWord, gheight);
  GET_ARG(ms, IntegerWord, Utgg);
  GET_ARG(ms, IntegerWord, y);
  GET_ARG(ms, IntegerWord, x);

  tgg = Utgg-1;

  tgg = min(max(0, tgg), 3);
  x = min(max(0, x), ms->config.rmWidth-1);
  y = min(max(0, y), ms->config.rmWidth-1);

  int class_idx = ms->config.focusedClass;
  cout << "annotate2dGrasp, class: " << class_idx << endl;
  if ( (class_idx > -1) && (class_idx < ms->config.classLabels.size()) ) {
    cout << "  annotating x y Utgg gheight, tgg: " << x << " " << y << " " << Utgg << " " << gheight << ", " << tgg << endl;
    guardGraspMemory(ms);
    zeroGraspMemoryAndRangeMap(ms);
    zeroClassGraspMemory(ms);
    ms->config.graspMemoryTries[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tgg] = 1;
    ms->config.graspMemoryPicks[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tgg] = 1; 
    ms->config.rangeMap[x + y*ms->config.rmWidth] = gheight;
    ms->config.rangeMapReg1[x + y*ms->config.rmWidth] = gheight;

    ms->config.classRangeMaps[ms->config.targetClass].at<double>(y,x) = -double(gheight)*0.001;
    ms->config.classGraspMemoryPicks1[ms->config.targetClass].at<double>(y,x) = 1;
    ms->config.classGraspMemoryPicks2[ms->config.targetClass].at<double>(y,x) = 1;
    ms->config.classGraspMemoryPicks3[ms->config.targetClass].at<double>(y,x) = 1;
    ms->config.classGraspMemoryPicks4[ms->config.targetClass].at<double>(y,x) = 1;
  } else {
    cout << "  invalid focused class, not annotating." << endl;
  }
}
END_WORD
REGISTER_WORD(Annotate2dGrasp)

WORD(CollectMoreCrops)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "About to collect more crops, make sure targetClass is set, i.e. you should have" << endl <<
    " \"<target class name>\" setTargetClass " << endl <<
    " in the repl before running this. " << endl;
    
  ms->pushWord("scanCentered"); 
  ms->pushWord("setPhotoPinHere");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("synchronicServo");
  ms->pushWord("synchronicServoTakeClosest");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("moveToMappingHeight");
  ms->pushWord("pauseStackExecution"); 
}
END_WORD
REGISTER_WORD(CollectMoreCrops)

WORD(PreAnnotateOffsetGrasp)
virtual void execute(std::shared_ptr<MachineState> ms) {
  zeroGraspMemoryAndRangeMap(ms);
  eePose offsetPose = ms->config.eepReg1;
  eePose difference = offsetPose.minusP(ms->config.currentEEPose);
  double offsetX = difference.px / ms->config.rmDelta;
  double offsetY = difference.py / ms->config.rmDelta;
  int rx = (int) round(ms->config.rmHalfWidth + offsetX);
  int ry = (int) round(ms->config.rmHalfWidth + offsetY);
  
  cout << "PreAnnotateOffsetGrasp: " << offsetPose << " " << ms->config.currentEEPose << " " << difference << endl <<
    offsetX << " " << offsetY << " " << rx << " " << ry << endl;

  int padding = ms->config.rangeMapTargetSearchPadding;
  if ( (rx < padding) || (ry < padding) || 
       (rx > ms->config.rmWidth-1-padding) || (ry > ms->config.rmWidth-1-padding) ) {
    ms->clearStack();
    cout << "Oops, annotation put the grasp point out of bounds. Clearing stack; you should delete this model. Try using setScanModeNotCentered." << endl;
    // we could push the other annotation here and go to eepReg1 to automatically recover from this
    return;
  } else {
  }


  ms->config.graspMemoryTries[rx + ry*ms->config.rmWidth + ms->config.rmWidth * ms->config.rmWidth * 0] = 1;
  ms->config.graspMemoryPicks[rx + ry*ms->config.rmWidth + ms->config.rmWidth * ms->config.rmWidth * 0] = 1;

  ms->config.rangeMap[rx + ry*ms->config.rmWidth] = ms->config.currentGraspZ;
  ms->config.rangeMapReg1[rx + ry*ms->config.rmWidth] = ms->config.currentGraspZ;
}
END_WORD
REGISTER_WORD(PreAnnotateOffsetGrasp)

WORD(HistogramDetectionIfBlueBoxes)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  if (ms->config.bTops.size() > 0) {
    ms->pushWord("replaceBlueBoxesWithHistogramWinner"); 
    ms->pushWord("histogramDetection");
  } else {
    cout << "No blue boxes detected, so not performing histogram detection." << endl;
  }
}
END_WORD
REGISTER_WORD(HistogramDetectionIfBlueBoxes)



WORD(HistogramDetection)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->pushWord("histogramDetectionReport");
  ms->pushWord("histogramDetectionNormalize");

  ms->pushWord("putCameraOverPhotoPin"); 
  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("shiftIntoGraspGear1");

  /* 
  ms->pushWord("yDown");
  ms->pushWord("detectionSpin");
  ms->pushWord("yUp");

  ms->pushWord("yUp");
  ms->pushWord("detectionSpin");
  ms->pushWord("yDown");

  ms->pushWord("xDown");
  ms->pushWord("detectionSpin");
  ms->pushWord("xUp");
  */

  ms->pushWord("xUp");
  ms->pushWord("detectionSpin");
  ms->pushWord("setPhotoPinHere");
  ms->pushWord("xDown");

  
  ms->pushWord("setPhotoPinHere");

  ms->pushWord("histogramDetectionInit");
}
END_WORD
REGISTER_WORD(HistogramDetection)

WORD(DetectionSpin)
virtual void execute(std::shared_ptr<MachineState> ms) {
  for (int angleCounter = 0; angleCounter < ms->config.totalGraspGears; angleCounter++) {
    //ms->pushWord("histgramAllExamplesFocusedClass");
    ms->pushWord("histogramExampleAsFocusedClass");
    ms->pushWord("visionCycle"); // vision cycle
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord("putCameraOverPhotoPin"); 
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord("setRandomOrientationForPhotospin"); 
    ms->pushWord("incrementGraspGear"); 
  }
  ms->pushWord("shiftIntoGraspGear1"); // change gear to 1
}
END_WORD
REGISTER_WORD(DetectionSpin)

WORD(HistogramExampleAsFocusedClass)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  recordBlueBoxInHistogram(ms, ms->config.pilotClosestBlueBoxNumber);
  ms->pushWord("histogramDetectionReport");
  ms->pushWord("histogramDetectionNormalize");
}
END_WORD
REGISTER_WORD(HistogramExampleAsFocusedClass)

WORD(HistgramAllExamplesFocusedClass)
virtual void execute(std::shared_ptr<MachineState> ms)       {
  for (int c = 0; c < ms->config.bTops.size(); c++) {
    recordBlueBoxInHistogram(ms, c);
  }
  ms->pushWord("histogramDetectionReport");
  ms->pushWord("histogramDetectionNormalize");
}
END_WORD
REGISTER_WORD(HistgramAllExamplesFocusedClass)

WORD(HistogramDetectionInit)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  int thisNC = ms->config.classLabels.size(); 
  ms->config.chHistogram.create(1, thisNC, CV_64F);
  ms->config.chDistribution.create(1, thisNC, CV_64F);
  ms->config.chWinner = -1;

  for (int i = 0; i < thisNC; i++) {
    ms->config.chHistogram.at<double>(0,i) = 0.0;
  }
}
END_WORD
REGISTER_WORD(HistogramDetectionInit)

WORD(HistogramDetectionNormalize)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  computeClassificationDistributionFromHistogram(ms);
}
END_WORD
REGISTER_WORD(HistogramDetectionNormalize)

WORD(HistogramDetectionReport)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  cout << "Histogam Results: " << endl;
  int thisNC = ms->config.chHistogram.cols; 
  assert( thisNC == ms->config.chDistribution.cols );

  int maxClass = -1;
  double maxClassScore = -1;
  for (int i = 0; i < thisNC; i++) {
    double thisScore = ms->config.chDistribution.at<double>(0,i);
    double thisCounts = ms->config.chHistogram.at<double>(0,i);
    cout << ms->config.classLabels[i] << ": Probability " << thisScore << ", Counts " << thisCounts << endl;
    if (thisScore > maxClassScore) {
      maxClass = i;
      maxClassScore = thisScore;
    } else {
    }
  }

  cout << endl << "Winner: " << ms->config.classLabels[maxClass] << ", " << maxClassScore << endl << endl;
  ms->config.chWinner = maxClass;
}
END_WORD
REGISTER_WORD(HistogramDetectionReport)

CONFIG_GETTER_INT(FakeBBWidth, ms->config.fakeBBWidth)
CONFIG_SETTER_INT(SetFakeBBWidth, ms->config.fakeBBWidth)

WORD(ReplaceBlueBoxesWithFocusedClass)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  if (isFocusedClassValid(ms) && ms->config.bTops.size() > 0) {
    cout << "replaceBlueBoxesWithFocusedClass: Focused class is valid, replacing bTops etc." << endl;
    ms->config.bTops.resize(1);
    ms->config.bBots.resize(1);
    ms->config.bCens.resize(1);
    ms->config.bLabels.resize(1);


    ms->config.bTops[0].x = ms->config.vanishingPointReticle.px - ms->config.fakeBBWidth;
    ms->config.bTops[0].y = ms->config.vanishingPointReticle.py - ms->config.fakeBBWidth;
    ms->config.bBots[0].x = ms->config.vanishingPointReticle.px + ms->config.fakeBBWidth;
    ms->config.bBots[0].y = ms->config.vanishingPointReticle.py + ms->config.fakeBBWidth;

    ms->config.bCens[0].x = (ms->config.bTops[0].x + ms->config.bBots[0].x)/2.0;
    ms->config.bCens[0].y = (ms->config.bTops[0].y + ms->config.bBots[0].y)/2.0;

    ms->config.bLabels[0] = ms->config.focusedClass;

    ms->config.pilotClosestBlueBoxNumber = 0;
  } else {
    cout << "replaceBlueBoxesWithFocusedClass: Focused class invalid, clearing bTops etc." << endl;
    ms->config.bTops.resize(0);
    ms->config.bBots.resize(0);
    ms->config.bCens.resize(0);
    ms->config.bLabels.resize(0);
    ms->config.pilotClosestBlueBoxNumber = -1;
  }
}
END_WORD
REGISTER_WORD(ReplaceBlueBoxesWithFocusedClass)


WORD(ReplaceBlueBoxesWithHistogramWinner)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  if (ms->config.chWinner > -1) {
    cout << "Replacing blue boxes with histogram winner..." << endl;
    ms->config.bTops.resize(1);
    ms->config.bBots.resize(1);
    ms->config.bCens.resize(1);
    ms->config.bLabels.resize(1);


    ms->config.bTops[0].x = ms->config.vanishingPointReticle.px - ms->config.fakeBBWidth;
    ms->config.bTops[0].y = ms->config.vanishingPointReticle.py - ms->config.fakeBBWidth;
    ms->config.bBots[0].x = ms->config.vanishingPointReticle.px + ms->config.fakeBBWidth;
    ms->config.bBots[0].y = ms->config.vanishingPointReticle.py + ms->config.fakeBBWidth;

    ms->config.bCens[0].x = (ms->config.bTops[0].x + ms->config.bBots[0].x)/2.0;
    ms->config.bCens[0].y = (ms->config.bTops[0].y + ms->config.bBots[0].y)/2.0;

    ms->config.bLabels[0] = ms->config.chWinner;

    ms->config.pilotClosestBlueBoxNumber = 0;
  } else {
    cout << "There is no histogram winner so clearing blue boxes..." << endl;
    ms->config.bTops.resize(0);
    ms->config.bBots.resize(0);
    ms->config.bCens.resize(0);
    ms->config.bLabels.resize(0);
    ms->config.pilotClosestBlueBoxNumber = -1;
  }
}
END_WORD
REGISTER_WORD(ReplaceBlueBoxesWithHistogramWinner)

WORD(WriteAlphaObjectToBetaFolders)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string newClassName;
  string oldClassName;
  GET_ARG(ms, StringWord, newClassName);
  GET_ARG(ms, StringWord, oldClassName);
  {
    int class_idx = classIdxForName(ms, oldClassName);
    if (class_idx != -1) {
      cout << "About to write data for class \"" << ms->config.classLabels[class_idx] << "\" index " << class_idx << " to folder \"" << ms->config.data_directory + "/" + newClassName << "\", unpause to proceed." << endl;
      ms->pushWord("writeAlphaObjectToBetaFoldersA");
      ms->pushWord(newClassName);
      ms->pushWord(oldClassName);

      ms->pushWord("pauseStackExecution");
    } else {
      cout << "No class for " << oldClassName << " for " << this->name() << endl;
    }
  }
}
END_WORD
REGISTER_WORD(WriteAlphaObjectToBetaFolders)

WORD(WriteAlphaObjectToBetaFoldersA)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string newClassName;
  string oldClassName;
  GET_ARG(ms, StringWord, newClassName);
  GET_ARG(ms, StringWord, oldClassName);
  {
    int class_idx = classIdxForName(ms, oldClassName);
    if (class_idx != -1) {
      cout << "Writing data for class \"" << ms->config.classLabels[class_idx] << "\" index " << class_idx << " to folder \"" << ms->config.data_directory + "/" + newClassName << "\"." << endl;
      writeClassToFolder(ms, class_idx, ms->config.data_directory + "/" + newClassName);
    } else {
      cout << "No class for " << oldClassName << " for " << this->name() << endl;
    }
  }
}
END_WORD
REGISTER_WORD(WriteAlphaObjectToBetaFoldersA)

WORD(RetrainVocabOn)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "retrain_vocab turned on,  was: " << ms->config.retrain_vocab << ", is: ";
  ms->config.retrain_vocab = 1;
  cout << ms->config.retrain_vocab << endl;
}
END_WORD
REGISTER_WORD(RetrainVocabOn)

WORD(RetrainVocabOff)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "retrain_vocab turned off,  was: " << ms->config.retrain_vocab << ", is: ";
  ms->config.retrain_vocab = 0;
  cout << ms->config.retrain_vocab << endl;
}
END_WORD
REGISTER_WORD(RetrainVocabOff)

WORD(ReinitRangeMaps)
virtual void execute(std::shared_ptr<MachineState> ms) {
  initRangeMaps(ms);
}
END_WORD
REGISTER_WORD(ReinitRangeMaps)

WORD(ResetCurrentFocusedClass)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int class_idx = ms->config.focusedClass;
  cout << "resetCurrentFocusedClass: " << class_idx << endl;
  if ( (class_idx > -1) && (class_idx < ms->config.classLabels.size()) ) {
    changeTargetClass(ms, class_idx);
  } else {
    cout << "  invalid focused class, not resetting." << endl;
  }
}
END_WORD
REGISTER_WORD(ResetCurrentFocusedClass)

WORD(SetScanModeCentered)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentScanMode = CENTERED;
  cout << "Setting currentScanMode to CENTERED: " << ms->config.currentScanMode << endl;
}
END_WORD
REGISTER_WORD(SetScanModeCentered)

WORD(SetScanModeNotCentered)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.currentScanMode = NOT_CENTERED;
  cout << "Setting currentScanMode to NOT_CENTERED: " << ms->config.currentScanMode << endl;
}
END_WORD
REGISTER_WORD(SetScanModeNotCentered)




WORD(BuildClassSimilarityMatrix)
virtual string description() {
  return "Builds the matrix of gradients of the current class labels.";
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  double * result =  new double[ms->config.numClasses * ms->config.numClasses];
  int nc = ms->config.numClasses;

  for (int i = 0; i < ms->config.classLabels.size(); i++) {
    for (int j = 0; j < ms->config.classLabels.size(); j++) {
      result[i + nc * j] = computeSimilarity(ms, i, j);
    }
  }

  for (int j = 0; j < ms->config.classLabels.size(); j++) {
    cout << std::setw(3) << j << ": " << ms->config.classLabels[j] << endl;
  }
  cout << "   " ;
  for (int j = 0; j < ms->config.classLabels.size(); j++) {
    cout << std::setw(10) << j ;
  }
  cout << endl;

  for (int i = 0; i < ms->config.classLabels.size(); i++) {
    cout << std::setw(3) << i;
    for (int j = 0; j < ms->config.classLabels.size(); j++) {
      cout << std::setw(10) << result[i + nc * j];
    }
    cout << endl;
  }
}
END_WORD
REGISTER_WORD(BuildClassSimilarityMatrix)

WORD(BuildClassSimilarityMatrixFromDensity)
virtual string description() {
  return "Builds the matrix of gradients of the current class labels.";
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  double * result =  new double[ms->config.numClasses];
  int nc = ms->config.numClasses;

  Mat frameFromDensity = makeGCrop(ms, 0, 0);

  for (int i = 0; i < ms->config.classLabels.size(); i++) {
    result[i] = computeSimilarity(ms, frameFromDensity, ms->config.classHeight1AerialGradients[i]);
  }

  for (int j = 0; j < ms->config.classLabels.size(); j++) {
    cout << std::setw(3) << j << ": " << ms->config.classLabels[j] << endl;
  }

  cout << endl;

  double max_of_classes = 0;

  for (int i = 0; i < ms->config.classLabels.size(); i++) {
    cout << std::setw(3) << i;
    cout << std::setw(10) << result[i];
    cout << endl;
    max_of_classes = max(max_of_classes, result[i]);
  }

  ms->pushWord(make_shared<DoubleWord>(max_of_classes));
}
END_WORD
REGISTER_WORD(BuildClassSimilarityMatrixFromDensity)

WORD(IrFixPick)
virtual string description() {
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "Commencing IR pick fix." << endl;

  // move back to the pose we were in 
  //ms->pushWord("moveEeToPoseWord");


  // XXX move to the best point and stay there, or whatever should be done to substitute the grasp

  // XXX todo write the little integrator
  //ms->pushWord("integrateRangeStreamBuffer");

  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 

  //ms->pushWord("neutralScanH");
  // XXX todo write the little scan program
  
  ms->pushWord("activateSensorStreaming"); 
  ms->pushWord("clearStreamBuffers"); 
  ms->pushWord("shutdownToSensorsAndMovement"); 
  
  ms->pushWord("setSisFlags"); 

  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(0));
  ms->pushWord(std::make_shared<IntegerWord>(1));
  ms->pushWord(std::make_shared<IntegerWord>(1));



  ms->pushWord("disableDiskStreaming");


  ms->pushWord("setPickFixMapAnchor");



  // move IR sensor onto trajectory line in correct orientation
  ms->pushWord("comeToStop");
  ms->pushWord("setGridSize");
  ms->pushWord(make_shared<DoubleWord>(0.01));

  ms->pushWord("oZUp");
  ms->pushWord("setGridSize");
  ms->pushWord(make_shared<DoubleWord>(3.1415926 / 2.0));
  
  // push current pose so we can return
  //ms->pushWord("currentPose");
}
END_WORD
REGISTER_WORD(IrFixPick)

WORD(SetPickFixMapAnchor)
virtual string description() {
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.pfmAnchorPose = ms->config.currentEEPose;
}
END_WORD
REGISTER_WORD(SetPickFixMapAnchor)

WORD(ClearClass3dGrasps)
virtual string description() {
}
virtual void execute(std::shared_ptr<MachineState> ms) {
  int class_idx = ms->config.focusedClass;
  cout << "clearClass3dGrasps: " << class_idx << endl;
  if ( (class_idx > -1) && (class_idx < ms->config.classLabels.size()) ) {
    ms->config.class3dGrasps[class_idx].resize(0);
  } else {
    cout << "  invalid focused class, not clearing." << endl;
  }
}
END_WORD
REGISTER_WORD(ClearClass3dGrasps)



}
