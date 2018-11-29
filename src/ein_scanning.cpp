#include <object_recognition_msgs/RecognizedObjectArray.h>

#include "eigen_util.h"
#include "ein_words.h"

#include "ein.h"
#include <dirent.h>
#include "camera.h"
#include "qtgui/einwindow.h"
#include <boost/filesystem.hpp>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>

using namespace std;
using namespace boost::filesystem;




void initializeAndFocusOnTempClass(MachineState * ms) {
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
      // do not make dir
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
      // do not make dir
      collision = 0;
      thisLabelName = thisLabelName + the_suffix;  
    }
  }
  ms->config.focusedClassLabel = thisLabelName;
  ms->config.classLabels.push_back(thisLabelName);
  ms->config.numClasses = ms->config.classLabels.size();

  // normal initRangeMaps here would try to load nonexistant material for and clobber previous temp objects
  guardSceneModels(ms);
  guard3dGrasps(ms);
}


void initializeAndFocusOnNewClass(MachineState * ms) {
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
      try {
	create_directories(dirToMakePath);
      } catch( ... ) {
	ROS_ERROR_STREAM("Could not create directory: " << dirToMakePath);
      }
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
      cout << "Creating " << dirToMakePath << endl;
      try {
	create_directories(dirToMakePath);
      } catch( ... ) {
	ROS_ERROR_STREAM("Could not create directory  " << dirToMakePath);
      }
      collision = 0;
      thisLabelName = thisLabelName + the_suffix;  
    }
  }
  ms->config.focusedClassLabel = thisLabelName;
  ms->config.classLabels.push_back(thisLabelName);
  ms->config.numClasses = ms->config.classLabels.size();

  guardSceneModels(ms);
  guard3dGrasps(ms);

  int idx = ms->config.focusedClass;
  string folderName = ms->config.data_directory + "/objects/" + ms->config.classLabels[idx] + "/";
  initClassFolders(ms, folderName);
}


namespace ein_words {


WORD(SetFocusedClass)
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("setTargetClass");
  return result;
}
virtual void execute(MachineState * ms) {
  string className;
  GET_ARG(ms, StringWord, className);

  int class_idx = classIdxForName(ms, className);
  changeTargetClass(ms, class_idx);
}
END_WORD
REGISTER_WORD(SetFocusedClass)

WORD(SetFocusedClassIdx)
virtual vector<string> names() {
  vector<string> result;
  result.push_back(name());
  result.push_back("setTargetClassIdx");
  return result;
}
virtual void execute(MachineState * ms) {
  int class_idx;
  GET_INT_ARG(ms, class_idx);
  changeTargetClass(ms, class_idx);
}
END_WORD
REGISTER_WORD(SetFocusedClassIdx)




WORD(SetLastLabelLearned)
CODE(1179732)    // capslock + numlock + t 
virtual void execute(MachineState * ms) {
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

virtual void execute(MachineState * ms)       {
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
virtual string description() {
  return "Print class labels to standard output.";
}
virtual void execute(MachineState * ms)       {
  CONSOLE(ms, "printClassLabels: " << ms->config.classLabels.size());

  for (int i = 0; i < ms->config.classLabels.size(); i++) {
    CONSOLE(ms, i << ": " << ms->config.classLabels[i]);
  }
}
END_WORD
REGISTER_WORD(PrintClassLabels)


WORD(PushClassLabelsReport)
virtual void execute(MachineState * ms)       {
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

CONFIG_GETTER_STRING(FocusedClassLabel, ms->config.focusedClassLabel, "The focused class.")

WORD(PushClassLabels)
virtual string description() {
  return "Push the class labels on the stack.";
}
virtual void execute(MachineState * ms)       {
  for (int i = 0; i < ms->config.classLabels.size(); i++) {
    shared_ptr<StringWord> outword = std::make_shared<StringWord>(ms->config.classLabels[i]);
    ms->pushWord(outword);
  }
}
END_WORD
REGISTER_WORD(PushClassLabels)


WORD(ReloadClassLabels)
virtual void execute(MachineState * ms)  {

  ms->pushWord("setTargetClassIdx");
  ms->pushWord(make_shared<IntegerWord>(ms->config.focusedClass));
  ms->pushWord("setClassLabels");
  for (int i = 0; i < ms->config.classLabels.size(); i++) {
    ms->pushWord(make_shared<StringWord>(ms->config.classLabels[i]));
  }
  ms->pushWord("endArgs");

}
END_WORD
REGISTER_WORD(ReloadClassLabels)


WORD(ClearClassLabels)
virtual void execute(MachineState * ms) {
  ms->config.classLabels.resize(0);
  ms->config.classPoseModels.resize(0);
  ms->pushWord("clearBlueBoxMemories");
  ms->config.numClasses = ms->config.classLabels.size();
}
END_WORD
REGISTER_WORD(ClearClassLabels)


WORD(SetClassLabels)
virtual string description() {
  return "Set the active classes.  useage:  endArgs \"class1\" \"class2\" setClassLabels.  The class names must be directorys in the ein/default/objects folder.";
}
virtual void execute(MachineState * ms)  {

  cout << "entering setClassLabels." << endl;

  vector<string> newLabels;

  int more_args = 1;
  while(more_args) {
    shared_ptr<Word> bWord = ms->popData();

    if (bWord == NULL) {
      CONSOLE_ERROR(ms, "oops, setClassLabels requires a number of StringWords followed by endArgs...");
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
      CONSOLE_ERROR(ms, "rejecting a word, not resetting labels, and pausing the stack. probably forgot endArgs.");
      ms->pushWord("pauseStackExecution");
      return;
    } else {
      string thisLabel = bStringWord->to_string();
      if (thisLabel.length() > 0) {
	newLabels.push_back(thisLabel);
      } else {
	CONSOLE_ERROR(ms, "found a string word of length 0, not resetting labels, and pausing the stack.");
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
    changeTargetClass(ms, 0);
  } else {
    CONSOLE_ERROR(ms, "didn't get any valid labels, are you sure this is what you want?");
    ms->config.classLabels.resize(0);
    ms->config.classPoseModels.resize(0);
    ms->pushWord("clearBlueBoxMemories");
    ms->config.numClasses = ms->config.classLabels.size();
    changeTargetClass(ms, 0);
    return;
  }
}
END_WORD
REGISTER_WORD(SetClassLabels)

WORD(SetClassLabelsBaseClassAbsolute)

virtual void execute(MachineState * ms)  {
  string baseClassPath;
  GET_STRING_ARG(ms, baseClassPath);

  int last = baseClassPath.size()-1;
  // find last non-slash character
  if (baseClassPath[last] == '/') {
    last = last-1;
    while (last > -1) {
      if (baseClassPath[last] == '/') {
	last = last-1;
      } else {
	break;
      }
    }
  }
  // then find the character after the next slash
  int first = last-1;
  while (first > -1) {
    if (baseClassPath[first] == '/') {
      break;
    } else {
      first = first-1;
    }
  }
  first = first+1;
  // then take substring
  string baseClassName = baseClassPath.substr(first, last-first+1);

  ms->pushWord("setClassLabels");

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  dpdf = opendir(baseClassPath.c_str());
  if (dpdf != NULL){
    cout << "setClassLabelsBaseClassAbsolute: checking " << baseClassPath << " during snoop...";
    while (epdf = readdir(dpdf)){
      string thisFileName(epdf->d_name);

      string thisFullFileName(baseClassPath.c_str());
      thisFullFileName = thisFullFileName + "/" + thisFileName;
      cout << "setClassLabelsBaseClassAbsolute: checking " << thisFullFileName << " during snoop...";

      struct stat buf2;
      stat(thisFullFileName.c_str(), &buf2);

      string varianceTrials("catScan5VarianceTrials");
      stringstream ss;
      ss << baseClassName << "/" << epdf->d_name;
      string newClassName = ss.str();

      int itIsADir = S_ISDIR(buf2.st_mode);
      if (varianceTrials.compare(epdf->d_name) && dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && itIsADir) {
	cout << " is a directory." << endl;
	ms->pushWord(make_shared<StringWord>(newClassName));
      } else {
	cout << " is NOT a directory." << endl;
      }
    }
  } else {
    ROS_ERROR_STREAM("setClassLabelsBaseClassAbsolute: could not open base class dir " << baseClassPath << " ." << endl);
  } 

  ms->pushWord("endArgs");
}
END_WORD
REGISTER_WORD(SetClassLabelsBaseClassAbsolute)

WORD(SetClassLabelsObjectFolderAbsolute)
virtual void execute(MachineState * ms)  {
  string objectFolderAbsolute;
  GET_STRING_ARG(ms, objectFolderAbsolute);

  int last = objectFolderAbsolute.size()-1;
  // find last non-slash character
  if (objectFolderAbsolute[last] == '/') {
    last = last-1;
    while (last > -1) {
      if (objectFolderAbsolute[last] == '/') {
	last = last-1;
      } else {
	break;
      }
    }
  }
  // then find the character after the next slash
  int first = last-1;
  while (first > -1) {
    if (objectFolderAbsolute[first] == '/') {
      break;
    } else {
      first = first-1;
    }
  }
  first = first+1;
  // then take substring
  string objectFolderName = objectFolderAbsolute.substr(first, last-first+1);


  ms->pushWord("setClassLabels");

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  dpdf = opendir(objectFolderAbsolute.c_str());
  if (dpdf != NULL){
    cout << "setClassLabelsObjectFolderAbsolute level 1: checking " << objectFolderAbsolute << " during snoop...";
    while (epdf = readdir(dpdf)){
      string thisFileName(epdf->d_name);

      string thisFullFileName(objectFolderAbsolute.c_str());
      thisFullFileName = thisFullFileName + "/" + thisFileName;
      cout << "setClassLabelsObjectFolderAbsolute level 1: checking " << thisFullFileName << " during snoop...";

      struct stat buf2;
      stat(thisFullFileName.c_str(), &buf2);

      int itIsADir = S_ISDIR(buf2.st_mode);
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && itIsADir) {
	cout << " is a directory." << endl;

	{
	  string thisFolderName = thisFullFileName;
	  DIR *dpdf_2;
	  struct dirent *epdf_2;

	  dpdf_2 = opendir(thisFolderName.c_str());
	  if (dpdf_2 != NULL){
	    cout << "setClassLabelsObjectFolderAbsolute level 2: checking " << thisFolderName << " during snoop...";
	    while (epdf_2 = readdir(dpdf_2)){
	      string thisFileName_2(epdf_2->d_name);

	      string thisFullFileName_2(thisFolderName.c_str());
	      thisFullFileName_2 = thisFullFileName_2 + "/" + thisFileName_2;
	      cout << "setClassLabelsObjectFolderAbsolute level 2: checking " << thisFullFileName_2 << " during snoop...";

	      struct stat buf2;
	      stat(thisFullFileName_2.c_str(), &buf2);

	      string varianceTrials("catScan5VarianceTrials");

	      int itIsADir_2 = S_ISDIR(buf2.st_mode);
	      if (varianceTrials.compare(epdf_2->d_name) && dot.compare(epdf_2->d_name) && dotdot.compare(epdf_2->d_name) && itIsADir_2) {
		cout << " is a directory." << endl;
		stringstream ss;
		ss << objectFolderName << "/" << epdf->d_name << "/" << epdf_2->d_name;
		ms->pushWord(make_shared<StringWord>(ss.str()));
	      } else {
		cout << " is NOT a directory." << endl;
	      }
	    }
	  } else {
	    ROS_ERROR_STREAM("setClassLabelsObjectFolderAbsolute level 2: could not open base class dir " << objectFolderAbsolute << " ." << endl);
	  } 
	}

      } else {
	cout << " is NOT a directory." << endl;
      }
    }
  } else {
    ROS_ERROR_STREAM("setClassLabelsObjectFolderAbsolute level 1: could not open base class dir " << objectFolderAbsolute << " ." << endl);
  } 

  ms->pushWord("endArgs");
}
END_WORD
REGISTER_WORD(SetClassLabelsObjectFolderAbsolute)


WORD(TrainModelsFromLabels)
virtual void execute(MachineState * ms)       {
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
virtual void execute(MachineState * ms)       {
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
virtual void execute(MachineState * ms)       {

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
    ms->config.kNN = ml::KNearest::create();
    cout << "done." << endl;
    for (int i = 0; i < ms->config.numClasses; i++) {
      if (ms->config.classPoseModels[i].compare("G") == 0) {
	cout << "Class " << i << " kNN..." << ms->config.classPosekNNfeatures[i].size() << ms->config.classPosekNNlabels[i].size() << endl;
	ms->config.classPosekNNs[i] = ml::KNearest::create(); //new CvKNearest(ms->config.classPosekNNfeatures[i], ms->config.classPosekNNlabels[i]);
	cout << "Done" << endl;
      }
    }
  }

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
virtual void execute(MachineState * ms)       {
  ms->pushWord("mapEmptySpace");
  ms->pushWord("goFindBlueBoxes"); // blue boxes
  ms->pushCopies("density", 1); // density
  ms->pushWord("hover"); // blue boxes
}
END_WORD
REGISTER_WORD(VisionCycleNoClassify)

WORD(RecordExampleAsFocusedClass)
CODE(131148)     // capslock + l 
virtual void execute(MachineState * ms)       {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  cout << "recordExamplesFocusedClass is deprecated." << endl;
  if ((ms->config.focusedClass > -1) && (ms->config.bTops.size() == 1)) {
    string thisLabelName = ms->config.focusedClassLabel;
    Mat crop = camera->cam_img(cv::Rect(ms->config.bTops[0].x, ms->config.bTops[0].y, ms->config.bBots[0].x-ms->config.bTops[0].x, ms->config.bBots[0].y-ms->config.bTops[0].y));
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
virtual void execute(MachineState * ms)       {
  cout << "recordAllExamplesFocusedClass is deprecated." << endl;
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  if ( ms->config.focusedClass > -1 ) {
    for (int c = 0; c < ms->config.bTops.size(); c++) {
      string thisLabelName = ms->config.focusedClassLabel;
      Mat crop = camera->cam_img(cv::Rect(ms->config.bTops[c].x, ms->config.bTops[c].y, ms->config.bBots[c].x-ms->config.bTops[c].x, ms->config.bBots[c].y-ms->config.bTops[c].y));
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
virtual void execute(MachineState * ms) {
  // this ensures that we explore randomly within each grasp gear sector
  double arcFraction = 0.125;
  double noTheta = arcFraction * 3.1415926 * ((drand48() - 0.5) * 2.0);
  ms->config.currentEEDeltaRPY.pz += noTheta;
}
END_WORD
REGISTER_WORD(SetRandomOrientationForPhotospin)

WORD(RgbScan)
CODE(131143)      // capslock + g
virtual void execute(MachineState * ms)       {
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
virtual void execute(MachineState * ms) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  ms->config.photoPinPose = pixelToGlobalEEPose(ms, camera->vanishingPointReticle.px, camera->vanishingPointReticle.py, ms->config.trueEEPoseEEPose.pz + ms->config.currentTableZ);
}
END_WORD
REGISTER_WORD(SetPhotoPinHere)

WORD(PutCameraOverPhotoPin)
virtual void execute(MachineState * ms) {
  // XXX TODO avoid two steps by using alternate pixelToGlobal and not waiting between
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  eePose underVanishingPointReticle = pixelToGlobalEEPose(ms, camera->vanishingPointReticle.px, camera->vanishingPointReticle.py, ms->config.trueEEPoseEEPose.pz + ms->config.currentTableZ);
  eePose toMove = ms->config.photoPinPose.minusP(underVanishingPointReticle);
  eePose nextCurrentPose = ms->config.currentEEPose.plusP(toMove);
  ms->config.currentEEPose.px = nextCurrentPose.px;
  ms->config.currentEEPose.py = nextCurrentPose.py;
}
END_WORD
REGISTER_WORD(PutCameraOverPhotoPin)

WORD(PhotoSpin)
CODE(196711)      // capslock + G
virtual void execute(MachineState * ms) {
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

WORD(InitializeAndFocusOnNewClass)
virtual string description() {
  return "Initialize a new class with a default date-based name and focus on it.  It will be created in the file system with default values and focused on.";
}
CODE(196720)     // capslock + P
virtual void execute(MachineState * ms) {
  initializeAndFocusOnNewClass(ms);
}
END_WORD
REGISTER_WORD(InitializeAndFocusOnNewClass)

WORD(InitializeAndFocusOnTempClass)
virtual void execute(MachineState * ms) {

  initializeAndFocusOnTempClass(ms);

}
END_WORD
REGISTER_WORD(InitializeAndFocusOnTempClass)


WORD(RenameFocusedClass)
virtual void execute(MachineState * ms) {
  string newname;
  GET_STRING_ARG(ms, newname);

  int idx = ms->config.focusedClass;

  if ((idx <= -1) || (idx >= ms->config.classLabels.size())) {
    CONSOLE_ERROR(ms, "writeFocusedClass: invalid idx, not writing.");
    ms->pushWord("pauseStackExecution");
    return;
  }

  string oldfolder = ms->config.data_directory + "/objects/" + ms->config.classLabels[idx] + "/";
  string newfolder = ms->config.data_directory + "/objects/" + newname + "/";
  try {
    rename(oldfolder, newfolder);
  } catch (boost::filesystem::filesystem_error e) {
    CONSOLE_ERROR(ms, "Could not rename focused class to " << newname << " with old folder: " << oldfolder << " and new folder: " << newfolder << " because of exception: " << e.what());
  }
  ms->config.classLabels[idx] = newname;
  ms->pushWord("reloadClassLabels");

}
END_WORD
REGISTER_WORD(RenameFocusedClass)


WORD(WriteFocusedClass)
virtual void execute(MachineState * ms) {
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

WORD(WriteFocusedClassGrasps)
virtual void execute(MachineState * ms) {
  int idx = ms->config.focusedClass;

  if ((idx > -1) && (idx < ms->config.classLabels.size())) {
    // do nothing
  } else {
    cout << "writeFocusedClass: invalid idx, not writing." << endl;
    return;
  }

  string outfolder = ms->config.data_directory + "/objects/" + ms->config.classLabels[idx] + "/";
  writeClassGraspsToFolder(ms, idx, outfolder);
}
END_WORD
REGISTER_WORD(WriteFocusedClassGrasps)


WORD(ScanCentered)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  ms->config.firstTableHeightTime = ros::Time::now();
  ms->config.mostRecentUntabledZLastValue = ms->config.mostRecentUntabledZ;
  ms->pushWord("setTableA");
}
END_WORD
REGISTER_WORD(SetTable)

WORD(SetTableA)
virtual void execute(MachineState * ms) {
  ros::Time thisTableHeightTime = ros::Time::now();

  ms->config.mostRecentUntabledZLastValue = ((1.0-ms->config.mostRecentUntabledZDecay)*ms->config.mostRecentUntabledZ) + (ms->config.mostRecentUntabledZDecay*ms->config.mostRecentUntabledZLastValue);

  ms->config.oneTable = ms->config.mostRecentUntabledZLastValue;
  ms->config.rightTableZ = ms->config.oneTable;
  ms->config.leftTableZ = ms->config.oneTable;
  ms->config.bagTableZ = ms->config.oneTable;
  ms->config.counterTableZ = ms->config.oneTable;
  ms->config.pantryTableZ = ms->config.oneTable;
  ms->config.currentTableZ = ms->config.oneTable;
  double waited = fabs((thisTableHeightTime - ms->config.firstTableHeightTime).toSec());
  if ( waited > ms->config.mostRecentUntabledZWait) {
    // do nothing
    CONSOLE(ms, "Set table to " << ms->config.mostRecentUntabledZLastValue);
  } else {
    double utZDelta = fabs(ms->config.mostRecentUntabledZ - ms->config.mostRecentUntabledZLastValue);
    ms->config.endThisStackCollapse = 1;
    ms->pushWord("setTableA");
    CONSOLE(ms, "Waiting " << setprecision(2) << waited << "s/" << ms->config.mostRecentUntabledZWait << "s for table reading to stabilize.");
    CONSOLE(ms, " current: " << std::left << setw(5) << setprecision(3) << ms->config.mostRecentUntabledZ 
            << "m last: " << std::left<< setw(5) << setprecision(3) << ms->config.mostRecentUntabledZLastValue << "m delta:" << std::left<< setw(5) << setprecision(3) << utZDelta << "m.");
  } 
}
END_WORD
REGISTER_WORD(SetTableA)

WORD(SetIROffset)
virtual void execute(MachineState * ms) {
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


WORD(MoveCropToCenter)
virtual void execute(MachineState * ms) {
  Size sz = ms->config.accumulatedImage.size();
  int imW = sz.width;
  int imH = sz.height;
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  camera->cropUpperLeftCorner.px = 320;
  camera->cropUpperLeftCorner.py = 200;
  ms->pushWord("moveCropToProperValue");
}
END_WORD
REGISTER_WORD(MoveCropToCenter)

WORD(MoveCropToProperValue)
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("subscribeCameraParameterTrackerToRosOut 0.25 waitForSeconds moveCropToProperValueNoUpdate 0.25 waitForSeconds unsubscribeCameraParameterTrackerToRosOut");
}
END_WORD
REGISTER_WORD(MoveCropToProperValue)




WORD(FixCameraLightingToAutomaticParameters)
virtual string description() {
  return "Fix the camera lighting using autogain.  The camera parameters will first adjust automatically, then ein will fix them to the automatically adjusted values.";
}
virtual void execute(MachineState * ms) {
  stringstream p;
  p << "subscribeCameraParameterTrackerToRosOut 0.25 waitForSeconds ";
  p << "unFixCameraLightingNoUpdate 0.5 waitForSeconds  fixCameraLightingToObservedValues ";
  p << "0.5 waitForSeconds unsubscribeCameraParameterTrackerToRosOut ";
  ms->evaluateProgram(p.str());
}
END_WORD
REGISTER_WORD(FixCameraLightingToAutomaticParameters)


WORD(FixCameraLightingToObservedValues)
virtual void execute(MachineState * ms) {
  stringstream p;
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  p << camera->observedCameraExposure << " " << camera->observedCameraGain << " " << camera->observedCameraWhiteBalanceRed << " " << camera->observedCameraWhiteBalanceGreen << " " << camera->observedCameraWhiteBalanceBlue << " fixCameraLightingNoUpdate ";
  CONSOLE(ms, p.str());
  ms->evaluateProgram(p.str());
}
END_WORD
REGISTER_WORD(FixCameraLightingToObservedValues)



WORD(FixCameraLightingNoUpdate)
virtual void execute(MachineState * ms) {
  CONSOLE(ms, "fixCameraLighting...");

  int gain = 0;
  int exposure = 0;
  int wbRed;
  int wbGreen;
  int wbBlue;
  GET_INT_ARG(ms, wbBlue);
  GET_INT_ARG(ms, wbGreen);
  GET_INT_ARG(ms, wbRed);
  GET_INT_ARG(ms, gain);
  GET_INT_ARG(ms, exposure);

  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  camera->cameraGain = gain;
  camera->cameraExposure = exposure;
  camera->cameraWhiteBalanceRed = wbRed;
  camera->cameraWhiteBalanceGreen = wbGreen;
  camera->cameraWhiteBalanceBlue = wbBlue;

  ms->pushWord("moveCropToProperValueNoUpdate");

}
END_WORD
REGISTER_WORD(FixCameraLightingNoUpdate)



WORD(FixCameraLighting)

virtual string description() {
  return "Fix the camera lighting.  Usage:  <exposure> <gain> <red> <green> <blue> fixCameraLighting.  You can see the current values with cameraGain, cameraExposure, cameraWhiteBalanceRed, cameraWhiteBlanaceGreen, cameraWhiteBalanceBlue.";
}
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("subscribeCameraParameterTrackerToRosOut 0.5 waitForSeconds fixCameraLightingNoUpdate 0.5 waitForSeconds unsubscribeCameraParameterTrackerToRosOut");

}
END_WORD
REGISTER_WORD(FixCameraLighting)


WORD(SubscribeCameraParameterTrackerToRosOut)
virtual string description() {
  return "Subscribe to rosout.  We don't want to do this for long periods since it's very noisy, but this is how we see the current camera parameters, since they aren't published any other way.";
}
virtual void execute(MachineState * ms) {
  // there are lots of messages sent, and we're interested in the
  // camera tracker messages.  So we need a big message buffer to make
  // sure we didn't drop any.
  ros::NodeHandle n("~");
  ms->config.rosout_sub = n.subscribe("/rosout", 100, &MachineState::rosoutCallback, ms);
}
END_WORD
REGISTER_WORD(SubscribeCameraParameterTrackerToRosOut)


WORD(UnsubscribeCameraParameterTrackerToRosOut)
virtual string description() {
  return "Unsubscribe from rosout.  We don't want to stay plugged into rosut for very long since it's so noisy.";
}
virtual void execute(MachineState * ms) {
  //ms->config.rosout_sub.shutdown();
}
END_WORD
REGISTER_WORD(UnsubscribeCameraParameterTrackerToRosOut)



WORD(UnFixCameraLighting)
virtual string description() {
  return "Let the camera parmeters automatically update (this is the Baxter default).";
}
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("subscribeCameraParameterTrackerToRosOut 0.5 waitForSeconds unFixCameraLightingNoUpdate 0.5 waitForSeconds unsubscribeCameraParameterTrackerToRosOut");

}
END_WORD
REGISTER_WORD(UnFixCameraLighting)

CONFIG_GETTER_DOUBLE(CameraGetIdxMagX, ms->config.cameras[ms->config.focused_camera]->m_x_h[ms->config.currentThompsonHeightIdx], "") 
CONFIG_SETTER_DOUBLE(CameraSetIdxMagX, ms->config.cameras[ms->config.focused_camera]->m_x_h[ms->config.currentThompsonHeightIdx]) 

CONFIG_GETTER_DOUBLE(CameraGetIdxMagY, ms->config.cameras[ms->config.focused_camera]->m_y_h[ms->config.currentThompsonHeightIdx], "") 
CONFIG_SETTER_DOUBLE(CameraSetIdxMagY, ms->config.cameras[ms->config.focused_camera]->m_y_h[ms->config.currentThompsonHeightIdx]) 


CONFIG_GETTER_DOUBLE(CameraGetVpX, ms->config.cameras[ms->config.focused_camera]->vanishingPointReticle.px, "Vanishing point of camera.") 
CONFIG_SETTER_DOUBLE(CameraSetVpX, ms->config.cameras[ms->config.focused_camera]->vanishingPointReticle.px) 

CONFIG_GETTER_DOUBLE(CameraGetVpY, ms->config.cameras[ms->config.focused_camera]->vanishingPointReticle.py, "Vanishing point of camera.") 
CONFIG_SETTER_DOUBLE(CameraSetVpY, ms->config.cameras[ms->config.focused_camera]->vanishingPointReticle.py) 


CONFIG_GETTER_DOUBLE(CameraGetCurrentHeightReticleX, ms->config.cameras[ms->config.focused_camera]->heightReticles[ms->config.currentThompsonHeightIdx].px, "Height reticle x") 
CONFIG_SETTER_DOUBLE(CameraSetCurrentHeightReticleX, ms->config.cameras[ms->config.focused_camera]->heightReticles[ms->config.currentThompsonHeightIdx].px) 

CONFIG_GETTER_DOUBLE(CameraGetCurrentHeightReticleY, ms->config.cameras[ms->config.focused_camera]->heightReticles[ms->config.currentThompsonHeightIdx].py, "Height reticle y") 
CONFIG_SETTER_DOUBLE(CameraSetCurrentHeightReticleY, ms->config.cameras[ms->config.focused_camera]->heightReticles[ms->config.currentThompsonHeightIdx].py) 


WORD(SetGripperMaskOnes)
virtual void execute(MachineState * ms) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  

  camera->gripperMask.create(camera->cam_img.size(), CV_8U);
  camera->cumulativeGripperMask.create(camera->cam_img.size(), CV_8U);

  Size sz = camera->gripperMask.size();
  int imW = sz.width;
  int imH = sz.height;


  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      camera->gripperMask.at<uchar>(y,x) = 1;
      camera->cumulativeGripperMask.at<uchar>(y,x) = 1;
    }
  }
}
END_WORD
REGISTER_WORD(SetGripperMaskOnes)

WORD(SetGripperMask)
virtual void execute(MachineState * ms) {
  cout << "Program paused; please present the first contrast medium." << endl;
  ms->pushWord("setGripperMaskA"); 
  ms->pushWord("comeToStop");
  ms->pushWord("pauseStackExecution"); 
}
END_WORD
REGISTER_WORD(SetGripperMask)


WORD(InitCumulativeGripperMask)
virtual void execute(MachineState * ms) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  camera->cumulativeGripperMask.create(ms->config.accumulatedImage.size(), CV_8U);
  Size sz = camera->cumulativeGripperMask.size();
  int imW = sz.width;
  int imH = sz.height;
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      camera->cumulativeGripperMask.at<uchar>(y,x) = 0;
    }
  }
}
END_WORD
REGISTER_WORD(InitCumulativeGripperMask)

WORD(AccumulateImage)
virtual string description() {
  return "Add the current image from the camera to the accumulated image buffer.";
}
virtual void execute(MachineState * ms) {
  accumulateImage(ms);
}
END_WORD
REGISTER_WORD(AccumulateImage)

WORD(ResetAccumulatedImageAndMass)
virtual string description() {
  return "Initialize the accumulated buffer for averaging from the camera in image space.";
}
virtual void execute(MachineState * ms) {
  resetAccumulatedImageAndMass(ms);
}
END_WORD
REGISTER_WORD(ResetAccumulatedImageAndMass)

WORD(SetGripperMaskFromAccumulatedImage)
virtual string description() {
  return "Set the gripper mask by thresholding based on the variance in the accumulated image.";
}
virtual void execute(MachineState * ms) {




  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  Size sz = camera->gripperMask.size();
  int imW = sz.width;
  int imH = sz.height;


  camera->gripperMaskFirstContrast = ms->config.accumulatedImage.clone();
  camera->gripperMaskSecondContrast = camera->gripperMaskFirstContrast.clone();
  camera->gripperMaskMean = camera->gripperMaskFirstContrast.clone();
  camera->gripperMaskMean = 0.0;
  camera->gripperMaskSquares = camera->gripperMaskFirstContrast.clone();
  camera->gripperMaskSquares = 0.0;
  camera->gripperMaskCounts = 0;

  camera->gripperMask.create(camera->gripperMaskFirstContrast.size(), CV_8U);



  int dilationPixels = 20;
  double baseThresh = camera->gripperMaskThresh;
  double multiThresh = 2*baseThresh*baseThresh; // for ycbcr

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      double denom = ms->config.accumulatedImageMass.at<double>(y,x);
      if (denom <= 1.0) {
	denom = 1.0;
      }

      camera->gripperMaskSecondContrast.at<Vec3d>(y,x)[0] = (ms->config.accumulatedImage.at<Vec3d>(y,x)[0] / denom);
      camera->gripperMaskSecondContrast.at<Vec3d>(y,x)[1] = (ms->config.accumulatedImage.at<Vec3d>(y,x)[1] / denom);
      camera->gripperMaskSecondContrast.at<Vec3d>(y,x)[2] = (ms->config.accumulatedImage.at<Vec3d>(y,x)[2] / denom);

      camera->gripperMaskMean.at<Vec3d>(y,x)[0] += (ms->config.accumulatedImage.at<Vec3d>(y,x)[0] / denom);
      camera->gripperMaskMean.at<Vec3d>(y,x)[1] += (ms->config.accumulatedImage.at<Vec3d>(y,x)[1] / denom);
      camera->gripperMaskMean.at<Vec3d>(y,x)[2] += (ms->config.accumulatedImage.at<Vec3d>(y,x)[2] / denom);

      camera->gripperMaskSquares.at<Vec3d>(y,x)[0] += pow((ms->config.accumulatedImage.at<Vec3d>(y,x)[0] / denom), 2);
      camera->gripperMaskSquares.at<Vec3d>(y,x)[1] += pow((ms->config.accumulatedImage.at<Vec3d>(y,x)[1] / denom), 2);
      camera->gripperMaskSquares.at<Vec3d>(y,x)[2] += pow((ms->config.accumulatedImage.at<Vec3d>(y,x)[2] / denom), 2);
    }
  }
  //ms->config.meanViewerWindow->updateImage(camera->gripperMaskMean);
  camera->gripperMaskMean = camera->gripperMaskMean / 255.0;

  ms->config.meanViewerWindow->updateImage(camera->gripperMaskMean);
  camera->gripperMaskCounts += 1;
  Mat firstFloat; Mat firstYCBCR;  camera->gripperMaskFirstContrast.convertTo(firstFloat, CV_32FC3); cvtColor(firstFloat, firstYCBCR, CV_BGR2YCrCb);
  Mat secondFloat; Mat secondYCBCR;  camera->gripperMaskSecondContrast.convertTo(secondFloat, CV_32FC3); cvtColor(secondFloat, secondYCBCR, CV_BGR2YCrCb);

  Mat varianceImage = camera->gripperMaskFirstContrast.clone();

  Mat differenceImage = camera->gripperMaskFirstContrast.clone();

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {

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

      varianceImage.at<Vec3d>(y,x)[0] = camera->gripperMaskSquares.at<Vec3d>(y, x)[0] / camera->gripperMaskCounts - pow(camera->gripperMaskMean.at<Vec3d>(y, x)[0] / camera->gripperMaskCounts, 2) ;
      varianceImage.at<Vec3d>(y,x)[1] = camera->gripperMaskSquares.at<Vec3d>(y, x)[1] / camera->gripperMaskCounts - pow(camera->gripperMaskMean.at<Vec3d>(y, x)[1] / camera->gripperMaskCounts, 2) ;
      varianceImage.at<Vec3d>(y,x)[2] = camera->gripperMaskSquares.at<Vec3d>(y, x)[2] / camera->gripperMaskCounts - pow(camera->gripperMaskMean.at<Vec3d>(y, x)[2] / camera->gripperMaskCounts, 2) ;

      varianceImage.at<Vec3d>(y,x)[0] = varianceImage.at<Vec3d>(y,x)[0] / pow(255.0, 2);
      varianceImage.at<Vec3d>(y,x)[1] = varianceImage.at<Vec3d>(y,x)[1] / pow(255.0, 2);
      varianceImage.at<Vec3d>(y,x)[2] = varianceImage.at<Vec3d>(y,x)[2] / pow(255.0, 2);

      double maskDiffVariance = sqrt( pow(varianceImage.at<Vec3d>(y,x)[1],2) + pow(varianceImage.at<Vec3d>(y,x)[2],2) );
      if (maskDiffVariance > multiThresh) {
	camera->gripperMask.at<uchar>(y,x) = 1;
      } else {
	camera->gripperMask.at<uchar>(y,x) = 0;
      }
    }
  }


  Mat tmpMask = camera->gripperMask.clone();

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      if (tmpMask.at<uchar>(y,x) == 0) {
	int xmin = max(0, x - dilationPixels);
	int xmax = min(imW-1, x + dilationPixels);
	int ymin = max(0, y - dilationPixels);
	int ymax = min(imH-1, y + dilationPixels);
	for (int xp = xmin; xp < xmax; xp++) {
	  for (int yp = ymin; yp < ymax; yp++) {
	    camera->gripperMask.at<uchar>(yp,xp) = 0;
	  }
	}
      }
    }
  }
}
END_WORD
REGISTER_WORD(SetGripperMaskFromAccumulatedImage)



WORD(SetGripperMaskWithMotion)
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("resetAccumulatedImageAndMass  ( waitUntilImageCallbackReceived accumulateImage setGripperMaskFromAccumulatedImage ( oXUp ) 10 replicateWord eighthTurn waitUntilAtCurrentPosition comeToStop  ) 10 replicateWord");
}
END_WORD
REGISTER_WORD(SetGripperMaskWithMotion)


WORD(LoadGripperMask)
virtual void execute(MachineState * ms) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  camera->loadGripperMask();
}
END_WORD
REGISTER_WORD(LoadGripperMask)

WORD(SaveGripperMask)
virtual void execute(MachineState * ms) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  camera->saveGripperMask();
}
END_WORD
REGISTER_WORD(SaveGripperMask)



WORD(AssumeCalibrationPose)
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose = ms->config.calibrationPose;
}
END_WORD
REGISTER_WORD(AssumeCalibrationPose)

WORD(InitializeConfig)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  camera->loadCalibration();
  ms->pushWord("moveCropToProperValue"); 
}
END_WORD
REGISTER_WORD(LoadCalibration)

WORD(LoadCalibrationRaw)
virtual void execute(MachineState * ms) {
  string fileName;
  GET_ARG(ms, StringWord, fileName);
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  camera->loadCalibration(fileName);
  ms->pushWord("moveCropToProperValue"); 
}
END_WORD
REGISTER_WORD(LoadCalibrationRaw)

WORD(LoadDefaultCalibration)
virtual void execute(MachineState * ms) {
  string fileName = ms->config.data_directory + "/config/defaultCamera/cameraCalibration.yml";
  cout << "Loading calibration file from " << fileName << endl;
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  camera->loadCalibration(fileName);
}
END_WORD
REGISTER_WORD(LoadDefaultCalibration)


WORD(SaveCalibration)
virtual void execute(MachineState * ms) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  camera->saveCalibration();
}
END_WORD
REGISTER_WORD(SaveCalibration)

WORD(SaveCalibrationToClass)
virtual void execute(MachineState * ms) {
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
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  camera->saveCalibration(fileName);
}
END_WORD
REGISTER_WORD(SaveCalibrationToClass)

WORD(SetColorReticles)
virtual void execute(MachineState * ms) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  ms->config.bDelta = camera->cReticleIndexDelta;
  ms->config.currentEEPose.pz = camera->firstCReticleIndexDepth;

  // leave it in a canonical state
  ms->pushWord("setGridSizeCoarse");

  int * ii = &(ms->config.scrI);
  (*ii) = 0;


  for (int i = 0; i < camera->numCReticleIndexes; i++) {
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
virtual void execute(MachineState * ms) {
  int lightX = 0;
  int lightY = 0;
  findLight(ms, &lightX, &lightY);

  ms->config.pilotTarget.px = lightX;
  ms->config.pilotTarget.py = lightY;
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  int * ii = &(ms->config.scrI);
  camera->xCR[(*ii)] = lightX;
  camera->yCR[(*ii)] = lightY;

  (*ii)++;
}
END_WORD
REGISTER_WORD(SetColorReticlesA)

WORD(ScanObjectFast)

virtual string description() {
  return "Scans an object without an IR scan, and with an annotated grasp.";
}

virtual void execute(MachineState * ms) {

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
virtual void execute(MachineState * ms) {

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
virtual void execute(MachineState * ms) {

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
virtual void execute(MachineState * ms) {

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
virtual void execute(MachineState * ms) {

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
virtual void execute(MachineState * ms) {

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
virtual void execute(MachineState * ms) {

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

WORD(ScanObjectScene)
virtual string description() {
  return "Scans a stack of objects in stream mode with an annotated 3d grasps in stack.";
}
virtual void execute(MachineState * ms) {

  cout << "scanObjectScene: start" << endl;

  ms->pushWord("waitUntilAtCurrentPosition"); 
  ms->pushWord("assumeCrane1");
  ms->pushWord("openGripper");

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

  // XXX take out for now, conserve space
  //ms->pushWord("streamScanCentered");

  ms->pushWord("activateSensorStreaming"); 
  ms->pushWord("clearStreamBuffers"); 
  ms->pushWord("shutdownToSensorsAndMovement"); 
  ms->pushWord("setSisFlags"); 

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
  ms->pushWord("moveToRegister1");


  ms->pushWord("bringUpAllNonessentialSystems"); 
  ms->pushWord("deactivateSensorStreaming"); 
  // XXX take out for now, conserve space
  /*
  {
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
  */
  {
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
  // slow speed here results in lost frames if robot chooses to unwind
  ms->pushWord("quarterImpulse");

  ms->pushWord("shutdownToSensorsAndMovement"); 
  ms->pushWord("setSisFlags"); 

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
  ms->pushWord("lock3dGraspBase"); 

  // unneeded for 3d grasps
  // ms->pushWord("preAnnotateOffsetGrasp"); // you need to saveRegister1 where the grasp should be before calling this
  ms->pushWord("setPhotoPinHere");

  //ms->pushWord("pauseStackExecution"); // pause to ensure being centered

  // don't make a new class, scene does it outside 

  ms->pushWord("tenthImpulse");
  ms->pushWord("setGridSizeCoarse");
  ms->pushWord("setBoundingBoxModeToMapping"); 

  ms->pushWord("setIdleModeToEmpty"); 
  ms->pushWord("setPlaceModeToShake"); 
  ms->pushWord("clearBlueBoxMemories"); 
  ms->pushWord("clearMapForPatrol"); 
  ms->pushWord("enableDiskStreaming"); 
  ms->pushWord("zeroGOff"); 
}
END_WORD
REGISTER_WORD(ScanObjectScene)

WORD(CollectMoreStreams)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  // uses ms->config.currentEEPose instead of ms->config.trueEEPose so that we can set it below the table
  double flushZ = -(ms->config.currentTableZ) + ms->config.pickFlushFactor;
  ms->config.currentGraspZ = -(ms->config.currentEEPose.pz - (-ms->config.currentTableZ));
  cout << "recordGraspZ flushZ currentGraspZ: " << flushZ << " " << ms->config.currentGraspZ << " " << endl;
}
END_WORD
REGISTER_WORD(RecordGraspZ)

WORD(Start3dGraspAnnotation)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual string description() {
  return "Add the current pose as a 3D grasp relative to the focused object.  Must have locked the 3d grasp first so it knows what pose to use relative to the object.";
}
virtual void execute(MachineState * ms) {
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
    Grasp toPush;
    toPush.grasp_pose = thisRelative3dGrasp;
    toPush.tries = 1;
    toPush.successes = 1;
    toPush.failures = 0;
    toPush.jams = 0;
    ms->config.class3dGrasps[ms->config.targetClass].push_back(toPush);
    cout << " added " << toPush << endl;
  } else {
    cout << " cannot add 3d grasp, targetClass, tnc: " << ms->config.targetClass << " " << tnc << endl;
  }
}
END_WORD
REGISTER_WORD(Add3dGrasp)

WORD(AssumeZOfPoseWord)
virtual void execute(MachineState * ms) {
  eePose thisAbsolute3dGrasp;
  GET_ARG(ms, EePoseWord, thisAbsolute3dGrasp);

  cout << "assumeZOfPoseWord: " << thisAbsolute3dGrasp << endl;
  
  ms->config.currentEEPose.pz = thisAbsolute3dGrasp.pz;
}
END_WORD
REGISTER_WORD(AssumeZOfPoseWord)

WORD(Add3dGraspPoseWord)
virtual string description() {
  return "Takes an argument on the stack of an EEPose and adds that pose as a 3d grasp relative to the base object.";
}
virtual void execute(MachineState * ms) {
  eePose thisAbsolute3dGrasp ;
  GET_ARG(ms, EePoseWord, thisAbsolute3dGrasp);

  cout << "Adding 3d grasp from below, global: " << thisAbsolute3dGrasp << endl;

  eePose thisRelative3dGrasp = thisAbsolute3dGrasp.getPoseRelativeTo(ms->config.c3dPoseBase);

  int tnc = ms->config.classLabels.size();
  if ( (ms->config.targetClass > -1) && (ms->config.targetClass < tnc) ) {
    Grasp toPush;
    toPush.grasp_pose = thisRelative3dGrasp;
    toPush.tries = 1;
    toPush.successes = 1;
    toPush.failures = 0;
    toPush.jams = 0;
    ms->config.class3dGrasps[ms->config.targetClass].push_back(toPush);
    cout << " added relative grasp: " << toPush << endl;
  } else {
    cout << " cannot add 3d grasp, targetClass, tnc: " << ms->config.targetClass << " " << tnc << endl;
  }
}
END_WORD
REGISTER_WORD(Add3dGraspPoseWord)

WORD(AddPlaceUnderPoint)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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

CONFIG_GETTER_DOUBLE(GraspBackoffDistance, ms->config.graspBackoffDistance, "") 
CONFIG_SETTER_DOUBLE(SetGraspBackoffDistance, ms->config.graspBackoffDistance) 


WORD(AssumeCurrent3dGrasp)
virtual void execute(MachineState * ms) {
  double p_backoffDistance = 0.10;
  int t3dGraspIndex = ms->config.current3dGraspIndex;
  cout << "assumCurrent3dGrasp, t3dGraspIndex: " << t3dGraspIndex << endl;

  eePose toApply = ms->config.class3dGrasps[ms->config.targetClass][t3dGraspIndex].grasp_pose;  

  eePose thisBase = ms->config.lastLockedPose;
  thisBase.pz = -ms->config.currentTableZ;
  eePose graspPose = toApply.applyAsRelativePoseTo(thisBase);

  int increments = floor(p_backoffDistance / GRID_COARSE); 
  Vector3d localUnitX;
  Vector3d localUnitY;
  Vector3d localUnitZ;
  fillLocalUnitBasis(graspPose, &localUnitX, &localUnitY, &localUnitZ);
  eePose retractedGraspPose = eePoseMinus(graspPose, p_backoffDistance * localUnitZ);

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


WORD(CollectMoreCrops)
virtual void execute(MachineState * ms) {
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

WORD(HistogramDetectionIfBlueBoxes)
virtual void execute(MachineState * ms)
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
virtual void execute(MachineState * ms)
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms)       {
  recordBlueBoxInHistogram(ms, ms->config.pilotClosestBlueBoxNumber);
  ms->pushWord("histogramDetectionReport");
  ms->pushWord("histogramDetectionNormalize");
}
END_WORD
REGISTER_WORD(HistogramExampleAsFocusedClass)

WORD(HistgramAllExamplesFocusedClass)
virtual void execute(MachineState * ms)       {
  for (int c = 0; c < ms->config.bTops.size(); c++) {
    recordBlueBoxInHistogram(ms, c);
  }
  ms->pushWord("histogramDetectionReport");
  ms->pushWord("histogramDetectionNormalize");
}
END_WORD
REGISTER_WORD(HistgramAllExamplesFocusedClass)

WORD(HistogramDetectionInit)
virtual void execute(MachineState * ms)
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
virtual void execute(MachineState * ms)
{
  computeClassificationDistributionFromHistogram(ms);
}
END_WORD
REGISTER_WORD(HistogramDetectionNormalize)

WORD(HistogramDetectionReport)
virtual void execute(MachineState * ms)
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
virtual void execute(MachineState * ms)
{
  if (isFocusedClassValid(ms) && ms->config.bTops.size() > 0) {
    cout << "replaceBlueBoxesWithFocusedClass: Focused class is valid, replacing bTops etc." << endl;
    ms->config.bTops.resize(1);
    ms->config.bBots.resize(1);
    ms->config.bCens.resize(1);
    ms->config.bLabels.resize(1);

    Camera * camera  = ms->config.cameras[ms->config.focused_camera];

    ms->config.bTops[0].x = camera->vanishingPointReticle.px - ms->config.fakeBBWidth;
    ms->config.bTops[0].y = camera->vanishingPointReticle.py - ms->config.fakeBBWidth;
    ms->config.bBots[0].x = camera->vanishingPointReticle.px + ms->config.fakeBBWidth;
    ms->config.bBots[0].y = camera->vanishingPointReticle.py + ms->config.fakeBBWidth;

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
virtual void execute(MachineState * ms)
{
  if (ms->config.chWinner > -1) {
    cout << "Replacing blue boxes with histogram winner..." << endl;
    ms->config.bTops.resize(1);
    ms->config.bBots.resize(1);
    ms->config.bCens.resize(1);
    ms->config.bLabels.resize(1);

    Camera * camera  = ms->config.cameras[ms->config.focused_camera];

    ms->config.bTops[0].x = camera->vanishingPointReticle.px - ms->config.fakeBBWidth;
    ms->config.bTops[0].y = camera->vanishingPointReticle.py - ms->config.fakeBBWidth;
    ms->config.bBots[0].x = camera->vanishingPointReticle.px + ms->config.fakeBBWidth;
    ms->config.bBots[0].y = camera->vanishingPointReticle.py + ms->config.fakeBBWidth;

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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  cout << "retrain_vocab turned on,  was: " << ms->config.retrain_vocab << ", is: ";
  ms->config.retrain_vocab = 1;
  cout << ms->config.retrain_vocab << endl;
}
END_WORD
REGISTER_WORD(RetrainVocabOn)

WORD(RetrainVocabOff)
virtual void execute(MachineState * ms) {
  cout << "retrain_vocab turned off,  was: " << ms->config.retrain_vocab << ", is: ";
  ms->config.retrain_vocab = 0;
  cout << ms->config.retrain_vocab << endl;
}
END_WORD
REGISTER_WORD(RetrainVocabOff)

WORD(ResetCurrentFocusedClass)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  ms->config.currentScanMode = CENTERED;
  cout << "Setting currentScanMode to CENTERED: " << ms->config.currentScanMode << endl;
}
END_WORD
REGISTER_WORD(SetScanModeCentered)

WORD(SetScanModeNotCentered)
virtual void execute(MachineState * ms) {
  ms->config.currentScanMode = NOT_CENTERED;
  cout << "Setting currentScanMode to NOT_CENTERED: " << ms->config.currentScanMode << endl;
}
END_WORD
REGISTER_WORD(SetScanModeNotCentered)




WORD(BuildClassSimilarityMatrix)
virtual string description() {
  return "Builds the matrix of gradients of the current class labels.";
}
virtual void execute(MachineState * ms) {
  int nc = ms->config.numClasses;
  double * result =  new double[nc * nc];

  for (int i = 0; i < nc; i++) {
    for (int j = 0; j < nc; j++) {
      result[i + nc * j] = computeSimilarity(ms, i, j);
    }
  }

  for (int j = 0; j < nc; j++) {
    cout << std::setw(3) << j << ": " << ms->config.classLabels[j] << endl;
  }
  cout << "   " ;
  for (int j = 0; j < nc; j++) {
    cout << std::setw(10) << j ;
  }
  cout << endl;

  for (int i = 0; i < nc; i++) {
    cout << std::setw(3) << i;
    for (int j = 0; j < nc; j++) {
      cout << std::setw(10) << result[i + nc * j];
    }
    cout << endl;
  }
  delete result;
}
END_WORD
REGISTER_WORD(BuildClassSimilarityMatrix)

WORD(BuildClassSimilarityMatrixFromDensity)
virtual string description() {
  return "Builds the matrix of gradients of the current class labels.";
}
virtual void execute(MachineState * ms) {
  int nc = ms->config.numClasses;
  double * result =  new double[nc];

  Mat frameFromDensity = makeGCrop(ms, 0, 0);

  for (int i = 0; i < nc; i++) {
    result[i] = computeSimilarity(ms, frameFromDensity, ms->config.classHeight1AerialGradients[i]);
  }

  for (int j = 0; j < nc; j++) {
    cout << std::setw(3) << j << ": " << ms->config.classLabels[j] << endl;
  }

  cout << endl;

  double max_of_classes = 0;

  for (int i = 0; i < nc; i++) {
    cout << std::setw(3) << i;
    cout << std::setw(10) << result[i];
    cout << endl;
    max_of_classes = max(max_of_classes, result[i]);
  }

  ms->pushWord(make_shared<DoubleWord>(max_of_classes));

  delete result;
}
END_WORD
REGISTER_WORD(BuildClassSimilarityMatrixFromDensity)

WORD(IrFixPick)
virtual void execute(MachineState * ms) {
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

WORD(ClearClass3dGrasps)
virtual void execute(MachineState * ms) {
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
