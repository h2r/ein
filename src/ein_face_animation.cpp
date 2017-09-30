#include "ein_words.h"
#include "ein_baxter_config.h"
#include "ein.h"
#include "qtgui/einwindow.h"
#include "word.h"

#include <dirent.h>
#include <highgui.h>


namespace ein_words {
CONFIG_SETTER_ENUM(FaceAnimationSetMode, ms->config.currentAnimationMode, (animationMode))
CONFIG_GETTER_INT(FaceAnimationGetMode, ms->config.currentAnimationMode)

CONFIG_SETTER_DOUBLE(FaceAnimationSetRate, ms->config.animationRate); 
CONFIG_GETTER_DOUBLE(FaceAnimationGetRate, ms->config.animationRate); 


WORD(FaceAnimationFindEmotions)
virtual void execute(MachineState * ms) {
	map<string,int> emotionInd; 
	DIR *dir; 
	String animationDirName = ms->config.data_directory + "/animation/"; 
	dir = opendir(animationDirName.c_str()); 
	struct dirent *pDirent; 
	int idx = 0; 
	while ((pDirent = readdir(dir)) != NULL) {
		String name = pDirent->d_name; 
		if (name[0] != '.') {
			emotionInd[name] = idx++; 
		}
	}

	closedir(dir); 

	ms->config.emotionIndex = emotionInd; 
	ms->config.emotionImages = vector<vector <Mat> >(emotionInd.size()); 

	cout << "found the following animation folders: " << endl; 

	for (auto const& x: ms->config.emotionIndex) {
		cout << x.second << ": " << x.first << endl; 
	}

}
END_WORD
REGISTER_WORD(FaceAnimationFindEmotions)

WORD(FaceAnimationLoadAllEmotions) 
virtual void execute(MachineState * ms) {
	for (auto const& x: ms->config.emotionIndex) { 
		std::stringstream program;
		program <<  "\"" << x.first << "\" faceAnimationLoadEmotion";
		ms->evaluateProgram(program.str());  
	}
}
END_WORD
REGISTER_WORD(FaceAnimationLoadAllEmotions)


WORD(FaceAnimationLoadEmotion)
virtual void execute(MachineState * ms) {
    string emotionName;
    GET_STRING_ARG(ms, emotionName);

	if (ms->config.emotionIndex.find(emotionName) == ms->config.emotionIndex.end()) {
		cout << "Emotion " << emotionName << " doesn't exist." << endl; 
		return; 
	}

	int index = ms->config.emotionIndex[emotionName]; 

	vector<string> paths; 
	String emotionDirName = ms->config.data_directory + "/animation/" + emotionName + "/"; 
	DIR *dir; 
	dir = opendir(emotionDirName.c_str()); 
	struct dirent *pDirent; 
	while ((pDirent = readdir(dir)) != NULL) {
		String name = pDirent->d_name; 
		if (name[0] != '.') {
			paths.push_back(name); 
		}

	}
	closedir(dir); 

	vector<Mat> cachedImages(paths.size());
	for (int i = 0; i < paths.size(); i++) {
		string name = paths[i]; 
		int fileNumber = atoi(name.substr(0, name.find(".")).c_str()); 
		String fileName = ms->config.data_directory + "/animation/" + emotionName + "/"  + name; 
		cachedImages[fileNumber] = imread(fileName); 
	}	
	ms->config.emotionImages[index] = cachedImages; 

}
END_WORD
REGISTER_WORD(FaceAnimationLoadEmotion)

WORD(FaceAnimationSetEmotionValue)
virtual void execute(MachineState * ms) {
	// if animation is not enabled
	if (!ms->config.currentAnimationMode) {
		// fail out nicely
		cout << "animation is not enabled. Exiting." << endl; 
		return; 
	}

	double v1;
	GET_NUMERIC_ARG(ms, v1);
	int value = (int) v1;

	string emotionName; 
    GET_STRING_ARG(ms, emotionName);
	
	if (ms->config.emotionIndex.find(emotionName) == ms->config.emotionIndex.end()) { 
		cout << "Not a valid emotion name." << endl; 
		return; 
	}

	AnimationState newState = {emotionName, value}; 
	ms->config.targetAnimationState = newState; 


	ms->pushWord("changeAnimationState");  


}
END_WORD
REGISTER_WORD(FaceAnimationSetEmotionValue)

WORD(ChangeAnimationState) 
virtual void execute(MachineState * ms) {
	AnimationState target = ms->config.targetAnimationState; 
	AnimationState current = ms->config.currentAnimationState; 
	int value = current.value; 
	string emotion = current.emotion; 
	if (target.emotion == current.emotion) {
		// we're in the same emotion
		if (target.value == current.value) {
			return; 
		}
		if (target.value > current.value) {
			value++; 
		} else {
			value--; 
		}
	} else {
		// different emotion; head towards 0 and then switch
		if (current.value == 0) { 
			// if we're at 0, switch to the new emotion
			emotion = target.emotion; 
			value = 0; 
		} else {
			// otherwise, keep heading back towards neutral
			value--; 
		}
	}
	AnimationState nextState = {emotion, value}; 
	int index = ms->config.emotionIndex[emotion]; 
	Mat image = ms->config.emotionImages[index][value]; 

	if (isSketchyMat(image)) {
		cout << "ChangeAnimationState: cannot load " << emotion << " " << value << endl;
		return;
	} else {
		sensor_msgs::Image msg;
		msg.header.stamp = ros::Time::now();
		msg.width = image.cols;
		msg.height = image.rows;
		msg.step = image.cols * image.elemSize();
		msg.is_bigendian = false;
		msg.encoding = sensor_msgs::image_encodings::BGR8;
		msg.data.assign(image.data, image.data + size_t(image.rows * msg.step));
		ms->config.baxterConfig->face_screen_pub.publish(msg);
	}

	ms->config.currentAnimationState = nextState; 

	std::stringstream program;
	double seconds = 1/ms->config.animationRate; 
	program <<  seconds << " spinForSeconds changeAnimationState";
	ms->evaluateProgram(program.str());  


}
END_WORD
REGISTER_WORD(ChangeAnimationState)



WORD(PublishImageToFace)
virtual void execute(MachineState * ms) {
	string fileName; 
    GET_STRING_ARG(ms, fileName);
	Mat image = imread(fileName); 

	if (isSketchyMat(image)) {
		cout << "publishImageFileToFace: cannot load file " << fileName << endl;
		return;
	} else {
		sensor_msgs::Image msg;
		msg.header.stamp = ros::Time::now();
		msg.width = image.cols;
		msg.height = image.rows;
		msg.step = image.cols * image.elemSize();
		msg.is_bigendian = false;
		msg.encoding = sensor_msgs::image_encodings::BGR8;
		msg.data.assign(image.data, image.data + size_t(image.rows * msg.step));
		ms->config.baxterConfig->face_screen_pub.publish(msg);
	}
}
END_WORD
REGISTER_WORD(PublishImageToFace)


}
