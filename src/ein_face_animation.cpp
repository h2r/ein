#include "ein_words.h"
#include "ein.h"
#include "qtgui/einwindow.h"
#include "word.h"


namespace ein_words {
CONFIG_SETTER_ENUM(FaceAnimationSetMode, ms->config.currentAnimationMode, (animationMode))
CONFIG_GETTER_INT(FaceAnimationGetMode, ms->config.currentAnimationMode)



WORD(FindEmotions)
virtual void execute(std::shared_ptr<MachineState> ms) {
	// fills out the emotionNames

	vector<string> emotionNames; 

	DIR *dir; 
	String animationDirName = ms->config.data_directory + "/animation/"; 
	dir = opendir(animationDirName.c_str()); 
	struct dirent *pDirent; 
	while ((pDirent = readdir(dir)) != NULL) {
		String name = pDirent->d_name; 
		if (name[0] != '.') {
			emotionNames.push_back(name);
		}

	}

	closedir(dir); 

	ms->config.emotionNames = emotionNames; 

	cout << "found the following animation folders: " << endl; 
	for (int i = 0; i < ms->config.emotionNames.size(); i++) {
		cout <<  ms->config.emotionNames[i] << endl; 
	}

}
END_WORD
REGISTER_WORD(FindEmotions)

WORD(FaceAnimationLoadEmotion)
virtual void execute(std::shared_ptr<MachineState> ms) {
	
	
	if (ms->config.emotionNames.empty()) {
		ms->pushWord("findEmotions"); 
	}

    string emotionName;
    GET_STRING_ARG(ms, emotionName);

	int index = 0; 
	for (int i = 0; i < ms->config.emotionNames.size(); i++) {
		if (emotionName == ms->config.emotionNames[i]) {
			index = i; 
			break; 
		}
	}

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

	for (int i = 0; i < paths.size(); i++) {
		cout << paths[i] << endl; 
	}


}
END_WORD
REGISTER_WORD(FaceAnimationLoadEmotion)

WORD(FaceAnimationSetEmotionValue)
virtual void execute(std::shared_ptr<MachineState> ms) {
}
END_WORD
REGISTER_WORD(FaceAnimationSetEmotionValue)

}
