#include "ein_words.h"
#include "ein.h"

CONFIG_SETTER_ENUM(FaceAnimationSetMode, ms->config.currentAnimationMode, (animationMode))
CONFIG_GETTER_INT(FaceAnimationGetMode, ms->config.currentAnimationMode)

WORD(FindEmotions)
virtual void execute(std::shared_ptr<MachineState> ms) {
	// fills out the emotionNames
  vector<String> emotionNames = ms->config.emotionNames;
  


  DIR *dir; 
  String animationDirName = ms->config.data_directory + "/animation/"; 
  dir = opendir(animationDirName.c_str()); 
  struct dirent *pDirent; 
  while ((pDirent = readdir(dir)) != NULL) {
	  cout << pDirent->d_name << endl; 
  }
  closedir(dir); 

}
END_WORD
REGISTER_WORD(FindEmotions)

WORD(FaceAnimationLoadEmotion)
virtual void execute(std::shared_ptr<MachineState> ms) {
	
}
END_WORD
REGISTER_WORD(FaceAnimationLoadEmotion)

WORD(FaceAnimationSetEmotionValue)
virtual void execute(std::shared_ptr<MachineState> ms) {
}
END_WORD
REGISTER_WORD(FaceAnimationSetEmotionValue)

