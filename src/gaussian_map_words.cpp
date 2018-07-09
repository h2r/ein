#include "ein_words.h"
#include "config.h"
#include "camera.h"
#include "gaussian_map_words.h"
#include <sys/stat.h>


GaussianMapWord::GaussianMapWord(shared_ptr<GaussianMap> imap)
{
  map = imap;
}


SceneWord::SceneWord(shared_ptr<Scene> iscene)
{
  scene = iscene;
}


namespace ein_words {

WORD(CurrentScene)
virtual string description() {
  return "Pushes the current scene on the stack.";
}
virtual void execute(MachineState * ms) {
  shared_ptr<SceneWord> w = make_shared<SceneWord>(ms->config.scene);
  ms->pushData(w);
}
END_WORD
REGISTER_WORD(CurrentScene)

WORD(SceneSetCurrentScene)
virtual string description() {
  return "Sets the current scene with its argument.";
}
virtual void execute(MachineState * ms) {
  shared_ptr<SceneWord> sceneWord;
  GET_WORD_ARG(ms, SceneWord, sceneWord);
  ms->config.scene = sceneWord->scene;
}
END_WORD
REGISTER_WORD(SceneSetCurrentScene)


WORD(CurrentBackgroundMap)
virtual string description() {
  return "Pushes the current background map on the stack.";
}
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("currentScene sceneBackgroundMap");
}
END_WORD
REGISTER_WORD(CurrentBackgroundMap)


WORD(CurrentObservedMap)
virtual string description() {
  return "Pushes the current observed map on the stack.";
}
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("currentScene sceneObservedMap");
}
END_WORD
REGISTER_WORD(CurrentObservedMap)

WORD(CurrentPredictedMap)
virtual string description() {
  return "Pushes the current predicted map on the stack.";
}
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("currentScene scenePredictedMap");
}
END_WORD
REGISTER_WORD(CurrentPredictedMap)


WORD(SceneAnchorPose)
virtual string description() {
  return "Takes a gaussian map on the stack and returns its anchor eePose.";
}
virtual void execute(MachineState * ms) {
  shared_ptr<Word> word;
  GET_WORD_ARG(ms, Word, word);

  std::shared_ptr<GaussianMapWord> map = std::dynamic_pointer_cast<GaussianMapWord>(word);
  std::shared_ptr<SceneWord> scene = std::dynamic_pointer_cast<SceneWord>(word);
  if (map != NULL) {
    ms->pushWord(make_shared<EePoseWord>(map->map->anchor_pose));    
  } else if (scene != NULL) {
    ms->pushWord(make_shared<EePoseWord>(scene->scene->anchor_pose));    
  } else {
    CONSOLE_ERROR(ms, "Must pass a Gaussian Map word or a Scene word.");
    ms->pushWord("pauseStackExecution");   
  }
}
END_WORD
REGISTER_WORD(SceneAnchorPose)


WORD(SceneBackgroundMap)
virtual string description() {
  return "Takes a scene on the stack and returns its background map.";
}
virtual void execute(MachineState * ms) {
  shared_ptr<SceneWord> sceneWord;
  GET_WORD_ARG(ms, SceneWord, sceneWord);
  ms->pushData(make_shared<GaussianMapWord>(sceneWord->scene->background_map));
}
END_WORD
REGISTER_WORD(SceneBackgroundMap)

WORD(SceneObservedMap)
virtual string description() {
  return "Takes a scene on the stack and returns its observed map.";
}
virtual void execute(MachineState * ms) {
  shared_ptr<SceneWord> sceneWord;
  GET_WORD_ARG(ms, SceneWord, sceneWord);
  ms->pushData(make_shared<GaussianMapWord>(sceneWord->scene->observed_map));
}
END_WORD
REGISTER_WORD(SceneObservedMap)

WORD(ScenePredictedMap)
virtual string description() {
  return "Takes a scene on the stack and returns its predicted map.";
}
virtual void execute(MachineState * ms) {
  shared_ptr<SceneWord> sceneWord;
  GET_WORD_ARG(ms, SceneWord, sceneWord);
  ms->pushData(make_shared<GaussianMapWord>(sceneWord->scene->predicted_map));
}
END_WORD
REGISTER_WORD(ScenePredictedMap)
  
WORD(SceneSetBackgroundMap)
virtual string description() {
  return "Takes a scene and a background map and sets the scene with the background map. ";
}
virtual void execute(MachineState * ms) {
  shared_ptr<GaussianMapWord> map;
  GET_WORD_ARG(ms, GaussianMapWord, map);

  // XXX this seems unintuitive
  //ms->config.scene->background_map = map->map;

  shared_ptr<SceneWord> sceneWord;
  GET_WORD_ARG(ms, SceneWord, sceneWord);

  sceneWord->scene->background_map = map->map;

}
END_WORD
REGISTER_WORD(SceneSetBackgroundMap)

WORD(SceneSetObservedMap)
virtual string description() {
  return "Takes a scene and an observed map and sets the scene with the observed map. ";
}
virtual void execute(MachineState * ms) {
  shared_ptr<GaussianMapWord> map;
  GET_WORD_ARG(ms, GaussianMapWord, map);
  // XXX this seems unintuitive
  //ms->config.scene->observed_map = map->map;

  shared_ptr<SceneWord> sceneWord;
  GET_WORD_ARG(ms, SceneWord, sceneWord);

  sceneWord->scene->observed_map = map->map;

}
END_WORD
REGISTER_WORD(SceneSetObservedMap)

WORD(SceneCopyScene)
virtual string description() {
  return "Takes a scene on the stack and performs a deep copy, leaving a new scene on the stack.";
}
virtual void execute(MachineState * ms) {
  shared_ptr<SceneWord> word;
  GET_WORD_ARG(ms, SceneWord, word);

  shared_ptr<SceneWord> w = make_shared<SceneWord>(word->scene->copy());
  ms->pushData(w);

}
END_WORD
REGISTER_WORD(SceneCopyScene)


WORD(SceneCopyGaussianMap)
virtual string description() {
  return "Takes a gaussian map on the stack and performs a deep copy, leaving a new gaussian map on the stack.";
}
virtual void execute(MachineState * ms) {
  shared_ptr<GaussianMapWord> word;
  GET_WORD_ARG(ms, GaussianMapWord, word);

  shared_ptr<GaussianMapWord> w = make_shared<GaussianMapWord>(word->map->copy());
  ms->pushData(w);

}
END_WORD
REGISTER_WORD(SceneCopyGaussianMap)

WORD(SceneLoadGaussianMap)
virtual string description() {
  return "Takes a string, loads the gaussian map and puts it on the stack.";
}
virtual void execute(MachineState * ms) {

  string message;
  GET_STRING_ARG(ms, message);

  stringstream ss;
  ss << ms->config.data_directory + "/maps/" + message + ".yml";
  stringstream ss_dir;
  ss_dir << ms->config.data_directory + "/maps/";
  mkdir(ss_dir.str().c_str(), 0777);

  shared_ptr<GaussianMapWord> mapWord = make_shared<GaussianMapWord>(GaussianMap::createFromFile(ms, ss.str()));
  ms->pushData(mapWord);

}
END_WORD
REGISTER_WORD(SceneLoadGaussianMap)



WORD(SceneSaveGaussianMap)
virtual string description() {
  return "Takes a string and saves a gaussian map to that string.";
}
virtual void execute(MachineState * ms) {
  string message;
  GET_STRING_ARG(ms, message);

  shared_ptr<GaussianMapWord> map;
  GET_WORD_ARG(ms, GaussianMapWord, map);


  stringstream ss;
  ss << ms->config.data_directory + "/maps/" + message + ".yml";
  stringstream ss_dir;
  ss_dir << ms->config.data_directory + "/maps/";
  mkdir(ss_dir.str().c_str(), 0777);

  map->map->saveToFile(ss.str());

}
END_WORD
REGISTER_WORD(SceneSaveGaussianMap)


}
