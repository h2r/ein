#include "ein_words.h"
#include "ein.h"
#include "camera.h"
#include "gaussian_map_words.h"

GaussianMapWord::GaussianMapWord(shared_ptr<GaussianMap> imap)
{
  map = imap;
}


namespace ein_words {
  
WORD(SceneBackgroundMap)
virtual void execute(MachineState * ms) {
  shared_ptr<GaussianMapWord> mapWord = make_shared<GaussianMapWord>(ms->config.scene->background_map);
  ms->pushData(mapWord);
}
END_WORD
REGISTER_WORD(SceneBackgroundMap)

  
WORD(SceneSetBackgroundMap)
virtual void execute(MachineState * ms) {
  shared_ptr<GaussianMapWord> map;
  GET_WORD_ARG(ms, GaussianMapWord, map);
  ms->config.scene->background_map = map->map;
}
END_WORD
REGISTER_WORD(SceneSetBackgroundMap)

}
