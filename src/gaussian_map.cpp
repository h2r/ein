#include "gaussian_map.h"
#include "ein_words.h"

void GaussianMap::reallocate() {
  if (cells == NULL) {
    cells = new GaussianMapCell[width*height];
  } else {
    delete cells;
    cells = new GaussianMapCell[width*height];
  }
}

GaussianMap::GaussianMap(int w, int h, double cw, eePose sp) {
  width = w;
  height = h;
  cell_width = cw;
  scene_pose = sp;
  cells = NULL;
  reallocate();
}

Scene::Scene(shared_ptr<MachineState> _ms) {
  ms = _ms;
}

namespace ein_words {

WORD(ClearTransitionTable)
virtual void execute(std::shared_ptr<MachineState> ms) {
// zero it out
}
END_WORD
REGISTER_WORD(ClearTransitionTable)

WORD(CountTransitionInTable)
virtual void execute(std::shared_ptr<MachineState> ms) {
// pass strings for classes and action, increment
}
END_WORD
REGISTER_WORD(CountTransitionInTable)

WORD(UpdateTransitionTable)
virtual void execute(std::shared_ptr<MachineState> ms) {
// make sure it is consistent with the number of classes
}
END_WORD
REGISTER_WORD(UpdateTransitionTable)

WORD(PlanWithTransitionTable)
virtual void execute(std::shared_ptr<MachineState> ms) {
// make sure it is consistent with the number of classes
}
END_WORD
REGISTER_WORD(PlanWithTransitionTable)

}



// XXX word to update observedMap based on current camera image
// XXX words to save and load current background and current object maps


// XXX when training from crops, render from the stream buffer into the observedMap then crop out


