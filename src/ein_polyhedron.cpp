
#include "ein_words.h"
#include "ein.h"

namespace ein_words {

/*
// in the intrinsic frame of the duck
typedef enum {
  TOP = 0,
  BOTTOM = 1,
  LEFT = 2,
  RIGHT = 3,
  FRONT = 4,
  BOTTOM = 5
} cubeFaces;
*/



WORD(SetClosestBlueBoxMemoryToFocusedClass)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  eePose targetPose;
  GET_ARG(ms, EePoseWord, targetPose);
  findClosestBlueBoxMemory(ms, targetPose);
}
END_WORD
REGISTER_WORD(SetClosestBlueBoxMemoryToFocusedClass)

WORD(PlayLoopL)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->pushWord("playLoopL");
  ms->pushWord("playLoop");
  ms->pushWord("clearMapForPatrol");
  ms->pushWord("clearBlueBoxMemories");
}
END_WORD
REGISTER_WORD(PlayLoopL)

WORD(PlayLoop)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->pushWord("quarterImpulse");

  ms->pushWord("1");
  ms->pushWord("changeToHeight");
  ms->pushWord("assumeBeeHome");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1");

    ms->pushWord("playLoopP");
  ms->pushWord("ifGrasp");

  ms->pushWord("deliverTargetObject");
  ms->pushWord(std::make_shared<EePoseWord>(ms->config.currentEEPose));
  ms->pushWord("setClosestBlueBoxMemoryToFocusedClass");
  ms->pushWord("mapLocal");
  ms->pushWord("1");
  ms->pushWord("changeToHeight");
  ms->pushWord("synchronicServo"); 
  ms->pushWord("synchronicServoTakeClosest");


  ms->pushWord("setPlaceModeToHold");
}
END_WORD
REGISTER_WORD(PlayLoop)

WORD(PlayLoopP)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ms->pushWord("quarterImpulse");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("xUp", 10);
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("zUp", 25);
  ms->pushCopies("xDown", 10);
  ms->pushWord("tenthImpulse");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("zUp", 3);
  ms->pushWord("pressAndRelease");
  ms->pushWord("comeToStop");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("zDown", 25);
  ms->pushWord("tenthImpulse");
  ms->pushWord("setGridSizeCoarse");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("xUp", 15);
  ms->pushCopies("yDown", 7);
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("turnAboutY");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushCopies("yUp", 7);
  ms->pushCopies("xDown", 15);
  ms->pushWord("1");
  ms->pushWord("changeToHeight");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("assumeBeeHome");
}
END_WORD
REGISTER_WORD(PlayLoopP)


}



