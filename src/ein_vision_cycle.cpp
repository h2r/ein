#include "ein_words.h"
#include "ein.h"
#include "qtgui/einwindow.h"
#include "camera.h"

namespace ein_words {

WORD(ClearStackIntoMappingPatrol)
virtual void execute(MachineState * ms) {
  ms->clearStack();
  ms->pushWord("mappingPatrol");
  ms->execute_stack = 1;
}
END_WORD
REGISTER_WORD(ClearStackIntoMappingPatrol)


WORD(ToggleShouldIDoIK)
virtual void execute(MachineState * ms) {
  ms->config.shouldIDoIK = !ms->config.shouldIDoIK;
}
END_WORD
REGISTER_WORD(ToggleShouldIDoIK)

WORD(ToggleShouldIRender)
virtual void execute(MachineState * ms) {
  ms->config.shouldIRender = !ms->config.shouldIRender;
}
END_WORD
REGISTER_WORD(ToggleShouldIRender)

WORD(ToggleDrawClearanceMap)
virtual void execute(MachineState * ms) {
  ms->config.drawClearanceMap = !ms->config.drawClearanceMap;
}
END_WORD
REGISTER_WORD(ToggleDrawClearanceMap)

WORD(ToggleDrawIKMap)
virtual void execute(MachineState * ms) {
  ms->config.drawIKMap = !ms->config.drawIKMap;
}
END_WORD
REGISTER_WORD(ToggleDrawIKMap)

WORD(ToggleUseGlow)
virtual void execute(MachineState * ms) {
  ms->config.useGlow = !ms->config.useGlow;
}
END_WORD
REGISTER_WORD(ToggleUseGlow)

WORD(ToggleUseFade)
virtual void execute(MachineState * ms) {
  ms->config.useFade = !ms->config.useFade;
}
END_WORD
REGISTER_WORD(ToggleUseFade)

CONFIG_GETTER_INT(PursuitProximity, ms->config.pursuitProximity)
CONFIG_SETTER_INT(SetPursuitProximity, ms->config.pursuitProximity)

CONFIG_GETTER_INT(SearchProximity, ms->config.searchProximity)
CONFIG_SETTER_INT(SetSearchProximity, ms->config.searchProximity)



}
