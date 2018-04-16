#include <object_recognition_msgs/RecognizedObjectArray.h>

#include "ein_words.h"
#include "ein.h"
#include "qtgui/einwindow.h"
#include "camera.h"

namespace ein_words {

WORD(GuiShowAll)
virtual void execute(MachineState * ms)
{

  if (!ms->config.showgui) {
    return;
  }  
  ms->config.dogSnoutViewWindow->setVisible(true);

  ms->config.rangeogramWindow->setVisible(true);

  ms->config.objectViewerWindow->setVisible(true);

  ms->config.objectMapViewerWindow->setVisible(true);

  ms->config.densityViewerWindow->setVisible(true);

  ms->config.mapBackgroundViewWindow->setVisible(true);

  ms->config.wristViewWindow->setVisible(true);
  ms->config.coreViewWindow->setVisible(true);
}
END_WORD
REGISTER_WORD(GuiShowAll)

WORD(GuiHideAll)
virtual void execute(MachineState * ms)
{

  if (!ms->config.showgui) {
    return;
  }
  ms->config.dogSnoutViewWindow->setVisible(false);

  ms->config.rangeogramWindow->setVisible(false);

  ms->config.objectViewerWindow->setVisible(false);

  ms->config.objectMapViewerWindow->setVisible(false);

  ms->config.densityViewerWindow->setVisible(false);

  ms->config.mapBackgroundViewWindow->setVisible(false);
  ms->config.wristViewWindow->setVisible(false);
  ms->config.coreViewWindow->setVisible(false);
}
END_WORD
REGISTER_WORD(GuiHideAll)

WORD(GuiCustom1)
virtual void execute(MachineState * ms)
{

    
  if (!ms->config.showgui) {
    return;
  }    
  ms->config.rangeogramWindow->setVisible(false);

  ms->config.dogSnoutViewWindow->setVisible(false);

  ms->config.objectViewerWindow->setVisible(false);
  ms->config.objectMapViewerWindow->setVisible(false);
  ms->config.densityViewerWindow->setVisible(false);
  ms->config.mapBackgroundViewWindow->setVisible(false);
  ms->config.wristViewWindow->setVisible(false);
  ms->config.coreViewWindow->setVisible(false);
  ms->config.observedWindow->setVisible(false);
  ms->config.observedStdDevWindow->setVisible(false);
  ms->config.zWindow->setVisible(false);

  ms->config.predictedWindow->setVisible(false);
  ms->config.predictedStdDevWindow->setVisible(false);
  ms->config.backgroundWindow->setVisible(false);
  ms->config.backgroundWindow->setVisible(false);

  
}

END_WORD
REGISTER_WORD(GuiCustom1)


CONFIG_GETTER_DOUBLE(RenderWristViewBrightnessScalar, ms->config.wristViewBrightnessScalar, "")
CONFIG_SETTER_DOUBLE(RenderSetWristViewBrightnessScalar, ms->config.wristViewBrightnessScalar)


}
