#include "ein.h"
#include <tf_conversions/tf_kdl.h>
#include <geometry_msgs/PoseStamped.h>

void WRAPPER_queryIKFast(shared_ptr<MachineState> ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest);
void WRAPPER_queryIKFastDebug(shared_ptr<MachineState> ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest);

