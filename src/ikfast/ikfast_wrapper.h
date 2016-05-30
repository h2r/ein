#include "ein.h"
#include <tf_conversions/tf_kdl.h>
#include <geometry_msgs/PoseStamped.h>



namespace MY_NAMESPACE {

void queryIKFast(MachineState * ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest);
void queryIKFastDebug(MachineState * ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest);

eePose ikfast_computeFK(MachineState * ms, vector<double> joint_angles);

}
