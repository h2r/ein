#include "ein.h"
#include <tf_conversions/tf_kdl.h>
#include <geometry_msgs/PoseStamped.h>



namespace MY_NAMESPACE {

void queryIKFast(shared_ptr<MachineState> ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest);
void queryIKFastDebug(shared_ptr<MachineState> ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest);

eePose ikfast_computeFK(shared_ptr<MachineState> ms, vector<double> joint_angles);

}
