#ifndef _EIN_IK_H_
#define _EIN_IK_H_

ikMapState ikAtPose(MachineState * ms, eePose pose);
vector<ikMapState> ikAtPoses(MachineState * ms, vector<eePose> poses);

bool willIkResultFail(MachineState * ms, baxter_core_msgs::SolvePositionIK thisIkRequest, int thisIkCallResult, bool * likelyInCollision, int i);

void fillIkRequest(eePose givenEEPose, baxter_core_msgs::SolvePositionIK * givenIkRequest);
void fillIkRequest(vector<eePose> poses, baxter_core_msgs::SolvePositionIK * givenIkRequest);

void reseedIkRequest(MachineState * ms, eePose *givenEEPose, baxter_core_msgs::SolvePositionIK * givenIkRequest, int it, int itMax);

void queryIK(MachineState * ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest);

#endif /* _EIN_IK_H_ */
