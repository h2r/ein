#include "ein.h"
#include <tf_conversions/tf_kdl.h>

#define isnan std::isnan
#define IKFAST_NO_MAIN // Don't include main() from IKFast
#include "ikfast/baxter_left_arm_ikfast_solver.cpp"

int ikfast_query(shared_ptr<MachineState> ms, baxter_core_msgs::SolvePositionIK * thisRequest, double free, string frame);
int ikfast_solve(shared_ptr <MachineState> ms, geometry_msgs::Pose pose, double free, IkSolutionList<IkReal> &solutions);


void fillIkRequest(eePose * givenEEPose, baxter_core_msgs::SolvePositionIK * givenIkRequest) {
  givenIkRequest->request.pose_stamp.resize(1);

  givenIkRequest->request.pose_stamp[0].header.seq = 0;
  givenIkRequest->request.pose_stamp[0].header.stamp = ros::Time::now();
  givenIkRequest->request.pose_stamp[0].header.frame_id = "/base";

  
  givenIkRequest->request.pose_stamp[0].pose.position.x = givenEEPose->px;
  givenIkRequest->request.pose_stamp[0].pose.position.y = givenEEPose->py;
  givenIkRequest->request.pose_stamp[0].pose.position.z = givenEEPose->pz;

//  Eigen::Quaternionf normalizer(givenEEPose->qw, givenEEPose->qx, givenEEPose->qy, givenEEPose->qz);
//  normalizer.normalize();
//  givenEEPose->qx = normalizer.x();
//  givenEEPose->qy = normalizer.y();
//  givenEEPose->qz = normalizer.z();
//  givenEEPose->qw = normalizer.w();

  givenIkRequest->request.pose_stamp[0].pose.orientation.x = givenEEPose->qx;
  givenIkRequest->request.pose_stamp[0].pose.orientation.y = givenEEPose->qy;
  givenIkRequest->request.pose_stamp[0].pose.orientation.z = givenEEPose->qz;
  givenIkRequest->request.pose_stamp[0].pose.orientation.w = givenEEPose->qw;
}

void reseedIkRequest(shared_ptr<MachineState> ms, eePose *givenEEPose, baxter_core_msgs::SolvePositionIK * givenIkRequest, int it, int itMax) {

  double jointSeedAmplitude = (3.1415926 * double(it) / double(itMax));
  double jointSeedAmplitudeMin = 0.02;
  jointSeedAmplitude = max(jointSeedAmplitude, jointSeedAmplitudeMin);

  if (ms->config.goodIkInitialized) {
    givenIkRequest->request.seed_mode = 1; // SEED_USER
    givenIkRequest->request.seed_angles.resize(1);
    givenIkRequest->request.seed_angles[0].position.resize(NUM_JOINTS);
    givenIkRequest->request.seed_angles[0].name.resize(NUM_JOINTS);
    for (int j = 0; j < NUM_JOINTS; j++) {
      givenIkRequest->request.seed_angles[0].name[j] = ms->config.lastGoodIkRequest.response.joints[0].name[j];
      givenIkRequest->request.seed_angles[0].position[j] = ms->config.lastGoodIkRequest.response.joints[0].position[j] + 
	((drand48() - 0.5)*2.0*jointSeedAmplitude);
    }
  } else {
    ROS_WARN_STREAM("_______**__________");
    ROS_WARN_STREAM("_____*____*________");
    ROS_ERROR_STREAM("Uh oh, tried to reseed ik before it was initialized, so returning with no action.");
    ROS_WARN_STREAM("_____*____*________");
    ROS_WARN_STREAM("_______**__________");
    return;
  }
}

bool willIkResultFail(shared_ptr<MachineState> ms, baxter_core_msgs::SolvePositionIK thisIkRequest, int thisIkCallResult, bool * likelyInCollision) {
  bool thisIkResultFailed = 0;
  *likelyInCollision = 0;

  if (thisIkCallResult && thisIkRequest.response.isValid[0]) {
    thisIkResultFailed = 0;
  } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].position.size() != NUM_JOINTS)) {
    thisIkResultFailed = 1;
    //cout << "Initial IK result appears to be truly invalid, not enough positions." << endl;
  } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].name.size() != NUM_JOINTS)) {
    thisIkResultFailed = 1;
    //cout << "Initial IK result appears to be truly invalid, not enough names." << endl;
  } else if (thisIkRequest.response.joints.size() == 1) {
    if( ms->config.usePotentiallyCollidingIK ) {
      //cout << "WARNING: using ik even though result was invalid under presumption of false collision..." << endl;
      //cout << "Received enough positions and names for ikPose: " << thisIkRequest.request.pose_stamp[0].pose << endl;
      thisIkResultFailed = 0;
      *likelyInCollision = 1;
    } else {
      thisIkResultFailed = 1;
      *likelyInCollision = 1;
    }
  } else {
    thisIkResultFailed = 1;
    //cout << "Initial IK result appears to be truly invalid, incorrect joint field." << endl;
  }

  return thisIkResultFailed;
}


void queryIK(shared_ptr<MachineState> ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest) {
  if (ms->config.currentRobotMode == PHYSICAL) {
    *thisResult = ms->config.ikClient.call(*thisRequest);
    cout << "Asking for IK: " << thisRequest->request.pose_stamp[0].pose.position.x << ",";
    cout << thisRequest->request.pose_stamp[0].pose.position.y << ",";
    cout << thisRequest->request.pose_stamp[0].pose.position.z << endl;
    cout << "Result: ";
    if (*thisResult && thisRequest->response.isValid[0]) {
      for (int j = 0; j < NUM_JOINTS; j++) {
        cout << thisRequest->response.joints[0].name[j] << ": ";
        cout << thisRequest->response.joints[0].position[j] << ", ";
      }
      cout << endl;
    } else {
      cout << "invalid" << endl;
    }
    
    //query_ikfast(ms, thisRequest, 1.13812, "left_arm_mount");
    if (*thisResult && thisRequest->response.isValid[0])  {
      //query_ikfast(ms, thisRequest, thisRequest->response.joints[0].position[5], "left_arm_mount");
      ikfast_query(ms, thisRequest, 1.52056, "left_arm_mount");
    }


  } else if (ms->config.currentRobotMode == SIMULATED) {
    *thisResult = 1;
  } else {
    assert(0);
  }
}


int ikfast_query(shared_ptr<MachineState> ms, baxter_core_msgs::SolvePositionIK * thisRequest, double free, string transform) {


  geometry_msgs::PoseStamped base_pose = thisRequest->request.pose_stamp[0];
  base_pose.header.stamp = ros::Time(0);
  geometry_msgs::PoseStamped transformed_pose;

  ms->config.tfListener->transformPose(transform, base_pose, transformed_pose);
  IkSolutionList<IkReal> solutions;
  ikfast_solve(ms, transformed_pose.pose, free, solutions);

  for (int i = 0; i < solutions.GetNumSolutions(); i++) {
    vector<double> solution;
    solution.clear();
    solution.resize(GetNumJoints());

    const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
    std::vector<IkReal> vsolfree( sol.GetFree().size() );
    sol.GetSolution(&solution[0],vsolfree.size()>0?&vsolfree[0]:NULL);
    cout << "ikfast: " ;
    for (int j = 0; j < solution.size(); j++) {
      cout << solution[j] << ", " ;
    }
    cout << endl;
    for (int j = 0; j < NUM_JOINTS; j++) {
      //thisRequest->response.joints[0].position[j] = solution[j];
    }

  }

  

}

int ikfast_solve(shared_ptr <MachineState> ms, geometry_msgs::Pose pose, double free, IkSolutionList<IkReal> &solutions) {

  KDL::Frame pose_frame;
  tf::poseMsgToKDL(pose, pose_frame);
  
  std::vector<double> vfree(1);
  vfree[0] = free;
  
  // For **Transform6D**, eerot is 9 values for the 3x3 rotation matrix.
  
  
  KDL::Rotation mult;
  KDL::Vector direction;
  
  double trans[3];
  trans[0] = pose_frame.p[0];//-.18;
  trans[1] = pose_frame.p[1];
  trans[2] = pose_frame.p[2];
  
  cout << "Trans: " << trans[0] << ", " << trans[1] << ", " << trans[2] << endl;
  
  mult = pose_frame.M;
  
  double vals[9];
  vals[0] = mult(0,0);
  vals[1] = mult(0,1);
  vals[2] = mult(0,2);
  vals[3] = mult(1,0);
  vals[4] = mult(1,1);
  vals[5] = mult(1,2);
  vals[6] = mult(2,0);
  vals[7] = mult(2,1);
  vals[8] = mult(2,2);
  
  // IKFast56/61
  ComputeIk(trans, vals, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
  cout <<  "Ik fast solution: " << solutions.GetNumSolutions() << endl;
  
  
  return solutions.GetNumSolutions();
}
