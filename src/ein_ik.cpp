#include "ein.h"
#include <tf_conversions/tf_kdl.h>

#define isnan std::isnan
#define IKFAST_NO_MAIN // Don't include main() from IKFast
#include "ikfast/baxter_left_arm_ikfast_solver.cpp"

void queryIKFast(shared_ptr<MachineState> ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest);
int ikfast_solve(shared_ptr <MachineState> ms, geometry_msgs::Pose pose, double free, IkSolutionList<IkReal> &solutions);
bool ikfast_search(shared_ptr <MachineState> ms, geometry_msgs::Pose pose, double free, std::vector<double>& solutions);
double joint_error(vector<double> p1, vector<double> p2);
bool obeys_limits(vector<double> joints, vector<double> joint_max_vector, vector<double> joint_min_vector, double tolerance);

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

void queryIKService(shared_ptr<MachineState> ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest) {
    *thisResult = ms->config.ikClient.call(*thisRequest);
    //cout << "Asking for IK: " << thisRequest->request.pose_stamp[0].pose.position.x << ",";
    //cout << thisRequest->request.pose_stamp[0].pose.position.y << ",";
    //cout << thisRequest->request.pose_stamp[0].pose.position.z << endl;
    //cout << "Result: ";
    // if (*thisResult && thisRequest->response.isValid[0]) {
    //   for (int j = 0; j < NUM_JOINTS; j++) {
    //     cout << thisRequest->response.joints[0].name[j] << ": ";
    //     cout << thisRequest->response.joints[0].position[j] << ", ";
    //   }
    //   cout << endl;
    // } else {
    //   cout << "invalid" << endl;
    // }
 
}


void queryIK(shared_ptr<MachineState> ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest) {
  if (ms->config.currentRobotMode == PHYSICAL) {
    if(ms->config.currentIKMode == IKSERVICE) {
      queryIKService(ms, thisResult, thisRequest);
    } else if (ms->config.currentIKMode == IKFAST) {
      queryIKFast(ms, thisResult, thisRequest);
    } else {
      assert(0);
    }
  } else if (ms->config.currentRobotMode == SIMULATED) {
    *thisResult = 1;
  } else {
    assert(0);
  }
}



void queryIKFast(shared_ptr<MachineState> ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest) {
  string transform = ms->config.left_or_right_arm + "_arm_mount";
  int num = GetNumFreeParameters();
  int * free_params = GetFreeParameters();
  assert(num == 1);
  int free_joint_idx = free_params[0];
  double free = ms->config.trueJointPositions[free_joint_idx];

  geometry_msgs::PoseStamped base_pose = thisRequest->request.pose_stamp[0];
  base_pose.header.stamp = ros::Time(0);
  geometry_msgs::PoseStamped transformed_pose;

  ms->config.tfListener->transformPose(transform, base_pose, transformed_pose);
  vector<double> solution;
  bool result = ikfast_search(ms, transformed_pose.pose, free, solution);
  thisRequest->response.isValid.resize(1);

  if (result) {
    thisRequest->response.isValid[0] = 1;
    thisRequest->response.joints.resize(1);
    thisRequest->response.joints[0].position.resize(NUM_JOINTS);
    for (int j = 0; j < NUM_JOINTS; j++) {
      thisRequest->response.joints[0].position[j] = solution[j];
    }
    *thisResult = 1;
  } else {
    *thisResult = 0;
    thisRequest->response.isValid[0] = 0;
  }

  //IkSolutionList<IkReal> solutions;
  //ikfast_solve(ms, transformed_pose.pose, free, solutions);
  

  // for (int i = 0; i < solutions.GetNumSolutions(); i++) {
  //   vector<double> solution;
  //   solution.clear();
  //   solution.resize(GetNumJoints());

  //   const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
  //   std::vector<IkReal> vsolfree( sol.GetFree().size() );
  //   sol.GetSolution(&solution[0],vsolfree.size()>0?&vsolfree[0]:NULL);
  //   cout << "ikfast: " ;
  //   for (int j = 0; j < solution.size(); j++) {
  //     cout << solution[j] << ", " ;
  //   }
  //   cout << endl;
  //   for (int j = 0; j < NUM_JOINTS; j++) {
  //     //thisRequest->response.joints[0].position[j] = solution[j];
  //   }

  // }


}

void ikfast_getSolution(const IkSolutionList<IkReal> &solutions, int i, std::vector<double>& solution) {
  solution.clear();
  solution.resize(NUM_JOINTS);

  // IKFast56/61
  const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
  std::vector<IkReal> vsolfree( sol.GetFree().size() );
  sol.GetSolution(&solution[0],vsolfree.size()>0?&vsolfree[0]:NULL);
}

bool ikfast_search(shared_ptr <MachineState> ms, geometry_msgs::Pose pose, double free, std::vector<double>& outsol)  {
  ROS_DEBUG_STREAM_NAMED("ikfast","searchPositionIK");

  
  double vfree = free;

  vector<double> current_joints;
  for (int i = 0; i < NUM_JOINTS; i++) {
    current_joints.push_back(ms->config.trueJointPositions[i]);
  }
  double joint_min[] = {-1.70168, -2.147, -3.05418, -0.05, -3.059, -1.5708, -3.059};
  double joint_max[] = {1.70168, 1.047, 3.05418, 2.618, 3.059, 2.094, 3.059};
  vector<double> joint_min_vector;
  vector<double> joint_max_vector;

  for (int i = 0; i < NUM_JOINTS; i++) {
    joint_min_vector.push_back(joint_min[i]);
    joint_max_vector.push_back(joint_max[i]);
  }


  double joint_safety = 0.01;

  int counter = 0;
  int num = GetNumFreeParameters();
  int * free_params = GetFreeParameters();
  assert(num == 1);
  int free_joint_idx = free_params[0];
  double step = (joint_max_vector[free_joint_idx] - joint_min_vector[free_joint_idx]) / 100;

  if (vfree <= joint_min_vector[free_joint_idx] || vfree >= joint_max_vector[free_joint_idx] || true) {
    vfree = 0.5 * (joint_min_vector[free_joint_idx] + joint_max_vector[free_joint_idx]);
  }

  vector<double> free_params_up;
  vector<double> free_params_down;
  for (double f = vfree; f <= joint_max_vector[free_joint_idx]; f+= step) {
    free_params_up.push_back(f);
  }
  for (double f = joint_min_vector[free_joint_idx]; f < vfree; f+= step) {
    free_params_down.push_back(f);
  }

  int i_up = 0;
  int i_down = 0;
  while(true)
  {
    IkSolutionList<IkReal> solutions;
    int numsol = ikfast_solve(ms, pose, vfree, solutions);
    double max_dist = VERYBIGNUMBER;
    int max_idx = -1;
    //ROS_INFO_STREAM_NAMED("ikfast","Found " << numsol << " solutions from IKFast");
    vector<double> solution;

    for(int s = 0; s < numsol; ++s) {
      ikfast_getSolution(solutions,s,solution);
      
      if (obeys_limits(solution, joint_min_vector, joint_max_vector, joint_safety)) {
        outsol = solution;
        return true;
        double error = joint_error(solution, current_joints);
        if (error < max_dist) {
          max_dist = error;
          max_idx = s;
        }
      }
    }
    if (max_idx != -1) {
      ikfast_getSolution(solutions, max_idx, outsol);
      return true;
    }
    
      

    if (counter > 100) {
      return false;
    }
    if (counter % 2 == 0) {
      if (i_up < free_params_up.size()) {
        vfree = free_params_up[i_up];
        i_up++;
      }
    } else {
      if (i_down < free_params_down.size()) {
        vfree = free_params_down[i_down];
        i_down++;
      }
    }

    //ROS_INFO_STREAM_NAMED("ikfast","Attempt " << counter << " with 0th free joint having value " << vfree);
    counter += 1;
  }

  ROS_ERROR("Returning zero in ein ik fast, falling out of the while loop.");
  return false;

}

bool obeys_limits(vector<double> joints, vector<double> joint_min_vector, vector<double> joint_max_vector, double tolerance) {
  for(unsigned int i = 0; i < joints.size(); i++) {
    if (joints[i] < joint_min_vector[i] + tolerance || joints[i] > joint_max_vector[i] - tolerance) {
      return false;
    }
  }
  return true;

}

double joint_error(vector<double> p1, vector<double> p2) {
  double error = 0;
  assert(p1.size() == p2.size());
  for (int i = 0; i < p1.size(); i++) {
    error += fabs(p1[i] - p2[i]);
  }
  return error;
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
  
  //cout << "Trans: " << trans[0] << ", " << trans[1] << ", " << trans[2] << endl;
  
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
  //cout <<  "Ik fast solution: " << solutions.GetNumSolutions() << endl;
  
  
  return solutions.GetNumSolutions();
}
