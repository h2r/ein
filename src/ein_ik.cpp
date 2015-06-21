#include "ein.h"
#include <tf_conversions/tf_kdl.h>

#define isnan std::isnan
#define IKFAST_NO_MAIN // Don't include main() from IKFast
#include "ikfast/baxter_left_arm_ikfast_solver.cpp"

int ikfast_query(shared_ptr<MachineState> ms, baxter_core_msgs::SolvePositionIK * thisRequest, double free, string frame);
int ikfast_solve(shared_ptr <MachineState> ms, geometry_msgs::Pose pose, double free, IkSolutionList<IkReal> &solutions);
bool ikfast_search(shared_ptr <MachineState> ms, geometry_msgs::Pose pose, double free, std::vector<double>& solutions);

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
}


void queryIK(shared_ptr<MachineState> ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest) {
  if (ms->config.currentRobotMode == PHYSICAL) {
    queryIKService(ms, thisResult, thisRequest);
    //queryIKFast(ms, thisResult, thisRequest);
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
  vector<double> solution;
  bool result = ikfast_search(ms, transformed_pose.pose, free, solution);

  if (result) {
    for (int j = 0; j < NUM_JOINTS; j++) {
      thisRequest->response.joints[0].position[j] = solution[j];
    }
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

bool ikfast_search(shared_ptr <MachineState> ms, geometry_msgs::Pose pose, double free, std::vector<double>& sol)  {
  ROS_DEBUG_STREAM_NAMED("ikfast","searchPositionIK");

  double vfree = free;
  double search_discretization = 0.005;

  double joint_min_vector[] = {-1.70168, -2.147, -3.05418, -0.05, -3.059, -1.5708, -3.059};
  double joint_max_vector[] = {1.70168, 1.047, 3.05418, 2.618, 3.059, 2.094, 3.059};

  int counter = 0;
  int num_positive_increments;
  int num_negative_increments;


  while(true)
  {
    IkSolutionList<IkReal> solutions;
    int numsol = ikfast_solve(ms, pose, vfree, solutions);

    //ROS_INFO_STREAM_NAMED("ikfast","Found " << numsol << " solutions from IKFast");

    if( numsol > 0 ) {
      for(int s = 0; s < numsol; ++s) {

        ikfast_getSolution(solutions,s,sol);

        bool obeys_limits = true;
        for(unsigned int i = 0; i < sol.size(); i++) {
          if(sol[i] < joint_min_vector[i] || sol[i] > joint_max_vector[i]) {
            obeys_limits = false;
            break;
          }
          //ROS_INFO_STREAM_NAMED("ikfast","Num " << i << " value " << sol[i] << " has limits " << joint_has_limits_vector_[i] << " " << joint_min_vector_[i] << " " << joint_max_vector_[i]);
        }
        if(obeys_limits) {
            return true;
        }
      }
      if (counter > 100) {
        return false;
      }
    }

    vfree = free + search_discretization*counter;
    //ROS_INFO_STREAM_NAMED("ikfast","Attempt " << counter << " with 0th free joint having value " << vfree);
    counter += 1;
  }

  ROS_ERROR("Returning zero in ein ik fast, falling out of the while loop.");
  return false;

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
