#include "ein.h"
#include <tf_conversions/tf_kdl.h>
#include <geometry_msgs/PoseStamped.h>

#include "ikfast/ikfast_wrapper_left.h"
#undef IKFAST_NAMESPACE
#undef IKFAST_SOLVER_CPP
#undef MY_NAMESPACE
#include "ikfast/ikfast_wrapper_right.h"


void fillIkRequest(eePose givenEEPose, baxter_core_msgs::SolvePositionIK * givenIkRequest) {
  givenIkRequest->request.pose_stamp.resize(1);

  givenIkRequest->request.pose_stamp[0].header.seq = 0;
  givenIkRequest->request.pose_stamp[0].header.stamp = ros::Time::now();
  givenIkRequest->request.pose_stamp[0].header.frame_id = "/base";

  
  givenIkRequest->request.pose_stamp[0].pose.position.x = givenEEPose.px;
  givenIkRequest->request.pose_stamp[0].pose.position.y = givenEEPose.py;
  givenIkRequest->request.pose_stamp[0].pose.position.z = givenEEPose.pz;

  givenIkRequest->request.pose_stamp[0].pose.orientation.x = givenEEPose.qx;
  givenIkRequest->request.pose_stamp[0].pose.orientation.y = givenEEPose.qy;
  givenIkRequest->request.pose_stamp[0].pose.orientation.z = givenEEPose.qz;
  givenIkRequest->request.pose_stamp[0].pose.orientation.w = givenEEPose.qw;
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
    // fill in 

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
  // cache for later
  eePose currentEEPoseRequested;
  {
    currentEEPoseRequested.px = thisRequest->request.pose_stamp[0].pose.position.x;
    currentEEPoseRequested.py = thisRequest->request.pose_stamp[0].pose.position.y;
    currentEEPoseRequested.pz = thisRequest->request.pose_stamp[0].pose.position.z;

    currentEEPoseRequested.qx = thisRequest->request.pose_stamp[0].pose.orientation.x;
    currentEEPoseRequested.qy = thisRequest->request.pose_stamp[0].pose.orientation.y;
    currentEEPoseRequested.qz = thisRequest->request.pose_stamp[0].pose.orientation.z;
    currentEEPoseRequested.qw = thisRequest->request.pose_stamp[0].pose.orientation.w;
  }

  eePose handPoint;
  if (ms->config.currentRobotMode == PHYSICAL) {
    if(ms->config.currentIKMode == IKSERVICE) {
      handPoint = ms->config.handToRethinkEndPointTransform;
    } else if (ms->config.currentIKMode == IKFAST) {
      handPoint = eePose(0,0,0.0437, 0,0,0,1);
    } else if (ms->config.currentIKMode == IKFASTDEBUG) {
    } else {
      assert(0);
    }
  } else if (ms->config.currentRobotMode == SIMULATED) {
    *thisResult = 1;
  } else {
    assert(0);
  }

  // correct for IK service
  eePose endPointCorrectedEEPoseToRequest;
  {
    eePose desiredHandPose = ms->config.handFromEndEffectorTransform.applyAsRelativePoseTo(currentEEPoseRequested);
    eePose desiredEndPointPose = handPoint.applyAsRelativePoseTo(desiredHandPose);
    //cout << currentEEPoseRequested << desiredHandPose << desiredEndPointPose << endl;
    //cout << ms->config.handFromEndEffectorTransform << ms->config.handToRethinkEndPointTransform << endl;
    //endPointCorrectedEEPoseToRequest = ms->config.handToRethinkEndPointTransform.applyAsRelativePoseTo(desiredHandPose);
    //endPointCorrectedEEPoseToRequest = ms->config.handToRethinkEndPointTransform.applyAsRelativePoseTo(currentEEPoseRequested);
    //cout << ms->config.handToRethinkEndPointTransform << endl;


    //endPointCorrectedEEPoseToRequest = currentEEPoseRequested;
    endPointCorrectedEEPoseToRequest = desiredEndPointPose;
  }

  {
    thisRequest->request.pose_stamp[0].pose.position.x = endPointCorrectedEEPoseToRequest.px;
    thisRequest->request.pose_stamp[0].pose.position.y = endPointCorrectedEEPoseToRequest.py;
    thisRequest->request.pose_stamp[0].pose.position.z = endPointCorrectedEEPoseToRequest.pz;

    thisRequest->request.pose_stamp[0].pose.orientation.x = endPointCorrectedEEPoseToRequest.qx;
    thisRequest->request.pose_stamp[0].pose.orientation.y = endPointCorrectedEEPoseToRequest.qy;
    thisRequest->request.pose_stamp[0].pose.orientation.z = endPointCorrectedEEPoseToRequest.qz;
    thisRequest->request.pose_stamp[0].pose.orientation.w = endPointCorrectedEEPoseToRequest.qw;
  }

  if (ms->config.currentRobotMode == PHYSICAL) {
    if(ms->config.currentIKMode == IKSERVICE) {
      queryIKService(ms, thisResult, thisRequest);
    } else if (ms->config.currentIKMode == IKFAST) {
      if (ms->config.left_or_right_arm == "left") {
        ikfast_left_ein::queryIKFast(ms, thisResult, thisRequest);
      } else if (ms->config.left_or_right_arm == "right") {
        ikfast_right_ein::queryIKFast(ms, thisResult, thisRequest);
      } else {
        assert(0);
      }
    } else if (ms->config.currentIKMode == IKFASTDEBUG) {
      if (ms->config.left_or_right_arm == "left") {
        ikfast_left_ein::queryIKFastDebug(ms, thisResult, thisRequest);
      } else if (ms->config.left_or_right_arm == "right") {
        ikfast_right_ein::queryIKFastDebug(ms, thisResult, thisRequest);
      } else {
        assert(0);
      }
    } else {
      assert(0);
    }
  } else if (ms->config.currentRobotMode == SIMULATED) {
    *thisResult = 1;
  } else {
    assert(0);
  }

  // replace with original before returning
  {
    thisRequest->request.pose_stamp[0].pose.position.x = currentEEPoseRequested.px;
    thisRequest->request.pose_stamp[0].pose.position.y = currentEEPoseRequested.py;
    thisRequest->request.pose_stamp[0].pose.position.z = currentEEPoseRequested.pz;

    thisRequest->request.pose_stamp[0].pose.orientation.x = currentEEPoseRequested.qx;
    thisRequest->request.pose_stamp[0].pose.orientation.y = currentEEPoseRequested.qy;
    thisRequest->request.pose_stamp[0].pose.orientation.z = currentEEPoseRequested.qz;
    thisRequest->request.pose_stamp[0].pose.orientation.w = currentEEPoseRequested.qw;
  }
}


ikMapState ikAtPose(shared_ptr<MachineState> ms, eePose pose) {

  baxter_core_msgs::SolvePositionIK thisIkRequest;
  fillIkRequest(pose, &thisIkRequest);
  
  bool likelyInCollision = 0;
  // ATTN 24
  //int thisIkCallResult = ms->config.ikClient.call(thisIkRequest);
  int thisIkCallResult = 0;
  queryIK(ms, &thisIkCallResult, &thisIkRequest);
  
  int ikResultFailed = 1;
  if (ms->config.currentRobotMode == PHYSICAL) {
    ikResultFailed = willIkResultFail(ms, thisIkRequest, thisIkCallResult, &likelyInCollision);
  } else if (ms->config.currentRobotMode == SIMULATED) {
    ikResultFailed = !positionIsSearched(ms, pose.px, pose.py);
  } else {
    assert(0);
  }
  
  int foundGoodPosition = !ikResultFailed;
  //ms->config.ikMap[i + ms->config.mapWidth * j] = ikResultFailed;
  //ms->config.ikMap[i + ms->config.mapWidth * j] = 1;
  //cout << i << " " << j << endl;
  if (ikResultFailed) {
    return IK_FAILED;
  } else {
    if (likelyInCollision) {
      return IK_LIKELY_IN_COLLISION;
    } else {
      return IK_GOOD;
    }
  }

}
