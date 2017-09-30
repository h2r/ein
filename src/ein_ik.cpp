#include "ein.h"
#include <tf_conversions/tf_kdl.h>
#include <geometry_msgs/PoseStamped.h>
#include "ein_baxter_config.h"
#include "ikfast/ikfast_wrapper_left.h"
#undef IKFAST_NAMESPACE
#undef IKFAST_SOLVER_CPP
#undef MY_NAMESPACE
#include "ikfast/ikfast_wrapper_right.h"
#include "ein_words.h"

void fillIkRequest(vector<eePose> poses, baxter_core_msgs::SolvePositionIK * givenIkRequest) {
  givenIkRequest->request.pose_stamp.resize(poses.size());
  
  for (int i = 0; i < poses.size(); i++) {
    givenIkRequest->request.pose_stamp[i].header.seq = 0;
    givenIkRequest->request.pose_stamp[i].header.stamp = ros::Time::now();
    givenIkRequest->request.pose_stamp[i].header.frame_id = "/base";

  
    givenIkRequest->request.pose_stamp[i].pose.position.x = poses[i].px;
    givenIkRequest->request.pose_stamp[i].pose.position.y = poses[i].py;
    givenIkRequest->request.pose_stamp[i].pose.position.z = poses[i].pz;

    givenIkRequest->request.pose_stamp[i].pose.orientation.x = poses[i].qx;
    givenIkRequest->request.pose_stamp[i].pose.orientation.y = poses[i].qy;
    givenIkRequest->request.pose_stamp[i].pose.orientation.z = poses[i].qz;
    givenIkRequest->request.pose_stamp[i].pose.orientation.w = poses[i].qw;
  }
}


void fillIkRequest(eePose givenEEPose, baxter_core_msgs::SolvePositionIK * givenIkRequest) {

  vector<eePose> poses;
  poses.push_back(givenEEPose);

  fillIkRequest(poses, givenIkRequest);
}

void reseedIkRequest(MachineState * ms, eePose *givenEEPose, baxter_core_msgs::SolvePositionIK * givenIkRequest, int it, int itMax) {

  double jointSeedAmplitude = (3.1415926 * double(it) / double(itMax));
  double jointSeedAmplitudeMin = 0.02;
  jointSeedAmplitude = max(jointSeedAmplitude, jointSeedAmplitudeMin);

  if (ms->config.goodIkInitialized) {
    givenIkRequest->request.seed_mode = 1; // SEED_USER
    givenIkRequest->request.seed_angles.resize(1);
    givenIkRequest->request.seed_angles[0].position.resize(NUM_JOINTS);
    givenIkRequest->request.seed_angles[0].name.resize(NUM_JOINTS);
    for (int j = 0; j < NUM_JOINTS; j++) {
      givenIkRequest->request.seed_angles[0].name[j] = ms->config.baxterConfig->lastGoodIkRequest.response.joints[0].name[j];
      givenIkRequest->request.seed_angles[0].position[j] = ms->config.baxterConfig->lastGoodIkRequest.response.joints[0].position[j] + 
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

bool willIkResultFail(MachineState * ms, baxter_core_msgs::SolvePositionIK thisIkRequest, int thisIkCallResult, bool * likelyInCollision, int i) {
  bool thisIkResultFailed = 0;
  *likelyInCollision = 0;

  if (thisIkCallResult && thisIkRequest.response.isValid[i]) {
    thisIkResultFailed = 0;
  } else if (i >= thisIkRequest.response.joints.size()) {
      thisIkResultFailed = 1;
      cout << "Warning: received " << thisIkRequest.response.joints.size() << " results but requested i: " << i << endl;
  } else if (thisIkRequest.response.joints[i].position.size() != NUM_JOINTS) {
    thisIkResultFailed = 1;
    //cout << "Initial IK result appears to be truly invalid, not enough positions." << endl;
  } else if (thisIkRequest.response.joints[i].name.size() != NUM_JOINTS) {
    thisIkResultFailed = 1;
    //cout << "Initial IK result appears to be truly invalid, not enough names." << endl;
  } else {
    if( ms->config.usePotentiallyCollidingIK ) {
      //cout << "WARNING: using ik even though result was invalid under presumption of false collision..." << endl;
      //cout << "Received enough positions and names for ikPose: " << thisIkRequest.request.pose_stamp[0].pose << endl;
      thisIkResultFailed = 0;
      *likelyInCollision = 1;
    } else {
      thisIkResultFailed = 1;
      *likelyInCollision = 1;
    }
  }
  return thisIkResultFailed;
}

void queryIKService(MachineState * ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest) {
    // fill in 

    *thisResult = ms->config.baxterConfig->ikClient.call(*thisRequest);
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


void queryIK(MachineState * ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest) {
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

vector<ikMapState> ikAtPoses(MachineState * ms, vector<eePose> poses) {
  baxter_core_msgs::SolvePositionIK thisIkRequest;
  fillIkRequest(poses, &thisIkRequest);

  bool likelyInCollision = 0;
  int thisIkCallResult = 0;

  queryIK(ms, &thisIkCallResult, &thisIkRequest);

  vector<ikMapState> results;
  results.resize(poses.size());

  for (int i = 0; i < poses.size(); i++) {
    int ikResultFailed = 1;
    if (ms->config.currentRobotMode == PHYSICAL) {
      ikResultFailed = willIkResultFail(ms, thisIkRequest, thisIkCallResult, &likelyInCollision, i);
    } else if (ms->config.currentRobotMode == SIMULATED) {
      ikResultFailed = !positionIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, poses[i].px, poses[i].py);
    } else {
      assert(0);
    }
  
    int foundGoodPosition = !ikResultFailed;

    if (ikResultFailed) {
      results[i] = IK_FAILED;
    } else {
      if (likelyInCollision) {
        results[i] = IK_LIKELY_IN_COLLISION;
      } else {
        results[i] = IK_GOOD;
      }
    }
  }
  return results;

}

ikMapState ikAtPose(MachineState * ms, eePose pose) {
  vector<eePose> poses;
  poses.push_back(pose);
  vector<ikMapState> results = ikAtPoses(ms, poses);
  return results[0];
}





namespace ein_words {


WORD(FillClearanceMap)
virtual void execute(MachineState * ms) {
  {
    int proximity = ms->config.pursuitProximity;
    for (int i = 0; i < ms->config.mapWidth; i++) {
      for (int j = 0; j < ms->config.mapHeight; j++) {
	    if ( cellIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, 
                                ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, i, j) ) {
	      int iIStart = max(0, i-proximity);
	      int iIEnd = min(ms->config.mapWidth-1, i+proximity);
	      int iJStart = max(0, j-proximity);
	      int iJEnd = min(ms->config.mapHeight-1, j+proximity);

	      int reject = 0;
	      for (int iI = iIStart; iI <= iIEnd; iI++) {
	        for (int iJ = iJStart; iJ <= iJEnd; iJ++) {
	          if (  ( !cellIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, 
                                ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, iI, iJ) ) || 
	    	    (( ms->config.ikMap[iI + ms->config.mapWidth * iJ] != 0 ) &&
	    	    ( sqrt((iI-i)*(iI-i) + (iJ-j)*(iJ-j)) < proximity ))  ) {
	    	    reject = 1;
	          } else {
			  }
	        }
	      }

	      if (reject) {
	        ms->config.clearanceMap[i + ms->config.mapWidth * j] = 0;
	      } else {
	        ms->config.clearanceMap[i + ms->config.mapWidth * j] = 1;
	      }
	    } else {
	      ms->config.clearanceMap[i + ms->config.mapWidth * j] = 0;
	    }
      }
    }
  }
  {
    int proximity = ms->config.searchProximity;
    for (int i = 0; i < ms->config.mapWidth; i++) {
      for (int j = 0; j < ms->config.mapHeight; j++) {
	    if ( cellIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, 
                                ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, i, j) ) {
	      int iIStart = max(0, i-proximity);
	      int iIEnd = min(ms->config.mapWidth-1, i+proximity);
	      int iJStart = max(0, j-proximity);
	      int iJEnd = min(ms->config.mapHeight-1, j+proximity);

	      int reject = 0;
	      for (int iI = iIStart; iI <= iIEnd; iI++) {
	        for (int iJ = iJStart; iJ <= iJEnd; iJ++) {
	          if (  ( !cellIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, 
                                ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, iI, iJ) ) || 
	    	    (( ms->config.ikMap[iI + ms->config.mapWidth * iJ] != 0 ) &&
	    	    ( sqrt((iI-i)*(iI-i) + (iJ-j)*(iJ-j)) < proximity ))  ) {
	    	     reject = 1;
	          }
	        }
	      }

	      if (reject) {
	      } else {
	        ms->config.clearanceMap[i + ms->config.mapWidth * j] = 2;
	      }
	    } else {
	    }
      }
    }
  }
}
END_WORD
REGISTER_WORD(FillClearanceMap)

WORD(PointToClearanceMap)
virtual void execute(MachineState * ms) {

  int currentI, currentJ;
  mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, ms->config.currentEEPose.px, ms->config.currentEEPose.py, &currentI, &currentJ);

  int minI, minJ;
  double minDistanceToGreen = INFINITY;

  for (int i = 0; i < ms->config.mapWidth; i++) {
    for (int j = 0; j < ms->config.mapHeight; j++) {
      if (ms->config.clearanceMap[i + ms->config.mapWidth * j] == CLEARANCE_SEARCH) {
	double thisDistSq = ((i-currentI)*(i-currentI)+(j-currentJ)*(j-currentJ));
	if ( thisDistSq < minDistanceToGreen ) {
	  minDistanceToGreen = thisDistSq;
	  minI = i;
	  minJ = j; 
	} else {
	}
      } else {
      }
    }
  }

  double minX, minY;
  mapijToxy(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, minI, minJ, &minX, &minY);
  
  double crane1I = 1;
  double crane1J = 0;
  double craneAngle = vectorArcTan(ms, crane1I, crane1J);

  double maxDirI = (minI-currentI);
  double maxDirJ = (minJ-currentJ);
  double maxDirAngle = vectorArcTan(ms, maxDirI, maxDirJ);

  ms->config.currentEEPose.px = minX;
  ms->config.currentEEPose.py = minY;

  // XXX NOT DONE
  ms->config.currentEEDeltaRPY = eePose::zero();
  ms->config.currentEEDeltaRPY.pz = ( maxDirAngle - craneAngle );
  ms->config.currentEEPose.qx = 0.0;
  ms->config.currentEEPose.qy = 1.0;
  ms->config.currentEEPose.qz = 0.0;
  ms->config.currentEEPose.qw = 0.0;
  endEffectorAngularUpdate( &ms->config.currentEEPose, &ms->config.currentEEDeltaRPY );
}
END_WORD
REGISTER_WORD(PointToClearanceMap)

WORD(SaveIkMapAtHeight)
virtual void execute(MachineState * ms) {
  ofstream ofile;
  string fileName = ms->config.data_directory + ms->config.config_directory + ms->config.left_or_right_arm + "IkMapAtHeight";
  cout << "Saving ikMapAtHeight to " << fileName << endl;
  ofile.open(fileName, ios::trunc | ios::binary);
  ofile.write((char*)ms->config.ikMapAtHeight, sizeof(int)*ms->config.mapWidth*ms->config.mapHeight*ms->config.numIkMapHeights);
  ofile.close();
}
END_WORD
REGISTER_WORD(SaveIkMapAtHeight)

WORD(LoadIkMapAtHeight)
virtual void execute(MachineState * ms) {
  // binary seems overkill but consider that this map is
  //  for only one height and is 360kB in binary... how
  //  big would it be in yml, and what if we want another height?
  ifstream ifile;
  string fileName = ms->config.data_directory + ms->config.config_directory + ms->config.left_or_right_arm + "IkMapAtHeight";
  cout << "Loading ikMapAtHeight from " << fileName << endl;
  ifile.open(fileName, ios::binary);
  ifile.read((char*)ms->config.ikMapAtHeight, sizeof(int)*ms->config.mapWidth*ms->config.mapHeight*ms->config.numIkMapHeights);
  ifile.close();
}
END_WORD
REGISTER_WORD(LoadIkMapAtHeight)


WORD(SaveIkMap)
virtual void execute(MachineState * ms) {
  ofstream ofile;
  string fileName = ms->config.data_directory + ms->config.config_directory + ms->config.left_or_right_arm + "IkMap";
  cout << "Saving ikMap to " << fileName << endl;
  ofile.open(fileName, ios::trunc | ios::binary);
  ofile.write((char*)ms->config.ikMap, sizeof(int)*ms->config.mapWidth*ms->config.mapHeight);
  ofile.close();
}
END_WORD
REGISTER_WORD(SaveIkMap)

WORD(LoadIkMap)
virtual void execute(MachineState * ms) {
  // binary seems overkill but consider that this map is
  //  for only one height and is 360kB in binary... how
  //  big would it be in yml, and what if we want another height?
  ifstream ifile;
  string fileName = ms->config.data_directory + ms->config.config_directory + ms->config.left_or_right_arm + "IkMap";
  cout << "Loading ikMap from " << fileName << endl;
  ifile.open(fileName, ios::binary);
  ifile.read((char*)ms->config.ikMap, sizeof(int)*ms->config.mapWidth*ms->config.mapHeight);
  ifile.close();
}
END_WORD
REGISTER_WORD(LoadIkMap)




WORD(FillIkMapAtHeights)
virtual string description() {
  return "Fill the IK map at different heights.";
}
virtual void execute(MachineState * ms) {
  double ikStep = (ms->config.ikMapEndHeight - ms->config.ikMapStartHeight) / (ms->config.numIkMapHeights - 1);

  for (int i = 0; i < ms->config.numIkMapHeights; i++) {
    
    double height = ms->config.ikMapStartHeight + ikStep * i;
    stringstream program;
    program << "0 0 " << height << " fillIkMap " << i << " copyIkMapToHeightIdx";
    ms->evaluateProgram(program.str());
  }
}
END_WORD
REGISTER_WORD(FillIkMapAtHeights)

WORD(FillIkMapFromCachedHeights)
virtual string description() {
  return "Fill the IK map by taking the and of the result at all the different heights.";
}
virtual void execute(MachineState * ms) {
  for (int i = 0; i < ms->config.mapWidth; i++) {
    for (int j = 0; j < ms->config.mapHeight; j++) {
      bool result = IK_GOOD;
      for (int heightIdx = 0; heightIdx < ms->config.numIkMapHeights; heightIdx++) {
	if (ms->config.ikMapAtHeight[i  + ms->config.mapWidth * j + ms->config.mapWidth * ms->config.mapHeight * heightIdx] == IK_FAILED ||
	    ms->config.ikMapAtHeight[i  + ms->config.mapWidth * j + ms->config.mapWidth * ms->config.mapHeight * heightIdx] == IK_LIKELY_IN_COLLISION) {
	  result = IK_FAILED;
	  break;
	}
      }
      ms->config.ikMap[i + ms->config.mapWidth * j] = result;
    }
  }
}

END_WORD
REGISTER_WORD(FillIkMapFromCachedHeights)


WORD(FillIkMapFromCachedHeightIdx)
virtual string description() {
  return "Fill the IK map by taking the height idx from the cache.";
}
virtual void execute(MachineState * ms) {
  int heightIdx;
  GET_INT_ARG(ms, heightIdx);
  if (heightIdx >= ms->config.numIkMapHeights) {
    CONSOLE_ERROR(ms, "Ooops, height out of bounds. " << heightIdx);
    ms->pushWord("pauseStackExecution");   
    return;
  }
  
  for (int i = 0; i < ms->config.mapWidth; i++) {
    for (int j = 0; j < ms->config.mapHeight; j++) {
      ms->config.ikMap[i + ms->config.mapWidth * j] = ms->config.ikMapAtHeight[i  + ms->config.mapWidth * j + ms->config.mapWidth * ms->config.mapHeight * heightIdx];
    }
  }
}

END_WORD
REGISTER_WORD(FillIkMapFromCachedHeightIdx)




WORD(CopyIkMapToHeightIdx)
virtual string description() {
  return "Copy the ik map to the height index.";
}
virtual void execute(MachineState * ms) {
  int heightIdx;
  GET_INT_ARG(ms, heightIdx);
  for (int i = 0; i < ms->config.mapWidth; i++) {
    for (int j = 0; j < ms->config.mapHeight; j++) {
      ms->config.ikMapAtHeight[i  + ms->config.mapWidth * j + ms->config.mapWidth * ms->config.mapHeight * heightIdx] = ms->config.ikMap[i + ms->config.mapWidth * j];
    }
  }
}
END_WORD
REGISTER_WORD(CopyIkMapToHeightIdx)


WORD(ClearIkMap)
virtual string description() {
  return "Reset the IK Map so that every cell is good.";
}
virtual void execute(MachineState * ms) {
  for (int i = 0; i < ms->config.mapWidth; i++) {
    for (int j = 0; j < ms->config.mapHeight; j++) {
      ms->config.ikMap[i + ms->config.mapWidth * j] = IK_GOOD;
    }
  }
}
END_WORD
REGISTER_WORD(ClearIkMap)


WORD(FillIkMap)

virtual string description() {
  return "Fill the IK map for the current range starting at the i and j and height on the stack.";
}
virtual void execute(MachineState * ms) {
  int cellsPerQuery = 200;

  int currentI, currentJ;
  double height;
  GET_NUMERIC_ARG(ms, height);
  GET_INT_ARG(ms, currentJ);
  GET_INT_ARG(ms, currentI);



  int queries = 0;
  int i=currentI, j=currentJ;
  vector<eePose> poses;
  vector<tuple<int, int> > mapIndexes;

  for (; i < ms->config.mapWidth; i++) {
    if (queries < cellsPerQuery) {
      for (; j < ms->config.mapHeight; j++) {
	if (queries < cellsPerQuery) {
	  if ( cellIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, 
                                ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, i, j) ) {
	    double X, Y;
	    mapijToxy(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, i, j, &X, &Y);

	    eePose nextEEPose = ms->config.straightDown;
	    nextEEPose.px = X;
	    nextEEPose.py = Y;
	    nextEEPose.pz = height;
            poses.push_back(nextEEPose);
            mapIndexes.push_back(make_tuple(i, j));

	    queries++;
	  }
	} else {
	  break;
	}
      }
      // reset here so we don't bash the initial restart
      if ( !(j < ms->config.mapHeight) ) {
	j = 0;
      }

      if (queries < cellsPerQuery) {
	continue;
      } else {
	break;
      }
    } else {
      break;
    }
  }
  //cout << "Sending " << poses.size() << " poses to the IK Service..." << endl;
  vector<ikMapState> results = ikAtPoses(ms, poses);
  //cout << "Received " << results.size() << " results from the IK Service." << endl;
  //assert(results.size() == poses.size());
  //cout << "Sending " << poses.size() << " poses to the IK Service... separately" << endl;
  for (int k = 0; k < mapIndexes.size(); k++) {
    int ci = std::get<0>(mapIndexes[k]);
    int cj = std::get<1>(mapIndexes[k]);
    //ikMapState result = ikAtPose(ms, poses[k]);
    ms->config.ikMap[ci + ms->config.mapWidth * cj] = results[k];
    //ms->config.ikMap[ci + ms->config.mapWidth * cj] = result;
  }
  //cout << "Done." << endl;

  if (j >= ms->config.mapHeight) {
    j = 0;
  }

  if (i >= ms->config.mapWidth) {
    i = 0;
  } else {

    ms->pushWord("fillIkMap");
    ms->pushWord(make_shared<DoubleWord>(height));  
    ms->pushWord(make_shared<IntegerWord>(j));  
    ms->pushWord(make_shared<IntegerWord>(i));  
  }

  ms->config.endThisStackCollapse = 1;


}
END_WORD
REGISTER_WORD(FillIkMap)

WORD(FillIkMapAtCurrentHeight)

virtual string description() {
  return "Fill the IK map using data at the current EE height.  We run at height 2 usually.";
}

virtual void execute(MachineState * ms) {
  ms->evaluateProgram("0 0 currentPose eePosePZ fillIkMap");
}
END_WORD
REGISTER_WORD(FillIkMapAtCurrentHeight)

WORD(MoveToNextMapPosition)
virtual void execute(MachineState * ms) {
  int p_maxNextTries = 100;
  for (int tries = 0; tries < p_maxNextTries; tries++) {
    //ros::Time oldestTime = ros::Time::now();
    int oldestI=-1, oldestJ=-1;
    int foundASpot = 0;
    for (int scanRestarter = 0; scanRestarter < 2; scanRestarter++) {
      ros::Time oldestTime = ms->config.lastScanStarted;
      for (int i = 0; i < ms->config.mapWidth; i++) {
	    for (int j = 0; j < ms->config.mapHeight; j++) {
	      if (cellIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, 
                                ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, i, j) &&
	          (ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime <= oldestTime) &&
	          (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 2) &&
	          (ms->config.ikMap[i + ms->config.mapWidth * j] == IK_GOOD) ) {
	        oldestTime = ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime;
	        oldestI = i;
	        oldestJ = j;
	        foundASpot = 1;
	      }
	    }
      }

      if (!foundASpot) {
	cout << "moveToNextMapPosition all spots visited, currentPatrolMode: " << ms->config.currentPatrolMode << endl;
	if (ms->config.currentPatrolMode == LOOP) {
	  ms->config.lastScanStarted = ros::Time::now();
	  cout << "Restarting mappingPatrol." << endl;
	} else if (ms->config.currentPatrolMode == ONCE) {
	  cout << "Patrolled once, idling." << endl;
	  ms->execute_stack = 1;
	  ms->pushWord("idler");
	  return;
	} else {
	  assert(0);
	}
      }
    }

    if (oldestI == -1 || oldestJ == -1) {
      cout << "moveToNextMapPosition: failed to find a position but is looping, logical error. Clearing callstack." << endl;
      ms->clearStack();
      ms->pushCopies("beep", 15); // beep
      return;
    }

    double oldestX, oldestY;
    mapijToxy(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, oldestI, oldestJ, &oldestX, &oldestY);

    eePose nextEEPose = ms->config.currentEEPose;
    nextEEPose.px = oldestX;
    nextEEPose.py = oldestY;

    baxter_core_msgs::SolvePositionIK thisIkRequest;
    fillIkRequest(nextEEPose, &thisIkRequest);

    bool likelyInCollision = 0;
    // ATTN 24
    //int thisIkCallResult = ms->config.ikClient.call(thisIkRequest);
    int thisIkCallResult = 0;
    queryIK(ms, &thisIkCallResult, &thisIkRequest);

    int ikResultFailed = 1;
    if (ms->config.currentRobotMode == PHYSICAL) {
      ikResultFailed = willIkResultFail(ms, thisIkRequest, thisIkCallResult, &likelyInCollision, 0);
    } else if (ms->config.currentRobotMode == SIMULATED) {
      ikResultFailed = !positionIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, nextEEPose.px, nextEEPose.py);
    } else {
      assert(0);
    }

    int foundGoodPosition = !ikResultFailed;

    if (foundGoodPosition) {
      ms->config.currentEEPose.qx = ms->config.straightDown.qx;
      ms->config.currentEEPose.qy = ms->config.straightDown.qy;
      ms->config.currentEEPose.qz = ms->config.straightDown.qz;
      ms->config.currentEEPose.qw = ms->config.straightDown.qw;
      ms->config.currentEEPose.px = oldestX;
      ms->config.currentEEPose.py = oldestY;
      cout << "This pose was accepted by ikClient:" << endl;
      cout << "Next EE Position (x,y,z): " << nextEEPose.px << " " << nextEEPose.py << " " << nextEEPose.pz << endl;
      cout << "Next EE Orientation (x,y,z,w): " << nextEEPose.qx << " " << nextEEPose.qy << " " << nextEEPose.qz << " " << nextEEPose.qw << endl;
      ms->pushWord("waitUntilAtCurrentPosition");
      cout << "moveToNextMapPosition tries foundGoodPosition oldestI oldestJ oldestX oldestY: "  << tries << " " << foundGoodPosition << " "  << oldestI << " " << oldestJ << " " << oldestX << " " << oldestY << endl;
      break;
    } else {
      cout << "moveToNextMapPosition tries foundGoodPosition oldestI oldestJ: "  << tries << " " << foundGoodPosition << " "  << oldestI << " " << oldestJ << " " << oldestX << " " << oldestY << endl;
      cout << "Try number try: " << tries << ", adding point to ikMap oldestI oldestJ ikMap[.]: " << " " << oldestI << " " << oldestJ;
      if (ikResultFailed) {
	ms->config.ikMap[oldestI + ms->config.mapWidth * oldestJ] = IK_FAILED;
      } else {
	if (likelyInCollision) {
	  ms->config.ikMap[oldestI + ms->config.mapWidth * oldestJ] = IK_LIKELY_IN_COLLISION;
	} else {
	  ms->config.ikMap[oldestI + ms->config.mapWidth * oldestJ] = IK_GOOD;
	}
      }
      cout << " " << ms->config.ikMap[oldestI + ms->config.mapWidth * oldestJ] << endl;
    }
  }

  // puts it back at the right height for scanning in
  //  case coming from exotic pose
  ms->pushWord("mappingPatrolA");
  ms->pushWord("sampleHeight");
}
END_WORD
REGISTER_WORD(MoveToNextMapPosition)



WORD(AssumeBest3dGrasp)
virtual void execute(MachineState * ms) {
  REQUIRE_FOCUSED_CLASS(ms,tfc);

  double p_backoffDistance = ms->config.graspBackoffDistance;

  vector<int> feasible_indeces;
  for (int _tc = 0; _tc < ms->config.class3dGrasps[ms->config.targetClass].size(); _tc++) {
    feasible_indeces.push_back(1);
  }

  cout << "assumeBest3dGrasp: trying with a backoff distance of " << p_backoffDistance << endl;

  for (int _tc = 0; _tc < ms->config.class3dGrasps[ms->config.targetClass].size(); _tc++) {

    double max_score = -DBL_MAX;
    int max_index = -1;
    for (int mc = 0; mc < ms->config.class3dGrasps[ms->config.targetClass].size(); mc++) {
      if (feasible_indeces[mc] == 1) {

	Grasp *thisGrasp = &(ms->config.class3dGrasps[tfc][mc]);

	int is_maxed_out = 0;
	{
	  if (ms->config.currentPickMode == LEARNING_SAMPLING) {
	    is_maxed_out = ( (thisGrasp->tries >= ms->config.graspLearningMaxTries) );
	  } else if (ms->config.currentPickMode == LEARNING_ALGORITHMC) {
	    // ATTN 20
	    double successes = thisGrasp->successes;
	    double failures = thisGrasp->tries - thisGrasp->successes;

	    successes = round(successes);
	    failures = round(failures);

	    double result = cephes_incbet(successes + 1, failures + 1, ms->config.algorithmCTarget);
	    is_maxed_out = (result > ms->config.algorithmCRT);
	  } else if (ms->config.currentPickMode == STATIC_MARGINALS) {
	    //is_maxed_out = (ms->config.graspMemoryTries[i] <= 1);
	  }
	}
	if (is_maxed_out) {
	  feasible_indeces[mc] = 0;
	  continue;
	} else {
	}

	double thisScore = thisGrasp->successes / thisGrasp->tries;
	if (thisScore > max_score) {
	  max_index = mc;
	  max_score = thisScore;
	} else {
	}
      } else {
	cout << "assumeBest3dGrasp: skipping infeasible in list... " << mc << endl;
      }
    }

    int tc = max_index;

    if ( (tc > -1) && (tc < ms->config.class3dGrasps[ms->config.targetClass].size()) ) {
    } else {
      break;
    }




    eePose toApply = ms->config.class3dGrasps[ms->config.targetClass][tc].grasp_pose;  

    eePose thisBase = ms->config.lastLockedPose;
    thisBase.pz = -ms->config.currentTableZ;

    cout << "assumeAny3dGrasp, tc: " << tc << endl;

    // this order is important because quaternion multiplication is not commutative
    //ms->config.currentEEPose = ms->config.currentEEPose.plusP(ms->config.currentEEPose.applyQTo(toApply));
    //ms->config.currentEEPose = ms->config.currentEEPose.multQ(toApply);
    eePose graspPose = toApply.applyAsRelativePoseTo(thisBase);

    int increments = floor(p_backoffDistance / GRID_COARSE); 
    Vector3d localUnitX;
    Vector3d localUnitY;
    Vector3d localUnitZ;
    fillLocalUnitBasis(graspPose, &localUnitX, &localUnitY, &localUnitZ);
    eePose retractedGraspPose = graspPose.minusP(p_backoffDistance * localUnitZ);

    int ikResultPassedBoth = 1;
    {
      cout << "Checking IK for 3D grasp number " << tc << ", "; 
      int ikCallResult = 0;
      baxter_core_msgs::SolvePositionIK thisIkRequest;
      eePose toRequest = graspPose;
      fillIkRequest(toRequest, &thisIkRequest);
      queryIK(ms, &ikCallResult, &thisIkRequest);

      cout << ikCallResult << "." << endl;

      int ikResultFailed = 1;
      if (ikCallResult && thisIkRequest.response.isValid[0]) {
	ikResultFailed = 0;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].position.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough positions." << endl;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].name.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough names." << endl;
      } else if (thisIkRequest.response.joints.size() == 1) {
	if( ms->config.usePotentiallyCollidingIK ) {
	  cout << "WARNING: using ik even though result was invalid under presumption of false collision..." << endl;
	  cout << "Received enough positions and names for ikPose: " << thisIkRequest.request.pose_stamp[0].pose << endl;
	  ikResultFailed = 0;
	} else {
	  ikResultFailed = 1;
	  cout << "ik result was reported as colliding and we are sensibly rejecting it..." << endl;
	}
      } else {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, incorrect joint field." << endl;
      }

      ikResultPassedBoth = ikResultPassedBoth && (!ikResultFailed);
    }
    {
      cout << "Checking IK for 3D pre-grasp number " << tc << ", ";
      int ikCallResult = 0;
      baxter_core_msgs::SolvePositionIK thisIkRequest;
      eePose toRequest = retractedGraspPose;
      fillIkRequest(toRequest, &thisIkRequest);
      queryIK(ms, &ikCallResult, &thisIkRequest);

      cout << ikCallResult << "." << endl;

      int ikResultFailed = 1;
      if (ikCallResult && thisIkRequest.response.isValid[0]) {
	ikResultFailed = 0;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].position.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough positions." << endl;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].name.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough names." << endl;
      } else if (thisIkRequest.response.joints.size() == 1) {
	if( ms->config.usePotentiallyCollidingIK ) {
	  cout << "WARNING: using ik even though result was invalid under presumption of false collision..." << endl;
	  cout << "Received enough positions and names for ikPose: " << thisIkRequest.request.pose_stamp[0].pose << endl;
	  ikResultFailed = 0;
	} else {
	  ikResultFailed = 1;
	  cout << "ik result was reported as colliding and we are sensibly rejecting it..." << endl;
	}
      } else {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, incorrect joint field." << endl;
      }

      ikResultPassedBoth = ikResultPassedBoth && (!ikResultFailed);
    }

    if (ikResultPassedBoth) {
      ms->config.current3dGraspIndex = tc;

      cout << "Grasp and pre-grasp both passed, accepting." << endl;

      ms->config.currentEEPose = retractedGraspPose;
      ms->config.lastPickPose = graspPose;
      int tbb = ms->config.targetBlueBox;
      if (tbb < ms->config.blueBoxMemories.size()) {
	ms->config.blueBoxMemories[tbb].pickedPose = ms->config.lastPickPose;  
      } else {
	assert(0);
      }

      if (ms->config.snapToFlushGrasp) {
	// using twist and effort
	ms->pushWord("closeGripper");
	ms->pushWord("pressUntilEffortCombo");

	ms->pushWord("setEffortThresh");
	ms->pushWord("7.0");

	ms->pushWord("setSpeed");
	ms->pushWord("0.03");

	ms->pushWord("pressUntilEffortInit");
	ms->pushWord("comeToStop");
	ms->pushWord("setMovementStateToMoving");
	ms->pushWord("comeToStop");
	ms->pushWord("waitUntilAtCurrentPosition");

	ms->pushWord("setSpeed");
	ms->pushWord("0.05");

	ms->pushWord("setGridSizeCoarse");
      } else {
	ms->pushWord("comeToStop"); 
	ms->pushWord("waitUntilAtCurrentPosition"); 

	ms->pushCopies("localZUp", increments);
	ms->pushWord("setGridSizeCoarse");
	ms->pushWord("approachSpeed");
      }

      ms->pushWord("waitUntilAtCurrentPosition"); 
      return;
    } else {
      feasible_indeces[tc] = 0;
    }
  }

  cout << "assumeBest3dGrasp: No 3D grasps were feasible. Proceeding to take any grasp, even during learning!" << endl;
  
  feasible_indeces.resize(0);
  for (int _tc = 0; _tc < ms->config.class3dGrasps[ms->config.targetClass].size(); _tc++) {
    feasible_indeces.push_back(1);
  }

  for (int _tc = 0; _tc < ms->config.class3dGrasps[ms->config.targetClass].size(); _tc++) {

    double max_score = -DBL_MAX;
    int max_index = -1;
    for (int mc = 0; mc < ms->config.class3dGrasps[ms->config.targetClass].size(); mc++) {
      if (feasible_indeces[mc] == 1) {

	Grasp *thisGrasp = &(ms->config.class3dGrasps[tfc][mc]);

	double thisScore = thisGrasp->successes / thisGrasp->tries;
	if (thisScore > max_score) {
	  max_index = mc;
	  max_score = thisScore;
	} else {
	}
      } else {
	cout << "assumeBest3dGrasp PASS 2: skipping infeasible in list... " << mc << endl;
      }
    }

    int tc = max_index;

    if ( (tc > -1) && (tc < ms->config.class3dGrasps[ms->config.targetClass].size()) ) {
    } else {
      break;
    }

    eePose toApply = ms->config.class3dGrasps[ms->config.targetClass][tc].grasp_pose;  

    eePose thisBase = ms->config.lastLockedPose;
    thisBase.pz = -ms->config.currentTableZ;

    cout << "assumeAny3dGrasp PASS 2, tc: " << tc << endl;

    // this order is important because quaternion multiplication is not commutative
    //ms->config.currentEEPose = ms->config.currentEEPose.plusP(ms->config.currentEEPose.applyQTo(toApply));
    //ms->config.currentEEPose = ms->config.currentEEPose.multQ(toApply);
    eePose graspPose = toApply.applyAsRelativePoseTo(thisBase);

    int increments = floor(p_backoffDistance / GRID_COARSE); 
    Vector3d localUnitX;
    Vector3d localUnitY;
    Vector3d localUnitZ;
    fillLocalUnitBasis(graspPose, &localUnitX, &localUnitY, &localUnitZ);
    eePose retractedGraspPose = graspPose.minusP(p_backoffDistance * localUnitZ);

    int ikResultPassedBoth = 1;
    {
      cout << "Checking IK for 3D grasp number " << tc << ", "; 
      int ikCallResult = 0;
      baxter_core_msgs::SolvePositionIK thisIkRequest;
      eePose toRequest = graspPose;
      fillIkRequest(toRequest, &thisIkRequest);
      queryIK(ms, &ikCallResult, &thisIkRequest);

      cout << ikCallResult << "." << endl;

      int ikResultFailed = 1;
      if (ikCallResult && thisIkRequest.response.isValid[0]) {
	ikResultFailed = 0;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].position.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough positions." << endl;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].name.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough names." << endl;
      } else if (thisIkRequest.response.joints.size() == 1) {
	if( ms->config.usePotentiallyCollidingIK ) {
	  cout << "WARNING: using ik even though result was invalid under presumption of false collision..." << endl;
	  cout << "Received enough positions and names for ikPose: " << thisIkRequest.request.pose_stamp[0].pose << endl;
	  ikResultFailed = 0;
	} else {
	  ikResultFailed = 1;
	  cout << "ik result was reported as colliding and we are sensibly rejecting it..." << endl;
	}
      } else {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, incorrect joint field." << endl;
      }

      ikResultPassedBoth = ikResultPassedBoth && (!ikResultFailed);
    }
    {
      cout << "Checking IK for 3D pre-grasp number " << tc << ", ";
      int ikCallResult = 0;
      baxter_core_msgs::SolvePositionIK thisIkRequest;
      eePose toRequest = retractedGraspPose;
      fillIkRequest(toRequest, &thisIkRequest);
      queryIK(ms, &ikCallResult, &thisIkRequest);

      cout << ikCallResult << "." << endl;

      int ikResultFailed = 1;
      if (ikCallResult && thisIkRequest.response.isValid[0]) {
	ikResultFailed = 0;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].position.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough positions." << endl;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].name.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough names." << endl;
      } else if (thisIkRequest.response.joints.size() == 1) {
	if( ms->config.usePotentiallyCollidingIK ) {
	  cout << "WARNING: using ik even though result was invalid under presumption of false collision..." << endl;
	  cout << "Received enough positions and names for ikPose: " << thisIkRequest.request.pose_stamp[0].pose << endl;
	  ikResultFailed = 0;
	} else {
	  ikResultFailed = 1;
	  cout << "ik result was reported as colliding and we are sensibly rejecting it..." << endl;
	}
      } else {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, incorrect joint field." << endl;
      }

      ikResultPassedBoth = ikResultPassedBoth && (!ikResultFailed);
    }

    if (ikResultPassedBoth) {
      ms->config.current3dGraspIndex = tc;

      cout << "Grasp and pre-grasp both passed, accepting." << endl;

      ms->config.currentEEPose = retractedGraspPose;
      ms->config.lastPickPose = graspPose;
      int tbb = ms->config.targetBlueBox;
      if (tbb < ms->config.blueBoxMemories.size()) {
	ms->config.blueBoxMemories[tbb].pickedPose = ms->config.lastPickPose;  
      } else {
	assert(0);
      }

      if (ms->config.snapToFlushGrasp) {
	// using twist and effort
	ms->pushWord("closeGripper");
	ms->pushWord("pressUntilEffortCombo");

	ms->pushWord("setEffortThresh");
	ms->pushWord("7.0");

	ms->pushWord("setSpeed");
	ms->pushWord("0.03");

	ms->pushWord("pressUntilEffortInit");
	ms->pushWord("comeToStop");
	ms->pushWord("setMovementStateToMoving");
	ms->pushWord("comeToStop");
	ms->pushWord("waitUntilAtCurrentPosition");

	ms->pushWord("setGridSizeCoarse");
      } else {
	ms->pushWord("comeToStop"); 
	ms->pushWord("waitUntilAtCurrentPosition"); 

	ms->pushCopies("localZUp", increments);
	ms->pushWord("setGridSizeCoarse");
	ms->pushWord("approachSpeed");
      }

      ms->pushWord("waitUntilAtCurrentPosition"); 
      ms->pushWord("quarterImpulse");
      return;
    } else {
      feasible_indeces[tc] = 0;
    }
  }

  cout << "assumeBest3dGrasp PASS 2: No 3D grasps were feasible after all. Proceeding without attempting a grasp." << endl;
}
END_WORD
REGISTER_WORD(AssumeBest3dGrasp)


  }
