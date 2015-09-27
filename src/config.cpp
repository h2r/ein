#include "ein.h"
#include "config.h"

string pickModeToString(pickMode mode) {
  string result;
  if (mode == STATIC_PRIOR) {
    result = "static prior";
  } else if (mode == LEARNING_SAMPLING) {
    result = "learning sampling";
  } else if (mode == LEARNING_ALGORITHMC) {
    result = "learning algorithm C";
  } else if (mode == STATIC_MARGINALS) {
    result = "static marginals";
  } else if (mode == MAPPING) {
    result = "mapping";
  } else {
    cout << "Invalid pick mode: " << mode << endl;
    assert(0);
  }
  return result;
}

string MachineState::currentState()
 {
    stringstream state;
    state.precision(7);
    int w = 10;
    state << fixed;
    state << setfill(' ');
    state << "CI: ";
    if (current_instruction != NULL) {
      state << current_instruction->name();
    } else {
      state << "NULL";
    }
    state << endl;
    state << "Current EE Position (x,y,z): "  << setw(w) << config.currentEEPose.px << " " << setw(w) << config.currentEEPose.py << " " << setw(w) << config.currentEEPose.pz << endl;
    state << "Current EE Orientation (x,y,z,w): " << setw(w) << config.currentEEPose.qx << " " << setw(w) << config.currentEEPose.qy << " " << setw(w) << config.currentEEPose.qz << " " << setw(w) << config.currentEEPose.qw << endl;
    state << "True EE Position (x,y,z): " <<  setw(w) << config.trueEEPose.position.x << " " << setw(w) << config.trueEEPose.position.y << " " << setw(w) << config.trueEEPose.position.z << endl;
    state << "True EE Orientation (x,y,z,w): "  << setw(w) << config.trueEEPose.orientation.x << " " << setw(w) << config.trueEEPose.orientation.y << " " << setw(w) << config.trueEEPose.orientation.z << " " << setw(w) << config.trueEEPose.orientation.w << endl;

    double poseError = eePose::distance(config.trueEEPoseEEPose, config.currentEEPose);
    eePose difference = config.trueEEPoseEEPose.minusP(config.currentEEPose);
    state << "position error distance: "  << poseError << endl;

    state << "position error (x,y,z): "  << setw(w) << difference.px << " " << setw(w) << difference.py << " " << setw(w) << difference.pz << endl;
    state << "position error (x,y,z,w): " << setw(w) << difference.qx << " " << setw(w) << difference.qy << " " << setw(w) << difference.qz << " " << setw(w) << difference.qw << endl;

    //state << "currentThompsonHeightIdx: " << config.currentThompsonHeightIdx << endl;
    //state << "mostRecentUntabledZ (remember this is inverted but correct): " << config.mostRecentUntabledZ << endl;
    //state << "currentPickMode: " << pickModeToString(config.currentPickMode) << endl;
    //state << "currentBoundingBoxMode: " << pickModeToString(config.currentBoundingBoxMode) << endl;
    //state << "gradientServoTakeClosest: " << config.gradientTakeClosest << endl;
    //state << "synchronicTakeClosest: " << config.synchronicTakeClosest << endl;
    state << "ikMode: " << ikModeToString(config.currentIKMode) << endl;
    state << "focusedClass: " << config.focusedClass;
    if (config.focusedClass != -1) {
      state << " " << config.classLabels[config.focusedClass];
    }
    state << endl;
    
    //state << "targetClass: " << config.targetClass;
    //if (config.targetClass != -1) {
    //state << " " << config.classLabels[config.targetClass];
    //}
    //state << endl;

    double wrenchNorm = sqrt( eePose::squareDistance(eePose::zero(), config.trueEEWrench) );
    state << "wrenchNorm: " << setw(w) << wrenchNorm << endl;

	if (config.averagedWrechMass < EPSILON) {
      config.averagedWrechMass = 1.0;
	} else {
	}

	double thisAveWN = config.averagedWrechAcc / config.averagedWrechMass;

    state << "averagedWrechNorm: " << setw(w) << thisAveWN << endl;

    
    //state << "collision: " << config.collisionStateBuffer.front().inCollision <<  " buffer: " << config.numCollisions() << "/" << config.collisionStateBuffer.size() << endl;

    return state.str();

  }


string ikModeToString(ikMode mode) {
  if (mode == IKSERVICE) {
    return "ikservice";
  } else if (mode == IKFAST) {
    return "ikfast";
  } else if (mode == IKFASTDEBUG) {
    return "ikfastdebug";
  } else {
    cout << "Unknown IK Mode: " << mode << endl;
    assert(0);
  }
}
