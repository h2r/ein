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
    state << endl;
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
    state <<
      "eePose = {.px = " << setw(w) << config.trueEEPose.position.x << ", .py = " << setw(w) << config.trueEEPose.position.y << ", .pz = " << setw(w) << config.trueEEPose.position.z << "," << endl <<
      "          .qx = " << setw(w) << config.trueEEPose.orientation.x << ", .qy = " << setw(w) << config.trueEEPose.orientation.y << ", .qz = " << setw(w) << config.trueEEPose.orientation.z << ", .qw = " << setw(w) << config.trueEEPose.orientation.w << "};" << endl;
    state << "currentThompsonHeightIdx: " << config.currentThompsonHeightIdx << endl;
    state << "mostRecentUntabledZ (remember this is inverted but correct): " << config.mostRecentUntabledZ << endl;
    state << "currentPickMode: " << pickModeToString(config.currentPickMode) << endl;
    state << "currentBoundingBoxMode: " << pickModeToString(config.currentBoundingBoxMode) << endl;
    state << "gradientServoTakeClosest: " << config.gradientTakeClosest << endl;
    state << "synchronicTakeClosest: " << config.synchronicTakeClosest << endl;
    state << "focusedClass: " << config.focusedClass;
    if (config.focusedClass != -1) {
      state << " " << config.classLabels[config.focusedClass];
    }
    state << endl;
    
    state << "targetClass: " << config.targetClass;
    if (config.targetClass != -1) {
      state << " " << config.classLabels[config.targetClass];
    }

    state << endl;
    double wrenchNorm = sqrt( eePose::squareDistance(eePose::zero(), config.trueEEWrench) );
    state << "wrenchNorm: " << setw(w) << wrenchNorm << endl;
    
    return state.str();

  }
