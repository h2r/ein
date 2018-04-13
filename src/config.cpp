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

    int w = 5;
    state << fixed;
    state << setfill(' ');
    state << "CI: " << setw(40) << std::left;
    if (current_instruction != NULL) {
      state << current_instruction->name();
    } else {
      state << "NULL";
    }
    state.precision(0);
    state << "Hz: " << config.aveFrequencyRange << endl;
    state.precision(3);
    state << "Target EE (x,y,z): "  << setw(w) << config.currentEEPose.px << " " << setw(w) << config.currentEEPose.py << " " << setw(w) << config.currentEEPose.pz << endl;
    state << "Target EE (x,y,z,w): " << setw(w) << config.currentEEPose.qx << " " << setw(w) << config.currentEEPose.qy << " " << setw(w) << config.currentEEPose.qz << " " << setw(w) << config.currentEEPose.qw << endl;
    state << "  True EE (x,y,z): " <<  setw(w) << config.trueEEPose.position.x << " " << setw(w) << config.trueEEPose.position.y << " " << setw(w) << config.trueEEPose.position.z << endl;
    state << "  True EE (x,y,z,w): "  << setw(w) << config.trueEEPose.orientation.x << " " << setw(w) << config.trueEEPose.orientation.y << " " << setw(w) << config.trueEEPose.orientation.z << " " << setw(w) << config.trueEEPose.orientation.w << endl;

    double poseError = eePose::distance(config.trueEEPoseEEPose, config.currentEEPose);
    double orError = eePose::distanceQ(config.trueEEPoseEEPose, config.currentEEPose);
    eePose difference = config.trueEEPoseEEPose.minusP(config.currentEEPose);
    state << "pose error: "  << poseError << ", " << orError << endl;

    //state << "position error (x,y,z): "  << setw(w) << difference.px << " " << setw(w) << difference.py << " " << setw(w) << difference.pz << endl;
    //state << "orientation error (x,y,z,w): " << setw(w) << difference.qx << " " << setw(w) << difference.qy << " " << setw(w) << difference.qz << " " << setw(w) << difference.qw << endl;

    //state << "currentThompsonHeightIdx: " << config.currentThompsonHeightIdx << endl;
    //state << "mostRecentUntabledZ (remember this is inverted but correct): " << config.mostRecentUntabledZ << endl;
    //state << "currentPickMode: " << pickModeToString(config.currentPickMode) << endl;
    //state << "currentBoundingBoxMode: " << pickModeToString(config.currentBoundingBoxMode) << endl;
    //state << "gradientServoTakeClosest: " << config.gradientTakeClosest << endl;
    //state << "synchronicTakeClosest: " << config.synchronicTakeClosest << endl;
    state << "ikMode: " << ikModeToString(config.currentIKMode) << endl;
    state << "zeroGMode: " << config.zero_g_toggle << endl;
    state << "focusedClass: " << config.focusedClass;
    if (config.focusedClass != -1 && config.classLabels.size() != 0) {
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

    //state << "averagedWrechNorm: " << setw(w) << thisAveWN << endl;

    
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


void Grasp::writeToFileStorage(FileStorage& fsvO) const {
  fsvO << "{:";
  fsvO << "px" << grasp_pose.px;
  fsvO << "py" << grasp_pose.py;
  fsvO << "pz" << grasp_pose.pz;
  fsvO << "qw" << grasp_pose.qw;
  fsvO << "qx" << grasp_pose.qx;
  fsvO << "qy" << grasp_pose.qy;
  fsvO << "qz" << grasp_pose.qz;
  fsvO << "tries" << tries;
  fsvO << "successes" << successes;
  fsvO << "failures" << failures;
  fsvO << "jams" << jams;
  fsvO << "}";
}

void Grasp::readFromFileNodeIterator(FileNodeIterator& it) {
  FileNode node = *it;
  readFromFileNode(node);
}

void Grasp::readFromFileNode(FileNode& it) {
  grasp_pose.px = (double)(it)["px"];
  grasp_pose.py = (double)(it)["py"];
  grasp_pose.pz = (double)(it)["pz"];
  grasp_pose.qw = (double)(it)["qw"];
  grasp_pose.qx = (double)(it)["qx"];
  grasp_pose.qy = (double)(it)["qy"];
  grasp_pose.qz = (double)(it)["qz"];
  tries = (double)(it)["tries"];
  successes = (double)(it)["successes"];
  failures = (double)(it)["failures"];
  jams = (double)(it)["jams"];
}

MachineState::MachineState() {
  this->ms = this;
  this->nil = make_shared<CompoundWord>();
}

ostream & operator<<(ostream & os, const Grasp& toPrint)
{
  FileStorage st;
  st.open("tmp.yml", FileStorage::WRITE | FileStorage::MEMORY);
  st << "GraspO"; 
  toPrint.writeToFileStorage(st);
  string result = st.releaseAndGetString();
  os << result.substr(10, result.size());
  return os;
} 


/*const double EinConfig::hrmDelta;
const double EinConfig::rmDelta;
const int EinConfig::pfmWidth;
const double EinConfig::minHeight;
const double EinConfig::maxHeight;
const double EinConfig::cReticleIndexDelta;
const double EinConfig::firstCReticleIndexDepth;
const double EinConfig::commonFreq;
const double EinConfig::vaDelta;
const double EinConfig::grayBlur;
const double EinConfig::mapXMin;
const double EinConfig::mapXMax;
const double EinConfig::mapYMin;
const double EinConfig::mapYMax;
const double EinConfig::mapStep;

*/
