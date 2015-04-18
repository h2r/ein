#include "ein_util.h"
#include <ros/console.h>

std::string operationStatusToString(operationStatusType mode) 
{
    string result;
  if (mode == UNKNOWN) {
    result = "Unknown";
  } else if (mode == FAILURE) {
    result = "Failure";
  } else if (mode == SUCCESS) {
    result = "Success";
  } else {
    cout << "Invalid operation status: " << mode << endl;
    assert(0);
  }
  return result;
}




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



void pushSpeedSign(shared_ptr<MachineState> ms, double speed) {

  if (speed == NOW_THATS_FAST) {
    ms->pushWord("setMovementSpeedNowThatsFast"); 
  } else if (speed == MOVE_EVEN_FASTER) {
    ms->pushWord("setMovementSpeedMoveEvenFaster"); 
  } else  if (speed == MOVE_FASTER) {
    ms->pushWord("setMovementSpeedMoveFaster"); 
  } else if (speed == MOVE_FAST) {
    ms->pushWord("setMovementSpeedMoveFast"); 
  } else if (speed == MOVE_MEDIUM) {
    ms->pushWord("setMovementSpeedMoveMedium"); 
  } else if (speed == MOVE_SLOW) {
    ms->pushWord("setMovementSpeedMoveSlow"); 
  } else if (speed == MOVE_VERY_SLOW) {
    ms->pushWord("setMovementSpeedMoveVerySlow"); 
  } else {
    ROS_ERROR_STREAM("Unknown speed: " << speed);
    assert(0);
  }

}
