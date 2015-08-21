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


void pushGridSign(shared_ptr<MachineState> ms, double speed) {

  if (speed == NOW_THATS_COARSE) {
    ms->pushWord("setGridSizeNowThatsCoarse"); 
  } else if (speed == GRID_EVEN_COARSER) {
    ms->pushWord("setGridSizeEvenCoarser"); 
  } else  if (speed == GRID_COARSER) {
    ms->pushWord("setGridSizeCoarser"); 
  } else if (speed == GRID_COARSE) {
    ms->pushWord("setGridSizeCoarse"); 
  } else if (speed == GRID_MEDIUM) {
    ms->pushWord("setGridSizeMedium"); 
  } else if (speed == GRID_FINE) {
    ms->pushWord("setGridSizeFine"); 
  } else if (speed == GRID_VERY_FINE) {
    ms->pushWord("setGridSizeVeryFine"); 
  } else {
    ROS_ERROR_STREAM("Unknown speed: " << speed);
    assert(0);
  }

}



bool isSketchyMat(Mat sketchy) {
  return ( (sketchy.rows <= 1) || (sketchy.rows <= 1) );
}



gsl_matrix * boxMemoryToPolygon(BoxMemory b) {
  double min_x = b.top.px;
  double min_y = b.top.py;
  double max_x = b.bot.px;
  double max_y = b.bot.py;
  double width = max_x - min_x;
  double height = max_y - min_y;

  gsl_matrix *  polygon = gsl_matrix_alloc(2, 4);
  gsl_matrix_set(polygon, 0, 0, min_x);
  gsl_matrix_set(polygon, 1, 0, min_y);

  gsl_matrix_set(polygon, 0, 1, min_x + width);
  gsl_matrix_set(polygon, 1, 1, min_y);

  gsl_matrix_set(polygon, 0, 2, min_x + width);
  gsl_matrix_set(polygon, 1, 2, min_y + height);

  gsl_matrix_set(polygon, 0, 3, min_x);
  gsl_matrix_set(polygon, 1, 3, min_y + height);
  return polygon;
}


eePose rosPoseToEEPose(geometry_msgs::Pose pose) {
  eePose result;
  result.px = pose.position.x;
  result.py = pose.position.y;
  result.pz = pose.position.z;
  result.qx = pose.orientation.x;
  result.qy = pose.orientation.y;
  result.qz = pose.orientation.z;
  result.qw = pose.orientation.w;
  return result;
}



void initializeMachine(shared_ptr<MachineState> ms) {
  if (ms->config.currentRobotMode != PHYSICAL) {
    return;
  }
  
  ms->pushWord("zeroGOff"); 
  ms->pushWord("waitUntilEndpointCallbackReceived"); 

  ms->pushWord("guiCustom1"); 
  ms->pushWord("printState");
  ms->pushCopies("zUp", 15);
  int devInit = 1;
  if (devInit) {
    ms->pushWord("incrementTargetClass"); 
    ms->pushWord("synchronicServoTakeClosest");
  }
  ms->pushWord("silenceSonar");
  ms->pushWord("printWords");
  ms->pushWord("openGripper");
  ms->pushWord("calibrateGripper");
  ms->pushWord("shiftIntoGraspGear1"); 

  {
    ms->pushWord("fillClearanceMap"); 
    ms->pushWord("moveCropToProperValue"); 
    ms->pushWord("loadCalibration"); 
    ms->pushWord("loadIkMap"); 
    ms->pushWord("loadGripperMask"); 
  }

  ms->execute_stack = 1;
}



