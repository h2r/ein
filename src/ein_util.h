#ifndef _EIN_UTIL_H_
#define _EIN_UTIL_H_

#include <string>
#include <iostream>
#include <assert.h>

#include <cv.h>
#include <ml.h>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <geometry_msgs/Pose.h>

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include "eePose.h"

#include "word.h"

#include "config.h"

using namespace std;
using namespace cv;

#define MY_FONT FONT_HERSHEY_SIMPLEX


#define RANGE_UPPER_INVALID 0.3
#define RANGE_LOWER_INVALID 0.08




std::string operationStatusToString(operationStatusType mode) ;


typedef enum {
  UNIFORM_PRIOR,
  ANALYTIC_PRIOR
} priorType;

typedef enum {
  MRT,
  SPOON,
  KNIFE,
  OFT_INVALID
} orientedFilterType;






#define ORIENTATIONS 180//12 
#define O_FILTER_WIDTH 25//25
#define O_FILTER_SPOON_HEAD_WIDTH 6 
#define O_FILTER_SPOON_SHAFT_WIDTH 2


void pushSpeedSign(shared_ptr<MachineState> ms, double speed);
bool isSketchyMat(Mat sketchy);
eePose rosPoseToEEPose(geometry_msgs::Pose pose);

gsl_matrix * boxMemoryToPolygon(BoxMemory b);
void initializeMachine(shared_ptr<MachineState> ms);


#endif /* _EIN_UTIL_H_ */
