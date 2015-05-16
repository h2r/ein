#ifndef _EIN_UTIL_H_
#define _EIN_UTIL_H_

#include <string>
#include <iostream>
#include <assert.h>

#include <cv.h>
#include <highgui.h>
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


typedef enum {
  STATIC_PRIOR = 1,
  LEARNING_SAMPLING = 2,
  LEARNING_ALGORITHMC = 3,
  STATIC_MARGINALS = 4,
  MAPPING = 5
} pickMode;


std::string pickModeToString(pickMode mode);


#define ORIENTATIONS 180//12 
#define O_FILTER_WIDTH 25//25
#define O_FILTER_SPOON_HEAD_WIDTH 6 
#define O_FILTER_SPOON_SHAFT_WIDTH 2


typedef enum {
  NO_LOCK = 0,
  CENTROID_LOCK = 1,
  POSE_LOCK = 2,
  POSE_REPORTED = 3
} memoryLockType;

void pushSpeedSign(shared_ptr<MachineState> ms, double speed);
void guardedImshow(string name, Mat image, bool shouldIRender);
bool isSketchyMat(Mat sketchy);
eePose rosPoseToEEPose(geometry_msgs::Pose pose);

struct BoxMemory {
  cv::Point bTop;
  cv::Point bBot;
  eePose cameraPose;
  eePose aimedPose;
  eePose pickedPose;
  eePose top;
  eePose bot;
  eePose centroid;
  ros::Time cameraTime;
  int labeledClassIndex;
  memoryLockType lockStatus;
  double trZ;
};

typedef struct MapCell {
  ros::Time lastMappedTime;
  int detectedClass; // -1 means not denied
  double r, g, b;
  double pixelCount;
} MapCell;

gsl_matrix * boxMemoryToPolygon(BoxMemory b);
void initializeMachine(shared_ptr<MachineState> ms);

#endif /* _EIN_UTIL_H_ */
