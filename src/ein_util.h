#ifndef _EIN_UTIL_H_
#define _EIN_UTIL_H_

#include <string>
#include <iostream>
#include <assert.h>
using namespace std;

#define NOW_THATS_FAST 0.08
#define MOVE_EVEN_FASTER 0.04
#define MOVE_FASTER 0.02
#define MOVE_FAST 0.01
#define MOVE_MEDIUM 0.005 //.005
#define MOVE_SLOW 0.0025
#define MOVE_VERY_SLOW 0.00125

#define RANGE_UPPER_INVALID 0.3
#define RANGE_LOWER_INVALID 0.08



typedef enum {
  ARMED,
  BLOCKED,
  STOPPED,
  HOVERING,
  MOVING
} movementState;


typedef enum {
  UNKNOWN = -1,
  FAILURE = 0,
  SUCCESS = 1
} operationStatusType;

std::string operationStatusToString(operationStatusType mode) ;

typedef enum {
  SIFTBOW_GLOBALCOLOR_HIST = 1,
  OPPONENTSIFTBOW_GLOBALCOLOR_HIST = 2, // this has not been sufficiently tested
  SIFTCOLORBOW_HIST = 3, // unimplemented, calculate color histograms at each keypoint and augment each SIFT feature before clustering
  GRADIENT = 4,
  OPPONENT_COLOR_GRADIENT = 5,
  CBCR_HISTOGRAM = 6
} featureType;


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
  PHYSICAL,
  SIMULATED
} robotMode;

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



#endif /* _EIN_UTIL_H_ */
