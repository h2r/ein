#ifndef _EEPOSEH_
#define _EEPOSEH_

#include <cv.h>
#include <highgui.h>
#include <ml.h>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/gpu/gpu.hpp>
using namespace cv;

typedef struct {
  double px;
  double py;
  double pz;

  double qx;
  double qy;
  double qz;
  double qw;
} eePose;

void printEEPose(eePose toPrint);
double squareDistanceEEPose(eePose pose1, eePose pose2);
const eePose eePoseZero = {.px = 0.0, .py = 0.0, .pz = 0.0,
                           .qx = 0.0, .qy = 0.0, .qz = 0.0, .qw = 0.0};


eePose rectCentroidToEEPose(Rect rect);

#endif /* _EEPOSEH_ */
