#ifndef _EEPOSEH_
#define _EEPOSEH_

#include "eigen_util.h"
#include <cv.h>
#include <highgui.h>
#include <ml.h>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/gpu/gpu.hpp>
using namespace cv;

typedef struct _eePose{
  double px;
  double py;
  double pz;

  double qx;
  double qy;
  double qz;
  double qw;

  _eePose operator+(const Vector3d& a) const {
    _eePose toReturn;
    toReturn.px = px + a.x();
    toReturn.py = py + a.y();
    toReturn.pz = pz + a.z();
    toReturn.qx = qx;
    toReturn.qy = qy;
    toReturn.qz = qz;
    toReturn.qw = qw;
    return toReturn;
  }

  _eePose operator-(const Vector3d& a) const {
    _eePose toReturn;
    toReturn.px = px - a.x();
    toReturn.py = py - a.y();
    toReturn.pz = pz - a.z();
    toReturn.qx = qx;
    toReturn.qy = qy;
    toReturn.qz = qz;
    toReturn.qw = qw;
    return toReturn;
  }

} eePose;

void printEEPose(eePose toPrint);
double squareDistanceEEPose(eePose pose1, eePose pose2);
const eePose eePoseZero = {.px = 0.0, .py = 0.0, .pz = 0.0,
                           .qx = 0.0, .qy = 0.0, .qz = 0.0, .qw = 0.0};


eePose rectCentroidToEEPose(Rect rect);

#endif /* _EEPOSEH_ */
