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

  _eePose plusP(const Vector3d& a) const {
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

  _eePose minusP(const Vector3d& a) const {
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

  _eePose plusP(const _eePose& a) const {
    _eePose toReturn;
    toReturn.px = px + a.px;
    toReturn.py = py + a.py;
    toReturn.pz = pz + a.pz;
    toReturn.qx = qx;
    toReturn.qy = qy;
    toReturn.qz = qz;
    toReturn.qw = qw;
    return toReturn;
  }

  _eePose minusP(const _eePose& a) const {
    _eePose toReturn;
    toReturn.px = px - a.px;
    toReturn.py = py - a.py;
    toReturn.pz = pz - a.pz;
    toReturn.qx = qx;
    toReturn.qy = qy;
    toReturn.qz = qz;
    toReturn.qw = qw;
    return toReturn;
  }

  _eePose negativeP() const {
    _eePose toReturn;
    toReturn.px = -px;
    toReturn.py = -py;
    toReturn.pz = -pz;
    toReturn.qx = qx;
    toReturn.qy = qy;
    toReturn.qz = qz;
    toReturn.qw = qw;
    return toReturn;
  }

  _eePose negativeQ() const {
    _eePose toReturn;
    toReturn.px = px;
    toReturn.py = py;
    toReturn.pz = pz;
    toReturn.qx = -qx;
    toReturn.qy = -qy;
    toReturn.qz = -qz;
    toReturn.qw = -qw;
    return toReturn;
  }

  _eePose multQ(const _eePose& a) const {
    Quaternionf thisQ(qw, qx, qy, qz); 
    Quaternionf aQ(a.qw, a.qx, a.qy, a.qz); 
    Quaternionf oQ = thisQ * aQ; 
    
    _eePose toReturn;
    toReturn.px = px;
    toReturn.py = py;
    toReturn.pz = pz;
    toReturn.qx = oQ.x();
    toReturn.qy = oQ.y();
    toReturn.qz = oQ.z();
    toReturn.qw = oQ.w();
    return toReturn;
  }

  _eePose invQ() const {
    Quaternionf thisQ(qw, qx, qy, qz); 
    Quaternionf oQ = thisQ.inverse(); 
    
    _eePose toReturn;
    toReturn.px = px;
    toReturn.py = py;
    toReturn.pz = pz;
    toReturn.qx = oQ.x();
    toReturn.qy = oQ.y();
    toReturn.qz = oQ.z();
    toReturn.qw = oQ.w();
    return toReturn;
  }

  void writeToFileStorage(FileStorage& fsvO) const {
    fsvO << "{:";
    fsvO << "px" << px;
    fsvO << "py" << py;
    fsvO << "pz" << pz;
    fsvO << "qw" << qw;
    fsvO << "qx" << qx;
    fsvO << "qy" << qy;
    fsvO << "qz" << qz;
    fsvO << "}";
  }

  void readFromFileStorage(FileNodeIterator& it) {
    px = (double)(*it)["px"];
    py = (double)(*it)["py"];
    pz = (double)(*it)["pz"];
    qw = (double)(*it)["qw"];
    qx = (double)(*it)["qx"];
    qy = (double)(*it)["qy"];
    qz = (double)(*it)["qz"];
  }
} eePose;

void printEEPose(eePose toPrint);
double squareDistanceEEPose(eePose pose1, eePose pose2);
const eePose eePoseZero = {.px = 0.0, .py = 0.0, .pz = 0.0,
                           .qx = 0.0, .qy = 0.0, .qz = 0.0, .qw = 0.0};


eePose rectCentroidToEEPose(Rect rect);

#endif /* _EEPOSEH_ */
