#include "eigen_util.h"


double unsignedQuaternionDistance(Quaternionf q1, Quaternionf q2) {
  float r = q1.dot(q2);
  if (r < -1.0 || r > 1.0) {
    return 0;
  }
  r = acos(r); 
  return r <= M_PI_2? r: M_PI-r;
}


eePose eePosePlus(const eePose& in, const Vector3d& a) {
  eePose toReturn;
  toReturn.px = in.px + a.x();
  toReturn.py = in.py + a.y();
  toReturn.pz = in.pz + a.z();
  toReturn.qx = in.qx;
  toReturn.qy = in.qy;
  toReturn.qz = in.qz;
  toReturn.qw = in.qw;
  return toReturn;
}

eePose eePoseMinus(const eePose& in, const Vector3d& a) {
  eePose toReturn;
  toReturn.px = in.px - a.x();
  toReturn.py = in.py - a.y();
  toReturn.pz = in.pz - a.z();
  toReturn.qx = in.qx;
  toReturn.qy = in.qy;
  toReturn.qz = in.qz;
  toReturn.qw = in.qw;
  return toReturn;
}
