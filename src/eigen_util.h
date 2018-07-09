#ifndef _EIGEN_UTIL_H_
#define _EIGEN_UTIL_H_

#include <Eigen/Geometry> 
#include "eePose.h"
using namespace Eigen;

double unsignedQuaternionDistance(Quaternionf q1, Quaternionf q2);

eePose eePosePlus(const eePose&, const Vector3d& a);
eePose eePoseMinus(const eePose&, const Vector3d& a);

#endif // _EIGEN_UTIL_H_
