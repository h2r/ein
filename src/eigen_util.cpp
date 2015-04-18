#include "eigen_util.h"

double unsignedQuaternionDistance(Quaternionf q1, Quaternionf q2) {
  float r = q1.dot(q2);
  if (r < -1.0 || r > 1.0) {
    return 0;
  }
  r = acos(r); 
  return r <= M_PI_2? r: M_PI-r;
}
