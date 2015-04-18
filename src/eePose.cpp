#include "eePose.h"
#include <iostream>

using namespace std;

void printEEPose(eePose toPrint) {
  cout << toPrint.px << " " << toPrint.py << " " << toPrint.pz << " " << endl << toPrint.qx << " " << toPrint.qy << " " << toPrint.qz << " " << toPrint.qw << endl;
}

double squareDistanceEEPose(eePose pose1, eePose pose2) {
  double dx = (pose1.px - pose2.px);
  double dy = (pose1.py - pose2.py);
  double dz = (pose1.pz - pose2.pz);
  double squareDistance = dx*dx + dy*dy + dz*dz;

  return squareDistance;
}



