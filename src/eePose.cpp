#include "eePose.h"
#include <iostream>

using namespace std;

void printEEPose(eePose toPrint) {
  cout << toPrint.px << " " << toPrint.py << " " << toPrint.pz << " " << endl << toPrint.qx << " " << toPrint.qy << " " << toPrint.qz << " " << toPrint.qw << endl;
}
