#include <iostream>
#include <stdio.h>
#include <random>
#include <math.h>
#include <chrono>
#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "ein_words.h"
#include "ein.h"

using namespace cv;
using namespace std;

namespace ein_words {
  WORD(testSlam)
  virtual void execute(MachineState * ms) {

  }
  END_WORD
  REGISTER_WORD(testSlam)
}
