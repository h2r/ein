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

vector<double> estimate_pos(Mat observed, Mat reconstructed) {
  double rot_max_val = 0;
  double rot_max_rot = 0;
  Point rot_max_loc;

  double width = min(observed.cols / 2, observed.rows / 2);

  /* Rect rect(width / 2, width / 2, width, width); */
  Point2f reconstructed_center(reconstructed.cols/2.0F, reconstructed.rows/2.0F);
  Mat roi_observed;
  roi_observed = observed;
  Mat max_rot;

#pragma omp parallel for
  for(int angle = 0; angle < 360; angle += 1) {
    Mat rot_reconstructed;
    Mat result;
    double min_val; double max_val; Point min_loc; Point max_loc;

    /* Deal with rotation */
    Mat rot_mat = getRotationMatrix2D(reconstructed_center, angle, 1.0);
    warpAffine(reconstructed, rot_reconstructed, rot_mat, reconstructed.size());

    Mat rot_reconstructed_old = rot_reconstructed;
    matchTemplate(rot_reconstructed, roi_observed, result, CV_TM_CCOEFF_NORMED);

    minMaxLoc(result, &min_val, &max_val, &min_loc, &max_loc, Mat());


    if (max_val > rot_max_val) {
      rot_max_val = max_val;
      rot_max_rot = angle;
      rot_max_loc = max_loc;
      max_rot = rot_reconstructed;
    }
  }

  vector<double> to_return(5);
  to_return.insert(to_return.begin(), rot_max_loc.x - width);
  to_return.insert(to_return.begin() + 1, rot_max_loc.x + width);
  to_return.insert(to_return.begin() + 2, rot_max_loc.y - width);
  to_return.insert(to_return.begin() + 3, rot_max_loc.y + width);
  to_return.insert(to_return.begin() + 4, rot_max_rot);

  return to_return;
}

namespace ein_words {
  WORD(initSlam)
    virtual void execute(MachineState * ms) {
      Mat observed;
      ms->config.scene->observed_map.get()->rgbMuToMat(observed);
      cvtColor(observed, ms->config.reconstructed, CV_YCrCb2BGR);
      imwrite("init.jpg", ms->config.reconstructed);
      /* waitKey(0); */
    }
  END_WORD
  REGISTER_WORD(initSlam)

  WORD(runSlam)
  virtual void execute(MachineState * ms) {
    Mat background = ms->config.reconstructed;
    Mat observed;
    ms->config.scene->observed_map.get()->rgbMuToMat(observed);
    cvtColor(observed, observed, CV_YCrCb2BGR);

    Rect crop = Rect(300, 250, 400, 400);

    vector<double> estimated = estimate_pos(observed(crop), background);

    cout << estimated[0] << ", " << estimated[1] << ", " << estimated[2] << ", " << estimated[3] << ", " << estimated[4] << endl;

    /* estimated[0] += crop.x; */
    /* estimated[1] += crop.x; */
    /* estimated[2] += crop.y; */
    /* estimated[3] += crop.y; */

    estimated[0] += 200;
    estimated[1] += 200;
    estimated[2] += 200;
    estimated[3] += 200;

    /* for(int i = 0; i < 4; i++) { */
    /*   estimated[i] += 300; */
    /* } */

    double x_pos = (estimated[0] + estimated[1]) / 2 - 500;
    double y_pos = (estimated[2] + estimated[3]) / 2 - 500;

    double x_global = 0.0;
    double y_global = 0.0;

    ms->config.scene->observed_map.get()->cellToMeters(x_pos + 500, y_pos + 500, &x_global, &y_global);

    cout << x_pos << ", " << y_pos << ", " << estimated[4] << endl;
    cout << x_global << ", " << y_global << ", " << estimated[4] << endl;

    Point2f center(background.cols/2.0F, background.rows/2.0F);
    Mat rot_mat = getRotationMatrix2D(center, estimated[4], 1.0);
    Mat rot_reconstructed;
    warpAffine(background, rot_reconstructed, rot_mat, background.size());

    /* imshow("observed", observed(crop)); */
    /* imshow("background", background(crop)); */
    /* waitKey(0); */

    cout << estimated[0] << ", " << estimated[1] << ", " << estimated[2] << ", " << estimated[3] << ", " << estimated[4] << endl;

    observed(crop).copyTo(rot_reconstructed.rowRange(estimated[2], estimated[3]).colRange(estimated[0], estimated[1]));

    Mat re_rot_mat = getRotationMatrix2D(center, -estimated[4], 1.0);
    warpAffine(rot_reconstructed, background, re_rot_mat, rot_reconstructed.size());

    imwrite("observed_" + std::to_string(ms->config.slamNumber) + ".jpg", observed(crop));
    imwrite("background_" + std::to_string(ms->config.slamNumber) + ".jpg", background);
    ms->config.slamNumber++;
    /* imshow("background", background); */
    /* waitKey(0); */

    bool needs_expansion = false;
    int border[] = {0, 0, 0, 0};

    if (y_pos < 400) {
      border[0] = 200;
      needs_expansion = true;
    }
    if (x_pos < 200) {
      border[2] = 200;
      needs_expansion = true;
    }
    if (y_pos > rot_reconstructed.cols - 400) {
      border[1] = 200;
      needs_expansion = true;
    }
    if (x_pos > rot_reconstructed.rows - 400) {
      border[3] = 200;
      needs_expansion = true;
    }

    cout << "###########################################################################################################################" << endl;
    ROS_INFO_STREAM(needs_expansion);
    ROS_INFO_STREAM(false);
    ROS_INFO_STREAM(true);
    cout << "###########################################################################################################################" << endl;

    if (needs_expansion) {
      /* imshow("before", background); */
      /* waitKey(0); */
      copyMakeBorder(background, ms->config.reconstructed, border[0], border[1], border[2], border[3], BORDER_CONSTANT, Scalar(0));
      /* imshow("after", ms->config.reconstructed); */
      /* waitKey(0); */
    }

    ms->pushWord("\"" + std::to_string(x_pos) + ", " + std::to_string(y_pos) + ", " + std::to_string(estimated[4]) + "\"");
  }
  END_WORD
  REGISTER_WORD(runSlam)

  WORD(slamWriteToLogFile)
    virtual void execute(MachineState * ms) {
    }
  END_WORD
  REGISTER_WORD(slamWriteToLogFile)
}
