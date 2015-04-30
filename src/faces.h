
#ifndef _FACES_H_
#define _FACES_H_

#include <cv.h>
#include <highgui.h>
#include <ml.h>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/gpu/gpu.hpp>

using namespace cv;

vector<Rect> faceDetectAndDisplay(string windowName, Mat frame ) ;
#endif /* _FACES_H_ */
