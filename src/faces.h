
#ifndef _FACES_H_
#define _FACES_H_

#include <cv.h>
#include <ml.h>
#include <vector>

using namespace cv;
using namespace std;

vector<Rect> faceDetectAndDisplay(string windowName, Mat frame ) ;
#endif /* _FACES_H_ */
