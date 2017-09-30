#include "faces.h"
#include "ein_util.h"
#include "eePose.h"
#include <ros/console.h>


string face_cascade_name = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";
string eyes_cascade_name = "/usr/share/opencv/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
int initialized = 0;
RNG rng(12345);

vector<Rect> faceDetectAndDisplay(string windowName, Mat frame ) {
  if (!initialized) {
    cout << "Initializing faces." << face_cascade_name << endl;
    if (!face_cascade.load(face_cascade_name)) {
      ROS_ERROR_STREAM("Error loading " << face_cascade_name); 
    }
    if (!eyes_cascade.load(eyes_cascade_name)) { 
      ROS_ERROR_STREAM("Error loading " << eyes_cascade_name);  
    }
    initialized = 1;
  }


  std::vector<Rect> faces;
  Mat frame_gray = frame.clone();

  cvtColor( frame, frame_gray, CV_BGR2GRAY );
  equalizeHist( frame_gray, frame_gray );

  //-- Detect faces
  face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

  for( size_t i = 0; i < faces.size(); i++ )
    {
      Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
      ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

      Mat faceROI = frame_gray( faces[i] );
      std::vector<Rect> eyes;

      //-- In each face, detect eyes
      eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, Size(30, 30) );

      for( size_t j = 0; j < eyes.size(); j++ )
        {
          Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
          int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
          circle( frame, center, radius, Scalar( 255, 0, 0 ), 4, 8, 0 );
        }
    }
  //-- Show what you got
  return faces;
}



