#ifndef _CAMERA_H_
#define _CAMERA_H_



#include <string>
#include <memory>

#include <image_transport/image_transport.h>

using namespace std;

class MachineState;

class Camera {
 public:
  std::string image_topic = "/camera/rgb/image_raw"; 
  shared_ptr<image_transport::ImageTransport> it;
  image_transport::Subscriber image_sub;

  ros::Time lastImageCallbackReceived;
  ros::Time lastImageStamp;

  cv_bridge::CvImagePtr cv_ptr = NULL;
  cv::Mat cam_img;
  const int imRingBufferSize = 300;
  std::vector<Mat> imRingBuffer;
  std::vector<ros::Time> imRBTimes;
  int imRingBufferStart = 0;
  int imRingBufferEnd = 0;


  MachineState * ms;


  void deactivateSensorStreaming();
  void activateSensorStreaming();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  Camera(MachineState * ms, string topic);


  int getRingImageAtTime(ros::Time t, Mat& value, int drawSlack = 0, bool debug=false);
  int getRingRangeAtTime(ros::Time t, double &value, int drawSlack = 0);
  int getRingPoseAtTime(ros::Time t, geometry_msgs::Pose &value, int drawSlack = 0, bool debug=false);
  void setRingImageAtTime(ros::Time t, Mat& imToSet);
  void imRingBufferAdvance();

};


#endif /* _CAMERA_H_ */
