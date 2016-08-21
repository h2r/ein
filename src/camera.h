#ifndef _CAMERA_H_
#define _CAMERA_H_



#include <string>
#include <memory>

#include <image_transport/image_transport.h>

using namespace std;

class MachineState;

class Camera {
 public:
  string image_topic = "/camera/rgb/image_raw"; 
  string tf_ee_link;
  string tf_camera_link;
  shared_ptr<image_transport::ImageTransport> it;
  image_transport::Subscriber image_sub;

  ros::Time lastImageCallbackReceived;
  ros::Time lastImageStamp;

  cv::Mat cam_img;
  cv::Mat cam_bgr_img;
  cv::Mat cam_ycrcb_img;
  const int imRingBufferSize = 300;
  std::vector<Mat> imRingBuffer;
  std::vector<ros::Time> imRBTimes;
  int imRingBufferStart = 0;
  int imRingBufferEnd = 0;
  std::vector<streamImage> streamImageBuffer;

  eePose handCameraOffset = {0.03815,0.01144,0.01589, 0,0,0,1};
  eePose truePose;


  Mat gripperMaskFirstContrast;
  Mat gripperMaskSecondContrast;
  Mat gripperMaskMean;
  Mat gripperMaskSquares;
  int gripperMaskCounts;
  Mat gripperMask;
  Mat cumulativeGripperMask;
  double gripperMaskThresh = 0.02;

  string name;
  string calibrationDirectory;
  string gripperMaskFilename;
  string calibrationFilename;

  cameraCalibrationMode currentCameraCalibrationMode = CAMCAL_HYPERBOLIC;

  eePose cropUpperLeftCorner = eePose(320, 200, 0.0,
                                      0.0, 1.0, 0.0, 0.0); // center of image



  eePose centerReticle = eePose(325, 127, 0.0,
                                0.0, 0.0, 0.0, 0.0);

  eePose defaultReticle = centerReticle;

  eePose probeReticle = defaultReticle;
  eePose reticle = defaultReticle;


  eePose vanishingPointReticle = defaultReticle;
  eePose heightReticles[4];
  constexpr static double cReticleIndexDelta = .01;
  const static int numCReticleIndexes = 14;
  constexpr static double firstCReticleIndexDepth = .08;
  int xCR[numCReticleIndexes];
  int yCR[numCReticleIndexes];
  double fEpsilon = 1.0e-9;

  int curseReticleX = 0;
  int curseReticleY = 0;

  // these corrective magnification factors should be close to 1
  //  these are set elsewhere according to chirality
  double m_x = 1.08;
  double m_y = 0.94;
  double m_x_h[4];
  double m_y_h[4];
  double m_XQ[3] = {0,0,0};
  double m_YQ[3] = {0,0,0};

  double transform_matrix[4] = {1, 0, 0, 1};

  Eigen::Quaternionf gear0offset;

  ros::Time lastCameraLogTime;
  bool observedCameraFlip;
  bool observedCameraMirror;
  int observedCameraExposure = -1;
  int observedCameraGain = -1;
  int observedCameraWhiteBalanceRed = -1;
  int observedCameraWhiteBalanceGreen = -1;
  int observedCameraWhiteBalanceBlue = -1;
  int observedCameraWindowX = -1;
  int observedCameraWindowY = -1;

  int cameraExposure = -1;
  int cameraGain = -1;
  int cameraWhiteBalanceRed = -1;
  int cameraWhiteBalanceGreen = -1;
  int cameraWhiteBalanceBlue = -1;



  MachineState * ms;


  void deactivateSensorStreaming();
  void activateSensorStreaming();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  Camera(MachineState * ms, string name, string topic, string tf_ee_link, string tf_camera_link);


  int getRingImageAtTime(ros::Time t, Mat& value, int drawSlack = 0, bool debug=false);
  int getRingRangeAtTime(ros::Time t, double &value, int drawSlack = 0);
  int getRingPoseAtTime(ros::Time t, geometry_msgs::Pose &value, int drawSlack = 0, bool debug=false);
  void setRingImageAtTime(ros::Time t, Mat& imToSet);
  void imRingBufferAdvance();

  int sibCurIdx = 0;

  streamImage * setIsbIdx(int idx);
  streamImage * setIsbIdxNoLoad(int idx);
  streamImage * setIsbIdxYesLoadNoKick(int idx);
  streamImage * setIsbIdxNoLoadNoKick(int idx);
  streamImage * getIsbIdxNoLoadNoKick(int idx);
  void resetAccumulatedStreamImage();
  void populateStreamImageBuffer();
  void clearStreamBuffer();
  void streamImageAsClass(Mat im, int classToStreamIdx, double now);

  void loadCalibration(string inFileName);
  void loadCalibration();
  void saveCalibration(string outFileName);
  void saveCalibration();

  void loadGripperMask(string inFileName);
  void loadGripperMask();
  void saveGripperMask(string outFileName);
  void saveGripperMask();

  void setHandCameraOffsetFromTf(ros::Time time);
  void setDefaultHandCameraOffset();

  void updateTrueCameraPoseFromTf(ros::Time time);
  void updateTrueCameraPoseWithHandCameraOffset(ros::Time time);



};


#endif /* _CAMERA_H_ */
