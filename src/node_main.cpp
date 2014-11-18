#ifndef PROGRAM_NAME
  #define PROGRAM_NAME "DAVE"
#endif
//// these macros below were moved to other files
//#define RUN_INFERENCE // generates the blue boxes
//#define PUBLISH_OBJECTS // requires blue, brown, and gray boxes, and inference
//#define RELOAD_DATA
//#define RELEARN_VOCAB
//#define LEARN_ONLY
//// these macros above were moved to other files

#define RUN_TRACKING // calculates the red boxes

#define DRAW_BLUE_KEYPOINTS
#define DRAW_RED_KEYPOINTS

int drawOrientor = 1;
int drawLabels = 1;
int drawPurple = 0;
int drawWhite = 0;
int drawGreen = 1;
int drawBlue = 1;
int drawRed = 1;
int drawRB = 0;
int drawGray = 1;
int drawPink = 0;
int drawBrown = 1;
int drawBlueKP = 1;
int drawRedKP = 1;

//#define DEBUG

int mask_gripper = 0;

int add_blinders = 0;
int blinder_stride = 10;
int blinder_columns = 5;

#include <signal.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
boost::mutex ros_mutex;
boost::mutex pcl_mutex;
boost::mutex redbox_mutex;
boost::thread *timercallback1_thread;
int ikResult = 1;

#include <tf/transform_listener.h>
tf::TransformListener* tfListener;
double tfPast = 10.0;

#include <ctime>
#include <sstream>
#include <iostream>
#include <math.h>
#include <string>
std::string left_or_right_arm = "";
std::string densityViewerName = "Density Viewer";
std::string objectViewerName = "Object Viewer";

typedef struct {
  double px;
  double py;
  double pz;

  double ox;
  double oy;
  double oz;

  double qx;
  double qy;
  double qz;
  double qw;
} eePose;

eePose beeLHome = {.px = 0.657579481614, .py = 0.851981417433, .pz = 0.0388352386502,
		   .ox = 0.0, .oy = 0.0, .oz = 0.0,
		   .qx = -0.366894936773, .qy = 0.885980397775, .qz = 0.108155782462, .qw = 0.262162481772};
eePose beeRHome = {.px = 0.657579481614, .py = -0.168019, .pz = 0.0388352386502,
		   .ox = 0.0, .oy = 0.0, .oz = 0.0,
		   .qx = -0.366894936773, .qy = 0.885980397775, .qz = 0.108155782462, .qw = 0.262162481772};

eePose beeHome = beeRHome;


int loTrackbarVariable = 55;
int hiTrackbarVariable = 50;
int redTrackbarVariable = 0;

double drawBingProb = .1;

// for objectness
double canny_hi_thresh = 7;
double canny_lo_thresh = 4;
// for sobel
//double canny_hi_thresh = 10;
//double canny_lo_thresh = 0.5;

double sobel_sigma = 1.0;
double sobel_scale_factor = 1e-12;
double local_sobel_sigma = 1.0;

double aveTime = 0.0;
double aveFrequency = 0.0;
double timeMass = 0.0;
double timeInterval = 15;
time_t thisTime = 0;
time_t firstTime = 0;

double densityDecay = 0.3;//0.7;
double depthDecay = 0.7;
double redDecay = 0.9;

// point cloud affine extrinsic calibration
/*
// this is for the uncalibrated point cloud
double pcbcX = 5;
double pcbcY = -25;
double pcgc11 = 1.0+(50.0 / 640.0);
double pcgc12 = 0.0;
double pcgc21 = 0.0;
double pcgc22 = 1.0+(12.0 / 480.0);
*/
// this is for the calibrated point cloud
double pcbcX = 10;//5;
double pcbcY = 0;//-25;
double pcgc11 = 1;//1.0+(50.0 / 640.0);
double pcgc12 = 0.0;
double pcgc21 = 0.0;
double pcgc22 = 1;//1.0+(12.0 / 480.0);

// if you add an immense number of examples or some new classes and
//   you begin having discriminative problems (confusion), you can
//   increase the number of words.
// XXX
const double bowSubSampleFactor = 0.1;
const int bowOverSampleFactor = 1;
const int kNNOverSampleFactor = 1;
const int poseOverSampleFactor = 1;

const int keypointPeriod = 1;
const double kpGreenThresh = 0;
//const double kpProb = 0.1;
const double kpProb = 1.0;

const int vocabNumWords = 1000;
const double grayBlur = 1.0;

const int k = 4;
int redK = 1;

int retrain_vocab = 0;
int reextract_knn = 0;
int rewrite_labels = 0;


// paramaters for the color histogram feature
const double colorHistNumBins = 8;
const double colorHistBinWidth = 256/colorHistNumBins;
// XXX
const double colorHistLambda = 0.5;
const double colorHistThresh = 0.1;
const int colorHistBoxHalfWidth = 1;

#include <baxter_core_msgs/EndpointState.h>
#include <sensor_msgs/Range.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/JointCommand.h>

#include <dirent.h>

#include "ros/ros.h"
#include "ros/package.h"
#include "ros/time.h"

#include "std_msgs/String.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "object_recognition_msgs/RecognizedObjectArray.h"
#include "object_recognition_msgs/RecognizedObject.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//#include <cv.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/nonfree/nonfree.hpp>
//#include <opencv2/opencv.hpp>

#include "../../bing/Objectness/stdafx.h"
#include "../../bing/Objectness/Objectness.h"
#include "../../bing/Objectness/ValStructVec.h"
#include "../../bing/Objectness/CmShow.h"

Eigen::Vector3d eeForward;
geometry_msgs::Pose trueEEPose;

// you can mine hard negatives by modifying the box saving routine 
//   to save examples not belonging to class C while only showint it
//   class C objects
//#define SAVE_ANNOTATED_BOXES
// always increment the prefix so that the next person doesn't overwrite
//   your examples.  
std::string data_directory = "unspecified_dd";
std::string vocab_file = "unspecified_vf";
std::string knn_file = "unspecified_kf";
std::string label_file = "unspecified_lf";

std::string run_prefix = "unspecified_rp";
std::string class_name = "unspecified_cn";

std::string class_labels= "unspecified_cl1 unspecified_cl2";
std::string class_pose_models = "unspecified_pm1 unspecified_pm2";

std::string red_box_list = "";

std::string image_topic = "/camera/rgb/image_raw"; // "/filter_time/filtered_image"
std::string pc_topic = "/camera/depth_registered/points";

std::string cache_prefix = "";

vector<string> redBoxLabels;
vector<string> classLabels; 
vector<string> classPoseModels;
vector<CvKNearest*> classPosekNNs;
vector<Mat> classPosekNNfeatures;
vector<Mat> classPosekNNlabels;
vector< vector< cv::Vec<double,4> > > classQuaternions;

DescriptorMatcher *matcher;
FeatureDetector *detector;
DescriptorExtractor *extractor;
BOWKMeansTrainer *bowtrainer; 
BOWImgDescriptorExtractor *bowExtractor;
CvKNearest *kNN;
std::string package_path;
std::string class_crops_path;
std::string bing_trained_models_path;
std::string objectness_matrix_path;
std::string objectness_path_prefix;
std::string saved_crops_path;

cv::Mat cam_img;
cv::Mat depth_img;
ros::Publisher rec_objs_blue;
ros::Publisher rec_objs_red;
ros::Publisher markers_blue;
ros::Publisher markers_red;

ros::Publisher ee_target_pub;

bool real_img = false;

Objectness *glObjectness;
int fc = 1;
int fcRange = 10;
int frames_per_click = 5;
int cropCounter;

double *temporalDensity = NULL;
double *temporalDepth = NULL;

pcl::PointCloud<pcl::PointXYZRGB> pointCloud;

#define MY_FONT FONT_HERSHEY_PLAIN

#define ORIENTATIONS 180//12 
#define O_FILTER_WIDTH 25//25
#define O_FILTER_SPOON_HEAD_WIDTH 6 
#define O_FILTER_SPOON_SHAFT_WIDTH 2
Mat *orientedFilters;
int biggestL1 = 0;
int oSearchWidth = 5;


// Top variables are top left corners of bounding boxes (smallest coordinates)
// Bot variables are bottom right corners of bounding boxes (largenst coordinates)
// Cen cariables are centers of bounding boxes

// white boxes come from ork. in the future they can lay down
//  green or red boxes to require 'accounting for'.
cv::vector<cv::Point> wTop;
cv::vector<cv::Point> wBot;

// objectness proposed bounding boxes
cv::vector<cv::Point> nTop;
cv::vector<cv::Point> nBot;

// bounding boxes of connected components of green matter,
//  they are the candidate blue boxes
vector<cv::Point> cTops; 
vector<cv::Point> cBots;

// create the blue boxes from the parental green boxes
vector<cv::Point> bTops; 
vector<cv::Point> bBots;
vector<cv::Point> bCens;
vector< vector<KeyPoint> > bKeypoints;
vector< vector<int> > bWords;
vector<Mat> bYCrCb;
vector<int> bLabels;

// adjust these to reject blue boxes
double rejectScale = 2.0;
double rejectAreaScale = 16;//6*6;


typedef struct {
  int classLabel;
  cv::Point top;
  cv::Point bot;
  int rootBlueBox;
  int numGreenBoxes;
  cv::Point anchor;
  double persistence;
  double lastDistance;
  double poseIndex;
  int winningO;

  eePose com; // center of mass
} redBox;

redBox *redBoxes;
int numRedBoxes = 0;
double persistenceThresh = 0.5;
int max_red_proposals = 1000;
double slidesPerFrame = 0.2;
int rbMinWidth = 50;
int rbMaxWidth = 200;

int redRounds = 10;
int redStride = 5;
int redPeriod = 4;
int redDigitsWSRN = 6; 

int redInitialWidth = 50;

// the brownBox
cv::Point brTop;
cv::Point brBot;
Eigen::Vector3d tablePositionSum;
Eigen::Vector3d tableNormalSum;
Eigen::Vector3d tableTangent1Sum;
Eigen::Vector3d tableTangent2Sum;
double tableBiasSum;
Eigen::Vector3d tableNormal;
Eigen::Vector3d tableTangent1;
Eigen::Vector3d tableTangent2;
Eigen::Vector3d tablePosition;
double tableBias;
double tableBiasMargin = -5.001;
//double tableBiasMargin = .01;
geometry_msgs::Pose tablePose;
Eigen::Quaternionf tableQuaternion;
Mat tablePerspective;
Eigen::Quaternionf tableLabelQuaternion;
string table_label_class_name = "";
string background_class_name = "";
int invertQuaternionLabel = 0;
string invert_sign_name = "";


double *gBoxIndicator;
int gBoxW = 10;
int gBoxH = 10;
//int gBoxThreshMultiplier = 1.1;
double gBoxThresh = 30000;//5;//3;
//double threshFraction = 0;//0.35;
double threshFraction = 0.2;

int gBoxStrideX;
int gBoxStrideY;


// pink box thresholds for the principle classes
double *pBoxIndicator;
double psPBT = 0.0;//5.0;
double wsPBT = 0.0;//6.5;
double gbPBT = 0.0;//6.0;
double mbPBT = 0.0;//7.0;

double pBoxThresh = 0;
double densityPower = 1.0;//1.0/4.0;

// gray box offset from the top and bottom of the screen
int tGO = 40;
int bGO = 140;
int lGO = 10;
int rGO = 10;
cv::Point grayTop;
cv::Point grayBot;

// all range mode switch and bounds
int all_range_mode = 0;
int tARM = 100;
int bARM = 100;
int lARM = 150;
int rARM = 150;
cv::Point armTop;
cv::Point armBot;

int rejectRedBox() {
  // check that the redBox has probable dimensions

  // check that the redBox is tight on green boxes 

}


void gridKeypoints(int gImW, int gImH, cv::Point top, cv::Point bot, int strideX, int strideY, vector<KeyPoint>& keypoints, int period) {
  keypoints.resize(0);

  // make sure feature pad is a multiple of the stride
  int featurePadX = strideX;
  int featurePadY = strideY;

  cv::Point sTop(featurePadX, featurePadY);
  cv::Point sBot(strideX*(((bot.x-top.x-featurePadX)/strideX)-2), strideY*(((bot.y-top.y-featurePadY)/strideY)-2));

  int responseIndex = 1;
  
  int mX = gBoxW / 2;
  int mY = gBoxH / 2;

#ifdef DEBUG
cout << sTop << sBot << endl;
#endif

  for (int y = sTop.y; y <= sBot.y; y+=strideX*period) {
    for (int x = sTop.x; x <= sBot.x; x+=strideY*period) {
      KeyPoint thisKeypoint;
      thisKeypoint.angle = -1;
      thisKeypoint.class_id = -1;
      thisKeypoint.octave = 0;
      thisKeypoint.pt.x = x+mX;
      thisKeypoint.pt.y = y+mY;
      thisKeypoint.response = responseIndex;
      thisKeypoint.size = min(gBoxW, gBoxH);
      responseIndex++;
      if (gImW > 0) {
	if ((pBoxIndicator[(top.y+y)*gImW+(top.x+x)] >= kpGreenThresh) && (drand48() < kpProb)) {
	  keypoints.push_back(thisKeypoint);
	}
      } else {
	if (drand48() < kpProb) {
	  keypoints.push_back(thisKeypoint);
	}
      }
      
      //if ((pBoxIndicator[(top.y+y)*gImW+(top.x+x)] >= kpGreenThresh))
	//keypoints.push_back(thisKeypoint);
    }
  }

}

void CallbackFunc(int event, int x, int y, int flags, void* userdata) {
  if ( event == EVENT_LBUTTONDOWN ) {
#ifdef CAPTURE_ONLY
    fc = frames_per_click;
#endif
#ifdef SAVE_ANNOTATED_BOXES
    fc = frames_per_click;
#endif
#ifdef CAPTURE_HARD_CLASS
    fc = frames_per_click;
#endif
 
    //cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if ( event == EVENT_RBUTTONDOWN ) {
    //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if  ( event == EVENT_MBUTTONDOWN ) {
    //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if ( event == EVENT_MOUSEMOVE ) {
    //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
  }
}

cv::Point pcCorrection(double x, double y, double imW, double imH) {
  cv::Point output;

  double bcX = pcbcX+x;
  double bcY = pcbcY+y;
  double bgcX = min(max(pcgc11*bcX + pcgc12*bcY,0.0), imW);
  double bgcY = min(max(pcgc21*bcX + pcgc22*bcY,0.0), imH);

  output.x = round(bgcX);
  output.y = round(bgcY);

  return output;
}

bool isFiniteNumber(double x) 
{
    return (x <= DBL_MAX && x >= -DBL_MAX); 
} 

void getPointCloudPoints(cv_bridge::CvImagePtr cv_ptr, vector<cv::Point>& pointCloudPoints, double *pBoxIndicator, double thisThresh, 
  cv::Point topIn, cv::Point botIn, int imW, int imH, int gBoxStrideX, int gBoxStrideY, int gBoxW, int gBoxH) {

  cv::Point top(gBoxStrideX*(topIn.x / gBoxStrideX), gBoxStrideY*(topIn.y / gBoxStrideY));
  cv::Point bot(gBoxStrideX*(botIn.x / gBoxStrideX), gBoxStrideY*(botIn.y / gBoxStrideY));
  for (int x = top.x; x <= bot.x-gBoxW; x+=gBoxStrideX) {
    for (int y = top.y; y <= bot.y-gBoxH; y+=gBoxStrideY) {
      int xt = x;
      int yt = y;
      int xb = x+gBoxW;
      int yb = y+gBoxH;
      cv::Point thisTop(xt,yt);
      cv::Point thisBot(xb,yb);

      int reject = 0;
      if (pBoxIndicator[y*imW+x] < thisThresh) {
	reject = 1;
      }

      // check to see if the point cloud point lies above the table
      if (reject == 0 && pointCloud.size() > 0) {
	cv::Point transformedPixel = pcCorrection( 
	  double(x)+(double(gBoxW)/2.0), double(y)+(double(gBoxH)/2.0), imW, imH);
	pcl::PointXYZRGB pcp = pointCloud.at(transformedPixel.x, transformedPixel.y);

	// this check also checks for nans in the points
	Eigen::Vector3d pinkPoint(pcp.x,pcp.y,pcp.z);
	if (all_range_mode) {
	  reject = 1;
	  if (pinkPoint.dot(tableNormal) >= -FLT_MAX)
	    reject = 0;
	} else {
	  reject = 1;
	  if (pinkPoint.dot(tableNormal) >= tableBias + tableBiasMargin)
	    reject = 0;
	}
      }

      if (!reject) {
	if (drawPink)
	  rectangle(cv_ptr->image, thisTop, thisBot, cv::Scalar(100,100,255));
	pointCloudPoints.push_back(cv::Point(x,y));
      }
    }
  }
}

void appendColorHist(Mat& yCrCb_image, vector<KeyPoint>& keypoints, Mat& descriptors, Mat& descriptors2) {
  Size sz = yCrCb_image.size();
  int imW = sz.width;
  int imH = sz.height;

  Mat colorHist(1, colorHistNumBins*colorHistNumBins, descriptors.type());
  for (int i = 0; i < colorHistNumBins*colorHistNumBins; i++) 
    colorHist.at<float>(i) = 0;

  double numPix = 0;
  // traverse all of the keypoints
  for (int kk = 0; kk < keypoints.size(); kk++) {
    // count pixels in a neighborhood of the keypoint
    int yMin = max(int(keypoints[kk].pt.y)-colorHistBoxHalfWidth,0);
    int yMax = min(int(keypoints[kk].pt.y)+colorHistBoxHalfWidth,imH);
    int xMin = max(int(keypoints[kk].pt.x)-colorHistBoxHalfWidth,0);
    int xMax = min(int(keypoints[kk].pt.x)+colorHistBoxHalfWidth,imW);
    for (int y = yMin; y < yMax; y++) {
      for (int x = xMin; x < xMax; x++) {

	cv::Vec3b thisColor = yCrCb_image.at<cv::Vec3b>(y,x);

	int CrBin = thisColor[1]/colorHistBinWidth;
	int CbBin = thisColor[2]/colorHistBinWidth;

	colorHist.at<float>(CrBin + CbBin*colorHistNumBins)++;

	numPix++;
      }
    }
  }
  // normalize the histogram
  for (int i = 0; i < colorHistNumBins*colorHistNumBins; i++) 
    colorHist.at<float>(i) = colorHist.at<float>(i) / numPix;

  descriptors2 = Mat(1, descriptors.size().width + colorHistNumBins*colorHistNumBins, descriptors.type());
  for (int i = 0; i < descriptors.size().width; i++) 
    descriptors2.at<float>(i) = descriptors.at<float>(i);
  for (int i = 0; i < colorHistNumBins*colorHistNumBins; i++) {
#ifdef DEBUG
//if ( colorHist.at<float>(i) > 0.1 )
  //cout << i << ":" << colorHist.at<float>(i) << " ";
#endif
    colorHist.at<float>(i) = min(colorHist.at<float>(i), float(colorHistThresh));
    descriptors2.at<float>(i+descriptors.size().width) = colorHistLambda * colorHist.at<float>(i);
  }
}

void processImage(Mat &image, Mat& gray_image, Mat& yCrCb_image, double sigma) {
  cvtColor(image, gray_image, CV_BGR2GRAY);
  cvtColor(image, yCrCb_image, CV_BGR2YCrCb);
  GaussianBlur(gray_image, gray_image, cv::Size(0,0), sigma);
  GaussianBlur(yCrCb_image, yCrCb_image, cv::Size(0,0), sigma);
}

void bowGetFeatures(std::string classDir, const char *className, double sigma) {

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  char buf[1024];
  sprintf(buf, "%s%s", classDir.c_str(), className);
  dpdf = opendir(buf);
  if (dpdf != NULL){
    while (epdf = readdir(dpdf)){
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

        vector<KeyPoint> keypoints1;
        vector<KeyPoint> keypoints2;
        Mat descriptors;

        char filename[1024];
        sprintf(filename, "%s%s/%s", classDir.c_str(), className, epdf->d_name);
        Mat image;
        image = imread(filename);
	Size sz = image.size();
	int cropW = sz.width;
	int cropH = sz.height;
	cv::Point bot(cropW, cropH);

        Mat gray_image;
        Mat yCrCb_image;
	processImage(image, gray_image, yCrCb_image, sigma);

	for (int i = 0; i < bowOverSampleFactor; i++) {
	  //detector->detect(gray_image, keypoints1);
	  gridKeypoints(0, 0, cv::Point(0,0), bot, gBoxStrideX, gBoxStrideY, keypoints1, keypointPeriod);
	  for (int kp = 0; kp < keypoints1.size(); kp++) {
	    if (drand48() < bowSubSampleFactor)
	      keypoints2.push_back(keypoints1[kp]);
	  }
	  extractor->compute(gray_image, keypoints2, descriptors);

	  cout << className << ":  "  << epdf->d_name << "  " << descriptors.size() << " " << endl;

	  if (!descriptors.empty() && !keypoints2.empty())
	    bowtrainer->add(descriptors);
	}
      }
    }
  }
}

void kNNGetFeatures(std::string classDir, const char *className, int label, double sigma, Mat &kNNfeatures, Mat &kNNlabels) {

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  char buf[1024];
  sprintf(buf, "%s%s", classDir.c_str(), className);
  dpdf = opendir(buf);
  if (dpdf != NULL){
    while (epdf = readdir(dpdf)){
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

        vector<KeyPoint> keypoints;
        Mat descriptors;
        Mat descriptors2;

        char filename[1024];
        sprintf(filename, "%s%s/%s", classDir.c_str(), className, epdf->d_name);
        Mat image;
        image = imread(filename);
	Size sz = image.size();
	int cropW = sz.width;
	int cropH = sz.height;
	cv::Point bot(cropW, cropH);

        Mat gray_image;
        Mat yCrCb_image;
	processImage(image, gray_image, yCrCb_image, sigma);

	for (int i = 0; i < kNNOverSampleFactor; i++) {
	  //detector->detect(gray_image, keypoints);
	  gridKeypoints(0, 0, cv::Point(0,0), bot, gBoxStrideX, gBoxStrideY, keypoints, keypointPeriod);
	  bowExtractor->compute(gray_image, keypoints, descriptors);

	  cout << className << ":  "  << epdf->d_name << "  " << descriptors.size() << " type: " << descriptors.type() << " tot: " << kNNfeatures.size() << endl;

	  if (!descriptors.empty() && !keypoints.empty()) {
	    appendColorHist(yCrCb_image, keypoints, descriptors, descriptors2);

	    kNNfeatures.push_back(descriptors2);
	    kNNlabels.push_back(label);
	  }
	}
      }
    }
  }
}

void posekNNGetFeatures(std::string classDir, const char *className, double sigma, Mat &kNNfeatures, Mat &kNNlabels,
  vector< cv::Vec<double,4> >& classQuaternions, int lIndexStart = 0) {

  string sClassName(className);

  int label = 0;

  int lIndex = lIndexStart;

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");
  string ppm(".ppm");
  

  char buf[1024];
  sprintf(buf, "%s%s", classDir.c_str(), className);
  dpdf = opendir(buf);
  if (dpdf != NULL){
    while (epdf = readdir(dpdf)){
      string fileName(epdf->d_name);
      cout << fileName << " " << endl;
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

	string fext = fileName.substr(fileName.size()-4, 4);
	if (fext.compare(ppm))
	  continue;

	//string poseIndex = fileName.substr(sClassName.size()+1, string::npos);
	//poseIndex = poseIndex.substr(0,  poseIndex.length()-4);
	//label = std::atoi(poseIndex.c_str());

	// remove .ppm to form key
	string thisCropLabel = fileName.substr(0,fileName.size()-4);
	string poseLabelsPath =  classDir + className + "/poseLabels.yml";

	cv::Vec<double,4> tLQ;

	FileStorage fsfI;
	fsfI.open(poseLabelsPath, FileStorage::READ);
	fsfI[thisCropLabel] >> tLQ; 
	fsfI.release();

        vector<KeyPoint> keypoints;
        Mat descriptors;
	Mat descriptors2;

        char filename[1024];
        sprintf(filename, "%s%s/%s", classDir.c_str(), className, epdf->d_name);
        Mat image;
        image = imread(filename);
	Size sz = image.size();
	int cropW = sz.width;
	int cropH = sz.height;
	cv::Point bot(cropW, cropH);
	
        Mat gray_image;
        Mat yCrCb_image;
	processImage(image, gray_image, yCrCb_image, sigma);

	for (int i = 0; i < poseOverSampleFactor; i++) {
	  //detector->detect(gray_image, keypoints);
	  gridKeypoints(0, 0, cv::Point(0,0), bot, gBoxStrideX, gBoxStrideY, keypoints, keypointPeriod);
	  bowExtractor->compute(gray_image, keypoints, descriptors);

	  cout << className << ":  "  << epdf->d_name << "  "  << fileName << " " << descriptors.size() << 
	    " type: " << descriptors.type() << " label: " << label << endl;

	  if (!descriptors.empty() && !keypoints.empty()) {
	    appendColorHist(yCrCb_image, keypoints, descriptors, descriptors2);
	    kNNfeatures.push_back(descriptors);
	    //kNNlabels.push_back(label);
	    kNNlabels.push_back(lIndex);
	    classQuaternions.push_back(tLQ);
	    lIndex++;
	  }
	}
      }
    }
  }
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg){
/*

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    depth_img = cv_ptr->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  Size sz = depth_img.size();
  int imW = sz.width;
  int imH = sz.height;

  if (temporalDepth == NULL) {
    temporalDepth= new double[imW*imH];
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	temporalDepth[y*imW + x] = 0;
      }
    }
  }

// /camera/depth_registered/points sensor_msgs:PointCloud2

  uint maxDepth = 0.0; 
  uint minDepth = 256*256-1;
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      uint tDepth = depth_img.at<unsigned short>(y,x);
      maxDepth = max(maxDepth, tDepth);
      if (tDepth > 0)
	minDepth = min(minDepth, tDepth);
#ifdef DEBUG
//cout << " " << float(depth_img.at<uint>(y,x));
#endif
    }
  }
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      uint tDepth = depth_img.at<unsigned short>(y,x);
      temporalDepth[y*imW+x] = depthDecay*temporalDepth[y*imW+x] + (1.0-depthDecay)*float(tDepth);
#ifdef DEBUG
//cout << " " << float(depth_img.at<uint>(y,x));
#endif
      if (tDepth <= minDepth)
	depth_img.at<short>(y,x) = 0;
      //else
	//depth_img.at<uint>(y,x) = tDepth - minDepth;
    }
  }

  maxDepth = max(maxDepth,uint(1));

  cv::imshow("Depth Viewer", depth_img*20);
#ifdef DEBUG
cout << depth_img.size() << " " << depth_img.type() << " " << maxDepth << " " << minDepth << endl;
#endif
*/
}

void getCluster(pcl::PointCloud<pcl::PointXYZRGB> &cluster, pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::vector<cv::Point> &points)
{
  cv::Point point;
  for (int i = 0; i < points.size(); i++)
  {
    point = points[i];
    pcl::PointXYZRGB pcp = cloud(point.x, point.y);
    if (isFiniteNumber(pcp.x) &&
	isFiniteNumber(pcp.y) &&
	isFiniteNumber(pcp.z) )
      cluster.push_back(pcp);
  }
}

geometry_msgs::Pose getPose(pcl::PointCloud<pcl::PointXYZRGB> &cluster)
{
  geometry_msgs::Pose pose;
  double sum_x = 0, sum_y = 0, sum_z = 0;
  for (int i = 0; i < cluster.size(); i++)
  {
    pcl::PointXYZRGB point = cluster[i];
    sum_x += point.x;
    sum_y += point.y;
    sum_z += point.z;
  }
  pose.position.x = sum_x / cluster.size();
  pose.position.y = sum_y / cluster.size();
  pose.position.z = sum_z / cluster.size();

  return pose;
}

// TODO probably don't need two separate functions for this
void loadROSParamsFromArgs()
{
  ros::NodeHandle nh("~");

  cout << "nh namespace: " << nh.getNamespace() << endl;

  nh.getParam("frames_per_click", frames_per_click);

  nh.getParam("vocab_file", vocab_file);
  nh.getParam("knn_file", knn_file);
  nh.getParam("label_file", label_file);

  nh.getParam("data_directory", data_directory);
  nh.getParam("class_labels", class_labels);
  nh.getParam("class_pose_models", class_pose_models);

  nh.getParam("class_name", class_name);
  nh.getParam("run_prefix", run_prefix);

  nh.getParam("all_range_mode", all_range_mode);
  nh.getParam("red_box_list", red_box_list);

  nh.getParam("gray_box_top", tGO);
  nh.getParam("gray_box_bot", bGO);
  nh.getParam("gray_box_left", lGO);
  nh.getParam("gray_box_right", rGO);

  nh.getParam("arm_box_top", tARM);
  nh.getParam("arm_box_bot", bARM);
  nh.getParam("arm_box_left", lARM);
  nh.getParam("arm_box_right", rARM);

  nh.getParam("image_topic", image_topic);
  nh.getParam("pc_topic", pc_topic);

  nh.getParam("invert_sign_name", invert_sign_name);

  nh.getParam("retrain_vocab", retrain_vocab);
  nh.getParam("reextract_knn", reextract_knn);
  nh.getParam("rewrite_labels", rewrite_labels);

  nh.getParam("cache_prefix", cache_prefix);

  nh.getParam("mask_gripper", mask_gripper);
  nh.getParam("add_blinders", add_blinders);

  nh.getParam("left_or_right_arm", left_or_right_arm);

  saved_crops_path = data_directory + "/" + class_name + "/";
}

void loadROSParams()
{
  ros::NodeHandle nh("~");

  nh.getParam("green_box_threshold", gBoxThresh);
  nh.getParam("pink_box_threshold", pBoxThresh);
  nh.getParam("threshold_fraction", threshFraction);
  nh.getParam("density_power", densityPower);
  nh.getParam("plastic_spoon_normalizer", psPBT);
  nh.getParam("wooden_spoon_normalizer", wsPBT);
  nh.getParam("gyrobowl_normalizer", gbPBT);
  nh.getParam("mixing_bowl_normalizer", mbPBT);
  nh.getParam("reject_scale", rejectScale);
  nh.getParam("reject_area_scale", rejectAreaScale);
  nh.getParam("frames_per_click", frames_per_click);
  nh.getParam("density_decay", densityDecay);
  nh.getParam("depth_decay", depthDecay);
  nh.getParam("red_decay", redDecay);
  nh.getParam("data_directory", data_directory);
  nh.getParam("class_labels", class_labels);
  nh.getParam("class_pose_models", class_pose_models);
  nh.getParam("class_name", class_name);
  nh.getParam("run_prefix", run_prefix);
  nh.getParam("all_range_mode", all_range_mode);
  nh.getParam("red_box_list", red_box_list);

  nh.getParam("gray_box_top", tGO);
  nh.getParam("gray_box_bot", bGO);
  nh.getParam("gray_box_left", lGO);
  nh.getParam("gray_box_right", rGO);

  nh.getParam("arm_box_top", tARM);
  nh.getParam("arm_box_bot", bARM);
  nh.getParam("arm_box_left", lARM);
  nh.getParam("arm_box_right", rARM);

  nh.getParam("image_topic", image_topic);
  nh.getParam("pc_topic", pc_topic);

  nh.getParam("table_label_class_name", table_label_class_name);
  nh.getParam("background_class_name", background_class_name);

  nh.getParam("orientation_search_width", oSearchWidth);

  nh.getParam("invert_sign_name", invert_sign_name);

  nh.getParam("retrain_vocab", retrain_vocab);
  nh.getParam("reextract_knn", reextract_knn);
  nh.getParam("rewrite_labels", rewrite_labels);

  nh.getParam("sobel_sigma", sobel_sigma);
  nh.getParam("local_sobel_sigma", local_sobel_sigma);
  nh.getParam("canny_hi_thresh",canny_hi_thresh);
  nh.getParam("canny_lo_thresh",canny_lo_thresh);
  nh.getParam("sobel_scale_factor",sobel_scale_factor);

  nh.getParam("mask_gripper", mask_gripper);
  nh.getParam("add_blinders", add_blinders);

  nh.getParam("left_or_right_arm", left_or_right_arm);

  saved_crops_path = data_directory + "/" + class_name + "/";
}

void saveROSParams()
{
  ros::NodeHandle nh("~");

  nh.setParam("green_box_threshold", gBoxThresh);
  nh.setParam("pink_box_threshold", pBoxThresh);
  nh.setParam("threshold_fraction", threshFraction);
  nh.setParam("density_power", densityPower);
  nh.setParam("plastic_spoon_normalizer", psPBT);
  nh.setParam("wooden_spoon_normalizer", wsPBT);
  nh.setParam("gyrobowl_normalizer", gbPBT);
  nh.setParam("mixing_bowl_normalizer", mbPBT);
  nh.setParam("reject_scale", rejectScale);
  nh.setParam("reject_area_scale", rejectAreaScale);
  nh.setParam("frames_per_click", frames_per_click);
  nh.setParam("density_decay", densityDecay);
  nh.setParam("depth_decay", depthDecay);
  nh.setParam("red_decay", redDecay);
  nh.setParam("data_directory", data_directory);
  nh.setParam("class_labels", class_labels);
  nh.setParam("class_pose_models", class_pose_models);
  nh.setParam("class_name", class_name);
  nh.setParam("run_prefix", run_prefix);
  nh.setParam("all_range_mode", all_range_mode);
  nh.setParam("red_box_list", red_box_list);

  nh.setParam("gray_box_top", tGO);
  nh.setParam("gray_box_bot", bGO);
  nh.setParam("gray_box_left", lGO);
  nh.setParam("gray_box_right", rGO);

  nh.setParam("arm_box_top", tARM);
  nh.setParam("arm_box_bot", bARM);
  nh.setParam("arm_box_left", lARM);
  nh.setParam("arm_box_right", rARM);

  nh.setParam("image_topic", image_topic);
  nh.setParam("pc_topic", pc_topic);

  nh.setParam("orientation_search_width", oSearchWidth);

  nh.setParam("invert_sign_name", invert_sign_name);

  nh.setParam("retrain_vocab", retrain_vocab);
  nh.setParam("reextract_knn", reextract_knn);
  nh.setParam("rewrite_labels", rewrite_labels);

  nh.setParam("sobel_sigma", sobel_sigma);
  nh.setParam("local_sobel_sigma", local_sobel_sigma);
  nh.setParam("canny_hi_thresh",canny_hi_thresh);
  nh.setParam("canny_lo_thresh",canny_lo_thresh);
  nh.setParam("sobel_scale_factor",sobel_scale_factor);

  nh.setParam("mask_gripper", mask_gripper);
  nh.setParam("add_blinders", add_blinders);

  nh.setParam("left_or_right_arm", left_or_right_arm);
}

// for publishing
void fill_RO_and_M_arrays(object_recognition_msgs::RecognizedObjectArray& roa_to_send, 
  visualization_msgs::MarkerArray& ma_to_send, vector<cv::Point>& pointCloudPoints, 
  int aI, int label, int winningO, int poseIndex) {

#ifdef DEBUG
cout << "check" << endl;
cout << "hit a publishable object " << label << " " << classLabels[label] 
<< " " << classPoseModels[label] << aI << " of total objects" << bTops.size() << endl;
#endif

  geometry_msgs::Pose object_pose;

  // XXX calculate orientation elsewhere
  cv::Matx33f R;
  R(0,0) = 1; R(0,1) = 0; R(0,2) = 0;
  R(1,0) = 0; R(1,1) = 1; R(1,2) = 0;
  R(2,0) = 0; R(2,1) = 0; R(2,2) = 1;

  // handle the rotation differently depending on the class
  // if we have a spoon
  if (0 == classPoseModels[label].compare("S")) {
    double theta = (M_PI / 2.0) + (winningO*2*M_PI/ORIENTATIONS);
    R(0,0) = cos(theta); R(0,1) = -sin(theta); R(0,2) = 0;
    R(1,0) = sin(theta); R(1,1) =  cos(theta); R(1,2) = 0;
    R(2,0) = 0;          R(2,1) = 0;           R(2,2) = 1;
  }

#ifdef DEBUG
cout << "constructing rotation matrix" << endl;
#endif

  Eigen::Matrix3f rotation;
  rotation << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2);
  Eigen::Quaternionf objectQuaternion(rotation);

  objectQuaternion = tableQuaternion * objectQuaternion;

  if (0 == classPoseModels[label].compare("G")) {

    cv::Vec<double,4> tLQ = classQuaternions[label][poseIndex];

    Eigen::Quaternionf thisLabelQuaternion;
    thisLabelQuaternion.x() = tLQ[0];
    thisLabelQuaternion.y() = tLQ[1];
    thisLabelQuaternion.z() = tLQ[2];
    thisLabelQuaternion.w() = tLQ[3];
    objectQuaternion = thisLabelQuaternion;

    //XXX objectQuaternion = classQuaternions[label][poseIndex];
    // TODO fix the quaternion saving so that we can save this table and perform this dereference
  }

  roa_to_send.objects[aI].pose.pose.pose.orientation.x = objectQuaternion.x();
  roa_to_send.objects[aI].pose.pose.pose.orientation.y = objectQuaternion.y();
  roa_to_send.objects[aI].pose.pose.pose.orientation.z = objectQuaternion.z();
  roa_to_send.objects[aI].pose.pose.pose.orientation.w = objectQuaternion.w();

#ifdef DEBUG
cout << "dealing with point cloud" << " of size " << pointCloud.size() << endl;
#endif

  // determine the x,y,z coordinates of the object from the point cloud
  // this bounding box has top left  bTops[x] and bBots[aI]
  if (pointCloud.size() > 0) {
    pcl::PointCloud<pcl::PointXYZRGB> object_cloud;
    getCluster(object_cloud, pointCloud, pointCloudPoints);
    geometry_msgs::Pose pose = getPose(object_cloud);
    roa_to_send.objects[aI].point_clouds.resize(1);
    pcl::toROSMsg(object_cloud, roa_to_send.objects[aI].point_clouds[0]);
    roa_to_send.objects[aI].pose.pose.pose.position = pose.position;
  } else {
    roa_to_send.objects[aI].pose.pose.pose.position = object_pose.position;
  }

  ma_to_send.markers[aI].pose = roa_to_send.objects[aI].pose.pose.pose;

  roa_to_send.header.stamp = ros::Time::now();
  roa_to_send.header.frame_id = "/camera_rgb_optical_frame";

  roa_to_send.objects[aI].header = roa_to_send.header;
  //roa_to_send.objects[aI].point_clouds[0].header = roa_to_send.header;
  roa_to_send.objects[aI].pose.header = roa_to_send.header;

  if (0 == classPoseModels[label].compare("B")) {
    ma_to_send.markers[aI].type =  visualization_msgs::Marker::SPHERE;
    ma_to_send.markers[aI].scale.x = 0.15;
    ma_to_send.markers[aI].scale.y = 0.15;
    ma_to_send.markers[aI].scale.z = 0.15;
    ma_to_send.markers[aI].color.a = 0.5;
    ma_to_send.markers[aI].color.r = 0.9;
    ma_to_send.markers[aI].color.g = 0.9;
    ma_to_send.markers[aI].color.b = 0.0;
  } else if (0 == classPoseModels[label].compare("S")) {
    ma_to_send.markers[aI].type =  visualization_msgs::Marker::CUBE;
    ma_to_send.markers[aI].scale.x = 0.2;
    ma_to_send.markers[aI].scale.y = 0.02;
    ma_to_send.markers[aI].scale.z = 0.02;
    ma_to_send.markers[aI].color.a = 0.50;
    ma_to_send.markers[aI].color.r = 0.25;
    ma_to_send.markers[aI].color.g = 0.25;
    ma_to_send.markers[aI].color.b = 0.25;
  } else {
    ma_to_send.markers[aI].type =  visualization_msgs::Marker::CUBE;
    ma_to_send.markers[aI].scale.x = 0.2;
    ma_to_send.markers[aI].scale.y = 0.02;
    ma_to_send.markers[aI].scale.z = 0.02;
    ma_to_send.markers[aI].color.a = 0.5;
    ma_to_send.markers[aI].color.r = 0.0;
    ma_to_send.markers[aI].color.g = 0.9;
    ma_to_send.markers[aI].color.b = 0.9;
  } 

  char labelName[256]; 
  /*
  if (label == 1) {
    sprintf(labelName, "gyroBowl");
    ma_to_send.markers[aI].type =  visualization_msgs::Marker::SPHERE;
    ma_to_send.markers[aI].scale.x = 0.15;
    ma_to_send.markers[aI].scale.y = 0.15;
    ma_to_send.markers[aI].scale.z = 0.15;
    ma_to_send.markers[aI].color.a = 1.0;
    ma_to_send.markers[aI].color.r = 0.9;
    ma_to_send.markers[aI].color.g = 0.9;
    ma_to_send.markers[aI].color.b = 0.0;

    ma_to_send.markers[aI].header =  roa_to_send.header;
    ma_to_send.markers[aI].action = visualization_msgs::Marker::ADD;
  }
  if (label == 2) {
    sprintf(labelName, "mixBowl");
    ma_to_send.markers[aI].type =  visualization_msgs::Marker::SPHERE;
    ma_to_send.markers[aI].scale.x = 0.17;
    ma_to_send.markers[aI].scale.y = 0.17;
    ma_to_send.markers[aI].scale.z = 0.17;
    ma_to_send.markers[aI].color.a = 1.0;
    ma_to_send.markers[aI].color.r = 0.8;
    ma_to_send.markers[aI].color.g = 0.8;
    ma_to_send.markers[aI].color.b = 0.8;

    ma_to_send.markers[aI].header =  roa_to_send.header;
    ma_to_send.markers[aI].action = visualization_msgs::Marker::ADD;
  }
  if (label == 3) {
    sprintf(labelName, "woodSpoon");
    ma_to_send.markers[aI].type =  visualization_msgs::Marker::CUBE;
    ma_to_send.markers[aI].scale.x = 0.2;
    ma_to_send.markers[aI].scale.y = 0.02;
    ma_to_send.markers[aI].scale.z = 0.02;
    ma_to_send.markers[aI].color.a = 1.0;
    ma_to_send.markers[aI].color.r = 0.80;
    ma_to_send.markers[aI].color.g = 0.80;
    ma_to_send.markers[aI].color.b = 0.50;

    ma_to_send.markers[aI].header =  roa_to_send.header;
    ma_to_send.markers[aI].action = visualization_msgs::Marker::ADD;
  }
  if (label == 4) {
    sprintf(labelName, "plasticSpoon");
    ma_to_send.markers[aI].type =  visualization_msgs::Marker::CUBE;
    ma_to_send.markers[aI].scale.x = 0.2;
    ma_to_send.markers[aI].scale.y = 0.02;
    ma_to_send.markers[aI].scale.z = 0.02;
    ma_to_send.markers[aI].color.a = 1.0;
    ma_to_send.markers[aI].color.r = 0.25;
    ma_to_send.markers[aI].color.g = 0.25;
    ma_to_send.markers[aI].color.b = 0.25;

    ma_to_send.markers[aI].header =  roa_to_send.header;
    ma_to_send.markers[aI].action = visualization_msgs::Marker::ADD;
  }
  */

  if (label == -1)
    sprintf(labelName, "VOID");
  else
    sprintf(labelName, "%s", classLabels[label].c_str());

  roa_to_send.objects[aI].type.key = labelName;


  ma_to_send.markers[aI].header =  roa_to_send.header;
  ma_to_send.markers[aI].action = visualization_msgs::Marker::ADD;
  ma_to_send.markers[aI].id = aI;
  ma_to_send.markers[aI].lifetime = ros::Duration(1.0);

#ifdef DEBUG
cout << "  finished a publishable object " << label << " " << classLabels[label] << " " << classPoseModels[label] << endl;
#endif
}

void getOrientation(cv_bridge::CvImagePtr cv_ptr, Mat& img_cvt, 
  vector<KeyPoint>& keypoints, Mat& descriptors, cv::Point top, cv::Point bot, 
  int label, string& labelName, string& augmentedLabelName, double& poseIndex, int& winningO) {

  if (label == -1)
    labelName = "VOID";
  else
    labelName = classLabels[label];

  augmentedLabelName = labelName;

  if (label >= 0) {
    if (0 == classPoseModels[label].compare("G")) {
      if (!descriptors.empty() && !keypoints.empty()) {
	poseIndex = classPosekNNs[label]->find_nearest(descriptors,k);
      }
    }

    if (0 == classPoseModels[label].compare("S")) {

#ifdef DEBUG
fprintf(stderr, " object checkS"); fflush(stderr);
cout << top << " " << bot << " "; cout.flush();
#endif

      int boxWidth  = bot.x-top.x;
      int boxHeight = bot.y-top.y;

      int xxs = max(0, top.x - oSearchWidth*gBoxStrideX);
      int xxf = min(gBoxStrideX*((img_cvt.size().width-1-boxWidth)/gBoxStrideX), top.x + oSearchWidth*gBoxStrideX);
      int yys = max(0, top.y - oSearchWidth*gBoxStrideY);
      int yyf = min(gBoxStrideY*((img_cvt.size().height-1-boxHeight)/gBoxStrideY), top.y + oSearchWidth*gBoxStrideY);

//xxs = top.x;
//xxf = top.x;
//yys = top.y;
//yyf = top.y;


      double winningScore = -1;
      int winningX = -1;
      int winningY = -1;
      for (int yy = yys; yy <= yyf; yy+=gBoxStrideY) {
	for (int xx = xxs; xx <= xxf; xx+=gBoxStrideX) {
	  //Mat gCrop1 = img_cvt(cv::Rect(top.x, top.y, bot.x-top.x, bot.y-top.y));
	  Mat gCrop1 = img_cvt(cv::Rect(xx, yy, bot.x-top.x, bot.y-top.y));

	  // grow to the max dimension to avoid distortion
	  int crows = gCrop1.rows;
	  int ccols = gCrop1.cols;
	  int maxDim = max(crows, ccols);
	  Mat gCrop(maxDim, maxDim, gCrop1.type());
	  int tRy = (maxDim-crows)/2;
	  int tRx = (maxDim-ccols)/2;

	  for (int x = 0; x < maxDim; x++) {
	    for (int y = 0; y < maxDim; y++) {
	      int tx = x - tRx;
	      int ty = y - tRy;
	      if (tx >= 0 && ty >= 0 && ty < crows && tx < ccols)
		gCrop.at<cv::Vec3b>(y, x) = gCrop1.at<cv::Vec3b>(ty, tx);
	      else
		gCrop.at<cv::Vec3b>(y, x) = cv::Vec<uchar, 3>(0,0,0);
	    }
	  }
#ifdef DEBUG
//cout << endl << gCrop1 << endl << endl << endl << endl << gCrop << endl;
#endif

	  cv::resize(gCrop, gCrop, orientedFilters[0].size());
	  gCrop.convertTo(gCrop, orientedFilters[0].type());

	  Mat gcChannels[3];
	  split(gCrop, gcChannels);

	  //double norm = gcChannels[1].dot(Mat::ones(O_FILTER_WIDTH, O_FILTER_WIDTH, CV_64F));
	  double norm = sqrt(gcChannels[1].dot(gcChannels[1]));

	  for (int o = 0; o < ORIENTATIONS; o++) {
	    //double thisScore = gcChannels[1].dot(orientedFilters[o]);
	    double thisScore = gcChannels[1].dot(orientedFilters[o]) / norm;
	    if (thisScore > winningScore) {
	      winningScore = thisScore;
	      winningO = o;
	      winningX = xx;
	      winningY = yy;
	    }
	  }
	}
      }
#ifdef DEBUG
cout << winningX << " " << winningY << " " << xxs << " " << yys  << endl;
#endif


      cv::Matx33f R;
      R(0,0) = 1; R(0,1) = 0; R(0,2) = 0;
      R(1,0) = 0; R(1,1) = 1; R(1,2) = 0;
      R(2,0) = 0; R(2,1) = 0; R(2,2) = 1;

      // handle the rotation differently depending on the class
      // if we have a spoon
      if (0 == classPoseModels[label].compare("S")) {
	double theta = (M_PI / 2.0) + (winningO*2*M_PI/ORIENTATIONS);
	R(0,0) = cos(theta); R(0,1) = -sin(theta); R(0,2) = 0;
	R(1,0) = sin(theta); R(1,1) =  cos(theta); R(1,2) = 0;
	R(2,0) = 0;          R(2,1) = 0;           R(2,2) = 1;
      }

#ifdef DEBUG
cout << "constructing rotation matrix" << endl;
#endif

      Eigen::Matrix3f rotation;
      rotation << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2);
      Eigen::Quaternionf objectQuaternion(rotation);

      objectQuaternion = tableQuaternion * objectQuaternion;

      if (0 == classLabels[label].compare(table_label_class_name)) {
	tableLabelQuaternion = objectQuaternion;
      }

#ifdef DEBUG
cout << "table label x: " << tableLabelQuaternion.x() << " y: " << 
  tableLabelQuaternion.y() << " z: " << tableLabelQuaternion.z() << " w: " << tableLabelQuaternion.w() << " " << endl;
#endif

      if (drawOrientor) {
	Mat vCrop = cv_ptr->image(cv::Rect(top.x, top.y, bot.x-top.x, bot.y-top.y));
	vCrop = vCrop.mul(0.5);

	Mat scaledFilter;
	// XXX
	cv::resize(orientedFilters[winningO], scaledFilter, vCrop.size());
	//cv::resize(orientedFilters[fc % ORIENTATIONS], scaledFilter, vCrop.size());
	//fc++;
  #ifdef DEBUG
  cout << "FILTERS: " << fc << " " << orientedFilters[fc % ORIENTATIONS].size() << endl;
  #endif
	scaledFilter = biggestL1*scaledFilter;
	 
	vector<Mat> channels;
	channels.push_back(Mat::zeros(scaledFilter.size(), scaledFilter.type()));
	channels.push_back(Mat::zeros(scaledFilter.size(), scaledFilter.type()));
	channels.push_back(scaledFilter);
	merge(channels, scaledFilter);

	scaledFilter.convertTo(scaledFilter, vCrop.type());
	vCrop = vCrop + 128*scaledFilter;
      }
    }

#ifdef DEBUG
fprintf(stderr, " object check4"); fflush(stderr);
#endif

    if (0 == classPoseModels[label].compare("G")) {
      string result;
      ostringstream convert;
      convert << poseIndex;
      result = convert.str();
      augmentedLabelName = augmentedLabelName + " " + result;
    }
  }
}

void init_oriented_filters() {

  for (int x = 0; x < O_FILTER_WIDTH; x++) {
    for (int y = 0; y < O_FILTER_WIDTH; y++) {
      orientedFilters[0].at<double>(y,x) = 0.0;
    }
  }
  // Spoon filters are used to estimate the orientation of spoon-like objects
  // based on their green maps. Hard coded for convenience. 
  // This approach is pretty flexible and could work for other types of objects.
  // A map could be estimated by some other process and loaded here, or just represented
  // as bitmap and converted here.
  // TODO make this comparison based based on an affine transformation, it will be more accurate.
/*
  // Diagonal Spoon Filter
  for (int x = 0; x < O_FILTER_SPOON_HEAD_WIDTH; x++) {
    for (int y = 0; y < O_FILTER_SPOON_HEAD_WIDTH; y++) {
      orientedFilters[0].at<double>(y,x) = 1.0;
    }
  }
  for (int x = 0; x < O_FILTER_WIDTH; x++) {
    for (int y = max(0,x-O_FILTER_SPOON_SHAFT_WIDTH); y < min(O_FILTER_WIDTH, x+O_FILTER_SPOON_SHAFT_WIDTH); y++) {
      orientedFilters[0].at<double>(y,x) = 1.0;
    }
  }
*/
/*
  // Vertical Spoon Filter
  int center = (O_FILTER_WIDTH-1)/2;
  for (int x = center-O_FILTER_SPOON_SHAFT_WIDTH; x <= center+O_FILTER_SPOON_SHAFT_WIDTH; x++) {
    for (int y = 0; y < O_FILTER_WIDTH; y++) {
      orientedFilters[0].at<double>(y,x) = 1.0;
    }
  }
  for (int x = center-O_FILTER_SPOON_SHAFT_WIDTH-1; x <= center+O_FILTER_SPOON_SHAFT_WIDTH+1; x++) {
    for (int y = 1; y < 3+O_FILTER_SPOON_HEAD_WIDTH; y++) {
      orientedFilters[0].at<double>(y,x) = 1.0;
    }
  }
  for (int x = center-O_FILTER_SPOON_HEAD_WIDTH; x <= center+O_FILTER_SPOON_HEAD_WIDTH; x++) {
    for (int y = 2; y < 2+O_FILTER_SPOON_HEAD_WIDTH; y++) {
      orientedFilters[0].at<double>(y,x) = 1.0;
    }
  }
*/
/*
  // Vertical Knife Filter
  int center = (O_FILTER_WIDTH-1)/2;
  for (int x = center-O_FILTER_SPOON_SHAFT_WIDTH; x <= center+O_FILTER_SPOON_SHAFT_WIDTH; x++) {
    for (int y = 0; y < O_FILTER_WIDTH; y++) {
      orientedFilters[0].at<double>(y,x) = 1.0;
    }
  }
*/
  // mrT Filter
  int center = (O_FILTER_WIDTH-1)/2;
  for (int x = center-O_FILTER_SPOON_SHAFT_WIDTH; x <= center+O_FILTER_SPOON_SHAFT_WIDTH; x++) {
    for (int y = 0; y < O_FILTER_WIDTH; y++) {
      orientedFilters[0].at<double>(y,x) = 1.0;
    }
  }
  for (int x = center-5*O_FILTER_SPOON_SHAFT_WIDTH; x <= center+5*O_FILTER_SPOON_SHAFT_WIDTH; x++) 
  //for (int x = 2*O_; x < O_FILTER_WIDTH; x++)
  //for (int x = O_FILTER_WIDTH/2-4; x < O_FILTER_WIDTH/2+4; x++)
  {
    for (int y = 0; y < 2*O_FILTER_SPOON_SHAFT_WIDTH-1; y++) {
      orientedFilters[0].at<double>(y,x) = 1.0;
    }
  }

  Mat tmp; 
  Mat tmp2;
  Mat tmp3;

  // it is important to L1 normalize the filters so that comparing dot products makes sense.
  // that is, they should all respond equally to a constant image.
  biggestL1 = -1;

  for (int o = 1; o < ORIENTATIONS; o++) {
    // Compute a rotation matrix with respect to the center of the image
    //Point center = Point(O_FILTER_WIDTH/2, O_FILTER_WIDTH/2);
    Point center = Point(O_FILTER_WIDTH, O_FILTER_WIDTH);
    double angle = o*360.0/ORIENTATIONS;
    double scale = 1.0;

    // Get the rotation matrix with the specifications above
    Mat rot_mat = getRotationMatrix2D( center, angle, scale );

    cv::Point2f srcTri[3]; 
    cv::Point2f dstTri[3]; 

    srcTri[0].x = 0;
    srcTri[0].y = 0;
    srcTri[1].x = 1;
    srcTri[1].y = 0;
    srcTri[2].x = 0;
    srcTri[2].y = 1;
    dstTri[0].x = 0+O_FILTER_WIDTH/2;
    dstTri[0].y = 0+O_FILTER_WIDTH/2;
    dstTri[1].x = 1+O_FILTER_WIDTH/2;
    dstTri[1].y = 0+O_FILTER_WIDTH/2;
    dstTri[2].x = 0+O_FILTER_WIDTH/2;
    dstTri[2].y = 1+O_FILTER_WIDTH/2;

    Mat trans_mat = getAffineTransform(srcTri, dstTri);

    cv::Size rangeSize = orientedFilters[0].size();
    rangeSize.width *= 2;
    rangeSize.height *= 2;
//cout << rangeSize;

    // Rotate the warped image
    //warpAffine(orientedFilters[0], orientedFilters[o], rot_mat, orientedFilters[o].size());
    warpAffine(orientedFilters[0], tmp, trans_mat, rangeSize);
    warpAffine(tmp, tmp2, rot_mat, rangeSize);
    warpPerspective(tmp2, tmp3, tablePerspective, rangeSize, INTER_NEAREST);

    //int topOne = O_FILTER_WIDTH/3;
    //int botOne = 2*O_FILTER_WIDTH-topOne;
    //int leftOne = O_FILTER_WIDTH/3;
    //int rightOne = 2*O_FILTER_WIDTH-leftOne;

    int topOne = 4*O_FILTER_WIDTH;
    int botOne = -1;
    int leftOne = 4*O_FILTER_WIDTH;
    int rightOne = -1;

    for (int y = 0; y < 2*O_FILTER_WIDTH; y++) {
      for (int x = 0; x < 2*O_FILTER_WIDTH; x++) {
	if(tmp3.at<double>(y,x) > 0) {
	  if (y < topOne) 
	    topOne = y;
	  if (x < leftOne)
	    leftOne = x;
	  if (y > botOne)
	    botOne = y;
	  if (x > rightOne)
	    rightOne = x;
	}
      }
    }

//cout << orientedFilters[0].size() << rangeSize << tmp.size() << tmp2.size() << tmp3.size() << endl;

    Mat validCrop = tmp3(cv::Rect(leftOne, topOne, rightOne-leftOne, botOne-topOne));


    // grow to the max dimension to avoid distortion
    int crows = validCrop.rows;
    int ccols = validCrop.cols;
    int maxDim = max(crows, ccols);
    Mat vCrop(maxDim, maxDim, validCrop.type());
    int tRy = (maxDim-crows)/2;
    int tRx = (maxDim-ccols)/2;

    for (int x = 0; x < maxDim; x++) {
      for (int y = 0; y < maxDim; y++) {
	int tx = x - tRx;
	int ty = y - tRy;
	if (tx >= 0 && ty >= 0 && ty < crows && tx < ccols)
	  vCrop.at<double>(y, x) = validCrop.at<double>(ty, tx);
	else
	  vCrop.at<double>(y, x) = 0.0;
      }
    }


    cv::resize(vCrop, orientedFilters[o], orientedFilters[0].size());
    //cv::resize(tmp3, orientedFilters[o], orientedFilters[0].size());

    double l1norm = orientedFilters[o].dot(Mat::ones(O_FILTER_WIDTH, O_FILTER_WIDTH, CV_64F));
    orientedFilters[o] = orientedFilters[o] / l1norm;
    
    if (l1norm > biggestL1)
      biggestL1 = l1norm;
  }

  tmp = orientedFilters[0].clone();
  warpPerspective(tmp, orientedFilters[0], tablePerspective, orientedFilters[0].size(), INTER_NEAREST);
#ifdef DEBUG
cout << endl << tablePerspective << endl;
cout << endl << orientedFilters[0] << endl;
#endif

  double l1norm = orientedFilters[0].dot(Mat::ones(O_FILTER_WIDTH, O_FILTER_WIDTH, CV_64F));
  orientedFilters[0] = orientedFilters[0] / l1norm;
  if (l1norm > biggestL1)
    biggestL1 = l1norm;
}

void handleKeyboardInput(int c) {

  switch (c) {
    case 'q':
      drawOrientor = !drawOrientor;
      break;
    case 'w':
      drawLabels = !drawLabels;
      break;
    case 'e':
      add_blinders = !add_blinders;
      break;
    case 'r':
      mask_gripper = !mask_gripper;
      break;

    case 'a':
      drawPurple= !drawPurple;
      break;
    case 's':
      drawWhite = !drawWhite;
      break;
    case 'd':
      drawGreen = !drawGreen;
      break;
    case 'f':
      drawBlue = !drawBlue;
      break;
    case 'g':
      drawRed = !drawRed;
      break;
    case 'h':
      drawRB = !drawRB;
      break;
    case 'j':
      drawGray = !drawGray;
      break;
    case 'k':
      drawPink = !drawPink;
      break;
    case 'l':
      drawBrown = !drawBrown;
      break;

    case 'z':
      drawBlueKP = !drawBlueKP;
      break;
    case 'x':
      drawRedKP = !drawRedKP;
      break;

    default:
      break;
  }

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){

  ros::NodeHandle nh("~");

  invertQuaternionLabel = 0;

  time(&thisTime);
  double deltaTime = difftime(thisTime, firstTime);
  timeMass = timeMass + 1;

  if (deltaTime > timeInterval) {
    deltaTime = 0;
    timeMass = 0;
    time(&firstTime);
  }

  if (timeMass > 0.0)
    aveTime = deltaTime / timeMass;

  if (deltaTime > 0.0)
    aveFrequency = timeMass / deltaTime;


cout << "Average time between frames: " << aveTime << 
  "   Average Frequency: " << aveFrequency << " Hz   Duration of sampling: " << 
  deltaTime << "   Frames since sampling: " << timeMass << endl; 


//cout << "here 1" << endl;

  int lastTime = thisTime; 

  gBoxStrideX = gBoxW / 2.0;
  gBoxStrideY = gBoxH / 2.0;

  loadROSParams();
  cv::Rect bound;
  cv::Mat boxed;
//cout << "here 2" << endl;

  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cam_img = cv_ptr->image;
    real_img = true;
  }catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
//cout << "here 3" << endl;

  Size sz = cv_ptr->image.size();
  int imW = sz.width;
  int imH = sz.height;

  if (add_blinders) {
    int thisWidth = blinder_stride*blinder_columns;
    for (int x = 0; x < thisWidth; x++) {
      for (int y = 0; y < imH; y++) {
	int bCol = x/blinder_stride;
	int bRow = y/blinder_stride;
	int blackOrWhite = ((bCol + bRow)%2)*255;
	cv_ptr->image.at<cv::Vec3b>(y, x) = cv::Vec<uchar, 3>(blackOrWhite, blackOrWhite, blackOrWhite);
      }
    }
    for (int x = imW-thisWidth; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	int bCol = x/blinder_stride;
	int bRow = y/blinder_stride;
	int blackOrWhite = ((bCol + bRow)%2)*255;
	cv_ptr->image.at<cv::Vec3b>(y, x) = cv::Vec<uchar, 3>(blackOrWhite, blackOrWhite, blackOrWhite);
      }
    }

  }

  object_recognition_msgs::RecognizedObjectArray roa_to_send_blue;
  visualization_msgs::MarkerArray ma_to_send_blue; 

  int boxesPerSize = 800;

  Mat cam_img2;
  cam_img2 = cam_img;
/* If you want to snap to 640 x 480 now is the time.
  if (cam_img.cols == 640 && cam_img.rows == 480) {
    cam_img2 = cam_img;
  } else {
    cv::Size sffe;
    sffe.width = 640;
    sffe.height = 480;
    cv::resize(cam_img, cam_img2, sffe);
  }
#ifdef DEBUG
  cout << cam_img.size() << cam_img2.size() << endl;
#endif
*/

  
  // XXX find best method and stop all this cloning, it might be relatively slow
  Mat original_cam_img = cam_img2.clone();

  Mat img_cvt = cam_img2.clone();
  Mat img_cvtH = cam_img2.clone();
  Mat img_cvtG = cam_img2.clone();
  Mat img_cvtGtmp = cam_img2.clone();

//cout << "here 4" << endl;
  Mat img_cvt_blur = cam_img2.clone();

/*
  // XXX Sobel business
  Mat local_ave;
  cvtColor(img_cvt_blur, img_cvt_blur, CV_RGB2GRAY );
  GaussianBlur(img_cvt_blur, img_cvt_blur, Size(max(4*sobel_sigma+1, 17.0),max(4*sobel_sigma+1, 17.0)), sobel_sigma, sobel_sigma, BORDER_DEFAULT); 
  Mat grad_x, grad_y;
  int sobelScale = 1;
  int sobelDelta = 0;
  int sobelDepth = CV_32F;
//cout << "here 5" << endl;
  /// Gradient X
  Sobel(img_cvt_blur, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
  /// Gradient Y
  Sobel(img_cvt_blur, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
//cout << "here 6" << endl;

  grad_x = grad_x.mul(grad_x);
  grad_y = grad_y.mul(grad_y);
  Mat totalSobel = grad_x + grad_y;
  // now totalSobel is gradient magnitude squared

  // make it gradient magnitude to the fourth to spread the values a little
  // this increases robustness and makes it easier to tune
  pow(totalSobel, 2.0, totalSobel);
  totalSobel = totalSobel * sobel_scale_factor;

  //totalSobel = abs(totalSobel);
  //cv::sqrt(totalSobel, totalSobel);
  //cv::sqrt(totalSobel, totalSobel);

  // try local contrast normalization
  //GaussianBlur(totalSobel, local_ave, Size(max(4*local_sobel_sigma+1, 17.0),max(4*local_sobel_sigma+1, 17.0)), local_sobel_sigma, local_sobel_sigma, BORDER_DEFAULT); 
  //local_ave = cv::max(local_ave,.000001);
  //totalSobel = local_ave / totalSobel;

  // try laplacian
  Mat lapl;
  Laplacian(img_cvt_blur, lapl, sobelDepth, 1, sobelScale, sobelDelta, BORDER_DEFAULT);
  //pow(totalSobel, 4.0, totalSobel);
  //totalSobel = max(totalSobel, .001);
  //totalSobel = 1 / totalSobel;
  totalSobel = lapl.mul(totalSobel);
*/

//cout << "here 7" << endl;


  cvtColor(cam_img2, img_cvtGtmp, CV_RGB2GRAY);
  cvtColor(cam_img2, img_cvtH, CV_RGB2HSV);

  //vector<Mat> channels;
  //channels.push_back(img_cvtGtmp);
  //channels.push_back(img_cvtGtmp);
  //channels.push_back(img_cvtGtmp);
  //merge(channels, img_cvtG);

  // input image is noisy so blurring is a good idea
  //GaussianBlur(img_cvt, img_cvt, cv::Size(0,0), 1.0);
//cout << "here 4" << img_cvt.size() << endl;

  ValStructVec<float, Vec4i> boxes;
  //glObjectness->getObjBndBoxes(cam_img2, boxes, boxesPerSize);
  glObjectness->getObjBndBoxes(img_cvt, boxes, boxesPerSize);
  //Mat test;
  //img_cvt.convertTo(test, CV_16U);
  //glObjectness->getObjBndBoxes(test, boxes, boxesPerSize);

//cout << "here 5" << endl;

  int numBoxes = boxes.size();
  // box[0] minx box[1] miny box[2] maxx box[3] maxy
//cout << "here 6" << endl;
#ifdef DEBUG
cout << "numBoxes: " << numBoxes << "  fc: " << fc <<  endl;
#endif

  nTop.resize(numBoxes);
  nBot.resize(numBoxes);

  int boxesToConsider= 50000;

  //fc = (fc + 1) % fcRange;
  // XXX
  //fc++;

  double *integralDensity = new double[imW*imH];
  double *density = new double[imW*imH];
  double *differentialDensity = new double[imW*imH];
  if (temporalDensity == NULL) {
    temporalDensity = new double[imW*imH];
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	temporalDensity[y*imW + x] = 0;
      }
    }
  }

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      density[y*imW + x] = 0;
      differentialDensity[y*imW + x] = 0;
    }
  }

  if (drawWhite) {
    // draw the ork bounding boxes
    for(int i =0; i<wTop.size(); i++){
      rectangle(cv_ptr->image, wTop[i], wBot[i], cv::Scalar(255,255,255));
    }
  }

  // XXX make this prettier
  for(int i =0; i < min(numBoxes, boxesToConsider); i++){
    int boxInd = numBoxes-1-i;
    boxInd = max(0, boxInd);
    nTop[i].x = boxes[boxInd][0] - 1;
    nTop[i].y = boxes[boxInd][1] - 1;
    nBot[i].x = boxes[boxInd][2] - 1;
    nBot[i].y = boxes[boxInd][3] - 1;
    double width = nBot[i].x - nTop[i].x;
    double height= nBot[i].y - nTop[i].y;
    double ratio = width / height;
    double area = width*height;
    double aspectThresh = 20.0;
    if (ratio < aspectThresh && ratio > 1.0/aspectThresh) {
      if (drawPurple) {
	if (drand48() < drawBingProb) {
	  cv::Point outTop = cv::Point(nTop[i].x, nTop[i].y);
	  cv::Point outBot = cv::Point(nBot[i].x, nBot[i].y);
	  cv::Point inTop = cv::Point(nTop[i].x+1,nTop[i].y+1);
	  cv::Point inBot = cv::Point(nBot[i].x-1,nBot[i].y-1);
	  rectangle(cv_ptr->image, outTop, outBot, cv::Scalar(188,40,140));
	  rectangle(cv_ptr->image, inTop, inBot, cv::Scalar(94,20,70));
	}
      }
      
      double toAdd = 1.0 / area;
      //double toAdd = 1.0;
      int x = nTop[i].x;
      int y = nTop[i].y;
      differentialDensity[y*imW + x] += toAdd;
      x = nBot[i].x;
      y = nBot[i].y;
      differentialDensity[y*imW + x] += toAdd;
      x = nTop[i].x;
      y = nBot[i].y;
      differentialDensity[y*imW + x] -= toAdd;
      x = nBot[i].x;
      y = nTop[i].y;
      differentialDensity[y*imW + x] -= toAdd;
    }
  }


  // integrate the differential density into the density
  density[0] = differentialDensity[0];
  for (int x = 1; x < imW; x++) {
    int y = 0;
    density[y*imW+x] = density[y*imW+(x-1)] + differentialDensity[y*imW + x];
  }
  for (int y = 1; y < imH; y++) {
    int x = 0;
    density[y*imW+x] = density[(y-1)*imW+x] + differentialDensity[y*imW + x];
  }
  for (int x = 1; x < imW; x++) {
    for (int y = 1; y < imH; y++) {
      density[y*imW+x] = 
	density[(y-1)*imW+x]+density[y*imW+(x-1)]-density[(y-1)*imW+(x-1)]+differentialDensity[y*imW + x];
    }
  }

  /*
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      density[y*imW+x] = totalSobel.at<float>(y,x);
    }
  }
  */

  // now update the exponential average of the density
  // and set the density to be a thresholded version of this
  double maxDensity = 0;
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      temporalDensity[y*imW+x] = densityDecay*temporalDensity[y*imW+x] + (1.0-densityDecay)*density[y*imW+x];
      density[y*imW+x] = pow(temporalDensity[y*imW+x], densityPower);
      maxDensity = max(maxDensity, density[y*imW+x]);
    }
  }
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      if (density[y*imW+x] < maxDensity* threshFraction)
	density[y*imW+x] = 0;
    }
  }


  // integrate density into the integral density
  integralDensity[0] = density[0];
  for (int x = 1; x < imW; x++) {
    int y = 0;
    integralDensity[y*imW+x] = integralDensity[y*imW+(x-1)] + density[y*imW + x];
  }
  for (int y = 1; y < imH; y++) {
    int x = 0;
    integralDensity[y*imW+x] = integralDensity[(y-1)*imW+x] + density[y*imW + x];
  }
  for (int x = 1; x < imW; x++) {
    for (int y = 1; y < imH; y++) {
      integralDensity[y*imW+x] = 
	integralDensity[(y-1)*imW+x]+integralDensity[y*imW+(x-1)]-integralDensity[(y-1)*imW+(x-1)]+density[y*imW + x];
    }
  }

  // determine table edges, i.e. the gray boxes
  lGO = gBoxW*(lGO/gBoxW);
  rGO = gBoxW*(rGO/gBoxW);
  tGO = gBoxH*(tGO/gBoxH);
  bGO = gBoxH*(bGO/gBoxH);
  grayTop = cv::Point(lGO, tGO);
  grayBot = cv::Point(imW-rGO, imH-bGO);

  if (all_range_mode) {
    grayTop = armTop;
    grayBot = armBot;
  }


  if (drawGray) {
    cv::Point outTop = cv::Point(grayTop.x, grayTop.y);
    cv::Point outBot = cv::Point(grayBot.x, grayBot.y);
    cv::Point inTop = cv::Point(grayTop.x+1,grayTop.y+1);
    cv::Point inBot = cv::Point(grayBot.x-1,grayBot.y-1);
    rectangle(cv_ptr->image, outTop, outBot, cv::Scalar(128,128,128));
    rectangle(cv_ptr->image, inTop, inBot, cv::Scalar(32,32,32));
  }

  if (mask_gripper) {
    int xs = 200;
    int xe = 295;
    int ys = 0;
    int ye = 75;
    for (int x = xs; x < xe; x++) {
      for (int y = ys; y < ye; y++) {
	density[y*imW+x] = 0;
      }
    }
    Mat vCrop = cv_ptr->image(cv::Rect(xs, ys, xe-xs, ye-ys));
    vCrop = vCrop/2;
    xs = 420;
    xe = 560;
    ys = 0;
    ye = 75;
    for (int x = xs; x < xe; x++) {
      for (int y = ys; y < ye; y++) {
	density[y*imW+x] = 0;
      }
    }
    Mat vCrop2 = cv_ptr->image(cv::Rect(xs, ys, xe-xs, ye-ys));
    vCrop2 = vCrop2/2;
  }

  // truncate the density outside the gray box
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < grayTop.y; y++) {
      density[y*imW+x] = 0;
    }
  }

  for (int x = 0; x < grayTop.x; x++) {
    for (int y = grayTop.y; y < grayBot.y; y++) {
      density[y*imW+x] = 0;
    }
  }

  for (int x = grayBot.x; x < imW; x++) {
    for (int y = grayTop.y; y < grayBot.y; y++) {
      density[y*imW+x] = 0;
    }
  }

  for (int x = 0; x < imW; x++) {
    for (int y = grayBot.y; y < imH; y++) {
      density[y*imW+x] = 0;
    }
  }

  // reintegrate the density
  //double maxIntegralDensity = 0;
  integralDensity[0] = density[0];
  for (int x = 1; x < imW; x++) {
    int y = 0;
    integralDensity[y*imW+x] = integralDensity[y*imW+(x-1)] + density[y*imW + x];
    //maxIntegralDensity = max(maxIntegralDensity, integralDensity[y*imW+x]);
  }
  for (int y = 1; y < imH; y++) {
    int x = 0;
    integralDensity[y*imW+x] = integralDensity[(y-1)*imW+x] + density[y*imW + x];
    //maxIntegralDensity = max(maxIntegralDensity, integralDensity[y*imW+x]);
  }
  for (int x = 1; x < imW; x++) {
    for (int y = 1; y < imH; y++) {
      integralDensity[y*imW+x] = 
	integralDensity[(y-1)*imW+x]+integralDensity[y*imW+(x-1)]-integralDensity[(y-1)*imW+(x-1)]+density[y*imW + x];
      //maxIntegralDensity = max(maxIntegralDensity, integralDensity[y*imW+x]);
    }
  }

  gBoxIndicator = new double[imW*imH];
  double *gBoxGrayNodes = new double[imW*imH];
  double *gBoxComponentLabels = new double[imW*imH];
  pBoxIndicator = new double[imW*imH];

  vector<int> parentX;
  vector<int> parentY;
  vector<int> parentD;
  
  cTops.resize(0);
  cBots.resize(0);

  const int directionX[] = {1, 0, -1,  0};
  const int directionY[] = {0, 1,  0, -1};

  int total_components = 0;

  // make sure that green boxes stay within the grey
  // box and stay on the canonical green matter grid
  int xS = gBoxW*(grayTop.x/gBoxW);
  int xF = min(grayBot.x-gBoxW, imW-gBoxW);
  int yS = gBoxH*(grayTop.y/gBoxH);
  int yF = min(grayBot.y-gBoxH, imH-gBoxH);

  double adjusted_canny_lo_thresh = canny_lo_thresh * (1.0 + (double(loTrackbarVariable-50) / 50.0));
  double adjusted_canny_hi_thresh = canny_hi_thresh * (1.0 + (double(hiTrackbarVariable-50) / 50.0));

  for (int x = xS; x <= xF; x+=gBoxStrideX) {
    for (int y = yS; y <= yF; y+=gBoxStrideY) {

      int xt = x;
      int yt = y;
      int xb = x+gBoxW;
      int yb = y+gBoxH;
      cv::Point thisTop(xt,yt);
      cv::Point thisBot(xb,yb);

      gBoxComponentLabels[y*imW+x] = -1;
      gBoxGrayNodes[y*imW+x] = 0;
      gBoxIndicator[y*imW+x] = 0;
      pBoxIndicator[y*imW+x] = 0;

      double thisIntegral = integralDensity[yb*imW+xb]-integralDensity[yb*imW+xt]-
	integralDensity[yt*imW+xb]+integralDensity[yt*imW+xt];

      if (thisIntegral > adjusted_canny_lo_thresh) {
	      gBoxIndicator[y*imW+x] = 1;
	      if (drawGreen)
		rectangle(cv_ptr->image, thisTop, thisBot, cv::Scalar(0,128,0));
      }
      if (thisIntegral > adjusted_canny_hi_thresh) {
	      gBoxIndicator[y*imW+x] = 2;
	      if (drawGreen)
		rectangle(cv_ptr->image, thisTop, thisBot, cv::Scalar(0,255,0));
      }
      pBoxIndicator[y*imW+x] = thisIntegral;

    }
  }

  // canny will start on a hi and spread on a lo or hi.
  for (int x = 0; x < imW-gBoxW; x+=gBoxStrideX) {
    for (int y = 0; y < imH-gBoxH; y+=gBoxStrideY) {
      if (gBoxIndicator[y*imW+x] == 2 && gBoxGrayNodes[y*imW+x] == 0) {

      	gBoxGrayNodes[y*imW+x] = 1;
      	parentX.push_back(x);
      	parentY.push_back(y);
      	parentD.push_back(0);

      	gBoxComponentLabels[y*imW+x] = total_components;
      	total_components++;

      	int xt = x;
      	int yt = y;
      	int xb = x+gBoxW;
      	int yb = y+gBoxH;
      	cv::Point thisTop(xt,yt);
      	cv::Point thisBot(xb,yb);
      	cTops.push_back(thisTop);
      	cBots.push_back(thisBot);

      	while( parentX.size() > 0 ) {
      	  int index = parentX.size()-1;
      	  int direction = parentD[index];
      	  parentD[index]++;
      	  int nextX = parentX[index] + gBoxStrideX*directionX[direction];
      	  int nextY = parentY[index] + gBoxStrideY*directionY[direction];

      	  // if we have no more directions, then pop this parent 
      	  if (direction > 3) {
      	    parentX.pop_back();
      	    parentY.pop_back();
      	    parentD.pop_back();
      	  } 
      	  // if the next direction is valid, push it on to the stack and increment direction counter
      	  else if(gBoxIndicator[nextY*imW+nextX] >= 1 && gBoxGrayNodes[nextY*imW+nextX] == 0
      		  && nextX > -1 && nextX < imW && nextY > -1 && nextY < imH) {

      	    gBoxGrayNodes[nextY*imW+nextX] = 1;
      	    gBoxComponentLabels[nextY*imW+nextX] = gBoxComponentLabels[parentY[index]*imW+parentX[index]];

      	    int nxt = nextX;
      	    int nyt = nextY;
      	    int nxb = nextX+gBoxW;
      	    int nyb = nextY+gBoxH;
      	    cTops[gBoxComponentLabels[nextY*imW+nextX]].x = min(cTops[gBoxComponentLabels[nextY*imW+nextX]].x, nxt);
      	    cTops[gBoxComponentLabels[nextY*imW+nextX]].y = min(cTops[gBoxComponentLabels[nextY*imW+nextX]].y, nyt);
      	    cBots[gBoxComponentLabels[nextY*imW+nextX]].x = max(cBots[gBoxComponentLabels[nextY*imW+nextX]].x, nxb);
      	    cBots[gBoxComponentLabels[nextY*imW+nextX]].y = max(cBots[gBoxComponentLabels[nextY*imW+nextX]].y, nyb);

      	    parentX.push_back(nextX);
      	    parentY.push_back(nextY);
      	    parentD.push_back(0);
      	  } 
      	}

      }
    }
  }

  bTops.resize(0);
  bBots.resize(0);
  bCens.resize(0);

  lARM = gBoxW*(lARM/gBoxW);
  rARM = gBoxW*(rARM/gBoxW);
  tARM = gBoxH*(tARM/gBoxH);
  bARM = gBoxH*(bARM/gBoxH);
  armTop = cv::Point(lARM, tARM);
  armBot = cv::Point(imW-rARM, imH-bARM);

  int biggestBB = -1;
  int biggestBBArea = 0;

  if (!all_range_mode) {
    double rejectArea = rejectAreaScale*gBoxW*gBoxH;
    for (int c = 0; c < total_components; c++) {
      int allow = 1;
      if (cBots[c].x - cTops[c].x < rejectScale*gBoxW || cBots[c].y - cTops[c].y < rejectScale*gBoxH)
	allow = 0;
      if ((cBots[c].x - cTops[c].x)*(cBots[c].y - cTops[c].y) < rejectArea)
	allow = 0;
      //if (cTops[c].y > rejectLow || cBots[c].y < rejectHigh)
	//allow = 0;
      if (allow == 1) {
	bTops.push_back(cTops[c]);
	bBots.push_back(cBots[c]);
	bCens.push_back(cv::Point((cTops[c].x+cBots[c].x)/2, (cTops[c].y+cBots[c].y)/2));
	int t = bTops.size()-1;

	int thisArea = (cBots[c].x - cTops[c].x)*(cBots[c].y - cTops[c].y);
	if (thisArea > biggestBBArea) {
	  biggestBBArea = thisArea;
	  biggestBB = t;
	}
	
	
      }
    }
  } else {
    bTops.push_back(armTop);
    bBots.push_back(armBot);
    bCens.push_back(cv::Point((armTop.x+armBot.x)/2, (armTop.y+armBot.y)/2));
  }

  if (bTops.size() > 0) {
    geometry_msgs::Point p;
    p.x = bCens[biggestBB].x;
    p.y = bCens[biggestBB].y;
    p.z = 0.0;
  
    ee_target_pub.publish(p);
  }


  // copy the density map to the rendered image
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      uchar val = uchar(min( 255.0 * density[y*imW+x] / maxDensity, 255.0));
      img_cvt.at<cv::Vec3b>(y,x) = cv::Vec<uchar, 3>(0,val,0);
      //img_cvt.at<uchar>(y, x, 0) = 0;
    }
  }


#ifdef RUN_INFERENCE

  // the brown box is a box representing the location of the dominant table in the scene.
  // locate a good choice for the brown box.
  int brownBoxWidth = gBoxW;//50;
  int brownPadding = 0;
  brTop = cv::Point(0,0);
  brBot = cv::Point(brownBoxWidth,brownBoxWidth);

  cv::Point thisBrTop(0,0);
  cv::Point thisBrBot(0,0);

  // raster scan the image looking for a box that doesn't intersect with blueBoxes
  int rejectAll = 1;
  int acceptedBrBoxes = 0;
  int reject = 0;
  tableNormalSum = Eigen::Vector3d(0,0,0);
  tableTangent1Sum = Eigen::Vector3d(0,0,0);
  tableTangent2Sum = Eigen::Vector3d(0,0,0);
  tablePositionSum = Eigen::Vector3d(0,0,0);
  tableBiasSum = 0;

  for (int bY = grayTop.y + brownPadding; 
	bY <= grayBot.y - brownBoxWidth-brownPadding; bY=bY+(brownBoxWidth/2)) {
    for (int bX = grayTop.x + brownPadding; 
	bX <= grayBot.x - brownBoxWidth-brownPadding; bX = bX+(brownBoxWidth/2)) {
      cv::Point thisCen = cv::Point(bX+brownBoxWidth/2, bY+brownBoxWidth/2);

      thisBrTop.x = bX;
      thisBrTop.y = bY;
      thisBrBot.x = bX+brownBoxWidth;
      thisBrBot.y = bY+brownBoxWidth;

      int reject = 0;
      for (int c = 0; c < bTops.size(); c++) {
#ifdef DEBUG
//cout << "brBox   " << c << " / " << bTops.size() << " " << fabs(bCens[c].x - thisCen.x) << endl;
#endif
	if ( fabs(bCens[c].x - thisCen.x) < ((fabs(bBots[c].x-bTops[c].x)+brownBoxWidth)/2)-1 && 
	      fabs(bCens[c].y - thisCen.y) < ((fabs(bBots[c].y-bTops[c].y)+brownBoxWidth)/2)-1 ) {
	  reject = 1;
	  break;
	} 
      }

      if (!reject) {


	if (pointCloud.size() > 0) {
	  pcl::PointXYZRGB p0 = pointCloud.at(thisBrTop.x, thisBrBot.y);
	  pcl::PointXYZRGB p1 = pointCloud.at(thisBrTop.x, thisBrTop.y);
	  pcl::PointXYZRGB p2 = pointCloud.at(thisBrBot.x, thisBrBot.y);
  
	  if (!isFiniteNumber(p0.x) ||
	      !isFiniteNumber(p0.y) ||
	      !isFiniteNumber(p0.z) ||
	      !isFiniteNumber(p1.x) ||
	      !isFiniteNumber(p1.y) ||
	      !isFiniteNumber(p1.z) ||
	      !isFiniteNumber(p2.x) ||
	      !isFiniteNumber(p2.y) ||
	      !isFiniteNumber(p2.z) ) {
	    reject = 1;

  #ifdef DRAW_BROWN
	    if (drawBrown)
	      rectangle(cv_ptr->image, thisBrTop, thisBrBot, cv::Scalar(0,102,204));
  #endif
	  }

#ifdef DEBUG
//cout << thisBrTop << thisBrBot << "p0, p1, p2:  " << p0 << p1 << p2 << endl;
//cout << "p0, p1, p2:  " << p0 << p1 << p2 << endl;
#endif

	  if (!reject) {
	    cv::Point tranTop = pcCorrection(thisBrTop.x, thisBrTop.y, imW, imH);
	    cv::Point tranBot = pcCorrection(thisBrBot.x, thisBrBot.y, imW, imH);

#ifdef DEBUG
//cout << "t " << thisBrTop << thisBrBot << endl << "  " << tranTop << tranBot << endl;
#endif

	    //pcl::PointXYZRGB p0 = pointCloud.at(thisBrTop.x, thisBrBot.y);
	    //pcl::PointXYZRGB p1 = pointCloud.at(thisBrTop.x, thisBrTop.y);
	    //pcl::PointXYZRGB p2 = pointCloud.at(thisBrBot.x, thisBrBot.y);

	    pcl::PointXYZRGB p0 = pointCloud.at(tranTop.x, tranBot.y);
	    pcl::PointXYZRGB p1 = pointCloud.at(tranTop.x, tranTop.y);
	    pcl::PointXYZRGB p2 = pointCloud.at(tranBot.x, tranBot.y);

	    Eigen::Vector3d localTablePosition(p0.x,p0.y,p0.z);

	    // a little gram-schmidt...
	    Eigen::Vector3d p01(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
	    Eigen::Vector3d p02(p2.x - p0.x, p2.y - p0.y, p2.z - p0.z);

	    double p02norm = sqrt(p02.dot(p02));
	    if (p02norm > 0) {
	      p02 = p02 / p02norm;
	    }

	    double p01p02dot = p01.dot(p02);
	    p01 = p01 - p01p02dot * p02;

	    double p01norm = sqrt(p01.dot(p01));
	    if (p01norm > 0) {
	      p01 = p01 / p01norm;
	    }

	    // ... and now p01 and p02 are orthonormal

	    // using the right hand rule for cross product order
	    Eigen::Vector3d p03 = p02.cross(p01);

	    if (!isFiniteNumber(p03.x()) ||
		!isFiniteNumber(p03.y()) ||
		!isFiniteNumber(p03.z()) ) {
	      reject = 1;

	      if (drawBrown)
		rectangle(cv_ptr->image, thisBrTop, thisBrBot, cv::Scalar(0,76,153));
	    }

	    if (!reject) {
	      acceptedBrBoxes++;
	      if (drawBrown)
		rectangle(cv_ptr->image, thisBrTop, thisBrBot, cv::Scalar(0,51,102));

	      tablePositionSum.x() = tablePositionSum.x() + p0.x + p1.x + p2.x;
	      tablePositionSum.y() = tablePositionSum.y() + p0.y + p1.y + p2.y;
	      tablePositionSum.z() = tablePositionSum.z() + p0.z + p1.z + p2.z;
	      tablePosition = tablePositionSum / (3*acceptedBrBoxes);

	      tableNormalSum = tableNormalSum + p03;
	      tableNormal = tableNormalSum;
	      tableNormal.normalize(); 

	      tableTangent1Sum = tableTangent1Sum + p01;
	      tableTangent2Sum = tableTangent2Sum + p02;
	      tableTangent1 = tableTangent1Sum;
	      tableTangent2 = tableTangent2Sum;
	      tableTangent1.normalize();
	      tableTangent2.normalize();

	      tableTangent1 = tableTangent1 - (tableTangent2.dot(tableTangent1) * tableTangent2);
	      tableTangent1.normalize();

	      tableNormal = tableTangent2.cross(tableTangent1);

#ifdef DEBUG
//cout << tableNormal << endl;
#endif

	      tableBiasSum = tableBiasSum + tableNormal.dot(localTablePosition);
	      tableBias = tableBiasSum / acceptedBrBoxes;

	      cv::Matx33f R;
	      R(0,0) = tableTangent2.x(); R(0,1) = tableTangent1.x(); R(0,2) = tableNormal.x();
	      R(1,0) = tableTangent2.y(); R(1,1) = tableTangent1.y(); R(1,2) = tableNormal.y();
	      R(2,0) = tableTangent2.z(); R(2,1) = tableTangent1.z(); R(2,2) = tableNormal.z();

	      Eigen::Matrix3f rotation;
	      rotation << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2);
	      Eigen::Quaternionf patchQuaternion(rotation);
	      tablePose.orientation.x = patchQuaternion.x();
	      tablePose.orientation.y = patchQuaternion.y();
	      tablePose.orientation.z = patchQuaternion.z();
	      tablePose.orientation.w = patchQuaternion.w();

	      tablePose.position.x = tablePosition.x();
	      tablePose.position.y = tablePosition.y();
	      tablePose.position.z = tablePosition.z();

	      // obtain the quaternion pose for the table
	      tableQuaternion.x() = tablePose.orientation.x;
	      tableQuaternion.y() = tablePose.orientation.y;
	      tableQuaternion.z() = tablePose.orientation.z;
	      tableQuaternion.w() = tablePose.orientation.w;



	      cv::Point2f srcQuad[4]; 
	      cv::Point2f dstQuad[4]; 

	      double normalizer = 1.0;

	      double scale = 1;
	      srcQuad[0].x = 0;
	      srcQuad[0].y = 0;

	      srcQuad[1].x = scale;
	      srcQuad[1].y = 0;

	      srcQuad[2].x = 0;
	      srcQuad[2].y = scale;

	      srcQuad[3].x = scale;
	      srcQuad[3].y = scale;

	      //dstQuad[0].x = 0;
	      //dstQuad[0].y = 0;

/*
	      tableNormal = -tableNormal;

	      normalizer = tableNormal.z();
	      if (fabs(normalizer) < 1e-6)
		normalizer = 1.0;
	      dstQuad[0].x = (tableNormal.x()) / normalizer;
	      dstQuad[0].y = (tableNormal.y()) / normalizer;

	      normalizer = tableTangent2.z() + tableNormal.z();
	      if (fabs(normalizer) < 1e-6)
		normalizer = 1.0;
	      dstQuad[1].x = (tableTangent2.x() + tableNormal.x()) / normalizer;
	      dstQuad[1].y = (tableTangent2.y() + tableNormal.y()) / normalizer;

	      normalizer = tableTangent1.z() + tableNormal.z();
	      if (fabs(normalizer) < 1e-6)
		normalizer = 1.0;
	      dstQuad[2].x = (-tableTangent1.x() + tableNormal.x()) / normalizer;
	      dstQuad[2].y = (-tableTangent1.y() + tableNormal.y()) / normalizer;

	      normalizer = tableTangent1.z() + tableTangent2.z() + tableNormal.z();
	      if (fabs(normalizer) < 1e-6)
		normalizer = 1.0;
	      dstQuad[3].x = (tableTangent2.x() + -tableTangent1.x() + tableNormal.x()) / normalizer;
	      dstQuad[3].y = (tableTangent2.y() + -tableTangent1.y() + tableNormal.y()) / normalizer;

	      tableNormal = -tableNormal;

	      dstQuad[0].x -= dstQuad[0].x;
	      dstQuad[0].y -= dstQuad[0].y;
	      dstQuad[1].x -= dstQuad[0].x;
	      dstQuad[1].y -= dstQuad[0].y;
	      dstQuad[2].x -= dstQuad[0].x;
	      dstQuad[2].y -= dstQuad[0].y;
	      dstQuad[3].x -= dstQuad[0].x;
	      dstQuad[3].y -= dstQuad[0].y;
*/



	      dstQuad[0].x = 0;
	      dstQuad[0].y = 0;
	      dstQuad[1].x = tableTangent2.x();
	      dstQuad[1].y = tableTangent2.y();
	      dstQuad[2].x = -tableTangent1.x();
	      dstQuad[2].y = -tableTangent1.y();
	      dstQuad[3].x = -tableTangent1.x() + tableTangent2.x();
	      dstQuad[3].y = -tableTangent1.y() + tableTangent2.y();

	      double toCenterX = (dstQuad[3].x-1.0)/2.0;
	      double toCenterY = (dstQuad[3].y-1.0)/2.0;

	      for (int dd = 0; dd < 4; dd++) {
		dstQuad[dd].x -= toCenterX;
		dstQuad[dd].y -= toCenterY;
	      }

	      tablePerspective = getPerspectiveTransform(srcQuad, dstQuad);

#ifdef DEBUG
//cout << endl << "tablePerspective, R: " << tablePerspective << R << endl;
//cout << srcQuad[0] << srcQuad[1] << srcQuad[2] << srcQuad[3] << endl; 
//cout << dstQuad[0] << dstQuad[1] << dstQuad[2] << dstQuad[3] << endl; 
#endif

	      rejectAll = 0;
	    }
	  }
	}

	//break;
      }
    }
    //if (!reject)
      //break;
  }
  
  init_oriented_filters();

  // draw it again to go over the brown boxes
  if (drawGray) {
    cv::Point outTop = cv::Point(grayTop.x, grayTop.y);
    cv::Point outBot = cv::Point(grayBot.x, grayBot.y);
    cv::Point inTop = cv::Point(grayTop.x+1,grayTop.y+1);
    cv::Point inBot = cv::Point(grayBot.x-1,grayBot.y-1);
    rectangle(cv_ptr->image, outTop, outBot, cv::Scalar(128,128,128));
    rectangle(cv_ptr->image, inTop, inBot, cv::Scalar(32,32,32));
  }

  vector< vector<int> > pIoCbuffer;

  // classify the crops
  roa_to_send_blue.objects.resize(bTops.size());
  ma_to_send_blue.markers.resize(bTops.size()+1);
  bKeypoints.resize(bTops.size());
  bWords.resize(bTops.size());
  bYCrCb.resize(bTops.size());
  bLabels.resize(bTops.size());
  for (int c = 0; c < bTops.size(); c++) {
#ifdef DEBUG
fprintf(stderr, " object check1"); fflush(stderr);
#endif
    vector<KeyPoint>& keypoints = bKeypoints[c];
    Mat descriptors;
    Mat descriptors2;

#ifdef DEBUG
fprintf(stderr, " a"); fflush(stderr);
cout << bTops[c] << bBots[c] << " "; cout.flush();
#endif

    Mat crop = original_cam_img(cv::Rect(bTops[c].x, bTops[c].y, bBots[c].x-bTops[c].x, bBots[c].y-bTops[c].y));
    Mat gray_image;
    Mat& yCrCb_image = bYCrCb[c];
    processImage(crop, gray_image, yCrCb_image, grayBlur);

    //detector->detect(gray_image, keypoints);
    gridKeypoints(imW, imH, bTops[c], bBots[c], gBoxStrideX, gBoxStrideY, keypoints, keypointPeriod);

    for (int kp = 0; kp < keypoints.size(); kp++) {
#ifdef DEBUG
//cout << keypoints[kp].angle << " " << keypoints[kp].class_id << " " << 
  //keypoints[kp].octave << " " << keypoints[kp].pt << " " <<
  //keypoints[kp].response << " " << keypoints[kp].size << endl;
#endif
    }

    bowExtractor->compute(gray_image, keypoints, descriptors, &pIoCbuffer);

    // save the word assignments for the keypoints so we can use them for red boxes
#ifdef DEBUG
fprintf(stderr, "e "); fflush(stderr);
cout << "pIoCbuffer: " << pIoCbuffer.size() << " "; cout.flush();
cout << "kpSize: " << keypoints.size() << " "; cout.flush();
#endif
    bWords[c].resize(keypoints.size());
    if ((pIoCbuffer.size() > 0) && (keypoints.size() > 0)) {
      for (int w = 0; w < vocabNumWords; w++) {
	int numDescrOfWord = pIoCbuffer[w].size();
#ifdef DEBUG
if (numDescrOfWord > 0)
  cout << "[" << w << "]: " << numDescrOfWord << " ";
#endif
	for (int w2 = 0; w2 < numDescrOfWord; w2++) {
	  bWords[c][pIoCbuffer[w][w2]] = w;
	}
      }
  
  #ifdef DRAW_BLUE_KEYPOINTS
      if (drawBlueKP) {
	for (int kp = 0; kp < keypoints.size(); kp++) {
	  int tX = keypoints[kp].pt.x;
	  int tY = keypoints[kp].pt.y;
	  cv::Point kpTop = cv::Point(bTops[c].x+tX-1,bTops[c].y+tY-1);
	  cv::Point kpBot = cv::Point(bTops[c].x+tX,bTops[c].y+tY);
	  if(
	    (kpTop.x >= 1) &&
	    (kpBot.x <= imW-2) &&
	    (kpTop.y >= 1) &&
	    (kpBot.y <= imH-2) 
	    ) {
	    rectangle(cv_ptr->image, kpTop, kpBot, cv::Scalar(255,0,0));
	  }
	}
      }
  #endif 

    }
    
#ifdef DEBUG
fprintf(stderr, " object check2"); fflush(stderr);
#endif
    double label = -1;
    if (!descriptors.empty() && !keypoints.empty()) {
    
      appendColorHist(yCrCb_image, keypoints, descriptors, descriptors2);

      label = kNN->find_nearest(descriptors2,k);
      bLabels[c] = label;

      if (classLabels[label].compare(invert_sign_name) == 0)
	invertQuaternionLabel = 1;
    }

#ifdef DEBUG
fprintf(stderr, " object check3 label %f", label); fflush(stderr);
#endif

    string labelName; 
    string augmentedLabelName;
    double poseIndex = -1;
    int winningO = -1;

    getOrientation(cv_ptr, img_cvt, 
      keypoints, descriptors, bTops[c], bBots[c], 
      label, labelName, augmentedLabelName, poseIndex, winningO);
    
  #ifdef SAVE_ANNOTATED_BOXES
    // save the crops
    if (fc > 0) {

    #ifdef CAPTURE_HARD_CLASS
      if ((0 != labelName.compare(table_label_class_name)) &&
	  (0 != labelName.compare(background_class_name) ) ) 
    #else
      if (0 == labelName.compare(class_name))
    #endif
      {

	string thisLabelName = labelName;

    #ifdef CAPTURE_HARD_CLASS
	thisLabelName = class_name;
    #endif


	{
	  string another_crops_path = data_directory + "/" + thisLabelName + "/";
	  char buf[1000];
	  sprintf(buf, "%s%s%s_%d.ppm", another_crops_path.c_str(), thisLabelName.c_str(), run_prefix.c_str(), cropCounter);
cout << buf << " " << bTops[c] << bBots[c] << original_cam_img.size() << crop.size() << endl;
	  imwrite(buf, crop);
	}

	Mat crop = original_cam_img(cv::Rect(bTops[c].x, bTops[c].y, bBots[c].x-bTops[c].x, bBots[c].y-bTops[c].y));
	char buf[1000];
	string this_crops_path = data_directory + "/" + thisLabelName + "Poses/";
	sprintf(buf, "%s%sPoses%s_%d.ppm", this_crops_path.c_str(), thisLabelName.c_str(), run_prefix.c_str(), cropCounter);
	//sprintf(buf, class_crops_path + "/%s_toAudit/%s%s_%d.ppm", 
	  //thisLabelName, thisLabelName, run_prefix, cropCounter);
	imwrite(buf, crop);
	cropCounter++;

	string poseLabelsPath = this_crops_path + "poseLabels.yml";
	char thisCropLabel[1000];
	sprintf(thisCropLabel, "%sPoses%s_%d", thisLabelName.c_str(), run_prefix.c_str(), cropCounter); 

	Eigen::Quaternionf quaternionToWrite = tableLabelQuaternion;
	if (invertQuaternionLabel) {
	  cv::Matx33f R;
	  R(0,0) = 1; R(0,1) =  0; R(0,2) = 0;
	  R(1,0) = 0; R(1,1) =  0; R(1,2) = 1;
	  R(2,0) = 0; R(2,1) = -1; R(2,2) = 0;

	  Eigen::Matrix3f rotation;
	  rotation << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2);
	  Eigen::Quaternionf tempQuaternion(rotation);
  
	  quaternionToWrite = tableLabelQuaternion * tempQuaternion;
	}

	cv::Vec <double,4>tLQ;
	tLQ[0] = quaternionToWrite.x();
	tLQ[1] = quaternionToWrite.y();
	tLQ[2] = quaternionToWrite.z();
	tLQ[3] = quaternionToWrite.w();

	FileStorage fsfO;
	fsfO.open(poseLabelsPath, FileStorage::APPEND);
	fsfO << thisCropLabel << tLQ; 
	fsfO.release();

cout << buf << " " << bTops[c] << bBots[c] << original_cam_img.size() << crop.size() << endl;
      } else {
cout << "Rejecting class " << labelName << endl;
      }


    }
  #endif

    if (drawLabels) {
      cv::Point text_anchor(bTops[c].x+1, bBots[c].y-2);
      cv::Point text_anchor2(bTops[c].x+1, bBots[c].y-2);
      putText(cv_ptr->image, augmentedLabelName, text_anchor, MY_FONT, 1.5, Scalar(255,192,192), 2.0);
      putText(cv_ptr->image, augmentedLabelName, text_anchor2, MY_FONT, 1.5, Scalar(255,0,0), 1.0);
    }

    double thisThresh = pBoxThresh;
    /*
    if (label == 1)
      thisThresh = gbPBT;
    if (label == 2)
      thisThresh = mbPBT;
    if (label == 3)
      thisThresh = wsPBT;
    if (label == 4)
      thisThresh = psPBT;
    */

#ifdef DEBUG
fprintf(stderr, " object check5"); fflush(stderr);
#endif

    vector<cv::Point> pointCloudPoints;
    getPointCloudPoints(cv_ptr, pointCloudPoints, pBoxIndicator, thisThresh, 
      bTops[c], bBots[c], imW, imH, gBoxStrideX, gBoxStrideY, gBoxW, gBoxH);

#ifdef DEBUG
fprintf(stderr, " object check6"); fflush(stderr);
#endif

  #ifdef PUBLISH_OBJECTS
    if (label >= 0) {
      fill_RO_and_M_arrays(roa_to_send_blue, 
	ma_to_send_blue, pointCloudPoints, c, label, winningO, poseIndex);
    }
  #endif
  }

  // publish the table
  {
#ifdef DEBUG
cout << "table check 1" << endl;
#endif
    int c = bTops.size();
    ma_to_send_blue.markers[c].pose = tablePose;
    ma_to_send_blue.markers[c].type =  visualization_msgs::Marker::CUBE;
    ma_to_send_blue.markers[c].scale.x = 1.0;
    ma_to_send_blue.markers[c].scale.y = 1.0;
    ma_to_send_blue.markers[c].scale.z = 0.002;
    ma_to_send_blue.markers[c].color.a = 0.5;
    ma_to_send_blue.markers[c].color.r = 176/255.0;
    ma_to_send_blue.markers[c].color.g = 133/255.0;
    ma_to_send_blue.markers[c].color.b = 14/255.0;

    ma_to_send_blue.markers[c].header =  roa_to_send_blue.header;
    ma_to_send_blue.markers[c].action = visualization_msgs::Marker::ADD;
    ma_to_send_blue.markers[c].id = c;
    ma_to_send_blue.markers[c].lifetime = ros::Duration(1.0);
#ifdef DEBUG
cout << "table check 2" << endl;
#endif
  }

  #ifdef PUBLISH_OBJECTS
#ifdef DEBUG
cout << "about to publish" << endl;
#endif
  if (bTops.size() > 0) {
    rec_objs_blue.publish(roa_to_send_blue);
  }
  markers_blue.publish(ma_to_send_blue);
#ifdef DEBUG
cout << "published" << endl;
#endif
  #endif


  #ifdef SAVE_ANNOTATED_BOXES
  if (fc > 0)
    fc--;
  #endif

  #ifdef RUN_TRACKING 
  object_recognition_msgs::RecognizedObjectArray roa_to_send_red;
  visualization_msgs::MarkerArray ma_to_send_red; 
  roa_to_send_red.objects.resize(0);
  ma_to_send_red.markers.resize(0);

  for (int r = 0; r < numRedBoxes; r++) {

#ifdef DEBUG
cout << "dealing with redBox[" << r << "]" << endl;
#endif

    redBox *thisRedBox = &(redBoxes[r]);
    int thisClass = thisRedBox->classLabel;

    int accepted = 0;
    // check to see if there is a class who wants this red box
    //   if so, scan it, if not, scan all
    for (int c = 0; c < bTops.size(); c++) {
      if (bLabels[c] == thisClass) {
	accepted = 1;
	thisRedBox->rootBlueBox = c;
      }
    }



// XXX
// check the old position of the redbox, keep its distance
// randomly alter the size of the test boxes, save the new size in that red box
// make feature computation efficient
// this should give interesting behavior

    int thisRedStride = redStride*redPeriod;
    int winJ = -1;
    float winD = 1e6;
    cv::Point winTop(0,0);
    cv::Point winBot(thisRedBox->bot.x, thisRedBox->bot.y);
    vector<KeyPoint> winKeypoints;
    Mat winDescriptors;

    cv::Point dTop;
    cv::Point dBot;

    if (thisRedBox->persistence < persistenceThresh) {
      redRounds = 1;
      //thisRedBox->bot = cv::Point(redInitialWidth, redInitialWidth);
    }

    for (int redR = 0; redR < redRounds; redR++) {

      int proposalValid = 0;
      int proposals = 0;

      //int deltaAmplitude = ((lrand48() % 3)*10);
      // "wide-scale random noise"
      int dmax = redDigitsWSRN;
      int digits = 1 + (lrand48() % (dmax));
      int deltaAmplitude = (1 << (digits-1)) + (lrand48() % (1 << (digits-1)));
      
      //int dmax = 5;
      //int digits = (lrand48() % dmax);
      //int deltaAmplitude = (1 << digits);
      //for (int d = 0; d < digits; d++)
	//deltaAmplitude += ((lrand48() % 2) << d);

      cv::Point rbDelta(0,0);
      cv::Point cornerDelta(0,0);

      double slideProb = slidesPerFrame / double(redRounds);
      int slideOrNot = (drand48() < slideProb);
      if (thisRedBox->persistence < persistenceThresh)
	slideOrNot = 1;

      while (!proposalValid && proposals < max_red_proposals) {
	proposals++;
	int deltaRnd = lrand48() % 6;
	int cornerRnd = lrand48() % 4;
	if (deltaRnd == 0) {
	  if (thisRedBox->bot.x+deltaAmplitude <= rbMaxWidth) {
	    rbDelta.x = deltaAmplitude;
	    proposalValid = 1;
	  }
	} else if (deltaRnd == 1) {
	  if (thisRedBox->bot.x-deltaAmplitude >= rbMinWidth) {
	    rbDelta.x = -deltaAmplitude;
	    proposalValid = 1;
	  }
	} else if (deltaRnd == 2) {
	  if (thisRedBox->bot.y+deltaAmplitude <= rbMaxWidth) {
	    rbDelta.y = deltaAmplitude;
	    proposalValid = 1;
	  }
	} else if (deltaRnd == 3) {
	  if (thisRedBox->bot.y-deltaAmplitude >= rbMinWidth) {
	    rbDelta.y = -deltaAmplitude;
	    proposalValid = 1;
	  }
	} else if (deltaRnd == 4) {
	  if ((thisRedBox->bot.y-deltaAmplitude >= rbMinWidth) &&
	      (thisRedBox->bot.x-deltaAmplitude >= rbMinWidth) ) {
	    rbDelta.y = -deltaAmplitude;
	    rbDelta.x = -deltaAmplitude;
	    proposalValid = 1;
	  }
	} else if (deltaRnd == 5) {
	  if ((thisRedBox->bot.y+deltaAmplitude <= rbMaxWidth) &&
	      (thisRedBox->bot.x+deltaAmplitude <= rbMaxWidth) ) {
	    rbDelta.y = deltaAmplitude;
	    rbDelta.x = deltaAmplitude;
	    proposalValid = 1;
	  }
	}

	if (cornerRnd == 0) {
	} else if (cornerRnd == 1) {
	  cornerDelta.x = -rbDelta.x;
	  cornerDelta.y = -rbDelta.y;
	} else if (cornerRnd == 2) {
	  cornerDelta.y = -rbDelta.y;
	} else if (cornerRnd == 3) {
	  cornerDelta.x = -rbDelta.x;
	}
      }
      
      vector<int> theseBlueBoxes;

      if (accepted) {
	theseBlueBoxes.push_back(thisRedBox->rootBlueBox);
      } else {
	int root_search = 0;
	if (root_search == 0) {
	  // search every box
	  for (int bb = 0; bb < bTops.size(); bb++) {
	    theseBlueBoxes.push_back(bb);
	  }
	} else if (root_search == 1) {
	  // search a random box
	  // consider removing
	  if (bTops.size() > 0)
	    theseBlueBoxes.push_back(lrand48() % bTops.size());
	}
      }

      for (int cc = 0; cc < theseBlueBoxes.size(); cc++) {
	int c = theseBlueBoxes[cc];
	cv::Point hTop = bTops[c];
	cv::Point hBot = bBots[c];

	if (!slideOrNot) {
	  hTop.x = thisRedBox->anchor.x + cornerDelta.x;
	  hTop.y = thisRedBox->anchor.y + cornerDelta.y;

	  hBot.x = thisRedBox->anchor.x + cornerDelta.x + thisRedBox->bot.x + rbDelta.x;
	  hBot.y = thisRedBox->anchor.y + cornerDelta.y + thisRedBox->bot.y + rbDelta.y;

	  hTop.x = min(hTop.x, bBots[c].x);
	  hTop.y = min(hTop.y, bBots[c].y);
	  hTop.x = max(hTop.x, bTops[c].x);
	  hTop.y = max(hTop.y, bTops[c].y);

	  hBot.x = min(hBot.x, bBots[c].x);
	  hBot.y = min(hBot.y, bBots[c].y);
	  hBot.x = max(hBot.x, bTops[c].x);
	  hBot.y = max(hBot.y, bTops[c].y);

	  if ((hBot.x-hTop.x < rbMinWidth) || (hBot.y-hTop.y < rbMinWidth)) {
#ifdef DEBUG
cout << "REJECTED class: " << thisClass << " bb: " << c << hTop << hBot << " prop: " << proposals << endl;
#endif
	    continue;
	  }
	}
	
	int ix = min(hTop.x + thisRedBox->bot.x + rbDelta.x, hBot.x);
	int iy = min(hTop.y + thisRedBox->bot.y + rbDelta.y, hBot.y);
	/*1*/cv::Point itTop(hTop.x,0);
	/*1*/cv::Point itBot(ix , 0);
	for (/*1*/; itBot.x <= hBot.x; /*2*/) {
	  
	  /*3*/itTop.y = hTop.y;
	  /*3*/itBot.y = iy;
	  for (/*3*/; itBot.y <= hBot.y; /*4*/) {
	    // score this crop
	    vector<KeyPoint> keypoints;
	    Mat descriptors(1,vocabNumWords, CV_32F);
	    Mat descriptors2;

	    Mat& yCrCb_image = bYCrCb[c];
#ifdef DEBUG
//cout << &(bYCrCb[c]) << endl;
#endif

	    cv::Point lItTop(itTop.x-bTops[c].x, itTop.y-bTops[c].y);
	    cv::Point lItBot(itBot.x-bTops[c].x, itBot.y-bTops[c].y);

	    float totalWords = bWords[c].size();
	    float countedWords = 0;
#ifdef DEBUG
//cout << "totalWords: " << totalWords;
#endif
	    for (int w = 0; w < totalWords; w++) {
	      int tX = bKeypoints[c][w].pt.x;
	      int tY = bKeypoints[c][w].pt.y;
	      // check for containment in this box
#ifdef DEBUG
//cout << " tX tY:" << tX << " " << tY << " " << itTop << itBot << lItTop << lItBot << hTop << hBot << endl;
#endif
	      if(
		(tX >= lItTop.x) &&
		(tX <= lItBot.x) &&
		(tY >= lItTop.y) &&
		(tY <= lItBot.y) 
		) {
#ifdef DEBUG
//cout << " w:" << w << " " << endl;
#endif
		descriptors.at<float>(bWords[c][w])++;
		keypoints.push_back(bKeypoints[c][w]);
		countedWords++;		
	#ifdef DRAW_RED_KEYPOINTS
		if (drawBlueKP) {
		  cv::Point kpTop = cv::Point(bTops[c].x+tX,bTops[c].y+tY);
		  cv::Point kpBot = cv::Point(bTops[c].x+tX+1,bTops[c].y+tY+1);
		  if(
		    (kpTop.x >= 1) &&
		    (kpBot.x <= imW-2) &&
		    (kpTop.y >= 1) &&
		    (kpBot.y <= imH-2) 
		    ) {
		    rectangle(cv_ptr->image, kpTop, kpBot, cv::Scalar(0,0,255));
		  }
		}
	#endif
	      }
	    }

	    Mat neighbors(1, redK, CV_32F);
	    Mat dist;
	    if (countedWords > 0) {
	      // normalize the BoW
	      for (int w = 0; w < vocabNumWords; w++) {
		descriptors.at<float>(w) = descriptors.at<float>(w) / countedWords;
	      }

	      appendColorHist(yCrCb_image, keypoints, descriptors, descriptors2);

	      //kNN->find_nearest(descriptors, redK, 0, 0, &neighbors, &dist);
	      kNN->find_nearest(descriptors2, redK, 0, 0, &neighbors, &dist);

	      int thisJ = 0;
	      float thisD = 1e6;
	      for (int n = 0; n < redK; n++) {
#ifdef DEBUG
cout << "  neighbors[" << n <<"] is " << (neighbors.at<float>(n));
#endif
		if (neighbors.at<float>(n) == float(thisClass)) {
		  thisJ++;
		  thisD = min(dist.at<float>(n), winD);
		}
	      }
#ifdef DEBUG
cout << " thisJ: " << thisJ << " slide: " << slideOrNot << " redR: " << redR << endl;
#endif
	      if (thisJ > 0 && thisD < winD) {
		winJ = thisJ;
		winD = thisD;
		winTop = itTop;
		winBot = itBot;
		winDescriptors = descriptors;
		winKeypoints = keypoints;
	      }

	      if (drawRB) {
		{
		  cv::Point outTop = cv::Point(dTop.x, dTop.y);
		  cv::Point outBot = cv::Point(dBot.x, dBot.y);
		  cv::Point inTop = cv::Point(dTop.x+1,dTop.y+1);
		  cv::Point inBot = cv::Point(dBot.x-1,dBot.y-1);
		  rectangle(cv_ptr->image, outTop, outBot, 0.5*cv::Scalar(64,64,192));
		  rectangle(cv_ptr->image, inTop, inBot, 0.5*cv::Scalar(160,160,224));
		}

		{
		  cv::Point outTop = cv::Point(itTop.x, itTop.y);
		  cv::Point outBot = cv::Point(itBot.x, itBot.y);
		  cv::Point inTop = cv::Point(winTop.x+1,winTop.y+1);
		  cv::Point inBot = cv::Point(winBot.x-1,winBot.y-1);
		  rectangle(cv_ptr->image, outTop, outBot, 0.5*cv::Scalar(0,0,255));
		  rectangle(cv_ptr->image, inTop, inBot, 0.5*cv::Scalar(192,192,255));
		}

		//{
		  //cv::Point outTop = cv::Point(winTop.x, winTop.y);
		  //cv::Point outBot = cv::Point(winBot.x, winBot.y);
		  //cv::Point inTop = cv::Point(winTop.x+1,winTop.y+1);
		  //cv::Point inBot = cv::Point(winBot.x-1,winBot.y-1);
		  //rectangle(cv_ptr->image, outTop, outBot, 0.5*cv::Scalar(0,0,255));
		  //rectangle(cv_ptr->image, inTop, inBot, 0.5*cv::Scalar(192,192,255));
		//}
	      }

	    }

#ifdef DEBUG
cout << "class: " << thisClass << " bb: " << c << " descriptors: " << keypoints.size() << " " 
  << itBot << itTop << "  " << hTop << hBot << " prop: " << proposals << endl;
#endif

	    /*4*/itTop.y += thisRedStride;
	    /*4*/itBot.y += thisRedStride;
	  }

	  /*2*/itTop.x += thisRedStride;
	  /*2*/itBot.x += thisRedStride;
	}
      }
      
      if (winD < thisRedBox->lastDistance) {
	if (thisRedBox->persistence > persistenceThresh)
	  thisRedBox->bot = cv::Point(redDecay*thisRedBox->bot.x + (1.0-redDecay)*(winBot.x - winTop.x), 
				      redDecay*thisRedBox->bot.y + (1.0-redDecay)*(winBot.y - winTop.y) );
	else
	  thisRedBox->bot = cv::Point((winBot.x - winTop.x), (winBot.y - winTop.y));
      }
      thisRedBox->lastDistance = winD;

      dTop = cv::Point(thisRedBox->anchor.x, thisRedBox->anchor.y);
      dBot = cv::Point(thisRedBox->anchor.x + thisRedBox->bot.x, thisRedBox->anchor.y + thisRedBox->bot.y);
    }



    string labelName; 
    string augmentedLabelName;
    double poseIndex = -1;
    int winningO = -1;

    getOrientation(cv_ptr, img_cvt, 
      winKeypoints, winDescriptors, winTop, winBot, 
      thisClass, labelName, augmentedLabelName, poseIndex, winningO);
    
    // always decay persistence but only add it if we got a hit
    thisRedBox->persistence = redDecay*thisRedBox->persistence;
    if (winD < 1e6) {
      if (thisRedBox->persistence > persistenceThresh) {
	thisRedBox->anchor.x = redDecay*thisRedBox->anchor.x + (1.0-redDecay)*winTop.x;
	thisRedBox->anchor.y = redDecay*thisRedBox->anchor.y + (1.0-redDecay)*winTop.y;
      } else {
	thisRedBox->anchor.x = winTop.x;
	thisRedBox->anchor.y = winTop.y;
	thisRedBox->persistence = persistenceThresh+.001;
      }
      thisRedBox->persistence = thisRedBox->persistence + (1.0-redDecay)*1.0;
    
      thisRedBox->poseIndex = poseIndex;
      thisRedBox->winningO = winningO;
    }

    if (drawRed) {
      //if (thisRedBox->persistence > 0.0) 
      {
	cv::Point outTop = cv::Point(dTop.x, dTop.y);
	cv::Point outBot = cv::Point(dBot.x, dBot.y);
	cv::Point inTop = cv::Point(dTop.x+1,dTop.y+1);
	cv::Point inBot = cv::Point(dBot.x-1,dBot.y-1);
	rectangle(cv_ptr->image, outTop, outBot, cv::Scalar(64,64,192));
	rectangle(cv_ptr->image, inTop, inBot, cv::Scalar(160,160,224));

	if (drawLabels) {
	  cv::Point text_anchor(dTop.x+1, dBot.y-2);
	  cv::Point text_anchor2(dTop.x+1, dBot.y-2);
	  putText(cv_ptr->image, augmentedLabelName, text_anchor, MY_FONT, 1.5, Scalar(160,160,224), 2.0);
	  putText(cv_ptr->image, augmentedLabelName, text_anchor2, MY_FONT, 1.5, Scalar(64,64,192), 1.0);
	}
      }

      {
	cv::Point outTop = cv::Point(winTop.x, winTop.y);
	cv::Point outBot = cv::Point(winBot.x, winBot.y);
	cv::Point inTop = cv::Point(winTop.x+1,winTop.y+1);
	cv::Point inBot = cv::Point(winBot.x-1,winBot.y-1);
	rectangle(cv_ptr->image, outTop, outBot, cv::Scalar(0,0,255));
	rectangle(cv_ptr->image, inTop, inBot, cv::Scalar(192,192,255));

	if (drawLabels) {
	  cv::Point text_anchor(winTop.x+1, winBot.y-2);
	  cv::Point text_anchor2(winTop.x+1, winBot.y-2);
	  putText(cv_ptr->image, augmentedLabelName, text_anchor, MY_FONT, 1.5, Scalar(192,192,255), 2.0);
	  putText(cv_ptr->image, augmentedLabelName, text_anchor2, MY_FONT, 1.5, Scalar(0,0,255), 1.0);
	}
      }
    }

    double thisThresh = pBoxThresh;

    #ifdef PUBLISH_OBJECTS
    vector<cv::Point> pointCloudPoints;
    getPointCloudPoints(cv_ptr, pointCloudPoints, pBoxIndicator, thisThresh, 
      dTop    , dBot    , imW, imH, gBoxStrideX, gBoxStrideY, gBoxW, gBoxH);

    if (thisClass >= 0) {
      roa_to_send_red.objects.resize(roa_to_send_red.objects.size()+1);
      ma_to_send_red.markers.resize(ma_to_send_red.markers.size()+1);

      fill_RO_and_M_arrays(roa_to_send_red, 
	ma_to_send_red, pointCloudPoints, roa_to_send_red.objects.size()-1, thisClass, thisRedBox->winningO, thisRedBox->poseIndex);
      
      thisRedBox->com.px = roa_to_send_red.objects[roa_to_send_red.objects.size()-1].pose.pose.pose.position.x;
      thisRedBox->com.py = roa_to_send_red.objects[roa_to_send_red.objects.size()-1].pose.pose.pose.position.y;
      thisRedBox->com.pz = roa_to_send_red.objects[roa_to_send_red.objects.size()-1].pose.pose.pose.position.z;
      thisRedBox->com.ox = 0.0;
      thisRedBox->com.oy = 0.0; 
      thisRedBox->com.oz = 0.0; 

      if (!isFiniteNumber(thisRedBox->com.px) ||
	  !isFiniteNumber(thisRedBox->com.py) ||
	  !isFiniteNumber(thisRedBox->com.pz) )
	thisRedBox->com = beeHome;
    }
    #endif
  }

    #ifdef PUBLISH_OBJECTS
  if (numRedBoxes > 0) {
    rec_objs_red.publish(roa_to_send_red);
    markers_red.publish(ma_to_send_red);
  }
    #endif

  #endif

#endif

#ifdef SAVE_BOXES
  // save the crops
  if (fc > 0) {
    fc--;
    for (int c = bTops.size()-1; c >= 0; c--) {
      Mat crop = original_cam_img(cv::Rect(bTops[c].x, bTops[c].y, bBots[c].x-bTops[c].x, bBots[c].y-bTops[c].y));
      char buf[1000];
      sprintf(buf, "%s%s%s_%d.ppm", saved_crops_path.c_str(), class_name.c_str(), run_prefix.c_str(), cropCounter);
      // uncomment if statement for hard negative mining
      //if(bLabels[c] != 7) {
cout << buf << " " << bTops[c] << bBots[c] << original_cam_img.size() << crop.size() << endl;
	imwrite(buf, crop);
	cropCounter++;
      //}
    }
  }
#endif

  if (drawBlue) {
    for (int c = bTops.size()-1; c >= 0; c--) {
      cv::Point outTop = cv::Point(bTops[c].x, bTops[c].y);
      cv::Point outBot = cv::Point(bBots[c].x, bBots[c].y);
      cv::Point inTop = cv::Point(bTops[c].x+1,bTops[c].y+1);
      cv::Point inBot = cv::Point(bBots[c].x-1,bBots[c].y-1);
      rectangle(cv_ptr->image, outTop, outBot, cv::Scalar(255,0,0));
      rectangle(cv_ptr->image, inTop, inBot, cv::Scalar(255,192,192));
    }
  }

  cv::imshow(objectViewerName, cv_ptr->image);
  cv::imshow(densityViewerName, img_cvt);

  delete pBoxIndicator;
  delete gBoxIndicator;
  delete gBoxGrayNodes;
  delete gBoxComponentLabels;
  delete integralDensity;
  delete density;
  delete differentialDensity;

  img_cvt.release();
  img_cvt_blur.release();
  img_cvtG.release();
  img_cvtGtmp.release();
  img_cvtH.release();

  int kc = cv::waitKey(1);
  handleKeyboardInput(kc);
  saveROSParams();
}

/*
void tableCallback(const object_recognition_msgs::Table& msg)
{
#ifdef DEBUG
  cout << "Hit tableCloudCallback" << endl;
#endif
  tablePose = msg.pose;
}
*/

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::fromROSMsg(*msg, pointCloud);
#ifdef DEBUG
cout << "Hit pointCloudCallback" <<  "  " << pointCloud.size() << endl;
#endif
}

void clusterCallback(const visualization_msgs::MarkerArray& msg){
	if(real_img){
		object_recognition_msgs::RecognizedObjectArray to_send;
		to_send.objects.resize(msg.markers.size());
		//MatrixXf imfea2;
		//VectorXf scores;
		IplImage temp = cam_img;
		int height = cam_img.size().height;
		int width = cam_img.size().width;
		ROS_INFO("(%d,%d)", height, width);
		float minx, miny, maxx, maxy, px, py;
		cv::Rect bound;
		cv::Mat boxed;
		geometry_msgs::Point p;
		wTop.resize(msg.markers.size());
		wBot.resize(msg.markers.size());
		ROS_INFO("Objects found: %d", int(msg.markers.size()));
		float f;
		std::string res;
		//if(LOC_MODEL_TYPE == 0) f= 580;
		//else f=525;
		f = 525;
		float cx = (width/2) - 0.5;
		float cy = (height/2) - 0.5;
		for(int i=0; i<msg.markers.size(); i++){
			minx = f*(msg.markers[i].points[0].x/msg.markers[i].points[0].z) + cx;
			maxx = f*(msg.markers[i].points[0].x/msg.markers[i].points[0].z) + cx;
			miny = f*(msg.markers[i].points[0].y/msg.markers[i].points[0].z) + cy;
			maxy = f*(msg.markers[i].points[0].y/msg.markers[i].points[0].z) + cy;
			for(int j = 0; j < msg.markers[i].points.size(); j++){
				p = msg.markers[i].points[j];
				px = f*(p.x/p.z) + cx;
				py = f*(p.y/p.z) + cy;
				if(px < minx){minx = px;}
				if(px > maxx){maxx = px;}
				if(py < miny){miny = py;}
				if(py > maxy){maxy = py;}
			}
			wTop[i] = cv::Point(minx - 5, miny - 5);
			wBot[i] = cv::Point(maxx + 5, maxy + 5);
			if(wTop[i].x < 0){wTop[i].x = 0;}
			else if(wTop[i].x > width - 1){wTop[i].x = width-1;}
			if(wTop[i].y < 0) wTop[i].y = 0;
			else if(wTop[i].y > height-1){wTop[i].y = height-1;}
			if(wBot[i].x < 0){wBot[i].x = 0;}
			else if(wBot[i].x > width - 1){wBot[i].x = width-1;}
			if(wBot[i].y < 0) wBot[i].y = 0;
			else if(wBot[i].y > height-1){wBot[i].y = height-1;}
		}/*
		for(int i=0; i<wTop.size(); i++){
			ROS_INFO("BOXED: (%d, %d) to (%d, %d)", wTop[i].x, wTop[i].y, wBot[i].x, wBot[i].y);
			bound = cv::Rect(wTop[i].x, wTop[i].y, wBot[i].x - wTop[i].x, wBot[i].y - wTop[i].y);
			boxed = cam_img(bound);
			temp = boxed;
			kdm->Process(imfea2, &temp);
			kdm->Classify(scores, imfea2);
			res = kdm->GetObjectName(scores);
			ROS_INFO("Tabletop %d identified as %s", i, res.c_str());
			meldon_detection::RecognizedObject to_add;
			to_add.points = msg.markers[i].points;
			to_add.name = res.c_str();
			to_send.objects[i] = to_add;
		}*/
		temp = cam_img;
		//kdm->Process(imfea2, &temp);
		//kdm->Classify(scores, imfea2);
		//rec_objs.publish(to_send);
	}
	ROS_INFO("Identification complete");
}


int main(int argc, char **argv) {

#ifdef RELEARN_VOCAB
  retrain_vocab = 1;
#endif 
#ifdef RELOAD_DATA
  reextract_knn = 1;
#endif
#ifdef REWRITE_LABELS
  rewrite_labels = 1;
#endif 

  srand(time(NULL));
  time(&firstTime);

  tableLabelQuaternion.x() = 0;
  tableLabelQuaternion.y() = 0;
  tableLabelQuaternion.z() = 0;
  tableLabelQuaternion.w() = 1;

  gBoxStrideX = gBoxW / 2.0;
  gBoxStrideY = gBoxH / 2.0;

  string bufstr; // Have a buffer string

  cout << "argc: " << argc << endl;
  for (int ccc = 0; ccc < argc; ccc++) {
    cout << argv[ccc] << endl;
  }
  cout << "argc: " << argc << endl;

  string programName;
  if (argc > 1) {
    programName = string(PROGRAM_NAME) + "_" + argv[argc-1];
    cout << programName << endl;
    //argc = argc-1;
    //argv[argc-1] = argv[argc-2];
  }
  else
    programName = string(PROGRAM_NAME);

  ros::init(argc, argv, programName);
  ros::NodeHandle n("~");
  std::string s;

  cout << "n namespace: " << n.getNamespace() << endl;

  cout << "all_range_mode: " << all_range_mode << endl;
  loadROSParamsFromArgs();
  cout << "mask_gripper: " << mask_gripper << " add_blinders: " << add_blinders << endl;
  cout << "all_range_mode: " << all_range_mode << endl;
  cout << endl << "numRedBoxes: " << numRedBoxes << endl;
  cout << "data_directory: " << data_directory << endl << "class_name: " << class_name << endl 
       << "run_prefix: " << run_prefix << endl << "class_pose_models: " << class_pose_models << endl 
       << "class_labels: " << class_labels << endl << "vocab_file: " << vocab_file << endl 
       << "knn_file: " << knn_file << endl << "label_file: " << label_file << endl
       << endl;
//exit(0);

  package_path = ros::package::getPath("node");
  class_crops_path = data_directory + "/";

  // The other models
  //ObjNessB2W8MAXBGR
  //ObjNessB2W8I
  //ObjNessB2W8HSV
  bing_trained_models_path = package_path + "/bing_trained_models/";
  objectness_matrix_path = bing_trained_models_path + "ObjNessB2W8I.idx.yml";
  objectness_path_prefix = bing_trained_models_path + "ObjNessB2W8MAXBGR";


  string vocPath = package_path + "/VOC2007/";
  //DataSetVOC voc("../VOC2007/");
  DataSetVOC voc(vocPath);
  //Objectness objNess(voc, 2, 8, 2);
  //glObjectness = &(objNess);
  glObjectness = new Objectness(voc, 2, 8, 2);

  printf("objectness_path_prefix: %s\n", objectness_path_prefix.c_str());
  //int result = objNess.loadTrainedModel(objectness_path_prefix);
  int result = glObjectness->loadTrainedModel(objectness_path_prefix);
  cout << "result: " << result << endl << endl;

  image_transport::Subscriber image_sub;
  image_transport::ImageTransport it(n);
  image_sub = it.subscribe(image_topic, 1, imageCallback);
  ros::Subscriber points = n.subscribe(pc_topic, 1, pointCloudCallback);

  ros::Subscriber clusters = n.subscribe("/tabletop/clusters", 1, clusterCallback);

  rec_objs_blue = n.advertise<object_recognition_msgs::RecognizedObjectArray>("blue_labeled_objects", 10);
  rec_objs_red = n.advertise<object_recognition_msgs::RecognizedObjectArray>("red_labeled_objects", 10);
  markers_blue = n.advertise<visualization_msgs::MarkerArray>("blue_object_markers", 10);
  markers_red = n.advertise<visualization_msgs::MarkerArray>("red_object_markers", 10);

  ee_target_pub = n.advertise<geometry_msgs::Point>("pilot_target_" + left_or_right_arm, 10);

  densityViewerName = "Density Viewer " + left_or_right_arm;
  objectViewerName = "Object Viewer " + left_or_right_arm;

  cv::namedWindow(densityViewerName);
  cv::namedWindow(objectViewerName);
  setMouseCallback(objectViewerName, CallbackFunc, NULL);

  createTrackbar("canny_lo", densityViewerName, &loTrackbarVariable, 100);
  createTrackbar("canny_hi", densityViewerName, &hiTrackbarVariable, 100);
  createTrackbar("blinder_columns", densityViewerName, &blinder_columns, 20);
  createTrackbar("blinder_stride", densityViewerName, &blinder_stride, 50);

  fc = 0;
  cropCounter = 0;
  tableNormal = Eigen::Vector3d(1,0,0);
  tableBias = 0;

  int numClasses = 0;

#ifdef RUN_INFERENCE

  // SIFT 
  //detector = new SiftFeatureDetector(0, 3, 0.04, 10, 1.6);
  detector = new FastFeatureDetector(4);
  extractor = new SiftDescriptorExtractor();

  // BOW time
  bowtrainer = new BOWKMeansTrainer(vocabNumWords);

  // read the class image data
  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  char buf[1024];

  char vocabularyPath[1024];
  char featuresPath[1024];
  char labelsPath[1024];
  sprintf(vocabularyPath, "%s/%s", data_directory.c_str(), vocab_file.c_str());
  sprintf(featuresPath, "%s/%s", data_directory.c_str(), knn_file.c_str());
  sprintf(labelsPath, "%s/%s", data_directory.c_str(), label_file.c_str());
  cout << "vocabularyPath: " << vocabularyPath << endl;
  cout << "featuresPath: " << featuresPath << endl;
  cout << "labelsPath: " << labelsPath << endl;


#ifdef TRAIN_ONLY
  stringstream ss_cl(class_labels); 
  while (ss_cl >> bufstr)
    classLabels.push_back(bufstr);

  stringstream ss_cpm(class_pose_models); 
  while (ss_cpm >> bufstr)
    classPoseModels.push_back(bufstr);

  cout << "Num labels: " << classLabels.size() << endl;
  cout << "Num pose models: " << classPoseModels.size() << endl;

  if ((classLabels.size() != classPoseModels.size()) || (classLabels.size() < 1)) {
    cout << "label or pose model problem. exiting." << endl;
    exit(0);
  }
#endif

  int numNewClasses = classLabels.size();
  int numCachedClasses = 0;

  if (rewrite_labels) {
    // load cached labels 
    vector<string> classCacheLabels;
    vector<string> classCachePoseModels;
    if (cache_prefix.size() > 0) {
      string labelsCacheFile = data_directory + "/" + cache_prefix + "labels.yml";

      FileStorage fsvI;
      cout<<"Reading CACHED labels and pose models..."<< endl << labelsCacheFile << endl << "...";
      fsvI.open(labelsCacheFile, FileStorage::READ);
      fsvI["labels"] >> classCacheLabels;
      fsvI["poseModels"] >> classCachePoseModels;
      //classLabels.insert(classLabels.end(), classCacheLabels.begin(), classCacheLabels.end());
      //classPoseModels.insert(classPoseModels.end(), classCachePoseModels.begin(), classCachePoseModels.end());
      cout << "done. Cache : " << classCacheLabels.size() << " " << classCachePoseModels.size() << " total: " ;
      numCachedClasses = classCacheLabels.size();

      classCacheLabels.insert(classCacheLabels.end(), classLabels.begin(), classLabels.end());
      classCachePoseModels.insert(classCachePoseModels.end(), classPoseModels.begin(), classPoseModels.end());
      classLabels = classCacheLabels;
      classPoseModels = classCachePoseModels;
      cout << classLabels.size() << " " << classPoseModels.size() << endl;
    }

    FileStorage fsvO;
    cout<<"Writing labels and pose models..."<< endl << labelsPath << endl << "...";
    fsvO.open(labelsPath, FileStorage::WRITE);
    fsvO << "labels" << classLabels;
    fsvO << "poseModels" << classPoseModels;
    fsvO.release();
    cout << "done." << endl;
  } else {
    FileStorage fsvI;
    cout<<"Reading labels and pose models..."<< endl << labelsPath << endl << "...";
    fsvI.open(labelsPath, FileStorage::READ);
    fsvI["labels"] >> classLabels;
    fsvI["poseModels"] >> classPoseModels;
    cout << "done. " << classLabels.size() << " " << classPoseModels.size() << endl;
  }

  for (int i = 0; i < classLabels.size(); i++) {
    cout << classLabels[i] << " " << classPoseModels[i] << endl;
  }

  // this is the total number of classes, so it is counted after the cache is dealt with
  numClasses = classLabels.size();

  Mat vocabulary;

  if (retrain_vocab) {
    for (int i = 0; i < classLabels.size(); i++) {
      cout << "Getting BOW features for class " << classLabels[i] 
	   << " with pose model " << classPoseModels[i] << " index " << i << endl;
      bowGetFeatures(class_crops_path, classLabels[i].c_str(), grayBlur);
      if (classPoseModels[i].compare("G") == 0) {
	string thisPoseLabel = classLabels[i] + "Poses";
	bowGetFeatures(class_crops_path, thisPoseLabel.c_str(), grayBlur);
      }
    }

    cout << "Clustering features..." << endl;
    vocabulary = bowtrainer->cluster();
    cout << "done." << endl;

    FileStorage fsvO;
    cout<<"Writing vocab..."<< endl << vocabularyPath << endl << "...";
    fsvO.open(vocabularyPath, FileStorage::WRITE);
    fsvO << "vocab" << vocabulary;
    fsvO.release();
    cout << "done." << endl;
  } else {
    FileStorage fsvI;
    cout<<"Reading vocab..."<< endl << vocabularyPath << endl << "...";
    fsvI.open(vocabularyPath, FileStorage::READ);
    fsvI["vocab"] >> vocabulary;
    cout << "done." << vocabulary.size() << endl;
  }

  matcher = new BFMatcher(NORM_L2);
  bowExtractor = new BOWImgDescriptorExtractor(extractor,matcher);
  bowExtractor->setVocabulary(vocabulary);

  Mat kNNfeatures;
  Mat kNNlabels;

  classPosekNNs.resize(numClasses);
  classPosekNNfeatures.resize(numClasses);
  classPosekNNlabels.resize(numClasses);
  classQuaternions.resize(numClasses);

  if (reextract_knn) {
    //for (int i = 0; i < numNewClasses; i++) 
    for (int i = numCachedClasses; i < numClasses; i++) 
    {
      cout << "Getting kNN features for class " << classLabels[i] 
	   << " with pose model " << classPoseModels[i] << " index " << i << endl;
      kNNGetFeatures(class_crops_path, classLabels[i].c_str(), i, grayBlur, kNNfeatures, kNNlabels);
      if (classPoseModels[i].compare("G") == 0) {
	string thisPoseLabel = classLabels[i] + "Poses";
	posekNNGetFeatures(class_crops_path, thisPoseLabel.c_str(), grayBlur, classPosekNNfeatures[i], classPosekNNlabels[i],
	  classQuaternions[i], 0);
      }
    }
    // XXX TODO warning, classQuaternions are not loaded as they should be

    // load cached kNN features 
    // XXX does not handle G pose models
    Mat kNNCachefeatures;
    Mat kNNCachelabels;
    if (cache_prefix.size() > 0) {
      string knnCacheFile = data_directory + "/" + cache_prefix + "knn.yml";

      FileStorage fsfI;
      cout<<"Reading CACHED features..."<< endl << knnCacheFile << endl << "...";
      fsfI.open(knnCacheFile, FileStorage::READ);
      fsfI["features"] >> kNNCachefeatures;
      fsfI["labels"] >> kNNCachelabels;
      kNNfeatures.push_back(kNNCachefeatures);
      kNNlabels.push_back(kNNCachelabels);
      cout << "done." << kNNCachefeatures.size() << " " << kNNCachelabels.size() << endl;
    }

    FileStorage fsfO;
    cout<<"Writing features and labels..."<< endl << featuresPath << endl << "...";
    fsfO.open(featuresPath, FileStorage::WRITE);
    fsfO << "features" << kNNfeatures;
    fsfO << "labels" << kNNlabels;

    // TODO should also cache the features for the pose models

    for (int i = 0; i < numClasses; i++) {
      if (classPoseModels[i].compare("G") == 0) {
	string fnOut = "features" + classLabels[i];
	string lnOut = "labels" + classLabels[i];
	string qnOut = "quaternions" + classLabels[i];
	cout << "G: " << classLabels[i] << " " << fnOut << " " << lnOut << endl;
	fsfO << fnOut << classPosekNNfeatures[i];
	fsfO << lnOut << classPosekNNlabels[i];
	fsfO << qnOut << classQuaternions[i];
      }
    }
    fsfO.release();
    cout << "done." << endl;
  } else { 
    FileStorage fsfI;
    cout<<"Reading features and labels..."<< endl << featuresPath << endl << "...";
    fsfI.open(featuresPath, FileStorage::READ);
    fsfI["features"] >> kNNfeatures;
    fsfI["labels"] >> kNNlabels;
    for (int i = 0; i < numClasses; i++) {
      if (classPoseModels[i].compare("G") == 0) {
	string fnIn = "features" + classLabels[i];
	string lnIn = "labels" + classLabels[i];
	string qnIn = "quaternions" + classLabels[i];
	cout << "G: " << classLabels[i] << " " << fnIn << " " << lnIn << endl;
	fsfI[fnIn] >> classPosekNNfeatures[i];
	fsfI[lnIn] >> classPosekNNlabels[i];
	fsfI[qnIn] >> classQuaternions[i];
      }
    }
    cout << "done." << kNNfeatures.size() << " " << kNNlabels.size() << endl;
  }

  cout << kNNlabels.size().height << " " << kNNlabels.size().width << endl;
  cout << kNNfeatures.size().height << " " << kNNfeatures.size().width << endl;

  cout << "Main kNN..." << endl;
  kNN = new CvKNearest(kNNfeatures, kNNlabels);
  cout << "Done" << endl;
  for (int i = 0; i < numClasses; i++) {
    if (classPoseModels[i].compare("G") == 0) {
      cout << "Class " << i << " kNN..." << classPosekNNfeatures[i].size() << classPosekNNlabels[i].size() << endl;
      classPosekNNs[i] = new CvKNearest(classPosekNNfeatures[i], classPosekNNlabels[i]);
      cout << "Done" << endl;
    }
  }

#endif

#ifdef TRAIN_ONLY
  exit(0);
#endif

  // manually definining spoon filters
  orientedFilters = new Mat[ORIENTATIONS];
  orientedFilters[0].create(O_FILTER_WIDTH, O_FILTER_WIDTH, CV_64F);

  tablePerspective = Mat::eye(3,3,CV_32F);
  init_oriented_filters();

#ifdef RUN_TRACKING
  // initialize the redBoxes
  stringstream ss_rb(red_box_list); 
  while (ss_rb >> bufstr)
    redBoxLabels.push_back(bufstr);

  int redBoxProposals = redBoxLabels.size();
  redBoxes = new redBox[redBoxProposals];

  for (int r = 0; r < redBoxProposals; r++) {
    int thisClassLabel = -1;
    for (int i = 0; i < numClasses; i++) {
      if (!classLabels[i].compare(redBoxLabels[r])) {
	thisClassLabel = i;
      }
    }

    if (thisClassLabel > -1) {
      numRedBoxes++;
      cout << "accepting red box suggestion " << r << " \"" << redBoxLabels[r] << "\" as red box number " << numRedBoxes-1 << endl;
      redBoxes[r].classLabel = thisClassLabel;
      redBoxes[r].top = cv::Point(0,0);
      redBoxes[r].bot = cv::Point(redInitialWidth, redInitialWidth);
      redBoxes[r].rootBlueBox = 0;
      redBoxes[r].numGreenBoxes;
      redBoxes[r].anchor = cv::Point(0,0);
      redBoxes[r].persistence = 0;
      redBoxes[r].lastDistance = FLT_MAX;
      redBoxes[r].poseIndex = 0;
      redBoxes[r].winningO = 0;
    } else {
      cout << "rejecting red box suggestion " << r << " \"" << redBoxLabels[r] << "\"" << endl;
    }
  }

#endif

  if (numRedBoxes > 0)
    createTrackbar("red target", "Object Viewer", &redTrackbarVariable, numRedBoxes);

  saveROSParams();




  // don't be a fool... lock that spool
  ros::spin();

  // multithreaded spinning causes what is probably a race condition...
  //ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  //spinner.spin(); // spin() will not return until the node has been shutdown

  //ros::AsyncSpinner spinner(4); // Use 4 threads
  //spinner.start();
  //ros::waitForShutdown();

  return 0;
}



