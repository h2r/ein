//// these macros below were moved to other files
//#define RUN_INFERENCE // generates the blue boxes
//#define PUBLISH_OBJECTS // requires blue, brown, and gray boxes, and inference
//#define DRAW_LABEL
//#define DRAW_ORIENTOR
//#define RELOAD_DATA
//#define RELEARN_VOCAB
//#define LEARN_ONLY
//// these macros above were moved to other files

//#define RUN_TRACKING // calculates the red boxes

#define DRAW_WHITE
#define DRAW_GREEN
#define DRAW_BLUE // depends on green boxes
#define DRAW_RED // depends on blue boxes
#define DRAW_GRAY
//#define DRAW_PINK // depends on blue boxes
#define DRAW_BROWN // depends on blue and gray boxes, inference, and pointcloud configuration

#define DRAW_RED_KEYPOINTS

const int k = 4;
int redK = 1;
int numRedBoxes = 3;
//int gBoxThreshMultiplier = 1.1;
double gBoxThresh = 3;
double pBoxThresh = 5;
double threshFraction = 0.35;
double densityPower = 1.0;//1.0/4.0;

// pink box thresholds for the principle classes
double psPBT = 0.0;//5.0;
double wsPBT = 0.0;//6.5;
double gbPBT = 0.0;//6.0;
double mbPBT = 0.0;//7.0;

// adjust these to reject blue boxes
double rejectScale = 2.0;
double rejectAreaScale = 6*6;

// gray box offset from the top and bottom of the screen
int tGO = 40;
int bGO = 140;

// point cloud affine calibration
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
const int vocabNumWords = 1000;
const double grayBlur = 1.0;

const double colorHistNumBins = 8;
const double colorHistBinWidth = 256/colorHistNumBins;
const double colorHistLambda = 1.0;
const double colorHistThresh = 0.1;
const int colorHistBoxHalfWidth = 1;

int fcRange = 10;

double densityDecay = 0.7;
double depthDecay = 0.7;
double redDecay = 0.9;

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

#include <sstream>
#include <iostream>
#include <math.h>
#include <string>

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

vector<string> classLabels; 
vector<string> classPoseModels;
vector<CvKNearest*> classPosekNNs;
vector<Mat> classPosekNNfeatures;
vector<Mat> classPosekNNlabels;

int retrain_vocab = 0;
int reextract_knn = 0;
int rewrite_labels = 0;
std::string class_list = "unspecified_class_list";


//KernelDescManager* kdm;
static unsigned int LOC_MODEL_TYPE=0; //0 or 3

cv::Mat cam_img;
cv::Mat depth_img;
ros::Publisher rec_objs;
ros::Publisher markers;
bool real_img = false;
bool vis = true;

// Top variables are top left corners of bounding boxes (smallest coordinates)
// Bot variables are bottom right corners of bounding boxes (largenst coordinates)
// Cen cariables are centers of bounding boxes
cv::vector<cv::Point> wTop;
cv::vector<cv::Point> wBot;
cv::vector<cv::Point> nTop;
cv::vector<cv::Point> nBot;

Objectness *glObjectness;
int fc;
int cropCounter;

double *temporalDensity = NULL;
double *temporalDepth = NULL;


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
pcl::PointCloud<pcl::PointXYZRGB> pointCloud;

geometry_msgs::Pose tablePose;

#define MY_FONT FONT_HERSHEY_PLAIN

#define ORIENTATIONS 36 
#define O_FILTER_WIDTH 25
#define O_FILTER_SPOON_HEAD_WIDTH 6 
#define O_FILTER_SPOON_SHAFT_WIDTH 1
Mat *orientedFilters;
int biggestL1 = 0;

typedef struct {
  int classLabel;
  cv::Point top;
  cv::Point bot;
  int rootBlueBox;
  int numGreenBoxes;
  cv::Point anchor;
  double persistence;
  double lastDistance;
} redBox;

redBox *redBoxes;

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


void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
  if ( event == EVENT_LBUTTONDOWN ) {
#ifdef CAPTURE_ONLY
    fc = 0;
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

        vector<KeyPoint> keypoints;
        Mat descriptors;

        char filename[1024];
        sprintf(filename, "%s%s/%s", classDir.c_str(), className, epdf->d_name);
        Mat image;
        image = imread(filename);
        Mat gray_image;
        cvtColor(image, gray_image, CV_BGR2GRAY);
        GaussianBlur(gray_image, gray_image, cv::Size(0,0), sigma);

        detector->detect(gray_image, keypoints);
        extractor->compute(gray_image, keypoints, descriptors);

        cout << className << ":  "  << epdf->d_name << "  " << descriptors.size() << " " << endl;

        if (!descriptors.empty())
          bowtrainer->add(descriptors);
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

        char filename[1024];
        sprintf(filename, "%s%s/%s", classDir.c_str(), className, epdf->d_name);
        Mat image;
        image = imread(filename);

	Size sz = image.size();
	int imW = sz.width;
	int imH = sz.height;

        Mat gray_image;
        Mat yCrCb_image;
        cvtColor(image, gray_image, CV_BGR2GRAY);
        cvtColor(image, yCrCb_image, CV_BGR2YCrCb);
        GaussianBlur(gray_image, gray_image, cv::Size(0,0), sigma);
        GaussianBlur(yCrCb_image, yCrCb_image, cv::Size(0,0), sigma);

        detector->detect(gray_image, keypoints);
        bowExtractor->compute(gray_image, keypoints, descriptors);

        cout << className << ":  "  << epdf->d_name << "  " << descriptors.size() << " type: " << descriptors.type() << endl;

        if (!descriptors.empty() && !keypoints.empty()) {
	
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

	  Mat descriptors2 = Mat(1, descriptors.size().width + colorHistNumBins*colorHistNumBins, descriptors.type());
	  for (int i = 0; i < descriptors.size().width; i++) 
	    descriptors2.at<float>(i) = descriptors.at<float>(i);
	  for (int i = 0; i < colorHistNumBins*colorHistNumBins; i++) {
	    //if ( colorHist.at<float>(i) > 0.1 )
	      //cout << i << ":" << colorHist.at<float>(i) << " ";
	    colorHist.at<float>(i) = min(colorHist.at<float>(i), float(colorHistThresh));
	    descriptors2.at<float>(i+descriptors.size().width) = colorHistLambda * colorHist.at<float>(i);
	  }
//exit(0);
/*
*/

          kNNfeatures.push_back(descriptors2);
          kNNlabels.push_back(label);
        }
      }
    }
  }
}

void posekNNGetFeatures(std::string classDir, const char *className, double sigma, Mat &kNNfeatures, Mat &kNNlabels) {

  string sClassName(className);

  int label = 0;

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

	string fileName(epdf->d_name);

	string poseIndex = fileName.substr(sClassName.size()+1, string::npos);
	poseIndex = poseIndex.substr(0,  poseIndex.length()-4);
	label = std::atoi(poseIndex.c_str());

        vector<KeyPoint> keypoints;
        Mat descriptors;

        char filename[1024];
        sprintf(filename, "%s%s/%s", classDir.c_str(), className, epdf->d_name);
        Mat image;
        image = imread(filename);

	Size sz = image.size();
	int imW = sz.width;
	int imH = sz.height;

        Mat gray_image;
        Mat yCrCb_image;
        cvtColor(image, gray_image, CV_BGR2GRAY);
        cvtColor(image, yCrCb_image, CV_BGR2YCrCb);
        GaussianBlur(gray_image, gray_image, cv::Size(0,0), sigma);
        GaussianBlur(yCrCb_image, yCrCb_image, cv::Size(0,0), sigma);

        detector->detect(gray_image, keypoints);
        bowExtractor->compute(gray_image, keypoints, descriptors);

        cout << className << ":  "  << epdf->d_name << "  "  << fileName << " " << descriptors.size() << 
	  " type: " << descriptors.type() << " label: " << label << endl;

        if (!descriptors.empty() && !keypoints.empty()) {
	
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

	  Mat descriptors2 = Mat(1, descriptors.size().width + colorHistNumBins*colorHistNumBins, descriptors.type());
	  for (int i = 0; i < descriptors.size().width; i++) 
	    descriptors2.at<float>(i) = descriptors.at<float>(i);
	  for (int i = 0; i < colorHistNumBins*colorHistNumBins; i++) {
	    //if ( colorHist.at<float>(i) > 0.1 )
	      //cout << i << ":" << colorHist.at<float>(i) << " ";
	    colorHist.at<float>(i) = min(colorHist.at<float>(i), float(colorHistThresh));
	    descriptors2.at<float>(i+descriptors.size().width) = colorHistLambda * colorHist.at<float>(i);
	  }
//exit(0);
/*
*/

          //kNNfeatures.push_back(descriptors2);
          kNNfeatures.push_back(descriptors);
          kNNlabels.push_back(label);
	  //label++;
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
      //cout << " " << float(depth_img.at<uint>(y,x));
    }
  }
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      uint tDepth = depth_img.at<unsigned short>(y,x);
      temporalDepth[y*imW+x] = depthDecay*temporalDepth[y*imW+x] + (1.0-depthDecay)*float(tDepth);
      //cout << " " << float(depth_img.at<uint>(y,x));
      if (tDepth <= minDepth)
	depth_img.at<short>(y,x) = 0;
      //else
	//depth_img.at<uint>(y,x) = tDepth - minDepth;
    }
  }

  maxDepth = max(maxDepth,uint(1));

  cv::imshow("Depth Viewer", depth_img*20);
  cout << depth_img.size() << " " << depth_img.type() << " " << maxDepth << " " << minDepth << endl;
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

void loadROSParamsFromArgs()
{
  ros::NodeHandle nh("~");

  nh.getParam("vocab_file", vocab_file);
  nh.getParam("knn_file", knn_file);
  nh.getParam("label_file", label_file);

  nh.getParam("data_directory", data_directory);
  nh.getParam("class_labels", class_labels);
  nh.getParam("class_pose_models", class_pose_models);

  nh.getParam("class_name", class_name);
  nh.getParam("run_prefix", run_prefix);

  saved_crops_path = data_directory + "/" + class_name + "/";
}

void loadROSParams()
{
  ros::NodeHandle nh("~");

  nh.getParam("number_red_boxes", numRedBoxes);
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
  nh.getParam("frame_count_range", fcRange);
  nh.getParam("density_decay", densityDecay);
  nh.getParam("depth_decay", depthDecay);
  nh.getParam("red_decay", redDecay);
  nh.getParam("data_directory", data_directory);
  nh.getParam("class_labels", class_labels);
  nh.getParam("class_pose_models", class_pose_models);
  nh.getParam("class_name", class_name);
  nh.getParam("run_prefix", run_prefix);

  saved_crops_path = data_directory + "/" + class_name + "/";
}

void saveROSParams()
{
  ros::NodeHandle nh("~");

  nh.setParam("/camera/number_red_boxes", numRedBoxes);
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
  nh.setParam("frame_count_range", fcRange);
  nh.setParam("density_decay", densityDecay);
  nh.setParam("depth_decay", depthDecay);
  nh.setParam("red_decay", redDecay);
  nh.setParam("data_directory", data_directory);
  nh.setParam("class_labels", class_labels);
  nh.setParam("class_pose_models", class_pose_models);
  nh.setParam("class_name", class_name);
  nh.setParam("run_prefix", run_prefix);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
  loadROSParams();
  cv::Rect bound;
  cv::Mat boxed;

  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cam_img = cv_ptr->image;
    real_img = true;
  }catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  object_recognition_msgs::RecognizedObjectArray roa_to_send;
  visualization_msgs::MarkerArray ma_to_send; 

  int boxesPerSize = 800;
  
  // XXX find best method and stop all this cloning, it might be relatively slow
  Mat original_cam_img = cam_img.clone();

  Mat img_cvt = cam_img.clone();
  Mat img_cvtH = cam_img.clone();
  Mat img_cvtG = cam_img.clone();
  Mat img_cvtGtmp = cam_img.clone();

  Mat img_cvt_blur = cam_img.clone();
  //img_cvt.copyTo(cv_ptr->image);

  cvtColor(cam_img, img_cvtGtmp, CV_RGB2GRAY);
  cvtColor(cam_img, img_cvtH, CV_RGB2HSV);

  vector<Mat> channels;
  channels.push_back(img_cvtGtmp);
  channels.push_back(img_cvtGtmp);
  channels.push_back(img_cvtGtmp);
  merge(channels, img_cvtG);

  // input image is noisy so blurring is a good idea
  //GaussianBlur(img_cvt, img_cvt, cv::Size(0,0), 1.0);

  ValStructVec<float, Vec4i> boxes;
  //glObjectness->getObjBndBoxes(cam_img, boxes, boxesPerSize);
  glObjectness->getObjBndBoxes(img_cvt, boxes, boxesPerSize);


  int numBoxes = boxes.size();
  // box[0] minx box[1] miny box[2] maxx box[3] maxy
  cout << numBoxes << "    " << fc <<  endl;

  nTop.resize(numBoxes);
  nBot.resize(numBoxes);

  int boxesToConsider= 50000;

  //fc = (fc + 1) % fcRange;

  Size sz = img_cvt.size();
  int imW = sz.width;
  int imH = sz.height;
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

  if(vis){
    // draw the ork bounding boxes
    for(int i =0; i<wTop.size(); i++){
#ifdef DRAW_WHITE
      rectangle(cv_ptr->image, wTop[i], wBot[i], cv::Scalar(255,255,255));
#endif
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
      //rectangle(cv_ptr->image, nTop[i], nBot[i], cv::Scalar(0,255,0));
      
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

  // now update the exponential average of the density
  // and set the density to be a thresholded version of this
  //double threshFraction = 0.35;
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
  int top_gray_bottom = 0 + tGO;
  int bottom_gray_top = imH - bGO;

#ifdef DRAW_GRAY
  rectangle(cv_ptr->image, cv::Point(0,0), cv::Point(imW-1, top_gray_bottom), cv::Scalar(128,128,128));
  rectangle(cv_ptr->image, cv::Point(0,bottom_gray_top), cv::Point(imW-1, imH-1), cv::Scalar(128,128,128));
#endif

  // truncate the density above the gray boxes
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < top_gray_bottom; y++) {
      density[y*imW+x] = 0;
    }
  }

  for (int x = 0; x < imW; x++) {
    for (int y = bottom_gray_top; y < imH; y++) {
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

  int gBoxW = 10;
  int gBoxH = 10;
  int gBoxStrideX = ceil(float(gBoxW / 2.0));
  int gBoxStrideY = ceil(float(gBoxH / 2.0));

  double *gBoxIndicator = new double[imW*imH];
  double *gBoxGrayNodes = new double[imW*imH];
  double *gBoxComponentLabels = new double[imW*imH];
  double *pBoxIndicator = new double[imW*imH];

  vector<cv::Point> cTops; 
  vector<cv::Point> cBots;

  vector<int> parentX;
  vector<int> parentY;
  vector<int> parentD;

  const int directionX[] = {1, 0, -1,  0};
  const int directionY[] = {0, 1,  0, -1};

  int total_components = 0;

  for (int x = 0; x < imW-gBoxW; x+=gBoxStrideX) {
    for (int y = 0; y < imH-gBoxH; y+=gBoxStrideY) {

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

      double thisIntegral = integralDensity[yb*imW+xb]-integralDensity[yb*imW+xt]-integralDensity[yt*imW+xb]+integralDensity[yt*imW+xt];
      if (thisIntegral > gBoxThresh) {
	      gBoxIndicator[y*imW+x] = 1;
#ifdef DRAW_GREEN
	      rectangle(cv_ptr->image, thisTop, thisBot, cv::Scalar(0,255,0));
#endif
      }
      pBoxIndicator[y*imW+x] = thisIntegral;

    }
  }


  for (int x = 0; x < imW-gBoxW; x+=gBoxStrideX) {
    for (int y = 0; y < imH-gBoxH; y+=gBoxStrideY) {
      if (gBoxIndicator[y*imW+x] == 1 && gBoxGrayNodes[y*imW+x] == 0) {

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
      	  else if(gBoxIndicator[nextY*imW+nextX] == 1 && gBoxGrayNodes[nextY*imW+nextX] == 0
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

  // create the blue boxes from the parental green boxes
  vector<cv::Point> bTops; 
  vector<cv::Point> bBots;
  vector<cv::Point> bCens;
  // track the half widths and half heights of the blue boxes for the merge step later
  vector<int> bHWs;
  vector<int> bHHs;

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
      bHWs.push_back(bCens[t].x-bTops[t].x);
      bHHs.push_back(bCens[t].y-bTops[t].y);
    }
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
  int brownBoxWidth = 2*gBoxW;//50;
  int brownPadding = 10;
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

  for (int bY = top_gray_bottom + brownPadding; 
	bY < bottom_gray_top-brownBoxWidth-brownPadding; bY=bY+(brownBoxWidth/2)) {
    for (int bX = brownPadding; bX < imW-brownBoxWidth-brownPadding; bX = bX+(brownBoxWidth/2)) {
      cv::Point thisCen = cv::Point(bX+brownBoxWidth/2, bY+brownBoxWidth/2);

      thisBrTop.x = bX;
      thisBrTop.y = bY;
      thisBrBot.x = bX+brownBoxWidth;
      thisBrBot.y = bY+brownBoxWidth;

      int reject = 0;
      for (int c = 0; c < bTops.size(); c++) {
	//cout << "brBox   " << c << " / " << bTops.size() << " " << fabs(bCens[c].x - thisCen.x) << endl;
	if ( fabs(bCens[c].x - thisCen.x) < 1+((fabs(bBots[c].x-bTops[c].x)+brownBoxWidth)/2) && 
	      fabs(bCens[c].y - thisCen.y) < 1+((fabs(bBots[c].y-bTops[c].y)+brownBoxWidth)/2) ) {
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
	    rectangle(cv_ptr->image, thisBrTop, thisBrBot, cv::Scalar(200,200,200));
	    #endif
	  }

//cout << thisBrTop << thisBrBot << "p0, p1, p2:  " << p0 << p1 << p2 << endl;
//cout << "p0, p1, p2:  " << p0 << p1 << p2 << endl;

	  if (!reject) {
	    cv::Point tranTop = pcCorrection(thisBrTop.x, thisBrTop.y, imW, imH);
	    cv::Point tranBot = pcCorrection(thisBrBot.x, thisBrBot.y, imW, imH);

cout << "t " << thisBrTop << thisBrBot << endl << "  " << tranTop << tranBot << endl;

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

	      #ifdef DRAW_BROWN
	      rectangle(cv_ptr->image, thisBrTop, thisBrBot, cv::Scalar(0,50,200));
	      #endif
	    }

	    if (!reject) {
	      acceptedBrBoxes++;
	      #ifdef DRAW_BROWN
	      rectangle(cv_ptr->image, thisBrTop, thisBrBot, cv::Scalar(0,51,102));
	      #endif

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

cout << tableNormal << endl;

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

  vector< vector<KeyPoint> > bKeypoints;
  vector< vector<int> > bWords;
  vector< vector<int> > pIoCbuffer;
  vector<Mat> bYCrCb;

  vector<int> bLabels;
  // classify the crops
  roa_to_send.objects.resize(bTops.size());
  ma_to_send.markers.resize(bTops.size()+1);
  bWords.resize(bTops.size());
  bKeypoints.resize(bTops.size());
  bYCrCb.resize(bTops.size());
  for (int c = 0; c < bTops.size(); c++) {
fprintf(stderr, " object check1"); fflush(stderr);
    vector<KeyPoint>& keypoints = bKeypoints[c];
    Mat descriptors;

fprintf(stderr, " a"); fflush(stderr);

    Mat crop = original_cam_img(cv::Rect(bTops[c].x, bTops[c].y, bBots[c].x-bTops[c].x, bBots[c].y-bTops[c].y));
    Mat gray_image;
    Mat& yCrCb_image = bYCrCb[c];
fprintf(stderr, "b"); fflush(stderr);
    cvtColor(crop, gray_image, CV_BGR2GRAY);
    cvtColor(crop, yCrCb_image, CV_BGR2YCrCb);
fprintf(stderr, "c"); fflush(stderr);
    GaussianBlur(gray_image, gray_image, cv::Size(0,0), grayBlur);
    GaussianBlur(yCrCb_image, yCrCb_image, cv::Size(0,0), grayBlur);
fprintf(stderr, "d"); fflush(stderr);

    detector->detect(gray_image, keypoints);
    bowExtractor->compute(gray_image, keypoints, descriptors, &pIoCbuffer);
fprintf(stderr, "e "); fflush(stderr);
cout << "pIoCbuffer: " << pIoCbuffer.size() < " "; cout.flush();
cout << "kpSize: " << keypoints.size() < " "; cout.flush();
    bWords[c].resize(keypoints.size());
    if ((pIoCbuffer.size() > 0) && (keypoints.size() > 0))
      for (int w = 0; w < vocabNumWords; w++) {
	int numDescrOfWord = pIoCbuffer[w].size();
if (numDescrOfWord > 0)
cout << "[" << w << "]: " << numDescrOfWord << " ";
	for (int w2 = 0; w2 < numDescrOfWord; w2++) {
	  bWords[c][pIoCbuffer[w][w2]] = w;
	}
      }
    
    int crH = bBots[c].y-bTops[c].y;
    int crW = bBots[c].x-bTops[c].x;

fprintf(stderr, " object check2"); fflush(stderr);
    double label = -1;
    double poseIndex = -1;
    if (!descriptors.empty() && !keypoints.empty()) {
    
      Mat colorHist(1, colorHistNumBins*colorHistNumBins, descriptors.type());
      for (int i = 0; i < colorHistNumBins*colorHistNumBins; i++) 
	colorHist.at<float>(i) = 0;

      double numPix = 0;
      // traverse all of the keypoints
      for (int kk = 0; kk < keypoints.size(); kk++) {
	// count pixels in a neighborhood of the keypoint
	int yMin = max(int(keypoints[kk].pt.y)-colorHistBoxHalfWidth,0);
	int yMax = min(int(keypoints[kk].pt.y)+colorHistBoxHalfWidth,crH);
	int xMin = max(int(keypoints[kk].pt.x)-colorHistBoxHalfWidth,0);
	int xMax = min(int(keypoints[kk].pt.x)+colorHistBoxHalfWidth,crW);
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

      Mat descriptors2 = Mat(1, descriptors.size().width + colorHistNumBins*colorHistNumBins, descriptors.type());
      for (int i = 0; i < descriptors.size().width; i++) 
	descriptors2.at<float>(i) = descriptors.at<float>(i);
      for (int i = 0; i < colorHistNumBins*colorHistNumBins; i++) {
	colorHist.at<float>(i) = min(colorHist.at<float>(i), float(colorHistThresh));
	descriptors2.at<float>(i+descriptors.size().width) = colorHistLambda * colorHist.at<float>(i);
      }

      label = kNN->find_nearest(descriptors2,k);
      if (0 == classPoseModels[label].compare("G")) {
	poseIndex = classPosekNNs[label]->find_nearest(descriptors,k);
      }
    }

    char labelName[256]; 
    if (label == -1)
      sprintf(labelName, "VOID");
    else
      sprintf(labelName, "%s", classLabels[label].c_str());

    bLabels.push_back(label);
  
  #ifdef SAVE_ANNOTATED_BOXES
    // save the crops
    if (fc == 0) {
      fc = 1;
      Mat crop = original_cam_img(cv::Rect(bTops[c].x, bTops[c].y, bBots[c].x-bTops[c].x, bBots[c].y-bTops[c].y));
      char buf[1000];
      sprintf(buf, class_crops_path + "/%s_toAudit/%s%s_%d.ppm", 
	labelName, labelName, run_prefix, cropCounter);
      imwrite(buf, crop);
      cropCounter++;
    }
  #endif
fprintf(stderr, " object check3 label %f", label); fflush(stderr);

    int winningO = -1;
    if (label >= 0) 
      if (0 == classPoseModels[label].compare("S")) {

  fprintf(stderr, " object checkS"); fflush(stderr);

	Mat gCrop = img_cvt(cv::Rect(bTops[c].x, bTops[c].y, bBots[c].x-bTops[c].x, bBots[c].y-bTops[c].y));
	cv::resize(gCrop, gCrop, orientedFilters[0].size());
	gCrop.convertTo(gCrop, orientedFilters[0].type());

	Mat gcChannels[3];
	split(gCrop, gcChannels);

	double winningScore = -1;
	for (int o = 0; o < ORIENTATIONS; o++) {
	  double thisScore = gcChannels[1].dot(orientedFilters[o]);
	  if (thisScore > winningScore) {
	    winningScore = thisScore;
	    winningO = o;
	  }
	}

	#ifdef DRAW_ORIENTOR
	Mat vCrop = cv_ptr->image(cv::Rect(bTops[c].x, bTops[c].y, bBots[c].x-bTops[c].x, bBots[c].y-bTops[c].y));
	vCrop = vCrop.mul(0.5);

	Mat scaledFilter;
	cv::resize(orientedFilters[winningO], scaledFilter, vCrop.size());
	scaledFilter = biggestL1*scaledFilter;
	 
	vector<Mat> channels;
	channels.push_back(Mat::zeros(scaledFilter.size(), scaledFilter.type()));
	channels.push_back(Mat::zeros(scaledFilter.size(), scaledFilter.type()));
	channels.push_back(scaledFilter);
	merge(channels, scaledFilter);

	scaledFilter.convertTo(scaledFilter, vCrop.type());
	vCrop = vCrop + 128*scaledFilter;
	#endif
      }
fprintf(stderr, " object check4"); fflush(stderr);

    string augmentedLabelName = labelName;
    if (label >= 0) 
      if (0 == classPoseModels[label].compare("G")) {
	string result;
	ostringstream convert;
	convert << poseIndex;
	result = convert.str();
	augmentedLabelName = augmentedLabelName + " " + result;
      }
  #ifdef DRAW_LABEL
    cv::Point text_anchor(bTops[c].x, bBots[c].y);
    putText(cv_ptr->image, augmentedLabelName, text_anchor, MY_FONT, 1.5, Scalar(255,0,0), 2.0);
  #endif

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

fprintf(stderr, " object check5"); fflush(stderr);
    vector<cv::Point> pointCloudPoints;
    for (int x = bTops[c].x; x <= bBots[c].x-gBoxW; x+=gBoxStrideX) {
      for (int y = bTops[c].y; y <= bBots[c].y-gBoxH; y+=gBoxStrideY) {
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
	  //double bcX = min(max(double(x)+pcbcX+(double(gBoxW)/2.0),0.0),double(imW));
	  //double bcY = min(max(double(y)+pcbcY+(double(gBoxH)/2.0),0.0),double(imH));
	  //double bgcX = pcgc11*bcX + pcgc12*bcY;
	  //double bgcY = pcgc21*bcX + pcgc22*bcY;
	  //pcl::PointXYZRGB pcp = pointCloud.at(floor(bgcX), floor(bgcY));


	  cv::Point transformedPixel = pcCorrection( 
	    double(x)+(double(gBoxW)/2.0), double(y)+(double(gBoxH)/2.0), imW, imH);
	  pcl::PointXYZRGB pcp = pointCloud.at(transformedPixel.x, transformedPixel.y);

	  Eigen::Vector3d pinkPoint(pcp.x,pcp.y,pcp.z);
	  reject = 1;
	  if (pinkPoint.dot(tableNormal) >= tableBias + tableBiasMargin)
	    reject = 0;
	}

	if (!reject) {
    #ifdef DRAW_PINK
      	  rectangle(cv_ptr->image, thisTop, thisBot, cv::Scalar(100,100,255));
    #endif
      	  pointCloudPoints.push_back(cv::Point(x,y));
      	}
      }
    }


fprintf(stderr, " object check6"); fflush(stderr);


  #ifdef PUBLISH_OBJECTS
    if (label >= 0) {
cout << "check" << endl;
cout << "hit a publishable object " << label << " " << classLabels[label] 
<< " " << classPoseModels[label] << c << " of total objects" << bTops.size() << endl;

      geometry_msgs::Pose object_pose;

      // XXX obtain the quaternion pose for the table
      Eigen::Quaternionf tableQuaternion;
      tableQuaternion.x() = tablePose.orientation.x;
      tableQuaternion.y() = tablePose.orientation.y;
      tableQuaternion.z() = tablePose.orientation.z;
      tableQuaternion.w() = tablePose.orientation.w;

      cv::Matx33f R;
      R(0,0) = 1; R(0,1) = 0; R(0,2) = 0;
      R(1,0) = 0; R(1,1) = 1; R(1,2) = 0;
      R(2,0) = 0; R(2,1) = 0; R(2,2) = 1;

      // handle the rotation differently depending on the class
      // if we have a spoon
      //if (label == 3 || label == 4) {
      if (0 == classPoseModels[label].compare("S")) {
      	double theta = (M_PI / 2.0) + (winningO*2*M_PI/ORIENTATIONS);
      	R(0,0) = cos(theta); R(0,1) = -sin(theta); R(0,2) = 0;
      	R(1,0) = sin(theta); R(1,1) =  cos(theta); R(1,2) = 0;
      	R(2,0) = 0;          R(2,1) = 0;           R(2,2) = 1;
      }
cout << "constructing rotation matrix" << endl;

      Eigen::Matrix3f rotation;
      rotation << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2);
      Eigen::Quaternionf objectQuaternion(rotation);

      objectQuaternion = tableQuaternion * objectQuaternion;

      roa_to_send.objects[c].pose.pose.pose.orientation.x = objectQuaternion.x();
      roa_to_send.objects[c].pose.pose.pose.orientation.y = objectQuaternion.y();
      roa_to_send.objects[c].pose.pose.pose.orientation.z = objectQuaternion.z();
      roa_to_send.objects[c].pose.pose.pose.orientation.w = objectQuaternion.w();

      // XXX fill out the point cloud using the vector<cv::Point>> pointCloudPoints.

      // XXX determine the x,y,z coordinates of the object from the point cloud
      // this bounding box has top left  bTops[x] and bBots[c]
cout << "dealing with point cloud" << " of size " << pointCloud.size() << endl;
      if (pointCloud.size() > 0) {
	pcl::PointCloud<pcl::PointXYZRGB> object_cloud;
	getCluster(object_cloud, pointCloud, pointCloudPoints);
	geometry_msgs::Pose pose = getPose(object_cloud);
	roa_to_send.objects[c].point_clouds.resize(1);
	pcl::toROSMsg(object_cloud, roa_to_send.objects[c].point_clouds[0]);
	roa_to_send.objects[c].pose.pose.pose.position = pose.position;
      } else {
	roa_to_send.objects[c].pose.pose.pose.position = object_pose.position;
      }

      ma_to_send.markers[c].pose = roa_to_send.objects[c].pose.pose.pose;

      roa_to_send.header.stamp = ros::Time::now();
      roa_to_send.header.frame_id = "/camera_rgb_optical_frame";

      roa_to_send.objects[c].header = roa_to_send.header;
      //roa_to_send.objects[c].point_clouds[0].header = roa_to_send.header;
      //roa_to_send.objects[c].pose.header = roa_to_send.header;

      roa_to_send.objects[c].type.key = augmentedLabelName;

      if (0 == classPoseModels[label].compare("B")) {
	ma_to_send.markers[c].type =  visualization_msgs::Marker::SPHERE;
	ma_to_send.markers[c].scale.x = 0.15;
	ma_to_send.markers[c].scale.y = 0.15;
	ma_to_send.markers[c].scale.z = 0.15;
	ma_to_send.markers[c].color.a = 1.0;
	ma_to_send.markers[c].color.r = 0.9;
	ma_to_send.markers[c].color.g = 0.9;
	ma_to_send.markers[c].color.b = 0.0;
      } else if (0 == classPoseModels[label].compare("S")) {
	ma_to_send.markers[c].type =  visualization_msgs::Marker::CUBE;
	ma_to_send.markers[c].scale.x = 0.2;
	ma_to_send.markers[c].scale.y = 0.02;
	ma_to_send.markers[c].scale.z = 0.02;
	ma_to_send.markers[c].color.a = 1.0;
	ma_to_send.markers[c].color.r = 0.25;
	ma_to_send.markers[c].color.g = 0.25;
	ma_to_send.markers[c].color.b = 0.25;
      }
      ma_to_send.markers[c].header =  roa_to_send.header;
      ma_to_send.markers[c].action = visualization_msgs::Marker::ADD;

      char labelName[256]; 
      /*
      if (label == 0)
	sprintf(labelName, "VOID");
      if (label == 1) {
	sprintf(labelName, "gyroBowl");
	ma_to_send.markers[c].type =  visualization_msgs::Marker::SPHERE;
	ma_to_send.markers[c].scale.x = 0.15;
	ma_to_send.markers[c].scale.y = 0.15;
	ma_to_send.markers[c].scale.z = 0.15;
	ma_to_send.markers[c].color.a = 1.0;
	ma_to_send.markers[c].color.r = 0.9;
	ma_to_send.markers[c].color.g = 0.9;
	ma_to_send.markers[c].color.b = 0.0;

	ma_to_send.markers[c].header =  roa_to_send.header;
	ma_to_send.markers[c].action = visualization_msgs::Marker::ADD;
      }
      if (label == 2) {
	sprintf(labelName, "mixBowl");
	ma_to_send.markers[c].type =  visualization_msgs::Marker::SPHERE;
	ma_to_send.markers[c].scale.x = 0.17;
	ma_to_send.markers[c].scale.y = 0.17;
	ma_to_send.markers[c].scale.z = 0.17;
	ma_to_send.markers[c].color.a = 1.0;
	ma_to_send.markers[c].color.r = 0.8;
	ma_to_send.markers[c].color.g = 0.8;
	ma_to_send.markers[c].color.b = 0.8;

	ma_to_send.markers[c].header =  roa_to_send.header;
	ma_to_send.markers[c].action = visualization_msgs::Marker::ADD;
      }
      if (label == 3) {
	sprintf(labelName, "woodSpoon");
	ma_to_send.markers[c].type =  visualization_msgs::Marker::CUBE;
	ma_to_send.markers[c].scale.x = 0.2;
	ma_to_send.markers[c].scale.y = 0.02;
	ma_to_send.markers[c].scale.z = 0.02;
	ma_to_send.markers[c].color.a = 1.0;
	ma_to_send.markers[c].color.r = 0.80;
	ma_to_send.markers[c].color.g = 0.80;
	ma_to_send.markers[c].color.b = 0.50;

	ma_to_send.markers[c].header =  roa_to_send.header;
	ma_to_send.markers[c].action = visualization_msgs::Marker::ADD;
      }
      if (label == 4) {
	sprintf(labelName, "plasticSpoon");
	ma_to_send.markers[c].type =  visualization_msgs::Marker::CUBE;
	ma_to_send.markers[c].scale.x = 0.2;
	ma_to_send.markers[c].scale.y = 0.02;
	ma_to_send.markers[c].scale.z = 0.02;
	ma_to_send.markers[c].color.a = 1.0;
	ma_to_send.markers[c].color.r = 0.25;
	ma_to_send.markers[c].color.g = 0.25;
	ma_to_send.markers[c].color.b = 0.25;

	ma_to_send.markers[c].header =  roa_to_send.header;
	ma_to_send.markers[c].action = visualization_msgs::Marker::ADD;
      }
      if (label == 5)
	sprintf(labelName, "background");
      if (label == 6)
	sprintf(labelName, "human");
      if (label == 7)
	sprintf(labelName, "sippyCup");
      */

      if (label == -1)
	sprintf(labelName, "VOID");
      else
	sprintf(labelName, "%s", classLabels[label].c_str());

      ma_to_send.markers[c].header =  roa_to_send.header;
      ma_to_send.markers[c].action = visualization_msgs::Marker::ADD;
      ma_to_send.markers[c].id = c;
      ma_to_send.markers[c].lifetime = ros::Duration(1.0);
    }
  #endif

    if (label >= 0) 
      cout << "  finished a publishable object " << label << " " << classLabels[label] << " " << classPoseModels[label] << endl;
  }

  // publish the table
  {
cout << "table check 1" << endl;
    int c = bTops.size();
    ma_to_send.markers[c].pose = tablePose;
    ma_to_send.markers[c].type =  visualization_msgs::Marker::CUBE;
    ma_to_send.markers[c].scale.x = 1.0;
    ma_to_send.markers[c].scale.y = 1.0;
    ma_to_send.markers[c].scale.z = 0.002;
    ma_to_send.markers[c].color.a = 0.5;
    ma_to_send.markers[c].color.r = 176/255.0;
    ma_to_send.markers[c].color.g = 133/255.0;
    ma_to_send.markers[c].color.b = 14/255.0;

    ma_to_send.markers[c].header =  roa_to_send.header;
    ma_to_send.markers[c].action = visualization_msgs::Marker::ADD;
    ma_to_send.markers[c].id = c;
    ma_to_send.markers[c].lifetime = ros::Duration(1.0);
cout << "table check 2" << endl;
  }

  #ifdef PUBLISH_OBJECTS
  cout << "about to publish" << endl;
  if (bTops.size() > 0) {
    rec_objs.publish(roa_to_send);
  }
  markers.publish(ma_to_send);
  cout << "published" << endl;
  #endif

  #ifdef RUN_TRACKING 
  for (int r = 0; r < numRedBoxes; r++) {

    cout << "dealing with redBox[" << r << "]" << endl;

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


    int redStride = 10;

// XXX
// check the old position of the redbox, keep its distance
// randomly alter the size of the test boxes, save the new size in that red box
// make feature computation efficient
// this should give interesting behavior

    int winJ = -1;
    float winD = 1e6;

    int proposalValid = 0;

    int deltaAmplitude = ((lrand48() % 2)*10);

    int rbMinWidth = 50;
    int rbMaxWidth = 500;
    cv::Point rbDelta(0,0);

    while (!proposalValid) {
      int deltaRnd = lrand48() % 6;
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
    }
    
    cv::Point winTop(0,0);
    cv::Point winBot(thisRedBox->bot.x, thisRedBox->bot.y);
    
    if (accepted) {

      int thisRootBlueBox = thisRedBox->rootBlueBox;
      
      cv::Point hTop = bTops[thisRootBlueBox];
      cv::Point hBot = bBots[thisRootBlueBox];
      int crW = hBot.x - hTop.x;
      int crH = hBot.y - hTop.y;

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
	  
	  /*
	  Mat descriptors;

	  Mat gray_image;
	  cvtColor(crop, gray_image, CV_BGR2GRAY);
	  GaussianBlur(gray_image, gray_image, cv::Size(0,0), grayBlur);

	  detector->detect(gray_image, keypoints);
	  bowExtractor->compute(gray_image, keypoints, descriptors);
	  */
      
	  /*
	  Mat crop = original_cam_img(cv::Rect(itTop.x, itTop.y, itBot.x-itTop.x, itBot.y-itTop.y));
	  Mat yCrCb_image;
	  cvtColor(crop, yCrCb_image, CV_BGR2YCrCb);
	  GaussianBlur(yCrCb_image, yCrCb_image, cv::Size(0,0), grayBlur);
	  */

	  Mat& yCrCb_image = bYCrCb[thisRootBlueBox];

	  cv::Point lItTop(itTop.x-hTop.x, itTop.y-hTop.y);
	  cv::Point lItBot(itBot.x-hTop.x, itBot.y-hTop.y);

	  Mat descriptors(1,vocabNumWords, CV_32F);
	  float totalWords = bWords[thisRootBlueBox].size();
	  float countedWords = 0;
//cout << "totalWords: " << totalWords;
	  for (int w = 0; w < totalWords; w++) {
	    int tX = bKeypoints[thisRootBlueBox][w].pt.x;
	    int tY = bKeypoints[thisRootBlueBox][w].pt.y;
	    // check for containment in this box
//cout << " tX tY:" << tX << " " << tY << " " << itTop << itBot << lItTop << lItBot << hTop << hBot << endl;
	    if(
	      (tX >= lItTop.x) &&
	      (tX <= lItBot.x) &&
	      (tY >= lItTop.y) &&
	      (tY <= lItBot.y) 
	      ) {
//cout << " w:" << w << " " << endl;
	      descriptors.at<float>(bWords[thisRootBlueBox][w])++;
	      keypoints.push_back(bKeypoints[thisRootBlueBox][w]);
	      countedWords++;
	#ifdef DRAW_RED_KEYPOINTS
	      rectangle(cv_ptr->image, cv::Point(hTop.x+tX,hTop.y+tY), 
		cv::Point(hTop.x+tX+1,hTop.y+tY+1), cv::Scalar(0,0,255));
	#endif
	
	    }
	  }
	  if (countedWords > 0)
	    for (int w = 0; w < vocabNumWords; w++) {
	      descriptors.at<float>(w) = descriptors.at<float>(w) / countedWords;
	    }

	  Mat neighbors(1, redK, CV_32F);
	  Mat dist;
	  if (!descriptors.empty()) {
	    Mat colorHist(1, colorHistNumBins*colorHistNumBins, descriptors.type());
	    /**/
	    for (int i = 0; i < colorHistNumBins*colorHistNumBins; i++) 
	      colorHist.at<float>(i) = 0;

	    double numPix = 0;
	    // traverse all of the keypoints
	    for (int kk = 0; kk < keypoints.size(); kk++) {
	      // count pixels in a neighborhood of the keypoint
	      int yMin = max(int(keypoints[kk].pt.y)-colorHistBoxHalfWidth,0);
	      int yMax = min(int(keypoints[kk].pt.y)+colorHistBoxHalfWidth,crH);
	      int xMin = max(int(keypoints[kk].pt.x)-colorHistBoxHalfWidth,0);
	      int xMax = min(int(keypoints[kk].pt.x)+colorHistBoxHalfWidth,crW);
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
	    /**/

	    Mat descriptors2 = Mat(1, descriptors.size().width + colorHistNumBins*colorHistNumBins, descriptors.type());
	    for (int i = 0; i < descriptors.size().width; i++) 
	      descriptors2.at<float>(i) = descriptors.at<float>(i);

	    /**/
	    for (int i = 0; i < colorHistNumBins*colorHistNumBins; i++) {
	      colorHist.at<float>(i) = min(colorHist.at<float>(i), float(colorHistThresh));
	      descriptors2.at<float>(i+descriptors.size().width) = colorHistLambda * colorHist.at<float>(i);
	    }
	    /**/

	    //kNN->find_nearest(descriptors, redK, 0, 0, &neighbors, &dist);
	    kNN->find_nearest(descriptors2, redK, 0, 0, &neighbors, &dist);

	    int thisJ = 0;
	    float thisD = 1e6;
	    // XXX this business is a little mucked up, redK=1 has funny behavior
	    for (int n = 0; n < redK; n++) 
	    //for (int n = 0; n < 1; n++) 
	    {
	      cout << "  neighbors[" << n <<"] is " << (neighbors.at<float>(n)) << endl;
	      if (neighbors.at<float>(n) == float(thisClass)) { 
		thisJ++;
		thisD = min(dist.at<float>(n), winD);
	      }
	    }
	    cout << thisJ << endl;
	    if (thisJ > 0 && thisD < winD) {
	      winJ = thisJ;
	      winD = thisD;
	      winTop = itTop;
	      winBot = itBot;
	    }
	  }

	  cout << "class: " << thisClass << " descriptors: " << keypoints.size() << " " << itBot << itTop << "  " << hTop << hBot << endl;

	  /*4*/itTop.y += redStride;
	  /*4*/itBot.y += redStride;
	}

	/*2*/itTop.x += redStride;
	/*2*/itBot.x += redStride;
      }
    } else {

      for (int c = 0; c < bTops.size(); c++) {
	cv::Point hTop = bTops[c];
	cv::Point hBot = bBots[c];
	int crW = hBot.x - hTop.x;
	int crH = hBot.y - hTop.y;
	
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
	    /*
	    Mat descriptors;

	    Mat crop = original_cam_img(cv::Rect(itTop.x, itTop.y, itBot.x-itTop.x, itBot.y-itTop.y));
	    Mat gray_image;
	    Mat yCrCb_image;
	    cvtColor(crop, gray_image, CV_BGR2GRAY);
	    cvtColor(crop, yCrCb_image, CV_BGR2YCrCb);
	    GaussianBlur(gray_image, gray_image, cv::Size(0,0), grayBlur);
	    GaussianBlur(yCrCb_image, yCrCb_image, cv::Size(0,0), grayBlur);

	    detector->detect(gray_image, keypoints);
	    bowExtractor->compute(gray_image, keypoints, descriptors);
	    */

	    Mat& yCrCb_image = bYCrCb[c];
	    cout << &(bYCrCb[c]) << endl;

	    cv::Point lItTop(itTop.x-hTop.x, itTop.y-hTop.y);
	    cv::Point lItBot(itBot.x-hTop.x, itBot.y-hTop.y);

	    Mat descriptors(1,vocabNumWords, CV_32F);
	    float totalWords = bWords[c].size();
	    float countedWords = 0;
//cout << "totalWords: " << totalWords;
	    for (int w = 0; w < totalWords; w++) {
	      int tX = bKeypoints[c][w].pt.x;
	      int tY = bKeypoints[c][w].pt.y;
	      // check for containment in this box
//cout << " tX tY:" << tX << " " << tY << " " << itTop << itBot << lItTop << lItBot << hTop << hBot << endl;
	      if(
		(tX >= lItTop.x) &&
		(tX <= lItBot.x) &&
		(tY >= lItTop.y) &&
		(tY <= lItBot.y) 
		) {
//cout << " w:" << w << " " << endl;
		descriptors.at<float>(bWords[c][w])++;
		keypoints.push_back(bKeypoints[c][w]);
		countedWords++;		
	#ifdef DRAW_RED_KEYPOINTS
		rectangle(cv_ptr->image, cv::Point(hTop.x+tX,hTop.y+tY), 
		  cv::Point(hTop.x+tX+1,hTop.y+tY+1), cv::Scalar(0,0,255));
	#endif
	      }
	    }
	    if (countedWords > 0)
	      for (int w = 0; w < vocabNumWords; w++) {
		descriptors.at<float>(w) = descriptors.at<float>(w) / countedWords;
	      }

	    Mat neighbors(1, redK, CV_32F);
	    Mat dist;
	    if (!descriptors.empty()) {
	      Mat colorHist(1, colorHistNumBins*colorHistNumBins, descriptors.type());
	      /**/
	      for (int i = 0; i < colorHistNumBins*colorHistNumBins; i++) 
		colorHist.at<float>(i) = 0;

	      double numPix = 0;
	      // traverse all of the keypoints
	      for (int kk = 0; kk < keypoints.size(); kk++) {
		// count pixels in a neighborhood of the keypoint
// XXX this should be min-ed against the blue box width and height
// XXX also do this at the other corresponding location
		int yMin = max(int(keypoints[kk].pt.y)-colorHistBoxHalfWidth,0);
		int yMax = min(int(keypoints[kk].pt.y)+colorHistBoxHalfWidth,crH);
		int xMin = max(int(keypoints[kk].pt.x)-colorHistBoxHalfWidth,0);
		int xMax = min(int(keypoints[kk].pt.x)+colorHistBoxHalfWidth,crW);
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
	      /**/

	      Mat descriptors2 = Mat(1, descriptors.size().width + colorHistNumBins*colorHistNumBins, descriptors.type());
	      for (int i = 0; i < descriptors.size().width; i++) 
		descriptors2.at<float>(i) = descriptors.at<float>(i);

	      /**/
	      for (int i = 0; i < colorHistNumBins*colorHistNumBins; i++) {
		colorHist.at<float>(i) = min(colorHist.at<float>(i), float(colorHistThresh));
		descriptors2.at<float>(i+descriptors.size().width) = colorHistLambda * colorHist.at<float>(i);
	      }
	      /**/

	      //kNN->find_nearest(descriptors, redK, 0, 0, &neighbors, &dist);
	      kNN->find_nearest(descriptors2, redK, 0, 0, &neighbors, &dist);

// XXX fill this out
//if (r == 2)
  //rectangle(cv_ptr->image, itTop, itBot, cv::Scalar(0,0,128));

	      int thisJ = 0;
	      float thisD = 1e6;
	      for (int n = 0; n < redK; n++) 
	      //for (int n = 0; n < 1; n++) 
	      {
		cout << "  neighbors[" << n <<"] is " << (neighbors.at<float>(n)) << endl;
		if (neighbors.at<float>(n) == float(thisClass)) {
		  thisJ++;
		  thisD = min(dist.at<float>(n), winD);
		}
	      }
	      cout << thisJ << endl;
	      if (thisJ > 0 && thisD < winD) {
		winJ = thisJ;
		winD = thisD;
		winTop = itTop;
		winBot = itBot;
	      }

	    }

	    cout << "class: " << thisClass << " descriptors: " << keypoints.size() << " " << itBot << itTop << "  " << hTop << hBot << endl;

	    /*4*/itTop.y += redStride;
	    /*4*/itBot.y += redStride;
	  }

	  /*2*/itTop.x += redStride;
	  /*2*/itBot.x += redStride;
	}
      }
    }

    // always decay persistence but only add it if we got a hit
    thisRedBox->persistence = redDecay*thisRedBox->persistence;
    if (winD < 1e6) {
      thisRedBox->anchor.x = redDecay*thisRedBox->anchor.x + (1.0-redDecay)*winTop.x;
      thisRedBox->anchor.y = redDecay*thisRedBox->anchor.y + (1.0-redDecay)*winTop.y;
      thisRedBox->persistence = thisRedBox->persistence + (1.0-redDecay)*1.0;
    }

    //cv::Point dTop(winTop.x+10, winTop.y+10);
    //cv::Point dBot(winBot.x-10, winBot.y-10);
    //cv::Point dTop(thisRedBox->anchor.x, thisRedBox->anchor.y);
    //cv::Point dBot(thisRedBox->anchor.x + winBot.x - winTop.x, thisRedBox->anchor.y + winBot.y - winTop.y);
    cv::Point dTop(thisRedBox->anchor.x, thisRedBox->anchor.y);
    cv::Point dBot(thisRedBox->anchor.x + thisRedBox->bot.x, thisRedBox->anchor.y + thisRedBox->bot.y);

    char labelName[256]; 
    if (thisClass == -1)
      sprintf(labelName, "VOID");
    else
      sprintf(labelName, "%s", classLabels[thisClass].c_str());

    #ifdef DRAW_RED
    //if (thisRedBox->persistence > 0.0) 
    {
      rectangle(cv_ptr->image, dTop, dBot, cv::Scalar(0,0,128));
      cv::Point text_anchor(dTop.x, dTop.y+20);
      putText(cv_ptr->image, labelName, text_anchor, MY_FONT, 1.5, Scalar(0,0,128), 2.0);
    }

    {
      rectangle(cv_ptr->image, winTop, winBot, cv::Scalar(0,0,255));
      cv::Point text_anchor(winTop.x, winTop.y+20);
      putText(cv_ptr->image, labelName, text_anchor, MY_FONT, 1.5, Scalar(0,0,255), 2.0);
    }
    #endif
    
    if (winD < thisRedBox->lastDistance) {
      if (thisRedBox->persistence > 0.5)
	thisRedBox->bot = cv::Point(redDecay*thisRedBox->bot.x + (1.0-redDecay)*(winBot.x - winTop.x), 
				    redDecay*thisRedBox->bot.y + (1.0-redDecay)*(winBot.y - winTop.y) );
      else
	thisRedBox->bot = cv::Point((winBot.x - winTop.x), (winBot.y - winTop.y));
    }
    thisRedBox->lastDistance = winD;
  }
  #endif

#endif

#ifdef SAVE_BOXES
  // save the crops
  if (fc == 0) {
    fc = 1;
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

#ifdef DRAW_BLUE
  for (int c = bTops.size()-1; c >= 0; c--) {
    rectangle(cv_ptr->image, bTops[c], bBots[c], cv::Scalar(255,0,0));
  }
#endif

  cv::imshow("Object Viewer", cv_ptr->image);
  cv::imshow("Density Viewer", img_cvt);

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

  cv::waitKey(1);
}

/*
void tableCallback(const object_recognition_msgs::Table& msg)
{
  cout << "Hit tableCloudCallback" << endl;
  tablePose = msg.pose;
}
*/

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::fromROSMsg(*msg, pointCloud);
cout << "Hit pointCloudCallback" <<  "  " << pointCloud.size() << endl;
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
		if(LOC_MODEL_TYPE == 0) f= 580;
		else f=525;
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

  srand(time(NULL));

  // initialize the redBoxes
  //   it is ok to add more redBoxes and ok to duplicate classes,
  //   i.e. you can add a second gyroBowl
  // number class
  // 1 gyro
  // 2 mix
  // 3 wood
  // 4 plastic
  // 7 sippy

#ifdef RUN_TRACKING

  redBoxes = new redBox[numRedBoxes];
  redBoxes[0].classLabel = 0;
  redBoxes[0].top = cv::Point(0,0);
  redBoxes[0].bot = cv::Point(50, 50);
  redBoxes[1].classLabel = 1;
  redBoxes[1].top = cv::Point(0,0);
  redBoxes[1].bot = cv::Point(50, 50);
  redBoxes[2].classLabel = 2;
  redBoxes[2].top = cv::Point(0,0);
  redBoxes[2].bot = cv::Point(50,50);
/*
  redBoxes[3].classLabel = 3;
  redBoxes[3].top = cv::Point(0,0);
  redBoxes[3].bot = cv::Point(250,250);
  redBoxes[4].classLabel = 4;
  redBoxes[4].top = cv::Point(0,0);
  redBoxes[4].bot = cv::Point(100,100);
*/

  // initialize the rest
  for (int r = 0; r < numRedBoxes; r++) {
    redBoxes[r].rootBlueBox = 0;
    redBoxes[r].numGreenBoxes;
    redBoxes[r].anchor = cv::Point(0,0);
    redBoxes[r].persistence = 0;
    redBoxes[r].lastDistance = 0;
  }

#endif

  ros::init(argc, argv, "oberlin_detection");
  ros::NodeHandle n("~");
  std::string s;

//cout << argc << " " << endl;
//for (int i = 0; i < argc; i++)
//  cout << argv[i] << " ";
//cout << endl;

  loadROSParamsFromArgs();
  cout << endl << "numRedBoxes: " << numRedBoxes << endl;
  cout << "data_directory: " << data_directory << endl << "class_name: " << class_name << endl 
       << "run_prefix: " << run_prefix << endl << "class_pose_models: " << class_pose_models << endl 
       << "class_labels: " << class_labels << endl << "vocab_file: " << vocab_file << endl 
       << "knn_file: " << knn_file << endl << "label_file: " << label_file << endl
       << endl;
//exit(0);

  saveROSParams();

  package_path = ros::package::getPath("oberlin_detection");
  class_crops_path = data_directory + "/";

  // The other models
  //ObjNessB2W8MAXBGR
  //ObjNessB2W8I
  //ObjNessB2W8HSV
  bing_trained_models_path = package_path + "/bing_trained_models/";
  objectness_matrix_path = bing_trained_models_path + "ObjNessB2W8I.idx.yml";
  objectness_path_prefix = bing_trained_models_path + "ObjNessB2W8MAXBGR";

  DataSetVOC voc("../VOC2007/");
  Objectness objNess(voc, 2, 8, 2);
  glObjectness = &(objNess);

  printf("objectness_path_prefix: %s\n", objectness_path_prefix.c_str());
  int result = objNess.loadTrainedModel(objectness_path_prefix);
  cout << "result: " << result << endl << endl;

  image_transport::Subscriber image_sub;
  image_transport::ImageTransport it(n);
  //image_sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);
  image_sub = it.subscribe("/filter_time/filtered_image", 1, imageCallback);

  ros::Subscriber clusters = n.subscribe("/tabletop/clusters", 1, clusterCallback);
  ros::Subscriber points = n.subscribe("/camera/depth_registered/points", 1, pointCloudCallback);

  rec_objs = n.advertise<object_recognition_msgs::RecognizedObjectArray>("labeled_objects", 10);
  markers = n.advertise<visualization_msgs::MarkerArray>("object_markers", 10);

  cv::namedWindow("Object Viewer");
  cv::namedWindow("Density Viewer");
  setMouseCallback("Object Viewer", CallBackFunc, NULL);

  fc = 0;
  cropCounter = 0;
  tableNormal = Eigen::Vector3d(1,0,0);
  tableBias = 0;

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
  string bufstr; // Have a buffer string

  stringstream ss_cl(class_labels); 
  while (ss_cl >> bufstr)
    classLabels.push_back(bufstr);

  stringstream ss_cpm(class_pose_models); 
  while (ss_cpm >> bufstr)
    classPoseModels.push_back(bufstr);

  cout << classLabels.size() << endl;
  cout << classPoseModels.size() << endl;

  if ((classLabels.size() != classPoseModels.size()) || (classLabels.size() < 1)) {
    cout << "label or pose model problem. exiting." << endl;
    exit(0);
  }
#endif

#ifdef REWRITE_LABELS
  rewrite_labels = 1;
#endif 
  if (rewrite_labels) {
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

  int numClasses = classLabels.size();
  
  for (int i = 0; i < numClasses; i++) {
    cout << classLabels[i] << " " << classPoseModels[i] << endl;
  }

  Mat vocabulary;

#ifdef RELEARN_VOCAB
  retrain_vocab = 1;
#endif 
  if (retrain_vocab) {
    for (int i = 0; i < numClasses; i++) {
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

#ifdef RELOAD_DATA
  reextract_knn = 1;
#endif
  if (reextract_knn) {
    for (int i = 0; i < numClasses; i++) {
      cout << "Getting kNN features for class " << classLabels[i] 
	   << " with pose model " << classPoseModels[i] << " index " << i << endl;
      kNNGetFeatures(class_crops_path, classLabels[i].c_str(), i, grayBlur, kNNfeatures, kNNlabels);
      if (classPoseModels[i].compare("G") == 0) {
	string thisPoseLabel = classLabels[i] + "Poses";
	//kNNGetFeatures(class_crops_path, thisPoseLabel.c_str(), i, grayBlur, kNNfeatures, kNNlabels);
	posekNNGetFeatures(class_crops_path, thisPoseLabel.c_str(), grayBlur, classPosekNNfeatures[i], classPosekNNlabels[i]);
      }
    }

    FileStorage fsfO;
    cout<<"Writing features and labels..."<< endl << featuresPath << endl << "...";
    fsfO.open(featuresPath, FileStorage::WRITE);
    fsfO << "features" << kNNfeatures;
    fsfO << "labels" << kNNlabels;
    for (int i = 0; i < numClasses; i++) {
      if (classPoseModels[i].compare("G") == 0) {
	string fnOut = "features" + classLabels[i];
	string lnOut = "labels" + classLabels[i];
	cout << "G: " << classLabels[i] << " " << fnOut << " " << lnOut << endl;
	fsfO << fnOut << classPosekNNfeatures[i];
	fsfO << lnOut << classPosekNNlabels[i];
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
	cout << "G: " << classLabels[i] << " " << fnIn << " " << lnIn << endl;
	fsfI[fnIn] >> classPosekNNfeatures[i];
	fsfI[lnIn] >> classPosekNNlabels[i];
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

  // it is important to L1 normalize the filters so that comparing dot products makes sense.
  // that is, they should all respond equally to a constant image.
  double l1norm = orientedFilters[0].dot(Mat::ones(O_FILTER_WIDTH, O_FILTER_WIDTH, CV_64F));
  biggestL1 = l1norm;

  for (int o = 1; o < ORIENTATIONS; o++) {
    // Compute a rotation matrix with respect to the center of the image
    Point center = Point(O_FILTER_WIDTH/2, O_FILTER_WIDTH/2);
    double angle = o*360.0/ORIENTATIONS;
    double scale = 1.0;

    // Get the rotation matrix with the specifications above
    Mat rot_mat = getRotationMatrix2D( center, angle, scale );

    // Rotate the warped image
    warpAffine(orientedFilters[0], orientedFilters[o], rot_mat, orientedFilters[o].size());

    double l1norm = orientedFilters[o].dot(Mat::ones(O_FILTER_WIDTH, O_FILTER_WIDTH, CV_64F));
    orientedFilters[o] = orientedFilters[o] / l1norm;
    
    if (l1norm > biggestL1)
      biggestL1 = l1norm;
  }

  l1norm = orientedFilters[0].dot(Mat::ones(O_FILTER_WIDTH, O_FILTER_WIDTH, CV_64F));
  orientedFilters[0] = orientedFilters[0] / l1norm;

  ros::spin();
  return 0;
}


/* Notes

Eventually features should be calculated on at most the green boxes.

*/
