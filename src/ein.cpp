// start Header
//
//  // start TODO XXX
//  //
//  // patch targeting
//  // redo cartesian autopilot to account for ee coordinate frame
//  // redo PID controller for absolute coordinates
//  // make sure chirality is accounted for properly in autopilot
//  // make more general grasping interface, i.e. have a grasp depth target register.
//  // 
//  // end   TODO XXX
//
//  // start structure of this program 
//  //
//  ////
//  // start pilot includes, usings, and defines
//  //// preprocessory stuff
//  // end pilot includes, usings, and defines 
//  //
//  // start node includes, usings, and defines
//  //// preprocessory stuff
//  // end node includes, usings, and defines
//  //
//  // start pilot variables 
//  //// global variables
//  // end pilot variables 
//  //
//  // start node variables 
//  //// global variables
//  // end node variables 
//  //
//  // start pilot prototypes 
//  //// prototypes
//  // end pilot prototypes 
//  //
//  // start node prototypes
//  //// prototypes
//  // end node prototypes 
//  //
//  // start pilot definitions 
//  //// function definitions
//  // end pilot definitions 
//  //
//  // start node definitions 
//  //// function definitions
//  // end node definitions 
//  //
//  // start ein 
//  //// main()
//  //
//  // end structure of this program
//
//  // start Tips
//  // it's dangerous to go alone. take this.
//  //               \|/
//  // o++{==========>*
//  //		     /|\
//  // end Tips
//
//  // Ein: Ein Isn't Node.
//  // Ein: A dog in Cowboy Bebop.
//  // Ein: Short for Eindecker.
//  // Ein: It's one program. 
//
// end Header

//#define DEBUG
#define EPSILON 1.0e-9
#define INFINITY 1e6

#define PROGRAM_NAME "ein"
#define MY_FONT FONT_HERSHEY_SIMPLEX

////////////////////////////////////////////////
// start pilot includes, usings, and defines
////////////////////////////////////////////////

#include <vector>
#include <string>
#include "ros/ros.h"
#include "ros/package.h"
#include "ros/time.h"
#include <ctime>

#include <tf/transform_listener.h>

#include <sstream>
#include <iostream>
#include <math.h>

#include <baxter_core_msgs/EndpointState.h>
#include <sensor_msgs/Range.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/JointCommand.h>

#include <dirent.h>

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

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/nonfree/nonfree.hpp>

using namespace std;
using namespace cv;
using namespace Eigen;

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

#define MOVE_FAST 0.01
#define MOVE_MEDIUM 0.005 //.005
#define MOVE_SLOW 0.0025
#define MOVE_VERY_SLOW 0.00125

////////////////////////////////////////////////
// end pilot includes, usings, and defines 
//
// start node includes, usings, and defines
////////////////////////////////////////////////

#include <signal.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "../../bing/Objectness/stdafx.h"
#include "../../bing/Objectness/Objectness.h"
#include "../../bing/Objectness/ValStructVec.h"
#include "../../bing/Objectness/CmShow.h"

typedef enum {
  SIFTBOW_GLOBALCOLOR_HIST = 1,
  OPPONENTSIFTBOW_GLOBALCOLOR_HIST = 2, // this has not been sufficiently tested
  SIFTCOLORBOW_HIST = 3 // unimplemented, calculate color histograms at each keypoint and augment each SIFT feature before clustering
} featureType;
featureType chosen_feature = SIFTBOW_GLOBALCOLOR_HIST;
int cfi = 1;

typedef enum {
  MRT,
  SPOON,
  KNIFE,
  OFT_INVALID
} orientedFilterType;

#define ORIENTATIONS 180//12 
#define O_FILTER_WIDTH 25//25
#define O_FILTER_SPOON_HEAD_WIDTH 6 
#define O_FILTER_SPOON_SHAFT_WIDTH 2

////////////////////////////////////////////////
// end node includes, usings, and defines
//
// start pilot variables 
////////////////////////////////////////////////

double rapidAmp1 = 0.00; //0.3 is great
double rapidAmp1Delta = 0.01;

double rapidAmp2 = 0.00;
double rapidAmp2Delta = 0.01;

const int numJoints = 7;
int driveVelocities = 0;
int testJoint = 3;
ros::Time oscilStart;

int jointNamesInit = 0;
std::vector<std::string> jointNames;

double spiralEta = 1.25;

//double trueJointPositions[numJoints] = {0, 0, 0, 0, 0, 0, 0};
//double rapidJointGlobalOmega[numJoints] = {0, 0, 0, 4, 4, 0, 4};
//double rapidJointLocalOmega[numJoints] = {0, 0, 0, 1, 1, 0, 1};
//double rapidJointLocalBias[numJoints] = {0, 0, 0, 0.7, 0, 0, 0};
//int rapidJointMask[numJoints] = {0, 0, 0, 1, 1, 0, 1};
//double rapidJointScales[numJoints] = {0, 0, 0, 1.0, 2.0, 0, 3.1415926};

double trueJointPositions[numJoints] = {0, 0, 0, 0, 0, 0, 0};
double rapidJointGlobalOmega[numJoints] = {4, 0, 0, 4, 4, 4, 4};
double rapidJointLocalOmega[numJoints] = {.2, 0, 0, 2, 2, .2, 2};
double rapidJointLocalBias[numJoints] = {0, 0, 0, 0.7, 0, 0, 0};
int rapidJointMask[numJoints] = {1, 0, 0, 1, 1, 1, 1};
double rapidJointScales[numJoints] = {.10, 0, 0, 1.0, 2.0, .20, 3.1415926};

double aveTime = 0.0;
double aveFrequency = 0.0;
double timeMass = 0.0;
double timeInterval = 30;
time_t thisTime = 0;
time_t firstTime = 0;

// this should be initted to 0 and set to its default setting only after an imageCallback has happened.
int shouldIRenderDefault = 1;
int shouldIRender = 0;
int renderInit = 0;

// if autopilot takes too long, it gets angry and takes a shot on a yellow frame
int take_yellow_thresh = 3600;
int autoPilotFrameCounter = 0;

int ik_reset_thresh = 3600;
int ik_reset_counter = 0;
int ikInitialized = 0.0;
double ikShare = 1.0;
double oscillating_ikShare = .1;
double default_ikShare = 1.0;
int oscillatingPeriod = 3200;
double oscillatingInvAmplitude = 4.0;
double oscillatingBias = 0.4;

int oscillating = 0;
int oscillatingSign = 1;
int oscillatorTimerThresh = 600;

int current_instruction = 'C';
double tap_factor = 0.1;
int execute_stack = 0;
std::vector<int> pilot_call_stack;

int go_on_lock = 0;
int slf_thresh = 5;
int successive_lock_frames = 0;

int lock_reset_thresh = 1800;
int lock_status = 0; // TODO enum

//double slow_aim_factor = 0.5;
double slow_aim_factor = 0.75;
int aim_thresh = 20;
int lock_thresh = 5;

int timerCounter = 0;
int timerThresh = 16;

int timesTimerCounted = 0;

double prevPx = 0;
double prevPy = 0;
//double Kd = 0.00001; // for P only
double Kd = 0.002;
double Kp = 0.0004;
double cCutoff = 20.0;

double a_thresh_close = .11;
// XXX TODO 
//double a_thresh_far = .3; // for visual servoing
double a_thresh_far = .2; // for depth scanning
//double a_thresh_far = .13; // for close depth scanning
double eeRange = 0.0;

double bDelta = MOVE_FAST;
double approachStep = .0005;
double hoverMultiplier = 0.5;

int zero_g_toggle = 0;
int holding_pattern = 0; // TODO enum
int auto_pilot = 0;

tf::TransformListener* tfListener;
double tfPast = 10.0;

std::string wristViewName = "Wrist View";
std::string coreViewName = "Core View";
std::string rangeogramViewName = "Rangeogram View";
std::string rangemapViewName = "Range Map View";
std::string hiRangemapViewName = "Hi Range Map View";
std::string hiColorRangemapViewName = "Hi Color Range Map View";

int reticleHalfWidth = 30;
int pilotTargetHalfWidth = 15;

eePose defaultRightReticle = {.px = 321, .py = 154, .pz = 0.0,
		   .ox = 0.0, .oy = 0.0, .oz = 0.0,
		   .qx = 0.0, .qy = 0.0, .qz = 0.0, .qw = 0.0};
eePose defaultLeftReticle = {.px = 328, .py = 113, .pz = 0.0,
		   .ox = 0.0, .oy = 0.0, .oz = 0.0,
		   .qx = 0.0, .qy = 0.0, .qz = 0.0, .qw = 0.0};

eePose beeLHome = {.px = 0.657579481614, .py = 0.851981417433, .pz = 0.0388352386502,
		   .ox = 0.0, .oy = 0.0, .oz = 0.0,
		   .qx = -0.366894936773, .qy = 0.885980397775, .qz = 0.108155782462, .qw = 0.262162481772};
eePose beeRHome = {.px = 0.657579481614, .py = -0.168019, .pz = 0.0388352386502,
		   .ox = 0.0, .oy = 0.0, .oz = 0.0,
		   .qx = -0.366894936773, .qy = 0.885980397775, .qz = 0.108155782462, .qw = 0.262162481772};
eePose workCenter = {.px = 0.686428, .py = -0.509836, .pz = 0.0883011,
		     .ox = 0.0, .oy = 0.0, .oz = 0.0,
		     .qx = -0.435468, .qy = 0.900181, .qz = 0.00453569, .qw = 0.00463141};

eePose crane1right = {.px = 0.0448714, .py = -1.04476, .pz = 0.698522,
		     .ox = 0, .oy = 0, .oz = 0,
		     .qx = 0.631511, .qy = 0.68929, .qz = -0.25435, .qw = 0.247748};
eePose crane2right = {.px = 0.617214, .py = -0.301658, .pz = 0.0533165,
		     .ox = 0, .oy = 0, .oz = 0,
		     //.qx = -0.0174863, .qy = 0.998142, .qz = 0.0583579, .qw = -0.000393204}; // 12A6S corrected *
		     //.qx = -0.0148346, .qy = 0.999022, .qz = 0.0289031, .qw = 0.0300052}; // 9A6S corrected
		     //.qx = -0.0165925, .qy = 0.99892, .qz = 0.0434064, .qw = -0.000183836}; // 6A corrected
		     //.qx = -0.01459, .qy = 0.999684, .qz = 0.0136977, .qw = 0.0152314}; // 3A3S corrected
		     //.qx = -0.0139279, .qy = 0.999439, .qz = -0.00107611, .qw = 0.0304367}; // original
		     //.qx = -0.0130087, .qy = 0.998957, .qz = -0.0310543, .qw = 0.0308408}; // 6D corrected 
		     //.qx = -0.0120778, .qy = 0.997576, .qz = -0.0610046, .qw = 0.0312171}; // 12D corrected 
		     //.qx = -0.0138943, .qy = 0.999903, .qz = -0.000868458, .qw = 0.000435656}; // ray calibrated
		     .qx = 0.0, .qy = 1.0, .qz = 0.0, .qw = 0.0}; // straight down 

eePose crane3right = {.px = 0.668384, .py = 0.166692, .pz = -0.120018,
		     .ox = 0, .oy = 0, .oz = 0,
		     .qx = 0.0328281, .qy = 0.999139, .qz = 0.00170545, .qw = 0.0253245};
eePose crane4right = {.px = 0.642291, .py = -0.659793, .pz = 0.244186,
		     .ox = 0, .oy = 0, .oz = 0,
		     .qx = 0.825064, .qy = 0.503489, .qz = 0.12954, .qw = 0.221331};

// poised
eePose crane5right = {.px = 0.68502, .py = -0.109639, .pz = 0.722995,
		     .ox = 0, .oy = 0, .oz = 0,
		     .qx = -0.425038, .qy = 0.79398, .qz = 0.183347, .qw = 0.39411};

eePose crane1left = {.px = -0.0155901, .py = 0.981296, .pz = 0.71078,
		     .ox = 0, .oy = 0, .oz = 0,
		     .qx = 0.709046, .qy = -0.631526, .qz = -0.226613, .qw = -0.216967};
eePose crane2left = {.px = 0.646069, .py = 0.253621, .pz = 0.0570906,
		     .ox = 0, .oy = 0, .oz = 0,
		     .qx = 0.999605, .qy = -0.0120443, .qz = 0.0253545, .qw = -0.00117847};
eePose crane3left = {.px = 0.652866, .py = -0.206966, .pz = -0.130561,
		     .ox = 0, .oy = 0, .oz = 0,
		     .qx = 0.999605, .qy = -0.0120443, .qz = 0.0253545, .qw = -0.00117847};

eePose defaultReticle = defaultRightReticle;
eePose reticle = defaultReticle;
eePose beeHome = beeRHome;
eePose pilotTarget = beeHome;
eePose lastGoodEEPose = beeHome;
eePose currentEEPose = beeHome;
eePose eepReg1 = workCenter;
eePose eepReg2 = beeHome;
eePose eepReg3 = beeHome;
eePose eepReg4 = beeHome;

string left_or_right_arm = "right";

eePose ik_reset_eePose = beeHome;

Vector3d eeForward;
geometry_msgs::Pose trueEEPose;

int bfc = 0;
int bfc_period = 3;
int resend_times = 1;

baxter_core_msgs::SolvePositionIK ikRequest;

ros::ServiceClient ikClient;
ros::Publisher joint_mover;
ros::Publisher gripperPub;


const int imRingBufferSize = 300;
const int epRingBufferSize = 100;
const int rgRingBufferSize = 100;

// we make use of a monotonicity assumption
// if the current index passes the last recorded index, then we just proceed
//  and lose the ranges we skipped over. alert when this happens
// first valid entries
int imRingBufferStart = 0;
int epRingBufferStart = 0;
int rgRingBufferStart = 0;

// first free entries
int imRingBufferEnd = 0;
int epRingBufferEnd = 0;
int rgRingBufferEnd = 0;

std::vector<Mat> imRingBuffer;
std::vector<geometry_msgs::Pose> epRingBuffer;
std::vector<double> rgRingBuffer;

std::vector<ros::Time> imRBTimes;
std::vector<ros::Time> epRBTimes;
std::vector<ros::Time> rgRBTimes;

Mat rangeogramImage;
Mat rangemapImage;
Mat hiRangemapImage;
Mat hiColorRangemapImage;
const int totalRangeHistoryLength = 100;
double rangeHistory[totalRangeHistoryLength];
int currentRangeHistoryIndex = 0;

int rggScale = 1;  
int rggStride = 5*rggScale;
int rggHeight = 300*rggScale;
int rggWidth = totalRangeHistoryLength*rggStride;

const int rmWidth = 21; // must be odd
const int rmHalfWidth = (rmWidth-1)/2; // must be odd
const double rmDelta = 0.01;
double rangeMap[rmWidth*rmWidth];
double rangeMapAccumulator[rmWidth*rmWidth];
double rangeMapMass[rmWidth*rmWidth];

double rangeMapReg1[rmWidth*rmWidth];
double rangeMapReg2[rmWidth*rmWidth];

const int hrmWidth = 201; // must be odd
const int hrmHalfWidth = (hrmWidth-1)/2; // must be odd
const double hrmDelta = 0.001;
double hiRangeMap[hrmWidth*hrmWidth];
double hiRangeMapAccumulator[hrmWidth*hrmWidth];
double hiRangeMapMass[hrmWidth*hrmWidth];

double hiColorRangeMapAccumulator[3*hrmWidth*hrmWidth];
double hiColorRangeMapMass[hrmWidth*hrmWidth];

double hiRangeMapReg1[hrmWidth*hrmWidth];
double hiRangeMapReg2[hrmWidth*hrmWidth];

// hi separable filter width / half width
const int hsfHw = 10;
const int hsfW = 2*hsfHw+1;
double hsFilter[hsfW];

double filter[9] = {1.0/16.0, 1.0/8.0, 1.0/16.0, 
		    1.0/8.0, 1.0/4.0, 1.0/8.0, 
		    1.0/16.0, 1.0/8.0, 1.0/16.0};;

// diagonalKappa: 0.72 deltaDiagonalKappa: 0.01
// below .72, the horizontal won when it should have. Set to .67 to be safe.
// .67 was a little unreliable, trimming a bit more.
double diagonalKappa = 0.60;
double deltaDiagonalKappa = 0.01;

double gibbsIota= 0.00;
double deltaGibbsIota= 0.01;

const int parzenKernelHalfWidth = 15;
const int parzenKernelWidth = 2*parzenKernelHalfWidth+1;
double parzenKernel[parzenKernelWidth*parzenKernelWidth];
//double parzenKernelSigma = 2.0;
double parzenKernelSigma = 1.0; // this is approximately what it should be at 20 cm height
//double parzenKernelSigma = 0.5; // 
// 13.8 cm high -> 2.2 cm gap
// 23.8 cm high -> 3.8 cm gap
// 4 sigma (centered at 0) should be the gap
// TODO can 'adjust' and bounds on the fly during lookup in proportion to the measured depth

int doParzen = 0;

// assumptions are made here so if these values changes, the code must
//  be audited.
const int vmWidth = hrmWidth;
const int vmHalfWidth = hrmHalfWidth;
const double vmDelta = hrmDelta;
double volumeMap[vmWidth*vmWidth*vmWidth];
double volumeMapAccumulator[vmWidth*vmWidth*vmWidth];
double volumeMapMass[vmWidth*vmWidth*vmWidth];

double vmColorRangeMapAccumulator[3*vmWidth*vmWidth*vmWidth];
double vmColorRangeMapMass[vmWidth*vmWidth*vmWidth];

const int parzen3DKernelHalfWidth = 9;
const int parzen3DKernelWidth = 2*parzen3DKernelHalfWidth+1;
double parzen3DKernel[parzen3DKernelWidth*parzen3DKernelWidth*parzen3DKernelWidth];
double parzen3DKernelSigma = 2.0; 

// range map center
double rmcX;
double rmcY;
double rmcZ;

double lastiX = 0;
double lastiY = 0;
double thisiX = 0;
double thisiY = 0;

int rmiCellWidth = 20;
//int rmiCellWidth = 10;
int rmiHeight = rmiCellWidth*rmWidth;
int rmiWidth = rmiCellWidth*rmWidth;

int hrmiHeight = hrmWidth;
int hrmiWidth = hrmWidth;

// the currently equipped depth reticle
double drX = .02; //.01;
double drY = .02;

// target reticle
double trX = 0;
double trY = 0;

double htrX = 0;
double htrY = 0;

int maxX = 0;
int maxY = 0;
double maxD = 0;
int maxGG = 0;

double graspDepth = -.02;

// grasp gear 
const int totalGraspGears = 8;
int currentGraspGear = -1;
//// reticles
double ggX[totalGraspGears];
double ggY[totalGraspGears];
double ggT[totalGraspGears];

int recordRangeMap = 1;

Quaternionf irGlobalPositionEEFrame;

Mat wristCamImage;
int wristCamInit = 0;

// reticle indeces
//x: 439 y: 153 eeRange: 0.102
//x: 428 y: 153 eeRange: 0.111
//x: 428 y: 153 eeRange: 0.111
//x: 418 y: 152 eeRange: 0.12
//x: 420 y: 152 eeRange: 0.12
//x: 410 y: 155 eeRange: 0.131
//x: 411 y: 156 eeRange: 0.131
//
//x: 405 y: 153 eeRange: 0.142
//x: 396 y: 152 eeRange: 0.149
//x: 394 y: 160 eeRange: 0.16
//x: 389 y: 154 eeRange: 0.169
//
//eeRange: 0.179
//x: 383 y: 155 eeRange: 0.183
//x: 383 y: 155 eeRange: 0.191
const double cReticleIndexDelta = .01;
//const int numCReticleIndeces = 10;
//const double firstCReticleIndexDepth = .10;
//const int xCR[numCReticleIndeces] = {439, 428, 419, 410, 405, 396, 394, 389, 383, 383};
//const int yCR[numCReticleIndeces] = {153, 153, 153, 152, 155, 153, 152, 160, 154, 155};
const int numCReticleIndeces = 14;
const double firstCReticleIndexDepth = .08;
//const int xCR[numCReticleIndeces] = {462, 450, 439, 428, 419, 410, 405, 399, 394, 389, 383, 381, 379, 378};
//const int yCR[numCReticleIndeces] = {153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153, 153};
const int xCR[numCReticleIndeces] = {462, 450, 439, 428, 419, 410, 405, 399, 394, 389, 383, 381, 379, 378};
const int yCR[numCReticleIndeces] = {153, 153, 153, 153, 153, 154, 154, 154, 154, 154, 155, 155, 155, 155};

ros::Publisher vmMarkerPublisher;

int getColorReticleX();
int getColorReticleY();

double fEpsilon = EPSILON;

int curseReticleX = 0;
int curseReticleY = 0;

////////////////////////////////////////////////
// end pilot variables 
//
// start node variables 
////////////////////////////////////////////////

// Start Intron: The below variables are left over and should be considered defunct
int retrain_vocab = 0;
int rewrite_labels = 0;
int reextract_knn = 0;
int runInference = 0;
int publishObjects = 0;
int saveAnnotatedBoxes = 0;
int captureHardClass = 0;
int captureOnly = 0;
int saveBoxes = 0;
int trainOnly = 0;
// End Intron 1: The above variables are left over and should be considered defunct

int runTracking = 1; // calculates the red boxes

int drawOrientor = 1;
int drawLabels = 1;
int drawPurple = 0;
//int drawWhite = 0;
int drawGreen = 1;
int drawBlue = 1;
int drawRed = 1;
int drawRB = 0;
int drawGray = 1;
int drawPink = 0;
int drawBrown = 1;
int drawBlueKP = 1;
int drawRedKP = 1;

object_recognition_msgs::RecognizedObjectArray roa_to_send_blue;
visualization_msgs::MarkerArray ma_to_send_blue; 

cv_bridge::CvImagePtr cv_ptr;
Mat objectViewerImage;
Mat densityViewerImage;
Mat wristViewImage;

int mask_gripper = 0;

int add_blinders = 0;
int blinder_stride = 10;
int blinder_columns = 5;

boost::mutex ros_mutex;
boost::mutex pcl_mutex;
boost::mutex redbox_mutex;
boost::thread *timercallback1_thread;
int ikResult = 1;

std::string densityViewerName = "Density Viewer";
std::string objectViewerName = "Object Viewer";

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

// paramaters for the color histogram feature
const double colorHistNumBins = 8;
const double colorHistBinWidth = 256/colorHistNumBins;
const double colorHistLambda = 0.5;
const double colorHistThresh = 0.1;
const int colorHistBoxHalfWidth = 1;

std::string data_directory = "unspecified_dd";
std::string vocab_file = "unspecified_vf";
std::string knn_file = "unspecified_kf";
std::string label_file = "unspecified_lf";

std::string run_prefix = "unspecified_rp";
std::string class_name = "unspecified_cn";

std::string class_labels= "unspecified_cl1 unspecified_cl2";
std::string class_pose_models = "unspecified_pm1 unspecified_pm2";

std::string red_box_list = "";

std::string image_topic = "/camera/rgb/image_raw"; 
std::string pc_topic = "/camera/depth_registered/points";

std::string cache_prefix = "";

int numClasses = 0;

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

double maxDensity = 0;
double *density = NULL;
double *integralDensity = NULL;
double *temporalDensity = NULL;
double *temporalDepth = NULL;

pcl::PointCloud<pcl::PointXYZRGB> pointCloud;

int biggestL1 = 0;
int oSearchWidth = 5;

Mat *orientedFiltersT;
Mat *orientedFiltersS;
Mat *orientedFiltersK;

// Top variables are top left corners of bounding boxes (smallest coordinates)
// Bot variables are bottom right corners of bounding boxes (largenst coordinates)
// Cen variables are centers of bounding boxes

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
double gBoxThresh = 30000;//5;//3;
//double threshFraction = 0;//0.35;
double threshFraction = 0.2;

int gBoxStrideX;
int gBoxStrideY;

// pink box thresholds for the principle classes
double *pBoxIndicator = NULL;
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

////////////////////////////////////////////////
// end node variables 
//
// start pilot prototypes 
////////////////////////////////////////////////

int getRingImageAtTime(ros::Time t, Mat& value, int drawSlack = 0);
int getRingRangeAtTime(ros::Time t, double &value, int drawSlack = 0);
int getRingPoseAtTime(ros::Time t, geometry_msgs::Pose &value, int drawSlack = 0);
void setRingImageAtTime(ros::Time t, Mat& imToSet);
void setRingRangeAtTime(ros::Time t, double rgToSet);
void setRingPoseAtTime(ros::Time t, geometry_msgs::Pose epToSet);
void imRingBufferAdvance();
void rgRingBufferAdvance();
void epRingBufferAdvance();
void allRingBuffersAdvance(ros::Time t);

void recordReadyRangeReadings();
void jointCallback(const sensor_msgs::JointState& js);
void endpointCallback(const baxter_core_msgs::EndpointState& eps);

void initialize3DParzen();
void l2Normalize3DParzen();
void initializeParzen();
void l2NormalizeParzen();
void l2NormalizeFilter();

int getColorReticleX();
int getColorReticleY();
cv::Vec3b getCRColor();
cv::Vec3b getCRColor(Mat im);
Quaternionf extractQuatFromPose(geometry_msgs::Pose poseIn);

void pushNoOps(int n);
void pushSpeedSign(double speed);

void scanXdirection(double speedOnLines, double speedBetweenLines);
void scanYdirection(double speedOnLines, double speedBetweenLines);
void scanYdirectionMedium(double speedOnLines, double speedBetweenLines);
void scanXdirectionMedium(double speedOnLines, double speedBetweenLines);
void scanYdirectionVerySlow(double speedOnLines, double speedBetweenLines);
void scanXdirectionVerySlow(double speedOnLines, double speedBetweenLines);

Eigen::Quaternionf getGGRotation(int givenGraspGear);
void setGGRotation(int thisGraspGear);

void rangeCallback(const sensor_msgs::Range& range);
void update_baxter(ros::NodeHandle &n);
void timercallback1(const ros::TimerEvent&);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void targetCallback(const geometry_msgs::Point& point);
void pilotCallbackFunc(int event, int x, int y, int flags, void* userdata);

void pilotInit();
void spinlessPilotMain();

////////////////////////////////////////////////
// end pilot prototypes 
//
// start node prototypes
////////////////////////////////////////////////

int rejectRedBox();

void gridKeypoints(int gImW, int gImH, cv::Point top, cv::Point bot, int strideX, int strideY, vector<KeyPoint>& keypoints, int period);

cv::Point pcCorrection(double x, double y, double imW, double imH);
bool isFiniteNumber(double x);

void appendColorHist(Mat& yCrCb_image, vector<KeyPoint>& keypoints, Mat& descriptors, Mat& descriptors2);
void processImage(Mat &image, Mat& gray_image, Mat& yCrCb_image, double sigma);

void bowGetFeatures(std::string classDir, const char *className, double sigma);
void kNNGetFeatures(std::string classDir, const char *className, int label, double sigma, Mat &kNNfeatures, Mat &kNNlabels);
void posekNNGetFeatures(std::string classDir, const char *className, double sigma, Mat &kNNfeatures, Mat &kNNlabels,
  vector< cv::Vec<double,4> >& classQuaternions, int lIndexStart = 0);

void getCluster(pcl::PointCloud<pcl::PointXYZRGB> &cluster, pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::vector<cv::Point> &points);
geometry_msgs::Pose getPose(pcl::PointCloud<pcl::PointXYZRGB> &cluster);

void getPointCloudPoints(vector<cv::Point>& pointCloudPoints, double *pBoxIndicator, double thisThresh, 
  cv::Point topIn, cv::Point botIn, int imW, int imH, int gBoxStrideX, int gBoxStrideY, int gBoxW, int gBoxH);

void fill_RO_and_M_arrays(object_recognition_msgs::RecognizedObjectArray& roa_to_send, 
  visualization_msgs::MarkerArray& ma_to_send, vector<cv::Point>& pointCloudPoints, 
  int aI, int label, int winningO, int poseIndex);

void getOrientation(vector<KeyPoint>& keypoints, Mat& descriptors, cv::Point top, cv::Point bot, 
  int label, string& labelName, string& augmentedLabelName, double& poseIndex, int& winningO);

void init_oriented_filters(orientedFilterType thisType);
void init_oriented_filters_all();
int isOrientedFilterPoseModel(string toCompare);
orientedFilterType getOrientedFilterType(string toCompare);

void nodeCallbackFunc(int event, int x, int y, int flags, void* userdata);

void goCalculateDensity();
void goFindBlueBoxes();
void goFindBrownBoxes();
void goClassifyBlueBoxes();
void goFindRedBoxes();

// Start Intron 2: nodeImageCallback() is retained as a reference for backporting.
void nodeImageCallback(const sensor_msgs::ImageConstPtr& msg);
// End Intron 2

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
//void clusterCallback(const visualization_msgs::MarkerArray& msg);

void loadROSParamsFromArgs();
void loadROSParams();
void saveROSParams();

void spinlessNodeMain();
void nodeInit();
void detectorsInit();
void initRedBoxes();

////////////////////////////////////////////////
// end node prototypes 
//
// start pilot definitions 
////////////////////////////////////////////////

int getRingImageAtTime(ros::Time t, Mat& value, int drawSlack) {
  if (imRingBufferStart == imRingBufferEnd) {
    cout << "Denied request in getRingImageAtTime(): Buffer empty." << endl;
    return 0;
  } else {
    int earliestSlot = imRingBufferStart;
    ros::Duration deltaTdur = t - imRBTimes[earliestSlot];
    // if the request comes before our earliest record, deny
    if (deltaTdur.toSec() <= 0.0) {
      cout << "Denied out of order range value in getRingImageAtTime(): Too small." << endl;
      cout << "  getRingImageAtTime() imRingBufferStart imRingBufferEnd t imRBTimes[earliestSlot]: " << 
	imRingBufferStart << " " << imRingBufferEnd << " " << t << " " << imRBTimes[earliestSlot] << endl;
      return -1;
    } else if (imRingBufferStart < imRingBufferEnd) {
      for (int s = imRingBufferStart; s < imRingBufferEnd; s++) {
	ros::Duration deltaTdurPre = t - imRBTimes[s];
	ros::Duration deltaTdurPost = t - imRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Mat m1 = imRingBuffer[s];
	  Mat m2 = imRingBuffer[s+1];
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;
	  if (w1 >= w2)
	    //value = m1;
	    value = m1*w1 + m2*w2;
	  else
	    //value = m2;
	    value = m1*w1 + m2*w2;

	  int newStart = s;
	  if(drawSlack) {
	    imRingBufferStart = newStart;
	  }
	  return 1;
	}
      }
      // if we didn't find it we should return failure
      //cout << "Denied out of order range value in getRingImageAtTime(): Too large." << endl;
      return -2;
    } else {
      for (int s = imRingBufferStart; s < imRingBufferSize-1; s++) {
	ros::Duration deltaTdurPre = t - imRBTimes[s];
	ros::Duration deltaTdurPost = t - imRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Mat m1 = imRingBuffer[s];
	  Mat m2 = imRingBuffer[s+1];
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;
	  if (w1 >= w2)
	    value = m1;
	  else
	    value = m2;

	  int newStart = s;
	  if(drawSlack) {
	    imRingBufferStart = newStart;
	  }
	  return 1;
	}
      } {
	ros::Duration deltaTdurPre = t - imRBTimes[imRingBufferSize-1];
	ros::Duration deltaTdurPost = t - imRBTimes[0];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Mat m1 = imRingBuffer[imRingBufferSize-1];
	  Mat m2 = imRingBuffer[0];
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;
	  if (w1 >= w2)
	    value = m1;
	  else
	    value = m2;

	  int newStart = imRingBufferSize-1;
	  if(drawSlack) {
	    imRingBufferStart = newStart;
	  }
	  return 1;
	}
      } for (int s = 0; s < imRingBufferEnd; s++) {
	ros::Duration deltaTdurPre = t - imRBTimes[s];
	ros::Duration deltaTdurPost = t - imRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Mat m1 = imRingBuffer[s];
	  Mat m2 = imRingBuffer[s+1];
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;
	  if (w1 >= w2)
	    value = m1;
	  else
	    value = m2;

	  int newStart = s;
	  if(drawSlack) {
	    imRingBufferStart = newStart;
	  }
	  return 1;
	}
      }
      // if we didn't find it we should return failure
      //cout << "Denied out of order range value in getRingImageAtTime(): Too large." << endl;
      return -2;
    }
  }
}
int getRingRangeAtTime(ros::Time t, double &value, int drawSlack) {
  if (rgRingBufferStart == rgRingBufferEnd) {
    cout << "Denied request in getRingRangeAtTime(): Buffer empty." << endl;
    return 0;
  } else {
    int earliestSlot = rgRingBufferStart;
    ros::Duration deltaTdur = t - rgRBTimes[earliestSlot];
    // if the request comes before our earliest record, deny
    if (deltaTdur.toSec() <= 0.0) {
      cout << "Denied out of order range value in getRingRangeAtTime(): Too small." << endl;
      return -1;
    } else if (rgRingBufferStart < rgRingBufferEnd) {
      for (int s = rgRingBufferStart; s < rgRingBufferEnd; s++) {
	ros::Duration deltaTdurPre = t - rgRBTimes[s];
	ros::Duration deltaTdurPost = t - rgRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  double r1 = rgRingBuffer[s];
	  double r2 = rgRingBuffer[s+1];
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;
	  value = w1*r1 + w2*r2;

	  int newStart = s;
	  if(drawSlack) {
	    rgRingBufferStart = newStart;
	  }
	  return 1;
	}
      }
      // if we didn't find it we should return failure
      cout << "Denied out of order range value in getRingRangeAtTime(): Too large." << endl;
      return -2;
    } else {
      for (int s = rgRingBufferStart; s < rgRingBufferSize-1; s++) {
	ros::Duration deltaTdurPre = t - rgRBTimes[s];
	ros::Duration deltaTdurPost = t - rgRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  double r1 = rgRingBuffer[s];
	  double r2 = rgRingBuffer[s+1];
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;
	  value = w1*r1 + w2*r2;

	  int newStart = s;
	  if(drawSlack) {
	    rgRingBufferStart = newStart;
	  }
	  return 1;
	}
      } {
	ros::Duration deltaTdurPre = t - rgRBTimes[rgRingBufferSize-1];
	ros::Duration deltaTdurPost = t - rgRBTimes[0];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  double r1 = rgRingBuffer[rgRingBufferSize-1];
	  double r2 = rgRingBuffer[0];
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;
	  value = w1*r1 + w2*r2;

	  int newStart = rgRingBufferSize-1;
	  if(drawSlack) {
	    rgRingBufferStart = newStart;
	  }
	  return 1;
	}
      } for (int s = 0; s < rgRingBufferEnd; s++) {
	ros::Duration deltaTdurPre = t - rgRBTimes[s];
	ros::Duration deltaTdurPost = t - rgRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  double r1 = rgRingBuffer[s];
	  double r2 = rgRingBuffer[s+1];
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;
	  value = w1*r1 + w2*r2;

	  int newStart = s;
	  if(drawSlack) {
	    rgRingBufferStart = newStart;
	  }
	  return 1;
	}
      }
      // if we didn't find it we should return failure
      cout << "Denied out of order range value in getRingRangeAtTime(): Too large." << endl;
      return -2;
    }
  }
}
int getRingPoseAtTime(ros::Time t, geometry_msgs::Pose &value, int drawSlack) {
  if (epRingBufferStart == epRingBufferEnd) {
    cout << "Denied request in getRingPoseAtTime(): Buffer empty." << endl;
    return 0;
  } else {
    int earliestSlot = epRingBufferStart;
    ros::Duration deltaTdur = t - epRBTimes[earliestSlot];
    // if the request comes before our earliest record, deny
    if (deltaTdur.toSec() <= 0.0) {
      cout << "Denied out of order range value in getRingPoseAtTime(): Too small." << endl;
      return -1;
    } else if (epRingBufferStart < epRingBufferEnd) {
      for (int s = epRingBufferStart; s < epRingBufferEnd; s++) {
	ros::Duration deltaTdurPre = t - epRBTimes[s];
	ros::Duration deltaTdurPost = t - epRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Quaternionf q1 = extractQuatFromPose(epRingBuffer[s]);
	  Quaternionf q2 = extractQuatFromPose(epRingBuffer[s+1]);
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;

	  Quaternionf tTerp = q1.slerp(w2, q2);
	  value.orientation.w = tTerp.w();
	  value.orientation.x = tTerp.x();
	  value.orientation.y = tTerp.y();
	  value.orientation.z = tTerp.z();
	  value.position.x = epRingBuffer[s].position.x*w1 + epRingBuffer[s+1].position.x*w2;
	  value.position.y = epRingBuffer[s].position.y*w1 + epRingBuffer[s+1].position.y*w2;
	  value.position.z = epRingBuffer[s].position.z*w1 + epRingBuffer[s+1].position.z*w2;
//cout << value << endl;
//cout << "33333c " << epRingBuffer[s] << " " << w1 << " " << w2 << " " << totalWeight << endl;
//cout << "44444c " << epRingBuffer[s+1] << endl;

	  int newStart = s;
	  if(drawSlack) {
	    epRingBufferStart = newStart;
	  }
	  return 1;
	}
      }
      // if we didn't find it we should return failure
      cout << "Denied out of order range value in getRingPoseAtTime(): Too large." << endl;
      return -2;
    } else {
      for (int s = epRingBufferStart; s < epRingBufferSize-1; s++) {
	ros::Duration deltaTdurPre = t - epRBTimes[s];
	ros::Duration deltaTdurPost = t - epRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Quaternionf q1 = extractQuatFromPose(epRingBuffer[s]);
	  Quaternionf q2 = extractQuatFromPose(epRingBuffer[s+1]);
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;

	  Quaternionf tTerp = q1.slerp(w2, q2);
	  value.orientation.w = tTerp.w();
	  value.orientation.x = tTerp.x();
	  value.orientation.y = tTerp.y();
	  value.orientation.z = tTerp.z();
	  value.position.x = epRingBuffer[s].position.x*w1 + epRingBuffer[s+1].position.x*w2;
	  value.position.y = epRingBuffer[s].position.y*w1 + epRingBuffer[s+1].position.y*w2;
	  value.position.z = epRingBuffer[s].position.z*w1 + epRingBuffer[s+1].position.z*w2;
//cout << value << endl;
//cout << "33333b " << epRingBuffer[s] << " " << w1 << " " << w2 << " " << totalWeight << endl;
//cout << "44444b " << epRingBuffer[s+1] << endl;

	  int newStart = s;
	  if(drawSlack) {
	    epRingBufferStart = newStart;
	  }
	  return 1;
	}
      } {
	ros::Duration deltaTdurPre = t - epRBTimes[epRingBufferSize-1];
	ros::Duration deltaTdurPost = t - epRBTimes[0];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Quaternionf q1 = extractQuatFromPose(epRingBuffer[epRingBufferSize-1]);
	  Quaternionf q2 = extractQuatFromPose(epRingBuffer[0]);
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;

	  Quaternionf tTerp = q1.slerp(w2, q2);
	  value.orientation.w = tTerp.w();
	  value.orientation.x = tTerp.x();
	  value.orientation.y = tTerp.y();
	  value.orientation.z = tTerp.z();
	  value.position.x = epRingBuffer[epRingBufferSize-1].position.x*w1 + epRingBuffer[0].position.x*w2;
	  value.position.y = epRingBuffer[epRingBufferSize-1].position.y*w1 + epRingBuffer[0].position.y*w2;
	  value.position.z = epRingBuffer[epRingBufferSize-1].position.z*w1 + epRingBuffer[0].position.z*w2;
//cout << value << endl;
//cout << "33333a " << epRingBuffer[epRingBufferSize-1] << " " << w1 << " " << w2 << " " << totalWeight << endl;
//cout << "44444a " << epRingBuffer[0] << endl;

	  int newStart = epRingBufferSize-1;
	  if(drawSlack) {
	    epRingBufferStart = newStart;
	  }
	  return 1;
	}
      } for (int s = 0; s < epRingBufferEnd; s++) {
	ros::Duration deltaTdurPre = t - epRBTimes[s];
	ros::Duration deltaTdurPost = t - epRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Quaternionf q1 = extractQuatFromPose(epRingBuffer[s]);
	  Quaternionf q2 = extractQuatFromPose(epRingBuffer[s+1]);
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;

	  Quaternionf tTerp = q1.slerp(w2, q2);
	  value.orientation.w = tTerp.w();
	  value.orientation.x = tTerp.x();
	  value.orientation.y = tTerp.y();
	  value.orientation.z = tTerp.z();
	  value.position.x = epRingBuffer[s].position.x*w1 + epRingBuffer[s+1].position.x*w2;
	  value.position.y = epRingBuffer[s].position.y*w1 + epRingBuffer[s+1].position.y*w2;
	  value.position.z = epRingBuffer[s].position.z*w1 + epRingBuffer[s+1].position.z*w2;
//cout << value << endl;
//cout << "33333d " << epRingBuffer[s] << " " << w1 << " " << w2 << " " << totalWeight << endl;
//cout << "44444d " << epRingBuffer[s+1] << endl;

	  int newStart = s;
	  if(drawSlack) {
	    epRingBufferStart = newStart;
	  }
	  return 1;
	}
      }
      // if we didn't find it we should return failure
      cout << "Denied out of order range value in getRingPoseAtTime(): Too large." << endl;
      return -2;
    }
  }
}

void setRingImageAtTime(ros::Time t, Mat& imToSet) {
  #ifdef DEBUG2
  cout << "setRingImageAtTime() start end size: " << imRingBufferStart << " " << imRingBufferEnd << " " << imRingBufferSize << endl;
  #endif

  // if the ring buffer is empty, always re-initialize
  if (imRingBufferStart == imRingBufferEnd) {
    imRingBufferStart = 0;
    imRingBufferEnd = 1;
    imRingBuffer[0] = imToSet;
    imRBTimes[0] = t;
  } else {
    ros::Duration deltaTdur = t - imRBTimes[imRingBufferStart];
    if (deltaTdur.toSec() <= 0.0) {
      cout << "Dropped out of order range value in setRingImageAtTime(). " << imRBTimes[imRingBufferStart].toSec() << " " << t.toSec() << " " << deltaTdur.toSec() << " " << endl;
    } else {
      int slot = imRingBufferEnd;
      imRingBuffer[slot] = imToSet;
      imRBTimes[slot] = t;

      if (imRingBufferEnd >= (imRingBufferSize-1)) {
	imRingBufferEnd = 0;
      } else {
	imRingBufferEnd++;
      }

      if (imRingBufferEnd == imRingBufferStart) {
	if (imRingBufferStart >= (imRingBufferSize-1)) {
	  imRingBufferStart = 0;
	} else {
	  imRingBufferStart++;
	}
      }
    }
  }
}
void setRingRangeAtTime(ros::Time t, double rgToSet) {
  #ifdef DEBUG2
  cout << "setRingRangeAtTime() start end size: " << rgRingBufferStart << " " << rgRingBufferEnd << " " << rgRingBufferSize << endl;
  #endif

  // if the ring buffer is empty, always re-initialize
  if (rgRingBufferStart == rgRingBufferEnd) {
    rgRingBufferStart = 0;
    rgRingBufferEnd = 1;
    rgRingBuffer[0] = rgToSet;
    rgRBTimes[0] = t;
  } else {
    ros::Duration deltaTdur = t - rgRBTimes[rgRingBufferStart];
    if (deltaTdur.toSec() <= 0.0) {
      cout << "Dropped out of order range value in setRingRangeAtTime(). " << rgRBTimes[rgRingBufferStart].toSec() << " " << t.toSec() << " " << deltaTdur.toSec() << " " << endl;
    } else {
      int slot = rgRingBufferEnd;
      rgRingBuffer[slot] = rgToSet;
      rgRBTimes[slot] = t;

      if (rgRingBufferEnd >= (rgRingBufferSize-1)) {
	rgRingBufferEnd = 0;
      } else {
	rgRingBufferEnd++;
      }

      if (rgRingBufferEnd == rgRingBufferStart) {
	if (rgRingBufferStart >= (rgRingBufferSize-1)) {
	  rgRingBufferStart = 0;
	} else {
	  rgRingBufferStart++;
	}
      }
    }
  }
}
void setRingPoseAtTime(ros::Time t, geometry_msgs::Pose epToSet) {
  #ifdef DEBUG2
  cout << "setRingPoseAtTime() start end size: " << epRingBufferStart << " " << epRingBufferEnd << " " << epRingBufferSize << endl;
  #endif

  // if the ring buffer is empty, always re-initialize
  if (epRingBufferStart == epRingBufferEnd) {
    epRingBufferStart = 0;
    epRingBufferEnd = 1;
    epRingBuffer[0] = epToSet;
//cout << epToSet << endl;
//cout << "11111 " << epRingBuffer[0] << endl;
    epRBTimes[0] = t;
  } else {
    ros::Duration deltaTdur = t - epRBTimes[epRingBufferStart];
    if (deltaTdur.toSec() <= 0.0) {
      cout << "Dropped out of order range value in setRingPoseAtTime(). " << epRBTimes[epRingBufferStart].toSec() << " " << t.toSec() << " " << deltaTdur.toSec() << " " << endl;
    } else {
      int slot = epRingBufferEnd;
      epRingBuffer[slot] = epToSet;
//cout << epToSet << endl;
//cout << "22222" << epRingBuffer[slot] << endl;
      epRBTimes[slot] = t;

      if (epRingBufferEnd >= (epRingBufferSize-1)) {
	epRingBufferEnd = 0;
      } else {
	epRingBufferEnd++;
      }

      if (epRingBufferEnd == epRingBufferStart) {
	if (epRingBufferStart >= (epRingBufferSize-1)) {
	  epRingBufferStart = 0;
	} else {
	  epRingBufferStart++;
	}
      }
    }
  }
}

void imRingBufferAdvance() {
  if (imRingBufferEnd != imRingBufferStart) {
    if (imRingBufferStart >= (imRingBufferSize-1)) {
      imRingBufferStart = 0;
    } else {
      imRingBufferStart++;
    }
  }
}
void rgRingBufferAdvance() {
  if (rgRingBufferEnd != rgRingBufferStart) {
    if (rgRingBufferStart >= (rgRingBufferSize-1)) {
      rgRingBufferStart = 0;
    } else {
      rgRingBufferStart++;
    }
  }
}
void epRingBufferAdvance() {
  if (epRingBufferEnd != epRingBufferStart) {
    if (epRingBufferStart >= (epRingBufferSize-1)) {
      epRingBufferStart = 0;
    } else {
      epRingBufferStart++;
    }
  }
}

// advance the buffers until we have only enough
//  data to account back to time t
void allRingBuffersAdvance(ros::Time t) {

// TODO XXX make sure that the drawslack parameter is implemented
  double thisRange;
  Mat thisIm;
  geometry_msgs::Pose thisPose;

  getRingPoseAtTime(t, thisPose, 1);
  getRingImageAtTime(t, thisIm, 1);
  //getRingRangeAtTime(t, thisRange, 1);
}

void recordReadyRangeReadings() {
  // if we have some range readings to process
  if (rgRingBufferEnd != rgRingBufferStart) {

    // continue until it is empty or we don't have data for a point yet
    int IShouldContinue = 1;
    while (IShouldContinue) {
      if (rgRingBufferEnd == rgRingBufferStart) {
	IShouldContinue = 0; // not strictly necessary
	break; 
      }
	
      double thisRange = rgRingBuffer[rgRingBufferStart];
      ros::Time thisTime = rgRBTimes[rgRingBufferStart];
    
      geometry_msgs::Pose thisPose;
      Mat thisImage;
      int weHavePoseData = getRingPoseAtTime(thisTime, thisPose);
      int weHaveImData = getRingImageAtTime(thisTime, thisImage);

      #ifdef DEBUG2
      cout << "  recordReadyRangeReadings()  weHavePoseData weHaveImData: " << weHavePoseData << " " << weHaveImData << endl;
      #endif

      // if this request will never be serviceable then forget about it
      if (weHavePoseData == -1) {
	rgRingBufferAdvance();
	IShouldContinue = 1; // not strictly necessary
	#ifdef DEBUG2
	#endif
	cout << "  recordReadyRangeReadings(): dropping stale packet due to epRing. consider increasing buffer size." << endl;
      }
      if (weHaveImData == -1) {
	rgRingBufferAdvance();
	IShouldContinue = 1; // not strictly necessary
	#ifdef DEBUG2
	#endif
	cout << "  recordReadyRangeReadings(): dropping stale packet due to imRing. consider increasing buffer size." << endl;
      } 
      if ((weHavePoseData == 1) && (weHaveImData == 1)) {

	// XXX TODO throw out the reading if it is outside of valid range bounds

	// actually storing the negative z for backwards compatibility
	double thisZmeasurement = -(thisPose.position.z - thisRange);
	double dX = 0;
	double dY = 0;
	double dZ = 0;

	Eigen::Vector3d rayDirection;

	{
	  // XXX 
	  //Eigen::Quaternionf crane2quat(crane2right.qw, crane2right.qx, crane2right.qy, crane2right.qz);
	  //Eigen::Quaternionf crane2quat(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);

	  //Eigen::Quaternionf crane2quat = getGGRotation(currentGraspGear);
	  //Eigen::Quaternionf gear0offset(0.0, ggX[currentGraspGear], ggY[currentGraspGear], 0.0); // for initial calibration
	  //irGlobalPositionEEFrame = crane2quat.conjugate() * gear0offset * crane2quat;


	  Eigen::Quaternionf ceeQuat(thisPose.orientation.w, thisPose.orientation.x, thisPose.orientation.y, thisPose.orientation.z);
	  Eigen::Quaternionf irSensorStartLocal = ceeQuat * irGlobalPositionEEFrame * ceeQuat.conjugate();
	  Eigen::Quaternionf irSensorStartGlobal(
						  0.0,
						 (thisPose.position.x - irSensorStartLocal.x()),
						 (thisPose.position.y - irSensorStartLocal.y()),
						 (thisPose.position.z - irSensorStartLocal.z())
						);

	  Eigen::Quaternionf globalUnitZ(0, 0, 0, 1);
	  Eigen::Quaternionf localUnitZ = ceeQuat * globalUnitZ * ceeQuat.conjugate();

	  //Eigen::Quaternionf irSensorEnd = irSensorStartLocal + (thisRange * localUnitZ);
	  Eigen::Vector3d irSensorEnd(
				       (thisPose.position.x - irSensorStartLocal.x()) + thisRange*localUnitZ.x(),
				       (thisPose.position.y - irSensorStartLocal.y()) + thisRange*localUnitZ.y(),
				       (thisPose.position.z - irSensorStartLocal.z()) + thisRange*localUnitZ.z()
				      );

	  dX = (irSensorEnd.x() - rmcX); //(thisPose.position.x - drX) - rmcX;
	  dY = (irSensorEnd.y() - rmcY); //(thisPose.position.y - drY) - rmcY;
	  dZ = (irSensorEnd.z() - rmcZ); //(thisPose.position.y - drY) - rmcY;

	  double eX = (irSensorEnd.x() - rmcX) / hrmDelta;
	  double eY = (irSensorEnd.y() - rmcY) / hrmDelta;
	  int eeX = (int)round(eX + hrmHalfWidth);
	  int eeY = (int)round(eY + hrmHalfWidth);

	  #ifdef DEBUG
	  cout << "irSensorEnd w x y z: " << irSensorEnd.w() << " " << 
	    irSensorEnd.x() << " " << irSensorEnd.y() << " " << irSensorEnd.z() << endl;
	  cout << "irSensorStartGlobal w x y z: " << irSensorStartGlobal.w() << " " << 
	    irSensorStartGlobal.x() << " " << irSensorStartGlobal.y() << " " << irSensorStartGlobal.z() << endl;
	  cout << "Corrected x y: " << (thisPose.position.x - drX) << " " << (thisPose.position.y - drY) << endl;
	  cout.flush();
	  #endif

//cout << thisPose.orientation << thisPose.position << " " << eX << " " << eY << " " << thisRange << endl;
	  if ((fabs(eX) <= hrmHalfWidth) && (fabs(eY) <= hrmHalfWidth))
	    hiRangemapImage.at<cv::Vec3b>(eeX,eeY) += cv::Vec3b(0,0,128);
	  // XXX
	  thisZmeasurement = -irSensorEnd.z();

	  rayDirection = Eigen::Vector3d(localUnitZ.x(), localUnitZ.y(), localUnitZ.z());
	}

	double iX = dX / rmDelta;
	double iY = dY / rmDelta;

	double hiX = dX / hrmDelta;
	double hiY = dY / hrmDelta;
	double hiZ = dZ / hrmDelta;

	// draw new cell
	if ((fabs(hiX) <= hrmHalfWidth) && (fabs(hiY) <= hrmHalfWidth)) {
	  int hiiX = (int)round(hiX + hrmHalfWidth);
	  int hiiY = (int)round(hiY + hrmHalfWidth);
	  int hiiZ = (int)round(hiZ + hrmHalfWidth);

	  // the wrong point without pose correction
	  //double upX = ((trueEEPose.position.x - drX) - rmcX)/hrmDelta;
	  //double upY = ((trueEEPose.position.y - drY) - rmcY)/hrmDelta;
	  //int iupX = (int)round(upX + hrmHalfWidth);
	  //int iupY = (int)round(upY + hrmHalfWidth);
	  //if ((fabs(upX) <= hrmHalfWidth) && (fabs(upY) <= hrmHalfWidth)) 
	    //hiRangemapImage.at<cv::Vec3b>(iupX,iupY) += cv::Vec3b(0,128,0);

	  // 2D map
	  {
	    int pxMin = max(0, hiiX-parzenKernelHalfWidth);
	    int pxMax = min(hrmWidth-1, hiiX+parzenKernelHalfWidth);
	    int pyMin = max(0, hiiY-parzenKernelHalfWidth);
	    int pyMax = min(hrmWidth-1, hiiY+parzenKernelHalfWidth);
	    // correct loop order for cache coherency
	    for (int py = pyMin; py <= pyMax; py++) {
	      for (int px = pxMin; px <= pxMax; px++) {
		int kpx = px - (hiiX - parzenKernelHalfWidth);
		int kpy = py - (hiiY - parzenKernelHalfWidth);

		cv::Vec3b thisSample = getCRColor(thisImage); 
		hiColorRangeMapAccumulator[px + py*hrmWidth + 0*hrmWidth*hrmWidth] += thisSample[0]*parzenKernel[kpx + kpy*parzenKernelWidth];
		hiColorRangeMapAccumulator[px + py*hrmWidth + 1*hrmWidth*hrmWidth] += thisSample[1]*parzenKernel[kpx + kpy*parzenKernelWidth];
		hiColorRangeMapAccumulator[px + py*hrmWidth + 2*hrmWidth*hrmWidth] += thisSample[2]*parzenKernel[kpx + kpy*parzenKernelWidth];
		hiColorRangeMapMass[px + py*hrmWidth] += parzenKernel[kpx + kpy*parzenKernelWidth];

		double denomC = max(hiColorRangeMapMass[px + py*hrmWidth], EPSILON);
		int tRed = min(255, max(0,int(round(hiColorRangeMapAccumulator[px + py*hrmWidth + 2*hrmWidth*hrmWidth] / denomC))));
		int tGreen = min(255, max(0,int(round(hiColorRangeMapAccumulator[px + py*hrmWidth + 1*hrmWidth*hrmWidth] / denomC))));
		int tBlue = min(255, max(0,int(round(hiColorRangeMapAccumulator[px + py*hrmWidth + 0*hrmWidth*hrmWidth] / denomC))));

		hiColorRangemapImage.at<cv::Vec3b>(px,py) = cv::Vec3b(tBlue, tGreen, tRed);

		hiRangeMapAccumulator[px + py*hrmWidth] += thisZmeasurement*parzenKernel[kpx + kpy*parzenKernelWidth];
		hiRangeMapMass[px + py*hrmWidth] += parzenKernel[kpx + kpy*parzenKernelWidth];
		double denom = max(hiRangeMapMass[px + py*hrmWidth], EPSILON);
		hiRangeMap[px + py*hrmWidth] = hiRangeMapAccumulator[px + py*hrmWidth] / denom;
	      }
	    }
	  }
	  // record the point in the 3D maps
	  // positive surface observation
	  {
	    int pxMin = max(0, hiiX-parzen3DKernelHalfWidth);
	    int pxMax = min(vmWidth-1, hiiX+parzen3DKernelHalfWidth);
	    int pyMin = max(0, hiiY-parzen3DKernelHalfWidth);
	    int pyMax = min(vmWidth-1, hiiY+parzen3DKernelHalfWidth);
	    int pzMin = max(0, hiiZ-parzen3DKernelHalfWidth);
	    int pzMax = min(vmWidth-1, hiiZ+parzen3DKernelHalfWidth);
	    // correct loop order for cache coherency
	    for (int pz = pzMin; pz <= pzMax; pz++) {
	      for (int py = pyMin; py <= pyMax; py++) {
		for (int px = pxMin; px <= pxMax; px++) {
		  int kpx = px - (hiiX - parzen3DKernelHalfWidth);
		  int kpy = py - (hiiY - parzen3DKernelHalfWidth);
		  int kpz = pz - (hiiZ - parzen3DKernelHalfWidth);

		  cv::Vec3b thisSample = getCRColor(thisImage); 
		  vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 0*vmWidth*vmWidth*vmWidth] += thisSample[0]*parzen3DKernel[kpx + kpy*parzen3DKernelWidth + kpz*parzen3DKernelWidth*parzen3DKernelWidth];
		  vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 1*vmWidth*vmWidth*vmWidth] += thisSample[1]*parzen3DKernel[kpx + kpy*parzen3DKernelWidth + kpz*parzen3DKernelWidth*parzen3DKernelWidth];
		  vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 2*vmWidth*vmWidth*vmWidth] += thisSample[2]*parzen3DKernel[kpx + kpy*parzen3DKernelWidth + kpz*parzen3DKernelWidth*parzen3DKernelWidth];
		  vmColorRangeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth] += parzen3DKernel[kpx + kpy*parzen3DKernelWidth + kpz*parzen3DKernelWidth*parzen3DKernelWidth];

		  //double denomC = max(vmColorRangeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth], EPSILON);
		  //int tRed = min(255, max(0,int(round(vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 2*vmWidth*vmWidth*vmWidth] / denomC))));
		  //int tGreen = min(255, max(0,int(round(vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 1*vmWidth*vmWidth*vmWidth] / denomC))));
		  //int tBlue = min(255, max(0,int(round(vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 0*vmWidth*vmWidth*vmWidth] / denomC))));

		  // slightly different than 2D
		  volumeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth] += parzen3DKernel[kpx + kpy*parzen3DKernelWidth + kpz*parzen3DKernelWidth*parzen3DKernelWidth];
		  //volumeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth] += 1.0;
		  volumeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth] += parzen3DKernel[kpx + kpy*parzen3DKernelWidth + kpz*parzen3DKernelWidth*parzen3DKernelWidth];

		  double denom = max(volumeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth], 1e-99); // XXX should be epsilon but there is clipping...
		  volumeMap[px + py*vmWidth + pz*vmWidth*vmWidth] = volumeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth] / denom;
		}
	      }
	    }
	  }
	  double negativeSpacing = 1.0*parzen3DKernelSigma*vmDelta;
	  //int numCastPoints = int(ceil(thisRange / negativeSpacing));
	  int numCastPoints = 10;
	  // negative surface observations
	  for (int castPoint = 1; castPoint <= numCastPoints; castPoint++) {
	    double piX = (dX - negativeSpacing*castPoint*rayDirection.x())/ hrmDelta;
	    double piY = (dY - negativeSpacing*castPoint*rayDirection.y()) / hrmDelta;
	    double piZ = (dZ - negativeSpacing*castPoint*rayDirection.z()) / hrmDelta;

	    int piiX = (int)round(piX + hrmHalfWidth);
	    int piiY = (int)round(piY + hrmHalfWidth);
	    int piiZ = (int)round(piZ + hrmHalfWidth);
	    

	    int pxMin = max(0, piiX-parzen3DKernelHalfWidth);
	    int pxMax = min(vmWidth-1, piiX+parzen3DKernelHalfWidth);
	    int pyMin = max(0, piiY-parzen3DKernelHalfWidth);
	    int pyMax = min(vmWidth-1, piiY+parzen3DKernelHalfWidth);
	    int pzMin = max(0, piiZ-parzen3DKernelHalfWidth);
	    int pzMax = min(vmWidth-1, piiZ+parzen3DKernelHalfWidth);
	    // correct loop order for cache coherency
	    for (int pz = pzMin; pz <= pzMax; pz++) {
	      for (int py = pyMin; py <= pyMax; py++) {
		for (int px = pxMin; px <= pxMax; px++) {
		  int kpx = px - (piiX - parzen3DKernelHalfWidth);
		  int kpy = py - (piiY - parzen3DKernelHalfWidth);
		  int kpz = pz - (piiZ - parzen3DKernelHalfWidth);

		  cv::Vec3b thisSample = getCRColor(thisImage); 
		  vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 0*vmWidth*vmWidth*vmWidth] += thisSample[0]*parzen3DKernel[kpx + kpy*parzen3DKernelWidth + kpz*parzen3DKernelWidth*parzen3DKernelWidth];
		  vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 1*vmWidth*vmWidth*vmWidth] += thisSample[1]*parzen3DKernel[kpx + kpy*parzen3DKernelWidth + kpz*parzen3DKernelWidth*parzen3DKernelWidth];
		  vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 2*vmWidth*vmWidth*vmWidth] += thisSample[2]*parzen3DKernel[kpx + kpy*parzen3DKernelWidth + kpz*parzen3DKernelWidth*parzen3DKernelWidth];
		  vmColorRangeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth] += parzen3DKernel[kpx + kpy*parzen3DKernelWidth + kpz*parzen3DKernelWidth*parzen3DKernelWidth];

		  //double denomC = max(vmColorRangeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth], EPSILON);
		  //int tRed = min(255, max(0,int(round(vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 2*vmWidth*vmWidth*vmWidth] / denomC))));
		  //int tGreen = min(255, max(0,int(round(vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 1*vmWidth*vmWidth*vmWidth] / denomC))));
		  //int tBlue = min(255, max(0,int(round(vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 0*vmWidth*vmWidth*vmWidth] / denomC))));

		  // slightly different than 2D
		  //volumeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth] += 0.0;
		  volumeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth] += parzen3DKernel[kpx + kpy*parzen3DKernelWidth + kpz*parzen3DKernelWidth*parzen3DKernelWidth];

		  double denom = max(volumeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth], 1e-99); // XXX should be epsilon but there is clipping...
		  volumeMap[px + py*vmWidth + pz*vmWidth*vmWidth] = volumeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth] / denom;
		}
	      }
	    }
	  }
	}
	if ((fabs(thisiX) <= rmHalfWidth) && (fabs(thisiY) <= rmHalfWidth)) {
	  int iiX = (int)round(thisiX + rmHalfWidth);
	  int iiY = (int)round(thisiY + rmHalfWidth);
	  
	  {
	    rangeMapMass[iiX + iiY*rmWidth] += 1;
	    //rangeMapAccumulator[iiX + iiY*rmWidth] += eeRange;
	    rangeMapAccumulator[iiX + iiY*rmWidth] += thisZmeasurement;
	    double denom = max(rangeMapMass[iiX + iiY*rmWidth], EPSILON);
	    rangeMap[iiX + iiY*rmWidth] = rangeMapAccumulator[iiX + iiY*rmWidth] / denom;
	  }
	  
	  double minDepth = INFINITY;
	  double maxDepth = 0;
	  for (int rx = 0; rx < rmWidth; rx++) {
	    for (int ry = 0; ry < rmWidth; ry++) {
	      minDepth = min(minDepth, rangeMap[rx + ry*rmWidth]);
	      maxDepth = max(maxDepth, rangeMap[rx + ry*rmWidth]);
	    }
	  }
	  double denom2 = max(EPSILON,maxDepth-minDepth);
	  if (denom2 <= EPSILON)
	    denom2 = INFINITY;
	  double intensity = 255 * (maxDepth - rangeMap[iiX + iiY*rmWidth]) / denom2;
	  cv::Scalar backColor(0,0,ceil(intensity));
	  cv::Point outTop = cv::Point(iiY*rmiCellWidth,iiX*rmiCellWidth);
	  cv::Point outBot = cv::Point((iiY+1)*rmiCellWidth,(iiX+1)*rmiCellWidth);
	  Mat vCrop = rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
	  vCrop = backColor;
	  // draw border
	  {
	    cv::Point outTop = cv::Point(iiY*rmiCellWidth+1,iiX*rmiCellWidth+1);
	    cv::Point outBot = cv::Point((iiY+1)*rmiCellWidth-1,(iiX+1)*rmiCellWidth-1);
	    cv::Point inTop = cv::Point(outTop.x+1, outTop.y+1);
	    cv::Point inBot = cv::Point(outBot.x-1, outBot.y-1);
	    rectangle(rangemapImage, outTop, outBot, cv::Scalar(0,192,0)); 
	    rectangle(rangemapImage, inTop, inBot, cv::Scalar(0,64,0)); 
	  }
	}




	// XXX TODO actually record the point
    
	rgRingBufferAdvance();
	allRingBuffersAdvance(thisTime);
	IShouldContinue = 1; // not strictly necessary
      } else {
	IShouldContinue = 0;
	break;
      }
    }
  }
  #ifdef DEBUG2
  cout << "recordReadyRangeReadings()  rgRingBufferStart rgRingBufferEnd: " << rgRingBufferStart << " " << rgRingBufferEnd << endl;
  #endif
}

void jointCallback(const sensor_msgs::JointState& js) {
  if (jointNamesInit) {
    int limit = js.position.size();
    for (int i = 0; i < limit; i++) {
      for (int j = 0; j < numJoints; j++) {
	if (0 == js.name[i].compare(jointNames[j]))
	  trueJointPositions[j] = js.position[i];
	//cout << "tJP[" << j << "]: " << trueJointPositions[j] << endl;
      }
    }
  }
}

void endpointCallback(const baxter_core_msgs::EndpointState& eps) {
  //cout << "endpoint frame_id: " << eps.header.frame_id << endl;

  trueEEPose = eps.pose;

  setRingPoseAtTime(eps.header.stamp, eps.pose);

  geometry_msgs::Pose thisPose;
  int weHavePoseData = getRingPoseAtTime(eps.header.stamp, thisPose);
}




void initialize3DParzen() {
  for (int kx = 0; kx < parzen3DKernelWidth; kx++) {
    for (int ky = 0; ky < parzen3DKernelWidth; ky++) {
      for (int kz = 0; kz < parzen3DKernelWidth; kz++) {
	double pkx = kx - parzen3DKernelHalfWidth;
	double pky = ky - parzen3DKernelHalfWidth;
	double pkz = ky - parzen3DKernelHalfWidth;
	parzen3DKernel[kx + ky*parzen3DKernelWidth + kz*parzen3DKernelWidth*parzen3DKernelWidth] = exp(-(pkx*pkx + pky*pky + pkz*pkz)/(2.0*parzen3DKernelSigma*parzen3DKernelSigma));
      }
    }
  }
}

void l2Normalize3DParzen() {
  double norm = 0;
  for (int kx = 0; kx < parzen3DKernelWidth; kx++) {
    for (int ky = 0; ky < parzen3DKernelWidth; ky++) {
      for (int kz = 0; kz < parzen3DKernelWidth; kz++) {
	double pkx = kx - parzen3DKernelHalfWidth;
	double pky = ky - parzen3DKernelHalfWidth;
	double pkz = ky - parzen3DKernelHalfWidth;
	norm += parzen3DKernel[kx + ky*parzen3DKernelWidth + kz*parzen3DKernelWidth*parzen3DKernelWidth];
      }
    }
  }
  if (fabs(norm) < fEpsilon)
    norm = 1;
  for (int kx = 0; kx < parzen3DKernelWidth; kx++) {
    for (int ky = 0; ky < parzen3DKernelWidth; ky++) {
      for (int kz = 0; kz < parzen3DKernelWidth; kz++) {
	double pkx = kx - parzen3DKernelHalfWidth;
	double pky = ky - parzen3DKernelHalfWidth;
	double pkz = ky - parzen3DKernelHalfWidth;

	parzen3DKernel[kx + ky*parzen3DKernelWidth + kz*parzen3DKernelWidth*parzen3DKernelWidth] /= norm;
	cout << "Parzen3D: " << parzenKernel[kx + ky*parzenKernelWidth] << endl;
      }
    }
  }
}

void initializeParzen() {
  for (int kx = 0; kx < parzenKernelWidth; kx++) {
    for (int ky = 0; ky < parzenKernelWidth; ky++) {
      double pkx = kx - parzenKernelHalfWidth;
      double pky = ky - parzenKernelHalfWidth;
      parzenKernel[kx + ky*parzenKernelWidth] = exp(-(pkx*pkx + pky*pky)/(2.0*parzenKernelSigma*parzenKernelSigma));
    }
  }
}


void l2NormalizeParzen() {
  double norm = 0;
  for (int kx = 0; kx < parzenKernelWidth; kx++) {
    for (int ky = 0; ky < parzenKernelWidth; ky++) {
      double pkx = kx - parzenKernelHalfWidth;
      double pky = ky - parzenKernelHalfWidth;
      norm += parzenKernel[kx + ky*parzenKernelWidth];
    }
  }
  if (fabs(norm) < fEpsilon)
    norm = 1;
  for (int kx = 0; kx < parzenKernelWidth; kx++) {
    for (int ky = 0; ky < parzenKernelWidth; ky++) {
      double pkx = kx - parzenKernelHalfWidth;
      double pky = ky - parzenKernelHalfWidth;
      parzenKernel[kx + ky*parzenKernelWidth] /= norm;
      cout << "Parzen: " << parzenKernel[kx + ky*parzenKernelWidth] << endl;
    }
  }
}

void l2NormalizeFilter() {
  double norm = 0;
  for (int fx = 0; fx < 9; fx++) {
    norm += filter[fx]*filter[fx];
  }
  if (fabs(norm) < fEpsilon)
    norm = 1;
  for (int fx = 0; fx < 9; fx++) {
    filter[fx] /= norm;
  }
}


int getColorReticleX() {
  // rounding
  //int tcri = int(round((eeRange - firstCReticleIndexDepth)/cReticleIndexDelta));
  //tcri = min(max(tcri,0),numCReticleIndeces-1);
  //return xCR[tcri];

  // interpolating
  int tcriL = int(floor((eeRange - firstCReticleIndexDepth)/cReticleIndexDelta));
  int tcriH = int(ceil((eeRange - firstCReticleIndexDepth)/cReticleIndexDelta));
  tcriL = min(max(tcriL,0),numCReticleIndeces-1);
  tcriH = min(max(tcriH,0),numCReticleIndeces-1);

  double tcrwL = ((eeRange - firstCReticleIndexDepth)/cReticleIndexDelta) - floor((eeRange - firstCReticleIndexDepth)/cReticleIndexDelta);
  double tcrwH = ceil((eeRange - firstCReticleIndexDepth)/cReticleIndexDelta) - ((eeRange - firstCReticleIndexDepth)/cReticleIndexDelta);

  if (tcriL == tcriH)
    return xCR[tcriL];
  else
    return int(round(tcrwL*double(xCR[tcriL]) + tcrwH*double(xCR[tcriH])));
}

int getColorReticleY() {
  // rounding
  //int tcri = int(round((eeRange - firstCReticleIndexDepth)/cReticleIndexDelta));
  //tcri = min(max(tcri,0),numCReticleIndeces-1);
  //return yCR[tcri];

  // interpolating
  int tcriL = int(floor((eeRange - firstCReticleIndexDepth)/cReticleIndexDelta));
  int tcriH = int(ceil((eeRange - firstCReticleIndexDepth)/cReticleIndexDelta));
  tcriL = min(max(tcriL,0),numCReticleIndeces-1);
  tcriH = min(max(tcriH,0),numCReticleIndeces-1);

  double tcrwL = ((eeRange - firstCReticleIndexDepth)/cReticleIndexDelta) - floor((eeRange - firstCReticleIndexDepth)/cReticleIndexDelta);
  double tcrwH = ceil((eeRange - firstCReticleIndexDepth)/cReticleIndexDelta) - ((eeRange - firstCReticleIndexDepth)/cReticleIndexDelta);

  if (tcriL == tcriH)
    return yCR[tcriL];
  else
    return int(round(tcrwL*double(yCR[tcriL]) + tcrwH*double(yCR[tcriH])));
}

cv::Vec3b getCRColor() {
  cv::Vec3b toReturn(0,0,0);
  if (wristCamInit) {
    int crX = getColorReticleX();
    int crY = getColorReticleY();

    if ((crX < wristCamImage.cols) && (crY < wristCamImage.rows))
      toReturn = wristCamImage.at<cv::Vec3b>(crY,crX); 
  }
  return toReturn;
}

cv::Vec3b getCRColor(Mat im) {
  cv::Vec3b toReturn(0,0,0);

  int crX = getColorReticleX();
  int crY = getColorReticleY();

  if ((crX < im.cols) && (crY < im.rows))
    toReturn = im.at<cv::Vec3b>(crY,crX); 

  return toReturn;
}

Quaternionf extractQuatFromPose(geometry_msgs::Pose poseIn) {
  return Quaternionf(poseIn.orientation.w, poseIn.orientation.x, poseIn.orientation.y, poseIn.orientation.z);
}


void pushNoOps(int n) {
  for (int i = 0; i < n; i++)
    pilot_call_stack.push_back('C'); 
}

void pushSpeedSign(double speed) {

  if (speed == MOVE_FAST)
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
  if (speed == MOVE_MEDIUM)
    pilot_call_stack.push_back(1048686); // set speed to MOVE_MEDIUM
  if (speed == MOVE_SLOW)
    pilot_call_stack.push_back(1114190); // set speed to MOVE_SLOW
  if (speed == MOVE_VERY_SLOW)
    pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW

}
void scanXdirection(double speedOnLines, double speedBetweenLines) {

  int scanPadding = 0;
  double onLineGain = rmDelta / speedOnLines;
  double betweenLineGain = rmDelta / speedBetweenLines;

  for (int g = 0; g < ((rmWidth*onLineGain)-(rmHalfWidth*onLineGain))+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('a');
  }
  for (int g = 0; g < rmHalfWidth*onLineGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('e');
  }
  pushSpeedSign(speedOnLines);

  //int gLimit = 1+((rmWidth*betweenLineGain+2*scanPadding)/2);
  int gLimit = ((rmWidth*betweenLineGain+2*scanPadding));
  for (int g = 0; g < gLimit; g++) {
    pilot_call_stack.push_back(1114183); // full render
    pilot_call_stack.push_back(1048677);
    pushSpeedSign(speedOnLines);
    pilot_call_stack.push_back('d');
    pushSpeedSign(speedBetweenLines);
    for (int gg = 0; gg < rmWidth*onLineGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('q');
    }
    //pushSpeedSign(speedOnLines);
    //pilot_call_stack.push_back('d');
    //pushSpeedSign(speedBetweenLines);
    for (int gg = 0; gg < rmWidth*onLineGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('e');
    }
  }
  for (int g = 0; g < rmHalfWidth*onLineGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('q');
  }
  for (int g = 0; g < rmHalfWidth*onLineGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('a');
  }
  pushSpeedSign(speedOnLines);

}

void scanYdirection(double speedOnLines, double speedBetweenLines) {

  int scanPadding = 0;
  double onLineGain = rmDelta / speedOnLines;
  double betweenLineGain = rmDelta / speedBetweenLines;

  for (int g = 0; g < ((rmWidth*onLineGain)-(rmHalfWidth*onLineGain))+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('q');
  }
  for (int g = 0; g < rmHalfWidth*onLineGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('d');
  }
  pushSpeedSign(speedOnLines);

  int gLimit = 1+((rmWidth*betweenLineGain+2*scanPadding)/2);
  for (int g = 0; g < gLimit; g++) {
    pilot_call_stack.push_back(1114183); // full render
    pilot_call_stack.push_back(1048677);
    pushSpeedSign(speedOnLines);
    pilot_call_stack.push_back('e');
    pushSpeedSign(speedBetweenLines);
    for (int gg = 0; gg < rmWidth*onLineGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('a');
    }
    pushSpeedSign(speedOnLines);
    pilot_call_stack.push_back('e');
    pushSpeedSign(speedBetweenLines);
    for (int gg = 0; gg < rmWidth*onLineGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('d');
    }
  }

  for (int g = 0; g < rmHalfWidth*onLineGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('q');
  }
  for (int g = 0; g < rmHalfWidth*onLineGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('a');
  }
  pushSpeedSign(speedOnLines);
}

void scanYdirectionMedium(double speedOnLines, double speedBetweenLines) {
  // VERY SLOW progressive scan
  int scanPadding = 0;
  double rmbGain = rmDelta / bDelta;
  //pilot_call_stack.push_back(1048689);
  for (int g = 0; g < ((rmWidth*rmbGain)-(rmHalfWidth*rmbGain))+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('q');
  }
  for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('d');
  }
  for (int g = 0; g < rmWidth*rmbGain+2*scanPadding; g++) {
    pilot_call_stack.push_back(1114183); // full render
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('e');
    pilot_call_stack.push_back(1048686); // set speed to MOVE_MEDIUM
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('a');
    }
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('e');
    pilot_call_stack.push_back(1048686); // set speed to MOVE_MEDIUM
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('d');
    }
  }
  for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('q');
  }
  for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('a');
  }
  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
}

void scanXdirectionMedium(double speedOnLines, double speedBetweenLines) {
  // VERY SLOW progressive scan
  int scanPadding = 0;
  double rmbGain = rmDelta / bDelta;
  //pilot_call_stack.push_back(1048689);
  for (int g = 0; g < ((rmWidth*rmbGain)-(rmHalfWidth*rmbGain))+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('a');
  }
  for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('e');
  }
  for (int g = 0; g < rmWidth*rmbGain+2*scanPadding; g++) {
    pilot_call_stack.push_back(1114183); // full render
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('d');
    pilot_call_stack.push_back(1048686); // set speed to MOVE_MEDIUM
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('q');
    }
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('d');
    pilot_call_stack.push_back(1048686); // set speed to MOVE_MEDIUM
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('e');
    }
  }
  for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('q');
  }
  for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('a');
  }
  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 

}

void scanYdirectionVerySlow(double speedOnLines, double speedBetweenLines) {
  // VERY SLOW progressive scan
  int scanPadding = 0;
  double rmbGain = rmDelta / bDelta;
  //pilot_call_stack.push_back(1048689);
  for (int g = 0; g < ((rmWidth*rmbGain)-(rmHalfWidth*rmbGain))+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('q');
  }
  for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('d');
  }
  for (int g = 0; g < rmWidth*rmbGain+2*scanPadding; g++) {
    pilot_call_stack.push_back(1114183); // full render
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('e');
    pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('a');
    }
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('e');
    pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('d');
    }
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('e');
    pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('a');
    }
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('e');
    pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('d');
    }
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('e');
    pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('a');
    }
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('e');
    pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('d');
    }
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('e');
    pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('a');
    }
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('e');
    pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('d');
    }
  }
  for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('q');
  }
  for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('a');
  }
  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
}

void scanXdirectionVerySlow(double speedOnLines, double speedBetweenLines) {
  // VERY SLOW progressive scan
  int scanPadding = 0;
  double rmbGain = rmDelta / bDelta;
  //pilot_call_stack.push_back(1048689);
  for (int g = 0; g < ((rmWidth*rmbGain)-(rmHalfWidth*rmbGain))+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('a');
  }
  for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('e');
  }
  for (int g = 0; g < rmWidth*rmbGain+2*scanPadding; g++) {
    pilot_call_stack.push_back(1114183); // full render
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('d');
    pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('q');
    }
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('d');
    pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('e');
    }
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('d');
    pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('q');
    }
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('d');
    pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('e');
    }
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('d');
    pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('q');
    }
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('d');
    pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('e');
    }
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('d');
    pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('q');
    }
    pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
    pilot_call_stack.push_back('d');
    pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
    for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
      pilot_call_stack.push_back(1048677);
      pilot_call_stack.push_back('e');
    }
  }
  for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('q');
  }
  for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
    pilot_call_stack.push_back(1048677);
    pilot_call_stack.push_back('a');
  }
  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 

}

Eigen::Quaternionf getGGRotation(int givenGraspGear) {
  Eigen::Vector3f localUnitX;
  {
    Eigen::Quaternionf qin(0, 1, 0, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    //Eigen::Quaternionf eeqform(eepReg2.qw, eepReg2.qx, eepReg2.qy, eepReg2.qz);
    Eigen::Quaternionf eeqform(0, 0, 1.0, 0);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitX.x() = qout.x();
    localUnitX.y() = qout.y();
    localUnitX.z() = qout.z();
  }

  Eigen::Vector3f localUnitY;
  {
    Eigen::Quaternionf qin(0, 0, 1, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    //Eigen::Quaternionf eeqform(eepReg2.qw, eepReg2.qx, eepReg2.qy, eepReg2.qz);
    Eigen::Quaternionf eeqform(0, 0, 1.0, 0);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitY.x() = qout.x();
    localUnitY.y() = qout.y();
    localUnitY.z() = qout.z();
  }

  Eigen::Vector3f localUnitZ;
  {
    Eigen::Quaternionf qin(0, 0, 0, 1);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    //Eigen::Quaternionf eeqform(eepReg2.qw, eepReg2.qx, eepReg2.qy, eepReg2.qz);
    Eigen::Quaternionf eeqform(0, 0, 1.0, 0);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitZ.x() = qout.x();
    localUnitZ.y() = qout.y();
    localUnitZ.z() = qout.z();
  }

  double deltaTheta = double(givenGraspGear)*2.0*3.1415926/double(totalGraspGears);
  double sinBuff = 0.0;
  double angleRate = 1.0;
  //Eigen::Quaternionf eeBaseQuat(eepReg2.qw, eepReg2.qx, eepReg2.qy, eepReg2.qz);
  Eigen::Quaternionf eeBaseQuat(0, 0, 1, 0);
  sinBuff = sin(angleRate*0.0/2.0);
  Eigen::Quaternionf eeRotatorX(cos(angleRate*0.0/2.0), localUnitX.x()*sinBuff, localUnitX.y()*sinBuff, localUnitX.z()*sinBuff);
  sinBuff = sin(angleRate*0.0/2.0);
  Eigen::Quaternionf eeRotatorY(cos(angleRate*0.0/2.0), localUnitY.x()*sinBuff, localUnitY.y()*sinBuff, localUnitY.z()*sinBuff);
  sinBuff = sin(angleRate*deltaTheta/2.0);
  Eigen::Quaternionf eeRotatorZ(cos(angleRate*deltaTheta/2.0), localUnitZ.x()*sinBuff, localUnitZ.y()*sinBuff, localUnitZ.z()*sinBuff);
  eeRotatorX.normalize();
  eeRotatorY.normalize();
  eeRotatorZ.normalize();

  eeBaseQuat = eeRotatorX * eeRotatorY * eeRotatorZ * eeBaseQuat;
  eeBaseQuat.normalize();

  return eeBaseQuat;
}

void setGGRotation(int thisGraspGear) {
  Eigen::Quaternionf eeBaseQuat = getGGRotation(thisGraspGear);

  currentEEPose.qx = eeBaseQuat.x();
  currentEEPose.qy = eeBaseQuat.y();
  currentEEPose.qz = eeBaseQuat.z();
  currentEEPose.qw = eeBaseQuat.w();
}

void rangeCallback(const sensor_msgs::Range& range) {

  //cout << "range frame_id: " << range.header.frame_id << endl;


  setRingRangeAtTime(range.header.stamp, range.range);
  //double thisRange;
  //int weHaveRangeData = getRingRangeAtTime(range.header.stamp, thisRange);

  #ifdef DEBUG
  cout << "debug 3" << endl;
  cout.flush();
  #endif

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

//cout << "Average time between frames: " << aveTime << 
  //"   Average Frequency: " << aveFrequency << " Hz   Duration of sampling: " << 
  //deltaTime << "   Frames since sampling: " << timeMass << endl; 


  eeRange = range.range;
  //cout << eeRange << endl;
  rangeHistory[currentRangeHistoryIndex] = eeRange;
  currentRangeHistoryIndex++;
  currentRangeHistoryIndex = currentRangeHistoryIndex % totalRangeHistoryLength;

  

  //rectangle(rangeogramImage, outTop, outBot, cv::Scalar(0,0,0)); 

  //cv::Scalar fillColor(0,0,0);
  //cv::Point outTop = cv::Point(0, 0);
  //cv::Point outBot = cv::Point(rggWidth, rggHeight);
  //Mat vCrop = rangeogramImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
  //vCrop = fillColor;

  #ifdef DEBUG
  cout << "debug 4" << endl;
  cout.flush();
  #endif

  for (int rr = currentRangeHistoryIndex-1; rr <= currentRangeHistoryIndex; rr++) {
    int r = 0;
    if (rr == -1)
      r = totalRangeHistoryLength-1;
    else
      r = rr;

    cv::Scalar fillColor(0,0,0);
    cv::Scalar backColor(0,0,0);
    int topY = 0;
    if (r == currentRangeHistoryIndex) {
      fillColor = cv::Scalar(0,0,255);
      topY = 0;
    } else {
      fillColor = cv::Scalar(0,64,0);
      double thisHeight = floor(rangeHistory[r]*rggHeight);
      thisHeight = min(thisHeight,double(rggHeight));
      topY = thisHeight;
      //cout << " " << rangeHistory[r] << " " << thisHeight << " " << rggHeight << " " << topY << endl;
    }
    int truH = rggHeight-topY;
    {
      cv::Point outTop = cv::Point(r*rggStride, 0);
      cv::Point outBot = cv::Point((r+1)*rggStride, rggHeight);
      Mat vCrop = rangeogramImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
      vCrop = backColor;
    }{
      cv::Point outTop = cv::Point(r*rggStride, topY);
      cv::Point outBot = cv::Point((r+1)*rggStride, rggHeight);
      Mat vCrop = rangeogramImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
      vCrop += fillColor;
    }
    if (r != currentRangeHistoryIndex) {
      {
	cv::Point outTop = cv::Point(r*rggStride, topY);
	cv::Point outBot = cv::Point((r+1)*rggStride, topY+truH/8);
	Mat vCrop = rangeogramImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
	vCrop += fillColor;
      }{
	cv::Point outTop = cv::Point(r*rggStride, topY);
	cv::Point outBot = cv::Point((r+1)*rggStride, topY+truH/16);
	Mat vCrop = rangeogramImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
	vCrop += fillColor;
      }
    }
  }

  #ifdef DEBUG
  cout << "debug 5" << endl;
  cout.flush();
  #endif

  if (recordRangeMap) {

    // actually storing the negative z for backwards compatibility
    //double thisZmeasurement = -(currentEEPose.pz - eeRange);
    double thisZmeasurement = -(trueEEPose.position.z - eeRange);
    double dX = 0;
    double dY = 0;

    {
      // XXX 
      //Eigen::Quaternionf crane2quat(crane2right.qw, crane2right.qx, crane2right.qy, crane2right.qz);
      //Eigen::Quaternionf crane2quat(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);

      //Eigen::Quaternionf crane2quat = getGGRotation(currentGraspGear);
      //Eigen::Quaternionf gear0offset(0.0, ggX[currentGraspGear], ggY[currentGraspGear], 0.0); // for initial calibration
      //irGlobalPositionEEFrame = crane2quat.conjugate() * gear0offset * crane2quat;


      Eigen::Quaternionf ceeQuat(trueEEPose.orientation.w, trueEEPose.orientation.x, trueEEPose.orientation.y, trueEEPose.orientation.z);
      Eigen::Quaternionf irSensorStartLocal = ceeQuat * irGlobalPositionEEFrame * ceeQuat.conjugate();
      Eigen::Quaternionf irSensorStartGlobal(
					      0.0,
					     (trueEEPose.position.x - irSensorStartLocal.x()),
					     (trueEEPose.position.y - irSensorStartLocal.y()),
					     (trueEEPose.position.z - irSensorStartLocal.z())
					    );

      Eigen::Quaternionf globalUnitZ(0, 0, 0, 1);
      Eigen::Quaternionf localUnitZ = ceeQuat * globalUnitZ * ceeQuat.conjugate();

      //Eigen::Quaternionf irSensorEnd = irSensorStartLocal + (eeRange * localUnitZ);
      Eigen::Vector3d irSensorEnd(
				   (trueEEPose.position.x - irSensorStartLocal.x()) + eeRange*localUnitZ.x(),
				   (trueEEPose.position.y - irSensorStartLocal.y()) + eeRange*localUnitZ.y(),
				   (trueEEPose.position.z - irSensorStartLocal.z()) + eeRange*localUnitZ.z()
				  );

      dX = (irSensorEnd.x() - rmcX); //(trueEEPose.position.x - drX) - rmcX;
      dY = (irSensorEnd.y() - rmcY); //(trueEEPose.position.y - drY) - rmcY;

      double eX = (irSensorEnd.x() - rmcX) / hrmDelta;
      double eY = (irSensorEnd.y() - rmcY) / hrmDelta;
      int eeX = (int)round(eX + hrmHalfWidth);
      int eeY = (int)round(eY + hrmHalfWidth);

      #ifdef DEBUG
      cout << "irSensorEnd w x y z: " << irSensorEnd.w() << " " << 
	irSensorEnd.x() << " " << irSensorEnd.y() << " " << irSensorEnd.z() << endl;
      cout << "irSensorStartGlobal w x y z: " << irSensorStartGlobal.w() << " " << 
	irSensorStartGlobal.x() << " " << irSensorStartGlobal.y() << " " << irSensorStartGlobal.z() << endl;
      cout << "Corrected x y: " << (trueEEPose.position.x - drX) << " " << (trueEEPose.position.y - drY) << endl;
      cout.flush();
      #endif

      if ((fabs(eX) <= hrmHalfWidth) && (fabs(eY) <= hrmHalfWidth))
	hiRangemapImage.at<cv::Vec3b>(eeX,eeY) += cv::Vec3b(128,0,0);
      // XXX
      thisZmeasurement = -irSensorEnd.z();
    }

    // find current rangemap slot
    // check to see if it falls in our mapped region
    // if so, update the arrays and draw the slot
    // XXX
    //double dX = (trueEEPose.position.x - drX) - rmcX;
    //double dY = (trueEEPose.position.y - drY) - rmcY;

    double iX = dX / rmDelta;
    double iY = dY / rmDelta;

    double hiX = dX / hrmDelta;
    double hiY = dY / hrmDelta;

    lastiX = thisiX;
    lastiY = thisiY;
    thisiX = iX;
    thisiY = iY;

    
  //cout << rmcX << " " << trueEEPose.position.x << " " << dX << " " << iX << " " << rmHalfWidth << endl;

    // erase old cell
    if ((fabs(lastiX) <= rmHalfWidth) && (fabs(lastiY) <= rmHalfWidth)) {
      int iiX = (int)round(lastiX + rmHalfWidth);
      int iiY = (int)round(lastiY + rmHalfWidth);

      double minDepth = INFINITY;
      double maxDepth = 0;
      for (int rx = 0; rx < rmWidth; rx++) {
	for (int ry = 0; ry < rmWidth; ry++) {
	  minDepth = min(minDepth, rangeMap[rx + ry*rmWidth]);
	  maxDepth = max(maxDepth, rangeMap[rx + ry*rmWidth]);
	}
      }
      double denom2 = max(EPSILON,maxDepth-minDepth);
      if (denom2 <= EPSILON)
	denom2 = INFINITY;
      double intensity = 255 * (maxDepth - rangeMap[iiX + iiY*rmWidth]) / denom2;
      cv::Scalar backColor(0,0,ceil(intensity));
      cv::Point outTop = cv::Point(iiY*rmiCellWidth,iiX*rmiCellWidth);
      cv::Point outBot = cv::Point((iiY+1)*rmiCellWidth,(iiX+1)*rmiCellWidth);
      Mat vCrop = rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
      vCrop = backColor;
    }



    // draw new cell
    if ((fabs(hiX) <= hrmHalfWidth) && (fabs(hiY) <= hrmHalfWidth)) {
      int hiiX = (int)round(hiX + hrmHalfWidth);
      int hiiY = (int)round(hiY + hrmHalfWidth);

      // the wrong point without pose correction
      //double upX = ((trueEEPose.position.x - drX) - rmcX)/hrmDelta;
      //double upY = ((trueEEPose.position.y - drY) - rmcY)/hrmDelta;
      //int iupX = (int)round(upX + hrmHalfWidth);
      //int iupY = (int)round(upY + hrmHalfWidth);
      //if ((fabs(upX) <= hrmHalfWidth) && (fabs(upY) <= hrmHalfWidth)) 
	//hiRangemapImage.at<cv::Vec3b>(iupX,iupY) += cv::Vec3b(0,128,0);

      int pxMin = max(0, hiiX-parzenKernelHalfWidth);
      int pxMax = min(hrmWidth-1, hiiX+parzenKernelHalfWidth);
      int pyMin = max(0, hiiY-parzenKernelHalfWidth);
      int pyMax = min(hrmWidth-1, hiiY+parzenKernelHalfWidth);
      for (int px = pxMin; px <= pxMax; px++) {
	for (int py = pyMin; py <= pyMax; py++) {
	  int kpx = px - (hiiX - parzenKernelHalfWidth);
	  int kpy = py - (hiiY - parzenKernelHalfWidth);

	  cv::Vec3b thisSample = getCRColor(); 
//	  hiColorRangeMapAccumulator[px + py*hrmWidth + 0*hrmWidth*hrmWidth] += thisSample[0]*parzenKernel[kpx + kpy*parzenKernelWidth];
//	  hiColorRangeMapAccumulator[px + py*hrmWidth + 1*hrmWidth*hrmWidth] += thisSample[1]*parzenKernel[kpx + kpy*parzenKernelWidth];
//	  hiColorRangeMapAccumulator[px + py*hrmWidth + 2*hrmWidth*hrmWidth] += thisSample[2]*parzenKernel[kpx + kpy*parzenKernelWidth];
//	  hiColorRangeMapMass[px + py*hrmWidth] += parzenKernel[kpx + kpy*parzenKernelWidth];
//
//	  double denomC = max(hiColorRangeMapMass[px + py*hrmWidth], EPSILON);
//	  int tRed = min(255, max(0,int(round(hiColorRangeMapAccumulator[px + py*hrmWidth + 2*hrmWidth*hrmWidth] / denomC))));
//	  int tGreen = min(255, max(0,int(round(hiColorRangeMapAccumulator[px + py*hrmWidth + 1*hrmWidth*hrmWidth] / denomC))));
//	  int tBlue = min(255, max(0,int(round(hiColorRangeMapAccumulator[px + py*hrmWidth + 0*hrmWidth*hrmWidth] / denomC))));
//
//	  hiColorRangemapImage.at<cv::Vec3b>(px,py) = cv::Vec3b(tBlue, tGreen, tRed);

	  //hiRangeMapAccumulator[px + py*hrmWidth] += eeRange*parzenKernel[kpx + kpy*parzenKernelWidth];
	  //hiRangeMapAccumulator[px + py*hrmWidth] += thisZmeasurement*parzenKernel[kpx + kpy*parzenKernelWidth];
	  //hiRangeMapMass[px + py*hrmWidth] += parzenKernel[kpx + kpy*parzenKernelWidth];
	  //double denom = max(hiRangeMapMass[px + py*hrmWidth], EPSILON);
	  //hiRangeMap[px + py*hrmWidth] = hiRangeMapAccumulator[px + py*hrmWidth] / denom;
	}
      }
    }
    if ((fabs(thisiX) <= rmHalfWidth) && (fabs(thisiY) <= rmHalfWidth)) {
      int iiX = (int)round(thisiX + rmHalfWidth);
      int iiY = (int)round(thisiY + rmHalfWidth);
      
      {
	//rangeMapMass[iiX + iiY*rmWidth] += 1;
	//rangeMapAccumulator[iiX + iiY*rmWidth] += thisZmeasurement;
	//double denom = max(rangeMapMass[iiX + iiY*rmWidth], EPSILON);
	//rangeMap[iiX + iiY*rmWidth] = rangeMapAccumulator[iiX + iiY*rmWidth] / denom;
      }
      
      double minDepth = INFINITY;
      double maxDepth = 0;
      for (int rx = 0; rx < rmWidth; rx++) {
	for (int ry = 0; ry < rmWidth; ry++) {
	  minDepth = min(minDepth, rangeMap[rx + ry*rmWidth]);
	  maxDepth = max(maxDepth, rangeMap[rx + ry*rmWidth]);
	}
      }
      double denom2 = max(EPSILON,maxDepth-minDepth);
      if (denom2 <= EPSILON)
	denom2 = INFINITY;
      double intensity = 255 * (maxDepth - rangeMap[iiX + iiY*rmWidth]) / denom2;
      cv::Scalar backColor(0,0,ceil(intensity));
      cv::Point outTop = cv::Point(iiY*rmiCellWidth,iiX*rmiCellWidth);
      cv::Point outBot = cv::Point((iiY+1)*rmiCellWidth,(iiX+1)*rmiCellWidth);
      Mat vCrop = rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
      vCrop = backColor;
      // draw border
      {
	cv::Point outTop = cv::Point(iiY*rmiCellWidth+1,iiX*rmiCellWidth+1);
	cv::Point outBot = cv::Point((iiY+1)*rmiCellWidth-1,(iiX+1)*rmiCellWidth-1);
	cv::Point inTop = cv::Point(outTop.x+1, outTop.y+1);
	cv::Point inBot = cv::Point(outBot.x-1, outBot.y-1);
	rectangle(rangemapImage, outTop, outBot, cv::Scalar(0,192,0)); 
	rectangle(rangemapImage, inTop, inBot, cv::Scalar(0,64,0)); 
      }
    }

    #ifdef DEBUG
    cout << "rangeMap: [" << endl;
    for (int rx = 0; rx < rmWidth; rx++) {
      for (int ry = 0; ry < rmWidth; ry++) {
	cout << rangeMap[rx + ry*rmWidth] << " ";
      }
      cout << endl;
    }
    cout << "]" << endl;
    cout.flush();
    #endif
  }

  if (shouldIRender) {
    cv::imshow(rangemapViewName, rangemapImage);
    //cv::imshow(hiRangemapViewName, hiRangemapImage);
    Mat hRIT;
    cv::resize(hiRangemapImage, hRIT, cv::Size(0,0), 2, 2);
    cv::imshow(hiRangemapViewName, hRIT);
    Mat hCRIT;
    cv::resize(hiColorRangemapImage, hCRIT, cv::Size(0,0), 2, 2);
    cv::imshow(hiColorRangemapViewName, hCRIT);

    cv::imshow(objectViewerName, objectViewerImage);
    cv::imshow(densityViewerName, densityViewerImage);
  }
  #ifdef DEBUG
  cout << "debug 1" << endl;
  cout.flush();
  #endif
  {
    cv::Point text_anchor = cv::Point(0,rangeogramImage.rows-1);
    {
      cv::Scalar backColor(0,0,0);
      cv::Point outTop = cv::Point(text_anchor.x,text_anchor.y+1-35);
      cv::Point outBot = cv::Point(text_anchor.x+200,text_anchor.y+1);
      Mat vCrop = rangeogramImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
      vCrop = backColor;
    }
    char buff[256];
    sprintf(buff, "Hz: %.2f", aveFrequency);
    string fpslabel(buff);
    putText(rangeogramImage, fpslabel, text_anchor, MY_FONT, 1.0, Scalar(0,0,160), 1.0);
  }
  cv::imshow(rangeogramViewName, rangeogramImage);
  #ifdef DEBUG
  cout << "debug 2" << endl;
  cout.flush();
  #endif
}


void update_baxter(ros::NodeHandle &n) {
//cout << "block5" << endl;
  bfc = bfc % bfc_period;


  baxter_core_msgs::SolvePositionIK thisIkRequest;
  thisIkRequest.request.pose_stamp.resize(1);

  thisIkRequest.request.pose_stamp[0].header.seq = 0;
  thisIkRequest.request.pose_stamp[0].header.stamp = ros::Time::now();
  thisIkRequest.request.pose_stamp[0].header.frame_id = "/base";

  
  thisIkRequest.request.pose_stamp[0].pose.position.x = currentEEPose.px;
  thisIkRequest.request.pose_stamp[0].pose.position.y = currentEEPose.py;
  thisIkRequest.request.pose_stamp[0].pose.position.z = currentEEPose.pz;

  /* global cartesian update 
  Eigen::Matrix3f m;
  m = Eigen::AngleAxisf(currentEEPose.ox*M_PI, Eigen::Vector3f::UnitX())
  * Eigen::AngleAxisf(currentEEPose.oy*M_PI, Eigen::Vector3f::UnitY())
  * Eigen::AngleAxisf(currentEEPose.oz*M_PI, Eigen::Vector3f::UnitZ());

  Eigen::Quaternionf eeRotator(m);
  Eigen::Quaternionf eeBaseQuat(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);

  eeBaseQuat = eeRotator * eeBaseQuat;
  */ 

  /* end effector local angular update */
  {
    Eigen::Vector3f localUnitX;
    {
      Eigen::Quaternionf qin(0, 1, 0, 0);
      Eigen::Quaternionf qout(0, 1, 0, 0);
      Eigen::Quaternionf eeqform(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
      qout = eeqform * qin * eeqform.conjugate();
      localUnitX.x() = qout.x();
      localUnitX.y() = qout.y();
      localUnitX.z() = qout.z();
    }

    Eigen::Vector3f localUnitY;
    {
      Eigen::Quaternionf qin(0, 0, 1, 0);
      Eigen::Quaternionf qout(0, 1, 0, 0);
      Eigen::Quaternionf eeqform(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
      qout = eeqform * qin * eeqform.conjugate();
      localUnitY.x() = qout.x();
      localUnitY.y() = qout.y();
      localUnitY.z() = qout.z();
    }

    Eigen::Vector3f localUnitZ;
    {
      Eigen::Quaternionf qin(0, 0, 0, 1);
      Eigen::Quaternionf qout(0, 1, 0, 0);
      Eigen::Quaternionf eeqform(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
      qout = eeqform * qin * eeqform.conjugate();
      localUnitZ.x() = qout.x();
      localUnitZ.y() = qout.y();
      localUnitZ.z() = qout.z();
    }

    double sinBuff = 0.0;
    double angleRate = 1.0;
    Eigen::Quaternionf eeBaseQuat(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
    sinBuff = sin(angleRate*currentEEPose.ox/2.0);
    Eigen::Quaternionf eeRotatorX(cos(angleRate*currentEEPose.ox/2.0), localUnitX.x()*sinBuff, localUnitX.y()*sinBuff, localUnitX.z()*sinBuff);
    sinBuff = sin(angleRate*currentEEPose.oy/2.0);
    Eigen::Quaternionf eeRotatorY(cos(angleRate*currentEEPose.oy/2.0), localUnitY.x()*sinBuff, localUnitY.y()*sinBuff, localUnitY.z()*sinBuff);
    sinBuff = sin(angleRate*currentEEPose.oz/2.0);
    Eigen::Quaternionf eeRotatorZ(cos(angleRate*currentEEPose.oz/2.0), localUnitZ.x()*sinBuff, localUnitZ.y()*sinBuff, localUnitZ.z()*sinBuff);
    currentEEPose.ox = 0;
    currentEEPose.oy = 0;
    currentEEPose.oz = 0;
    eeRotatorX.normalize();
    eeRotatorY.normalize();
    eeRotatorZ.normalize();
    //eeBaseQuat = eeRotatorX * eeRotatorY * eeRotatorZ * 
		  //eeBaseQuat * 
		//eeRotatorZ.conjugate() * eeRotatorY.conjugate() * eeRotatorX.conjugate();
    eeBaseQuat = eeRotatorX * eeRotatorY * eeRotatorZ * eeBaseQuat;
    eeBaseQuat.normalize();

    thisIkRequest.request.pose_stamp[0].pose.orientation.x = eeBaseQuat.x();
    thisIkRequest.request.pose_stamp[0].pose.orientation.y = eeBaseQuat.y();
    thisIkRequest.request.pose_stamp[0].pose.orientation.z = eeBaseQuat.z();
    thisIkRequest.request.pose_stamp[0].pose.orientation.w = eeBaseQuat.w();

    currentEEPose.qx = eeBaseQuat.x();
    currentEEPose.qy = eeBaseQuat.y();
    currentEEPose.qz = eeBaseQuat.z();
    currentEEPose.qw = eeBaseQuat.w();
  }

//cout << "block6" << endl;

  int ikResult = 0;


  // do not start in a state with ikShare 
  if ((drand48() <= ikShare) || !ikInitialized) {
    ikResult = (!ikClient.call(thisIkRequest) || !thisIkRequest.response.isValid[0]);

  /*
    if ( ikClient.waitForExistence(ros::Duration(1, 0)) ) {
  //cout << "block6.1" << endl;
      ikResult = (!ikClient.call(thisIkRequest) || !thisIkRequest.response.isValid[0]);
    } else {
      cout << "waitForExistence timed out" << endl;
      ikResult = 1;
    }
  */

    if (ikResult) 
    {
      ROS_ERROR_STREAM("ikClient says pose request is invalid.");
      ik_reset_counter++;

      if (ik_reset_counter > ik_reset_thresh)
	currentEEPose = ik_reset_eePose;
      else
	currentEEPose = lastGoodEEPose;

      return;
    }
    ik_reset_counter = 0;

    lastGoodEEPose = currentEEPose;
    ikRequest = thisIkRequest;
    ikInitialized = 1;
  }
  
//cout << "block7" << endl;


  /*
  // using the joint controllers
  // rosmsg show control_msgs/FollowJointTrajectoryAction
  control_msgs::FollowJointTrajectoryActionGoal goal;

  goal.header.seq = 0;
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "base";
  goal.goal.trajectory.header = goal.header;

  goal.goal.trajectory.points.resize(1);
  goal.goal.trajectory.points[0].positions.resize(7);
  goal.goal.trajectory.points[0].velocities.resize(7);
  goal.goal.trajectory.points[0].time_from_start = ros::Duration(2.0);


  for (int j = 0; j < numJoints; j++) {
    goal.goal.trajectory.joint_names.push_back(ikRequest.response.joints[0].name[j]);
    goal.goal.trajectory.points[0].positions[j] = ikRequest.response.joints[0].position[j];
    goal.goal.trajectory.points[0].velocities[j] = 0;
  }


  cout << "1" << endl;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> myClient(n, "/robot/left_velocity_trajectory_controller/follow_joint_trajectory", true);
  cout << "2" << endl;
  myClient.waitForServer();
  cout << "3" << endl;
  myClient.sendGoal(goal.goal);
  cout << "4" << endl;
  myClient.waitForResult(ros::Duration(5.0));
  cout << "5" << endl;
  if (myClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Yay! The dishes are now clean");
  */ 

  // but in theory we can bypass the joint controllers by publishing to this topic
  // /robot/limb/left/joint_command

  baxter_core_msgs::JointCommand myCommand;

  if (!jointNamesInit) {
    jointNames.resize(numJoints);
    for (int j = 0; j < numJoints; j++) {
      jointNames[j] = ikRequest.response.joints[0].name[j];
    }
    jointNamesInit = 1;
  }

  if (driveVelocities) {

    double l2Gravity = 0.0;

    myCommand.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;
    myCommand.command.resize(numJoints);
    myCommand.names.resize(numJoints);

    ros::Time theNow = ros::Time::now();
    ros::Duration howLong = theNow - oscilStart;

    for (int j = 0; j < numJoints; j++) {
      myCommand.names[j] = ikRequest.response.joints[0].name[j];
      //myCommand.command[j] = 0.0;
      myCommand.command[j] = spiralEta*rapidJointScales[j]*(ikRequest.response.joints[0].position[j] - trueJointPositions[j]);
      //myCommand.command[j] = sin(rapidJointGlobalOmega[j]*howLong.toSec());
    }
    {
      double tim = howLong.toSec();
      myCommand.command[4] += -rapidAmp1*rapidJointScales[4]*sin(rapidJointLocalBias[4] + (rapidJointLocalOmega[4]*rapidJointGlobalOmega[4]*tim));
      myCommand.command[3] +=  rapidAmp1*rapidJointScales[3]*cos(rapidJointLocalBias[3] + (rapidJointLocalOmega[3]*rapidJointGlobalOmega[3]*tim));

      //myCommand.command[5] += -rapidAmp1*rapidJointScales[4]*sin(rapidJointLocalBias[4] + (rapidJointLocalOmega[4]*rapidJointGlobalOmega[4]*tim));
      //myCommand.command[0] +=  rapidAmp1*rapidJointScales[3]*cos(rapidJointLocalBias[3] + (rapidJointLocalOmega[3]*rapidJointGlobalOmega[3]*tim));

      myCommand.command[5] += -rapidAmp2*rapidJointScales[5]*sin(rapidJointLocalBias[5] + (rapidJointLocalOmega[5]*rapidJointGlobalOmega[5]*tim));
      myCommand.command[0] +=  rapidAmp2*rapidJointScales[0]*cos(rapidJointLocalBias[0] + (rapidJointLocalOmega[0]*rapidJointGlobalOmega[0]*tim));

      //myCommand.command[6] +=  rapidAmp1*rapidJointScales[6]*2.0*((0 < cos(0.1*3.1415926*tim))-0.5);
    }
  } else {
    myCommand.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    myCommand.command.resize(numJoints);
    myCommand.names.resize(numJoints);

    for (int j = 0; j < numJoints; j++) {
      myCommand.names[j] = ikRequest.response.joints[0].name[j];
      //myCommand.command[j] = ikRequest.response.joints[0].position[j] + rapidJointGlobalOmega[j];
      myCommand.command[j] = ikRequest.response.joints[0].position[j];
    }
  }
//cout << "block8" << endl;

  for (int r = 0; r < resend_times; r++) {
    joint_mover.publish(myCommand);
  }

  bfc++;
//cout << "block9" << endl;
}

void timercallback1(const ros::TimerEvent&) {

  #ifdef DEBUG
  cout << "debug 6" << endl;
  cout.flush();
  #endif

//cout << "block1" << endl;
  //eePose redTargetEEPose;

  eePose redTargetEEPose = beeHome;

  // target a chosen red box
  /*
  if ((numRedBoxes > 0) && (redTrackbarVariable > 0))
    if (redBoxes[redTrackbarVariable - 1].persistence > persistenceThresh) {
      tf::StampedTransform transform;
      try {
	geometry_msgs::PointStamped pStampedIn;
	geometry_msgs::PointStamped pStampedOut;
	pStampedIn.header.seq = 0;
	pStampedIn.header.stamp = ros::Time::now() - ros::Duration(tfPast);
	pStampedIn.header.frame_id = "/camera_rgb_optical_frame";
	pStampedIn.point.x = redBoxes[redTrackbarVariable - 1].com.px;
	pStampedIn.point.y = redBoxes[redTrackbarVariable - 1].com.py;
	pStampedIn.point.z = redBoxes[redTrackbarVariable - 1].com.pz;

//	try {
//	    //listener.waitForTransform(destination_frame, original_frame, ros::Time(0), ros::Duration(10.0) );
//	    //listener.lookupTransform(destination_frame, original_frame, ros::Time(0), transform);
//	    tfListener->waitForTransform("/base", "/camera_rgb_optical_frame", ros::Time(0), ros::Duration(10.0) );
//	    tfListener->transformPoint("/base", pStampedIn, pStampedOut); 
//	} catch (tf::TransformException ex) {
//	    ROS_ERROR("%s",ex.what());
//	}


	tfListener->transformPoint("/base", pStampedIn, pStampedOut); 



	redTargetEEPose.px = pStampedOut.point.x;
	redTargetEEPose.py = pStampedOut.point.y;
	redTargetEEPose.pz = pStampedOut.point.z;
	redTargetEEPose.ox = currentEEPose.ox; 
	redTargetEEPose.oy = currentEEPose.oy;
	redTargetEEPose.oz = currentEEPose.oz;
      } catch (tf::TransformException &ex) {
	ROS_ERROR("%s",ex.what());
	//ROS_ERROR("unable to lookup transform...");
      }
    }
  */

//cout << "block2" << endl;
  //ROS_INFO("Callback 1 triggered");
  ros::NodeHandle n("~");

  int c = cvWaitKey(1);
  int takeSymbol = 1;
  if (c != -1) {
    cout << "You pressed " << c << " which when shifted is " << c + 65504 << " and the state is " << currentEEPose.ox << " " << currentEEPose.oy << " " <<currentEEPose.oz << endl;
    takeSymbol = 0;
  }
//cout << "block3" << endl;


  #ifdef DEBUG
  cout << "debug 7" << endl;
  cout.flush();
  #endif
  
  if (!auto_pilot)
    autoPilotFrameCounter = 0;
  // both of these could be moved into the symbolic logic
  // if we have a lock, proceed to grab 
  if ((auto_pilot && go_on_lock && (successive_lock_frames >= slf_thresh)) || ((autoPilotFrameCounter > take_yellow_thresh) && (lock_status == 1))) {
    lock_status = 0;
    successive_lock_frames = 0;
    auto_pilot = 0;
    go_on_lock = 0;
    c = -1;
    zero_g_toggle = 0;
    holding_pattern = 1;
    autoPilotFrameCounter = 0;
  }
  
  // deal with the stack
  if (execute_stack && takeSymbol) {
    if (pilot_call_stack.size() > 0) {
      c = pilot_call_stack.back();
      pilot_call_stack.pop_back();
      current_instruction = c;
    } else {
      execute_stack = 0;
    }
  }

  if (oscillating && (lock_status == 0) && (holding_pattern == 0) && (timerCounter >= oscillatorTimerThresh)) {
    currentEEPose.py = eepReg2.py + oscillatingSign*(oscillatingBias + sin( 2.0 * 3.1415926 * double(timesTimerCounted % oscillatingPeriod) / oscillatingPeriod)) / oscillatingInvAmplitude;
    currentEEPose.px = eepReg2.px;
    currentEEPose.pz = eepReg2.pz;
    ikShare = oscillating_ikShare;
  } else {
    ikShare = default_ikShare;
  }

  if (timerCounter >= lock_reset_thresh) {
    lock_status = 0;
  }
  
  #ifdef DEBUG
  cout << "debug 8 c: " << c << endl;
  cout.flush();
  #endif

  switch (c) {
    case 30: // up arrow
      break;
    case 'j':
      {
	baxter_core_msgs::EndEffectorCommand command;
	command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
	command.args = "{\"position\": 0.0}";
	command.id = 65538;
	gripperPub.publish(command);
      }
      break;
    case 'k':
      {
	baxter_core_msgs::EndEffectorCommand command;
	command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
	command.args = "{\"position\": 100.0}";
	command.id = 65538;
	gripperPub.publish(command);
      }
      break;
    case 'l':
      oscillating = !oscillating;
      break;
    case 'r':
      pilot_call_stack.resize(0);
      break;
    case 't':
      currentEEPose.pz += bDelta * tap_factor;
      break;
    case '7':
      {
	//double deltaX = workCenter.px - currentEEPose.px;
	//double deltaY = workCenter.py - currentEEPose.py;
	double deltaX = eepReg2.px - currentEEPose.px;
	double deltaY = eepReg2.py - currentEEPose.py;
	double xTimes = fabs(floor(deltaX / bDelta)); 
	double yTimes = fabs(floor(deltaY / bDelta)); 

	int tapTimes = 30;

	int numNoOps = 6*(yTimes + xTimes + floor(2 * tapTimes * tap_factor));
	for (int cc = 0; cc < numNoOps; cc++) {
	  pilot_call_stack.push_back('C');
	}

	if (deltaX > 0)
	  for (int xc = 0; xc < xTimes; xc++)
	    pilot_call_stack.push_back('e');
	if (deltaX < 0)
	  for (int xc = 0; xc < xTimes; xc++)
	    pilot_call_stack.push_back('q');
	if (deltaY > 0)
	  for (int yc = 0; yc < yTimes; yc++)
	    pilot_call_stack.push_back('d');
	if (deltaY < 0)
	  for (int yc = 0; yc < yTimes; yc++)
	    pilot_call_stack.push_back('a');
      

	for (int tc = 0; tc < tapTimes; tc++) {
	  pilot_call_stack.push_back('t');
	}
      }
      break;
    case '8': // load program 8
      {
	//double deltaX = workCenter.px - currentEEPose.px;
	//double deltaY = workCenter.py - currentEEPose.py;
	double deltaX = eepReg3.px - currentEEPose.px;
	double deltaY = eepReg3.py - currentEEPose.py;
	double xTimes = fabs(floor(deltaX / bDelta)); 
	double yTimes = fabs(floor(deltaY / bDelta)); 

	int tapTimes = 30;

	int numNoOps = 6*(yTimes + xTimes + floor(2 * tapTimes * tap_factor));
	for (int cc = 0; cc < numNoOps; cc++) {
	  pilot_call_stack.push_back('C');
	}

	if (deltaX > 0)
	  for (int xc = 0; xc < xTimes; xc++)
	    pilot_call_stack.push_back('e');
	if (deltaX < 0)
	  for (int xc = 0; xc < xTimes; xc++)
	    pilot_call_stack.push_back('q');
	if (deltaY > 0)
	  for (int yc = 0; yc < yTimes; yc++)
	    pilot_call_stack.push_back('d');
	if (deltaY < 0)
	  for (int yc = 0; yc < yTimes; yc++)
	    pilot_call_stack.push_back('a');
      

	for (int tc = 0; tc < tapTimes; tc++) {
	  pilot_call_stack.push_back('t');
	}
      }
      break;
    case '9': // load program 9
      {
	pilot_call_stack.push_back('9');
	pilot_call_stack.push_back('x');

	pilot_call_stack.push_back('7');
	pilot_call_stack.push_back('k');
	pilot_call_stack.push_back('8');

	pilot_call_stack.push_back('C');
	pilot_call_stack.push_back(65616);
	pilot_call_stack.push_back('C');
      }
      break;
    case '0': // load program 0
      {
	pilot_call_stack.push_back('0');
	pilot_call_stack.push_back('x');
	int tapTimes = 30;
	for (int tc = 0; tc < tapTimes; tc++) {
	  pilot_call_stack.push_back('t');
	}
	pilot_call_stack.push_back('C');
	pilot_call_stack.push_back(65616);
	pilot_call_stack.push_back('C');
      }
      break;
    case 'y': // execute stack
      execute_stack = 1;
      break;
    case 'u':
      cout << "Current EE Position (x,y,z): " << currentEEPose.px << " " << currentEEPose.py << " " << currentEEPose.pz << endl;
      cout << "Current EE Orientation (x,y,z,w): " << currentEEPose.qx << " " << currentEEPose.qy << " " << currentEEPose.qz << " " << currentEEPose.qw << endl;
      cout << "True EE Position (x,y,z): " << trueEEPose.position.x << " " << trueEEPose.position.y << " " << trueEEPose.position.z << endl;
      cout << "True EE Orientation (x,y,z,w): " << trueEEPose.orientation.x << " " << trueEEPose.orientation.y << " " << trueEEPose.orientation.z << " " << trueEEPose.orientation.w << endl;
      break;
    case 'i':
      {
	baxter_core_msgs::EndEffectorCommand command;
	command.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
	command.id = 65538;
	gripperPub.publish(command);
      }
      break;
    case 'o':
      reticle = defaultReticle;
      break;
    case 'p':
      auto_pilot = !auto_pilot;
      break;
    case 65616: // 'P'
      auto_pilot = 1;
      go_on_lock = 1;
      break;
    case 'x':
      holding_pattern = -1;
      break;
    case 'c':
      {
	holding_pattern = 0;
	auto_pilot = 0;
	go_on_lock = 0;
	execute_stack = 0;
      }
      break;
    case 'C': // continue OR no-op
      {
	if (auto_pilot || (holding_pattern != 0)) {
	  pilot_call_stack.push_back('C');
	} else {
	  holding_pattern = 0;
	  auto_pilot = 0;
	  go_on_lock = 0;
	}
      }
      break;
    case 'v':
      holding_pattern = 1;
      break;
    case 'b':
      holding_pattern = 2;
      break;
    case 'z':
      zero_g_toggle = !zero_g_toggle;
      break;
    // begin cartesian controls
    case 'a':
      currentEEPose.py -= bDelta;
      break;
    case 'd':
      currentEEPose.py += bDelta;
      break;
    case 'w':
      currentEEPose.pz += bDelta;
      break;
    case 's':
      currentEEPose.pz -= bDelta;
      break;
    case 'q':
      currentEEPose.px -= bDelta;
      break;
    case 'e':
      currentEEPose.px += bDelta;
      break;
    // begin angular controls
    /* Global angular controls
    case 'a'+65504:
      currentEEPose.oy -= bDelta;
      break;
    case 'd'+65504:
      currentEEPose.oy += bDelta;
      break;
    case 'w'+65504:
      currentEEPose.oz += bDelta;
      break;
    case 's'+65504:
      currentEEPose.oz -= bDelta;
      break;
    case 'q'+65504:
      currentEEPose.ox -= bDelta;
      break;
    case 'e'+65504:
      currentEEPose.ox += bDelta;
      break;
    */
    /* Local angular controls */
    case 'w'+65504:
      currentEEPose.oy -= bDelta;
      break;
    case 's'+65504:
      currentEEPose.oy += bDelta;
      break;
    case 'e'+65504:
      currentEEPose.oz += bDelta;
      break;
    case 'q'+65504:
      currentEEPose.oz -= bDelta;
      break;
    case 'a'+65504:
      currentEEPose.ox -= bDelta;
      break;
    case 'd'+65504:
      currentEEPose.ox += bDelta;
      break;
    // begin register assignment controls
    case 65568+1: // !
      eepReg1 = currentEEPose;
      break;
    case 65600: // @
      eepReg2 = currentEEPose;
      break;
    case 65568+3: // #
      eepReg3 = currentEEPose;
      break;
    case 65568+4: // $
      eepReg4 = currentEEPose;
      break;
    // begin register recall controls
    case '1':
      currentEEPose = eepReg1;
      break;
    case '2':
      currentEEPose = eepReg2;
      break;
    case '3':
      currentEEPose = eepReg3;
      break;
    case '4':
      currentEEPose = eepReg4;
      break;
    // begin register target acquisition calls
    case 1+262192:
      eepReg1 = redTargetEEPose;
      break;
    case 2+262192:
      eepReg2 = redTargetEEPose;
      break;
    case 3+262192:
      eepReg3 = redTargetEEPose;
      break;
    case 4+262192:
      eepReg4 = redTargetEEPose;
      break;
    case 1048689: 
      // numlock + q
      // future program:
      // execute a grab
      // estimate proper grasp depth and stow in a register
      // move back to the center of the grid      
      // snake around the grid
	// at each point, increment grid counters
	//  and store the current range
      // move to beginning of grid
      {
	currentEEPose.px = rmcX + drX;
	currentEEPose.py = rmcY + drY;

	/*
	// constant speed
	int scanPadding = 0;
	double rmbGain = rmDelta / bDelta;
	//pilot_call_stack.push_back(1048689);
	for (int g = 0; g < ((rmWidth*rmbGain)-(rmHalfWidth*rmbGain))+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('q');
	}
	for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('d');
	}
	for (int g = 0; g < rmWidth*rmbGain+2*scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('e');
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('a');
	  }
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('d');
	  }
	}
	for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('q');
	}
	for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('a');
	}
	*/

	/*
	*/
	// VERY SLOW progressive scan
	int scanPadding = 0;
	double rmbGain = rmDelta / bDelta;
	//pilot_call_stack.push_back(1048689);
	for (int g = 0; g < ((rmWidth*rmbGain)-(rmHalfWidth*rmbGain))+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('q');
	}
	for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('d');
	}
	for (int g = 0; g < rmWidth*rmbGain+2*scanPadding; g++) {
	  pilot_call_stack.push_back(1114183); // full render
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('a');
	  }
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('d');
	  }
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('a');
	  }
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('d');
	  }
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('a');
	  }
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('d');
	  }
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('a');
	  }
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1114178); // set speed to MOVE_VERY_SLOW
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('d');
	  }
	}
	for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('q');
	}
	for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('a');
	}
	pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 

	/*
	// SLOW progressive scan
	int scanPadding = 0;
	double rmbGain = rmDelta / bDelta;
	//pilot_call_stack.push_back(1048689);
	for (int g = 0; g < ((rmWidth*rmbGain)-(rmHalfWidth*rmbGain))+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('q');
	}
	for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('d');
	}
	for (int g = 0; g < rmWidth*rmbGain+2*scanPadding; g++) {
	  pilot_call_stack.push_back(1114183); // full render
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1114190); // set speed to MOVE_SLOW
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('a');
	  }
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1114190); // set speed to MOVE_SLOW
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('d');
	  }
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1114190); // set speed to MOVE_SLOW
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('a');
	  }
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1114190); // set speed to MOVE_SLOW
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('d');
	  }
	}
	for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('q');
	}
	for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('a');
	}
	pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	*/

	/*
	// interlaced scan
	int scanPadding = 0;
	double rmbGain = rmDelta / bDelta;
	//pilot_call_stack.push_back(1048689);
	for (int g = 0; g < ((rmWidth*rmbGain)-(rmHalfWidth*rmbGain))+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('q');
	}
	for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('d');
	}
	for (int g = 0; g < rmWidth*rmbGain+2*scanPadding; g++) {
	  pilot_call_stack.push_back(1114183); // full render
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1048686); // set speed to MOVE_MEDIUM
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('a');
	  }
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1048686); // set speed to MOVE_MEDIUM
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('d');
	  }

//	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
//	  pilot_call_stack.push_back('e');
//	  pilot_call_stack.push_back(1048686); // set speed to MOVE_MEDIUM
//	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
//	    pilot_call_stack.push_back(1048677);
//	    pilot_call_stack.push_back('a');
//	  }
//	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
//	  pilot_call_stack.push_back('e');
//	  pilot_call_stack.push_back(1048686); // set speed to MOVE_MEDIUM
//	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
//	    pilot_call_stack.push_back(1048677);
//	    pilot_call_stack.push_back('d');
//	  }
	  
	}
	for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('q');
	}
	for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('a');
	}
	pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	pilot_call_stack.push_back(1048621); // change offset of position based on current gear 
	*/


	/*
	// progressive with interleaved gear changes...
	int scanPadding = 0;
	double rmbGain = rmDelta / bDelta;
	//pilot_call_stack.push_back(1048689);
	for (int g = 0; g < ((rmWidth*rmbGain)-(rmHalfWidth*rmbGain))+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('q');
	}
	for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('d');
	}
	for (int g = 0; g < rmWidth*rmbGain+2*scanPadding; g++) {
	  pilot_call_stack.push_back(1114183); // full render
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1048686); // set speed to MOVE_MEDIUM
	  pilot_call_stack.push_back(1114155); // rotate gear
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('a');
	  }
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('d');
	  }
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1048686); // set speed to MOVE_MEDIUM
	  pilot_call_stack.push_back(1114155); // rotate gear
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('a');
	  }
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('d');
	  }
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1048686); // set speed to MOVE_MEDIUM
	  pilot_call_stack.push_back(1114155); // rotate gear
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('a');
	  }
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('d');
	  }
	  pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	  pilot_call_stack.push_back('e');
	  pilot_call_stack.push_back(1048686); // set speed to MOVE_MEDIUM
	  pilot_call_stack.push_back(1114155); // rotate gear
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('a');
	  }
	  for (int gg = 0; gg < rmWidth*rmbGain+2*scanPadding; gg++) {
	    pilot_call_stack.push_back(1048677);
	    pilot_call_stack.push_back('d');
	  }
	}
	for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('q');
	}
	for (int g = 0; g < rmHalfWidth*rmbGain+scanPadding; g++) {
	  pilot_call_stack.push_back(1048677);
	  pilot_call_stack.push_back('a');
	}
	pilot_call_stack.push_back(1048674); // set speed to MOVE_FAST 
	*/
      }
      break;
    case 1048695: // numlock + w
      {
	cout << "Set rmcX and rmcY. Resetting maps. " << rmcX << " " << trueEEPose.position.x << endl;
        rmcX = trueEEPose.position.x;
	rmcY = trueEEPose.position.y;
	rmcZ = trueEEPose.position.z - eeRange;
	for (int rx = 0; rx < rmWidth; rx++) {
	  for (int ry = 0; ry < rmWidth; ry++) {
	    rangeMap[rx + ry*rmWidth] = 0;
	    rangeMapReg1[rx + ry*rmWidth] = 0;
	    rangeMapReg2[rx + ry*rmWidth] = 0;
	    rangeMapMass[rx + ry*rmWidth] = 0;
	    rangeMapAccumulator[rx + ry*rmWidth] = 0;
	  }
	}
	{
	  cv::Scalar backColor(128,0,0);
	  cv::Point outTop = cv::Point(0,0);
	  cv::Point outBot = cv::Point(rmiWidth,rmiHeight);
	  Mat vCrop = rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
	  vCrop = backColor;
	}
	for (int rx = 0; rx < hrmWidth; rx++) {
	  for (int ry = 0; ry < hrmWidth; ry++) {
	    hiRangeMap[rx + ry*hrmWidth] = 0;
	    hiRangeMapReg1[rx + ry*hrmWidth] = 0;
	    hiRangeMapReg2[rx + ry*hrmWidth] = 0;
	    hiRangeMapMass[rx + ry*hrmWidth] = 0;
	    hiRangeMapAccumulator[rx + ry*hrmWidth] = 0;
	  }
	}
	{
	  cv::Scalar backColor(128,0,0);
	  cv::Point outTop = cv::Point(0,0);
	  cv::Point outBot = cv::Point(hrmiWidth,hrmiHeight);
	  Mat vCrop = hiRangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
	  vCrop = backColor;
	}
	for (int h = 0; h < hrmWidth; h++) {
	  for (int i = 0; i < hrmWidth; i++) {
	    hiColorRangeMapMass[h + i*hrmWidth] = 0;
	    for (int j = 0; j < 3; j++) {
	      hiColorRangeMapAccumulator[h + i*hrmWidth + j*hrmWidth*hrmWidth] = 0;
	    }
	  }
	}
	for (int pz = 0; pz < vmWidth; pz++) {
	  for (int py = 0; py < vmWidth; py++) {
	    for (int px = 0; px < vmWidth; px++) {
	      volumeMap[px + py*vmWidth + pz*vmWidth*vmWidth] = 0;
	      volumeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth] = 0;
	      volumeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth] = 0;
	      vmColorRangeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth] = 0;
	      for (int pc = 0; pc < 3; pc++) {
		vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + pc*vmWidth*vmWidth*vmWidth] = 0;
	      }
	    }
	  }
	}
	{
	  cv::Scalar backColor(128,0,0);
	  cv::Point outTop = cv::Point(0,0);
	  cv::Point outBot = cv::Point(hiColorRangemapImage.cols,hiColorRangemapImage.rows);
	  Mat vCrop = hiColorRangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
	  vCrop = backColor;
	}
      }
      break;
    case 1048677: // numlock + e
      {
	pilot_call_stack.push_back('C');
	pilot_call_stack.push_back('C');
	pilot_call_stack.push_back('C');
	pilot_call_stack.push_back('C');
	pilot_call_stack.push_back('C');
      }
      break;
    case 1048690: // numlock + r
      {
	for (int rx = 0; rx < rmWidth; rx++) {
	  for (int ry = 0; ry < rmWidth; ry++) {
	    rangeMapReg1[rx + ry*rmWidth] = rangeMap[rx + ry*rmWidth];
	  }
	}
	for (int rx = 0; rx < hrmWidth; rx++) {
	  for (int ry = 0; ry < hrmWidth; ry++) {
	    hiRangeMapReg1[rx + ry*hrmWidth] = hiRangeMap[rx + ry*hrmWidth];
	  }
	}
      }
      break;
    case 1048692: // numlock + t
      {
	cout << "Applying filter to rangeMapReg1 and storing result in rangeMapReg1." << endl;
	int dx[9] = { -1,  0,  1, 
		      -1,  0,  1, 
		      -1,  0,  1};
	int dy[9] = { -1, -1, -1, 
		       0,  0,  0, 
		       1,  1,  1};

	int transformPadding = 2;

	for (int rx = 0; rx < rmWidth; rx++) {
	  for (int ry = 0; ry < rmWidth; ry++) {
	    rangeMapReg2[rx + ry*rmWidth] = 0.0;
	  }
	}
	for (int rx = transformPadding; rx < rmWidth-transformPadding; rx++) {
	  for (int ry = transformPadding; ry < rmWidth-transformPadding; ry++) {
	    for (int fx = 0; fx < 9; fx++)
	      rangeMapReg2[rx + ry*rmWidth] += filter[fx] * rangeMapReg1[(rx+dx[fx]) + (ry+dy[fx])*rmWidth];
	  }
	}
	for (int rx = 0; rx < rmWidth; rx++) {
	  for (int ry = 0; ry < rmWidth; ry++) {
	    rangeMapReg1[rx + ry*rmWidth] = rangeMapReg2[rx + ry*rmWidth];
	  }
	}

	// XXX TODO Consider: 
	// Push boundary to deepest point...
	double minDepth = INFINITY;
	double maxDepth = 0;
	for (int rx = 0; rx < rmWidth; rx++) {
	  for (int ry = 0; ry < rmWidth; ry++) {
	    minDepth = min(minDepth, rangeMapReg1[rx + ry*rmWidth]);
	    maxDepth = max(maxDepth, rangeMapReg1[rx + ry*rmWidth]);
	  }
	}
	for (int rx = 0; rx < rmWidth; rx++) {
	  for (int ry = 0; ry < transformPadding; ry++) {
	    rangeMapReg1[rx + ry*rmWidth] = maxDepth;
	    rangeMapReg2[rx + ry*rmWidth] = maxDepth;
	  }
	  for (int ry = rmWidth-transformPadding; ry < rmWidth; ry++) {
	    rangeMapReg1[rx + ry*rmWidth] = maxDepth;
	    rangeMapReg2[rx + ry*rmWidth] = maxDepth;
	  }
	}
	for (int ry = 0; ry < rmWidth; ry++) {
	  for (int rx = 0; rx < transformPadding; rx++) {
	    rangeMapReg1[rx + ry*rmWidth] = maxDepth;
	    rangeMapReg2[rx + ry*rmWidth] = maxDepth;
	  }
	  for (int rx = rmWidth-transformPadding; rx < rmWidth; rx++) {
	    rangeMapReg1[rx + ry*rmWidth] = maxDepth;
	    rangeMapReg2[rx + ry*rmWidth] = maxDepth;
	  }
	}
      }
      break;
    case 1048697: // numlock + y
      {
	double tfilter[9] = { 1.0/16.0, 1.0/8.0, 1.0/16.0, 
			      1.0/8.0, 1.0/4.0, 1.0/8.0, 
			      1.0/16.0, 1.0/8.0, 1.0/16.0};
	for (int fx = 0; fx < 9; fx++)
	  filter[fx] = tfilter[fx];
      }
      break;
    case 1048693: // numlock + u
      {
	double tfilter[9]    = {   0,  0,  0, 
				  -1,  2, -1, 
				   0,  0,  0};
	for (int fx = 0; fx < 9; fx++)
	  filter[fx] = tfilter[fx];
	l2NormalizeFilter();
	for (int fx = 0; fx < 9; fx++) {
	  cout << filter[fx] << endl;
	}
      }
      break;
    case 1048681: // numlock + i
      {
	double tfilter[9]    = {   0, -1,  0, 
				   0,  2,  0, 
				   0, -1,  0};
	for (int fx = 0; fx < 9; fx++)
	  filter[fx] = tfilter[fx];
	l2NormalizeFilter();
	for (int fx = 0; fx < 9; fx++) {
	  cout << filter[fx] << endl;
	}
      }
      break;
    case 1114185: // numlock + I
      {
	//double tfilter[9]    = {   0, 0,  0, 
				   //-gibbsIota, 1,  gibbsIota, 
				   //0, 0,  0};
	//double tfilter[9]    = {   -gibbsIota, 1,  gibbsIota,
				   //-gibbsIota, 1,  gibbsIota, 
				   //-gibbsIota, 1,  gibbsIota,};

	double tfilter[9]    = {0, -gibbsIota,  0, 
				   0, 1,  0, 
				   0, gibbsIota,  0};
	for (int fx = 0; fx < 9; fx++)
	  filter[fx] = tfilter[fx];
	l2NormalizeFilter();
	for (int fx = 0; fx < 9; fx++) {
	  cout << filter[fx] << endl;
	}
      }
      break;
    case 1048687: // numlock + o
      {
	double tfilter[9]    = {  -1,  0,  0, 
				   0,  2,  0, 
				   0,  0, -1};
	//double tfilter[9]    = {  -1,  0,  0, 
				   //0,  2-diagonalKappa,  0, 
				   //0,  0, -1};
	for (int fx = 0; fx < 9; fx++)
	  filter[fx] = tfilter[fx];
	l2NormalizeFilter();
	for (int fx = 0; fx < 9; fx++) {
	  cout << filter[fx] << " ";
	  filter[fx] *= diagonalKappa;
	  cout << filter[fx] << endl;
	}
      }
      break;
    case 1048688: // numlock + p
      {
	double tfilter[9]    = {   0,  0, -1, 
				   0,  2,  0, 
				  -1,  0,  0};
	//double tfilter[9]    = {   0,  0, -1, 
				   //0,  2-diagonalKappa,  0, 
				  //-1,  0,  0};
	for (int fx = 0; fx < 9; fx++)
	  filter[fx] = tfilter[fx];
	l2NormalizeFilter();
	for (int fx = 0; fx < 9; fx++) {
	  cout << filter[fx] << " ";
	  filter[fx] *= diagonalKappa;
	  cout << filter[fx] << endl;
	}
      }
      break;
    // drawMapRegisters
    // numlock + a 
    case 1048673:
      {
	{
	  double minDepth = INFINITY;
	  double maxDepth = 0;
	  for (int rx = 0; rx < rmWidth; rx++) {
	    for (int ry = 0; ry < rmWidth; ry++) {
	      minDepth = min(minDepth, rangeMapReg1[rx + ry*rmWidth]);
	      maxDepth = max(maxDepth, rangeMapReg1[rx + ry*rmWidth]);
	    }
	  }
	  for (int rx = 0; rx < rmWidth; rx++) {
	    for (int ry = 0; ry < rmWidth; ry++) {
	      double denom = max(EPSILON,maxDepth-minDepth);
	      if (denom <= EPSILON)
		denom = INFINITY;
	      double intensity = 255 * (maxDepth - rangeMapReg1[rx + ry*rmWidth]) / denom;
//cout << denom << " " << maxDepth << " " << rangeMapReg1[rx + ry*rmWidth] << " " << (maxDepth - rangeMapReg1[rx + ry*rmWidth]) << " " << endl;
	      cv::Scalar backColor(0,0,ceil(intensity));
	      cv::Point outTop = cv::Point((ry+rmWidth)*rmiCellWidth,rx*rmiCellWidth);
	      cv::Point outBot = cv::Point(((ry+rmWidth)+1)*rmiCellWidth,(rx+1)*rmiCellWidth);
	      Mat vCrop = rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
	      vCrop = backColor;
	    }
	  }
	}
	{
	  double minDepth = INFINITY;
	  double maxDepth = 0;
	  for (int rx = 0; rx < rmWidth; rx++) {
	    for (int ry = 0; ry < rmWidth; ry++) {
	      minDepth = min(minDepth, rangeMapReg2[rx + ry*rmWidth]);
	      maxDepth = max(maxDepth, rangeMapReg2[rx + ry*rmWidth]);
	    }
	  }
	  for (int rx = 0; rx < rmWidth; rx++) {
	    for (int ry = 0; ry < rmWidth; ry++) {
	      double denom = max(EPSILON,maxDepth-minDepth);
	      if (denom <= EPSILON)
		denom = INFINITY;
	      double intensity = 255 * (maxDepth - rangeMapReg2[rx + ry*rmWidth]) / denom;
	      cv::Scalar backColor(0,0,ceil(intensity));
	      cv::Point outTop = cv::Point((ry+2*rmWidth)*rmiCellWidth,rx*rmiCellWidth);
	      cv::Point outBot = cv::Point(((ry+2*rmWidth)+1)*rmiCellWidth,(rx+1)*rmiCellWidth);
	      Mat vCrop = rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
	      vCrop = backColor;
	    }
	  }
	}
	{
	  double minDepth = INFINITY;
	  double maxDepth = 0;
	  for (int rx = 0; rx < hrmWidth; rx++) {
	    for (int ry = 0; ry < hrmWidth; ry++) {
	      minDepth = min(minDepth, hiRangeMap[rx + ry*hrmWidth]);
	      maxDepth = max(maxDepth, hiRangeMap[rx + ry*hrmWidth]);
	    }
	  }
	  for (int rx = 0; rx < hrmWidth; rx++) {
	    for (int ry = 0; ry < hrmWidth; ry++) {
	      double denom = max(EPSILON,maxDepth-minDepth);
	      if (denom <= EPSILON)
		denom = INFINITY;
	      double intensity = 255 * (maxDepth - hiRangeMap[rx + ry*hrmWidth]) / denom;
	      hiRangemapImage.at<cv::Vec3b>(rx,ry) = cv::Vec3b(0,0,ceil(intensity));
	    }
	  }
	}
	{
	  double minDepth = INFINITY;
	  double maxDepth = 0;
	  for (int rx = 0; rx < hrmWidth; rx++) {
	    for (int ry = 0; ry < hrmWidth; ry++) {
	      minDepth = min(minDepth, hiRangeMapReg1[rx + ry*hrmWidth]);
	      maxDepth = max(maxDepth, hiRangeMapReg1[rx + ry*hrmWidth]);
	    }
	  }
	  for (int rx = 0; rx < hrmWidth; rx++) {
	    for (int ry = 0; ry < hrmWidth; ry++) {
	      double denom = max(EPSILON,maxDepth-minDepth);
	      if (denom <= EPSILON)
		denom = INFINITY;
	      double intensity = 255 * (maxDepth - hiRangeMapReg1[rx + ry*hrmWidth]) / denom;
	      hiRangemapImage.at<cv::Vec3b>(rx,ry+hrmWidth) = cv::Vec3b(0,0,ceil(intensity));
	    }
	  }
	}
	{
	  double minDepth = INFINITY;
	  double maxDepth = 0;
	  for (int rx = 0; rx < hrmWidth; rx++) {
	    for (int ry = 0; ry < hrmWidth; ry++) {
	      minDepth = min(minDepth, hiRangeMapReg2[rx + ry*hrmWidth]);
	      maxDepth = max(maxDepth, hiRangeMapReg2[rx + ry*hrmWidth]);
	    }
	  }
	  for (int rx = 0; rx < hrmWidth; rx++) {
	    for (int ry = 0; ry < hrmWidth; ry++) {
	      double denom = max(EPSILON,maxDepth-minDepth);
	      if (denom <= EPSILON)
		denom = INFINITY;
	      double intensity = 255 * (maxDepth - hiRangeMapReg2[rx + ry*hrmWidth]) / denom;
	      hiRangemapImage.at<cv::Vec3b>(rx,ry+2*hrmWidth) = cv::Vec3b(0,0,ceil(intensity));
	    }
	  }
	}
      }
      break;
    // manual render
    // numlock + A 
    case 1114177:
      {
	pilot_call_stack.push_back(1114189); // rendering off 
	pilot_call_stack.push_back(1048677); // wait 
	pilot_call_stack.push_back(1048677); // wait 
	pilot_call_stack.push_back(1048685); // rendering on
      }
      break;
    // full render
    // numlock + G 
    case 1114183:
      {
	if (!shouldIRender) {
	  pilot_call_stack.push_back(1114177); // manual render
	}
	pilot_call_stack.push_back(1048679); // render reticle
	pilot_call_stack.push_back(1048673); // render register 1
	pilot_call_stack.push_back(1048690); // load map to register 1
      }
      break;
      // stow the max coordinate and grasp angle of MapReg1
      // calculate z coordinate to grab at and assume the max grasp angle orientation
      // move to z coordinate and grab
    // reset window positions
    // numlock + z
    case 1048698:
      {
	int sideOffset = 0;
	int coreWidth = 640;
	int toolbarWidth = 65+1;
	int menuHeight = 28+38;
	int uiOffsetX = toolbarWidth + sideOffset;
	int uiOffsetY = 0;


	cv::moveWindow(rangemapViewName, uiOffsetX, uiOffsetY);
	uiOffsetX += coreWidth;
	cv::moveWindow(hiRangemapViewName, uiOffsetX, uiOffsetY);
	uiOffsetX += 320;
	cv::moveWindow(densityViewerName, uiOffsetX, uiOffsetY);
	uiOffsetX += 320;
	cv::moveWindow(objectViewerName, uiOffsetX, uiOffsetY);
	uiOffsetX = toolbarWidth + sideOffset;
	uiOffsetY += rmiHeight + menuHeight;
	cv::moveWindow(coreViewName, uiOffsetX, uiOffsetY);
	uiOffsetX += coreWidth;
	cv::moveWindow(wristViewName, uiOffsetX, uiOffsetY);
	uiOffsetX += coreWidth;
	cv::moveWindow(rangeogramViewName, uiOffsetX, uiOffsetY);
      }
      break;
    // numlock + x
    case 1048696:
      {
	int sideOffset = 1920+30;
	int coreWidth = 640;
	int toolbarWidth = 65+1;
	int menuHeight = 28+36;
	int uiOffsetX = toolbarWidth + sideOffset;
	int uiOffsetY = 0;

	cv::moveWindow(rangemapViewName, uiOffsetX, uiOffsetY);
	uiOffsetX += coreWidth;
	cv::moveWindow(hiRangemapViewName, uiOffsetX, uiOffsetY);
	uiOffsetX += 320;
	cv::moveWindow(densityViewerName, uiOffsetX, uiOffsetY);
	uiOffsetX += 320;
	cv::moveWindow(objectViewerName, uiOffsetX, uiOffsetY);
	uiOffsetX = toolbarWidth + sideOffset;
	uiOffsetY += rmiHeight + menuHeight;
	cv::moveWindow(coreViewName, uiOffsetX, uiOffsetY);
	cv::moveWindow(hiColorRangemapViewName, uiOffsetX, uiOffsetY);
	uiOffsetX += coreWidth;
	cv::moveWindow(wristViewName, uiOffsetX, uiOffsetY);
	uiOffsetX += coreWidth;
	cv::moveWindow(rangeogramViewName, uiOffsetX, uiOffsetY);
      }
      break;
    // find maxGrasp and save
    // reset min
    // numlock + s
    case 1048691:
      {
	int maxSearchPadding = 3;
	double minDepth = INFINITY;
	for (int rx = maxSearchPadding; rx < rmWidth-maxSearchPadding; rx++) {
	  for (int ry = maxSearchPadding; ry < rmWidth-maxSearchPadding; ry++) {
	    if (rangeMapReg1[rx + ry*rmWidth] < minDepth) {
	      minDepth = rangeMapReg1[rx + ry*rmWidth];
	      maxX = rx;
	      maxY = ry;
	      maxD = rangeMapReg1[rx + ry*rmWidth];
	      maxGG = currentGraspGear;
	    }
	  }
	}
	cout << "non-cumulative maxX: " << maxX << " maxY: " << maxY <<  " maxD: " << maxD << " maxGG: " << maxGG << endl;
      }
      break;
    // cumulative min
    // numlock + S
    case 1114195:
      {
	int maxSearchPadding = 3;
	double minDepth = maxD;
	for (int rx = maxSearchPadding; rx < rmWidth-maxSearchPadding; rx++) {
	  for (int ry = maxSearchPadding; ry < rmWidth-maxSearchPadding; ry++) {
	    if (rangeMapReg1[rx + ry*rmWidth] < minDepth) {
	      minDepth = rangeMapReg1[rx + ry*rmWidth];
	      maxX = rx;
	      maxY = ry;
	      maxD = rangeMapReg1[rx + ry*rmWidth];
	      maxGG = currentGraspGear;
	      cout << "cumulative update maxX: " << maxX << " maxY: " << maxY << " maxD: " << maxD << " maxGG: " << maxGG << endl;
	    }
	  }
	}
      }
      break;
    // set depth reticle to the max mapped position
    // numlock + d
    case 1048676:
      {
	drX = rmDelta*(maxX-rmHalfWidth);
	drY = rmDelta*(maxY-rmHalfWidth);
	cout << "drX: " << drX << " drY: " << drY << endl;
      }
      break;
    // set target reticle to the max mapped position
    // numlock + f
    case 1048678:
      {
	trX = rmcX + rmDelta*(maxX-rmHalfWidth);
	trY = rmcY + rmDelta*(maxY-rmHalfWidth);
	cout << "trX: " << trX << " trY: " << trY << endl;
      }
      break;
    // set hi target reticle by searching over all pixels and all acceptable grasps
    // numlock + F
    case 1114182:
      {
	double weights[7] = 
	    {-1.0/7.0, -1.0/7.0, 
	      1.0/7.0,  1.0/7.0, 1.0/7.0, 
	     -1.0/7.0, -1.0/7.0};

	int rootBuffer = 40;
	
	int searchWidth = 20;

	double httrX = (trX-rmcX)/hrmDelta;
	double httrY = (trY-rmcY)/hrmDelta;
	int hiiX = (int)round(httrX + hrmHalfWidth);
	int hiiY = (int)round(httrY + hrmHalfWidth);
	int hiCellWidth = 5;
	if ((fabs(httrX) <= hrmHalfWidth-hiCellWidth) && (fabs(httrY) <= hrmHalfWidth-hiCellWidth)) {

	  int rootX = min(hrmWidth - rootBuffer, hiiX);
	  rootX = max(rootBuffer, rootX);
	  int rootY = min(hrmWidth - rootBuffer, hiiY);
	  rootY = max(rootBuffer, rootY);

	  double maxScore = -INFINITY;
	  int hMaxX = 0;
	  int hMaxY = 0;

	  for (int rx = rootX - searchWidth; rx <= rootX + searchWidth; rx++) {
	    for (int ry = rootY - searchWidth; ry <= rootY + searchWidth; ry++) {

	      // double thisScore = TODO XXX;
	      double thisScore = 0.0;

	      if (thisScore > maxScore) {
		hMaxX = rx;
		hMaxY = ry;
	      }
	    }
	  }

	  //htrX = TODO XXX;
	  //htrY = TODO XXX;
	}
      }
      break;
    // paint reticles
    // numlock + g
    case 1048679:
      {
	double ddrX = (drX)/rmDelta;
	double ddrY = (drY)/rmDelta;
	double ttrX = (trX-rmcX)/rmDelta;
	double ttrY = (trY-rmcY)/rmDelta;
	if ((fabs(ddrX) <= rmHalfWidth) && (fabs(ddrY) <= rmHalfWidth)) {
	  int iiX = (int)round(ddrX + rmHalfWidth);
	  int iiY = (int)round(ddrY + rmHalfWidth);

	  double intensity = 128;
	  cv::Scalar backColor(ceil(intensity),0,0);
	  cv::Point outTop = cv::Point((iiY+rmWidth)*rmiCellWidth,iiX*rmiCellWidth);
	  cv::Point outBot = cv::Point(((iiY+rmWidth)+1)*rmiCellWidth,(iiX+1)*rmiCellWidth);
	  Mat vCrop = rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
	  vCrop += backColor;
	}
	if ((fabs(ttrX) <= rmHalfWidth) && (fabs(ttrY) <= rmHalfWidth)) {
	  int iiX = (int)round(ttrX + rmHalfWidth);
	  int iiY = (int)round(ttrY + rmHalfWidth);

	  double intensity = 128;
	  cv::Scalar backColor(0,ceil(intensity),0);
	  cv::Point outTop = cv::Point((iiY+rmWidth)*rmiCellWidth,iiX*rmiCellWidth);
	  cv::Point outBot = cv::Point(((iiY+rmWidth)+1)*rmiCellWidth,(iiX+1)*rmiCellWidth);
	  Mat vCrop = rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
	  vCrop += backColor;

	  cv::Point text_anchor = cv::Point(outTop.x+4, outBot.y-4);
	  char buff[256];
	  sprintf(buff, "%d", maxGG+1);
	  string reticleLabel(buff);
	  putText(rangemapImage, reticleLabel, text_anchor, MY_FONT, 0.5, Scalar(192,192,192), 1.0);
	}
	for (int gg = 0; gg < totalGraspGears; gg++){
	  double gggX = (ggX[gg])/rmDelta;
	  double gggY = (ggY[gg])/rmDelta;
	  if ((fabs(gggX) <= rmHalfWidth) && (fabs(gggY) <= rmHalfWidth)) {
	    int iiX = (int)round(gggX + rmHalfWidth);
	    int iiY = (int)round(gggY + rmHalfWidth);

	    cv::Point outTop = cv::Point((iiY+rmWidth)*rmiCellWidth,iiX*rmiCellWidth);
	    cv::Point outBot = cv::Point(((iiY+rmWidth)+1)*rmiCellWidth-1,(iiX+1)*rmiCellWidth-1);
	    cv::Point inTop = cv::Point(outTop.x+1, outTop.y+1);
	    cv::Point inBot = cv::Point(outBot.x-1, outBot.y-1);
	    rectangle(rangemapImage, outTop, outBot, cv::Scalar(192,0,0)); 
	    rectangle(rangemapImage, inTop, inBot, cv::Scalar(64,0,0)); 

	    cv::Point text_anchor = cv::Point(outTop.x+4, outBot.y-4);
	    char buff[256];
	    sprintf(buff, "%d", gg+1);
	    string reticleLabel(buff);
	    putText(rangemapImage, reticleLabel, text_anchor, MY_FONT, 0.5, Scalar(192,192,192), 1.0);
	  }
	}
//circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
//line(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
	double httrX = (trX-rmcX)/hrmDelta;
	double httrY = (trY-rmcY)/hrmDelta;
	int hiCellWidth = 5;
	if ((fabs(httrX) <= hrmHalfWidth-hiCellWidth) && (fabs(httrY) <= hrmHalfWidth-hiCellWidth)) {
	  int hiiX = (int)round(httrX + hrmHalfWidth);
	  int hiiY = (int)round(httrY + hrmHalfWidth);

	  double intensity = 128;
	  cv::Scalar backColor(0,ceil(intensity),0);
	  cv::Point l1p1 = cv::Point(hiiY+hrmWidth-hiCellWidth,hiiX);
	  cv::Point l1p2 = cv::Point((hiiY+hrmWidth)+hiCellWidth,hiiX);
	  cv::Point l2p1 = cv::Point(hiiY+hrmWidth,hiiX-hiCellWidth);
	  cv::Point l2p2 = cv::Point((hiiY+hrmWidth),hiiX+hiCellWidth);
	  line(hiRangemapImage, l1p1, l1p2, backColor);
	  line(hiRangemapImage, l2p1, l2p2, backColor);
	}
	double cttrX = curseReticleX - hrmHalfWidth;
	double cttrY = curseReticleY - hrmHalfWidth;
	if ((fabs(cttrX) <= hrmHalfWidth-hiCellWidth) && (fabs(cttrY) <= hrmHalfWidth-hiCellWidth)) {
	  int ciiX = (int)round(cttrX + hrmHalfWidth);
	  int ciiY = (int)round(cttrY + hrmHalfWidth);

	  double intensity = 128;
	  cv::Scalar backColor(0,ceil(intensity),ceil(intensity));
	  cv::Point l1p1 = cv::Point(ciiY+hrmWidth-hiCellWidth,ciiX);
	  cv::Point l1p2 = cv::Point((ciiY+hrmWidth)+hiCellWidth,ciiX);
	  cv::Point l2p1 = cv::Point(ciiY+hrmWidth,ciiX-hiCellWidth);
	  cv::Point l2p2 = cv::Point((ciiY+hrmWidth),ciiX+hiCellWidth);
	  line(hiRangemapImage, l1p1, l1p2, backColor);
	  line(hiRangemapImage, l2p1, l2p2, backColor);
	  cout << "printing curseReticle xy globalz: " << curseReticleX << " " << curseReticleY << " " << hiRangeMap[ciiX + ciiY*hrmWidth] << endl;
	}
	{
	  double intensity = 128;
	  cv::Scalar backColor(0,ceil(intensity),0);
	  circle(hiRangemapImage, cv::Point(hrmHalfWidth+hrmWidth, hrmHalfWidth), hiCellWidth, backColor);
	}
      }
      break;
    // move to target x,y 
    // numlock + h
    case 1048680:
      {
	// XXX
	//double targetX = trX - (drX);
	//double targetY = trY - (drY);
	double targetX = trX;
	double targetY = trY;

	double deltaX = targetX - currentEEPose.px;
	double deltaY = targetY - currentEEPose.py;
	double xTimes = fabs(floor(deltaX / bDelta)); 
	double yTimes = fabs(floor(deltaY / bDelta)); 

	int numNoOps = 6*(yTimes + xTimes);
	for (int cc = 0; cc < numNoOps; cc++) {
	  pilot_call_stack.push_back('C');
	}

	if (deltaX > 0)
	  for (int xc = 0; xc < xTimes; xc++)
	    pilot_call_stack.push_back('e');
	if (deltaX < 0)
	  for (int xc = 0; xc < xTimes; xc++)
	    pilot_call_stack.push_back('q');
	if (deltaY > 0)
	  for (int yc = 0; yc < yTimes; yc++)
	    pilot_call_stack.push_back('d');
	if (deltaY < 0)
	  for (int yc = 0; yc < yTimes; yc++)
	    pilot_call_stack.push_back('a');
      
	cout << "Move to target x,y. deltaX: " << deltaX << " xTimes: " << xTimes << endl;
      }
      break;
    // move to target z and grasp
    // numlock + j
    case 1048682:
      {
	pilot_call_stack.push_back('j');

	//double deltaZ = -maxD - graspDepth;
	double deltaZ = (-maxD -graspDepth) - currentEEPose.pz;

	double zTimes = fabs(floor(deltaZ / bDelta)); 

	int numNoOps = 16;
	if (deltaZ > 0)
	  for (int zc = 0; zc < zTimes; zc++) {
	    for (int cc = 0; cc < numNoOps; cc++) {
	      pilot_call_stack.push_back('C');
	    }
	    pilot_call_stack.push_back('w');
	  }
	if (deltaZ < 0)
	  for (int zc = 0; zc < zTimes; zc++) {
	    for (int cc = 0; cc < numNoOps; cc++) {
	      pilot_call_stack.push_back('C');
	    }
	    pilot_call_stack.push_back('s');
	  }
	cout << "Move to target z and grasp. deltaZ: " << deltaZ << " zTimes: " << zTimes << endl;
      }
      break;
    // add switches disable recording
    // numlock + k
    case 1048683:
      {
	recordRangeMap = 1;
      }
      break;
    // numlock + l
    case 1048684:
      {
	recordRangeMap = 0;
      }
      break;
    // set gg reticles
    // numlock + shift + 1
    case 1114145:
      {
	ggX[0] = rmDelta*(maxX-rmHalfWidth);
	ggY[0] = rmDelta*(maxY-rmHalfWidth);
	cout << "ggX[0]: " << ggX[0] << " ggY[0]: " << ggY[0] << endl;
      }
      break;
    // numlock + shift + 2
    case 1114176:
      {
	ggX[1] = rmDelta*(maxX-rmHalfWidth);
	ggY[1] = rmDelta*(maxY-rmHalfWidth);
	cout << "ggX[1]: " << ggX[1] << " ggY[1]: " << ggY[1] << endl;
      }
      break;
    // numlock + shift + 3
    case 1114147:
      {
	ggX[2] = rmDelta*(maxX-rmHalfWidth);
	ggY[2] = rmDelta*(maxY-rmHalfWidth);
	cout << "ggX[2]: " << ggX[2] << " ggY[2]: " << ggY[2] << endl;
      }
      break;
    // numlock + shift + 4
    case 1114148:
      {
	ggX[3] = rmDelta*(maxX-rmHalfWidth);
	ggY[3] = rmDelta*(maxY-rmHalfWidth);
	cout << "ggX[3]: " << ggX[3] << " ggY[3]: " << ggY[3] << endl;
      }
      break;
    // shift into grasp gear
    // numlock + 1
    case 1048625:
      {
	int thisGraspGear = 0;

	//   set drX
	drX = ggX[thisGraspGear];
	drY = ggY[thisGraspGear];

	//   rotate
	setGGRotation(thisGraspGear);

	//   set currentGraspGear;
	currentGraspGear = thisGraspGear;
      }
      break;
    // numlock + 2
    case 1048626:
      {
	int thisGraspGear = 1;

	//   set drX
	drX = ggX[thisGraspGear];
	drY = ggY[thisGraspGear];

	//   rotate
	setGGRotation(thisGraspGear);

	//   set currentGraspGear;
	currentGraspGear = thisGraspGear;
      }
      break;
    // numlock + 3
    case 1048627:
      {
	int thisGraspGear = 2;

	//   set drX
	drX = ggX[thisGraspGear];
	drY = ggY[thisGraspGear];

	//   rotate
	setGGRotation(thisGraspGear);

	//   set currentGraspGear;
	currentGraspGear = thisGraspGear;
      }
      break;
    // numlock + 4
    case 1048628:
      {
	int thisGraspGear = 3;

	//   set drX
	drX = ggX[thisGraspGear];
	drY = ggY[thisGraspGear];

	//   rotate
	setGGRotation(thisGraspGear);

	//   set currentGraspGear;
	currentGraspGear = thisGraspGear;
      }
      break;
    // reset drX and drY to 0 and invalidate graspGear
    // numlock + 5
    case 1048629:
      {
	//   set drX
	drX = 0.0;
	drY = 0.0;
	currentGraspGear = -1;
      }
      break;
    // select best available grasp
    // numlock + 6
    case 1048630:
      {
	cout << "Selecting best of 4 grasps..." << endl;
	// select max target cumulative
	pilot_call_stack.push_back(1114195);
	// apply grasp filter for 4
	pilot_call_stack.push_back(1048673);
	pilot_call_stack.push_back(1048692);
	pilot_call_stack.push_back(1048688);
	// blur
	pilot_call_stack.push_back(1048673);
	pilot_call_stack.push_back(1048692);
	pilot_call_stack.push_back(1048697);
	// load reg1
	pilot_call_stack.push_back(1048690);
	// change gear to 4
	pilot_call_stack.push_back(1048628);

	// select max target cumulative
	pilot_call_stack.push_back(1114195	);
	// apply grasp filter for 3
	pilot_call_stack.push_back(1048673);
	pilot_call_stack.push_back(1048692);
	pilot_call_stack.push_back(1048693);
	// blur
	pilot_call_stack.push_back(1048673);
	pilot_call_stack.push_back(1048692);
	pilot_call_stack.push_back(1048697);
	// load reg1
	pilot_call_stack.push_back(1048690);
	// change gear to 3
	pilot_call_stack.push_back(1048627);

	// select max target cumulative
	pilot_call_stack.push_back(1114195);
	// apply grasp filter for 2
	pilot_call_stack.push_back(1048673);
	pilot_call_stack.push_back(1048692);
	pilot_call_stack.push_back(1048687);
	// blur
	pilot_call_stack.push_back(1048673);
	pilot_call_stack.push_back(1048692);
	pilot_call_stack.push_back(1048697);
	// load reg1
	pilot_call_stack.push_back(1048690);
	// change gear to 2
	pilot_call_stack.push_back(1048626);

	// select max target NOT cumulative
	pilot_call_stack.push_back(1048691);
	// apply grasp filter for 1
	pilot_call_stack.push_back(1048673);
	pilot_call_stack.push_back(1048692);
	pilot_call_stack.push_back(1048681);
	// blur
	pilot_call_stack.push_back(1048673);
	pilot_call_stack.push_back(1048692);
	pilot_call_stack.push_back(1048697);
	// load reg1
	pilot_call_stack.push_back(1048690);
	// change gear to 1
	pilot_call_stack.push_back(1048625);

	// turn off recording
	pilot_call_stack.push_back(1048684);
      }
      break;
    // assume winning gg
    // numlock + 7
    case 1048631:
      {
	cout << "Assuming maxGG: " << maxGG << endl;
	if (maxGG == 0)
	  pilot_call_stack.push_back(1048625);
	if (maxGG == 1)
	  pilot_call_stack.push_back(1048626);
	if (maxGG == 2)
	  pilot_call_stack.push_back(1048627);
	if (maxGG == 3)
	  pilot_call_stack.push_back(1048628);
      }
      break;
    // change diagonal filter balance
    // numlock + c
    case 1048675:
      {
	gibbsIota += deltaGibbsIota;
	cout << "gibbsIota: " << gibbsIota << " deltaGibbsIota: " << deltaGibbsIota << endl;
      }
      break;
    // numlock + v
    case 1048694:
      {
	gibbsIota -= deltaGibbsIota;
	cout << "gibbsIota: " << gibbsIota << " deltaGibbsIota: " << deltaGibbsIota << endl;
      }
      break;
    // XXX
    // try damping the diagonal by the value it's supposed to have
    //  according to gaussian
    // scan, grab, put in the box
    // numlock + &
    case 1114150:
      {
	currentEEPose.px = rmcX + drX;
	currentEEPose.py = rmcY + drY;
      }
      break;
    // numlock + *
    case 1114154:
      {
	double lineSpeed = MOVE_MEDIUM;
	double betweenSpeed = MOVE_MEDIUM;
	pilot_call_stack.push_back('2'); // assume pose at register 2
	pushNoOps(200);
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('k'); // open gripper
	pushNoOps(200);
	pilot_call_stack.push_back('3'); // assume pose at register 3
	pushNoOps(200);
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back(1048682); // grasp at z inferred from target
	pushNoOps(200);

	pilot_call_stack.push_back(1048680); // assume x,y of target 
	pilot_call_stack.push_back(1114183); // full render
	pilot_call_stack.push_back(1048679); // render reticle
	pilot_call_stack.push_back(1048691); // find max on register 1
	pilot_call_stack.push_back(1048673); // render register 1
	pilot_call_stack.push_back(1048690); // load map to register 1
	pilot_call_stack.push_back(1048631); // assume best gear
	pilot_call_stack.push_back(1048678); // target best grasp
	pilot_call_stack.push_back(1048630); // find best grasp
	scanYdirection(lineSpeed, betweenSpeed); // load scan program
	pilot_call_stack.push_back(1114150); // prepare for search
	//pilot_call_stack.push_back(1048621); // change offset of position based on current gear 

	pilot_call_stack.push_back(1048683); // turn on scanning
	pilot_call_stack.push_back(1114183); // full render
	pilot_call_stack.push_back(1048679); // render reticle
	pilot_call_stack.push_back(1114155); // rotate gear
	pilot_call_stack.push_back(1048627); // change gear to 3
	pilot_call_stack.push_back(1048673); // render register 1
	pilot_call_stack.push_back(1048690); // load map to register 1
	pilot_call_stack.push_back(1048678); // target best grasp
	pilot_call_stack.push_back(1048630); // find best grasp
	scanYdirection(lineSpeed, betweenSpeed); // load scan program
	pilot_call_stack.push_back(1114150); // prepare for search

	pilot_call_stack.push_back(1048683); // turn on scanning
	pilot_call_stack.push_back(1114183); // full render
	pilot_call_stack.push_back(1048679); // render reticle
	pilot_call_stack.push_back(1048627); // change gear to 3
	pilot_call_stack.push_back(1048673); // render register 1
	pilot_call_stack.push_back(1048690); // load map to register 1
	pilot_call_stack.push_back(1048678); // target best grasp
	pilot_call_stack.push_back(1048630); // find best grasp
	scanXdirection(lineSpeed, betweenSpeed); // load scan program
	pilot_call_stack.push_back(1114150); // prepare for search
	//pilot_call_stack.push_back(1048621); // change offset of position based on current gear 

	pilot_call_stack.push_back(1048683); // turn on scanning
	pilot_call_stack.push_back(1114183); // full render
	pilot_call_stack.push_back(1048679); // render reticle
	pilot_call_stack.push_back(1114155); // rotate gear
	pilot_call_stack.push_back(1048625); // change to first gear
	pilot_call_stack.push_back(1048673); // render register 1
	pilot_call_stack.push_back(1048690); // load map to register 1
	pilot_call_stack.push_back(1048678); // target best grasp
	pilot_call_stack.push_back(1048630); // find best grasp
	scanXdirection(lineSpeed, betweenSpeed); // load scan program
	pilot_call_stack.push_back(1114150); // prepare for search

	pilot_call_stack.push_back(1048695); // clear scan history
	pilot_call_stack.push_back(1048683); // turn on scanning
	pilot_call_stack.push_back(1048625); // change to first gear
	pushNoOps(200);
	pilot_call_stack.push_back('x'); // retract
	pilot_call_stack.push_back(1048677); // wait
	pilot_call_stack.push_back('v'); // advance until closed
	pilot_call_stack.push_back('2'); // assume pose at register 2
	pilot_call_stack.push_back('k'); // open gripper
	pilot_call_stack.push_back('i'); // initialize gripper
      }
      break;
    // numlock + 8
    case 1048632:
      {
	double lineSpeed = MOVE_FAST;
	double betweenSpeed = MOVE_FAST;
	//pilot_call_stack.push_back(1048632); // push this program
	pilot_call_stack.push_back('2'); // assume pose at register 2
	pushNoOps(200);
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('d'); // move away
	pilot_call_stack.push_back('k'); // open gripper
	pushNoOps(200);
	pilot_call_stack.push_back('3'); // assume pose at register 3
	pushNoOps(200);
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back('w'); // raise arm
	pilot_call_stack.push_back(1048682); // grasp at z inferred from target
	pushNoOps(200);
	pilot_call_stack.push_back(1048680); // assume x,y of target 
	pilot_call_stack.push_back(1048679); // render reticle
	pilot_call_stack.push_back(1048691); // find max on register 1
	pilot_call_stack.push_back(1048673); // render register 1
	pilot_call_stack.push_back(1048690); // load map to register 1
	pilot_call_stack.push_back(1048631); // assume best gear
	pilot_call_stack.push_back(1048678); // target best grasp
	pilot_call_stack.push_back(1048630); // find best grasp

	//pilot_call_stack.push_back(1048689); // load scan program
	//scanXdirectionMedium(MOVE_FAST, MOVE_VERY_SLOW); // load scan program
	//scanXdirection(MOVE_SLOW, MOVE_SLOW); // load scan program
	//scanXdirection(MOVE_MEDIUM, MOVE_MEDIUM); // load scan program
	//scanXdirection(MOVE_FAST, MOVE_FAST); // load scan program
	scanXdirection(lineSpeed, betweenSpeed); // load scan program
	//pilot_call_stack.push_back(1048621); // change offset of position based on current gear 
	pilot_call_stack.push_back(1114150); // prepare for search

	pilot_call_stack.push_back(1048683); // turn on scanning
	pilot_call_stack.push_back(1114155); // rotate gear
	pilot_call_stack.push_back(1114183); // full render
	pilot_call_stack.push_back(1048679); // render reticle
	pilot_call_stack.push_back(1048625); // change to first gear
	pilot_call_stack.push_back(1048673); // render register 1
	pilot_call_stack.push_back(1048690); // load map to register 1
	pilot_call_stack.push_back(1048678); // target best grasp
	pilot_call_stack.push_back(1048630); // find best grasp

	//pilot_call_stack.push_back(1048689); // load scan program
	//scanXdirectionMedium(MOVE_FAST, MOVE_VERY_SLOW); // load scan program
	//scanXdirection(MOVE_SLOW, MOVE_SLOW); // load scan program
	//scanXdirection(MOVE_MEDIUM, MOVE_MEDIUM); // load scan program
	//scanXdirection(MOVE_FAST, MOVE_FAST); // load scan program
	scanXdirection(lineSpeed, betweenSpeed); // load scan program
	pilot_call_stack.push_back(1114150); // prepare for search

	pilot_call_stack.push_back(1048695); // clear scan history
	pilot_call_stack.push_back(1048683); // turn on scanning
	pilot_call_stack.push_back(1048625); // change to first gear
	pushNoOps(200);
	pilot_call_stack.push_back('x'); // retract
	pilot_call_stack.push_back(1048677); // wait
	pilot_call_stack.push_back('v'); // advance until closed
	pilot_call_stack.push_back('2'); // assume pose at register 2
	pilot_call_stack.push_back('k'); // open gripper
	pilot_call_stack.push_back('i'); // initialize gripper
      }
      break;
    // for current gear, find best grasp and move to x,y 
    // numlock + 9
    case 1048633:
      {
	pilot_call_stack.push_back(1048679); // render reticle
	pilot_call_stack.push_back(1048678); // target best grasp
	pilot_call_stack.push_back(1048691); // find max on register 1
	pilot_call_stack.push_back(1048673); // render register 1

	// apply grasp filter 
	pilot_call_stack.push_back(1048673);
	pilot_call_stack.push_back(1048692);

	if (currentGraspGear == 1)
	  pilot_call_stack.push_back(1048681);
	if (currentGraspGear == 2)
	  pilot_call_stack.push_back(1048687);
	if (currentGraspGear == 3)
	  pilot_call_stack.push_back(1048693);
	if (currentGraspGear == 4)
	  pilot_call_stack.push_back(1048688);

	// blur
	pilot_call_stack.push_back(1048673);
	pilot_call_stack.push_back(1048692);
	pilot_call_stack.push_back(1048697);
	// load reg1
	pilot_call_stack.push_back(1048690);
      }
      break;
    // prepare for and execute a grab at the current location and target
    // numlock + 0
    case 1048624:
      {
	pilot_call_stack.push_back(1048682); // grasp at z inferred from target
	pushNoOps(200);
	pilot_call_stack.push_back(1048680); // assume x,y of target 
	pilot_call_stack.push_back(1048679); // render reticle
	pilot_call_stack.push_back(1048691); // find max on register 1
	pilot_call_stack.push_back(1048673); // render register 1
	pilot_call_stack.push_back(1048690); // load map to register 1
      }
      break;
    // set movement speed
    // numlock + b
    case 1048674:
      {
	bDelta = MOVE_FAST;
      }
      break;
    // numlock + n
    case 1048686:
      {
	bDelta = MOVE_MEDIUM;
      }
      break;
    // numlock + N
    case 1114190:
      {
	bDelta = MOVE_SLOW;
      }
      break;
    // numlock + B
    case 1114178:
      {
	bDelta = MOVE_VERY_SLOW;
      }
      break;
    // enable / disable rendering
    // numlock + m
    case 1048685:
      {
	shouldIRender = 1;
      }
      break;
    // numlock + M
    case 1114189:
      {
	shouldIRender = 0;
      }
      break;
    // shift up / down
    // numlock + +
    case 1114155:
      {
	pushNoOps(480);
	int thisGraspGear = (currentGraspGear+4) % totalGraspGears;

	//   set drX
	drX = ggX[thisGraspGear];
	drY = ggY[thisGraspGear];

	//   rotate
	setGGRotation(thisGraspGear);

	//   set currentGraspGear;
	currentGraspGear = thisGraspGear;
      }
      break;
    // set offset based on selected gear
    // numlock + -
    case 1048621:
      {
	if (currentGraspGear >= 4)
	  currentEEPose.py += 0.0025;
      }
      break;
    // numlock + uparrow
    case 1113938:
      {
	curseReticleY = max(curseReticleY - 1, 0);
	cout << "curseReticle xy: " << curseReticleX << " " << curseReticleY << endl;
      }
      break;
    // numlock + leftarrow 
    case 1113937:
      {
	curseReticleX = max(curseReticleX - 1, 0);
	cout << "curseReticle xy: " << curseReticleX << " " << curseReticleY << endl;
      }
      break;
    // numlock + downarrow 
    case 1113940:
      {
	curseReticleY = min(curseReticleY + 1, hrmWidth-1);
	cout << "curseReticle xy: " << curseReticleX << " " << curseReticleY << endl;
      }
      break;
    // numlock + rightarrow
    case 1113939:
      {
	curseReticleX = min(curseReticleX + 1, hrmWidth-1);
	cout << "curseReticle xy: " << curseReticleX << " " << curseReticleY << endl;
      }
      break;
    // numlock + pageup
    case 1113941:
      {
	//rapidJointGlobalOmega[5] += 3.14159/8.0;
	//cout << "rapidJointGlobalOmega 5: " << rapidJointGlobalOmega << endl;
	//rapidJointGlobalOmega[testJoint] += .1;
	//cout << "rapidJointGlobalOmega testJoint: " << rapidJointGlobalOmega[testJoint] << endl;
	for (int j = 0; j < numJoints; j++) {
	  rapidJointGlobalOmega[j] += .01*rapidJointMask[j];
	  cout << "rapidJointGlobalOmega " << j << ": " << rapidJointGlobalOmega[j] << endl;
	}
      }
      break;
    // numlock + pagedown
    case 1113942:
      {
	//rapidJointGlobalOmega[5] -= 3.14159/8.0;
	//cout << "rapidJointGlobalOmega 5: " << rapidJointGlobalOmega << endl;
	//rapidJointGlobalOmega[testJoint] -= .1;
	//cout << "rapidJointGlobalOmega testJoint: " << rapidJointGlobalOmega[testJoint] << endl;
	for (int j = 0; j < numJoints; j++) {
	  rapidJointGlobalOmega[j] -= .01*rapidJointMask[j];
	  cout << "rapidJointGlobalOmega " << j << ": " << rapidJointGlobalOmega[j] << endl;
	}
      }
      break;
    // numlock + home
    case 1113936:
      {
	driveVelocities = 1;
	oscilStart = ros::Time::now();
	cout << "driveVelocities: " << driveVelocities << endl;
      }
      break;
    // numlock + end 
    case 1113943:
      {
	driveVelocities = 0;
	cout << "driveVelocities: " << driveVelocities << endl;
      }
      break;
    // numlock + insert
    case 1113955:
      {
	//rapidJointGlobalOmega[testJoint] *= -1;
	//cout << "rapidJointGlobalOmega testJoint: " << rapidJointGlobalOmega[testJoint] << endl;
	for (int j = 0; j < numJoints; j++) {
	  rapidJointGlobalOmega[j] *= -1*rapidJointMask[j];
	  cout << "rapidJointGlobalOmega " << j << ": " << rapidJointGlobalOmega[j] << endl;
	}
      }
      break;
    // numlock + shift + pageup 
    case 1179477:
      {
	rapidAmp1 += rapidAmp1Delta;
	cout << "rapidAmp1: " << rapidAmp1 << endl;
      }
      break;
    // numlock + shift + pagedown 
    case 1179478:
      {
	rapidAmp1 -= rapidAmp1Delta;
	cout << "rapidAmp1 " << rapidAmp1 << endl;
      }
      break;
    // numlock + ctrl + shift + pageup 
    case 1441621:
      {
	rapidAmp2 += rapidAmp2Delta;
	cout << "rapidAmp2: " << rapidAmp2 << endl;
      }
      break;
    // numlock + ctrl + shift + pagedown 
    case 1441622:
      {
	rapidAmp2 -= rapidAmp2Delta;
	cout << "rapidAmp2 " << rapidAmp2 << endl;
      }
      break;
    // numlock + shift + home 
    case 1179472:
      {
	spiralEta += .01;
	cout << "spiralEta: " << spiralEta << endl;
      }
      break;
    // numlock + shift + end 
    case 1179479:
      {
	spiralEta -= .01;
	cout << "spiralEta " << spiralEta << endl;
      }
      break;
    // numlock + alt + pageup
    case 1638229:
      {
	rapidJointScales[3] += .01;
	cout << "rapidJointScales[3]: " << rapidJointScales[3] << endl;
      }
      break;
    // numlock + alt + pagedown 
    case 1638230:
      {
	rapidJointScales[3] -= .01;
	cout << "rapidJointScales[3] " << rapidJointScales[3] << endl;
      }
      break;
    // numlock + ctrl+ pageup 
    case 1376085:
      {
	rapidJointLocalOmega[3] += .01;
	cout << "rapidJointLocalOmega[3]: " << rapidJointLocalOmega[3] << endl;
      }
      break;
    // numlock + ctrl + pagedown 
    case 1376086:
      {
	rapidJointLocalOmega[3] -= .01;
	cout << "rapidJointLocalOmega[3] " << rapidJointLocalOmega[3] << endl;
      }
      break;
    // numlock + alt + home 
    case 1638224:
      {
	rapidJointLocalBias[3] += .01;
	cout << "rapidJointLocalBias[3]: " << rapidJointLocalBias[3] << endl;
      }
      break;
    // numlock + alt + end
    case 1638231:
      {
	rapidJointLocalBias[3] -= .01;
	cout << "rapidJointLocalBias[3] " << rapidJointLocalBias[3] << endl;
      }
      break;
    // numlock + (
    case 1114152:
      {
	int numToGoOut = 70;
	int noOpScale = 10;

	pilot_call_stack.push_back(1113943); // turn off velocity control

	// and the red matter brings it back
	for (int ra = 0; ra < numToGoOut; ra++) {
	  pilot_call_stack.push_back(1179478); // lower rapidAmp1
	  pushNoOps(int(double(noOpScale) * (1.0 + double(ra)/double(numToGoOut))));
	}

	// cast it out
	for (int ra = 0; ra < numToGoOut; ra++) {
	  pilot_call_stack.push_back(1179477); // raise rapidAmp1
	  pushNoOps(int(double(noOpScale) * (1.0 + 2.0*double(ra)/double(numToGoOut))));
	}

	// zero it
	int numToZero = rapidAmp1 / rapidAmp1Delta;
	for (int ra = 0; ra < numToZero; ra++)
	  pilot_call_stack.push_back(1179478); // lower rapidAmp1
	
	pilot_call_stack.push_back(1113936); // turn on velocity control
      }
      break;
    // numlock + )
    case 1114153:
      {
	int numToGoOut1 = 20;
	int numToGoOut2 = 10;
	int noOpScale = 5;

	pilot_call_stack.push_back(1113943); // turn off velocity control

	// and the red matter brings it back
	for (int ra2 = 0; ra2 < numToGoOut2; ra2++) {
	  pilot_call_stack.push_back(1441622); // lower rapidAmp2
	  pilot_call_stack.push_back(1441622); // lower rapidAmp2
	  pilot_call_stack.push_back(1441622); // lower rapidAmp2
	  pilot_call_stack.push_back(1441622); // lower rapidAmp2
	  pilot_call_stack.push_back(1441622); // lower rapidAmp2
	  for (int ra1 = 0; ra1 < numToGoOut1; ra1++) {
	    pilot_call_stack.push_back(1179478); // lower rapidAmp1
	    pushNoOps(noOpScale);
	  }
	  for (int ra1 = 0; ra1 < numToGoOut1; ra1++) {
	    pilot_call_stack.push_back(1179477); // raise rapidAmp1
	    pushNoOps(noOpScale);
	  }
	}

	// cast it out
	for (int ra2 = 0; ra2 < numToGoOut2; ra2++) {
	  pilot_call_stack.push_back(1441621); // raise rapidAmp2
	  pilot_call_stack.push_back(1441621); // raise rapidAmp2
	  pilot_call_stack.push_back(1441621); // raise rapidAmp2
	  pilot_call_stack.push_back(1441621); // raise rapidAmp2
	  pilot_call_stack.push_back(1441621); // raise rapidAmp2
	  for (int ra1 = 0; ra1 < numToGoOut1; ra1++) {
	    pilot_call_stack.push_back(1179478); // lower rapidAmp1
	    pushNoOps(noOpScale);
	  }
	  for (int ra1 = 0; ra1 < numToGoOut1; ra1++) {
	    pilot_call_stack.push_back(1179477); // raise rapidAmp1
	    pushNoOps(noOpScale);
	  }
	}

	// zero it
	int numToZero1 = (rapidAmp1 - 0.1) / rapidAmp1Delta;
	for (int ra1 = 0; ra1 < numToZero1; ra1++)
	  pilot_call_stack.push_back(1179478); // lower rapidAmp1
	for (int ra1 = 0; ra1 < -numToZero1; ra1++)
	    pilot_call_stack.push_back(1179477); // raise rapidAmp1

	int numToZero2 = (rapidAmp2 - 0.1) / rapidAmp2Delta;
	for (int ra2 = 0; ra2 < numToZero2; ra2++)
	  pilot_call_stack.push_back(1441622); // lower rapidAmp2
	for (int ra2 = 0; ra2 < -numToZero2; ra2++)
	  pilot_call_stack.push_back(1441621); // raise rapidAmp2
	
	pilot_call_stack.push_back(1113936); // turn on velocity control
      }
      break;
    // planetary 4 joint
    // constant speed scan
    //////////
    case 1:
      {
      }
      break;
    /////
    // node keys
    /////
    // 
    // density 
    // capslock + 1
    case 131121:
      {
	cout << "Updating density estimate..." << endl;
	goCalculateDensity();
      }
      break;
    // blue boxes
    // capslock + 2
    case 131122:
      {
	cout << "Finding blue boxes..." << endl;
      }
      break;
    // classify
    // capslock + 3
    case 131123:
      {
	cout << "Classifying blue boxes..." << endl;
      }
      break;
    // capslock + q
    case 2:
      drawOrientor = !drawOrientor;
      break;
    // capslock + w
    case 3:
      drawLabels = !drawLabels;
      break;
    // capslock + e
    case 4:
      add_blinders = !add_blinders;
      break;
    // capslock + r
    case 5:
      mask_gripper = !mask_gripper;
      break;
    // capslock + a
    case 6:
      drawPurple= !drawPurple;
      break;
    // capslock + s
    case 7:
      //drawWhite = !drawWhite;
      break;
    // capslock + d
    case 8:
      drawGreen = !drawGreen;
      break;
    // capslock + f
    case 9:
      drawBlue = !drawBlue;
      break;
    // capslock + g
    case 10:
      drawRed = !drawRed;
      break;
    // capslock + h
    case 11:
      drawRB = !drawRB;
      break;
    // capslock + j
    case 12:
      drawGray = !drawGray;
      break;
    // capslock + k
    case 13:
      drawPink = !drawPink;
      break;
    // capslock + l
    case 14:
      drawBrown = !drawBrown;
      break;
    // capslock + z
    case 15:
      drawBlueKP = !drawBlueKP;
      break;
    // capslock + x
    case 16:
      drawRedKP = !drawRedKP;
      break;

    default:
      {
      }
      break;
  }

  #ifdef DEBUG
  cout << "debug 9" << endl;
  cout.flush();
  #endif

//cout << "block4" << endl;


  // stateful updates
  // this could be moved into the 'no-op' / 'continue' logic
  if (!zero_g_toggle) {

    Eigen::Quaternionf qin(0, 0, 0, 1);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);

    qout = eeqform * qin * eeqform.conjugate();

    eeForward.x() = qout.x();
    eeForward.y() = qout.y();
    eeForward.z() = qout.z();

    // autopilot update
    // XXX TODO this really needs to be in the coordinate frame of the
    //  end effector to be general enough to use. now we are assuming that
    //  we are aligned with the table.
    if (auto_pilot && (timerCounter < timerThresh)) {
      currentEEPose.ox = 0.0;
      currentEEPose.oy = 0.0;
      currentEEPose.oz = 0.0;

      double PxPre = reticle.px - pilotTarget.px; 
      double PyPre = reticle.py - pilotTarget.py;

      double Px = PxPre;
      double Py = PyPre;

      double thisDelta = bDelta;
      lock_status = 0;
      if ((fabs(Px) < aim_thresh) && (fabs(Py) < aim_thresh)) {
	thisDelta = bDelta * slow_aim_factor;
	lock_status = 1;
      } 
      if ((fabs(Px) < lock_thresh) && (fabs(Py) < lock_thresh)) {
	thisDelta = bDelta * slow_aim_factor;
	lock_status = 2;
      }
      // angular control
      //currentEEPose.oy -= thisDelta*Kp*Py;
      //currentEEPose.ox -= thisDelta*Kp*Px;
  
      // cartesian control
      // square is flatter near 0 and we need the si
      /*
      if (Px < -cCutoff)
	Px = -cCutoff;
      if (Px > cCutoff)
	Px = cCutoff;
      if (Py < -cCutoff)
	Py = -cCutoff;
      if (Py > cCutoff)
	Py = cCutoff;
      */

      double Dx = pilotTarget.px - prevPx;
      double Dy = pilotTarget.py - prevPy;

      // parallel update
      //double pTermX = thisDelta*Kp*Px*fabs(Px);
      //double pTermY = thisDelta*Kp*Py*fabs(Py);
      double pTermX = thisDelta*Kp*Px;
      double pTermY = thisDelta*Kp*Py;
      double dTermX = -thisDelta*Kd*Dx;
      double dTermY = -thisDelta*Kd*Dy;
      //currentEEPose.px -= (thisDelta*Kp*Px*fabs(Px);
      //currentEEPose.py += (thisDelta*Kp*Py*fabs(Py);
      //currentEEPose.px -= pTermX;
      //currentEEPose.py += pTermY;
      currentEEPose.px -= pTermX + dTermX;
      currentEEPose.py += pTermY + dTermY;

      autoPilotFrameCounter++;


      cout << "ikShare: " << ikShare << " Px: " << Px << " Py: " << Py << "      Dx: " << Dx << " Dy: " << Dy << "      lock_status: " << lock_status << " slf: " << successive_lock_frames << " slf_t: " << slf_thresh << endl;
    }

    // holding pattern update
    switch (holding_pattern) {
      case -1:
	{
	  if (eeRange < a_thresh_far) {
	    currentEEPose.px -= eeForward.x()*approachStep;
	    currentEEPose.py -= eeForward.y()*approachStep;
	    currentEEPose.pz -= eeForward.z()*approachStep;
	  } else {
	    holding_pattern = 0;
	  }
	  baxter_core_msgs::EndEffectorCommand command;
	  command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
	  command.args = "{\"position\": 100.0}";
	  command.id = 65538;
	  gripperPub.publish(command);
	}
	break;
      case 0:
	break;
      case 1:
	if (eeRange > a_thresh_close) {
	  baxter_core_msgs::EndEffectorCommand command;
	  command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
	  command.args = "{\"position\": 100.0}";
	  command.id = 65538;
	  gripperPub.publish(command);
	  currentEEPose.px += eeForward.x()*approachStep;
	  currentEEPose.py += eeForward.y()*approachStep;
	  currentEEPose.pz += eeForward.z()*approachStep;
	} else {
	  baxter_core_msgs::EndEffectorCommand command;
	  command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
	  command.args = "{\"position\": 0.0}";
	  command.id = 65538;
	  gripperPub.publish(command);
	  holding_pattern = 0;
	}
	break;
      case 2:
	{
	  baxter_core_msgs::EndEffectorCommand command;
	  command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
	  command.args = "{\"position\": 100.0}";
	  command.id = 65538;
	  gripperPub.publish(command);
	  currentEEPose.qx = -0.283134;
	  currentEEPose.qy = 0.958744;
	  currentEEPose.qz = -0.00634737;
	  currentEEPose.qw = 0.0245754;

	  if (eeRange > a_thresh_far) {
	    currentEEPose.px += eeForward.x()*approachStep*hoverMultiplier;
	    currentEEPose.py += eeForward.y()*approachStep*hoverMultiplier;
	    currentEEPose.pz += eeForward.z()*approachStep*hoverMultiplier;
	  } else {
	    currentEEPose.px -= eeForward.x()*approachStep*hoverMultiplier;
	    currentEEPose.py -= eeForward.y()*approachStep*hoverMultiplier;
	    currentEEPose.pz -= eeForward.z()*approachStep*hoverMultiplier;
	  }
	}
	break;
    }
  }

  if (!zero_g_toggle) {
    update_baxter(n);
  }
  else {
    currentEEPose.px = trueEEPose.position.x;
    currentEEPose.py = trueEEPose.position.y;
    currentEEPose.pz = trueEEPose.position.z;
    currentEEPose.qx = trueEEPose.orientation.x;
    currentEEPose.qy = trueEEPose.orientation.y;
    currentEEPose.qz = trueEEPose.orientation.z;
    currentEEPose.qw = trueEEPose.orientation.w;
    currentEEPose.ox = 0.0;
    currentEEPose.oy = 0.0;
    currentEEPose.oz = 0.0;
  }


  //cv::waitKey(1);
  timerCounter++;
  timesTimerCounted++;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg){

  if (!renderInit) {
    renderInit = 1;
    shouldIRender = shouldIRenderDefault;
  }

  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cam_img = cv_ptr->image.clone();
    //real_img = true;
  }catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


  wristCamImage = cv_ptr->image.clone();
  wristCamInit = 1;

  wristViewImage = cv_ptr->image.clone();

  setRingImageAtTime(msg->header.stamp, wristCamImage);
  Mat thisImage;
  int weHaveImData = getRingImageAtTime(msg->header.stamp, thisImage);

  if (recordRangeMap) 
    recordReadyRangeReadings();

  // publish volumetric representation to a marker array
  {
    int aI = 0;
    int vmSubsampleStride = 10;
    visualization_msgs::MarkerArray ma_to_send; 
    for (int pz = 0; pz < vmWidth; pz+=vmSubsampleStride) {
      for (int py = 0; py < vmWidth; py+=vmSubsampleStride) {
	for (int px = 0; px < vmWidth; px+=vmSubsampleStride) {
	  if (volumeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth] > 0) {
	    aI++;
	  }
	}
      }
    }
    int numCubesToShow = aI;

    /*
    ma_to_send.markers.resize(aI);
    aI = 0;
    for (int pz = 0; pz < vmWidth; pz+=vmSubsampleStride) {
      for (int py = 0; py < vmWidth; py+=vmSubsampleStride) {
	for (int px = 0; px < vmWidth; px+=vmSubsampleStride) {
	  if (volumeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth] > 0) {
	    double denomC = max(vmColorRangeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth], EPSILON);
	    int tRed = min(255, max(0,int(round(vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 2*vmWidth*vmWidth*vmWidth] / denomC))));
	    int tGreen = min(255, max(0,int(round(vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 1*vmWidth*vmWidth*vmWidth] / denomC))));
	    int tBlue = min(255, max(0,int(round(vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 0*vmWidth*vmWidth*vmWidth] / denomC))));

//cout << tBlue << " " << vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 0*vmWidth*vmWidth*vmWidth] << " " <<  denomC << endl;
	    ma_to_send.markers[aI].pose.position.x = rmcX + (px - vmHalfWidth)*vmDelta;
	    ma_to_send.markers[aI].pose.position.y = rmcY + (py - vmHalfWidth)*vmDelta;
	    ma_to_send.markers[aI].pose.position.z = rmcZ + (pz - vmHalfWidth)*vmDelta;
	    ma_to_send.markers[aI].pose.orientation.w = 1.0;
	    ma_to_send.markers[aI].pose.orientation.x = 0.0;
	    ma_to_send.markers[aI].pose.orientation.y = 0.0;
	    ma_to_send.markers[aI].pose.orientation.z = 0.0;
	    ma_to_send.markers[aI].type =  visualization_msgs::Marker::CUBE;
	    ma_to_send.markers[aI].scale.x = vmDelta*vmSubsampleStride;
	    ma_to_send.markers[aI].scale.y = vmDelta*vmSubsampleStride;
	    ma_to_send.markers[aI].scale.z = vmDelta*vmSubsampleStride;
	    ma_to_send.markers[aI].color.a = volumeMap[px + py*vmWidth + pz*vmWidth*vmWidth];
	    ma_to_send.markers[aI].color.r = double(tRed)/255.0;
	    ma_to_send.markers[aI].color.g = double(tGreen)/255.0;
	    ma_to_send.markers[aI].color.b = double(tBlue)/255.0;

	    ma_to_send.markers[aI].header.stamp = ros::Time::now();
	    ma_to_send.markers[aI].header.frame_id = "/base";
	    ma_to_send.markers[aI].action = visualization_msgs::Marker::ADD;
	    ma_to_send.markers[aI].id = aI;
	    ma_to_send.markers[aI].lifetime = ros::Duration(1.0);

	    aI++;
	  }
	}
      }
    }
    */

    ma_to_send.markers.resize(1);

    ma_to_send.markers[0].pose.orientation.w = 1.0;
    ma_to_send.markers[0].pose.orientation.x = 0.0;
    ma_to_send.markers[0].pose.orientation.y = 0.0;
    ma_to_send.markers[0].pose.orientation.z = 0.0;
    ma_to_send.markers[0].type =  visualization_msgs::Marker::CUBE_LIST;
    ma_to_send.markers[0].scale.x = vmDelta*vmSubsampleStride;
    ma_to_send.markers[0].scale.y = vmDelta*vmSubsampleStride;
    ma_to_send.markers[0].scale.z = vmDelta*vmSubsampleStride;

    ma_to_send.markers[0].header.stamp = ros::Time::now();
    ma_to_send.markers[0].header.frame_id = "/base";
    ma_to_send.markers[0].action = visualization_msgs::Marker::ADD;
    ma_to_send.markers[0].id = 0;
    ma_to_send.markers[0].lifetime = ros::Duration(1.0);

    for (int pz = 0; pz < vmWidth; pz+=vmSubsampleStride) {
      for (int py = 0; py < vmWidth; py+=vmSubsampleStride) {
	for (int px = 0; px < vmWidth; px+=vmSubsampleStride) {
	  if (volumeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth] > 0) {
	    double denomC = max(vmColorRangeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth], EPSILON);
	    int tRed = min(255, max(0,int(round(vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 2*vmWidth*vmWidth*vmWidth] / denomC))));
	    int tGreen = min(255, max(0,int(round(vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 1*vmWidth*vmWidth*vmWidth] / denomC))));
	    int tBlue = min(255, max(0,int(round(vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + 0*vmWidth*vmWidth*vmWidth] / denomC))));

	    std_msgs::ColorRGBA p;
	    //p.a = volumeMap[px + py*vmWidth + pz*vmWidth*vmWidth] > 0;
	    p.a = 4.0*volumeMap[px + py*vmWidth + pz*vmWidth*vmWidth];
	    p.r = double(tRed)/255.0;
	    p.g = double(tGreen)/255.0;
	    p.b = double(tBlue)/255.0;
	    ma_to_send.markers[0].colors.push_back(p);

	    geometry_msgs::Point temp;
	    temp.x = rmcX + (px - vmHalfWidth)*vmDelta;
	    temp.y = rmcY + (py - vmHalfWidth)*vmDelta;
	    temp.z = rmcZ + (pz - vmHalfWidth)*vmDelta;
	    ma_to_send.markers[0].points.push_back(temp);
	  }
	}
      }
    }
    vmMarkerPublisher.publish(ma_to_send);
  }

  // draw color reticle
  {
    for (int cr = 0; cr < numCReticleIndeces; cr++) {
      cv::Point outTop = cv::Point(xCR[cr]-3, yCR[cr]-3);
      cv::Point outBot = cv::Point(xCR[cr]+3, yCR[cr]+3);
      cv::Point inTop = cv::Point(outTop.x+1, outTop.y+1);
      cv::Point inBot = cv::Point(outBot.x-1, outBot.y-1);
      rectangle(wristViewImage, outTop, outBot, cv::Scalar(0,192,0)); 
      rectangle(wristViewImage, inTop, inBot, cv::Scalar(0,64,0)); 
    }
    {
      int tcrx = getColorReticleX();
      int tcry = getColorReticleY();
      cv::Point outTop = cv::Point(tcrx-5, tcry-5);
      cv::Point outBot = cv::Point(tcrx+5, tcry+5);
      cv::Point inTop = cv::Point(outTop.x+1, outTop.y+1);
      cv::Point inBot = cv::Point(outBot.x-1, outBot.y-1);
      rectangle(wristViewImage, outTop, outBot, cv::Scalar(227,104,193)); 
      rectangle(wristViewImage, inTop, inBot, cv::Scalar(133,104,109)); 
    }
  }

  {
    cv::Point outTop = cv::Point(reticle.px-reticleHalfWidth, reticle.py-reticleHalfWidth);
    cv::Point outBot = cv::Point(reticle.px+reticleHalfWidth, reticle.py+reticleHalfWidth);
    cv::Point inTop = cv::Point(reticle.px+1-reticleHalfWidth,reticle.py+1-reticleHalfWidth);
    cv::Point inBot = cv::Point(reticle.px-1+reticleHalfWidth,reticle.py-1+reticleHalfWidth);

    if (auto_pilot) {
      int boxBrightness = 128;
      if (go_on_lock)
	boxBrightness = 255;
      Mat vCrop = wristViewImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
      cv::Scalar fillColor(0,boxBrightness,0);
      switch (lock_status) {
	case 0:
	  break;
	case 1:
	  fillColor = cv::Scalar(0,boxBrightness,boxBrightness);
	  break;
	case 2:
	  fillColor = cv::Scalar(0,0,boxBrightness);
	  break;
      }
      vCrop = vCrop + fillColor;
    }

    rectangle(wristViewImage, outTop, outBot, cv::Scalar(82,70,22)); // RGB: 22 70 82
    rectangle(wristViewImage, inTop, inBot, cv::Scalar(239,205,68)); // RGB: 68 205 239
  }

  {
    cv::Point outTop = cv::Point(pilotTarget.px-pilotTargetHalfWidth, pilotTarget.py-pilotTargetHalfWidth);
    cv::Point outBot = cv::Point(pilotTarget.px+pilotTargetHalfWidth, pilotTarget.py+pilotTargetHalfWidth);
    cv::Point inTop = cv::Point(pilotTarget.px+1-pilotTargetHalfWidth,pilotTarget.py+1-pilotTargetHalfWidth);
    cv::Point inBot = cv::Point(pilotTarget.px-1+pilotTargetHalfWidth,pilotTarget.py-1+pilotTargetHalfWidth);
    rectangle(wristViewImage, outTop, outBot, cv::Scalar(53,10,97)); // RGB: 97 10 53
    rectangle(wristViewImage, inTop, inBot, cv::Scalar(142,31,255)); // RGB: 255 31 142
  }

  if (shouldIRender) {
    cv::imshow(wristViewName, wristViewImage);
  }

  //Mat coreImage = wristViewImage.clone();
  Mat coreImage(2*wristViewImage.rows, wristViewImage.cols, wristViewImage.type());
  coreImage = 0.0*coreImage;

  cv::Scalar dataColor(192,192,192);
  cv::Scalar labelColor(160,160,160);

  cv::Point ciAnchor(10,50);
  putText(coreImage, "Current Registers: ", ciAnchor, MY_FONT, 0.5, labelColor, 1.0);

  char buf[256];
  cv::Point lAnchor(170,50);
  string lText;
  lText += "CI: ";
  lText += current_instruction;
  lText += "  ZG: ";
  sprintf(buf, "%d", zero_g_toggle);
  lText += buf;
  lText += "  HP: ";
  sprintf(buf, "%d", holding_pattern);
  lText += buf;
  lText += "  AP: ";
  sprintf(buf, "%d", auto_pilot);
  lText += buf;
  putText(coreImage, lText, lAnchor, MY_FONT, 0.5, dataColor, 2.0);

  lAnchor.y += 20;
  lText = "";
  lText += "LS: ";
  sprintf(buf, "%d", lock_status);
  lText += buf;
  lText += "  GOL: ";
  sprintf(buf, "%d", go_on_lock);
  lText += buf;
  lText += "  SLF: ";
  sprintf(buf, "%d", successive_lock_frames);
  lText += buf;
  lText += "  O: ";
  sprintf(buf, "%d", oscillating);
  lText += buf;
  putText(coreImage, lText, lAnchor, MY_FONT, 0.5, dataColor, 2.0);

  lAnchor.y += 20;
  lText = "";
  lText += "TYT: ";
  sprintf(buf, "%d", take_yellow_thresh);
  lText += buf;
  lText += " APFC: ";
  sprintf(buf, "%d", autoPilotFrameCounter);
  lText += buf;
  lText += " GG: ";
  sprintf(buf, "%d", currentGraspGear);
  lText += buf;
  putText(coreImage, lText, lAnchor, MY_FONT, 0.5, dataColor, 2.0);

  lAnchor.y += 20;
  lText = "";
  lText += "rgRB: ";
  sprintf(buf, "%+.02d/%d", rgRingBufferEnd-rgRingBufferStart, rgRingBufferSize);
  lText += buf;
  lText += " epRB: ";
  sprintf(buf, "%+.02d/%d", epRingBufferEnd-epRingBufferStart, epRingBufferSize);
  lText += buf;
  lText += " imRB: ";
  sprintf(buf, "%+.02d/%d", imRingBufferEnd-imRingBufferStart, imRingBufferSize);
  lText += buf;
  putText(coreImage, lText, lAnchor, MY_FONT, 0.5, dataColor, 2.0);

  int stackRowY = 140; //remember to increment
  cv::Point csAnchor(10,stackRowY);
  putText(coreImage, "Call Stack: ", csAnchor, MY_FONT, 0.5, labelColor, 1.0);
  
  int instructionsPerRow = 25;
  int rowAnchorStep = 25;
  cv::Point rowAnchor(120,stackRowY);
  int insCount = 0; 

  int numCommandsToShow = 400;
  int lowerBound = max(int(pilot_call_stack.size() - numCommandsToShow), 0);
  insCount = lowerBound;

  while (insCount < pilot_call_stack.size()) {
    string outRowText;

    for (int rowCount = 0; (insCount < pilot_call_stack.size()) && (rowCount < instructionsPerRow); insCount++, rowCount++) {
      outRowText += pilot_call_stack[pilot_call_stack.size() - (insCount - lowerBound)];
      outRowText += " ";
    }

    putText(coreImage, outRowText, rowAnchor, MY_FONT, 0.5, dataColor, 2.0);
    rowAnchor.y += rowAnchorStep;
  }

  if (shouldIRender) {
    cv::imshow(coreViewName, coreImage);
  }
}

void targetCallback(const geometry_msgs::Point& point) {
  prevPx = pilotTarget.px;
  prevPy = pilotTarget.py;

  pilotTarget.px = point.x;
  pilotTarget.py = point.y;
  pilotTarget.pz = point.z;
      
  //cout << ">>received target<<" << endl;

  timerCounter = 0;

  if (lock_status == 2)
    successive_lock_frames++;
  else
    successive_lock_frames = 0;
}

void pilotCallbackFunc(int event, int x, int y, int flags, void* userdata) {
  if ( event == EVENT_LBUTTONDOWN ) {
    //cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    reticle.px = x;
    reticle.py = y;
    cout << "x: " << x << " y: " << y << " eeRange: " << eeRange << endl;
  } else if ( event == EVENT_RBUTTONDOWN ) {
    //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if  ( event == EVENT_MBUTTONDOWN ) {
    //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if ( event == EVENT_MOUSEMOVE ) {
    //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
  }
}

void pilotInit() {
  eeForward = Eigen::Vector3d(1,0,0);

  if (0 == left_or_right_arm.compare("left")) {
    cout << "Possessing left arm..." << endl;
    beeHome = crane1left;
    eepReg1 = crane1left;
    eepReg2 = crane2left;
    eepReg3 = crane3left;
    eepReg4 = beeLHome;
    oscillatingSign = 1;
    defaultReticle = defaultLeftReticle;
    reticle = defaultReticle;
  } else if (0 == left_or_right_arm.compare("right")) {
    cout << "Possessing right arm..." << endl;
    beeHome = crane1right;
    eepReg1 = crane1right;
    eepReg2 = crane2right;
    eepReg3 = crane3right;
    eepReg4 = beeRHome;
    oscillatingSign = -1;
    defaultReticle = defaultRightReticle;
    reticle = defaultReticle;
  } else {
    cout << "Invalid chirality. Exiting." << endl;
    exit(0);
  }
  pilotTarget = beeHome;
  lastGoodEEPose = beeHome;
  currentEEPose = beeHome;

  ik_reset_eePose = eepReg2;

  for (int r = 0; r < totalRangeHistoryLength; r++) {
    rangeHistory[r] = 0;
  }

  for (int rx = 0; rx < rmWidth; rx++) {
    for (int ry = 0; ry < rmWidth; ry++) {
      rangeMap[rx + ry*rmWidth] = 0;
      rangeMapReg1[rx + ry*rmWidth] = 0;
      rangeMapReg2[rx + ry*rmWidth] = 0;
      rangeMapMass[rx + ry*rmWidth] = 0;
      rangeMapAccumulator[rx + ry*rmWidth] = 0;
    }
  }
  rangemapImage = Mat(rmiHeight, 3*rmiWidth, CV_8UC3);

  for (int rx = 0; rx < hrmWidth; rx++) {
    for (int ry = 0; ry < hrmWidth; ry++) {
      hiRangeMap[rx + ry*hrmWidth] = 0;
      hiRangeMapReg1[rx + ry*hrmWidth] = 0;
      hiRangeMapReg2[rx + ry*hrmWidth] = 0;
      hiRangeMapMass[rx + ry*hrmWidth] = 0;
      hiRangeMapAccumulator[rx + ry*hrmWidth] = 0;
    }
  }
  hiRangemapImage = Mat(hrmiHeight, 3*hrmiWidth, CV_8UC3);

  hiColorRangemapImage = Mat(hrmiHeight, hrmiWidth, CV_8UC3);

  rangeogramImage = Mat(rggHeight, rggWidth, CV_8UC3);

  rmcX = 0;
  rmcY = 0;
  rmcZ = 0;

  for (int g = 0; g < totalGraspGears; g++) {
    ggX[g] = 0;
    ggY[g] = 0;
    ggT[g] = double(g)*2.0*3.1415926/double(totalGraspGears);
  }
  // old orientation
  //ggX[0] =  0.03;
  //ggY[0] =  0.02;
  //ggX[1] =  0.04;
  //ggY[1] =  0.00;
  //ggX[2] =  0.03;
  //ggY[2] = -0.02;
  //ggX[3] =  0.00;
  //ggY[3] = -0.03; //-0.04

  // new orientation
  // verticle calibration
  ggX[0] =  0.02;
  ggY[0] =  0.02;
  ggX[1] =  0.03;
  ggY[1] =  0.00;
  ggX[2] =  0.02;
  ggY[2] = -0.02;
  ggX[3] =  0.00;
  ggY[3] = -0.03;//-0.03; //-0.04

  ggX[4] = -0.02;
  ggY[4] = -0.02;
  ggX[5] = -0.03;
  ggY[5] = -0.00;
  ggX[6] = -0.02;
  ggY[6] =  0.02;
  ggX[7] = -0.00;
  ggY[7] =  0.03;//-0.03; //-0.04

  // XXX set this to be arm-generic
  // XXX add symbols to change register sets
  eepReg3 = crane4right;

  initializeParzen();
  //l2NormalizeParzen();
  initialize3DParzen();
  //l2Normalize3DParzen();

#ifdef DEBUG
#endif
  {
    Eigen::Quaternionf crane2quat(crane2right.qw, crane2right.qx, crane2right.qy, crane2right.qz);
    //Eigen::Quaternionf gear0offset(0.0, 0.0, 0.0, 0.0); // for calibration
    //Eigen::Quaternionf gear0offset(0.0, ggX[0], ggY[0], 0.0); // for initial calibration
    //Eigen::Quaternionf gear0offset(0.0, .023, .022, 0.0); // for latest ray calibration
    //Eigen::Quaternionf gear0offset(0.0, .023, .023, 0.0); // eyeball correction

    // gripper x y z
    // 0.617956
    // -0.301304
    // 0.0527889
    // right range x y z
    // 0.585962
    // -0.321487
    // 0.0695117
    // 
    // 0.037829849
    //Eigen::Quaternionf gear0offset(0.0, 0.031996, 0.020183, 0.0167228); // from TF, accounts for upside down rotation of crane2
    Eigen::Quaternionf gear0offset(0.0, 0.023, 0.023, 0.0167228); // z is from TF, good for depth alignment
    //Eigen::Quaternionf gear0offset(0.0, 0.020183, 0.031996, 0.0167228); // from TF, accounts for upside down rotation of crane2

    // invert the transformation
    irGlobalPositionEEFrame = crane2quat.conjugate() * gear0offset * crane2quat;

    // initial calibration
    // irGlobalPositionEEFrame w x y z: 1.62094e-11 -0.0205133 0.0194367 0.00119132
    //irGlobalPositionEEFrame = Eigen::Quaternionf(1.62094e-11,-0.0205133,0.0194367,0.00119132);
    // ray calibration
    // irGlobalPositionEEFrame w x y z: -3.73708e-14 -0.0206869 0.0244346 -2.52088e-05
    //irGlobalPositionEEFrame = Eigen::Quaternionf(,,,);

    cout << "irGlobalPositionEEFrame w x y z: " << irGlobalPositionEEFrame.w() << " " << 
      irGlobalPositionEEFrame.x() << " " << irGlobalPositionEEFrame.y() << " " << irGlobalPositionEEFrame.z() << endl;
  }

  for (int h = 0; h < hrmWidth; h++) {
    for (int i = 0; i < hrmWidth; i++) {
      hiColorRangeMapMass[h + i*hrmWidth] = 0;
      for (int j = 0; j < 3; j++) {
	hiColorRangeMapAccumulator[h + i*hrmWidth + j*hrmWidth*hrmWidth] = 0;
      }
    }
  }
  
  imRingBuffer.resize(imRingBufferSize);
  epRingBuffer.resize(epRingBufferSize);
  rgRingBuffer.resize(rgRingBufferSize);

  imRBTimes.resize(imRingBufferSize);
  epRBTimes.resize(epRingBufferSize);
  rgRBTimes.resize(rgRingBufferSize);

  for (int pz = 0; pz < vmWidth; pz++) {
    for (int py = 0; py < vmWidth; py++) {
      for (int px = 0; px < vmWidth; px++) {
	volumeMap[px + py*vmWidth + pz*vmWidth*vmWidth] = 0;
	volumeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth] = 0;
	volumeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth] = 0;
	vmColorRangeMapMass[px + py*vmWidth + pz*vmWidth*vmWidth] = 0;
	for (int pc = 0; pc < 3; pc++) {
	  vmColorRangeMapAccumulator[px + py*vmWidth + pz*vmWidth*vmWidth + pc*vmWidth*vmWidth*vmWidth] = 0;
	}
      }
    }
  }
}

void spinlessPilotMain() {
  cout << endl << endl << "Pilot main begin..." << endl;
  pilotInit();
}

////////////////////////////////////////////////
// end pilot definitions 
//
// start node definitions 
////////////////////////////////////////////////

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

bool isFiniteNumber(double x) {
    return (x <= DBL_MAX && x >= -DBL_MAX); 
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
  vector< cv::Vec<double,4> >& classQuaternions, int lIndexStart) {

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

/*
void depthCallback(const sensor_msgs::ImageConstPtr& msg) {

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
}
*/

void getCluster(pcl::PointCloud<pcl::PointXYZRGB> &cluster, pcl::PointCloud<pcl::PointXYZRGB> &cloud, std::vector<cv::Point> &points) {
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

geometry_msgs::Pose getPose(pcl::PointCloud<pcl::PointXYZRGB> &cluster) {
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

void getPointCloudPoints(vector<cv::Point>& pointCloudPoints, double *pBoxIndicator, double thisThresh, 
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
	  rectangle(objectViewerImage, thisTop, thisBot, cv::Scalar(100,100,255));
	pointCloudPoints.push_back(cv::Point(x,y));
      }
    }
  }
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
  if (isOrientedFilterPoseModel(classPoseModels[label])) {
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
  } else if (isOrientedFilterPoseModel(classPoseModels[label])) {
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

void getOrientation(vector<KeyPoint>& keypoints, Mat& descriptors, cv::Point top, cv::Point bot, 
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

    if (isOrientedFilterPoseModel(classPoseModels[label])) {
      Mat *orientedFilters;

      orientedFilterType thisType = getOrientedFilterType(classPoseModels[label]);

      if (thisType == MRT)
	orientedFilters = orientedFiltersT;
      else if (thisType == SPOON)
	orientedFilters = orientedFiltersS;
      else if (thisType == KNIFE)
	orientedFilters = orientedFiltersK;
      else {
	cout << "Invalid oriented filter type. Exiting." << endl;
	exit(EXIT_FAILURE);
      }

#ifdef DEBUG
fprintf(stderr, " object checkS"); fflush(stderr);
cout << top << " " << bot << " "; cout.flush();
#endif

      int boxWidth  = bot.x-top.x;
      int boxHeight = bot.y-top.y;

      int xxs = max(0, top.x - oSearchWidth*gBoxStrideX);
      int xxf = min(gBoxStrideX*((densityViewerImage.size().width-1-boxWidth)/gBoxStrideX), top.x + oSearchWidth*gBoxStrideX);
      int yys = max(0, top.y - oSearchWidth*gBoxStrideY);
      int yyf = min(gBoxStrideY*((densityViewerImage.size().height-1-boxHeight)/gBoxStrideY), top.y + oSearchWidth*gBoxStrideY);

//xxs = top.x;
//xxf = top.x;
//yys = top.y;
//yyf = top.y;

      double winningScore = -1;
      int winningX = -1;
      int winningY = -1;
      for (int yy = yys; yy <= yyf; yy+=gBoxStrideY) {
	for (int xx = xxs; xx <= xxf; xx+=gBoxStrideX) {
	  //Mat gCrop1 = densityViewerImage(cv::Rect(top.x, top.y, bot.x-top.x, bot.y-top.y));
	  Mat gCrop1 = densityViewerImage(cv::Rect(xx, yy, bot.x-top.x, bot.y-top.y));

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
      if (isOrientedFilterPoseModel(classPoseModels[label])) {
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
	Mat vCrop = objectViewerImage(cv::Rect(top.x, top.y, bot.x-top.x, bot.y-top.y));
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

void init_oriented_filters(orientedFilterType thisType) {
  Mat *orientedFilters; 
  if (thisType == MRT)
    orientedFilters = orientedFiltersT;
  else if (thisType == SPOON)
    orientedFilters = orientedFiltersS;
  else if (thisType == KNIFE)
    orientedFilters = orientedFiltersK;
  else {
    cout << "Invalid oriented filter type. Exiting." << endl;
    exit(EXIT_FAILURE);
  }

  for (int x = 0; x < O_FILTER_WIDTH; x++) {
    for (int y = 0; y < O_FILTER_WIDTH; y++) {
      orientedFilters[0].at<double>(y,x) = 0.0;
    }
  }
  // Spoon filters are used to estimate the orientation of spoon-like objects
  // based on their green maps. Hard coded for convenience. 
  // This approach is pretty flexible and works for other types of objects.
  // E.g., mrT and knives.
  // A map could be estimated by some other process and loaded here, or just represented
  // as bitmap and converted here.
  // We go on to make this comparison based based on an affine transformation, it is more accurate.
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

  if (thisType == MRT) {
    // mrT Filter
    int center = (O_FILTER_WIDTH-1)/2;
    for (int x = center-O_FILTER_SPOON_SHAFT_WIDTH; x <= center+O_FILTER_SPOON_SHAFT_WIDTH; x++) {
      for (int y = 0; y < O_FILTER_WIDTH; y++) {
	orientedFilters[0].at<double>(y,x) = 1.0;
      }
    }
    for (int x = center-5*O_FILTER_SPOON_SHAFT_WIDTH; x <= center+5*O_FILTER_SPOON_SHAFT_WIDTH; x++) {
      for (int y = 0; y < 2*O_FILTER_SPOON_SHAFT_WIDTH-1; y++) {
	orientedFilters[0].at<double>(y,x) = 1.0;
      }
    }
  } else if (thisType == SPOON) {
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
  } else if (thisType == KNIFE) {
    // Vertical Knife Filter
    int center = (O_FILTER_WIDTH-1)/2;
    for (int x = center-O_FILTER_SPOON_SHAFT_WIDTH; x <= center+O_FILTER_SPOON_SHAFT_WIDTH; x++) {
      for (int y = 0; y < O_FILTER_WIDTH; y++) {
	orientedFilters[0].at<double>(y,x) = 1.0;
      }
    }
  } else {
    cout << "Invalid oriented filter type. Exiting." << endl;
    exit(EXIT_FAILURE);
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

void init_oriented_filters_all() {
  init_oriented_filters(MRT);
  init_oriented_filters(SPOON);
  init_oriented_filters(KNIFE);
}

int isOrientedFilterPoseModel(string toCompare) {
  return ((0 == toCompare.compare("S")) || 
	  (0 == toCompare.compare("T")) || 
	  (0 == toCompare.compare("K")) );
}

orientedFilterType getOrientedFilterType(string toCompare) {
  if (0 == toCompare.compare("T"))
    return MRT;
  if (0 == toCompare.compare("S"))
    return SPOON;
  if (0 == toCompare.compare("K"))
    return KNIFE;
  else
    return OFT_INVALID;
}

void nodeCallbackFunc(int event, int x, int y, int flags, void* userdata) {
  if ( event == EVENT_LBUTTONDOWN ) {
    if (captureOnly) 
      fc = frames_per_click;
    if (saveAnnotatedBoxes)
      fc = frames_per_click;
    if (captureHardClass)
      fc = frames_per_click;
 
    //cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if ( event == EVENT_RBUTTONDOWN ) {
    //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if  ( event == EVENT_MBUTTONDOWN ) {
    //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if ( event == EVENT_MOUSEMOVE ) {
    //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
  }
}









void goAddBlinders() {
  Size sz = objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;

  int thisWidth = blinder_stride*blinder_columns;
  for (int x = 0; x < thisWidth; x++) {
    for (int y = 0; y < imH; y++) {
      int bCol = x/blinder_stride;
      int bRow = y/blinder_stride;
      int blackOrWhite = ((bCol + bRow)%2)*255;
      objectViewerImage.at<cv::Vec3b>(y, x) = cv::Vec<uchar, 3>(blackOrWhite, blackOrWhite, blackOrWhite);
    }
  }
  for (int x = imW-thisWidth; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      int bCol = x/blinder_stride;
      int bRow = y/blinder_stride;
      int blackOrWhite = ((bCol + bRow)%2)*255;
      objectViewerImage.at<cv::Vec3b>(y, x) = cv::Vec<uchar, 3>(blackOrWhite, blackOrWhite, blackOrWhite);
    }
  }
  /*
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < thisWidth; y++) {
      int bCol = x/blinder_stride;
      int bRow = y/blinder_stride;
      int blackOrWhite = ((bCol + bRow)%2)*255;
      objectViewerImage.at<cv::Vec3b>(y, x) = cv::Vec<uchar, 3>(blackOrWhite, blackOrWhite, blackOrWhite);
    }
  }
  for (int x = 0; x < imW; x++) {
    for (int y = imH-thisWidth; y < imH; y++) {
      int bCol = x/blinder_stride;
      int bRow = y/blinder_stride;
      int blackOrWhite = ((bCol + bRow)%2)*255;
      objectViewerImage.at<cv::Vec3b>(y, x) = cv::Vec<uchar, 3>(blackOrWhite, blackOrWhite, blackOrWhite);
    }
  }
  */
}

void goCalculateDensity() {
  Size sz = objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;

  objectViewerImage = cv_ptr->image.clone();
  densityViewerImage = cv_ptr->image.clone();

  int boxesPerSize = 800;

  //Mat cam_img2;
  //cam_img2 = cam_img;
  
  // XXX find best method and clean up all this mess
  //Mat original_cam_img = cam_img2.clone();
  //densityViewerImage = cam_img2.clone();

//  // XXX Sobel business
//  Mat local_ave;
//  cvtColor(densityViewerImage_blur, densityViewerImage_blur, CV_RGB2GRAY );
//  GaussianBlur(densityViewerImage_blur, densityViewerImage_blur, Size(max(4*sobel_sigma+1, 17.0),max(4*sobel_sigma+1, 17.0)), sobel_sigma, sobel_sigma, BORDER_DEFAULT); 
//  Mat grad_x, grad_y;
//  int sobelScale = 1;
//  int sobelDelta = 0;
//  int sobelDepth = CV_32F;
//  /// Gradient X
//  Sobel(densityViewerImage_blur, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
//  /// Gradient Y
//  Sobel(densityViewerImage_blur, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
//
//  grad_x = grad_x.mul(grad_x);
//  grad_y = grad_y.mul(grad_y);
//  Mat totalSobel = grad_x + grad_y;
//  // now totalSobel is gradient magnitude squared
//
//  // make it gradient magnitude to the fourth to spread the values a little
//  // this increases robustness and makes it easier to tune
//  pow(totalSobel, 2.0, totalSobel);
//  totalSobel = totalSobel * sobel_scale_factor;
//
//  //totalSobel = abs(totalSobel);
//  //cv::sqrt(totalSobel, totalSobel);
//  //cv::sqrt(totalSobel, totalSobel);
//
//  // try local contrast normalization
//  //GaussianBlur(totalSobel, local_ave, Size(max(4*local_sobel_sigma+1, 17.0),max(4*local_sobel_sigma+1, 17.0)), local_sobel_sigma, local_sobel_sigma, BORDER_DEFAULT); 
//  //local_ave = cv::max(local_ave,.000001);
//  //totalSobel = local_ave / totalSobel;
//
//  // try laplacian
//  Mat lapl;
//  Laplacian(densityViewerImage_blur, lapl, sobelDepth, 1, sobelScale, sobelDelta, BORDER_DEFAULT);
//  //pow(totalSobel, 4.0, totalSobel);
//  //totalSobel = max(totalSobel, .001);
//  //totalSobel = 1 / totalSobel;
//  totalSobel = lapl.mul(totalSobel);

  //vector<Mat> channels;
  //channels.push_back(densityViewerImageGtmp);
  //channels.push_back(densityViewerImageGtmp);
  //channels.push_back(densityViewerImageGtmp);
  //merge(channels, densityViewerImageG);

  // input image is noisy so blurring is a good idea
  //GaussianBlur(densityViewerImage, densityViewerImage, cv::Size(0,0), 1.0);

  ValStructVec<float, Vec4i> boxes;
  glObjectness->getObjBndBoxes(densityViewerImage, boxes, boxesPerSize);

  int numBoxes = boxes.size();

  #ifdef DEBUG
  cout << "numBoxes: " << numBoxes << "  fc: " << fc <<  endl;
  #endif

  nTop.resize(numBoxes);
  nBot.resize(numBoxes);

  int boxesToConsider= 50000;

  if (integralDensity == NULL)
    integralDensity = new double[imW*imH];
  if (density == NULL)
    density = new double[imW*imH];
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

  //if (drawWhite) {
    //// draw the ork bounding boxes
    //for(int i =0; i<wTop.size(); i++){
      //rectangle(objectViewerImage, wTop[i], wBot[i], cv::Scalar(255,255,255));
    //}
  //}

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
	  rectangle(objectViewerImage, outTop, outBot, cv::Scalar(188,40,140));
	  rectangle(objectViewerImage, inTop, inBot, cv::Scalar(94,20,70));
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
  maxDensity = 0;
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
    rectangle(objectViewerImage, outTop, outBot, cv::Scalar(128,128,128));
    rectangle(objectViewerImage, inTop, inBot, cv::Scalar(32,32,32));
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
    Mat vCrop = objectViewerImage(cv::Rect(xs, ys, xe-xs, ye-ys));
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
    Mat vCrop2 = objectViewerImage(cv::Rect(xs, ys, xe-xs, ye-ys));
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

  delete differentialDensity;
}

void goFindBlueBoxes() {
  Size sz = objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;

  gBoxIndicator = new double[imW*imH];
  double *gBoxGrayNodes = new double[imW*imH];
  double *gBoxComponentLabels = new double[imW*imH];
  if (pBoxIndicator == NULL)
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
		rectangle(objectViewerImage, thisTop, thisBot, cv::Scalar(0,128,0));
      }
      if (thisIntegral > adjusted_canny_hi_thresh) {
	      gBoxIndicator[y*imW+x] = 2;
	      if (drawGreen)
		rectangle(objectViewerImage, thisTop, thisBot, cv::Scalar(0,255,0));
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
      densityViewerImage.at<cv::Vec3b>(y,x) = cv::Vec<uchar, 3>(0,val,0);
    }
  }

  if (drawBlue) {
    for (int c = bTops.size()-1; c >= 0; c--) {
      cv::Point outTop = cv::Point(bTops[c].x, bTops[c].y);
      cv::Point outBot = cv::Point(bBots[c].x, bBots[c].y);
      cv::Point inTop = cv::Point(bTops[c].x+1,bTops[c].y+1);
      cv::Point inBot = cv::Point(bBots[c].x-1,bBots[c].y-1);
      rectangle(objectViewerImage, outTop, outBot, cv::Scalar(255,0,0));
      rectangle(objectViewerImage, inTop, inBot, cv::Scalar(255,192,192));
    }
  }

  if (shouldIRender)
    cv::imshow(objectViewerName, objectViewerImage);
  if (shouldIRender)
    cv::imshow(densityViewerName, densityViewerImage);

  delete gBoxIndicator;
  delete gBoxGrayNodes;
  delete gBoxComponentLabels;
}

void goFindBrownBoxes() {
  Size sz = objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;

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

	    if (drawBrown)
	      rectangle(objectViewerImage, thisBrTop, thisBrBot, cv::Scalar(0,102,204));
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
		rectangle(objectViewerImage, thisBrTop, thisBrBot, cv::Scalar(0,76,153));
	    }

	    if (!reject) {
	      acceptedBrBoxes++;
	      if (drawBrown)
		rectangle(objectViewerImage, thisBrTop, thisBrBot, cv::Scalar(0,51,102));

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
  
  init_oriented_filters_all();

  // draw it again to go over the brown boxes
  if (drawGray) {
    cv::Point outTop = cv::Point(grayTop.x, grayTop.y);
    cv::Point outBot = cv::Point(grayBot.x, grayBot.y);
    cv::Point inTop = cv::Point(grayTop.x+1,grayTop.y+1);
    cv::Point inBot = cv::Point(grayBot.x-1,grayBot.y-1);
    rectangle(objectViewerImage, outTop, outBot, cv::Scalar(128,128,128));
    rectangle(objectViewerImage, inTop, inBot, cv::Scalar(32,32,32));
  }

  if (shouldIRender)
    cv::imshow(objectViewerName, objectViewerImage);
}

void goClassifyBlueBoxes() {
  Size sz = objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;

  vector< vector<int> > pIoCbuffer;

  // classify the crops
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

    Mat original_cam_img = cam_img;
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
	    rectangle(objectViewerImage, kpTop, kpBot, cv::Scalar(255,0,0));
	  }
	}
      }

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

    // XXX 
    //getOrientation(keypoints, descriptors, bTops[c], bBots[c], 
      //label, labelName, augmentedLabelName, poseIndex, winningO);
    
    if (saveAnnotatedBoxes) {
      // save the crops
      if (fc > 0) {

	int conditionFulfilled = false;
	if (captureHardClass)
	  conditionFulfilled = ( (0 != labelName.compare(table_label_class_name)) &&
				 (0 != labelName.compare(background_class_name) ) );
	else
	  conditionFulfilled = ( 0 == labelName.compare(class_name) );

	if (conditionFulfilled) {

	  string thisLabelName = labelName;
	  if (captureHardClass)
	    thisLabelName = class_name;

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
    }

    if (drawLabels) {
      cv::Point text_anchor(bTops[c].x+1, bBots[c].y-2);
      cv::Point text_anchor2(bTops[c].x+1, bBots[c].y-2);
      putText(objectViewerImage, augmentedLabelName, text_anchor, MY_FONT, 1.5, Scalar(255,192,192), 2.0);
      putText(objectViewerImage, augmentedLabelName, text_anchor2, MY_FONT, 1.5, Scalar(255,0,0), 1.0);
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
    getPointCloudPoints(pointCloudPoints, pBoxIndicator, thisThresh, 
      bTops[c], bBots[c], imW, imH, gBoxStrideX, gBoxStrideY, gBoxW, gBoxH);

    #ifdef DEBUG
    fprintf(stderr, " object check6"); fflush(stderr);
    #endif

    if (publishObjects) {
      if (label >= 0) {
	fill_RO_and_M_arrays(roa_to_send_blue, 
	  ma_to_send_blue, pointCloudPoints, c, label, winningO, poseIndex);
      }
    }
  }

  if (shouldIRender)
    cv::imshow(objectViewerName, objectViewerImage);
  if (shouldIRender)
    cv::imshow(densityViewerName, densityViewerImage);
}

void goFindRedBoxes() {
  Size sz = objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;

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
		if (drawBlueKP) {
		  cv::Point kpTop = cv::Point(bTops[c].x+tX,bTops[c].y+tY);
		  cv::Point kpBot = cv::Point(bTops[c].x+tX+1,bTops[c].y+tY+1);
		  if(
		    (kpTop.x >= 1) &&
		    (kpBot.x <= imW-2) &&
		    (kpTop.y >= 1) &&
		    (kpBot.y <= imH-2) 
		    ) {
		    rectangle(objectViewerImage, kpTop, kpBot, cv::Scalar(0,0,255));
		  }
		}
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
		  rectangle(objectViewerImage, outTop, outBot, 0.5*cv::Scalar(64,64,192));
		  rectangle(objectViewerImage, inTop, inBot, 0.5*cv::Scalar(160,160,224));
		}

		{
		  cv::Point outTop = cv::Point(itTop.x, itTop.y);
		  cv::Point outBot = cv::Point(itBot.x, itBot.y);
		  cv::Point inTop = cv::Point(winTop.x+1,winTop.y+1);
		  cv::Point inBot = cv::Point(winBot.x-1,winBot.y-1);
		  rectangle(objectViewerImage, outTop, outBot, 0.5*cv::Scalar(0,0,255));
		  rectangle(objectViewerImage, inTop, inBot, 0.5*cv::Scalar(192,192,255));
		}

		//{
		  //cv::Point outTop = cv::Point(winTop.x, winTop.y);
		  //cv::Point outBot = cv::Point(winBot.x, winBot.y);
		  //cv::Point inTop = cv::Point(winTop.x+1,winTop.y+1);
		  //cv::Point inBot = cv::Point(winBot.x-1,winBot.y-1);
		  //rectangle(objectViewerImage, outTop, outBot, 0.5*cv::Scalar(0,0,255));
		  //rectangle(objectViewerImage, inTop, inBot, 0.5*cv::Scalar(192,192,255));
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

    getOrientation(winKeypoints, winDescriptors, winTop, winBot, 
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
	rectangle(objectViewerImage, outTop, outBot, cv::Scalar(64,64,192));
	rectangle(objectViewerImage, inTop, inBot, cv::Scalar(160,160,224));

	if (drawLabels) {
	  cv::Point text_anchor(dTop.x+1, dBot.y-2);
	  cv::Point text_anchor2(dTop.x+1, dBot.y-2);
	  putText(objectViewerImage, augmentedLabelName, text_anchor, MY_FONT, 1.5, Scalar(160,160,224), 2.0);
	  putText(objectViewerImage, augmentedLabelName, text_anchor2, MY_FONT, 1.5, Scalar(64,64,192), 1.0);
	}
      }

      {
	cv::Point outTop = cv::Point(winTop.x, winTop.y);
	cv::Point outBot = cv::Point(winBot.x, winBot.y);
	cv::Point inTop = cv::Point(winTop.x+1,winTop.y+1);
	cv::Point inBot = cv::Point(winBot.x-1,winBot.y-1);
	rectangle(objectViewerImage, outTop, outBot, cv::Scalar(0,0,255));
	rectangle(objectViewerImage, inTop, inBot, cv::Scalar(192,192,255));

	if (drawLabels) {
	  cv::Point text_anchor(winTop.x+1, winBot.y-2);
	  cv::Point text_anchor2(winTop.x+1, winBot.y-2);
	  putText(objectViewerImage, augmentedLabelName, text_anchor, MY_FONT, 1.5, Scalar(192,192,255), 2.0);
	  putText(objectViewerImage, augmentedLabelName, text_anchor2, MY_FONT, 1.5, Scalar(0,0,255), 1.0);
	}
      }
    }

    double thisThresh = pBoxThresh;

    if (publishObjects) {
      vector<cv::Point> pointCloudPoints;
      getPointCloudPoints(pointCloudPoints, pBoxIndicator, thisThresh, 
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
    }
  }

  if (publishObjects) {
    if (numRedBoxes > 0) {
      rec_objs_red.publish(roa_to_send_red);
      markers_red.publish(ma_to_send_red);
    }
  }
}

void nodeImageCallback(const sensor_msgs::ImageConstPtr& msg) {

  ros::NodeHandle nh("~");

  loadROSParams();

  invertQuaternionLabel = 0;

  gBoxStrideX = gBoxW / 2.0;
  gBoxStrideY = gBoxH / 2.0;

  //cv::Rect bound;
  //cv::Mat boxed;

  Size sz = objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;

  if (add_blinders) {
    goAddBlinders();
  }

  goCalculateDensity();

  goFindBlueBoxes();

  roa_to_send_blue.objects.resize(bTops.size());
  ma_to_send_blue.markers.resize(bTops.size()+1);

  if (runInference) {

    goFindBrownBoxes();

    goClassifyBlueBoxes();

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

    if (publishObjects) {
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
    }

    if (saveAnnotatedBoxes)
      if (fc > 0)
	fc--;

    if (runTracking) {
      goFindRedBoxes();
    }
  }


  if (saveBoxes) {
    // save the crops
    if (fc > 0) {
      fc--;
      for (int c = bTops.size()-1; c >= 0; c--) {
	Mat original_cam_img = cam_img;
	Mat crop = original_cam_img(cv::Rect(bTops[c].x, bTops[c].y, bBots[c].x-bTops[c].x, bBots[c].y-bTops[c].y));
	char buf[1000];
	sprintf(buf, "%s%s%s_%d.ppm", saved_crops_path.c_str(), class_name.c_str(), run_prefix.c_str(), cropCounter);
	cout << buf << " " << bTops[c] << bBots[c] << original_cam_img.size() << crop.size() << endl;
	imwrite(buf, crop);
	cropCounter++;
      }
    }
  }

  cv::imshow(objectViewerName, objectViewerImage);
  cv::imshow(densityViewerName, densityViewerImage);

  int kc = cv::waitKey(1);
  saveROSParams();
}

/*
void tableCallback(const object_recognition_msgs::Table& msg) {
#ifdef DEBUG
  cout << "Hit tableCloudCallback" << endl;
#endif
  tablePose = msg.pose;
}
*/

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  pcl::fromROSMsg(*msg, pointCloud);
#ifdef DEBUG
cout << "Hit pointCloudCallback" <<  "  " << pointCloud.size() << endl;
#endif
}

//void clusterCallback(const visualization_msgs::MarkerArray& msg) {
//	if(real_img){
//		object_recognition_msgs::RecognizedObjectArray to_send;
//		to_send.objects.resize(msg.markers.size());
//		//MatrixXf imfea2;
//		//VectorXf scores;
//		IplImage temp = cam_img;
//		int height = cam_img.size().height;
//		int width = cam_img.size().width;
//		ROS_INFO("(%d,%d)", height, width);
//		float minx, miny, maxx, maxy, px, py;
//		cv::Rect bound;
//		cv::Mat boxed;
//		geometry_msgs::Point p;
//		wTop.resize(msg.markers.size());
//		wBot.resize(msg.markers.size());
//		ROS_INFO("Objects found: %d", int(msg.markers.size()));
//		float f;
//		std::string res;
//		//if(LOC_MODEL_TYPE == 0) f= 580;
//		//else f=525;
//		f = 525;
//		float cx = (width/2) - 0.5;
//		float cy = (height/2) - 0.5;
//		for(int i=0; i<msg.markers.size(); i++){
//			minx = f*(msg.markers[i].points[0].x/msg.markers[i].points[0].z) + cx;
//			maxx = f*(msg.markers[i].points[0].x/msg.markers[i].points[0].z) + cx;
//			miny = f*(msg.markers[i].points[0].y/msg.markers[i].points[0].z) + cy;
//			maxy = f*(msg.markers[i].points[0].y/msg.markers[i].points[0].z) + cy;
//			for(int j = 0; j < msg.markers[i].points.size(); j++){
//				p = msg.markers[i].points[j];
//				px = f*(p.x/p.z) + cx;
//				py = f*(p.y/p.z) + cy;
//				if(px < minx){minx = px;}
//				if(px > maxx){maxx = px;}
//				if(py < miny){miny = py;}
//				if(py > maxy){maxy = py;}
//			}
//			wTop[i] = cv::Point(minx - 5, miny - 5);
//			wBot[i] = cv::Point(maxx + 5, maxy + 5);
//			if(wTop[i].x < 0){wTop[i].x = 0;}
//			else if(wTop[i].x > width - 1){wTop[i].x = width-1;}
//			if(wTop[i].y < 0) wTop[i].y = 0;
//			else if(wTop[i].y > height-1){wTop[i].y = height-1;}
//			if(wBot[i].x < 0){wBot[i].x = 0;}
//			else if(wBot[i].x > width - 1){wBot[i].x = width-1;}
//			if(wBot[i].y < 0) wBot[i].y = 0;
//			else if(wBot[i].y > height-1){wBot[i].y = height-1;}
//		}/*
//		for(int i=0; i<wTop.size(); i++){
//			ROS_INFO("BOXED: (%d, %d) to (%d, %d)", wTop[i].x, wTop[i].y, wBot[i].x, wBot[i].y);
//			bound = cv::Rect(wTop[i].x, wTop[i].y, wBot[i].x - wTop[i].x, wBot[i].y - wTop[i].y);
//			boxed = cam_img(bound);
//			temp = boxed;
//			kdm->Process(imfea2, &temp);
//			kdm->Classify(scores, imfea2);
//			res = kdm->GetObjectName(scores);
//			ROS_INFO("Tabletop %d identified as %s", i, res.c_str());
//			meldon_detection::RecognizedObject to_add;
//			to_add.points = msg.markers[i].points;
//			to_add.name = res.c_str();
//			to_send.objects[i] = to_add;
//		}*/
//		temp = cam_img;
//		//kdm->Process(imfea2, &temp);
//		//kdm->Classify(scores, imfea2);
//		//rec_objs.publish(to_send);
//	}
//	ROS_INFO("Identification complete");
//}

// TODO probably don't need two separate functions for this
void loadROSParamsFromArgs() {
  ros::NodeHandle nh("~");

  nh.getParam("default_ikShare", default_ikShare);

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

  nh.getParam("chosen_feature", cfi);
  chosen_feature = static_cast<featureType>(cfi);

  saved_crops_path = data_directory + "/" + class_name + "/";
}

void loadROSParams() {
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

void saveROSParams() {
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

  nh.setParam("chosen_feature", cfi);
  chosen_feature = static_cast<featureType>(cfi);

}

void spinlessNodeMain() {
  cout << endl << endl << "Node main begin..." << endl;

  nodeInit();

  if (runInference) {
    detectorsInit();
  }

  if (trainOnly)
    exit(EXIT_SUCCESS);
}

void nodeInit() {
  tableLabelQuaternion.x() = 0;
  tableLabelQuaternion.y() = 0;
  tableLabelQuaternion.z() = 0;
  tableLabelQuaternion.w() = 1;

  gBoxStrideX = gBoxW / 2.0;
  gBoxStrideY = gBoxH / 2.0;
  fc = 0;
  cropCounter = 0;
  tableNormal = Eigen::Vector3d(1,0,0);
  tableBias = 0;

  // manually definining spoon filters
  orientedFiltersT = new Mat[ORIENTATIONS];
  orientedFiltersT[0].create(O_FILTER_WIDTH, O_FILTER_WIDTH, CV_64F);
  orientedFiltersS = new Mat[ORIENTATIONS];
  orientedFiltersS[0].create(O_FILTER_WIDTH, O_FILTER_WIDTH, CV_64F);
  orientedFiltersK = new Mat[ORIENTATIONS];
  orientedFiltersK[0].create(O_FILTER_WIDTH, O_FILTER_WIDTH, CV_64F);

  tablePerspective = Mat::eye(3,3,CV_32F);
  init_oriented_filters_all();

  initRedBoxes();
}

void detectorsInit() {
  // XXX TODO this function should check for the existence of and destroy 
  //  old copies of the structures if this function is to be called multiple times

  // SIFT 
  //detector = new SiftFeatureDetector(0, 3, 0.04, 10, 1.6);
  cout << "chosen_feature: " << chosen_feature << endl;
  detector = new FastFeatureDetector(4);
  if (chosen_feature == SIFTBOW_GLOBALCOLOR_HIST)
    extractor = new SiftDescriptorExtractor();
  else if (chosen_feature == OPPONENTSIFTBOW_GLOBALCOLOR_HIST)
    extractor = DescriptorExtractor::create("OpponentSIFT");

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

  string bufstr; // Have a buffer string

  if (trainOnly) {
    stringstream ss_cl(class_labels); 
    while (ss_cl >> bufstr)
      classLabels.push_back(bufstr);

    stringstream ss_cpm(class_pose_models); 
    while (ss_cpm >> bufstr)
      classPoseModels.push_back(bufstr);

    cout << "Num labels: " << classLabels.size() << endl;
    cout << "Num pose models: " << classPoseModels.size() << endl;

    if ((classLabels.size() != classPoseModels.size()) || (classLabels.size() < 1)) {
      cout << "Label and pose model list size problem. Exiting." << endl;
      exit(EXIT_FAILURE);
    }
  }

  int numNewClasses = classLabels.size();
  int numCachedClasses = 0;

  if (rewrite_labels) {
    // load cached labels 
    vector<string> classCacheLabels;
    vector<string> classCachePoseModels;
    if (cache_prefix.size() > 0) {
      string labelsCacheFile = data_directory + "/" + cache_prefix + "labels.yml";

      FileStorage fsvI;
      cout<<"Reading CACHED labels and pose models from " << labelsCacheFile << " ...";
      fsvI.open(labelsCacheFile, FileStorage::READ);
      fsvI["labels"] >> classCacheLabels;
      fsvI["poseModels"] >> classCachePoseModels;
      //classLabels.insert(classLabels.end(), classCacheLabels.begin(), classCacheLabels.end());
      //classPoseModels.insert(classPoseModels.end(), classCachePoseModels.begin(), classCachePoseModels.end());
      cout << "done." << endl << "classCacheLabels size: " << classCacheLabels.size() << " classCachePoseModels size: " << classCachePoseModels.size() << endl;
      numCachedClasses = classCacheLabels.size();

      classCacheLabels.insert(classCacheLabels.end(), classLabels.begin(), classLabels.end());
      classCachePoseModels.insert(classCachePoseModels.end(), classPoseModels.begin(), classPoseModels.end());
      classLabels = classCacheLabels;
      classPoseModels = classCachePoseModels;
      cout << "classLabels size: " << classLabels.size() << " classPoseModels size: " << classPoseModels.size() << endl;
    }

    FileStorage fsvO;
    cout<<"Writing labels and pose models... " << labelsPath << " ...";
    fsvO.open(labelsPath, FileStorage::WRITE);
    fsvO << "labels" << classLabels;
    fsvO << "poseModels" << classPoseModels;
    fsvO.release();
    cout << "done." << endl;
  } else {
    FileStorage fsvI;
    cout<<"Reading labels and pose models... "<< labelsPath << " ...";
    fsvI.open(labelsPath, FileStorage::READ);
    fsvI["labels"] >> classLabels;
    fsvI["poseModels"] >> classPoseModels;
    cout << "done. classLabels size: " << classLabels.size() << " classPoseModels size: " << classPoseModels.size() << endl;
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

    cout << "Clustering features...";
    cout.flush();
    vocabulary = bowtrainer->cluster();
    cout << "done." << endl;

    FileStorage fsvO;
    cout << "Writing vocab... " << vocabularyPath << " ...";
    fsvO.open(vocabularyPath, FileStorage::WRITE);
    fsvO << "vocab" << vocabulary;
    fsvO.release();
    cout << "done." << endl;
  } else {
    FileStorage fsvI;
    cout << "Reading vocab... " << vocabularyPath << " ...";
    fsvI.open(vocabularyPath, FileStorage::READ);
    fsvI["vocab"] >> vocabulary;
    cout << "done. vocabulary size: " << vocabulary.size() << endl;
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

    // load cached kNN features 
    // XXX experimental handling of G pose models
    Mat kNNCachefeatures;
    Mat kNNCachelabels;
    if (cache_prefix.size() > 0) {
      string knnCacheFile = data_directory + "/" + cache_prefix + "knn.yml";

      FileStorage fsfI;
      cout<<"Reading CACHED features... " << knnCacheFile << " ..." << endl;
      fsfI.open(knnCacheFile, FileStorage::READ);
      fsfI["features"] >> kNNCachefeatures;
      fsfI["labels"] >> kNNCachelabels;
      kNNfeatures.push_back(kNNCachefeatures);
      kNNlabels.push_back(kNNCachelabels);

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

      cout << "done." << kNNCachefeatures.size() << " " << kNNCachelabels.size() << endl;
    }

    FileStorage fsfO;
    cout<<"Writing features and labels... " << featuresPath << " ..." << endl;
    fsfO.open(featuresPath, FileStorage::WRITE);
    fsfO << "features" << kNNfeatures;
    fsfO << "labels" << kNNlabels;

    // TODO also cache the features for the pose models

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
    cout<<"Reading features and labels... " << featuresPath << " ..." << endl;
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
    cout << "done. knnFeatures size: " << kNNfeatures.size() << " kNNlabels size: " << kNNlabels.size() << endl;
  }

  cout << "kNNlabels dimensions: " << kNNlabels.size().height << " by " << kNNlabels.size().width << endl;
  cout << "kNNfeatures dimensions: " << kNNfeatures.size().height << " by " << kNNfeatures.size().width << endl;

  cout << "Main kNN...";
  kNN = new CvKNearest(kNNfeatures, kNNlabels);
  cout << "done." << endl;
  for (int i = 0; i < numClasses; i++) {
    if (classPoseModels[i].compare("G") == 0) {
      cout << "Class " << i << " kNN..." << classPosekNNfeatures[i].size() << classPosekNNlabels[i].size() << endl;
      classPosekNNs[i] = new CvKNearest(classPosekNNfeatures[i], classPosekNNlabels[i]);
      cout << "Done" << endl;
    }
  }
}

void initRedBoxes() {
  string bufstr; // Have a buffer string
  if (runTracking) {
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
	cout << "Accepting red box suggestion " << r << " \"" << redBoxLabels[r] << "\" as red box number " << numRedBoxes-1 << endl;
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
	cout << "Rejecting red box suggestion " << r << " \"" << redBoxLabels[r] << "\"" << endl;
      }
    }
  }

  if (numRedBoxes > 0)
    createTrackbar("red target", objectViewerName, &redTrackbarVariable, numRedBoxes);
}

////////////////////////////////////////////////
// end node definitions 
//
// start ein 
////////////////////////////////////////////////

int main(int argc, char **argv) {

  srand(time(NULL));
  time(&firstTime);

  cout << "argc: " << argc << endl;
  for (int ccc = 0; ccc < argc; ccc++) {
    cout << argv[ccc] << endl;
  }
  cout << endl << endl;

  string programName;
  if (argc > 1) {
    programName = string(PROGRAM_NAME) + "_" + argv[argc-1];
    cout << "programName: " << programName << endl;
  }
  else
    programName = string(PROGRAM_NAME);

  ros::init(argc, argv, programName);
  ros::NodeHandle n("~");

  cout << "n namespace: " << n.getNamespace() << endl;

  loadROSParamsFromArgs();
  cout << "mask_gripper: " << mask_gripper << " add_blinders: " << add_blinders << endl;
  cout << "all_range_mode: " << all_range_mode << endl;
  cout << endl << "numRedBoxes: " << numRedBoxes << endl;
  cout << "data_directory: " << data_directory << endl << "class_name: " << class_name << endl 
       << "run_prefix: " << run_prefix << endl << "class_pose_models: " << class_pose_models << endl 
       << "class_labels: " << class_labels << endl << "vocab_file: " << vocab_file << endl 
       << "knn_file: " << knn_file << endl << "label_file: " << label_file << endl
       << endl;

  package_path = ros::package::getPath("node");
  class_crops_path = data_directory + "/";

  // The other models
  //ObjNessB2W8MAXBGR
  //ObjNessB2W8I
  //ObjNessB2W8HSV
  bing_trained_models_path = package_path + "/bing_trained_models/";
  objectness_path_prefix = bing_trained_models_path + "ObjNessB2W8MAXBGR";

  string vocPath = package_path + "/VOC2007/";
  DataSetVOC voc(vocPath);
  glObjectness = new Objectness(voc, 2, 8, 2);

  cout << "objectness_path_prefix: " << objectness_path_prefix << endl;
  int result = glObjectness->loadTrainedModel(objectness_path_prefix);
  cout << "result: " << result << endl << endl;

  if (result != 1) {
    cout << "ERROR: failed to load BING objectness model. Check the path prefix above. Exiting." << endl;
    exit(EXIT_FAILURE);
  }

  if ( (left_or_right_arm.compare("right") == 0) || (left_or_right_arm.compare("left") == 0) )
    image_topic = "/cameras/" + left_or_right_arm + "_hand_camera/image";

  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub;
  image_sub = it.subscribe(image_topic, 1, imageCallback);

  ros::Subscriber points = n.subscribe(pc_topic, 1, pointCloudCallback);
  //ros::Subscriber clusters = n.subscribe("/tabletop/clusters", 1, clusterCallback);

  rec_objs_blue = n.advertise<object_recognition_msgs::RecognizedObjectArray>("blue_labeled_objects", 10);
  rec_objs_red = n.advertise<object_recognition_msgs::RecognizedObjectArray>("red_labeled_objects", 10);
  markers_blue = n.advertise<visualization_msgs::MarkerArray>("blue_object_markers", 10);
  markers_red = n.advertise<visualization_msgs::MarkerArray>("red_object_markers", 10);
  ee_target_pub = n.advertise<geometry_msgs::Point>("pilot_target_" + left_or_right_arm, 10);

  densityViewerName = "Density Viewer " + left_or_right_arm;
  objectViewerName = "Object Viewer " + left_or_right_arm;

  cv::namedWindow(densityViewerName);
  cv::namedWindow(objectViewerName);
  setMouseCallback(objectViewerName, nodeCallbackFunc, NULL);

  createTrackbar("canny_lo", densityViewerName, &loTrackbarVariable, 100);
  createTrackbar("canny_hi", densityViewerName, &hiTrackbarVariable, 100);
  createTrackbar("blinder_columns", densityViewerName, &blinder_columns, 20);
  createTrackbar("blinder_stride", densityViewerName, &blinder_stride, 50);
  createTrackbar("add_blinders", densityViewerName, &add_blinders, 1);

  ros::Subscriber epState = n.subscribe("/robot/limb/" + left_or_right_arm + "/endpoint_state", 1, endpointCallback);
  ros::Subscriber eeRanger = n.subscribe("/robot/range/" + left_or_right_arm + "_hand_range/state", 1, rangeCallback);
  ros::Subscriber eeTarget = n.subscribe("/publish_detections_" + left_or_right_arm + "/pilot_target_" + left_or_right_arm, 1, targetCallback);
  ros::Subscriber jointSubscriber = n.subscribe("/robot/joint_states", 1, jointCallback);

  wristViewName = "Wrist View " + left_or_right_arm;
  coreViewName = "Core View " + left_or_right_arm;
  rangeogramViewName = "Rangeogram View " + left_or_right_arm;
  rangemapViewName = "Range Map View " + left_or_right_arm;
  hiRangemapViewName = "Hi Range Map View " + left_or_right_arm;
  hiColorRangemapViewName = "Hi Color Range Map View " + left_or_right_arm;

  cv::namedWindow(wristViewName);
  cv::setMouseCallback(wristViewName, pilotCallbackFunc, NULL);
  cv::namedWindow(coreViewName);
  cv::namedWindow(rangeogramViewName);

  ros::Timer timer1 = n.createTimer(ros::Duration(0.01), timercallback1);

  tfListener = new tf::TransformListener();

  ikClient = n.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/" + left_or_right_arm + "/PositionKinematicsNode/IKService");
  joint_mover = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/" + left_or_right_arm + "/joint_command", 10);
  gripperPub = n.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/" + left_or_right_arm + "_gripper/command",10);
  vmMarkerPublisher = n.advertise<visualization_msgs::MarkerArray>("volumetric_rgb_map", 10);

  {
    baxter_core_msgs::EndEffectorCommand command;
    command.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
    command.id = 65538;
    gripperPub.publish(command);
  }

  //spinlessNodeMain();
  spinlessPilotMain();

  saveROSParams();
  ros::spin();

  return 0;
}
