//#define DEBUG3
//#define DEBUG4
// start Header
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
//  //               /|\ 
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
#define VERYBIGNUMBER 1e6

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

// slu
#include "slu/math2d.h"

// numpy library 1 (randomkit, for original beta)
#include "distributions.h"
// numpy library 2 (cephes, for betainc)
//#include "cephes/protos.h"

#include <tf/transform_listener.h>

#include <sstream>
#include <iostream>
#include <math.h>

#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <sensor_msgs/Range.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/HeadPanCommand.h>

#include <dirent.h>

#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float64.h"
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

#include <opencv2/gpu/gpu.hpp>

#include "word.h"

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

#define NOW_THATS_FAST 0.08
#define MOVE_EVEN_FASTER 0.04
#define MOVE_FASTER 0.02
#define MOVE_FAST 0.01
#define MOVE_MEDIUM 0.005 //.005
#define MOVE_SLOW 0.0025
#define MOVE_VERY_SLOW 0.00125

#define RANGE_UPPER_INVALID 0.3
#define RANGE_LOWER_INVALID 0.08

#define STAR_SCAN 131144
#define CART_SCAN 131118
#define QUICK_RANGE_MAP CART_SCAN

////////////////////////////////////////////////
// end pilot includes, usings, and defines 
//
// start node includes, usings, and defines
////////////////////////////////////////////////

#include <signal.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>

#include <sys/stat.h>

#include "../../bing/Objectness/stdafx.h"
#include "../../bing/Objectness/Objectness.h"
#include "../../bing/Objectness/ValStructVec.h"
#include "../../bing/Objectness/CmShow.h"

typedef enum {
  UNKNOWN = -1,
  FAILURE = 0,
  SUCCESS = 1
} operationStatusType;
std::string operationStatusToString(operationStatusType mode) {
    string result;
  if (mode == UNKNOWN) {
    result = "Unknown";
  } else if (mode == FAILURE) {
    result = "Failure";
  } else if (mode == SUCCESS) {
    result = "Success";
  } else {
    cout << "Invalid operation status: " << mode << endl;
    assert(0);
  }
  return result;
}


typedef enum {
  SIFTBOW_GLOBALCOLOR_HIST = 1,
  OPPONENTSIFTBOW_GLOBALCOLOR_HIST = 2, // this has not been sufficiently tested
  SIFTCOLORBOW_HIST = 3, // unimplemented, calculate color histograms at each keypoint and augment each SIFT feature before clustering
  GRADIENT = 4,
  OPPONENT_COLOR_GRADIENT = 5
} featureType;
//featureType chosen_feature = SIFTBOW_GLOBALCOLOR_HIST;
//featureType chosen_feature = GRADIENT;
featureType chosen_feature = OPPONENT_COLOR_GRADIENT;
int cfi = 1;

int gradientFeatureWidth = 50;

typedef enum {
  UNIFORM_PRIOR,
  ANALYTIC_PRIOR
} priorType;

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
double rapidAmp2Delta = 0.03;

const int numJoints = 7;
int driveVelocities = 0;
int testJoint = 3;

int jointNamesInit = 0;
std::vector<std::string> jointNames;
rk_state random_state;

double spiralEta = 1.25;

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

double aveTimeRange = 0.0;
double aveFrequencyRange = 0.0;
double timeMassRange = 0.0;
double timeIntervalRange = 30;
time_t thisTimeRange = 0;
time_t firstTimeRange = 0;

// this should be initted to 0 and set to its default setting only after an imageCallback has happened.
int shouldIRenderDefault = 1;
int shouldIRender = 0;
int shouldIDoIK = 1;
int renderInit = 0;
int shouldIImageCallback = 1;
int shouldIMiscCallback = 1;
int shouldIRangeCallback = 1;

int ik_reset_thresh = 20;
int ik_reset_counter = 0;
int ikInitialized = 0.0;
// ATTN 14
double ikShare = 1.0;
double oscillating_ikShare = .1;
double default_ikShare = 1.0;

Word * current_instruction = NULL;
double tap_factor = 0.1;
int execute_stack = 0;

std::vector<Word *> call_stack;
void clearStack();
Word * popWord();
bool pushWord(int code);
bool pushWord(string name);
bool pushWord(Word * word);
Word * nameToWord(string name);

std::vector<Word *> words;
std::map<int, Word *> character_code_to_word;
std::map<string, Word *> name_to_word;

int slf_thresh = 5;
int successive_lock_frames = 0;

int lock_reset_thresh = 1800;
int lock_status = 0; // TODO enum

double slow_aim_factor = 0.75;
int aim_thresh = 20;
int lock_thresh = 5;

int timerCounter = 0;
int timerThresh = 16;

int timesTimerCounted = 0;

double prevPx = 0;
double prevPy = 0;
double Kd = 0.002;
double Kp = 0.0004;
double cCutoff = 20.0;

double a_thresh_close = .11;
double a_thresh_far = .2; // for depth scanning
double eeRange = 0.0;

double bDelta = MOVE_FAST;
double approachStep = .0005;
double hoverMultiplier = 0.5;

int zero_g_toggle = 0;

tf::TransformListener* tfListener;
double tfPast = 10.0;

std::string wristViewName = "Wrist View";
std::string coreViewName = "Core View";
std::string rangeogramViewName = "Rangeogram View";
std::string rangemapViewName = "Range Map View";
std::string hiRangemapViewName = "Hi Range Map View";
std::string hiColorRangemapViewName = "Hi Color Range Map View";
std::string graspMemoryViewName = "Grasp Memory View";
std::string graspMemorySampleViewName = "Grasp Memory Sample View";


std::string heightMemorySampleViewName = "Height Memory Sample View";

int reticleHalfWidth = 18;
int pilotTargetHalfWidth = 15;

eePose centerReticle = {.px = 325, .py = 127, .pz = 0.0,
		   .ox = 0.0, .oy = 0.0, .oz = 0.0,
		   .qx = 0.0, .qy = 0.0, .qz = 0.0, .qw = 0.0};

eePose defaultRightReticle = {.px = 325, .py = 127, .pz = 0.0,
		   .ox = 0.0, .oy = 0.0, .oz = 0.0,
		   .qx = 0.0, .qy = 0.0, .qz = 0.0, .qw = 0.0};
//eePose defaultRightReticle = {.px = 321, .py = 154, .pz = 0.0,
//		   .ox = 0.0, .oy = 0.0, .oz = 0.0,
//		   .qx = 0.0, .qy = 0.0, .qz = 0.0, .qw = 0.0};
eePose defaultLeftReticle = {.px = 334, .py = 100, .pz = 0.0,
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
eePose crane4right = {.px = 0.642291, .py = -0.659793, .pz = 0.144186,
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


// whole foods waypoints 
eePose wholeFoodsBagR = {.px = 0.618641, .py = -0.502567, .pz = 0.054811,
			 .ox = 0, .oy = 0, .oz = 0,
			 .qx = 0.0, .qy = 1.0, .qz = 0.0, .qw = 0.0}; // straight down 
eePose wholeFoodsPantryR = {.px = 0.616353, .py = -0.182223, .pz = 0.0528802,
                      .ox = 0, .oy = 0, .oz = 0,
		      .qx = 0.0, .qy = 1.0, .qz = 0.0, .qw = 0.0}; // straight down 
eePose wholeFoodsCounterR = {.px = -0.114591, .py = -0.771545, .pz = 0.0365494,
                      .ox = 0, .oy = 0, .oz = 0,
		      .qx = 0.0, .qy = 1.0, .qz = 0.0, .qw = 0.0}; // straight down 




eePose wholeFoodsBagL = {.px = 0.64828, .py = 0.762787, .pz = 0.0592764,
                      .ox = 0, .oy = 0, .oz = 0,
		      .qx = 0.0, .qy = 1.0, .qz = 0.0, .qw = 0.0}; // straight down 
eePose wholeFoodsPantryL = {.px = 0.645494, .py = 0.4937, .pz = 0.0568737,
                      .ox = 0, .oy = 0, .oz = 0,
		      .qx = 0.0, .qy = 1.0, .qz = 0.0, .qw = 0.0}; // straight down 
eePose wholeFoodsCounterL = {.px = -0.114389, .py = 0.803518, .pz = 0.056705,
                      .ox = 0, .oy = 0, .oz = 0,
		      .qx = 0.0, .qy = 1.0, .qz = 0.0, .qw = 0.0}; // straight down 

eePose rssPoseL = {.px = 0.334217, .py = 0.75386, .pz = 0.0362593,
                  .ox = 0, .oy = 0, .oz = 0,
                  .qx = -0.00125253, .qy = 0.999999, .qz = -0.000146851, .qw = 0.000236656};

eePose rssPoseR = {.px = 0.525866, .py = -0.710611, .pz = 0.0695764,
		      .ox = 0, .oy = 0, .oz = 0,
		      .qx = -0.00122177, .qy = 0.999998, .qz = 0.00116169, .qw = -0.001101};


eePose rssPose = rssPoseR;

eePose wholeFoodsBag1 = wholeFoodsBagR;
eePose wholeFoodsPantry1 = wholeFoodsPantryR;
eePose wholeFoodsCounter1 = wholeFoodsCounterR;

eePose defaultReticle = centerReticle;

eePose heightReticles[4];

eePose probeReticle = defaultReticle;
eePose vanishingPointReticle = defaultReticle;

eePose reticle = defaultReticle;
eePose beeHome = beeRHome;
eePose pilotTarget = beeHome;
eePose pilotClosestTarget = beeHome;
eePose lastGoodEEPose = beeHome;
eePose currentEEPose = beeHome;
eePose eepReg1 = workCenter;
eePose eepReg2 = beeHome;
eePose eepReg3 = beeHome;
eePose eepReg4 = beeHome;
eePose eepReg5 = beeHome;
eePose eepReg6 = beeHome;

std::vector<eePose> deliveryPoses;
int currentDeliveryPose = 0;

int pilotTargetBlueBoxNumber = -1;
int pilotClosestBlueBoxNumber = -1;

string left_or_right_arm = "right";

eePose ik_reset_eePose = beeHome;

Vector3d eeForward;
geometry_msgs::Pose trueEEPose;
eePose trueEEPoseEEPose;
std::string fetchCommand;
ros::Time fetchCommandTime;
double fetchCommandCooldown = 5;
int acceptingFetchCommands = 0;

std::string forthCommand;

int bfc = 0;
int bfc_period = 3;
int resend_times = 1;

baxter_core_msgs::SolvePositionIK ikRequest;

ros::ServiceClient ikClient;
ros::Publisher joint_mover;
ros::Publisher gripperPub;
ros::Publisher facePub;
ros::Publisher moveSpeedPub;
ros::Publisher sonarPub;
ros::Publisher headPub;
ros::Publisher nodPub;

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
Mat graspMemoryImage;
Mat graspMemorySampleImage;
Mat heightMemorySampleImage;

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
double rangeMapReg3[rmWidth*rmWidth];
double rangeMapReg4[rmWidth*rmWidth];

const int hrmWidth = 211; // must be odd
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

const int parzenKernelHalfWidth = 15;
const int parzenKernelWidth = 2*parzenKernelHalfWidth+1;
double parzenKernel[parzenKernelWidth*parzenKernelWidth];
double parzenKernelSigma = 4.0;
//double parzenKernelSigma = 2.0;
//double parzenKernelSigma = 1.0; // this is approximately what it should be at 20 cm height
//double parzenKernelSigma = 0.5;  
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
int rmiHeight = rmiCellWidth*rmWidth;
int rmiWidth = rmiCellWidth*rmWidth;

int hrmiHeight = hrmWidth;
int hrmiWidth = hrmWidth;

const int hmWidth = 4; 
int hmiCellWidth = 100;
int hmiWidth = hmiCellWidth;
int hmiHeight = hmiCellWidth*hmWidth;

// the currently equipped depth reticle
double drX = .02; //.01;
double drY = .02;

// target reticle
double trX = 0;
double trY = 0;
double trZ = 0;

double localTrX = 0;
double localTrY = 0;

double htrX = 0;
double htrY = 0;

int maxX = 0;
int maxY = 0;
double maxD = 0;
int maxGG = 0;
int localMaxX = 0;
int localMaxY = 0;
int localMaxGG = 0;

// grasp gear should always be even
const int totalGraspGears = 8;
// XXX maybe we should initialize this to a reasonable value
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
// most recent, smoothed:
//const int xCR[numCReticleIndeces] = {462, 450, 439, 428, 419, 410, 405, 399, 394, 389, 383, 381, 379, 378};
//const int yCR[numCReticleIndeces] = {153, 153, 153, 153, 153, 154, 154, 154, 154, 154, 155, 155, 155, 155};

int xCR[numCReticleIndeces];
int yCR[numCReticleIndeces];

ros::Publisher vmMarkerPublisher;

int getColorReticleX();
int getColorReticleY();


eePose rosPoseToEEPose(geometry_msgs::Pose pose) {
  eePose result;
  result.px = pose.position.x;
  result.py = pose.position.y;
  result.pz = pose.position.z;
  result.qx = pose.orientation.x;
  result.qy = pose.orientation.y;
  result.qz = pose.orientation.z;
  result.qw = pose.orientation.w;
  result.ox = 0.0;
  result.oy = 0.0;
  result.oz = 0.0;
  return result;
}

double fEpsilon = EPSILON;

int curseReticleX = 0;
int curseReticleY = 0;

int targetClass = -1;

ros::Time lastVisionCycle;
ros::Duration accumulatedTime;




double w1GoThresh = 0.03;//0.01;
double w1AngleThresh = 0.02; 
double synKp = 0.0005;
double gradKp = 0.00025;//0.0005;
double kPtheta1 = 1.0;//0.75;
double kPtheta2 = 0.125;//0.75;
int kPThresh = 3;
double lastPtheta = INFINITY;

// ATTN 4
// ATTN 22
// pre-absolute
//int synServoPixelThresh = 10;//15;//10;
//int gradServoPixelThresh = 2;
//int gradServoThetaThresh = 1;
int synServoPixelThresh = 15;//15;//10;
int gradServoPixelThresh = 5;
int gradServoThetaThresh = 2;

int synServoLockFrames = 0;
int synServoLockThresh = 20;

double synServoMinKp = synKp/20.0;
double synServoKDecay = .95;


ros::Time oscilStart;
double oscCenX = 0.0;
double oscCenY = 0.0;
double oscCenZ = 0.0;
double oscAmpX = 0.10;//.0.16;//0.08;//0.1;
double oscAmpY = 0.10;//0.16;//0.2;
double oscAmpZ = 0.0;

 const double commonFreq = 1.0;//1.0/2.0;
 double oscFreqX = commonFreq*1.0/3.0;
 double oscFreqY = commonFreq*1.0/20.0;
 double oscFreqZ = commonFreq*1.0;

double visionCycleInterval = 7.5 / 7.0 * (1.0/commonFreq);

// class focused for learning
int focusedClass = -1;
int newClassCounter = 0;
string focusedClassLabel;

// variables for survey during servoing
vector<double> surveyHistogram;
int surveyWinningClass = -1;
int viewsWithNoise = 0;
int publishObjects = 1;


int histogramDuringClassification = 0;
double surveyNoiseScale = 50;
int synchronicTakeClosest = 0;
int gradientTakeClosest = 0;
int gradientServoDuringHeightLearning = 1;
int bailAfterSynchronic = 1;
int bailAfterGradient = 0;

int gripperMoving = 0;
double gripperPosition = 0;
int gripperGripping = 0;
double gripperThresh = 3.5;//6.0;//7.0;
ros::Time gripperLastUpdated;
double gripperNotMovingConfirmTime = 0.25;

double graspAttemptCounter = 0;
double graspSuccessCounter = 0;
double graspSuccessRate = 0;
operationStatusType thisGraspPicked = UNKNOWN;
operationStatusType thisGraspReleased = UNKNOWN;

int thisGraspSucceed = -1;

ros::Time graspTrialStart;

double rightTableZ = 0.18;
double leftTableZ = 0.177;

double bagTableZ = 0.18; //0.195;//0.22;
double counterTableZ = 0.18;//0.209123; //0.20;//0.18;
double pantryTableZ = 0.18;//0.209123; //0.195;

double currentTableZ = leftTableZ;

double mostRecentUntabledZ = 0.0;
eePose bestOrientationEEPose = crane2right;
double bestOrientationAngle = 0;

Mat lastAerialGradient;
Mat lastRangeMap;
string lastLabelLearned;

double perturbScale = 0.05;//0.1;
double bbLearnPerturbScale = 0.07;//0.1;//.05;//
double bbLearnPerturbBias = 0.01;  //0.04;//0.05;
// ATTN 17
double bbLearnThresh = 0.05;//0.04;
double bbQuatThresh = 1000;//0.05;

// grasp Thompson parameters
double graspMemoryTries[4*rmWidth*rmWidth];
double graspMemoryPicks[4*rmWidth*rmWidth];
double graspMemorySample[4*rmWidth*rmWidth];
double graspMemoryReg1[4*rmWidth*rmWidth];

// height Thompson parameters
const double minHeight = -0.10;
const double maxHeight = 0.3;
double heightMemoryTries[hmWidth];
double heightMemoryPicks[hmWidth];
double heightMemorySample[hmWidth];

double heightAttemptCounter = 0;
double heightSuccessCounter = 0;
double thompsonTries = 50;

int gmTargetX = -1;
int gmTargetY = -1;

// the last value the gripper was at when it began to open from a closed position
double lastMeasuredBias = 1;
double lastMeasuredClosed = 3.0;

typedef enum {
  STATIC_PRIOR = 1,
  LEARNING_SAMPLING = 2,
  LEARNING_ALGORITHMC = 3,
  STATIC_MARGINALS = 4,
  MAPPING = 5
} pickMode;
pickMode currentPickMode = STATIC_MARGINALS;
pickMode currentBoundingBoxMode = STATIC_MARGINALS;
pickMode currentDepthMode = STATIC_MARGINALS;

std::string pickModeToString(pickMode mode) {
  string result;
  if (mode == STATIC_PRIOR) {
    result = "static prior";
  } else if (mode == LEARNING_SAMPLING) {
    result = "learning sampling";
  } else if (mode == LEARNING_ALGORITHMC) {
    result = "learning algorithm C";
  } else if (mode == STATIC_MARGINALS) {
    result = "static marginals";
  } else if (mode == MAPPING) {
    result = "mapping";
  } else {
    cout << "Invalid pick mode: " << mode << endl;
    assert(0);
  }
  return result;
}

int ARE_GENERIC_PICK_LEARNING() {
  return ( (currentPickMode == LEARNING_SAMPLING) ||
	   (currentPickMode == LEARNING_ALGORITHMC) );
}

int ARE_GENERIC_HEIGHT_LEARNING() {
  return ( (currentBoundingBoxMode == LEARNING_SAMPLING) ||
	   (currentBoundingBoxMode == LEARNING_ALGORITHMC) );
}

int orientationCascade = 0;
int lPTthresh = 3;
int orientationCascadeHalfWidth = 2;

int heightLearningServoTimeout = 10;
double currentThompsonHeight = 0;
int currentThompsonHeightIdx = 0;

int useGradientServoThresh = 0;
double gradientServoResetThresh = 0.7/(6.0e5);

vector<double> classL2RangeComparator;
double irHeightPadding = 0.16;

vector<double> rgbServoSample;
vector<double> rgbServoMarginals;
vector<double> rgbServoTries;
vector<double> rgbServoPicks;
int rgbServoTrials;
int rgbServoTrialsMax = 20;

int useTemporalAerialGradient = 1;
double aerialGradientDecayIteratedDensity = 0.9;
double aerialGradientDecayImageAverage = 0.0;
double aerialGradientDecay = 0.9;//0.965;//0.9;
Mat aerialGradientTemporalFrameAverage;
Mat aerialGradientSobelCbCr;
Mat aerialGradientSobelGray;
Mat preFrameGraySobel;
int densityIterationsForGradientServo = 10;//3;//10;

double graspDepth = -.07;//-.09;
double lastPickHeight = 0;
double lastPrePickHeight = 0;
double pickFlushFactor = 0.11;

int bbLearningMaxTries = 15;
int graspLearningMaxTries = 10;

int thompsonHardCutoff = 0;
int thompsonMinTryCutoff = 5;
double thompsonMinPassRate = 0.80;
int thompsonAdaptiveCutoff = 1;
int thompsonPickHaltFlag = 0;
int thompsonHeightHaltFlag = 0;

int useContinuousGraspTransform = 1;

double pickEccentricity = 100.0;
double heightEccentricity = 1.0;

// algorithmC accecpt and reject thresholds
double algorithmCEPS = 0.2;
double algorithmCTarget = 0.7;
double algorithmCAT = 0.7;
double algorithmCRT = 0.95;

int paintEEandReg1OnWrist = 1;

// d values obtained by putting laser in gripper
//  to find end effector projection, then using
//  a tape dot to find the vanishing point of
//  the camera
// the estimated vanishing point is actually pretty
//  close to the measured one
double d_y = -0.04;
double d_x = 0.018;
double offX = 0;
double offY = 0;
// these corrective magnification factors should be close to 1
//  these are set elsewhere according to chirality
double m_x = 1.08;
double m_y = 0.94;
double m_x_h[4];
double m_y_h[4];

int mappingServoTimeout = 5;

const int mappingHeightIdx = 1;

const int vaNumAngles = 360;
const double vaDelta = (2.0 * 3.1415926) / vaNumAngles;
double vaX[vaNumAngles];
double vaY[vaNumAngles];

int waitUntilAtCurrentPositionCounter = 0;
int waitUntilAtCurrentPositionCounterTimeout = 300;
int waitUntilGripperNotMovingCounter = 0;
int waitUntilGripperNotMovingTimeout = 100;
ros::Time waitUntilGripperNotMovingStamp;

double currentEESpeedRatio = 0.5;

int endCollapse = 0;
int endThisStackCollapse = 0;

baxter_core_msgs::HeadPanCommand currentHeadPanCommand;
std_msgs::Bool currentHeadNodCommand;
std_msgs::UInt16 currentSonarCommand;

int heartBeatCounter = 0;
int heartBeatPeriod = 150;

ros::Time lastImageCallbackRequest;
ros::Time lastGripperCallbackRequest;
ros::Time lastRangeCallbackRequest;
ros::Time lastFullMiscCallbackRequest;
ros::Time lastJointCallbackRequest;

ros::Time lastImageCallbackReceived;
ros::Time lastGripperCallbackReceived;
ros::Time lastRangeCallbackReceived;
ros::Time lastFullMiscCallbackReceived;
ros::Time lastJointCallbackReceived;

bool usePotentiallyCollidingIK = 0;

Mat objectViewerYCbCrBlur;
Mat objectViewerGrayBlur;

ros::Time lastHoverRequest;
double hoverTimeout = 2.0; // seconds
double hoverGoThresh = 0.02;
double hoverAngleThresh = 0.02;
eePose lastHoverTrueEEPoseEEPose;

////////////////////////////////////////////////
// end pilot variables 
//
// start node variables 
////////////////////////////////////////////////

int retrain_vocab = 0;
int rewrite_labels = 0;
int reextract_knn = 0;
int runInference = 1;
int trainOnly = 0;
// Start Intron: The below variables are left over and should be considered defunct
int saveAnnotatedBoxes = 0;
int captureHardClass = 0;
int captureOnly = 0;
int saveBoxes = 0;
// End Intron 1: The above variables are left over and should be considered defunct

int runTracking = 1; // calculates the red boxes

int drawOrientor = 1;
int drawLabels = 1;
int drawPurple = 0;
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

cv_bridge::CvImagePtr cv_ptr = NULL;
Mat objectViewerImage;
Mat objectMapViewerImage;
Mat densityViewerImage;
Mat wristViewImage;
Mat gradientViewerImage;
Mat objectnessViewerImage;
Mat aerialGradientViewerImage;

int mask_gripper = 1;

int add_blinders = 0;
int blinder_stride = 10;
int blinder_columns = 5;

boost::mutex ros_mutex;
boost::mutex pcl_mutex;
boost::mutex redbox_mutex;
boost::thread *timercallback1_thread;

std::string densityViewerName = "Density Viewer";
std::string objectViewerName = "Object Viewer";
std::string objectMapViewerName = "Object Map View";
std::string gradientViewerName = "Gradient Viewer";
std::string objectnessViewerName = "Objectness Viewer";
std::string aerialGradientViewerName = "Aerial Gradient Viewer";

int loTrackbarVariable = 30;//45;//75;
int hiTrackbarVariable = 35;//40;//50;
int redTrackbarVariable = 0;
int postDensitySigmaTrackbarVariable = 10.0;

double drawBingProb = .1;

double canny_hi_thresh = 5e5;//7;
double canny_lo_thresh = 5e5;//4;

double sobel_sigma = 2.0;//4.0;
double sobel_sigma_substitute_latest = 4.0;
double sobel_sigma_substitute_accumulated = 4.0;//2.0; reflections are a problem for low sigma...
double sobel_scale_factor = 1e-12;
double local_sobel_sigma = 1.0;

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
const double bowSubSampleFactor = 0.05;//0.02;//0.01;
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
const double colorHistLambda = 1.0;//0.5;
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

DescriptorMatcher *matcher = NULL;
FeatureDetector *detector = NULL;
DescriptorExtractor *extractor = NULL;
BOWKMeansTrainer *bowtrainer = NULL; 
BOWImgDescriptorExtractor *bowExtractor = NULL;
CvKNearest *kNN = NULL;
std::string package_path;
std::string class_crops_path;
std::string bing_trained_models_path;
std::string objectness_path_prefix;
std::string saved_crops_path;

cv::Mat cam_img;
cv::Mat depth_img;
ros::Publisher rec_objs_blue;
ros::Publisher rec_objs_blue_memory;
ros::Publisher rec_objs_red;
ros::Publisher markers_blue;
ros::Publisher markers_blue_memory;
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
double *preDensity = NULL;
double *differentialDensity = NULL;
double *integralDensity = NULL;
double *temporalDensity = NULL;
double *temporalDepth = NULL;

double *objDensity = NULL;
double *preObjDensity = NULL;
double *differentialObjDensity = NULL;
double *integralObjDensity = NULL;
double *temporalObjDensity = NULL;

double densityDecay = 0.5;//0.9;//0.3;//0.7;
double objDensityDecay = 0.9;
double threshFraction = 0.2;
double objectnessThreshFraction = 0.5;

pcl::PointCloud<pcl::PointXYZRGB> pointCloud;

int biggestL1 = 0;
int oSearchWidth = 5;

Mat *orientedFiltersT;
Mat *orientedFiltersS;
Mat *orientedFiltersK;

// Top variables are top left corners of bounding boxes (smallest coordinates)
// Bot variables are bottom right corners of bounding boxes (largest coordinates)
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

struct BoxMemory {
  cv::Point bTop;
  cv::Point bBot;
  eePose cameraPose;
  eePose aimedPose;
  eePose pickedPose;
  eePose top;
  eePose bot;
  eePose centroid;
  ros::Time cameraTime;
  int labeledClassIndex;
  int lockStatus;
};

typedef struct MapCell {
  ros::Time lastMappedTime;
  int detectedClass; // -1 means not denied
  double r, g, b;
  double pixelCount;
} MapCell;

void mapijToxy(int i, int j, double * x, double * y);
void mapxyToij(double x, double y, int * i, int * j); 
void initializeMap() ;
void randomizeNanos(ros::Time * time);
int blueBoxForPixel(int px, int py);
int skirtedBlueBoxForPixel(int px, int py, int skirtPixels);
bool cellIsSearched(int i, int j);
bool positionIsSearched(double x, double y);
vector<BoxMemory> memoriesForClass(int classIdx);
vector<BoxMemory> memoriesForClass(int classIdx, int * memoryIdxOfFirst);

// XXX TODO searched and mapped are redundant. just need one to talk about the fence.
bool cellIsMapped(int i, int j);
bool positionIsMapped(double x, double y);
bool boxMemoryIntersectPolygons(BoxMemory b1, BoxMemory b2);
bool boxMemoryIntersectCentroid(BoxMemory b1, BoxMemory b2);
bool boxMemoryContains(BoxMemory b, double x, double y);
bool boxMemoryIntersectsMapCell(BoxMemory b, int map_i, int map_j);

// XXX TODO these just check the corners, they should check all the interior points instead
bool isBoxMemoryIKPossible(BoxMemory b);
bool isBlueBoxIKPossible(cv::Point tbTop, cv::Point tbBot);

bool isCellInPursuitZone(int i, int j);
bool isCellInPatrolZone(int i, int j);

bool isCellInteresting(int i, int j);
void markCellAsInteresting(int i, int j);
void markCellAsNotInteresting(int i, int j);

bool isCellIkColliding(int i, int j);
bool isCellIkPossible(int i, int j);
bool isCellIkImpossible(int i, int j);


const double mapXMin = -1.5;
const double mapXMax = 1.5;
const double mapYMin = -1.5;
const double mapYMax = 1.5;
const double mapStep = 0.01;

double mapSearchFenceXMin;
double mapSearchFenceXMax;
double mapSearchFenceYMin;
double mapSearchFenceYMax;

double mapRejectFenceXMin;
double mapRejectFenceXMax;
double mapRejectFenceYMin;
double mapRejectFenceYMax;

const int mapWidth = (mapXMax - mapXMin) / mapStep;
const int mapHeight = (mapYMax - mapYMin) / mapStep;
const ros::Duration mapMemoryTimeout(10);

MapCell objectMap[mapWidth * mapHeight];
ros::Time lastScanStarted;
int mapFreeSpacePixelSkirt = 25;
int mapBlueBoxPixelSkirt = 50;
double mapBlueBoxCooldown = 20; // cooldown is a temporal skirt
int mapGrayBoxPixelSkirt = 50;
int ikMap[mapWidth * mapHeight];
int clearanceMap[mapWidth * mapHeight];
int drawClearanceMap = 1;
int drawIKMap = 1;
int useGlow = 0;
int useFade = 1;

vector<BoxMemory> blueBoxMemories;


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

int gBoxStrideX;
int gBoxStrideY;

// pink box thresholds for the principle classes
double *pBoxIndicator = NULL;
double psPBT = 0.0;//5.0;
double wsPBT = 0.0;//6.5;
double gbPBT = 0.0;//6.0;
double mbPBT = 0.0;//7.0;

double pBoxThresh = 0;

// gray box offset from the top and bottom of the screen
int tGO = 30;
int bGO = 30;
int lGO = 60;
int rGO = 60;
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

int loadRange = 1;
vector<Mat> classRangeMaps;

// ATTN 16
vector<Mat> classAerialGradients;
vector<Mat> classHeight0AerialGradients;
vector<Mat> classHeight1AerialGradients;
vector<Mat> classHeight2AerialGradients;
vector<Mat> classHeight3AerialGradients;

vector<Mat> classGraspMemoryTries1;
vector<Mat> classGraspMemoryPicks1;
vector<Mat> classGraspMemoryTries2;
vector<Mat> classGraspMemoryPicks2;
vector<Mat> classGraspMemoryTries3;
vector<Mat> classGraspMemoryPicks3;
vector<Mat> classGraspMemoryTries4;
vector<Mat> classGraspMemoryPicks4;

vector<Mat> classHeightMemoryTries;
vector<Mat> classHeightMemoryPicks;


Mat frameGraySobel;
// XXX this should probably be odd
int aerialGradientWidth = 100;
int aerialGradientReticleWidth = 200;

// XXX TODO
int softMaxGradientServoIterations = 2;//5;//3;//10;//3;
int hardMaxGradientServoIterations = 2;//5;//5;//3;//10;//20;//3;//10;
int currentGradientServoIterations = 0;

int fuseBlueBoxes = 1;
int fusePasses = 5;

int g1xs = 200;
int g1xe = 295;
int g1ys = 0;
int g1ye = 75;

int g2xs = 420;
int g2xe = 560;
int g2ys = 0;
int g2ye = 75;

Mat accumulatedImage;
Mat accumulatedImageMass;

////////////////////////////////////////////////
// end node variables 
//
// start pilot prototypes 
////////////////////////////////////////////////

int getRingImageAtTime(ros::Time t, Mat& value, int drawSlack = 0);
int getRingRangeAtTime(ros::Time t, double &value, int drawSlack = 0);
int getRingPoseAtTime(ros::Time t, geometry_msgs::Pose &value, int drawSlack = 0);
extern "C" {
double cephes_incbet(double a, double b, double x) ;
}
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
void gripStateCallback(const baxter_core_msgs::EndEffectorState& ees);
void fetchCommandCallback(const std_msgs::String::ConstPtr& msg);
void forthCommandCallback(const std_msgs::String::ConstPtr& msg);

bool isGripperGripping();
void initialize3DParzen();
void l2Normalize3DParzen();
void initializeParzen();
void l2NormalizeParzen();
void l2NormalizeFilter();
double squareDistanceEEPose(eePose pose1, eePose pose2);

int getColorReticleX();
int getColorReticleY();
cv::Vec3b getCRColor();
cv::Vec3b getCRColor(Mat im);
Quaternionf extractQuatFromPose(geometry_msgs::Pose poseIn);

void pushNoOps(int n);
void pushSpeedSign(double speed);

void scanXdirection(double speedOnLines, double speedBetweenLines);
void scanYdirection(double speedOnLines, double speedBetweenLines);

Eigen::Quaternionf getGGRotation(int givenGraspGear);
void setGGRotation(int thisGraspGear);

Eigen::Quaternionf getCCRotation(int givenGraspGear, double angle);
void setCCRotation(int thisGraspGear);

void rangeCallback(const sensor_msgs::Range& range);
void endEffectorAngularUpdate(eePose *givenEEPose);
void fillIkRequest(eePose givenEEPose, baxter_core_msgs::SolvePositionIK * givenIkRequest);
void update_baxter(ros::NodeHandle &n);
void timercallback1(const ros::TimerEvent&);
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void renderCoreView();
void renderObjectMapView();
void drawMapPolygon(gsl_matrix * poly, cv::Scalar color);
void targetCallback(const geometry_msgs::Point& point);
void pilotCallbackFunc(int event, int x, int y, int flags, void* userdata);
void graspMemoryCallbackFunc(int event, int x, int y, int flags, void* userdata);
gsl_matrix * boxMemoryToPolygon(BoxMemory b);
gsl_matrix * mapCellToPolygon(int map_i, int map_j) ;

void pilotInit();
void spinlessPilotMain();

int doCalibrateGripper();
int calibrateGripper();
int shouldIPick(int classToPick);
int getLocalGraspGear(int globalGraspGearIn);
int getGlobalGraspGear(int localGraspGearIn);
void convertGlobalGraspIdxToLocal(const int rx, const int ry, 
                                  int * localX, int * localY);

void convertLocalGraspIdxToGlobal(const int localX, const int localY,
                                  int * rx, int * ry);

void changeTargetClass(int);

void guardGraspMemory();
void loadSampledGraspMemory();
void loadMarginalGraspMemory();
void loadPriorGraspMemory(priorType);
void estimateGlobalGraspGear();
void drawMapRegisters();

void guardHeightMemory();
void loadSampledHeightMemory();
void loadMarginalHeightMemory();
void loadPriorHeightMemory(priorType);
double convertHeightIdxToGlobalZ(int);
int convertHeightGlobalZToIdx(double);
void testHeightConversion();
void drawHeightMemorySample();
void copyHeightMemoryTriesToClassHeightMemoryTries();

void applyGraspFilter(double * rangeMapRegA, double * rangeMapRegB);
void prepareGraspFilter(int i);
void prepareGraspFilter1();
void prepareGraspFilter2();
void prepareGraspFilter3();
void prepareGraspFilter4();

void copyRangeMapRegister(double * src, double * target);
void copyGraspMemoryRegister(double * src, double * target);
void loadGlobalTargetClassRangeMap(double * rangeMapRegA, double * rangeMapRegB);
void loadLocalTargetClassRangeMap(double * rangeMapRegA, double * rangeMapRegB);
void copyGraspMemoryTriesToClassGraspMemoryTries();
void copyClassGraspMemoryTriesToGraspMemoryTries();

void selectMaxTarget(double minDepth);
void selectMaxTargetThompson(double minDepth);
void selectMaxTargetThompsonContinuous(double minDepth);
void selectMaxTargetThompsonContinuous2(double minDepth);
void selectMaxTargetThompsonRotated(double minDepth);
void selectMaxTargetThompsonRotated2(double minDepth);
void selectMaxTargetLinearFilter(double minDepth);

void recordBoundingBoxSuccess();
void recordBoundingBoxFailure();

void restartBBLearning();

double unsignedQuaternionDistance(Quaternionf a, Quaternionf b);

eePose analyticServoPixelToReticle(eePose givenPixel, eePose givenReticle, double angle);
void moveCurrentGripperRayToCameraVanishingRay();
void gradientServo();
void synchronicServo();
int simulatedServo();

void initRangeMaps();

int isThisGraspMaxedOut(int i);

void pixelToGlobal(int pX, int pY, double gZ, double * gX, double * gY);
void pixelToGlobal(int pX, int pY, double gZ, double * gX, double * gY, eePose givenEEPose);
void globalToPixel(int * pX, int * pY, double gZ, double gX, double gY);
eePose pixelToGlobalEEPose(int pX, int pY, double gZ);

void paintEEPoseOnWrist(eePose toPaint, cv::Scalar theColor);

double vectorArcTan(double y, double x);
void initVectorArcTan();

void mapBlueBox(cv::Point tbTop, cv::Point tbBot, int detectedClass, ros::Time timeToMark);
void mapBox(BoxMemory boxMemory);

////////////////////////////////////////////////
// end pilot prototypes 
//
// start node prototypes
////////////////////////////////////////////////

int doubleToByte(double in);

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

void goCalculateObjectness();
void goCalculateDensity();
void goFindBlueBoxes();
void goFindBrownBoxes();
void goClassifyBlueBoxes();
void goFindRedBoxes();

void resetAccumulatedImageAndMass();
void renderAccumulatedImageAndDensity();
void substituteAccumulatedImageQuantities();
void substituteLatestImageQuantities();

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

void loadROSParamsFromArgs();
void loadROSParams();
void saveROSParams();

void spinlessNodeMain();
void nodeInit();
void detectorsInit();
void initRedBoxes();

void tryToLoadRangeMap(std::string classDir, const char *className, int i);

void processSaliency(Mat in, Mat out);

void happy();
void sad();
void neutral();

bool isSketchyMat(Mat sketchy);
void guardViewers();
void guardedImshow(string name, Mat image);

////////////////////////////////////////////////
// end node prototypes 
//
// start pilot definitions 
////////////////////////////////////////////////

void happy() {
  std_msgs::Int32 msg;
  msg.data = 0;
  facePub.publish(msg);
}

void sad() {
  std_msgs::Int32 msg;
  msg.data = 99;
  facePub.publish(msg);
}

void neutral() {
  std_msgs::Int32 msg;
  msg.data = 50;
  facePub.publish(msg);
}


int getRingImageAtTime(ros::Time t, Mat& value, int drawSlack) {
  if (imRingBufferStart == imRingBufferEnd) {
    
    #ifdef DEBUG4
    cout << "Denied request in getRingImageAtTime(): Buffer empty." << endl;
    #endif
    return 0;
  } else {
    int earliestSlot = imRingBufferStart;
    ros::Duration deltaTdur = t - imRBTimes[earliestSlot];
    // if the request comes before our earliest record, deny
    if (deltaTdur.toSec() <= 0.0) {
    #ifdef DEBUG4
      cout << "Denied out of order range value in getRingImageAtTime(): Too small." << endl;
      cout << "  getRingImageAtTime() imRingBufferStart imRingBufferEnd t imRBTimes[earliestSlot]: " << 
	imRingBufferStart << " " << imRingBufferEnd << " " << t << " " << imRBTimes[earliestSlot] << endl;
    #endif
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
    #ifdef DEBUG4
      //cout << "Denied out of order range value in getRingImageAtTime(): Too large." << endl;
    #endif
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
    #ifdef DEBUG4
      //cout << "Denied out of order range value in getRingImageAtTime(): Too large." << endl;
    #endif
      return -2;
    }
  }
}
int getRingRangeAtTime(ros::Time t, double &value, int drawSlack) {
  if (rgRingBufferStart == rgRingBufferEnd) {
    #ifdef DEBUG4
    cout << "Denied request in getRingRangeAtTime(): Buffer empty." << endl;
    #endif
    return 0;
  } else {
    int earliestSlot = rgRingBufferStart;
    ros::Duration deltaTdur = t - rgRBTimes[earliestSlot];
    // if the request comes before our earliest record, deny
    if (deltaTdur.toSec() <= 0.0) {
    #ifdef DEBUG4
      cout << "Denied out of order range value in getRingRangeAtTime(): Too small." << endl;
    #endif
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
    #ifdef DEBUG4
      cout << "Denied out of order range value in getRingRangeAtTime(): Too large." << endl;
    #endif
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
    #ifdef DEBUG4
      cout << "Denied out of order range value in getRingRangeAtTime(): Too large." << endl;
    #endif
      return -2;
    }
  }
}
int getRingPoseAtTime(ros::Time t, geometry_msgs::Pose &value, int drawSlack) {
  if (epRingBufferStart == epRingBufferEnd) {
    #ifdef DEBUG4
    cout << "Denied request in getRingPoseAtTime(): Buffer empty." << endl;
    #endif
    return 0;
  } else {
    int earliestSlot = epRingBufferStart;
    ros::Duration deltaTdur = t - epRBTimes[earliestSlot];
    // if the request comes before our earliest record, deny
    if (deltaTdur.toSec() <= 0.0) {
    #ifdef DEBUG4
      cout << "Denied out of order range value in getRingPoseAtTime(): Too small." << endl;
    #endif
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
    #ifdef DEBUG4
//cout << value << endl;
//cout << "33333c " << epRingBuffer[s] << " " << w1 << " " << w2 << " " << totalWeight << endl;
//cout << "44444c " << epRingBuffer[s+1] << endl;
    #endif

	  int newStart = s;
	  if(drawSlack) {
	    epRingBufferStart = newStart;
	  }
	  return 1;
	}
      }
      // if we didn't find it we should return failure
    #ifdef DEBUG4
      //cout << "Denied out of order range value in getRingPoseAtTime(): Too large." << endl;
    #endif
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
    #ifdef DEBUG4
//cout << value << endl;
//cout << "33333b " << epRingBuffer[s] << " " << w1 << " " << w2 << " " << totalWeight << endl;
//cout << "44444b " << epRingBuffer[s+1] << endl;
    #endif

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
    #ifdef DEBUG4
//cout << value << endl;
//cout << "33333a " << epRingBuffer[epRingBufferSize-1] << " " << w1 << " " << w2 << " " << totalWeight << endl;
//cout << "44444a " << epRingBuffer[0] << endl;
    #endif

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
    #ifdef DEBUG4
//cout << value << endl;
//cout << "33333d " << epRingBuffer[s] << " " << w1 << " " << w2 << " " << totalWeight << endl;
//cout << "44444d " << epRingBuffer[s+1] << endl;
    #endif

	  int newStart = s;
	  if(drawSlack) {
	    epRingBufferStart = newStart;
	  }
	  return 1;
	}
      }
      // if we didn't find it we should return failure
    #ifdef DEBUG4
      //cout << "Denied out of order range value in getRingPoseAtTime(): Too large." << endl;
    #endif
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
      #ifdef DEBUG2 
      cout << "Dropped out of order range value in setRingImageAtTime(). " << imRBTimes[imRingBufferStart].toSec() << " " << t.toSec() << " " << deltaTdur.toSec() << " " << endl;
      #endif
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
      #ifdef DEBUG2 
      cout << "Dropped out of order range value in setRingRangeAtTime(). " << rgRBTimes[rgRingBufferStart].toSec() << " " << t.toSec() << " " << deltaTdur.toSec() << " " << endl;
      #endif
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
    #ifdef DEBUG4
//cout << epToSet << endl;
//cout << "11111 " << epRingBuffer[0] << endl;
    #endif
    epRBTimes[0] = t;
  } else {
    ros::Duration deltaTdur = t - epRBTimes[epRingBufferStart];
    if (deltaTdur.toSec() <= 0.0) {
      #ifdef DEBUG2 
      cout << "Dropped out of order range value in setRingPoseAtTime(). " << epRBTimes[epRingBufferStart].toSec() << " " << t.toSec() << " " << deltaTdur.toSec() << " " << endl;
      #endif
    } else {
      int slot = epRingBufferEnd;
      epRingBuffer[slot] = epToSet;
    #ifdef DEBUG4
//cout << epToSet << endl;
//cout << "22222" << epRingBuffer[slot] << endl;
    #endif
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

	if (thisRange >= RANGE_UPPER_INVALID) {
	  //cout << "DISCARDED large range reading." << endl;
	  IShouldContinue = 1;
	  rgRingBufferAdvance();
	  continue;
	}
	if (thisRange <= RANGE_LOWER_INVALID) {
	  //cout << "DISCARDED small range reading." << endl;
	  IShouldContinue = 1;
	  rgRingBufferAdvance();
	  continue;
	}

	// actually storing the negative z for backwards compatibility
	double thisZmeasurement = -(thisPose.position.z - thisRange);
	double dX = 0;
	double dY = 0;
	double dZ = 0;

	Eigen::Vector3d rayDirection;

	{
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
	  // ATTN 0 this is negative because it used to be range and not Z but we have to chase the min / max switches to correct it
	  thisZmeasurement = -irSensorEnd.z();
	  mostRecentUntabledZ = thisZmeasurement;
	  // ATTN 1 currently accounting for table models
	  thisZmeasurement = thisZmeasurement - currentTableZ;

	  rayDirection = Eigen::Vector3d(localUnitZ.x(), localUnitZ.y(), localUnitZ.z());
	}

	double iX = dX / rmDelta;
	double iY = dY / rmDelta;

	double hiX = dX / hrmDelta;
	double hiY = dY / hrmDelta;
	double hiZ = dZ / hrmDelta;

	if (recordRangeMap) {
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
		  // nonexperimental
		  //double denom = max(hiRangeMapMass[px + py*hrmWidth], EPSILON);
		  // XXX experimental
		  double denom = 1.0;
		  if (hiRangeMapMass[px + py*hrmWidth] > 0)
		    denom = hiRangeMapMass[px + py*hrmWidth];
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
	    
	    double minDepth = VERYBIGNUMBER;
	    double maxDepth = 0;
	    for (int rx = 0; rx < rmWidth; rx++) {
	      for (int ry = 0; ry < rmWidth; ry++) {
		minDepth = min(minDepth, rangeMap[rx + ry*rmWidth]);
		maxDepth = max(maxDepth, rangeMap[rx + ry*rmWidth]);
	      }
	    }
	    double denom2 = max(EPSILON,maxDepth-minDepth);
	    if (denom2 <= EPSILON)
	      denom2 = VERYBIGNUMBER;
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
	}
    
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

//  if (!shouldIMiscCallback) {
//    return;
//  }

  lastJointCallbackReceived = ros::Time::now();
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

void fetchCommandCallback(const std_msgs::String::ConstPtr& msg) {

  if (!shouldIMiscCallback) {
    return;
  }

  if ( (ros::Time::now() - fetchCommandTime < ros::Duration(fetchCommandCooldown)) ||
       (!acceptingFetchCommands) ) {
    return;
  }
  acceptingFetchCommands = 0;

  fetchCommand = msg->data;
  fetchCommandTime = ros::Time::now();
  ROS_INFO_STREAM("Received " << fetchCommand << endl);

  int class_idx = -1;

  for (int i = 0; i < classLabels.size(); i++) {
    class_idx = i;
    if (classLabels[i] == fetchCommand) {

      break;
    }
  }
  if (class_idx == -1) {
    cout << "Could not find class " << fetchCommand << endl; 
  } else {
    clearStack();

    changeTargetClass(class_idx);
    pushWord("mappingPatrol");
    pushWord("deliverObject");
    execute_stack = 1;
  }
}

vector<string> split(const char *str, char c = ' ')
{
    vector<string> result;
    do
    {
      const char *begin = str;
      
      while(*str != c && *str)
        str++;
      
      result.push_back(string(begin, str));
    } while (0 != *str++);
    
    return result;
}



void forthCommandCallback(const std_msgs::String::ConstPtr& msg) {

  // disabling this would be unwise

  forthCommand = msg->data;
  ROS_INFO_STREAM("Received " << forthCommand << endl);
  vector<string> tokens = split(forthCommand.c_str(), ' ');
  for (unsigned int i = 0; i < tokens.size(); i++) {
    trim(tokens[i]);
    if (tokens[i] == "executeStack" || tokens[i] == ";") {
      execute_stack = 1;
    } else {
      if (!pushWord(tokens[i])) {
        cout << "Warning, ignoring unknown word from the forth topic: " << tokens[i] << endl;
      }
    }
  }
}

void endpointCallback(const baxter_core_msgs::EndpointState& eps) {

//  if (!shouldIMiscCallback) {
//    return;
//  }

  //cout << "endpoint frame_id: " << eps.header.frame_id << endl;
  trueEEPose = eps.pose;
  trueEEPoseEEPose.px = eps.pose.position.x;
  trueEEPoseEEPose.py = eps.pose.position.y;
  trueEEPoseEEPose.pz = eps.pose.position.z;
  trueEEPoseEEPose.qx = eps.pose.orientation.x;
  trueEEPoseEEPose.qy = eps.pose.orientation.y;
  trueEEPoseEEPose.qz = eps.pose.orientation.z;
  trueEEPoseEEPose.qw = eps.pose.orientation.w;

  setRingPoseAtTime(eps.header.stamp, eps.pose);
  geometry_msgs::Pose thisPose;
  int weHavePoseData = getRingPoseAtTime(eps.header.stamp, thisPose);
}

void gripStateCallback(const baxter_core_msgs::EndEffectorState& ees) {

//  if (!shouldIMiscCallback) {
//    return;
//  }

  lastGripperCallbackReceived = ros::Time::now();

  //cout << "setting gripper position: " << gripperPosition << endl;
  gripperLastUpdated = ros::Time::now();
  gripperPosition  = ees.position;
  gripperMoving = ees.moving;
  gripperGripping = ees.gripping;
}

bool isGripperGripping() {
  //return (gripperPosition >= gripperThresh);
  return gripperGripping; 
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
    #ifdef DEBUG4
	cout << "Parzen3D: " << parzenKernel[kx + ky*parzenKernelWidth] << endl;
    #endif
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
    #ifdef DEBUG4
      cout << "Parzen: " << parzenKernel[kx + ky*parzenKernelWidth] << endl;
    #endif
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

// XXX TODO this should really use a buffered eeRange
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
    pushWord('C'); 
}

void pushCopies(int symbol, int times) {
  for (int i = 0; i < times; i++) {
    pushWord(symbol); 
  }
}
void pushCopies(string symbol, int times) {
  for (int i = 0; i < times; i++) {
    pushWord(symbol); 
  }
}

void pushSpeedSign(double speed) {

  if (speed == NOW_THATS_FAST)
    pushWord(1114193); // set speed to NOW_THATS_FAST
  if (speed == MOVE_EVEN_FASTER)
    pushWord(1114199); // set speed to MOVE_EVEN_FASTER 
  if (speed == MOVE_FASTER)
    pushWord(1114181); // set speed to MOVE_FASTER
  if (speed == MOVE_FAST)
    pushWord(1048674); // set speed to MOVE_FAST 
  if (speed == MOVE_MEDIUM)
    pushWord(1048686); // set speed to MOVE_MEDIUM
  if (speed == MOVE_SLOW)
    pushWord(1114190); // set speed to MOVE_SLOW
  if (speed == MOVE_VERY_SLOW)
    pushWord(1114178); // set speed to MOVE_VERY_SLOW

}

void scanXdirection(double speedOnLines, double speedBetweenLines) {
// XXX TODO work this out so that it scans from -rmHalfWidth*rmDelta to rmHalfWidth*rmDelta

  double onLineGain = rmDelta / speedOnLines;
  double betweenLineGain = rmDelta / speedBetweenLines;

  int scanPadding = int(floor(1 * onLineGain));

  pushWord("cruisingSpeed");
  pushWord("waitUntilAtCurrentPosition"); 
  for (int g = 0; g < ((rmWidth*onLineGain)-(rmHalfWidth*onLineGain))+scanPadding; g++) {
    pushWord('a');
  }
  for (int g = 0; g < rmHalfWidth*onLineGain+scanPadding; g++) {
    pushWord('e');
  }

  pushWord("waitUntilAtCurrentPosition"); 
  //int gLimit = 1+((rmWidth*betweenLineGain+2*scanPadding)/2);
  int gLimit = ((rmWidth*betweenLineGain+2*scanPadding));
  for (int g = 0; g < gLimit; g++) {
    pushWord("fullRender"); 
    pushWord("waitUntilAtCurrentPosition"); 
    pushWord('d');
    pushWord("waitUntilAtCurrentPosition");
    for (int gg = 0; gg < rmWidth*onLineGain+2*scanPadding; gg++) {
      pushWord('q');
    }
    pushWord("waitUntilAtCurrentPosition"); 
    for (int gg = 0; gg < rmWidth*onLineGain+2*scanPadding; gg++) {
      pushWord('e');
    }
  }

  pushWord("waitUntilAtCurrentPosition"); 
  for (int g = 0; g < rmHalfWidth*onLineGain+scanPadding; g++) {
    pushWord('q');
  }
  for (int g = 0; g < rmHalfWidth*onLineGain+scanPadding; g++) {
    pushWord('a');
  }
  pushWord("rasterScanningSpeed");
}

void oldScanXdirection(double speedOnLines, double speedBetweenLines) {

  double onLineGain = rmDelta / speedOnLines;
  double betweenLineGain = rmDelta / speedBetweenLines;

  int scanPadding = int(floor(1 * onLineGain));

  for (int g = 0; g < ((rmWidth*onLineGain)-(rmHalfWidth*onLineGain))+scanPadding; g++) {
    // ATTN 2
    //pushWord(1048677);
    pushWord("waitUntilAtCurrentPosition"); 
    pushWord('a');
  }
  for (int g = 0; g < rmHalfWidth*onLineGain+scanPadding; g++) {
    //pushWord(1048677);
    pushWord("waitUntilAtCurrentPosition"); 
    pushWord('e');
  }
  pushSpeedSign(speedOnLines);

  //int gLimit = 1+((rmWidth*betweenLineGain+2*scanPadding)/2);
  int gLimit = ((rmWidth*betweenLineGain+2*scanPadding));
  for (int g = 0; g < gLimit; g++) {
    pushWord("fullRender"); 
    //pushWord(1048677);
    pushWord("waitUntilAtCurrentPosition"); 
    pushSpeedSign(speedOnLines);
    pushWord('d');
    pushSpeedSign(speedBetweenLines);
    for (int gg = 0; gg < rmWidth*onLineGain+2*scanPadding; gg++) {
      //pushWord(1048677);
      pushWord("waitUntilAtCurrentPosition");
      pushWord('q');
    }
    //pushSpeedSign(speedOnLines);
    //pushWord('d');
    //pushSpeedSign(speedBetweenLines);
    for (int gg = 0; gg < rmWidth*onLineGain+2*scanPadding; gg++) {
      //pushWord(1048677);
      pushWord("waitUntilAtCurrentPosition"); 
      pushWord('e');
    }
  }
  for (int g = 0; g < rmHalfWidth*onLineGain+scanPadding; g++) {
    //pushWord(1048677);
    pushWord("waitUntilAtCurrentPosition"); 
    pushWord('q');
  }
  for (int g = 0; g < rmHalfWidth*onLineGain+scanPadding; g++) {
    //pushWord(1048677);
    pushWord("waitUntilAtCurrentPosition"); 
    pushWord('a');
  }
  pushSpeedSign(speedOnLines);

}

void scanYdirection(double speedOnLines, double speedBetweenLines) {

  double onLineGain = rmDelta / speedOnLines;
  double betweenLineGain = rmDelta / speedBetweenLines;

  int scanPadding = int(floor(1 * onLineGain));

  for (int g = 0; g < ((rmWidth*onLineGain)-(rmHalfWidth*onLineGain))+scanPadding; g++) {
    // ATTN 2
    //pushWord(1048677);
    pushWord("waitUntilAtCurrentPosition"); 
    pushWord('q');
  }
  for (int g = 0; g < rmHalfWidth*onLineGain+scanPadding; g++) {
    //pushWord(1048677);
    pushWord("waitUntilAtCurrentPosition"); 
    pushWord('d');
  }
  pushSpeedSign(speedOnLines);

  //int gLimit = 1+((rmWidth*betweenLineGain+2*scanPadding)/2);
  int gLimit = ((rmWidth*betweenLineGain+2*scanPadding));
  for (int g = 0; g < gLimit; g++) {
    pushWord("fullRender"); // full render
    //pushWord(1048677);
    pushWord("waitUntilAtCurrentPosition"); 
    pushSpeedSign(speedOnLines);
    pushWord('e');
    pushSpeedSign(speedBetweenLines);
    for (int gg = 0; gg < rmWidth*onLineGain+2*scanPadding; gg++) {
      //pushWord(1048677);
      pushWord("waitUntilAtCurrentPosition"); 
      pushWord('a');
    }
    //pushSpeedSign(speedOnLines);
    //pushWord('e');
    //pushSpeedSign(speedBetweenLines);
    for (int gg = 0; gg < rmWidth*onLineGain+2*scanPadding; gg++) {
      //pushWord(1048677);
      pushWord("waitUntilAtCurrentPosition"); 
      pushWord('d');
    }
  }

  for (int g = 0; g < rmHalfWidth*onLineGain+scanPadding; g++) {
    //pushWord(1048677);
    pushWord("waitUntilAtCurrentPosition"); 
    pushWord('q');
  }
  for (int g = 0; g < rmHalfWidth*onLineGain+scanPadding; g++) {
    //pushWord(1048677);
    pushWord("waitUntilAtCurrentPosition"); 
    pushWord('a');
  }
  pushSpeedSign(speedOnLines);
}

Eigen::Quaternionf getGGRotation(int givenGraspGear) {
  Eigen::Vector3f localUnitX;
  {
    Eigen::Quaternionf qin(0, 1, 0, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    //Eigen::Quaternionf eeqform(eepReg2.qw, eepReg2.qx, eepReg2.qy, eepReg2.qz);
    Eigen::Quaternionf eeqform(0, 0, 1.0, 0);
    //Eigen::Quaternionf eeqform(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
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
    //Eigen::Quaternionf eeqform(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
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
    //Eigen::Quaternionf eeqform(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
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

Eigen::Quaternionf getCCRotation(int givenGraspGear, double angle) {
  Eigen::Vector3f localUnitX;
  {
    Eigen::Quaternionf qin(0, 1, 0, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    //Eigen::Quaternionf eeqform(eepReg2.qw, eepReg2.qx, eepReg2.qy, eepReg2.qz);
    Eigen::Quaternionf eeqform(0, 0, 1.0, 0);
    //Eigen::Quaternionf eeqform(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
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
    //Eigen::Quaternionf eeqform(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
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
    //Eigen::Quaternionf eeqform(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitZ.x() = qout.x();
    localUnitZ.y() = qout.y();
    localUnitZ.z() = qout.z();
  }

  double deltaTheta = angle + (double(givenGraspGear)*2.0*3.1415926/double(totalGraspGears));
  double sinBuff = 0.0;
  double angleRate = 1.0;
  //Eigen::Quaternionf eeBaseQuat(eepReg2.qw, eepReg2.qx, eepReg2.qy, eepReg2.qz);
  //Eigen::Quaternionf eeBaseQuat(0, 0, 1, 0);
  Eigen::Quaternionf eeBaseQuat(bestOrientationEEPose.qw, bestOrientationEEPose.qx, bestOrientationEEPose.qy, bestOrientationEEPose.qz);
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

void setCCRotation(int thisGraspGear) {
  //Eigen::Quaternionf eeBaseQuat = getCCRotation(thisGraspGear, -bestOrientationAngle);
  Eigen::Quaternionf eeBaseQuat = getCCRotation(thisGraspGear, 0.0);

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

  time(&thisTimeRange);
  double deltaTimeRange = difftime(thisTimeRange, firstTimeRange);
  timeMassRange = timeMassRange + 1;

  if (deltaTimeRange > timeIntervalRange) {
    deltaTimeRange = 0;
    timeMassRange = 0;
    time(&firstTimeRange);
  }

  if (timeMassRange > 0.0) {
    aveTimeRange = deltaTimeRange / timeMassRange;
  }

  if (deltaTimeRange > 0.0) {
    aveFrequencyRange = timeMassRange / deltaTimeRange;
  }

  eeRange = range.range;
  rangeHistory[currentRangeHistoryIndex] = eeRange;
  currentRangeHistoryIndex++;
  currentRangeHistoryIndex = currentRangeHistoryIndex % totalRangeHistoryLength;

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

  {
    cv::Point text_anchor = cv::Point(0,rangeogramImage.rows-1);
    {
      cv::Scalar backColor(0,0,0);
      cv::Point outTop = cv::Point(text_anchor.x,text_anchor.y+1-35);
      cv::Point outBot = cv::Point(text_anchor.x+350,text_anchor.y+1);
      Mat vCrop = rangeogramImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
      vCrop = backColor;
    }
    {
      char buff[256];
      sprintf(buff, "            Hz: %.2f", aveFrequency);
      string fpslabel(buff);
      putText(rangeogramImage, fpslabel, text_anchor, MY_FONT, 1.0, Scalar(160,0,0), 1.0);
    }
    {
      char buff[256];
      sprintf(buff, "Hz: %.2f", aveFrequencyRange);
      string fpslabel(buff);
      putText(rangeogramImage, fpslabel, text_anchor, MY_FONT, 1.0, Scalar(0,0,160), 1.0);
    }
  }
  guardedImshow(rangeogramViewName, rangeogramImage);

  if (!shouldIRangeCallback) {
    return;
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

      Eigen::Vector3d irSensorEnd(
				   (trueEEPose.position.x - irSensorStartLocal.x()) + eeRange*localUnitZ.x(),
				   (trueEEPose.position.y - irSensorStartLocal.y()) + eeRange*localUnitZ.y(),
				   (trueEEPose.position.z - irSensorStartLocal.z()) + eeRange*localUnitZ.z()
				  );

      dX = (irSensorEnd.x() - rmcX); 
      dY = (irSensorEnd.y() - rmcY); 

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

      double minDepth = VERYBIGNUMBER;
      double maxDepth = 0;
      for (int rx = 0; rx < rmWidth; rx++) {
	for (int ry = 0; ry < rmWidth; ry++) {
	  minDepth = min(minDepth, rangeMap[rx + ry*rmWidth]);
	  maxDepth = max(maxDepth, rangeMap[rx + ry*rmWidth]);
	}
      }
      double denom2 = max(EPSILON,maxDepth-minDepth);
      if (denom2 <= EPSILON)
	denom2 = VERYBIGNUMBER;
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
      
      double minDepth = VERYBIGNUMBER;
      double maxDepth = 0;
      for (int rx = 0; rx < rmWidth; rx++) {
	for (int ry = 0; ry < rmWidth; ry++) {
	  minDepth = min(minDepth, rangeMap[rx + ry*rmWidth]);
	  maxDepth = max(maxDepth, rangeMap[rx + ry*rmWidth]);
	}
      }
      double denom2 = max(EPSILON,maxDepth-minDepth);
      if (denom2 <= EPSILON)
	denom2 = VERYBIGNUMBER;
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
    guardedImshow(rangemapViewName, rangemapImage);
    guardedImshow(graspMemoryViewName, graspMemoryImage);
    guardedImshow(graspMemorySampleViewName, graspMemorySampleImage);
    guardedImshow(heightMemorySampleViewName, heightMemorySampleImage);
    //guardedImshow(hiRangemapViewName, hiRangemapImage);
    Mat hRIT;
    cv::resize(hiRangemapImage, hRIT, cv::Size(0,0), 2, 2);
    guardedImshow(hiRangemapViewName, hRIT);
    Mat hCRIT;
    cv::resize(hiColorRangemapImage, hCRIT, cv::Size(0,0), 2, 2);
    guardedImshow(hiColorRangemapViewName, hCRIT);

    guardedImshow(objectViewerName, objectViewerImage);
    guardedImshow(objectMapViewerName, objectMapViewerImage);
    //cv::moveWindow(objectMapViewerName, 0, 0);

    guardedImshow(densityViewerName, densityViewerImage);
    guardedImshow(gradientViewerName, gradientViewerImage);
    guardedImshow(objectnessViewerName, objectnessViewerImage);
    
    if (targetClass > -1) {
      if (classHeight0AerialGradients[targetClass].rows == aerialGradientWidth) {
	Mat crop0 = aerialGradientViewerImage(cv::Rect(0, 3*aerialGradientWidth, aerialGradientWidth, aerialGradientWidth));
	double min0 = 0;
	double max0 = 0;
	minMaxLoc(classHeight0AerialGradients[targetClass], &min0, &max0);
	double denom0 = max0-min0;
	if (fabs(denom0) < EPSILON)
	  denom0 = 1;
	crop0 = (classHeight0AerialGradients[targetClass] - min0) / denom0;

	Mat crop1 = aerialGradientViewerImage(cv::Rect(0, 2*aerialGradientWidth, aerialGradientWidth, aerialGradientWidth));
	double min1 = 0;
	double max1 = 0;
	minMaxLoc(classHeight1AerialGradients[targetClass], &min1, &max1);
	double denom1 = max1-min1;
	if (fabs(denom1) < EPSILON)
	  denom1 = 1;
	crop1 = (classHeight1AerialGradients[targetClass] - min1) / denom1;

	Mat crop2 = aerialGradientViewerImage(cv::Rect(0, 1*aerialGradientWidth, aerialGradientWidth, aerialGradientWidth));
	double min2 = 0;
	double max2 = 0;
	minMaxLoc(classHeight2AerialGradients[targetClass], &min2, &max2);
	double denom2 = max2-min2;
	if (fabs(denom2) < EPSILON)
	  denom2 = 1;
	crop2 = (classHeight2AerialGradients[targetClass] - min2) / denom2;

	Mat crop3 = aerialGradientViewerImage(cv::Rect(0, 0*aerialGradientWidth, aerialGradientWidth, aerialGradientWidth));
	double min3 = 0;
	double max3 = 0;
	minMaxLoc(classHeight3AerialGradients[targetClass], &min3, &max3);
	double denom3 = max3-min3;
	if (fabs(denom3) < EPSILON)
	  denom3 = 1;
	crop3 = (classHeight3AerialGradients[targetClass] - min3) / denom3;

	guardedImshow(aerialGradientViewerName, aerialGradientViewerImage);
      }
    }
  }
}

void endEffectorAngularUpdate(eePose *givenEEPose) {

  /* global cartesian update 
  Eigen::Matrix3f m;
  m = Eigen::AngleAxisf(givenEEPose.ox*M_PI, Eigen::Vector3f::UnitX())
  * Eigen::AngleAxisf(givenEEPose.oy*M_PI, Eigen::Vector3f::UnitY())
  * Eigen::AngleAxisf(givenEEPose.oz*M_PI, Eigen::Vector3f::UnitZ());

  Eigen::Quaternionf eeRotator(m);
  Eigen::Quaternionf eeBaseQuat(givenEEPose.qw, givenEEPose.qx, givenEEPose.qy, givenEEPose.qz);

  eeBaseQuat = eeRotator * eeBaseQuat;
  */ 

  /* end effector local angular update */
  Eigen::Vector3f localUnitX;
  {
    Eigen::Quaternionf qin(0, 1, 0, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(givenEEPose->qw, givenEEPose->qx, givenEEPose->qy, givenEEPose->qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitX.x() = qout.x();
    localUnitX.y() = qout.y();
    localUnitX.z() = qout.z();
  }

  Eigen::Vector3f localUnitY;
  {
    Eigen::Quaternionf qin(0, 0, 1, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(givenEEPose->qw, givenEEPose->qx, givenEEPose->qy, givenEEPose->qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitY.x() = qout.x();
    localUnitY.y() = qout.y();
    localUnitY.z() = qout.z();
  }

  Eigen::Vector3f localUnitZ;
  {
    Eigen::Quaternionf qin(0, 0, 0, 1);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(givenEEPose->qw, givenEEPose->qx, givenEEPose->qy, givenEEPose->qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitZ.x() = qout.x();
    localUnitZ.y() = qout.y();
    localUnitZ.z() = qout.z();
  }

  double sinBuff = 0.0;
  double angleRate = 1.0;
  Eigen::Quaternionf eeBaseQuat(givenEEPose->qw, givenEEPose->qx, givenEEPose->qy, givenEEPose->qz);
  sinBuff = sin(angleRate*givenEEPose->ox/2.0);
  Eigen::Quaternionf eeRotatorX(cos(angleRate*givenEEPose->ox/2.0), localUnitX.x()*sinBuff, localUnitX.y()*sinBuff, localUnitX.z()*sinBuff);
  sinBuff = sin(angleRate*givenEEPose->oy/2.0);
  Eigen::Quaternionf eeRotatorY(cos(angleRate*givenEEPose->oy/2.0), localUnitY.x()*sinBuff, localUnitY.y()*sinBuff, localUnitY.z()*sinBuff);
  sinBuff = sin(angleRate*givenEEPose->oz/2.0);
  Eigen::Quaternionf eeRotatorZ(cos(angleRate*givenEEPose->oz/2.0), localUnitZ.x()*sinBuff, localUnitZ.y()*sinBuff, localUnitZ.z()*sinBuff);
  givenEEPose->ox = 0;
  givenEEPose->oy = 0;
  givenEEPose->oz = 0;
  eeRotatorX.normalize();
  eeRotatorY.normalize();
  eeRotatorZ.normalize();
  //eeBaseQuat = eeRotatorX * eeRotatorY * eeRotatorZ * 
		//eeBaseQuat * 
	      //eeRotatorZ.conjugate() * eeRotatorY.conjugate() * eeRotatorX.conjugate();
  eeBaseQuat = eeRotatorX * eeRotatorY * eeRotatorZ * eeBaseQuat;
  eeBaseQuat.normalize();

  givenEEPose->qx = eeBaseQuat.x();
  givenEEPose->qy = eeBaseQuat.y();
  givenEEPose->qz = eeBaseQuat.z();
  givenEEPose->qw = eeBaseQuat.w();
}

void fillIkRequest(eePose * givenEEPose, baxter_core_msgs::SolvePositionIK * givenIkRequest) {
  givenIkRequest->request.pose_stamp.resize(1);

  givenIkRequest->request.pose_stamp[0].header.seq = 0;
  givenIkRequest->request.pose_stamp[0].header.stamp = ros::Time::now();
  givenIkRequest->request.pose_stamp[0].header.frame_id = "/base";

  
  givenIkRequest->request.pose_stamp[0].pose.position.x = givenEEPose->px;
  givenIkRequest->request.pose_stamp[0].pose.position.y = givenEEPose->py;
  givenIkRequest->request.pose_stamp[0].pose.position.z = givenEEPose->pz;

//  Eigen::Quaternionf normalizer(givenEEPose->qw, givenEEPose->qx, givenEEPose->qy, givenEEPose->qz);
//  normalizer.normalize();
//  givenEEPose->qx = normalizer.x();
//  givenEEPose->qy = normalizer.y();
//  givenEEPose->qz = normalizer.z();
//  givenEEPose->qw = normalizer.w();

  givenIkRequest->request.pose_stamp[0].pose.orientation.x = givenEEPose->qx;
  givenIkRequest->request.pose_stamp[0].pose.orientation.y = givenEEPose->qy;
  givenIkRequest->request.pose_stamp[0].pose.orientation.z = givenEEPose->qz;
  givenIkRequest->request.pose_stamp[0].pose.orientation.w = givenEEPose->qw;
}

bool willIkResultFail(baxter_core_msgs::SolvePositionIK thisIkRequest, int thisIkCallResult, bool * likelyInCollision) {
  bool thisIkResultFailed = 0;
  *likelyInCollision = 0;

  if (thisIkCallResult && thisIkRequest.response.isValid[0]) {
    thisIkResultFailed = 0;
  } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].position.size() != numJoints)) {
    thisIkResultFailed = 1;
    //cout << "Initial IK result appears to be truly invalid, not enough positions." << endl;
  } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].name.size() != numJoints)) {
    thisIkResultFailed = 1;
    //cout << "Initial IK result appears to be truly invalid, not enough names." << endl;
  } else if (thisIkRequest.response.joints.size() == 1) {
    if( usePotentiallyCollidingIK ) {
      //cout << "WARNING: using ik even though result was invalid under presumption of false collision..." << endl;
      //cout << "Received enough positions and names for ikPose: " << thisIkRequest.request.pose_stamp[0].pose << endl;
      thisIkResultFailed = 0;
      *likelyInCollision = 1;
    } else {
      thisIkResultFailed = 1;
      *likelyInCollision = 1;
    }
  } else {
    thisIkResultFailed = 1;
    //cout << "Initial IK result appears to be truly invalid, incorrect joint field." << endl;
  }

  return thisIkResultFailed;
}

void update_baxter(ros::NodeHandle &n) {
  bfc = bfc % bfc_period;

  if (!shouldIDoIK) {
    return;
  }

  baxter_core_msgs::SolvePositionIK thisIkRequest;
  endEffectorAngularUpdate(&currentEEPose);
  fillIkRequest(&currentEEPose, &thisIkRequest);

  int ikResultFailed = 0;

  // do not start in a state with ikShare 
  if ((drand48() <= ikShare) || !ikInitialized) {

    int numIkRetries = 100;
    double ikNoiseAmplitude = 0.03;
    double useZOnly = 1;
    double ikNoiseAmplitudeQuat = 0;
    for (int ikRetry = 0; ikRetry < numIkRetries; ikRetry++) {
      int ikCallResult = ikClient.call(thisIkRequest);
      //ikResultFailed = (!ikClient.call(thisIkRequest) || !thisIkRequest.response.isValid[0]);
      //cout << "ik call result: " << ikCallResult << " joints: " << (thisIkRequest.response.joints.size()) << " "; 

      //if (thisIkRequest.response.joints.size()) {
	//cout << "position size: " << (thisIkRequest.response.joints[0].position.size()) << endl;
      //}


      //ikResultFailed = (!ikCallResult || (thisIkRequest.response.joints.size() == 0) || (thisIkRequest.response.joints[0].position.size() != numJoints));

//      // XXX This is ridiculous
//      if (ikCallResult && thisIkRequest.response.isValid[0]) {
//	// set this here in case noise was added
//	currentEEPose.px = thisIkRequest.request.pose_stamp[0].pose.position.x;
//	currentEEPose.py = thisIkRequest.request.pose_stamp[0].pose.position.y;
//	currentEEPose.pz = thisIkRequest.request.pose_stamp[0].pose.position.z;
//	ikResultFailed = 0;
//      } else {
//	ikResultFailed = 1;
//	cout << "Initial IK result appears to be invalid." << endl;
//      } 

      if (ikCallResult && thisIkRequest.response.isValid[0]) {
	// set this here in case noise was added
	currentEEPose.px = thisIkRequest.request.pose_stamp[0].pose.position.x;
	currentEEPose.py = thisIkRequest.request.pose_stamp[0].pose.position.y;
	currentEEPose.pz = thisIkRequest.request.pose_stamp[0].pose.position.z;
	ikResultFailed = 0;
	if (ikRetry > 0) {
	  ROS_WARN_STREAM("___________________");
	  ROS_ERROR_STREAM("Accepting perturbed IK result.");
	  ROS_WARN_STREAM("___________________");
	}
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].position.size() != numJoints)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough positions." << endl;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].name.size() != numJoints)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough names." << endl;
      } else if (thisIkRequest.response.joints.size() == 1) {
	if( usePotentiallyCollidingIK ) {
	  cout << "WARNING: using ik even though result was invalid under presumption of false collision..." << endl;
	  cout << "Received enough positions and names for ikPose: " << thisIkRequest.request.pose_stamp[0].pose << endl;

	  ikResultFailed = 0;
	  currentEEPose.px = thisIkRequest.request.pose_stamp[0].pose.position.x;
	  currentEEPose.py = thisIkRequest.request.pose_stamp[0].pose.position.y;
	  currentEEPose.pz = thisIkRequest.request.pose_stamp[0].pose.position.z;
	} else {
	  ikResultFailed = 1;
	  cout << "ik result was reported as colliding and we are sensibly rejecting it..." << endl;
	}
      } else {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, incorrect joint field." << endl;
      }

      if (!ikResultFailed) {
	break;
      }

      ROS_WARN_STREAM("Initial IK result invalid... adding noise and retrying.");
      cout << thisIkRequest.request.pose_stamp[0].pose << endl;

      eePose noisedCurrentEEPose = currentEEPose;
      noisedCurrentEEPose.px = currentEEPose.px + (drand48() - 0.5)*2.0*ikNoiseAmplitude*(1.0-useZOnly);
      noisedCurrentEEPose.py = currentEEPose.py + (drand48() - 0.5)*2.0*ikNoiseAmplitude*(1.0-useZOnly);
      noisedCurrentEEPose.pz = currentEEPose.pz + (drand48() - 0.5)*2.0*ikNoiseAmplitude*useZOnly;

      noisedCurrentEEPose.qx = currentEEPose.qx + (drand48() - 0.5)*2.0*ikNoiseAmplitudeQuat;
      noisedCurrentEEPose.qy = currentEEPose.qy + (drand48() - 0.5)*2.0*ikNoiseAmplitudeQuat;
      noisedCurrentEEPose.qz = currentEEPose.qz + (drand48() - 0.5)*2.0*ikNoiseAmplitudeQuat;
      noisedCurrentEEPose.qw = currentEEPose.qw + (drand48() - 0.5)*2.0*ikNoiseAmplitudeQuat;

      fillIkRequest(&noisedCurrentEEPose, &thisIkRequest);
    }
  }

  /*
    if ( ikClient.waitForExistence(ros::Duration(1, 0)) ) {
  //cout << "block6.1" << endl;
      ikResultFailed = (!ikClient.call(thisIkRequest) || !thisIkRequest.response.isValid[0]);
    } else {
      cout << "waitForExistence timed out" << endl;
      ikResultFailed = 1;
    }
  */

    if (ikResultFailed) 
    {
      ROS_ERROR_STREAM("ikClient says pose request is invalid.");
      ik_reset_counter++;

      cout << "ik_reset_counter, ik_reset_thresh: " << ik_reset_counter << " " << ik_reset_thresh << endl;
      if (ik_reset_counter > ik_reset_thresh) {
	ik_reset_counter = 0;
	currentEEPose = ik_reset_eePose;
	pushWord('Y'); // pause stack execution
	pushCopies("beep", 15); // beep
	cout << "target position denied by ik, please reset the object.";
      }
      else {
	cout << "This pose was rejected by ikClient:" << endl;
	cout << "Current EE Position (x,y,z): " << currentEEPose.px << " " << currentEEPose.py << " " << currentEEPose.pz << endl;
	cout << "Current EE Orientation (x,y,z,w): " << currentEEPose.qx << " " << currentEEPose.qy << " " << currentEEPose.qz << " " << currentEEPose.qw << endl;

	currentEEPose = lastGoodEEPose;
      }

      return;
    }

    ik_reset_counter = max(ik_reset_counter-1, 0);

    lastGoodEEPose = currentEEPose;
    ikRequest = thisIkRequest;
    ikInitialized = 1;
  
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
      myCommand.command[j] = ikRequest.response.joints[0].position[j];
    }
  }

  std_msgs::Float64 speedCommand;
  speedCommand.data = currentEESpeedRatio;
  for (int r = 0; r < resend_times; r++) {
    joint_mover.publish(myCommand);
    moveSpeedPub.publish(speedCommand);
  }

  bfc++;
}

void clearStack() {
  call_stack.resize(0);
}
Word * popWord() {
  if (call_stack.size() > 0) {
    Word * word = call_stack.back();
    call_stack.pop_back();
    return word; 
  } else {
    return NULL;
  }
}

bool pushWord(int code) {
  if (character_code_to_word.count(code) > 0) {
    Word * word = character_code_to_word[code];      
    return pushWord(word);
  } else {
    cout << "No word for code " << code << endl;
    return false;
  }
}


Word * forthletParse(string token) {
  if (IntegerWord::isInteger(token)) {
    return IntegerWord::parse(token);
  } else if (StringWord::isString(token)) {
    return StringWord::parse(token);
  } else if (name_to_word.count(token) > 0) {
    Word * word = name_to_word[token];
    return word;
  } else if (SymbolWord::isSymbol(token)) {
    Word * word = SymbolWord::parse(token);
    return word;
  } else {
    cout << "Cannot parse " << token << endl;
    return NULL;
  }
}
bool pushWord(string token) {
  Word * word = forthletParse(token);
  if (word != NULL) {
    pushWord(word);
  }
}


bool pushWord(Word * word) {
  call_stack.push_back(word);
  return true;
}

Word * nameToWord(string name) {
  if (name_to_word.count(name) > 0) {
    Word * word = name_to_word[name];
    return word;
  } else  {
    cout << "Could not find word with name " << name << endl;
    assert(0);
  }
}

void timercallback1(const ros::TimerEvent&) {

  ros::NodeHandle n("~");
  Word * word = NULL;

  int c = -1;
  int takeSymbol = 1;
  if (shouldIMiscCallback) {
    c = cvWaitKey(1);
  } else if ((heartBeatCounter % heartBeatPeriod) == 0) {
    c = cvWaitKey(1);
    heartBeatCounter = 0;
  }
  heartBeatCounter++;

  if (c != -1) {
    // don't print for capslock, shift, alt (for alt-tab)
    if (!(c == 65509 || c == 196581 || c == 196577 || c == 65505 ||
          c == 65513 || c == 196578)) {
      cout << "You pressed " << c << "." << endl;

      takeSymbol = 0;
      if (character_code_to_word.count(c) > 0) {
        word = character_code_to_word[c];      
      } else {
        cout  << "Could not find word for " << c << endl;
      }
    }
  }

  // deal with the stack
//  if (execute_stack && takeSymbol) {
//    if (call_stack.size() > 0 && 
//        !call_stack[call_stack.size() - 1]->is_value()) {
//      word = popWord();
//    } else {
//      execute_stack = 0;
//    }
//  }
//  if (timerCounter >= lock_reset_thresh) {
//    lock_status = 0;
//  }
//  
//  if (word != NULL) {
//    current_instruction = word;
//    word->execute();
//  }

  endThisStackCollapse = endCollapse;
  while (1) {
    time(&thisTime);
    double deltaTime = difftime(thisTime, firstTime);
    timeMass = timeMass + 1;

    if (deltaTime > timeInterval) {
      deltaTime = 0;
      timeMass = 0;
      time(&firstTime);
    }

    if (timeMass > 0.0) {
      aveTime = deltaTime / timeMass;
    }

    if (deltaTime > 0.0) {
      aveFrequency = timeMass / deltaTime;
    }

    // deal with the stack
    if (execute_stack && takeSymbol) {
      if (call_stack.size() > 0 && 
	  !call_stack[call_stack.size() - 1]->is_value()) {
	word = popWord();
      } else {
	execute_stack = 0;
	endThisStackCollapse = 1;
      }
    } else {
      endThisStackCollapse = 1;
    }

    if (timerCounter >= lock_reset_thresh) {
      lock_status = 0;
    }
  
    if (word != NULL) {
      current_instruction = word;
      word->execute();
    }

    if( endThisStackCollapse || (call_stack.size() == 0) ) {
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

  timerCounter++;
  timesTimerCounted++;

  renderCoreView();

  if (shouldIRender) {
    renderObjectMapView();
  }
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg){

  lastImageCallbackReceived = ros::Time::now();

  if (!shouldIImageCallback) {
    return;
  }

  if (!renderInit) {
    renderInit = 1;
    shouldIRender = shouldIRenderDefault;

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

    accumulatedImage = Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_64FC3);
    accumulatedImageMass = Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_64F);

    densityViewerImage = cv_ptr->image.clone();
    densityViewerImage *= 0;
    gradientViewerImage = Mat(2*cv_ptr->image.rows, cv_ptr->image.cols, cv_ptr->image.type());
    objectnessViewerImage = Mat(cv_ptr->image.rows, cv_ptr->image.cols, cv_ptr->image.type());
    objectnessViewerImage *= 0;
    aerialGradientViewerImage = Mat(4*aerialGradientWidth, aerialGradientWidth, CV_64F);
    objectViewerImage = cv_ptr->image.clone();
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

  guardViewers();

  Size sz = accumulatedImage.size();
  int imW = sz.width;
  int imH = sz.height;

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      accumulatedImage.at<Vec3d>(y,x)[0] = accumulatedImage.at<Vec3d>(y,x)[0] + wristCamImage.at<Vec3b>(y,x)[0];
      accumulatedImage.at<Vec3d>(y,x)[1] = accumulatedImage.at<Vec3d>(y,x)[1] + wristCamImage.at<Vec3b>(y,x)[1];
      accumulatedImage.at<Vec3d>(y,x)[2] = accumulatedImage.at<Vec3d>(y,x)[2] + wristCamImage.at<Vec3b>(y,x)[2];
      accumulatedImageMass.at<double>(y,x) += 1.0;
    }
  }

  setRingImageAtTime(msg->header.stamp, wristCamImage);
  Mat thisImage;
  int weHaveImData = getRingImageAtTime(msg->header.stamp, thisImage);

  //if (recordRangeMap) 
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

    double volumeRenderThresh = 0.333;

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
	    //p.a = volumeMap[px + py*vmWidth + pz*vmWidth*vmWidth] > 0.5;
	    p.a = volumeMap[px + py*vmWidth + pz*vmWidth*vmWidth] > volumeRenderThresh;
	    //p.a = 4.0*volumeMap[px + py*vmWidth + pz*vmWidth*vmWidth];
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

  // paint gripper reticle centerline
  if (1) {
    eePose teePose;
    teePose.px = trueEEPose.position.x;
    teePose.py = trueEEPose.position.y;
    teePose.pz = trueEEPose.position.z;
    cv::Scalar theColor(192, 64, 64);
    cv::Scalar THEcOLOR(64, 192, 192);

    double zStart = (minHeight + currentTableZ) + currentTableZ;
    double zEnd = (maxHeight + currentTableZ) + currentTableZ; 
    double deltaZ = 0.005;
    
    for (double zCounter = zStart; zCounter < zEnd; zCounter += deltaZ) {
      int pX = 0, pY = 0;  
      double zToUse = zCounter;

      globalToPixel(&pX, &pY, zToUse, teePose.px, teePose.py);
      Point pt1(pX, pY);

      zToUse = zCounter+deltaZ;
      globalToPixel(&pX, &pY, zToUse, teePose.px, teePose.py);
      Point pt2(pX, pY);

      line(wristViewImage, pt1, pt2, theColor, 3);
    }
    for (double zCounter = zStart; zCounter < zEnd; zCounter += deltaZ) {
      int pX = 0, pY = 0;  
      double zToUse = zCounter;

      globalToPixel(&pX, &pY, zToUse, teePose.px, teePose.py);
      Point pt1(pX, pY);

      zToUse = zCounter+deltaZ;
      globalToPixel(&pX, &pY, zToUse, teePose.px, teePose.py);
      Point pt2(pX, pY);

      line(wristViewImage, pt1, pt2, THEcOLOR, 1);
    }

  }

  // paint transform reticles
  if (paintEEandReg1OnWrist) {
    eePose teePose;
    teePose.px = trueEEPose.position.x;
    teePose.py = trueEEPose.position.y;
    teePose.pz = trueEEPose.position.z;
    paintEEPoseOnWrist(teePose, cv::Scalar(0,0,255));
    paintEEPoseOnWrist(eepReg1, cv::Scalar(0,255,0));

    {
      eePose irPose;
      {
	geometry_msgs::Pose thisPose = trueEEPose;
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

	Eigen::Vector3d irSensorEnd(
				     (thisPose.position.x - irSensorStartLocal.x()) + eeRange*localUnitZ.x(),
				     (thisPose.position.y - irSensorStartLocal.y()) + eeRange*localUnitZ.y(),
				     (thisPose.position.z - irSensorStartLocal.z()) + eeRange*localUnitZ.z()
				    );
	irPose.px = irSensorEnd.x();
	irPose.py = irSensorEnd.y();
	irPose.pz = irSensorEnd.z();
      }
      paintEEPoseOnWrist(irPose, cv::Scalar(255,0,0));
    }
  }

  // draw vanishing point reticle
  {
    int vanishingPointReticleRadius = 15;

    int x0 = vanishingPointReticle.px;
    int y0 = vanishingPointReticle.py;
    Point pt1(x0, y0);

    cv::Scalar theColor(192, 64, 64);
    cv::Scalar THEcOLOR(64, 192, 192);

    circle(wristViewImage, pt1, vanishingPointReticleRadius, theColor, 1);
    circle(wristViewImage, pt1, vanishingPointReticleRadius+1, THEcOLOR, 1);
    circle(wristViewImage, pt1, vanishingPointReticleRadius+3, theColor, 1);
  }

  // draw probe reticle
  {
    int probeReticleHalfWidth = 7;
    int x0 = probeReticle.px;
    int y0 = probeReticle.py;

    int x1 = max(int(probeReticle.px-probeReticleHalfWidth), 0);
    int x2 = min(int(probeReticle.px+probeReticleHalfWidth), wristViewImage.cols);
    int y1 = max(int(probeReticle.py-probeReticleHalfWidth), 0);
    int y2 = min(int(probeReticle.py+probeReticleHalfWidth), wristViewImage.rows);

    int probeReticleShortHalfWidth = 3;
    int x1s = max(int(probeReticle.px-probeReticleShortHalfWidth), 0);
    int x2s = min(int(probeReticle.px+probeReticleShortHalfWidth), wristViewImage.cols);
    int y1s = max(int(probeReticle.py-probeReticleShortHalfWidth), 0);
    int y2s = min(int(probeReticle.py+probeReticleShortHalfWidth), wristViewImage.rows);

    cv::Scalar theColor(255, 0, 0);
    cv::Scalar THEcOLOR(0, 255, 255);
    {
      int xs = x0;
      int xf = x0;
      int ys = y1;
      int yf = y1s;
      {
	Point pt1(xs, ys);
	Point pt2(xf, yf);
	line(wristViewImage, pt1, pt2, theColor, 2.0);
      }
      {
	Point pt1(xs, ys+1);
	Point pt2(xf, yf-1);
	line(wristViewImage, pt1, pt2, THEcOLOR, 1.0);
      }
    }
    {
      int xs = x0;
      int xf = x0;
      int ys = y2s;
      int yf = y2;
      {
	Point pt1(xs, ys);
	Point pt2(xf, yf);
	line(wristViewImage, pt1, pt2, theColor, 2.0);
      }
      {
	Point pt1(xs, ys+1);
	Point pt2(xf, yf-1);
	line(wristViewImage, pt1, pt2, THEcOLOR, 1.0);
      }
    }
    {
      int xs = x1;
      int xf = x1s;
      int ys = y0;
      int yf = y0;
      {
	Point pt1(xs, ys);
	Point pt2(xf, yf);
	line(wristViewImage, pt1, pt2, theColor, 2.0);
      }
      {
	Point pt1(xs+1, ys);
	Point pt2(xf-1, yf);
	line(wristViewImage, pt1, pt2, THEcOLOR, 1.0);
      }
    }
    {
      int xs = x2s;
      int xf = x2;
      int ys = y0;
      int yf = y0;
      {
	Point pt1(xs, ys);
	Point pt2(xf, yf);
	line(wristViewImage, pt1, pt2, theColor, 2.0);
      }
      {
	Point pt1(xs+1, ys);
	Point pt2(xf-1, yf);
	line(wristViewImage, pt1, pt2, THEcOLOR, 1.0);
      }
    }
  
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

  // ATTN 16
  if (1) {
    for (int hri = 0; hri < 4; hri++) {
      if (hri != currentThompsonHeightIdx)
	continue;
      eePose thisReticle = heightReticles[hri];
      int thisReticleHalfWidth = int(  ceil( double(reticleHalfWidth) / double(1+hri) )  );
      cv::Point outTop = cv::Point(thisReticle.px-thisReticleHalfWidth, thisReticle.py-thisReticleHalfWidth);
      cv::Point outBot = cv::Point(thisReticle.px+thisReticleHalfWidth, thisReticle.py+thisReticleHalfWidth);
      cv::Point inTop = cv::Point(thisReticle.px+1-thisReticleHalfWidth,thisReticle.py+1-thisReticleHalfWidth);
      cv::Point inBot = cv::Point(thisReticle.px-1+thisReticleHalfWidth,thisReticle.py-1+thisReticleHalfWidth);

      if (hri == currentThompsonHeightIdx) {
	rectangle(wristViewImage, outTop, outBot, cv::Scalar(22,70,82)); 
	rectangle(wristViewImage, inTop, inBot, cv::Scalar(68,205,239));
      } else {
	rectangle(wristViewImage, outTop, outBot, cv::Scalar(82,70,22)); // RGB: 22 70 82
	rectangle(wristViewImage, inTop, inBot, cv::Scalar(239,205,68)); // RGB: 68 205 239
      }
    }
  }

  {
    cv::Point outTop = cv::Point(pilotTarget.px-pilotTargetHalfWidth, pilotTarget.py-pilotTargetHalfWidth);
    cv::Point outBot = cv::Point(pilotTarget.px+pilotTargetHalfWidth, pilotTarget.py+pilotTargetHalfWidth);
    cv::Point inTop = cv::Point(pilotTarget.px+1-pilotTargetHalfWidth,pilotTarget.py+1-pilotTargetHalfWidth);
    cv::Point inBot = cv::Point(pilotTarget.px-1+pilotTargetHalfWidth,pilotTarget.py-1+pilotTargetHalfWidth);
    if ( (outTop.x > 0) && (outTop.y > 0) && (outBot.x < imW) && (outBot.y < imH) ) {
      rectangle(wristViewImage, outTop, outBot, cv::Scalar(53,10,97)); // RGB: 97 10 53
      rectangle(wristViewImage, inTop, inBot, cv::Scalar(142,31,255)); // RGB: 255 31 142
    }
  }

  if (shouldIRender) {
    guardedImshow(wristViewName, wristViewImage);
  }
}

cv::Point worldToPixel(Mat image, double xMin, double xMax, double yMin, double yMax, double x, double y) {
  double pxMin = 0;
  double pxMax = objectMapViewerImage.cols;
  double pyMin = 0;
  double pyMax = objectMapViewerImage.rows;
  cv::Point center = cv::Point(pxMax/2, pyMax/2);

  cv::Point out = cv::Point((pyMax - pyMin) / (yMax - yMin) * y + center.y,
                            (pxMax - pxMin) / (xMax - xMin) * x + center.x);
  return out;
}

void renderObjectMapView() {
  if (objectMapViewerImage.rows <= 0 ) {
    objectMapViewerImage = Mat(800, 800, CV_8UC3);
  }

  if (0) { // drawGrid
    for (int i = 0; i < mapWidth; i++) {
      for (int j = 0; j < mapHeight; j++) {
        gsl_matrix * mapcell = mapCellToPolygon(i, j);
        drawMapPolygon(mapcell, CV_RGB(0, 255, 0));
        gsl_matrix_free(mapcell);
      }
    }
  }

  objectMapViewerImage = CV_RGB(0, 0, 0);
  double pxMin = 0;
  double pxMax = objectMapViewerImage.cols;
  double pyMin = 0;
  double pyMax = objectMapViewerImage.rows;

  cv::Point center = cv::Point(pxMax/2, pyMax/2);

  double fadeBias = 0.50;
  double fadeLast = 30.0;

  for (int i = 0; i < mapWidth; i++) {
    for (int j = 0; j < mapHeight; j++) {

      ros::Duration longAgo = ros::Time::now() - objectMap[i + mapWidth * j].lastMappedTime;
      double fadeFraction = (1.0-fadeBias)*(1.0-(min(max(longAgo.toSec(), 0.0), fadeLast) / fadeLast)) + fadeBias;
      fadeFraction = min(max(fadeFraction, 0.0), 1.0);
      if (!useGlow) {
	fadeFraction = 1.0;
      }
      if (!useFade) {
	fadeFraction = 1.0;
      }

      double x, y;
      mapijToxy(i, j, &x, &y);

      if (objectMap[i + mapWidth * j].detectedClass != -1) {
        cv::Point outTop = worldToPixel(objectMapViewerImage, mapXMin, mapXMax, mapYMin, mapYMax, x, y);
        cv::Point outBot = worldToPixel(objectMapViewerImage, mapXMin, mapXMax, mapYMin, mapYMax, x + mapStep, y + mapStep);
        cv::Scalar color = CV_RGB((int) (objectMap[i + mapWidth * j].r / objectMap[i + mapWidth * j].pixelCount),
                                  (int) (objectMap[i + mapWidth * j].g / objectMap[i + mapWidth * j].pixelCount),
                                  (int) (objectMap[i + mapWidth * j].b / objectMap[i + mapWidth * j].pixelCount) );
	color = color*fadeFraction;
        rectangle(objectMapViewerImage, outTop, outBot, 
                  color,
                  CV_FILLED);
      }
    }
  }


  double glowBias = 0.15;
  double glowLast = 30.0;

  if (drawIKMap) { // draw ikMap
    int ikMapRenderStride = 1;
    for (int i = 0; i < mapWidth; i+=ikMapRenderStride) {
      for (int j = 0; j < mapHeight; j+=ikMapRenderStride) {
	if ( cellIsSearched(i, j) ) {
	    ros::Duration longAgo = ros::Time::now() - objectMap[i + mapWidth * j].lastMappedTime;
	    double glowFraction = (1.0-glowBias)*(1.0-(min(max(longAgo.toSec(), 0.0), glowLast) / glowLast)) + glowBias;
	    glowFraction = min(max(glowFraction, 0.0), 1.0);
	    if (!useGlow) {
	      glowFraction = 1.0;
	    }
	    double x=-1, y=-1;
	    mapijToxy(i, j, &x, &y);
	    cv::Point cvp1 = worldToPixel(objectMapViewerImage, 
	      mapXMin, mapXMax, mapYMin, mapYMax, x, y);
	    if ( (ikMap[i + mapWidth * j] == 1) ) {
	      Scalar tColor = CV_RGB(192, 32, 32);
	      cv::Vec3b cColor;
	      cColor[0] = tColor[0]*glowFraction;
	      cColor[1] = tColor[1]*glowFraction;
	      cColor[2] = tColor[2]*glowFraction;
	      //gsl_matrix * mapcell = mapCellToPolygon(i, j);
	      //drawMapPolygon(mapcell, tColor);
	      //gsl_matrix_free(mapcell);
	      //line(objectMapViewerImage, cvp1, cvp1, tColor);
	      objectMapViewerImage.at<cv::Vec3b>(cvp1.y, cvp1.x) = 
		objectMapViewerImage.at<cv::Vec3b>(cvp1.y, cvp1.x) + cColor;
	    } else if ( (ikMap[i + mapWidth * j] == 2) ) {
	      Scalar tColor = CV_RGB(224, 64, 64);
	      cv::Vec3b cColor;
	      cColor[0] = tColor[0]*glowFraction;
	      cColor[1] = tColor[1]*glowFraction;
	      cColor[2] = tColor[2]*glowFraction;
	      //gsl_matrix * mapcell = mapCellToPolygon(i, j);
	      //drawMapPolygon(mapcell, tColor);
	      //gsl_matrix_free(mapcell);
	      //line(objectMapViewerImage, cvp1, cvp1, tColor);
	      objectMapViewerImage.at<cv::Vec3b>(cvp1.y, cvp1.x) = 
		objectMapViewerImage.at<cv::Vec3b>(cvp1.y, cvp1.x) + cColor;
	    }
	}
      }
    }
  }

  if (drawClearanceMap) { // draw clearanceMap 
    int ikMapRenderStride = 1;
    for (int i = 0; i < mapWidth; i+=ikMapRenderStride) {
      for (int j = 0; j < mapHeight; j+=ikMapRenderStride) {
	if ( cellIsSearched(i, j) ) {
	    ros::Duration longAgo = ros::Time::now() - objectMap[i + mapWidth * j].lastMappedTime;
	    double glowFraction = (1.0-glowBias)*(1.0-(min(max(longAgo.toSec(), 0.0), glowLast) / glowLast)) + glowBias;
	    if (!useGlow) {
	      glowFraction = 1.0;
	    }
	    double x=-1, y=-1;
	    mapijToxy(i, j, &x, &y);
	    cv::Point cvp1 = worldToPixel(objectMapViewerImage, 
	      mapXMin, mapXMax, mapYMin, mapYMax, x, y);
	    if ( (clearanceMap[i + mapWidth * j] == 1) ) {
	      Scalar tColor = CV_RGB(224, 224, 0);
	      cv::Vec3b cColor;
	      cColor[0] = tColor[0]*glowFraction;
	      cColor[1] = tColor[1]*glowFraction;
	      cColor[2] = tColor[2]*glowFraction;
	      //gsl_matrix * mapcell = mapCellToPolygon(i, j);
	      //drawMapPolygon(mapcell, CV_RGB(128, 128, 0));
	      //gsl_matrix_free(mapcell);
	      //line(objectMapViewerImage, cvp1, cvp1, tColor);
	      objectMapViewerImage.at<cv::Vec3b>(cvp1.y, cvp1.x) = 
		objectMapViewerImage.at<cv::Vec3b>(cvp1.y, cvp1.x) + cColor;
	    } else if ( (clearanceMap[i + mapWidth * j] == 2) ) {
	      Scalar tColor = CV_RGB(0, 224, 0);
	      cv::Vec3b cColor;
	      cColor[0] = tColor[0]*glowFraction;
	      cColor[1] = tColor[1]*glowFraction;
	      cColor[2] = tColor[2]*glowFraction;
	      //gsl_matrix * mapcell = mapCellToPolygon(i, j);
	      //drawMapPolygon(mapcell, CV_RGB(32, 128, 32));
	      //gsl_matrix_free(mapcell);
	      //line(objectMapViewerImage, cvp1, cvp1, tColor);
	      objectMapViewerImage.at<cv::Vec3b>(cvp1.y, cvp1.x) = 
		objectMapViewerImage.at<cv::Vec3b>(cvp1.y, cvp1.x) + cColor;
	    }
	}
      }
    }
  }

  { // drawMapSearchFence
    
    cv::Point outTop = worldToPixel(objectMapViewerImage, mapXMin, mapXMax, mapYMin, mapYMax, 
                                    mapSearchFenceXMin, mapSearchFenceYMin);
    cv::Point outBot = worldToPixel(objectMapViewerImage, mapXMin, mapXMax, mapYMin, mapYMax, 
                                    mapSearchFenceXMax, mapSearchFenceYMax);

    rectangle(objectMapViewerImage, outTop, outBot, 
              CV_RGB(255, 255, 0));
  }

  { // drawMapRejectFence
    
    cv::Point outTop = worldToPixel(objectMapViewerImage, mapXMin, mapXMax, mapYMin, mapYMax, 
                                    mapRejectFenceXMin, mapRejectFenceYMin);
    cv::Point outBot = worldToPixel(objectMapViewerImage, mapXMin, mapXMax, mapYMin, mapYMax, 
                                    mapRejectFenceXMax, mapRejectFenceYMax);

    rectangle(objectMapViewerImage, outTop, outBot, 
              CV_RGB(255, 0, 0));
  }

  // draw blue boxes
  for (int i = 0; i < blueBoxMemories.size(); i++) {
    BoxMemory memory = blueBoxMemories[i];
    string class_name = classLabels[memory.labeledClassIndex];
    
    double cx, cy;
    
    cx = memory.centroid.px;
    cy = memory.centroid.py;
    
    cv::Point objectPoint = worldToPixel(objectMapViewerImage, mapXMin, mapXMax, mapYMin, mapYMax, 
                                         cx, cy);
    objectPoint.x += 15;

    cv::Point outTop = worldToPixel(objectMapViewerImage, mapXMin, mapXMax, mapYMin, mapYMax, 
                                    memory.top.px, memory.top.py);
    cv::Point outBot = worldToPixel(objectMapViewerImage, mapXMin, mapXMax, mapYMin, mapYMax, 
                                    memory.bot.px, memory.bot.py);

    cv::Point outTopOriginal = outTop;
    cv::Point outBotOriginal = outBot;
    outTop.x = min(outTopOriginal.x, outBotOriginal.x);
    outTop.y = min(outTopOriginal.y, outBotOriginal.y);
    outBot.x = max(outTopOriginal.x, outBotOriginal.x);
    outBot.y = max(outTopOriginal.y, outBotOriginal.y);

    rectangle(objectMapViewerImage, outTop, outBot, 
              CV_RGB(0, 0, 255));

    putText(objectMapViewerImage, class_name, objectPoint, MY_FONT, 0.5, Scalar(255, 255, 255), 2.0);
  }
  
  { // drawRobot
    double radius = 20;
    cv::Point orientation_point = cv::Point(pxMax/2, pyMax/2 + radius);
    
    circle(objectMapViewerImage, center, radius, cv::Scalar(0, 0, 255));
    line(objectMapViewerImage, center, orientation_point, cv::Scalar(0, 0, 255));
  }
  { // drawHand
    eePose tp = rosPoseToEEPose(trueEEPose);
    double radius = 10;
    cv::Point handPoint = worldToPixel(objectMapViewerImage, mapXMin, mapXMax, mapYMin, mapYMax, 
                                       tp.px, tp.py);
    
    Eigen::Quaternionf handQuat(tp.qw, tp.qx, tp.qy, tp.qz);

    double rotated_magnitude = radius / sqrt(pow(pxMax - pxMin, 2) +  pow(pyMax - pyMin, 2)) * sqrt(pow(mapXMax - mapXMin, 2) + pow(mapYMax - mapYMin, 2));
    Eigen::Vector3f point(rotated_magnitude, 0, 0);
    Eigen::Vector3f rotated = handQuat * point;
    
    cv::Point orientation_point = worldToPixel(objectMapViewerImage, mapXMin, mapXMax, mapYMin, mapYMax,
                                               tp.px + rotated[0], 
                                               tp.py + rotated[1]);


    circle(objectMapViewerImage, handPoint, radius, cv::Scalar(0, 0, 255));

    line(objectMapViewerImage, handPoint, orientation_point, cv::Scalar(0, 0, 255));

  }

  if (0) { // drawBoxMemoryIntersectTests

    for (int i = 0; i < mapWidth; i++) {
      for (int j = 0; j < mapHeight; j++) {
        gsl_matrix * mapcell = mapCellToPolygon(i, j);

        for (int b_i = 0; b_i < blueBoxMemories.size(); b_i++) {
          BoxMemory b = blueBoxMemories[b_i];
          gsl_matrix * poly = boxMemoryToPolygon(b);
          cv::Scalar color;
          if (boxMemoryIntersectsMapCell(b, i, j)) {
            ros::Duration diff = objectMap[i + mapWidth * j].lastMappedTime - b.cameraTime;
            cout << "box time: " << b.cameraTime << endl;
            cout << "cell time: " << objectMap[i + mapWidth * j].lastMappedTime << endl;
            cout << "diff: " << diff << endl;
            if (diff < ros::Duration(2.0)) {
              drawMapPolygon(poly, color);
              drawMapPolygon(mapcell, CV_RGB(255, 255, 0));

            }
          }
     
          
          gsl_matrix_free(poly);
        }
        gsl_matrix_free(mapcell);
      }
    }
  }

  if (shouldIRender) {
    guardedImshow(objectMapViewerName, objectMapViewerImage);
  }


}

void drawMapPolygon(gsl_matrix * polygon_xy, cv::Scalar color) {
  for (size_t i = 0; i < polygon_xy->size2; i++) {
    int j = (i + 1) % polygon_xy->size2;
    gsl_vector_view p1 = gsl_matrix_column(polygon_xy, i);
    gsl_vector_view p2 = gsl_matrix_column(polygon_xy, j);
    double x1 = gsl_vector_get(&p1.vector, 0);
    double y1 = gsl_vector_get(&p1.vector, 1);


    double x2 = gsl_vector_get(&p2.vector, 0);
    double y2 = gsl_vector_get(&p2.vector, 1);
    double px1, px2, py1, py2;
    cv::Point cvp1 = worldToPixel(objectMapViewerImage, mapXMin, mapXMax, mapYMin, mapYMax, 
                                  x1, y1);
    cv::Point cvp2 = worldToPixel(objectMapViewerImage, mapXMin, mapXMax, mapYMin, mapYMax, 
                                  x2, y2);
    line(objectMapViewerImage, cvp1, cvp2, color);
  }

}


void renderCoreView() {
  Mat coreImage(800, 800, CV_64F);
  coreImage = 0.0*coreImage;

  cv::Scalar dataColor(192,192,192);
  cv::Scalar labelColor(160,160,160);

  cv::Point ciAnchor(10,50);
  putText(coreImage, "Current Registers: ", ciAnchor, MY_FONT, 0.5, labelColor, 1.0);

  char buf[256];
  cv::Point lAnchor(170,50);
  string lText = "";
  lText += "CI: ";
  if (current_instruction != NULL) {
    lText += current_instruction->name();
  } else {
    lText += "NULL";
  }
  putText(coreImage, lText, lAnchor, MY_FONT, 0.5, dataColor, 2.0);

  lAnchor.y += 20;
  lText = "";
  lText += "ZG: ";
  sprintf(buf, "%d", zero_g_toggle);
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

  int stackRowY = lAnchor.y + 40; 
  cv::Point csAnchor(10,stackRowY);
  putText(coreImage, "Call Stack: ", csAnchor, MY_FONT, 0.5, labelColor, 1.0);
  
  int instructionsPerRow = 1;
  int rowAnchorStep = 25;
  cv::Point rowAnchor(120,stackRowY);
  int insCount = 0; 

  int numCommandsToShow = 400;
  int lowerBound = max(int(call_stack.size() - numCommandsToShow), 0);
  insCount = lowerBound;

  while (insCount < call_stack.size()) {
    string outRowText;

    for (int rowCount = 0; (insCount < call_stack.size()) && (rowCount < instructionsPerRow); insCount++, rowCount++) {
      outRowText += call_stack[max(int(call_stack.size() - (insCount - lowerBound) - 1),0)]->name();
      outRowText += " ";
    }

    putText(coreImage, outRowText, rowAnchor, MY_FONT, 0.5, dataColor, 2.0);
    rowAnchor.y += rowAnchorStep;
    if (rowAnchor.y >= coreImage.rows) {
      break;
    }
  }

//  {
//    cv::Point text_anchor = cv::Point(0,coreImage.rows-1);
//    {
//      cv::Scalar backColor(0,0,0);
//      cv::Point outTop = cv::Point(text_anchor.x,text_anchor.y+1-35);
//      cv::Point outBot = cv::Point(text_anchor.x+200,text_anchor.y+1);
//      Mat vCrop = coreImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
//      vCrop = backColor;
//    }
//    char buff[256];
//    sprintf(buff, "Hz: %.2f", aveFrequency);
//    string fpslabel(buff);
//    putText(coreImage, fpslabel, text_anchor, MY_FONT, 1.0, Scalar(0,0,160), 1.0);
//  }
  if ( (rangeogramImage.rows > 0) && (rangeogramImage.cols > 0) ) {
    cv::Point text_anchor = cv::Point(0,rangeogramImage.rows-1);
    {
      cv::Scalar backColor(0,0,0);
      cv::Point outTop = cv::Point(text_anchor.x,text_anchor.y+1-35);
      cv::Point outBot = cv::Point(text_anchor.x+400,text_anchor.y+1);
      Mat vCrop = rangeogramImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
      vCrop = backColor;
    }
    {
      char buff[256];
      sprintf(buff, "            Hz: %.2f", aveFrequency);
      string fpslabel(buff);
      putText(rangeogramImage, fpslabel, text_anchor, MY_FONT, 1.0, Scalar(160,0,0), 1.0);
    }
    {
      char buff[256];
      sprintf(buff, "Hz: %.2f", aveFrequencyRange);
      string fpslabel(buff);
      putText(rangeogramImage, fpslabel, text_anchor, MY_FONT, 1.0, Scalar(0,0,160), 1.0);
    }
  }
  guardedImshow(rangeogramViewName, rangeogramImage);

  guardedImshow(coreViewName, coreImage);
}

void targetCallback(const geometry_msgs::Point& point) {

  if (!shouldIMiscCallback) {
    return;
  }

//  prevPx = pilotTarget.px;
//  prevPy = pilotTarget.py;
//
//  pilotTarget.px = point.x;
//  pilotTarget.py = point.y;
//  pilotTarget.pz = point.z;
//      
//  //cout << ">>received target<<" << endl;
//
//  timerCounter = 0;
//
//  if (lock_status == 2)
//    successive_lock_frames++;
//  else
//    successive_lock_frames = 0;
}

void pilotCallbackFunc(int event, int x, int y, int flags, void* userdata) {

  if (!shouldIMiscCallback) {
    return;
  }

  if ( event == EVENT_LBUTTONDOWN ) {
    cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    probeReticle.px = x;
    probeReticle.py = y;
    cout << "x: " << x << " y: " << y << " eeRange: " << eeRange << endl;
  } else if ( event == EVENT_RBUTTONDOWN ) {
    //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if  ( event == EVENT_MBUTTONDOWN ) {
    //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if ( event == EVENT_MOUSEMOVE ) {
    //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
  }
}

void graspMemoryCallbackFunc(int event, int x, int y, int flags, void* userdata) {

  if (!shouldIMiscCallback) {
    return;
  }

  if ( event == EVENT_LBUTTONDOWN ) {
    int bigX = x / rmiCellWidth;
    int bigY = y / rmiCellWidth;
    if ((bigX >= rmWidth) && (bigX < 2*rmWidth) && (bigY < rmWidth)) {
      // weight the grasp at a single point
      gmTargetY = (bigX-rmWidth);
      gmTargetX = bigY;

      // ATTN 5
      // XXX no check
//      for (int delX = -1; delX <= 1; delX++) {
//	for (int delY = -1; delY <= 1; delY++) {
//	  graspMemoryTries[(gmTargetX+delX) + (gmTargetY+delY)*rmWidth] = 1;
//	  graspMemoryPicks[(gmTargetX+delX) + (gmTargetY+delY)*rmWidth] = 1;
//	}
//      }
      graspMemoryTries[gmTargetX + gmTargetY*rmWidth + rmWidth*rmWidth*currentGraspGear] += 1;
      graspMemoryPicks[gmTargetX + gmTargetY*rmWidth + rmWidth*rmWidth*currentGraspGear] += 1;
    }
    pushWord("paintReticles"); // render reticle
    pushWord("drawMapRegisters"); // render register 1
    execute_stack = 1;

    cout << "Grasp Memory Left Click x: " << x << " y: " << y << " eeRange: " << eeRange << 
      " bigX: " << bigX << " bigY: " << bigY << " gmTargetX gmTargetY: " << gmTargetX << " " << gmTargetY << endl;
  } else if ( event == EVENT_RBUTTONDOWN ) {
    int bigX = x / rmiCellWidth;
    int bigY = y / rmiCellWidth;
    if ((bigX >= rmWidth) && (bigX < 2*rmWidth) && (bigY < rmWidth)) {
      graspMemoryTries[gmTargetX + gmTargetY*rmWidth + rmWidth*rmWidth*currentGraspGear] += 1;
    }
    pushWord("paintReticles"); // render reticle
    pushWord("drawMapRegisters"); // render register 1
    execute_stack = 1;

    cout << "Grasp Memory Left Click x: " << x << " y: " << y << " eeRange: " << eeRange << 
      " bigX: " << bigX << " bigY: " << bigY << " gmTargetX gmTargetY: " << gmTargetX << " " << gmTargetY << endl;
  } else if  ( event == EVENT_MBUTTONDOWN ) {
    int bigX = x / rmiCellWidth;
    int bigY = y / rmiCellWidth;
    if ((bigX >= rmWidth) && (bigX < 2*rmWidth) && (bigY < rmWidth)) {
      // reset to uniform failure
      for (int rx = 0; rx < rmWidth; rx++) {
	for (int ry = 0; ry < rmWidth; ry++) {
	  graspMemoryTries[rx + ry*rmWidth + rmWidth*rmWidth*currentGraspGear] = 10;
	  graspMemoryPicks[rx + ry*rmWidth + rmWidth*rmWidth*currentGraspGear] = 0;
	}
      }
    }
    pushWord("paintReticles"); // render reticle
    pushWord("drawMapRegisters"); // render register 1
    execute_stack = 1;

    cout << "Grasp Memory Left Click x: " << x << " y: " << y << " eeRange: " << eeRange << 
      " bigX: " << bigX << " bigY: " << bigY << " gmTargetX gmTargetY: " << gmTargetX << " " << gmTargetY << endl;
  } else if ( event == EVENT_MOUSEMOVE ) {
    //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
  }
}

void pilotInit() {
  eeForward = Eigen::Vector3d(1,0,0);

  if (0 == left_or_right_arm.compare("left")) {
    cout << "Possessing left arm..." << endl;
    beeHome = rssPoseL; //wholeFoodsPantryL;
    //beeHome = crane1left;
    //eepReg1 = crane1left;
    //eepReg2 = crane2left;
    //eepReg3 = crane3left;
    eepReg4 = rssPoseL; //beeLHome;
    defaultReticle = defaultLeftReticle;
    //defaultReticle = centerReticle;
    reticle = defaultReticle;

    double ystart = 0.1;
    double yend = 0.7;
    int numposes = 4;
    double ystep = (yend - ystart) / numposes;
    eePose pose1 = {.px = 0.65, .py = 0.0544691, .pz = -0.0582791,
                       .ox = 0, .oy = 0, .oz = 0,
                       .qx = 0, .qy = 1, .qz = 0, .qw = 0};
    for (int i = 0; i < numposes; i++) {
      deliveryPoses.push_back(pose1);
    }
    for (int i = 0; i < numposes; i++) {
      deliveryPoses[i].py = ystart + i * ystep;
    }

    rssPose = rssPoseL;
    ik_reset_eePose = rssPose;

    currentTableZ = leftTableZ;
    bagTableZ = leftTableZ;
    counterTableZ = leftTableZ;
    pantryTableZ  = leftTableZ;

    wholeFoodsBag1 = rssPoseL; //wholeFoodsBagL;
    wholeFoodsPantry1 = rssPoseL; //wholeFoodsPantryL;
    wholeFoodsCounter1 = rssPoseL; //wholeFoodsCounterL;

    eepReg1 = rssPoseL; //wholeFoodsBagL;
    eepReg2 = rssPoseL; //wholeFoodsPantryL;
    eepReg3 = rssPoseL; //wholeFoodsCounterL;


    // good fence values
    //mapSearchFenceXMin = -0.4;
    //mapSearchFenceXMax = 0.45;
    //mapSearchFenceYMin = 0.63;
    //mapSearchFenceYMax = 0.96;
    // full workspace
    //mapSearchFenceXMin = -0.75;
    //mapSearchFenceXMax = 1.00;
    //mapSearchFenceYMin = -1.25;
    //mapSearchFenceYMax = 1.25;
    mapSearchFenceXMin = -0.75;
    mapSearchFenceXMax = 1.0;
    mapSearchFenceYMin = -1.25;
    mapSearchFenceYMax = 1.25;

    //mapSearchFenceXMin = -0.75;
    //mapSearchFenceXMax = 1.0;
    //mapSearchFenceYMin = 0.45;
    //mapSearchFenceYMax = 1.25;

    //mapRejectFenceXMin = -0.4;
    //mapRejectFenceXMax = 0.45;
    //mapRejectFenceYMin = 0.63;
    //mapRejectFenceYMax = 0.96;
    mapRejectFenceXMin = mapSearchFenceXMin;
    mapRejectFenceXMax = mapSearchFenceXMax;
    mapRejectFenceYMin = mapSearchFenceYMin;
    mapRejectFenceYMax = mapSearchFenceYMax;

    // small fence values (for debugging)
    //mapSearchFenceXMin = 0;
    //mapRejectFenceXMin = 0;

    //mapSearchFenceXMax = 0.3;
    //mapRejectFenceXMax = 0.3;



    // left arm
    // (313, 163)
    vanishingPointReticle.px = 313;
    vanishingPointReticle.py = 163;
    probeReticle = vanishingPointReticle;

    // ATTN 16
    heightReticles[0] = defaultReticle;
    heightReticles[1] = defaultReticle;
    heightReticles[2] = defaultReticle;
    heightReticles[3] = defaultReticle;

    // values used for IJCAI
    //heightReticles[3].px = 323;
    //heightReticles[2].px = 326;
    //heightReticles[1].px = 331;
    //heightReticles[0].px = 347;
    //heightReticles[3].py = 137;
    //heightReticles[2].py = 132;
    //heightReticles[1].py = 118;
    //heightReticles[0].py = 72;
    heightReticles[3].px = 323;
    heightReticles[2].px = 326;
    heightReticles[1].px = 329;
    heightReticles[0].px = 336;

    heightReticles[3].py = 135;
    heightReticles[2].py = 128;
    heightReticles[1].py = 117;
    heightReticles[0].py = 94;

    /* color reticle init */
    /* XXX TODO needs recalibrating */
    //const int xCR[numCReticleIndeces] = {462, 450, 439, 428, 419, 410, 405, 399, 394, 389, 383, 381, 379, 378};
    xCR[0] = 462;
    xCR[1] = 450;
    xCR[2] = 439;
    xCR[3] = 428;
    xCR[4] = 419;
    xCR[5] = 410;
    xCR[6] = 405;
    xCR[7] = 399;
    xCR[8] = 394;
    xCR[9] = 389;
    xCR[10] = 383;
    xCR[11] = 381;
    xCR[12] = 379;
    xCR[13] = 378;

    /* left arm */
    //const int yCR[numCReticleIndeces] = {153, 153, 153, 153, 153, 154, 154, 154, 154, 154, 155, 155, 155, 155};
    yCR[0] = 153;
    yCR[1] = 153;
    yCR[2] = 153;
    yCR[3] = 153;
    yCR[4] = 153;
    yCR[5] = 154;
    yCR[6] = 154;
    yCR[7] = 154;
    yCR[8] = 154;
    yCR[9] = 154;
    yCR[10] = 155;
    yCR[11] = 155;
    yCR[12] = 155;
    yCR[13] = 155;

    /* lens correction */
    m_x_h[0] = 1.2;
    m_x_h[1] = 1.06;
    m_x_h[2] = 0.98;
    m_x_h[3] = 0.94;

    m_y_h[0] = 0.95;
    m_y_h[1] = 0.93;
    m_y_h[2] = 0.92;
    m_y_h[3] = 0.92;
  } else if (0 == left_or_right_arm.compare("right")) {
    cout << "Possessing right arm..." << endl;
    beeHome = rssPoseR;
    //beeHome = crane1right;
    //eepReg1 = crane1right;
    //eepReg2 = crane2right;
    //eepReg3 = crane3right;
    eepReg4 = rssPoseR; //beeRHome;
    defaultReticle = defaultRightReticle;
    //defaultReticle = centerReticle;
    reticle = defaultReticle;

    double ystart = -0.7;
    double yend = -0.1;
    int numposes = 4;
    double ystep = (yend - ystart) / numposes;
    eePose pose1 = {.px = 0.65, .py = 0.0544691, .pz = -0.0582791,
                       .ox = 0, .oy = 0, .oz = 0,
                       .qx = 0, .qy = 1, .qz = 0, .qw = 0};
    for (int i = 0; i < numposes; i++) {
      deliveryPoses.push_back(pose1);
    }
    for (int i = 0; i < numposes; i++) {
      deliveryPoses[i].py = ystart + i * ystep;
    }


    rssPose = rssPoseR;
    ik_reset_eePose = rssPose;


    currentTableZ = rightTableZ;
    bagTableZ = rightTableZ;
    counterTableZ = rightTableZ;
    pantryTableZ  = rightTableZ;

    wholeFoodsBag1 = rssPoseR; //wholeFoodsBagR;
    wholeFoodsPantry1 = rssPoseR; //wholeFoodsPantryR;
    wholeFoodsCounter1 = rssPoseR; //wholeFoodsCounterR;

    eepReg1 = rssPoseR; //wholeFoodsBagR;
    eepReg2 = rssPoseR; //wholeFoodsPantryR;
    eepReg3 = rssPoseR; //wholeFoodsCounterR;


    // raw fence values (from John estimating arm limits)
    // True EE Position (x,y,z): -0.329642 -0.77571 0.419954
    //True EE Position (x,y,z): 0.525236 -0.841226 0.217111


    //mapSearchFenceXMin = -0.25;
    //mapSearchFenceXMax = 0.45;
    //mapSearchFenceYMin = -0.86;
    //mapSearchFenceYMax = -0.78;
    //mapRejectFenceXMin = -0.1;
    //mapRejectFenceXMax = 0.7;
    //mapRejectFenceYMin = -1.06;
    //mapRejectFenceYMax = -0.58;

    //mapSearchFenceXMin = -0.4;
    //mapSearchFenceXMax = 0.45;
    //mapSearchFenceYMin = -0.96;
    //mapSearchFenceYMax = -0.63;
    // full workspace
    mapSearchFenceXMin = -0.75;
    mapSearchFenceXMax = 1.00;
    mapSearchFenceYMin = -1.25;
    mapSearchFenceYMax = 1.25;
    //mapSearchFenceXMin = -0.75;
    //mapSearchFenceXMax = 0.75;
    //mapSearchFenceYMin = -0.45;
    //mapSearchFenceYMax = 1.25;

    //mapRejectFenceXMin = -0.4;
    //mapRejectFenceXMax = 0.45;
    //mapRejectFenceYMin = -0.96;
    //mapRejectFenceYMax = -0.63;
    mapRejectFenceXMin = mapSearchFenceXMin;
    mapRejectFenceXMax = mapSearchFenceXMax;
    mapRejectFenceYMin = mapSearchFenceYMin;
    mapRejectFenceYMax = mapSearchFenceYMax;


    // right arm
    // (313, 185)
    vanishingPointReticle.px = 313;
    vanishingPointReticle.py = 185;
    probeReticle = vanishingPointReticle;

    // ATTN 16
    heightReticles[0] = defaultReticle;
    heightReticles[1] = defaultReticle;
    heightReticles[2] = defaultReticle;
    heightReticles[3] = defaultReticle;
    
    // values used for IJCAI
    //heightReticles[3].px = 319;
    //heightReticles[2].px = 323;
    //heightReticles[1].px = 328;
    //heightReticles[0].px = 341;
    //heightReticles[3].py = 170;
    //heightReticles[2].py = 159;
    //heightReticles[1].py = 145;
    //heightReticles[0].py = 100;
    heightReticles[3].px = 314;
    heightReticles[2].px = 317;
    heightReticles[1].px = 320;
    heightReticles[0].px = 328;

    heightReticles[3].py = 154;
    heightReticles[2].py = 149;
    heightReticles[1].py = 139;
    heightReticles[0].py = 120;

    /* color reticle init */
    /* XXX TODO needs recalibrating */
    //const int xCR[numCReticleIndeces] = {462, 450, 439, 428, 419, 410, 405, 399, 394, 389, 383, 381, 379, 378};
    xCR[0] = 462;
    xCR[1] = 450;
    xCR[2] = 439;
    xCR[3] = 428;
    xCR[4] = 419;
    xCR[5] = 410;
    xCR[6] = 405;
    xCR[7] = 399;
    xCR[8] = 394;
    xCR[9] = 389;
    xCR[10] = 383;
    xCR[11] = 381;
    xCR[12] = 379;
    xCR[13] = 378;

    /* right arm */
    //const int yCR[numCReticleIndeces] = {153, 153, 153, 153, 153, 154, 154, 154, 154, 154, 155, 155, 155, 155};
    yCR[0] = 153;
    yCR[1] = 153;
    yCR[2] = 153;
    yCR[3] = 153;
    yCR[4] = 153;
    yCR[5] = 154;
    yCR[6] = 154;
    yCR[7] = 154;
    yCR[8] = 154;
    yCR[9] = 154;
    yCR[10] = 155;
    yCR[11] = 155;
    yCR[12] = 155;
    yCR[13] = 155;

    /* lens correction */
    m_x_h[0] = 1.18;
    m_x_h[1] = 1.12;
    m_x_h[2] = 1.09;
    m_x_h[3] = 1.08;

    m_y_h[0] = 1.16;
    m_y_h[1] = 1.17;
    m_y_h[2] = 1.16;
    m_y_h[3] = 1.2;
  } else {
    cout << "Invalid chirality: " << left_or_right_arm << ".  Exiting." << endl;
    exit(0);
  }
  pilotTarget = beeHome;
  lastGoodEEPose = beeHome;
  currentEEPose = beeHome;

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

      // ATTN 6 change initialization to determine speed of learning
      for (int tGG = 0; tGG < totalGraspGears/2; tGG++) {
	graspMemoryTries[rx + ry*rmWidth + rmWidth*rmWidth*tGG] = 1;
	graspMemoryPicks[rx + ry*rmWidth + rmWidth*rmWidth*tGG] = 1;
      }
    }
  }

  rangemapImage = Mat(rmiHeight, 3*rmiWidth, CV_8UC3);
  graspMemoryImage = Mat(rmiHeight, 2*rmiWidth, CV_8UC3);
  graspMemorySampleImage = Mat(2*rmiHeight, 2*rmiWidth, CV_8UC3);
  heightMemorySampleImage = Mat(hmiHeight, 2*hmiWidth, CV_8UC3);

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
    //Eigen::Quaternionf gear0offset(0.0, 0.020183, 0.031996, 0.0167228); // from TF, accounts for upside down rotation of crane2
    Eigen::Quaternionf gear0offset(0.0, 0.023, 0.023, 0.0167228); // z is from TF, good for depth alignment

    if (0 == left_or_right_arm.compare("left")) {
      gear0offset = Eigen::Quaternionf(0.0, 0.03, 0.023, 0.0167228); // z is from TF, good for depth alignment
    } else if (0 == left_or_right_arm.compare("right")) {
      gear0offset = Eigen::Quaternionf(0.0, 0.023, 0.023, 0.0167228); // z is from TF, good for depth alignment
    }

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

int shouldIPick(int classToPick) {

  int toReturn = 0;

  // 6 is black tape
  // 12 is green block
  // 2 is greenPepper 
  // 9 is chile
  // 10 is cucumber
//  if (
//      (classToPick == 2) ||
//      (classToPick == 9) ||
//      (classToPick == 10)||
//      (classToPick == 12)||
//      (classToPick == 6)
//    ) {
//    toReturn = 1;
//  }
  
  toReturn = (classToPick == targetClass);

  cout << classToPick << " " << targetClass << " " << toReturn;

  return toReturn;
}

int getLocalGraspGear(int globalGraspGearIn) {
  // ATTN 7
  // diagnostic line
  //Quaternionf eeqform(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
  // correct line
  Quaternionf eeqform(bestOrientationEEPose.qw, bestOrientationEEPose.qx, bestOrientationEEPose.qy, bestOrientationEEPose.qz);

  Quaternionf gear1Orient = getGGRotation(0);
  Quaternionf rel = eeqform * gear1Orient.inverse();
  Quaternionf ex(0,1,0,0);
  Quaternionf zee(0,0,0,1);
  

  Quaternionf result = rel * ex * rel.conjugate();
  Quaternionf thumb = rel * zee * rel.conjugate();
  double aY = result.y();
  double aX = result.x();

  // ATTN 1
  // this is here to get the signs right
  aX = -aX;

  //double angle = atan2(aY, aX)*180.0/3.1415926;
  // no degrees here
  // ATTN 22
  //double angle = atan2(aY, aX);
  double angle = vectorArcTan(aY, aX);
  // no inversion necessary
  //angle = -angle;
  
  double deltaGG = floor(angle * totalGraspGears / (2.0 * 3.1415926));
  int ggToReturn = (totalGraspGears + globalGraspGearIn + int(deltaGG)) % (totalGraspGears / 2);

  //cout << "getLocalGraspGear angle deltaGG ggToReturn: " << angle << " " << deltaGG << " " << ggToReturn << endl;

  assert(getGlobalGraspGear(ggToReturn) == globalGraspGearIn);

  return ggToReturn;
}

int getGlobalGraspGear(int localGraspGearIn) {
  // ATTN 7
  // diagnostic line
  //Quaternionf eeqform(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
  // correct line
  Quaternionf eeqform(bestOrientationEEPose.qw, bestOrientationEEPose.qx, bestOrientationEEPose.qy, bestOrientationEEPose.qz);

  Quaternionf gear1Orient = getGGRotation(0);
  Quaternionf rel = eeqform * gear1Orient.inverse();
  Quaternionf ex(0,1,0,0);
  Quaternionf zee(0,0,0,1);
  

  Quaternionf result = rel * ex * rel.conjugate();
  Quaternionf thumb = rel * zee * rel.conjugate();
  double aY = result.y();
  double aX = result.x();

  // ATTN 1
  // this is here to get the signs right
  aX = -aX;

  //double angle = atan2(aY, aX)*180.0/3.1415926;
  // no degrees here
  // ATTN 22
  //double angle = atan2(aY, aX);
  double angle = vectorArcTan(aY, aX);
  // inversion to convert to global
  angle = -angle;
  
  double deltaGG = floor(angle * totalGraspGears / (2.0 * 3.1415926));
  // we are doing ceiling by taking the floor and then adding one, the inverse of getLocalGraspGear.
  int ggToReturn = (totalGraspGears + localGraspGearIn + 1 + int(deltaGG)) % (totalGraspGears / 2);

  //assert(getLocalGraspGear(ggToReturn) == localGraspGearIn);

  return ggToReturn;
}

void changeTargetClass(int newTargetClass) {
  targetClass = newTargetClass;
  focusedClass = targetClass;
  focusedClassLabel = classLabels[focusedClass];
  cout << "class " << targetClass << " " << classLabels[targetClass] << endl;
  execute_stack = 1;	


  pushWord(1179709); // loadMarginalHeightMemory


  pushWord("drawMapRegisters"); // render register 1
  // ATTN 10
  //pushWord(196360); // loadPriorGraspMemory
  //pushWord(1179721); // set graspMemories from classGraspMemories
  switch (currentPickMode) {
  case STATIC_PRIOR:
    {
      pushWord(196360); // loadPriorGraspMemory
    }
    break;
  case LEARNING_ALGORITHMC:
  case LEARNING_SAMPLING:
    {
      pushWord(1179721); // set graspMemories from classGraspMemories
    }
    break;
  case STATIC_MARGINALS:
    {
      pushWord(1179721); // set graspMemories from classGraspMemories
    }
    break;
  default:
    {
      assert(0);
    }
    break;
  }
  
  switch (currentBoundingBoxMode) {
  case STATIC_PRIOR:
    {
      pushWord(1244936); // loadPriorHeightMemory
    }
    break;
  case LEARNING_ALGORITHMC:
  case LEARNING_SAMPLING:
    {
      pushWord(1245289); // set heightMemories from classHeightMemories
    }
    break;
  case STATIC_MARGINALS:
    {
      //cout << "Pushing set heightMemories from classHeightMemories" << endl;
      pushWord(1245289); // set heightMemories from classHeightMemories
    }
    break;
  case MAPPING:
    {
    }
    break;
  default:
    {
      assert(0);
    }
    break;
  }
}

void guardGraspMemory() {

  {
    if (classGraspMemoryTries1.size() <= focusedClass) {
      classGraspMemoryTries1.resize(focusedClass + 1);
    }
    if (classGraspMemoryPicks1.size() <= focusedClass) {
      classGraspMemoryPicks1.resize(focusedClass + 1);
    }

    if (classGraspMemoryTries2.size() <= focusedClass) {
      classGraspMemoryTries2.resize(focusedClass + 1);
    }
    if (classGraspMemoryPicks2.size() <= focusedClass) {
      classGraspMemoryPicks2.resize(focusedClass + 1);
    }

    if (classGraspMemoryTries3.size() <= focusedClass) {
      classGraspMemoryTries3.resize(focusedClass + 1);
    }
    if (classGraspMemoryPicks3.size() <= focusedClass) {
      classGraspMemoryPicks3.resize(focusedClass + 1);
    }

    if (classGraspMemoryTries4.size() <= focusedClass) {
      classGraspMemoryTries4.resize(focusedClass + 1);
    }
    if (classGraspMemoryPicks4.size() <= focusedClass) {
      classGraspMemoryPicks4.resize(focusedClass + 1);
    }

  }

  {
    bool loadPrior = false;
    if (!((classGraspMemoryTries1[focusedClass].rows > 1) && (classGraspMemoryTries1[focusedClass].cols > 1) &&
	(classGraspMemoryPicks1[focusedClass].rows > 1) && (classGraspMemoryPicks1[focusedClass].cols > 1) )) {
      classGraspMemoryTries1[focusedClass] = Mat(rmWidth, rmWidth, CV_64F);
      classGraspMemoryPicks1[focusedClass] = Mat(rmWidth, rmWidth, CV_64F);
      loadPrior = true;
    }
    if (!((classGraspMemoryTries2[focusedClass].rows > 1) && (classGraspMemoryTries2[focusedClass].cols > 1) &&
	(classGraspMemoryPicks2[focusedClass].rows > 1) && (classGraspMemoryPicks2[focusedClass].cols > 1) )) {
      classGraspMemoryTries2[focusedClass] = Mat(rmWidth, rmWidth, CV_64F);
      classGraspMemoryPicks2[focusedClass] = Mat(rmWidth, rmWidth, CV_64F);
      loadPrior = true;
    }
    if (!((classGraspMemoryTries3[focusedClass].rows > 1) && (classGraspMemoryTries3[focusedClass].cols > 1) &&
	(classGraspMemoryPicks3[focusedClass].rows > 1) && (classGraspMemoryPicks3[focusedClass].cols > 1) )) {
      classGraspMemoryTries3[focusedClass] = Mat(rmWidth, rmWidth, CV_64F);
      classGraspMemoryPicks3[focusedClass] = Mat(rmWidth, rmWidth, CV_64F);
      loadPrior = true;
    }
    if (!((classGraspMemoryTries4[focusedClass].rows > 1) && (classGraspMemoryTries4[focusedClass].cols > 1) &&
	(classGraspMemoryPicks4[focusedClass].rows > 1) && (classGraspMemoryPicks4[focusedClass].cols > 1) )) {
      classGraspMemoryTries4[focusedClass] = Mat(rmWidth, rmWidth, CV_64F);
      classGraspMemoryPicks4[focusedClass] = Mat(rmWidth, rmWidth, CV_64F);
      loadPrior = true;
    }
    if (loadPrior) {
      loadPriorGraspMemory(ANALYTIC_PRIOR);
    }
  }

}

void guardHeightMemory() {
  if (focusedClass == -1) {
    ROS_ERROR_STREAM("Focused class not initialized! " << focusedClass);
  }
  if (classHeightMemoryTries.size() <= focusedClass) {
    classHeightMemoryTries.resize(focusedClass + 1);
  }
  if (classHeightMemoryPicks.size() <= focusedClass) {
    classHeightMemoryPicks.resize(focusedClass + 1);
  }
  if (!((classHeightMemoryTries[focusedClass].rows > 1) && (classHeightMemoryTries[focusedClass].cols == 1) &&
	(classHeightMemoryPicks[focusedClass].rows > 1) && (classHeightMemoryPicks[focusedClass].cols == 1) )) {
    classHeightMemoryTries[focusedClass] = Mat(hmWidth, 1, CV_64F);
    classHeightMemoryPicks[focusedClass] = Mat(hmWidth, 1, CV_64F);
    loadPriorHeightMemory(ANALYTIC_PRIOR);
  }
}

int calibrateGripper() {
  for (int i = 0; i < 10; i++) {
    int return_value = doCalibrateGripper();
    if (return_value == 0) {
      return return_value;
    }
  }
  cout << "Gripper could not calibrate!" << endl;
  pushWord('Y'); // pause stack execution
  pushCopies("beep", 15); // beep
  return -1;
}
int doCalibrateGripper() {
  int return_value;
  if (0 == left_or_right_arm.compare("left")) {
    return_value =system("bash -c \"echo -e \'c\003\' | rosrun baxter_examples gripper_keyboard.py\"");
  } else if (0 == left_or_right_arm.compare("right")) {
    return_value = system("bash -c \"echo -e \'C\003\' | rosrun baxter_examples gripper_keyboard.py\"");
  }
  if (return_value != 0) {
    cout << "Error running calibrate: " << return_value << endl;
  }
  return return_value;
}

void convertGlobalGraspIdxToLocal(const int rx, const int ry, 
                                  int * localX, int * localY) {
  // COMPLETELY UNTESTED
  assert(0);
  // find global coordinate of current point
  double thX = (rx-rmHalfWidth) * rmDelta;
  double thY = (ry-rmHalfWidth) * rmDelta;
  // transform it into local coordinates
  double unangle = -bestOrientationAngle;
  double unscale = 1.0;
  Point uncenter = Point(0, 0);
  Mat un_rot_mat = getRotationMatrix2D(uncenter, unangle, unscale);
  Mat toUn(3,1,CV_64F);
  toUn.at<double>(0,0)=thX;
  toUn.at<double>(1,0)=thY;
  toUn.at<double>(2,0)=1.0;
  Mat didUn = un_rot_mat*toUn;
  double localThX = didUn.at<double>(0,0);
  double localThY = didUn.at<double>(1,0);
  *localX = ((localThX)/rmDelta) + rmHalfWidth; 
  *localY = ((localThY)/rmDelta) + rmHalfWidth; 

}

void convertLocalGraspIdxToGlobal(const int localX, const int localY,
                                  int * rx, int * ry) {
  // find local coordinate of current point
  double thX = (localX-rmHalfWidth) * rmDelta;
  double thY = (localY-rmHalfWidth) * rmDelta;
  // transform it into local coordinates
  double unangle = bestOrientationAngle;
  double unscale = 1.0;
  Point uncenter = Point(0, 0);
  Mat un_rot_mat = getRotationMatrix2D(uncenter, unangle, unscale);
  Mat toUn(3,1,CV_64F);
  toUn.at<double>(0,0)=thX;
  toUn.at<double>(1,0)=thY;
  toUn.at<double>(2,0)=1.0;
  Mat didUn = un_rot_mat*toUn;
  double localThX = didUn.at<double>(0,0);
  double localThY = didUn.at<double>(1,0);
  *rx = (int) round(((localThX)/rmDelta) + rmHalfWidth); 
  *ry = (int) round(((localThY)/rmDelta) + rmHalfWidth); 
}


void loadSampledGraspMemory() {
  ROS_INFO("Loading sampled grasp memory.");
  for (int tGG = 0; tGG < totalGraspGears/2; tGG++) {
    for (int rx = 0; rx < rmWidth; rx++) {
      for (int ry = 0; ry < rmWidth; ry++) {
        

	// ATTN 19 this isn't quite Thompson sampling...
	//   regularization.
        int i = rx + ry * rmWidth + rmWidth*rmWidth*tGG;
        double nsuccess = pickEccentricity * (graspMemoryPicks[i]);
        double nfailure = pickEccentricity * (graspMemoryTries[i] - graspMemoryPicks[i]);
        graspMemorySample[i] = rk_beta(&random_state, 
                                       nsuccess + 1, 
                                       nfailure + 1);
      }
    }
  }
}


void loadMarginalGraspMemory() {
  ROS_INFO("Loading marginal grasp memory.");
  for (int tGG = 0; tGG < totalGraspGears/2; tGG++) {
    for (int rx = 0; rx < rmWidth; rx++) {
      for (int ry = 0; ry < rmWidth; ry++) {
        int i = rx + ry * rmWidth + rmWidth*rmWidth*tGG;
        double nsuccess = graspMemoryPicks[i];
        double nfailure = graspMemoryTries[i] - graspMemoryPicks[i];
        graspMemorySample[i] = (nsuccess + 1) / (nsuccess + nfailure + 2);
      }
    }
  }
}

void loadPriorGraspMemory(priorType prior) {
  ROS_INFO("Loading prior grasp memory.");
  double max_range_value = -VERYBIGNUMBER;
  double min_range_value = VERYBIGNUMBER;


  for (int tGG = 0; tGG < totalGraspGears/2; tGG++) {
    prepareGraspFilter(tGG);
    loadLocalTargetClassRangeMap(rangeMapReg3, rangeMapReg4);
    applyGraspFilter(rangeMapReg3, rangeMapReg4);

    for (int rx = 0; rx < rmWidth; rx++) {
      for (int ry = 0; ry < rmWidth; ry++) {
        int i = rx + ry * rmWidth + rmWidth*rmWidth*tGG;
        graspMemoryReg1[i] = rangeMapReg3[rx + ry * rmWidth];
        if (graspMemoryReg1[i] < min_range_value) {
          min_range_value = graspMemoryReg1[i];
        }
        if (graspMemoryReg1[i] > max_range_value) {
          max_range_value = graspMemoryReg1[i];
        }
      }
    }
  }

  // ATTN 18
  // make the grasp gears symmetric so we can reliably 
  // populate a fixed number of grasps.
  int symmetrizeGraspGears = 0;
  if (symmetrizeGraspGears) {
    for (int rx = 0; rx < rmWidth; rx++) {
      for (int ry = 0; ry < rmWidth; ry++) {
	double minAtThisXY = INFINITY;
	for (int tGG = 0; tGG < totalGraspGears/2; tGG++) {
	  int i = rx + ry * rmWidth + rmWidth*rmWidth*tGG;
	  if (graspMemoryReg1[i] < minAtThisXY) {
	    minAtThisXY = graspMemoryReg1[i];
	  }
	}
	for (int tGG = 0; tGG < totalGraspGears/2; tGG++) {
	  int i = rx + ry * rmWidth + rmWidth*rmWidth*tGG;
	  graspMemoryReg1[i] = minAtThisXY; 
	}
      }
    }
  }

  for (int tGG = 0; tGG < totalGraspGears/2; tGG++) {
    for (int rx = 0; rx < rmWidth; rx++) {
      for (int ry = 0; ry < rmWidth; ry++) {
        int i = rx + ry * rmWidth + rmWidth*rmWidth*tGG;
        graspMemoryReg1[i] = (max_range_value - graspMemoryReg1[i]) / (max_range_value - min_range_value);
      }
    }
  }

  // make everything peakier
  // for (int tGG = 0; tGG < totalGraspGears/2; tGG++) {
  //   for (int rx = 0; rx < rmWidth; rx++) {
  //     for (int ry = 0; ry < rmWidth; ry++) {
  //       int i = rx + ry * rmWidth + rmWidth*rmWidth*tGG;
  //       graspMemoryReg1[i] = pow(graspMemoryReg1[i], 4);
  //     }
  //   }
  // }
  
  std::vector<double> sorted;
  for (int tGG = 0; tGG < totalGraspGears/2; tGG++) {
    for (int rx = 0; rx < rmWidth; rx++) {
      for (int ry = 0; ry < rmWidth; ry++) {
        int i = rx + ry * rmWidth + rmWidth*rmWidth*tGG;
        sorted.push_back(graspMemoryReg1[i]);
      }
    }
  }
  
  std::sort (sorted.begin(), sorted.end());

  int numLocationsToTry = 10;
  double threshold = sorted[sorted.size() - 4*numLocationsToTry];

  for (int tGG = 0; tGG < totalGraspGears/2; tGG++) {
    for (int rx = 0; rx < rmWidth; rx++) {
      for (int ry = 0; ry < rmWidth; ry++) {
        int i = rx + ry * rmWidth + rmWidth*rmWidth*tGG;
        if (graspMemoryReg1[i] < threshold && prior == UNIFORM_PRIOR) {
          graspMemoryReg1[i] = 0;
        }
      }
    }
  }


  for (int tGG = 0; tGG < totalGraspGears/2; tGG++) {
    for (int rx = 0; rx < rmWidth; rx++) {
      for (int ry = 0; ry < rmWidth; ry++) {
        int i = rx + ry * rmWidth + rmWidth*rmWidth*tGG;
        double mu = graspMemoryReg1[i];
        double nfailure;
        //double eccentricity = 3.0;//100;
	// ATTN 19
        //double nsuccess = (eccentricity * mu);
        double nsuccess = mu;

        if (mu == 0) {
          //nfailure = VERYBIGNUMBER;
          nfailure = 1;
        } else {
          if (prior == UNIFORM_PRIOR) {
            nfailure = 0;
          } else if (prior == ANALYTIC_PRIOR) {
	    // ATTN 19
            //nfailure = round(eccentricity * (1 - mu));
            //nfailure = eccentricity * (1 - mu);
            nfailure = (1 - mu);
          } else {
            cout << "Invalid prior: " << prior << endl;
            assert(0);
          }
        }

	if (prior == UNIFORM_PRIOR) {
	  nsuccess = 0;
	}
	graspMemoryPicks[i] = nsuccess;
	graspMemoryTries[i] = nsuccess + nfailure;
      }
    }
  }
}

void loadMarginalHeightMemory() {
  //ROS_INFO("Loading marginal height memory.");
  for (int i = 0; i < hmWidth; i++) {
    double nsuccess = heightMemoryPicks[i];
    double nfailure = heightMemoryTries[i] - heightMemoryPicks[i];
    heightMemorySample[i] = (nsuccess + 1) / (nsuccess + nfailure + 2);
  }
}
 
void loadSampledHeightMemory() {
  ROS_INFO("Loading sampled height memory.");
  for (int i = 0; i < hmWidth; i++) {
    double nsuccess = heightEccentricity * (heightMemoryPicks[i]);
    double nfailure = heightEccentricity * (heightMemoryTries[i] - heightMemoryPicks[i]);
    heightMemorySample[i] = rk_beta(&random_state, 
                                    nsuccess + 1, 
                                    nfailure + 1);
  }
  drawHeightMemorySample();
}

double convertHeightIdxToGlobalZ(int heightIdx) {
  double scaledHeight = (double(heightIdx)/double(hmWidth-1)) * (maxHeight - minHeight);
  double scaledTranslatedHeight = scaledHeight + minHeight;
  double tableTranslatedScaledHeight = scaledTranslatedHeight + currentTableZ;
  return tableTranslatedScaledHeight;
}

int convertHeightGlobalZToIdx(double globalZ) {
  double scaledHeight = (globalZ - currentTableZ) / (maxHeight - minHeight);
  int heightIdx = floor(scaledHeight * (hmWidth - 1));
}

void testHeightConversion() {
  for (int i = 0; i < hmWidth; i++) {
    double height = convertHeightIdxToGlobalZ(i);
    int newIdx = convertHeightGlobalZToIdx(height);
    cout << "i: " << i << " height: " << height << " newIdx: " << newIdx << endl;
    //assert(newIdx == i);
  }
}

void loadPriorHeightMemory(priorType prior) {
  for (int i = 0; i < hmWidth; i++) {
    heightMemoryPicks[i] = 1;
    heightMemoryTries[i] = 1;
  }
  if (prior == ANALYTIC_PRIOR) {
    heightMemoryPicks[1] = 1;
    heightMemoryTries[1] = 1;
  }
}

void drawHeightMemorySample() {
  
  {
    double max_value = -VERYBIGNUMBER;
    int max_i=0, max_ry=0, max_rx=0;
    for (int i = 0; i < hmWidth; i++) {
      if (heightMemorySample[i] > max_value) {
	max_value = heightMemorySample[i];
	max_i = i;
	max_rx = hmWidth - 1 - max_i;
	max_ry = 0;
      }
      {
	int ry = 0;
	int rx = hmWidth - 1 - i;
	double blueIntensity = 255 * heightMemorySample[i];
	double greenIntensity = 255 * heightMemorySample[i];
	double redIntensity = 255 * heightMemorySample[i];
	//cout << "Height Memory Sample: " << "rx: " << rx << " ry: " << ry << " tGG:" << tGG << "sample: " << heightMemorySample[i] << endl;
	cv::Scalar color(ceil(blueIntensity),ceil(greenIntensity),ceil(redIntensity));
	
	cv::Point outTop = cv::Point((ry)*hmiCellWidth,(rx)*hmiCellWidth);
	cv::Point outBot = cv::Point(((ry)+1)*hmiCellWidth,((rx)+1)*hmiCellWidth);
	Mat vCrop = heightMemorySampleImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
	vCrop = color;
      }
    }
    {
      // draw the max
      char buff[256];
      cv::Point text_anchor = cv::Point((max_ry) * hmiCellWidth - 5, 
					(max_rx + 1) * hmiCellWidth);
      sprintf(buff, "x");
      putText(heightMemorySampleImage, buff, text_anchor, MY_FONT, 7, 
	      Scalar(0,0,255), 2);
    }
  }
  {
    double max_value = -VERYBIGNUMBER;
    int max_i=0, max_ry=0, max_rx=0;
    for (int i = 0; i < hmWidth; i++) {
      double thisMarginal = (heightMemoryPicks[i]+1)/(heightMemoryTries[i]+2);
      if (thisMarginal > max_value) {
	max_value = thisMarginal;
	max_i = i;
	max_rx = hmWidth - 1 - max_i;
	max_ry = 0;
      }
      {
	int ry = 0;
	int rx = hmWidth - 1 - i;
	double blueIntensity = 255 * thisMarginal;
	double greenIntensity = 255 * thisMarginal;
	double redIntensity = 255 * thisMarginal;
	//cout << "Height Memory Marginal: " << "rx: " << rx << " ry: " << ry << " tGG:" << tGG << "sample: " << thisMarginal << endl;
	cv::Scalar color(ceil(blueIntensity),ceil(greenIntensity),ceil(redIntensity));
	
	cv::Point outTop = cv::Point((ry+1)*hmiCellWidth,(rx)*hmiCellWidth);
	cv::Point outBot = cv::Point(((ry+1)+1)*hmiCellWidth,((rx)+1)*hmiCellWidth);
	Mat vCrop = heightMemorySampleImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
	vCrop = color;
      }
    }
    {
      // draw the max
      char buff[256];
      cv::Point text_anchor = cv::Point((max_ry+1) * hmiCellWidth, 
					(max_rx + 1) * hmiCellWidth);
      sprintf(buff, "x");
      putText(heightMemorySampleImage, buff, text_anchor, MY_FONT, 7, 
	      Scalar(0,0,255), 2);
    }
  }
}

void copyHeightMemoryTriesToClassHeightMemoryTries() {
  guardHeightMemory();
  for (int i = 0; i < hmWidth; i++) {
    classHeightMemoryTries[focusedClass].at<double>(i,0) = heightMemoryTries[i];
    classHeightMemoryPicks[focusedClass].at<double>(i,0) = heightMemoryPicks[i];
  }
}

void estimateGlobalGraspGear() {
  ROS_INFO("Estimating global grasp gear.");
  double max_range_value = -VERYBIGNUMBER;
  double min_range_value = VERYBIGNUMBER;
  int eMinGG = 0;

  for (int tGG = 0; tGG < totalGraspGears/2; tGG++) {
    prepareGraspFilter(tGG);
    loadGlobalTargetClassRangeMap(rangeMapReg3, rangeMapReg4);
    applyGraspFilter(rangeMapReg3, rangeMapReg4);

    int rx = maxX;
    int ry = maxY;

    int i = rx + ry * rmWidth + rmWidth*rmWidth*tGG;
    graspMemoryReg1[i] = rangeMapReg3[rx + ry * rmWidth];
    if (graspMemoryReg1[i] < min_range_value) {
      min_range_value = graspMemoryReg1[i];
      eMinGG = tGG;
    }
    if (graspMemoryReg1[i] > max_range_value) {
      max_range_value = graspMemoryReg1[i];
    }
  }

  maxGG = eMinGG;
  localMaxGG = getLocalGraspGear(eMinGG);
}

void drawMapRegisters() {
  {
    double minDepth = VERYBIGNUMBER;
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
          denom = VERYBIGNUMBER;
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
    double minDepth = VERYBIGNUMBER;
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
          denom = VERYBIGNUMBER;
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
    double minDepth = VERYBIGNUMBER;
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
          denom = VERYBIGNUMBER;
        double intensity = 255 * (maxDepth - hiRangeMap[rx + ry*hrmWidth]) / denom;
        hiRangemapImage.at<cv::Vec3b>(rx,ry) = cv::Vec3b(0,0,ceil(intensity));
      }
    }
  }
  {
    double minDepth = VERYBIGNUMBER;
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
          denom = VERYBIGNUMBER;
        double intensity = 255 * (maxDepth - hiRangeMapReg1[rx + ry*hrmWidth]) / denom;
        hiRangemapImage.at<cv::Vec3b>(rx,ry+hrmWidth) = cv::Vec3b(0,0,ceil(intensity));
      }
    }
  }
  {
    double minDepth = VERYBIGNUMBER;
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
          denom = VERYBIGNUMBER;
        double intensity = 255 * (maxDepth - hiRangeMapReg2[rx + ry*hrmWidth]) / denom;
        hiRangemapImage.at<cv::Vec3b>(rx,ry+2*hrmWidth) = cv::Vec3b(0,0,ceil(intensity));
      }
    }
  }

  // draw grasp memory window
  {
    {
      double minDepth = VERYBIGNUMBER;
      double maxDepth = 0;
      for (int rx = 0; rx < rmWidth; rx++) {
        for (int ry = 0; ry < rmWidth; ry++) {
          minDepth = min(minDepth, graspMemoryTries[rx + ry*rmWidth + rmWidth*rmWidth*currentGraspGear]);
          maxDepth = max(maxDepth, graspMemoryTries[rx + ry*rmWidth + rmWidth*rmWidth*currentGraspGear]);
        }
      }
      for (int rx = 0; rx < rmWidth; rx++) {
        for (int ry = 0; ry < rmWidth; ry++) {
          double denom = max(1.0,maxDepth);
          if (denom <= EPSILON)
            denom = VERYBIGNUMBER;
          double blueIntensity = 128 * (graspMemoryPicks[rx + ry*rmWidth + rmWidth*rmWidth*currentGraspGear]) / denom;
          double redIntensity = 128 * (graspMemoryTries[rx + ry*rmWidth + rmWidth*rmWidth*currentGraspGear] - graspMemoryPicks[rx + ry*rmWidth + rmWidth*rmWidth*currentGraspGear]) / denom;
          cv::Scalar backColor(ceil(blueIntensity),0,ceil(redIntensity));
          cv::Point outTop = cv::Point((ry)*rmiCellWidth,rx*rmiCellWidth);
          cv::Point outBot = cv::Point(((ry)+1)*rmiCellWidth,(rx+1)*rmiCellWidth);
          Mat vCrop = graspMemoryImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
          vCrop = backColor;
        }
      }
    }
    if ((targetClass > -1) && (classRangeMaps[targetClass].rows > 1) && (classRangeMaps[targetClass].cols > 1)) {
      double minDepth = VERYBIGNUMBER;
      double maxDepth = 0;
      for (int rx = 0; rx < rmWidth; rx++) {
        for (int ry = 0; ry < rmWidth; ry++) {

          minDepth = min(minDepth, classRangeMaps[targetClass].at<double>(ry,rx));
          maxDepth = max(maxDepth, classRangeMaps[targetClass].at<double>(ry,rx));
        }
      }
      for (int rx = 0; rx < rmWidth; rx++) {
        for (int ry = 0; ry < rmWidth; ry++) {
          double denom = max(EPSILON,maxDepth-minDepth);
          if (denom <= EPSILON)
            denom = VERYBIGNUMBER;
          double greenIntensity = 255 * (maxDepth - classRangeMaps[targetClass].at<double>(ry,rx)) / denom;
          {
            cv::Scalar backColor(0,ceil(greenIntensity),0);
            cv::Point outTop = cv::Point((ry+rmWidth)*rmiCellWidth,rx*rmiCellWidth);
            cv::Point outBot = cv::Point(((ry+rmWidth)+1)*rmiCellWidth,(rx+1)*rmiCellWidth);
            Mat vCrop = graspMemoryImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
            vCrop = backColor;
          }
          {
            cv::Scalar backColor(0,ceil(greenIntensity/2),0);
            cv::Point outTop = cv::Point((ry)*rmiCellWidth,rx*rmiCellWidth);
            cv::Point outBot = cv::Point(((ry)+1)*rmiCellWidth,(rx+1)*rmiCellWidth);
            Mat vCrop = graspMemoryImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
            vCrop = vCrop + backColor;
          }
        }
      }
    }
  }
  // draw grasp memory sample window
  {
    double max_value = -VERYBIGNUMBER;
    int max_rx=0, max_ry=0, max_tGG=0;
    int dy[4] = {0,1,0,1};
    int dx[4] = {0,0,1,1};
    
    for (int tGG = 0; tGG < totalGraspGears/2; tGG++) {

      for (int rx = 0; rx < rmWidth; rx++) {
        for (int ry = 0; ry < rmWidth; ry++) {
          int i = rx + ry*rmWidth + rmWidth*rmWidth*tGG;
          if (graspMemorySample[i] > max_value) {
            max_value = graspMemorySample[i];
            max_rx = rx;
            max_ry = ry;
            max_tGG = tGG;
          }

          
          {
            double blueIntensity = 255 * graspMemorySample[i];
            double greenIntensity = 255 * graspMemorySample[i];
            double redIntensity = 255 * graspMemorySample[i];
            //cout << "Grasp Memory Sample: " << "rx: " << rx << " ry: " << ry << " tGG:" << tGG << "sample: " << graspMemorySample[i] << endl;

            cv::Scalar color(ceil(blueIntensity),ceil(greenIntensity),ceil(redIntensity));

            cv::Point outTop = cv::Point((ry + dy[tGG]*rmWidth)*rmiCellWidth,(rx + dx[tGG]*rmWidth)*rmiCellWidth);
            cv::Point outBot = cv::Point(((ry + dy[tGG]*rmWidth)+1)*rmiCellWidth,((rx + dx[tGG]*rmWidth)+1)*rmiCellWidth);
            Mat vCrop = graspMemorySampleImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
            vCrop = color;
          }
        }
      }
    }

    
    for (int tGG = 0; tGG < totalGraspGears/2; tGG++) {
      {
        char buff[256];
        cv::Point text_anchor = cv::Point((dy[tGG]*rmWidth)*rmiCellWidth, 
                                          (dx[tGG]*rmWidth + 1)*rmiCellWidth);
        sprintf(buff, "%d", tGG+1);
        putText(graspMemorySampleImage, buff, text_anchor, MY_FONT, 1, Scalar(192,192,192), 2);
      }
    }

    {
      // draw the max
      char buff[256];
      cv::Point text_anchor = cv::Point((max_ry + dy[max_tGG]*rmWidth) * rmiCellWidth, 
                                        (max_rx + dx[max_tGG]*rmWidth + 1) * rmiCellWidth);
      sprintf(buff, "x");
      putText(graspMemorySampleImage, buff, text_anchor, MY_FONT, 1, 
              Scalar(0,0,255), 2);
    }
  }
}


void applyGraspFilter(double * rangeMapRegA, double * rangeMapRegB) {
  cout << "Applying filter to rangeMapRegA and storing result in rangeMapRegA." << endl;

  // ATTN 2
  int dx[9] = { -1,  0,  1, 
                -1,  0,  1, 
                -1,  0,  1};
  int dy[9] = { -1, -1, -1, 
                0,  0,  0, 
                1,  1,  1};
  //	int dx[9] = { -2,  0,  2, 
  //		      -2,  0,  2, 
  //		      -2,  0,  2};
  //	int dy[9] = { -2, -2, -2, 
  //		       0,  0,  0, 
  //		       2,  2,  2};
  // ATTN 2
  int transformPadding = 2;
  //int transformPadding = 4;

  for (int rx = 0; rx < rmWidth; rx++) {
    for (int ry = 0; ry < rmWidth; ry++) {
      rangeMapRegB[rx + ry*rmWidth] = 0.0;
    }
  }
  for (int rx = transformPadding; rx < rmWidth-transformPadding; rx++) {
    for (int ry = transformPadding; ry < rmWidth-transformPadding; ry++) {
      for (int fx = 0; fx < 9; fx++)
        rangeMapRegB[rx + ry*rmWidth] += filter[fx] * rangeMapRegA[(rx+dx[fx]) + (ry+dy[fx])*rmWidth];
    }
  }
  for (int rx = 0; rx < rmWidth; rx++) {
    for (int ry = 0; ry < rmWidth; ry++) {
      rangeMapRegA[rx + ry*rmWidth] = rangeMapRegB[rx + ry*rmWidth];
    }
  }

  // XXX TODO Consider: 
  // Push boundary to deepest point...
  double minDepth = VERYBIGNUMBER;
  double maxDepth = 0;
  for (int rx = 0; rx < rmWidth; rx++) {
    for (int ry = 0; ry < rmWidth; ry++) {
      minDepth = min(minDepth, rangeMapRegA[rx + ry*rmWidth]);
      maxDepth = max(maxDepth, rangeMapRegA[rx + ry*rmWidth]);
    }
  }
  for (int rx = 0; rx < rmWidth; rx++) {
    for (int ry = 0; ry < transformPadding; ry++) {
      rangeMapRegA[rx + ry*rmWidth] = maxDepth;
      rangeMapRegB[rx + ry*rmWidth] = maxDepth;
    }
    for (int ry = rmWidth-transformPadding; ry < rmWidth; ry++) {
      rangeMapRegA[rx + ry*rmWidth] = maxDepth;
      rangeMapRegB[rx + ry*rmWidth] = maxDepth;
    }
  }
  for (int ry = 0; ry < rmWidth; ry++) {
    for (int rx = 0; rx < transformPadding; rx++) {
      rangeMapRegA[rx + ry*rmWidth] = maxDepth;
      rangeMapRegB[rx + ry*rmWidth] = maxDepth;
    }
    for (int rx = rmWidth-transformPadding; rx < rmWidth; rx++) {
      rangeMapRegA[rx + ry*rmWidth] = maxDepth;
      rangeMapRegB[rx + ry*rmWidth] = maxDepth;
    }
  }
}
void copyRangeMapRegister(double * src, double * target) {
  for (int ry = 0; ry < rmWidth; ry++) {
    for (int rx = 0; rx < rmWidth; rx++) {
      target[rx + ry*rmWidth] = src[rx + ry * rmWidth];
    }
  }
}

void copyGraspMemoryRegister(double * src, double * target) {
  for (int tGG = 0; tGG < totalGraspGears/2; tGG++) {
    for (int ry = 0; ry < rmWidth; ry++) {
      for (int rx = 0; rx < rmWidth; rx++) {
        target[rx + ry*rmWidth + rmWidth*rmWidth*tGG] = src[rx + ry * rmWidth + rmWidth*rmWidth*tGG];
      }
    }
  }
}

void loadGlobalTargetClassRangeMap(double * rangeMapRegA, double * rangeMapRegB) {
  //Quaternionf eeqform(currentEEPose.qw, currentEEPose.qx, currentEEPose.qy, currentEEPose.qz);
  Quaternionf eeqform(bestOrientationEEPose.qw, bestOrientationEEPose.qx, bestOrientationEEPose.qy, bestOrientationEEPose.qz);
  Quaternionf crane2Orient(0, 1, 0, 0);
  Quaternionf rel = eeqform * crane2Orient.inverse();
  Quaternionf ex(0,1,0,0);
  Quaternionf zee(0,0,0,1);
	
    
  Quaternionf result = rel * ex * rel.conjugate();
  Quaternionf thumb = rel * zee * rel.conjugate();
  double aY = result.y();
  double aX = result.x();

  // ATTN 1
  // this is here to get the signs right
  aX = -aX;

  // ATTN 22
  //double angle = atan2(aY, aX)*180.0/3.1415926;
  double angle = vectorArcTan(aY, aX)*180.0/3.1415926;
  double scale = 1.0;
  Point center = Point(rmWidth/2, rmWidth/2);
  Size toBecome(rmWidth, rmWidth);

  cout << "load target class range map angle result eeqform thumb: " << angle << " | " << result.x() << " "  << result.y() << " "  << result.z() << " "  << result.w() << " | " << eeqform.x() << " "  << eeqform.y() << " "  << eeqform.z() << " "  << eeqform.w() << " | " << thumb.x() << " "  << thumb.y() << " "  << thumb.z() << " "  << thumb.w() << endl;

  // Get the rotation matrix with the specifications above
  Mat rotatedClassRangeMap;
  Mat rot_mat = getRotationMatrix2D(center, angle, scale);
  warpAffine(classRangeMaps[targetClass], rotatedClassRangeMap, rot_mat, toBecome, INTER_LINEAR, BORDER_REPLICATE);

  bestOrientationAngle = angle;

  if ((targetClass < numClasses) && (targetClass >= 0)) {
    for (int y = 0; y < rmWidth; y++) {
      for (int x = 0; x < rmWidth; x++) {
        // unrotated
        //rangeMap[x + y*rmWidth] = classRangeMaps[targetClass].at<double>(y,x);
        //rangeMapReg1[x + y*rmWidth] = classRangeMaps[targetClass].at<double>(y,x);
        // rotated
        rangeMapRegA[x + y*rmWidth] = rotatedClassRangeMap.at<double>(y,x);
        rangeMapRegB[x + y*rmWidth] = rotatedClassRangeMap.at<double>(y,x);
      } 
    } 
  } 
}


void loadLocalTargetClassRangeMap(double * rangeMapRegA, double * rangeMapRegB) {
  if ((targetClass < numClasses) && (targetClass >= 0)) {
    for (int y = 0; y < rmWidth; y++) {
      for (int x = 0; x < rmWidth; x++) {
        rangeMapRegA[x + y*rmWidth] = classRangeMaps[targetClass].at<double>(y,x);
        rangeMapRegB[x + y*rmWidth] = classRangeMaps[targetClass].at<double>(y,x);
      } 
    } 
  } 
}


void prepareGraspFilter(int i) {
  if (i == 0) {
    prepareGraspFilter1();
  } else if (i == 1) {
    prepareGraspFilter2();
  } else if (i == 2) {
    prepareGraspFilter3();
  } else if (i == 3) {
    prepareGraspFilter4();
  }
}
void prepareGraspFilter1() {
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

void prepareGraspFilter2() {
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
void prepareGraspFilter3() {
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
void prepareGraspFilter4() {
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

void copyClassGraspMemoryTriesToGraspMemoryTries() {
  if ((classGraspMemoryTries1[targetClass].rows > 1) && (classGraspMemoryTries1[targetClass].cols > 1) &&
      (classGraspMemoryPicks1[targetClass].rows > 1) && (classGraspMemoryPicks1[targetClass].cols > 1) ) {
    cout << "graspMemoryTries[] = classGraspMemoryTries1" << endl;
    //cout << "classGraspMemoryTries1 " << classGraspMemoryTries1[targetClass] << endl; 
    //cout << "classGraspMemoryPicks1 " << classGraspMemoryPicks1[targetClass] << endl; 
    for (int y = 0; y < rmWidth; y++) {
      for (int x = 0; x < rmWidth; x++) {
        graspMemoryTries[x + y*rmWidth + rmWidth*rmWidth*0] = classGraspMemoryTries1[targetClass].at<double>(y,x);
      } 
    } 
    for (int y = 0; y < rmWidth; y++) {
      for (int x = 0; x < rmWidth; x++) {
        graspMemoryPicks[x + y*rmWidth + rmWidth*rmWidth*0] = classGraspMemoryPicks1[targetClass].at<double>(y,x);
      } 
    } 
  } else {
    cout << "Whoops, tried to set grasp memories 1 but they don't exist for this class." << targetClass << " " << classLabels[targetClass] << endl;
  }
  if ((classGraspMemoryTries2[targetClass].rows > 1) && (classGraspMemoryTries2[targetClass].cols > 1) &&
      (classGraspMemoryPicks2[targetClass].rows > 1) && (classGraspMemoryPicks2[targetClass].cols > 1) ) {
    cout << "graspMemoryTries[] = classGraspMemoryTries2" << endl;
    //cout << "classGraspMemoryTries2 " << classGraspMemoryTries2[targetClass] << endl; 
    //cout << "classGraspMemoryPicks2 " << classGraspMemoryPicks2[targetClass] << endl; 
    for (int y = 0; y < rmWidth; y++) {
      for (int x = 0; x < rmWidth; x++) {
        graspMemoryTries[x + y*rmWidth + rmWidth*rmWidth*1] = classGraspMemoryTries2[targetClass].at<double>(y,x);
      } 
    } 
    for (int y = 0; y < rmWidth; y++) {
      for (int x = 0; x < rmWidth; x++) {
        graspMemoryPicks[x + y*rmWidth + rmWidth*rmWidth*1] = classGraspMemoryPicks2[targetClass].at<double>(y,x);
      } 
    } 
  } else {
    cout << "Whoops, tried to set grasp memories 2 but they don't exist for this class." << targetClass << " " << classLabels[targetClass] << endl;
  }
  if ((classGraspMemoryTries3[targetClass].rows > 1) && (classGraspMemoryTries3[targetClass].cols > 1) &&
      (classGraspMemoryPicks3[targetClass].rows > 1) && (classGraspMemoryPicks3[targetClass].cols > 1) ) {
    cout << "graspMemoryTries[] = classGraspMemoryTries3" << endl;
    //cout << "classGraspMemoryTries3 " << classGraspMemoryTries3[targetClass] << endl; 
    //cout << "classGraspMemoryPicks3 " << classGraspMemoryPicks3[targetClass] << endl; 
    for (int y = 0; y < rmWidth; y++) {
      for (int x = 0; x < rmWidth; x++) {
        graspMemoryTries[x + y*rmWidth + rmWidth*rmWidth*2] = classGraspMemoryTries3[targetClass].at<double>(y,x);
      } 
    } 
    for (int y = 0; y < rmWidth; y++) {
      for (int x = 0; x < rmWidth; x++) {
        graspMemoryPicks[x + y*rmWidth + rmWidth*rmWidth*2] = classGraspMemoryPicks3[targetClass].at<double>(y,x);
      } 
    } 
  } else {
    cout << "Whoops, tried to set grasp memories 3 but they don't exist for this class." << targetClass << " " << classLabels[targetClass] << endl;
  }
  if ((classGraspMemoryTries4[targetClass].rows > 1) && (classGraspMemoryTries4[targetClass].cols > 1) &&
      (classGraspMemoryPicks4[targetClass].rows > 1) && (classGraspMemoryPicks4[targetClass].cols > 1) ) {
    cout << "graspMemoryTries[] = classGraspMemoryTries4" << endl;
    //cout << "classGraspMemoryTries4 " << classGraspMemoryTries4[targetClass] << endl; 
    //cout << "classGraspMemoryPicks4 " << classGraspMemoryPicks4[targetClass] << endl; 
    for (int y = 0; y < rmWidth; y++) {
      for (int x = 0; x < rmWidth; x++) {
        graspMemoryTries[x + y*rmWidth + rmWidth*rmWidth*3] = classGraspMemoryTries4[targetClass].at<double>(y,x);
      } 
    } 
    for (int y = 0; y < rmWidth; y++) {
      for (int x = 0; x < rmWidth; x++) {
        graspMemoryPicks[x + y*rmWidth + rmWidth*rmWidth*3] = classGraspMemoryPicks4[targetClass].at<double>(y,x);
      } 
    } 
  } else {
    cout << "Whoops, tried to set grasp memories 4 but they don't exist for this class." << targetClass << " " << classLabels[targetClass] << endl;
  }
        
  cout << "class " << classLabels[targetClass] << " number ";

}

void copyGraspMemoryTriesToClassGraspMemoryTries() {
  guardGraspMemory();
  for (int y = 0; y < rmWidth; y++) {
    for (int x = 0; x < rmWidth; x++) {
      classGraspMemoryTries1[focusedClass].at<double>(y,x) = graspMemoryTries[x + y*rmWidth + 0*rmWidth*rmWidth];
    } 
  }
  for (int y = 0; y < rmWidth; y++) {
    for (int x = 0; x < rmWidth; x++) {
      classGraspMemoryPicks1[focusedClass].at<double>(y,x) = graspMemoryPicks[x + y*rmWidth + 0*rmWidth*rmWidth];
    } 
  } 

  for (int y = 0; y < rmWidth; y++) {
    for (int x = 0; x < rmWidth; x++) {
      classGraspMemoryTries2[focusedClass].at<double>(y,x) = graspMemoryTries[x + y*rmWidth + 1*rmWidth*rmWidth];
    } 
  } 
  for (int y = 0; y < rmWidth; y++) {
    for (int x = 0; x < rmWidth; x++) {
      classGraspMemoryPicks2[focusedClass].at<double>(y,x) = graspMemoryPicks[x + y*rmWidth + 1*rmWidth*rmWidth];
    } 
  } 

  for (int y = 0; y < rmWidth; y++) {
    for (int x = 0; x < rmWidth; x++) {
      classGraspMemoryTries3[focusedClass].at<double>(y,x) = graspMemoryTries[x + y*rmWidth + 2*rmWidth*rmWidth];
    } 
  }
  for (int y = 0; y < rmWidth; y++) {
    for (int x = 0; x < rmWidth; x++) {
      classGraspMemoryPicks3[focusedClass].at<double>(y,x) = graspMemoryPicks[x + y*rmWidth + 2*rmWidth*rmWidth];
    } 
  }

  for (int y = 0; y < rmWidth; y++) {
    for (int x = 0; x < rmWidth; x++) {
      classGraspMemoryTries4[focusedClass].at<double>(y,x) = graspMemoryTries[x + y*rmWidth + 3*rmWidth*rmWidth];
    } 
  } 
  for (int y = 0; y < rmWidth; y++) {
    for (int x = 0; x < rmWidth; x++) {
      classGraspMemoryPicks4[focusedClass].at<double>(y,x) = graspMemoryPicks[x + y*rmWidth + 3*rmWidth*rmWidth];
    } 
  }
}

void selectMaxTarget(double minDepth) {
  // ATTN 10
  //selectMaxTargetLinearFilter(minDepth);
  //selectMaxTargetThompsonRotated(minDepth);
  //selectMaxTargetThompsonRotated2(minDepth);
  // ATTN 19
  //selectMaxTargetThompson(minDepth);
  //selectMaxTargetThompsonContinuous(minDepth);
  selectMaxTargetThompsonContinuous2(minDepth);
}

void selectMaxTargetLinearFilter(double minDepth) {
  // ATTN 2
  int maxSearchPadding = 3;
  //int maxSearchPadding = 4;

  for (int rx = maxSearchPadding; rx < rmWidth-maxSearchPadding; rx++) {
    for (int ry = maxSearchPadding; ry < rmWidth-maxSearchPadding; ry++) {

      // ATTN 5
      double graspMemoryWeight = 0.0;
      double graspMemoryBias = VERYBIGNUMBER;
      int localIntThX = -1; 
      int localIntThY = -1; 
      {
        // find global coordinate of current point
        double thX = (rx-rmHalfWidth) * rmDelta;
        double thY = (ry-rmHalfWidth) * rmDelta;
        // transform it into local coordinates
        double unangle = -bestOrientationAngle;
        double unscale = 1.0;
        Point uncenter = Point(0, 0);
        Mat un_rot_mat = getRotationMatrix2D(uncenter, unangle, unscale);
        Mat toUn(3,1,CV_64F);
        toUn.at<double>(0,0)=thX;
        toUn.at<double>(1,0)=thY;
        toUn.at<double>(2,0)=1.0;
        Mat didUn = un_rot_mat*toUn;
        double localThX = didUn.at<double>(0,0);
        double localThY = didUn.at<double>(1,0);
        localIntThX = ((localThX)/rmDelta) + rmHalfWidth; 
        localIntThY = ((localThY)/rmDelta) + rmHalfWidth; 
        // retrieve its value
        double mDenom = max(graspMemoryTries[localIntThX + localIntThY*rmWidth + rmWidth*rmWidth*getLocalGraspGear(currentGraspGear)], 1.0);
        if ((localIntThX < rmWidth) && (localIntThY < rmWidth)) {

          // Thompson
          //graspMemoryWeight = (graspMemorySample[localIntThX + localIntThY*rmWidth + rmWidth*rmWidth*getLocalGraspGear(currentGraspGear)]) * -1;  

          // Original
          //graspMemoryWeight = graspMemoryPicks[localIntThX + localIntThY*rmWidth + rmWidth*rmWidth*getLocalGraspGear(currentGraspGear)] / mDenom;
          //graspMemoryWeight = graspMemoryWeight * rangeMapReg1[rx + ry*rmWidth]);  
           
          // No memory; just linear filter
          graspMemoryWeight = rangeMapReg1[rx + ry*rmWidth];
          graspMemoryBias = 0;
        } else {
          graspMemoryWeight = 0;
        }
      }


      //cout << "graspMemory Incorporation rx ry lthx lthy gmw: " << rx << " " << ry << " LL: " << localIntThX << " " << localIntThY << " " << graspMemoryWeight << endl;
      //cout << "  gmTargetX gmTargetY eval: " << gmTargetX << " " << gmTargetY << " " << graspMemoryPicks[gmTargetX + gmTargetY*rmWidth + rmWidth*rmWidth*getLocalGraspGear(currentGraspGear)] << endl;
	    
      // 
      if (graspMemoryBias + graspMemoryWeight < minDepth) 
      //if (graspMemoryBias + graspMemoryWeight < minDepth)  // thompson
      {
	minDepth = rangeMapReg1[rx + ry*rmWidth];
	maxX = rx;
	maxY = ry;
	localMaxX = localIntThX;
	localMaxY = localIntThY;
	localMaxGG = getLocalGraspGear(currentGraspGear);
	maxD = rangeMapReg1[rx + ry*rmWidth];
	maxGG = currentGraspGear;
	useContinuousGraspTransform = 0;
      }
    }
  }
  cout << "non-cumulative maxX: " << maxX << " maxY: " << maxY <<  " maxD: " << maxD << " maxGG: " << maxGG << endl;
}

void selectMaxTargetThompson(double minDepth) {
  // ATTN 2
  int maxSearchPadding = 3;
  //int maxSearchPadding = 4;

  for (int localX = 0; localX < rmWidth; localX++) {
    for (int localY = 0; localY < rmWidth; localY++) {
      // ATTN 5
      double graspMemoryWeight = 0.0;
      double graspMemoryBias = VERYBIGNUMBER;
      int rx, ry;
      convertLocalGraspIdxToGlobal(localX, localY, &rx, &ry);
      if ((rx < rmWidth) && (ry < rmWidth)) {
        
        // Thompson
        graspMemoryWeight = (graspMemorySample[localX + localY*rmWidth + rmWidth*rmWidth*getLocalGraspGear(currentGraspGear)]) * -1;  
        
        graspMemoryBias = 0;
      } else {
        graspMemoryWeight = 0;
      }
      
      //cout << "graspMemory Thompson incorporation rx ry lthx lthy gmw: " << rx << " " << ry << " LL: " << localX << " " << localY << " " << graspMemoryWeight << endl;
      
      // ATTN 19
      int i = localX + localY * rmWidth + rmWidth*rmWidth*getLocalGraspGear(currentGraspGear);
      int maxedOutTries = isThisGraspMaxedOut(i);

      //if (graspMemoryBias + graspMemoryWeight < minDepth) 
      if ((graspMemoryBias + graspMemoryWeight < minDepth) && !maxedOutTries) {
	minDepth = graspMemoryWeight;
	maxX = rx;
	maxY = ry;
	localMaxX = localX;
	localMaxY = localY;
	localMaxGG = getLocalGraspGear(currentGraspGear);
	maxD = graspMemoryWeight;
	maxGG = currentGraspGear;
	useContinuousGraspTransform = 0;
      }
    }
  }
  cout << "non-cumulative Thompson maxX: " << maxX << " maxY: " << maxY <<  " maxD: " << maxD << " maxGG: " << maxGG << endl;
}

void selectMaxTargetThompsonContinuous(double minDepth) {
  // ATTN 2
  int maxSearchPadding = 3;
  //int maxSearchPadding = 4;

  for (int rx = maxSearchPadding; rx < rmWidth-maxSearchPadding; rx++) {
    for (int ry = maxSearchPadding; ry < rmWidth-maxSearchPadding; ry++) {

      // ATTN 5
      double graspMemoryWeight = 0.0;
      double graspMemoryBias = VERYBIGNUMBER;
      int localIntThX = -1; 
      int localIntThY = -1; 
      {
        // find global coordinate of current point
        double thX = (rx-rmHalfWidth) * rmDelta;
        double thY = (ry-rmHalfWidth) * rmDelta;
        // transform it into local coordinates
        double unangle = -bestOrientationAngle;
        double unscale = 1.0;
        Point uncenter = Point(0, 0);
        Mat un_rot_mat = getRotationMatrix2D(uncenter, unangle, unscale);
        Mat toUn(3,1,CV_64F);
        toUn.at<double>(0,0)=thX;
        toUn.at<double>(1,0)=thY;
        toUn.at<double>(2,0)=1.0;
        Mat didUn = un_rot_mat*toUn;
        double localThX = didUn.at<double>(0,0);
        double localThY = didUn.at<double>(1,0);
        localIntThX = ((localThX)/rmDelta) + rmHalfWidth; 
        localIntThY = ((localThY)/rmDelta) + rmHalfWidth; 
        // retrieve its value
        double mDenom = max(graspMemoryTries[localIntThX + localIntThY*rmWidth + rmWidth*rmWidth*getLocalGraspGear(currentGraspGear)], 1.0);
        if ((localIntThX < rmWidth) && (localIntThY < rmWidth)) {

          // Thompson
          graspMemoryWeight = (graspMemorySample[localIntThX + localIntThY*rmWidth + rmWidth*rmWidth*getLocalGraspGear(currentGraspGear)]) * -1;  

          graspMemoryBias = 0;
        } else {
          graspMemoryWeight = 0;
        }
      }

      //cout << "graspMemory Incorporation rx ry lthx lthy gmw: " << rx << " " << ry << " LL: " << localIntThX << " " << localIntThY << " " << graspMemoryWeight << endl;
      //cout << "  gmTargetX gmTargetY eval: " << gmTargetX << " " << gmTargetY << " " << graspMemoryPicks[gmTargetX + gmTargetY*rmWidth + rmWidth*rmWidth*getLocalGraspGear(currentGraspGear)] << endl;
	    
      // ATTN 19
      int i = localIntThX + localIntThY * rmWidth + rmWidth*rmWidth*getLocalGraspGear(currentGraspGear);
      int maxedOutTries = isThisGraspMaxedOut(i);

      //if (graspMemoryBias + graspMemoryWeight < minDepth) 
      if ((graspMemoryBias + graspMemoryWeight < minDepth) && !maxedOutTries) {
	minDepth = graspMemoryWeight;
	maxX = rx;
	maxY = ry;
	localMaxX = localIntThX;
	localMaxY = localIntThY;
	//localMaxGG = currentGraspGear;
	localMaxGG = getLocalGraspGear(currentGraspGear);
	maxD = graspMemoryWeight;
	//maxGG = currentGraspGear;
	maxGG = getLocalGraspGear(currentGraspGear);
	useContinuousGraspTransform = 1;
	//useContinuousGraspTransform = 0;
      }
    }
  }
  cout << "non-cumulative maxX: " << maxX << " maxY: " << maxY <<  " maxD: " << maxD << " maxGG: " << maxGG << endl;
}

void selectMaxTargetThompsonContinuous2(double minDepth) {
  // ATTN 2
  int maxSearchPadding = 3;
  //int maxSearchPadding = 4;

  for (int rx = maxSearchPadding; rx < rmWidth-maxSearchPadding; rx++) {
    for (int ry = maxSearchPadding; ry < rmWidth-maxSearchPadding; ry++) {

      // ATTN 5
      double graspMemoryWeight = 0.0;
      double graspMemoryBias = VERYBIGNUMBER;
      int localIntThX = -1; 
      int localIntThY = -1; 
      double localThX = 0.0;
      double localThY = 0.0;
      {
        // find local coordinate of current point
        double thX = (rx-rmHalfWidth) * rmDelta;
        double thY = (ry-rmHalfWidth) * rmDelta;
        // transform it into global coordinates
        double angle = bestOrientationAngle;
        double unscale = 1.0;
        Point uncenter = Point(0, 0);
        Mat un_rot_mat = getRotationMatrix2D(uncenter, angle, unscale);
        Mat toUn(3,1,CV_64F);
        toUn.at<double>(0,0)=thX;
        toUn.at<double>(1,0)=thY;
        toUn.at<double>(2,0)=1.0;
        Mat didUn = un_rot_mat*toUn;
        localThX = didUn.at<double>(0,0);
        localThY = didUn.at<double>(1,0);
        localIntThX = ((localThX)/rmDelta) + rmHalfWidth; 
        localIntThY = ((localThY)/rmDelta) + rmHalfWidth; 
        // retrieve its value
        double mDenom = max(graspMemoryTries[rx + ry*rmWidth + rmWidth*rmWidth*(currentGraspGear)], 1.0);
        if ((rx < rmWidth) && (ry < rmWidth)) {

          // Thompson
          graspMemoryWeight = (graspMemorySample[rx + ry*rmWidth + rmWidth*rmWidth*(currentGraspGear)]) * -1;  

          graspMemoryBias = 0;
        } else {
          graspMemoryWeight = 0;
        }
      }


      //cout << "graspMemory Incorporation rx ry lthx lthy gmw: " << rx << " " << ry << " LL: " << localIntThX << " " << localIntThY << " " << graspMemoryWeight << endl;
      //cout << "  gmTargetX gmTargetY eval: " << gmTargetX << " " << gmTargetY << " " << graspMemoryPicks[gmTargetX + gmTargetY*rmWidth + rmWidth*rmWidth*(currentGraspGear)] << endl;
	    
      // ATTN 19
      int i = rx + ry * rmWidth + rmWidth*rmWidth*(currentGraspGear);
      int maxedOutTries = isThisGraspMaxedOut(i);

      //if (graspMemoryBias + graspMemoryWeight < minDepth) 
      if ((graspMemoryBias + graspMemoryWeight < minDepth) && !maxedOutTries) {
          minDepth = graspMemoryWeight;
          maxX = localIntThX;
          maxY = localIntThY;
          localMaxX = rx;
          localMaxY = ry;
          localMaxGG = (currentGraspGear);
          maxD = graspMemoryWeight;
          //maxGG = getGlobalGraspGear(currentGraspGear);
          maxGG = (currentGraspGear);
	  //useContinuousGraspTransform = 0;
	  useContinuousGraspTransform = 1;
	  cout << "ZZZ ZZZ ZZZ" << endl;
        }
    }
  }
  cout << "non-cumulative maxX: " << maxX << " maxY: " << maxY <<  " maxD: " << 
    maxD << " maxGG: " << maxGG << " localMaxGG: " << localMaxGG << endl;
}

void selectMaxTargetThompsonRotated(double minDepth) {
  // ATTN 2
  int maxSearchPadding = 3;
  //int maxSearchPadding = 4;

  for (int rx = maxSearchPadding; rx < rmWidth-maxSearchPadding; rx++) {
    for (int ry = maxSearchPadding; ry < rmWidth-maxSearchPadding; ry++) {

      // ATTN 5
      double graspMemoryWeight = 0.0;
      double graspMemoryBias = VERYBIGNUMBER;
      int localIntThX = -1; 
      int localIntThY = -1; 
      {
        // find global coordinate of current point
        double thX = (rx-rmHalfWidth) * rmDelta;
        double thY = (ry-rmHalfWidth) * rmDelta;
        // transform it into local coordinates
        double unangle = -bestOrientationAngle;
        double unscale = 1.0;
        Point uncenter = Point(0, 0);
        Mat un_rot_mat = getRotationMatrix2D(uncenter, unangle, unscale);
        Mat toUn(3,1,CV_64F);
        toUn.at<double>(0,0)=thX;
        toUn.at<double>(1,0)=thY;
        toUn.at<double>(2,0)=1.0;
        Mat didUn = un_rot_mat*toUn;
        double localThX = didUn.at<double>(0,0);
        double localThY = didUn.at<double>(1,0);
        localIntThX = ((localThX)/rmDelta) + rmHalfWidth; 
        localIntThY = ((localThY)/rmDelta) + rmHalfWidth; 
        // retrieve its value
        double mDenom = max(graspMemoryTries[localIntThX + localIntThY*rmWidth + rmWidth*rmWidth*getLocalGraspGear(currentGraspGear)], 1.0);
        if ((localIntThX < rmWidth) && (localIntThY < rmWidth)) {

          // Thompson
          graspMemoryWeight = (graspMemorySample[localIntThX + localIntThY*rmWidth + rmWidth*rmWidth*getLocalGraspGear(currentGraspGear)]) * -1;  

          // Original
          //graspMemoryWeight = graspMemoryPicks[localIntThX + localIntThY*rmWidth + rmWidth*rmWidth*getLocalGraspGear(currentGraspGear)] / mDenom;
          //graspMemoryWeight = graspMemoryWeight * rangeMapReg1[rx + ry*rmWidth]);  
           
          // No memory; just linear filter
          // graspMemoryWeight = rangeMapReg1[rx + ry*rmWidth];
          graspMemoryBias = 0;
        } else {
          graspMemoryWeight = 0;
        }
      }


      //cout << "graspMemory Incorporation rx ry lthx lthy gmw: " << rx << " " << ry << " LL: " << localIntThX << " " << localIntThY << " " << graspMemoryWeight << endl;
      //cout << "  gmTargetX gmTargetY eval: " << gmTargetX << " " << gmTargetY << " " << graspMemoryPicks[gmTargetX + gmTargetY*rmWidth + rmWidth*rmWidth*getLocalGraspGear(currentGraspGear)] << endl;
	    
      // ATTN 19
      int i = localIntThX + localIntThY * rmWidth + rmWidth*rmWidth*getLocalGraspGear(currentGraspGear);
      int maxedOutTries = isThisGraspMaxedOut(i);

      //if (graspMemoryBias + graspMemoryWeight < minDepth) 
      if ((graspMemoryBias + graspMemoryWeight < minDepth) && !maxedOutTries) {
          minDepth = graspMemoryWeight;
          maxX = rx;
          maxY = ry;
          localMaxX = localIntThX;
          localMaxY = localIntThY;
          localMaxGG = getLocalGraspGear(currentGraspGear);
          maxD = graspMemoryWeight;
          maxGG = currentGraspGear;
	  useContinuousGraspTransform = 0;
        }
    }
  }
  cout << "non-cumulative maxX: " << maxX << " maxY: " << maxY <<  " maxD: " << maxD << " maxGG: " << maxGG << endl;
}

void selectMaxTargetThompsonRotated2(double minDepth) {
  // ATTN 2
  int maxSearchPadding = 3;
  //int maxSearchPadding = 4;

  for (int rx = maxSearchPadding; rx < rmWidth-maxSearchPadding; rx++) {
    for (int ry = maxSearchPadding; ry < rmWidth-maxSearchPadding; ry++) {

      // ATTN 5
      double graspMemoryWeight = 0.0;
      double graspMemoryBias = VERYBIGNUMBER;
      int localIntThX = -1; 
      int localIntThY = -1; 
      double localThX = 0.0;
      double localThY = 0.0;
      {
        // find local coordinate of current point
        double thX = (rx-rmHalfWidth) * rmDelta;
        double thY = (ry-rmHalfWidth) * rmDelta;
        // transform it into global coordinates
        double angle = bestOrientationAngle;
        double unscale = 1.0;
        Point uncenter = Point(0, 0);
        Mat un_rot_mat = getRotationMatrix2D(uncenter, angle, unscale);
        Mat toUn(3,1,CV_64F);
        toUn.at<double>(0,0)=thX;
        toUn.at<double>(1,0)=thY;
        toUn.at<double>(2,0)=1.0;
        Mat didUn = un_rot_mat*toUn;
        localThX = didUn.at<double>(0,0);
        localThY = didUn.at<double>(1,0);
        localIntThX = ((localThX)/rmDelta) + rmHalfWidth; 
        localIntThY = ((localThY)/rmDelta) + rmHalfWidth; 
        // retrieve its value
        double mDenom = max(graspMemoryTries[rx + ry*rmWidth + rmWidth*rmWidth*(currentGraspGear)], 1.0);
        if ((rx < rmWidth) && (ry < rmWidth)) {

          // Thompson
          graspMemoryWeight = (graspMemorySample[rx + ry*rmWidth + rmWidth*rmWidth*(currentGraspGear)]) * -1;  

          graspMemoryBias = 0;
        } else {
          graspMemoryWeight = 0;
        }
      }


      //cout << "graspMemory Incorporation rx ry lthx lthy gmw: " << rx << " " << ry << " LL: " << localIntThX << " " << localIntThY << " " << graspMemoryWeight << endl;
      //cout << "  gmTargetX gmTargetY eval: " << gmTargetX << " " << gmTargetY << " " << graspMemoryPicks[gmTargetX + gmTargetY*rmWidth + rmWidth*rmWidth*(currentGraspGear)] << endl;
	    
      // ATTN 19
      int i = rx + ry * rmWidth + rmWidth*rmWidth*(currentGraspGear);
      int maxedOutTries = isThisGraspMaxedOut(i);

      //if (graspMemoryBias + graspMemoryWeight < minDepth) 
      if ((graspMemoryBias + graspMemoryWeight < minDepth) && !maxedOutTries) {
          minDepth = graspMemoryWeight;
          maxX = localIntThX;
          maxY = localIntThY;
          localMaxX = rx;
          localMaxY = ry;
          localMaxGG = (currentGraspGear);
          maxD = graspMemoryWeight;
          maxGG = getGlobalGraspGear(currentGraspGear);
	  useContinuousGraspTransform = 0;
        }
    }
  }
  cout << "non-cumulative maxX: " << maxX << " maxY: " << maxY <<  " maxD: " << maxD << " maxGG: " << maxGG << endl;
}

double squareDistanceEEPose(eePose pose1, eePose pose2) {
  double dx = (pose1.px - pose2.px);
  double dy = (pose1.py - pose2.py);
  double dz = (pose1.pz - pose2.pz);
  double squareDistance = dx*dx + dy*dy + dz*dz;

  return squareDistance;
}

void recordBoundingBoxSuccess() {
  heightMemoryTries[currentThompsonHeightIdx]++;
  heightMemoryPicks[currentThompsonHeightIdx]++;
  heightSuccessCounter++;
  heightAttemptCounter++;
  cout << "Successful bounding box on floor " << currentThompsonHeightIdx << endl;
  cout << "Tries: " << heightMemoryTries[currentThompsonHeightIdx] << endl;
  cout << "Picks: " << heightMemoryPicks[currentThompsonHeightIdx] << endl;
  int ttotalTries = 0;
  int ttotalPicks = 0;
  for (int i = 0; i < hmWidth; i++) {
    ttotalTries += heightMemoryTries[i];
    ttotalPicks += heightMemoryPicks[i];
  }
  cout << "Total Tries: " << ttotalTries << endl;
  cout << "Total Picks: " << ttotalPicks << endl;

  double thisPickRate = double(heightMemoryPicks[currentThompsonHeightIdx]) / double(heightMemoryTries[currentThompsonHeightIdx]);
  int thisNumTries = heightMemoryTries[currentThompsonHeightIdx];
  cout << "Thompson Early Out: thisPickrate = " << thisPickRate << ", thisNumTries = " << thisNumTries << endl;
  if (currentBoundingBoxMode == LEARNING_SAMPLING) {
    if ( (thisNumTries >= thompsonMinTryCutoff) && 
	 (thisPickRate >= thompsonMinPassRate) ) {
      thompsonHeightHaltFlag = 1;
    }
  }

  // ATTN 20
  {
    double successes = heightMemoryPicks[currentThompsonHeightIdx];
    double failures =  heightMemoryTries[currentThompsonHeightIdx] - heightMemoryPicks[currentThompsonHeightIdx];
    // returns probability that mu <= d given successes and failures.
    double presult = cephes_incbet(successes + 1, failures + 1, algorithmCTarget);
    // we want probability that mu > d
    double result = 1.0 - presult;

    double presult2a = cephes_incbet(successes + 1, failures + 1, algorithmCTarget + algorithmCEPS);
    double presult2b = cephes_incbet(successes + 1, failures + 1, algorithmCTarget - algorithmCEPS);
    // we want probability that 
    //  algorithmCTarget - algorithmCEPS < mu < algorithmCTarget + algorithmCEPS
    double result2 = presult2a - presult2b;

    cout << "prob that mu > d: " << result << " algorithmCAT: " << algorithmCAT << endl;
    if (currentBoundingBoxMode == LEARNING_ALGORITHMC) {
      thompsonHeightHaltFlag = (result > algorithmCAT);
      if (result2 > algorithmCAT) {
	thompsonHeightHaltFlag = 1;
      }
    }
  }
}

void recordBoundingBoxFailure() {
  heightMemoryTries[currentThompsonHeightIdx]++;
  heightAttemptCounter++;
  cout << "Failed to learn bounding box on floor " << currentThompsonHeightIdx << endl;
  cout << "Tries: " << heightMemoryTries[currentThompsonHeightIdx] << endl;
  cout << "Picks: " << heightMemoryPicks[currentThompsonHeightIdx] << endl;
  int ttotalTries = 0;
  int ttotalPicks = 0;
  for (int i = 0; i < hmWidth; i++) {
    ttotalTries += heightMemoryTries[i];
    ttotalPicks += heightMemoryPicks[i];
  }
  cout << "Total Tries: " << ttotalTries << endl;
  cout << "Total Picks: " << ttotalPicks << endl;
}

void restartBBLearning() {
  recordBoundingBoxFailure();
  clearStack();
  pushWord(1179707); // continue bounding box learning
}

double unsignedQuaternionDistance(Quaternionf q1, Quaternionf q2) {
  float r = q1.dot(q2);
  if(r < -1.0 || r > 1.0) return 0;
    r = acos(r); 
  return r <= M_PI_2? r: M_PI-r;
}

void moveCurrentGripperRayToCameraVanishingRay() {
  bool useLaser = 0;
  if (useLaser) {
    //double d_y = -0.04;
    //double d_x = 0.018;
    // this is really the right way of doing it... the question is,
    // which of the three ways gives the best result?
    double pTermY = -0.018;
    double pTermX = 0.04;
    Eigen::Vector3f localUnitX;
    {
      Eigen::Quaternionf qin(0, 1, 0, 0);
      Eigen::Quaternionf qout(0, 1, 0, 0);
      Eigen::Quaternionf eeqform(trueEEPose.orientation.w, trueEEPose.orientation.x, trueEEPose.orientation.y, trueEEPose.orientation.z);
      qout = eeqform * qin * eeqform.conjugate();
      localUnitX.x() = qout.x();
      localUnitX.y() = qout.y();
      localUnitX.z() = qout.z();
    }
      
    Eigen::Vector3f localUnitY;
    {
      Eigen::Quaternionf qin(0, 0, 1, 0);
      Eigen::Quaternionf qout(0, 1, 0, 0);
      Eigen::Quaternionf eeqform(trueEEPose.orientation.w, trueEEPose.orientation.x, trueEEPose.orientation.y, trueEEPose.orientation.z);
      qout = eeqform * qin * eeqform.conjugate();
      localUnitY.x() = qout.x();
      localUnitY.y() = qout.y();
      localUnitY.z() = qout.z();
    }
    cout << "moveCurrentGripperRayToCameraVanishingRay localUnitY localUnitX: " << localUnitY << " " << localUnitX << endl;
    cout << "moveCurrentGripperRayToCameraVanishingRay localUnitY localUnitX: " << localUnitY.norm() << " " << localUnitX.norm() << endl;
    double xToAdd = (pTermX*localUnitY.x() - pTermY*localUnitX.x());
    double yToAdd = (pTermX*localUnitY.y() - pTermY*localUnitX.y());
    cout << "moveCurrentGripperRayToCameraVanishingRay xToAdd yToAdd: " << xToAdd << " " << yToAdd << endl;
    currentEEPose.px += xToAdd;
    currentEEPose.py += yToAdd;
  } else {
    double zToUse = trueEEPose.position.z+currentTableZ;
    pixelToGlobal(vanishingPointReticle.px, vanishingPointReticle.py, zToUse, &(currentEEPose.px), &(currentEEPose.py));
  }
  { // yet another way to do this
    // 0 assumes no rotation 
    //eePose finalGlobalTarget = analyticServoPixelToReticle(pilotTarget, thisGripperReticle, 0);
    //currentEEPose.px = finalGlobalTarget.px;
    //currentEEPose.py = finalGlobalTarget.py;
  }
}

void gradientServo() {
  Size sz = objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;

  // ATTN 23
  //reticle = heightReticles[currentThompsonHeightIdx];
  eePose thisGripperReticle;
  double zToUse = trueEEPose.position.z+currentTableZ;
  int xOut=-1, yOut=-1;
  globalToPixel(&xOut, &yOut, zToUse, trueEEPoseEEPose.px, trueEEPoseEEPose.py);
  thisGripperReticle.px = xOut;
  thisGripperReticle.py = yOut;
  reticle = vanishingPointReticle;

  // ATTN 12
  //        if ((synServoLockFrames > heightLearningServoTimeout) && (currentBoundingBoxMode == LEARNING_SAMPLING)) {
  //          cout << "bbLearning: synchronic servo timed out, early outting." << endl;
  //          restartBBLearning();
  //        }

  cout << "entered gradient servo... iteration " << currentGradientServoIterations << endl;
  if (targetClass < 0 || targetClass >= numClasses) {
    cout << "bad target class, not servoing." << endl;
    return;
  }
  if ((classAerialGradients[targetClass].rows <= 1) && (classAerialGradients[targetClass].cols <= 1)) {
    cout << "no aerial gradients for this class, not servoing." << endl;
    return;
  }

  {
    int i, j;
    mapxyToij(currentEEPose.px, currentEEPose.py, &i, &j);
    int doWeHaveClearance = (clearanceMap[i + mapWidth * j] != 0);
    if (!doWeHaveClearance) {
      //pushWord("clearStackIntoMappingPatrol"); 
      cout << ">>>> Gradient servo strayed out of clearance area during mapping. <<<<" << endl;
      return;
    }
  }

  // ATTN 16
  switch (currentThompsonHeightIdx) {
  case 0:
    {
      classAerialGradients[targetClass] = classHeight0AerialGradients[targetClass];
    }
    break;
  case 1:
    {
      classAerialGradients[targetClass] = classHeight1AerialGradients[targetClass];
    }
    break;
  case 2:
    {
      classAerialGradients[targetClass] = classHeight2AerialGradients[targetClass];
    }
    break;
  case 3:
    {
      classAerialGradients[targetClass] = classHeight3AerialGradients[targetClass];
    }
    break;
  default:
    {
      assert(0);
    }
    break;
  }

  double Px = 0;
  double Py = 0;

  double Ps = 0;

  //cout << "computing scores... ";

  Size toBecome(aerialGradientWidth, aerialGradientWidth);

  int numOrientations = 37;
  vector<Mat> rotatedAerialGrads;

  // ATTN 3
  // gradientServoScale should be even
  int gradientServoScale = 3;//11;
  double gradientServoScaleStep = 1.02;
  if (orientationCascade) {
    if (lastPtheta < lPTthresh) {
      //gradientServoScale = 1;
      //gradientServoScaleStep = 1.0;
    }
  }
  double startScale = pow(gradientServoScaleStep, -(gradientServoScale-1)/2);

  //rotatedAerialGrads.resize(numOrientations);
  rotatedAerialGrads.resize(gradientServoScale*numOrientations);

  if ((lastPtheta < lPTthresh) && orientationCascade) {
    cout << "orientation cascade activated" << endl;
  }

  for (int etaS = 0; etaS < gradientServoScale; etaS++) {
    double thisScale = startScale * pow(gradientServoScaleStep, etaS);
    for (int thisOrient = 0; thisOrient < numOrientations; thisOrient++) {
      // orientation cascade
      if (orientationCascade) {
        if (lastPtheta < lPTthresh) {
          if (thisOrient < orientationCascadeHalfWidth) {
            //cout << "skipping orientation " << thisOrient << endl;
            continue;
          }
          if (thisOrient > numOrientations - orientationCascadeHalfWidth) {
            //cout << "skipping orientation " << thisOrient << endl;
            continue;
          }
        }
      }
      
      // rotate the template and L1 normalize it
      Point center = Point(aerialGradientWidth/2, aerialGradientWidth/2);
      double angle = thisOrient*360.0/numOrientations;
      
      //double scale = 1.0;
      double scale = thisScale;
      
      // Get the rotation matrix with the specifications above
      Mat rot_mat = getRotationMatrix2D(center, angle, scale);
      warpAffine(classAerialGradients[targetClass], rotatedAerialGrads[thisOrient + etaS*numOrientations], rot_mat, toBecome);
      
      processSaliency(rotatedAerialGrads[thisOrient + etaS*numOrientations], rotatedAerialGrads[thisOrient + etaS*numOrientations]);
      
      //double l1norm = rotatedAerialGrads[thisOrient + etaS*numOrientations].dot(Mat::ones(aerialGradientWidth, aerialGradientWidth, rotatedAerialGrads[thisOrient + etaS*numOrientations].type()));
      //if (l1norm <= EPSILON)
      //l1norm = 1.0;
      //rotatedAerialGrads[thisOrient + etaS*numOrientations] = rotatedAerialGrads[thisOrient + etaS*numOrientations] / l1norm;
      //cout << "classOrientedGradients[targetClass]: " << classAerialGradients[targetClass] << "rotatedAerialGrads[thisOrient + etaS*numOrientations] " << rotatedAerialGrads[thisOrient + etaS*numOrientations] << endl;
      
      double mean = rotatedAerialGrads[thisOrient + etaS*numOrientations].dot(Mat::ones(aerialGradientWidth, aerialGradientWidth, rotatedAerialGrads[thisOrient + etaS*numOrientations].type())) / double(aerialGradientWidth*aerialGradientWidth);
      rotatedAerialGrads[thisOrient + etaS*numOrientations] = rotatedAerialGrads[thisOrient + etaS*numOrientations] - mean;
      double l2norm = rotatedAerialGrads[thisOrient + etaS*numOrientations].dot(rotatedAerialGrads[thisOrient + etaS*numOrientations]);
      if (l2norm <= EPSILON)
        l2norm = 1.0;
      rotatedAerialGrads[thisOrient + etaS*numOrientations] = rotatedAerialGrads[thisOrient + etaS*numOrientations] / l2norm;
    }
  }

  int bestOrientation = -1;
  double bestOrientationScore = -INFINITY;
  double bestCropNorm = 1.0;
  int bestX = -1;
  int bestY = -1;
  int bestS = -1;

  int crows = aerialGradientReticleWidth;
  int ccols = aerialGradientReticleWidth;
  int maxDim = max(crows, ccols);
  int tRy = (maxDim-crows)/2;
  int tRx = (maxDim-ccols)/2;

  //int gradientServoTranslation = 40;
  //int gsStride = 2;
  int gradientServoTranslation = 40;
  int gsStride = 2;
  if (orientationCascade) {
    if (lastPtheta < lPTthresh) {
      //int gradientServoTranslation = 20;
      //int gsStride = 2;
      int gradientServoTranslation = 40;
      int gsStride = 2;
    }
  }
  
  //rotatedAerialGrads.resize(gradientServoScale*numOrientations);
  int gSTwidth = 2*gradientServoTranslation + 1;
  double allScores[gSTwidth][gSTwidth][gradientServoScale][numOrientations];
  
  for (int etaS = 0; etaS < gradientServoScale; etaS++) {
#pragma omp parallel for
    for (int etaY = -gradientServoTranslation; etaY < gradientServoTranslation; etaY += gsStride) {
      for (int etaX = -gradientServoTranslation; etaX < gradientServoTranslation; etaX += gsStride) {
        // get the patch
        int topCornerX = etaX + reticle.px - (aerialGradientReticleWidth/2);
        int topCornerY = etaY + reticle.py - (aerialGradientReticleWidth/2);
        Mat gCrop(maxDim, maxDim, CV_64F);
        
        for (int x = 0; x < maxDim; x++) {
          for (int y = 0; y < maxDim; y++) {
            int tx = x - tRx;
            int ty = y - tRy;
            int tCtx = topCornerX + tx;
            int tCty = topCornerY + ty;
            if ( (tx >= 0 && ty >= 0 && ty < crows && tx < ccols) &&
                 (tCtx > 0) && (tCty > 0) && (tCtx < imW) && (tCty < imH) ) {
              gCrop.at<double>(y, x) = frameGraySobel.at<double>(topCornerY + ty, topCornerX + tx);
            } else {
              gCrop.at<double>(y, x) = 0.0;
            }
          }
        }
        
        cv::resize(gCrop, gCrop, toBecome);
        
        processSaliency(gCrop, gCrop);
        
        {
          double mean = gCrop.dot(Mat::ones(aerialGradientWidth, aerialGradientWidth, gCrop.type())) / double(aerialGradientWidth*aerialGradientWidth);
          gCrop = gCrop - mean;
          double l2norm = gCrop.dot(gCrop);
          // ATTN 17
          // removed normalization for discriminative servoing
          // ATTN 15
          // normalization hoses rejection
          if (useGradientServoThresh) {
          } else {
            if (l2norm <= EPSILON)
              l2norm = 1.0;
            gCrop = gCrop / l2norm;
          }
        }
        
        for (int thisOrient = 0; thisOrient < numOrientations; thisOrient++) {
          // orientation cascade
          if (orientationCascade) {
            if (lastPtheta < lPTthresh) {
              if (thisOrient < orientationCascadeHalfWidth) {
                //cout << "skipping orientation " << thisOrient << endl;
                continue;
              }
              if (thisOrient > numOrientations - orientationCascadeHalfWidth) {
                //cout << "skipping orientation " << thisOrient << endl;
                continue;
              }
            }
          }
          
          // compute the score
          double thisScore = 0;
          thisScore = rotatedAerialGrads[thisOrient + etaS*numOrientations].dot(gCrop);
          
          int tEtaX = etaX+gradientServoTranslation;
          int tEtaY = etaY+gradientServoTranslation;
          allScores[tEtaX][tEtaY][etaS][thisOrient] = thisScore;
        }
      }
    }
  }
  
  // perform max
  for (int etaS = 0; etaS < gradientServoScale; etaS++) {
    for (int etaY = -gradientServoTranslation; etaY < gradientServoTranslation; etaY += gsStride) {
      for (int etaX = -gradientServoTranslation; etaX < gradientServoTranslation; etaX += gsStride) {
        // get the patch
        int topCornerX = etaX + reticle.px - (aerialGradientReticleWidth/2);
        int topCornerY = etaY + reticle.py - (aerialGradientReticleWidth/2);
        Mat gCrop(maxDim, maxDim, CV_64F);
        
        // throw it out if it isn't contained in the image
        //    if ( (topCornerX+aerialGradientWidth >= imW) || (topCornerY+aerialGradientWidth >= imH) )
        //      continue;
        //    if ( (topCornerX < 0) || (topCornerY < 0) )
        //      continue;
        
        for (int thisOrient = 0; thisOrient < numOrientations; thisOrient++) {
          // orientation cascade
          if (orientationCascade) {
            if (lastPtheta < lPTthresh) {
              if (thisOrient < orientationCascadeHalfWidth) {
                //cout << "skipping orientation " << thisOrient << endl;
                continue;
              }
              if (thisOrient > numOrientations - orientationCascadeHalfWidth) {
                //cout << "skipping orientation " << thisOrient << endl;
                continue;
              }
            }
          }
          
          int tEtaX = etaX+gradientServoTranslation;
          int tEtaY = etaY+gradientServoTranslation;
          double thisScore = allScores[tEtaX][tEtaY][etaS][thisOrient];
          
          if (thisScore > bestOrientationScore) {
            bestOrientation = thisOrient;
            bestOrientationScore = thisScore;
            bestCropNorm = sqrt(gCrop.dot(gCrop));
            bestX = etaX;
            bestY = etaY;
            bestS = etaS;
          }
          //cout << " this best: " << thisScore << " " << bestOrientationScore << " " << bestX << " " << bestY << endl;
        } 
      }
    }
  }

  // set the target reticle
  pilotTarget.px = reticle.px + bestX;
  pilotTarget.py = reticle.py + bestY;
  
  bestOrientationEEPose = currentEEPose;
  
  int oneToDraw = bestOrientation;
  Px = -bestX;
  Py = -bestY;
  
  //Ps = bestS - ((gradientServoScale-1)/2);
  Ps = 0;
  
  Mat toShow;
  Size toUnBecome(maxDim, maxDim);
  //cv::resize(classAerialGradients[targetClass], toShow, toUnBecome);
  //cv::resize(rotatedAerialGrads[oneToDraw], toShow, toUnBecome);
  cv::resize(rotatedAerialGrads[bestOrientation + bestS*numOrientations], toShow, toUnBecome);
  //cout << rotatedAerialGrads[oneToDraw];
  
  double maxTS = -INFINITY;
  double minTS = INFINITY;
  for (int x = 0; x < maxDim; x++) {
    for (int y = 0; y < maxDim; y++) {
      maxTS = max(maxTS, toShow.at<double>(y, x));
      minTS = min(minTS, toShow.at<double>(y, x));
    }
  }
  
  // draw the winning score in place
  for (int x = 0; x < maxDim; x++) {
    for (int y = 0; y < maxDim; y++) {
      //int tx = x - tRx;
      //int ty = y - tRy;
      int tx = x - tRx;
      int ty = y - tRy;
      if (tx >= 0 && ty >= 0 && ty < crows && tx < ccols) {
        Vec3b thisColor = Vec3b(0,0,min(255, int(floor(255.0*8*(toShow.at<double>(y, x)-minTS)/(maxTS-minTS)))));
        //Vec3b thisColor = Vec3b(0,0,min(255, int(floor(100000*toShow.at<double>(y, x)))));
        //Vec3b thisColor = Vec3b(0,0,min(255, int(floor(0.2*sqrt(toShow.at<double>(y, x))))));
        //cout << thisColor;
        int thisTopCornerX = bestX + reticle.px - (aerialGradientReticleWidth/2);
        int thisTopCornerY = bestY + reticle.py - (aerialGradientReticleWidth/2);
        
        int tgX = thisTopCornerX + tx;
        int tgY = thisTopCornerY + ty;
        if ((tgX > 0) && (tgX < imW) && (tgY > 0) && (tgY < imH)) {
          gradientViewerImage.at<Vec3b>(tgY, tgX) += thisColor;
        }
      }
    }
  }
  
  oneToDraw = oneToDraw % numOrientations;
  double Ptheta = min(bestOrientation, numOrientations - bestOrientation);
  lastPtheta = Ptheta;
  
  // change orientation according to winning rotation
  //currentEEPose.oz -= bestOrientation*2.0*3.1415926/double(numOrientations);
  
  // update after
  currentGradientServoIterations++;
  
  // XXX this still might miss if it nails the correct orientation on the last try
  // TODO but we could set the bestOrientationEEPose here according to what it would have been`
  // but we don't want to move because we want all the numbers to be consistent
  if (currentGradientServoIterations > hardMaxGradientServoIterations) {
    //cout << "LAST ITERATION indefinite orientation ";
  } else {
    double kPtheta = 0.0;
    if (Ptheta < kPThresh)
      kPtheta = kPtheta2;
    else
      kPtheta = kPtheta1;
    
    if (bestOrientation <= numOrientations/2)
      currentEEPose.oz -= kPtheta * bestOrientation*2.0*3.1415926/double(numOrientations);
    else
      currentEEPose.oz -= kPtheta * (-(numOrientations - bestOrientation))*2.0*3.1415926/double(numOrientations);
  }
  
  double doublePtheta = currentEEPose.oz;
  
  //cout << "gradient servo Px Py Ps bestOrientation Ptheta doublePtheta: " << Px << " " << Py << " " << Ps << " : " << reticle.px << " " << 
  //pilotTarget.px << " " << reticle.py << " " << pilotTarget.py << " " <<
  //bestOrientation << " " << Ptheta << " " << doublePtheta << endl;
  
  double dx = (currentEEPose.px - trueEEPose.position.x);
  double dy = (currentEEPose.py - trueEEPose.position.y);
  double dz = (currentEEPose.pz - trueEEPose.position.z);
  double distance = dx*dx + dy*dy + dz*dz;
  
  // ATTN 15
  // return to synchronic if the match is poor
  if (useGradientServoThresh) {
    cout << "ATTN score, thresh, norm, product: " << bestOrientationScore << " " << gradientServoResetThresh << " " << bestCropNorm << " " << (gradientServoResetThresh * bestCropNorm) << endl;
    if (bestOrientationScore < (gradientServoResetThresh * bestCropNorm) ) {
      pushWord("synchronicServo"); 
      pushWord("visionCycle"); 
      cout << " XXX BAD GRADIENT SERVO SCORE, RETURN TO SYNCHRONIC XXX" << endl;
      return;
    }
  }
  

  if (distance > w1GoThresh*w1GoThresh) {
    
    pushWord("gradientServo"); 
    // ATTN 8
    //pushCopies("density", densityIterationsForGradientServo); 
    //pushCopies("accumulateDensity", densityIterationsForGradientServo); 
    //pushCopies("resetTemporalMap", 1); 
    //pushWord("resetAerialGradientTemporalFrameAverage"); 
    //pushCopies("density", 1); 
    //pushCopies("waitUntilAtCurrentPosition", 5); 
    pushCopies("waitUntilAtCurrentPosition", 1); 
    
  } else {
    // ATTN 5
    // cannot proceed unless Ptheta = 0, since our best eePose is determined by our current pose and not where we WILL be after adjustment
    if (((fabs(Px) < gradServoPixelThresh) && (fabs(Py) < gradServoPixelThresh) && (fabs(Ptheta) < gradServoThetaThresh)) ||
        ((currentGradientServoIterations > softMaxGradientServoIterations) && (fabs(Ptheta) < gradServoThetaThresh)) || 
        (currentGradientServoIterations > hardMaxGradientServoIterations) )
      {
	// ATTN 23
	// move from vanishing point reticle to gripper reticle
	//moveCurrentGripperRayToCameraVanishingRay();
	//bestOrientationEEPose = currentEEPose;

        // ATTN 12
        if (ARE_GENERIC_HEIGHT_LEARNING()) {
          cout << "bbLearning: gradient servo succeeded. gradientServoDuringHeightLearning: " << gradientServoDuringHeightLearning << endl;
          cout << "bbLearning: returning from gradient servo." << endl;
          return;
        }
        
        // ATTN 17
        if (bailAfterGradient) {
          cout << "gradient servo set to bail. returning." << endl;
          return;
        }
        
        //cout << "got within thresh, returning." << endl;
        cout << "got within thresh, fetching." << endl;
        lastPtheta = INFINITY;
        cout << "resetting lastPtheta: " << lastPtheta << endl;
    
        if (synchronicTakeClosest) {
          if (gradientTakeClosest) {
            if ((classRangeMaps[targetClass].rows > 1) && (classRangeMaps[targetClass].cols > 1))
              pushWord("prepareForAndExecuteGraspFromMemoryLearning"); // prepare for and execute the best grasp from memory at the current location and target
            else {
              ROS_ERROR_STREAM("Cannot pick object with incomplete map.");
            }
          } else {
            return;
          }
        } else {
          if ((classRangeMaps[targetClass].rows > 1) && (classRangeMaps[targetClass].cols > 1)) {
            pushWord("prepareForAndExecuteGraspFromMemoryLearning"); 
          } else {
            ROS_ERROR_STREAM("Cannot pick object with incomplete map.");
          }
        }
        
        return;
      } else {
      
      pushWord("gradientServo"); 
      
      double pTermX = gradKp*Px;
      double pTermY = gradKp*Py;
      
      double pTermS = Ps * .005;
      currentEEPose.pz += pTermS;
      
      // invert the current eePose orientation to decide which direction to move from POV
      Eigen::Vector3f localUnitX;
      {
        Eigen::Quaternionf qin(0, 1, 0, 0);
        Eigen::Quaternionf qout(0, 1, 0, 0);
        Eigen::Quaternionf eeqform(trueEEPose.orientation.w, trueEEPose.orientation.x, trueEEPose.orientation.y, trueEEPose.orientation.z);
        qout = eeqform * qin * eeqform.conjugate();
        localUnitX.x() = qout.x();
        localUnitX.y() = qout.y();
        localUnitX.z() = qout.z();
      }
      
    Eigen::Vector3f localUnitY;
    {
      Eigen::Quaternionf qin(0, 0, 1, 0);
      Eigen::Quaternionf qout(0, 1, 0, 0);
      Eigen::Quaternionf eeqform(trueEEPose.orientation.w, trueEEPose.orientation.x, trueEEPose.orientation.y, trueEEPose.orientation.z);
      qout = eeqform * qin * eeqform.conjugate();
      localUnitY.x() = qout.x();
      localUnitY.y() = qout.y();
      localUnitY.z() = qout.z();
    }
    
    // ATTN 21
    //double newx = currentEEPose.px + pTermX*localUnitY.x() - pTermY*localUnitX.x();
    //double newy = currentEEPose.py + pTermX*localUnitY.y() - pTermY*localUnitX.y();
    double newx = 0;
    double newy = 0;
    // first analytic
    //double zToUse = trueEEPose.position.z+currentTableZ;
    //pixelToGlobal(pilotTarget.px, pilotTarget.py, zToUse, &newx, &newy);
    // old PID
    //currentEEPose.py += pTermX*localUnitY.y() - pTermY*localUnitX.y();
    //currentEEPose.px += pTermX*localUnitY.x() - pTermY*localUnitX.x();
    // ATTN 23
    // second analytic
    eePose newGlobalTarget = analyticServoPixelToReticle(pilotTarget, reticle, currentEEPose.oz);
    newx = newGlobalTarget.px;
    newy = newGlobalTarget.py;
    currentEEPose.px = newx;
    currentEEPose.py = newy;
    
    // ATTN 8
    //pushWord("visionCycle"); // vision cycle
    //pushWord(196721); // vision cycle no classify
    //pushCopies("density", densityIterationsForGradientServo); // density
    //pushCopies("accumulateDensity", densityIterationsForGradientServo); 
    //pushCopies("resetTemporalMap", 1); // reset temporal map
    //pushWord("resetAerialGradientTemporalFrameAverage"); // reset aerialGradientTemporalFrameAverage
    //pushCopies("density", 1); // density
    //pushWord("waitUntilAtCurrentPosition"); 
    
    // ATTN 7
    // if you don't wait multiple times, it could get triggered early by weird ik or latency could cause a loop
    // this is a very aggressive choice and we should also be using the ring buffers for Ode calls
    //pushCopies("waitUntilAtCurrentPosition", 40); 
    //pushCopies("waitUntilAtCurrentPosition", 5); 
    pushCopies("waitUntilAtCurrentPosition", 1); 
    
    // ATTN 16
    //	    { // prepare to servo
    //	      currentEEPose.pz = wholeFoodsCounter1.pz+.1;
    //	    }
    }
  }
}

// given pixel is the pixel in the current frame that you want to be at the vanishing point
//  after undergoing a rotaion of ozAngle about the end effector Z axis
eePose analyticServoPixelToReticle(eePose givenPixel, eePose givenReticle, double ozAngle) {
  eePose toReturn = trueEEPoseEEPose;
  eePose grGlobalPostRotation = trueEEPoseEEPose;
  eePose grGlobalPreRotation = trueEEPoseEEPose;
  eePose gpGlobalPreRotation = trueEEPoseEEPose;
  {
    double zToUse = trueEEPose.position.z+currentTableZ;
    pixelToGlobal(givenPixel.px, givenPixel.py, zToUse, &(gpGlobalPreRotation.px), &(gpGlobalPreRotation.py));
  }
  {
    double zToUse = trueEEPose.position.z+currentTableZ;
    pixelToGlobal(givenReticle.px, givenReticle.py, zToUse, &(grGlobalPreRotation.px), &(grGlobalPreRotation.py));
  }

  eePose fakeEndEffector = currentEEPose;
  fakeEndEffector.ox = 0;
  fakeEndEffector.oy = 0;
  fakeEndEffector.oz = ozAngle;
  endEffectorAngularUpdate(&fakeEndEffector);
  {
    double zToUse = trueEEPose.position.z+currentTableZ;
    pixelToGlobal(givenReticle.px, givenReticle.py, zToUse, &(grGlobalPostRotation.px), &(grGlobalPostRotation.py), fakeEndEffector);
  }
  double  postRotationTranslationX = (gpGlobalPreRotation.px - grGlobalPostRotation.px);
  double  postRotationTranslationY = (gpGlobalPreRotation.py - grGlobalPostRotation.py);

  toReturn.px += postRotationTranslationX;
  toReturn.py += postRotationTranslationY;
  return toReturn;
}

void synchronicServo() {
  ROS_WARN_STREAM("___________________ Synchronic Servo");
  synServoLockFrames++;

  // ATTN 23
  //reticle = heightReticles[currentThompsonHeightIdx];
  eePose thisGripperReticle;
  double zToUse = trueEEPose.position.z+currentTableZ;
  int xOut=-1, yOut=-1;
  globalToPixel(&xOut, &yOut, zToUse, trueEEPoseEEPose.px, trueEEPoseEEPose.py);
  thisGripperReticle.px = xOut;
  thisGripperReticle.py = yOut;
  reticle = vanishingPointReticle;

  // ATTN 17
  currentGradientServoIterations = 0;

  // ATTN 12
  // if we time out, reset the bblearning program
  if ( ((synServoLockFrames > heightLearningServoTimeout) || (bTops.size() <= 0)) && 
	(ARE_GENERIC_HEIGHT_LEARNING()) ) {
    cout << "bbLearning: synchronic servo early outting: ";
    if (bTops.size() <= 0) {
      cout << "NO BLUE BOXES ";
    }
    if ((synServoLockFrames > heightLearningServoTimeout) && (ARE_GENERIC_HEIGHT_LEARNING())) {
      cout << "TIMED OUT ";
    }
    cout << endl;
    restartBBLearning();
    return;
  }

  if ( ((synServoLockFrames > mappingServoTimeout) || (bTops.size() <= 0)) && 
	(currentBoundingBoxMode == MAPPING) ) {
    cout << ">>>> Synchronic servo timed out or no blue boxes during mapping. <<<<" << endl;
    return;
  }

  {
    int i, j;
    mapxyToij(currentEEPose.px, currentEEPose.py, &i, &j);
    int doWeHaveClearance = (clearanceMap[i + mapWidth * j] != 0);
    if (!doWeHaveClearance) {
      cout << ">>>> Synchronic servo strayed out of clearance area during mapping. <<<<" << endl;
      return;
    }
  }

  // ATTN 19
  // if we time out, reset the pick learning 
  if ( ((synServoLockFrames > heightLearningServoTimeout)) && 
	ARE_GENERIC_PICK_LEARNING() ) {
    // record a failure
    cout << ">>>> Synchronic servo timed out.  Going back on patrol. <<<<" << endl;
    thisGraspPicked = FAILURE; 
    pushWord("shiftIntoGraspGear1"); 
    pushCopies("beep", 15); 
    pushWord("countGrasp"); 
    return; 
  }

  if (bTops.size() <= 0) {
    cout << ">>>> HELP,  I CAN'T SEE!!!!! Going back on patrol. <<<<" << endl;
    pushWord("visionCycle"); 
    pushWord("waitUntilAtCurrentPosition"); 
    return;
  }

  if (synchronicTakeClosest &&
      (pilotClosestTarget.px != -1) && (pilotClosestTarget.py != -1)) {
    pilotTarget.px = pilotClosestTarget.px;
    pilotTarget.py = pilotClosestTarget.py;
    pilotTarget.pz = pilotClosestTarget.pz;
    pilotTargetBlueBoxNumber = pilotClosestBlueBoxNumber;
  } else {
    return;
  }

  // target the closest blue box that hasn't been mapped since
  //  the last mapping started
  if (currentBoundingBoxMode == MAPPING) {
    int foundAnUnmappedTarget = 0;
    int closestUnmappedBBToReticle = -1;
    double closestBBDistance = VERYBIGNUMBER;
    for (int c = 0; c < bTops.size(); c++) {
      double tbx, tby;
      int tbi, tbj;
      double zToUse = trueEEPose.position.z+currentTableZ;
      pixelToGlobal(bCens[c].x, bCens[c].y, zToUse, &tbx, &tby);
      mapxyToij(tbx, tby, &tbi, &tbj);
      
      ros::Time thisLastMappedTime = objectMap[tbi + mapWidth * tbj].lastMappedTime;
      ros::Time thisNow = ros::Time::now();

      // ATTN 23
      //int isUnmapped = (thisLastMappedTime < lastScanStarted);
      int isCooldownComplete = (thisNow.sec - thisLastMappedTime.sec) > mapBlueBoxCooldown;

      int isOutOfReach = ( !positionIsSearched(tbx, tby) || 
			 !isBlueBoxIKPossible(bTops[c], bBots[c]) ); 

      double thisDistance = sqrt((bCens[c].x-reticle.px)*(bCens[c].x-reticle.px) + (bCens[c].y-reticle.py)*(bCens[c].y-reticle.py));
      cout << "   Servo CUB distance for box " << c << " : " << thisDistance << ", isCooldownComplete isOutOfReach: " <<
	      isCooldownComplete << " " << isOutOfReach << endl;
      cout << "      (thisNow - thisLastMappedTime) mapBlueBoxCooldown:" << 
	      thisNow.sec - thisLastMappedTime.sec << " " << mapBlueBoxCooldown << " " <<  endl;

      if (isOutOfReach) {
	mapBlueBox(bTops[c], bBots[c], 0, ros::Time::now()+ros::Duration(mapBlueBoxCooldown));
      }

      if ( isCooldownComplete  && 
	   !isOutOfReach ) {
	if (thisDistance < closestBBDistance) {
	  closestBBDistance = thisDistance;
	  closestUnmappedBBToReticle = c;
	  foundAnUnmappedTarget = 1;
	}
      } else {
	// XXX this causes a bug
	//mapBlueBox(bTops[c], bBots[c], 0);
      }
    }

    if (foundAnUnmappedTarget) {
      pilotClosestBlueBoxNumber = closestUnmappedBBToReticle;
      pilotTarget.px = bCens[pilotClosestBlueBoxNumber].x;
      pilotTarget.py = bCens[pilotClosestBlueBoxNumber].y;
      pilotTarget.pz = 0;
      pilotClosestTarget = pilotTarget;
    } else {
      // this prevents gradient servo 
      pilotClosestBlueBoxNumber = -1;
      bTops.resize(0);
      bBots.resize(0);
      bCens.resize(0);
      bLabels.resize(0);
    }
  }


  double Px = reticle.px - pilotTarget.px;
  double Py = reticle.py - pilotTarget.py;

  double dx = (currentEEPose.px - trueEEPose.position.x);
  double dy = (currentEEPose.py - trueEEPose.position.y);
  double dz = (currentEEPose.pz - trueEEPose.position.z);
  double distance = dx*dx + dy*dy + dz*dz;

  // if we are not there yet, continue
  if (distance > w1GoThresh*w1GoThresh) {
    synServoLockFrames = 0;
    pushWord("synchronicServo"); 
    if (currentBoundingBoxMode == MAPPING) {
      pushWord("visionCycleNoClassify");
    } else {
      pushWord("visionCycle"); // vision cycle
    }
  } else {
    if ((fabs(Px) < synServoPixelThresh) && (fabs(Py) < synServoPixelThresh)) {
      // ATTN 12
      if (ARE_GENERIC_HEIGHT_LEARNING()) {
	cout << "bbLearning: synchronic servo succeeded. gradientServoDuringHeightLearning: " << gradientServoDuringHeightLearning << endl;
	if (gradientServoDuringHeightLearning) {
	  cout << "bbLearning: proceeding to gradient servo." << endl;
	} else {
	  cout << "bbLearning: returning from synchronic servo." << endl;
	  return;
	}
      }

      // ATTN 17
      if (bailAfterSynchronic) {
	cout << "synchronic servo set to bail. returning." << endl;
	return;
      }

      cout << "got within thresh. ";
      if ((classAerialGradients[targetClass].rows > 1) && (classAerialGradients[targetClass].cols > 1)) {
        pushWord("gradientServo"); 
        cout << "Queuing gradient servo." << endl;
        //pushCopies("density", densityIterationsForGradientServo); 
	//pushCopies("accumulateDensity", densityIterationsForGradientServo); 
        //pushCopies("resetTemporalMap", 1); 
        //pushWord("resetAerialGradientTemporalFrameAverage"); 
        //pushCopies("density", 1); 
        //pushCopies("waitUntilAtCurrentPosition", 5); 
        pushCopies("waitUntilAtCurrentPosition", 1); 
        
      } else {
        ROS_ERROR_STREAM("No gradient map for class " << targetClass << endl);
        clearStack();
      }

      return;	
    } else {

      double thisKp = synKp;
      double pTermX = thisKp*Px;
      double pTermY = thisKp*Py;

      // invert the current eePose orientation to decide which direction to move from POV
      Eigen::Vector3f localUnitX;
      {
	Eigen::Quaternionf qin(0, 1, 0, 0);
	Eigen::Quaternionf qout(0, 1, 0, 0);
	Eigen::Quaternionf eeqform(trueEEPose.orientation.w, trueEEPose.orientation.x, trueEEPose.orientation.y, trueEEPose.orientation.z);
	qout = eeqform * qin * eeqform.conjugate();
	localUnitX.x() = qout.x();
	localUnitX.y() = qout.y();
	localUnitX.z() = qout.z();
      }

      Eigen::Vector3f localUnitY;
      {
	Eigen::Quaternionf qin(0, 0, 1, 0);
	Eigen::Quaternionf qout(0, 1, 0, 0);
	Eigen::Quaternionf eeqform(trueEEPose.orientation.w, trueEEPose.orientation.x, trueEEPose.orientation.y, trueEEPose.orientation.z);
	qout = eeqform * qin * eeqform.conjugate();
	localUnitY.x() = qout.x();
	localUnitY.y() = qout.y();
	localUnitY.z() = qout.z();
      }

      // ATTN 21
      // old PID
      //double newx = currentEEPose.px + pTermX*localUnitY.x() - pTermY*localUnitX.x();
      //double newy = currentEEPose.py + pTermX*localUnitY.y() - pTermY*localUnitX.y();
      double newx = 0;
      double newy = 0;
      // first analytic
      //double zToUse = trueEEPose.position.z+currentTableZ;
      //pixelToGlobal(pilotTarget.px, pilotTarget.py, zToUse, &newx, &newy);
      // ATTN 23
      eePose newGlobalTarget = analyticServoPixelToReticle(pilotTarget, reticle, 0);
      newx = newGlobalTarget.px;
      newy = newGlobalTarget.py;

      if (!positionIsMapped(newx, newy)) {
        cout << "Returning because position is out of map bounds." << endl;
        return;
      } else {
        pushWord("synchronicServo"); 
	// ATTN 21
        //currentEEPose.px += pTermX*localUnitY.x() - pTermY*localUnitX.x();
        //currentEEPose.py += pTermX*localUnitY.y() - pTermY*localUnitX.y();
        currentEEPose.px = newx;
        currentEEPose.py = newy;



	if (currentBoundingBoxMode == MAPPING) {
	  pushWord("visionCycleNoClassify");
	} else {
	  pushWord("visionCycle"); // vision cycle
	}

        pushWord("waitUntilAtCurrentPosition"); 
      }
    }
  }
}

/*
int simulatedServo() {
  cout << "Starting simulated servo with " << numClasses << " classes." << endl;

  Size sz = objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;

  reticle = heightReticles[currentThompsonHeightIdx];

  int bestClass = -1;
  double bestOrientationScore = -INFINITY;

  for (int thisCandidateClass = 0; thisCandidateClass < numClasses; thisCandidateClass++) {
    cout << "Simulated Servo class " << thisCandidateClass << ": "; 
    if ((classAerialGradients[thisCandidateClass].rows <= 1) && (classAerialGradients[thisCandidateClass].cols <= 1)) {
      cout << "no aerial gradients for this class, skipping." << endl;
      continue;
    } else {
      cout << "found aerial gradients for this class, considering." << endl;
    } 

    switch (currentThompsonHeightIdx) {
    case 0:
      {
	classAerialGradients[thisCandidateClass] = classHeight0AerialGradients[thisCandidateClass];
      }
      break;
    case 1:
      {
	classAerialGradients[thisCandidateClass] = classHeight1AerialGradients[thisCandidateClass];
      }
      break;
    case 2:
      {
	classAerialGradients[thisCandidateClass] = classHeight2AerialGradients[thisCandidateClass];
      }
      break;
    case 3:
      {
	classAerialGradients[thisCandidateClass] = classHeight3AerialGradients[thisCandidateClass];
      }
      break;
    default:
      {
	assert(0);
      }
      break;
    }


    Size toBecome(aerialGradientWidth, aerialGradientWidth);

    int numOrientations = 37;
    vector<Mat> rotatedAerialGrads;

    // ATTN 3
    // gradientServoScale should be even
    int gradientServoScale = 3;//11;
    double gradientServoScaleStep = 1.02;
    double startScale = pow(gradientServoScaleStep, -(gradientServoScale-1)/2);

    //rotatedAerialGrads.resize(numOrientations);
    rotatedAerialGrads.resize(gradientServoScale*numOrientations);

    for (int etaS = 0; etaS < gradientServoScale; etaS++) {
      double thisScale = startScale * pow(gradientServoScaleStep, etaS);
      for (int thisOrient = 0; thisOrient < numOrientations; thisOrient++) {
	// rotate the template and L2 normalize it
	Point center = Point(aerialGradientWidth/2, aerialGradientWidth/2);
	double angle = thisOrient*360.0/numOrientations;

	double scale = thisScale;

	// Get the rotation matrix with the specifications above
	Mat rot_mat = getRotationMatrix2D(center, angle, scale);
	warpAffine(classAerialGradients[thisCandidateClass], rotatedAerialGrads[thisOrient + etaS*numOrientations], rot_mat, toBecome);

	processSaliency(rotatedAerialGrads[thisOrient + etaS*numOrientations], rotatedAerialGrads[thisOrient + etaS*numOrientations]);


	double mean = rotatedAerialGrads[thisOrient + etaS*numOrientations].dot(Mat::ones(aerialGradientWidth, aerialGradientWidth, rotatedAerialGrads[thisOrient + etaS*numOrientations].type())) / double(aerialGradientWidth*aerialGradientWidth);
	rotatedAerialGrads[thisOrient + etaS*numOrientations] = rotatedAerialGrads[thisOrient + etaS*numOrientations] - mean;
	double l2norm = rotatedAerialGrads[thisOrient + etaS*numOrientations].dot(rotatedAerialGrads[thisOrient + etaS*numOrientations]);
	if (l2norm <= EPSILON)
	  l2norm = 1.0;
	rotatedAerialGrads[thisOrient + etaS*numOrientations] = rotatedAerialGrads[thisOrient + etaS*numOrientations] / l2norm;
      }
    }

    int bestOrientation = -1;
    double bestCropNorm = 1.0;
    int bestX = -1;
    int bestY = -1;
    int bestS = -1;

    int crows = aerialGradientReticleWidth;
    int ccols = aerialGradientReticleWidth;
    int maxDim = max(crows, ccols);
    int tRy = (maxDim-crows)/2;
    int tRx = (maxDim-ccols)/2;

    //int gradientServoTranslation = 40;
    //int gsStride = 2;
    int gradientServoTranslation = 40;
    int gsStride = 2;

    //rotatedAerialGrads.resize(gradientServoScale*numOrientations);
    int gSTwidth = 2*gradientServoTranslation + 1;
    double allScores[gSTwidth][gSTwidth][gradientServoScale][numOrientations];

    for (int etaS = 0; etaS < gradientServoScale; etaS++) {
#pragma omp parallel for
      for (int etaY = -gradientServoTranslation; etaY < gradientServoTranslation; etaY += gsStride) {
	for (int etaX = -gradientServoTranslation; etaX < gradientServoTranslation; etaX += gsStride) {
	  // get the patch
	  int topCornerX = etaX + reticle.px - (aerialGradientReticleWidth/2);
	  int topCornerY = etaY + reticle.py - (aerialGradientReticleWidth/2);
	  Mat gCrop(maxDim, maxDim, CV_64F);

	  for (int x = 0; x < maxDim; x++) {
	    for (int y = 0; y < maxDim; y++) {
	      int tx = x - tRx;
	      int ty = y - tRy;
	      int tCtx = topCornerX + tx;
	      int tCty = topCornerY + ty;
	      if ( (tx >= 0 && ty >= 0 && ty < crows && tx < ccols) &&
					       (tCtx > 0) && (tCty > 0) && (tCtx < imW) && (tCty < imH) ) {
	      gCrop.at<double>(y, x) = frameGraySobel.at<double>(topCornerY + ty, topCornerX + tx);
	      } else {
		gCrop.at<double>(y, x) = 0.0;
	      }
	    }
	  }

	  cv::resize(gCrop, gCrop, toBecome);

	  processSaliency(gCrop, gCrop);

	  {
	    double mean = gCrop.dot(Mat::ones(aerialGradientWidth, aerialGradientWidth, gCrop.type())) / double(aerialGradientWidth*aerialGradientWidth);
	    gCrop = gCrop - mean;
	    double l2norm = gCrop.dot(gCrop);

	    //cout << l2norm << " " << endl;
	    if (l2norm <= EPSILON)
	      l2norm = 1.0;
	    gCrop = gCrop / l2norm;
	  }

	  for (int thisOrient = 0; thisOrient < numOrientations; thisOrient++) {
	    // orientation cascade

	    // compute the score
	    double thisScore = 0;
	    thisScore = rotatedAerialGrads[thisOrient + etaS*numOrientations].dot(gCrop);

	    int tEtaX = etaX+gradientServoTranslation;
	    int tEtaY = etaY+gradientServoTranslation;
	    allScores[tEtaX][tEtaY][etaS][thisOrient] = thisScore;
	  }
	}
      }
    }

    // perform max
    for (int etaS = 0; etaS < gradientServoScale; etaS++) {
      for (int etaY = -gradientServoTranslation; etaY < gradientServoTranslation; etaY += gsStride) {
	for (int etaX = -gradientServoTranslation; etaX < gradientServoTranslation; etaX += gsStride) {
	  // get the patch
	  int topCornerX = etaX + reticle.px - (aerialGradientReticleWidth/2);
	  int topCornerY = etaY + reticle.py - (aerialGradientReticleWidth/2);
	  Mat gCrop(maxDim, maxDim, CV_64F);

	  // throw it out if it isn't contained in the image
//	  if ( (topCornerX+aerialGradientWidth >= imW) || (topCornerY+aerialGradientWidth >= imH) )
//	    continue;
//	  if ( (topCornerX < 0) || (topCornerY < 0) )
//	    continue;

	  for (int thisOrient = 0; thisOrient < numOrientations; thisOrient++) {
	    int tEtaX = etaX+gradientServoTranslation;
	    int tEtaY = etaY+gradientServoTranslation;
	    double thisScore = allScores[tEtaX][tEtaY][etaS][thisOrient];

	    if (thisScore > bestOrientationScore) {
	      bestOrientation = thisOrient;
	      bestOrientationScore = thisScore;
	      bestCropNorm = sqrt(gCrop.dot(gCrop));
	      bestX = etaX;
	      bestY = etaY;
	      bestS = etaS;
	      bestClass = thisCandidateClass;
	      cout << " new best score, class: " << bestOrientationScore << " " << bestClass << " " << classLabels[bestClass] << endl;
	    }
	  } 
	}
      }
    }

    if (bestOrientation > -1) {
      int oneToDraw = bestOrientation;

      Mat toShow;
      Size toUnBecome(maxDim, maxDim);
      cv::resize(rotatedAerialGrads[bestOrientation + bestS*numOrientations], toShow, toUnBecome);
      double maxTS = -INFINITY;
      double minTS = INFINITY;

      for (int x = 0; x < maxDim; x++) {
	for (int y = 0; y < maxDim; y++) {
	  maxTS = max(maxTS, toShow.at<double>(y, x));
	  minTS = min(minTS, toShow.at<double>(y, x));
	}
      }

      // draw the winning score in place
      for (int x = 0; x < maxDim; x++) {
	for (int y = 0; y < maxDim; y++) {
	  //int tx = x - tRx;
	  //int ty = y - tRy;
	  int tx = x - tRx;
	  int ty = y - tRy;
	  if (tx >= 0 && ty >= 0 && ty < crows && tx < ccols) {
	    Vec3b thisColor = Vec3b(0,0,min(255, int(floor(255.0*8*(toShow.at<double>(y, x)-minTS)/(maxTS-minTS)))));
	    //Vec3b thisColor = Vec3b(0,0,min(255, int(floor(100000*toShow.at<double>(y, x)))));
	    //Vec3b thisColor = Vec3b(0,0,min(255, int(floor(0.2*sqrt(toShow.at<double>(y, x))))));
	    //cout << thisColor;
	    int thisTopCornerX = bestX + reticle.px - (aerialGradientReticleWidth/2);
	    int thisTopCornerY = bestY + reticle.py - (aerialGradientReticleWidth/2);
	    gradientViewerImage.at<Vec3b>(thisTopCornerY + ty, thisTopCornerX + tx) += thisColor;
	  }
	}
      }
    }



  }

  return bestClass;
}
*/

void initRangeMaps() {
  classRangeMaps.resize(numClasses);
  classGraspMemoryTries1.resize(numClasses);
  classGraspMemoryPicks1.resize(numClasses);
  classGraspMemoryTries2.resize(numClasses);
  classGraspMemoryPicks2.resize(numClasses);
  classGraspMemoryTries3.resize(numClasses);
  classGraspMemoryPicks3.resize(numClasses);
  classGraspMemoryTries4.resize(numClasses);
  classGraspMemoryPicks4.resize(numClasses);
  classAerialGradients.resize(numClasses);

  // ATTN 16
  classHeight0AerialGradients.resize(numClasses);
  classHeight1AerialGradients.resize(numClasses);
  classHeight2AerialGradients.resize(numClasses);
  classHeight3AerialGradients.resize(numClasses);

  classHeightMemoryTries.resize(numClasses);
  classHeightMemoryPicks.resize(numClasses);
  for (int i = 0; i < classLabels.size(); i++) {
    tryToLoadRangeMap(class_crops_path, classLabels[i].c_str(), i);
  }
}

int isThisGraspMaxedOut(int i) {
  int toReturn = 0;

  if (currentPickMode == LEARNING_SAMPLING) {
    toReturn = ( (graspMemoryTries[i] >= graspLearningMaxTries) );
  } else if (currentPickMode == LEARNING_ALGORITHMC) {
    // ATTN 20
    double successes = graspMemoryPicks[i];
    double failures = graspMemoryTries[i] - graspMemoryPicks[i];
    cout << "YYY failures, successes: " << failures << " " << successes << endl;
    successes = round(successes);
    failures = round(failures);
    cout << "XXX failures, successes: " << failures << " " << successes << endl;
    // returns probability that mu <= d given successes and failures.
    double result = cephes_incbet(successes + 1, failures + 1, algorithmCTarget);
    toReturn = (result > algorithmCRT);
  } else if (currentPickMode == STATIC_MARGINALS) {
    //toReturn = (graspMemoryTries[i] <= 1);
  }

  return toReturn;
}

eePose pixelToGlobalEEPose(int pX, int pY, double gZ) {
  eePose result;
  pixelToGlobal(pX, pY, gZ, &result.px, &result.py);
  result.pz = trueEEPose.position.z - currentTableZ;
  result.ox = 0;
  result.oy = 0;
  result.oz = 0;
  result.qx = 0;
  result.qy = 0;
  result.qz = 0;
  return result;
}

void interpolateM_xAndM_yFromZ(double dZ, double * m_x, double * m_y) {

  // XXX disabling for calibration
  //return;

  double bBZ[4];
  bBZ[0] = convertHeightIdxToGlobalZ(0) + currentTableZ;
  bBZ[1] = convertHeightIdxToGlobalZ(1) + currentTableZ;
  bBZ[2] = convertHeightIdxToGlobalZ(2) + currentTableZ;
  bBZ[3] = convertHeightIdxToGlobalZ(3) + currentTableZ;

  if (dZ <= bBZ[0]) {
    *m_x = m_x_h[0];
    *m_y = m_y_h[0];
  } else if (dZ <= bBZ[1]) {
    double gap = bBZ[1] - bBZ[0];
    double c0 = 1.0 - ((dZ - bBZ[0])/gap);
    double c1 = 1.0 - ((bBZ[1] - dZ)/gap);
    *m_x = c0*m_x_h[0] + c1*m_x_h[1];
    *m_y = c0*m_y_h[0] + c1*m_y_h[1];
  } else if (dZ <= bBZ[2]) {
    double gap = bBZ[2] - bBZ[1];
    double c1 = 1.0 - ((dZ - bBZ[1])/gap);
    double c2 = 1.0 - ((bBZ[2] - dZ)/gap);
    *m_x = c1*m_x_h[1] + c2*m_x_h[2];
    *m_y = c1*m_y_h[1] + c2*m_y_h[2];
  } else if (dZ <= bBZ[3]) {
    double gap = bBZ[3] - bBZ[2];
    double c2 = 1.0 - ((dZ - bBZ[2])/gap);
    double c3 = 1.0 - ((bBZ[3] - dZ)/gap);
    *m_x = c2*m_x_h[2] + c3*m_x_h[3];
    *m_y = c2*m_y_h[2] + c3*m_y_h[3];
  } else if (dZ > bBZ[3]) {
    *m_x = m_x_h[3];
    *m_y = m_y_h[3];
  } else {
    assert(0); // my my
  }
  //cout << m_x_h[0] << " " << m_x_h[1] << " " << m_x_h[2] << " " << m_x_h[3] << " " << *m_x << endl;
  //cout << m_y_h[0] << " " << m_y_h[1] << " " << m_y_h[2] << " " << m_y_h[3] << " " << *m_y << endl;
}

void pixelToGlobal(int pX, int pY, double gZ, double * gX, double * gY) {
  pixelToGlobal(pX, pY, gZ, gX, gY, trueEEPoseEEPose);
}

void pixelToGlobal(int pX, int pY, double gZ, double * gX, double * gY, eePose givenEEPose) {
  interpolateM_xAndM_yFromZ(gZ, &m_x, &m_y);

  int x1 = heightReticles[0].px;
  int x2 = heightReticles[1].px;
  int x3 = heightReticles[2].px;
  int x4 = heightReticles[3].px;

  int y1 = heightReticles[0].py;
  int y2 = heightReticles[1].py;
  int y3 = heightReticles[2].py;
  int y4 = heightReticles[3].py;

  double z1 = convertHeightIdxToGlobalZ(0) + currentTableZ;
  double z2 = convertHeightIdxToGlobalZ(1) + currentTableZ;
  double z3 = convertHeightIdxToGlobalZ(2) + currentTableZ;
  double z4 = convertHeightIdxToGlobalZ(3) + currentTableZ;

  double reticlePixelX = 0.0;
  double reticlePixelY = 0.0;
  {
    double d = d_x;
    double c = ((z4*x4-z2*x2)*(x3-x1)-(z3*x3-z1*x1)*(x4-x2))/((z1-z3)*(x4-x2)-(z2-z4)*(x3-x1));

    double b42 = (z4*x4-z2*x2+(z2-z4)*c)/(x4-x2);
    double b31 = (z3*x3-z1*x1+(z1-z3)*c)/(x3-x1);

    double bDiff = b42-b31;
    double b = (b42+b31)/2.0;

    int x_thisZ = c + ( (x1-c)*(z1-b) )/(gZ-b);
    x_thisZ = c + ( (d)*(x_thisZ-c) )/(d);
    reticlePixelX = x_thisZ;
  }
  {
    double d = d_y;
    double c = ((z4*y4-z2*y2)*(y3-y1)-(z3*y3-z1*y1)*(y4-y2))/((z1-z3)*(y4-y2)-(z2-z4)*(y3-y1));

    double b42 = (z4*y4-z2*y2+(z2-z4)*c)/(y4-y2);
    double b31 = (z3*y3-z1*y1+(z1-z3)*c)/(y3-y1);

    double bDiff = b42-b31;
    double b = (b42+b31)/2.0;

    int y_thisZ = c + ( (y1-c)*(z1-b) )/(gZ-b);
    y_thisZ = c + ( (d)*(y_thisZ-c) )/(d);
    reticlePixelY = y_thisZ;
  }

  // account for rotation of the end effector 
  Quaternionf eeqform(givenEEPose.qw, givenEEPose.qx, givenEEPose.qy, givenEEPose.qz);
  Quaternionf crane2Orient(0, 1, 0, 0);
  Quaternionf rel = eeqform * crane2Orient.inverse();
  Quaternionf ex(0,1,0,0);
  Quaternionf zee(0,0,0,1);
	
  Quaternionf result = rel * ex * rel.conjugate();
  Quaternionf thumb = rel * zee * rel.conjugate();
  double aY = result.y();
  double aX = result.x();

  // ATTN 22
  //double angle = atan2(aY, aX)*180.0/3.1415926;
  double angle = vectorArcTan(aY, aX)*180.0/3.1415926;
  angle = (angle);
  double scale = 1.0;
  Point center = Point(reticlePixelX, reticlePixelY);

  Mat un_rot_mat = getRotationMatrix2D( center, angle, scale );

  Mat toUn(3,1,CV_64F);
  toUn.at<double>(0,0)=pX;
  toUn.at<double>(1,0)=pY;
  toUn.at<double>(2,0)=1.0;
  Mat didUn = un_rot_mat*toUn;
  pX = didUn.at<double>(0,0);
  pY = didUn.at<double>(1,0);

  double oldPx = pX;
  double oldPy = pY;
  pX = reticlePixelX + (oldPy - reticlePixelY) - offX;
  pY = reticlePixelY + (oldPx - reticlePixelX) - offY;

  {
    double d = d_x/m_x;
    double c = ((z4*x4-z2*x2)*(x3-x1)-(z3*x3-z1*x1)*(x4-x2))/((z1-z3)*(x4-x2)-(z2-z4)*(x3-x1));

    double b42 = (z4*x4-z2*x2+(z2-z4)*c)/(x4-x2);
    double b31 = (z3*x3-z1*x1+(z1-z3)*c)/(x3-x1);

    double bDiff = b42-b31;
    //cout << "x1 x2 x3 x4: " << x1 << " " << x2 << " " << x3 << " " << x4 << endl;
    //cout << "y1 y2 y3 y4: " << y1 << " " << y2 << " " << y3 << " " << y4 << endl;
    //cout << "z1 z2 z3 z4: " << z1 << " " << z2 << " " << z3 << " " << z4 << endl;
    //cout << "bDiff = " << bDiff << ", c = " << c << " b42, b31: " << b42 << " " << b31 << " " << endl;
    double b = (b42+b31)/2.0;

    int x_thisZ = c + ( (x1-c)*(z1-b) )/(gZ-b);
    //int x_thisZ = c + ( m_x*(x1-c)*(z1-b) )/(gZ-b);
    //*gX = d + ( (pX-c)*(currentEEPose.px-d) )/(x1-c) ;
    //*gX = givenEEPose.px - d + ( (pX-c)*(d) )/( (x_thisZ-c)*m_x ) ;
    *gX = givenEEPose.px - d + ( (pX-c)*(d) )/( (x_thisZ-c) ) ;
    x_thisZ = c + ( (d)*(x_thisZ-c) )/(d);
  }
  {
    double d = d_y/m_y;
    double c = ((z4*y4-z2*y2)*(y3-y1)-(z3*y3-z1*y1)*(y4-y2))/((z1-z3)*(y4-y2)-(z2-z4)*(y3-y1));

    double b42 = (z4*y4-z2*y2+(z2-z4)*c)/(y4-y2);
    double b31 = (z3*y3-z1*y1+(z1-z3)*c)/(y3-y1);

    double bDiff = b42-b31;
    //cout << "x1 x2 x3 x4: " << x1 << " " << x2 << " " << x3 << " " << x4 << endl;
    //cout << "y1 y2 y3 y4: " << y1 << " " << y2 << " " << y3 << " " << y4 << endl;
    //cout << "z1 z2 z3 z4: " << z1 << " " << z2 << " " << z3 << " " << z4 << endl;
    //cout << "bDiff = " << bDiff << ", c = " << c << " b42, b31: " << b42 << " " << b31 << " " << endl;
    double b = (b42+b31)/2.0;

    int y_thisZ = c + ( (y1-c)*(z1-b) )/(gZ-b);
    //int y_thisZ = c + ( m_y*(y1-c)*(z1-b) )/(gZ-b);
    //*gY = d + ( (pY-c)*(currentEEPose.py-d) )/(y1-c) ;
    //*gY = givenEEPose.py - d + ( (pY-c)*(d) )/( (y_thisZ-c)*m_y ) ;
    *gY = givenEEPose.py - d + ( (pY-c)*(d) )/( (y_thisZ-c) ) ;
    y_thisZ = c + ( (d)*(y_thisZ-c) )/(d);
  }
}

void globalToPixel(int * pX, int * pY, double gZ, double gX, double gY) {
  interpolateM_xAndM_yFromZ(gZ, &m_x, &m_y);

  int x1 = heightReticles[0].px;
  int x2 = heightReticles[1].px;
  int x3 = heightReticles[2].px;
  int x4 = heightReticles[3].px;

  int y1 = heightReticles[0].py;
  int y2 = heightReticles[1].py;
  int y3 = heightReticles[2].py;
  int y4 = heightReticles[3].py;

  double z1 = convertHeightIdxToGlobalZ(0) + currentTableZ;
  double z2 = convertHeightIdxToGlobalZ(1) + currentTableZ;
  double z3 = convertHeightIdxToGlobalZ(2) + currentTableZ;
  double z4 = convertHeightIdxToGlobalZ(3) + currentTableZ;

  double reticlePixelX = 0.0;
  double reticlePixelY = 0.0;
  {
    //double d = d_x;
    double d = d_x/m_x;
    double c = ((z4*x4-z2*x2)*(x3-x1)-(z3*x3-z1*x1)*(x4-x2))/((z1-z3)*(x4-x2)-(z2-z4)*(x3-x1));

    double b42 = (z4*x4-z2*x2+(z2-z4)*c)/(x4-x2);
    double b31 = (z3*x3-z1*x1+(z1-z3)*c)/(x3-x1);

    double bDiff = b42-b31;
    //cout << "x1 x2 x3 x4: " << x1 << " " << x2 << " " << x3 << " " << x4 << endl;
    //cout << "y1 y2 y3 y4: " << y1 << " " << y2 << " " << y3 << " " << y4 << endl;
    //cout << "z1 z2 z3 z4: " << z1 << " " << z2 << " " << z3 << " " << z4 << endl;
    //cout << "bDiff = " << bDiff << ", c = " << c << " b42, b31: " << b42 << " " << b31 << " " << endl;
    double b = (b42+b31)/2.0;

    int x_thisZ = c + ( (x1-c)*(z1-b) )/(gZ-b);
    //int x_thisZ = c + ( m_x*(x1-c)*(z1-b) )/(gZ-b);
    //*pX = c + ( (gX-d)*(x1-c) )/(currentEEPose.px-d);
    //*pX = c + ( (gX-d)*(x_thisZ-c) )/(currentEEPose.px-d);
    //*pX = c + ( m_x*(gX-trueEEPose.position.x+d)*(x_thisZ-c) )/(d);
    *pX = c + ( (gX-trueEEPose.position.x+d)*(x_thisZ-c) )/(d);
    // need to set this again so things match up if gX is truEEpose
    //x_thisZ = c + ( m_x*(x1-c)*(z1-b) )/(gZ-b);
    x_thisZ = c + ( (d)*(x_thisZ-c) )/(d);
    reticlePixelX = x_thisZ;
  }
  {
    //double d = d_y;
    double d = d_y/m_y;
    double c = ((z4*y4-z2*y2)*(y3-y1)-(z3*y3-z1*y1)*(y4-y2))/((z1-z3)*(y4-y2)-(z2-z4)*(y3-y1));

    double b42 = (z4*y4-z2*y2+(z2-z4)*c)/(y4-y2);
    double b31 = (z3*y3-z1*y1+(z1-z3)*c)/(y3-y1);

    double bDiff = b42-b31;
    //cout << "x1 x2 x3 x4: " << x1 << " " << x2 << " " << x3 << " " << x4 << endl;
    //cout << "y1 y2 y3 y4: " << y1 << " " << y2 << " " << y3 << " " << y4 << endl;
    //cout << "z1 z2 z3 z4: " << z1 << " " << z2 << " " << z3 << " " << z4 << endl;
    //cout << "bDiff = " << bDiff << ", c = " << c << " b42, b31: " << b42 << " " << b31 << " " << endl;
    double b = (b42+b31)/2.0;

    int y_thisZ = c + ( (y1-c)*(z1-b) )/(gZ-b);
    //int y_thisZ = c + ( m_y*(y1-c)*(z1-b) )/(gZ-b);
    //*pY = c + ( (gY-d)*(y1-c) )/(currentEEPose.py-d);
    //*pY = c + ( (gY-d)*(y_thisZ-c) )/(currentEEPose.py-d);
    //*pY = c + ( m_y*(gY-trueEEPose.position.y+d)*(y_thisZ-c) )/(d);
    *pY = c + ( (gY-trueEEPose.position.y+d)*(y_thisZ-c) )/(d);
    // need to set this again so things match up if gX is truEEpose
    //y_thisZ = c + ( m_y*(y1-c)*(z1-b) )/(gZ-b);
    y_thisZ = c + ( (d)*(y_thisZ-c) )/(d);
    reticlePixelY = y_thisZ;
  }

  //cout << "reticlePixelX, reticlePixelY: " << reticlePixelX << " " << reticlePixelY << endl;

  // account for rotation of the end effector 
  Quaternionf eeqform(trueEEPose.orientation.w, trueEEPose.orientation.x, trueEEPose.orientation.y, trueEEPose.orientation.z);
  Quaternionf crane2Orient(0, 1, 0, 0);
  Quaternionf rel = eeqform * crane2Orient.inverse();
  Quaternionf ex(0,1,0,0);
  Quaternionf zee(0,0,0,1);
	
  Quaternionf result = rel * ex * rel.conjugate();
  Quaternionf thumb = rel * zee * rel.conjugate();
  double aY = result.y();
  double aX = result.x();

  // ATTN 22
  //double angle = atan2(aY, aX)*180.0/3.1415926;
  double angle = vectorArcTan(aY, aX)*180.0/3.1415926;
  angle = angle;
  double scale = 1.0;
  Point center = Point(reticlePixelX, reticlePixelY);

  Mat un_rot_mat = getRotationMatrix2D( center, angle, scale );

  Mat toUn(3,1,CV_64F);
  toUn.at<double>(0,0)=*pX;
  toUn.at<double>(1,0)=*pY;
  toUn.at<double>(2,0)=1.0;
  Mat didUn = un_rot_mat*toUn;
  *pX = didUn.at<double>(0,0);
  *pY = didUn.at<double>(1,0);

  double oldPx = *pX;
  double oldPy = *pY;
  //*pX = reticlePixelX + m_y*(oldPy - reticlePixelY) + offX;
  //*pY = reticlePixelY + m_x*(oldPx - reticlePixelX) + offY;
  *pX = reticlePixelX + (oldPy - reticlePixelY) + offX;
  *pY = reticlePixelY + (oldPx - reticlePixelX) + offY;
}

void paintEEPoseOnWrist(eePose toPaint, cv::Scalar theColor) {
  cv::Scalar THEcOLOR(255-theColor[0], 255-theColor[1], 255-theColor[2]);
  int lineLength = 5;
  int pX = 0, pY = 0;  
  double zToUse = trueEEPose.position.z+currentTableZ;

  globalToPixel(&pX, &pY, zToUse, toPaint.px, toPaint.py);
  pX = pX - lineLength;
  pY = pY - lineLength;
  //cout << "paintEEPoseOnWrist pX pY zToUse: " << pX << " " << pY << " " << zToUse << endl;
  if ( (pX > 0+lineLength) && (pX < wristViewImage.cols-lineLength) && (pY > 0+lineLength) && (pY < wristViewImage.rows-lineLength) ) {
    {
      Point pt1(pX+lineLength, pY);
      Point pt2(pX+lineLength, pY+lineLength*2);
      line(wristViewImage, pt1, pt2, theColor);
    }
    {
      Point pt1(pX, pY+lineLength);
      Point pt2(pX+lineLength*2, pY+lineLength);
      line(wristViewImage, pt1, pt2, theColor);
    }
  }

  // draw the test pattern for the inverse transformation 
  if (0) {
    double gX = 0, gY = 0;
    pixelToGlobal(pX, pY, zToUse, &gX, &gY);
    globalToPixel(&pX, &pY, zToUse, gX, gY);
    //cout << "PAINTeepOSEoNwRIST pX pY gX gY: " << pX << " " << pY << " " << gX << " " << gY << endl;
    if ( (pX > 0+lineLength) && (pX < wristViewImage.cols-lineLength) && (pY > 0+lineLength) && (pY < wristViewImage.rows-lineLength) ) {
      {
	Point pt1(pX+lineLength, pY);
	Point pt2(pX+lineLength, pY+lineLength*2);
	line(wristViewImage, pt1, pt2, THEcOLOR);
      }
      {
	Point pt1(pX, pY+lineLength);
	Point pt2(pX+lineLength*2, pY+lineLength);
	line(wristViewImage, pt1, pt2, THEcOLOR);
      }
    }
  }

  //guardedImshow(objectViewerName, objectViewerImage);
}

void ikLast(eePose targetPose, double *jointsOut) {


}

double vectorArcTan(double y, double x) {
  int maxVaSlot = 0;
  double maxVaDot = -INFINITY;
  for (int vaSlot = 0; vaSlot < vaNumAngles; vaSlot++) {
    double product = vaX[vaSlot]*x + vaY[vaSlot]*y;
    if (product > maxVaDot) {
      maxVaDot = product;
      maxVaSlot = vaSlot;
    }
  } 
  // return value in interval [-pi, pi]

  double angleZeroTwopi = (maxVaSlot * vaDelta);
  if (angleZeroTwopi <= 3.1415926) {
    return angleZeroTwopi;
  } else {
    return ( angleZeroTwopi - (2.0*3.1415926) );
  }
}

void initVectorArcTan() {
  for (int vaSlot = 0; vaSlot < vaNumAngles; vaSlot++) {
    vaX[vaSlot] = cos(vaSlot*vaDelta);
    vaY[vaSlot] = sin(vaSlot*vaDelta);
  }
  /* smoke test
  for (int vaSlot = 0; vaSlot < vaNumAngles; vaSlot++) {
    cout << "atan2 vectorArcTan: " << 
      vectorArcTan(vaY[vaSlot], vaX[vaSlot]) << " " << 
      atan2(vaY[vaSlot], vaX[vaSlot]) << endl; 
  }
  */
}

void mapBlueBox(cv::Point tbTop, cv::Point tbBot, int detectedClass, ros::Time timeToMark) {
  for (double px = tbTop.x-mapBlueBoxPixelSkirt; px <= tbBot.x+mapBlueBoxPixelSkirt; px++) {
    for (double py = tbTop.y-mapBlueBoxPixelSkirt; py <= tbBot.y+mapBlueBoxPixelSkirt; py++) {
      double x, y;
      double z = trueEEPose.position.z + currentTableZ;

      pixelToGlobal(px, py, z, &x, &y);
      int i, j;
      mapxyToij(x, y, &i, &j);

      if (i >= 0 && i < mapWidth && j >= 0 && j < mapHeight) {
	objectMap[i + mapWidth * j].lastMappedTime = timeToMark;
	objectMap[i + mapWidth * j].detectedClass = detectedClass;

  //      if (timeToMark - objectMap[i + mapWidth * j].lastMappedTime > mapMemoryTimeout) {
  //        objectMap[i + mapWidth * j].b = 0;
  //        objectMap[i + mapWidth * j].g = 0;
  //        objectMap[i + mapWidth * j].r = 0;
  //        objectMap[i + mapWidth * j].pixelCount = 0;
  //      }

	double blueBoxWeight = 0.1;
	if (cam_img.rows != 0 && cam_img.cols != 0) {
	  objectMap[i + mapWidth * j].b = (cam_img.at<cv::Vec3b>(py, px)[0] * blueBoxWeight);
	  objectMap[i + mapWidth * j].g = (cam_img.at<cv::Vec3b>(py, px)[1] * blueBoxWeight);
	  objectMap[i + mapWidth * j].r = (cam_img.at<cv::Vec3b>(py, px)[2] * blueBoxWeight);
	  objectMap[i + mapWidth * j].pixelCount = blueBoxWeight;
	}
      }
    }
  }
}

void mapBox(BoxMemory boxMemory) {
 mapBlueBox(boxMemory.bTop, boxMemory.bBot, boxMemory.labeledClassIndex, ros::Time::now());
}


////////////////////////////////////////////////
// end pilot definitions 
//
// start node definitions 
////////////////////////////////////////////////

int doubleToByte(double in) {
  return min(max(round(in),0.0),255.0);
}

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

  int totalDescriptors = 0;
  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  char buf[1024];
  sprintf(buf, "%s%s/rgb", classDir.c_str(), className);
  dpdf = opendir(buf);
  if (dpdf != NULL){
    while (epdf = readdir(dpdf)){
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

        vector<KeyPoint> keypoints1;
        vector<KeyPoint> keypoints2;
        Mat descriptors;

        char filename[1024];
        sprintf(filename, "%s%s/rgb/%s", classDir.c_str(), className, epdf->d_name);
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

	  totalDescriptors += int(descriptors.rows);
	  cout << className << ":  "  << epdf->d_name << "  " << descriptors.size() << " total descriptors: " << totalDescriptors << endl;

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
  sprintf(buf, "%s%s/rgb", classDir.c_str(), className);
  dpdf = opendir(buf);
  if (dpdf != NULL){
    while (epdf = readdir(dpdf)){
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

        vector<KeyPoint> keypoints;
        Mat descriptors;
        Mat descriptors2;

        char filename[1024];
        sprintf(filename, "%s%s/rgb/%s", classDir.c_str(), className, epdf->d_name);
        Mat image;
        image = imread(filename);
	Size sz = image.size();
	int cropW = sz.width;
	int cropH = sz.height;
	cv::Point bot(cropW, cropH);

        Mat gray_image;
        Mat yCrCb_image;

	//if ((chosen_feature == SIFTBOW_GLOBALCOLOR_HIST) || (chosen_feature == OPPONENTSIFTBOW_GLOBALCOLOR_HIST))
	if (chosen_feature == SIFTBOW_GLOBALCOLOR_HIST) 
	{
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
	} else if (chosen_feature == OPPONENTSIFTBOW_GLOBALCOLOR_HIST) {
	  processImage(image, gray_image, yCrCb_image, sigma);
	  for (int i = 0; i < kNNOverSampleFactor; i++) {
	    //detector->detect(gray_image, keypoints);
	    gridKeypoints(0, 0, cv::Point(0,0), bot, gBoxStrideX, gBoxStrideY, keypoints, keypointPeriod);
	    //bowExtractor->compute(gray_image, keypoints, descriptors);

	    Mat tmpC;
	    image.convertTo(tmpC, CV_32FC3);
	    bowExtractor->compute(tmpC, keypoints, descriptors);

	    cout << className << ":  "  << epdf->d_name << "  " << descriptors.size() << " type: " << descriptors.type() << " tot: " << kNNfeatures.size() << endl;

	    if (!descriptors.empty() && !keypoints.empty()) {
	      appendColorHist(yCrCb_image, keypoints, descriptors, descriptors2);

	      kNNfeatures.push_back(descriptors2);
	      kNNlabels.push_back(label);
	    }
	  }
	} else if (chosen_feature == GRADIENT) {
	  processImage(image, gray_image, yCrCb_image, sobel_sigma);

	  Mat totalGraySobel;
	  {
	    Mat grad_x, grad_y;
	    int sobelScale = 1;
	    int sobelDelta = 0;
	    int sobelDepth = CV_32F;
	    /// Gradient X
	    Sobel(gray_image, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
	    /// Gradient Y
	    Sobel(gray_image, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

	    grad_x = grad_x.mul(grad_x);
	    grad_y = grad_y.mul(grad_y);
	    totalGraySobel = grad_x + grad_y;
	    // now totalGraySobel is gradient magnitude squared
	  }

	  // grow to the max dimension to avoid distortion
	  // find the dimensions that pad the sobel image up to a square
	  // raster scan a 'virtual image' to generate the 1D vector, adding 0's when on the pad

	  int crows = totalGraySobel.rows;
	  int ccols = totalGraySobel.cols;
	  int maxDim = max(crows, ccols);
	  int tRy = (maxDim-crows)/2;
	  int tRx = (maxDim-ccols)/2;
	  Mat gCrop(maxDim, maxDim, totalGraySobel.type());

	  float totalMass = 0.0;

	  for (int x = 0; x < maxDim; x++) {
	    for (int y = 0; y < maxDim; y++) {
	      int tx = x - tRx;
	      int ty = y - tRy;
	      if (tx >= 0 && ty >= 0 && ty < crows && tx < ccols) {
		gCrop.at<float>(y, x) = totalGraySobel.at<float>(ty, tx);
		//totalMass += gCrop.at<float>(y, x);
		totalMass += gCrop.at<float>(y, x) * gCrop.at<float>(y, x);
	      } else {
		gCrop.at<float>(y, x) = 0.0;
	      }
	    }
	  }
	  totalMass = sqrt(totalMass);
	  Mat descriptorsG = Mat(1, gradientFeatureWidth*gradientFeatureWidth, CV_32F);
	  for (int y = 0; y < gradientFeatureWidth; y++) {
	    for (int x = 0; x < gradientFeatureWidth; x++) {
	      int tranX = floor(float(x)*float(maxDim)/float(gradientFeatureWidth));
	      int tranY = floor(float(y)*float(maxDim)/float(gradientFeatureWidth));
	      //descriptorsG.at<float>(x + y*gradientFeatureWidth) = gCrop.at<float>(y,x);
	      descriptorsG.at<float>(x + y*gradientFeatureWidth) = gCrop.at<float>(y,x)/totalMass;
	    }
	  }
	  kNNfeatures.push_back(descriptorsG);
	  kNNlabels.push_back(label);
	} else if (chosen_feature == OPPONENT_COLOR_GRADIENT) {
	  processImage(image, gray_image, yCrCb_image, sobel_sigma);

	  Mat totalGraySobel;
	  {
	    Mat grad_x, grad_y;
	    int sobelScale = 1;
	    int sobelDelta = 0;
	    int sobelDepth = CV_32F;
	    /// Gradient X
	    Sobel(gray_image, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
	    /// Gradient Y
	    Sobel(gray_image, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

	    grad_x = grad_x.mul(grad_x);
	    grad_y = grad_y.mul(grad_y);
	    totalGraySobel = grad_x + grad_y;
	    // now totalGraySobel is gradient magnitude squared
	  }
	  Mat totalCrSobel = totalGraySobel.clone();
	  {
	    for (int y = 0; y < image.rows; y++) {
	      for (int x = 0; x < image.cols; x++) {
		cv::Vec3b thisColor = yCrCb_image.at<cv::Vec3b>(y,x);
		totalCrSobel.at<float>(y,x) = thisColor[1];
	      }
	    }
	    Mat grad_x, grad_y;
	    int sobelScale = 1;
	    int sobelDelta = 0;
	    int sobelDepth = CV_32F;
	    /// Gradient X
	    Sobel(totalCrSobel, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
	    /// Gradient Y
	    Sobel(totalCrSobel, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

	    //grad_x = grad_x.mul(grad_x);
	    //grad_y = grad_y.mul(grad_y);
	    totalCrSobel = grad_x + grad_y;
	  }
	  Mat totalCbSobel = totalGraySobel.clone();
	  {
	    for (int y = 0; y < image.rows; y++) {
	      for (int x = 0; x < image.cols; x++) {
		cv::Vec3b thisColor = yCrCb_image.at<cv::Vec3b>(y,x);
		totalCbSobel.at<float>(y,x) = thisColor[2];
	      }
	    }
	    Mat grad_x, grad_y;
	    int sobelScale = 1;
	    int sobelDelta = 0;
	    int sobelDepth = CV_32F;
	    /// Gradient X
	    Sobel(totalCbSobel, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
	    /// Gradient Y
	    Sobel(totalCbSobel, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

	    //grad_x = grad_x.mul(grad_x);
	    //grad_y = grad_y.mul(grad_y);
	    totalCbSobel = grad_x + grad_y;
	  }

	  // grow to the max dimension to avoid distortion
	  // find the dimensions that pad the sobel image up to a square
	  // raster scan a 'virtual image' to generate the 1D vector, adding 0's when on the pad

	  int crows = totalGraySobel.rows;
	  int ccols = totalGraySobel.cols;
	  int maxDim = max(crows, ccols);
	  int tRy = (maxDim-crows)/2;
	  int tRx = (maxDim-ccols)/2;
	  Mat gCrop(maxDim, maxDim, totalGraySobel.type());
	  Mat crCrop(maxDim, maxDim, totalGraySobel.type());
	  Mat cbCrop(maxDim, maxDim, totalGraySobel.type());

	  float totalGMass = 0.0;
	  float totalCrMass = 0.0;
	  float totalCbMass = 0.0;

	  for (int x = 0; x < maxDim; x++) {
	    for (int y = 0; y < maxDim; y++) {
	      int tx = x - tRx;
	      int ty = y - tRy;
	      if (tx >= 0 && ty >= 0 && ty < crows && tx < ccols) {
		gCrop.at<float>(y, x) = totalGraySobel.at<float>(ty, tx);
		crCrop.at<float>(y, x) = totalCrSobel.at<float>(ty, tx);
		cbCrop.at<float>(y, x) = totalCbSobel.at<float>(ty, tx);
		//totalGMass += gCrop.at<float>(y, x);
		totalGMass += gCrop.at<float>(y, x) * gCrop.at<float>(y, x);
		totalCrMass += gCrop.at<float>(y, x) * gCrop.at<float>(y, x);
		totalCbMass += gCrop.at<float>(y, x) * gCrop.at<float>(y, x);
	      } else {
		gCrop.at<float>(y, x) = 0.0;
	      }
	    }
	  }
	  totalGMass = sqrt(totalGMass);
	  totalCrMass = sqrt(totalCrMass);
	  totalCbMass = sqrt(totalCbMass);
	  //Mat descriptorsG = Mat(1, gradientFeatureWidth*gradientFeatureWidth, CV_32F);
	  Mat descriptorsCbCr = Mat(1, 2*gradientFeatureWidth*gradientFeatureWidth, CV_32F);
	  for (int y = 0; y < gradientFeatureWidth; y++) {
	    for (int x = 0; x < gradientFeatureWidth; x++) {
	      int tranX = floor(float(x)*float(maxDim)/float(gradientFeatureWidth));
	      int tranY = floor(float(y)*float(maxDim)/float(gradientFeatureWidth));
	      //descriptorsG.at<float>(x + y*gradientFeatureWidth) = gCrop.at<float>(y,x);
	      descriptorsCbCr.at<float>(x + y*gradientFeatureWidth) = crCrop.at<float>(y,x)/totalCrMass;
	      descriptorsCbCr.at<float>(x + y*gradientFeatureWidth + gradientFeatureWidth*gradientFeatureWidth) = cbCrop.at<float>(y,x)/totalCbMass;
	    }
	  }

	  kNNfeatures.push_back(descriptorsCbCr);
	  kNNlabels.push_back(label);
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
  sprintf(buf, "%s%s/rgbPose", classDir.c_str(), className);
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
        sprintf(filename, "%s%s/rgbPose/%s", classDir.c_str(), className, epdf->d_name);
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

  guardedImshow("Depth Viewer", depth_img*20);
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
  //ROS_INFO_STREAM("quaternion: " << objectQuaternion.x());
  //ROS_INFO_STREAM("roa: " << roa_to_send.objects[aI]);

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
  // Actually some might say its better to make them mean 0 and L2 unit norm.  But L1 works
  // for now.
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

  if (!shouldIMiscCallback) {
    return;
  }

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

void goCalculateObjectness() {
  Size sz = objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;

  objectnessViewerImage = cv_ptr->image.clone();

  int boxesPerSize = 800;
  ValStructVec<float, Vec4i> boxes;
  glObjectness->getObjBndBoxes(objectViewerImage, boxes, boxesPerSize);

  int numBoxes = boxes.size();

  #ifdef DEBUG
  cout << "numBoxes: " << numBoxes << "  fc: " << fc <<  endl;
  #endif

  nTop.resize(numBoxes);
  nBot.resize(numBoxes);

  int boxesToConsider= 50000;

  if (integralObjDensity == NULL)
    integralObjDensity = new double[imW*imH];
  if (objDensity == NULL)
    objDensity = new double[imW*imH];
  if (preObjDensity == NULL)
    preObjDensity = new double[imW*imH];
  if (differentialObjDensity == NULL)
    differentialObjDensity = new double[imW*imH];
  if (temporalObjDensity == NULL) {
    temporalObjDensity = new double[imW*imH];
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	temporalObjDensity[y*imW + x] = 0;
      }
    }
  }

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      objDensity[y*imW + x] = 0;
      differentialObjDensity[y*imW + x] = 0;
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
	  rectangle(objectViewerImage, outTop, outBot, cv::Scalar(188,40,140));
	  rectangle(objectViewerImage, inTop, inBot, cv::Scalar(94,20,70));
	}
      }
      
      double toAdd = 1.0 / area;
      //double toAdd = 1.0;
      int x = nTop[i].x;
      int y = nTop[i].y;
      differentialObjDensity[y*imW + x] += toAdd;
      x = nBot[i].x;
      y = nBot[i].y;
      differentialObjDensity[y*imW + x] += toAdd;
      x = nTop[i].x;
      y = nBot[i].y;
      differentialObjDensity[y*imW + x] -= toAdd;
      x = nBot[i].x;
      y = nTop[i].y;
      differentialObjDensity[y*imW + x] -= toAdd;
    }
  }

  // integrate the differential objDensity into the objDensity
  objDensity[0] = differentialObjDensity[0];
  for (int x = 1; x < imW; x++) {
    int y = 0;
    objDensity[y*imW+x] = objDensity[y*imW+(x-1)] + differentialObjDensity[y*imW + x];
  }
  for (int y = 1; y < imH; y++) {
    int x = 0;
    objDensity[y*imW+x] = objDensity[(y-1)*imW+x] + differentialObjDensity[y*imW + x];
  }
  for (int x = 1; x < imW; x++) {
    for (int y = 1; y < imH; y++) {
      objDensity[y*imW+x] = 
	objDensity[(y-1)*imW+x]+objDensity[y*imW+(x-1)]-objDensity[(y-1)*imW+(x-1)]+differentialObjDensity[y*imW + x];
    }
  }

  // truncate the objDensity outside the gray box
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < grayTop.y; y++) {
      objDensity[y*imW+x] = 0;
    }
  }

  for (int x = 0; x < grayTop.x; x++) {
    for (int y = grayTop.y; y < grayBot.y; y++) {
      objDensity[y*imW+x] = 0;
    }
  }

  for (int x = grayBot.x; x < imW; x++) {
    for (int y = grayTop.y; y < grayBot.y; y++) {
      objDensity[y*imW+x] = 0;
    }
  }

  for (int x = 0; x < imW; x++) {
    for (int y = grayBot.y; y < imH; y++) {
      objDensity[y*imW+x] = 0;
    }
  }

  if (mask_gripper) {
    int xs = g1xs;
    int xe = g1xe;
    int ys = g1ys;
    int ye = g1ye;
    for (int x = xs; x < xe; x++) {
      for (int y = ys; y < ye; y++) {
	objDensity[y*imW+x] = 0;
      }
    }

    xs = g2xs;
    xe = g2xe;
    ys = g2ys;
    ye = g2ye;
    for (int x = xs; x < xe; x++) {
      for (int y = ys; y < ye; y++) {
	objDensity[y*imW+x] = 0;
      }
    }
  }

  double maxObjDensity = -INFINITY;
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      maxObjDensity = max(maxObjDensity, objDensity[y*imW+x]);
    }
  }
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      if (objDensity[y*imW+x] < maxObjDensity* objectnessThreshFraction)
	objDensity[y*imW+x] = 0;
    }
  }

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      temporalObjDensity[y*imW+x] = objDensityDecay*temporalObjDensity[y*imW+x] + (1.0-objDensityDecay)*objDensity[y*imW+x];
    }
  }

  // we don't use the integral objDensity for objectness at the moment.
  int calculateObjectnessIntegralObjDensity = 0;
  if (calculateObjectnessIntegralObjDensity) {
    // integrate objDensity into the integral objDensity
    integralObjDensity[0] = objDensity[0];
    for (int x = 1; x < imW; x++) {
      int y = 0;
      integralObjDensity[y*imW+x] = integralObjDensity[y*imW+(x-1)] + objDensity[y*imW + x];
    }
    for (int y = 1; y < imH; y++) {
      int x = 0;
      integralObjDensity[y*imW+x] = integralObjDensity[(y-1)*imW+x] + objDensity[y*imW + x];
    }
    for (int x = 1; x < imW; x++) {
      for (int y = 1; y < imH; y++) {
	integralObjDensity[y*imW+x] = 
	  integralObjDensity[(y-1)*imW+x]+integralObjDensity[y*imW+(x-1)]-integralObjDensity[(y-1)*imW+(x-1)]+objDensity[y*imW + x];
      }
    }
  }

  double *objMapToUse = NULL;
  objMapToUse = temporalObjDensity;

  double minObjDen = INFINITY;
  double maxObjDen = -INFINITY;
  for (int y = 0; y < imH; y++) {
    for (int x = 0; x < imW; x++) {
      minObjDen = min(minObjDen, objMapToUse[y*imW+x]);
      maxObjDen = max(maxObjDen, objMapToUse[y*imW+x]);
    }
  }
  double objDenRange = maxObjDen - minObjDen;

  // copy the objMapToUse map to the rendered image
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      uchar val = uchar(min( 1*255.0 *  (objMapToUse[y*imW+x] - minObjDen) / objDenRange, 255.0));
      objectnessViewerImage.at<cv::Vec3b>(y,x) = cv::Vec<uchar, 3>(0,val,0);
    }
  }
}

void goAccumulateForAerial() {

}

void goAccumulateDensityFromAccumulated() {
  Size sz = objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;

  if (cv_ptr == NULL) {
    ROS_ERROR("Not receiving camera data, clearing call stack.");
    clearStack();
    return;
  }

  Mat tmpImage = cv_ptr->image.clone();

  if (add_blinders) {
    goAddBlinders();
  }

  // determine table edges, i.e. the gray boxes
  lGO = gBoxW*(lGO/gBoxW);
  rGO = gBoxW*(rGO/gBoxW);
  tGO = gBoxH*(tGO/gBoxH);
  bGO = gBoxH*(bGO/gBoxH);
  grayTop = cv::Point(lGO, tGO);
  grayBot = cv::Point(imW-rGO-1, imH-bGO-1);

  if (all_range_mode) {
    grayTop = armTop;
    grayBot = armBot;
  }

  // Sobel business
  Mat sobelGrayBlur;
  Mat sobelYCrCbBlur;

  sobelGrayBlur = aerialGradientSobelGray;
  sobelYCrCbBlur = aerialGradientSobelCbCr;
  
  Mat totalGraySobel;
//  {
//    Mat grad_x, grad_y;
//    int sobelScale = 1;
//    int sobelDelta = 0;
//    int sobelDepth = CV_64F;
//    /// Gradient X
//    Sobel(sobelGrayBlur, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
//    /// Gradient Y
//    Sobel(sobelGrayBlur, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
//
//    grad_x = grad_x.mul(grad_x);
//    grad_y = grad_y.mul(grad_y);
//    totalGraySobel = grad_x + grad_y;
//    // now totalGraySobel is gradient magnitude squared
//  }

  Mat totalCrSobel(imH, imW, CV_64F);
  Mat totalCrSobelMag;
  {
    for (int y = 0; y < imH; y++) {
      for (int x = 0; x < imW; x++) {
	cv::Vec3b thisColor = sobelYCrCbBlur.at<cv::Vec3b>(y,x);
	totalCrSobel.at<double>(y,x) = thisColor[1];
      }
    }
    Mat grad_x, grad_y;
    int sobelScale = 1;
    int sobelDelta = 0;
    int sobelDepth = CV_64F;
    /// Gradient X
    Sobel(totalCrSobel, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
    /// Gradient Y
    Sobel(totalCrSobel, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

    totalCrSobel = grad_x + grad_y;
    grad_x = grad_x.mul(grad_x);
    grad_y = grad_y.mul(grad_y);
    totalCrSobelMag = grad_x + grad_y;
  }

  Mat totalCbSobel(imH, imW, CV_64F);
  Mat totalCbSobelMag;
  {
    for (int y = 0; y < imH; y++) {
      for (int x = 0; x < imW; x++) {
	cv::Vec3b thisColor = sobelYCrCbBlur.at<cv::Vec3b>(y,x);
	totalCbSobel.at<double>(y,x) = thisColor[2];
      }
    }
    Mat grad_x, grad_y;
    int sobelScale = 1;
    int sobelDelta = 0;
    int sobelDepth = CV_64F;
    /// Gradient X
    Sobel(totalCbSobel, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
    /// Gradient Y
    Sobel(totalCbSobel, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

    totalCbSobel = grad_x + grad_y;
    grad_x = grad_x.mul(grad_x);
    grad_y = grad_y.mul(grad_y);
    totalCbSobelMag = grad_x + grad_y;
  }

  Mat totalYSobel(imH, imW, CV_64F);
  {
    for (int y = 0; y < imH; y++) {
      for (int x = 0; x < imW; x++) {
	cv::Vec3b thisColor = sobelYCrCbBlur.at<cv::Vec3b>(y,x);
	totalYSobel.at<double>(y,x) = thisColor[0];
      }
    }
    Mat grad_x, grad_y;
    int sobelScale = 1;
    int sobelDelta = 0;
    int sobelDepth = CV_64F;
    /// Gradient X
    Sobel(totalYSobel, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
    /// Gradient Y
    Sobel(totalYSobel, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

    grad_x = grad_x.mul(grad_x);
    grad_y = grad_y.mul(grad_y);
    totalYSobel = grad_x + grad_y;
  }

  // total becomes sum of Cr and Cb
  int totalBecomes = 1;
  if (totalBecomes) {
    totalGraySobel = totalCrSobelMag + totalCbSobelMag;
  }

  // truncate the Sobel image outside the gray box
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < grayTop.y; y++) {
      totalGraySobel.at<double>(y,x) = 0;
      totalYSobel.at<double>(y,x) = 0;
    }
  }

  for (int x = 0; x < grayTop.x; x++) {
    for (int y = grayTop.y; y < grayBot.y; y++) {
      totalGraySobel.at<double>(y,x) = 0;
      totalYSobel.at<double>(y,x) = 0;
    }
  }

  for (int x = grayBot.x; x < imW; x++) {
    for (int y = grayTop.y; y < grayBot.y; y++) {
      totalGraySobel.at<double>(y,x) = 0;
      totalYSobel.at<double>(y,x) = 0;
    }
  }

  for (int x = 0; x < imW; x++) {
    for (int y = grayBot.y; y < imH; y++) {
      totalGraySobel.at<double>(y,x) = 0;
      totalYSobel.at<double>(y,x) = 0;
    }
  }

  if (integralDensity == NULL)
    integralDensity = new double[imW*imH];
  if (density == NULL)
    density = new double[imW*imH];
  if (preDensity == NULL)
    preDensity = new double[imW*imH];
  if (temporalDensity == NULL) {
    temporalDensity = new double[imW*imH];
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	temporalDensity[y*imW + x] = 0;
      }
    }
  }

  double maxGsob = -INFINITY;
  double maxYsob = -INFINITY;
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      maxGsob = max(maxGsob, totalGraySobel.at<double>(y,x));
      maxYsob = max(maxYsob, totalYSobel.at<double>(y,x));
    }
  }
  
  // ATTN 11
  // experimental
  int combineYandGray = 1;
  double yWeight = 1.0;
  if (combineYandGray) {
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	double thisY2G = min(maxYsob, yWeight * totalYSobel.at<double>(y,x));
	totalGraySobel.at<double>(y,x) += maxGsob * thisY2G * thisY2G / (maxYsob * maxYsob);
      }
    }
  }

  if (mask_gripper) {
    int xs = g1xs;
    int xe = g1xe;
    int ys = g1ys;
    int ye = g1ye;
    for (int x = xs; x < xe; x++) {
      for (int y = ys; y < ye; y++) {
	totalGraySobel.at<double>(y,x) = 0;
      }
    }
    xs = g2xs;
    xe = g2xe;
    ys = g2ys;
    ye = g2ye;
    for (int x = xs; x < xe; x++) {
      for (int y = ys; y < ye; y++) {
	totalGraySobel.at<double>(y,x) = 0;
      }
    }
  }

  { // temporal averaging of aerial gradient
    if ( (aerialGradientTemporalFrameAverage.rows < aerialGradientReticleWidth) ||
	 (aerialGradientTemporalFrameAverage.cols < aerialGradientReticleWidth) ) {
      aerialGradientTemporalFrameAverage = Mat(imH,imW,totalGraySobel.type()); 
    }

    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	aerialGradientTemporalFrameAverage.at<double>(y, x) = 
	  aerialGradientDecay*aerialGradientTemporalFrameAverage.at<double>(y, x) + 
	  (1.0 - aerialGradientDecay)*totalGraySobel.at<double>(y, x);
      }
    }
  }
}

void resetAccumulatedImageAndMass() {
  Size sz = accumulatedImageMass.size();
  int imW = sz.width;
  int imH = sz.height;

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      accumulatedImageMass.at<double>(y,x) = 0;
      accumulatedImage.at<Vec3d>(y,x)[0] = 0;
      accumulatedImage.at<Vec3d>(y,x)[1] = 0;
      accumulatedImage.at<Vec3d>(y,x)[2] = 0;
    }
  }
}

void renderAccumulatedImageAndDensity() {
/*
  // copy the density map to the rendered image
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      //uchar val = uchar(min( 1*255.0 *  (totalGraySobel.at<double>(y,x) - minGraySob) / sobGrayRange, 255.0));
      uchar val = uchar(min( 1*255.0 *  (frameGraySobel.at<double>(y,x) - minAerTemp) / aerTempRange, 255.0));
      gradientViewerImage.at<cv::Vec3b>(y,x) = cv::Vec<uchar, 3>(0,val,0);

      gradientViewerImage.at<cv::Vec3b>(y+imH,x) = convertedYCbCrGradientImage.at<cv::Vec3b>(y,x);
    }
  }
*/
  Size sz = objectViewerYCbCrBlur.size();
  int imW = sz.width;
  int imH = sz.height;

  int YConstant = 128;
  Mat oviToConstantize = objectViewerYCbCrBlur.clone();

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      //oviToConstantize.at<cv::Vec3b>(y,x)[0] = YConstant;
    }
  }

  Mat oviInBGR;
  cvtColor(oviToConstantize, oviInBGR, CV_YCrCb2BGR);

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      gradientViewerImage.at<cv::Vec3b>(y+imH,x) = oviInBGR.at<cv::Vec3b>(y,x);
      //gradientViewerImage.at<cv::Vec3b>(y+imH,x) = oviToConstantize.at<cv::Vec3b>(y,x);

      //gradientViewerImage.at<cv::Vec3b>(y+imH,x) = objectViewerYCbCrBlur.at<cv::Vec3b>(y,x);

      //gradientViewerImage.at<cv::Vec3b>(y+imH,x)[0] = objectViewerGrayBlur.at<uchar>(y,x);
      //gradientViewerImage.at<cv::Vec3b>(y+imH,x)[1] = objectViewerGrayBlur.at<uchar>(y,x);
      //gradientViewerImage.at<cv::Vec3b>(y+imH,x)[2] = objectViewerGrayBlur.at<uchar>(y,x);
    }
  }

  if (shouldIRender) {
    guardedImshow(gradientViewerName, gradientViewerImage);
  }

}

void substituteAccumulatedImageQuantities() {
  aerialGradientDecay = aerialGradientDecayImageAverage;
  sobel_sigma = sobel_sigma_substitute_accumulated;
  Size sz = accumulatedImage.size();
  int imW = sz.width;
  int imH = sz.height;

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      double denom = accumulatedImageMass.at<double>(y,x);
      if (denom <= 1.0) {
	denom = 1.0;
      }
      objectViewerImage.at<Vec3b>(y,x)[0] = doubleToByte(accumulatedImage.at<Vec3d>(y,x)[0] / denom);
      objectViewerImage.at<Vec3b>(y,x)[1] = doubleToByte(accumulatedImage.at<Vec3d>(y,x)[1] / denom);
      objectViewerImage.at<Vec3b>(y,x)[2] = doubleToByte(accumulatedImage.at<Vec3d>(y,x)[2] / denom);
    }
  }
}

void substituteLatestImageQuantities() {
  aerialGradientDecay = aerialGradientDecayIteratedDensity;
  sobel_sigma = sobel_sigma_substitute_latest;
  if (cv_ptr == NULL) {
    ROS_ERROR("Not receiving camera data, clearing call stack.");
    clearStack();
    return;
  }

  objectViewerImage = cv_ptr->image.clone();
}

void goCalculateDensity() {
  Size sz = objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;

  // XXX TODO might be able to pick up some time here if their allocation is slow
  // by making these global
  densityViewerImage = objectViewerImage.clone();
  Mat tmpImage = objectViewerImage.clone();

  Mat yCbCrGradientImage = objectViewerImage.clone();

  if (add_blinders) {
    goAddBlinders();
  }

  // determine table edges, i.e. the gray boxes
  lGO = gBoxW*(lGO/gBoxW);
  rGO = gBoxW*(rGO/gBoxW);
  tGO = gBoxH*(tGO/gBoxH);
  bGO = gBoxH*(bGO/gBoxH);
  grayTop = cv::Point(lGO, tGO);
  grayBot = cv::Point(imW-rGO-1, imH-bGO-1);

  if (all_range_mode) {
    grayTop = armTop;
    grayBot = armBot;
  }

  // Sobel business
  Mat sobelGrayBlur;
  Mat sobelYCrCbBlur;
  processImage(tmpImage, sobelGrayBlur, sobelYCrCbBlur, sobel_sigma);
  objectViewerYCbCrBlur = sobelYCrCbBlur;
  objectViewerGrayBlur = sobelYCrCbBlur;
  
  Mat totalGraySobel;
  {
    Mat grad_x, grad_y;
    int sobelScale = 1;
    int sobelDelta = 0;
    int sobelDepth = CV_64F;
    /// Gradient X
    Sobel(sobelGrayBlur, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
    /// Gradient Y
    Sobel(sobelGrayBlur, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

    grad_x = grad_x.mul(grad_x);
    grad_y = grad_y.mul(grad_y);
    totalGraySobel = grad_x + grad_y;
    // now totalGraySobel is gradient magnitude squared
  }

  Mat totalCrSobel = totalGraySobel.clone();
  Mat totalCrSobelMag;
  {
    for (int y = 0; y < imH; y++) {
      for (int x = 0; x < imW; x++) {
	cv::Vec3b thisColor = sobelYCrCbBlur.at<cv::Vec3b>(y,x);
	totalCrSobel.at<double>(y,x) = thisColor[1];
      }
    }
    Mat grad_x, grad_y;
    int sobelScale = 1;
    int sobelDelta = 0;
    int sobelDepth = CV_64F;
    /// Gradient X
    Sobel(totalCrSobel, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
    /// Gradient Y
    Sobel(totalCrSobel, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

    totalCrSobel = grad_x + grad_y;
    grad_x = grad_x.mul(grad_x);
    grad_y = grad_y.mul(grad_y);
    totalCrSobelMag = grad_x + grad_y;
  }

  Mat totalCbSobel = totalGraySobel.clone();
  Mat totalCbSobelMag;
  {
    for (int y = 0; y < imH; y++) {
      for (int x = 0; x < imW; x++) {
	cv::Vec3b thisColor = sobelYCrCbBlur.at<cv::Vec3b>(y,x);
	totalCbSobel.at<double>(y,x) = thisColor[2];
      }
    }
    Mat grad_x, grad_y;
    int sobelScale = 1;
    int sobelDelta = 0;
    int sobelDepth = CV_64F;
    /// Gradient X
    Sobel(totalCbSobel, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
    /// Gradient Y
    Sobel(totalCbSobel, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

    totalCbSobel = grad_x + grad_y;
    grad_x = grad_x.mul(grad_x);
    grad_y = grad_y.mul(grad_y);
    totalCbSobelMag = grad_x + grad_y;
  }

  Mat totalYSobel = totalGraySobel.clone();
  {
    for (int y = 0; y < imH; y++) {
      for (int x = 0; x < imW; x++) {
	cv::Vec3b thisColor = sobelYCrCbBlur.at<cv::Vec3b>(y,x);
	totalYSobel.at<double>(y,x) = thisColor[0];
      }
    }
    Mat grad_x, grad_y;
    int sobelScale = 1;
    int sobelDelta = 0;
    int sobelDepth = CV_64F;
    /// Gradient X
    Sobel(totalYSobel, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
    /// Gradient Y
    Sobel(totalYSobel, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

    grad_x = grad_x.mul(grad_x);
    grad_y = grad_y.mul(grad_y);
    totalYSobel = grad_x + grad_y;
  }

  // total becomes sum of Cr and Cb
  int totalBecomes = 1;
  if (totalBecomes) {
    totalGraySobel = totalCrSobelMag + totalCbSobelMag;
  }

  // truncate the Sobel image outside the gray box
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < grayTop.y; y++) {
      totalGraySobel.at<double>(y,x) = 0;
      totalCrSobel.at<double>(y,x) = 0;
      totalCbSobel.at<double>(y,x) = 0;
      totalYSobel.at<double>(y,x) = 0;
    }
  }

  for (int x = 0; x < grayTop.x; x++) {
    for (int y = grayTop.y; y < grayBot.y; y++) {
      totalGraySobel.at<double>(y,x) = 0;
      totalCrSobel.at<double>(y,x) = 0;
      totalCbSobel.at<double>(y,x) = 0;
      totalYSobel.at<double>(y,x) = 0;
    }
  }

  for (int x = grayBot.x; x < imW; x++) {
    for (int y = grayTop.y; y < grayBot.y; y++) {
      totalGraySobel.at<double>(y,x) = 0;
      totalCrSobel.at<double>(y,x) = 0;
      totalCbSobel.at<double>(y,x) = 0;
      totalYSobel.at<double>(y,x) = 0;
    }
  }

  for (int x = 0; x < imW; x++) {
    for (int y = grayBot.y; y < imH; y++) {
      totalGraySobel.at<double>(y,x) = 0;
      totalCrSobel.at<double>(y,x) = 0;
      totalCbSobel.at<double>(y,x) = 0;
      totalYSobel.at<double>(y,x) = 0;
    }
  }

  // input image is noisy so blurring is a good idea
  //GaussianBlur(densityViewerImage, densityViewerImage, cv::Size(0,0), 1.0);

  if (integralDensity == NULL)
    integralDensity = new double[imW*imH];
  if (density == NULL)
    density = new double[imW*imH];
  if (preDensity == NULL)
    preDensity = new double[imW*imH];
  if (temporalDensity == NULL) {
    temporalDensity = new double[imW*imH];
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	temporalDensity[y*imW + x] = 0;
      }
    }
  }

  int replaceDensityWithGrad = 1;
  if (replaceDensityWithGrad) {
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	density[y*imW+x] = totalGraySobel.at<double>(y,x);
      }
    }
  }

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      preDensity[y*imW+x] = totalGraySobel.at<double>(y,x);
    }
  }

  // now update the exponential average of the density
  // and set the density to be a thresholded version of this
  maxDensity = 0;
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      maxDensity = max(maxDensity, density[y*imW+x]);
    }
  }
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      temporalDensity[y*imW+x] = densityDecay*temporalDensity[y*imW+x] + (1.0-densityDecay)*density[y*imW+x];
    }
  }

  // optionally feed it back in
  int sobelBecomesDensity = 0;
  if (sobelBecomesDensity) {
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	totalGraySobel.at<double>(y,x) = density[y*imW+x];
      }
    }
  }

  double maxGsob = -INFINITY;
  double maxYsob = -INFINITY;
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      maxGsob = max(maxGsob, totalGraySobel.at<double>(y,x));
      maxYsob = max(maxYsob, totalYSobel.at<double>(y,x));
    }
  }
  
  // ATTN 11
  // experimental
  int combineYandGray = 1;
  double yWeight = 1.0;
  if (combineYandGray) {
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	double thisY2G = min(maxYsob, yWeight * totalYSobel.at<double>(y,x));
	totalGraySobel.at<double>(y,x) += maxGsob * thisY2G * thisY2G / (maxYsob * maxYsob);
      }
    }
  }

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      if (density[y*imW+x] < maxDensity* threshFraction)
	density[y*imW+x] = 0;
    }
  }

  // smooth the density
  int smoothDensity = 1;
  double densitySigma = max(0.5, double(postDensitySigmaTrackbarVariable));//3.0;
  Mat denTemp = totalGraySobel.clone();
  if (smoothDensity) {
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	denTemp.at<double>(y,x) = density[y*imW+x];
      }
    }

    GaussianBlur(denTemp, denTemp, cv::Size(0,0), densitySigma);

    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	density[y*imW+x] = denTemp.at<double>(y,x);
      }
    }
  }

  // inject some of the Y gradient map back in AFTER feeding back to totalGraySobel
  //   so that Y contributes to objectness to help catch objects with poor color contrast,
  //   but not to pose since it is corrupted by shadows.
  int injectYGrad = 1;
  double yThresh = 0.9*maxGsob;
  if (injectYGrad) {
    // truncate again after reinjection
    maxDensity = 0;
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	maxDensity = max(maxDensity, density[y*imW+x]);
      }
    }
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	if (totalYSobel.at<double>(y,x) > yThresh) {
	  density[y*imW+x] += 0.5*maxDensity;
	}
      }
    }

    // truncate again after reinjection
    maxDensity = 0;
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	maxDensity = max(maxDensity, density[y*imW+x]);
      }
    }
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	if (density[y*imW+x] < maxDensity* threshFraction)
	  density[y*imW+x] = 0;
      }
    }
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
    int xs = g1xs;
    int xe = g1xe;
    int ys = g1ys;
    int ye = g1ye;
    for (int x = xs; x < xe; x++) {
      for (int y = ys; y < ye; y++) {
	density[y*imW+x] = 0;
	totalGraySobel.at<double>(y,x) = 0;
      }
    }
    Mat vCrop = objectViewerImage(cv::Rect(xs, ys, xe-xs, ye-ys));
    vCrop = vCrop/2;
    xs = g2xs;
    xe = g2xe;
    ys = g2ys;
    ye = g2ye;
    for (int x = xs; x < xe; x++) {
      for (int y = ys; y < ye; y++) {
	density[y*imW+x] = 0;
	totalGraySobel.at<double>(y,x) = 0;
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

  // integrate the density into the integral density
  //double maxIntegralDensity = 0;
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

  // copy the density map to the rendered image
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      uchar val = uchar(min( 255.0 * density[y*imW+x] / maxDensity, 255.0));
      densityViewerImage.at<cv::Vec3b>(y,x) = cv::Vec<uchar, 3>(0,val,0);
    }
  }

  // masked this too
  frameGraySobel = totalGraySobel.clone();
  preFrameGraySobel = totalGraySobel.clone();

  { // temporal averaging of aerial gradient
    if ( (aerialGradientTemporalFrameAverage.rows < aerialGradientReticleWidth) ||
	 (aerialGradientTemporalFrameAverage.cols < aerialGradientReticleWidth) ) {
      aerialGradientTemporalFrameAverage = Mat(imH,imW,frameGraySobel.type()); 
    }

    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	aerialGradientTemporalFrameAverage.at<double>(y, x) = 
	  aerialGradientDecay*aerialGradientTemporalFrameAverage.at<double>(y, x) + 
	  (1.0 - aerialGradientDecay)*frameGraySobel.at<double>(y, x);
      }
    }
  }

  if (useTemporalAerialGradient) {
    frameGraySobel = aerialGradientTemporalFrameAverage;
  }

  double minGraySob = INFINITY;
  double maxGraySob = -INFINITY;
  double minCrSob = INFINITY;
  double maxCrSob = -INFINITY;
  double minCbSob = INFINITY;
  double maxCbSob = -INFINITY;
  double minYSob = INFINITY;
  double maxYSob = -INFINITY;
  double minAerTemp = INFINITY;
  double maxAerTemp = -INFINITY;
  for (int y = 0; y < imH; y++) {
    for (int x = 0; x < imW; x++) {
      minGraySob = min(minGraySob, double(totalGraySobel.at<double>(y,x)));
      maxGraySob = max(maxGraySob, double(totalGraySobel.at<double>(y,x)));

      minCrSob = min(minCrSob, double(totalCrSobel.at<double>(y,x)));
      maxCrSob = max(maxCrSob, double(totalCrSobel.at<double>(y,x)));

      minCbSob = min(minCbSob, double(totalCbSobel.at<double>(y,x)));
      maxCbSob = max(maxCbSob, double(totalCbSobel.at<double>(y,x)));

      minYSob = min(minYSob, double(totalYSobel.at<double>(y,x)));
      maxYSob = max(maxYSob, double(totalYSobel.at<double>(y,x)));
      
      minAerTemp = min(minAerTemp, double(frameGraySobel.at<double>(y,x)));
      maxAerTemp = max(maxAerTemp, double(frameGraySobel.at<double>(y,x)));
    }
  }

  double sobGrayRange = maxGraySob - minGraySob;
  double sobCrRange = maxCrSob - minCrSob;
  double sobCbRange = maxCbSob - minCbSob;
  double sobYRange = maxYSob - minYSob;
  double aerTempRange = maxAerTemp - minAerTemp;

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      yCbCrGradientImage.at<cv::Vec3b>(y,x) = cv::Vec<uchar, 3>(
	uchar(max(0.0, min((128+255.0*(totalYSobel.at<double>(y,x) - minYSob - (sobYRange/2.0)) / sobYRange), 255.0))) ,
	uchar(max(0.0, min((128+255.0*(totalCrSobel.at<double>(y,x) - minCrSob - (sobCrRange/2.0)) / sobCrRange), 255.0))) ,
	uchar(max(0.0, min((128+255.0*(totalCbSobel.at<double>(y,x) - minCbSob - (sobCbRange/2.0)) / sobCbRange), 255.0))) );
    }
  }
  Mat convertedYCbCrGradientImage;
  cvtColor(yCbCrGradientImage, convertedYCbCrGradientImage, CV_YCrCb2BGR);

  // copy the density map to the rendered image
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      //uchar val = uchar(min( 1*255.0 *  (totalGraySobel.at<double>(y,x) - minGraySob) / sobGrayRange, 255.0));
      uchar val = uchar(min( 1*255.0 *  (frameGraySobel.at<double>(y,x) - minAerTemp) / aerTempRange, 255.0));
      gradientViewerImage.at<cv::Vec3b>(y,x) = cv::Vec<uchar, 3>(0,val,0);

      gradientViewerImage.at<cv::Vec3b>(y+imH,x) = convertedYCbCrGradientImage.at<cv::Vec3b>(y,x);
    }
  }

  if (shouldIRender) {
    guardedImshow(densityViewerName, densityViewerImage);
    guardedImshow(gradientViewerName, gradientViewerImage);
    guardedImshow(objectnessViewerName, objectnessViewerImage);
  }
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

  // fine tune
  //double adjusted_canny_lo_thresh = canny_lo_thresh * (1.0 + (double(loTrackbarVariable-50) / 50.0));
  //double adjusted_canny_hi_thresh = canny_hi_thresh * (1.0 + (double(hiTrackbarVariable-50) / 50.0));
  // broad tune
  double adjusted_canny_lo_thresh = canny_lo_thresh * double(loTrackbarVariable)/100.0;
  double adjusted_canny_hi_thresh = canny_hi_thresh * double(hiTrackbarVariable)/100.0;

//cout << "Here 1" << endl;
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

//cout << thisIntegral << " ";

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
//cout << "Here 2" << endl;

  // canny will start on a hi and spread on a lo or hi.
  //{for (int x = 0; x < imW-gBoxW; x+=gBoxStrideX)}
    //{for (int y = 0; y < imH-gBoxH; y+=gBoxStrideY)}
  for (int x = xS; x <= xF; x+=gBoxStrideX) {
    for (int y = yS; y <= yF; y+=gBoxStrideY) {
  
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
      	  //else if(nextX > -1 && nextX < imW && nextY > -1 && nextY < imH && 
	    //gBoxIndicator[nextY*imW+nextX] >= 1 && gBoxGrayNodes[nextY*imW+nextX] == 0) 
      	  else if(nextX >= xS && nextX <= xF && nextY >= yS && nextY <= yF && 
	    gBoxIndicator[nextY*imW+nextX] >= 1 && gBoxGrayNodes[nextY*imW+nextX] == 0) 
	    {

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
//cout << "Here 3" << endl;

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

  int closestBBToReticle = -1;
  double closestBBDistance = VERYBIGNUMBER;

  // this should be -1 if we don't find the target class 
  pilotTarget.px = -1;
  pilotTarget.py = -1;
  pilotClosestTarget.px = -1;
  pilotClosestTarget.py = -1;

  if (!all_range_mode) {
    double rejectArea = rejectAreaScale*gBoxW*gBoxH;
    for (int c = 0; c < total_components; c++) {

      cTops[c].x = max(0,min(imW-1, cTops[c].x));
      cTops[c].y = max(0,min(imH-1, cTops[c].y));
      cBots[c].x = max(0,min(imW-1, cBots[c].x));
      cBots[c].y = max(0,min(imH-1, cBots[c].y));

      int allow = 1;
      if (cBots[c].x - cTops[c].x < rejectScale*gBoxW || cBots[c].y - cTops[c].y < rejectScale*gBoxH)
	allow = 0;
      if ((cBots[c].x - cTops[c].x)*(cBots[c].y - cTops[c].y) < rejectArea)
	allow = 0;
      //if (cTops[c].y > rejectLow || cBots[c].y < rejectHigh)
	//allow = 0;
      
      // XXX for some reason there were spurious blue boxes outside of the gray box, with no green boxes,
      //  so we reject them here for now
//      if ( (cTops[c].x < max(grayTop.x-gBoxW, 0)) || (cBots[c].x > min(grayBot.x+gBoxW, imW-1)) ||
//	   (cTops[c].y < max(grayTop.y-gBoxW, 0)) || (cBots[c].y > min(grayBot.y+gBoxW, imH-1)) )
//	allow = 0;

      // ATTN 5
      // check for overlap and fuse
      cv::Point thisCen = cv::Point((cTops[c].x+cBots[c].x)/2, (cTops[c].y+cBots[c].y)/2);
      if (fuseBlueBoxes) {
	if (allow) {
	  for (int fuseIter = 0; fuseIter < fusePasses; fuseIter++) {
	    for (int cbc = 0; cbc < bTops.size(); cbc++) {

	      int smallWidth = min(bCens[cbc].x-bTops[cbc].x, thisCen.x-cTops[c].x);
	      int bigWidth = max(bCens[cbc].x-bTops[cbc].x, thisCen.x-cTops[c].x);

	      // this tests overlap
	      //if ( fabs(thisCen.x - bCens[cbc].x) < fabs(bCens[cbc].x-bTops[cbc].x+thisCen.x-cTops[c].x) && 
		   //fabs(thisCen.y - bCens[cbc].y) < fabs(bCens[cbc].y-bTops[cbc].y+thisCen.y-cTops[c].y) ) 
	      //this tests containment
	      if ( fabs(thisCen.x - bCens[cbc].x) < fabs(bigWidth - smallWidth) && 
		   fabs(thisCen.y - bCens[cbc].y) < fabs(bigWidth - smallWidth) ) 
	      {
		allow = 0;
		bTops[cbc].x = min(bTops[cbc].x, cTops[c].x);
		bTops[cbc].y = min(bTops[cbc].y, cTops[c].y);
		bBots[cbc].x = max(bBots[cbc].x, cBots[c].x);
		bBots[cbc].y = max(bBots[cbc].y, cBots[c].y);

		// gotta do this and continue searching to fuse everything, need a better algorithm in the future
		cTops[c].x = bTops[cbc].x;
		cTops[c].y = bTops[cbc].y;
		cBots[c].x = bBots[cbc].x;
		cBots[c].y = bBots[cbc].y;
	      }
	    }
	  }
	}
      }

      if (allow == 1) {
	bTops.push_back(cTops[c]);
	bBots.push_back(cBots[c]);
	bCens.push_back(thisCen);
	int t = bTops.size()-1;

	int thisArea = (cBots[c].x - cTops[c].x)*(cBots[c].y - cTops[c].y);
	if (thisArea > biggestBBArea) {
	  biggestBBArea = thisArea;
	  biggestBB = t;
	}

	double thisDistance = sqrt((bCens[t].x-reticle.px)*(bCens[t].x-reticle.px) + (bCens[t].y-reticle.py)*(bCens[t].y-reticle.py));
	cout << "   (density) Distance for box " << t << " : " << thisDistance << endl;
	if (thisDistance < closestBBDistance) {
	  closestBBDistance = thisDistance;
	  closestBBToReticle = t;
	}
      }
    }
  } else {
    bTops.push_back(armTop);
    bBots.push_back(armBot);
    bCens.push_back(cv::Point((armTop.x+armBot.x)/2, (armTop.y+armBot.y)/2));
  }

  if ((bTops.size() > 0) && (biggestBB > -1)) {
    geometry_msgs::Point p;
    p.x = bCens[biggestBB].x;
    p.y = bCens[biggestBB].y;
    p.z = 0.0;
    
      //ee_target_pub.publish(p);
    pilotTarget.px = p.x;
    pilotTarget.py = p.y;
    pilotTarget.pz = p.z;
    
    pilotTargetBlueBoxNumber = biggestBB;
  }
  if (closestBBToReticle > -1) {
    geometry_msgs::Point p;
    p.x = bCens[closestBBToReticle].x;
    p.y = bCens[closestBBToReticle].y;
    p.z = 0.0;
  
    //ee_target_pub.publish(p);
    pilotClosestTarget.px = p.x;
    pilotClosestTarget.py = p.y;
    pilotClosestTarget.pz = p.z;

    pilotClosestBlueBoxNumber = closestBBToReticle;
  } else {
    pilotClosestBlueBoxNumber = -1;
  }

  if (bTops.size() > 0) {
    geometry_msgs::Point p;
    p.x = bCens[biggestBB].x;
    p.y = bCens[biggestBB].y;
    p.z = 0.0;
  
    //ee_target_pub.publish(p);
    //pilotTarget.px = p.x;
    //pilotTarget.py = p.y;
    //pilotTarget.pz = p.z;
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

//cout << "Here 4" << endl;

  if (shouldIRender) {
    guardedImshow(objectViewerName, objectViewerImage);
  }

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

  if (shouldIRender) {
    guardedImshow(objectViewerName, objectViewerImage);
  }

}

void goClassifyBlueBoxes() {
  //cout << "entered gCBB()" << endl; cout.flush();
  Size sz = objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;
  //cout << imW << " " << imH << endl; cout.flush();

  vector< vector<int> > pIoCbuffer;

  // classify the crops
  bKeypoints.resize(bTops.size());
  bWords.resize(bTops.size());
  bYCrCb.resize(bTops.size());
  bLabels.resize(bTops.size());

  int biggestBB = -1;
  int biggestBBArea = 0;

  int closestBBToReticle = -1;
  double closestBBDistance = VERYBIGNUMBER;

  double label = -1;
  roa_to_send_blue.objects.resize(bTops.size());
  ma_to_send_blue.markers.resize(bTops.size()+1);

  for (int c = 0; c < bTops.size(); c++) {
    //cout << "  gCBB() c = " << c << endl; cout.flush();
    #ifdef DEBUG3
    fprintf(stderr, " object check1"); fflush(stderr);
    #endif
    vector<KeyPoint>& keypoints = bKeypoints[c];
    Mat descriptors;
    Mat descriptors2;

    #ifdef DEBUG3
    fprintf(stderr, " a"); fflush(stderr);
    cout << bTops[c] << bBots[c] << " "; cout.flush();
    #endif

    Mat original_cam_img = cam_img;
    Mat crop = original_cam_img(cv::Rect(bTops[c].x, bTops[c].y, bBots[c].x-bTops[c].x, bBots[c].y-bTops[c].y));
    Mat gray_image;
    Mat& yCrCb_image = bYCrCb[c];

    //if ((chosen_feature == SIFTBOW_GLOBALCOLOR_HIST) || (chosen_feature == OPPONENTSIFTBOW_GLOBALCOLOR_HIST))
    if (chosen_feature == SIFTBOW_GLOBALCOLOR_HIST) 
    {
      processImage(crop, gray_image, yCrCb_image, grayBlur);

      //detector->detect(gray_image, keypoints);
      gridKeypoints(imW, imH, bTops[c], bBots[c], gBoxStrideX, gBoxStrideY, keypoints, keypointPeriod);

      for (int kp = 0; kp < keypoints.size(); kp++) {
      #ifdef DEBUG3
      //cout << keypoints[kp].angle << " " << keypoints[kp].class_id << " " << 
      //keypoints[kp].octave << " " << keypoints[kp].pt << " " <<
      //keypoints[kp].response << " " << keypoints[kp].size << endl;
      #endif
      }

      bowExtractor->compute(gray_image, keypoints, descriptors, &pIoCbuffer);

      // save the word assignments for the keypoints so we can use them for red boxes
      #ifdef DEBUG3
      fprintf(stderr, "e "); fflush(stderr);
      cout << "pIoCbuffer: " << pIoCbuffer.size() << " "; cout.flush();
      cout << "kpSize: " << keypoints.size() << " "; cout.flush();
      #endif

      bWords[c].resize(keypoints.size());
      if ((pIoCbuffer.size() > 0) && (keypoints.size() > 0)) {
	for (int w = 0; w < vocabNumWords; w++) {
	  int numDescrOfWord = pIoCbuffer[w].size();

	  #ifdef DEBUG3
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
      
      #ifdef DEBUG3
      fprintf(stderr, " object check2"); fflush(stderr);
      #endif

      if (!descriptors.empty() && !keypoints.empty()) {
      
	appendColorHist(yCrCb_image, keypoints, descriptors, descriptors2);
	label = kNN->find_nearest(descriptors2,k);
	bLabels[c] = label;
      }
    } else if (chosen_feature == OPPONENTSIFTBOW_GLOBALCOLOR_HIST) {
      processImage(crop, gray_image, yCrCb_image, grayBlur);

      //detector->detect(gray_image, keypoints);
      gridKeypoints(imW, imH, bTops[c], bBots[c], gBoxStrideX, gBoxStrideY, keypoints, keypointPeriod);

      for (int kp = 0; kp < keypoints.size(); kp++) {
      #ifdef DEBUG3
      //cout << keypoints[kp].angle << " " << keypoints[kp].class_id << " " << 
      //keypoints[kp].octave << " " << keypoints[kp].pt << " " <<
      //keypoints[kp].response << " " << keypoints[kp].size << endl;
      #endif
      }

      //bowExtractor->compute(gray_image, keypoints, descriptors, &pIoCbuffer);

      Mat tmpC;
      crop.convertTo(tmpC, CV_32FC3);
      bowExtractor->compute(tmpC, keypoints, descriptors);

      // save the word assignments for the keypoints so we can use them for red boxes
      #ifdef DEBUG3
      fprintf(stderr, "e "); fflush(stderr);
      cout << "pIoCbuffer: " << pIoCbuffer.size() << " "; cout.flush();
      cout << "kpSize: " << keypoints.size() << " "; cout.flush();
      #endif

      bWords[c].resize(keypoints.size());
      if ((pIoCbuffer.size() > 0) && (keypoints.size() > 0)) {
	for (int w = 0; w < vocabNumWords; w++) {
	  int numDescrOfWord = pIoCbuffer[w].size();

	  #ifdef DEBUG3
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
      
      #ifdef DEBUG3
      fprintf(stderr, " object check2"); fflush(stderr);
      #endif

      if (!descriptors.empty() && !keypoints.empty()) {
      
	//appendColorHist(yCrCb_image, keypoints, descriptors, descriptors2);
	//label = kNN->find_nearest(descriptors2,k);
	label = kNN->find_nearest(descriptors,k);
	bLabels[c] = label;
      }
    } else if (chosen_feature == GRADIENT) {
      processImage(crop, gray_image, yCrCb_image, sobel_sigma);

      Mat totalGraySobel;
      {
	Mat grad_x, grad_y;
	int sobelScale = 1;
	int sobelDelta = 0;
	int sobelDepth = CV_32F;
	/// Gradient X
	Sobel(gray_image, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
	/// Gradient Y
	Sobel(gray_image, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

	grad_x = grad_x.mul(grad_x);
	grad_y = grad_y.mul(grad_y);
	totalGraySobel = grad_x + grad_y;
	// now totalGraySobel is gradient magnitude squared
      }

      // grow to the max dimension to avoid distortion
      // find the dimensions that pad the sobel image up to a square
      // raster scan a 'virtual image' to generate the 1D vector, adding 0's when on the pad

      int crows = totalGraySobel.rows;
      int ccols = totalGraySobel.cols;
      int maxDim = max(crows, ccols);
      int tRy = (maxDim-crows)/2;
      int tRx = (maxDim-ccols)/2;
      Mat gCrop(maxDim, maxDim, totalGraySobel.type());

      float totalMass = 0.0;

      for (int x = 0; x < maxDim; x++) {
	for (int y = 0; y < maxDim; y++) {
	  int tx = x - tRx;
	  int ty = y - tRy;
	  if (tx >= 0 && ty >= 0 && ty < crows && tx < ccols) {
	    gCrop.at<float>(y, x) = totalGraySobel.at<float>(ty, tx);
	    //totalMass += gCrop.at<float>(y, x);
	    totalMass += gCrop.at<float>(y, x) * gCrop.at<float>(y, x);
	  } else {
	    gCrop.at<float>(y, x) = 0.0;
	  }
	}
      }
      totalMass = sqrt(totalMass);
      Mat descriptorsG = Mat(1, gradientFeatureWidth*gradientFeatureWidth, CV_32F);
      for (int y = 0; y < gradientFeatureWidth; y++) {
	for (int x = 0; x < gradientFeatureWidth; x++) {
	  int tranX = floor(float(x)*float(maxDim)/float(gradientFeatureWidth));
	  int tranY = floor(float(y)*float(maxDim)/float(gradientFeatureWidth));
	  //descriptorsG.at<float>(x + y*gradientFeatureWidth) = gCrop.at<float>(y,x);
	  descriptorsG.at<float>(x + y*gradientFeatureWidth) = gCrop.at<float>(y,x)/totalMass;
	}
      }

      label = kNN->find_nearest(descriptorsG,k);
      bLabels[c] = label;
    } else if (chosen_feature == OPPONENT_COLOR_GRADIENT) {
      processImage(crop, gray_image, yCrCb_image, sobel_sigma);

      Mat totalGraySobel;
      {
	Mat grad_x, grad_y;
	int sobelScale = 1;
	int sobelDelta = 0;
	int sobelDepth = CV_32F;
	/// Gradient X
	Sobel(gray_image, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
	/// Gradient Y
	Sobel(gray_image, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

	grad_x = grad_x.mul(grad_x);
	grad_y = grad_y.mul(grad_y);
	totalGraySobel = grad_x + grad_y;
	// now totalGraySobel is gradient magnitude squared
      }
      Mat totalCrSobel = totalGraySobel.clone();
      {
	for (int y = 0; y < crop.rows; y++) {
	  for (int x = 0; x < crop.cols; x++) {
	    cv::Vec3b thisColor = yCrCb_image.at<cv::Vec3b>(y,x);
	    totalCrSobel.at<float>(y,x) = thisColor[1];
	  }
	}
	Mat grad_x, grad_y;
	int sobelScale = 1;
	int sobelDelta = 0;
	int sobelDepth = CV_32F;
	/// Gradient X
	Sobel(totalCrSobel, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
	/// Gradient Y
	Sobel(totalCrSobel, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

	//grad_x = grad_x.mul(grad_x);
	//grad_y = grad_y.mul(grad_y);
	totalCrSobel = grad_x + grad_y;
      }
      Mat totalCbSobel = totalGraySobel.clone();
      {
	for (int y = 0; y < crop.rows; y++) {
	  for (int x = 0; x < crop.cols; x++) {
	    cv::Vec3b thisColor = yCrCb_image.at<cv::Vec3b>(y,x);
	    totalCbSobel.at<float>(y,x) = thisColor[2];
	  }
	}
	Mat grad_x, grad_y;
	int sobelScale = 1;
	int sobelDelta = 0;
	int sobelDepth = CV_32F;
	/// Gradient X
	Sobel(totalCbSobel, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
	/// Gradient Y
	Sobel(totalCbSobel, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

	//grad_x = grad_x.mul(grad_x);
	//grad_y = grad_y.mul(grad_y);
	totalCbSobel = grad_x + grad_y;
      }

      // grow to the max dimension to avoid distortion
      // find the dimensions that pad the sobel image up to a square
      // raster scan a 'virtual image' to generate the 1D vector, adding 0's when on the pad

      int crows = totalGraySobel.rows;
      int ccols = totalGraySobel.cols;
      int maxDim = max(crows, ccols);
      int tRy = (maxDim-crows)/2;
      int tRx = (maxDim-ccols)/2;
      Mat gCrop(maxDim, maxDim, totalGraySobel.type());
      Mat crCrop(maxDim, maxDim, totalGraySobel.type());
      Mat cbCrop(maxDim, maxDim, totalGraySobel.type());

      float totalGMass = 0.0;
      float totalCrMass = 0.0;
      float totalCbMass = 0.0;

      for (int x = 0; x < maxDim; x++) {
	for (int y = 0; y < maxDim; y++) {
	  int tx = x - tRx;
	  int ty = y - tRy;
	  if (tx >= 0 && ty >= 0 && ty < crows && tx < ccols) {
	    gCrop.at<float>(y, x) = totalGraySobel.at<float>(ty, tx);
	    crCrop.at<float>(y, x) = totalCrSobel.at<float>(ty, tx);
	    cbCrop.at<float>(y, x) = totalCbSobel.at<float>(ty, tx);
	    //totalGMass += gCrop.at<float>(y, x);
	    totalGMass += gCrop.at<float>(y, x) * gCrop.at<float>(y, x);
	    totalCrMass += gCrop.at<float>(y, x) * gCrop.at<float>(y, x);
	    totalCbMass += gCrop.at<float>(y, x) * gCrop.at<float>(y, x);
	  } else {
	    gCrop.at<float>(y, x) = 0.0;
	  }
	}
      }
      totalGMass = sqrt(totalGMass);
      totalCrMass = sqrt(totalCrMass);
      totalCbMass = sqrt(totalCbMass);
      //Mat descriptorsG = Mat(1, gradientFeatureWidth*gradientFeatureWidth, CV_32F);
      Mat descriptorsCbCr = Mat(1, 2*gradientFeatureWidth*gradientFeatureWidth, CV_32F);
      for (int y = 0; y < gradientFeatureWidth; y++) {
	for (int x = 0; x < gradientFeatureWidth; x++) {
	  int tranX = floor(float(x)*float(maxDim)/float(gradientFeatureWidth));
	  int tranY = floor(float(y)*float(maxDim)/float(gradientFeatureWidth));
	  //descriptorsG.at<float>(x + y*gradientFeatureWidth) = gCrop.at<float>(y,x);
	  descriptorsCbCr.at<float>(x + y*gradientFeatureWidth) = crCrop.at<float>(y,x)/totalCrMass;
	  descriptorsCbCr.at<float>(x + y*gradientFeatureWidth + gradientFeatureWidth*gradientFeatureWidth) = cbCrop.at<float>(y,x)/totalCbMass;
	}
      }

      label = kNN->find_nearest(descriptorsCbCr,k);
      bLabels[c] = label;
    }

    if (classLabels[label].compare(invert_sign_name) == 0)
      invertQuaternionLabel = 1;


    #ifdef DEBUG3
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

    if (label == -1)
      labelName = "VOID";
    else
      labelName = classLabels[label];
    augmentedLabelName = labelName;

    if (drawLabels) {
      cv::Point text_anchor(bTops[c].x+1, bBots[c].y-2);
      cv::Point text_anchor2(bTops[c].x+1, bBots[c].y-2);
      putText(objectViewerImage, augmentedLabelName, text_anchor, MY_FONT, 0.5, Scalar(255,192,192), 2.0);
      putText(objectViewerImage, augmentedLabelName, text_anchor2, MY_FONT, 0.5, Scalar(255,0,0), 1.0);
    }


    vector<cv::Point> pointCloudPoints;

    if (label >= 0) {
      if (publishObjects) {

	fill_RO_and_M_arrays(roa_to_send_blue, 
	  ma_to_send_blue, pointCloudPoints, c, label, winningO, poseIndex);
      }

      int thisArea = (bBots[c].x - bTops[c].x)*(bBots[c].y - bTops[c].y);
      if ((thisArea > biggestBBArea) && (label == targetClass)) 
      //if ((thisArea > biggestBBArea) && (shouldIPick(label))) 
      {
	biggestBBArea = thisArea;
	biggestBB = c;
      }

      //int thisDistance = int(fabs(bCens[c].x-reticle.px) + fabs(bCens[c].y-reticle.py));
      double thisDistance = sqrt((bCens[c].x-reticle.px)*(bCens[c].x-reticle.px) + (bCens[c].y-reticle.py)*(bCens[c].y-reticle.py));
      cout << "   Distance for box " << c << " : " << thisDistance << endl;
      if (thisDistance < closestBBDistance) {
	closestBBDistance = thisDistance;
	closestBBToReticle = c;
      }
    }
  }

  if ((bTops.size() > 0) && (biggestBB > -1)) {
    geometry_msgs::Point p;
    p.x = bCens[biggestBB].x;
    p.y = bCens[biggestBB].y;
    p.z = 0.0;
    
      //ee_target_pub.publish(p);
    pilotTarget.px = p.x;
    pilotTarget.py = p.y;
    pilotTarget.pz = p.z;
    
    pilotTargetBlueBoxNumber = biggestBB;
  }
  if (closestBBToReticle > -1) {
    geometry_msgs::Point p;
    p.x = bCens[closestBBToReticle].x;
    p.y = bCens[closestBBToReticle].y;
    p.z = 0.0;
  
    //ee_target_pub.publish(p);
    pilotClosestTarget.px = p.x;
    pilotClosestTarget.py = p.y;
    pilotClosestTarget.pz = p.z;

    pilotClosestBlueBoxNumber = closestBBToReticle;
  } else {
    pilotClosestBlueBoxNumber = -1;
  }

  if (shouldIRender) {
    guardedImshow(objectViewerName, objectViewerImage);
  }

  cout << "Publish objects: " << publishObjects << endl;
  if (publishObjects) {
    rec_objs_blue.publish(roa_to_send_blue);
    markers_blue.publish(ma_to_send_blue);
  }

  //cout << "leaving gCBB()" << endl; cout.flush();
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {

  if (!shouldIMiscCallback) {
    return;
  }

  pcl::fromROSMsg(*msg, pointCloud);
#ifdef DEBUG
cout << "Hit pointCloudCallback" <<  "  " << pointCloud.size() << endl;
#endif
}

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

  nh.getParam("pink_box_threshold", pBoxThresh);
  nh.getParam("threshold_fraction", threshFraction);
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

  nh.setParam("pink_box_threshold", pBoxThresh);
  nh.setParam("threshold_fraction", threshFraction);
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
  //init_oriented_filters_all();

  //initRedBoxes();
}

void detectorsInit() {

  // XXX TODO this function should reinit the structures if this function is to be called multiple times

  // SIFT 
  //detector = new SiftFeatureDetector(0, 3, 0.04, 10, 1.6);
  cout << "chosen_feature: " << chosen_feature << endl;
  if (detector == NULL)
    detector = new FastFeatureDetector(4);

  if (extractor == NULL) {
    if (chosen_feature == SIFTBOW_GLOBALCOLOR_HIST)
      extractor = new SiftDescriptorExtractor();
    else if (chosen_feature == OPPONENTSIFTBOW_GLOBALCOLOR_HIST)
      extractor = DescriptorExtractor::create("OpponentSIFT");
    else {
      extractor = new SiftDescriptorExtractor();
    }
  }
  
  if ((chosen_feature == GRADIENT) || (chosen_feature == OPPONENT_COLOR_GRADIENT)){
    retrain_vocab = 0;
  }

  // BOW time
  bowtrainer = new BOWKMeansTrainer(vocabNumWords);

  // read the class image data
  string dot(".");
  string dotdot("..");

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

  for (unsigned int i = 0; i < classLabels.size(); i++) {
    cout << classLabels[i] << " " << classPoseModels[i] << endl;
  }

  // this is the total number of classes, so it is counted after the cache is dealt with
  numClasses = classLabels.size();

  if (loadRange) {
    initRangeMaps();
  }

  Mat vocabulary;

  if (retrain_vocab) {
    for (unsigned int i = 0; i < classLabels.size(); i++) {
      cout << "Getting BOW features for class " << classLabels[i] 
	   << " with pose model " << classPoseModels[i] << " index " << i << endl;
      bowGetFeatures(class_crops_path, classLabels[i].c_str(), grayBlur);
      if (classPoseModels[i].compare("G") == 0) {
	string thisPoseLabel = classLabels[i] + "Poses";
	bowGetFeatures(class_crops_path, thisPoseLabel.c_str(), grayBlur);
      }
    }

    cout << "Clustering features... ";
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

  if (matcher == NULL)
    matcher = new BFMatcher(NORM_L2);
  if (bowExtractor == NULL)
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

void tryToLoadRangeMap(std::string classDir, const char *className, int i) {
  {
    string thisLabelName(className);

    string dirToMakePath = data_directory + "/" + thisLabelName + "/ir2D/";
    string this_range_path = dirToMakePath + "xyzRange.yml";

    FileStorage fsfI;
    fsfI.open(this_range_path, FileStorage::READ);
    if (fsfI.isOpened()) {
      fsfI["rangeMap"] >> classRangeMaps[i]; 

      fsfI["graspMemoryTries1"] >> classGraspMemoryTries1[i];
      fsfI["graspMemoryPicks1"] >> classGraspMemoryPicks1[i];
      fsfI["graspMemoryTries2"] >> classGraspMemoryTries2[i];
      fsfI["graspMemoryPicks2"] >> classGraspMemoryPicks2[i];
      fsfI["graspMemoryTries3"] >> classGraspMemoryTries3[i];
      fsfI["graspMemoryPicks3"] >> classGraspMemoryPicks3[i];
      fsfI["graspMemoryTries4"] >> classGraspMemoryTries4[i];
      fsfI["graspMemoryPicks4"] >> classGraspMemoryPicks4[i];

      fsfI["heightMemoryTries"] >> classHeightMemoryTries[i];
      fsfI["heightMemoryPicks"] >> classHeightMemoryPicks[i];

      fsfI.release();
      cout << "Loaded rangeMap from " << this_range_path << classRangeMaps[i].size() << endl; 
      cout << "Loaded classGraspMemoryTries1 from " << this_range_path << classGraspMemoryTries1[i].size() << endl; 
      cout << "Loaded classGraspMemoryPicks1 from " << this_range_path << classGraspMemoryPicks1[i].size() << endl; 
      cout << "Loaded classGraspMemoryTries2 from " << this_range_path << classGraspMemoryTries2[i].size() << endl; 
      cout << "Loaded classGraspMemoryPicks2 from " << this_range_path << classGraspMemoryPicks2[i].size() << endl; 
      cout << "Loaded classGraspMemoryTries3 from " << this_range_path << classGraspMemoryTries3[i].size() << endl; 
      cout << "Loaded classGraspMemoryPicks3 from " << this_range_path << classGraspMemoryPicks3[i].size() << endl; 
      cout << "Loaded classGraspMemoryTries4 from " << this_range_path << classGraspMemoryTries4[i].size() << endl; 
      cout << "Loaded classGraspMemoryPicks4 from " << this_range_path << classGraspMemoryPicks4[i].size() << endl; 

      cout << "Loaded classHeightMemoryTries from " << this_range_path << classHeightMemoryTries[i].size() << endl;
      cout << "Loaded classHeightMemoryPicks from " << this_range_path << classHeightMemoryPicks[i].size() << endl;
    } else {
      classRangeMaps[i] = Mat(1, 1, CV_64F);
      classGraspMemoryTries1[i] = Mat(1, 1, CV_64F);
      classGraspMemoryPicks1[i] = Mat(1, 1, CV_64F);
      classGraspMemoryTries2[i] = Mat(1, 1, CV_64F);
      classGraspMemoryPicks2[i] = Mat(1, 1, CV_64F);
      classGraspMemoryTries3[i] = Mat(1, 1, CV_64F);
      classGraspMemoryPicks3[i] = Mat(1, 1, CV_64F);
      classGraspMemoryTries4[i] = Mat(1, 1, CV_64F);
      classGraspMemoryPicks4[i] = Mat(1, 1, CV_64F);

      classHeightMemoryTries[i] = Mat(1, 1, CV_64F);
      classHeightMemoryPicks[i] = Mat(1, 1, CV_64F);

      cout << "Failed to load rangeMap from " << this_range_path << endl; 
    }
  }
  
  // ATTN 16
  {
    {
      string thisLabelName(className);

      string dirToMakePath = data_directory + "/" + thisLabelName + "/aerialGradient/";
      string this_ag_path = dirToMakePath + "aerialHeight0Gradients.yml";

      FileStorage fsfI;
      fsfI.open(this_ag_path, FileStorage::READ);
      if (fsfI.isOpened()) {
	fsfI["aerialHeight0Gradients"] >> classHeight0AerialGradients[i]; 
	fsfI.release();
	cout << "Loaded aerial height 0 gradient from " << this_ag_path << classHeight0AerialGradients[i].size() << endl;
      } else {
	classHeight0AerialGradients[i] = Mat(1, 1, CV_64F);
	cout << "Failed to load aerialHeight0Gradients from " << this_ag_path << endl; 
      }
    }
    {
      string thisLabelName(className);

      string dirToMakePath = data_directory + "/" + thisLabelName + "/aerialGradient/";
      string this_ag_path = dirToMakePath + "aerialHeight1Gradients.yml";

      FileStorage fsfI;
      fsfI.open(this_ag_path, FileStorage::READ);
      if (fsfI.isOpened()) {
	fsfI["aerialHeight1Gradients"] >> classHeight1AerialGradients[i]; 
	fsfI.release();
	cout << "Loaded aerial height 1 gradient from " << this_ag_path << classHeight1AerialGradients[i].size() << endl;
      } else {
	classHeight1AerialGradients[i] = Mat(1, 1, CV_64F);
	cout << "Failed to load aerialHeight1Gradients from " << this_ag_path << endl; 
      }
    }
    {
      string thisLabelName(className);

      string dirToMakePath = data_directory + "/" + thisLabelName + "/aerialGradient/";
      string this_ag_path = dirToMakePath + "aerialHeight2Gradients.yml";

      FileStorage fsfI;
      fsfI.open(this_ag_path, FileStorage::READ);
      if (fsfI.isOpened()) {
	fsfI["aerialHeight2Gradients"] >> classHeight2AerialGradients[i]; 
	fsfI.release();
	cout << "Loaded aerial height 2 gradient from " << this_ag_path << classHeight2AerialGradients[i].size() << endl;
      } else {
	classHeight2AerialGradients[i] = Mat(1, 1, CV_64F);
	cout << "Failed to load aerialHeight2Gradients from " << this_ag_path << endl; 
      }
    }
    {
      string thisLabelName(className);

      string dirToMakePath = data_directory + "/" + thisLabelName + "/aerialGradient/";
      string this_ag_path = dirToMakePath + "aerialHeight3Gradients.yml";

      FileStorage fsfI;
      fsfI.open(this_ag_path, FileStorage::READ);
      if (fsfI.isOpened()) {
	fsfI["aerialHeight3Gradients"] >> classHeight3AerialGradients[i]; 
	fsfI.release();
	cout << "Loaded aerial height 3 gradient from " << this_ag_path << classHeight3AerialGradients[i].size() << endl;
      } else {
	classHeight3AerialGradients[i] = Mat(1, 1, CV_64F);
	cout << "Failed to load aerialHeight3Gradients from " << this_ag_path << endl; 
      }
    }
    cout << "Initializing classAerialGradients with classAerialHeight0Gradients." << endl;
    classAerialGradients[i] = classHeight0AerialGradients[i];
  }
}

void processSaliency(Mat in, Mat out) {
//  out = in.clone();
//
//  double saliencyPreSigma = 4.0;//0.5;//2.0;
//  GaussianBlur(out, out, cv::Size(0,0), saliencyPreSigma);
//  
//  double tMax = -INFINITY;
//  double tMin =  INFINITY;
//  for (int x = 0; x < in.cols; x++) {
//    for (int y = 0; y < in.rows; y++) {
//      tMax = max(tMax, out.at<double>(y,x));
//      tMin = min(tMin, out.at<double>(y,x));
//    }
//  }
//
//  double saliencyThresh = 0.33*(tMax-tMin) + tMin;
//  for (int x = 0; x < in.cols; x++) {
//    for (int y = 0; y < in.rows; y++) {
//      if (out.at<double>(y,x) >= saliencyThresh)
//	out.at<double>(y,x) = 1;
//      else
//	out.at<double>(y,x) = 0;
//    }
//  }
//
//  double saliencyPostSigma = 0.5;//4.0;//0.5;//2.0;
//  GaussianBlur(out, out, cv::Size(0,0), saliencyPostSigma);
  out = in.clone();

  double saliencyPreSigma = 4.0;//0.5;//2.0;
  GaussianBlur(out, out, cv::Size(0,0), saliencyPreSigma);
  
  double tMax = -INFINITY;
  double tMin =  INFINITY;
  for (int x = 0; x < in.cols; x++) {
    for (int y = 0; y < in.rows; y++) {
      tMax = max(tMax, out.at<double>(y,x));
      tMin = min(tMin, out.at<double>(y,x));
    }
  }

  double saliencyThresh = 0.1*(tMax-tMin) + tMin;
  for (int x = 0; x < in.cols; x++) {
    for (int y = 0; y < in.rows; y++) {
      if (out.at<double>(y,x) >= saliencyThresh)
	out.at<double>(y,x) = 1;
      else
	out.at<double>(y,x) = 0;
    }
  }

  double saliencyPostSigma = 4.0;//0.5;//4.0;//0.5;//2.0;
  GaussianBlur(out, out, cv::Size(0,0), saliencyPostSigma);
}

void testIncbet() {
  cout << "no trials" << endl;
  double successes = 0;
  double failures = 0;
  cout << "Successes: " << successes << " Failures: " << failures << endl;
  for (double d = 0; d < 1; d +=0.01) {
    // returns probability that mu <= d given successes and failures.
    double result = cephes_incbet(successes + 1, failures + 1, d);
    cout << "Result: " << result << endl;
  }

  successes = 10;
  failures = 10;
  cout << "Successes: " << successes << " Failures: " << failures << endl;
  for (double d = 0; d < 1; d +=0.01) {
    // returns probability that mu <= d given successes and failures.
    double result = cephes_incbet(successes + 1, failures + 1, d);
    cout << "Result: " << result << endl;
  }
}


void mapxyToij(double x, double y, int * i, int * j) 
{
  *i = round((x - mapXMin) / mapStep);
  *j = round((y - mapYMin) / mapStep);
}
void mapijToxy(int i, int j, double * x, double * y) 
{
  *x = mapXMin + i * mapStep;
  *y = mapYMin + j * mapStep;
}
bool cellIsSearched(int i, int j) {
  double x, y;
  mapijToxy(i, j, &x, &y);
  return positionIsSearched(x, y);
}

bool positionIsSearched(double x, double y) {
  if ((mapSearchFenceXMin <= x && x <= mapSearchFenceXMax) &&
      (mapSearchFenceYMin <= y && y <= mapSearchFenceYMax)) {
    return true;
  } else {
    return false;
  }

}

gsl_matrix * boxMemoryToPolygon(BoxMemory b) {
  double min_x = b.top.px;
  double min_y = b.top.py;
  double max_x = b.bot.px;
  double max_y = b.bot.py;
  double width = max_x - min_x;
  double height = max_y - min_y;

  gsl_matrix *  polygon = gsl_matrix_alloc(2, 4);
  gsl_matrix_set(polygon, 0, 0, min_x);
  gsl_matrix_set(polygon, 1, 0, min_y);

  gsl_matrix_set(polygon, 0, 1, min_x + width);
  gsl_matrix_set(polygon, 1, 1, min_y);

  gsl_matrix_set(polygon, 0, 2, min_x + width);
  gsl_matrix_set(polygon, 1, 2, min_y + height);

  gsl_matrix_set(polygon, 0, 3, min_x);
  gsl_matrix_set(polygon, 1, 3, min_y + height);
  return polygon;
}

gsl_matrix * mapCellToPolygon(int map_i, int map_j) {
  
  double min_x, min_y;
  mapijToxy(map_i, map_j, &min_x, &min_y);
  double max_x = min_x + mapStep;
  double max_y = min_y + mapStep;
  double width = max_x - min_x;
  double height = max_y - min_y;

  gsl_matrix *  polygon = gsl_matrix_alloc(2, 4);
  gsl_matrix_set(polygon, 0, 0, min_x);
  gsl_matrix_set(polygon, 1, 0, min_y);

  gsl_matrix_set(polygon, 0, 1, min_x + width);
  gsl_matrix_set(polygon, 1, 1, min_y);

  gsl_matrix_set(polygon, 0, 2, min_x + width);
  gsl_matrix_set(polygon, 1, 2, min_y + height);

  gsl_matrix_set(polygon, 0, 3, min_x);
  gsl_matrix_set(polygon, 1, 3, min_y + height);
  return polygon;
}

bool isBoxMemoryIKPossible(BoxMemory b) {
  int toReturn = 1;
  {
    int i, j;
    mapxyToij(b.top.px, b.top.py, &i, &j);
    toReturn &= isCellIkPossible(i, j);
  }
  {
    int i, j;
    mapxyToij(b.bot.px, b.bot.py, &i, &j);
    toReturn &= isCellIkPossible(i, j);
  }
  {
    int i, j;
    mapxyToij(b.bot.px, b.top.py, &i, &j);
    toReturn &= isCellIkPossible(i, j);
  }
  {
    int i, j;
    mapxyToij(b.top.px, b.bot.py, &i, &j);
    toReturn &= isCellIkPossible(i, j);
  }
  return toReturn;
}

bool isBlueBoxIKPossible(cv::Point tbTop, cv::Point tbBot) {
  double zToUse = trueEEPose.position.z+currentTableZ;
  int toReturn = 1;
  {
    double tbx, tby;
    int tbi, tbj;
    pixelToGlobal(tbTop.x, tbTop.y, zToUse, &tbx, &tby);
    mapxyToij(tbx, tby, &tbi, &tbj);
    toReturn &= isCellIkPossible(tbi, tbj);
  }
  {
    double tbx, tby;
    int tbi, tbj;
    pixelToGlobal(tbBot.x, tbBot.y, zToUse, &tbx, &tby);
    mapxyToij(tbx, tby, &tbi, &tbj);
    toReturn &= isCellIkPossible(tbi, tbj);
  }
  {
    double tbx, tby;
    int tbi, tbj;
    pixelToGlobal(tbTop.x, tbBot.y, zToUse, &tbx, &tby);
    mapxyToij(tbx, tby, &tbi, &tbj);
    toReturn &= isCellIkPossible(tbi, tbj);
  }
  {
    double tbx, tby;
    int tbi, tbj;
    pixelToGlobal(tbBot.x, tbTop.y, zToUse, &tbx, &tby);
    mapxyToij(tbx, tby, &tbi, &tbj);
    toReturn &= isCellIkPossible(tbi, tbj);
  }
  return toReturn;
}

bool boxMemoryIntersectsMapCell(BoxMemory b, int map_i, int map_j) {
  gsl_matrix * bpolygon = boxMemoryToPolygon(b);

  gsl_matrix * map_cell = mapCellToPolygon(map_i, map_j);

  bool result = math2d_overlaps(bpolygon, map_cell);
  gsl_matrix_free(bpolygon);
  gsl_matrix_free(map_cell);

  return result;
}

bool boxMemoryTooOld(BoxMemory b) {
  
}

bool boxMemoryIntersectPolygons(BoxMemory b1, BoxMemory b2) {
  gsl_matrix * p1 = boxMemoryToPolygon(b1);
  gsl_matrix * p2 = boxMemoryToPolygon(b2);

  bool result = math2d_overlaps(p1, p2);
  
  gsl_matrix_free(p1);
  gsl_matrix_free(p2);

  return result;
}

bool boxMemoryIntersectCentroid(BoxMemory b1, BoxMemory b2) {
  gsl_matrix * p1 = boxMemoryToPolygon(b1);
  gsl_matrix * p2 = boxMemoryToPolygon(b2);
  gsl_vector * p1_center = math2d_point(b1.centroid.px, b1.centroid.py);
  gsl_vector * p2_center = math2d_point(b2.centroid.px, b2.centroid.py);

  bool result;
  if (math2d_is_interior_point(p2_center, p1) || 
      math2d_is_interior_point(p1_center, p2)) {
    result = true;
  } else {
    result = false;
  }
    
  gsl_matrix_free(p1);
  gsl_matrix_free(p2);
  gsl_vector_free(p1_center);
  gsl_vector_free(p2_center);

  return result;
}


vector<BoxMemory> memoriesForClass(int classIdx) {
  vector<BoxMemory> results;
  for (int j = 0; j < blueBoxMemories.size(); j++) {
    if (blueBoxMemories[j].labeledClassIndex == focusedClass) {
      results.push_back(blueBoxMemories[j]);
    }
  }
  return results;
}

vector<BoxMemory> memoriesForClass(int classIdx, int * memoryIdxOfFirst) {
  vector<BoxMemory> results;
  int haventFoundFirst = 1;
  for (int j = 0; j < blueBoxMemories.size(); j++) {
    if (blueBoxMemories[j].labeledClassIndex == focusedClass) {
      results.push_back(blueBoxMemories[j]);
      if (haventFoundFirst) {
	*memoryIdxOfFirst = j;
	haventFoundFirst = 0;
      }
    }
  }
  return results;
}

bool cellIsMapped(int i, int j) {
  double x, y;
  mapijToxy(i, j, &x, &y);
  return positionIsMapped(x, y);
}
bool positionIsMapped(double x, double y) {
  if ((mapRejectFenceXMin <= x && x <= mapRejectFenceXMax) &&
      (mapRejectFenceYMin <= y && y <= mapRejectFenceYMax)) {
    return true;
  } else {
    return false;
  }
}

bool isCellInPursuitZone(int i, int j) {
  return ( (clearanceMap[i + mapWidth * j] == 1) ||
	   (clearanceMap[i + mapWidth * j] == 2) );
} 
bool isCellInPatrolZone(int i, int j) {
  return (clearanceMap[i + mapWidth * j] == 2);
} 

bool isCellInteresting(int i, int j) {
  if ( (clearanceMap[i + mapWidth * j] == 1) ||
       (clearanceMap[i + mapWidth * j] == 2) ) {
    return ( objectMap[i + mapWidth * j].lastMappedTime < lastScanStarted );
  } else {
    return false;
  }
} 
void markCellAsInteresting(int i, int j) {
  if ( (clearanceMap[i + mapWidth * j] == 1) ||
       (clearanceMap[i + mapWidth * j] == 2) ) {
    objectMap[i + mapWidth * j].lastMappedTime = ros::Time(0.001);
    return;
  } else {
    return;
  }
} 
void markCellAsNotInteresting(int i, int j) {
  if ( (clearanceMap[i + mapWidth * j] == 1) ||
       (clearanceMap[i + mapWidth * j] == 2) ) {
    objectMap[i + mapWidth * j].lastMappedTime = ros::Time::now() + ros::Duration(VERYBIGNUMBER);
    return;
  } else {
    return;
  }
} 

bool isCellIkColliding(int i, int j) {
  return (ikMap[i + mapWidth * j] == 2);
} 
bool isCellIkPossible(int i, int j) {
  return (ikMap[i + mapWidth * j] == 0);
} 
bool isCellIkImpossible(int i, int j) {
  return (ikMap[i + mapWidth * j] == 1);
} 


int blueBoxForPixel(int px, int py)
{
  for (int c = 0; c < bTops.size(); c++) {
    if ((bTops[c].x <= px && px <= bBots[c].x) &&
        (bTops[c].y <= py && py <= bBots[c].y)) {
      return c;
    }
  }
  return -1;
}

int skirtedBlueBoxForPixel(int px, int py, int skirtPixels) {
  vector<cv::Point> newBTops;
  vector<cv::Point> newBBots;
  newBTops.resize(bBots.size());
  newBBots.resize(bTops.size()); 
  for (int c = 0; c < bTops.size(); c++) {
    newBTops[c].x = bTops[c].x-skirtPixels;
    newBTops[c].y = bTops[c].y-skirtPixels;
    newBBots[c].x = bBots[c].x+skirtPixels;
    newBBots[c].y = bBots[c].y+skirtPixels;
  }

  for (int c = 0; c < newBTops.size(); c++) {
    if ((newBTops[c].x <= px && px <= newBBots[c].x) &&
        (newBTops[c].y <= py && py <= newBBots[c].y)) {
      return c;
    }
  }
  return -1;
}

void randomizeNanos(ros::Time * time) {
  double nanoseconds = rk_double(&random_state) * 1000;
  time->nsec = nanoseconds;
}


void initializeMap() 
{
  for (int i = 0; i < mapWidth; i++) {
    for(int j = 0; j < mapHeight; j++) {
      objectMap[i + mapWidth * j].lastMappedTime = ros::Time::now() - ros::Duration(mapBlueBoxCooldown);
      // make the search more random
      randomizeNanos(&objectMap[i + mapWidth * j].lastMappedTime);


      objectMap[i + mapWidth * j].detectedClass = -1;
      objectMap[i + mapWidth * j].pixelCount = 0;
      objectMap[i + mapWidth * j].r = 0;
      objectMap[i + mapWidth * j].g = 0;
      objectMap[i + mapWidth * j].b = 0;

      ikMap[i + mapWidth * j] = 0;
      clearanceMap[i + mapWidth * j] = 0;
    }
  }
  lastScanStarted = ros::Time::now();
}

bool isSketchyMat(Mat sketchy) {
  return ( (sketchy.rows <= 1) || (sketchy.rows <= 1) );
}

void guardViewers() {
  if ( isSketchyMat(objectViewerYCbCrBlur) ) {
    objectViewerYCbCrBlur = Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_64FC3);
  }
  if ( isSketchyMat(objectViewerGrayBlur) ) {
    objectViewerGrayBlur = Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_64FC3);
  }
  if ( isSketchyMat(densityViewerImage) ) {
    densityViewerImage = cv_ptr->image.clone();
    densityViewerImage *= 0;
  }
  if ( isSketchyMat(accumulatedImage) ) {
    accumulatedImage = Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_64FC3);
  }
  if ( isSketchyMat(accumulatedImageMass) ) {
    accumulatedImageMass = Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_64F);
  }
  if ( isSketchyMat(gradientViewerImage) ) {
    gradientViewerImage = Mat(2*cv_ptr->image.rows, cv_ptr->image.cols, cv_ptr->image.type());
  }
  if ( isSketchyMat(objectnessViewerImage) ) {
    objectnessViewerImage = Mat(cv_ptr->image.rows, cv_ptr->image.cols, cv_ptr->image.type());
    objectnessViewerImage *= 0;
  }
  if ( isSketchyMat(aerialGradientViewerImage) ) {
    aerialGradientViewerImage = Mat(4*aerialGradientWidth, aerialGradientWidth, CV_64F);
  }
  if ( isSketchyMat(objectViewerImage) ) {
    objectViewerImage = cv_ptr->image.clone();
  }
}

void guardedImshow(string name, Mat image) {
  if ( !isSketchyMat(image) ) {
    imshow(name, image);
  }
}

////////////////////////////////////////////////
// end node definitions 
//
// start ein 
////////////////////////////////////////////////

int main(int argc, char **argv) {
  initVectorArcTan();

  //testIncbet();
  //exit(0);
  //auto lfunc = [] () { cout << "Hello world"; }; 
  //lfunc();
  //exit(0);

  words = create_words();
  character_code_to_word = create_character_code_to_word(words);
  name_to_word = create_name_to_word(words);

  srand(time(NULL));
  time(&firstTime);
  time(&firstTimeRange);

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
  else {
    programName = string(PROGRAM_NAME);
  }

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

  unsigned long seed = 1;
  rk_seed(seed, &random_state);

  // Models: ObjNessB2W8MAXBGR ObjNessB2W8I ObjNessB2W8HSV
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

  rec_objs_blue_memory = n.advertise<object_recognition_msgs::RecognizedObjectArray>("blue_memory_objects", 10);
  markers_blue_memory = n.advertise<visualization_msgs::MarkerArray>("blue_memory_markers", 10);

  rec_objs_blue = n.advertise<object_recognition_msgs::RecognizedObjectArray>("blue_labeled_objects", 10);
  rec_objs_red = n.advertise<object_recognition_msgs::RecognizedObjectArray>("red_labeled_objects", 10);
  markers_blue = n.advertise<visualization_msgs::MarkerArray>("blue_object_markers", 10);
  markers_red = n.advertise<visualization_msgs::MarkerArray>("red_object_markers", 10);
  ee_target_pub = n.advertise<geometry_msgs::Point>("pilot_target_" + left_or_right_arm, 10);

  densityViewerName = "Density Viewer " + left_or_right_arm;
  objectViewerName = "Object Viewer " + left_or_right_arm;
  gradientViewerName = "Gradient Viewer " + left_or_right_arm;
  aerialGradientViewerName = "Aerial Gradient Viewer " + left_or_right_arm;
  objectnessViewerName = "Objectness Viewer " + left_or_right_arm;

  cv::namedWindow(gradientViewerName);
  cv::namedWindow(aerialGradientViewerName);
  cv::namedWindow(densityViewerName);
  cv::namedWindow(objectViewerName);
  setMouseCallback(objectViewerName, nodeCallbackFunc, NULL);

  createTrackbar("post_density_sigma", densityViewerName, &postDensitySigmaTrackbarVariable, 40);
  createTrackbar("canny_lo", densityViewerName, &loTrackbarVariable, 100);
  createTrackbar("canny_hi", densityViewerName, &hiTrackbarVariable, 100);
  createTrackbar("blinder_columns", densityViewerName, &blinder_columns, 20);
  createTrackbar("blinder_stride", densityViewerName, &blinder_stride, 50);
  createTrackbar("add_blinders", densityViewerName, &add_blinders, 1);

  ros::Subscriber epState =   n.subscribe("/robot/limb/" + left_or_right_arm + "/endpoint_state", 1, endpointCallback);
  ros::Subscriber gripState = n.subscribe("/robot/end_effector/" + left_or_right_arm + "_gripper/state", 1, gripStateCallback);
  ros::Subscriber eeRanger =  n.subscribe("/robot/range/" + left_or_right_arm + "_hand_range/state", 1, rangeCallback);
  ros::Subscriber eeTarget =  n.subscribe("/ein_" + left_or_right_arm + "/pilot_target_" + left_or_right_arm, 1, targetCallback);
  ros::Subscriber jointSubscriber = n.subscribe("/robot/joint_states", 1, jointCallback);

  wristViewName = "Wrist View " + left_or_right_arm;
  coreViewName = "Core View " + left_or_right_arm;
  rangeogramViewName = "Rangeogram View " + left_or_right_arm;
  rangemapViewName = "Range Map View " + left_or_right_arm;
  graspMemoryViewName = "Grasp Memory View " + left_or_right_arm;
  graspMemorySampleViewName = "Grasp Memory Sample View " + left_or_right_arm;
  heightMemorySampleViewName = "Height Memory Sample View " + left_or_right_arm;
  hiRangemapViewName = "Hi Range Map View " + left_or_right_arm;
  hiColorRangemapViewName = "Hi Color Range Map View " + left_or_right_arm;

  cv::namedWindow(wristViewName);
  cv::setMouseCallback(wristViewName, pilotCallbackFunc, NULL);
  cv::namedWindow(graspMemoryViewName);
  cv::setMouseCallback(graspMemoryViewName, graspMemoryCallbackFunc, NULL);
  cv::namedWindow(coreViewName);
  cv::namedWindow(rangeogramViewName);

  ros::Subscriber fetchCommandSubscriber;
  fetchCommandSubscriber = n.subscribe("/fetch_commands", 1, 
                                       fetchCommandCallback);

  ros::Subscriber forthCommandSubscriber;
  forthCommandSubscriber = n.subscribe("/ein/" + left_or_right_arm + "/forth_commands", 1, 
                                       forthCommandCallback);

  ros::Timer timer1 = n.createTimer(ros::Duration(0.0001), timercallback1);

  tfListener = new tf::TransformListener();

  ikClient = n.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/" + left_or_right_arm + "/PositionKinematicsNode/IKService");
  joint_mover = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/" + left_or_right_arm + "/joint_command", 10);
  gripperPub = n.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/" + left_or_right_arm + "_gripper/command",10);
  moveSpeedPub = n.advertise<std_msgs::Float64>("/robot/limb/" + left_or_right_arm + "/set_speed_ratio",10);
  sonarPub = n.advertise<std_msgs::UInt16>("/robot/sonar/head_sonar/set_sonars_enabled",10);
  headPub = n.advertise<baxter_core_msgs::HeadPanCommand>("/robot/head/command_head_pan",10);
  nodPub = n.advertise<std_msgs::Bool>("/robot/head/command_head_nod",10);

  currentHeadPanCommand.target = 0;
  currentHeadPanCommand.speed = 50;
  currentHeadNodCommand.data = 0;
  currentSonarCommand.data = 0;


  facePub = n.advertise<std_msgs::Int32>("/confusion/target/command", 10);

  vmMarkerPublisher = n.advertise<visualization_msgs::MarkerArray>("volumetric_rgb_map", 10);

  {
    baxter_core_msgs::EndEffectorCommand command;
    command.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
    command.id = 65538;
    gripperPub.publish(command);
  }

  frameGraySobel = Mat(1,1,CV_64F);

  initializeMap();

  spinlessNodeMain();
  spinlessPilotMain();

  saveROSParams();
  //pushWord("visionPatrol");
  pushWord("printState");
  pushCopies("zUp", 15);
  int devInit = 1;
  if (devInit) {
    pushWord("incrementTargetClass"); 
    pushWord("gradientServoTakeClosest"); 
    pushWord("synchronicServoTakeClosest");
  }
  pushWord("silenceSonar");
  pushWord("printWords");
  pushWord("openGripper");
  pushWord("calibrateGripper");
  pushWord("shiftIntoGraspGear1"); // change gear to 1

  execute_stack = 1;

  int cudaCount = gpu::getCudaEnabledDeviceCount();
  cout << "cuda count: " << cudaCount << endl;;

  cvWaitKey(1); // this might be good to init cv gui stuff
  lastImageCallbackReceived = ros::Time::now();

  ros::spin();

  return 0;
}


#include "ein_words.cpp"
#include "slu/math2d.c"
#include "slu/gsl_utilities.c"

