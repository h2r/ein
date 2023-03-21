#ifndef _CONFIG_H_
#define _CONFIG_H_

#define EPSILON 1.0e-9
#define VERYBIGNUMBER 1e12


#if defined(USE_ROBOT_AIBO)
#include "aibo/ein_aibo.h"
#elif defined(USE_ROBOT_BAXTER)
#include "baxter/ein_baxter.h"
#elif defined(USE_ROBOT_PIDRONE)
#include "pidrone/ein_pidrone.h"
#elif defined(USE_ROBOT_JACO)
#include "jaco/ein_jaco.h"
#elif defined(USE_ROBOT_MOVO)
#include "movo/ein_movo.h"
#elif defined(USE_ROBOT_KUKA)
#include "kuka/ein_kuka.h"
#elif defined(USE_ROBOT_SPOT)
#include "spot/ein_spot.h"
#else
#include "defaultrobot/ein_robot.h"
#endif

#include "gaussian_map.h"
#include "ein_util.h"
#include "eePose.h"

class GaussianMap;
class TransitionTable;
class Scene;
class OrientedRay;
class Camera;

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <image_transport/image_transport.hpp>

#include <opencv2/opencv.hpp> 

#include "ein/msg/ein_state.hpp"
#include "ein/msg/ein_console.hpp"
#include "distributions.h"

#define NUM_JOINTS 7

class EinWindow;
class ArmWidget;
class GaussianMapWindow;
class StreamViewerWindow;
class DiscrepancyWindow;

typedef enum {
  IK_GOOD = 0,
  IK_LIKELY_IN_COLLISION = 2,
  IK_FAILED = 1
} ikMapState;

typedef enum {
  DISCREPANCY_POINT = 0,
  DISCREPANCY_DOT = 1,
  DISCREPANCY_NOISY_OR = 2,
  DISCREPANCY_NOISY_AND = 3,
} discrepancyModeState;

typedef enum {
  CLEARANCE_DO_NOT_PURSUE = 0,
  CLEARANCE_PURSUE = 1,
  CLEARANCE_SEARCH = 2
} clearanceMapState;

typedef enum {
  ARMED = 0,
  BLOCKED = 1,
  STOPPED = 2,
  HOVERING = 3,
  MOVING = 4
} movementState;

typedef enum {
  IDLING = 0,
  PATROLLING = 1,
  PICKING = 2,
  PLACING = 3,
  HANDING = 4
} patrolState;

typedef enum {
  ONCE = 0,
  LOOP = 1
} patrolMode;

typedef enum {
  HAND = 0,
  PLACE_REGISTER = 2,
  HOLD = 3,
  SHAKE= 4
} placeMode;

typedef enum {
  EMPTY = 0,
  STOPCLEAR = 1,
  PATROL = 2,
  CRANE = 3,
  SHRUG = 4
} idleMode;

typedef enum {
  SIFTBOW_GLOBALCOLOR_HIST = 1,
  OPPONENTSIFTBOW_GLOBALCOLOR_HIST = 2, // this has not been sufficiently tested
  SIFTCOLORBOW_HIST = 3, // unimplemented, calculate color histograms at each keypoint and augment each SIFT feature before clustering
  GRADIENT = 4,
  OPPONENT_COLOR_GRADIENT = 5,
  CBCR_HISTOGRAM = 6
} featureType;

typedef enum {
  PHYSICAL,
  SIMULATED,
  SNOOP
} robotMode;

typedef enum {
  IK_BOUNDARY_STOP = 0,
  IK_BOUNDARY_PASS = 1
} ikBoundaryMode;

typedef enum {
  IKSERVICE,
  IKFAST,
  IKFASTDEBUG
} ikMode;

typedef enum {
  IKF_NO_ARGUMENTS_LOCAL = 0,
  IKF_NO_ARGUMENTS_GLOBAL = 1,
  IKF_SWITCHING = 2
} ikFastMode;

string ikModeToString(ikMode mode);


typedef enum {
  GRASP_CRANE,
  GRASP_3D
} graspMode;


typedef enum {
  STATIC_PRIOR = 1,
  LEARNING_SAMPLING = 2,
  LEARNING_ALGORITHMC = 3,
  STATIC_MARGINALS = 4,
  MAPPING = 5
} pickMode;

typedef enum {
  NOT_CENTERED = 0,
  CENTERED = 1
} scanMode;

typedef enum {
  HISTOGRAM_CLASSIFY = 0,
  ONCE_CLASSIFY = 1,
  FIXED_CLASS_ACCUMULATED = 2,
  FIXED_CLASS_CONTINUOUS = 3,
  FIXED_CLASS_ACCUMULATED_NOSYN = 4,
  FIXED_CLASS_CONTINUOUS_NOSYN = 5
} mapServoMode;

typedef enum {
  FOCUSED_CLASS = 0,
  LATENT_CLASS = 1
} gradientServoMode;

typedef enum {
  INSTANT = 0,
  STEP = 1
} executionMode;

typedef enum {
  EEPOSITION = 0,
  ANGLES = 1,
  VELOCITY = 2
} controlMode;

typedef enum {
  SC_DISCREPANCY_ONLY = 0,
  SC_DISCREPANCY_THEN_LOGLIKELIHOOD = 1
} sceneClassificationMode;

typedef enum {
  WAIT_KEEP_ON = 0,
  WAIT_BACK_UP = 1
} waitMode;

typedef enum {
  CAMCAL_LINBOUNDED = 0,
  CAMCAL_QUADRATIC = 1,
  CAMCAL_HYPERBOLIC = 2
} cameraCalibrationMode;

typedef enum {
  FIXATE_STREAM = 0,
  FIXATE_CURRENT = 1
} sceneFixationMode;

std::string pickModeToString(pickMode mode);


typedef enum {
  NO_LOCK = 0,
  CENTROID_LOCK = 1,
  POSE_LOCK = 2,
  POSE_REPORTED = 3
} memoryLockType;



typedef enum { 
	ANIMATION_OFF = 0,
	ANIMATION_ON = 1 
} animationMode; 

struct AnimationState {
	String emotion; 
	int value; 
}; 




struct Grasp {
  eePose grasp_pose;
  double tries;
  double successes;
  double failures;
  double jams;

  void writeToFileStorage(FileStorage& fsvO) const;

  void readFromFileNodeIterator(FileNodeIterator& it);
  void readFromFileNode(FileNode& it);

  friend ostream & operator<<(ostream &, const Grasp&);
};

struct CollisionDetection {
  rclcpp::Time time;
  bool inCollision;
};


struct BoxMemory {
  cv::Point bTop;
  cv::Point bBot;
  eePose cameraPose;
  eePose lockedPose;
  eePose aimedPose;
  eePose pickedPose;
  eePose top;
  eePose bot;
  eePose centroid;
  rclcpp::Time cameraTime;
  int labeledClassIndex;
  memoryLockType lockStatus;
  double trZ;

  vector<eePose> aff3dGraspPoses;
  vector<eePose> affPlaceOverPoses;
  vector<eePose> affPlaceUnderPoses;
};

typedef struct MapCell {
  rclcpp::Time lastMappedTime;
  int detectedClass; // -1 means not denied
  double r, g, b;
  double pixelCount;
} MapCell;

typedef struct Sprite {
  // sprites are the objects which are rendered in the simulation,
  //   modeled physically as axis aligned bounding boxes
  // the aa-bb associated with
  //  the sprite encompasses the rotated blitted box of the image
  // there is a master array of sprite types loaded from files, and then
  //  there is a vector of active sprites, each of which is a clone of an
  //  entry in the master array but with a unique name and state information
  Mat image;
  string name; // unique identifier
  double scale; // this is pixels / cm
  rclcpp::Time creationTime;
  eePose top;
  eePose bot;
  eePose pose;
} Sprite;

typedef struct streamEePose {
  eePose arm_pose;
  eePose base_pose;
  double time;
} streamEePose;

typedef struct streamRange{
  double range;
  double time;
} streamRange;

typedef struct streamImage{
  string filename;
  Mat image;
  int loaded;
  double time;
} streamImage;

typedef struct streamJoints{
  double jointPositions[NUM_JOINTS];
  double jointVelocities[NUM_JOINTS];
  double jointEfforts[NUM_JOINTS];
  double time;
} streamJoints;

typedef struct streamWord {
  string word;
  string command;
  double time;
} streamWord;

typedef struct streamLabel {
  string label;
  double time;
} streamLabel;





#define NOW_THATS_COARSE 0.08
#define GRID_EVEN_COARSER 0.04
#define GRID_COARSER 0.02
#define GRID_COARSE 0.01
#define GRID_MEDIUM 0.005 //.005
#define GRID_FINE 0.0025
#define GRID_VERY_FINE 0.00125

class Word;
class CompoundWord;

class EinBaxterConfig;
class EinAiboConfig;
class EinPidroneConfig;
class EinJacoConfig;
class EinMovoConfig;
class EinKukaConfig;

class EinConfig {
 public:

  EinBaxterConfig * baxterConfig;
  EinAiboConfig * aiboConfig;
  EinPidroneConfig * pidroneConfig;
  EinJacoConfig * jacoConfig;
  EinMovoConfig * movoConfig;
  EinKukaConfig * kukaConfig;

  
  std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;  

  rclcpp::Time lastStatePubTime;
  rclcpp::Publisher<ein::msg::EinState>::SharedPtr einStatePub;
  rclcpp::Publisher<ein::msg::EinConsole>::SharedPtr einConsolePub;

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ee_target_pub;
  image_transport::ImageTransport * it;


  int zero_g_toggle = 1;
  int publish_commands_mode = 1;


  const int epRingBufferSize = 50000;
  const int rgRingBufferSize = 100;

  // we make use of a monotonicity assumption
  // if the current index passes the last recorded index, then we just proceed
  //  and lose the ranges we skipped over. alert when this happens
  // first valid entries
  int epRingBufferStart = 0;
  int rgRingBufferStart = 0;
  
  // first free entries
  int epRingBufferEnd = 0;
  int rgRingBufferEnd = 0;



  std::vector<geometry_msgs::msg::Pose> epRingBuffer;
  std::vector<double> rgRingBuffer;
  

  std::vector<rclcpp::Time> epRBTimes;
  std::vector<rclcpp::Time> rgRBTimes;


  movementState currentMovementState = STOPPED;
  patrolState currentPatrolState = IDLING;
  patrolMode currentPatrolMode = ONCE;
  placeMode currentPlaceMode = HOLD;
  idleMode currentIdleMode = CRANE;
  graspMode currentGraspMode = GRASP_3D;
  robotMode currentRobotMode = PHYSICAL;
  ikMode currentIKMode = IKSERVICE;
  ikFastMode currentIKFastMode = IKF_SWITCHING;
  ikBoundaryMode currentIKBoundaryMode = IK_BOUNDARY_STOP;
  scanMode currentScanMode = CENTERED;
  mapServoMode currentMapServoMode = HISTOGRAM_CLASSIFY;
  gradientServoMode currentGradientServoMode = FOCUSED_CLASS;
  controlMode currentControlMode = EEPOSITION;
  sceneClassificationMode currentSceneClassificationMode = SC_DISCREPANCY_THEN_LOGLIKELIHOOD;
  bool setRandomPositionAfterPick = false;
  bool streamPicks = false;
  bool mapAutoPick = false;
  bool snapToFlushGrasp = true;

  double graspBackoffDistance = 0.20;

  int fakeBBWidth = 50;

  eePose placeTarget;

  float eeLinearAcceleration[4];

  float irGlobalPositionEEFrame[4];


  // set color reticles iterator
  int scrI = 0;

  double currentGraspZ = 0;
  vector<double> classGraspZs;
  vector<double> classGraspZsSet;

  int current3dGraspIndex = -1;
  eePose c3dPoseBase;
  vector< vector<Grasp> > class3dGrasps;
  vector< vector<eePose> > classPlaceOverPoints;
  vector< vector<eePose> > classPlaceUnderPoints;

  double movingThreshold = 0.02;
  double hoverThreshold = 0.003; 
  double stoppedTimeout = 0.25;


  featureType chosen_feature = SIFTBOW_GLOBALCOLOR_HIST;
  int cfi = 1;

  int gradientFeatureWidth = 50;


  int driveVelocities = 0;
  int testJoint = 3;
  
  int jointNamesInit = 0;
  std::vector<std::string> jointNames;

  double trueJointPositions[NUM_JOINTS] = {0, 0, 0, 0, 0, 0, 0};
  double trueJointVelocities[NUM_JOINTS] = {0, 0, 0, 0, 0, 0, 0};
  double trueJointEfforts[NUM_JOINTS] = {0, 0, 0, 0, 0, 0, 0};

  double joint_min[NUM_JOINTS];
  double joint_max[NUM_JOINTS];

  double last_joint_actual_effort[NUM_JOINTS];
  double actual_effort_thresh = 0.0; 
  double target_joint_actual_effort[NUM_JOINTS];

  rk_state random_state;

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

  int shouldIRenderDefault = 1;
  int shouldIRender = 0;
  int shouldIDoIK = 1;
  int renderInit = 0;
  int shouldIImageCallback = 1;
  int shouldIMiscCallback = 1;
  int shouldIRangeCallback = 1;

  int ik_reset_counter = 0;
  int ikInitialized = 0;
  int goodIkInitialized = 0;
  double ikShare = 1.0;
  int ik_reset_thresh = 20;
  bool lastIkWasSuccessful = true;


  int sensorStreamOn = 0;
  int diskStreamingEnabled = 0;
  double sensorStreamLastActivated = 0.0;
  double sensorStreamTimeout= 3600.0;
  double expectedCropsToStream = 500;
  // should I stream
  int sisPose = 0;
  int sisRange = 0;
  int sisImage = 0;
  int sisJoints= 0;
  int sisWord = 0;
  int sisLabel = 0;
  int streamPoseBatchSize = 100;
  int streamRangeBatchSize = 100;
  int streamJointsBatchSize = 100;
  int streamWordBatchSize = 100;
  int streamLabelBatchSize = 5;
  std::vector<streamEePose> streamPoseBuffer;
  std::vector<streamRange> streamRangeBuffer;
  std::vector<streamJoints> streamJointsBuffer;
  std::vector<streamWord> streamWordBuffer;
  std::vector<streamLabel> streamLabelBuffer;
  // stream image buffer current index
  int srbCurIdx = 0;
  int spbCurIdx = 0;
  int sjbCurIdx = 0;
  int swbCurIdx = 0;
  int slbCurIdx = 0;

  Mat accumulatedStreamImage;
  Mat accumulatedStreamImageMass;
  Mat accumulatedStreamImageBytes;

  int globalPngCompression = 0;


  const double eeRangeMaxValue = 65.535;
  double eeRange = 0.0;


  double bDelta = GRID_COARSE;

  bool showgui = true;

  EinWindow * dogSnoutViewWindow;

  EinWindow * rangeogramWindow;
  EinWindow * wristViewWindow;
  EinWindow * renderedWristViewWindow;
  EinWindow * coreViewWindow;
  EinWindow * mapBackgroundViewWindow;
  EinWindow * faceViewWindow;
  EinWindow * heightMemorySampleWindow;

  EinWindow * meanViewerWindow;
  EinWindow * objectViewerWindow;
  EinWindow * objectMapViewerWindow;
  EinWindow * stereoViewerWindow;
  EinWindow * backgroundWindow;
  EinWindow * discrepancyWindow;
  EinWindow * discrepancyDensityWindow;
  EinWindow * zWindow;
  EinWindow * observedWindow;
  EinWindow * observedStdDevWindow;
  EinWindow * predictedWindow;
  EinWindow * predictedStdDevWindow;

  GaussianMapWindow * backgroundMapWindow;
  GaussianMapWindow * observedMapWindow;
  GaussianMapWindow * predictedMapWindow;

  StreamViewerWindow * streamViewerWindow;

  DiscrepancyWindow * discrepancyViewerWindow;

  ArmWidget * armWidget;

  int last_key;

  eePose calibrationPose;
  eePose shrugPose;
  eePose handingPose;

  //eePose straightDown = {.px = 0.0, .py = 0.0, .pz = 0.0,
  //.qx = 0.0, .qy = 1.0, .qz = 0.0, .qw = 0.0}; 
  eePose straightDown = eePose(0.0, 0.0, 0.0,
                               0.0, 1.0, 0.0, 0.0); 



  eePose eepReg1;
  eePose eepReg2;
  eePose eepReg3;
  eePose eepReg4;
  eePose eepReg5;
  eePose eepReg6;

  eePose beeHome;
  eePose backScanningPose;
  eePose pilotTarget;
  eePose pilotClosestTarget;
  eePose lastGoodEEPose;
  eePose currentEEPose;
  eePose currentEEDeltaRPY;
  eePose ik_reset_eePose;
  eePose crane1;


  std::vector<eePose> deliveryPoses;
  int currentDeliveryPose = 0;

  int pilotTargetBlueBoxNumber = -1;
  int pilotClosestBlueBoxNumber = -1;
  string left_or_right_arm = "right";
  string other_arm = "left";
  string robot_serial;
  string robot_description;
  string robot_software_version;
  string ein_software_version;
  string robot_type;

  eePose trueEEWrench;
  double averagedWrechAcc = 0;
  double averagedWrechMass = 0;
  double averagedWrechDecay = 0.95;
  eePose trueEEPoseEEPose;
  eePose lastHandEEPose;

  eePose trueRangePose;

  std::string baseTfFrame;
  std::string eeTfFrame;

  std::string forthCommand;



  double oneTable = 0.175;
  double rightTableZ = 0.172;//0.165;//0.19;//0.18;
  double leftTableZ = 0.172;//0.165;//0.19;//0.177;
  
  double bagTableZ = oneTable;//0.165;//0.19;//0.18; //0.195;//0.22;
  double counterTableZ = oneTable;//0.165;//0.19;//0.18;//0.209123; //0.20;//0.18;
  double pantryTableZ = oneTable;//0.165;//0.19;//0.18;//0.209123; //0.195;
  
  double currentTableZ = leftTableZ;
  

  rclcpp::Time firstTableHeightTime;
  double mostRecentUntabledZWait = 2.0;
  double mostRecentUntabledZLastValue = INFINITY;
  double mostRecentUntabledZDecay = 0.97;
  double mostRecentUntabledZ = 0.0;
  eePose bestOrientationEEPose = straightDown;
  double bestOrientationAngle = 0;

  int bfc = 0;
  int bfc_period = 3;

  Mat coreViewImage;
  Mat rangeogramImage;

  Mat wristCamImage;
  int wristCamInit = 0;





  const static int totalRangeHistoryLength = 100;
  const static int rggScale = 1;  
  const static int rggStride = 5*rggScale;
  const static int rggHeight = 300*rggScale;
  const static int rggWidth = totalRangeHistoryLength*rggStride;

  double rangeHistory[totalRangeHistoryLength];
  int currentRangeHistoryIndex = 0;


  double filter[9] = {1.0/16.0, 1.0/8.0, 1.0/16.0, 
                      1.0/8.0, 1.0/4.0, 1.0/8.0, 
                      1.0/16.0, 1.0/8.0, 1.0/16.0};;
  
  // diagonalKappa: 0.72 deltaDiagonalKappa: 0.01
  // below .72, the horizontal won when it should have. Set to .67 to be safe.
  // .67 was a little unreliable, trimming a bit more.
  double diagonalKappa = 0.60;
  
  const static int parzenKernelHalfWidth = 15;
  const static int parzenKernelWidth = 2*parzenKernelHalfWidth+1;
  double parzenKernel[parzenKernelWidth*parzenKernelWidth];
  double parzenKernelSigma = 4.0;
  //double parzenKernelSigma = 2.0;
  //double parzenKernelSigma = 1.0; // this is approximately what it should be at 20 cm height
  //double parzenKernelSigma = 0.5;  
  // 13.8 cm high -> 2.2 cm gap
  // 23.8 cm high -> 3.8 cm gap
  // 4 sigma (centered at 0) should be the gap
  // TODO can 'adjust' and bounds on the fly during lookup in proportion to the measured depth
  
  // assumptions are made here so if these values changes, the code must
  //  be audited.
  
  double lastiX = 0;
  double lastiY = 0;
  double thisiX = 0;
  double thisiY = 0;
  

  // height Thompson parameters
  constexpr static double minHeight = 0.255;//0.09;//-0.10;
  constexpr static double maxHeight = 0.655;//0.49;//0.3;

  
  
  const static int hmWidth = 4;
  double currentThompsonHeight = 0;
  int currentThompsonHeightIdx = 0;
  
  // the currently equipped depth reticle
  double drX = .02; //.01;
  double drY = .02;
  
  // target reticle
  double trX = 0;
  double trY = 0;
  double trZ = 0;
  
  int maxX = 0;
  int maxY = 0;
  double maxD = 0;
  int maxGG = 0;
  int localMaxX = 0;
  int localMaxY = 0;
  int localMaxGG = 0;
  bool breakGraspTiesWithNoise = true;
  
  // grasp gear should always be even
  static const int totalGraspGears = 8;
  int currentGraspGear = -1;

  // XXX maybe we should initialize this to a reasonable value
  //// reticles
  double ggX[totalGraspGears];
  double ggY[totalGraspGears];
  double ggT[totalGraspGears];

  int castRecentRangeRay = 1;
  int recordRangeMap = 0;

  double w1GoThresh = 0.03;//0.01;
  double w1AngleThresh = 0.02; 
  double synKp = 0.0005;
  double darkKp = 0.0005;
  double faceKp = 0.001;
  double gradKp = 0.00025;//0.0005;
  double kPtheta1 = 1.0;//0.75;
  double kPtheta2 = 0.125;//0.75;
  int kPThresh = 3;
  double lastPtheta = INFINITY;

  // pre-absolute
  //int synServoPixelThresh = 10;//15;//10;
  //int gradServoPixelThresh = 2;
  //int gradServoThetaThresh = 1;
  // absolute
  int synServoPixelThresh = 15;//30;//15;//10;
  int gradServoPixelThresh = 5;
  int gradServoThetaThresh = 2;
  
  int synServoLockFrames = 0;


  rclcpp::Time oscilStart;
  double oscCenX = 0.0;
  double oscCenY = 0.0;
  double oscCenZ = 0.0;
  double oscAmpX = 0.10;//.0.16;//0.08;//0.1;
  double oscAmpY = 0.10;//0.16;//0.2;
  double oscAmpZ = 0.0;
  
  constexpr static double commonFreq = 1.0;//1.0/2.0;
  double oscFreqX = commonFreq*1.0/3.0;
  double oscFreqY = commonFreq*1.0/20.0;
  double oscFreqZ = commonFreq*1.0;
  double visionCycleInterval = 7.5 / 7.0 * (1.0/commonFreq);

  rclcpp::Time lastVisionCycle;
  rclcpp::Duration accumulatedTime;

  int targetClass = -1;

  // class focused for learning
  int focusedClass = -1;
  string focusedClassLabel;

  int synchronicTakeClosest = 0;
  int gradientTakeClosest = 0;
  int gradientServoDuringHeightLearning = 1;
  int bailAfterSynchronic = 1;
  int bailAfterGradient = 0;
  double gradientServoResetThresh = 0.7/(6.0e5);
  int densityIterationsForGradientServo = 10;//3;//10;

  int softMaxGradientServoIterations = 2;//5;//3;//10;//3;
  int hardMaxGradientServoIterations = 5;//10;//2;//5;//5;//3;//10;//20;//3;//10;
  int currentGradientServoIterations = 0;


  int gripperMoving = 0;
  double gripperPosition = 0;
  int gripperGripping = 0;
  double gripperThresh = 3.5;//6.0;//7.0;
  rclcpp::Time gripperLastUpdated;
  double gripperNotMovingConfirmTime = 0.25;
  // the last value the gripper was at when it began to open from a closed position
  double lastMeasuredClosed = 3.0;

  rclcpp::Time graspTrialStart;
  double graspAttemptCounter = 0;
  double graspSuccessCounter = 0;
  double graspSuccessRate = 0;
  operationStatusType thisGraspPicked = UNKNOWN;
  operationStatusType thisGraspReleased = UNKNOWN;

  string lastLabelLearned;

  int gmTargetX = -1;
  int gmTargetY = -1;

  int orientationCascade = 0;
  int lPTthresh = 3;
  int orientationCascadeHalfWidth = 2;

  
  double aerialGradientDecay = 0.9;//0.965;//0.9;
  Mat aerialGradientTemporalFrameAverage;
  Mat preFrameGraySobel;
  Mat frameGraySobel;


  double graspDepthOffset = -0.02;//-0.01;
  eePose lastPickPose;
  eePose lastPrePickPose;
  eePose lastLockedPose;
  
  // this needs to place the gripper BELOW the table
  //  by a margin, or it could prevent getting flush
  //  with the table near a sag
  //almost vertical for signs
  //camera: 0.25525; 0.85732; 0.94282
  //range: 0.24561; 0.82523; 0.956
  //hand: 0.21624; 0.84881; 0.92597
  //straight down for magnitudes
  //
  //camera 0.29581; 0.76695; 0.066762
  //range  0.30204; 0.73505; 0.053803
  //hand   0.33396; 0.75551; 0.082641
  double pickFlushFactor = 0.108;//0.08;//0.09;//0.11;
  eePose handEndEffectorOffset = {0,0,0.028838, 0,0,0,1};
  eePose handRangeOffset = {0.03192,-0.02046,0.028838, 0,0,0,1};
  eePose handToRethinkEndPointTransform = {0,0,0, 0,0,0,1};
  eePose handFromEndEffectorTransform = {0,0,0, 0,0,0,1};


  int useContinuousGraspTransform = 1;


  int paintEEandReg1OnWrist = 1;

  // d values obtained by putting laser in gripper
  //  to find end effector projection, then using
  //  a tape dot to find the vanishing point of
  //  the camera
  // the estimated vanishing point is actually pretty
  //  close to the measured one
  //double d_y = -0.04;
  //double d_x = 0.018;
  //eePose handCameraOffset = {0.03815,0.01144,0.01589, 0,0,0,1};
  double offX = 0;
  double offY = 0;
  sceneFixationMode currentSceneFixationMode = FIXATE_STREAM;

  int mappingServoTimeout = 5;
  //const int mappingHeightIdx = 0;
  const int mappingHeightIdx = 1;


  const static int vaNumAngles = 360;
  constexpr static double vaDelta = (2.0 * 3.1415926) / vaNumAngles;
  double vaX[vaNumAngles];
  double vaY[vaNumAngles];


  waitMode currentWaitMode = WAIT_KEEP_ON;
  int waitUntilAtCurrentPositionCounter = 0;
  int waitUntilAtCurrentPositionCounterTimeout = 300;
  rclcpp::Time waitUntilAtCurrentPositionStart;
  double  waitUntilAtCurrentPositionTimeout = 60.0;
  int waitUntilEffortCounter = 0;
  int waitUntilEffortCounterTimeout = 3000;
  rclcpp::Time pressUntilEffortStart;
  double pressUntilEffortTimeout = 30.0;
  int waitUntilGripperNotMovingCounter = 0;
  int waitUntilGripperNotMovingTimeout = 100;
  rclcpp::Time waitUntilGripperNotMovingStamp;

  double currentEESpeedRatio = 0.5;

  int endCollapse = 0;
  int endThisStackCollapse = 0;




  int heartBeatCounter = 0;
  int heartBeatPeriod = 150;


  rclcpp::Time lastAccelerometerCallbackRequest;
  rclcpp::Time lastGripperCallbackRequest;
  rclcpp::Time lastEndpointCallbackRequest;
  
  rclcpp::Time lastAccelerometerCallbackReceived;
  rclcpp::Time lastGripperCallbackReceived;
  rclcpp::Time lastEndpointCallbackReceived;

  rclcpp::Time lastImageStamp;
  rclcpp::Time lastImageFromDensityReceived;

  rclcpp::Time lastImageCallbackRequest;

  bool usePotentiallyCollidingIK = 0;

  Mat objectViewerYCbCrBlur;
  Mat objectViewerGrayBlur;


  rclcpp::Time lastHoverRequest;
  double hoverTimeout = 3.0;//2.0; // seconds
  double hoverGoThresh = 0.02;
  double hoverAngleThresh = 0.02;
  eePose lastHoverTrueEEPoseEEPose;

  rclcpp::Time lastMovementStateSet;
  eePose lastTrueEEPoseEEPose;
  
  rclcpp::Time comeToHoverStart;
  double comeToHoverTimeout = 3.0;
  rclcpp::Time comeToStopStart;
  double comeToStopTimeout = 30.0;
  rclcpp::Time waitForTugStart;
  double waitForTugTimeout = 1e10;
  double armedThreshold = 0.05;



  double simulatorCallbackFrequency = 30.0;
  rclcpp::TimerBase::SharedPtr simulatorCallbackTimer;

  rclcpp::TimerBase::SharedPtr timer1;
  
  int mbiWidth = 2000;
  int mbiHeight = 2000;
  Mat mapBackgroundImage;
  Mat originalMapBackgroundImage;
  
  int objectInHandLabel = -1;
  int simulatedObjectHalfWidthPixels = 50;
  
  int numCornellTables = 10;
  vector<eePose> cornellTables;
  int currentCornellTableIndex = 0;
  
  
  string robot_mode = "";
  
  int targetInstanceSprite = 0;
  int targetMasterSprite = 0;

  vector<Sprite> masterSprites;
  vector<Sprite> instanceSprites;



  int darkServoIterations = 0;
  int darkServoTimeout = 20;
  int darkServoPixelThresh = 10;
  
  int faceServoIterations = 0;
  int faceServoTimeout = 2000;
  int faceServoPixelThresh = 1;
  
  int setVanishingPointPixelThresh = 3;
  int setVanishingPointIterations = 0;
  int setVanishingPointTimeout = 6;


  // node variables
  int retrain_vocab = 0;
  int rewrite_labels = 0;
  int reextract_knn = 0;

  int drawOrientor = 1;
  int drawGreen = 1;
  int drawBlue = 1;
  int drawGray = 1;
  int drawBlueKP = 1;



  Mat stereoViewerImage;
  Mat objectViewerImage;
  Mat objectMapViewerImage;
  Mat densityViewerImage;
  Mat wristViewImage;
  Mat faceViewImage;

  int mask_gripper_blocks = 0;
  int mask_gripper = 1;

  int loTrackbarVariable = 20;//30;//45;//75;
  int hiTrackbarVariable = 35;//40;//50;
  int postDensitySigmaTrackbarVariable = 10.0;
  

  double canny_hi_thresh = 5e5;//7;
  double canny_lo_thresh = 5e5;//4;


  double sobel_sigma = 2.0;//4.0;

  double sobel_scale_factor = 1e-12;

  const int keypointPeriod = 1;


  // ATTN 25
  //const int vocabNumWords = 1000;
  const int vocabNumWords = 1000;//2000;

  constexpr static double grayBlur = 1.0;
  int grandTotalDescriptors = 0;



  std::string data_directory;
  std::string vocab_file = "vocab.yml";
  std::string knn_file = "knn.yml";
  std::string label_file = "labels.yml";
  std::string config_directory = "/config/";
  std::string config_filename;
  
  std::string run_prefix = "";
  std::string scan_group = "";
  //std::string class_name = "unspecified_cn";
  //std::string class_labels= "unspecified_cl1 unspecified_cl2";
  //std::string class_pose_models = "unspecified_pm1 unspecified_pm2";
  
  vector<Camera *> cameras;
  int focused_camera = -1;

  std::string cache_prefix = "";

  
  int cropCounter;

  double maxDensity = 0;
  double *density = NULL;
  double *preDensity = NULL;
  double *integralDensity = NULL;
  double *temporalDensity = NULL;
  
  double densityDecay = 0.5;//0.9;//0.3;//0.7;
  double threshFraction = 0.2;
  
  int biggestL1 = 0;



  // Top variables are top left corners of bounding boxes (smallest coordinates)
  // Bot variables are bottom right corners of bounding boxes (largest coordinates)
  // Cen variables are centers of bounding boxes
  
  // bounding boxes of connected components of green matter,
  //  they are the candidate blue boxes
  vector<cv::Point> cTops; 
  vector<cv::Point> cBots;


  constexpr static double mapXMin = -0.9375;
  constexpr static double mapXMax = 0.9375;
  constexpr static double mapYMin = -1.5;
  constexpr static double mapYMax = 1.5;
  constexpr static double mapStep = 0.01;


  //  constexpr static double mapXMin = -1.5;
  //  constexpr static double mapXMax = 1.5; 
  //  constexpr static double mapYMin = -1.5;
  //  constexpr static double mapYMax = 1.5; 
  //  constexpr static double mapStep = 0.01;


  //constexpr static double mapXMin = -0.75;
  //constexpr static double mapXMax = 1.0; 
  //constexpr static double mapYMin = -0.1;
  //constexpr static double mapYMax = 1.25; 
  //constexpr static double mapStep = 0.01; 
  
  double mapSearchFenceXMin;
  double mapSearchFenceXMax;
  double mapSearchFenceYMin;
  double mapSearchFenceYMax;
  
  double mapRejectFenceXMin;
  double mapRejectFenceXMax;
  double mapRejectFenceYMin;
  double mapRejectFenceYMax;
  
  double mapBackgroundXMin;
  double mapBackgroundXMax;
  double mapBackgroundYMin;
  double mapBackgroundYMax;
  
  double mapBackgroundBufferMeters = 1.0;
  
  const static int mapWidth = (mapXMax - mapXMin) / mapStep;
  const static int mapHeight = (mapYMax - mapYMin) / mapStep;

  MapCell objectMap[mapWidth * mapHeight];
  rclcpp::Time lastScanStarted;


  int ikMap[mapWidth * mapHeight];
  int clearanceMap[mapWidth * mapHeight];
  int drawClearanceMap = 1;
  int drawIKMap = 1;
  int useGlow = 0;
  int useFade = 1;

  int pursuitProximity = 5;
  int searchProximity = 23;//15;//10;


  double ikMapStartHeight;
  double ikMapEndHeight;
  const static int numIkMapHeights = 10;
  int ikMapAtHeight[mapWidth * mapHeight * numIkMapHeights];
  
  
  vector<BoxMemory> blueBoxMemories;
  int targetBlueBox = 0;


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
  double rejectAreaScale = 16;//6*6


  // XXX this should probably be odd
  int aerialGradientWidth = 100;
  int aerialGradientReticleWidth = 200;
  
  
  double *gBoxIndicator;
  int gBoxW = 10;
  int gBoxH = 10;
  
  int gBoxStrideX;
  int gBoxStrideY;


  // pink box thresholds for the principle classes
  double *pBoxIndicator = NULL;

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

  double stereoFocal = 1.0; // needs to be tuned
  double stereoBaseline = 0.01;
  double stereoMaxDisparity = 128;
  Mat stereoImage1;
  Mat stereoImage2;
  Mat stereoDisparity;
  Mat stereoDepth;

  eePose photoPinPose;

  Mat chHistogram;
  Mat chDistribution;
  int chWinner;

  eePose gshHistogram;
  double gshCounts;
  eePose gshPose;

  list<CollisionDetection> collisionStateBuffer;

  int numCollisions() {
    int numCollisions = 0;
    for (std::list<CollisionDetection>::iterator it=collisionStateBuffer.begin(); it != collisionStateBuffer.end(); ++it) {
      if (it->inCollision) {
	numCollisions++;
      }
    }
    return numCollisions;
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr moveEndEffectorCommandCallbackSub;
  rclcpp::Subscription<ein::msg::EinState>::SharedPtr einSub;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr forthCommandSubscriber;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr forthCommandPublisher;

  rclcpp::Time waitForSecondsTarget;
  rclcpp::Time spinForSecondsTarget;

  rclcpp::Time measureTimeTarget;
  rclcpp::Time measureTimeStart;
  double measureTimePeriod = 1.0;
  

  eePose pressPose;
  double twistThresh = 0.01;

  double rockDiffA = 0.0;
  double rockDiffB = 0.0;

  eePose targetWrench;
  double wrenchThresh = 15.0;

  int intendedEnableState = 1;
  int lastShoulderState = 0;

  int lastArmOkButtonState = 0;
  int lastArmShowButtonState = 0;
  int lastArmBackButtonState = 0;

  double torsoFanState;
  

  shared_ptr<TransitionTable> transition_table;
  shared_ptr<Scene> scene;
  shared_ptr<GaussianMap> gaussian_map_register;
  double sceneMinSigmaSquared = 10;
  int sceneCellCountThreshold = 20;
  int sceneDiscrepancySearchDepth = 3000;
  discrepancyModeState discrepancyMode = DISCREPANCY_POINT;

  int sceneInitWidth  = 901;
  int sceneInitHeight = 901;

  vector<shared_ptr<Scene> > class_scene_models;
  double scene_score_thresh = 0.01;

  vector<shared_ptr<GaussianMap> > depth_maps;
  int sceneDepthPatchHalfWidth = 0;

  shared_ptr<GaussianMap> reprojection_buffer;


  vector<OrientedRay> rayBuffer;

  int angular_aperture_cols = 351;
  int angular_aperture_rows = 351;
  int angular_baffle_cols = 0;
  int angular_baffle_rows = 0;

  animationMode currentAnimationMode = ANIMATION_ON; 
  AnimationState currentAnimationState = {"confused", 0}; 
  AnimationState targetAnimationState = {"confused", 0}; 
  std::map<string, int> emotionIndex; 
  vector< vector<Mat> > emotionImages;
  double animationRate = 60; 

  double wristViewBrightnessScalar = 1.0;


  EinConfig() : accumulatedTime(0,0) {
  }
  
}; // config end



/**
 * MachineState represents the call stack for einl.  It consists of
 * the call stack, the variables, the current instruction, and whether
 * the stack is currently executing.
 */
class MachineState: public std::enable_shared_from_this<MachineState> {
 private:
 public:
  MachineState();
  MachineState * ms;

  std::vector<std::shared_ptr<Word> > call_stack;
  std::vector<std::shared_ptr<Word> > data_stack;
  std::vector<std::shared_ptr<Word> > control_stack;

  std::map<string, std::shared_ptr<Word> > variables;

  std::shared_ptr<Word> current_instruction = NULL;

  EinConfig config;

  int execute_stack = 0;

  executionMode execution_mode = INSTANT;
  map<string, shared_ptr<Word> > wordsInNamespace();

  shared_ptr<CompoundWord> nil;
  
  bool pushWord(int code);
  bool pushWord(string name);
  bool pushWord(std::shared_ptr<Word> word);
  bool pushData(int code);
  bool pushData(string name);
  bool pushData(std::shared_ptr<Word> word);
  bool pushControl(int code);
  bool pushControl(string name);
  bool pushControl(std::shared_ptr<Word> word);
  std::shared_ptr<Word> popWord();
  std::shared_ptr<Word> popData();
  std::shared_ptr<Word> popControl();
  void clearStack();
  void clearData();
  void clearControl();
  void pushNoOps(int n);
  void pushCopies(int symbol, int times);
  void pushCopies(string symbol, int times);
  void pushCopies(std::shared_ptr<Word> word, int times);

  void execute(std::shared_ptr<Word> word);

  string currentState();

  void moveEndEffectorCommandCallback(const geometry_msgs::msg::Pose& msg);  

  void forthCommandCallback(const std_msgs::msg::String& msg);
  void evaluateProgram(const string program);
  void accelerometerCallback(const sensor_msgs::msg::Imu& moment);
  void rangeCallback(const sensor_msgs::msg::Range& range);
  void timercallback1();
  void imageCallback(Camera * camera);

  void targetCallback(const geometry_msgs::msg::Point& point);

  void einStateCallback(const ein::msg::EinState & msg);

  void publishConsoleMessage(string msg);

  int getStreamPoseAtTime(double tin, eePose * outArm, eePose * outBase);
  int getStreamPoseAtTimeThreadSafe(double tin, eePose * outArm, eePose * outBase);

};


#endif /* _CONFIG_H_ */
