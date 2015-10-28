#ifndef _CONFIG_H_
#define _CONFIG_H_


#include <ros/package.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/RecognizedObject.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>


#include <baxter_core_msgs/CameraControl.h>
#include <baxter_core_msgs/OpenCamera.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/CollisionDetectionState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/HeadPanCommand.h>
#include <baxter_core_msgs/SEAJointState.h>



#include <ros/package.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/RecognizedObject.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <baxter_core_msgs/CameraControl.h>
#include <baxter_core_msgs/OpenCamera.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/HeadPanCommand.h>
#include <baxter_core_msgs/ITBState.h>
#include <baxter_core_msgs/DigitalIOState.h>


#include "eigen_util.h"
#include <ein/EinState.h>
#include "distributions.h"

#define NUM_JOINTS 7

class EinWindow;
class ArmWidget;

typedef enum {
  IK_GOOD = 0,
  IK_LIKELY_IN_COLLISION = 2,
  IK_FAILED = 1
} ikMapState;

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

string ikModeToString(ikMode mode);


typedef enum {
  GRASP_CRANE,
  GRASP_3D
} graspMode;

typedef enum {
  UNKNOWN = -1,
  FAILURE = 0,
  SUCCESS = 1
} operationStatusType;

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
  INSTANT = 0,
  STEP = 1
} executionMode;


std::string pickModeToString(pickMode mode);


typedef enum {
  NO_LOCK = 0,
  CENTROID_LOCK = 1,
  POSE_LOCK = 2,
  POSE_REPORTED = 3
} memoryLockType;

struct CollisionDetection {
  ros::Time time;
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
  ros::Time cameraTime;
  int labeledClassIndex;
  memoryLockType lockStatus;
  double trZ;

  vector<eePose> aff3dGraspPoses;
  vector<eePose> affPlaceOverPoses;
  vector<eePose> affPlaceUnderPoses;
};

typedef struct MapCell {
  ros::Time lastMappedTime;
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
  ros::Time creationTime;
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


class EinConfig {
 public:


  baxter_core_msgs::HeadPanCommand currentHeadPanCommand;
  std_msgs::Bool currentHeadNodCommand;
  std_msgs::UInt16 currentSonarCommand;
  std_msgs::UInt32 currentStiffnessCommand;
  
  tf::TransformListener* tfListener;
  
  baxter_core_msgs::SolvePositionIK ikRequest;
  baxter_core_msgs::SolvePositionIK lastGoodIkRequest;
  
  ros::ServiceClient ikClient;
  ros::ServiceClient cameraClient;
  ros::Publisher joint_mover;
  ros::Publisher gripperPub;
  ros::Publisher facePub;
  ros::Publisher moveSpeedPub;
  ros::Publisher sonarPub;
  ros::Publisher headPub;
  ros::Publisher nodPub;
  ros::Publisher stiffPub;
  ros::Publisher einPub;
  ros::Publisher vmMarkerPublisher;
  ros::Publisher rec_objs_blue_memory;
  ros::Publisher markers_blue_memory;
  ros::Publisher ee_target_pub;

  ros::Publisher digital_io_pub;

  ros::Publisher sonar_pub;
  ros::Publisher red_halo_pub;
  ros::Publisher green_halo_pub;

  int sonar_led_state = 0;
  double red_halo_state = 100.0;
  double green_halo_state = 100.0;
  int repeat_halo = 1;


  int zero_g_toggle = 1;

  const int imRingBufferSize = 300;
  const int epRingBufferSize = 10000;
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


  movementState currentMovementState = STOPPED;
  patrolState currentPatrolState = IDLING;
  patrolMode currentPatrolMode = ONCE;
  placeMode currentPlaceMode = HAND;
  idleMode currentIdleMode = CRANE;
  graspMode currentGraspMode = GRASP_3D;
  robotMode currentRobotMode = PHYSICAL;
  ikMode currentIKMode = IKSERVICE;
  ikBoundaryMode currentIKBoundaryMode = IK_BOUNDARY_STOP;
  scanMode currentScanMode = CENTERED;
  mapServoMode currentMapServoMode = HISTOGRAM_CLASSIFY;
  bool setRandomPositionAfterPick = false;
  bool streamPicks = false;
  bool mapAutoPick = false;
  bool snapToFlushGrasp = true;

  int fakeBBWidth = 50;

  eePose placeTarget;

  Vector3d eeLinearAcceleration;

  // set color reticles iterator
  int scrI = 0;

  double currentGraspZ = 0;
  vector<double> classGraspZs;
  vector<double> classGraspZsSet;

  int current3dGraspIndex = 0;
  eePose c3dPoseBase;
  vector< vector<eePose> > class3dGrasps;
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
  std::vector<streamImage> streamImageBuffer;
  std::vector<streamJoints> streamJointsBuffer;
  std::vector<streamWord> streamWordBuffer;
  std::vector<streamLabel> streamLabelBuffer;
  // stream image buffer current index
  int sibCurIdx = 0;
  int srbCurIdx = 0;
  int spbCurIdx = 0;
  int sjbCurIdx = 0;
  int swbCurIdx = 0;
  int slbCurIdx = 0;

  Mat accumulatedStreamImage;
  Mat accumulatedStreamImageMass;
  Mat accumulatedStreamImageBytes;

  int globalPngCompression = 0;



  double eeRange = 0.0;


  double bDelta = GRID_COARSE;

  EinWindow * rangeogramWindow;
  EinWindow * wristViewWindow;
  EinWindow * coreViewWindow;
  EinWindow * rangemapWindow;
  EinWindow * hiRangemapWindow;
  EinWindow * hiColorRangemapWindow;
  EinWindow * graspMemoryWindow;
  EinWindow * graspMemorySampleWindow;
  EinWindow * mapBackgroundViewWindow;
  EinWindow * faceViewWindow;
  EinWindow * heightMemorySampleWindow;

  EinWindow * gripperMaskFirstContrastWindow;
  EinWindow * gripperMaskSecondContrastWindow;
  EinWindow * gripperMaskDifferenceWindow;

  EinWindow * gripperMaskMeanWindow;
  EinWindow * gripperMaskVarianceWindow;
  EinWindow * gripperMaskSquaresWindow;


  EinWindow * densityViewerWindow;
  EinWindow * objectViewerWindow;
  EinWindow * objectMapViewerWindow;
  EinWindow * gradientViewerWindow;
  EinWindow * aerialGradientViewerWindow;
  EinWindow * stereoViewerWindow;
  ArmWidget * armWidget;

  int last_key;

  eePose calibrationPose;
  eePose shrugPose;
  eePose handingPose;

  eePose straightDown = {.px = 0.0, .py = 0.0, .pz = 0.0,
                         .qx = 0.0, .qy = 1.0, .qz = 0.0, .qw = 0.0}; 

  eePose cropUpperLeftCorner = {.px = 320, .py = 200, .pz = 0.0,
                                .qx = 0.0, .qy = 1.0, .qz = 0.0, .qw = 0.0}; // center of image

  eePose centerReticle = {.px = 325, .py = 127, .pz = 0.0,
                          .qx = 0.0, .qy = 0.0, .qz = 0.0, .qw = 0.0};

  eePose defaultReticle = centerReticle;
  eePose heightReticles[4];

  eePose probeReticle = defaultReticle;
  eePose vanishingPointReticle = defaultReticle;
  eePose reticle = defaultReticle;



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
  string robot_serial;
  string robot_description;


  geometry_msgs::Pose trueEEPose;
  eePose trueEEWrench;
  double averagedWrechAcc = 0;
  double averagedWrechMass = 0;
  double averagedWrechDecay = 0.95;
  eePose trueEEPoseEEPose;

  std::string forthCommand;



  double oneTable = 0.175;
  double rightTableZ = 0.172;//0.165;//0.19;//0.18;
  double leftTableZ = 0.172;//0.165;//0.19;//0.177;
  
  double bagTableZ = oneTable;//0.165;//0.19;//0.18; //0.195;//0.22;
  double counterTableZ = oneTable;//0.165;//0.19;//0.18;//0.209123; //0.20;//0.18;
  double pantryTableZ = oneTable;//0.165;//0.19;//0.18;//0.209123; //0.195;
  
  double currentTableZ = leftTableZ;
  

  ros::Time firstTableHeightTime;
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
  Mat rangemapImage;
  Mat hiRangemapImage;
  Mat hiColorRangemapImage;
  Mat graspMemoryImage;
  Mat graspMemorySampleImage;
  Mat heightMemorySampleImage;

  Mat wristCamImage;
  int wristCamInit = 0;





  const static int totalRangeHistoryLength = 100;
  const static int rggScale = 1;  
  const static int rggStride = 5*rggScale;
  const static int rggHeight = 300*rggScale;
  const static int rggWidth = totalRangeHistoryLength*rggStride;

  double rangeHistory[totalRangeHistoryLength];
  int currentRangeHistoryIndex = 0;

  const static int rmWidth = 21; // must be odd
  const static int rmHalfWidth = (rmWidth-1)/2; // must be odd
  const static int rmiCellWidth = 20;
  const static int rmiHeight = rmiCellWidth*rmWidth;
  const static int rmiWidth = rmiCellWidth*rmWidth;


  constexpr static double rmDelta = 0.01;
  int rangeMapTargetSearchPadding = 3;
  double rangeMap[rmWidth*rmWidth];
  double rangeMapAccumulator[rmWidth*rmWidth];
  double rangeMapMass[rmWidth*rmWidth];
  
  double rangeMapReg1[rmWidth*rmWidth];
  double rangeMapReg2[rmWidth*rmWidth];
  double rangeMapReg3[rmWidth*rmWidth];
  double rangeMapReg4[rmWidth*rmWidth];

  constexpr static int pfmWidth = 70;
  double pickFixMap[pfmWidth];
  eePose pfmAnchorPose;

  // grasp Thompson parameters
  double graspMemoryTries[4*rmWidth*rmWidth];
  double graspMemoryPicks[4*rmWidth*rmWidth];
  double graspMemorySample[4*rmWidth*rmWidth];
  double graspMemoryReg1[4*rmWidth*rmWidth];

  pickMode currentPickMode = STATIC_MARGINALS;
  pickMode currentBoundingBoxMode = STATIC_MARGINALS;
  

  const static int hrmWidth = 211; // must be odd
  const static int hrmHalfWidth = (hrmWidth-1)/2; // must be odd
  constexpr static double hrmDelta = 0.001;
  double hiRangeMap[hrmWidth*hrmWidth];
  double hiRangeMapAccumulator[hrmWidth*hrmWidth];
  double hiRangeMapMass[hrmWidth*hrmWidth];
  
  double hiColorRangeMapAccumulator[3*hrmWidth*hrmWidth];
  double hiColorRangeMapMass[hrmWidth*hrmWidth];
  
  double hiRangeMapReg1[hrmWidth*hrmWidth];
  double hiRangeMapReg2[hrmWidth*hrmWidth];


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
  const static int vmWidth = hrmWidth;
  const static int vmHalfWidth = hrmHalfWidth;
  const double vmDelta = hrmDelta;
  double volumeMap[vmWidth*vmWidth*vmWidth];
  double volumeMapAccumulator[vmWidth*vmWidth*vmWidth];
  double volumeMapMass[vmWidth*vmWidth*vmWidth];
  
  double vmColorRangeMapAccumulator[3*vmWidth*vmWidth*vmWidth];
  double vmColorRangeMapMass[vmWidth*vmWidth*vmWidth];
  
  const static int parzen3DKernelHalfWidth = 9;
  const static int parzen3DKernelWidth = 2*parzen3DKernelHalfWidth+1;
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
  
  
  int hrmiHeight = hrmWidth;
  int hrmiWidth = hrmWidth;
  
  const static int hmWidth = 4; 
  int hmiCellWidth = 100;
  int hmiWidth = hmiCellWidth;
  int hmiHeight = hmiCellWidth*hmWidth;



  // height Thompson parameters
  constexpr static double minHeight = 0.255;//0.09;//-0.10;
  constexpr static double maxHeight = 0.655;//0.49;//0.3;
  double heightMemoryTries[hmWidth];
  double heightMemoryPicks[hmWidth];
  double heightMemorySample[hmWidth];
  
  double heightAttemptCounter = 0;
  double heightSuccessCounter = 0;
  double thompsonTries = 50;


  int heightLearningServoTimeout = 10;
  double currentThompsonHeight = 0;
  int currentThompsonHeightIdx = 0;

  int bbLearningMaxTries = 15;
  int graspLearningMaxTries = 10;
  
  int thompsonHardCutoff = 0;
  int thompsonMinTryCutoff = 5;
  double thompsonMinPassRate = 0.80;
  int thompsonAdaptiveCutoff = 1;
  int thompsonPickHaltFlag = 0;
  int thompsonHeightHaltFlag = 0;


  double pickEccentricity = 100.0;
  double heightEccentricity = 1.0;


  // algorithmC accecpt and reject thresholds
  double algorithmCEPS = 0.2;
  double algorithmCTarget = 0.7;
  double algorithmCAT = 0.7;
  double algorithmCRT = 0.95;



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
  Eigen::Quaternionf gear0offset;
  // XXX maybe we should initialize this to a reasonable value
  //// reticles
  double ggX[totalGraspGears];
  double ggY[totalGraspGears];
  double ggT[totalGraspGears];

  int castRecentRangeRay = 1;
  int recordRangeMap = 0;

  Quaternionf irGlobalPositionEEFrame;
 
  constexpr static double cReticleIndexDelta = .01;
  const static int numCReticleIndeces = 14;
  constexpr static double firstCReticleIndexDepth = .08;
  int xCR[numCReticleIndeces];
  int yCR[numCReticleIndeces];
  double fEpsilon = 1.0e-9;

  int curseReticleX = 0;
  int curseReticleY = 0;


  
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


  ros::Time oscilStart;
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

  ros::Time lastVisionCycle;
  ros::Duration accumulatedTime;

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

  // XXX TODO
  int softMaxGradientServoIterations = 2;//5;//3;//10;//3;
  int hardMaxGradientServoIterations = 5;//10;//2;//5;//5;//3;//10;//20;//3;//10;
  int currentGradientServoIterations = 0;


  int gripperMoving = 0;
  double gripperPosition = 0;
  int gripperGripping = 0;
  double gripperThresh = 3.5;//6.0;//7.0;
  ros::Time gripperLastUpdated;
  double gripperNotMovingConfirmTime = 0.25;
  // the last value the gripper was at when it began to open from a closed position
  double lastMeasuredClosed = 3.0;

  ros::Time graspTrialStart;
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
  double pickFlushFactor = 0.108;//0.08;//0.09;//0.11;


  int useContinuousGraspTransform = 1;


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
  //const int mappingHeightIdx = 0;
  const int mappingHeightIdx = 1;


  const static int vaNumAngles = 360;
  constexpr static double vaDelta = (2.0 * 3.1415926) / vaNumAngles;
  double vaX[vaNumAngles];
  double vaY[vaNumAngles];


  int waitUntilAtCurrentPositionCounter = 0;
  int waitUntilAtCurrentPositionCounterTimeout = 300;
  int waitUntilEffortCounter = 0;
  int waitUntilEffortCounterTimeout = 3000;
  int pressUntilEffortCounter = 0;
  int pressUntilEffortCounterTimeout = 200;
  int waitUntilGripperNotMovingCounter = 0;
  int waitUntilGripperNotMovingTimeout = 100;
  ros::Time waitUntilGripperNotMovingStamp;

  double currentEESpeedRatio = 0.5;

  int endCollapse = 0;
  int endThisStackCollapse = 0;




  int heartBeatCounter = 0;
  int heartBeatPeriod = 150;


  ros::Time lastAccelerometerCallbackRequest;
  ros::Time lastImageCallbackRequest;
  ros::Time lastGripperCallbackRequest;
  ros::Time lastEndpointCallbackRequest;
  
  ros::Time lastAccelerometerCallbackReceived;
  ros::Time lastImageCallbackReceived;
  ros::Time lastGripperCallbackReceived;
  ros::Time lastEndpointCallbackReceived;

  ros::Time lastImageStamp;
  ros::Time lastImageFromDensityReceived;

  bool usePotentiallyCollidingIK = 0;

  Mat objectViewerYCbCrBlur;
  Mat objectViewerGrayBlur;


  ros::Time lastHoverRequest;
  double hoverTimeout = 3.0;//2.0; // seconds
  double hoverGoThresh = 0.02;
  double hoverAngleThresh = 0.02;
  eePose lastHoverTrueEEPoseEEPose;

  ros::Time lastMovementStateSet;
  eePose lastTrueEEPoseEEPose;
  
  ros::Time comeToHoverStart;
  double comeToHoverTimeout = 3.0;
  ros::Time comeToStopStart;
  double comeToStopTimeout = 30.0;
  ros::Time waitForTugStart;
  double waitForTugTimeout = 1e10;
  double armedThreshold = 0.05;



  double simulatorCallbackFrequency = 30.0;
  ros::Timer simulatorCallbackTimer;

  ros::Timer timer1;
  
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


  Mat gripperMaskFirstContrast;
  Mat gripperMaskSecondContrast;
  Mat gripperMaskMean;
  Mat gripperMaskSquares;
  int gripperMaskCounts;
  Mat gripperMask;
  Mat cumulativeGripperMask;
  double gripperMaskThresh = 0.02;

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


  cv_bridge::CvImagePtr cv_ptr = NULL;
  Mat stereoViewerImage;
  Mat objectViewerImage;
  Mat objectMapViewerImage;
  Mat densityViewerImage;
  Mat wristViewImage;
  Mat gradientViewerImage;
  Mat aerialGradientViewerImage;
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



  std::string data_directory = "unspecified_dd";
  std::string vocab_file = "unspecified_vf";
  std::string knn_file = "unspecified_kf";
  std::string label_file = "unspecified_lf";
  std::string config_directory = "/config/";
  
  std::string run_prefix = "";
  std::string scan_group = "";
  //std::string class_name = "unspecified_cn";
  //std::string class_labels= "unspecified_cl1 unspecified_cl2";
  //std::string class_pose_models = "unspecified_pm1 unspecified_pm2";
  
  std::string image_topic = "/camera/rgb/image_raw"; 
  

  std::string cache_prefix = "";


  int numClasses = 0;

  vector<string> classLabels; 
  vector<string> classPoseModels;
  vector<CvKNearest*> classPosekNNs;
  vector<Mat> classPosekNNfeatures;
  vector<Mat> classPosekNNlabels;
  vector< vector< cv::Vec<double,4> > > classQuaternions;


  DescriptorMatcher *matcher = NULL;
  FeatureDetector *detector = NULL;
  DescriptorExtractor *extractor = NULL;
  BOWKMeansTrainer *bowTrainer = NULL; 
  BOWImgDescriptorExtractor *bowExtractor = NULL;
  CvKNearest *kNN = NULL;


  std::string class_crops_path;

  cv::Mat cam_img;

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
  ros::Time lastScanStarted;
  int mapFreeSpacePixelSkirt = 25;
  int mapBlueBoxPixelSkirt = 50;
  double mapBlueBoxCooldown = 180; // cooldown is a temporal skirt
  int mapGrayBoxPixelSkirtRows = 60;
  int mapGrayBoxPixelSkirtCols = 110;
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
  const static int numIkMapHeights = 3;
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

  image_transport::Subscriber image_sub;
  ros::Subscriber eeRanger;
  ros::Subscriber epState;
  ros::Subscriber gravity_comp_sub;
  ros::Subscriber cuff_grasp_sub;
  ros::Subscriber cuff_ok_sub;
  ros::Subscriber shoulder_sub;



  shared_ptr<image_transport::ImageTransport> it;

  ros::Subscriber collisionDetectionState;
  ros::Subscriber gripState;
  ros::Subscriber eeAccelerator;
  ros::Subscriber eeTarget;
  ros::Subscriber jointSubscriber;

  ros::Subscriber pickObjectUnderEndEffectorCommandCallbackSub;
  ros::Subscriber placeObjectInEndEffectorCommandCallbackSub;
  ros::Subscriber moveEndEffectorCommandCallbackSub;
  ros::Subscriber einSub;

  ros::Subscriber armItbCallbackSub;
  ros::Subscriber forthCommandSubscriber;
  ros::Publisher forthCommandPublisher;

  ros::Time waitForSecondsTarget;
  ros::Time spinForSecondsTarget;

  ros::Time measureTimeTarget;
  ros::Time measureTimeStart;
  double measureTimePeriod = 1.0;

  baxter_core_msgs::ITBState lastItbs;

  eePose pressPose;
  double twistThresh = 0.01;

  double rockDiffA = 0.0;
  double rockDiffB = 0.0;

  eePose targetWrench;
  double wrenchThresh = 15.0;

  int intendedEnableState = 1;
  int lastShoulderState = 1;
}; // config end

class Word;


/**
 * MachineState represents the call stack for einl.  It consists of
 * the call stack, the variables, the current instruction, and whether
 * the stack is currently executing.
 */
class MachineState: public std::enable_shared_from_this<MachineState> {
 private:
 public:
  std::shared_ptr<MachineState> sharedThis;

  std::vector<std::shared_ptr<Word> > call_stack;
  std::vector<std::shared_ptr<Word> > data_stack;
  std::vector<std::shared_ptr<Word> > control_stack;

  std::map<string, std::shared_ptr<Word> > variables;

  std::shared_ptr<Word> current_instruction = NULL;
  EinConfig config;

  int execute_stack = 0;

  executionMode execution_mode = INSTANT;

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

  void jointCallback(const sensor_msgs::JointState& js);
  void moveEndEffectorCommandCallback(const geometry_msgs::Pose& msg);
  void armItbCallback(const baxter_core_msgs::ITBState& itbs);
  void pickObjectUnderEndEffectorCommandCallback(const std_msgs::Empty& msg);
  void placeObjectInEndEffectorCommandCallback(const std_msgs::Empty& msg);
  void forthCommandCallback(const std_msgs::String::ConstPtr& msg);
  void evaluateProgram(const string program);
  void endpointCallback(const baxter_core_msgs::EndpointState& eps);
  void collisionDetectionStateCallback(const baxter_core_msgs::CollisionDetectionState& cds);
  void gripStateCallback(const baxter_core_msgs::EndEffectorState& ees);
  void accelerometerCallback(const sensor_msgs::Imu& moment);
  void rangeCallback(const sensor_msgs::Range& range);
  void update_baxter(ros::NodeHandle &n);
  void timercallback1(const ros::TimerEvent&);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void gravityCompCallback(const baxter_core_msgs::SEAJointState& seaJ) ;
  void cuffGraspCallback(const baxter_core_msgs::DigitalIOState& cuffDIOS) ;
  void cuffOkCallback(const baxter_core_msgs::DigitalIOState& cuffDIOS) ;
  void shoulderCallback(const baxter_core_msgs::DigitalIOState& shoulderDIOS) ;
  void targetCallback(const geometry_msgs::Point& point);
  void simulatorCallback(const ros::TimerEvent&);
  void einStateCallback(const ein::EinState & msg);
};


#endif /* _CONFIG_H_ */
