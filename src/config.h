#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "ein_util.h"
#include "eigen_util.h"

#include "distributions.h"


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
  WAREHOUSE = 1
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
  SIMULATED
} robotMode;

typedef enum {
  GRASP_CRANE,
  GRASP_3D
} graspMode;

#define NOW_THATS_FAST 0.08
#define MOVE_EVEN_FASTER 0.04
#define MOVE_FASTER 0.02
#define MOVE_FAST 0.01
#define MOVE_MEDIUM 0.005 //.005
#define MOVE_SLOW 0.0025
#define MOVE_VERY_SLOW 0.00125

#define NUM_JOINTS 7

class EinConfig {
 public:
  int zero_g_toggle = 0;
  int currentGraspGear = -1;
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


  movementState currentMovementState = STOPPED;
  patrolState currentPatrolState = IDLING;
  patrolMode currentPatrolMode = ONCE;
  placeMode currentPlaceMode = HAND;
  idleMode currentIdleMode = CRANE;
  graspMode currentGraspMode = GRASP_CRANE;
  robotMode currentRobotMode = PHYSICAL;

  Vector3d eeLinearAcceleration;

  // set color reticles iterator
  int scrI = 0;

  double currentGraspZ = 0;
  vector<double> classGraspZs;
  vector<double> classGraspZsSet;

  int current3dGraspIndex = 0;
  vector< vector<eePose> > class3dGrasps;
  eePose c3dPoseBase;

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

  // this should be initted to 0 and set to its default setting only after an imageCallback has happened.
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

  double eeRange = 0.0;


  double bDelta = MOVE_FAST;

  std::string wristViewName = "Wrist View";
  std::string coreViewName = "Core View";
  std::string rangeogramViewName = "Rangeogram View";
  std::string rangemapViewName = "Range Map View";
  std::string hiRangemapViewName = "Hi Range Map View";
  std::string hiColorRangemapViewName = "Hi Color Range Map View";
  std::string graspMemoryViewName = "Grasp Memory View";
  std::string graspMemorySampleViewName = "Grasp Memory Sample View";
  std::string mapBackgroundViewName = "Map Background Vew";
  std::string faceViewName = "Face View";
  std::string heightMemorySampleViewName = "Height Memory Sample View";

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


  geometry_msgs::Pose trueEEPose;
  eePose trueEEWrench;
  eePose trueEEPoseEEPose;
  std::string fetchCommand;
  ros::Time fetchCommandTime;
  double fetchCommandCooldown = 5;
  int acceptingFetchCommands = 0;

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
  double rangeMap[rmWidth*rmWidth];
  double rangeMapAccumulator[rmWidth*rmWidth];
  double rangeMapMass[rmWidth*rmWidth];
  
  double rangeMapReg1[rmWidth*rmWidth];
  double rangeMapReg2[rmWidth*rmWidth];
  double rangeMapReg3[rmWidth*rmWidth];
  double rangeMapReg4[rmWidth*rmWidth];

  // grasp Thompson parameters
  double graspMemoryTries[4*rmWidth*rmWidth];
  double graspMemoryPicks[4*rmWidth*rmWidth];
  double graspMemorySample[4*rmWidth*rmWidth];
  double graspMemoryReg1[4*rmWidth*rmWidth];
  
  

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
  
  // grasp gear should always be even
  static const int totalGraspGears = 8;
  // XXX maybe we should initialize this to a reasonable value
  //// reticles
  double ggX[totalGraspGears];
  double ggY[totalGraspGears];
  double ggT[totalGraspGears];

  int recordRangeMap = 1;

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
  int synServoPixelThresh = 15;//15;//10;
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
  int newClassCounter = 0;
  string focusedClassLabel;
  // config variables that don't seem to be used
};

class Word;

class MachineState: public std::enable_shared_from_this<MachineState> {
 private:
 public:
  std::vector<std::shared_ptr<Word> > call_stack;
  std::map<string, std::shared_ptr<Word> > variables;

  std::shared_ptr<Word> current_instruction = NULL;
  EinConfig config;

  int execute_stack = 0;
  bool pushWord(int code);
  bool pushWord(string name);
  bool pushWord(std::shared_ptr<Word> word);
  std::shared_ptr<Word> popWord();
  void clearStack();
  void pushNoOps(int n);
  void pushCopies(int symbol, int times);
  void pushCopies(string symbol, int times);

  void execute(std::shared_ptr<Word> word);
};


#endif /* _CONFIG_H_ */
