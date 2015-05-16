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


  double rmDelta = 0.01;
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
