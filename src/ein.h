#ifndef _EIN_H_
#define _EIN_H_


//#define DEBUG_RING_BUFFER // ring buffer

#define EPSILON 1.0e-9
#define VERYBIGNUMBER 1e12

#define PROGRAM_NAME "ein"


#include <ein/EinState.h>

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <ctime>

#include <math.h>
#include <dirent.h>
#include <signal.h>
#include <sys/stat.h>

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
#include <std_msgs/Float32.h>
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
#include <baxter_core_msgs/DigitalOutputCommand.h>


#include <cv.h>
#include <highgui.h>
#include <ml.h>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/gpu/gpu.hpp>


// slu
#include "slu/math2d.h"

// numpy library 1 (randomkit, for original beta)
#include "distributions.h"
#include "eePose.h"
#include "eigen_util.h"
#include "ein_util.h"
#include "ein_ik.h"
//#include "faces.h"

using namespace std;
using namespace cv;

using namespace ein;

extern MachineState machineState;
extern shared_ptr<MachineState> pMachineState;



int ARE_GENERIC_PICK_LEARNING(shared_ptr<MachineState> ms);
int ARE_GENERIC_HEIGHT_LEARNING(shared_ptr<MachineState> ms);


int getColorReticleX(shared_ptr<MachineState> ms);
int getColorReticleY(shared_ptr<MachineState> ms);


void mapijToxy(shared_ptr<MachineState> ms, int i, int j, double * x, double * y);
void mapxyToij(shared_ptr<MachineState> ms, double x, double y, int * i, int * j); 
void voidMapRegion(shared_ptr<MachineState> ms, double xc, double yc);
void clearMapForPatrol(shared_ptr<MachineState> ms);
void initializeMap(shared_ptr<MachineState> ms);
void randomizeNanos(shared_ptr<MachineState> ms, ros::Time * time);
int blueBoxForPixel(int px, int py);
int skirtedBlueBoxForPixel(shared_ptr<MachineState> ms, int px, int py, int skirtPixels);
bool cellIsSearched(shared_ptr<MachineState> ms, int i, int j);
bool positionIsSearched(shared_ptr<MachineState> ms, double x, double y);
void markMapAsCompleted(shared_ptr<MachineState> ms);

vector<BoxMemory> memoriesForClass(shared_ptr<MachineState> ms, int classIdx);
vector<BoxMemory> memoriesForClass(shared_ptr<MachineState> ms, int classIdx, int * memoryIdxOfFirst);
int getBoxMemoryOfLabel(std::shared_ptr<MachineState> ms, string label, int * idxOfLabel, BoxMemory * out);
int placementPoseLabel1AboveLabel2By3dFirst(std::shared_ptr<MachineState> ms, string label1, string label2, double zAbove, eePose * out);
int placementPoseLabel1AboveLabel2By(std::shared_ptr<MachineState> ms, string label1, string label2, double zAbove, eePose * out);
int placementPoseHeldAboveLabel2By(std::shared_ptr<MachineState> ms, string label2, double zAbove, eePose * out);
int placementPoseLabel1BetweenLabel2AndLabel3(std::shared_ptr<MachineState> ms, string label1, 
  string label2, string label3, eePose * out);
void recordBlueBoxInHistogram(shared_ptr<MachineState> ms, int idx);
void computeClassificationDistributionFromHistogram(shared_ptr<MachineState> ms);

// XXX TODO searched and mapped are redundant. just need one to talk about the fence.
bool cellIsMapped(int i, int j);
bool positionIsMapped(shared_ptr<MachineState> ms, double x, double y);
bool boxMemoryIntersectPolygons(BoxMemory b1, BoxMemory b2);
bool boxMemoryIntersectCentroid(BoxMemory b1, BoxMemory b2);
bool boxMemoryContains(BoxMemory b, double x, double y);
bool boxMemoryIntersectsMapCell(shared_ptr<MachineState> ms, BoxMemory b, int map_i, int map_j);
const ros::Duration mapMemoryTimeout(10);

// XXX TODO these just check the corners, they should check all the interior points instead
bool isBoxMemoryIkPossible(shared_ptr<MachineState> ms, BoxMemory b);
bool isBlueBoxIkPossible(shared_ptr<MachineState> ms, cv::Point tbTop, cv::Point tbBot);

bool isCellInPursuitZone(shared_ptr<MachineState> ms, int i, int j);
bool isCellInPatrolZone(shared_ptr<MachineState> ms, int i, int j);

bool isCellInteresting(shared_ptr<MachineState> ms, int i, int j);
void markCellAsInteresting(shared_ptr<MachineState> ms, int i, int j);
void markCellAsNotInteresting(shared_ptr<MachineState> ms, int i, int j);

bool isCellIkColliding(shared_ptr<MachineState> ms, int i, int j);
bool isCellIkPossible(shared_ptr<MachineState> ms, int i, int j);
bool isCellIkImpossible(shared_ptr<MachineState> ms, int i, int j);


//
// start pilot prototypes 
////////////////////////////////////////////////

int getRingImageAtTime(shared_ptr<MachineState> ms, ros::Time t, Mat& value, int drawSlack = 0);
int getRingRangeAtTime(shared_ptr<MachineState> ms, ros::Time t, double &value, int drawSlack = 0);
int getRingPoseAtTime(shared_ptr<MachineState> ms, ros::Time t, geometry_msgs::Pose &value, int drawSlack = 0);
extern "C" {
double cephes_incbet(double a, double b, double x) ;
}
void setRingImageAtTime(shared_ptr<MachineState> ms, ros::Time t, Mat& imToSet);
void setRingRangeAtTime(shared_ptr<MachineState> ms, ros::Time t, double rgToSet);
void setRingPoseAtTime(shared_ptr<MachineState> ms, ros::Time t, geometry_msgs::Pose epToSet);
void imRingBufferAdvance(shared_ptr<MachineState> ms);
void rgRingBufferAdvance(shared_ptr<MachineState> ms);
void epRingBufferAdvance(shared_ptr<MachineState> ms);
void allRingBuffersAdvance(shared_ptr<MachineState> ms, ros::Time t);

void recordReadyRangeReadings(shared_ptr<MachineState> ms);
void doEndpointCallback(shared_ptr<MachineState> ms, const baxter_core_msgs::EndpointState& eps);
int classIdxForName(shared_ptr<MachineState> ms, string name);

void initClassFolders(std::shared_ptr<MachineState> ms, string folderName);
void writeClassToFolder(std::shared_ptr<MachineState> ms, int idx, string folderName);
void writeAerialGradientsToServoCrop(std::shared_ptr<MachineState> ms, int idx, string servoCrop_file_path);
void writeThumbnail(std::shared_ptr<MachineState> ms, int idx, string servoCrop_file_path);
void writeIr2D(std::shared_ptr<MachineState> ms, int idx, string this_range_path);
void write3dGrasps(std::shared_ptr<MachineState> ms, int idx, string this_grasp_path);
void writeGraspMemory(std::shared_ptr<MachineState> ms, int idx, string this_grasp_path);
void writeSceneModel(std::shared_ptr<MachineState> ms, int idx, string this_grasp_path);

void saveAccumulatedStreamToPath(shared_ptr<MachineState> ms, string path);
streamImage * setIsbIdx(std::shared_ptr<MachineState> ms, int idx);
streamImage * setIsbIdxNoLoad(std::shared_ptr<MachineState> ms, int idx);
void resetAccumulatedStreamImage(std::shared_ptr<MachineState> ms);
int getStreamPoseAtTime(std::shared_ptr<MachineState> ms, double tin, eePose * outArm, eePose * outBase);
void castRangeRay(std::shared_ptr<MachineState> ms, double thisRange, eePose thisPose, Vector3d * castPointOut, Vector3d * rayDirectionOut);
void update2dRangeMaps(std::shared_ptr<MachineState> ms, Vector3d castPoint);

bool streamRangeComparator(streamRange i, streamRange j);
bool streamPoseComparator(streamEePose i, streamEePose j);
bool streamImageComparator(streamImage i, streamImage j);
bool streamJointsComparator(streamJoints i, streamJoints j);
bool streamWordComparator(streamWord i, streamWord j);
bool streamLabelComparator(streamLabel i, streamLabel j);

void activateSensorStreaming(std::shared_ptr<MachineState> ms);
void deactivateSensorStreaming(std::shared_ptr<MachineState> ms);

int didSensorStreamTimeout(std::shared_ptr<MachineState> ms);

void populateStreamImageBuffer(std::shared_ptr<MachineState> ms);
void populateStreamPoseBuffer(std::shared_ptr<MachineState> ms);
void populateStreamRangeBuffer(std::shared_ptr<MachineState> ms);
void populateStreamWordBuffer(std::shared_ptr<MachineState> ms);
void populateStreamLabelBuffer(std::shared_ptr<MachineState> ms);

void streamImageAsClass(std::shared_ptr<MachineState> ms, Mat im, int classToStreamIdx, double now);
void streamRangeAsClass(std::shared_ptr<MachineState> ms, double range, int classToStreamIdx, double now);
void streamPoseAsClass(std::shared_ptr<MachineState> ms, eePose poseIn, int classToStreamIdx, double now);
void streamWordAsClass(std::shared_ptr<MachineState> ms, string wordIn, string commandIn, int classToStreamIdx, double now);
void streamLabelAsClass(std::shared_ptr<MachineState> ms, string labelIn, int classToStreamIdx, double now);

void writeRangeBatchAsClass(std::shared_ptr<MachineState> ms, int classToStreamIdx);
void writePoseBatchAsClass(std::shared_ptr<MachineState> ms, int classToStreamIdx);
void writeWordBatchAsClass(std::shared_ptr<MachineState> ms, int classToStreamIdx);
void writeLabelBatchAsClass(std::shared_ptr<MachineState> ms, int classToStreamIdx);

void checkAndStreamWord(std::shared_ptr<MachineState> ms, string wordIn, string commandIn);

void writeSideAndSerialToFileStorage(FileStorage& fsvO);
void readSideAndSerialFromFileStorage(std::shared_ptr<MachineState> ms, FileStorage fsvI, string * serial, string * side);
string appendSideAndSerial(std::shared_ptr<MachineState> ms, string root);

void populateStreamJointsBuffer(std::shared_ptr<MachineState> ms);
void streamJointsAsClass(std::shared_ptr<MachineState> ms, int classToStreamIdx, double now);
void writeJointsBatchAsClass(std::shared_ptr<MachineState> ms, int classToStreamIdx);



bool isInGripperMask(shared_ptr<MachineState> ms, int x, int y);
bool isInGripperMaskBlocks(shared_ptr<MachineState> ms, int x, int y);
bool isGripperGripping(shared_ptr<MachineState> ms);
void initialize3DParzen(shared_ptr<MachineState> ms);
void l2Normalize3DParzen(shared_ptr<MachineState> ms);
void initializeParzen(shared_ptr<MachineState> ms);
void l2NormalizeParzen(shared_ptr<MachineState> ms);
void l2NormalizeFilter(shared_ptr<MachineState> ms);


cv::Vec3b getCRColor(shared_ptr<MachineState> ms);
cv::Vec3b getCRColor(shared_ptr<MachineState> ms, Mat im);
Quaternionf extractQuatFromPose(geometry_msgs::Pose poseIn);



void scanXdirection(shared_ptr<MachineState> ms, double speedOnLines, double speedBetweenLines);
void scanYdirection(shared_ptr<MachineState> ms, double speedOnLines, double speedBetweenLines);

Eigen::Quaternionf getGGRotation(shared_ptr<MachineState> ms, int givenGraspGear);
void setGGRotation(shared_ptr<MachineState> ms, int thisGraspGear);

Eigen::Quaternionf getCCRotation(shared_ptr<MachineState> ms, int givenGraspGear, double angle);
void setCCRotation(shared_ptr<MachineState> ms, int thisGraspGear);
void publishVolumetricMap(shared_ptr<MachineState> ms);

void endEffectorAngularUpdate(eePose *givenEEPose, eePose *deltaEEPose);
void endEffectorAngularUpdateOuter(eePose *givenEEPose, eePose *deltaEEPose);



void renderRangeogramView(shared_ptr<MachineState> ms);
void renderObjectMapView(shared_ptr<MachineState> leftArm, shared_ptr<MachineState> rightArm);
void renderObjectMapViewOneArm(shared_ptr<MachineState> ms);
void objectMapCallbackFunc(int event, int x, int y, int flags, void* userdata);
void doObjectMapCallbackFunc(int event, int x, int y, int flags, shared_ptr<MachineState> ms);


void renderAccumulatedImageAndDensity(shared_ptr<MachineState> ms);
void drawMapPolygon(Mat mapImage, double mapXMin, double mapXMax, double mapYMin, double mapYMax, gsl_matrix * poly, cv::Scalar color);
gsl_matrix * mapCellToPolygon(shared_ptr<MachineState> ms, int map_i, int map_j) ;

void pilotInit(shared_ptr<MachineState> ms);
void spinlessPilotMain(shared_ptr<MachineState> ms);

int doCalibrateGripper(shared_ptr<MachineState> ms);
int calibrateGripper(shared_ptr<MachineState> ms);
int shouldIPick(shared_ptr<MachineState> ms, int classToPick);
int getLocalGraspGear(shared_ptr<MachineState> ms, int globalGraspGearIn);
int getGlobalGraspGear(shared_ptr<MachineState> ms, int localGraspGearIn);
void convertGlobalGraspIdxToLocal(shared_ptr<MachineState> ms, const int rx, const int ry, 
                                  int * localX, int * localY);

void convertLocalGraspIdxToGlobal(shared_ptr<MachineState> ms, const int localX, const int localY,
                                  int * rx, int * ry);

void changeTargetClass(shared_ptr<MachineState> ms, int);

void zeroGraspMemoryAndRangeMap(shared_ptr<MachineState> ms);
void zeroClassGraspMemory(shared_ptr<MachineState> ms);
void guard3dGrasps(shared_ptr<MachineState> ms);
void guardSceneModels(shared_ptr<MachineState> ms);
void guardGraspMemory(shared_ptr<MachineState> ms);
void loadSampledGraspMemory(shared_ptr<MachineState> ms);
void loadMarginalGraspMemory(shared_ptr<MachineState> ms);
void loadPriorGraspMemory(shared_ptr<MachineState> ms, priorType);
void estimateGlobalGraspGear();
void drawMapRegisters(shared_ptr<MachineState> ms);


void guardHeightMemory(shared_ptr<MachineState> ms);
void loadSampledHeightMemory(shared_ptr<MachineState> ms);
void loadMarginalHeightMemory(shared_ptr<MachineState> ms);
void loadPriorHeightMemory(shared_ptr<MachineState> ms, priorType);
double convertHeightIdxToGlobalZ(shared_ptr<MachineState> ms, int);
double convertHeightIdxToLocalZ(shared_ptr<MachineState> ms, int);
int convertHeightGlobalZToIdx(shared_ptr<MachineState> ms, double);
void testHeightConversion(shared_ptr<MachineState> ms);
void drawHeightMemorySample(shared_ptr<MachineState> ms);
void copyHeightMemoryTriesToClassHeightMemoryTries(shared_ptr<MachineState> ms);

void applyGraspFilter(shared_ptr<MachineState> ms, double * rangeMapRegA, double * rangeMapRegB);
void prepareGraspFilter(shared_ptr<MachineState> ms, int i);
void prepareGraspFilter1(shared_ptr<MachineState> ms);
void prepareGraspFilter2(shared_ptr<MachineState> ms);
void prepareGraspFilter3(shared_ptr<MachineState> ms);
void prepareGraspFilter4(shared_ptr<MachineState> ms);

void copyRangeMapRegister(shared_ptr<MachineState> ms, double * src, double * target);
void copyGraspMemoryRegister(shared_ptr<MachineState> ms, double * src, double * target);
void loadGlobalTargetClassRangeMap(shared_ptr<MachineState> ms, double * rangeMapRegA, double * rangeMapRegB);
void loadLocalTargetClassRangeMap(shared_ptr<MachineState> ms, double * rangeMapRegA, double * rangeMapRegB);
void copyGraspMemoryTriesToClassGraspMemoryTries(shared_ptr<MachineState> ms);
void copyClassGraspMemoryTriesToGraspMemoryTries(shared_ptr<MachineState> ms);

void selectMaxTarget(shared_ptr<MachineState> ms, double minDepth);
void selectMaxTargetThompsonContinuous2(shared_ptr<MachineState> ms, double minDepth);

void recordBoundingBoxSuccess(shared_ptr<MachineState> ms);
void recordBoundingBoxFailure(shared_ptr<MachineState> ms);

void restartBBLearning(shared_ptr<MachineState> ms);

eePose analyticServoPixelToReticle(shared_ptr<MachineState> ms, eePose givenPixel, eePose givenReticle, double angle, eePose givenCameraPose);
void moveCurrentGripperRayToCameraVanishingRay(shared_ptr<MachineState> ms);
Mat makeGCrop(shared_ptr<MachineState> ms, int etaX, int etaY);
void pixelServo(shared_ptr<MachineState> ms, int servoDeltaX, int servoDeltaY, double servoDeltaTheta);
void gradientServo(shared_ptr<MachineState> ms);
void gradientServoLatentClass(shared_ptr<MachineState> ms);
void continuousServo(shared_ptr<MachineState> ms);
void synchronicServo(shared_ptr<MachineState> ms);
void darkServo(shared_ptr<MachineState> ms);
void faceServo(shared_ptr<MachineState> ms, vector<Rect> faces);
int simulatedServo();

void initRangeMaps(shared_ptr<MachineState> ms);

int isThisGraspMaxedOut(shared_ptr<MachineState> ms, int i);

void pixelToGlobal(shared_ptr<MachineState> ms, int pX, int pY, double gZ, double * gX, double * gY);
void pixelToGlobal(shared_ptr<MachineState> ms, int pX, int pY, double gZ, double * gX, double * gY, eePose givenEEPose);
void pixelToGlobalFromCache(shared_ptr<MachineState> ms, int pX, int pY, double gZ, double * gX, double * gY, eePose givenEEPose, pixelToGlobalCache * cache);
void computePixelToGlobalCache(shared_ptr<MachineState> ms, double gZ, eePose givenEEPose, pixelToGlobalCache * cache);
void globalToPixel(shared_ptr<MachineState> ms, int * pX, int * pY, double gZ, double gX, double gY);
void globalToPixelPrint(shared_ptr<MachineState> ms, int * pX, int * pY, double gZ, double gX, double gY);
eePose pixelToGlobalEEPose(shared_ptr<MachineState> ms, int pX, int pY, double gZ);

void pixelToWorld(Mat mapImage, double xMin, double xMax, double yMin, double yMax, int px, int py, double &x, double &y) ;
cv::Point worldToPixel(Mat mapImage, double xMin, double xMax, double yMin, double yMax, double x, double y);

void paintEEPoseOnWrist(shared_ptr<MachineState> ms, eePose toPaint, cv::Scalar theColor);

double vectorArcTan(shared_ptr<MachineState> ms, double y, double x);
void initVectorArcTan(shared_ptr<MachineState> ms);

void mapBlueBox(shared_ptr<MachineState> ms, cv::Point tbTop, cv::Point tbBot, int detectedClass, ros::Time timeToMark);
void mapBox(shared_ptr<MachineState> ms, BoxMemory boxMemory);

void queryIK(shared_ptr<MachineState> ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest);

void globalToMapBackground(shared_ptr<MachineState> ms, double gX, double gY, double zToUse, int * mapGpPx, int * mapGpPy);

void loadCalibration(shared_ptr<MachineState> ms, string inFileName);
void saveCalibration(shared_ptr<MachineState> ms, string outFileName);

void findDarkness(shared_ptr<MachineState> ms, int * xout, int * yout);
void findLight(shared_ptr<MachineState> ms, int * xout, int * yout);
void findOptimum(shared_ptr<MachineState> ms, int * xout, int * yout, int sign);

void fillLocalUnitBasis(eePose localFrame, Vector3d * localUnitX, Vector3d * localUnitY, Vector3d * localUnitZ);

////////////////////////////////////////////////
// end pilot prototypes 
//
// start node prototypes
////////////////////////////////////////////////

int doubleToByte(double in);



void gridKeypoints(shared_ptr<MachineState> ms, int gImW, int gImH, cv::Point top, cv::Point bot, int strideX, int strideY, vector<KeyPoint>& keypoints, int period);

bool isFiniteNumber(double x);

void appendColorHist(Mat& yCrCb_image, vector<KeyPoint>& keypoints, Mat& descriptors, Mat& descriptors2);
void processImage(Mat &image, Mat& gray_image, Mat& yCrCb_image, double sigma);

void bowGetFeatures(shared_ptr<MachineState> ms, std::string classDir, const char *className, double sigma, int keypointPeriod, int * grandTotalDescriptors, DescriptorExtractor * extractor, BOWKMeansTrainer * bowTrainer);
void kNNGetFeatures(shared_ptr<MachineState> ms, std::string classDir, const char *className, int label, double sigma, Mat &kNNfeatures, Mat &kNNlabels, double sobel_sigma);
void posekNNGetFeatures(shared_ptr<MachineState> ms, std::string classDir, const char *className, double sigma, Mat &kNNfeatures, Mat &kNNlabels,
                        vector< cv::Vec<double,4> >& classQuaternions, int keypointPeriod, BOWImgDescriptorExtractor *bowExtractor, int lIndexStart = 0);


void drawDensity(shared_ptr<MachineState> ms, double scale);
void goCalculateDensity(shared_ptr<MachineState> ms);
void goFindBlueBoxes(shared_ptr<MachineState> ms);
void goClassifyBlueBoxes(shared_ptr<MachineState> ms);
void goFindRedBoxes();

void resetAccumulatedImageAndMass(shared_ptr<MachineState> ms);
void substituteStreamAccumulatedImageQuantities(shared_ptr<MachineState> ms);
void substituteStreamImageQuantities(shared_ptr<MachineState> ms);

void substituteAccumulatedImageQuantities(shared_ptr<MachineState> ms);
void substituteLatestImageQuantities(shared_ptr<MachineState> ms);

void loadROSParamsFromArgs(shared_ptr<MachineState> ms);
void saveROSParams(shared_ptr<MachineState> ms);

void spinlessNodeMain(shared_ptr<MachineState> ms);
void nodeInit(shared_ptr<MachineState> ms);
void detectorsInit(shared_ptr<MachineState> ms);
void initRedBoxes();

void tryToLoadRangeMap(shared_ptr<MachineState> ms, std::string classDir, const char *className, int i);
void clearAllRangeMaps(shared_ptr<MachineState> ms);

void processSaliency(Mat in, Mat out);

void happy(shared_ptr<MachineState> ms);
void sad(shared_ptr<MachineState> ms);
void neutral(shared_ptr<MachineState> ms);


void guardViewers(shared_ptr<MachineState> ms);

int findClosestBlueBoxMemory(shared_ptr<MachineState> ms, eePose targetPose, int classToSearch = -1);
void fillRecognizedObjectArrayFromBlueBoxMemory(shared_ptr<MachineState> ms, object_recognition_msgs::RecognizedObjectArray * roa);
void promoteBlueBoxes(shared_ptr<MachineState> ms);
void fillEinStateMsg(shared_ptr<MachineState> ms, EinState * stateOut);
void targetBoxMemory(shared_ptr<MachineState> ms, int idx);

bool isFocusedClassValid(std::shared_ptr<MachineState> ms);

double computeSimilarity(std::shared_ptr<MachineState> ms, int class1, int class2);
double computeSimilarity(std::shared_ptr<MachineState> ms, Mat im1, Mat im2);

void prepareForCrossCorrelation(std::shared_ptr<MachineState> ms, Mat input, Mat& output, int thisOrient, int numOrientations, double thisScale, Size toBecome);
void normalizeForCrossCorrelation(std::shared_ptr<MachineState> ms, Mat input, Mat& output);
void pilotCallbackFunc(int event, int x, int y, int flags, void* userdata);

////////////////////////////////////////////////
// end node prototypes 
#endif /* _EIN_H_ */
