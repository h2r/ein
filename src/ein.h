#ifndef _EIN_H_
#define _EIN_H_


//#define DEBUG_RING_BUFFER // ring buffer

#define EPSILON 1.0e-9
#define VERYBIGNUMBER 1e12

#define PROGRAM_NAME "ein"

#include <ein/EinState.h>
#include <ein/EinConsole.h>

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
#include <baxter_core_msgs/AnalogOutputCommand.h>


#include <cv.h>
#include <highgui.h>
#include <ml.h>
/*#include <opencv2/nonfree/nonfree.hpp>*/
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





int ARE_GENERIC_PICK_LEARNING(MachineState * ms);
int ARE_GENERIC_HEIGHT_LEARNING(MachineState * ms);


int getColorReticleX(MachineState * ms);
int getColorReticleY(MachineState * ms);

void mapxyToij(double xmin, double ymin, double mapStep, double x, double y, int * i, int * j);
void mapijToxy(double xmin, double ymin, double mapStep, int i, int j, double * x, double * y); 
void voidMapRegion(MachineState * ms, double xc, double yc);
void clearMapForPatrol(MachineState * ms);
void initializeMap(MachineState * ms);
void randomizeNanos(MachineState * ms, ros::Time * time);
int blueBoxForPixel(int px, int py);
int skirtedBlueBoxForPixel(MachineState * ms, int px, int py, int skirtPixels);
bool cellIsSearched(double fenceXMin, double fenceXMax, double fenceYMin, double fenceYMax, double xmin, double ymin, double mapStep, int i, int j);
bool positionIsSearched(double fenceXMin, double fenceXMax, double fenceYMin, double fenceYMax, double x, double y);
void markMapAsCompleted(MachineState * ms);


vector<BoxMemory> memoriesForClass(MachineState * ms, int classIdx);
vector<BoxMemory> memoriesForClass(MachineState * ms, int classIdx, int * memoryIdxOfFirst);
int getBoxMemoryOfLabel(MachineState * ms, string label, int * idxOfLabel, BoxMemory * out);
int placementPoseLabel1AboveLabel2By3dFirst(MachineState * ms, string label1, string label2, double zAbove, eePose * out);
int placementPoseLabel1AboveLabel2By(MachineState * ms, string label1, string label2, double zAbove, eePose * out);
int placementPoseHeldAboveLabel2By(MachineState * ms, string label2, double zAbove, eePose * out);
int placementPoseLabel1BetweenLabel2AndLabel3(MachineState * ms, string label1, 
  string label2, string label3, eePose * out);
void recordBlueBoxInHistogram(MachineState * ms, int idx);
void computeClassificationDistributionFromHistogram(MachineState * ms);

// XXX TODO searched and mapped are redundant. just need one to talk about the fence.
bool cellIsMapped(int i, int j);
bool positionIsMapped(MachineState * ms, double x, double y);
bool boxMemoryIntersectPolygons(BoxMemory b1, BoxMemory b2);
bool boxMemoryIntersectCentroid(BoxMemory b1, BoxMemory b2);
bool boxMemoryContains(BoxMemory b, double x, double y);
bool boxMemoryIntersectsMapCell(MachineState * ms, BoxMemory b, int map_i, int map_j);
const ros::Duration mapMemoryTimeout(10);

// XXX TODO these just check the corners, they should check all the interior points instead
bool isBoxMemoryIkPossible(MachineState * ms, BoxMemory b);
bool isBlueBoxIkPossible(MachineState * ms, cv::Point tbTop, cv::Point tbBot);

bool isCellInPursuitZone(MachineState * ms, int i, int j);
bool isCellInPatrolZone(MachineState * ms, int i, int j);

bool isCellInteresting(MachineState * ms, int i, int j);
void markCellAsInteresting(MachineState * ms, int i, int j);
void markCellAsNotInteresting(MachineState * ms, int i, int j);

bool isCellIkColliding(MachineState * ms, int i, int j);
bool isCellIkPossible(MachineState * ms, int i, int j);
bool isCellIkImpossible(MachineState * ms, int i, int j);


//
// start pilot prototypes 
////////////////////////////////////////////////

int getMostRecentRingImageAndPose(MachineState * ms, Mat * image, eePose * pose, ros::Time * time, bool debug=false);
int getRingImageAtTime(MachineState * ms, ros::Time t, Mat& value, int drawSlack = 0, bool debug=false);
int getRingRangeAtTime(MachineState * ms, ros::Time t, double &value, int drawSlack = 0);
int getRingPoseAtTime(MachineState * ms, ros::Time t, geometry_msgs::Pose &value, int drawSlack = 0, bool debug=false);


extern "C" {
double cephes_incbet(double a, double b, double x) ;
}
void setRingImageAtTime(MachineState * ms, ros::Time t, Mat& imToSet);
void setRingRangeAtTime(MachineState * ms, ros::Time t, double rgToSet);
void setRingPoseAtTime(MachineState * ms, ros::Time t, geometry_msgs::Pose epToSet);
void imRingBufferAdvance(MachineState * ms);
void rgRingBufferAdvance(MachineState * ms);
void epRingBufferAdvance(MachineState * ms);
void allRingBuffersAdvance(MachineState * ms, ros::Time t);

void recordReadyRangeReadings(MachineState * ms);
void doEndpointCallback(MachineState * ms, const baxter_core_msgs::EndpointState& eps);
int classIdxForName(MachineState * ms, string name);

void initClassFolders(MachineState * ms, string folderName);
void writeClassToFolder(MachineState * ms, int idx, string folderName);
void writeAerialGradientsToServoCrop(MachineState * ms, int idx, string servoCrop_file_path);
void writeThumbnail(MachineState * ms, int idx, string servoCrop_file_path);
void writeIr2D(MachineState * ms, int idx, string this_range_path);
void write3dGrasps(MachineState * ms, int idx, string this_grasp_path);
void writeGraspMemory(MachineState * ms, int idx, string this_grasp_path);
void writeSceneModel(MachineState * ms, int idx, string this_grasp_path);

void saveAccumulatedStreamToPath(MachineState * ms, string path);
streamImage * setIsbIdx(MachineState * ms, int idx);
streamImage * setIsbIdxNoLoad(MachineState * ms, int idx);
streamImage * setIsbIdxYesLoadNoKick(MachineState * ms, int idx);
streamImage * setIsbIdxNoLoadNoKick(MachineState * ms, int idx);
streamImage * getIsbIdxNoLoadNoKick(MachineState * ms, int idx);
void resetAccumulatedStreamImage(MachineState * ms);
int getStreamPoseAtTime(MachineState * ms, double tin, eePose * outArm, eePose * outBase);
int getStreamPoseAtTimeThreadSafe(MachineState * ms, double tin, eePose * outArm, eePose * outBase);
void castRangeRay(MachineState * ms, double thisRange, eePose thisPose, Vector3d * castPointOut, Vector3d * rayDirectionOut);
void update2dRangeMaps(MachineState * ms, Vector3d castPoint);

bool streamRangeComparator(streamRange i, streamRange j);
bool streamPoseComparator(streamEePose i, streamEePose j);
bool streamImageComparator(streamImage i, streamImage j);
bool streamJointsComparator(streamJoints i, streamJoints j);
bool streamWordComparator(streamWord i, streamWord j);
bool streamLabelComparator(streamLabel i, streamLabel j);

void activateSensorStreaming(MachineState * ms);
void deactivateSensorStreaming(MachineState * ms);

int didSensorStreamTimeout(MachineState * ms);

void populateStreamImageBuffer(MachineState * ms);
void populateStreamPoseBuffer(MachineState * ms);
void populateStreamRangeBuffer(MachineState * ms);
void populateStreamWordBuffer(MachineState * ms);
void populateStreamLabelBuffer(MachineState * ms);

void streamImageAsClass(MachineState * ms, Mat im, int classToStreamIdx, double now);
void streamRangeAsClass(MachineState * ms, double range, int classToStreamIdx, double now);
void streamPoseAsClass(MachineState * ms, eePose poseIn, int classToStreamIdx, double now);
void streamWordAsClass(MachineState * ms, string wordIn, string commandIn, int classToStreamIdx, double now);
void streamLabelAsClass(MachineState * ms, string labelIn, int classToStreamIdx, double now);

void writeRangeBatchAsClass(MachineState * ms, int classToStreamIdx);
void writePoseBatchAsClass(MachineState * ms, int classToStreamIdx);
void writeWordBatchAsClass(MachineState * ms, int classToStreamIdx);
void writeLabelBatchAsClass(MachineState * ms, int classToStreamIdx);

void checkAndStreamWord(MachineState * ms, string wordIn, string commandIn);

void writeSideAndSerialToFileStorage(FileStorage& fsvO);
void readSideAndSerialFromFileStorage(MachineState * ms, FileStorage fsvI, string * serial, string * side);
string appendSideAndSerial(MachineState * ms, string root);

void populateStreamJointsBuffer(MachineState * ms);
void streamJointsAsClass(MachineState * ms, int classToStreamIdx, double now);
void writeJointsBatchAsClass(MachineState * ms, int classToStreamIdx);



bool isInGripperMask(MachineState * ms, int x, int y);
bool isInGripperMaskBlocks(MachineState * ms, int x, int y);
bool isGripperGripping(MachineState * ms);
void initialize3DParzen(MachineState * ms);
void l2Normalize3DParzen(MachineState * ms);
void initializeParzen(MachineState * ms);
void l2NormalizeParzen(MachineState * ms);
void l2NormalizeFilter(MachineState * ms);


cv::Vec3b getCRColor(MachineState * ms);
cv::Vec3b getCRColor(MachineState * ms, Mat im);
Quaternionf extractQuatFromPose(geometry_msgs::Pose poseIn);



void scanXdirection(MachineState * ms, double speedOnLines, double speedBetweenLines);
void scanYdirection(MachineState * ms, double speedOnLines, double speedBetweenLines);

Eigen::Quaternionf getGGRotation(MachineState * ms, int givenGraspGear);
void setGGRotation(MachineState * ms, int thisGraspGear);

Eigen::Quaternionf getCCRotation(MachineState * ms, int givenGraspGear, double angle);
void setCCRotation(MachineState * ms, int thisGraspGear);
void publishVolumetricMap(MachineState * ms);

void endEffectorAngularUpdate(eePose *givenEEPose, eePose *deltaEEPose);
void endEffectorAngularUpdateOuter(eePose *givenEEPose, eePose *deltaEEPose);



void renderRangeogramView(MachineState * ms);
void renderObjectMapView(MachineState * leftArm, MachineState * rightArm);
void renderObjectMapViewOneArm(MachineState * ms);
void objectMapCallbackFunc(int event, int x, int y, int flags, void* userdata);
void doObjectMapCallbackFunc(int event, int x, int y, int flags, MachineState * ms);


void renderAccumulatedImageAndDensity(MachineState * ms);
void drawMapPolygon(Mat mapImage, double mapXMin, double mapXMax, double mapYMin, double mapYMax, gsl_matrix * poly, cv::Scalar color);
gsl_matrix * mapCellToPolygon(MachineState * ms, int map_i, int map_j) ;

void pilotInit(MachineState * ms);
void spinlessPilotMain(MachineState * ms);

int doCalibrateGripper(MachineState * ms);
int calibrateGripper(MachineState * ms);
int shouldIPick(MachineState * ms, int classToPick);
int getLocalGraspGear(MachineState * ms, int globalGraspGearIn);
int getGlobalGraspGear(MachineState * ms, int localGraspGearIn);
void convertGlobalGraspIdxToLocal(MachineState * ms, const int rx, const int ry, 
                                  int * localX, int * localY);

void convertLocalGraspIdxToGlobal(MachineState * ms, const int localX, const int localY,
                                  int * rx, int * ry);

void changeTargetClass(MachineState * ms, int);

void zeroGraspMemoryAndRangeMap(MachineState * ms);
void zeroClassGraspMemory(MachineState * ms);
void guard3dGrasps(MachineState * ms);
void guardSceneModels(MachineState * ms);
void guardGraspMemory(MachineState * ms);
void loadSampledGraspMemory(MachineState * ms);
void loadMarginalGraspMemory(MachineState * ms);
void loadPriorGraspMemory(MachineState * ms, priorType);
void estimateGlobalGraspGear();
void drawMapRegisters(MachineState * ms);


void guardHeightMemory(MachineState * ms);
void loadSampledHeightMemory(MachineState * ms);
void loadMarginalHeightMemory(MachineState * ms);
void loadPriorHeightMemory(MachineState * ms, priorType);
double convertHeightIdxToGlobalZ(MachineState * ms, int);
double convertHeightIdxToLocalZ(MachineState * ms, int);
void convertHeightGlobalZToIdx(MachineState * ms, double);
void testHeightConversion(MachineState * ms);
void drawHeightMemorySample(MachineState * ms);
void copyHeightMemoryTriesToClassHeightMemoryTries(MachineState * ms);

void applyGraspFilter(MachineState * ms, double * rangeMapRegA, double * rangeMapRegB);
void prepareGraspFilter(MachineState * ms, int i);
void prepareGraspFilter1(MachineState * ms);
void prepareGraspFilter2(MachineState * ms);
void prepareGraspFilter3(MachineState * ms);
void prepareGraspFilter4(MachineState * ms);

void copyRangeMapRegister(MachineState * ms, double * src, double * target);
void copyGraspMemoryRegister(MachineState * ms, double * src, double * target);
void loadGlobalTargetClassRangeMap(MachineState * ms, double * rangeMapRegA, double * rangeMapRegB);
void loadLocalTargetClassRangeMap(MachineState * ms, double * rangeMapRegA, double * rangeMapRegB);
void copyGraspMemoryTriesToClassGraspMemoryTries(MachineState * ms);
void copyClassGraspMemoryTriesToGraspMemoryTries(MachineState * ms);

void selectMaxTarget(MachineState * ms, double minDepth);
void selectMaxTargetThompsonContinuous2(MachineState * ms, double minDepth);

void recordBoundingBoxSuccess(MachineState * ms);
void recordBoundingBoxFailure(MachineState * ms);

void restartBBLearning(MachineState * ms);

eePose analyticServoPixelToReticle(MachineState * ms, eePose givenPixel, eePose givenReticle, double angle, eePose givenCameraPose);
void moveCurrentGripperRayToCameraVanishingRay(MachineState * ms);
Mat makeGCrop(MachineState * ms, int etaX, int etaY);
void pixelServo(MachineState * ms, int servoDeltaX, int servoDeltaY, double servoDeltaTheta);
void gradientServo(MachineState * ms);
void gradientServoLatentClass(MachineState * ms);
void continuousServo(MachineState * ms);
void synchronicServo(MachineState * ms);
void darkServo(MachineState * ms);
void faceServo(MachineState * ms, vector<Rect> faces);
int simulatedServo();

void initRangeMaps(MachineState * ms);
void initRangeMapsNoLoad(MachineState * ms);

int isThisGraspMaxedOut(MachineState * ms, int i);

void pixelToGlobal(MachineState * ms, int pX, int pY, double gZ, double * gX, double * gY);
void pixelToGlobal(MachineState * ms, int pX, int pY, double gZ, double * gX, double * gY, eePose givenEEPose);
void pixelToGlobalFromCache(MachineState * ms, int pX, int pY, double * gX, double * gY, pixelToGlobalCache * cache);
void pixelToGlobalFromCacheBackCast(MachineState * ms, int pX, int pY, double * gX, double * gY, pixelToGlobalCache * cache);
void computePixelToGlobalCache(MachineState * ms, double gZ, eePose givenEEPose, pixelToGlobalCache * cache);
void globalToPixel(MachineState * ms, int * pX, int * pY, double gZ, double gX, double gY);
void globalToPixel(MachineState * ms, int * pX, int * pY, double gZ, double gX, double gY, eePose givenEEPose);
void globalToPixelPrint(MachineState * ms, int * pX, int * pY, double gZ, double gX, double gY);
eePose pixelToGlobalEEPose(MachineState * ms, int pX, int pY, double gZ);

void pixelToPlane(MachineState * ms, int pX, int pY, double gZ, double * gX, double * gY, eePose givenEEPose, eePose referenceFrame);
void computePixelToPlaneCache(MachineState * ms, double gZ, eePose givenEEPose, eePose referenceFrame, pixelToGlobalCache * cache);


void mapPixelToWorld(Mat mapImage, double xMin, double xMax, double yMin, double yMax, int px, int py, double &x, double &y) ;
cv::Point worldToMapPixel(Mat mapImage, double xMin, double xMax, double yMin, double yMax, double x, double y);

void paintEEPoseOnWrist(MachineState * ms, eePose toPaint, cv::Scalar theColor);

double vectorArcTan(MachineState * ms, double y, double x);
void initVectorArcTan(MachineState * ms);

void mapBlueBox(MachineState * ms, cv::Point tbTop, cv::Point tbBot, int detectedClass, ros::Time timeToMark);
void mapBox(MachineState * ms, BoxMemory boxMemory);

void queryIK(MachineState * ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest);

void globalToMapBackground(MachineState * ms, double gX, double gY, double zToUse, int * mapGpPx, int * mapGpPy);

void loadCalibration(MachineState * ms, string inFileName);
void saveCalibration(MachineState * ms, string outFileName);

void findDarkness(MachineState * ms, int * xout, int * yout);
void findLight(MachineState * ms, int * xout, int * yout);
void findOptimum(MachineState * ms, int * xout, int * yout, int sign);

void fillLocalUnitBasis(eePose localFrame, Vector3d * localUnitX, Vector3d * localUnitY, Vector3d * localUnitZ);

////////////////////////////////////////////////
// end pilot prototypes 
//
// start node prototypes
////////////////////////////////////////////////

int doubleToByte(double in);



void gridKeypoints(MachineState * ms, int gImW, int gImH, cv::Point top, cv::Point bot, int strideX, int strideY, vector<KeyPoint>& keypoints, int period);

bool isFiniteNumber(double x);

void appendColorHist(Mat& yCrCb_image, vector<KeyPoint>& keypoints, Mat& descriptors, Mat& descriptors2);
void processImage(Mat &image, Mat& gray_image, Mat& yCrCb_image, double sigma);

void bowGetFeatures(MachineState * ms, std::string classDir, const char *className, double sigma, int keypointPeriod, int * grandTotalDescriptors, DescriptorExtractor * extractor, BOWKMeansTrainer * bowTrainer);
void kNNGetFeatures(MachineState * ms, std::string classDir, const char *className, int label, double sigma, Mat &kNNfeatures, Mat &kNNlabels, double sobel_sigma);
void posekNNGetFeatures(MachineState * ms, std::string classDir, const char *className, double sigma, Mat &kNNfeatures, Mat &kNNlabels,
                        vector< cv::Vec<double,4> >& classQuaternions, int keypointPeriod, BOWImgDescriptorExtractor *bowExtractor, int lIndexStart = 0);


void drawDensity(MachineState * ms, double scale);
void goCalculateDensity(MachineState * ms);
void goFindBlueBoxes(MachineState * ms);
void goClassifyBlueBoxes(MachineState * ms);
void goFindRedBoxes();

void resetAccumulatedImageAndMass(MachineState * ms);
void substituteStreamAccumulatedImageQuantities(MachineState * ms);
void substituteStreamImageQuantities(MachineState * ms);

void substituteAccumulatedImageQuantities(MachineState * ms);
void substituteLatestImageQuantities(MachineState * ms);

void loadROSParamsFromArgs(MachineState * ms);
void saveROSParams(MachineState * ms);

void spinlessNodeMain(MachineState * ms);
void nodeInit(MachineState * ms);
void detectorsInit(MachineState * ms);
void initRedBoxes();

void tryToLoadRangeMap(MachineState * ms, std::string classDir, const char *className, int i);
void clearAllRangeMaps(MachineState * ms);

void processSaliency(Mat in, Mat out);

void happy(MachineState * ms);
void sad(MachineState * ms);
void neutral(MachineState * ms);


void guardViewers(MachineState * ms);

int findClosestBlueBoxMemory(MachineState * ms, eePose targetPose, int classToSearch = -1);
void fillRecognizedObjectArrayFromBlueBoxMemory(MachineState * ms, object_recognition_msgs::RecognizedObjectArray * roa);
void promoteBlueBoxes(MachineState * ms);
void fillEinStateMsg(MachineState * ms, EinState * stateOut);
void targetBoxMemory(MachineState * ms, int idx);

bool isFocusedClassValid(MachineState * ms);
void initializeAndFocusOnTempClass(MachineState * ms);
void initializeAndFocusOnNewClass(MachineState * ms);


double computeSimilarity(MachineState * ms, int class1, int class2);
double computeSimilarity(MachineState * ms, Mat im1, Mat im2);

void prepareForCrossCorrelation(MachineState * ms, Mat input, Mat& output, int thisOrient, int numOrientations, double thisScale, Size toBecome);
void normalizeForCrossCorrelation(MachineState * ms, Mat input, Mat& output);
void pilotCallbackFunc(int event, int x, int y, int flags, void* userdata);

void publishConsoleMessage(MachineState * ms, string msg);



////////////////////////////////////////////////
// end node prototypes 
#endif /* _EIN_H_ */
