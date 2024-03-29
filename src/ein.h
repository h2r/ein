#ifndef _EIN_H_
#define _EIN_H_


//#define DEBUG_RING_BUFFER // ring buffer


#define PROGRAM_NAME "ein"

#include <Eigen/Geometry> 
using namespace Eigen;

#include <ein/EinState.h>
#include <ein/EinConsole.h>

#include <object_recognition_msgs/RecognizedObjectArray.h>

#include <vector>
#include <string>

#include <cv.h>
#include <Eigen/Geometry> 


// slu
#include "slu/math2d.h"

// numpy library 1 (randomkit, for original beta)
#include "distributions.h"
#include "config.h"
//#include "faces.h"

using namespace std;
using namespace cv;

using namespace ein;



typedef struct pixelToGlobalCache {
  eePose givenEEPose;
  double gZ;
  int x1;
  int x2;
  int x3;
  int x4;

  int y1;
  int y2;
  int y3;
  int y4;

  double z1;
  double z2;
  double z3;
  double z4;

  double reticlePixelX;
  double reticlePixelY;
  double reticlePixelXOffset;
  double reticlePixelYOffset;

  double x_thisZ;
  double y_thisZ;

  double gXFactor;
  double gYFactor;
  double finalXOffset;
  double finalYOffset;

  Mat un_rot_mat;
  double rotx[3];
  double roty[3];

  double dx;
  double cx;
  double b42x;
  double b31x;
  double bDiffx;
  double bx;


  double dy;
  double cy;
  double b42y;
  double b31y;
  double bDiffy;
  double by;

  Eigen::Matrix4f p2gComposedZBuilt;
  Eigen::Matrix4f g2pComposedZBuilt;
  Eigen::Matrix4f p2gComposedZNotBuilt;
  Eigen::Matrix4f g2pComposedZNotBuilt;

  Eigen::Matrix4f ap;
  Eigen::Matrix4f apInv;
  Eigen::Matrix4f ccpRot;
  Eigen::Matrix4f ccpRotInv;
  Eigen::Matrix4f ccpTrans;
  Eigen::Matrix4f ccpTransInv;

  double mu_x;
  double mu_y;
  double kappa_x;
  double kappa_y;

  Vec4d target_plane;
} pixelToGlobalCache;


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


gsl_matrix * boxMemoryToPolygon(BoxMemory b);
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
int getRingPoseAtTime(MachineState * ms, ros::Time t, geometry_msgs::Pose &value, int drawSlack = 0, bool debug=false);


extern "C" {
double cephes_incbet(double a, double b, double x) ;
}
void setRingImageAtTime(MachineState * ms, ros::Time t, Mat& imToSet);
void setRingRangeAtTime(MachineState * ms, ros::Time t, double rgToSet);
void setRingPoseAtTime(MachineState * ms, ros::Time t, geometry_msgs::Pose epToSet);
void rgRingBufferAdvance(MachineState * ms);
void epRingBufferAdvance(MachineState * ms);
void allRingBuffersAdvance(MachineState * ms, ros::Time t);

void recordReadyRangeReadings(MachineState * ms);
int classIdxForName(MachineState * ms, string name);

void initClassFolders(MachineState * ms, string folderName);
void writeClassToFolder(MachineState * ms, int idx, string folderName);
void writeClassGraspsToFolder(MachineState * ms, int idx, string folderName);
void writeThumbnail(MachineState * ms, int idx, string servoCrop_file_path);
void write3dGrasps(MachineState * ms, int idx, string this_grasp_path);
void writeSceneModel(MachineState * ms, int idx, string this_grasp_path);

void saveAccumulatedStreamToPath(MachineState * ms, string path);
void castRangeRay(MachineState * ms, double thisRange, eePose thisPose, Vector3d * rayDirectionOut);
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

int loadStreamImage(MachineState * ms, streamImage * tsi);
void checkAndStreamWord(MachineState * ms, string wordIn, string commandIn);

void writeSideAndSerialToFileStorage(MachineState * ms, FileStorage& fsvO);
void readSideAndSerialFromFileStorage(MachineState * ms, FileStorage fsvI, string * serial, string * side);
string appendSideAndSerial(MachineState * ms, string root);

void populateStreamJointsBuffer(MachineState * ms);
void streamJointsAsClass(MachineState * ms, int classToStreamIdx, double now);
void writeJointsBatchAsClass(MachineState * ms, int classToStreamIdx);



bool isInGripperMask(MachineState * ms, int x, int y);
bool isInGripperMaskBlocks(MachineState * ms, int x, int y);
bool isGripperGripping(MachineState * ms);
bool isGripperMoving(MachineState * ms);

void accumulateImage(MachineState * ms);
void resetAccumulatedImageAndMass(MachineState * ms);


cv::Vec3b getCRColor(MachineState * ms);
cv::Vec3b getCRColor(MachineState * ms, Mat im);
Quaternionf extractQuatFromPose(geometry_msgs::Pose poseIn);



Eigen::Quaternionf getGGRotation(MachineState * ms, int givenGraspGear);
void setGGRotation(MachineState * ms, int thisGraspGear);

Eigen::Quaternionf getCCRotation(MachineState * ms, int givenGraspGear, double angle);
void setCCRotation(MachineState * ms, int thisGraspGear);

void endEffectorAngularUpdate(eePose *givenEEPose, eePose *deltaEEPose);
void endEffectorAngularUpdateOuter(eePose *givenEEPose, eePose *deltaEEPose);



void renderRangeogramView(MachineState * ms);
void renderObjectMapView(MachineState * leftArm, MachineState * rightArm);
void renderObjectMapViewOneArm(MachineState * ms);
void objectMapCallbackFunc(int event, int x, int y, int flags, void* userdata);
void doObjectMapCallbackFunc(int event, int x, int y, int flags, MachineState * ms);


void drawMapPolygon(Mat mapImage, double mapXMin, double mapXMax, double mapYMin, double mapYMax, gsl_matrix * poly, cv::Scalar color);
gsl_matrix * mapCellToPolygon(MachineState * ms, int map_i, int map_j) ;


int doCalibrateGripper(MachineState * ms);
int calibrateGripper(MachineState * ms);
int shouldIPick(MachineState * ms, int classToPick);

void changeTargetClass(MachineState * ms, int);
void changeCamera(MachineState * ms, int);

void guard3dGrasps(MachineState * ms);
void guardSceneModels(MachineState * ms);
void drawMapRegisters(MachineState * ms);


double convertHeightIdxToGlobalZ(MachineState * ms, int);
double convertHeightIdxToLocalZ(MachineState * ms, int);
void convertHeightGlobalZToIdx(MachineState * ms, double);
void testHeightConversion(MachineState * ms);

void selectMaxTarget(MachineState * ms, double minDepth);
void selectMaxTargetThompsonContinuous2(MachineState * ms, double minDepth);


void moveCurrentGripperRayToCameraVanishingRay(MachineState * ms);
Mat makeGCrop(MachineState * ms, int etaX, int etaY);

void pixelToGlobal(MachineState * ms, int pX, int pY, double gZ, double * gX, double * gY);
void pixelToGlobal(MachineState * ms, int pX, int pY, double gZ, double * gX, double * gY, eePose givenEEPose);
void pixelToGlobalFromCache(MachineState * ms, int pX, int pY, double * gX, double * gY, pixelToGlobalCache * cache);
void pixelToGlobalFromCacheBackCast(MachineState * ms, int pX, int pY, double * gX, double * gY, pixelToGlobalCache * cache);
void computePixelToGlobalCache(MachineState * ms, double gZ, eePose givenEEPose, pixelToGlobalCache * cache);
void globalToPixel(MachineState * ms, int * pX, int * pY, double gZ, double gX, double gY);
void globalToPixel(MachineState * ms, int * pX, int * pY, double gZ, double gX, double gY, eePose givenEEPose);
eePose pixelToGlobalEEPose(MachineState * ms, int pX, int pY, double gZ);
string pixelToGlobalCacheToString(const pixelToGlobalCache &cache);

void pixelToPlane(MachineState * ms, int pX, int pY, double gZ, double * gX, double * gY, eePose givenEEPose, eePose referenceFrame);
void computePixelToPlaneCache(MachineState * ms, double gZ, eePose givenEEPose, eePose referenceFrame, pixelToGlobalCache * cache);

void pixelToGlobalFullFromCacheZNotBuilt(MachineState * ms, int pX, int pY, double * gX, double * gY, pixelToGlobalCache * cache, double z);
void pixelToGlobalFullFromCacheZBuilt(MachineState * ms, int pX, int pY, double * gX, double * gY, pixelToGlobalCache * cache);
void globalToPixelFullFromCache(MachineState * ms, int * pX, int * pY, double gX, double gY, double gZ, pixelToGlobalCache * cache);
void computePixelToGlobalFullCache(MachineState * ms, double gZ, eePose givenEEPose, pixelToGlobalCache * cache);

void pixelToGlobalFullFromCacheZOOP(MachineState * ms, int pX, int pY, double * gX, double * gY, pixelToGlobalCache * cache);
void computePixelToGlobalFullOOPCache(MachineState * ms, double gZ, eePose givenEEPose, eePose otherPlane, pixelToGlobalCache * cache);

void mapPixelToWorld(Mat mapImage, double xMin, double xMax, double yMin, double yMax, int px, int py, double &x, double &y) ;
cv::Point worldToMapPixel(Mat mapImage, double xMin, double xMax, double yMin, double yMax, double x, double y);

void paintEEPoseOnWrist(MachineState * ms, eePose toPaint, cv::Scalar theColor);

double vectorArcTan(MachineState * ms, double y, double x);
void initVectorArcTan(MachineState * ms);

void mapBlueBox(MachineState * ms, cv::Point tbTop, cv::Point tbBot, int detectedClass, ros::Time timeToMark);
void mapBox(MachineState * ms, BoxMemory boxMemory);


void globalToMapBackground(MachineState * ms, double gX, double gY, double zToUse, int * mapGpPx, int * mapGpPy);


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


void goFindBlueBoxes(MachineState * ms);
void goClassifyBlueBoxes(MachineState * ms);
void goFindRedBoxes();

void loadROSParamsFromArgs(MachineState * ms);

void irInit(MachineState * ms);
void nodeInit(MachineState * ms);
void detectorsInit(MachineState * ms);
void initRedBoxes();

void tryToLoadRangeMap(MachineState * ms, std::string classDir, const char *className, int i);
void clearAllRangeMaps(MachineState * ms);

void processSaliency(Mat in, Mat out);

void initializeViewers(MachineState * ms);

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
void mapCallbackFunc(int event, int x, int y, int flags, void* userdata);

void loadConfig(MachineState * ms, string filename);
void saveConfig(MachineState * ms, string outFileName);

////////////////////////////////////////////////
// end node prototypes 
#endif /* _EIN_H_ */
