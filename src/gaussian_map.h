#ifndef _EIN_SCENE_H_
#define _EIN_SCENE_H_

#include "eePose.h"
#include "ein_util.h"

class MachineState;
class Word;

typedef struct _GaussianMapCell {
  // mus and sigmas should be updated whenever anything else is modified
  double rcounts;
  double gcounts;
  double bcounts;
  double rsquaredcounts;
  double gsquaredcounts;
  double bsquaredcounts;
  double rmu;
  double gmu;
  double bmu;
  double rsigmasquared;
  double gsigmasquared;
  double bsigmasquared;
  double rgbsamples;
  double zcounts;
  double zsquaredcounts;
  double zmu;
  double zsigmasquared;
  double zsamples;

  void zero();

  void writeToFileStorage(FileStorage& fsvO) const;
  void readFromFileNodeIterator(FileNodeIterator& it);
  void readFromFileNode(FileNode& it);

  void newObservation(Vec3b obs);
  
} GaussianMapCell;

class GaussianMap {

  public:
  int width; // or columns
  int height; // or rows
  int x_center_cell;
  int y_center_cell;

  

  double cell_width = 0.01;
  GaussianMapCell *cells = NULL;

  GaussianMap(int w, int h, double cw);
  ~GaussianMap();
  void reallocate();

  int safeAt(int x, int y);
  GaussianMapCell *refAtCell(int x, int y);  
  GaussianMapCell valAtCell(int x, int y);
  GaussianMapCell bilinValAtCell(double x, double y);
  GaussianMapCell bilinValAtMeters(double x, double y);

  void metersToCell(double xm, double ym, int * xc, int * yc);
  void cellToMeters(int xc, int yc, double * xm, double * ym);

  void writeToFileStorage(FileStorage& fsvO);
  void readFromFileNodeIterator(FileNodeIterator& it);
  void readFromFileNode(FileNode& it);
  void saveToFile(string filename);
  void loadFromFile(string filename);

  void recalculateMusAndSigmas();

  void rgbDiscrepancyMuToMat(Mat& out);
  void rgbMuToMat(Mat& out);
  void rgbSigmaSquaredToMat(Mat& out);
  void rgbCountsToMat(Mat& out);

  void zMuToMat(Mat& out);
  void zSigmaSquaredToMat(Mat& out);
  void zCountsToMat(Mat& out);

  shared_ptr<GaussianMap> copyBox(int _x1, int _y1, int _x2, int _y2);

  void zeroBox(int _x1, int _y1, int _x2, int _y2);
  void zero();
};

typedef enum {
  BACKGROUND = 0,
  PREDICTED = 1,
  SPACE = 2
} sceneObjectType;

class SceneObject {
  public:
  // pose within the scene
  eePose scene_pose;
  int labeledClassIndex;
  string objectLabel;
  sceneObjectType sot;

  void writeToFileStorage(FileStorage& fsvO);
  void readFromFileNodeIterator(FileNodeIterator& it);
  void readFromFileNode(FileNode& it);
};

class Scene {
  public:
  int width; // or columns
  int height; // or rows
  int x_center_cell;
  int y_center_cell;

  double cell_width = 0.01;

  std::shared_ptr<MachineState> ms;

  Scene(shared_ptr<MachineState> ms, int w, int h, double cw);
  void reallocate();

  eePose background_pose;
  shared_ptr<GaussianMap> background_map;
  shared_ptr<GaussianMap> predicted_map;
  Mat predicted_segmentation;
  shared_ptr<GaussianMap> observed_map;
  shared_ptr<GaussianMap> discrepancy;
  Mat discrepancy_magnitude;
  // transform image to find hot spots
  Mat discrepancy_density;

  vector<SceneObject> predicted_objects;

  double score;

  void composePredictedMap();
  void measureDiscrepancy();
  double assignScore();
  double measureScoreRegion(int _x1, int _y1, int _x2, int _y2);

  void proposeRegion();
  // object only makes map better, but must "win" on at least a fraction of its pixels (prior on number of parts)
  void proposeObject();

  void tryToAddObjectToScene();
  void addObjectToPredictedMap();
  void removeObjectFromPredictedMap();

  void removeSpaceObjects();
  void addSpaceObjects();
  void reregisterBackground();
  void reregisterObject(int i);

  void writeToFileStorage(FileStorage& fsvO);
  void readFromFileNodeIterator(FileNodeIterator& it);
  void readFromFileNode(FileNode& it);
  void saveToFile(string filename);
  void loadFromFile(string filename);
};

// transition tables can be instanced for particular settings; transitions may differ in low gravity, high wind, or soft ground, 
//   especially helpful when running experiments to compare two subroutimnes or parameter choices.
// state_labels can be populated by classLabels or can contain statements about objects, too
class TransitionTable {
  public:
  std::vector<string> state_labels;
  std::vector<string> actions;
  std::vector<double> action_probabilities;

  shared_ptr<int> counts;

  shared_ptr<Scene> prescene;
  shared_ptr<Scene> postscene;
  int performed_action;

  void setPrescene(shared_ptr<Scene> s);
  void setPostscene(shared_ptr<Scene> s);
  void setPerformedAction();

  void recordTransitionSceneObject();
  void setStateLabelsFromClassLabels();

  void setActions(std::vector<string> * actions);
  void setActionProbabilities(std::vector<double> * actions);

  void initCounts();

  void writeToFileStorage(FileStorage& fsvO);
  void readFromFileNodeIterator(FileNodeIterator& it);
  void readFromFileNode(FileNode& it);
  void saveToFile(string filename);
  void loadFromFile(string filename);
};

#endif /* _EIN_SCENE_H_ */
