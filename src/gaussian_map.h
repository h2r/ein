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
  double rmus;
  double gmus;
  double bmus;
  double rsigmas;
  double gsigmas;
  double bsigmas;
  double rgbsamples;
  double zcounts;
  double zsquaredcounts;
  double zmus;
  double zsigmas;
  double zsamples;
} GaussianMapCell;

class GaussianMap {
  private:
  public:

  int width; // or columns
  int height; // or rows
  double cell_width = 0.01;
  GaussianMapCell *cells = NULL;

  GaussianMap(int w, int h, double cw);
  void reallocate();

  int safeAt(int x, int y);
  GaussianMapCell *refAtCell(int x, int y);  
  GaussianMapCell valAtCell(int x, int y);
  GaussianMapCell bilinValAtCell(double x, double y);
  GaussianMapCell bilinValAtMeters(double x, double y);

  void saveToFile(string filename);
  void loadFromFile(string filename);
  Mat rgbToMat();
  Mat zToMat();

  shared_ptr<GaussianMap> copyBox(int x1, int y1, int x2, int y2);

  void invalidateBox(int x1, int y1, int x2, int y2);
  void invalidate();
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
  sceneObjectType sot;
};

class Scene {
  public:

  std::shared_ptr<MachineState> ms;

  Scene(shared_ptr<MachineState> ms);

  eePose background_pose;
  shared_ptr<GaussianMap> background_map;
  shared_ptr<GaussianMap> predicted_map;
  shared_ptr<GaussianMap> observed_map;
  // differeces between predicted mean and variance are approximately equally relevant,
  //   consider the kl divergence of two gaussians
  shared_ptr<GaussianMap> discrepancy;
  // transform image to find hot spots
  Mat discrepancyDensity;

  vector<SceneObject> predictedObjects;

  double score;

  void composePredictedMap();
  void measureDiscrepancy();
  void assignScore();
  void assignScoreRegion();

  void proposeRegion();
  // object only makes map better, but must "win" on at least a fraction of its pixels (prior on number of parts)
  void proposeObject();

  void tryToAddObjectToScene();

  void removeSpaceObjects();
  void addSpaceObjects();
  void reregisterBackground();
  void reregisterObject(int i);
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
};

#endif /* _EIN_SCENE_H_ */
