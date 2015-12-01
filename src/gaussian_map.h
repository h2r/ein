#ifndef _EIN_SCENE_H_
#define _EIN_SCENE_H_

#include "eePose.h"
#include "ein_util.h"

class MachineState;
class Word;


typedef struct _GaussianMapCell {
  // mus and sigmas should be updated whenever anything else is modified
  double rcounts, gcounts, bcounts;
  double rsquaredcounts, gsquaredcounts, bsquaredcounts;
  double rmus, gmus, bmus;
  double rsigmas, gsigmas, bsigmas;
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

  eePose scene_pose;
  
  GaussianMapCell *cells = NULL;

  void reallocate();
  GaussianMap(int w, int h, double cw, eePose sp);

  int safeAt(int x, int y) {
    return ( (cells != NULL) && (x >= 0) && (x < width) && (y >= 0) && (y < height) );
  }

  GaussianMapCell *refAtCell(int x, int y) {
    return (cells + x + width*y);
  }
  GaussianMapCell valAtCell(int x, int y) {
    return *(cells + x + width*y);
  }


  GaussianMapCell bilinValAtCell(double x, double y) {
    // XXX

  }

  GaussianMapCell bilinValAtMeters(double x, double y) {
    // XXX
    // call bilinValAtCell

  }

// XXX write to file
// XXX read from file
// XXX class maps
// XXX dump to mat

// XXX extract a bounding box


};

class SceneObject {
  public:
  eePose scene_pose;
  int labeledClassIndex;
};

// XXX how should these be passed around? shared pointers?
class Scene {
  public:
  std::shared_ptr<MachineState> ms;

  Scene(shared_ptr<MachineState> ms);

  shared_ptr<GaussianMap> backgroundMap;
  shared_ptr<GaussianMap> predictedMap;
  shared_ptr<GaussianMap> observedMap;
  // differeces between predicted mean and variance are approximately equally relevant,
  //   consider the kl divergence of two gaussians
  shared_ptr<GaussianMap> discrepancy;
  // transform image to find hot spots
  Mat discrepancyDensity;

  vector<SceneObject> predictedObjects;

  double score;


  // XXX
  void composePredictedMap();
  // XXX
  void measureDiscrepancy();
  // XXX
  void assignScore();
  void assignScoreRegion();


  // XXX
  void sampleRegion();
  // XXX
  void sampleObject();

};


// transition tables can be instanced for particular settings; transitions may differ in low gravity, high wind, or soft ground, 
//   especially helpful when running experiments to compare two subroutimnes or parameter choices.
// classLabels can contain statements about objects, too
class TransitionTable {
  public:

  // numClasses is apt to change so we should store the numbers as they were when the matrix was last initialized
  std::vector<string> class_labels;
  // XXX maybe these should be strings
  std::vector<string> actions;
  std::vector<double> action_probabilities;
  int sNumActions;
  int NumClasses;
  shared_ptr<int> counts;

  shared_ptr<Scene> prescene;
  shared_ptr<Scene> postscene;
  string performed_action;

  // XXX word
  void setPrescene(shared_ptr<Scene> s);
  // XXX word
  void setPostscene(shared_ptr<Scene> s);
  // XXX word
  void recordTransition();

  // XXX word
  // XXX word setter
  // XXX word getter
  void setActions(std::vector<string> * actions);
  // XXX word
  // XXX word setter
  // XXX word getter
  void setActionProbabilities(std::vector<double> * actions);
};

#endif /* _EIN_SCENE_H_ */
