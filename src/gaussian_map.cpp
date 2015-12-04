#include "gaussian_map.h"
#include "ein_words.h"

void GaussianMap::reallocate() {
  if (cells == NULL) {
    cells = new GaussianMapCell[width*height];
  } else {
    delete cells;
    cells = new GaussianMapCell[width*height];
  }
}

GaussianMap::GaussianMap(int w, int h, double cw) {
  width = w;
  height = h;
  cell_width = cw;
  cells = NULL;
  reallocate();
}

int GaussianMap::safeAt(int x, int y) {
  return ( (cells != NULL) && (x >= 0) && (x < width) && (y >= 0) && (y < height) );
}

GaussianMapCell *GaussianMap::refAtCell(int x, int y) {
  return (cells + x + width*y);
}

GaussianMapCell GaussianMap::valAtCell(int x, int y) {
  return *(cells + x + width*y);
}

#define BILIN_MAPCELL(field) \
  toReturn.field = wx0*wy0*refAtCell(x0,y0)->field + wx1*wy0*refAtCell(x1,y0)->field + wx0*wy1*refAtCell(x0,y1)->field + wx1*wy1*refAtCell(x1,y1)->field;

GaussianMapCell GaussianMap::bilinValAtCell(double _x, double _y) {

  double x = min( max(0.0, _x), width-1);
  double y = min( max(0.0, _y), height-1);

  // -2 makes for appropriate behavior on the upper boundary
  double x0 = min( max(0.0, floor(x)), width-2);
  double x1 = x0+1;

  double y0 = min( max(0.0, floor(y), height-2));
  double y1 = y0+1;

  wx0 = x1-x;
  wx1 = x-x0;

  wy0 = y1-y;
  wy1 = y-y0;

  GaussianMapCell toReturn;

  // wx0*wy0*val(x0,y0) + wx1*wy0*val(x1,y0) + wx1*wy1*val(x1,y1) + wx0*wy1*val(x0,y1) ~sub 1 for vals =~
  // wx0*(wy0+wy1) + wx1*(wy0 + wy1) = wx0 + wx1 = 1

  BILIN_MAPCELL(rcounts);
  BILIN_MAPCELL(gcounts);
  BILIN_MAPCELL(bcounts);
  BILIN_MAPCELL(rsquaredcounts);
  BILIN_MAPCELL(gsquaredcounts);
  BILIN_MAPCELL(bsquaredcounts);
  BILIN_MAPCELL(rmus);
  BILIN_MAPCELL(gmus);
  BILIN_MAPCELL(bmus);
  BILIN_MAPCELL(rsigmas);
  BILIN_MAPCELL(gsigmas);
  BILIN_MAPCELL(bsigmas);
  BILIN_MAPCELL(rgbsamples);
  BILIN_MAPCELL(zcounts);
  BILIN_MAPCELL(zsquaredcounts);
  BILIN_MAPCELL(zmus);
  BILIN_MAPCELL(zsigmas);
  BILIN_MAPCELL(zsamples);

  return toReturn;
}

GaussianMapCell GaussianMap::bilinValAtMeters(double x, double y) {
  // XXX
  // call bilinValAtCell

}

// XXX write to file
void GaussianMap::saveToFile(string filename) {
}

// XXX read from file
void GaussianMap::loadFromFile(string filename) {
}

// XXX 
Mat GaussianMap::rgbToMat() {
}

// XXX 
Mat GaussianMap::zToMat() {
}

// XXX extract a bounding box at specified corners top left (x1,y1) bottom right (x2,y2)
shared_ptr<GaussianMap> GaussianMap::copyBox(int x1, int y1, int x2, int y2) {
}

void GaussianMap::invalidateBox(int x1, int y1, int x2, int y2) {
}

void GaussianMap::invalidate() {
}




Scene::Scene(shared_ptr<MachineState> _ms) {
  ms = _ms;
}

// XXX 
void Scene::composePredictedMap() {
}
// XXX 
void Scene::measureDiscrepancy() {
}
// XXX 
void Scene::assignScore() {
}
// XXX 
void Scene::assignScoreRegion() {
}

// XXX 
void Scene::proposeRegion() {
}
// XXX 
void Scene::proposeObject() {
}

// XXX 
void Scene::tryToAddObjectToScene() {
}

// XXX 
void Scene::removeSpaceObjects() {
}

// XXX 
void Scene::addSpaceObjects() {
}

// XXX 
void Scene::reregisterBackground() {
}

// XXX 
void Scene::reregisterObject(int i) {
}




// XXX word
void TransitionTable::setPrescene(shared_ptr<Scene> s) {
}

// XXX word
void TransitionTable::setPostscene(shared_ptr<Scene> s) {
}

// XXX word
void TransitionTable::setPerformedAction() {
}

// XXX word
void TransitionTable::recordTransitionSceneObject() {
}
// XXX word
void setStateLabelsFromClassLabels() {
}

// XXX word
// XXX word setter
// XXX word getter
//   the word should take a compound word of words, which may be compound words, 
//   and stores their reprs as strings; this makes it easier to serialize
void TransitionTable::setActions(std::vector<string> * actions) {
}

// XXX word
// XXX word setter
// XXX word getter
void TransitionTable::setActionProbabilities(std::vector<double> * actions) {
}

// XXX 
void TransitionTable::initCounts() {
}





/*


// XXX accessors for sceneObjects

// XXX word to add objects to scene until evidence is accounted for

// XXX words to save and load current background and current object maps
// XXX word to remove object from scene and invalidate region around it
// XXX word to invalidate entire observedMap
// XXX word to search clearance area for the cell with lowest number of counts, push its eePose




// XXX sample action word and put string word onto stack
// XXX evaluate string word 
// XXX use string word to set current action 

// XXX calls to init class maps
//  XXX add calls to writeClassToFolder

// XXX word to update observed_map based on current wrist camera image
// XXX map the table by using image stream buffer 
//   XXX  word to update observed_map  based on stream buffer, images and IR
// XXX when training from crops, render from the stream buffer into the observed_map then crop out

// XXX word to fly on a path over all the sceneObjects, streaming images
// XXX word to densely explore map streaming images and IR 
*/





namespace ein_words {

WORD(SceneSaveBackgroundMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
//XXX 
}
END_WORD
REGISTER_WORD(SceneSaveBackgroundMap)

WORD(SceneLoadBackgroundMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
//XXX 
}
END_WORD
REGISTER_WORD(SceneLoadBackgroundMap)

WORD(SceneSaveObservedMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
//XXX 
}
END_WORD
REGISTER_WORD(SceneSaveObservedMap)

WORD(SceneLoadObservedMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
//XXX 
}
END_WORD
REGISTER_WORD(SceneLoadObservedMap)

WORD(SceneInitObservedMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
//XXX 
}
END_WORD
REGISTER_WORD(SceneInitObservedMap)

WORD(SceneUpdateObservedFromSnout)
virtual void execute(std::shared_ptr<MachineState> ms) {
//XXX 
}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromSnout)

WORD(SceneUpdateDiscrepancy)
virtual void execute(std::shared_ptr<MachineState> ms) {
//XXX 
}
END_WORD
REGISTER_WORD(SceneUpdateDiscrepancy)

WORD(SceneGrabCenterCropAsClass)
virtual void execute(std::shared_ptr<MachineState> ms) {
//XXX 
}
END_WORD
REGISTER_WORD(SceneGrabCenterCropAsClass)

WORD(SceneDensityFromDiscrepancy)
virtual void execute(std::shared_ptr<MachineState> ms) {
// this enables denisty based models to use the new channel
//XXX 
}
END_WORD
REGISTER_WORD(SceneDensityFromDiscrepancy)

/* 
WORD()
virtual void execute(std::shared_ptr<MachineState> ms) {
}
END_WORD
REGISTER_WORD()

WORD(TransitionTableInit)
virtual void execute(std::shared_ptr<MachineState> ms) {
// zero it out
}
END_WORD
REGISTER_WORD(TransitionTableInit)

WORD(TransitionTableCount)
virtual void execute(std::shared_ptr<MachineState> ms) {
}
END_WORD
REGISTER_WORD(TransitionTableCount)


WORD(PlanWithTransitionTable)
virtual void execute(std::shared_ptr<MachineState> ms) {

// takes a desired state and outputs the action that best takes the prescene to that state

}
END_WORD
REGISTER_WORD(PlanWithTransitionTable)

*/

}




