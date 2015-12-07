#include "gaussian_map.h"
#include "ein_words.h"

void _GaussianMapCell::zero() {
  rcounts = 0.0;
  gcounts = 0.0;
  bcounts = 0.0;
  rsquaredcounts = 0.0;
  gsquaredcounts = 0.0;
  bsquaredcounts = 0.0;
  rmu = 0.0;
  gmu = 0.0;
  bmu = 0.0;
  rsigmasquared = 0.0;
  gsigmasquared = 0.0;
  bsigmasquared = 0.0;
  rgbsamples = 0.0;
  zcounts = 0.0;
  zsquaredcounts = 0.0;
  zmu = 0.0;
  zsigmasquared = 0.0;
  zsamples = 0.0;
}

void _GaussianMapCell::writeToFileStorage(FileStorage& fsvO) const {
  fsvO << "{:";
  fsvO << "rcounts" << rcounts;
  fsvO << "gcounts" << gcounts;
  fsvO << "bcounts" << bcounts;
  fsvO << "rsquaredcounts" << rsquaredcounts;
  fsvO << "gsquaredcounts" << gsquaredcounts;
  fsvO << "bsquaredcounts" << bsquaredcounts;
  fsvO << "rmu" << rmu;
  fsvO << "gmu" << gmu;
  fsvO << "bmu" << bmu;
  fsvO << "rsigmasquared" << rsigmasquared;
  fsvO << "gsigmasquared" << gsigmasquared;
  fsvO << "bsigmasquared" << bsigmasquared;
  fsvO << "rgbsamples" << rgbsamples;
  fsvO << "zcounts" << zcounts;
  fsvO << "zsquaredcounts" << zsquaredcounts;
  fsvO << "zmu" << zmu;
  fsvO << "zsigmasquared" << zsigmasquared;
  fsvO << "zsamples" << zsamples;
  fsvO << "}";
}

void _GaussianMapCell::readFromFileNodeIterator(FileNodeIterator& it) {
  rcounts         =  (double)(*it)["rcounts"];         
  gcounts         =  (double)(*it)["gcounts"];         
  bcounts         =  (double)(*it)["bcounts"];         
  rsquaredcounts  =  (double)(*it)["rsquaredcounts"];                
  gsquaredcounts  =  (double)(*it)["gsquaredcounts"];                
  bsquaredcounts  =  (double)(*it)["bsquaredcounts"];                
  rmu             =  (double)(*it)["rmu"];     
  gmu             =  (double)(*it)["gmu"];     
  bmu             =  (double)(*it)["bmu"];     
  rsigmasquared   =  (double)(*it)["rsigmasquared"];               
  gsigmasquared   =  (double)(*it)["gsigmasquared"];               
  bsigmasquared   =  (double)(*it)["bsigmasquared"];               
  rgbsamples      =  (double)(*it)["rgbsamples"];            
  zcounts         =  (double)(*it)["zcounts"];         
  zsquaredcounts  =  (double)(*it)["zsquaredcounts"];                
  zmu             =  (double)(*it)["zmu"];     
  zsigmasquared   =  (double)(*it)["zsigmasquared"];               
  zsamples        =  (double)(*it)["zsamples"];          
}

void _GaussianMapCell::readFromFileNode(FileNode& it) {
  rcounts         =  (double)(it)["rcounts"];         
  gcounts         =  (double)(it)["gcounts"];         
  bcounts         =  (double)(it)["bcounts"];         
  rsquaredcounts  =  (double)(it)["rsquaredcounts"];                
  gsquaredcounts  =  (double)(it)["gsquaredcounts"];                
  bsquaredcounts  =  (double)(it)["bsquaredcounts"];                
  rmu             =  (double)(it)["rmu"];     
  gmu             =  (double)(it)["gmu"];     
  bmu             =  (double)(it)["bmu"];     
  rsigmasquared   =  (double)(it)["rsigmasquared"];               
  gsigmasquared   =  (double)(it)["gsigmasquared"];               
  bsigmasquared   =  (double)(it)["bsigmasquared"];               
  rgbsamples      =  (double)(it)["rgbsamples"];            
  zcounts         =  (double)(it)["zcounts"];         
  zsquaredcounts  =  (double)(it)["zsquaredcounts"];                
  zmu             =  (double)(it)["zmu"];     
  zsigmasquared   =  (double)(it)["zsigmasquared"];               
  zsamples        =  (double)(it)["zsamples"];          
}

void GaussianMap::reallocate() {
  if (width <= 0 || height <= 0) {
    cout << "GaussianMap area error: tried to allocate width, height: " << width << " " << height << endl;
  } else if ((width % 2 == 0) || (height % 2 == 0)) {
    cout << "GaussianMap parity error: tried to allocate width, height: " << width << " " << height << endl;
  } else {
    if (cells == NULL) {
      cells = new GaussianMapCell[width*height];
    } else {
      delete cells;
      cells = new GaussianMapCell[width*height];
    }
  }
}

GaussianMap::GaussianMap(int w, int h, double cw) {
  width = w;
  height = h;
  x_center_cell = (width-1)/2;
  y_center_cell = (height-1)/2;
  cell_width = cw;
  cells = NULL;
  reallocate();
}

GaussianMap::~GaussianMap() {
  if (cells != NULL) {
    delete cells;
    cells = NULL;
  } else {
  }
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

  double x = min( max(0.0, _x), double(width-1));
  double y = min( max(0.0, _y), double(height-1));

  // -2 makes for appropriate behavior on the upper boundary
  double x0 = std::min( std::max(0.0, floor(x)), double(width-2));
  double x1 = x0+1;

  double y0 = std::min( std::max(0.0, floor(y)), double(height-2));
  double y1 = y0+1;

  double wx0 = x1-x;
  double wx1 = x-x0;

  double wy0 = y1-y;
  double wy1 = y-y0;

  GaussianMapCell toReturn;

  // wx0*wy0*val(x0,y0) + wx1*wy0*val(x1,y0) + wx1*wy1*val(x1,y1) + wx0*wy1*val(x0,y1) ~sub 1 for vals =~
  // wx0*(wy0+wy1) + wx1*(wy0 + wy1) = wx0 + wx1 = 1

  BILIN_MAPCELL(rcounts);
  BILIN_MAPCELL(gcounts);
  BILIN_MAPCELL(bcounts);
  BILIN_MAPCELL(rsquaredcounts);
  BILIN_MAPCELL(gsquaredcounts);
  BILIN_MAPCELL(bsquaredcounts);
  BILIN_MAPCELL(rmu);
  BILIN_MAPCELL(gmu);
  BILIN_MAPCELL(bmu);
  BILIN_MAPCELL(rsigmasquared);
  BILIN_MAPCELL(gsigmasquared);
  BILIN_MAPCELL(bsigmasquared);
  BILIN_MAPCELL(rgbsamples);
  BILIN_MAPCELL(zcounts);
  BILIN_MAPCELL(zsquaredcounts);
  BILIN_MAPCELL(zmu);
  BILIN_MAPCELL(zsigmasquared);
  BILIN_MAPCELL(zsamples);

  return toReturn;
}

GaussianMapCell GaussianMap::bilinValAtMeters(double x, double y) {
  int cell_x;
  int cell_y;
  metersToCell(x, y, &cell_x, &cell_y);
  return bilinValAtCell(cell_x, cell_y);
}

void GaussianMap::metersToCell(double xm, double ym, int * xc, int * yc) {
  (*xc) = round(xm / cell_width) + x_center_cell;
  (*yc) = round(ym / cell_width) + y_center_cell;
} 

void GaussianMap::cellToMeters(int xc, int yc, double * xm, double * ym) {
  (*xm) = (xc - x_center_cell) * cell_width; 
  (*ym) = (yc - y_center_cell) * cell_width; 
} 

void GaussianMap::writeToFileStorage(FileStorage& fsvO) {
  fsvO << "{";
  {
    fsvO << "width" << width;
    fsvO << "height" << height;
    fsvO << "x_center_cell" << x_center_cell;
    fsvO << "y_center_cell" << y_center_cell;
    fsvO << "cell_width" << cell_width;

    fsvO << "cells" << "[" ;

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
	refAtCell(x,y)->writeToFileStorage(fsvO);
      }
    }
    fsvO << "]";
  }
  fsvO << "}";
}

void GaussianMap::saveToFile(string filename) {
  FileStorage fsvO;
  cout << "GaussianMap::saveToFile writing: " << filename << endl;
  fsvO.open(filename, FileStorage::WRITE);
  fsvO << "GaussianMap";
  writeToFileStorage(fsvO);
  fsvO.release();
}

void GaussianMap::readFromFileNodeIterator(FileNodeIterator& it) {
  {
    (*it)["width"] >> width;
    (*it)["height"] >> height;
    (*it)["x_center_cell"] >> x_center_cell;
    (*it)["y_center_cell"] >> y_center_cell;
    (*it)["cell_width"] >> cell_width;
  }
  reallocate();
  {
    FileNode bnode = (*it)["cells"];

    int numLoadedCells= 0;
    FileNodeIterator itc = bnode.begin(), itc_end = bnode.end();
    for ( ; (itc != itc_end) && (numLoadedCells < width*height); itc++, numLoadedCells++) {
      cells[numLoadedCells].readFromFileNodeIterator(itc);
    }

    if (numLoadedCells != width*height) {
      ROS_ERROR_STREAM("Error, GaussianMap loaded " << numLoadedCells << " but expected " << width*height << endl);
    } else {
      cout << "successfully loaded " << numLoadedCells << " GaussianMapCells." << endl;
    }
  }
}

void GaussianMap::readFromFileNode(FileNode& it) {
  {
    it["width"] >> width;
    it["height"] >> height;
    it["x_center_cell"] >> x_center_cell;
    it["y_center_cell"] >> y_center_cell;
    it["cell_width"] >> cell_width;
  }
  reallocate();
  {
    FileNode bnode = it["cells"];

    int numLoadedCells= 0;
    FileNodeIterator itc = bnode.begin(), itc_end = bnode.end();
    for ( ; (itc != itc_end) && (numLoadedCells < width*height); itc++, numLoadedCells++) {
      cells[numLoadedCells].readFromFileNodeIterator(itc);
    }

    if (numLoadedCells != width*height) {
      ROS_ERROR_STREAM("Error, GaussianMap loaded " << numLoadedCells << " but expected " << width*height << endl);
    } else {
      cout << "successfully loaded " << numLoadedCells << " GaussianMapCells." << endl;
    }
  }
}

void GaussianMap::loadFromFile(string filename) {
  FileStorage fsvI;
  cout << "GaussianMap::loadFromFile reading: " << filename<< " ..." << endl;
  fsvI.open(filename, FileStorage::READ);

  FileNode anode = fsvI["GaussianMap"];
  readFromFileNode(anode);

  cout << "done." << endl;
}

#define GAUSSIAN_MAP_UPDATE_MU_SIGMA(para, samples) \
refAtCell(x,y)->para ## mu = refAtCell(x,y)->para ## counts / samples; \
refAtCell(x,y)->para ## sigmasquared = (refAtCell(x,y)->para ## squaredcounts / samples) - ((refAtCell(x,y)->para ## mu) * (refAtCell(x,y)->para ## mu)); 

void GaussianMap::recalculateMusAndSigmas() {
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      double safe_rgbsamples = max(refAtCell(x,y)->rgbsamples, 1.0);
      double safe_zsamples = max(refAtCell(x,y)->zsamples, 1.0);

      GAUSSIAN_MAP_UPDATE_MU_SIGMA(z, safe_zsamples);
      GAUSSIAN_MAP_UPDATE_MU_SIGMA(r, safe_rgbsamples);
      GAUSSIAN_MAP_UPDATE_MU_SIGMA(g, safe_rgbsamples);
      GAUSSIAN_MAP_UPDATE_MU_SIGMA(b, safe_rgbsamples);
    }
  }
}

void GaussianMap::rgbMuToMat(Mat& out) {
  out = Mat(height, width, CV_64FC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<Vec3d>(y,x)[0] = refAtCell(x,y)->bmu;
      out.at<Vec3d>(y,x)[1] = refAtCell(x,y)->gmu;
      out.at<Vec3d>(y,x)[2] = refAtCell(x,y)->rmu;
    }
  }
}

void GaussianMap::rgbSigmaSquaredToMat(Mat& out) {
  out = Mat(height, width, CV_64FC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<Vec3d>(y,x)[0] = refAtCell(x,y)->bsigmasquared;
      out.at<Vec3d>(y,x)[1] = refAtCell(x,y)->gsigmasquared;
      out.at<Vec3d>(y,x)[2] = refAtCell(x,y)->rsigmasquared;
    }
  }
}

void GaussianMap::rgbCountsToMat(Mat& out) {
  out = Mat(height, width, CV_64FC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<Vec3d>(y,x)[0] = refAtCell(x,y)->bcounts;
      out.at<Vec3d>(y,x)[1] = refAtCell(x,y)->gcounts;
      out.at<Vec3d>(y,x)[2] = refAtCell(x,y)->rcounts;
    }
  }
}

void GaussianMap::zMuToMat(Mat& out) {
  out = Mat(height, width, CV_64F);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<double>(y,x) = refAtCell(x,y)->zmu;
    }
  }
}

void GaussianMap::zSigmaSquaredToMat(Mat& out) {
  out = Mat(height, width, CV_64F);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<double>(y,x) = refAtCell(x,y)->zsigmasquared;
    }
  }
}

void GaussianMap::zCountsToMat(Mat& out) {
  out = Mat(height, width, CV_64F);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<double>(y,x) = refAtCell(x,y)->zcounts;
    }
  }
}


// extract a bounding box at specified corners top left (x1,y1) bottom right (x2,y2)
shared_ptr<GaussianMap> GaussianMap::copyBox(int _x1, int _y1, int _x2, int _y2) {
  int x1 = min(_x1, _x2);
  int x2 = max(_x1, _x2);
  int y1 = min(_y1, _y2);
  int y2 = max(_y1, _y2);

  x1 = min( max(0,x1), width-1);
  x2 = min( max(0,x2), width-1);
  y1 = min( max(0,y1), height-1);
  y2 = min( max(0,y2), height-1);

  shared_ptr<GaussianMap> toReturn = std::make_shared<GaussianMap>(x2-x1, y2-y1, cell_width);
  for (int y = y1; y <= y2; y++) {
    for (int x = x1; x <= x2; x++) {
      *(toReturn->refAtCell(x-x1,y-y1)) = *(refAtCell(x,y));
    }
  }
  
  return toReturn;
}

void GaussianMap::zeroBox(int _x1, int _y1, int _x2, int _y2) {
  int x1 = min( max(0,x1), width-1);
  int x2 = min( max(0,x2), width-1);
  int y1 = min( max(0,y1), height-1);
  int y2 = min( max(0,y2), height-1);
  
  for (int y = y1; y <= y2; y++) {
    for (int x = x1; x <= x2; x++) {
      refAtCell(x,y)->zero();
    }
  }
}

void GaussianMap::zero() {
  zeroBox(0,0,width-1,height-1);
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

// XXX Scene Viewer
// Observed    Predicted    Segmentation
// Background  Discrepancy  Discrepancy Density
// update with words


// XXX accessors for sceneObjects: number, pose...

// XXX word to add objects to scene until evidence is accounted for

// XXX words to save and load current background and current object maps
// XXX word to remove object from scene and zero region around it
// XXX word to zero entire observedMap
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




