#include "gaussian_map.h"
#include "ein_words.h"
#include "ein.h"
#include "qtgui/einwindow.h"


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

void _GaussianMapCell::newObservation(Vec3b vec) {
  rcounts += vec[2];
  gcounts += vec[1];
  bcounts += vec[0];
  rsquaredcounts += pow(vec[2], 2);
  gsquaredcounts += pow(vec[1], 2);
  bsquaredcounts += pow(vec[0], 2);
  rgbsamples += 1;
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
  //out = Mat(height, width, CV_64FC3);
  Mat big = Mat(height, width, CV_8UC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {

      if (refAtCell(x, y)->rgbsamples > 0) {
	big.at<Vec3b>(y,x)[0] = uchar(refAtCell(x,y)->bmu);
	big.at<Vec3b>(y,x)[1] = uchar(refAtCell(x,y)->gmu);
	big.at<Vec3b>(y,x)[2] = uchar(refAtCell(x,y)->rmu);
      } else {
	big.at<Vec3b>(y,x)[0] = 0;
	big.at<Vec3b>(y,x)[1] = 128;
	big.at<Vec3b>(y,x)[2] = 128;
      }
    }
  }

  cv::resize(big, out, cv::Size(301, 301), 2, 2);

}

void GaussianMap::rgbDiscrepancyMuToMat(Mat& out) {
  //out = Mat(height, width, CV_64FC3);
  out = Mat(height, width, CV_8UC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {

      if (refAtCell(x, y)->rgbsamples > 0) {
	out.at<Vec3b>(y,x)[0] = fabs(refAtCell(x,y)->bmu);
	out.at<Vec3b>(y,x)[1] = refAtCell(x,y)->gmu * 0.5 + 128;
	out.at<Vec3b>(y,x)[2] = refAtCell(x,y)->rmu * 0.5 + 128;
      } else {
	out.at<Vec3b>(y,x)[0] = 0;
	out.at<Vec3b>(y,x)[1] = 128;
	out.at<Vec3b>(y,x)[2] = 128;
      }
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
  int x1 = min( max(0,_x1), width-1);
  int x2 = min( max(0,_x2), width-1);
  int y1 = min( max(0,_y1), height-1);
  int y2 = min( max(0,_y2), height-1);
  
  for (int y = y1; y <= y2; y++) {
    for (int x = x1; x <= x2; x++) {
      refAtCell(x,y)->zero();
    }
  }
}

void GaussianMap::zero() {
  zeroBox(0,0,width-1,height-1);
}



void SceneObject::writeToFileStorage(FileStorage& fsvO) {
// XXX
}

void SceneObject::readFromFileNodeIterator(FileNodeIterator& it) {
// XXX
}

void SceneObject::readFromFileNode(FileNode& it) {
// XXX
}




Scene::Scene(shared_ptr<MachineState> _ms, int w, int h, double cw) {
  ms = _ms;
  width = w;
  height = h;
  x_center_cell = (width-1)/2;
  y_center_cell = (height-1)/2;
  cell_width = cw;
  background_pose = eePose::zero();
  score = 0;
  predicted_objects.resize(0);
  reallocate();
}

void Scene::reallocate() {
  background_map = make_shared<GaussianMap>(width, height, cell_width);
  predicted_map = make_shared<GaussianMap>(width, height, cell_width);
  predicted_segmentation = Mat(height, width, CV_64F);
  observed_map = make_shared<GaussianMap>(width, height, cell_width);
  discrepancy = make_shared<GaussianMap>(width, height, cell_width);

  discrepancy_magnitude = Mat(height, width, CV_64F);
  discrepancy_density = Mat(height, width, CV_64F);
}

void Scene::composePredictedMap() {
  // choose the argMAP distribution
  //   assign that color to the predicted map
  //   assign the source to the segmentation
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      *(predicted_map->refAtCell(x,y)) = *(background_map->refAtCell(x,y));
    }
  }
  // XXX  currently only incorporates background
}

void Scene::measureDiscrepancy() {
  // close to kl-divergence
  // for now this only does rgb
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      if ((predicted_map->refAtCell(x,y)->rgbsamples > 0) && (observed_map->refAtCell(x,y)->rgbsamples > 0)) {

	double rmu_diff = (predicted_map->refAtCell(x,y)->rmu - observed_map->refAtCell(x,y)->rmu);
	double gmu_diff = (predicted_map->refAtCell(x,y)->gmu - observed_map->refAtCell(x,y)->gmu);
	double bmu_diff = (predicted_map->refAtCell(x,y)->bmu - observed_map->refAtCell(x,y)->bmu);
	
	double oos = 1.0 / predicted_map->refAtCell(x,y)->rgbsamples;

	double rd_observed = max(oos, observed_map->refAtCell(x,y)->rsigmasquared);
	double rd_predicted = max(oos, predicted_map->refAtCell(x,y)->rsigmasquared);
	double rvar_quot = (predicted_map->refAtCell(x,y)->rsigmasquared / rd_observed) + 
			   (observed_map->refAtCell(x,y)->rsigmasquared / rd_predicted);

	double gd_observed = max(oos, observed_map->refAtCell(x,y)->gsigmasquared);
	double gd_predicted = max(oos, predicted_map->refAtCell(x,y)->gsigmasquared);
	double gvar_quot = (predicted_map->refAtCell(x,y)->gsigmasquared / gd_observed) + 
			   (observed_map->refAtCell(x,y)->gsigmasquared / gd_predicted);

	double bd_observed = max(oos, observed_map->refAtCell(x,y)->bsigmasquared);
	double bd_predicted = max(oos, predicted_map->refAtCell(x,y)->bsigmasquared);
	double bvar_quot = (predicted_map->refAtCell(x,y)->bsigmasquared / bd_observed) + 
			   (observed_map->refAtCell(x,y)->bsigmasquared / bd_predicted);

	discrepancy->refAtCell(x,y)->rgbsamples = observed_map->refAtCell(x,y)->rgbsamples;
	discrepancy->refAtCell(x,y)->rmu = rmu_diff;
	discrepancy->refAtCell(x,y)->gmu = gmu_diff;
	discrepancy->refAtCell(x,y)->bmu = bmu_diff;
	discrepancy->refAtCell(x,y)->rsigmasquared = rvar_quot;
	discrepancy->refAtCell(x,y)->gsigmasquared = gvar_quot;
	discrepancy->refAtCell(x,y)->bsigmasquared = bvar_quot;
  
	discrepancy_magnitude.at<double>(y,x) = rmu_diff*rmu_diff + gmu_diff*gmu_diff + bmu_diff*bmu_diff ;
	//+ rvar_quot + gvar_quot + bvar_quot;

	discrepancy_density.at<double>(y,x) = sqrt(discrepancy_magnitude.at<double>(y,x) / 3.0) / 255.0; 

      } else {
	discrepancy->refAtCell(x,y)->rgbsamples = 0.0;
	discrepancy->refAtCell(x,y)->rmu = 0.0;
	discrepancy->refAtCell(x,y)->gmu = 0.0;
	discrepancy->refAtCell(x,y)->bmu = 0.0;
	discrepancy->refAtCell(x,y)->rsigmasquared = 0.0;
	discrepancy->refAtCell(x,y)->gsigmasquared = 0.0;
	discrepancy->refAtCell(x,y)->bsigmasquared = 0.0;
  
	discrepancy_magnitude.at<double>(y,x) = 0.0;
	discrepancy_density.at<double>(y,x) = 0.0;
      }
    }
  }
}

double Scene::assignScore() {
  score = measureScoreRegion(0,0,width-1,height-1);
  return score;
}

double Scene::measureScoreRegion(int _x1, int _y1, int _x2, int _y2) {
  int x1 = min(_x1, _x2);
  int x2 = max(_x1, _x2);
  int y1 = min(_y1, _y2);
  int y2 = max(_y1, _y2);

  x1 = min( max(0,x1), width-1);
  x2 = min( max(0,x2), width-1);
  y1 = min( max(0,y1), height-1);
  y2 = min( max(0,y2), height-1);

  double totalScore = 0.0;
  for (int y = y1; y <= y2; y++) {
    for (int x = x1; x <= x2; x++) {
      if (discrepancy->refAtCell(x,y)->rgbsamples > 0) {
	totalScore += discrepancy_magnitude.at<double>(y,x); 
      } else {
      }
    }
  }
  
  return totalScore;
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
void Scene::removeObjectFromPredictedMap() {
}

// XXX 
void Scene::addObjectToPredictedMap() {
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

void Scene::writeToFileStorage(FileStorage& fsvO) {
// XXX 
}

void Scene::readFromFileNodeIterator(FileNodeIterator& it) {
// XXX 
}

void Scene::readFromFileNode(FileNode& it) {
// XXX
}

void Scene::saveToFile(string filename) {
// XXX
}

void Scene::loadFromFile(string filename) {
// XXX
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

void TransitionTable::writeToFileStorage(FileStorage& fsvO) {
// XXX 
}

void TransitionTable::readFromFileNodeIterator(FileNodeIterator& it) {
// XXX 
}

void TransitionTable::readFromFileNode(FileNode& it) {
// XXX
}

void TransitionTable::saveToFile(string filename) {
// XXX
}

void TransitionTable::loadFromFile(string filename) {
// XXX
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
// XXX init checker MACRO for Scene and TransitionTable 
*/





namespace ein_words {

WORD(SceneSaveBackgroundMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string message;
  GET_STRING_ARG(ms, message);

  stringstream ss;
  ss << ms->config.data_directory + "/maps/" + message + ".yml";
  stringstream ss_dir;
  ss_dir << ms->config.data_directory + "/maps/";
  mkdir(ss_dir.str().c_str(), 0777);

  ms->config.scene->background_map->saveToFile(ss.str());

}
END_WORD
REGISTER_WORD(SceneSaveBackgroundMap)

WORD(SceneLoadBackgroundMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string message;
  GET_STRING_ARG(ms, message);

  stringstream ss;
  ss << ms->config.data_directory + "/maps/" + message + ".yml";
  stringstream ss_dir;
  ss_dir << ms->config.data_directory + "/maps/";
  mkdir(ss_dir.str().c_str(), 0777);

  ms->config.scene->background_map->loadFromFile(ss.str());

  Mat backgroundImage;
  ms->config.scene->background_map->rgbMuToMat(backgroundImage);
  Mat rgb = backgroundImage.clone();  
  cvtColor(backgroundImage, rgb, CV_YCrCb2BGR);
  ms->config.backgroundWindow->updateImage(rgb);

}
END_WORD
REGISTER_WORD(SceneLoadBackgroundMap)

WORD(SceneSaveObservedMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string message;
  GET_STRING_ARG(ms, message);

  stringstream ss;
  ss << ms->config.data_directory + "/maps/" + message + ".yml";
  stringstream ss_dir;
  ss_dir << ms->config.data_directory + "/maps/";
  mkdir(ss_dir.str().c_str(), 0777);

  ms->config.scene->observed_map->saveToFile(ss.str());

}
END_WORD
REGISTER_WORD(SceneSaveObservedMap)

WORD(SceneLoadObservedMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string message;
  GET_STRING_ARG(ms, message);

  stringstream ss;
  ss << ms->config.data_directory + "/maps/" + message + ".yml";
  stringstream ss_dir;
  ss_dir << ms->config.data_directory + "/maps/";
  mkdir(ss_dir.str().c_str(), 0777);

  ms->config.scene->observed_map->loadFromFile(ss.str());

}
END_WORD
REGISTER_WORD(SceneLoadObservedMap)

WORD(SceneSaveDiscrepancyMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string message;
  GET_STRING_ARG(ms, message);

  stringstream ss;
  ss << ms->config.data_directory + "/maps/" + message + ".yml";
  stringstream ss_dir;
  ss_dir << ms->config.data_directory + "/maps/";
  mkdir(ss_dir.str().c_str(), 0777);

  ms->config.scene->discrepancy->saveToFile(ss.str());
}
END_WORD
REGISTER_WORD(SceneSaveDiscrepancyMap)

WORD(SceneLoadDiscrepancyMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string message;
  GET_STRING_ARG(ms, message);

  stringstream ss;
  ss << ms->config.data_directory + "/maps/" + message + ".yml";
  stringstream ss_dir;
  ss_dir << ms->config.data_directory + "/maps/";
  mkdir(ss_dir.str().c_str(), 0777);

  ms->config.scene->discrepancy->loadFromFile(ss.str());
}
END_WORD
REGISTER_WORD(SceneLoadDiscrepancyMap)

WORD(SceneClearPredictedObjects)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.scene->predicted_objects.resize(0);
}
END_WORD
REGISTER_WORD(SceneClearPredictedObjects)

WORD(SceneInit)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double p_cell_width = 0.005; //0.01;
  int p_width = 601;
  int p_height = 601;
  ms->config.scene = make_shared<Scene>(ms, p_width, p_height, p_cell_width);
}
END_WORD
REGISTER_WORD(SceneInit)

WORD(SceneClearObservedMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.scene->observed_map->zero();
  Mat observedImage;
  ms->config.scene->observed_map->rgbMuToMat(observedImage);
  //observedImage = observedImage / 255.0;
  Mat rgb = observedImage.clone();  
  cvtColor(observedImage, rgb, CV_YCrCb2BGR);
  ms->config.observedWindow->updateImage(rgb);
}
END_WORD
REGISTER_WORD(SceneClearObservedMap)

WORD(SceneSetBackgroundFromObserved)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.scene->background_map->zero();
}
END_WORD
REGISTER_WORD(SceneSetBackgroundFromObserved)

WORD(SceneUpdateObservedFromSnout)
virtual void execute(std::shared_ptr<MachineState> ms) {
//XXX 
}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromSnout)

WORD(SceneUpdateObservedFromWrist)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Mat wristViewYCbCr = ms->config.wristCamImage.clone();  
  cvtColor(ms->config.wristCamImage, wristViewYCbCr, CV_BGR2YCrCb);
    
  for (int px = ms->config.grayTop.x+ms->config.mapGrayBoxPixelSkirtCols; px < ms->config.grayBot.x-ms->config.mapGrayBoxPixelSkirtCols; px++) {
    for (int py = ms->config.grayTop.y+ms->config.mapGrayBoxPixelSkirtRows; py < ms->config.grayBot.y-ms->config.mapGrayBoxPixelSkirtRows; py++) {
      if (isInGripperMask(ms, px, py)) {
	continue;
      }
      double x, y;
      double z = ms->config.trueEEPose.position.z + ms->config.currentTableZ;
      pixelToGlobal(ms, px, py, z, &x, &y);
      int i, j;
      ms->config.scene->observed_map->metersToCell(x, y, &i, &j);
      GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(i, j);
      Vec3b pixel = wristViewYCbCr.at<Vec3b>(py, px);
      cell->newObservation(pixel);
    }
  }  
  ms->config.scene->observed_map->recalculateMusAndSigmas();
  Mat image;
  ms->config.scene->observed_map->rgbMuToMat(image);
  Mat rgb = image.clone();  
  cvtColor(image, rgb, CV_YCrCb2BGR);
  ms->config.observedWindow->updateImage(rgb);
}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromWrist)

WORD(SceneComposePredictedMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.scene->composePredictedMap();

  Mat image;
  ms->config.scene->predicted_map->rgbMuToMat(image);
  Mat rgb = image.clone();  
  cvtColor(image, rgb, CV_YCrCb2BGR);
  ms->config.predictedWindow->updateImage(rgb);
}
END_WORD
REGISTER_WORD(SceneComposePredictedMap)

WORD(SceneUpdateDiscrepancy)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.scene->measureDiscrepancy();
  Mat image;
  ms->config.scene->discrepancy->rgbDiscrepancyMuToMat(image);
  //image = image / 255.0;
  Mat rgb;  cvtColor(image, rgb, CV_YCrCb2BGR);
  ms->config.discrepancyWindow->updateImage(rgb);
}
END_WORD
REGISTER_WORD(SceneUpdateDiscrepancy)

WORD(SceneGrabCenterCropAsClass)
virtual void execute(std::shared_ptr<MachineState> ms) {
//XXX 
  //shared_ptr<GaussianMap> camera_frame = copyBox(ul_cell_x, ul_cell_y, br_cell_x, br_cell_y);
}
END_WORD
REGISTER_WORD(SceneGrabCenterCropAsClass)

WORD(SceneDensityFromDiscrepancy)
virtual void execute(std::shared_ptr<MachineState> ms) {
// this enables denisty based models to use the new channel
// XXX this does not take the rotation of the wrist into account
  Size sz = ms->config.wristCamImage.size();
  int imW = sz.width;
  int imH = sz.height;

  for (int y = 0; y < imH; y++) {
    for (int x = 0; x < imW; x++) {
      double meter_x = 0;
      double meter_y = 0;
      double zToUse = ms->config.currentEEPose.pz+ms->config.currentTableZ;
      pixelToGlobal(ms, x, y, zToUse, &meter_x, &meter_y, ms->config.currentEEPose);
      int cell_x = 0;
      int cell_y = 0;
      ms->config.scene->discrepancy->metersToCell(meter_x, meter_y, &cell_x, &cell_y);
      ms->config.density[y*imW+x] = ms->config.scene->discrepancy_density.at<double>(cell_y,cell_x);
    }
  }
  drawDensity(ms, 1);
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




