#include "gaussian_map.h"
#include "ein_words.h"
#include "ein.h"
#include "qtgui/einwindow.h"

void checkProb(string label, double prob) {
  cout << "Checking " << label << " " << prob << endl;
  if (prob > 1.0) {
    cout << label << " greater than 1: " << prob << endl;
  }
  if (prob < 0.0) {
    cout << label << " less than 0: " << prob << endl;
  }
}
void _GaussianMapChannel::zero() {
  counts = 0.0;
  squaredcounts = 0.0;
  mu = 0.0;
  sigmasquared = 0.0;
  samples = 0.0;
}
void _GaussianMapCell::zero() {
  red.zero();
  green.zero();
  blue.zero();
  z.zero();
}

double normal_pdf(double mu, double sigma, double x) {
  return 1 / (sigma * sqrt(2 * M_PI)) * exp(-pow(x - mu, 2) / (2 * sigma * sigma));
}


void computeInnerProduct(GaussianMapChannel & channel1, GaussianMapChannel & channel2, double * channel_term_out) {
  double ip_normalizer = 0.0;				       
  double newsigmasquared = 1 / (1 / channel1.sigmasquared + 1 / channel2.sigmasquared); 
  double newmu = newsigmasquared * (channel1.mu / channel1.sigmasquared + channel2.mu / channel2.sigmasquared); 
  for (int i = 0; i < 256; i++) {
    ip_normalizer += normal_pdf(newmu, sqrt(newsigmasquared), i);
  }
/*
  double channel_term = exp(  -0.5*pow(channel1.mu-channel2.mu, 2)/( channel1.sigmasquared + channel2.sigmasquared )  ) / sqrt( 2.0*M_PI*(channel1.sigmasquared + channel2.sigmasquared) ); 
  channel_term = channel_term * ip_normalizer * 0.5 * 256;
  *(channel_term_out) = channel_term;
*/

  double ip_val = exp(  -0.5*pow(channel1.mu-channel2.mu, 2)/( channel1.sigmasquared + channel2.sigmasquared )  ) / sqrt( 2.0*M_PI*(channel1.sigmasquared + channel2.sigmasquared) ); 
  double likelihood = ip_normalizer * ip_val;
  double prior = 0.5;
  double nb_normalizer = likelihood * prior +  (1.0/256.0) * (1 - prior);

  *(channel_term_out) = likelihood * prior / nb_normalizer; 
}

void _GaussianMapChannel::multS(double scalar) {
  counts *= scalar;
  squaredcounts *= scalar;
  mu *= scalar;
  sigmasquared *= scalar;
  samples *= scalar;
}  

void _GaussianMapChannel::addC(_GaussianMapChannel * channel) {
  counts += channel->counts;
  squaredcounts += channel->squaredcounts;
  mu += channel->mu;
  sigmasquared += channel->sigmasquared;
  samples += channel->samples;
}  

void _GaussianMapCell::multS(double scalar) {
  red.multS(scalar);
  green.multS(scalar);
  blue.multS(scalar);
}  

void _GaussianMapCell::addC(_GaussianMapCell * cell) {
  red.addC(&(cell->red));
  green.addC(&(cell->green));
  blue.addC(&(cell->blue));
}  

void GaussianMap::multS(double scalar) {
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      refAtCell(x,y)->multS(scalar);
    }
  }
}  

void GaussianMap::addM(shared_ptr<GaussianMap> map) {
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      refAtCell(x,y)->addC(map->refAtCell(x,y));
    }
  }
} 

double _GaussianMapCell::innerProduct(_GaussianMapCell * other, double * rterm_out, double * gterm_out, double * bterm_out) {
  computeInnerProduct(red, other->red, rterm_out);
  computeInnerProduct(green, other->green, gterm_out);
  computeInnerProduct(blue, other->blue, bterm_out);
  return *rterm_out * *bterm_out * *gterm_out;
}


void computePointDiscrepancy(GaussianMapChannel & channel1, GaussianMapChannel & channel2, double * channel_term_out) {
  double likelihood = normal_pdf(channel1.mu, sqrt(channel1.sigmasquared), channel2.mu);
  double prior = 0.5;
  double normalizer = likelihood * prior +  (1.0/256.0) * (1 - prior);
  *channel_term_out = likelihood * prior / normalizer;
}
double _GaussianMapCell::pointDiscrepancy(_GaussianMapCell * other, double * rterm_out, double * gterm_out, double * bterm_out) {
  computePointDiscrepancy(red, other->red, rterm_out);
  computePointDiscrepancy(green, other->green, gterm_out);
  computePointDiscrepancy(blue, other->blue, bterm_out);
  //return *rterm_out * *bterm_out * *gterm_out;
  double prior = 0.5;
  double rlikelihood = normal_pdf(red.mu, sqrt(red.sigmasquared), other->red.mu);
  double glikelihood = normal_pdf(green.mu, sqrt(green.sigmasquared), other->green.mu);
  double blikelihood = normal_pdf(blue.mu, sqrt(blue.sigmasquared), other->blue.mu);

  double likelihood = rlikelihood * glikelihood * blikelihood;

  double normalizer = likelihood * prior + pow(1.0/256, 3) * (1 - prior);
  return likelihood * prior / normalizer;
}



void _GaussianMapCell::writeToFileStorage(FileStorage& fsvO) const {

  fsvO << "{:";
  fsvO << "rcounts" << red.counts;
  fsvO << "gcounts" << green.counts;
  fsvO << "bcounts" << blue.counts;
  fsvO << "rsquaredcounts" << red.squaredcounts;
  fsvO << "gsquaredcounts" << green.squaredcounts;
  fsvO << "bsquaredcounts" << blue.squaredcounts;
  fsvO << "rmu" << red.mu;
  fsvO << "gmu" << green.mu;
  fsvO << "bmu" << blue.mu;
  fsvO << "rsigmasquared" << red.sigmasquared;
  fsvO << "gsigmasquared" << green.sigmasquared;
  fsvO << "bsigmasquared" << blue.sigmasquared;
  fsvO << "rsamples" << red.samples;
  fsvO << "gsamples" << green.samples;
  fsvO << "bsamples" << blue.samples;
  fsvO << "zcounts" << z.counts;
  fsvO << "zsquaredcounts" << z.squaredcounts;
  fsvO << "zmu" << z.mu;
  fsvO << "zsigmasquared" << z.sigmasquared;
  fsvO << "zsamples" << z.samples;
  fsvO << "}";
}

void _GaussianMapCell::readFromFileNodeIterator(FileNodeIterator& it) {
  FileNode node = *it;
  readFromFileNode(node);
}
void _GaussianMapCell::readFromFileNode(FileNode& it) {
  red.counts         =  (double)(it)["rcounts"];         
  green.counts         =  (double)(it)["gcounts"];         
  blue.counts         =  (double)(it)["bcounts"];         
  red.squaredcounts  =  (double)(it)["rsquaredcounts"];                
  green.squaredcounts  =  (double)(it)["gsquaredcounts"];                
  blue.squaredcounts  =  (double)(it)["bsquaredcounts"];                
  red.mu             =  (double)(it)["rmu"];     
  green.mu             =  (double)(it)["gmu"];     
  blue.mu             =  (double)(it)["bmu"];     
  red.sigmasquared   =  (double)(it)["rsigmasquared"];               
  green.sigmasquared   =  (double)(it)["gsigmasquared"];               
  blue.sigmasquared   =  (double)(it)["bsigmasquared"];               
  red.samples      =  (double)(it)["rgbsamples"];
  green.samples      =  (double)(it)["rgbsamples"];            
  blue.samples      =  (double)(it)["rgbsamples"];            
  z.counts         =  (double)(it)["zcounts"];         
  z.squaredcounts  =  (double)(it)["zsquaredcounts"];                
  z.mu             =  (double)(it)["zmu"];     
  z.sigmasquared   =  (double)(it)["zsigmasquared"];               
  z.samples        =  (double)(it)["zsamples"];          
}

void _GaussianMapCell::newObservation(Vec3b vec) {
  red.counts += vec[2];
  green.counts += vec[1];
  blue.counts += vec[0];
  red.squaredcounts += pow(vec[2], 2);
  green.squaredcounts += pow(vec[1], 2);
  blue.squaredcounts += pow(vec[0], 2);
  red.samples += 1;
  green.samples += 1;
  blue.samples += 1;
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
  if (x < 0 || x >= width) {
    ROS_ERROR_STREAM("Bad x. " << x);
    assert (0);
  }
  if (y < 0 || y >= height) {
    ROS_ERROR_STREAM("Bad y. " << y);
    assert (0);
  }
  
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

  BILIN_MAPCELL(red.counts);
  BILIN_MAPCELL(green.counts);
  BILIN_MAPCELL(blue.counts);
  BILIN_MAPCELL(red.squaredcounts);
  BILIN_MAPCELL(green.squaredcounts);
  BILIN_MAPCELL(blue.squaredcounts);
  BILIN_MAPCELL(red.mu);
  BILIN_MAPCELL(green.mu);
  BILIN_MAPCELL(blue.mu);
  BILIN_MAPCELL(red.sigmasquared);
  BILIN_MAPCELL(green.sigmasquared);
  BILIN_MAPCELL(blue.sigmasquared);
  BILIN_MAPCELL(red.samples);
  BILIN_MAPCELL(green.samples);
  BILIN_MAPCELL(blue.samples);
  BILIN_MAPCELL(z.counts);
  BILIN_MAPCELL(z.squaredcounts);
  BILIN_MAPCELL(z.mu);
  BILIN_MAPCELL(z.sigmasquared);
  BILIN_MAPCELL(z.samples);

  return toReturn;
}

GaussianMapCell GaussianMap::bilinValAtMeters(double x, double y) {
  double cell_x;
  double cell_y;
  metersToCell(x, y, &cell_x, &cell_y);
  return bilinValAtCell(cell_x, cell_y);
}

void GaussianMap::metersToCell(double xm, double ym, int * xc, int * yc) {
  (*xc) = round(xm / cell_width) + x_center_cell;
  (*yc) = round(ym / cell_width) + y_center_cell;
} 

void GaussianMap::metersToCell(double xm, double ym, double * xc, double * yc) {
  (*xc) = (xm / cell_width) + x_center_cell;
  (*yc) = (ym / cell_width) + y_center_cell;
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

    fsvO << "cells";
    writeCells(fsvO);
    //fsvO << "cells" << "[" ;

    //for (int y = 0; y < height; y++) {
    //  for (int x = 0; x < width; x++) {
    //refAtCell(x,y)->writeToFileStorage(fsvO);
    //}
    //}
    //fsvO << "]";
  }
  fsvO << "}";
}

void GaussianMap::writeCells(FileStorage & fsvO) {

  unsigned char * data = (unsigned char *) cells;
  int length = sizeof(GaussianMapCell) * width * height;
  writeBinaryToYaml(data, length, fsvO);
  

  /*fsvO << "[:";
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      GaussianMapCell * cell = refAtCell(x, y);
      unsigned char * data  = (unsigned char * ) refAtCell(x, y);
      int length = sizeof(GaussianMapCell);
      string result = base64_encode(data, length);
      fsvO << result;

    }
  }
  fsvO << "]";*/
  
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
  FileNode node = *it;
  readFromFileNode(node);
}

void GaussianMap::readFromFileNode(FileNode& it) {
  {
    it["width"] >> width;
    it["height"] >> height;
    it["x_center_cell"] >> x_center_cell;
    it["y_center_cell"] >> y_center_cell;
    it["cell_width"] >> cell_width;
  }
  cout << "width: " << width << " height: " << height << endl;
  reallocate();
  {
    FileNode bnode = it["cells"];

    string stringdata = readBinaryFromYaml(bnode);
    GaussianMapCell * data = (GaussianMapCell * ) stringdata.data();
    int numLoadedCells = stringdata.size() / sizeof(GaussianMapCell);
    if (stringdata.size() != width * height * sizeof(GaussianMapCell)) {
      ROS_ERROR_STREAM("Inconsistency in saved data.");
      ROS_ERROR_STREAM("Read width: " << width << " height: " << height << " but got " << stringdata.size() << " from the file.");
    } else  {
      memcpy(cells, data, sizeof(GaussianMapCell) * width * height);
    }
      
    

    /*int numLoadedCells= 0;
    FileNodeIterator itc = bnode.begin(), itc_end = bnode.end();
    for ( ; (itc != itc_end) && (numLoadedCells < width*height); itc++, numLoadedCells++) {
      string encoded_data = (string) (*itc);
      string decoded_data = base64_decode(encoded_data);
      GaussianMapCell * data = (GaussianMapCell * ) decoded_data.data();
      
      memcpy(&cells[numLoadedCells], data, sizeof(GaussianMapCell));
      
      }*/

    if (numLoadedCells != width*height) {
      ROS_ERROR_STREAM("Error, GaussianMap loaded " << numLoadedCells << " but expected " << width*height << endl);
    } else {
      cout << "successfully loaded " << numLoadedCells << " GaussianMapCells." << endl;
    }
  }
}

void GaussianMap::loadFromFile(string filename) {
  FileStorage fsvI;
  cout << "GaussianMap::loadFromFile reading: " << filename << " ..." << endl;
  fsvI.open(filename, FileStorage::READ);
  if (fsvI.isOpened()) {
    FileNode anode = fsvI["GaussianMap"];
    readFromFileNode(anode);
  } else {
    ROS_ERROR_STREAM("Could not open file " << filename);
  }
  cout << "done." << endl;
}


void GaussianMapChannel::recalculateMusAndSigmas() {
  double safe_samples = max(samples, 1.0);
  mu = counts / samples;			
  sigmasquared = (squaredcounts / samples) - (mu * mu); 
}

void GaussianMap::recalculateMusAndSigmas() {
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {

      refAtCell(x, y)->red.recalculateMusAndSigmas();
      refAtCell(x, y)->green.recalculateMusAndSigmas();
      refAtCell(x, y)->blue.recalculateMusAndSigmas();
      refAtCell(x, y)->z.recalculateMusAndSigmas();
      
    }
  }
}


void GaussianMap::rgbMuToMat(Mat& out) {
  //out = Mat(height, width, CV_64FC3);
  Mat big = Mat(height, width, CV_8UC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {

      if (refAtCell(x, y)->red.samples > 0) {
	big.at<Vec3b>(y,x)[0] = uchar(refAtCell(x,y)->blue.mu);
	big.at<Vec3b>(y,x)[1] = uchar(refAtCell(x,y)->green.mu);
	big.at<Vec3b>(y,x)[2] = uchar(refAtCell(x,y)->red.mu);
      } else {
	big.at<Vec3b>(y,x)[0] = 0;
	big.at<Vec3b>(y,x)[1] = 128;
	big.at<Vec3b>(y,x)[2] = 128;
      }
    }
  }

  //cv::resize(big, out, cv::Size(301, 301), 2, 2);
  out = big;
}

void GaussianMap::rgbDiscrepancyMuToMat(Mat& out) {
  //out = Mat(height, width, CV_64FC3);
  out = Mat(height, width, CV_8UC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<Vec3b>(y,x)[0] = refAtCell(x,y)->blue.mu;
      out.at<Vec3b>(y,x)[1] = refAtCell(x,y)->green.mu;
      out.at<Vec3b>(y,x)[2] = refAtCell(x,y)->red.mu;
    }
  }
}


void GaussianMap::rgbSigmaSquaredToMat(Mat& out) {
  out = Mat(height, width, CV_64FC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<Vec3d>(y,x)[0] = refAtCell(x,y)->blue.sigmasquared;
      out.at<Vec3d>(y,x)[1] = refAtCell(x,y)->green.sigmasquared;
      out.at<Vec3d>(y,x)[2] = refAtCell(x,y)->red.sigmasquared;
    }
  }
}

void GaussianMap::rgbCountsToMat(Mat& out) {
  out = Mat(height, width, CV_64FC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<Vec3d>(y,x)[0] = refAtCell(x,y)->blue.counts;
      out.at<Vec3d>(y,x)[1] = refAtCell(x,y)->green.counts;
      out.at<Vec3d>(y,x)[2] = refAtCell(x,y)->red.counts;
    }
  }
}

void GaussianMap::zMuToMat(Mat& out) {
  out = Mat(height, width, CV_64F);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<double>(y,x) = refAtCell(x,y)->z.mu;
    }
  }
}

void GaussianMap::zSigmaSquaredToMat(Mat& out) {
  out = Mat(height, width, CV_64F);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<double>(y,x) = refAtCell(x,y)->z.sigmasquared;
    }
  }
}

void GaussianMap::zCountsToMat(Mat& out) {
  out = Mat(height, width, CV_64F);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<double>(y,x) = refAtCell(x,y)->z.counts;
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
  for (int y = y1; y < y2; y++) {
    for (int x = x1; x < x2; x++) {
      *(toReturn->refAtCell(x-x1,y-y1)) = *(refAtCell(x,y));
    }
  }
  
  return toReturn;
}

void GaussianMap::zeroBox(int _x1, int _y1, int _x2, int _y2) {
  int x1 = min(_x1, _x2);
  int x2 = max(_x1, _x2);
  int y1 = min(_y1, _y2);
  int y2 = max(_y1, _y2);

  x1 = min( max(0,x1), width-1);
  x2 = min( max(0,x2), width-1);
  y1 = min( max(0,y1), height-1);
  y2 = min( max(0,y2), height-1);

  for (int y = y1; y <= y2; y++) {
    for (int x = x1; x <= x2; x++) {
      refAtCell(x,y)->zero();
    }
  }
}

void GaussianMap::zero() {
  zeroBox(0,0,width-1,height-1);
}

string sceneObjectTypeToString(sceneObjectType sot) {
  if (sot == BACKGROUND) {
    return "background";
  } else if (sot == PREDICTED) {
    return "predicted";
  } else if (sot == SPACE) {
    return "space";
  } else {
    cout << "bad sot: " << sot << endl;
    assert(0);
  }
}

sceneObjectType sceneObjectTypeFromString(string str) {
  if (str == "background") {
    return BACKGROUND;
  } else if (str == "predicted") {
    return PREDICTED;
  } else if (str == "space") {
    return SPACE;
  } else {
    cout << "bad string: " << str << endl;
    assert(0);
  }
}


SceneObject::SceneObject(eePose _eep, int _lci, string _ol, sceneObjectType _sot) {
  scene_pose = _eep;
  labeled_class_index = _lci;
  object_label = _ol;
  sot = _sot;
}

SceneObject::SceneObject() {
  scene_pose = eePose::identity();
  labeled_class_index = -1;
  object_label = string("");
  sot = BACKGROUND;
}

void SceneObject::writeToFileStorage(FileStorage& fsvO) {
  fsvO << "{";
  fsvO << "scene_pose"; scene_pose.writeToFileStorage(fsvO);
  fsvO << "labeled_class_index" << labeled_class_index;
  fsvO << "object_label" << object_label;
  fsvO << "sceneObjectType" << sceneObjectTypeToString(sot);
  fsvO << "}";
}

void SceneObject::readFromFileNodeIterator(FileNodeIterator& it) {
  FileNode node = *it;
  readFromFileNode(node);
}

void SceneObject::readFromFileNode(FileNode& it) {
  FileNode p = (it)["scene_pose"];
  scene_pose.readFromFileNode(p);
  (it)["labeled_class_index"] >> labeled_class_index;
  (it)["object_label"] >> object_label;
  string sotString;
  
  (it)["sceneObjectType"] >> sotString;
  sot = sceneObjectTypeFromString(sotString);
  
}




Scene::Scene(shared_ptr<MachineState> _ms, int w, int h, double cw) {
  ms = _ms;
  width = w;
  height = h;
  x_center_cell = (width-1)/2;
  y_center_cell = (height-1)/2;
  cell_width = cw;
  background_pose = eePose::identity();
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

bool Scene::isDiscrepantCell(double threshold, int x, int y) {
  if (!safeAt(x,y)) {
    return false;
  } else {
    return (discrepancy_density.at<double>(y,x) > threshold);
  }
} 
bool Scene::isDiscrepantCellBilin(double threshold, double x, double y) {
  int x1 = floor(x);
  int x2 = x1+1;
  int y1 = floor(y);
  int y2 = y1+1;

  x1 = min( max(0,x1), width-1);
  x2 = min( max(0,x2), width-1);
  y1 = min( max(0,y1), height-1);
  y2 = min( max(0,y2), height-1);

  return ( (discrepancy_density.at<double>(y1,x1) > threshold) || 
	   (discrepancy_density.at<double>(y1,x2) > threshold) ||
	   (discrepancy_density.at<double>(y2,x1) > threshold) ||
	   (discrepancy_density.at<double>(y2,x2) > threshold) );
} 
bool Scene::isDiscrepantMetersBilin(double threshold, double x, double y) {
  double cell_x;
  double cell_y;
  metersToCell(x, y, &cell_x, &cell_y);
  return isDiscrepantCellBilin(threshold, cell_x, cell_y);
} 

void Scene::composePredictedMap(double threshold) {
  // XXX
  // choose the argMAP distribution
  //   assign that color to the predicted map
  //   assign the source to the segmentation
  //
  // Currently uses a "fallen leaves" model of composition, assuming objects
  //   are painted onto the scene in reverse order of discovery 
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      *(predicted_map->refAtCell(x,y)) = *(background_map->refAtCell(x,y));
    }
  }

  for (int i = predicted_objects.size()-1; i >= 0; i--) {
    shared_ptr<SceneObject> tsob = predicted_objects[i];
    shared_ptr<Scene> tos = ms->config.class_scene_models[ tsob->labeled_class_index ];

    int center_x, center_y;
    metersToCell(tsob->scene_pose.px, tsob->scene_pose.py, &center_x, &center_y);

    int mdim = max(tos->width, tos->height);
    int mpad = ceil(mdim*sqrt(2.0)/2.0);
    int top_x, top_y;
    metersToCell(tsob->scene_pose.px - mpad*cell_width, tsob->scene_pose.py - mpad*cell_width, &top_x, &top_y);
    int bot_x, bot_y;
    metersToCell(tsob->scene_pose.px + mpad*cell_width, tsob->scene_pose.py + mpad*cell_width, &bot_x, &bot_y);

    for (int x = top_x; x < bot_x; x++) {
      for (int y = top_y; y < bot_y; y++) {
	if (!safeAt(x,y)) {
	  continue;
	} 

	double meters_scene_x, meters_scene_y;
	cellToMeters(x, y, &meters_scene_x, &meters_scene_y);

	
	double meters_object_x, meters_object_y;
	eePose cell_eep = eePose::identity();
	cell_eep.px = meters_scene_x;
	cell_eep.py = meters_scene_y;
	cell_eep.pz = 0.0;
	eePose eep_object = cell_eep.getPoseRelativeTo(tsob->scene_pose);
	meters_object_x = eep_object.px;
	meters_object_y = eep_object.py;

	if (tos->isDiscrepantMetersBilin(threshold, meters_object_x, meters_object_y)) {
	  *(predicted_map->refAtCell(x,y)) = tos->observed_map->bilinValAtMeters(meters_object_x, meters_object_y);
	} else {
	}
	
	// take exaggerated bounding box of object in scene
	//   look up each scene cell in the object's frame
	//   if one of the contributors is valid, replace this scene cell with the interpolated object cell 
      }
    }

  }
}

void Scene::measureDiscrepancy() {
  // close to kl-divergence
  // for now this only does rgb
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      if ((predicted_map->refAtCell(x,y)->red.samples > 0) && (observed_map->refAtCell(x,y)->red.samples > 0)) {

/*
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

*/
	double rmu_diff = 0.0;
	double gmu_diff = 0.0;
	double bmu_diff = 0.0;

	//double total_discrepancy = predicted_map->refAtCell(x,y)->innerProduct(observed_map->refAtCell(x,y), &rmu_diff, &gmu_diff, &bmu_diff);
	//double total_discrepancy = predicted_map->refAtCell(x,y)->pointDiscrepancy(observed_map->refAtCell(x,y), &rmu_diff, &gmu_diff, &bmu_diff);

	double point_discrepancy = predicted_map->refAtCell(x,y)->pointDiscrepancy(observed_map->refAtCell(x,y), &rmu_diff, &gmu_diff, &bmu_diff);
	//predicted_map->refAtCell(x,y)->innerProduct(observed_map->refAtCell(x,y), &rmu_diff, &gmu_diff, &bmu_diff);
	double total_discrepancy = 0.0;

	//identitycheckProb("rmu_diff", rmu_diff);
	//identitycheckProb("bmu_diff", bmu_diff);
	//identitycheckProb("gmu_diff", gmu_diff);

	discrepancy->refAtCell(x,y)->red.samples = observed_map->refAtCell(x,y)->red.samples;
	discrepancy->refAtCell(x,y)->green.samples = observed_map->refAtCell(x,y)->green.samples;
	discrepancy->refAtCell(x,y)->blue.samples = observed_map->refAtCell(x,y)->blue.samples;
	discrepancy->refAtCell(x,y)->red.mu = (1 - rmu_diff); 
	discrepancy->refAtCell(x,y)->green.mu = (1 - gmu_diff); 
	discrepancy->refAtCell(x,y)->blue.mu = (1 - bmu_diff);
	discrepancy->refAtCell(x,y)->red.sigmasquared = 0;
	discrepancy->refAtCell(x,y)->green.sigmasquared = 0;
	discrepancy->refAtCell(x,y)->blue.sigmasquared = 0;
  
	//total_discrepancy = discrepancy->refAtCell(x,y)->red.mu * discrepancy->refAtCell(x,y)->green.mu * discrepancy->refAtCell(x,y)->blue.mu;
	total_discrepancy = 1.0 - point_discrepancy;
	discrepancy_magnitude.at<double>(y,x) = total_discrepancy;

	//identitycheckProb("total_discrepancy", total_discrepancy);
	//identitycheckProb("rmu", discrepancy->refAtCell(x,y)->red.mu);
	//identitycheckProb("bmu", discrepancy->refAtCell(x,y)->blue.mu);
	//identitycheckProb("gmu", discrepancy->refAtCell(x,y)->green.mu);
	//rmu_diff*rmu_diff + gmu_diff*gmu_diff + bmu_diff*bmu_diff ;
	//+ rvar_quot + gvar_quot + bvar_quot;

	discrepancy_density.at<double>(y,x) = discrepancy_magnitude.at<double>(y,x);
	//sqrt(discrepancy_magnitude.at<double>(y,x) / 3.0) / 255.0; 

      } else {
	discrepancy->refAtCell(x,y)->zero();
  
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
      if (discrepancy->refAtCell(x,y)->red.samples > 0) {
	totalScore += discrepancy_magnitude.at<double>(y,x); 
      } else {
      }
    }
  }
  
  return totalScore;
}

shared_ptr<Scene> Scene::copyBox(int _x1, int _y1, int _x2, int _y2) {
  int x1 = min(_x1, _x2);
  int x2 = max(_x1, _x2);
  int y1 = min(_y1, _y2);
  int y2 = max(_y1, _y2);

  x1 = min( max(0,x1), width-1);
  x2 = min( max(0,x2), width-1);
  y1 = min( max(0,y1), height-1);
  y2 = min( max(0,y2), height-1);

  shared_ptr<Scene> toReturn = std::make_shared<Scene>(ms, x2-x1, y2-y1, cell_width);

  toReturn->background_map = background_map->copyBox(x1,y1,x2,y2);
  toReturn->predicted_map = predicted_map->copyBox(x1,y1,x2,y2);
  toReturn->observed_map = observed_map->copyBox(x1,y1,x2,y2);
  toReturn->discrepancy= discrepancy->copyBox(x1,y1,x2,y2);

  toReturn->discrepancy_magnitude = discrepancy_magnitude(cv::Range(y1, y2), cv::Range(x1, x2)).clone();
  toReturn->discrepancy_density = discrepancy_density(cv::Range(y1, y2), cv::Range(x1, x2)).clone();
  toReturn->predicted_segmentation = predicted_segmentation(cv::Range(y1, y2), cv::Range(x1, x2)).clone();

  return toReturn;
}

shared_ptr<Scene> Scene::copyPaddedDiscrepancySupport(double threshold, double pad_meters) {
  int xmin = width;
  int xmax = 0;
  int ymin = height;
  int ymax = 0;
  cout << xmin << " " << xmax << " " << ymin << " " << ymax << " initial " << endl;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if (discrepancy_density.at<double>(y,x) != 0) {
	cout << discrepancy_density.at<double>(y,x) << endl;
      }

      if ((predicted_map->refAtCell(x,y)->red.samples > 0) && (discrepancy_density.at<double>(y,x) > threshold)) {
	xmin = min(xmin, x);
	xmax = max(xmax, x);
	ymin = min(ymin, y);
	ymax = max(ymax, y);
      } else {
      }
    }
  }
  cout << xmin << " " << xmax << " " << ymin << " " << ymax << " secondary " << endl;

  xmin = xmin - ceil(pad_meters/cell_width);
  xmax = xmax + ceil(pad_meters/cell_width);
  ymin = ymin - ceil(pad_meters/cell_width);
  ymax = ymax + ceil(pad_meters/cell_width);

  int new_width = xmax - xmin;
  int new_height = ymax - ymin;
  // make sure the crop is odd dimensioned
  if ((new_width % 2) == 0) {
    xmax = xmax-1;
  } else {}
  if ((new_height % 2) == 0) {
    ymax = ymax-1;
  } else {}
  new_width = xmax - xmin;
  new_height = ymax - ymin;

  cout << xmin << " " << xmax << " " << ymin << " " << ymax << " third " << endl;

  shared_ptr<Scene> scene_to_return;
  if ((xmin <= xmax) && (ymin <= ymax)) {
    scene_to_return = copyBox(xmin,ymin,xmax,ymax);
  } else {
    ROS_ERROR_STREAM("region contained no discrepant cells." << endl);
    cout << xmin << " " << xmax << " " << ymin << " " << ymax << endl;
  }
  
  return scene_to_return;
}

int Scene::countDiscrepantCells(double threshold, int _x1, int _y1, int _x2, int _y2) {
  int x1 = min(_x1, _x2);
  int x2 = max(_x1, _x2);
  int y1 = min(_y1, _y2);
  int y2 = max(_y1, _y2);

  x1 = min( max(0,x1), width-1);
  x2 = min( max(0,x2), width-1);
  y1 = min( max(0,y1), height-1);
  y2 = min( max(0,y2), height-1);

  int discrepant_cells = 0;
  for (int y = y1; y <= y2; y++) {
    for (int x = x1; x <= x2; x++) {
      if ((predicted_map->refAtCell(x,y)->red.samples > 0) && (discrepancy_density.at<double>(y,x) > threshold)) {
	discrepant_cells++;
      } else {
      }
    }
  }

  return discrepant_cells;
}

// XXX 
void Scene::proposeRegion() {
}
// XXX 
void Scene::proposeObject() {
}

// XXX 
void Scene::tryToAddObjectToScene(int class_idx) {
  REQUIRE_VALID_CLASS(ms,class_idx);
  guardSceneModels(ms);

  vector<Mat> rotated_object_imgs;
  int numScales = 1;//11;
  int numOrientations = 37;
  double scaleStepSize = 1.02;
  int etaS = 0;

  rotated_object_imgs.resize(numScales*numOrientations);
  double startScale = pow(scaleStepSize, -(numScales-1)/2);  
  double thisScale = startScale * pow(scaleStepSize, etaS);

  Mat prepared_discrepancy;
  {
    prepared_discrepancy = discrepancy_density.clone();
  }
  Mat prepared_object = ms->config.class_scene_models[class_idx]->discrepancy_density.clone();
  double po_l1norm = prepared_object.dot(Mat::ones(prepared_object.rows, prepared_object.cols, prepared_object.type()));
  cout << "  po_l1norm: " << po_l1norm << endl;
  double overlap_thresh = 0.5;

  double max_dim = max(prepared_object.rows, prepared_object.cols);
  Size toBecome(max_dim, max_dim);

  //double globalMax = 0.0;
  int max_x = -1;
  int max_y = -1;
  int max_orient = -1;
  double max_score = -1;

  for (int thisOrient = 0; thisOrient < numOrientations; thisOrient++) {
/*
    // rotate the template and L1 normalize it
    Point center = Point(ms->config.aerialGradientWidth/2, ms->config.aerialGradientWidth/2);
    double angle = thisOrient*360.0/numOrientations;
    
    //double scale = 1.0;
    double scale = thisScale;
    
    // Get the rotation matrix with the specifications above
    Mat rot_mat = getRotationMatrix2D(center, angle, scale);
    warpAffine(im2, rotatedAerialGrads[thisOrient + etaS*numOrientations], rot_mat, toBecome);
    
    processSaliency(rotatedAerialGrads[thisOrient + etaS*numOrientations], rotatedAerialGrads[thisOrient + etaS*numOrientations]);
    
    double mean = rotatedAerialGrads[thisOrient + etaS*numOrientations].dot(Mat::ones(ms->config.aerialGradientWidth, ms->config.aerialGradientWidth, rotatedAerialGrads[thisOrient + etaS*numOrientations].type())) / double(ms->config.aerialGradientWidth*ms->config.aerialGradientWidth);
    rotatedAerialGrads[thisOrient + etaS*numOrientations] = rotatedAerialGrads[thisOrient + etaS*numOrientations] - mean;
    double l2norm = rotatedAerialGrads[thisOrient + etaS*numOrientations].dot(rotatedAerialGrads[thisOrient + etaS*numOrientations]);
    l2norm = sqrt(l2norm);
    if (l2norm <= EPSILON) {
      l2norm = 1.0;
    }
    rotatedAerialGrads[thisOrient + etaS*numOrientations] = rotatedAerialGrads[thisOrient + etaS*numOrientations] / l2norm;
*/

/*
    prepareForCrossCorrelation(ms, im2, rotatedAerialGrads[thisOrient + etaS*numOrientations], thisOrient, numOrientations, thisScale, toBecome);

    Mat output = preparedClass1.clone(); 
    filter2D(preparedClass1, output, -1, rotatedAerialGrads[thisOrient + etaS*numOrientations], Point(-1,-1), 0, BORDER_CONSTANT);
    double minValue, maxValue;
    minMaxLoc(output, &minValue, &maxValue);
    globalMax = max(maxValue, globalMax);
*/
    Point center = Point(max_dim/2, max_dim/2);
    double angle = thisOrient*360.0/numOrientations;
    
    //double scale = 1.0;
    double scale = thisScale;
    
    // Get the rotation matrix with the specifications above
    Mat rot_mat = getRotationMatrix2D(center, angle, scale);
    cout << rot_mat << rot_mat.size() << endl;
    rot_mat.at<double>(0,2) += ((max_dim - prepared_object.cols)/2.0);
    rot_mat.at<double>(1,2) += ((max_dim - prepared_object.rows)/2.0);
    warpAffine(prepared_object, rotated_object_imgs[thisOrient + etaS*numOrientations], rot_mat, toBecome);

    Mat output = prepared_discrepancy.clone(); 
    filter2D(prepared_discrepancy, output, -1, rotated_object_imgs[thisOrient + etaS*numOrientations], Point(-1,-1), 0, BORDER_CONSTANT);

/*
    Mat tob = rotated_object_imgs[thisOrient + etaS*numOrientations];
    int tob_half_width = ceil(tob.cols/2.0);
    int tob_half_height = ceil(tob.rows/2.0);

    cout << "tob " << tob_half_width << " " << tob_half_height << endl;

    #pragma omp parallel for
    for (int ys = tob_half_height; ys < output.rows-tob_half_height; ys++) {
      for (int xs = tob_half_width; xs < output.cols-tob_half_width; xs++) {
	output.at<double>(ys,xs) = 0.0;
	//if (prepared_discrepancy.at<double>(ys,xs) > 0) 
	if (discrepancy->refAtCell(xs,ys)->red.samples > 0) 
	//if ( 0 ) 
	//if ( 1 ) 
	{
	  int xst_part = xs-tob_half_width;
	  int yst_part = ys-tob_half_height;
	  for (int yo = 0; yo < tob.rows; yo++) {
	    for (int xo = 0; xo < tob.cols; xo++) {
	      int xst = xst_part+xo;
	      int yst = yst_part+yo;
	      //int xst = xs-tob_half_width+xo;
	      //int yst = ys-tob_half_height+yo;
	      //if ( 
		  //(xo >= 0 && xo < tob.cols) &&
		  //(yo >= 0 && yo < tob.rows) &&
		  //(xst >= 0 && xst < output.cols) &&
		  //(yst >= 0 && yst < output.rows) 
		 //) {
		output.at<double>(ys,xs) += prepared_discrepancy.at<double>(yst,xst) * tob.at<double>(yo,xo);
	      //}
	    }
	  }
	}
      }
    }
*/

    //cout << output ;
    for (int y = 0; y < output.rows; y++) {
      for (int x = 0; x < output.cols; x++) {
	if (output.at<double>(y,x) > overlap_thresh * po_l1norm) {
	  cout << output.at<double>(y,x) << "  ";
	}
	if (output.at<double>(y,x) > max_score) {
	  max_score = output.at<double>(y,x);
	  max_x = x;
	  max_y = y;
	  max_orient = thisOrient;
	}
      }
    }
  }

  //cout << prepared_discrepancy << prepared_object ;

  double max_theta = -max_orient * 2.0 * M_PI / numOrientations;

  double max_x_meters, max_y_meters;
  cellToMeters(max_x, max_y, &max_x_meters, &max_y_meters);

  cout << max_x << " " << max_y << " " << max_orient << " " << max_x_meters << " " << max_y_meters << " " << max_theta << endl;

  if (max_x > -1) {
    ms->pushWord("sceneAddPredictedFocusedObject");
    ms->pushWord(make_shared<DoubleWord>(max_theta));
    ms->pushWord(make_shared<DoubleWord>(max_y_meters));
    ms->pushWord(make_shared<DoubleWord>(max_x_meters));
  } else {
    cout << "Did not find a valid cell... not adding object." << endl;
  }
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

int Scene::safeAt(int x, int y) {
  return ( (x >= 0) && (x < width) && (y >= 0) && (y < height) );
}

void Scene::metersToCell(double xm, double ym, int * xc, int * yc) {
  (*xc) = round(xm / cell_width) + x_center_cell;
  (*yc) = round(ym / cell_width) + y_center_cell;
} 

void Scene::metersToCell(double xm, double ym, double * xc, double * yc) {
  (*xc) = (xm / cell_width) + x_center_cell;
  (*yc) = (ym / cell_width) + y_center_cell;
} 


void Scene::cellToMeters(int xc, int yc, double * xm, double * ym) {
  (*xm) = (xc - x_center_cell) * cell_width; 
  (*ym) = (yc - y_center_cell) * cell_width; 
} 


void Scene::writeToFileStorage(FileStorage& fsvO) {
  fsvO << "{";
  fsvO << "width" << width;
  fsvO << "height" << height;
  fsvO << "x_center_cell" << x_center_cell;
  fsvO << "y_center_cell" << y_center_cell;
  fsvO << "cell_width" << cell_width;
  fsvO << "score" << score;
  fsvO << "background_pose";
  background_pose.writeToFileStorage(fsvO);

  fsvO << "background_map";
  background_map->writeToFileStorage(fsvO);

  fsvO << "predicted_map";
  predicted_map->writeToFileStorage(fsvO);

  fsvO << "observed_map";
  observed_map->writeToFileStorage(fsvO);

  fsvO << "discrepancy_map";
  discrepancy->writeToFileStorage(fsvO);

  fsvO << "predicted_objects";
  writePredictedObjects(fsvO);

  fsvO << "predicted_segmentation" << predicted_segmentation;
  fsvO << "discrepancy_magnitude" << discrepancy_magnitude;
  fsvO << "discrepancy_density" << discrepancy_density;

  fsvO << "}";
}

void Scene::writePredictedObjects(FileStorage & fsvO) {
  fsvO << "[";
  for (int i = 0; i < predicted_objects.size(); i++) {
    predicted_objects[i]->writeToFileStorage(fsvO);
  }
  fsvO << "]";
}

void Scene::readPredictedObjects(FileNode & fn) {
  predicted_objects.resize(0);

  for (FileNodeIterator it = fn.begin(); it != fn.end(); it++) {
    shared_ptr<SceneObject> obj = make_shared<SceneObject>();
    FileNode node = *it;
    obj->readFromFileNode(node);
    predicted_objects.push_back(obj);
  }
}

void Scene::readFromFileNodeIterator(FileNodeIterator& it) {
  FileNode node = *it;
  readFromFileNode(node);
}

void Scene::readFromFileNode(FileNode& it) {

  (it)["width"] >> width;
  (it)["height"] >> height;
  (it)["x_center_cell"] >> x_center_cell;
  (it)["y_center_cell"] >> y_center_cell;
  (it)["cell_width"] >> cell_width;
  (it)["score"] >> score;
  
  FileNode bg_pose_node = (it)["background_pose"];
  background_pose.readFromFileNode(bg_pose_node);
  
  FileNode bg_map_node = (it)["background_map"];
  background_map->readFromFileNode(bg_map_node);

  FileNode predicted_map_node = (it)["predicted_map"];
  predicted_map->readFromFileNode(predicted_map_node);

  FileNode observed_map_node = (it)["observed_map"];
  observed_map->readFromFileNode(observed_map_node);

  FileNode discrepancy_map_node = (it)["discrepancy_map"];
  discrepancy->readFromFileNode(discrepancy_map_node);

  FileNode node = it["predicted_objects"];
  readPredictedObjects(node);

  it["predicted_segmentation"] >> predicted_segmentation;
  it["discrepancy_magnitutde"] >> discrepancy_magnitude;
  it["discrepancy_density"] >> discrepancy_density;

}

void Scene::saveToFile(string filename) {
  FileStorage fsvO;
  cout << "Scene::saveToFile writing: " << filename << "...." << endl;
  fsvO.open(filename, FileStorage::WRITE);
  fsvO << "Scene";
  writeToFileStorage(fsvO);
  fsvO.release();
  cout << "done." << endl;

}

void Scene::loadFromFile(string filename) {
  FileStorage fsvI;
  cout << "Scene::loadFromFile reading: " << filename<< " ..." << endl;
  fsvI.open(filename, FileStorage::READ);
  if (fsvI.isOpened()) {
    FileNode anode = fsvI["Scene"];
    readFromFileNode(anode);
  } else {
    ROS_ERROR_STREAM("Could not open file " << filename);
  }
  cout << "done." << endl;
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

// XXX samples logic is no longer correct since samples is stored per channel

*/





namespace ein_words {

WORD(SceneSaveScene)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string message;
  GET_STRING_ARG(ms, message);

  stringstream ss;
  ss << ms->config.data_directory + "/scenes/" + message + ".yml";
  stringstream ss_dir;
  ss_dir << ms->config.data_directory + "/scenes/";
  mkdir(ss_dir.str().c_str(), 0777);

  ms->config.scene->saveToFile(ss.str());

}
END_WORD
REGISTER_WORD(SceneSaveScene)

WORD(SceneLoadScene)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string message;
  GET_STRING_ARG(ms, message);

  stringstream ss;
  ss << ms->config.data_directory + "/scenes/" + message + ".yml";
  stringstream ss_dir;
  ss_dir << ms->config.data_directory + "/scenes/";
  mkdir(ss_dir.str().c_str(), 0777);

  ms->config.scene->loadFromFile(ss.str());
  ms->evaluateProgram("sceneRenderObservedMap sceneRenderBackgroundMap sceneRenderPredictedMap sceneRenderDiscrepancy");
}
END_WORD
REGISTER_WORD(SceneLoadScene)


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

  ms->pushWord("sceneRenderBackgroundMap");
}
END_WORD
REGISTER_WORD(SceneLoadBackgroundMap)


WORD(SceneRenderBackgroundMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Mat backgroundImage;
  ms->config.scene->background_map->rgbMuToMat(backgroundImage);
  Mat rgb = backgroundImage.clone();  
  cvtColor(backgroundImage, rgb, CV_YCrCb2BGR);
  ms->config.backgroundWindow->updateImage(rgb);

}
END_WORD
REGISTER_WORD(SceneRenderBackgroundMap)


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

WORD(SceneSaveFocusedSceneModel)
virtual void execute(std::shared_ptr<MachineState> ms) {
  REQUIRE_FOCUSED_CLASS(ms,tfc);
  guardSceneModels(ms);

  string message;
  GET_STRING_ARG(ms, message);

  stringstream ss;
  ss << ms->config.data_directory + "/scenes/" + message + ".yml";
  stringstream ss_dir;
  ss_dir << ms->config.data_directory + "/scenes/";
  mkdir(ss_dir.str().c_str(), 0777);

  ms->config.class_scene_models[tfc]->saveToFile(ss.str());
}
END_WORD
REGISTER_WORD(SceneSaveFocusedSceneModel)

WORD(SceneLoadFocusedSceneModel)
virtual void execute(std::shared_ptr<MachineState> ms) {
  REQUIRE_FOCUSED_CLASS(ms,tfc);
  guardSceneModels(ms);

  string message;
  GET_STRING_ARG(ms, message);

  stringstream ss;
  ss << ms->config.data_directory + "/scenes/" + message + ".yml";
  stringstream ss_dir;
  ss_dir << ms->config.data_directory + "/scenes/";
  mkdir(ss_dir.str().c_str(), 0777);

  ms->config.class_scene_models[tfc]->loadFromFile(ss.str());
}
END_WORD
REGISTER_WORD(SceneLoadFocusedSceneModel)

WORD(SceneClearPredictedObjects)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.scene->predicted_objects.resize(0);
}
END_WORD
REGISTER_WORD(SceneClearPredictedObjects)

WORD(SceneAddPredictedFocusedObject)
virtual void execute(std::shared_ptr<MachineState> ms) {
  REQUIRE_FOCUSED_CLASS(ms,tfc);
  guardSceneModels(ms);

  double x_in=0, y_in=0, theta_in=0;
  GET_NUMERIC_ARG(ms, theta_in);
  GET_NUMERIC_ARG(ms, y_in);
  GET_NUMERIC_ARG(ms, x_in);

  eePose topass = eePose::identity().applyRPYTo(theta_in,0,0); 
  topass.px = x_in;
  topass.py = y_in;
  shared_ptr<SceneObject> topush = make_shared<SceneObject>(topass, tfc, ms->config.classLabels[tfc], PREDICTED);

  ms->config.scene->predicted_objects.push_back(topush);
}
END_WORD
REGISTER_WORD(SceneAddPredictedFocusedObject)

WORD(ScenePredictFocusedObject)
virtual void execute(std::shared_ptr<MachineState> ms) {
  REQUIRE_FOCUSED_CLASS(ms,tfc);
  guardSceneModels(ms);
  ms->config.scene->tryToAddObjectToScene(tfc);
}
END_WORD
REGISTER_WORD(ScenePredictFocusedObject)

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

WORD(SceneSetBackgroundStdDevY)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double stddev = 0;
  GET_NUMERIC_ARG(ms, stddev);

  for (int y = 0; y < ms->config.scene->background_map->height; y++) {
    for (int x = 0; x < ms->config.scene->background_map->width; x++) {
      ms->config.scene->background_map->refAtCell(x,y)->blue.sigmasquared = pow(stddev, 2);
    }
  }
}
END_WORD
REGISTER_WORD(SceneSetBackgroundStdDevY)

WORD(SceneSetBackgroundStdDevColor)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double stddev = 0;
  GET_NUMERIC_ARG(ms, stddev);

  for (int y = 0; y < ms->config.scene->background_map->height; y++) {
    for (int x = 0; x < ms->config.scene->background_map->width; x++) {
      ms->config.scene->background_map->refAtCell(x,y)->red.sigmasquared = pow(stddev, 2);
      ms->config.scene->background_map->refAtCell(x,y)->green.sigmasquared = pow(stddev, 2);
    }
  }
}
END_WORD
REGISTER_WORD(SceneSetBackgroundStdDevColor)

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

  Size sz = ms->config.wristCamImage.size();
  int imW = sz.width;
  int imH = sz.height;
    
  //for (int px = ms->config.grayTop.x+ms->config.mapGrayBoxPixelSkirtCols; px < ms->config.grayBot.x-ms->config.mapGrayBoxPixelSkirtCols; px++) 
    //for (int py = ms->config.grayTop.y+ms->config.mapGrayBoxPixelSkirtRows; py < ms->config.grayBot.y-ms->config.mapGrayBoxPixelSkirtRows; py++) 
  for (int px = 0; px < imW; px++) {
    for (int py = 0; py < imH; py++) {
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
  ms->pushWord("sceneRenderObservedMap");
}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromWrist)


WORD(SceneRenderObservedMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Mat image;
  ms->config.scene->observed_map->rgbMuToMat(image);
  Mat rgb = image.clone();  
  cvtColor(image, rgb, CV_YCrCb2BGR);
  ms->config.observedWindow->updateImage(rgb);
}
END_WORD
REGISTER_WORD(SceneRenderObservedMap)

WORD(SceneComposePredictedMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double p_threshold = 0.5;
  ms->config.scene->composePredictedMap(p_threshold);
  ms->pushWord("sceneRenderPredictedMap");
}
END_WORD
REGISTER_WORD(SceneComposePredictedMap)

WORD(SceneRenderPredictedMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Mat image;
  ms->config.scene->predicted_map->rgbMuToMat(image);
  Mat rgb = image.clone();  
  cvtColor(image, rgb, CV_YCrCb2BGR);
  ms->config.predictedWindow->updateImage(rgb);
}
END_WORD
REGISTER_WORD(SceneRenderPredictedMap)


WORD(SceneUpdateDiscrepancy)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.scene->measureDiscrepancy();
  ms->pushWord("sceneRenderDiscrepancy");
}
END_WORD
REGISTER_WORD(SceneUpdateDiscrepancy)


WORD(SceneRenderDiscrepancy)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Mat image;
  ms->config.scene->discrepancy->rgbDiscrepancyMuToMat(image);
  image = image * 255;
  ms->config.discrepancyWindow->updateImage(image);
}
END_WORD
REGISTER_WORD(SceneRenderDiscrepancy)



WORD(SceneGrabDiscrepantCropAsClass)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int tfc = ms->config.focusedClass;
  if ( (tfc > -1) && (tfc < ms->config.classLabels.size()) ) {
  } else {
    ROS_ERROR_STREAM("Invalid focused class, not grabbing..." << endl);
    return;
  }

  double p_crop_pad = 0.05;
  guardSceneModels(ms);
  double threshold = 0.0;
  GET_NUMERIC_ARG(ms, threshold);
  shared_ptr<Scene> scene_crop = ms->config.scene->copyPaddedDiscrepancySupport(threshold, p_crop_pad);

  ms->config.class_scene_models[tfc] = scene_crop;
}
END_WORD
REGISTER_WORD(SceneGrabDiscrepantCropAsClass)

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

WORD(SceneCountDiscrepantCells)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int width = ms->config.scene->width;
  int height = ms->config.scene->height;
  double threshold = 0.0;
  GET_NUMERIC_ARG(ms, threshold);
  int nc = ms->config.scene->countDiscrepantCells(threshold,0,0,width-1,height-1); 
  ms->pushWord(make_shared<IntegerWord>(nc));
}
END_WORD
REGISTER_WORD(SceneCountDiscrepantCells)

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




