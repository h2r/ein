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

double safeSigmaSquared(double sigmasquared) {
  if (sigmasquared == 0.0) {
    return 1.0;
  } else {
    return sigmasquared;
  }
}


double computeLogLikelihood(GaussianMapChannel & channel1, GaussianMapChannel & channel2) {
  double safesigmasquared1 = safeSigmaSquared(channel1.sigmasquared);
  double term1 = - pow((channel2.mu - channel1.mu), 2)  / (2 * safesigmasquared1);
  // XXX maybe term2 should be cached in the map
  double term2 = -log(sqrt(2 * M_PI * safesigmasquared1));
  double result = term1 + term2; 
  return result;
}
void computeInnerProduct(GaussianMapChannel & channel1, GaussianMapChannel & channel2, double * likelihood, double * channel_term_out) {
  double ip_normalizer = 0.0;
  double safesigmasquared1 = safeSigmaSquared(channel1.sigmasquared);
  double safesigmasquared2 = safeSigmaSquared(channel1.sigmasquared);
  double newsigmasquared = 1.0 / (1.0 / safesigmasquared1 + 1.0 / safesigmasquared2); 
  double newmu = newsigmasquared * (channel1.mu / safesigmasquared1 + channel2.mu / safesigmasquared2); 
  for (int i = 0; i < 256; i++) {
    ip_normalizer += normal_pdf(newmu, sqrt(newsigmasquared), i);
  }
/*
  double channel_term = exp(  -0.5*pow(channel1.mu-channel2.mu, 2)/( channel1.sigmasquared + channel2.sigmasquared )  ) / sqrt( 2.0*M_PI*(channel1.sigmasquared + channel2.sigmasquared) ); 
  channel_term = channel_term * ip_normalizer * 0.5 * 256;
  *(channel_term_out) = channel_term;
*/

  double ip_val = exp(  -0.5*pow(channel1.mu-channel2.mu, 2)/( safesigmasquared1 + safesigmasquared2 )  ) / sqrt( 2.0*M_PI*(safesigmasquared1 + safesigmasquared2) ); 
  *likelihood = ip_normalizer * ip_val;
  double prior = 0.5;
  double nb_normalizer = *likelihood * prior +  (1.0/256.0) * (1 - prior);

  *(channel_term_out) = *likelihood * prior / nb_normalizer; 
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
  double rlikelihood, glikelihood, blikelihood;
  computeInnerProduct(red, other->red, &rlikelihood, rterm_out);
  computeInnerProduct(green, other->green, &glikelihood, gterm_out);
  computeInnerProduct(blue, other->blue, &blikelihood, bterm_out);
  return normalizeDiscrepancy(rlikelihood, glikelihood, blikelihood);

}

void computePointDiscrepancy(GaussianMapChannel & channel1, GaussianMapChannel & channel2, double * likelihood, double * channel_term_normalized) {
  double safesigmasquared = safeSigmaSquared(channel1.sigmasquared);
  *likelihood = normal_pdf(channel1.mu, sqrt(safesigmasquared), channel2.mu);

  double prior = 0.5;
  double normalizer = *likelihood * prior +  (1.0/256.0) * (1 - prior);
  *channel_term_normalized = *likelihood * prior / normalizer;
}

double _GaussianMapCell::pointDiscrepancy(_GaussianMapCell * other, double * rterm_out, double * gterm_out, double * bterm_out) {
  double rlikelihood, glikelihood, blikelihood;
  computePointDiscrepancy(red, other->red, &rlikelihood, rterm_out);
  computePointDiscrepancy(green, other->green, &glikelihood, gterm_out);
  computePointDiscrepancy(blue, other->blue, &blikelihood, bterm_out);
  return normalizeDiscrepancy(rlikelihood, glikelihood, blikelihood);
}

double _GaussianMapCell::normalizeDiscrepancy(double rlikelihood,  double glikelihood, double blikelihood) {

  //return *rterm_out * *bterm_out * *gterm_out;
  double prior = 0.5;



  double likelihood = rlikelihood * glikelihood * blikelihood;

  if (std::isnan(likelihood)) {
    cout << "r: " << rlikelihood << endl;
    cout << "g: " << glikelihood << endl;
    cout << "b: " << blikelihood << endl;

    if (std::isnan(rlikelihood)) {
      cout << "rmu: " << red.mu << " sigma: " << red.sigmasquared << " rlikelihood: " << rlikelihood << endl;
    }
    if (std::isnan(glikelihood)) {
      cout << "gmu: " << green.mu << " sigma: " << green.sigmasquared << " glikelihood: " << glikelihood << endl;
    }
    if (std::isnan(blikelihood)) {
      cout << "bmu: " << blue.mu << " sigma: " << blue.sigmasquared << " blikelihood: " << blikelihood << endl;
    }
  }

  double normalizer = likelihood * prior + pow(1.0/256, 3) * (1 - prior);
  assert(normalizer != 0);
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
int GaussianMap::safeBilinAt(int x, int y) {
  return ( (cells != NULL) && (x >= 2) && (x < width-2) && (y >= 2) && (y < height-2) );
}

GaussianMapCell *GaussianMap::refAtCell(int x, int y) {
  if (x < 0 || x >= width) {
    ROS_ERROR_STREAM("GaussianMapCell::refAtCell: Bad x. " << x);
    return NULL;
  }
  if (y < 0 || y >= height) {
    ROS_ERROR_STREAM("GaussianMapCell::refAtCell: Bad y. " << y);
    return NULL;
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
  BILIN_MAPCELL(z.counts);

  BILIN_MAPCELL(red.squaredcounts);
  BILIN_MAPCELL(green.squaredcounts);
  BILIN_MAPCELL(blue.squaredcounts);
  BILIN_MAPCELL(z.squaredcounts);

  BILIN_MAPCELL(red.samples);
  BILIN_MAPCELL(green.samples);
  BILIN_MAPCELL(blue.samples);
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


CONFIG_GETTER_DOUBLE(SceneMinSigmaSquared, ms->config.sceneMinSigmaSquared)
CONFIG_SETTER_DOUBLE(SceneSetMinSigmaSquared, ms->config.sceneMinSigmaSquared)

void GaussianMapChannel::recalculateMusAndSigmas(shared_ptr<MachineState> ms) {
  double safe_samples = max(samples, 1.0);
  mu = counts / safe_samples;			
  sigmasquared = (squaredcounts / safe_samples) - (mu * mu); 
  if (sigmasquared < ms->config.sceneMinSigmaSquared) {
    sigmasquared = ms->config.sceneMinSigmaSquared;
  }
}
 
void GaussianMap::recalculateMusAndSigmas(shared_ptr<MachineState> ms) {
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {

      refAtCell(x, y)->red.recalculateMusAndSigmas(ms);
      refAtCell(x, y)->green.recalculateMusAndSigmas(ms);
      refAtCell(x, y)->blue.recalculateMusAndSigmas(ms);
      refAtCell(x, y)->z.recalculateMusAndSigmas(ms);
      
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

void GaussianMap::rgbDiscrepancyMuToMat(shared_ptr<MachineState> ms, Mat& out) {
  //out = Mat(height, width, CV_64FC3);
  out = Mat(height, width, CV_8UC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<Vec3b>(y,x)[0] = uchar(refAtCell(x,y)->blue.mu * 255);
      out.at<Vec3b>(y,x)[1] = uchar(refAtCell(x,y)->green.mu * 255);
      out.at<Vec3b>(y,x)[2] = uchar(refAtCell(x,y)->red.mu * 255); 
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

  shared_ptr<GaussianMap> toReturn = std::make_shared<GaussianMap>(x2-x1+1, y2-y1+1, cell_width);
  for (int y = y1; y < y2; y++) {
    for (int x = x1; x < x2; x++) {
      *(toReturn->refAtCell(x-x1,y-y1)) = *(refAtCell(x,y));
    }
  }
  
  return toReturn;
}

shared_ptr<GaussianMap> GaussianMap::copy() {
  return copyBox(0,0,width-1,height-1);
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

bool compareDiscrepancyDescending(const SceneObjectScore &i, const SceneObjectScore &j) {
  return (i.discrepancy_score > j.discrepancy_score);
}

SceneObject::SceneObject(eePose _eep, int _lci, string _ol, sceneObjectType _sot) {
  scene_pose = _eep;
  labeled_class_index = _lci;
  object_label = _ol;
  sot = _sot;
  scores.resize(0);
}

SceneObject::SceneObject() {
  scene_pose = eePose::identity();
  labeled_class_index = -1;
  object_label = string("");
  sot = BACKGROUND;
  scores.resize(0);
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

void Scene::smoothDiscrepancyDensity(double sigma) {
  GaussianBlur(discrepancy_density, discrepancy_density, cv::Size(0,0), sigma);
}
void Scene::setDiscrepancyDensityFromMagnitude(double sigma) {
  GaussianBlur(discrepancy_magnitude, discrepancy_density, cv::Size(0,0), sigma);
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

    // XXX optimize by transforming corners 
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

	double cells_object_x, cells_object_y;
	tos->metersToCell(meters_object_x, meters_object_y, &cells_object_x, &cells_object_y);
	if ( tos->safeBilinAt(cells_object_x, cells_object_y) ) {
	  if (tos->isDiscrepantMetersBilin(threshold, meters_object_x, meters_object_y)) {
	    *(predicted_map->refAtCell(x,y)) = tos->observed_map->bilinValAtMeters(meters_object_x, meters_object_y);
	  } else {
	  }
	} else {
	}
	
	// take exaggerated bounding box of object in scene
	//   look up each scene cell in the object's frame
	//   if one of the contributors is valid, replace this scene cell with the interpolated object cell 
      }
    }
  }
  predicted_map->recalculateMusAndSigmas(ms);
}
double Scene::computeScore() { 
  double score = 0.0;
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      if ((predicted_map->refAtCell(x,y)->red.samples > 0) && (observed_map->refAtCell(x,y)->red.samples > 0)) {
	GaussianMapCell * observed_cell = observed_map->refAtCell(x, y);
	GaussianMapCell * predicted_cell = predicted_map->refAtCell(x, y);
	score += computeLogLikelihood(predicted_cell->red, observed_cell->red);
	score += computeLogLikelihood(predicted_cell->green, observed_cell->green);
	score += computeLogLikelihood(predicted_cell->blue, observed_cell->blue);
      }
    }
  }
  return score;
}
void Scene::measureDiscrepancy() {
  // close to kl-divergence
  // for now this only does rgb
  int c_enough = 0;
  int c_total = 0;
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      if ((predicted_map->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold) && (observed_map->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold)) {


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

	// XXX investigate inner product performance
	//double total_discrepancy = predicted_map->refAtCell(x,y)->innerProduct(observed_map->refAtCell(x,y), &rmu_diff, &gmu_diff, &bmu_diff);
	//double total_discrepancy = predicted_map->refAtCell(x,y)->pointDiscrepancy(observed_map->refAtCell(x,y), &rmu_diff, &gmu_diff, &bmu_diff);
	double discrepancy_value;
	if (ms->config.discrepancyMode == DISCREPANCY_POINT) {
	  discrepancy_value = predicted_map->refAtCell(x,y)->pointDiscrepancy(observed_map->refAtCell(x,y), &rmu_diff, &gmu_diff, &bmu_diff);
	} else if (ms->config.discrepancyMode == DISCREPANCY_DOT) {
	  discrepancy_value = predicted_map->refAtCell(x,y)->innerProduct(observed_map->refAtCell(x,y), &rmu_diff, &gmu_diff, &bmu_diff);
	} else {
	  cout << "Invalid discrepancy mode: " << ms->config.discrepancyMode << endl;
	  assert(0);
	}

	//double point_discrepancy = predicted_map->refAtCell(x,y)->pointDiscrepancy(observed_map->refAtCell(x,y), &rmu_diff, &gmu_diff, &bmu_diff);
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
	total_discrepancy = 1.0 - discrepancy_value;
	if (std::isnan((double) total_discrepancy)) {
	  cout << "Total discrepancy is nan. " << total_discrepancy << endl;
	  total_discrepancy = 0.0;
	}
	discrepancy_magnitude.at<double>(y,x) = total_discrepancy;

	//identitycheckProb("total_discrepancy", total_discrepancy);
	//identitycheckProb("rmu", discrepancy->refAtCell(x,y)->red.mu);
	//identitycheckProb("bmu", discrepancy->refAtCell(x,y)->blue.mu);
	//identitycheckProb("gmu", discrepancy->refAtCell(x,y)->green.mu);
	//rmu_diff*rmu_diff + gmu_diff*gmu_diff + bmu_diff*bmu_diff ;
	//+ rvar_quot + gvar_quot + bvar_quot;

	//sqrt(discrepancy_magnitude.at<double>(y,x) / 3.0) / 255.0; 

	c_enough++;
	//cout << " enough ";
	//cout << predicted_map->refAtCell(x,y)->red.samples << " " << observed_map->refAtCell(x,y)->red.samples << " ";
      } else {
	discrepancy->refAtCell(x,y)->zero();
	discrepancy_magnitude.at<double>(y,x) = 0.0;
      }
      if ((predicted_map->refAtCell(x,y)->red.samples > 0) && (observed_map->refAtCell(x,y)->red.samples > 0)) {
	c_total++;
      } else {
      }
    }
  }

  cout << "sceneCellCountThreshold: " << ms->config.sceneCellCountThreshold << "   c_enough / c_total: " << c_enough << " / " << c_total << " = " << double(c_enough) / double(c_total) << endl;

  double p_density_sigma = 2.0;
  setDiscrepancyDensityFromMagnitude(p_density_sigma);
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

double Scene::recomputeScore(shared_ptr<SceneObject> obj, double threshold) {

  double score = 0.0;
  {
    shared_ptr<SceneObject> tsob = obj;
    shared_ptr<Scene> tos = ms->config.class_scene_models[ tsob->labeled_class_index ];

    int center_x, center_y;
    metersToCell(tsob->scene_pose.px, tsob->scene_pose.py, &center_x, &center_y);

    // XXX optimize by transforming corners 
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

	double cells_object_x, cells_object_y;
	tos->metersToCell(meters_object_x, meters_object_y, &cells_object_x, &cells_object_y);
	if ( tos->safeBilinAt(cells_object_x, cells_object_y) ) {
	  if (tos->isDiscrepantMetersBilin(threshold, meters_object_x, meters_object_y)) {
	    //*(predicted_map->refAtCell(x,y)) = tos->observed_map->bilinValAtMeters(meters_object_x, meters_object_y);
/*
	    if ((predicted_map->refAtCell(x,y)->red.samples > 0) && (observed_map->refAtCell(x,y)->red.samples > 0)) {
	      GaussianMapCell * observed_cell = observed_map->refAtCell(x, y);
	      GaussianMapCell * predicted_cell = predicted_map->refAtCell(x, y);
	      score += computeLogLikelihood(predicted_cell->red, observed_cell->red);
	      score += computeLogLikelihood(predicted_cell->green, observed_cell->green);
	      score += computeLogLikelihood(predicted_cell->blue, observed_cell->blue);
	    }
*/
	    if ( 
		(predicted_map->refAtCell(x,y)->red.samples > 0) &&   
		(observed_map->refAtCell(x,y)->red.samples > 0) &&
		(tos->observed_map->refAtCell(cells_object_x,cells_object_y)->red.samples > 0) 
	       ) {
	      GaussianMapCell * observed_cell = observed_map->refAtCell(x, y);
	      GaussianMapCell * predicted_cell = predicted_map->refAtCell(x, y);
	      GaussianMapCell * object_cell = tos->observed_map->refAtCell(cells_object_x, cells_object_y);

	      double temp = 0.0;
//cout << "score " << score << " ";
	      //temp = object_cell->red.sigmasquared;
	      //object_cell->red.sigmasquared = predicted_cell->red.sigmasquared;
	      score += computeLogLikelihood(object_cell->red, observed_cell->red);
	      //object_cell->red.sigmasquared = temp;

	      //temp = object_cell->green.sigmasquared;
	      //object_cell->green.sigmasquared = predicted_cell->green.sigmasquared;
	      score += computeLogLikelihood(object_cell->green, observed_cell->green);
	      //object_cell->green.sigmasquared = temp;

	      //temp = object_cell->blue.sigmasquared;
	      //object_cell->blue.sigmasquared = predicted_cell->blue.sigmasquared;
	      score += computeLogLikelihood(object_cell->blue, observed_cell->blue);
	      //object_cell->blue.sigmasquared = temp;

//cout << "score " << score << " ";
	      score -= computeLogLikelihood(predicted_cell->red, observed_cell->red);
	      score -= computeLogLikelihood(predicted_cell->green, observed_cell->green);
	      score -= computeLogLikelihood(predicted_cell->blue, observed_cell->blue);
	    }
	  } else {
	  }
	} else {
	}
	
	// take exaggerated bounding box of object in scene
	//   look up each scene cell in the object's frame
	//   if one of the contributors is valid, replace this scene cell with the interpolated object cell 
      }
    }
  }

//cout << "score " << score << " ";
  return score;
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

  shared_ptr<Scene> toReturn = std::make_shared<Scene>(ms, x2-x1+1, y2-y1+1, cell_width);

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
	//cout << discrepancy_density.at<double>(y,x) << endl;
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

  int new_width = xmax - xmin + 1;
  int new_height = ymax - ymin + 1;
  cout << " new_width, new_height: " << new_width << " " << new_height << endl;
  // make sure the crop is odd dimensioned
  if ((new_width % 2) == 0) {
    xmax = xmax-1;
  } else {}
  if ((new_height % 2) == 0) {
    ymax = ymax-1;
  } else {}
  new_width = xmax - xmin + 1;
  new_height = ymax - ymin + 1;

  cout << xmin << " " << xmax << " " << ymin << " " << ymax << " third " << endl;
  cout << " new_width, new_height: " << new_width << " " << new_height << endl;

  shared_ptr<Scene> scene_to_return;
  if ((xmin <= xmax) && (ymin <= ymax)) {
    scene_to_return = copyBox(xmin,ymin,xmax,ymax);
  } else {
    ROS_ERROR_STREAM("region contained no discrepant cells, grabbing one cell." << endl);
    cout << xmin << " " << xmax << " " << ymin << " " << ymax << endl;
    xmax = min(width-1, 2);
    ymax = min(height-1, 2);
    xmin = 0;
    ymin = 0;
    scene_to_return = copyBox(xmin,ymin,xmax,ymax);
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

void Scene::findBestScoreForObject(int class_idx, int num_orientations, int * l_max_x, int * l_max_y, int * l_max_orient, double * l_max_score, int * l_max_i) {
  REQUIRE_VALID_CLASS(ms,class_idx);
  guardSceneModels(ms);

  vector<Mat> rotated_object_imgs;
  int numScales = 1;//11;
  int numOrientations = num_orientations;
  double scaleStepSize = 1.02;
  int etaS = 0;

  rotated_object_imgs.resize(numScales*numOrientations);
  double startScale = pow(scaleStepSize, -(numScales-1)/2);  
  double thisScale = startScale * pow(scaleStepSize, etaS);

  double p_scene_sigma = 2.0;

  Mat prepared_discrepancy;
  {
    prepared_discrepancy = discrepancy_density.clone();
    //GaussianBlur(prepared_discrepancy, prepared_discrepancy, cv::Size(0,0), p_scene_sigma);
    //normalizeForCrossCorrelation(ms, prepared_discrepancy, prepared_discrepancy);
  }
  Mat object_to_prepare = ms->config.class_scene_models[class_idx]->discrepancy_density.clone();

  double max_dim = max(object_to_prepare.rows, object_to_prepare.cols);

  Mat prepared_object = object_to_prepare;

/*
  Mat prepared_object = Mat(max_dim, max_dim, CV_64F);
  {
    int crows = object_to_prepare.rows;
    int ccols = object_to_prepare.cols;
    int tRy = (max_dim-crows)/2;
    int tRx = (max_dim-ccols)/2;
    
    for (int x = 0; x < max_dim; x++) {
      for (int y = 0; y < max_dim; y++) {
	int tx = x - tRx;
	int ty = y - tRy;
	if ( tx >= 0 && ty >= 0 && ty < crows && tx < ccols )  {
	  prepared_object.at<double>(y, x) = object_to_prepare.at<double>(ty, tx);
	} else {
	  prepared_object.at<double>(y, x) = 0.0;
	}
      }
    }
    //GaussianBlur(prepared_object, prepared_object, cv::Size(0,0), p_scene_sigma);
    //normalizeForCrossCorrelation(ms, prepared_object, prepared_object);
  }
*/
  
  //double po_l1norm = prepared_object.dot(Mat::ones(prepared_object.rows, prepared_object.cols, prepared_object.type()));
  double po_l2norm = prepared_object.dot(prepared_object);
  cout << "  po_l2norm: " << po_l2norm << endl;
  double overlap_thresh = 0.05;

  Size toBecome(max_dim, max_dim);

  //double globalMax = 0.0;
  int max_x = -1;
  int max_y = -1;
  int max_orient = -1;
  int max_score = -1;


  vector<SceneObjectScore> local_scores;

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
    //Point center = Point(max_dim/2.0, max_dim/2.0);
    Point center = Point(prepared_object.cols/2.0, prepared_object.rows/2.0);
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

  //imshow("test", rotated_object_imgs[thisOrient + etaS*numOrientations]);
  //waitKey(0);

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
	double model_score = 0.0;

/*
	if (output.at<double>(y,x) > overlap_thresh * po_l2norm) {
	  cout << output.at<double>(y,x) << "  running inference...";
	  double this_theta = -thisOrient * 2.0 * M_PI / numOrientations;
	  double x_m_tt, y_m_tt;
	  ms->config.scene->cellToMeters(x,y,&x_m_tt,&y_m_tt);
	  model_score = -ms->config.scene->scoreObjectAtPose(x_m_tt, y_m_tt, this_theta, class_idx);
	  cout << " score " << score << " max_score " << max_score << endl;
	}
*/
	if (output.at<double>(y,x) > overlap_thresh * po_l2norm) {
	  SceneObjectScore to_push;
	  to_push.x_c = x;
	  to_push.y_c = y;
	  cellToMeters(to_push.x_c, to_push.y_c, &(to_push.x_m), &(to_push.y_m));
	  to_push.orient_i = thisOrient;
	  to_push.theta_r = -(to_push.orient_i)* 2.0 * M_PI / numOrientations;
	  to_push.discrepancy_valid = true;
	  to_push.discrepancy_score = output.at<double>(y,x);
	  to_push.loglikelihood_valid = false;
	  to_push.loglikelihood_score = 0.0;
	  local_scores.push_back(to_push);
	}
  
	//if (model_score > max_score) 
	if (output.at<double>(y,x) > max_score) 
	{
	  //max_score = model_score;
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
  cout << "  discrepancy says: " << endl;
  cout << max_x << " " << max_y << " " << max_orient << " " << max_x_meters << " " << max_y_meters << " " << max_theta << endl << "max_score: " << max_score << endl;


  *l_max_x = -1;
  *l_max_y = -1;
  *l_max_score = -DBL_MAX;
  *l_max_orient = -1;
  *l_max_i = -1;
  std::sort (local_scores.begin(), local_scores.end(), compareDiscrepancyDescending);
  int to_check = min( int(ms->config.sceneDiscrepancySearchDepth), int(local_scores.size()) );
  for (int i = 0; i < to_check; i++) {
    if ( ! local_scores[i].loglikelihood_valid ) {
      // XXX score should return the delta of including vs not including
      local_scores[i].loglikelihood_score = ms->config.scene->scoreObjectAtPose(local_scores[i].x_m, local_scores[i].y_m, local_scores[i].theta_r, class_idx, overlap_thresh);
      local_scores[i].loglikelihood_valid = true;
      cout << "  running inference on " << i << "/" << local_scores.size() << " ... ds: " << local_scores[i].discrepancy_score << " ls: " << local_scores[i].loglikelihood_score << endl;
    }
  }

  for (int i = 0; i < local_scores.size(); i++) {
    if ( (local_scores[i].loglikelihood_valid) && (local_scores[i].loglikelihood_score > *l_max_score) ) {
      cout << " score " << local_scores[i].loglikelihood_score << " l_max_score " << *l_max_score << endl;
      *l_max_score = local_scores[i].loglikelihood_score;
      *l_max_x = local_scores[i].x_c;
      *l_max_y = local_scores[i].y_c;
      *l_max_orient = local_scores[i].orient_i;
      *l_max_i = i;
    }
  }
  if (*l_max_i >= ms->config.sceneDiscrepancySearchDepth - 25) {
    ROS_ERROR_STREAM("Take note that the best one was near our search limit; maybe you need to increase the search depth.  l_max_i: " << *l_max_i << " search depth: " << ms->config.sceneDiscrepancySearchDepth);
  }
}

void Scene::tryToAddObjectToScene(int class_idx) {
  REQUIRE_VALID_CLASS(ms,class_idx);
  guardSceneModels(ms);

  int num_orientations = 37;

  int l_max_x = -1;
  int l_max_y = -1;
  int l_max_orient = -1;
  double l_max_score = -1;
  int l_max_i = -1;

  findBestScoreForObject(class_idx, num_orientations, &l_max_x, &l_max_y, &l_max_orient, &l_max_score, &l_max_i);

  double l_max_theta = -l_max_orient * 2.0 * M_PI / num_orientations;
  double l_max_x_meters, l_max_y_meters;
  cellToMeters(l_max_x, l_max_y, &l_max_x_meters, &l_max_y_meters);
  cout << "  loglikelihood says: " << endl;
  cout << l_max_x << " " << l_max_y << " " << l_max_orient << " " << l_max_x_meters << " " << l_max_y_meters << " " << 
    l_max_theta << endl << "l_max_score: " << l_max_score << " l_max_i: " << l_max_i << endl;

  //if (max_x > -1)
  if (l_max_x > -1)
  {
    if (l_max_score > 0) {
      cout << "best detection made an improvement..." << endl;
      cout << "adding object." << endl;
      ms->pushWord("sceneAddPredictedFocusedObject");
  /*
      ms->pushWord(make_shared<DoubleWord>(max_theta));
      ms->pushWord(make_shared<DoubleWord>(max_y_meters));
      ms->pushWord(make_shared<DoubleWord>(max_x_meters));
  */
      ms->pushWord(make_shared<DoubleWord>(l_max_theta));
      ms->pushWord(make_shared<DoubleWord>(l_max_y_meters));
      ms->pushWord(make_shared<DoubleWord>(l_max_x_meters));
    } else {
      cout << "best detection made things worse alone..." << endl;
      cout << "should NOT adding object but for now we are..." << endl;
      ms->pushWord("sceneAddPredictedFocusedObject");
      ms->pushWord(make_shared<DoubleWord>(l_max_theta));
      ms->pushWord(make_shared<DoubleWord>(l_max_y_meters));
      ms->pushWord(make_shared<DoubleWord>(l_max_x_meters));
    }
  } else {
    cout << "Did not find a valid cell... not adding object." << endl;
  }
}

void Scene::findBestObjectAndScore(int * class_idx, int num_orientations, int * l_max_x, int * l_max_y, int * l_max_orient, double * l_max_score, int * l_max_i) {
  guardSceneModels(ms);

  *l_max_x = -1;
  *l_max_y = -1;
  *l_max_orient = -1;
  *l_max_score = -1;
  *l_max_i = -1;

  for (int j = 0; j < ms->config.classLabels.size(); j++) {
    int j_max_x = -1;
    int j_max_y = -1;
    int j_max_orient = -1;
    double j_max_score = -1;
    int j_max_i = -1;

    findBestScoreForObject(j, num_orientations, &j_max_x, &j_max_y, &j_max_orient, &j_max_score, &j_max_i);
    if (j_max_score > *l_max_score) {
      *l_max_score = j_max_score;
      *l_max_x = j_max_x;
      *l_max_y = j_max_y;
      *l_max_orient = j_max_orient;
      *l_max_i = j_max_i;

      cout << "  findBestObjectAndScore, class " << j << " " << ms->config.classLabels[j] << " : " << endl;
      cout << l_max_x << " " << l_max_y << " " << *l_max_orient << " " << "l_max_score: " << *l_max_score << " l_max_i: " << *l_max_i << endl;
    } else {
    }
  }
}


void Scene::removeObjectFromPredictedMap(shared_ptr<SceneObject> obj) {
  int idx = -1;
  for (int i = 0; i < predicted_objects.size(); i++) {
    if (predicted_objects[i] == obj) {
      idx = i;
      break;
    }
  } 
  if (idx != -1) {
    predicted_objects.erase(predicted_objects.begin() + idx);
  }
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

shared_ptr<SceneObject> Scene::addPredictedObject(double x, double y, double theta, int class_idx) {
  eePose topass = eePose::identity().applyRPYTo(theta,0,0); 
  topass.px = x;
  topass.py = y;
  shared_ptr<SceneObject> topush = make_shared<SceneObject>(topass, class_idx,  ms->config.classLabels[class_idx], PREDICTED);
  predicted_objects.push_back(topush);
  return topush;
}

double Scene::scoreObjectAtPose(double x, double y, double theta, int class_idx, double threshold) {
  shared_ptr<Scene> object_scene = ms->config.class_scene_models[class_idx];

/*
  shared_ptr<SceneObject> obj = addPredictedObject(x, y, theta, class_idx);
  composePredictedMap();
  double score = ms->config.scene->computeScore();
  removeObjectFromPredictedMap(obj);
*/

  shared_ptr<SceneObject> obj = addPredictedObject(x, y, theta, class_idx);
  double score = recomputeScore(obj, threshold);
  removeObjectFromPredictedMap(obj);

  //cout << "zzz: " << score << endl;

  return score;
}

int Scene::safeAt(int x, int y) {
  return ( (x >= 0) && (x < width) && (y >= 0) && (y < height) );
}
int Scene::safeBilinAt(int x, int y) {
  return ( (x >= 2) && (x < width-2) && (y >= 2) && (y < height-2) );
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

WORD(SceneScoreObjectAtPose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  REQUIRE_FOCUSED_CLASS(ms,tfc);
  guardSceneModels(ms);

  double x_in=0, y_in=0, theta_in=0;
  GET_NUMERIC_ARG(ms, theta_in);
  GET_NUMERIC_ARG(ms, y_in);
  GET_NUMERIC_ARG(ms, x_in);

  double score = ms->config.scene->scoreObjectAtPose(x_in, y_in, theta_in, tfc);

  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(score);
  ms->pushWord(newWord);
  
}
END_WORD
REGISTER_WORD(SceneScoreObjectAtPose)


WORD(SceneComputeScore)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double score = ms->config.scene->computeScore();
  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(score);
  ms->pushWord(newWord);

}
END_WORD
REGISTER_WORD(SceneComputeScore)



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

WORD(SceneRenderScene)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->evaluateProgram("sceneRenderObservedMap sceneRenderBackgroundMap sceneRenderPredictedMap sceneRenderDiscrepancy");
}
END_WORD
REGISTER_WORD(SceneRenderScene)

WORD(SceneLoadObjectModel)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string object_name;
  GET_STRING_ARG(ms, object_name);
  
  std::stringstream buffer;
  buffer << "\"" << sceneModelFile(ms, object_name) << "\" sceneLoadSceneRaw";
  ms->evaluateProgram(buffer.str());
}
END_WORD
REGISTER_WORD(SceneLoadObjectModel)



WORD(SceneLoadFocusedObjectModel)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::stringstream buffer;
  buffer << "\"" << sceneModelFile(ms, ms->config.focusedClassLabel) << "\" sceneLoadSceneRaw";
  ms->evaluateProgram(buffer.str());
}
END_WORD
REGISTER_WORD(SceneLoadFocusedObjectModel)

WORD(SceneLoadSceneRaw)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string file;
  GET_STRING_ARG(ms, file);
  cout << "Loading scene " << file << endl;
  ms->config.scene->loadFromFile(file);
  ms->pushWord("sceneRenderScene");
}
END_WORD
REGISTER_WORD(SceneLoadSceneRaw)



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
  ms->pushWord("sceneRenderScene");
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
  ms->pushWord("sceneRenderObservedMap");
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
  REQUIRE_FOCUSED_CLASS(ms, tfc);
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

  ms->config.scene->addPredictedObject(x_in, y_in, theta_in, tfc);
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
  double p_cell_width = 0.0025; //0.01;
  int p_width = 1001; // 601;
  int p_height = 1001; // 601;
  ms->config.scene = make_shared<Scene>(ms, p_width, p_height, p_cell_width);
  ms->pushWord("sceneRenderScene");
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
  ms->config.scene->background_map = ms->config.scene->observed_map->copy();
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

WORD(SceneSetFocusedSceneStdDevY)
virtual void execute(std::shared_ptr<MachineState> ms) {
  REQUIRE_FOCUSED_CLASS(ms,tfc);
  guardSceneModels(ms);

  double stddev = 0;
  GET_NUMERIC_ARG(ms, stddev);

  int t_height = ms->config.class_scene_models[tfc]->observed_map->height;
  int t_width = ms->config.class_scene_models[tfc]->observed_map->width;
  for (int y = 0; y < t_height; y++) {
    for (int x = 0; x < t_width; x++) {
      ms->config.class_scene_models[tfc]->observed_map->refAtCell(x,y)->blue.sigmasquared = pow(stddev, 2);
    }
  }
}
END_WORD
REGISTER_WORD(SceneSetFocusedSceneStdDevY)

WORD(SceneSetFocusedSceneStdDevColor)
virtual void execute(std::shared_ptr<MachineState> ms) {
  REQUIRE_FOCUSED_CLASS(ms,tfc);
  guardSceneModels(ms);

  double stddev = 0;
  GET_NUMERIC_ARG(ms, stddev);

  int t_height = ms->config.class_scene_models[tfc]->observed_map->height;
  int t_width = ms->config.class_scene_models[tfc]->observed_map->width;
  for (int y = 0; y < t_height; y++) {
    for (int x = 0; x < t_width; x++) {
      ms->config.class_scene_models[tfc]->observed_map->refAtCell(x,y)->red.sigmasquared = pow(stddev, 2);
      ms->config.class_scene_models[tfc]->observed_map->refAtCell(x,y)->green.sigmasquared = pow(stddev, 2);
    }
  }
}
END_WORD
REGISTER_WORD(SceneSetFocusedSceneStdDevColor)

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
  for (int px = ms->config.grayTop.x; px < ms->config.grayBot.x; px++) {
    for (int py = ms->config.grayTop.y; py < ms->config.grayBot.y; py++) {
  //for (int px = 0; px < imW; px++) 
    //for (int py = 0; py < imH; py++) 
      if (isInGripperMask(ms, px, py)) {
	continue;
      }
      double x, y;
      double z = ms->config.trueEEPose.position.z + ms->config.currentTableZ;
      pixelToGlobal(ms, px, py, z, &x, &y);
      if (1) {
	// single sample update
	int i, j;
	ms->config.scene->observed_map->metersToCell(x, y, &i, &j);
	GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(i, j);
	if (cell != NULL) {
	  Vec3b pixel = wristViewYCbCr.at<Vec3b>(py, px);
	  cell->newObservation(pixel);
	}
      } else {
/*
	Vec3b pixel = wristViewYCbCr.at<Vec3b>(py, px);

	// bilinear update
	double _i, _j;
	ms->config.scene->observed_map->metersToCell(x, y, &_i, &_j);

	if (ms->config.scene->safeBilinAt(_i,_j)) {

	  double i = min( max(0.0, _i), double(width-1));
	  double j = min( max(0.0, _j), double(height-1));

	  // -2 makes for appropriate behavior on the upper boundary
	  double i0 = std::min( std::max(0.0, floor(i)), double(width-2));
	  double i1 = i0+1;

	  double j0 = std::min( std::max(0.0, floor(j)), double(height-2));
	  double j1 = j0+1;

	  double wi0 = i1-i;
	  double wi1 = i-i0;

	  double wj0 = j1-j;
	  double wj1 = j-j0;

	  GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(i0, j0);
	  cell->newObservation(pixel, wi0*wj0);

	  GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(i1, j0);
	  cell->newObservation(pixel, wi1*wj0);

	  GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(i0, j1);
	  cell->newObservation(pixel, wi0*wj1);

	  GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(i1, j1);
	  cell->newObservation(pixel, wi1*wj1);
	}
*/
      }
    }
  }  
  ms->config.scene->observed_map->recalculateMusAndSigmas(ms);
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
  ms->config.scene->composePredictedMap();
  ms->pushWord("sceneRenderPredictedMap");
}
END_WORD
REGISTER_WORD(SceneComposePredictedMap)

WORD(SceneComposePredictedMapThreshed)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double threshold = 0.0;
  GET_NUMERIC_ARG(ms, threshold);
  ms->config.scene->composePredictedMap(threshold);
  ms->pushWord("sceneRenderPredictedMap");
}
END_WORD
REGISTER_WORD(SceneComposePredictedMapThreshed)

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
  ms->config.scene->discrepancy->rgbDiscrepancyMuToMat(ms, image);
  ms->config.discrepancyWindow->updateImage(image);

  Mat densityImage = ms->config.scene->discrepancy_density.clone();

  for (int x = 0; x < ms->config.scene->width; x++) {
    for (int y = 0; y < ms->config.scene->height; y++) {
      if ((ms->config.scene->predicted_map->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold) && (ms->config.scene->observed_map->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold)) {
      } else {
	densityImage.at<double>(y, x) = 0;
      }
    }
  }

  
  ms->config.discrepancyDensityWindow->updateImage(ms->config.scene->discrepancy_density);
}
END_WORD
REGISTER_WORD(SceneRenderDiscrepancy)



WORD(SceneGrabDiscrepantCropAsClass)
virtual void execute(std::shared_ptr<MachineState> ms) {
  REQUIRE_FOCUSED_CLASS(ms,tfc);

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

WORD(SceneExponentialAverageObservedIntoBackground)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double fraction;
  GET_NUMERIC_ARG(ms, fraction);

  ms->config.scene->background_map->multS(1.0-fraction);
  shared_ptr<GaussianMap> temp = ms->config.scene->observed_map->copy();
  temp->multS(fraction);
  ms->config.scene->background_map->addM(temp);
}
END_WORD
REGISTER_WORD(SceneExponentialAverageObservedIntoBackground)

WORD(SceneSmoothDiscrepancyDensity)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double sigma;
  GET_NUMERIC_ARG(ms, sigma);
  ms->config.scene->smoothDiscrepancyDensity(sigma);
}
END_WORD
REGISTER_WORD(SceneSmoothDiscrepancyDensity)

WORD(ScenePushSceneObjectPose)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int so_idx = 0;
  GET_INT_ARG(ms, so_idx);
  so_idx = min( max( int(0), int(so_idx)), int(ms->config.scene->predicted_objects.size())-1 );

  if ( so_idx < ms->config.scene->predicted_objects.size() ) {
    cout << "scenePushSceneObjectPose: there are " << ms->config.scene->predicted_objects.size() << " objects so using idx " << so_idx << endl;
    ms->pushWord(make_shared<EePoseWord>(ms->config.scene->predicted_objects[so_idx]->scene_pose));
  } else {
    cout << "scenePushSceneObjectPose: there are " << ms->config.scene->predicted_objects.size() << " objects so " << so_idx << " is invalid..." << endl;
  }
}
END_WORD
REGISTER_WORD(ScenePushSceneObjectPose)

WORD(ScenePushNumSceneObjects)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->pushWord(make_shared<IntegerWord>(ms->config.scene->predicted_objects.size()));
}
END_WORD
REGISTER_WORD(ScenePushNumSceneObjects)

WORD(SceneMapSceneObject)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int to_map = 0;
  GET_INT_ARG(ms, to_map);
  REQUIRE_VALID_SCENE_OBJECT(ms, to_map);

  BoxMemory box;
  box.cameraPose = ms->config.scene->predicted_objects[to_map]->scene_pose;

  shared_ptr<SceneObject> tso = ms->config.scene->predicted_objects[to_map];
  shared_ptr<Scene> tso_s = ms->config.class_scene_models[ tso->labeled_class_index ];
  box.top = ms->config.scene->predicted_objects[to_map]->scene_pose.minusP(eePose(0.5 * tso_s->width * tso_s->cell_width, 0.5 * tso_s->height * tso_s->cell_width, 0, 0,0,0,0));
  box.bot = ms->config.scene->predicted_objects[to_map]->scene_pose.plusP(eePose(0.5 * tso_s->width * tso_s->cell_width, 0.5 * tso_s->height * tso_s->cell_width, 0, 0,0,0,0));

  box.bTop = cv::Point(0,0);
  box.bBot = cv::Point(1,1);
  mapxyToij(ms, box.top.px, box.top.py, &(box.bTop.x), &(box.bTop.y));
  mapxyToij(ms, box.bot.px, box.bot.py, &(box.bBot.x), &(box.bBot.y));

  box.centroid.px = (box.top.px + box.bot.px) * 0.5;
  box.centroid.py = (box.top.py + box.bot.py) * 0.5;
  box.centroid.pz = (box.top.pz + box.bot.pz) * 0.5;
  box.cameraTime = ros::Time::now();

  box.labeledClassIndex = tso->labeled_class_index;

  box.lockStatus = CENTROID_LOCK;
  
  int i, j;
  mapxyToij(ms, box.centroid.px, box.centroid.py, &i, &j);

  // this only does the timestamp to avoid obsessive behavior
  mapBox(ms, box);
  
  if ( !positionIsSearched(ms, box.centroid.px, box.centroid.py) || 
       !isBoxMemoryIkPossible(ms, box) ) 
  {
    cout << "Not mapping box... " << " searched: " << positionIsSearched(ms, box.centroid.px, box.centroid.py) << " ikPossible: " << isBoxMemoryIkPossible(ms, box) << " " << box.cameraPose << endl;
    return;
  } else {
    vector<BoxMemory> newMemories;
    for (int i = 0; i < ms->config.blueBoxMemories.size(); i++) {
      if (!boxMemoryIntersectCentroid(box, ms->config.blueBoxMemories[i])) {
	newMemories.push_back(ms->config.blueBoxMemories[i]);
      }
    }
    newMemories.push_back(box);
    ms->config.blueBoxMemories = newMemories;
  }
}
END_WORD
REGISTER_WORD(SceneMapSceneObject)

WORD(SceneZeroBox)
virtual void execute(std::shared_ptr<MachineState> ms) {
  // point width height sceneZeroBox
  eePose g_pose;
  GET_ARG(ms, EePoseWord, g_pose);

  double g_width, g_height;
  GET_NUMERIC_ARG(ms, g_height);
  GET_NUMERIC_ARG(ms, g_width);

  int t_x_c, t_y_c;
  ms->config.scene->metersToCell(g_pose.px - g_width/2.0, g_pose.py - g_height/2.0, &t_x_c, &t_y_c);
  int b_x_c, b_y_c;
  ms->config.scene->metersToCell(g_pose.px + g_width/2.0, g_pose.py + g_height/2.0, &b_x_c, &b_y_c);

  cout << "sceneZeroBox: " << g_pose << " " << g_width << " " << g_height << endl;

  ms->config.scene->observed_map->zeroBox(t_x_c, t_y_c, b_x_c, b_y_c);
}
END_WORD
REGISTER_WORD(SceneZeroBox)


WORD(EePoseGetPoseRelativeTo)
virtual void execute(std::shared_ptr<MachineState> ms) {
/* call with "base_pose to_apply EePoseGetPoseRelativeTo" */
  eePose to_apply;
  GET_ARG(ms, EePoseWord, to_apply);

  eePose base_pose;
  GET_ARG(ms, EePoseWord, base_pose);

  ms->pushWord(make_shared<EePoseWord>(base_pose.getPoseRelativeTo(to_apply)));
}
END_WORD
REGISTER_WORD(EePoseGetPoseRelativeTo)

WORD(EePoseApplyRelativePoseTo)
virtual void execute(std::shared_ptr<MachineState> ms) {
/* call with "to_apply base_pose EePoseApplyRelativePoseTo" */
  eePose base_pose;
  GET_ARG(ms, EePoseWord, base_pose);

  eePose to_apply;
  GET_ARG(ms, EePoseWord, to_apply);

  ms->pushWord(make_shared<EePoseWord>(to_apply.applyAsRelativePoseTo(base_pose)));
}
END_WORD
REGISTER_WORD(EePoseApplyRelativePoseTo)



WORD(SceneSetDiscrepancyModeDot)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.discrepancyMode = DISCREPANCY_DOT;
}
END_WORD
REGISTER_WORD(SceneSetDiscrepancyModeDot)

WORD(SceneSetDiscrepancyModePoint)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.discrepancyMode = DISCREPANCY_POINT;
}
END_WORD
REGISTER_WORD(SceneSetDiscrepancyModePoint)




/* 
WORD(GaussianMapCalibrateVanishingPoint)
virtual void execute(std::shared_ptr<MachineState> ms) {
// XXX
}
END_WORD
REGISTER_WORD(GaussianMapCalibrateVanishingPoint)

WORD(GaussianMapCalibrateMagnifications)
virtual void execute(std::shared_ptr<MachineState> ms) {
// XXX
}
END_WORD
REGISTER_WORD(GaussianMapCalibrateMagnifications)

WORD(GaussianMapCompilePointCloud)
virtual void execute(std::shared_ptr<MachineState> ms) {
// XXX
}
END_WORD
REGISTER_WORD(GaussianMapCompilePointCloud)



WORD(GaussianMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
}
END_WORD
REGISTER_WORD(GaussianMap)

WORD(GaussianMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
}
END_WORD
REGISTER_WORD(GaussianMap)

WORD(GaussianMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
}
END_WORD
REGISTER_WORD(GaussianMap)

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




