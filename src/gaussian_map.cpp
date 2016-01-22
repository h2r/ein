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

void GaussianMapCell::recalculateMusAndSigmas(shared_ptr<MachineState> ms) {
  red.recalculateMusAndSigmas(ms);
  green.recalculateMusAndSigmas(ms);
  blue.recalculateMusAndSigmas(ms);
  z.recalculateMusAndSigmas(ms);
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



// double computeEnergy(GaussianMapChannel & channel1, double p) {
//   double safesigmasquared1 = safeSigmaSquared(channel1.sigmasquared);
//   double term1 = - pow((p - channel1.mu), 2)  / (2 * safesigmasquared1);
//   // XXX maybe term2 should be cached in the map
//   double term2 = -log(sqrt(2 * M_PI * safesigmasquared1));
//   double result = term1 + term2; 
//   return result;
// }
// double computeEnergy(GaussianMapChannel & channel1, GaussianMapChannel & channel2) {
//   return computeEnergy(channel1, channel2.mu);
// }


double computeLogLikelihood(GaussianMapChannel & channel1, GaussianMapChannel & channel2) {
  /*double total = 0.0;
  for (int i = 0; i < 256; i++) {
    total += exp(computeEnergy(channel1, i));
  }
  double likelihood = computeEnergy(channel1, channel2);

  return likelihood - log(total);*/
  double safesigmasquared1 = safeSigmaSquared(channel1.sigmasquared);
  double energy = 1.0 / (2.0 * safesigmasquared1) * pow(channel2.mu - channel1.mu, 2);
  double normalized = -0.5 * log(2 * M_PI) - 0.5 * log(safesigmasquared1) - energy;
  return normalized;
  
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
  z.multS(scalar);
}  

void _GaussianMapCell::addC(_GaussianMapCell * cell) {
  red.addC(&(cell->red));
  green.addC(&(cell->green));
  blue.addC(&(cell->blue));
  z.addC(&(cell->z));
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

void _GaussianMapCell::newObservation(Vec3b obs) {
  red.counts += obs[2];
  green.counts += obs[1];
  blue.counts += obs[0];
  red.squaredcounts += pow(obs[2], 2);
  green.squaredcounts += pow(obs[1], 2);
  blue.squaredcounts += pow(obs[0], 2);
  red.samples += 1;
  green.samples += 1;
  blue.samples += 1;
}

void _GaussianMapCell::newObservation(Vec3b obs, double zobs) {
  newObservation(obs);
  z.counts += zobs;
  z.squaredcounts += pow(zobs, 2);
  z.samples += 1;
}

void GaussianMap::reallocate() {
  if (width <= 0 || height <= 0) {
    cout << "GaussianMap area error: tried to allocate width, height: " << width << " " << height << endl;
  } else {

    if (width % 2 == 0) {
      cout << "GaussianMap parity error: tried to allocate width: " << width << " " << height << endl;
      width = width+1;
    } 
    if (height % 2 == 0) {
      cout << "GaussianMap parity error: tried to allocate height: " << width << " " << height << endl;
      height = height+1;
    } 

    if (cells != NULL) {
      delete cells;

    }
    cells = new GaussianMapCell[width*height];
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
    ROS_ERROR_STREAM("GaussianMapCell::refAtCell: Bad x. " << x << " width: " << width);
    return NULL;
  }
  if (y < 0 || y >= height) {
    ROS_ERROR_STREAM("GaussianMapCell::refAtCell: Bad y. " << y << " height: " << height);
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
      cout << "successfully loaded " << numLoadedCells << " GaussianMapCells. ";
      cout << "width: " << width << " height: " << height << endl;
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
      if (refAtCell(x, y)->red.samples > 0) {
	refAtCell(x, y)->recalculateMusAndSigmas(ms);
      }
    }
  }
}

#define CELLREF_EQUALS_VEC3(cellRef, vec, statistic) \
  cellRef->blue.statistic = vec[0]; \
  cellRef->green.statistic = vec[1]; \
  cellRef->red.statistic = vec[2]; \

#define VEC3_EQUALS_CELLREF(vec, cellRef, statistic) \
  vec[0] = cellRef->blue.statistic; \
  vec[1] = cellRef->green.statistic; \
  vec[2] = cellRef->red.statistic ; \

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

void GaussianMap::rgbSigmaToMat(Mat& out) {
  Mat big = Mat(height, width, CV_8UC3);
  double max_val = -DBL_MAX;
  double min_val = DBL_MAX;
  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {

      double bval = sqrt(refAtCell(x,y)->blue.sigmasquared);
      if (bval > max_val) {
	max_val = bval;
      }
      if (bval < min_val) {
	min_val = bval;
      }
      big.at<Vec3b>(y,x)[0] = uchar(sqrt(refAtCell(x,y)->blue.sigmasquared) * 4);
      big.at<Vec3b>(y,x)[1] = uchar(sqrt(refAtCell(x,y)->green.sigmasquared) * 4);
      big.at<Vec3b>(y,x)[2] = uchar(sqrt(refAtCell(x,y)->red.sigmasquared) * 4);
    }
  }
  //cout << "max: " << max_val << endl;
  //cout << "min: " << min_val << endl;
  out = big;
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

void GaussianMap::rgbSquaredCountsToMat(Mat& out) {
  out = Mat(height, width, CV_64FC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<Vec3d>(y,x)[0] = refAtCell(x,y)->blue.squaredcounts;
      out.at<Vec3d>(y,x)[1] = refAtCell(x,y)->green.squaredcounts;
      out.at<Vec3d>(y,x)[2] = refAtCell(x,y)->red.squaredcounts;
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

shared_ptr<Scene> Scene::copy() {
  shared_ptr<Scene> toReturn = std::make_shared<Scene>(ms, width, height, cell_width);
  toReturn->annotated_class_name = annotated_class_name;
  toReturn->predicted_class_name = predicted_class_name;
  toReturn->background_map = background_map->copy();
  toReturn->predicted_map = predicted_map->copy();
  toReturn->observed_map = observed_map->copy();
  toReturn->discrepancy = discrepancy->copy();

  toReturn->predicted_segmentation = predicted_segmentation.clone();
  toReturn->discrepancy_magnitude = discrepancy_magnitude.clone();
  toReturn->discrepancy_density = discrepancy_density.clone();
  toReturn->background_pose = background_pose;
  toReturn->discrepancy_before = discrepancy_before.clone();
  toReturn->discrepancy_after = discrepancy_after.clone();
  
  toReturn->predicted_objects.reserve(predicted_objects.size());
  std::copy(predicted_objects.begin(), predicted_objects.end(), back_inserter(toReturn->predicted_objects));

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

  if (w % 2 == 0) {
    cout << "Scene parity error: tried to allocate width: " << w << " so adding 1..." << endl;
    w = w+1;
  }
  if (h % 2 == 0) {
    cout << "Scene parity error: tried to allocate height: " << h << " so adding 1..." << endl;
    h = h+1;
  }


  predicted_class_name = string("NONAME");
  annotated_class_name = string("NONAME");
  ms = _ms;
  width = w;
  height = h;
  x_center_cell = (width-1)/2;
  y_center_cell = (height-1)/2;
  cell_width = cw;
  background_pose = eePose::identity();
  predicted_objects.resize(0);
  reallocate();
}

void Scene::reallocate() {
  if (width <= 0 || height <= 0) {
    cout << "Scene area error: tried to allocate width, height: " << width << " " << height << endl;
  } else {
    if (width % 2 == 0) {
      cout << "Scene parity error: tried to allocate width: " << width << " " << height << endl;
      width = width+1;
    } 
    if (height % 2 == 0) {
      cout << "Scene parity error: tried to allocate height: " << width << " " << height << endl;
      height = height+1;
    } 
  }
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



void Scene::initializePredictedMapWithBackground() {
 for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      *(predicted_map->refAtCell(x,y)) = *(background_map->refAtCell(x,y));
    }
  }
}


void Scene::composePredictedMap(double threshold) {
  //REQUIRE_FOCUSED_CLASS(ms, tfc);
  // XXX
  // choose the argMAP distribution
  //   assign that color to the predicted map
  //   assign the source to the segmentation
  //
  // Currently uses a "fallen leaves" model of composition, assuming objects
  //   are painted onto the scene in reverse order of discovery 

  initializePredictedMapWithBackground();
 
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
	    predicted_map->refAtCell(x,y)->recalculateMusAndSigmas(ms);
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
  return measureScoreRegion(0,0,width-1,height-1);
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



    double top_meters_object_x;
    double top_meters_object_y;
    double top_meters_scene_x, top_meters_scene_y;
    cellToMeters(top_x, top_y, &top_meters_scene_x, &top_meters_scene_y);
    eePose top_cell_eep = eePose::identity();
    top_cell_eep.px = top_meters_scene_x;
    top_cell_eep.py = top_meters_scene_y;
    top_cell_eep.pz = 0.0;
    eePose top_eep_object = top_cell_eep.getPoseRelativeTo(tsob->scene_pose);
    top_meters_object_x = top_eep_object.px;
    top_meters_object_y = top_eep_object.py;


    double nextx_meters_object_x;
    double nextx_meters_object_y;
    double nextx_meters_scene_x, nextx_meters_scene_y;
    cellToMeters(top_x + 1, top_y + 0, &nextx_meters_scene_x, &nextx_meters_scene_y);
    eePose nextx_cell_eep = eePose::identity();
    nextx_cell_eep.px = nextx_meters_scene_x;
    nextx_cell_eep.py = nextx_meters_scene_y;
    nextx_cell_eep.pz = 0.0;
    eePose nextx_eep_object = nextx_cell_eep.getPoseRelativeTo(tsob->scene_pose);
    nextx_meters_object_x = nextx_eep_object.px;
    nextx_meters_object_y = nextx_eep_object.py;

    double nexty_meters_object_x;
    double nexty_meters_object_y;
    double nexty_meters_scene_x, nexty_meters_scene_y;
    cellToMeters(top_x + 0, top_y + 1, &nexty_meters_scene_x, &nexty_meters_scene_y);
    eePose nexty_cell_eep = eePose::identity();
    nexty_cell_eep.px = nexty_meters_scene_x;
    nexty_cell_eep.py = nexty_meters_scene_y;
    nexty_cell_eep.pz = 0.0;
    eePose nexty_eep_object = nexty_cell_eep.getPoseRelativeTo(tsob->scene_pose);
    nexty_meters_object_x = nexty_eep_object.px;
    nexty_meters_object_y = nexty_eep_object.py;

    double diffx_x = nextx_meters_object_x - top_meters_object_x;
    double diffx_y = nextx_meters_object_y - top_meters_object_y;

    double diffy_x = nexty_meters_object_x - top_meters_object_x;
    double diffy_y = nexty_meters_object_y - top_meters_object_y;


    int xidx = -1;
    int nerrors = 0;

    for (int x = top_x; x < bot_x; x++) {
      xidx++;
      int yidx = -1;
      for (int y = top_y; y < bot_y; y++) {
	yidx++;
	if (!safeAt(x,y)) {
	  continue;
	} 

	/*double meters_scene_x, meters_scene_y;
	cellToMeters(x, y, &meters_scene_x, &meters_scene_y);

	

	eePose cell_eep = eePose::identity();
	cell_eep.px = meters_scene_x;
	cell_eep.py = meters_scene_y;
	cell_eep.pz = 0.0;
	eePose eep_object = cell_eep.getPoseRelativeTo(tsob->scene_pose);
	double meters_object_x1 = eep_object.px;
	double meters_object_y1 = eep_object.py;*/

	double meters_object_x = top_meters_object_x + diffx_x * xidx + diffy_x * yidx;
	double meters_object_y = top_meters_object_y + diffx_y * xidx + diffy_y * yidx;
	
	/*if (fabs(meters_object_x1 - meters_object_x) > EPSILON) {
	  cout << "xidx: " << xidx << " yidx: " << yidx << " meters_object_x1: " << meters_object_x1 << " new: " << meters_object_x << endl;
	  //cout << "diffx_x: " << diffx_x << " diffx_y: " << diffx_y << " diffy_x: " << diffy_x << " diffy_y: " << diffy_y << endl;
	  nerrors += 1;
	}

	if (fabs(meters_object_y1 - meters_object_y) > EPSILON) {
	  cout << "xidx: " << xidx << " yidx: " << yidx << " meters_object_y1: " << meters_object_y1 << " new: " << meters_object_y << endl;
	  nerrors += 1;
	}

	if (nerrors > 100) {
	  assert(0);
	  }*/
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

CONFIG_GETTER_DOUBLE(SceneScoreThresh, ms->config.scene_score_thresh)
CONFIG_SETTER_DOUBLE(SceneSetScoreThresh, ms->config.scene_score_thresh)

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

  double p_discrepancy_thresh = ms->config.scene_score_thresh;
  double overlap_thresh = 0.05;
  if (ms->config.currentSceneClassificationMode == SC_DISCREPANCY_THEN_LOGLIKELIHOOD) {
  } else if (ms->config.currentSceneClassificationMode == SC_DISCREPANCY_ONLY) {

    /* 
    // l2 normalizing might mess up comparison on the positive regions vs negative... 
    double po_safe_l2norm = sqrt( std::max(1.0e-12, prepared_object.dot(prepared_object)) );
    prepared_object = prepared_object / po_safe_l2norm;
    {
      double po_l1norm = prepared_object.dot(Mat::ones(prepared_object.rows, prepared_object.cols, prepared_object.type()));
      prepared_object = prepared_object - po_l1norm;
      double po_meanless_l2norm = std::max(1.0e-12, prepared_object.dot(prepared_object));
      prepared_object = prepared_object / po_meanless_l2norm;
      cout << "TTTTTT: " << po_l1norm << " " << po_meanless_l2norm << endl;
    }
    {
      double pd_l1norm = prepared_discrepancy.dot(Mat::ones(prepared_discrepancy.rows, prepared_discrepancy.cols, prepared_discrepancy.type()));
      prepared_discrepancy = prepared_discrepancy - pd_l1norm;
      double pd_meanless_l2norm = std::max(1.0e-12, prepared_discrepancy.dot(prepared_discrepancy));
      prepared_discrepancy = prepared_discrepancy / pd_meanless_l2norm;
    }
    overlap_thresh = -DBL_MAX; 
    */
  } else {
    assert(0);
  }

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

  Size toBecome(max_dim, max_dim);

  //double globalMax = 0.0;
  int max_x = -1;
  int max_y = -1;
  int max_orient = -1;
  double max_score = -DBL_MAX;


  vector<SceneObjectScore> local_scores;
  local_scores.reserve(1e5);
  int pushed = 0;

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
//cout << rot_mat << rot_mat.size() << endl;
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

	  if (ms->config.currentSceneClassificationMode == SC_DISCREPANCY_THEN_LOGLIKELIHOOD) {
	    to_push.loglikelihood_valid = false;
	    to_push.loglikelihood_score = 0.0;
	  } else if (ms->config.currentSceneClassificationMode == SC_DISCREPANCY_ONLY) {
	    to_push.loglikelihood_valid = true;
	    to_push.loglikelihood_score = to_push.discrepancy_score;
	  } else {
	    assert(0);
	  }

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

  cout << "  local_scores size:  " << local_scores.size() << endl;

  //cout << prepared_discrepancy << prepared_object ;
  double max_theta = -max_orient * 2.0 * M_PI / numOrientations;
  double max_x_meters, max_y_meters;
  cellToMeters(max_x, max_y, &max_x_meters, &max_y_meters);
  cout << "  discrepancy says: " << endl;
  cout << max_x << " " << max_y << " " << max_orient << " " << max_x_meters << " " << max_y_meters << " " << max_theta << endl << "max_score: " << max_score << endl;
  cout << "  currentSceneClassificationMode: " << ms->config.currentSceneClassificationMode << endl;


  *l_max_x = -1;
  *l_max_y = -1;
  //*l_max_score = -DBL_MAX;
  *l_max_orient = -1;
  *l_max_i = -1;

  if (ms->config.currentSceneClassificationMode == SC_DISCREPANCY_THEN_LOGLIKELIHOOD) {
    std::sort(local_scores.begin(), local_scores.end(), compareDiscrepancyDescending);
    int to_check = min( int(ms->config.sceneDiscrepancySearchDepth), int(local_scores.size()) );
    for (int i = 0; i < to_check; i++) {
      if ( ! local_scores[i].loglikelihood_valid ) {
	// XXX score should return the delta of including vs not including
	local_scores[i].loglikelihood_score = ms->config.scene->scoreObjectAtPose(local_scores[i].x_m, local_scores[i].y_m, local_scores[i].theta_r, class_idx, p_discrepancy_thresh);
	local_scores[i].loglikelihood_valid = true;
	//cout << "  running inference on class " << class_idx << " of " << ms->config.classLabels.size() << " detection " << i << "/" << local_scores.size() << " ... ds: " << local_scores[i].discrepancy_score << " ls: " << local_scores[i].loglikelihood_score << " l_max_i: " << *l_max_i << endl;
      }
    }
  } else if (ms->config.currentSceneClassificationMode == SC_DISCREPANCY_ONLY) {
  } else {
    assert(0);
  }


  for (int i = 0; i < local_scores.size(); i++) {
    if ( (local_scores[i].loglikelihood_valid) && (local_scores[i].loglikelihood_score > *l_max_score) ) {
      cout << " score " << local_scores[i].loglikelihood_score << " old l_max_score " << *l_max_score << endl;
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
  double l_max_score = -DBL_MAX;
  int l_max_i = -1;

  findBestScoreForObject(class_idx, num_orientations, &l_max_x, &l_max_y, &l_max_orient, &l_max_score, &l_max_i);

  double l_max_theta = -l_max_orient * 2.0 * M_PI / num_orientations;
  double l_max_x_meters, l_max_y_meters;
  cellToMeters(l_max_x, l_max_y, &l_max_x_meters, &l_max_y_meters);
  cout << "  loglikelihood says: " << endl;
  cout << l_max_x << " " << l_max_y << " " << l_max_orient << " " << l_max_x_meters << " " << l_max_y_meters << " " << 
    l_max_theta << endl << "l_max_score: " << l_max_score << " l_max_i: " << l_max_i << endl;

  //if (max_x > -1)
  if (l_max_score > -DBL_MAX)
  {
    if (l_max_score > 0) {
      cout << "best detection made an improvement..." << endl;
      cout << "adding object." << endl;
      this->addPredictedObject(l_max_x_meters, l_max_y_meters, l_max_theta, class_idx);
    } else {
      cout << "best detection made things worse alone..." << endl;
      cout << "should NOT adding object but for now we are..." << endl;
      this->addPredictedObject(l_max_x_meters, l_max_y_meters, l_max_theta, class_idx);
    }
  } else {
    cout << "Did not find a valid cell... not adding object." << endl;
  }
}



#include <boost/multiprecision/cpp_dec_float.hpp>

typedef boost::multiprecision::number<boost::multiprecision::cpp_dec_float<2000> > doubleWithLotsOfDigits;

double Scene::computeProbabilityOfMap() {
  int numCells = 0;
  doubleWithLotsOfDigits logLikelihood = 0.0;
  doubleWithLotsOfDigits normalizerLogLikelihood = 0.0;

  double p_crop_pad = 0.05;
  double threshold = 0.1;

  shared_ptr<Scene> new_component_scene = this->copy();
  new_component_scene->predicted_objects.resize(0);
  new_component_scene->composePredictedMap(0.01);
  new_component_scene->measureDiscrepancy();
  shared_ptr<Scene> new_class_crop = new_component_scene->copyPaddedDiscrepancySupport(threshold, p_crop_pad);

  initializeAndFocusOnTempClass(ms);
  ms->config.class_scene_models[ms->config.focusedClass] = new_class_crop;

  new_component_scene->tryToAddObjectToScene(ms->config.focusedClass);
  new_component_scene->composePredictedMap(0.01);

  for (int y = 0; y < new_component_scene->background_map->height; y++) {
    for (int x = 0; x < new_component_scene->background_map->width; x++) {
      //ms->config.scene->background_map->refAtCell(x,y)->blue.sigmasquared = pow(stddev, 2);
    }
  }  
  
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      if ((predicted_map->refAtCell(x,y)->red.samples > 0) && (observed_map->refAtCell(x,y)->red.samples > 0)) {
	GaussianMapCell * observed_cell = observed_map->refAtCell(x, y);
	GaussianMapCell * predicted_cell = predicted_map->refAtCell(x, y);
	GaussianMapCell * background_cell = background_map->refAtCell(x, y);
	GaussianMapCell * new_scene_cell = new_component_scene->predicted_map->refAtCell(x, y);

	double rmu_diff, gmu_diff, bmu_diff;
	doubleWithLotsOfDigits discrepancy = background_map->refAtCell(x, y)->pointDiscrepancy(observed_map->refAtCell(x, y), &rmu_diff, &gmu_diff, &bmu_diff);
	

	if (discrepancy > 0.01) {
	  logLikelihood += computeLogLikelihood(predicted_cell->red, observed_cell->red);
	  logLikelihood += computeLogLikelihood(predicted_cell->green, observed_cell->green);
	  logLikelihood += computeLogLikelihood(predicted_cell->blue, observed_cell->blue);
	  
	  //normalizerLogLikelihood += computeLogLikelihood(background_cell->red, observed_cell->red);
	  //normalizerLogLikelihood += computeLogLikelihood(background_cell->green, observed_cell->green);
	  //normalizerLogLikelihood += computeLogLikelihood(background_cell->blue, observed_cell->blue);

	  //normalizerLogLikelihood += log(1.0/256.0) * 3;

	  normalizerLogLikelihood += computeLogLikelihood(new_scene_cell->red, observed_cell->red);
	  normalizerLogLikelihood += computeLogLikelihood(new_scene_cell->green, observed_cell->green);
	  normalizerLogLikelihood += computeLogLikelihood(new_scene_cell->blue, observed_cell->blue);

	} else {
	  
	}
	numCells += 1;
      }
    }
  }
  cout << std::setprecision(50);
  cout << "*************************************" << endl;
  cout << "numCells: " << numCells << endl;
  cout << "logLikelihood: " << logLikelihood << endl;
  doubleWithLotsOfDigits prior = 0.5;
  doubleWithLotsOfDigits logPrior = boost::multiprecision::log(prior);
  doubleWithLotsOfDigits logNotPrior = boost::multiprecision::log(1-prior);
  
  doubleWithLotsOfDigits logNumerator = logLikelihood + logPrior;

  
  //doubleWithLotsOfDigits normalizer = 1.0/256.0;
  //doubleWithLotsOfDigits logNormalizer = numCells * 3 * boost::multiprecision::log(normalizer);
  doubleWithLotsOfDigits logNormalizer = normalizerLogLikelihood;
  
  doubleWithLotsOfDigits t1 = logNumerator - logNormalizer;
  doubleWithLotsOfDigits t1Prob = boost::multiprecision::exp(t1);
  //checkProb("t1Prob", t1Prob);
  doubleWithLotsOfDigits t2Prob = 1.0-prior;
  //checkProb("t2Prob", t2Prob);

  doubleWithLotsOfDigits logDenominatorLog = logNormalizer + boost::multiprecision::log(t1Prob + t2Prob);

  doubleWithLotsOfDigits probNumerator = boost::multiprecision::exp(logNumerator);
  doubleWithLotsOfDigits probNormalizer = boost::multiprecision::exp(logNormalizer + logNotPrior);
  doubleWithLotsOfDigits probSum = probNumerator + probNormalizer;
  doubleWithLotsOfDigits logDenominator = boost::multiprecision::log(probSum);

  doubleWithLotsOfDigits logResult = logNumerator - logDenominator;
  
  doubleWithLotsOfDigits resultProb = boost::multiprecision::exp(logResult);

  doubleWithLotsOfDigits ratio = logLikelihood - logNormalizer;
  //checkProb("result", resultProb);

  cout << "            prior: " << prior << endl;
  cout << "    logLikelihood: " << logLikelihood << endl;
  cout << "     logNumerator: " << logNumerator << endl;
  cout << "    logNormalizer: " << logNormalizer << endl;
  cout << "    probNumerator: " << probNumerator << endl;
  cout << "   probNormalizer: " << probNormalizer << endl;
  cout << "          probSum: " << probSum << endl;
  cout << "               t1: " << t1 << endl;
  cout << "           t1Prob: " << t1Prob << endl;
  cout << "   logDenominator: " << logDenominator << endl;
  cout << "logDenominatorLog: " << logDenominatorLog << endl;
  cout << "        logResult: " << logResult << endl;
  cout << "       resultProb: " << resultProb << endl;
  cout << "            ratio: " << ratio << endl;
  double resultDouble = (double) resultProb;
  cout << " resultProbDouble: " << resultDouble << endl;
  ms->config.scene = new_component_scene;
  //ms->config.scene = new_class_crop;
  return resultDouble;

  
  
}

double Scene::computeProbabilityOfMap1() {
  int numCells = 0;
  doubleWithLotsOfDigits logLikelihood = 0.0;
  doubleWithLotsOfDigits normalizerLogLikelihood = 0.0;

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      if ((predicted_map->refAtCell(x,y)->red.samples > 0) && (observed_map->refAtCell(x,y)->red.samples > 0)) {
	GaussianMapCell * observed_cell = observed_map->refAtCell(x, y);
	GaussianMapCell * predicted_cell = predicted_map->refAtCell(x, y);
	GaussianMapCell * background_cell = background_map->refAtCell(x, y);
	logLikelihood += computeLogLikelihood(predicted_cell->red, observed_cell->red);
	logLikelihood += computeLogLikelihood(predicted_cell->green, observed_cell->green);
	logLikelihood += computeLogLikelihood(predicted_cell->blue, observed_cell->blue);


	normalizerLogLikelihood += computeLogLikelihood(background_cell->red, observed_cell->red);
	normalizerLogLikelihood += computeLogLikelihood(background_cell->green, observed_cell->green);
	normalizerLogLikelihood += computeLogLikelihood(background_cell->blue, observed_cell->blue);

	numCells += 1;
      }
    }
  }
  cout << std::setprecision(50);
  cout << "*************************************" << endl;
  cout << "numCells: " << numCells << endl;
  cout << "logLikelihood: " << logLikelihood << endl;
  doubleWithLotsOfDigits prior = 0.5;
  doubleWithLotsOfDigits logPrior = boost::multiprecision::log(prior);
  doubleWithLotsOfDigits logNotPrior = boost::multiprecision::log(1-prior);
  
  doubleWithLotsOfDigits logNumerator = logLikelihood + logPrior;

  
  doubleWithLotsOfDigits normalizer = 1.0/256.0;
  //doubleWithLotsOfDigits logNormalizer = numCells * 3 * boost::multiprecision::log(normalizer);
  doubleWithLotsOfDigits logNormalizer = normalizerLogLikelihood;
  
  doubleWithLotsOfDigits t1 = logNumerator - logNormalizer;
  doubleWithLotsOfDigits t1Prob = boost::multiprecision::exp(t1);
  //checkProb("t1Prob", t1Prob);
  doubleWithLotsOfDigits t2Prob = 1.0-prior;
  //checkProb("t2Prob", t2Prob);

  doubleWithLotsOfDigits logDenominatorLog = logNormalizer + boost::multiprecision::log(t1Prob + t2Prob);

  doubleWithLotsOfDigits probNumerator = boost::multiprecision::exp(logNumerator);
  doubleWithLotsOfDigits probNormalizer = boost::multiprecision::exp(logNormalizer + logNotPrior);
  doubleWithLotsOfDigits probSum = probNumerator + probNormalizer;
  doubleWithLotsOfDigits logDenominator = boost::multiprecision::log(probSum);

  doubleWithLotsOfDigits logResult = logNumerator - logDenominator;
  
  doubleWithLotsOfDigits resultProb = boost::multiprecision::exp(logResult);
  //checkProb("result", resultProb);

  cout << "prior: " << prior << endl;
  cout << "    logLikelihood: " << logLikelihood << endl;
  cout << "     logNumerator: " << logNumerator << endl;
  cout << "    logNormalizer: " << logNormalizer << endl;
  cout << "    probNumerator: " << probNumerator << endl;
  cout << "   probNormalizer: " << probNormalizer << endl;
  cout << "          probSum: " << probSum << endl;
  cout << "               t1: " << t1 << endl;
  cout << "           t1Prob: " << t1Prob << endl;
  cout << "   logDenominator: " << logDenominator << endl;
  cout << "logDenominatorLog: " << logDenominatorLog << endl;
  cout << "        logResult: " << logResult << endl;
  cout << "       resultProb: " << resultProb << endl;
  double resultDouble = (double) resultProb;
  cout << " resultProbDouble: " << resultDouble << endl;
  return resultDouble;
}


double Scene::computeProbabilityOfMapDouble() {
  int numCells = 0;
  double logLikelihood = 0.0;
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      if ((predicted_map->refAtCell(x,y)->red.samples > 0) && (observed_map->refAtCell(x,y)->red.samples > 0)) {
	GaussianMapCell * observed_cell = observed_map->refAtCell(x, y);
	GaussianMapCell * predicted_cell = predicted_map->refAtCell(x, y);
	logLikelihood += computeLogLikelihood(predicted_cell->red, observed_cell->red);
	logLikelihood += computeLogLikelihood(predicted_cell->green, observed_cell->green);
	logLikelihood += computeLogLikelihood(predicted_cell->blue, observed_cell->blue);
	numCells += 1;
      }
    }
  }
  cout << "total likelihood: " << logLikelihood << endl;
  double prior = 0.5;
  double logNumerator = logLikelihood + log(prior);

  
  double logNormalizer = numCells * 3 * log(1.0/256);
  
  double t1 = logLikelihood - logNormalizer;
  double t1Prob = exp(t1);
  checkProb("t1Prob", t1Prob);
  double t2Prob = 1-prior;
  checkProb("t2Prob", t2Prob);

  double logDenominator = logNormalizer + log(t1Prob + t2Prob);

  double result = logNumerator - logDenominator;
  
  double resultProb = exp(result);
  checkProb("result", resultProb);
  assert(0);
  return resultProb;
}

void Scene::findBestObjectAndScore(int * class_idx, int num_orientations, int * l_max_x, int * l_max_y, int * l_max_orient, double * l_max_score, int * l_max_i) {
  guardSceneModels(ms);

  *l_max_x = -1;
  *l_max_y = -1;
  *l_max_orient = -1;
  //*l_max_score = -DBL_MAX;
  *l_max_i = -1;

  *class_idx = -1;
  

  for (int j = 0; j < ms->config.classLabels.size(); j++) {
    int j_max_x = -1;
    int j_max_y = -1;
    int j_max_orient = -1;
    double j_max_score = -DBL_MAX;
    int j_max_i = -1;

    findBestScoreForObject(j, num_orientations, &j_max_x, &j_max_y, &j_max_orient, &j_max_score, &j_max_i);

    if (j_max_score > *l_max_score) {
      *l_max_score = j_max_score;
      *l_max_x = j_max_x;
      *l_max_y = j_max_y;
      *l_max_orient = j_max_orient;
      *l_max_i = j_max_i;

      *class_idx = j;
    } else {
    }

    cout << "  findBestObjectAndScore, class " << j << " " << ms->config.classLabels[j] << " : " << endl;
    cout << *l_max_x << " " << *l_max_y << " " << *l_max_orient << " " << "l_max_score: " << *l_max_score << " l_max_i: " << *l_max_i << " -DBL_MAX: " << -DBL_MAX << endl;
  }
}

void Scene::tryToAddBestObjectToScene() {
  int tfc = ms->config.focusedClass;
  assert(tfc != -1);

  guardSceneModels(ms);

  int num_orientations = 37;

  int l_max_class = -1;
  int l_max_x = -1;
  int l_max_y = -1;
  int l_max_orient = -1;
  double l_max_score = -DBL_MAX;
  int l_max_i = -1;

  findBestObjectAndScore(&l_max_class, num_orientations, &l_max_x, &l_max_y, &l_max_orient, &l_max_score, &l_max_i);

  double l_max_theta = -l_max_orient * 2.0 * M_PI / num_orientations;
  double l_max_x_meters, l_max_y_meters;
  cellToMeters(l_max_x, l_max_y, &l_max_x_meters, &l_max_y_meters);

  
  cout << "findBestObjectAndScore: best object was class " << l_max_class;
  if (l_max_class != -1) {
    cout << " " << ms->config.classLabels[l_max_class];
  }
  cout << endl;

  //if (l_max_x > -1)
  if (l_max_score > -DBL_MAX)
  {
    if (l_max_score > 0) {
      cout << "best detection made an improvement..." << endl;
      cout << "adding object." << endl;
      this->addPredictedObject(l_max_x_meters, l_max_y_meters, l_max_theta, l_max_class);
    } else {
      cout << "best detection made things worse alone..." << endl;
      cout << "should NOT adding object but for now we are..." << endl;
      this->addPredictedObject(l_max_x_meters, l_max_y_meters, l_max_theta, l_max_class);
    }
  } else {
    cout << "Did not find a valid cell... not adding object." << endl;
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
  fsvO << "predicted_class_name" << predicted_class_name;
  fsvO << "annotated_class_name" << annotated_class_name;
  fsvO << "width" << width;
  fsvO << "height" << height;
  fsvO << "x_center_cell" << x_center_cell;
  fsvO << "y_center_cell" << y_center_cell;
  fsvO << "cell_width" << cell_width;
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

  fsvO << "predicted_segmentation";
  writeMatToYaml(predicted_segmentation, fsvO);

  
  fsvO << "discrepancy_magnitude";
  writeMatToYaml(discrepancy_magnitude, fsvO);

  fsvO << "discrepancy_density";
  writeMatToYaml(discrepancy_density, fsvO);

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

  (it)["predicted_class_name"] >> predicted_class_name;
  (it)["annotated_class_name"] >> annotated_class_name;
  (it)["width"] >> width;
  (it)["height"] >> height;
  (it)["x_center_cell"] >> x_center_cell;
  (it)["y_center_cell"] >> y_center_cell;
  (it)["cell_width"] >> cell_width;
  
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

  cout << "Loading predicted segmentation...";
  node = it["predicted_segmentation"];
  predicted_segmentation = readMatFromYaml(node);
  cout << "done" << endl;

  cout << "Loading discrepancy magnitude...";
  node = it["discrepancy_magnitude"];
  discrepancy_magnitude = readMatFromYaml(node);
  cout << "done" << endl;

  cout << "Loading discrepancy density...";
  node = it["discrepancy_density"];
  discrepancy_density = readMatFromYaml(node);
  cout << "done" << endl;

  bool error = false;
  if (discrepancy_density.rows != width || discrepancy_density.cols != height) {
    ROS_ERROR_STREAM("Discrepancy density has inconsistent dimensions.  Scene: " << width << "x" << height << ".  Mat: " << discrepancy_density.rows << "x" << discrepancy_density.cols);
    discrepancy_density = Mat(height, width, CV_64F);
    error = true;
  }

  if (discrepancy_magnitude.rows != width || discrepancy_magnitude.cols != height) {
    ROS_ERROR_STREAM("Discrepancy magnitude has inconsistent dimensions.  Scene: " << width << "x" << height << ".  Mat: " << discrepancy_magnitude.rows << "x" << discrepancy_magnitude.cols);
    discrepancy_magnitude = Mat(height, width, CV_64F);
    error = true;
  }

  if (predicted_segmentation.rows != width || predicted_segmentation.cols != height) {
    ROS_ERROR_STREAM("Discrepancy magnitude has inconsistent dimensions.  Scene: " << width << "x" << height << ".  Mat: " << predicted_segmentation.rows << "x" << predicted_segmentation.cols);
    predicted_segmentation = Mat(height, width, CV_64F);
  }
  
  if (error) {
    measureDiscrepancy();
  }

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

WORD(SceneComputeProbabilityOfMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double score = ms->config.scene->computeProbabilityOfMap();
  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(score);
  ms->pushWord(newWord);

}
END_WORD
REGISTER_WORD(SceneComputeProbabilityOfMap)


WORD(SceneSaveSceneAbsolute)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string dir;
  GET_STRING_ARG(ms, dir);
  ms->config.scene->saveToFile(dir);
  // XX check return code
}
END_WORD
REGISTER_WORD(SceneSaveSceneAbsolute)



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

WORD(SceneAddPredictedObject)
virtual void execute(std::shared_ptr<MachineState> ms) {
  guardSceneModels(ms);

  double x_in=0, y_in=0, theta_in=0; 
  int class_in=0;
  GET_INT_ARG(ms, class_in);
  GET_NUMERIC_ARG(ms, theta_in);
  GET_NUMERIC_ARG(ms, y_in);
  GET_NUMERIC_ARG(ms, x_in);

  ms->config.scene->addPredictedObject(x_in, y_in, theta_in, class_in);
}
END_WORD
REGISTER_WORD(SceneAddPredictedObject)

WORD(ScenePredictFocusedObject)
virtual void execute(std::shared_ptr<MachineState> ms) {
  REQUIRE_FOCUSED_CLASS(ms,tfc);
  guardSceneModels(ms);
  ms->config.scene->tryToAddObjectToScene(tfc);
}
END_WORD
REGISTER_WORD(ScenePredictFocusedObject)

WORD(ScenePredictBestObject)
virtual void execute(std::shared_ptr<MachineState> ms) {
  guardSceneModels(ms);
  ms->config.scene->tryToAddBestObjectToScene();
}
END_WORD
REGISTER_WORD(ScenePredictBestObject)

WORD(ScenePushTotalDiscrepancy)
virtual void execute(std::shared_ptr<MachineState> ms) {
  guardSceneModels(ms);

  Mat tsd = ms->config.scene->discrepancy_density;
  double tsd_l1 = tsd.dot(Mat::ones(tsd.rows, tsd.cols, tsd.type()));
  ms->pushWord(make_shared<DoubleWord>(tsd_l1));

/*
  //---> seems solidly discriminative, counts the difference it makes. should stay in the log and normalize by the 
  //number norm of the chosen object
  Mat tsd_log;
  log(1.0 - tsd, tsd_log);
  double tsd_log_l1 = tsd_log.dot(Mat::ones(tsd_log.rows, tsd_log.cols, tsd_log.type()));
  double tsd_prob = exp(tsd_log_l1);
  ms->pushWord(make_shared<DoubleWord>(tsd_prob));
*/

/*
  double totalProb = 1.0;
  int W = tsd.cols;
  int H = tsd.rows;
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      double val = tsd.at<double>(y,x);
      if (val > 1.0) {
	cout << "scenePushTotalDiscrepancy: XXX invalid density " << val << endl;
      } else if (val < 0.0) {
	cout << "scenePushTotalDiscrepancy: XXX invalid density " << val << endl;
      } else {
	double p_safe_discrepancy_min = 1e-10;
	double safeVal = max(p_safe_discrepancy_min, val);
	totalProb *= (1.0 - safeVal);
      }
    }
  }
  ms->pushWord(make_shared<DoubleWord>(totalProb));
*/

/*
  double totalProb = 1.0;
  int W = tsd.cols;
  int H = tsd.rows;
  double root = 0.0;
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      double val = tsd.at<double>(y,x);
      if (val > 1.0) {
	cout << "scenePushTotalDiscrepancy: XXX invalid density " << val << endl;
      } else if (val < 0.0) {
	cout << "scenePushTotalDiscrepancy: XXX invalid density " << val << endl;
      } else {
	double p_safe_discrepancy_min = 1e-10;
	if (val > p_safe_discrepancy_min) {
	  double safeVal = max(p_safe_discrepancy_min, val);
	  totalProb *= (1.0 - safeVal);
	  root += 1.0;
	} else {
	}
      }
    }
  }
  double safe_root = max(root, 1.0);
  double geometrically_rectified = pow(totalProb, 1.0/safe_root);
  ms->pushWord(make_shared<DoubleWord>(geometrically_rectified));
*/
}
END_WORD
REGISTER_WORD(ScenePushTotalDiscrepancy)

WORD(ScenePushTotalRelevantOneMinusDiscrepancy)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double thresh = 0.0;
  GET_NUMERIC_ARG(ms, thresh);
  guardSceneModels(ms);

  Mat tsd = ms->config.scene->discrepancy_density.clone();
  Mat omtsd = ms->config.scene->discrepancy_density.clone();

  int W = tsd.cols;
  int H = tsd.rows;
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      if (tsd.at<double>(y,x) <= thresh) {
	omtsd.at<double>(y,x) = 0.0;
      } else {
	omtsd.at<double>(y,x) = 1.0 - tsd.at<double>(y,x);
      }
    }
  }

  double omtsd_l1 = omtsd.dot(Mat::ones(omtsd.rows, omtsd.cols, omtsd.type()));
  ms->pushWord(make_shared<DoubleWord>(omtsd_l1));
}
END_WORD
REGISTER_WORD(ScenePushTotalRelevantOneMinusDiscrepancy)

WORD(ScenePushFocusedClassModelArea)
virtual void execute(std::shared_ptr<MachineState> ms) {
  guardSceneModels(ms);
  REQUIRE_FOCUSED_CLASS(ms,tfc);
  ms->pushWord(make_shared<DoubleWord>(ms->config.class_scene_models[tfc]->width * ms->config.class_scene_models[tfc]->height));
}
END_WORD
REGISTER_WORD(ScenePushFocusedClassModelArea)

WORD(ScenePushFocusedClassTotalDiscrepancy)
virtual void execute(std::shared_ptr<MachineState> ms) {
  guardSceneModels(ms);
  REQUIRE_FOCUSED_CLASS(ms,tfc);

  Mat tsd = ms->config.class_scene_models[tfc]->discrepancy_density;
  double tsd_l1 = tsd.dot(Mat::ones(tsd.rows, tsd.cols, tsd.type()));
  ms->pushWord(make_shared<DoubleWord>(tsd_l1));
}
END_WORD
REGISTER_WORD(ScenePushFocusedClassTotalDiscrepancy)

WORD(ScenePushTotalLogDiscrepancy)
virtual void execute(std::shared_ptr<MachineState> ms) {
  guardSceneModels(ms);

  Mat tsd = ms->config.scene->discrepancy_density;
  Mat omtsd = 1.0 - tsd;
/*
  double tsd_l1 = tsd.dot(Mat::ones(tsd.rows, tsd.cols, tsd.type()));
  ms->pushWord(make_shared<DoubleWord>(tsd_l1));
*/

  //---> seems solidly discriminative, counts the difference it makes. should stay in the log and normalize by the 
  //number norm of the chosen object
  Mat omtsd_log;
  //log(1.0 - tsd, omtsd_log);
  //log(1.0 + tsd, omtsd_log);

  double p_epsilon = DBL_MIN;
  log(min(omtsd, p_epsilon), omtsd_log);
  int W = tsd.cols;
  int H = tsd.rows;
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      if (omtsd.at<double>(y,x) <= 0) {
	omtsd_log.at<double>(y,x) = 0.0;
      } else {
      }
    }
  }

  double omtsd_log_l1 = omtsd_log.dot(Mat::ones(omtsd_log.rows, omtsd_log.cols, omtsd_log.type()));
  //double tsd_prob = exp(tsd_log_l1);
  ms->pushWord(make_shared<DoubleWord>(omtsd_log_l1));

/*
  double totalProb = 1.0;
  int W = tsd.cols;
  int H = tsd.rows;
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      double val = tsd.at<double>(y,x);
      if (val > 1.0) {
	cout << "scenePushTotalDiscrepancy: XXX invalid density " << val << endl;
      } else if (val < 0.0) {
	cout << "scenePushTotalDiscrepancy: XXX invalid density " << val << endl;
      } else {
	double p_safe_discrepancy_min = 1e-10;
	double safeVal = max(p_safe_discrepancy_min, val);
	totalProb *= (1.0 - safeVal);
      }
    }
  }
  ms->pushWord(make_shared<DoubleWord>(totalProb));
*/

/*
  double totalProb = 1.0;
  int W = tsd.cols;
  int H = tsd.rows;
  double root = 0.0;
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      double val = tsd.at<double>(y,x);
      if (val > 1.0) {
	cout << "scenePushTotalDiscrepancy: XXX invalid density " << val << endl;
      } else if (val < 0.0) {
	cout << "scenePushTotalDiscrepancy: XXX invalid density " << val << endl;
      } else {
	double p_safe_discrepancy_min = 1e-10;
	if (val > p_safe_discrepancy_min) {
	  double safeVal = max(p_safe_discrepancy_min, val);
	  totalProb *= (1.0 - safeVal);
	  root += 1.0;
	} else {
	}
      }
    }
  }
  double safe_root = max(root, 1.0);
  double geometrically_rectified = pow(totalProb, 1.0/safe_root);
  ms->pushWord(make_shared<DoubleWord>(geometrically_rectified));
*/
}
END_WORD
REGISTER_WORD(ScenePushTotalLogDiscrepancy)

WORD(ScenePushTotalDiscrepancyMagnitude)
virtual void execute(std::shared_ptr<MachineState> ms) {
  guardSceneModels(ms);

  Mat tsd = ms->config.scene->discrepancy_magnitude;
  double tsd_l1 = tsd.dot(Mat::ones(tsd.rows, tsd.cols, tsd.type()));
  ms->pushWord(make_shared<DoubleWord>(tsd_l1));
}
END_WORD
REGISTER_WORD(ScenePushTotalDiscrepancyMagnitude)

WORD(SceneTakeBeforeDensity)
virtual void execute(std::shared_ptr<MachineState> ms) {
  guardSceneModels(ms);
  ms->config.scene->discrepancy_before = ms->config.scene->discrepancy_density;
}
END_WORD
REGISTER_WORD(SceneTakeBeforeDensity)


WORD(SceneTakeAfterDensity)
virtual void execute(std::shared_ptr<MachineState> ms) {
  guardSceneModels(ms);
  ms->config.scene->discrepancy_after = ms->config.scene->discrepancy_density;
}
END_WORD
REGISTER_WORD(SceneTakeAfterDensity)

bool doubleDescending(const double &i, const double &j) {
  return (i > j);
}
bool doubleAscending(const double &i, const double &j) {
  return (i < j);
}

WORD(SceneHighPrecisionBeforeAfterDiffOfLogs)
virtual void execute(std::shared_ptr<MachineState> ms) {
  guardSceneModels(ms);

  Mat tsd_b = ms->config.scene->discrepancy_before;
  Mat tsd_a = ms->config.scene->discrepancy_after;

  vector<double> before_scores;
  vector<double> after_scores;

  int W = tsd_a.cols;
  int H = tsd_a.rows;
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      before_scores.push_back( tsd_b.at<double>(y,x) );
      after_scores.push_back( tsd_a.at<double>(y,x) );
    }
  }

  std::sort(before_scores.begin(), before_scores.end(), doubleAscending);
  std::sort(after_scores.begin(), after_scores.end(), doubleAscending);


  double hp_total = 0.0;

  // deal with all the pairs
  int total_left = std::min( before_scores.size(), after_scores.size() );
  while (total_left > 0) {

    double t_before = before_scores.back(); before_scores.pop_back();
    double t_after = after_scores.back(); after_scores.pop_back();

    if (t_before > t_after) {

      if (before_scores.size() == 0) {
	// use these two
	hp_total += t_after - t_before;
      } else {

	double t_before_second = before_scores.back(); before_scores.pop_back();
	if (t_before_second > t_after) {
	  // accumulate before and return after
	  double t_new = t_before + t_before_second;
	  before_scores.push_back(t_new);
	  after_scores.push_back(t_after);
	} else if (t_before_second == t_after ) {
	  // annihilate the equal values and push the before back on the stack 
	  before_scores.push_back(t_before);
	} else {
	  // t_before_second < t_after
	  // subtract first before and first after, return second before
	  hp_total += t_after - t_before;
	  before_scores.push_back(t_before_second);
	}

      }
    } else if (t_before == t_after ) {
      // do nothing, they cancel eachother out
    } else {
      // t_before < t_after

      if (after_scores.size() == 0) {
	// use these two
	hp_total += t_after - t_before;
      } else {

	double t_after_second = after_scores.back(); after_scores.pop_back();
	if (t_after_second > t_before ) {
	  // accumulate after and return before
	  double t_new = t_after + t_after_second;
	  after_scores.push_back(t_new);
	  before_scores.push_back(t_before);
	} else if (t_after_second == t_before ) {
	  // annihilate equal values and push the after back on the stack
	  after_scores.push_back(t_after);
	} else {
	  // t_after_second < t_before
	  // subtract first before and first after, return second after 
	  hp_total += t_after - t_before;
	  after_scores.push_back(t_after_second);
	}
      }
    }
    total_left = std::min( before_scores.size(), after_scores.size() );
  }

  // cleanup befores
  total_left = before_scores.size();
  while (total_left > 0) {
    double t_before = before_scores.back(); before_scores.pop_back();
    hp_total += - t_before;
    total_left = before_scores.size();
  }

  // cleanup afters
  total_left = after_scores.size();
  while (total_left > 0) {
    double t_after = after_scores.back(); after_scores.pop_back();
    hp_total += t_after;
    total_left = after_scores.size();
  }

  // push result
  cout << "sceneHighPrecisionBeforeAfterDiffOfLogs: pushing " << hp_total << endl;
  ms->pushWord(make_shared<DoubleWord>(hp_total));
}
END_WORD
REGISTER_WORD(SceneHighPrecisionBeforeAfterDiffOfLogs)

WORD(SceneIsNewConfiguration)
virtual void execute(std::shared_ptr<MachineState> ms) {
  guardSceneModels(ms);

  /*
  int num_orientations = 37;

  int l_max_class = -1;
  int l_max_x = -1;
  int l_max_y = -1;
  int l_max_orient = -1;
  double l_max_score = -DBL_MAX;
  int l_max_i = -1;

  ms->config.scene->findBestObjectAndScore(&l_max_class, num_orientations, &l_max_x, &l_max_y, &l_max_orient, &l_max_score, &l_max_i);

  double best_density_l1 = 0.0;  
  double best_ll_l1_ratio = 0.0;  

  if ( (l_max_class > -1) && (l_max_class < ms->config.classLabels.size()) ) {
    Mat best_object_density = ms->config.class_scene_models[l_max_class]->discrepancy_density;
    best_density_l1 =  best_object_density.dot(Mat::ones(best_object_density.rows, best_object_density.cols, best_object_density.type()));
    best_ll_l1_ratio = l_max_score / best_density_l1;
    cout << "sceneIsNewConfiguration best score, best l1, ratio:" << l_max_score << " " << best_density_l1 << " " << best_ll_l1_ratio << endl;
  } else {
    cout << "sceneIsNewConfiguration: oops, bad class won..." << endl;
  }
  ms->pushWord(make_shared<DoubleWord>(best_ll_l1_ratio));
  */

  int to_map = 0;
  REQUIRE_VALID_SCENE_OBJECT(ms, to_map);

  shared_ptr<SceneObject> tso = ms->config.scene->predicted_objects[to_map];
  shared_ptr<Scene> tso_s = ms->config.class_scene_models[ tso->labeled_class_index ];
  Mat tod = tso_s->discrepancy_density;
  double tod_l1 =  tod.dot(Mat::ones(tod.rows, tod.cols, tod.type()));

  Mat tsd = ms->config.scene->discrepancy_density;
  double tsd_l1 = tsd.dot(Mat::ones(tsd.rows, tsd.cols, tsd.type()));
  
  double d_ratio = tod_l1 / tsd_l1;
  cout << "sceneIsNewConfiguration tod_l1, tsd_l1, ratio:" << tod_l1 << " " << tsd_l1 << " " << d_ratio << endl;

  ms->pushWord(make_shared<DoubleWord>(d_ratio));

}
END_WORD
REGISTER_WORD(SceneIsNewConfiguration)

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

WORD(SceneInitSmall)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double p_cell_width = 0.0025; //0.01;
  int p_width = 50; // 601;
  int p_height = 50; // 601;
  ms->config.scene = make_shared<Scene>(ms, p_width, p_height, p_cell_width);
  ms->pushWord("sceneRenderScene");
}
END_WORD
REGISTER_WORD(SceneInitSmall)


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
  Mat ringImage;
  eePose thisPose;
  ros::Time time;
  int result = getMostRecentRingImageAndPose(ms, &ringImage, &thisPose, &time);

  if (result != 1) {
    ROS_ERROR("Not doing update because of ring buffer errors.");
    return;
  }

  Mat wristViewYCbCr = ringImage.clone(); //ms->config.wristCamImage.clone();  
  //Mat wristViewYCbCr = ms->config.wristCamImage.clone();  
  //thisPose = ms->config.trueEEPoseEEPose;
  cvtColor(ms->config.wristCamImage, wristViewYCbCr, CV_BGR2YCrCb);

  Size sz = ms->config.wristCamImage.size();
  int imW = sz.width;
  int imH = sz.height;

  int topx = ms->config.grayTop.x+ms->config.mapGrayBoxPixelSkirtCols; //+ 20; // ms->config.grayTop.x;  
  int botx = ms->config.grayBot.x-ms->config.mapGrayBoxPixelSkirtCols; //- 20; // ms->config.grayBot.x;  
  int topy = ms->config.grayTop.y+ms->config.mapGrayBoxPixelSkirtRows; //+ 50; // ms->config.grayTop.y;
  int boty = ms->config.grayBot.y-ms->config.mapGrayBoxPixelSkirtRows; //- 50; // ms->config.grayBot.y;  
    
  //for (int px = ; px < ; px++) 
    //for (int py = ; py < ; py++) 
  pixelToGlobalCache data;
  double z = ms->config.trueEEPose.position.z + ms->config.currentTableZ;
  computePixelToGlobalCache(ms, z, thisPose, &data);

  for (int px = topx; px < botx; px++) {
    for (int py = topy; py < boty; py++) {
  //for (int px = 0; px < imW; px++) 
    //for (int py = 0; py < imH; py++) 
      if (isInGripperMask(ms, px, py)) {
	continue;
      }
      double x, y;
      pixelToGlobalFromCache(ms, px, py, z, &x, &y, thisPose, &data);

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

// XXX TODO NOT DONE
WORD(SceneUpdateObservedFromStreamBuffer)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int thisIdx = ms->config.sibCurIdx;
  //cout << "sceneUpdateObservedFromStreamBuffer: " << thisIdx << endl;

  Mat bufferImage;
  eePose thisPose, tBaseP;

  int success = 1;
  if ( (thisIdx > -1) && (thisIdx < ms->config.streamImageBuffer.size()) ) {
    streamImage &tsi = ms->config.streamImageBuffer[thisIdx];
    if (tsi.image.data == NULL) {
      cout << "  encountered NULL data in sib, returning." << endl;
      return;
    } else {
      bufferImage = tsi.image.clone();
    }
    success = getStreamPoseAtTime(ms, tsi.time, &thisPose, &tBaseP);
  } else {
    ROS_ERROR_STREAM("No images in the buffer, returning." << endl);
    return;
  }

  if (success != 1) {
    ROS_ERROR("  Not doing update because of stream buffer errors.");
    return;
  }

  if (fabs(thisPose.qz) > 0.005) {
    ROS_ERROR("  Not doing update because arm not vertical.");
    return;
  }

  Mat wristViewYCbCr = bufferImage.clone();

  cvtColor(bufferImage, wristViewYCbCr, CV_BGR2YCrCb);
  
  Size sz = bufferImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int topx = ms->config.grayTop.x+ms->config.mapGrayBoxPixelSkirtCols; //+ 20; // ms->config.grayTop.x;  
  int botx = ms->config.grayBot.x-ms->config.mapGrayBoxPixelSkirtCols; //- 20; // ms->config.grayBot.x;  
  int topy = ms->config.grayTop.y+ms->config.mapGrayBoxPixelSkirtRows; //+ 50; // ms->config.grayTop.y;
  int boty = ms->config.grayBot.y-ms->config.mapGrayBoxPixelSkirtRows; //- 50; // ms->config.grayBot.y;  
  
  //for (int px = ; px < ; px++) 
  //for (int py = ; py < ; py++) 
  pixelToGlobalCache data;
  double z = ms->config.trueEEPose.position.z + ms->config.currentTableZ;
  computePixelToGlobalCache(ms, z, thisPose, &data);
  
  for (int px = topx; px < botx; px++) {
    for (int py = topy; py < boty; py++) {
      //for (int px = 0; px < imW; px++) 
      //for (int py = 0; py < imH; py++) 
      if (isInGripperMask(ms, px, py)) {
	continue;
      }
      double x, y;
      pixelToGlobalFromCache(ms, px, py, z, &x, &y, thisPose, &data);
      
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
REGISTER_WORD(SceneUpdateObservedFromStreamBuffer)


WORD(SceneRenderObservedMap)
virtual void execute(std::shared_ptr<MachineState> ms) {
  {
    Mat image;
    ms->config.scene->observed_map->rgbMuToMat(image);
    Mat rgb = image.clone();  
    cvtColor(image, rgb, CV_YCrCb2BGR);
    ms->config.observedWindow->updateImage(rgb);
  }

  {
    Mat image;
    ms->config.scene->observed_map->rgbSigmaToMat(image);
    Mat rgb = image.clone();  
    //cvtColor(image, rgb, CV_YCrCb2BGR);
    ms->config.observedStdDevWindow->updateImage(rgb);
  }
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
  {
    Mat image;
    ms->config.scene->predicted_map->rgbMuToMat(image);
    Mat rgb = image.clone();  
    cvtColor(image, rgb, CV_YCrCb2BGR);
    ms->config.predictedWindow->updateImage(rgb);
  }

  {
    Mat image;
    ms->config.scene->predicted_map->rgbSigmaToMat(image);
    Mat rgb = image.clone();  
    //cvtColor(image, rgb, CV_YCrCb2BGR);
    ms->config.predictedStdDevWindow->updateImage(rgb);
  }
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

  //cout << "scene: " << ms->config.scene.get() << endl;
  //cout << "discrepancy_density: " << ms->config.scene->discrepancy_density << endl;
  ms->config.discrepancyDensityWindow->updateImage(ms->config.scene->discrepancy_density);
  // XXX considered changing this because it looked wrong
  //ms->config.discrepancyDensityWindow->updateImage(densityImage);
}
END_WORD
REGISTER_WORD(SceneRenderDiscrepancy)

WORD(SceneRenderZ)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Mat image;
  ms->config.scene->discrepancy->rgbDiscrepancyMuToMat(ms, image);
  ms->config.discrepancyWindow->updateImage(image);

  Mat toShow;
  ms->config.scene->observed_map->zMuToMat(toShow);
  for (int x = 0; x < ms->config.scene->width; x++) {
    for (int y = 0; y < ms->config.scene->height; y++) {
      if ((ms->config.scene->predicted_map->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold) && (ms->config.scene->observed_map->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold)) {
      } else {
	//toShow.at<double>(y, x) = 0;
      }
    }
  }
  double maxVal, minVal;
  minMaxLoc(toShow, &minVal, &maxVal);
  minVal = max(minVal, ms->config.currentTableZ);
  double denom = max(1e-6, maxVal-minVal);
  cout << "sceneRenderZ min max denom: " << minVal << " " << maxVal << " " << denom << endl;
  for (int x = 0; x < ms->config.scene->width; x++) {
    for (int y = 0; y < ms->config.scene->height; y++) {
      if ((ms->config.scene->predicted_map->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold) && (ms->config.scene->observed_map->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold)) {
	//toShow.at<double>(y, x) = (toShow.at<double>(y, x) - minVal) / denom;
	toShow.at<double>(y, x) = (maxVal - toShow.at<double>(y, x)) / denom;
      } else {
	//toShow.at<double>(y, x) = 0;
      }
    }
  }
  
  ms->config.zWindow->updateImage(toShow);
}
END_WORD
REGISTER_WORD(SceneRenderZ)


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
  pixelToGlobalCache data;
  double zToUse = ms->config.currentEEPose.pz+ms->config.currentTableZ;
  computePixelToGlobalCache(ms, zToUse, ms->config.currentEEPose, &data);

  for (int y = 0; y < imH; y++) {
    for (int x = 0; x < imW; x++) {
      double meter_x = 0;
      double meter_y = 0;
      pixelToGlobalFromCache(ms, x, y, zToUse, &meter_x, &meter_y, ms->config.currentEEPose, &data);
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

WORD(ScenePushSceneObjectLabel)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int to_map = 0;
  GET_INT_ARG(ms, to_map);
  REQUIRE_VALID_SCENE_OBJECT(ms, to_map);
  shared_ptr<SceneObject> tso = ms->config.scene->predicted_objects[to_map];
  REQUIRE_VALID_CLASS(ms, tso->labeled_class_index);
  ms->pushWord( make_shared<StringWord>(ms->config.classLabels[ tso->labeled_class_index ]) );
}
END_WORD
REGISTER_WORD(ScenePushSceneObjectLabel)

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
  
  // XXX didn't seem to be working quite right
  //if ( !positionIsSearched(ms, box.centroid.px, box.centroid.py) || 
       //!isBoxMemoryIkPossible(ms, box) ) 
  if (0)
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






WORD(SceneSetPredictedClassNameToFocusedClass)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.scene->predicted_class_name = ms->config.focusedClassLabel;
  cout << "sceneSetPredictedClassNameToFocusedClass: setting to " << ms->config.scene->predicted_class_name << endl;
}
END_WORD
REGISTER_WORD(SceneSetPredictedClassNameToFocusedClass)

WORD(SceneSetAnnotatedClassNameToFocusedClass)
virtual void execute(std::shared_ptr<MachineState> ms) {
  ms->config.scene->annotated_class_name = ms->config.focusedClassLabel;
  cout << "sceneSetAnnotatedClassNameToFocusedClass: setting to " << ms->config.scene->annotated_class_name << endl;
}
END_WORD
REGISTER_WORD(SceneSetAnnotatedClassNameToFocusedClass)

CONFIG_GETTER_STRING(SceneGetPredictedClassName, ms->config.scene->predicted_class_name)
CONFIG_SETTER_STRING(SceneSetPredictedClassName, ms->config.scene->predicted_class_name)
CONFIG_GETTER_STRING(SceneGetAnnotatedClassName, ms->config.scene->annotated_class_name)
CONFIG_SETTER_STRING(SceneSetAnnotatedClassName, ms->config.scene->annotated_class_name)

// count the number of equal digits
int numberOfEqualCharacters(string first, string second) {
  int i = 0;
  while ( (i<first.length()) && (i<second.length()) ) {
    if (first[i] < second[i]) {
      break;
    } else if (first[i] > second[i]) {
      break;
    } else {
      i++;
    }
  }
  return i;
}

vector<double> poseVarianceOfEvaluationScenes(shared_ptr<MachineState> ms, vector<string> scene_files) {

  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_theta = 0.0;
  double sum_x_squared = 0.0;
  double sum_y_squared = 0.0;
  double sum_theta_squared = 0.0;
  double counts = 0.0;

  vector<shared_ptr<Scene> > scenes;

  try {

  for (int i = 0; i < scene_files.size(); i++) {
    cout << " i " << i << " size: " << scene_files.size() << endl;
    shared_ptr<Scene> this_scene = make_shared<Scene>(ms, 3, 3, 0.02);
    this_scene->loadFromFile(scene_files[i]);
    scenes.push_back(this_scene);
  }
  } catch( ... ) {
        ROS_ERROR("In the weird sketchy exception block in ein main.");    
    cout << "In the weird sketchy exception block in ein main." << endl;    
    
    std::exception_ptr p = std::current_exception();
    std::clog <<(p ? p.__cxa_exception_type()->name() : "null") << std::endl;
    throw;
  }
  cout << "Finished loop." << endl;


  for (int i = 0; i < scenes.size(); i++) {
    cout << "Scene " << i << endl;
    shared_ptr<Scene> this_scene = scenes[i];
    if (this_scene->predicted_objects.size() != 1) {
      ROS_ERROR("Must be exactly one predicted object in these scenes.");
    }
    shared_ptr<SceneObject> predictedObject = this_scene->predicted_objects[0];

    eePose scene_pose = predictedObject->scene_pose;
    double roll, pitch, yaw;
    scene_pose.getRollPitchYaw(&roll, &pitch, &yaw);
    //this_scene.findBestObjectAndScore(&this_class, num_orientations, &this_x_cell, &this_y_cell, &this_orient, &this_score, &this_i);

    double this_theta = yaw;
    double this_x = scene_pose.px;
    double this_y = scene_pose.py;
    cout << "Got pose: " << this_x << ", " << this_y << " and theta " << this_theta << endl;
    
    // integrate
    sum_x += this_x;
    sum_y += this_y;
    sum_theta += this_theta;
    sum_x_squared += this_x*this_x;
    sum_y_squared += this_y*this_y;
    sum_theta_squared += this_theta*this_theta;
    counts++;
  }

  double safe_counts = std::max(counts, 1.0);
  double mean_x = sum_x / safe_counts;
  double mean_y = sum_y / safe_counts;
  double mean_theta = sum_theta / safe_counts;


  double sum_dist = 0.0;
  double sum_dist_squared = 0.0;
  vector<double> result;
  for (int i = 0; i < scenes.size(); i++) {
    shared_ptr<Scene> this_scene = scenes[i];
    shared_ptr<SceneObject> predictedObject = this_scene->predicted_objects[0];

    eePose scene_pose = predictedObject->scene_pose;
    double roll, pitch, yaw;
    scene_pose.getRollPitchYaw(&roll, &pitch, &yaw);
    //this_scene.findBestObjectAndScore(&this_class, num_orientations, &this_x_cell, &this_y_cell, &this_orient, &this_score, &this_i);

    double this_theta = yaw;
    double this_x = scene_pose.px;
    double this_y = scene_pose.py;


    double squaredist = pow(mean_x  - this_x, 2) + pow(mean_y - this_y, 2);
    double dist = sqrt(squaredist);
    result.push_back(dist);
    sum_dist_squared += squaredist;
    sum_dist += dist;
      
  }


  
  double mean_dist = sum_dist / safe_counts;
  
  double variance_x = ( sum_x_squared / safe_counts ) - ( mean_x * mean_x ); 
  double variance_y = ( sum_y_squared / safe_counts ) - ( mean_y * mean_y ); 
  double variance_theta = ( sum_theta_squared / safe_counts ) - ( mean_theta * mean_theta ); 
  double variance_dist = ( sum_dist_squared / safe_counts ) - ( mean_dist * mean_dist );

  double stddev_x = sqrt(variance_x);
  double stddev_y = sqrt(variance_y);
  double stddev_theta = sqrt(variance_theta);
  double stddev_dist = sqrt(variance_dist);

  double stderror_x = stddev_x / safe_counts;
  double stderror_y = stddev_y / safe_counts;
  double stderror_theta = stddev_theta / safe_counts;
  double stderror_dist = stddev_dist / safe_counts;

  double muconf95_x = stderror_x / 1.96;
  double muconf95_y = stderror_theta / 1.96;
  double muconf95_theta = stderror_theta / 1.96;
  double muconf95_dist = stderror_dist / 1.96;


  double stddevconf95_xlow = sqrt(pow((safe_counts - 1), 2) * variance_x) / 13.844;
  double stddevconf95_xhigh = sqrt(pow((safe_counts - 1), 2) * variance_x) / 41.923;

  double stddevconf95_ylow = sqrt(pow((safe_counts - 1), 2) * variance_y) / 13.844;
  double stddevconf95_yhigh = sqrt(pow((safe_counts - 1), 2) * variance_y) / 41.923;

  double stddevconf95_thetalow = sqrt(pow((safe_counts - 1), 2) * variance_theta) / 13.844;
  double stddevconf95_thetahigh = sqrt(pow((safe_counts - 1), 2) * variance_theta) / 41.923;

  double stddevconf95_distlow = sqrt(pow((safe_counts - 1), 2) * variance_dist) / 13.844;
  double stddevconf95_disthigh = sqrt(pow((safe_counts - 1), 2) * variance_dist) / 41.923;




  cout << "variance report: " << endl;
  cout << "mu_x: " << mean_x << "+/-" << muconf95_x << " sigma_squared_x: " << variance_x << " stddev_x: " << stddev_x << "+/-" << stddevconf95_xlow << "-" << stddevconf95_xhigh << endl;
  cout << "mu_y: " << mean_y << "+/-" << muconf95_y << " sigma_squared_y: " << variance_y << " stddev_y: " << stddev_y << "+/-" << stddevconf95_ylow << "-" << stddevconf95_yhigh << endl;
  cout << "mu_theta: " << mean_theta << "+/-" << muconf95_theta << " sigma_squared_theta: " << variance_theta << " stddev_theta: " << stddev_theta << "+/-" << stddevconf95_thetalow << "-" << stddevconf95_thetahigh << endl;
  cout << "mu_dist: " << mean_dist << "+/-" << muconf95_dist << " sigma_squared_dist: " << variance_dist << " stddev_dist: " << stddev_dist << "+/-" << stddevconf95_distlow << "-" << stddevconf95_disthigh << endl;

  return result;
}

WORD(CatScan5VarianceTrialCalculatePoseVariances)
virtual void execute(std::shared_ptr<MachineState> ms) {
// XXX 
  /* loop over all variance trial files, estimate poses and configurations, 
     and calculate the variance of those estimates.
       pre-requisite: you should use setClassLabelsBaseClassAbsolute to load
       configurations for the object whose variance trial folder you pass to this word. */
  string baseClassTrialFolderName;
  GET_STRING_ARG(ms, baseClassTrialFolderName);
  vector< string > dir_files;
  vector< vector<string> > scene_files;

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  dpdf = opendir(baseClassTrialFolderName.c_str());
  if (dpdf == NULL){
    ROS_ERROR_STREAM("catScan5VarianceTrialCalculatePoseVariances: could not open base class dir " << baseClassTrialFolderName << " ." << endl);
    return;
  }

  while (epdf = readdir(dpdf)){
    string thisFileName(epdf->d_name);
    
    string thisFullFileName(baseClassTrialFolderName.c_str());
    thisFullFileName = thisFullFileName + "/" + thisFileName;
    
    struct stat buf2;
    stat(thisFullFileName.c_str(), &buf2);
    
    int itIsADir = S_ISDIR(buf2.st_mode);
    if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && itIsADir) {
      cout << " is a directory." << endl;
      dir_files.push_back(thisFullFileName);
    } else if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {
      cout << " is NOT a directory." << endl;
	//scene_files.push_back(thisFullFileName);
    }
  }
  closedir(dpdf);
  
  for (int i = 0; i < dir_files.size(); i++){
    vector<string> thisdir_files;
    dpdf = opendir(dir_files[i].c_str());
    cout << "Opening: " << dir_files[i] << endl;
    while (epdf = readdir(dpdf)){
      string thisFileName(epdf->d_name);
      string thisFullFileName(dir_files[i].c_str());
      thisFullFileName = thisFullFileName + "/" + thisFileName;
      struct stat buf2;
      stat(thisFullFileName.c_str(), &buf2);
      int itIsADir = S_ISDIR(buf2.st_mode);
      if (itIsADir) {
	cout << " is a directory." << endl;
	//dir_files.push_back(thisFullFileName);
      } else {
	cout << " is NOT a directory." << endl;
	thisdir_files.push_back(thisFullFileName);
      }
    }
    closedir(dpdf);
    scene_files.push_back(thisdir_files);
  }

  cout << "catScan5VarianceTrialCalculatePoseVariances found the following files:" << endl;;
  for (int i = 0; i < scene_files.size(); i++) {
    for (int j = 0; j < scene_files[i].size(); j++) {
      cout << "File: " << scene_files[i][j] << endl;
    }
  }
  cout << endl;
  vector<double> distances;
  for (int i = 0; i < scene_files.size(); i++) {
    vector<double> batch_distances = poseVarianceOfEvaluationScenes(ms, scene_files[i]);
    for (int j = 0; j < batch_distances.size(); j++) {
      distances.push_back(batch_distances[j]);
    }
  }
  double sum_squared = 0;
  double sum = 0;
  for (int i = 0; i < distances.size(); i++) {
    cout << "dist: " << distances[i] << endl;
    sum += distances[i];
    sum_squared += pow(distances[i], 2);
  }
  cout << "Sum: " << sum << endl;
  double safe_counts = std::max((double) distances.size(), 1.0);
  double mean = sum / safe_counts;
  double variance = ( sum_squared / safe_counts ) - ( mean * mean ); 
  double stddev = sqrt(variance);
  double stderror = stddev / safe_counts;
  double muconf95 = stderror / 1.96;
  cout << "overall variance report: " << endl;
  cout << "mu_d: " << mean << "+/-" << muconf95 << " sigma_squared: " << variance << " stddev: " << stddev << endl;

}
END_WORD
REGISTER_WORD(CatScan5VarianceTrialCalculatePoseVariances)

WORD(CatScan5VarianceTrialAutolabelClassNames)
virtual void execute(std::shared_ptr<MachineState> ms) {
// XXX 
  /* loop over all variance trial files, estimate poses and configurations, 
     and pause to allow a human to set the true label.
       pre-requisite: you should use setClassLabelsBaseClassAbsolute to load
       configurations for the object whose variance trial folder you pass to this word. 
       use note: stack will pause between each example, allowing you to relabel the scene
       before proceeding, at which point the scene will be saved to disk and the next scene
       loaded, pausing again. */

  string baseClassTrialFolderName;
  GET_STRING_ARG(ms, baseClassTrialFolderName);

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  dpdf = opendir(baseClassTrialFolderName.c_str());
  if (dpdf != NULL){
    cout << "catScan5VarianceTrialAutolabelClassNames: checking " << baseClassTrialFolderName << " during snoop...";
    while (epdf = readdir(dpdf)){
      string thisFileName(epdf->d_name);

      string thisFullFileName(baseClassTrialFolderName.c_str());
      thisFullFileName = thisFullFileName + "/" + thisFileName;
      cout << "catScan5VarianceTrialAutolabelClassNames: checking " << thisFullFileName << " during snoop...";

      struct stat buf2;
      stat(thisFullFileName.c_str(), &buf2);

      int itIsADir = S_ISDIR(buf2.st_mode);
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && itIsADir) {
	cout << " is a directory." << endl;
      } else if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {
	cout << " is NOT a directory." << endl;
	ms->pushWord("sceneSaveSceneAbsolute");
	ms->pushWord( make_shared<StringWord>(thisFullFileName) );
	ms->pushWord("endStackCollapseNoop");
	ms->pushWord("tempUpdateMaps");
	// labels and leaves these detections in the scene
	ms->pushWord("sceneSetClassNameToFocusedClass");
	ms->pushWord("scenePredictBestObject");
	ms->pushWord("tempUpdateMaps");
	ms->pushWord("sceneClearPredictedObjects");
	ms->pushWord("sceneLoadSceneRaw");
	ms->pushWord( make_shared<StringWord>(thisFullFileName) );
      }
    }
  } else {
    ROS_ERROR_STREAM("catScan5VarianceTrialAutolabelClassNames: could not open base class dir " << baseClassTrialFolderName << " ." << endl);
  } 
}
END_WORD
REGISTER_WORD(CatScan5VarianceTrialAutolabelClassNames)

WORD(CatScan5VarianceTrialAuditClassNames)
virtual void execute(std::shared_ptr<MachineState> ms) {
// XXX 
  /* loop over all variance trial files, estimate poses and configurations, 
     and pause to allow a human to set the true label.
       pre-requisite: you should use setClassLabelsBaseClassAbsolute to load
       configurations for the object whose variance trial folder you pass to this word. 
       use note: stack will pause between each example, allowing you to relabel the scene
       before proceeding, at which point the scene will be saved to disk and the next scene
       loaded, pausing again. */

  string baseClassTrialFolderName;
  GET_STRING_ARG(ms, baseClassTrialFolderName);

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  dpdf = opendir(baseClassTrialFolderName.c_str());
  if (dpdf != NULL){
    cout << "catScan5VarianceTrialAuditClassNames: checking " << baseClassTrialFolderName << " during snoop...";
    while (epdf = readdir(dpdf)){
      string thisFileName(epdf->d_name);

      string thisFullFileName(baseClassTrialFolderName.c_str());
      thisFullFileName = thisFullFileName + "/" + thisFileName;
      cout << "catScan5VarianceTrialAuditClassNames: checking " << thisFullFileName << " during snoop...";

      struct stat buf2;
      stat(thisFullFileName.c_str(), &buf2);

      int itIsADir = S_ISDIR(buf2.st_mode);
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && itIsADir) {
	cout << " is a directory." << endl;
      } else if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {
	cout << " is NOT a directory." << endl;
	ms->pushWord("pauseStackExecution");
	ms->pushWord("tempUpdateMaps");
	// preserves the prediction that was last saved
	ms->pushWord("sceneLoadSceneRaw");
	ms->pushWord( make_shared<StringWord>(thisFullFileName) );
       
      }
    }
  } else {
    ROS_ERROR_STREAM("catScan5VarianceTrialAuditClassNames: could not open base class dir " << baseClassTrialFolderName << " ." << endl);
  } 

}
END_WORD
REGISTER_WORD(CatScan5VarianceTrialAuditClassNames)

WORD(CatScan5VarianceTrialCalculateConfigurationAccuracy)
virtual void execute(std::shared_ptr<MachineState> ms) {
// XXX 
  /* loop over all variance trial files and classify under all classes all configurations.
       pre-requisite: you should use setClassLabelsBaseClassAbsolute to load
       configurations for the object whose variance trial folder you pass to this word. */

  int nc = ms->config.numClasses;
  double * result =  new double[nc * nc];
  for (int i = 0; i < nc; i++) {
    for (int j = 0; j < nc; j++) {
      result[i + nc * j] = 0;
    }
  }

  string baseClassTrialFolderName;
  GET_STRING_ARG(ms, baseClassTrialFolderName);

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  dpdf = opendir(baseClassTrialFolderName.c_str());
  if (dpdf != NULL){
    cout << "catScan5VarianceTrialCalculateConfigurationAccuracy: checking " << baseClassTrialFolderName << " during snoop...";
    while (epdf = readdir(dpdf)){
      string thisFileName(epdf->d_name);

      string thisFullFileName(baseClassTrialFolderName.c_str());
      thisFullFileName = thisFullFileName + "/" + thisFileName;
      cout << "catScan5VarianceTrialCalculateConfigurationAccuracy: checking " << thisFullFileName << " during snoop...";

      struct stat buf2;
      stat(thisFullFileName.c_str(), &buf2);

      int itIsADir = S_ISDIR(buf2.st_mode);
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && itIsADir) {
	cout << " is a directory." << endl;
      } else if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {
	cout << " is NOT a directory." << endl;
	// load scene and detect 
	// XXX
	int num_orientations = 37;
	int this_class = -1;
	double this_score = -DBL_MAX;
	int this_i = -1;
	int this_x_cell = 0,this_y_cell = 1,this_orient= 2;
	Scene this_scene(ms, 2, 2, 0.02);
	this_scene.loadFromFile(thisFullFileName);
	this_scene.predicted_objects.resize(0);
	this_scene.composePredictedMap(0.01);
	this_scene.measureDiscrepancy();

	this_scene.findBestObjectAndScore(&this_class, num_orientations, &this_x_cell, &this_y_cell, &this_orient, &this_score, &this_i);
	int thisSceneLabelIdx = -1;
	for (int i = 0; i < nc; i++) {
	  if ( 0 == ms->config.classLabels[i].compare(this_scene.annotated_class_name) ) {
	    thisSceneLabelIdx = i;
	    break;
	  } else {
	  }
	}
	if ( (this_class != -1) && (thisSceneLabelIdx != -1) ) {
	  result[thisSceneLabelIdx + nc * this_class]++;
	} else {
	  cout << "catScan5VarianceTrialCalculateConfigurationAccuracy: could not find match for label " << this_scene.annotated_class_name << " for file " << thisFullFileName << endl;
	}
      }
    }
  } else {
    ROS_ERROR_STREAM("catScan5VarianceTrialCalculateConfigurationAccuracy: could not open base class dir " << baseClassTrialFolderName << " ." << endl);
  } 

  cout << "catScan5VarianceTrialCalculateConfigurationAccuracy report: row (i) is true label, column (j) is assigned label" << endl;

  for (int j = 0; j < nc; j++) {
    cout << std::setw(3) << j << ": " << ms->config.classLabels[j] << endl;
  }
  cout << "   " ;
  for (int j = 0; j < nc; j++) {
    cout << std::setw(10) << j ;
  }
  cout << endl;

  for (int i = 0; i < nc; i++) {
    cout << std::setw(3) << i << " ";
    for (int j = 0; j < nc; j++) {
      cout << std::setw(10) << result[i + nc * j] << " ";
    }
    cout << endl;
  }

  delete result;
}
END_WORD
REGISTER_WORD(CatScan5VarianceTrialCalculateConfigurationAccuracy)

WORD(CatScan5VarianceTrialCalculateAllClassesAccuracy)
virtual void execute(std::shared_ptr<MachineState> ms) {
// XXX 
  /* loop over all variance trial files and classify under all classes all configurations.
       pre-requisite: you should use setClassLabelsObjectFolderAbsolute to load all
       configurations for all objects whose base dirs are in the folder passed to this word. */
  int nc = ms->config.numClasses;
  double * result =  new double[nc * nc];
  for (int i = 0; i < nc; i++) {
    for (int j = 0; j < nc; j++) {
      result[i + nc * j] = 0;
    }
  }

  string objectFolderAbsolute;
  GET_STRING_ARG(ms, objectFolderAbsolute);

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  dpdf = opendir(objectFolderAbsolute.c_str());
  if (dpdf != NULL){
    cout << "catScan5VarianceTrialCalculateAllClassesAccuracy level 1: checking " << objectFolderAbsolute << " during snoop...";
    while (epdf = readdir(dpdf)){
      string thisFileName(epdf->d_name);

      string thisFullFileName(objectFolderAbsolute.c_str());
      thisFullFileName = thisFullFileName + "/" + thisFileName + "/catScan5VarianceTrials/";
      cout << "catScan5VarianceTrialCalculateAllClassesAccuracy level 1: checking " << thisFullFileName << " during snoop...";

      struct stat buf2;
      stat(thisFullFileName.c_str(), &buf2);

      //string varianceTrials("catScan5VarianceTrials");

      int itIsADir = S_ISDIR(buf2.st_mode);
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && itIsADir) {
	cout << " is a directory." << endl;

	{
	  string thisFolderName = thisFullFileName;
	  DIR *dpdf_2;
	  struct dirent *epdf_2;

	  dpdf_2 = opendir(thisFolderName.c_str());
	  if (dpdf_2 != NULL){
	    cout << "catScan5VarianceTrialCalculateAllClassesAccuracy level 2: checking " << thisFolderName << " during snoop...";
	    while (epdf_2 = readdir(dpdf_2)){
	      string thisFileName_2(epdf_2->d_name);

	      string thisFullFileName_2(thisFolderName.c_str());
	      thisFullFileName_2 = thisFullFileName_2 + "/" + thisFileName_2;
	      cout << "catScan5VarianceTrialCalculateAllClassesAccuracy level 2: checking " << thisFullFileName_2 << " during snoop...";

	      struct stat buf2;
	      stat(thisFullFileName_2.c_str(), &buf2);

	      string varianceTrials("catScan5VarianceTrials");

	      int itIsADir = S_ISDIR(buf2.st_mode);
	      if (dot.compare(epdf_2->d_name) && dotdot.compare(epdf_2->d_name) && itIsADir) {
		cout << " is a directory." << endl;
	      } else if (dot.compare(epdf_2->d_name) && dotdot.compare(epdf_2->d_name)) {
		cout << " is NOT a directory." << endl;
		// load scene and detect 
		// XXX
		int num_orientations = 37;
		int this_class = -1;
		double this_score = -DBL_MAX;
		int this_i = -1;
		int this_x_cell = 0,this_y_cell = 1,this_orient= 2;
		Scene this_scene(ms, 2, 2, 0.02);
		this_scene.loadFromFile(thisFullFileName_2);
		this_scene.predicted_objects.resize(0);
		this_scene.composePredictedMap(0.01);
		this_scene.measureDiscrepancy();

		this_scene.findBestObjectAndScore(&this_class, num_orientations, &this_x_cell, &this_y_cell, &this_orient, &this_score, &this_i);
		int thisSceneLabelIdx = -1;
		for (int i = 0; i < nc; i++) {
		  // remove object folder token from front
		  string class_label_to_trim = ms->config.classLabels[i]; 
		  int first = 0;
		  // find first non-slash character
		  if (class_label_to_trim[first] == '/') {
		    first = first+1;
		    while (first < class_label_to_trim.size()) {
		      if (class_label_to_trim[first] == '/') {
			first = first+1;
		      } else {
			break;
		      }
		    }
		  }
		  // then find the next slash
		  int next = first;
		  while (next < class_label_to_trim.size()) {
		    if (class_label_to_trim[next] == '/') {
		      break;
		    } else {
		      next = next+1;
		    }
		  }
		  // and the character after that series of slashes
		  int after_next_series = next;
		  while (after_next_series < class_label_to_trim.size()) {
		    if (class_label_to_trim[after_next_series] == '/') {
		      after_next_series = after_next_series+1;
		    } else {
		      break;
		    }
		  }
		  std::min(after_next_series, int(class_label_to_trim.size()-1));
		  // then take substring
		  string trimmed_class_label = class_label_to_trim.substr(after_next_series, class_label_to_trim.size()-after_next_series);
		  //cout << "TTTTTT: " << trimmed_class_label << "    " << this_scene.annotated_class_name << endl;

		  if ( 0 == trimmed_class_label.compare(this_scene.annotated_class_name) ) {
		    thisSceneLabelIdx = i;
		    break;
		  } else {
		  }
		}
		if ( (this_class != -1) && (thisSceneLabelIdx != -1) ) {
		  result[thisSceneLabelIdx + nc * this_class]++;
		} else {
		  cout << "catScan5VarianceTrialCalculateConfigurationAccuracy: could not find match for label " << this_scene.annotated_class_name << " for file " << thisFullFileName << endl;
		}
	      }
	    }
	  } else {
	    ROS_ERROR_STREAM("catScan5VarianceTrialCalculateAllClassesAccuracy level 2: could not open base class dir " << objectFolderAbsolute << " ." << endl);
	  } 
	}

      } else if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {
	cout << " is NOT a directory." << endl;
      }
    }
  } else {
    ROS_ERROR_STREAM("catScan5VarianceTrialCalculateAllClassesAccuracy level 1: could not open base class dir " << objectFolderAbsolute << " ." << endl);
  } 

  cout << "catScan5VarianceTrialCalculateAllClassesAccuracy report: row (i) is true label, column (j) is assigned label" << endl;

  for (int j = 0; j < nc; j++) {
    cout << std::setw(3) << j << ": " << ms->config.classLabels[j] << endl;
  }
  cout << "   " ;
  for (int j = 0; j < nc; j++) {
    cout << std::setw(10) << j ;
  }
  cout << endl;

  for (int i = 0; i < nc; i++) {
    cout << std::setw(3) << i << " ";
    for (int j = 0; j < nc; j++) {
      cout << std::setw(10) << result[i + nc * j] << " ";
    }
    cout << endl;
  }
  delete result;
}
END_WORD
REGISTER_WORD(CatScan5VarianceTrialCalculateAllClassesAccuracy)

CONFIG_SETTER_ENUM(SceneSetClassificationMode, ms->config.currentSceneClassificationMode, (sceneClassificationMode))
CONFIG_GETTER_INT(SceneGetClassificationMode, ms->config.currentSceneClassificationMode)

WORD(SceneFabricateIdealBlockModel)
virtual void execute(std::shared_ptr<MachineState> ms) {
  /* the density filter will not be accurately updated if it is recomposed
      because it contains negative values to enforce comparability. 
     this should only be used with SC_DISCREPANCY_ONLY */
  // these could be passed an additional weighting term to change the favor 

  // XXX TODO
  double breadth_m, length_m;
  GET_NUMERIC_ARG(ms, breadth_m);
  GET_NUMERIC_ARG(ms, length_m);

  stringstream ss;
  ss << "ideal_block_" << length_m << "_" << breadth_m;

  initializeAndFocusOnTempClass(ms);
  REQUIRE_FOCUSED_CLASS(ms, tfc);
  ms->config.classLabels[tfc] = ss.str();

  double scale = 1;
  double negative_space_weight_ratio = 1.0;
  double this_collar_width_m = 0.02;
  double this_cw = ms->config.scene->cell_width; 
  int this_w = ceil( (3.0 * this_collar_width_m + breadth_m) / this_cw);
  int this_h = ceil( (3.0 * this_collar_width_m + length_m) / this_cw);
  ms->config.class_scene_models[tfc] = make_shared<Scene>(ms, this_w, this_h, this_cw);

  // fill in the magnitude and density maps; positive region sums to 1, negative collar sums to -1
  Mat fake_filter = ms->config.class_scene_models[tfc]->discrepancy_magnitude;
  int w = fake_filter.cols;
  int h = fake_filter.rows;
  // count cells
  double num_pos = 0;
  double num_neg = 0;
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      double this_x_m, this_y_m;
      ms->config.class_scene_models[tfc]->cellToMeters(x, y, &this_x_m, &this_y_m);
      if ( (fabs(this_y_m) <= (length_m/2.0)) && (fabs(this_x_m) <= (breadth_m/2.0)) ) {
	num_pos++;
      } else if ( (fabs(this_y_m) <= (length_m/2.0)) && (  fabs(this_x_m) <= (this_collar_width_m + (   breadth_m/2.0   ))  ) ) {
	num_neg++;
      } else {
      }
    }
  }

  cout << "w: " << w << " h: " << h << " num_pos: " << num_pos << " num_neg: " << num_neg << " this_w: " << this_w << " this_h: " << this_h << " this_cw: " << this_cw << endl;

  num_pos = std::max(num_pos, 1.0);
  num_neg = std::max(num_neg, 1.0);

  // l2 norm is sqrt(num_pos)
  //double pos_factor = 1.0/sqrt(num_pos);
  //double neg_factor = 10.0/sqrt(num_neg);
  double pos_factor = 1.0/(num_pos);
  double neg_factor = 3.0/(num_neg);

  double counts_scale = 1e4;

  // fill out proper values
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      double this_x_m, this_y_m;
      ms->config.class_scene_models[tfc]->cellToMeters(x, y, &this_x_m, &this_y_m);
      if ( (fabs(this_y_m) <= (length_m/2.0)) && (fabs(this_x_m) <= (breadth_m/2.0)) ) {
	fake_filter.at<double>(y,x) = scale*pos_factor;
	if ( (fabs(this_y_m) <= (length_m/2.0)) && (fabs(this_x_m) <= (breadth_m/4.0)) ) {
	  ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->red.counts = counts_scale*128;
	  ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->blue.counts = counts_scale*128;
	  ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->green.counts = counts_scale*192;
	} else {
	  ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->red.counts = counts_scale*128;
	  ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->blue.counts = counts_scale*128;
	  ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->green.counts = counts_scale*64;
	}
      } else if ( (fabs(this_y_m) <= (length_m/2.0)) && (  fabs(this_x_m) <= (this_collar_width_m + (   breadth_m/2.0   ))  ) ) {
	fake_filter.at<double>(y,x) = -negative_space_weight_ratio*scale*neg_factor;
	ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->red.counts = counts_scale*128;
	ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->blue.counts = counts_scale*128;
	ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->green.counts = counts_scale*128;
      } else {
	fake_filter.at<double>(y,x) = 0.0;
	ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->red.counts = counts_scale*128;
	ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->blue.counts = counts_scale*128;
	ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->green.counts = counts_scale*128;
      }
      ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->red.samples = counts_scale;
      ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->blue.samples = counts_scale;
      ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->green.samples = counts_scale;
      ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->red.squaredcounts = 
	pow(ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->red.counts / counts_scale, 2.0)*counts_scale;
      ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->blue.squaredcounts = 
	pow(ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->blue.counts / counts_scale, 2.0)*counts_scale;
      ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->green.squaredcounts = 
	pow(ms->config.class_scene_models[tfc]->observed_map->refAtCell(x, y)->green.counts / counts_scale, 2.0)*counts_scale;
    }
  }
  ms->config.class_scene_models[tfc]->discrepancy_density = fake_filter.clone();
  // XXX push a table flush 3d crane grasp at the center
  Grasp toPush;
  double flushGraspZ = -ms->config.currentTableZ + ms->config.pickFlushFactor;
  toPush.grasp_pose = eePose(0,0,flushGraspZ,0.0,0.0,0.707106,0.7071068);
  //toPush.grasp_pose = eePose(0,0,flushGraspZ,0.0,0.0,0.0,1.0);
  toPush.tries = 1;
  toPush.successes = 1;
  toPush.failures = 0;
  ms->config.class3dGrasps[tfc].resize(0);
  ms->config.class3dGrasps[tfc].push_back(toPush);
}
END_WORD
REGISTER_WORD(SceneFabricateIdealBlockModel)





WORD(SceneInitRegisterMax)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "sceneInitRegister: copying and maxing variances..." << endl;
  ms->config.gaussian_map_register = ms->config.scene->observed_map->copy();
  int t_height = ms->config.gaussian_map_register->height;
  int t_width = ms->config.gaussian_map_register->width;
  for (int y = 0; y < t_height; y++) {
    for (int x = 0; x < t_width; x++) {
      ms->config.gaussian_map_register->refAtCell(x,y)->zero();
      ms->config.gaussian_map_register->refAtCell(x,y)->red.sigmasquared = DBL_MAX;
      ms->config.gaussian_map_register->refAtCell(x,y)->green.sigmasquared = DBL_MAX;
      ms->config.gaussian_map_register->refAtCell(x,y)->blue.sigmasquared = DBL_MAX;
    }
  }
}
END_WORD
REGISTER_WORD(SceneInitRegisterMax)

WORD(SceneInitRegisterZero)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "sceneInitRegister: copying and maxing variances..." << endl;
  ms->config.gaussian_map_register = ms->config.scene->observed_map->copy();
  int t_height = ms->config.gaussian_map_register->height;
  int t_width = ms->config.gaussian_map_register->width;
  for (int y = 0; y < t_height; y++) {
    for (int x = 0; x < t_width; x++) {
      ms->config.gaussian_map_register->refAtCell(x,y)->zero();
      ms->config.gaussian_map_register->refAtCell(x,y)->red.sigmasquared = 0;
      ms->config.gaussian_map_register->refAtCell(x,y)->green.sigmasquared = 0;
      ms->config.gaussian_map_register->refAtCell(x,y)->blue.sigmasquared = 0;
    }
  }
}
END_WORD
REGISTER_WORD(SceneInitRegisterZero)

WORD(SceneRecallFromRegister)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "sceneRecallFromRegister: copying..." << endl;
  ms->config.scene->observed_map = ms->config.gaussian_map_register->copy();
}
END_WORD
REGISTER_WORD(SceneRecallFromRegister)

WORD(SceneUpdateObservedFromStreamBufferAtZ)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);

  int thisIdx = ms->config.sibCurIdx;
  //cout << "sceneUpdateObservedFromStreamBuffer: " << thisIdx << endl;

  Mat bufferImage;
  eePose thisPose, tBaseP;

  int success = 1;
  if ( (thisIdx > -1) && (thisIdx < ms->config.streamImageBuffer.size()) ) {
    streamImage &tsi = ms->config.streamImageBuffer[thisIdx];
    if (tsi.image.data == NULL) {
      cout << "  encountered NULL data in sib, returning." << endl;
      return;
    } else {
      bufferImage = tsi.image.clone();
    }
    success = getStreamPoseAtTime(ms, tsi.time, &thisPose, &tBaseP);
  } else {
    ROS_ERROR_STREAM("No images in the buffer, returning." << endl);
    return;
  }

  if (success != 1) {
    ROS_ERROR("  Not doing update because of stream buffer errors.");
    return;
  }

  if (fabs(thisPose.qz) > 0.01) {
    ROS_ERROR("  Not doing update because arm not vertical.");
    return;
  }

  Mat wristViewYCbCr = bufferImage.clone();

  cvtColor(bufferImage, wristViewYCbCr, CV_BGR2YCrCb);
  
  Size sz = bufferImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int topx = ms->config.grayTop.x+ms->config.mapGrayBoxPixelSkirtCols; //+ 20; // ms->config.grayTop.x;  
  int botx = ms->config.grayBot.x-ms->config.mapGrayBoxPixelSkirtCols; //- 20; // ms->config.grayBot.x;  
  int topy = ms->config.grayTop.y+ms->config.mapGrayBoxPixelSkirtRows; //+ 50; // ms->config.grayTop.y;
  int boty = ms->config.grayBot.y-ms->config.mapGrayBoxPixelSkirtRows; //- 50; // ms->config.grayBot.y;  
  
  pixelToGlobalCache data;
  double z = z_to_use;
  computePixelToGlobalCache(ms, z, thisPose, &data);
  
  for (int px = topx; px < botx; px++) {
    for (int py = topy; py < boty; py++) {
      if (isInGripperMask(ms, px, py)) {
	continue;
      }
      double x, y;
      pixelToGlobalFromCache(ms, px, py, z, &x, &y, thisPose, &data);
      
      if (1) {
	// single sample update
	int i, j;
	ms->config.scene->observed_map->metersToCell(x, y, &i, &j);
	GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(i, j);
	if (cell != NULL) {
	    Vec3b pixel = wristViewYCbCr.at<Vec3b>(py, px);
	    cell->newObservation(pixel, z);
	}
      } else {
      }
    }
  }
  ms->config.scene->observed_map->recalculateMusAndSigmas(ms);
  ms->pushWord("sceneRenderObservedMap");
}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZ)

void sceneMinIntoRegisterHelper(std::shared_ptr<MachineState> ms, shared_ptr<GaussianMap> toMin) {

  int t_height = toMin->height;
  int t_width = toMin->width;

  assert( ms->config.gaussian_map_register->width == t_width );
  assert( ms->config.gaussian_map_register->height == t_height );

  for (int y = 0; y < t_height; y++) {
    for (int x = 0; x < t_width; x++) {
      if ( (toMin->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold) ) {
	double this_observed_sigma_squared = 
	toMin->refAtCell(x,y)->red.sigmasquared +
	toMin->refAtCell(x,y)->green.sigmasquared +
	toMin->refAtCell(x,y)->blue.sigmasquared;

	double this_register_sigma_squared = 
	ms->config.gaussian_map_register->refAtCell(x,y)->red.sigmasquared +
	ms->config.gaussian_map_register->refAtCell(x,y)->green.sigmasquared +
	ms->config.gaussian_map_register->refAtCell(x,y)->blue.sigmasquared;

	
	double thisObservedPixelArea = 1.0;
	double thisRegisterPixelArea = 1.0;
	/*
	{
	  double meters_scene_x_0, meters_scene_y_0;
	  ms->config.scene->cellToMeters(0, 0, &meters_scene_x_0, &meters_scene_y_0);
	  double meters_scene_x_1, meters_scene_y_1;
	  ms->config.scene->cellToMeters(1, 1, &meters_scene_x_1, &meters_scene_y_1);

	  double zToUse = toMin->refAtCell(x,y)->z.mu;
	  int pixel_scene_x_0, pixel_scene_y_0;
	  globalToPixel(ms, &pixel_scene_x_0, &pixel_scene_y_0, zToUse, meters_scene_x_0, meters_scene_y_0);
	  int pixel_scene_x_1, pixel_scene_y_1;
	  globalToPixel(ms, &pixel_scene_x_1, &pixel_scene_y_1, zToUse, meters_scene_x_1, meters_scene_y_1);

	  thisObservedPixelArea =  fabs( pixel_scene_x_1 - pixel_scene_x_0 ) * fabs( pixel_scene_y_1 - pixel_scene_y_0 );
	}

	{
	  double meters_scene_x_0, meters_scene_y_0;
	  ms->config.gaussian_map_register->cellToMeters(0, 0, &meters_scene_x_0, &meters_scene_y_0);
	  double meters_scene_x_1, meters_scene_y_1;
	  ms->config.gaussian_map_register->cellToMeters(1, 1, &meters_scene_x_1, &meters_scene_y_1);

	  double zToUse = ms->config.gaussian_map_register->refAtCell(x,y)->z.mu;
	  int pixel_scene_x_0, pixel_scene_y_0;
	  globalToPixel(ms, &pixel_scene_x_0, &pixel_scene_y_0, zToUse, meters_scene_x_0, meters_scene_y_0);
	  int pixel_scene_x_1, pixel_scene_y_1;
	  globalToPixel(ms, &pixel_scene_x_1, &pixel_scene_y_1, zToUse, meters_scene_x_1, meters_scene_y_1);

	  thisRegisterPixelArea = fabs( pixel_scene_x_1 - pixel_scene_x_0 ) * fabs( pixel_scene_y_1 - pixel_scene_y_0 );
	}
	
	
	thisRegisterPixelArea = std::max(thisRegisterPixelArea, 1.0);
	thisObservedPixelArea = std::max(thisObservedPixelArea, 1.0);
	*/

	if ( this_observed_sigma_squared/thisObservedPixelArea < this_register_sigma_squared/thisRegisterPixelArea ) {
	  *(ms->config.gaussian_map_register->refAtCell(x,y)) = *(toMin->refAtCell(x,y));
	} else {
	}
      } else {
      }
    }
  }

}

void sceneMarginalizeIntoRegisterHelper(std::shared_ptr<MachineState> ms, shared_ptr<GaussianMap> toMin) {

// XXX 
// XXX  NOT DONE TODO
  int t_height = toMin->height;
  int t_width = toMin->width;

  assert( ms->config.gaussian_map_register->width == t_width );
  assert( ms->config.gaussian_map_register->height == t_height );

  for (int y = 0; y < t_height; y++) {
    for (int x = 0; x < t_width; x++) {
      if ( (toMin->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold) ) {
	double this_observed_sigma_squared = 
	toMin->refAtCell(x,y)->red.sigmasquared +
	toMin->refAtCell(x,y)->green.sigmasquared +
	toMin->refAtCell(x,y)->blue.sigmasquared;

	double this_register_sigma_squared = 
	ms->config.gaussian_map_register->refAtCell(x,y)->red.sigmasquared +
	ms->config.gaussian_map_register->refAtCell(x,y)->green.sigmasquared +
	ms->config.gaussian_map_register->refAtCell(x,y)->blue.sigmasquared;

	double thisObservedPixelArea = 1.0;
	double thisRegisterPixelArea = 1.0;
	/*
	{
	  double meters_scene_x_0, meters_scene_y_0;
	  ms->config.scene->cellToMeters(0, 0, &meters_scene_x_0, &meters_scene_y_0);
	  double meters_scene_x_1, meters_scene_y_1;
	  ms->config.scene->cellToMeters(1, 1, &meters_scene_x_1, &meters_scene_y_1);
	  double zToUse = toMin->refAtCell(x,y)->z.mu;
	  int pixel_scene_x_0, pixel_scene_y_0;
	  globalToPixel(ms, &pixel_scene_x_0, &pixel_scene_y_0, zToUse, meters_scene_x_0, meters_scene_y_0);
	  int pixel_scene_x_1, pixel_scene_y_1;
	  globalToPixel(ms, &pixel_scene_x_1, &pixel_scene_y_1, zToUse, meters_scene_x_1, meters_scene_y_1);
	  thisObservedPixelArea =  fabs( pixel_scene_x_1 - pixel_scene_x_0 ) * fabs( pixel_scene_y_1 - pixel_scene_y_0 );
	}
	{
	  double meters_scene_x_0, meters_scene_y_0;
	  ms->config.gaussian_map_register->cellToMeters(0, 0, &meters_scene_x_0, &meters_scene_y_0);
	  double meters_scene_x_1, meters_scene_y_1;
	  ms->config.gaussian_map_register->cellToMeters(1, 1, &meters_scene_x_1, &meters_scene_y_1);

	  double zToUse = ms->config.gaussian_map_register->refAtCell(x,y)->z.mu;
	  int pixel_scene_x_0, pixel_scene_y_0;
	  globalToPixel(ms, &pixel_scene_x_0, &pixel_scene_y_0, zToUse, meters_scene_x_0, meters_scene_y_0);
	  int pixel_scene_x_1, pixel_scene_y_1;
	  globalToPixel(ms, &pixel_scene_x_1, &pixel_scene_y_1, zToUse, meters_scene_x_1, meters_scene_y_1);
	  thisRegisterPixelArea = fabs( pixel_scene_x_1 - pixel_scene_x_0 ) * fabs( pixel_scene_y_1 - pixel_scene_y_0 );
	}
	thisRegisterPixelArea = std::max(thisRegisterPixelArea, 1.0);
	thisObservedPixelArea = std::max(thisObservedPixelArea, 1.0);
	*/

	// weight by normalizing function
	  double rescalar = 1.0;
	  //cout << " hit " << this_observed_sigma_squared << " " ;
	  GaussianMapCell toAdd = *(toMin->refAtCell(x,y));
	  toAdd.multS(rescalar/sqrt(this_observed_sigma_squared * 2.0 * M_PI));
	  ms->config.gaussian_map_register->refAtCell(x,y)->addC(&toAdd);

	  if ( ( ms->config.gaussian_map_register->refAtCell(x,y)->red.samples < 1.1 ) &&
	       ( ms->config.gaussian_map_register->refAtCell(x,y)->red.samples > 0.0 ) ) {
	    ms->config.gaussian_map_register->refAtCell(x,y)->multS( 1.0 / ms->config.gaussian_map_register->refAtCell(x,y)->red.samples ); 
	  } else {}

	  ms->config.gaussian_map_register->refAtCell(x,y)->recalculateMusAndSigmas(ms);
      } else {
      }
    }
  }

}

WORD(SceneMinIntoRegister)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "sceneMinIntoRegister: copying..." << endl;

  sceneMinIntoRegisterHelper(ms, ms->config.scene->observed_map);
}
END_WORD
REGISTER_WORD(SceneMinIntoRegister)

WORD(SceneTrimDepthWithDiscrepancy)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double thresh = 0.0;
  GET_NUMERIC_ARG(ms, thresh);

  double zToUse = ms->config.currentEEPose.pz + ms->config.currentTableZ;
  cout << "sceneTrimDepthWithDiscrepancy: trimming with z " << zToUse << endl;
  int t_height = ms->config.scene->observed_map->height;
  int t_width = ms->config.scene->observed_map->width;
  for (int y = 0; y < t_height; y++) {
    for (int x = 0; x < t_width; x++) {
      if (ms->config.scene->discrepancy_density.at<double>(y,x) > thresh) {
	// keep its z value
      } else {
	// zero it out
	double samples = ms->config.scene->observed_map->refAtCell(x,y)->z.samples;
	ms->config.scene->observed_map->refAtCell(x,y)->z.counts = zToUse * samples;
	ms->config.scene->observed_map->refAtCell(x,y)->z.squaredcounts = zToUse * zToUse * samples;
      }
    }
  }

  ms->config.scene->observed_map->recalculateMusAndSigmas(ms);
}
END_WORD
REGISTER_WORD(SceneTrimDepthWithDiscrepancy)

WORD(SceneSmoothSquaredCountsAndSamplesXY)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double p_xySigma = 0.5;
  GET_NUMERIC_ARG(ms, p_xySigma);
  //double darkSigma = 1.0;
  //GaussianBlur(accToBlur, accToBlur, cv::Size(0,0), darkSigma, BORDER_REFLECT);

  /*
  Mat scs;
  ms->config.scene->observed_map->rgbSquaredCountsToMat(scs);
  Mat cs;
  ms->config.scene->observed_map->rgbCountsToMat(cs);
  Mat mus;
  ms->config.scene->observed_map->rgbMuToMat(mus);
  */

  Mat ss;
  ms->config.scene->observed_map->rgbSigmaSquaredToMat(ss);
  /* 0.5 give somewhat reasonable results */
  GaussianBlur(ss, ss, cv::Size(0,0), p_xySigma, BORDER_REFLECT);

  int t_height = ms->config.scene->observed_map->height;
  int t_width = ms->config.scene->observed_map->width;
  for (int y = 0; y < t_height; y++) {
    for (int x = 0; x < t_width; x++) {

      CELLREF_EQUALS_VEC3(ms->config.scene->observed_map->refAtCell(x,y), ss.at<Vec3d>(y,x), sigmasquared);

    }
  }
  //ms->config.scene->observed_map->recalculateMusAndSigmas(ms);
}
END_WORD
REGISTER_WORD(SceneSmoothSquaredCountsAndSamplesXY)

WORD(SceneSmoothDepthStackInZ)
virtual void execute(std::shared_ptr<MachineState> ms) {
  double this_zSigma = 1.0;
  GET_NUMERIC_ARG(ms, this_zSigma);

  cout << "sceneSmoothDepthStackInZ: " << endl;
  int tns = ms->config.depth_maps.size();
  Mat stack_core(1,tns, CV_64FC3);

  int t_height = ms->config.scene->observed_map->height;
  int t_width = ms->config.scene->observed_map->width;
  for (int y = 0; y < t_height; y++) {
    for (int x = 0; x < t_width; x++) {

      for (int i = 0; i < tns; i++) {
	VEC3_EQUALS_CELLREF(stack_core.at<Vec3d>(0,i), ms->config.depth_maps[i]->refAtCell(x,y), sigmasquared);
      }

      //GaussianBlur(stack_core, stack_core, cv::Size(1,5), this_zSigma, BORDER_REFLECT);
      //GaussianBlur(stack_core, stack_core, cv::Size(5,1), this_zSigma, this_zSigma, BORDER_REFLECT);
      GaussianBlur(stack_core, stack_core, cv::Size(0,0), this_zSigma, this_zSigma, BORDER_REFLECT);

      for (int i = 0; i < tns; i++) {
	CELLREF_EQUALS_VEC3(ms->config.depth_maps[i]->refAtCell(x,y), stack_core.at<Vec3d>(0,i), sigmasquared);
      }

    }
  }
}
END_WORD
REGISTER_WORD(SceneSmoothDepthStackInZ)

WORD(ScenePushOntoDepthStack)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "scenePushOntoDepthStack: " << endl;
  ms->config.depth_maps.push_back(ms->config.scene->observed_map->copy());
}
END_WORD
REGISTER_WORD(ScenePushOntoDepthStack)

WORD(ScenePushDepthStackSize)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "scenePushDepthStackSize: " << ms->config.depth_maps.size() << endl;
  ms->pushWord( make_shared<DoubleWord>(ms->config.depth_maps.size()) );
}
END_WORD
REGISTER_WORD(ScenePushDepthStackSize)

WORD(SceneClearDepthStack)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "sceneClearDepthStack: " << endl;
  ms->config.depth_maps.resize(0);
}
END_WORD
REGISTER_WORD(SceneClearDepthStack)

WORD(SceneMinDepthStackIntoRegister)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "sceneMinDepthStackIntoRegister: " << endl;

  int tns = ms->config.depth_maps.size();

  for (int i = 0; i < tns; i++) {
    cout << "  minning " << i << " with max " << tns << endl;
    sceneMinIntoRegisterHelper(ms, ms->config.depth_maps[i]);
  }
}
END_WORD
REGISTER_WORD(SceneMinDepthStackIntoRegister)

WORD(SceneMarginalizeDepthStackIntoRegister)
virtual void execute(std::shared_ptr<MachineState> ms) {
  cout << "sceneMarginalizeDepthStackIntoRegister: " << endl;

  int tns = ms->config.depth_maps.size();

  for (int i = 0; i < tns; i++) {
    cout << "  marginalizing" << i << " with max " << tns << endl;
    sceneMarginalizeIntoRegisterHelper(ms, ms->config.depth_maps[i]);
  }
}
END_WORD
REGISTER_WORD(SceneMarginalizeDepthStackIntoRegister)

WORD(SceneRecallDepthStackIndex)
virtual void execute(std::shared_ptr<MachineState> ms) {
  int idx = 0;
  GET_INT_ARG(ms, idx);
  cout << "sceneRecallDepthStackIndex: " << idx << endl;

  int tns = ms->config.depth_maps.size();

  if (idx >= 0 && idx < tns) {
    ms->config.scene->observed_map = ms->config.depth_maps[idx]->copy();
  } else {
    cout << "  sorry, invalid idx." << endl;
  }
}
END_WORD
REGISTER_WORD(SceneRecallDepthStackIndex)

WORD(SceneSpawnClassHarmonics)
virtual void execute(std::shared_ptr<MachineState> ms) {


}
END_WORD
REGISTER_WORD(SceneSpawnClassHarmonics)

WORD(SceneCoalesceClassHarmonics)
virtual void execute(std::shared_ptr<MachineState> ms) {


}
END_WORD
REGISTER_WORD(SceneCoalesceClassHarmonics)

WORD(SceneSaveObservedMapImage)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string fname;
  GET_STRING_ARG(ms, fname);
  
  Mat image;
  ms->config.scene->observed_map->rgbMuToMat(image);
  Mat rgb = image.clone();  
  cvtColor(image, rgb, CV_YCrCb2BGR);
  cout << "Writing " << fname << endl;
  imwrite(fname, rgb);

}
END_WORD
REGISTER_WORD(SceneSaveObservedMapImage)


/*
WORD(SceneUpdateObservedFromStreamBufferAtZWithRecastThroughDepthStack)
virtual void execute(std::shared_ptr<MachineState> ms) {
// updates entire stack from bottom up
}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZWithRecastThroughDepthStack)

WORD(SceneTrimDepthWithDiscrepancy)
virtual void execute(std::shared_ptr<MachineState> ms) {
}
END_WORD
REGISTER_WORD(SceneTrimDepthWithDiscrepancy)

WORD(SceneUpdateObservedFromStreamBufferRecast)
virtual void execute(std::shared_ptr<MachineState> ms) {
// XXX this seems to require determining when a pixel in the stream buffer
//  hits a cell known to be at a higher height. so this should loop height top down and for each height
//  throw out pixels for the remaining heights below if they project to a cell at that height, and accumulate 
//  at that height otherwise


}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferRecast)
*/



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



WORD(Scene)
virtual void execute(std::shared_ptr<MachineState> ms) {
}
END_WORD
REGISTER_WORD(Scene)

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




