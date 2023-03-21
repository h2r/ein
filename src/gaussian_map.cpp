
#include <dirent.h>
#include <sys/stat.h>

#include "gaussian_map.h"
#include "ein_words.h"
#include "ein.h"
#include "qtgui/einwindow.h"
#include "qtgui/gaussianmapwindow.h"
#include "qtgui/discrepancywindow.h"
#include "camera.h"
#include <boost/filesystem.hpp>

using namespace boost::filesystem;


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
shared_ptr<Scene> Scene::createEmptyScene(MachineState * ms) {
  return make_shared<Scene>(ms, 1, 1, 0.02, eePose::identity());
}

shared_ptr<Scene> Scene::createFromFile(MachineState * ms, string filename) {
  shared_ptr<Scene> scene = createEmptyScene(ms);
  scene->loadFromFile(filename);
  return scene;
}

shared_ptr<GaussianMap> GaussianMap::createEmptyMap(MachineState * ms) {
  return make_shared<GaussianMap>(ms, 1, 1, 0.02, eePose::identity());
}

shared_ptr<GaussianMap> GaussianMap::createFromFile(MachineState * ms, string filename) {
  shared_ptr<GaussianMap> map = createEmptyMap(ms);
  map->loadFromFile(filename);
  return map;
}

void _GaussianMapCell::zero() {
  red.zero();
  green.zero();
  blue.zero();
  z.zero();
}

void GaussianMapCell::recalculateMusAndSigmas(MachineState * ms) {
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


double computeLogLikelihood(MachineState * ms, GaussianMapChannel & channel1, GaussianMapChannel & channel2) {
  /*double total = 0.0;
  for (int i = 0; i < 256; i++) {
    total += exp(computeEnergy(channel1, i));
  }
  double likelihood = computeEnergy(channel1, channel2);

  return likelihood - log(total);*/
  if (channel2.samples < ms->config.sceneCellCountThreshold) {
    //cout << "Returning small likelihood: " << -10e100 << endl;
    return -10e100;
  } else {
    double safesigmasquared1 = safeSigmaSquared(channel1.sigmasquared);
    double energy = 1.0 / (2.0 * safesigmasquared1) * pow(channel2.mu - channel1.mu, 2);
    double normalized = -0.5 * log(2 * M_PI) - 0.5 * log(safesigmasquared1) - energy;
    return normalized;
  }
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
  // not correct to scale variance... recalc must take care of it.
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

double _GaussianMapCell::noisyOrDiscrepancy(_GaussianMapCell * other, double * rterm_out, double * gterm_out, double * bterm_out) {
  double rlikelihood, glikelihood, blikelihood;
  computePointDiscrepancy(red, other->red, &rlikelihood, rterm_out);
  computePointDiscrepancy(green, other->green, &glikelihood, gterm_out);
  computePointDiscrepancy(blue, other->blue, &blikelihood, bterm_out);
  int debug;
  if ((1 - *bterm_out) * 255 > 200  && (1 - *gterm_out) * 255 < 20 && (1 - *rterm_out) * 255 < 20) {
    debug = 0;
  } else {
    debug = 0;
  }

  if (debug) {
    cout << "******** computing noisy or for " << red.mu << ", " << green.mu << ", " << blue.mu << endl;
    cout << "Channel discrepancy probability: " << *rterm_out << ", " << *gterm_out << ", " << *bterm_out << endl;
  }

  double prd = 1 - *rterm_out;
  double pgd = 1 - *gterm_out;
  double pbd = 1 - *bterm_out;

  double sum = 0.0;
  for (int rd = 0; rd <= 1; rd ++) {
    for (int gd = 0; gd <= 1; gd ++) {
      for (int bd = 0; bd <= 1; bd ++) {
        double normalizer = ((rd ? prd : (1 - prd)) *
                             (gd ? pgd : (1 - pgd)) *
                             (bd ? pbd : (1 - pbd)));
        double noisyOr = 1 - ((rd ? (1 - prd) : 1) *
                              (gd ? (1 - pgd) : 1) *
                              (bd ? (1 - pbd) : 1));
        if (debug) {
          cout << "rd: " << rd << ", gd: " << gd << ", bd: " << bd << " ";
          cout << " normalizer: " << setw(20) << normalizer << " ";
          cout << " noisyOr: " << setw(20) << noisyOr << endl;
        }
        sum += normalizer * noisyOr;
      }
    }
  }
  if (debug) {
    cout << "sum: " << sum << endl;
    cout  << endl;
  }
  return 1.0 - sum;
}

double _GaussianMapCell::noisyAndDiscrepancy(_GaussianMapCell * other, double * rterm_out, double * gterm_out, double * bterm_out) {
  double rlikelihood, glikelihood, blikelihood;
  computePointDiscrepancy(red, other->red, &rlikelihood, rterm_out);
  computePointDiscrepancy(green, other->green, &glikelihood, gterm_out);
  computePointDiscrepancy(blue, other->blue, &blikelihood, bterm_out);
  int debug;
  if ((1 - *bterm_out) * 255 > 200  && (1 - *gterm_out) * 255 < 20 && (1 - *rterm_out) * 255 < 20) {
    debug = 0;
  } else {
    debug = 0;
  }

  if (debug) {
    cout << "******** computing noisy and for " << red.mu << ", " << green.mu << ", " << blue.mu << endl;
    cout << "Channel discrepancy probability: " << *rterm_out << ", " << *gterm_out << ", " << *bterm_out << endl;
  }

  // use demorgan for noisy and
  // 1 - 
  double _prd = *rterm_out;
  double _pgd = *gterm_out;
  double _pbd = *bterm_out;


  double sum = 0.0;
  for (int rd = 0; rd <= 1; rd ++) {
    for (int gd = 0; gd <= 1; gd ++) {
      for (int bd = 0; bd <= 1; bd ++) {
        double normalizer = ((rd ? _prd : (1 - _prd)) *
                             (gd ? _pgd : (1 - _pgd)) *
                             (bd ? _pbd : (1 - _pbd)));
        double noisyAnd = 1 - ((rd ? (1 - _prd) : 1) *
                              (gd ? (1 - _pgd) : 1) *
                              (bd ? (1 - _pbd) : 1));
        if (debug) {
          cout << "rd: " << rd << ", gd: " << gd << ", bd: " << bd << " ";
          cout << " normalizer: " << setw(20) << normalizer << " ";
          cout << " noisyAnd: " << setw(20) << noisyAnd << endl;
        }
        sum += normalizer * noisyAnd;
      }
    }
  }
  if (debug) {
    cout << "sum: " << sum << endl;
    cout  << endl;
  }
  // 1 - 
  return sum;
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
  red.samples      =  (double)(it)["rsamples"];
  green.samples      =  (double)(it)["gsamples"];            
  blue.samples      =  (double)(it)["bsamples"];            
  z.counts         =  (double)(it)["zcounts"];         
  z.squaredcounts  =  (double)(it)["zsquaredcounts"];                
  z.mu             =  (double)(it)["zmu"];     
  z.sigmasquared   =  (double)(it)["zsigmasquared"];               
  z.samples        =  (double)(it)["zsamples"];          
}

void _GaussianMapCell::newObservation(uchar r, uchar g, uchar b) {
  red.counts += r;
  green.counts += g;
  blue.counts += b;
  red.squaredcounts += r * r;
  green.squaredcounts += g * g;
  blue.squaredcounts += b * b;
  red.samples += 1;
  green.samples += 1;
  blue.samples += 1;

}

void _GaussianMapCell::newObservation(uchar r, uchar g, uchar b, double zobs) {
  red.counts += r;
  green.counts += g;
  blue.counts += b;
  red.squaredcounts += r * r;
  green.squaredcounts += g * g;
  blue.squaredcounts += b * b;
  red.samples += 1;
  green.samples += 1;
  blue.samples += 1;
  z.counts += zobs;
  z.squaredcounts += zobs * zobs;
  z.samples += 1;
}

void _GaussianMapCell::newObservation(const Vec3b & obs) {
  newObservation(obs[2], obs[1], obs[0]);

}

void _GaussianMapCell::newObservation(const Vec3b & obs, double zobs) {
  newObservation(obs);
  z.counts += zobs;
  z.squaredcounts += zobs * zobs;
  z.samples += 1;
}

void _GaussianMapCell::newObservation(const Vec3d & obs) {
  red.counts += obs[2];
  green.counts += obs[1];
  blue.counts += obs[0];
  red.squaredcounts += obs[2] * obs[2];
  green.squaredcounts += obs[1] * obs[1];
  blue.squaredcounts += obs[0] * obs[0];
  red.samples += 1;
  green.samples += 1;
  blue.samples += 1;
}

void _GaussianMapCell::newObservation(const Vec3d & obs, double zobs) {
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

GaussianMap::GaussianMap(MachineState * ims, int w, int h, double cw, eePose pose) {
  ms = ims;
  width = w;
  height = h;
  x_center_cell = (width-1)/2;
  y_center_cell = (height-1)/2;
  cell_width = cw;
  cells = NULL;
  anchor_pose = pose;
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
    // XXX CONSOLE_ERROR(ms, "GaussianMapCell::refAtCell: Bad x. " << x << " width: " << width);
    return NULL;
  }
  if (y < 0 || y >= height) {
    // XXX CONSOLE_ERROR(ms, "GaussianMapCell::refAtCell: Bad y. " << y << " height: " << height);
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
  //if (*xc < 0 || *xc >= width) {
  //CONSOLE_ERROR(ms, "GaussianMapCell::metersToCell: Bad x. xc: " << *xc << " yc: " << *yc << " input xm: " << xm << " ym: " << ym);
  //assert(0);
  //}
  //if (*yc < 0 || *yc >= height) {
  //CONSOLE_ERROR(ms, "GaussianMapCell::metersToCell: Bad y. xc: " << *xc << " yc: " << *yc << " input xm: " << xm << " ym: " << ym);
  //assert(0);
  //}
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

    fsvO << "anchor_pose";
    anchor_pose.writeToFileStorage(fsvO);
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

    FileNode bg_pose_node = (it)["anchor_pose"];
    anchor_pose.readFromFileNode(bg_pose_node);
  }

  reallocate();
  {
    FileNode bnode = it["cells"];

    string stringdata = readBinaryFromYaml(bnode);
    GaussianMapCell * data = (GaussianMapCell * ) stringdata.data();
    int numLoadedCells = stringdata.size() / sizeof(GaussianMapCell);
    if (stringdata.size() != width * height * sizeof(GaussianMapCell)) {
      CONSOLE_ERROR(ms, "Inconsistency in saved data.");
      CONSOLE_ERROR(ms, "Read width: " << width << " height: " << height << " but got " << stringdata.size() << " from the file.");
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
      CONSOLE_ERROR(ms, "Error, GaussianMap loaded " << numLoadedCells << " but expected " << width*height << endl);
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
    CONSOLE_ERROR(ms, "Could not open file " << filename);
  }
}

CONFIG_GETTER_INT(SceneNumPredictedObjects, ms->config.scene->predicted_objects.size());
CONFIG_GETTER_DOUBLE(SceneMinSigmaSquared, ms->config.sceneMinSigmaSquared, "Variance on scene")
CONFIG_SETTER_DOUBLE(SceneSetMinSigmaSquared, ms->config.sceneMinSigmaSquared)

void GaussianMapChannel::recalculateMusAndSigmas(MachineState * ms) {
  double safe_samples = max(samples, 1.0);
  mu = counts / safe_samples;			
  sigmasquared = (squaredcounts / safe_samples) - (mu * mu); 
  if (sigmasquared < ms->config.sceneMinSigmaSquared) {
    sigmasquared = ms->config.sceneMinSigmaSquared;
  }
}
 
void GaussianMap::recalculateMusAndSigmas(MachineState * ms) {
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
  Mat big = Mat(width, height, CV_8UC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {

      if (refAtCell(x, y)->red.samples > 0) {
	big.at<Vec3b>(x,y)[0] = uchar(std::min(std::max(0.0, refAtCell(x,y)->blue.mu ), 255.0));
	big.at<Vec3b>(x,y)[1] = uchar(std::min(std::max(0.0, refAtCell(x,y)->green.mu), 255.0));
	big.at<Vec3b>(x,y)[2] = uchar(std::min(std::max(0.0, refAtCell(x,y)->red.mu  ), 255.0));
      } else {
	big.at<Vec3b>(x,y)[0] = 0;
	big.at<Vec3b>(x,y)[1] = 128;
	big.at<Vec3b>(x,y)[2] = 128;
      }
    }
  }

  //cv::resize(big, out, cv::Size(301, 301), 2, 2);
  out = big;
}

void GaussianMap::rgbMuToBgrMat(Mat& out) {
  Mat newImage;
  rgbMuToMat(newImage);
  out = newImage.clone();  
  cvtColor(newImage, out, cv::COLOR_YCrCb2BGR);
}

void GaussianMap::rgbDiscrepancyMuToMat(MachineState * ms, Mat& out) {
  out = Mat(width, height, CV_8UC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<Vec3b>(x,y)[0] = uchar(refAtCell(x,y)->blue.mu * 255);
      out.at<Vec3b>(x,y)[1] = uchar(refAtCell(x,y)->green.mu * 255);
      out.at<Vec3b>(x,y)[2] = uchar(refAtCell(x,y)->red.mu * 255); 
    }
  }
}


void GaussianMap::rgbSigmaSquaredToMat(Mat& out) {
  out = Mat(width, height, CV_64FC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<Vec3d>(x,y)[0] = refAtCell(x,y)->blue.sigmasquared;
      out.at<Vec3d>(x,y)[1] = refAtCell(x,y)->green.sigmasquared;
      out.at<Vec3d>(x,y)[2] = refAtCell(x,y)->red.sigmasquared;
    }
  }
}

void GaussianMap::rgbSigmaToMat(Mat& out) {
  Mat big = Mat(width, height, CV_8UC3);
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
      big.at<Vec3b>(x,y)[0] = uchar(sqrt(refAtCell(x,y)->blue.sigmasquared) * 4);
      big.at<Vec3b>(x,y)[1] = uchar(sqrt(refAtCell(x,y)->green.sigmasquared) * 4);
      big.at<Vec3b>(x,y)[2] = uchar(sqrt(refAtCell(x,y)->red.sigmasquared) * 4);
    }
  }
  //cout << "max: " << max_val << endl;
  //cout << "min: " << min_val << endl;
  out = big;
}

void GaussianMap::rgbCountsToMat(Mat& out) {
  out = Mat(width, height, CV_64FC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<Vec3d>(x,y)[0] = refAtCell(x,y)->blue.counts;
      out.at<Vec3d>(x,y)[1] = refAtCell(x,y)->green.counts;
      out.at<Vec3d>(x,y)[2] = refAtCell(x,y)->red.counts;
    }
  }
}

void GaussianMap::rgbSquaredCountsToMat(Mat& out) {
  out = Mat(width, height, CV_64FC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<Vec3d>(x,y)[0] = refAtCell(x,y)->blue.squaredcounts;
      out.at<Vec3d>(x,y)[1] = refAtCell(x,y)->green.squaredcounts;
      out.at<Vec3d>(x,y)[2] = refAtCell(x,y)->red.squaredcounts;
    }
  }
}

void GaussianMap::zMuToMat(Mat& out) {
  out = Mat(width, height, CV_64F);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<double>(x,y) = refAtCell(x,y)->z.mu;
    }
  }
}
void GaussianMap::zMuToScaledMat(Mat& out) {
  Mat toShow;
  zMuToMat(toShow);

  double positiveMin = DBL_MAX;
  double positiveMax = 0.0;
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      if ( (refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold) ) {
	positiveMin = std::min(positiveMin, toShow.at<double>(x,y));
	positiveMax = std::max(positiveMax, toShow.at<double>(x,y));
      } else {
	//toShow.at<double>(x,y) = 0;
      }
    }
  }

  double maxVal, minVal;
  minMaxLoc(toShow, &minVal, &maxVal);
  //minVal = max(minVal, ms->config.currentEEPose.pz+ms->config.currentTableZ);
  //double denom = max(1e-6, maxVal-minVal);
  double denom = max(1e-6, positiveMax-positiveMin);
  cout << "sceneRenderZ min max denom positiveMin positiveMax: " << minVal << " " << maxVal << " " << denom << " " << positiveMin << " " << positiveMax << endl;
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      if ( (refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold) ) {
	//toShow.at<double>(x,y) = (toShow.at<double>(x,y) - minVal) / denom;
	toShow.at<double>(x,y) = (positiveMax - toShow.at<double>(x,y)) / denom;
      } else {
	//toShow.at<double>(x,y) = 0;
      }
    }
  }
  out = toShow;
}

void GaussianMap::zSigmaSquaredToMat(Mat& out) {
  out = Mat(width, height, CV_64F);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<double>(x,y) = refAtCell(x,y)->z.sigmasquared;
    }
  }
}

void GaussianMap::zCountsToMat(Mat& out) {
  out = Mat(width, height, CV_64F);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      out.at<double>(x,y) = refAtCell(x,y)->z.counts;
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

  shared_ptr<GaussianMap> toReturn = std::make_shared<GaussianMap>(ms, x2-x1+1, y2-y1+1, cell_width, anchor_pose);
  for (int y = y1; y <= y2; y++) {
    for (int x = x1; x <= x2; x++) {
      *(toReturn->refAtCell(x-x1,y-y1)) = *(refAtCell(x,y));
    }
  }
  
  return toReturn;
}

shared_ptr<GaussianMap> GaussianMap::copy() {
  return copyBox(0,0,width-1,height-1);
}

shared_ptr<Scene> Scene::copy() {
  shared_ptr<Scene> toReturn = std::make_shared<Scene>(ms, width, height, cell_width, anchor_pose);
  toReturn->annotated_class_name = annotated_class_name;
  toReturn->predicted_class_name = predicted_class_name;
  toReturn->background_map = background_map->copy();
  toReturn->predicted_map = predicted_map->copy();
  toReturn->observed_map = observed_map->copy();
  toReturn->discrepancy = discrepancy->copy();

  toReturn->predicted_segmentation = predicted_segmentation.clone();
  toReturn->discrepancy_magnitude = discrepancy_magnitude.clone();
  toReturn->discrepancy_density = discrepancy_density.clone();
  toReturn->anchor_pose = anchor_pose;
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




Scene::Scene(MachineState * _ms, int w, int h, double cw, eePose pose) {

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
  //anchor_pose = eePose::identity();
  anchor_pose = pose;
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
  background_map = make_shared<GaussianMap>(ms, width, height, cell_width, anchor_pose);
  predicted_map = make_shared<GaussianMap>(ms, width, height, cell_width, anchor_pose);
  predicted_segmentation = Mat(width, height, CV_64F);
  observed_map = make_shared<GaussianMap>(ms, width, height, cell_width, anchor_pose);
  discrepancy = make_shared<GaussianMap>(ms, width, height, cell_width, anchor_pose);

  discrepancy_magnitude = Mat(width, height, CV_64F);
  discrepancy_density = Mat(width, height, CV_64F);
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
    return (discrepancy_density.at<double>(x,y) > threshold);
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

  return ( (discrepancy_density.at<double>(x1,y1) > threshold) || 
	   (discrepancy_density.at<double>(x1,y2) > threshold) ||
	   (discrepancy_density.at<double>(x2,y1) > threshold) ||
	   (discrepancy_density.at<double>(x2,y2) > threshold) );
} 
bool Scene::isDiscrepantMetersBilin(double threshold, double x, double y) {
  double cell_x;
  double cell_y;
  metersToCell(x, y, &cell_x, &cell_y);
  return isDiscrepantCellBilin(threshold, cell_x, cell_y);
} 



void Scene::initializePredictedMapWithBackground() {

  predicted_map = observed_map->copy();
  predicted_map->zero();

  eePose predicted_anchor = predicted_map->anchor_pose;
  eePose background_anchor = background_map->anchor_pose;
  
  double px0 = 0;
  double py0 = 0;

  double px1 = 1;
  double py1 = 0;

  double px2 = 0;
  double py2 = 1;


  double bx0 = 0;
  double by0 = 0;

  double bx1 = 0;
  double by1 = 0;

  double bx2 = 0;
  double by2 = 0;

  {
    double meters_predicted_x, meters_predicted_y;
    predicted_map->cellToMeters(px0, py0, &meters_predicted_x, &meters_predicted_y);
    eePose toTransform(meters_predicted_x, meters_predicted_y, 0, 0, 0, 0, 1);
    eePose inBase = toTransform.applyAsRelativePoseTo(predicted_anchor);
    eePose inBackground = inBase.getPoseRelativeTo(background_anchor);
    
    double cell_background_x, cell_background_y;
    background_map->metersToCell(inBackground.px, inBackground.py, &cell_background_x, &cell_background_y);
    bx0 = cell_background_x;
    by0 = cell_background_y;
  }
  {
    double meters_predicted_x, meters_predicted_y;
    predicted_map->cellToMeters(px1, py1, &meters_predicted_x, &meters_predicted_y);
    eePose toTransform(meters_predicted_x, meters_predicted_y, 0, 0, 0, 0, 1);
    eePose inBase = toTransform.applyAsRelativePoseTo(predicted_anchor);
    eePose inBackground = inBase.getPoseRelativeTo(background_anchor);
    
    double cell_background_x, cell_background_y;
    background_map->metersToCell(inBackground.px, inBackground.py, &cell_background_x, &cell_background_y);
    bx1 = cell_background_x;
    by1 = cell_background_y;
  }

  {
    double meters_predicted_x, meters_predicted_y;
    predicted_map->cellToMeters(px2, py2, &meters_predicted_x, &meters_predicted_y);
    eePose toTransform(meters_predicted_x, meters_predicted_y, 0, 0, 0, 0, 1);
    eePose inBase = toTransform.applyAsRelativePoseTo(predicted_anchor);
    eePose inBackground = inBase.getPoseRelativeTo(background_anchor);
    
    double cell_background_x, cell_background_y;
    background_map->metersToCell(inBackground.px, inBackground.py, &cell_background_x, &cell_background_y);
    bx2 = cell_background_x;
    by2 = cell_background_y;
  }

  double diffx_x = bx1 - bx0;
  double diffx_y = by1 - by0;
  
  double diffy_x = bx2 - bx0;
  double diffy_y = by2 - by0;

cout << "AAA: diffx_x diffx_y diffy_x diffy_y: " << diffx_x << " " << diffx_y << " " << diffy_x << " " << diffy_y << " " << endl;

  double normx = sqrt(diffx_x * diffx_x + diffx_y * diffx_y);
  double normy = sqrt(diffy_x * diffy_x + diffy_y * diffy_y);

/*
  diffx_x *= predicted_map->cell_width / (background_map->cell_width * normx);
  diffx_y *= predicted_map->cell_width / (background_map->cell_width * normx);
	    
  diffy_x *= predicted_map->cell_width / (background_map->cell_width * normy);
  diffy_y *= predicted_map->cell_width / (background_map->cell_width * normy);
*/

cout << "BBB: diffx_x diffx_y diffy_x diffy_y: " << diffx_x << " " << diffx_y << " " << diffy_x << " " << diffy_y << " " << endl;

  normx = sqrt(diffx_x * diffx_x + diffx_y * diffx_y);
  normy = sqrt(diffy_x * diffy_x + diffy_y * diffy_y);

/*
  diffx_x *=  1.0 / normx;
  diffx_y *=  1.0 / normx;
	    
  diffy_x *=  1.0 / normy;
  diffy_y *=  1.0 / normy;
*/

cout << "CCC: diffx_x diffx_y diffy_x diffy_y: " << diffx_x << " " << diffx_y << " " << diffy_x << " " << diffy_y << " " << endl;
cout << "DDD: normx normy : " << normx << " " << normy << " " << predicted_map->cell_width << " " << background_map->cell_width << endl;
  

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {

      //double meters_predicted_x, meters_predicted_y;
      //predicted_map->cellToMeters(x, y, &meters_predicted_x, &meters_predicted_y);
      //eePose toTransform(meters_predicted_x, meters_predicted_y, 0, 0, 0, 0, 1);
      //eePose inBase = toTransform.applyAsRelativePoseTo(predicted_anchor);
      //eePose inBackground = inBase.getPoseRelativeTo(background_anchor);

      //double cell_background_x, cell_background_y;
      //background_map->metersToCell(inBackground.px, inBackground.py, &cell_background_x, &cell_background_y);


      double cell_background_x = bx0 + diffx_x * x + diffy_x * y;
      double cell_background_y = by0 + diffx_y * x + diffy_y * y;

      //cout << "      exact: " << cell_background_x << "," << cell_background_y << endl; 
      //cout << "estimated x: " << cell_x << "," << cell_y << endl << endl;
      //assert(fabs(cell_x - cell_background_x) < 0.0001);
      //assert(fabs(cell_y - cell_background_y) < 0.0001);

      if ( background_map->safeBilinAt(cell_background_x, cell_background_y) ) {
	*(predicted_map->refAtCell(x,y)) = background_map->bilinValAtCell(cell_background_x, cell_background_y);
	predicted_map->refAtCell(x,y)->recalculateMusAndSigmas(ms);
      }

      //cout << "x y bx by: " << x << " " << y << " " << inBase.px << " " << inBase.py << " " << predicted_anchor << " " << background_anchor << " " << toTransform << inBase << inBackground << endl;
    }
  }
}

void Scene::composePredictedMap() {
  composePredictedMap(ms->config.scene_score_thresh);
}
void Scene::composePredictedMap(double threshold) {
  //REQUIRE_FOCUSED_CLASS(ms, tfc);
  // XXX
  // choose the argMAP distribution
  //   asign that color to the predicted map
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


void Scene::addPredictedObjectsToObservedMap(double threshold, double learning_weight) {
  //REQUIRE_FOCUSED_CLASS(ms, tfc);
  // XXX
  // choose the argMAP distribution
  //   assign that color to the predicted map
  //   assign the source to the segmentation
  //
  // Currently uses a "fallen leaves" model of composition, assuming objects
  //   are painted onto the scene in reverse order of discovery 

 
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
	    GaussianMapCell cell = tos->observed_map->bilinValAtMeters(meters_object_x, meters_object_y);
	    cell.multS(learning_weight);
	    observed_map->refAtCell(x,y)->addC(&cell);
	    observed_map->refAtCell(x,y)->recalculateMusAndSigmas(ms);
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
	score += computeLogLikelihood(ms, predicted_cell->red, observed_cell->red);
	score += computeLogLikelihood(ms, predicted_cell->green, observed_cell->green);
	score += computeLogLikelihood(ms, predicted_cell->blue, observed_cell->blue);
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
	} else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_OR) {
          discrepancy_value = predicted_map->refAtCell(x,y)->noisyOrDiscrepancy(observed_map->refAtCell(x,y), &rmu_diff, &gmu_diff, &bmu_diff);
	} else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_AND) {
          discrepancy_value = predicted_map->refAtCell(x,y)->noisyAndDiscrepancy(observed_map->refAtCell(x,y), &rmu_diff, &gmu_diff, &bmu_diff);
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
	discrepancy_magnitude.at<double>(x,y) = total_discrepancy;

	//identitycheckProb("total_discrepancy", total_discrepancy);
	//identitycheckProb("rmu", discrepancy->refAtCell(x,y)->red.mu);
	//identitycheckProb("bmu", discrepancy->refAtCell(x,y)->blue.mu);
	//identitycheckProb("gmu", discrepancy->refAtCell(x,y)->green.mu);
	//rmu_diff*rmu_diff + gmu_diff*gmu_diff + bmu_diff*bmu_diff ;
	//+ rvar_quot + gvar_quot + bvar_quot;

	//sqrt(discrepancy_magnitude.at<double>(x,y) / 3.0) / 255.0; 

	c_enough++;
	//cout << " enough ";
	//cout << predicted_map->refAtCell(x,y)->red.samples << " " << observed_map->refAtCell(x,y)->red.samples << " ";
      } else {
	discrepancy->refAtCell(x,y)->zero();
	discrepancy_magnitude.at<double>(x,y) = 0.0;
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
  //discrepancy_density = discrepancy_magnitude;
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
	totalScore += discrepancy_magnitude.at<double>(x,y); 
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
	      score += computeLogLikelihood(ms, predicted_cell->red, observed_cell->red);
	      score += computeLogLikelihood(ms, predicted_cell->green, observed_cell->green);
	      score += computeLogLikelihood(ms, predicted_cell->blue, observed_cell->blue);
	    }
*/
	    if ( 
		(predicted_map->refAtCell(x,y)->red.samples > 0) &&   
		//(observed_map->refAtCell(x,y)->red.samples > 0) &&
		(tos->observed_map->refAtCell(cells_object_x,cells_object_y)->red.samples > 0) 
	       ) {
	      GaussianMapCell * observed_cell = observed_map->refAtCell(x, y);
	      GaussianMapCell * predicted_cell = predicted_map->refAtCell(x, y);
	      GaussianMapCell * object_cell = tos->observed_map->refAtCell(cells_object_x, cells_object_y);

	      double temp = 0.0;
//cout << "score " << score << " ";
	      //temp = object_cell->red.sigmasquared;
	      //object_cell->red.sigmasquared = predicted_cell->red.sigmasquared;
	      score += computeLogLikelihood(ms, object_cell->red, observed_cell->red);
	      //object_cell->red.sigmasquared = temp;

	      //temp = object_cell->green.sigmasquared;
	      //object_cell->green.sigmasquared = predicted_cell->green.sigmasquared;
	      score += computeLogLikelihood(ms, object_cell->green, observed_cell->green);
	      //object_cell->green.sigmasquared = temp;

	      //temp = object_cell->blue.sigmasquared;
	      //object_cell->blue.sigmasquared = predicted_cell->blue.sigmasquared;
	      score += computeLogLikelihood(ms, object_cell->blue, observed_cell->blue);
	      //object_cell->blue.sigmasquared = temp;

//cout << "score " << score << " ";
	      // XXX should that check predicted instead?
	      if (observed_cell->red.samples >= ms->config.sceneCellCountThreshold) {
		score -= computeLogLikelihood(ms, predicted_cell->red, observed_cell->red);
		score -= computeLogLikelihood(ms, predicted_cell->green, observed_cell->green);
		score -= computeLogLikelihood(ms, predicted_cell->blue, observed_cell->blue);
	      }
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

  if ((x2 - x1 + 1) % 2 == 0) {
    if (x1 != 0) {
      x1 -= 1;
    } else if (x2 != width - 1)  {
      x2 += 1;
    } else {
      cout << "Invalid parity correction: x1: " << x1 << " x2: " << x2 << " width: " << width << endl;
      assert(0);
    }
  }

  if ((y2 - y1 + 1) % 2 == 0) {
    if (y1 != 0) {
      y1 -= 1;
    } else if (y2 != height - 1)  {
      y2 += 1;
    } else {
      cout << "Invalid parity correction: y1: " << y1 << " y2: " << y2 << " height: " << height << endl;
      assert(0);
    }
  }



  shared_ptr<Scene> toReturn = std::make_shared<Scene>(ms, x2-x1+1, y2-y1+1, cell_width, anchor_pose);


  toReturn->background_map = background_map->copyBox(x1,y1,x2,y2);
  toReturn->predicted_map = predicted_map->copyBox(x1,y1,x2,y2);
  toReturn->observed_map = observed_map->copyBox(x1,y1,x2,y2);
  toReturn->discrepancy= discrepancy->copyBox(x1,y1,x2,y2);

  toReturn->discrepancy_magnitude = discrepancy_magnitude(cv::Range(x1, x1 + toReturn->width), cv::Range(y1, y1 + toReturn->height)).clone();
  toReturn->discrepancy_density = discrepancy_density(cv::Range(x1, x1 + toReturn->width), cv::Range(y1, y1 + toReturn->height)).clone();
  toReturn->predicted_segmentation = predicted_segmentation(cv::Range(x1, x1 + toReturn->width), cv::Range(y1, y1 + toReturn->height)).clone();
  //toReturn->discrepancy_magnitude = discrepancy_magnitude(cv::Range(y1, y2), cv::Range(x1, x2)).clone();
  //toReturn->discrepancy_density = discrepancy_density(cv::Range(y1, y2), cv::Range(x1, x2)).clone();
  //toReturn->predicted_segmentation = predicted_segmentation(cv::Range(y1, y2), cv::Range(x1, x2)).clone();



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
      if (discrepancy_density.at<double>(x,y) != 0) {
	//cout << discrepancy_density.at<double>(x,y) << endl;
      }

      if ((predicted_map->refAtCell(x,y)->red.samples > 0) && (discrepancy_density.at<double>(x,y) > threshold)) {
	xmin = min(xmin, x);
	xmax = max(xmax, x);
	ymin = min(ymin, y);
	ymax = max(ymax, y);
      } else {
      }
    }
  }
  //cout << xmin << " " << xmax << " " << ymin << " " << ymax << " secondary " << endl;

  xmin = xmin - ceil(pad_meters/cell_width);
  xmax = xmax + ceil(pad_meters/cell_width);
  ymin = ymin - ceil(pad_meters/cell_width);
  ymax = ymax + ceil(pad_meters/cell_width);

  int new_width = xmax - xmin + 1;
  int new_height = ymax - ymin + 1;
  //cout << " new_width, new_height: " << new_width << " " << new_height << endl;
  // make sure the crop is odd dimensioned
  if ((new_width % 2) == 0) {
    xmax = xmax-1;
  }
  if ((new_height % 2) == 0) {
    ymax = ymax-1;
  }
  new_width = xmax - xmin + 1;
  new_height = ymax - ymin + 1;

  //cout << xmin << " " << xmax << " " << ymin << " " << ymax << " third " << endl;
  //cout << " new_width, new_height: " << new_width << " " << new_height << endl;

  shared_ptr<Scene> scene_to_return;
  if ((xmin <= xmax) && (ymin <= ymax)) {
    scene_to_return = copyBox(xmin,ymin,xmax,ymax);
  } else {
    CONSOLE_ERROR(ms, "region contained no discrepant cells, grabbing one cell." << endl);
    cout << xmin << " " << xmax << " " << ymin << " " << ymax << endl;
    xmax = min(width-1, 2);
    ymax = min(height-1, 2);
    xmin = 0;
    ymin = 0;
    scene_to_return = copyBox(xmin,ymin,xmax,ymax);
  }
  cout << "new object dimensions: " << (xmax - xmin) << "x" << (ymax - ymin) << endl;
  
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
      if ((predicted_map->refAtCell(x,y)->red.samples > 0) && (discrepancy_density.at<double>(x,y) > threshold)) {
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

CONFIG_GETTER_DOUBLE(SceneCellWidth, ms->config.scene->cell_width, "Cell width of scene.")
CONFIG_GETTER_DOUBLE(SceneScoreThresh, ms->config.scene_score_thresh, "Score threshold for discrepancy.")
CONFIG_SETTER_DOUBLE(SceneSetScoreThresh, ms->config.scene_score_thresh)




#include <boost/multiprecision/cpp_dec_float.hpp>

typedef boost::multiprecision::number<boost::multiprecision::cpp_dec_float<2000> > doubleWithLotsOfDigits;


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
  fsvO << "anchor_pose";
  anchor_pose.writeToFileStorage(fsvO);

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
  
  FileNode bg_pose_node = (it)["anchor_pose"];
  anchor_pose.readFromFileNode(bg_pose_node);
  
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
    CONSOLE_ERROR(ms, "Discrepancy density has inconsistent dimensions.  Scene: " << width << "x" << height << ".  Mat: " << discrepancy_density.rows << "x" << discrepancy_density.cols);
    discrepancy_density = Mat(width, height, CV_64F);
    error = true;
  }

  if (discrepancy_magnitude.rows != width || discrepancy_magnitude.cols != height) {
    CONSOLE_ERROR(ms, "Discrepancy magnitude has inconsistent dimensions.  Scene: " << width << "x" << height << ".  Mat: " << discrepancy_magnitude.rows << "x" << discrepancy_magnitude.cols);
    discrepancy_magnitude = Mat(width, height, CV_64F);
    error = true;
  }

  if (predicted_segmentation.rows != width || predicted_segmentation.cols != height) {
    CONSOLE_ERROR(ms, "Predicted segmentation has inconsistent dimensions.  Scene: " << width << "x" << height << ".  Mat: " << predicted_segmentation.rows << "x" << predicted_segmentation.cols);
    predicted_segmentation = Mat(width, height, CV_64F);
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
    CONSOLE_ERROR(ms, "Could not open file " << filename);
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



WORD(SceneComputeScore)
virtual void execute(MachineState * ms) {
  double score = ms->config.scene->computeScore();
  std::shared_ptr<DoubleWord> newWord = std::make_shared<DoubleWord>(score);
  ms->pushWord(newWord);

}
END_WORD
REGISTER_WORD(SceneComputeScore)


WORD(SceneSaveSceneAbsolute)
virtual void execute(MachineState * ms) {
  string dir;
  GET_STRING_ARG(ms, dir);
  ms->config.scene->saveToFile(dir);
  // XX check return code
}
END_WORD
REGISTER_WORD(SceneSaveSceneAbsolute)



WORD(SceneSaveScene)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("sceneRenderObservedMap sceneRenderBackgroundMap sceneRenderPredictedMap sceneRenderDiscrepancy streamRenderStreamWindow");
}
END_WORD
REGISTER_WORD(SceneRenderScene)

WORD(SceneLoadObjectModel)
virtual string description() {
  return "Load the object model.  Takes an object name as argument, which must be a directory name in the objects directory.";
}

virtual void execute(MachineState * ms) {
  string object_name;
  GET_STRING_ARG(ms, object_name);
  
  std::stringstream buffer;
  buffer << "\"" << sceneModelFile(ms, object_name) << "\" sceneLoadSceneRaw";
  ms->evaluateProgram(buffer.str());
}
END_WORD
REGISTER_WORD(SceneLoadObjectModel)



WORD(SceneLoadFocusedObjectModel)
virtual string description() {
  return "Load the object model of the focused class from disk.";
}
virtual void execute(MachineState * ms) {
  std::stringstream buffer;
  buffer << "\"" << sceneModelFile(ms, ms->config.focusedClassLabel) << "\" sceneLoadSceneRaw";
  ms->evaluateProgram(buffer.str());
}
END_WORD
REGISTER_WORD(SceneLoadFocusedObjectModel)

WORD(SceneLoadSceneRaw)
virtual void execute(MachineState * ms) {
  string file;
  GET_STRING_ARG(ms, file);
  ms->config.scene->loadFromFile(file);
  ms->pushWord("sceneRenderScene");
}
END_WORD
REGISTER_WORD(SceneLoadSceneRaw)



WORD(SceneLoadScene)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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

WORD(SceneInitDefaultBackgroundMap)

virtual string description() {
  return "Check if the map exists; otherwise create a default one.";
}
virtual void execute(MachineState * ms) {
  string message;
  GET_STRING_ARG(ms, message);


  stringstream ss;
  ss << ms->config.data_directory + "/maps/" + message + ".yml";
  stringstream ss_dir;
  ss_dir << ms->config.data_directory + "/maps/";

  mkdir(ss_dir.str().c_str(), 0777);

  if (! exists(ss.str())) {
    ms->config.scene->background_map->saveToFile(ss.str());
  }

  ms->config.scene->background_map->loadFromFile(ss.str());

  ms->pushWord("sceneRenderBackgroundMap");
}
END_WORD
REGISTER_WORD(SceneInitDefaultBackgroundMap)



WORD(SceneRenderBackgroundMap)
virtual void execute(MachineState * ms) {
  if (!ms->config.showgui) {
    return;
  }      
  Mat backgroundImage;
  ms->config.scene->background_map->rgbMuToBgrMat(backgroundImage);
  ms->config.backgroundWindow->updateImage(backgroundImage);
  ms->config.backgroundMapWindow->updateMap(ms->config.scene->background_map);
}
END_WORD
REGISTER_WORD(SceneRenderBackgroundMap)


WORD(SceneSaveObservedMap)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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




bool doubleDescending(const double &i, const double &j) {
  return (i > j);
}
bool doubleAscending(const double &i, const double &j) {
  return (i < j);
}

WORD(SceneInit)
virtual void execute(MachineState * ms) {
  double p_cell_width = 0.0025;//0.00175; //0.0025; //0.01;
  cout << "Initializing scene with " << ms->config.sceneInitWidth << endl;
  int p_width = ms->config.sceneInitWidth; //1001; //1501; // 1001 // 601;
  int p_height = ms->config.sceneInitHeight; //1001; //1501; // 1001 / 601;
  eePose scenePose = eePose::identity();
  scenePose.pz = -ms->config.currentTableZ;
  ms->config.scene = make_shared<Scene>(ms, p_width, p_height, p_cell_width, scenePose);
  ms->pushWord("sceneRenderScene");
}
END_WORD
REGISTER_WORD(SceneInit)

WORD(SceneInitFromEePose)
virtual void execute(MachineState * ms) {
  eePose destPose;
  GET_ARG(ms,EePoseWord,destPose);
  // XXX THIS ONE IS FINER GRAINED since it is centerered
  double p_cell_width = 0.001;//0.0025;//0.00175; //0.0025; //0.01;
  int p_width = 1001; //1501; // 1001 // 601;
  int p_height = 1001; //1501; // 1001 / 601;
  //eePose flipZ = {.px = 0.0, .py = 0.0, .pz = 0.0,
  //.qx = 0.0, .qy = 1.0, .qz = 1.0, .qw = 0.0}; 

  destPose = destPose.multQ(ms->config.straightDown.invQ());
  ms->config.scene = make_shared<Scene>(ms, p_width, p_height, p_cell_width, destPose);
  cout << "sceneInitFromEePose: destPose is " << destPose << endl;
  ms->pushWord("sceneRenderScene");
}
END_WORD
REGISTER_WORD(SceneInitFromEePose)

WORD(SceneInitFromEePoseScale)
virtual void execute(MachineState * ms) {
  eePose destPose;
  GET_ARG(ms,EePoseWord,destPose);
  double gridSize = 0.0;
  GET_NUMERIC_ARG(ms, gridSize);

  // XXX THIS ONE IS FINER GRAINED since it is centerered
  double p_cell_width = gridSize;//0.0025;//0.00175; //0.0025; //0.01;
  int p_width = 1001; //1501; // 1001 // 601;
  int p_height = 1001; //1501; // 1001 / 601;

  destPose = destPose.multQ(ms->config.straightDown.invQ());
  ms->config.scene = make_shared<Scene>(ms, p_width, p_height, p_cell_width, destPose);
  cout << "sceneInitFromEePoseScale: destPose is " << destPose << endl;
  ms->pushWord("sceneRenderScene");
}
END_WORD
REGISTER_WORD(SceneInitFromEePoseScale)

WORD(SceneInitFromEePoseScaleDimensions)
virtual void execute(MachineState * ms) {
  eePose destPose;
  GET_ARG(ms,EePoseWord,destPose);
  double gridSize = 0.0;
  int p_width = 1001; 
  int p_height = 1001; 
  
  GET_INT_ARG(ms, p_height);
  GET_INT_ARG(ms, p_width);
  GET_NUMERIC_ARG(ms, gridSize);

  if (p_width <= 0) {
    CONSOLE_ERROR(ms, "p_width must be positive.  Received: " << p_width);
    ms->pushWord("pauseStackExecution");
    return;
  }
  if (p_height <= 0) {
    CONSOLE_ERROR(ms, "p_height must be positive.  Received: " << p_height);
    ms->pushWord("pauseStackExecution");
    return;
  }

  double p_cell_width = gridSize;

  destPose = destPose.multQ(ms->config.straightDown.invQ());
  ms->config.scene = make_shared<Scene>(ms, p_width, p_height, p_cell_width, destPose);
  cout << "sceneInitFromEePoseScaleDimensions: destPose is " << destPose << endl;
  ms->pushWord("sceneRenderScene");
}
END_WORD
REGISTER_WORD(SceneInitFromEePoseScaleDimensions)

WORD(SceneInitDimensions)
virtual void execute(MachineState * ms) {
  double p_cell_width = 0.0025;//0.00175; //0.0025; //0.01;
  int p_width = 1001; //1501; // 1001 // 601;
  int p_height = 1001; //1501; // 1001 / 601;
  
  GET_INT_ARG(ms, p_height);
  GET_INT_ARG(ms, p_width);
  GET_NUMERIC_ARG(ms, p_cell_width);

  cout << "sceneInitDimensions cell_width width height: " << p_cell_width << " " << p_width << " " << p_height << endl;

  ms->config.scene = make_shared<Scene>(ms, p_width, p_height, p_cell_width, eePose::identity());
  ms->config.scene->background_map = ms->config.scene->observed_map->copy();
  ms->pushWord("sceneRenderScene");
}
END_WORD
REGISTER_WORD(SceneInitDimensions)


WORD(SceneInitSmall)
virtual void execute(MachineState * ms) {
  double p_cell_width = 0.0025; //0.01;
  int p_width = 50; // 601;
  int p_height = 50; // 601;
  ms->config.scene = make_shared<Scene>(ms, p_width, p_height, p_cell_width, eePose::identity());
  ms->pushWord("sceneRenderScene");
}
END_WORD
REGISTER_WORD(SceneInitSmall)


WORD(SceneClearObservedMap)
virtual void execute(MachineState * ms) {
  ms->config.scene->observed_map->zero();
  Mat observedImage;
  ms->config.scene->observed_map->rgbMuToMat(observedImage);
  //observedImage = observedImage / 255.0;
  Mat rgb = observedImage.clone();  
  cvtColor(observedImage, rgb, cv::COLOR_YCrCb2BGR);
  if (ms->config.showgui) {
    ms->config.observedWindow->updateImage(rgb);
  }
}
END_WORD
REGISTER_WORD(SceneClearObservedMap)

WORD(SceneSetBackgroundFromObserved)
virtual void execute(MachineState * ms) {
  ms->config.scene->background_map = ms->config.scene->observed_map->copy();
}
END_WORD
REGISTER_WORD(SceneSetBackgroundFromObserved)

WORD(SceneSetLightingModelStdDevY)
virtual void execute(MachineState * ms) {
  double stddev = 0;
  GET_NUMERIC_ARG(ms, stddev);

  ms->config.scene->light_model.blue.sigmasquared = pow(stddev, 2);
}
END_WORD
REGISTER_WORD(SceneSetLightingModelStdDevY)

WORD(SceneSetLightingModelStdDevColor)
virtual void execute(MachineState * ms) {
  double stddev = 0;
  GET_NUMERIC_ARG(ms, stddev);

  ms->config.scene->light_model.red.sigmasquared = pow(stddev, 2);
  ms->config.scene->light_model.green.sigmasquared = pow(stddev, 2);
}
END_WORD
REGISTER_WORD(SceneSetLightingModelStdDevColor)

WORD(SceneSetBackgroundStdDevY)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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

WORD(SceneSetPredictedStdDevY)
virtual void execute(MachineState * ms) {
  double stddev = 0;
  GET_NUMERIC_ARG(ms, stddev);

  for (int y = 0; y < ms->config.scene->predicted_map->height; y++) {
    for (int x = 0; x < ms->config.scene->predicted_map->width; x++) {
      ms->config.scene->predicted_map->refAtCell(x,y)->blue.sigmasquared = pow(stddev, 2);
    }
  }
}
END_WORD
REGISTER_WORD(SceneSetPredictedStdDevY)

WORD(SceneSetPredictedStdDevColor)
virtual void execute(MachineState * ms) {
  double stddev = 0;
  GET_NUMERIC_ARG(ms, stddev);

  for (int y = 0; y < ms->config.scene->predicted_map->height; y++) {
    for (int x = 0; x < ms->config.scene->predicted_map->width; x++) {
      ms->config.scene->predicted_map->refAtCell(x,y)->red.sigmasquared = pow(stddev, 2);
      ms->config.scene->predicted_map->refAtCell(x,y)->green.sigmasquared = pow(stddev, 2);
    }
  }
}
END_WORD
REGISTER_WORD(SceneSetPredictedStdDevColor)



WORD(SceneUpdateObservedFromWrist)
virtual void execute(MachineState * ms) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  Size sz = camera->cam_ycrcb_img.size();
  int imW = sz.width;
  int imH = sz.height;

  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 
    
  //for (int px = ; px < ; px++) 
    //for (int py = ; py < ; py++) 
  pixelToGlobalCache data;
  double z = ms->config.trueEEPoseEEPose.pz + ms->config.currentTableZ;
  //computePixelToGlobalCache(ms, z, thisPose, &data);

  eePose thisPose, basePose;
  int success = ms->getStreamPoseAtTime(camera->lastImageStamp.seconds(), &thisPose, &basePose);

  computePixelToPlaneCache(ms, z, thisPose, ms->config.scene->anchor_pose, &data);
  //  imshow("image being used", camera->cam_ycrcb_img);
  //waitKey(0);
  cout << "top: " << topx << ", " << topy << endl;
  cout << "bot: " << botx << ", " << boty << endl;

  for (int px = topx; px < botx; px++) {
    for (int py = topy; py < boty; py++) {
  //for (int px = 0; px < imW; px++) 
    //for (int py = 0; py < imH; py++) 
      if (isInGripperMask(ms, px, py)) {
	continue;
      }
      double x, y;
      pixelToGlobalFromCache(ms, px, py, &x, &y, &data);

      if (std::isnan(x) || std::isnan(y)) {
        CONSOLE_ERROR(ms, "Nan in pixel to global; check your calibration! " << x << ", " << y << " from " << px << ", " << py);
        CONSOLE_ERROR(ms, "Pixel to global cache: " << pixelToGlobalCacheToString(data));
        return;
      }


      // single sample update
      int i, j;
      ms->config.scene->observed_map->metersToCell(x, y, &i, &j);
      GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(i, j);
      if (cell != NULL) {
        Vec3b pixel = camera->cam_ycrcb_img.at<Vec3b>(py, px);
        cell->newObservation(pixel);
      }
    }
  }
  ms->config.scene->observed_map->recalculateMusAndSigmas(ms);
  ms->pushWord("sceneRenderObservedMap");
}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromWrist)

// XXX TODO NOT DONE
WORD(SceneUpdateObservedFromStreamBufferNoRecalc)
virtual void execute(MachineState * ms) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  int thisIdx = camera->sibCurIdx;
  //cout << "sceneUpdateObservedFromStreamBuffer: " << thisIdx << endl;

  Mat bufferImage;
  eePose thisPose, tBaseP;
  

  int success = 1;
  if ( (thisIdx > -1) && (thisIdx < camera->streamImageBuffer.size()) ) {
    streamImage &tsi = camera->streamImageBuffer[thisIdx];
    loadStreamImage(ms, &tsi);
    bufferImage = tsi.image.clone();

    if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
      success = ms->getStreamPoseAtTime(tsi.time, &thisPose, &tBaseP);
    } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
      success = 1;
      thisPose = ms->config.currentEEPose;
    } else {
      assert(0);
    }
  } else {
    CONSOLE_ERROR(ms, "No images in the buffer, returning." << endl);
    return;
  }

  if (success != 1) {
    CONSOLE_ERROR(ms, "  Not doing update because of stream buffer errors.");
    return;
  }

  //if (fabs(thisPose.qz) > 0.005) {
  //CONSOLE_ERROR(ms, "  Not doing update because arm not vertical.");
  //return;
  //}

  eePose transformed = thisPose.getPoseRelativeTo(ms->config.scene->anchor_pose);
  cout << "untransformed qz: " << thisPose.qz << endl;
  cout << "  transformed qz: " << transformed.qz << endl;
  cout << "    reference qz: " << ms->config.scene->anchor_pose.qz << endl;

  if (fabs(transformed.qz) > 0.01) {
    CONSOLE_ERROR(ms, " Maybe shouldn't do update because arm not perpendicular to plane of scene.");
    //return;
  }

  Mat wristViewYCbCr = bufferImage.clone();

  cvtColor(bufferImage, wristViewYCbCr, cv::COLOR_BGR2YCrCb);

  Size sz = bufferImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 
  
  //for (int px = ; px < ; px++) 
  //for (int py = ; py < ; py++) 
  pixelToGlobalCache data;
  //double z = thisPose.pz + ms->config.currentTableZ;
  double z = transformed.pz;
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  cout << "z: " << z << endl;
  computePixelToPlaneCache(ms, z, thisPose, ms->config.scene->anchor_pose, &data);

  for (int px = topx; px < botx; px++) {
    for (int py = topy; py < boty; py++) {
      //for (int px = 0; px < imW; px++) 
      //for (int py = 0; py < imH; py++) 
      if (isInGripperMask(ms, px, py)) {
	continue;
      }
      double x, y;
      pixelToGlobalFromCache(ms, px, py, &x, &y, &data);
      
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
}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferNoRecalc)

WORD(SceneUpdateObservedFromStreamBuffer)
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("sceneUpdateObservedFromStreamBufferNoRecalc sceneRecalculateObservedMusAndSigmas sceneRenderObservedMap");
}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBuffer)

WORD(SceneRenderObservedMap)
virtual void execute(MachineState * ms) {
  if (!ms->config.showgui) {
    return;
  }    
  {
    Mat image;
    ms->config.scene->observed_map->rgbMuToBgrMat(image);
    ms->config.observedWindow->updateImage(image);
  }

  {
    Mat image;
    ms->config.scene->observed_map->rgbSigmaToMat(image);
    Mat rgb = image.clone();  
    //cvtColor(image, rgb, cv::COLOR_YCrCb2BGR);
    ms->config.observedStdDevWindow->updateImage(rgb);
  }
  ms->config.observedMapWindow->updateMap(ms->config.scene->observed_map);
}
END_WORD
REGISTER_WORD(SceneRenderObservedMap)

WORD(SceneComposePredictedMap)
virtual void execute(MachineState * ms) {
  ms->config.scene->composePredictedMap();
  ms->pushWord("sceneRenderPredictedMap");
}
END_WORD
REGISTER_WORD(SceneComposePredictedMap)

WORD(SceneComposePredictedMapThreshed)
virtual void execute(MachineState * ms) {
  double threshold = 0.0;
  GET_NUMERIC_ARG(ms, threshold);
  ms->config.scene->composePredictedMap(threshold);
  ms->pushWord("sceneRenderPredictedMap");
}
END_WORD
REGISTER_WORD(SceneComposePredictedMapThreshed)

WORD(SceneRenderPredictedMap)
virtual void execute(MachineState * ms) {
  {
    if (!ms->config.showgui) {
      return;
    }    
    Mat image;
    ms->config.scene->predicted_map->rgbMuToMat(image);
    Mat rgb = image.clone();  
    cvtColor(image, rgb, cv::COLOR_YCrCb2BGR);
    ms->config.predictedWindow->updateImage(rgb);
  }

  {
    Mat image;
    ms->config.scene->predicted_map->rgbSigmaToMat(image);
    Mat rgb = image.clone();  
    //cvtColor(image, rgb, cv::COLOR_YCrCb2BGR);
    ms->config.predictedStdDevWindow->updateImage(rgb);
  }

  ms->config.predictedMapWindow->updateMap(ms->config.scene->predicted_map);

}
END_WORD
REGISTER_WORD(SceneRenderPredictedMap)


WORD(SceneUpdateDiscrepancy)
virtual void execute(MachineState * ms) {
  ms->config.scene->measureDiscrepancy();
  ms->pushWord("sceneRenderDiscrepancy");
}
END_WORD
REGISTER_WORD(SceneUpdateDiscrepancy)


WORD(SceneRenderDiscrepancy)
virtual void execute(MachineState * ms) {

  if (!ms->config.showgui) {
    return;
  }    
  
  Mat image;
  ms->config.scene->discrepancy->rgbDiscrepancyMuToMat(ms, image);
  ms->config.discrepancyWindow->updateImage(image);

  Mat densityImage = ms->config.scene->discrepancy_density.clone();

  for (int x = 0; x < ms->config.scene->width; x++) {
    for (int y = 0; y < ms->config.scene->height; y++) {
      if ((ms->config.scene->predicted_map->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold) && (ms->config.scene->observed_map->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold)) {
      } else {
	densityImage.at<double>(x,y) = 0;
      }
    }
  }

  //cout << "scene: " << ms->config.scene.get() << endl;
  //cout << "discrepancy_density: " << ms->config.scene->discrepancy_density << endl;
  ms->config.discrepancyDensityWindow->updateImage(ms->config.scene->discrepancy_density);
  ms->config.discrepancyViewerWindow->update(image, densityImage);
  // XXX considered changing this because it looked wrong
  //ms->config.discrepancyDensityWindow->updateImage(densityImage);
}
END_WORD
REGISTER_WORD(SceneRenderDiscrepancy)

WORD(SceneRenderZ)
virtual void execute(MachineState * ms) {
  if (!ms->config.showgui) {
    return;
  }    
  
  Mat toShow;
  ms->config.scene->observed_map->zMuToScaledMat(toShow);
  ms->config.zWindow->updateImage(toShow);
}
END_WORD
REGISTER_WORD(SceneRenderZ)



CONFIG_GETTER_INT(SceneDiscrepancyMode, ms->config.discrepancyMode);

WORD(SceneSetDiscrepancyModeDot)
virtual void execute(MachineState * ms) {
  ms->config.discrepancyMode = DISCREPANCY_DOT;
}
END_WORD
REGISTER_WORD(SceneSetDiscrepancyModeDot)

WORD(SceneSetDiscrepancyModePoint)
virtual void execute(MachineState * ms) {
  ms->config.discrepancyMode = DISCREPANCY_POINT;
}
END_WORD
REGISTER_WORD(SceneSetDiscrepancyModePoint)

WORD(SceneSetDiscrepancyModeNoisyOr)
virtual void execute(MachineState * ms) {
  ms->config.discrepancyMode = DISCREPANCY_NOISY_OR;
}
END_WORD
REGISTER_WORD(SceneSetDiscrepancyModeNoisyOr)

WORD(SceneSetDiscrepancyModeNoisyAnd)
virtual void execute(MachineState * ms) {
  ms->config.discrepancyMode = DISCREPANCY_NOISY_AND;
}
END_WORD
REGISTER_WORD(SceneSetDiscrepancyModeNoisyAnd)

WORD(SceneSetDiscrepancyDensityFromZ)
virtual void execute(MachineState * ms) {
  ms->config.scene->observed_map->zMuToMat(ms->config.scene->discrepancy_density);
  // XXX needs bias for render
  //ms->config.scene->discrepancy_density = -ms->config.scene->discrepancy_density;
}
END_WORD
REGISTER_WORD(SceneSetDiscrepancyDensityFromZ)

WORD(SceneSetObservedRGBFromZ)
virtual void execute(MachineState * ms) {
// XXX this might need controls to bias it
  double viewScale = 1000.0;
  GET_NUMERIC_ARG(ms, viewScale);

  Mat toShow;
  ms->config.scene->observed_map->zMuToMat(toShow);

  double positiveMin = DBL_MAX;
  double positiveMax = 0.0;
  for (int x = 0; x < ms->config.scene->width; x++) {
    for (int y = 0; y < ms->config.scene->height; y++) {
      if ( (ms->config.scene->observed_map->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold) ) {
	positiveMin = std::min(positiveMin, toShow.at<double>(x,y));
	positiveMax = std::max(positiveMax, toShow.at<double>(x,y));
      }
    }
  }
  
  cout << "sceneSetObservedFromZ, viewScale positiveMin positiveMax: " << viewScale << " " << positiveMin << " " << positiveMax << endl;

  double p_zSetSamples = 100.0;
  for (int x = 0; x < ms->config.scene->width; x++) {
    for (int y = 0; y < ms->config.scene->height; y++) {
      if ( (ms->config.scene->observed_map->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold) ) {
	GaussianMapCell * toSet = (ms->config.scene->observed_map->refAtCell(x,y));

	double val = (positiveMax - toShow.at<double>(x,y)) * viewScale;

	toSet->red.counts = 128 * p_zSetSamples;
	toSet->green.counts = 128 * p_zSetSamples;
	toSet->blue.counts = val * p_zSetSamples;
	toSet->red.squaredcounts = 128 * 128 * p_zSetSamples;
	toSet->green.squaredcounts = 128 * 128 * p_zSetSamples;
	toSet->blue.squaredcounts = val * val * p_zSetSamples;
	toSet->red.samples = p_zSetSamples;
	toSet->green.samples = p_zSetSamples;
	toSet->blue.samples = p_zSetSamples;
	toSet->recalculateMusAndSigmas(ms);
      }
    }
  }
}
END_WORD
REGISTER_WORD(SceneSetObservedRGBFromZ)

WORD(SceneSetObservedRGBFromVariance)
virtual void execute(MachineState * ms) {
// XXX this might need controls to bias it
  double viewScale = 1.0;
  GET_NUMERIC_ARG(ms, viewScale);

  Mat toShow;
  ms->config.scene->observed_map->rgbSigmaSquaredToMat(toShow);

  double positiveMin = DBL_MAX;
  double positiveMax = 0.0;
  for (int x = 0; x < ms->config.scene->width; x++) {
    for (int y = 0; y < ms->config.scene->height; y++) {
      if ( (ms->config.scene->observed_map->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold) ) {
	double valnot = toShow.at<Vec3d>(x,y)[0] + toShow.at<Vec3d>(x,y)[1] + toShow.at<Vec3d>(x,y)[2];
	positiveMin = std::min(positiveMin, valnot);
	positiveMax = std::max(positiveMax, valnot);
      }
    }
  }
  
  cout << "sceneSetObservedFromZ, viewScale positiveMin positiveMax: " << viewScale << " " << positiveMin << " " << positiveMax << endl;

  double p_zSetSamples = 100.0;
  for (int x = 0; x < ms->config.scene->width; x++) {
    for (int y = 0; y < ms->config.scene->height; y++) {
      if ( (ms->config.scene->observed_map->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold) ) {
	GaussianMapCell * toSet = (ms->config.scene->observed_map->refAtCell(x,y));

	double valnot = toShow.at<Vec3d>(x,y)[0] + toShow.at<Vec3d>(x,y)[1] + toShow.at<Vec3d>(x,y)[2];
	double val = valnot * viewScale;

	toSet->red.counts = 128 * p_zSetSamples;
	toSet->green.counts = 128 * p_zSetSamples;
	toSet->blue.counts = val * p_zSetSamples;
	toSet->red.squaredcounts = 128 * 128 * p_zSetSamples;
	toSet->green.squaredcounts = 128 * 128 * p_zSetSamples;
	toSet->blue.squaredcounts = val * val * p_zSetSamples;
	toSet->red.samples = p_zSetSamples;
	toSet->green.samples = p_zSetSamples;
	toSet->blue.samples = p_zSetSamples;
	toSet->recalculateMusAndSigmas(ms);
      }
    }
  }
}
END_WORD
REGISTER_WORD(SceneSetObservedRGBFromVariance)



WORD(SceneSetPredictedClassNameToFocusedClass)
virtual void execute(MachineState * ms) {
  ms->config.scene->predicted_class_name = ms->config.focusedClassLabel;
  cout << "sceneSetPredictedClassNameToFocusedClass: setting to " << ms->config.scene->predicted_class_name << endl;
}
END_WORD
REGISTER_WORD(SceneSetPredictedClassNameToFocusedClass)

WORD(SceneSetAnnotatedClassNameToFocusedClass)
virtual void execute(MachineState * ms) {
  ms->config.scene->annotated_class_name = ms->config.focusedClassLabel;
  cout << "sceneSetAnnotatedClassNameToFocusedClass: setting to " << ms->config.scene->annotated_class_name << endl;
}
END_WORD
REGISTER_WORD(SceneSetAnnotatedClassNameToFocusedClass)

CONFIG_GETTER_STRING(SceneGetPredictedClassName, ms->config.scene->predicted_class_name, "The predicted class.")
CONFIG_SETTER_STRING(SceneSetPredictedClassName, ms->config.scene->predicted_class_name)
CONFIG_GETTER_STRING(SceneGetAnnotatedClassName, ms->config.scene->annotated_class_name, "The annotated class, for evaluation purposes.")
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


CONFIG_SETTER_ENUM(SceneSetClassificationMode, ms->config.currentSceneClassificationMode, (sceneClassificationMode))
CONFIG_GETTER_INT(SceneGetClassificationMode, ms->config.currentSceneClassificationMode)

WORD(SceneInitRegisterMax)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  cout << "sceneRecallFromRegister: copying..." << endl;
  ms->config.scene->observed_map = ms->config.gaussian_map_register->copy();
}
END_WORD
REGISTER_WORD(SceneRecallFromRegister)

WORD(SceneStoreObservedInRegister)
virtual void execute(MachineState * ms) {
  cout << "sceneStoreObservedInRegister: copying..." << endl;
  ms->config.gaussian_map_register = ms->config.scene->observed_map->copy();
}
END_WORD
REGISTER_WORD(SceneStoreObservedInRegister)

WORD(SceneUpdateObservedFromStreamBufferAtZ)
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("sceneUpdateObservedFromStreamBufferAtZNoRecalc sceneRecalculateObservedMusAndSigmas sceneRenderObservedMap");
}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZ)

WORD(SceneRecalculateObservedMusAndSigmas)
virtual void execute(MachineState * ms) {
  cout << "sceneRecalculateObservedMusAndSigmas: ." << endl;
  ms->config.scene->observed_map->recalculateMusAndSigmas(ms);
}
END_WORD
REGISTER_WORD(SceneRecalculateObservedMusAndSigmas)

WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalc)
virtual void execute(MachineState * ms) {
  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  int thisIdx = camera->sibCurIdx;
  //cout << "sceneUpdateObservedFromStreamBuffer: " << thisIdx << endl;

  Mat bufferImage;
  eePose thisPose, tBaseP;


  int success = 1;
  if ( (thisIdx > -1) && (thisIdx < camera->streamImageBuffer.size()) ) {
    streamImage &tsi = camera->streamImageBuffer[thisIdx];
    loadStreamImage(ms, &tsi);

    bufferImage = tsi.image.clone();

    if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
      success = ms->getStreamPoseAtTime(tsi.time, &thisPose, &tBaseP);
    } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
      success = 1;
      thisPose = ms->config.currentEEPose;
      z_to_use = ms->config.currentEEPose.pz + ms->config.currentTableZ;
    } else {
      assert(0);
    }
  } else {
    CONSOLE_ERROR(ms, "No images in the buffer, returning." << endl);
    return;
  }

  if (success != 1) {
    CONSOLE_ERROR(ms, "  Not doing update because of stream buffer errors.");
    return;
  }

  eePose transformed = thisPose.getPoseRelativeTo(ms->config.scene->anchor_pose);
  if (fabs(transformed.qz) > 0.01) {
    CONSOLE_ERROR(ms, "  Not doing update because arm not vertical.");
    return;
  }

  Mat wristViewYCbCr = bufferImage.clone();

  cvtColor(bufferImage, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
  
  Size sz = bufferImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 
  
  pixelToGlobalCache data;
  double z = z_to_use;
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  computePixelToPlaneCache(ms, z, thisPose, ms->config.scene->anchor_pose, &data);  
  int numThreads = 8;
  // there is a faster way to stride it but i am risk averse atm


  int numPixels = 0;
  int numNulls = 0;
  Mat gripperMask = camera->gripperMask;
  if (isSketchyMat(gripperMask)) {
    CONSOLE_ERROR(ms, "Gripper mask is messed up.");
    return;
  }

  //#pragma omp parallel for
  for (int i = 0; i < numThreads; i++) {
    //double frac = double(boty - topy) / double(numThreads);
    //double bfrac = i*frac;
    //double tfrac = (i+1)*frac;

    double frac = double(boty - topy) / double(numThreads);
    int ttopy = floor(topy + i*frac);
    int tboty = floor(topy + (i+1)*frac);

    //for (int py = topy; py <= boty; py++) 
      for (int py = ttopy; py < tboty; py++) 
      {
        uchar* gripperMaskPixel = camera->gripperMask.ptr<uchar>(py); // point to first pixel in row
        cv::Vec3b* wristViewPixel = wristViewYCbCr.ptr<cv::Vec3b>(py);

      //double opy = py-topy;
      // this is superior
      //if ( (bfrac <= opy) && (opy < tfrac) ) 
      //{
	for (int px = topx; px <= botx; px++) {
	  if (gripperMaskPixel[px] == 0) {
	    continue;
	  }

	  if ( (abhr > 0) && (abhc > 0) ) {
	    if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
		 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
	      continue;
	    } 
	  } 

	  double x, y;
	  pixelToGlobalFromCache(ms, px, py, &x, &y, &data);
	  
	  if (1) {
	    // single sample update
	    int i, j;
	    ms->config.scene->observed_map->metersToCell(x, y, &i, &j);
	    GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(i, j);
	    if (cell != NULL) {
              //Vec3b pixel = wristViewYCbCr.at<Vec3b>(py, px);
              //cell->newObservation(pixel, z);
              //uchar val = img.data[img.step.p[0]*row + img.step.p[1]*col + cn];

              cell->newObservation(wristViewPixel[px]);
              numPixels++;
	    }
	  } else {
	      numNulls++;
	  }
	}
      //}
    }
  }
  //cout << "numPixels numNulls sum apertureSize: " << numPixels << " " << numNulls << " " << numPixels + numNulls << " " << ms->config.angular_aperture_rows * ms->config.angular_aperture_cols << endl;
  //ms->config.scene->observed_map->recalculateMusAndSigmas(ms);
}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalc)





WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcAll)

virtual string description() {
  return "Updates the observed map from the stream buffer in a big for loop; fastest version without GPU.  This is the one to copy and call.";
}

virtual void execute(MachineState * ms) {
  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);

  int stride = 1;
  GET_INT_ARG(ms, stride);



  Size sz = ms->config.wristViewImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 
  
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  Mat gripperMask = camera->gripperMask;
  if (isSketchyMat(gripperMask)) {
    CONSOLE_ERROR(ms, "Gripper mask is messed up.");
    return;
  }

  int numThreads = 8;


  vector<shared_ptr<GaussianMap> > maps;
  maps.resize(numThreads);

  #pragma omp parallel for
  for (int thread = 0; thread < numThreads; thread++) {
    
    maps[thread] = make_shared<GaussianMap>(ms, 
                                            ms->config.scene->observed_map->width, 
                                            ms->config.scene->observed_map->height, 
                                            ms->config.scene->observed_map->cell_width,
                                            ms->config.scene->observed_map->anchor_pose); 
    maps[thread]->zero();

    
    int thisStart = thread * (camera->streamImageBuffer.size()  / numThreads);
    int thisEnd = (thread + 1) * (camera->streamImageBuffer.size()  / numThreads); 
    stringstream buf;    
    buf << "thread: " << thread << " start: " << thisStart << " end: " << thisEnd << endl;
    cout << buf.str();
    for (int i = thisStart; i < thisEnd; i+=stride) {
      streamImage * tsi = camera->setIsbIdxNoLoadNoKick(i);
      
      
      
      if (tsi == NULL) {
        CONSOLE_ERROR(ms, "Stream image null.");
      }
      loadStreamImage(ms, tsi);

      eePose tArmP, tBaseP;
      
      int success = 0;
      double z = z_to_use;
      if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
        success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
      } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
        success = 1;
        tArmP = ms->config.currentEEPose;
        z = ms->config.currentEEPose.pz + ms->config.currentTableZ;
      } else {
        assert(0);
      }
      
      
      if (success != 1) {
        CONSOLE_ERROR(ms, "Couldn't get stream pose.  Return code: " << success << " time: " << tsi->time);
        continue;
      }
      
      eePose transformed = tArmP.getPoseRelativeTo(ms->config.scene->anchor_pose);
      if (fabs(transformed.qz) > 0.01) {
        CONSOLE_ERROR(ms, "Not doing update because arm not vertical.");
        continue;
        //CONSOLE_ERROR(ms, "Would normally not be doing update because arm not vertical, but we are.");
      }
      pixelToGlobalCache data;      
      computePixelToPlaneCache(ms, z, tArmP, ms->config.scene->anchor_pose, &data);  
      
      // there is a faster way to stride it but i am risk averse atm
      
      Mat wristViewYCbCr = tsi->image.clone();
      
      cvtColor(tsi->image, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
      int numPixels = 0;
      int numNulls = 0;
      
      uchar *input = (uchar*) (wristViewYCbCr.data);
      
      for (int py = topy; py <= boty; py++) {
        uchar* gripperMaskPixel = camera->gripperMask.ptr<uchar>(py); // point to first pixel in row
        for (int px = topx; px <= botx; px++) {
          if (gripperMaskPixel[px] == 0) {
            continue;
          }
          
          if ( (abhr > 0) && (abhc > 0) ) {
            if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
                 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
              continue;
            } 
          } 
          
          double x, y;
          pixelToGlobalFromCache(ms, px, py, &x, &y, &data);
          
          if (1) {
            // single sample update
            int i, j;
            maps[thread]->metersToCell(x, y, &i, &j);
            GaussianMapCell * cell = maps[thread]->refAtCell(i, j);
            //ms->config.scene->observed_map->metersToCell(x, y, &i, &j);
            //GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(i, j);
            
            
            if (cell != NULL) {
              int base_idx = py * wristViewYCbCr.cols*3 + px * 3;
              cell->newObservation(input[base_idx + 2], input[base_idx + 1], input[base_idx + 0], z);
              numPixels++;
            }
          } else {
            numNulls++;
          }
        }
      }
    }
  }
  
  #pragma omp for
  for (int y = 0; y < ms->config.scene->observed_map->height; y++) {
    for (int x = 0; x < ms->config.scene->observed_map->width; x++) {
      for (int thread = 0; thread < numThreads; thread++) {
        if (maps[thread]->refAtCell(x, y)->red.samples > 0) {
          ms->config.scene->observed_map->refAtCell(x, y)->addC(maps[thread]->refAtCell(x, y));
        }
      }
    }
  }

}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcAll)

WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcAllLightModelRenderGlareOnly)
virtual void execute(MachineState * ms) {
  double bright_cast_thresh = 0.0;
  GET_NUMERIC_ARG(ms, bright_cast_thresh);

  double color_cast_thresh = 0.0;
  GET_NUMERIC_ARG(ms, color_cast_thresh);

  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);

  int stride = 1;
  GET_INT_ARG(ms, stride);



  Size sz = ms->config.wristViewImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 
  
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  Mat gripperMask = camera->gripperMask;
  if (isSketchyMat(gripperMask)) {
    CONSOLE_ERROR(ms, "Gripper mask is messed up.");
    return;
  }

  int numThreads = 8;


  vector<shared_ptr<GaussianMap> > maps;
  maps.resize(numThreads);

  GaussianMapCell * lm = &(ms->config.scene->light_model);

  #pragma omp parallel for
  for (int thread = 0; thread < numThreads; thread++) {
    
    maps[thread] = make_shared<GaussianMap>(ms, 
                                            ms->config.scene->observed_map->width, 
                                            ms->config.scene->observed_map->height, 
                                            ms->config.scene->observed_map->cell_width,
                                            ms->config.scene->observed_map->anchor_pose); 
    maps[thread]->zero();

    
    int thisStart = thread * (camera->streamImageBuffer.size()  / numThreads);
    int thisEnd = (thread + 1) * (camera->streamImageBuffer.size()  / numThreads); 
    stringstream buf;    
    buf << "thread: " << thread << " start: " << thisStart << " end: " << thisEnd << endl;
    cout << buf.str();
    for (int i = thisStart; i < thisEnd; i+=stride) {
      streamImage * tsi = camera->setIsbIdxNoLoadNoKick(i);
      
      
      
      if (tsi == NULL) {
        CONSOLE_ERROR(ms, "Stream image null.");
      }
      eePose tArmP, tBaseP;
      
      int success = 0;
      double z = z_to_use;
      if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
        success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
      } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
        success = 1;
        tArmP = ms->config.currentEEPose;
        z = ms->config.currentEEPose.pz + ms->config.currentTableZ;
      } else {
        assert(0);
      }
      
      
      if (success != 1) {
        CONSOLE_ERROR(ms, "Couldn't get stream pose.  Return code: " << success << " time: " << tsi->time);
        continue;
      }
      
      eePose transformed = tArmP.getPoseRelativeTo(ms->config.scene->anchor_pose);
      if (fabs(transformed.qz) > 0.01) {
        CONSOLE_ERROR(ms, "Not doing update because arm not vertical.");
        continue;
      }
      pixelToGlobalCache data;      
      computePixelToPlaneCache(ms, z, tArmP, ms->config.scene->anchor_pose, &data);  
      
      // there is a faster way to stride it but i am risk averse atm
      
      Mat wristViewYCbCr = tsi->image.clone();
      
      cvtColor(tsi->image, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
      int numPixels = 0;
      int numNulls = 0;
      
      uchar *input = (uchar*) (wristViewYCbCr.data);
      
      for (int py = topy; py <= boty; py++) {
        uchar* gripperMaskPixel = camera->gripperMask.ptr<uchar>(py); // point to first pixel in row
        for (int px = topx; px <= botx; px++) {
          if (gripperMaskPixel[px] == 0) {
            continue;
          }

	  // measure discrepancy with lighting model and throw out this pixel if it
	  // is too much like the overhead light.
	  int base_idx = py * wristViewYCbCr.cols*3 + px * 3;

	  double total_discrepancy = 0.0;
	  double color_discrepancy = 0.0;
	  double bright_discrepancy = 0.0;
	  double p_ray_samples = 100.0;
	  {
	    GaussianMapCell rayCell;
	    rayCell.red.samples = p_ray_samples;
	    rayCell.green.samples = p_ray_samples;
	    rayCell.blue.samples = p_ray_samples;
	    rayCell.red.mu = input[base_idx + 2]; 
	    rayCell.green.mu = input[base_idx + 1]; 
	    rayCell.blue.mu = input[base_idx + 0];
	    rayCell.red.sigmasquared = 0;
	    rayCell.green.sigmasquared = 0;
	    rayCell.blue.sigmasquared = 0;

	    double rmu_diff = 0.0;
	    double gmu_diff = 0.0;
	    double bmu_diff = 0.0;

	    double discrepancy_value;
	    if (ms->config.discrepancyMode == DISCREPANCY_POINT) {
	      discrepancy_value = lm->pointDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_DOT) {
	      discrepancy_value = lm->innerProduct(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_OR) {
	      discrepancy_value = lm->noisyOrDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_AND) {
	      discrepancy_value = lm->noisyAndDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else {
	      cout << "Invalid discrepancy mode: " << ms->config.discrepancyMode << endl;
	      assert(0);
	    }
      
	    total_discrepancy = 1.0 - discrepancy_value;
	  } 
	  bright_discrepancy = total_discrepancy;
	  color_discrepancy = total_discrepancy;

	  /*
	  {
	    color_discrepancy = 
	      ( (input[base_idx + 2] - lm->red.mu)*(input[base_idx + 2] - lm->red.mu) +
	      (input[base_idx + 1] - lm->green.mu)*(input[base_idx + 1] - lm->green.mu) ) / 2.0
	    ;
	    bright_discrepancy = 
	      (input[base_idx + 0] - lm->blue.mu)*(input[base_idx + 0] - lm->blue.mu)
	    ;
	  }
	  */

	  if (color_discrepancy  > color_cast_thresh) {
	    //cout << "SK: " << color_discrepancy << " " ;
	    continue;
	  }
	  if (bright_discrepancy  > bright_cast_thresh) {
	    //cout << "SK: " << color_discrepancy << " " ;
	    continue;
	  }

          
          if ( (abhr > 0) && (abhc > 0) ) {
            if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
                 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
              continue;
            } 
          } 
          
          double x, y;
          pixelToGlobalFromCache(ms, px, py, &x, &y, &data);
          
	  // single sample update
	  int i, j;
	  maps[thread]->metersToCell(x, y, &i, &j);
	  GaussianMapCell * cell = maps[thread]->refAtCell(i, j);
	  //ms->config.scene->observed_map->metersToCell(x, y, &i, &j);
	  //GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(i, j);
	  
	  
	  if (cell != NULL) {
	    cell->newObservation(input[base_idx + 2], input[base_idx + 1], input[base_idx + 0], z);
	    numPixels++;
	  }
        }
      }
    }
  }
  
  #pragma omp for
  for (int y = 0; y < ms->config.scene->observed_map->height; y++) {
    for (int x = 0; x < ms->config.scene->observed_map->width; x++) {
      for (int thread = 0; thread < numThreads; thread++) {
        if (maps[thread]->refAtCell(x, y)->red.samples > 0) {
          ms->config.scene->observed_map->refAtCell(x, y)->addC(maps[thread]->refAtCell(x, y));
        }
      }
    }
  }

}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcAllLightModelRenderGlareOnly)

WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcAllLightModelRenderWithoutGlare)
virtual void execute(MachineState * ms) {
  double bright_cast_thresh = 0.0;
  GET_NUMERIC_ARG(ms, bright_cast_thresh);

  double color_cast_thresh = 0.0;
  GET_NUMERIC_ARG(ms, color_cast_thresh);

  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);

  int stride = 1;
  GET_INT_ARG(ms, stride);



  Size sz = ms->config.wristViewImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 
  
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  Mat gripperMask = camera->gripperMask;
  if (isSketchyMat(gripperMask)) {
    CONSOLE_ERROR(ms, "Gripper mask is messed up.");
    return;
  }

  int numThreads = 8;


  vector<shared_ptr<GaussianMap> > maps;
  maps.resize(numThreads);

  GaussianMapCell * lm = &(ms->config.scene->light_model);

  #pragma omp parallel for
  for (int thread = 0; thread < numThreads; thread++) {
    
    maps[thread] = make_shared<GaussianMap>(ms, 
                                            ms->config.scene->observed_map->width, 
                                            ms->config.scene->observed_map->height, 
                                            ms->config.scene->observed_map->cell_width,
                                            ms->config.scene->observed_map->anchor_pose); 
    maps[thread]->zero();

    
    int thisStart = thread * (camera->streamImageBuffer.size()  / numThreads);
    int thisEnd = (thread + 1) * (camera->streamImageBuffer.size()  / numThreads); 
    stringstream buf;    
    buf << "thread: " << thread << " start: " << thisStart << " end: " << thisEnd << endl;
    cout << buf.str();
    for (int i = thisStart; i < thisEnd; i+=stride) {
      streamImage * tsi = camera->setIsbIdxNoLoadNoKick(i);
      
      
      
      if (tsi == NULL) {
        CONSOLE_ERROR(ms, "Stream image null.");
      }
      eePose tArmP, tBaseP;
      
      int success = 0;
      double z = z_to_use;
      if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
        success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
      } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
        success = 1;
        tArmP = ms->config.currentEEPose;
        z = ms->config.currentEEPose.pz + ms->config.currentTableZ;
      } else {
        assert(0);
      }
      
      
      if (success != 1) {
        CONSOLE_ERROR(ms, "Couldn't get stream pose.  Return code: " << success << " time: " << tsi->time);
        continue;
      }
      
      eePose transformed = tArmP.getPoseRelativeTo(ms->config.scene->anchor_pose);
      if (fabs(transformed.qz) > 0.01) {
        CONSOLE_ERROR(ms, "Not doing update because arm not vertical.");
        continue;
      }
      pixelToGlobalCache data;      
      computePixelToPlaneCache(ms, z, tArmP, ms->config.scene->anchor_pose, &data);  
      
      // there is a faster way to stride it but i am risk averse atm
      
      Mat wristViewYCbCr = tsi->image.clone();
      
      cvtColor(tsi->image, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
      int numPixels = 0;
      int numNulls = 0;
      
      uchar *input = (uchar*) (wristViewYCbCr.data);
      
      for (int py = topy; py <= boty; py++) {
        uchar* gripperMaskPixel = camera->gripperMask.ptr<uchar>(py); // point to first pixel in row
        for (int px = topx; px <= botx; px++) {
          if (gripperMaskPixel[px] == 0) {
            continue;
          }

	  // measure discrepancy with lighting model and throw out this pixel if it
	  // is too much like the overhead light.
	  int base_idx = py * wristViewYCbCr.cols*3 + px * 3;

	  double total_discrepancy = 0.0;
	  double color_discrepancy = 0.0;
	  double bright_discrepancy = 0.0;
	  double p_ray_samples = 100.0;
	  {
	    GaussianMapCell rayCell;
	    rayCell.red.samples = p_ray_samples;
	    rayCell.green.samples = p_ray_samples;
	    rayCell.blue.samples = p_ray_samples;
	    rayCell.red.mu = input[base_idx + 2]; 
	    rayCell.green.mu = input[base_idx + 1]; 
	    rayCell.blue.mu = input[base_idx + 0];
	    rayCell.red.sigmasquared = 0;
	    rayCell.green.sigmasquared = 0;
	    rayCell.blue.sigmasquared = 0;

	    double rmu_diff = 0.0;
	    double gmu_diff = 0.0;
	    double bmu_diff = 0.0;

	    double discrepancy_value;
	    if (ms->config.discrepancyMode == DISCREPANCY_POINT) {
	      discrepancy_value = lm->pointDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_DOT) {
	      discrepancy_value = lm->innerProduct(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_OR) {
	      discrepancy_value = lm->noisyOrDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_AND) {
	      discrepancy_value = lm->noisyAndDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else {
	      cout << "Invalid discrepancy mode: " << ms->config.discrepancyMode << endl;
	      assert(0);
	    }
      
	    total_discrepancy = 1.0 - discrepancy_value;
	  } 
	  bright_discrepancy = total_discrepancy;
	  color_discrepancy = total_discrepancy;

	  /*
	  {
	    color_discrepancy = 
	      ( (input[base_idx + 2] - lm->red.mu)*(input[base_idx + 2] - lm->red.mu) +
	      (input[base_idx + 1] - lm->green.mu)*(input[base_idx + 1] - lm->green.mu) ) / 2.0
	    ;
	    bright_discrepancy = 
	      (input[base_idx + 0] - lm->blue.mu)*(input[base_idx + 0] - lm->blue.mu)
	    ;
	  }
	  */

	  if (color_discrepancy  < color_cast_thresh) {
	    //cout << "SK: " << color_discrepancy << " " ;
	    continue;
	  }
	  if (bright_discrepancy  < bright_cast_thresh) {
	    //cout << "SK: " << color_discrepancy << " " ;
	    continue;
	  }

          
          if ( (abhr > 0) && (abhc > 0) ) {
            if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
                 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
              continue;
            } 
          } 
          
          double x, y;
          pixelToGlobalFromCache(ms, px, py, &x, &y, &data);
          
	  // single sample update
	  int i, j;
	  maps[thread]->metersToCell(x, y, &i, &j);
	  GaussianMapCell * cell = maps[thread]->refAtCell(i, j);
	  //ms->config.scene->observed_map->metersToCell(x, y, &i, &j);
	  //GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(i, j);
	  
	  
	  if (cell != NULL) {
	    cell->newObservation(input[base_idx + 2], input[base_idx + 1], input[base_idx + 0], z);
	    numPixels++;
	  }
        }
      }
    }
  }
  
  #pragma omp for
  for (int y = 0; y < ms->config.scene->observed_map->height; y++) {
    for (int x = 0; x < ms->config.scene->observed_map->width; x++) {
      for (int thread = 0; thread < numThreads; thread++) {
        if (maps[thread]->refAtCell(x, y)->red.samples > 0) {
          ms->config.scene->observed_map->refAtCell(x, y)->addC(maps[thread]->refAtCell(x, y));
        }
      }
    }
  }

}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcAllLightModelRenderWithoutGlare)

WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcAllLightModelRenderGlareOnlyRising)
virtual void execute(MachineState * ms) {
  double bright_cast_thresh = 0.0;
  GET_NUMERIC_ARG(ms, bright_cast_thresh);

  double color_cast_thresh = 0.0;
  GET_NUMERIC_ARG(ms, color_cast_thresh);

  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);

  int stride = 1;
  GET_INT_ARG(ms, stride);



  Size sz = ms->config.wristViewImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 
  
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  Mat gripperMask = camera->gripperMask;
  if (isSketchyMat(gripperMask)) {
    CONSOLE_ERROR(ms, "Gripper mask is messed up.");
    return;
  }

  int numThreads = 8;


  vector<shared_ptr<GaussianMap> > maps;
  maps.resize(numThreads);

  GaussianMapCell * lm = &(ms->config.scene->light_model);

  #pragma omp parallel for
  for (int thread = 0; thread < numThreads; thread++) {
    
    maps[thread] = make_shared<GaussianMap>(ms, 
                                            ms->config.scene->observed_map->width, 
                                            ms->config.scene->observed_map->height, 
                                            ms->config.scene->observed_map->cell_width,
                                            ms->config.scene->observed_map->anchor_pose); 
    maps[thread]->zero();

    
    int thisStart = thread * (camera->streamImageBuffer.size()  / numThreads);
    int thisEnd = (thread + 1) * (camera->streamImageBuffer.size()  / numThreads); 
    stringstream buf;    
    buf << "thread: " << thread << " start: " << thisStart << " end: " << thisEnd << endl;
    cout << buf.str();
    for (int i = thisStart; i < thisEnd; i+=stride) {
      streamImage * tsi = camera->setIsbIdxNoLoadNoKick(i);
      
      
      
      if (tsi == NULL) {
        CONSOLE_ERROR(ms, "Stream image null.");
      }
      eePose tArmP, tBaseP;
      
      int success = 0;
      double z = z_to_use;
      if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
        success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
      } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
        success = 1;
        tArmP = ms->config.currentEEPose;
        z = ms->config.currentEEPose.pz + ms->config.currentTableZ;
      } else {
        assert(0);
      }
      
      
      if (success != 1) {
        CONSOLE_ERROR(ms, "Couldn't get stream pose.  Return code: " << success << " time: " << tsi->time);
        continue;
      }
      
      eePose transformed = tArmP.getPoseRelativeTo(ms->config.scene->anchor_pose);
      if (fabs(transformed.qz) > 0.01) {
        CONSOLE_ERROR(ms, "Not doing update because arm not vertical.");
        continue;
      }
      pixelToGlobalCache data;      
      computePixelToPlaneCache(ms, z, tArmP, ms->config.scene->anchor_pose, &data);  
      
      // there is a faster way to stride it but i am risk averse atm
      
      Mat wristViewYCbCr = tsi->image.clone();
      
      cvtColor(tsi->image, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
      int numPixels = 0;
      int numNulls = 0;
      
      uchar *input = (uchar*) (wristViewYCbCr.data);
      
      for (int py = topy; py <= boty; py++) {
        uchar* gripperMaskPixel = camera->gripperMask.ptr<uchar>(py); // point to first pixel in row
        for (int px = topx; px <= botx; px++) {
          if (gripperMaskPixel[px] == 0) {
            continue;
          }

	  // measure discrepancy with lighting model and throw out this pixel if it
	  // is too much like the overhead light.
	  int base_idx = py * wristViewYCbCr.cols*3 + px * 3;

	  double total_discrepancy = 0.0;
	  double color_discrepancy = 0.0;
	  double bright_discrepancy = 0.0;
	  double p_ray_samples = 100.0;
	  {
	    GaussianMapCell rayCell;
	    rayCell.red.samples = p_ray_samples;
	    rayCell.green.samples = p_ray_samples;
	    rayCell.blue.samples = p_ray_samples;
	    rayCell.red.mu = input[base_idx + 2]; 
	    rayCell.green.mu = input[base_idx + 1]; 
	    rayCell.blue.mu = input[base_idx + 0];
	    rayCell.red.sigmasquared = 0;
	    rayCell.green.sigmasquared = 0;
	    rayCell.blue.sigmasquared = 0;

	    double rmu_diff = 0.0;
	    double gmu_diff = 0.0;
	    double bmu_diff = 0.0;

	    double discrepancy_value;
	    if (ms->config.discrepancyMode == DISCREPANCY_POINT) {
	      discrepancy_value = lm->pointDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_DOT) {
	      discrepancy_value = lm->innerProduct(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_OR) {
	      discrepancy_value = lm->noisyOrDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_AND) {
	      discrepancy_value = lm->noisyAndDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else {
	      cout << "Invalid discrepancy mode: " << ms->config.discrepancyMode << endl;
	      assert(0);
	    }
      
	    total_discrepancy = 1.0 - discrepancy_value;
	  } 
	  bright_discrepancy = total_discrepancy;
	  color_discrepancy = total_discrepancy;

	  /*
	  {
	    color_discrepancy = 
	      ( (input[base_idx + 2] - lm->red.mu)*(input[base_idx + 2] - lm->red.mu) +
	      (input[base_idx + 1] - lm->green.mu)*(input[base_idx + 1] - lm->green.mu) ) / 2.0
	    ;
	    bright_discrepancy = 
	      (input[base_idx + 0] - lm->blue.mu)*(input[base_idx + 0] - lm->blue.mu)
	    ;
	  }
	  */

	  if (color_discrepancy  > color_cast_thresh) {
	    //cout << "SK: " << color_discrepancy << " " ;
	    continue;
	  }
	  if (bright_discrepancy  > bright_cast_thresh) {
	    //cout << "SK: " << color_discrepancy << " " ;
	    continue;
	  }

          
          if ( (abhr > 0) && (abhc > 0) ) {
            if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
                 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
              continue;
            } 
          } 
          
          double x, y;
          pixelToGlobalFromCache(ms, px, py, &x, &y, &data);
          
	  // single sample update
	  int i, j;
	  maps[thread]->metersToCell(x, y, &i, &j);
	  GaussianMapCell * cell = maps[thread]->refAtCell(i, j);
	  int ri, rj;
	  ms->config.gaussian_map_register->metersToCell(x, y, &ri, &rj);
	  GaussianMapCell * register_cell = ms->config.gaussian_map_register->refAtCell(ri, rj);
	  
	  if (register_cell == NULL) {
	    continue;
	  }
	  if (z_to_use > register_cell->z.mu) {
	    //cout << "SK: " << color_discrepancy << " " ;
	    continue;
	  }
	  
	  if (cell != NULL) {
	    cell->newObservation(input[base_idx + 2], input[base_idx + 1], input[base_idx + 0], z);
	    numPixels++;
	  }
        }
      }
    }
  }
  
  #pragma omp for
  for (int y = 0; y < ms->config.scene->observed_map->height; y++) {
    for (int x = 0; x < ms->config.scene->observed_map->width; x++) {
      for (int thread = 0; thread < numThreads; thread++) {
        if (maps[thread]->refAtCell(x, y)->red.samples > 0) {
          ms->config.scene->observed_map->refAtCell(x, y)->addC(maps[thread]->refAtCell(x, y));
        }
      }
    }
  }

}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcAllLightModelRenderGlareOnlyRising)

WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcAllLightModelRenderWithoutGlareRising)
virtual void execute(MachineState * ms) {
  double bright_cast_thresh = 0.0;
  GET_NUMERIC_ARG(ms, bright_cast_thresh);

  double color_cast_thresh = 0.0;
  GET_NUMERIC_ARG(ms, color_cast_thresh);

  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);

  int stride = 1;
  GET_INT_ARG(ms, stride);



  Size sz = ms->config.wristViewImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 
  
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  Mat gripperMask = camera->gripperMask;
  if (isSketchyMat(gripperMask)) {
    CONSOLE_ERROR(ms, "Gripper mask is messed up.");
    return;
  }

  int numThreads = 8;


  vector<shared_ptr<GaussianMap> > maps;
  maps.resize(numThreads);

  GaussianMapCell * lm = &(ms->config.scene->light_model);

  #pragma omp parallel for
  for (int thread = 0; thread < numThreads; thread++) {
    
    maps[thread] = make_shared<GaussianMap>(ms, 
                                            ms->config.scene->observed_map->width, 
                                            ms->config.scene->observed_map->height, 
                                            ms->config.scene->observed_map->cell_width,
                                            ms->config.scene->observed_map->anchor_pose); 
    maps[thread]->zero();

    
    int thisStart = thread * (camera->streamImageBuffer.size()  / numThreads);
    int thisEnd = (thread + 1) * (camera->streamImageBuffer.size()  / numThreads); 
    stringstream buf;    
    buf << "thread: " << thread << " start: " << thisStart << " end: " << thisEnd << endl;
    cout << buf.str();
    for (int i = thisStart; i < thisEnd; i+=stride) {
      streamImage * tsi = camera->setIsbIdxNoLoadNoKick(i);
      
      
      
      if (tsi == NULL) {
        CONSOLE_ERROR(ms, "Stream image null.");
      }
      eePose tArmP, tBaseP;
      
      int success = 0;
      double z = z_to_use;
      if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
        success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
      } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
        success = 1;
        tArmP = ms->config.currentEEPose;
        z = ms->config.currentEEPose.pz + ms->config.currentTableZ;
      } else {
        assert(0);
      }
      
      
      if (success != 1) {
        CONSOLE_ERROR(ms, "Couldn't get stream pose.  Return code: " << success << " time: " << tsi->time);
        continue;
      }
      
      eePose transformed = tArmP.getPoseRelativeTo(ms->config.scene->anchor_pose);
      if (fabs(transformed.qz) > 0.01) {
        CONSOLE_ERROR(ms, "Not doing update because arm not vertical.");
        continue;
      }
      pixelToGlobalCache data;      
      computePixelToPlaneCache(ms, z, tArmP, ms->config.scene->anchor_pose, &data);  
      
      // there is a faster way to stride it but i am risk averse atm
      
      Mat wristViewYCbCr = tsi->image.clone();
      
      cvtColor(tsi->image, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
      int numPixels = 0;
      int numNulls = 0;
      
      uchar *input = (uchar*) (wristViewYCbCr.data);
      
      for (int py = topy; py <= boty; py++) {
        uchar* gripperMaskPixel = camera->gripperMask.ptr<uchar>(py); // point to first pixel in row
        for (int px = topx; px <= botx; px++) {
          if (gripperMaskPixel[px] == 0) {
            continue;
          }

	  // measure discrepancy with lighting model and throw out this pixel if it
	  // is too much like the overhead light.
	  int base_idx = py * wristViewYCbCr.cols*3 + px * 3;

	  double total_discrepancy = 0.0;
	  double color_discrepancy = 0.0;
	  double bright_discrepancy = 0.0;
	  double p_ray_samples = 100.0;
	  {
	    GaussianMapCell rayCell;
	    rayCell.red.samples = p_ray_samples;
	    rayCell.green.samples = p_ray_samples;
	    rayCell.blue.samples = p_ray_samples;
	    rayCell.red.mu = input[base_idx + 2]; 
	    rayCell.green.mu = input[base_idx + 1]; 
	    rayCell.blue.mu = input[base_idx + 0];
	    rayCell.red.sigmasquared = 0;
	    rayCell.green.sigmasquared = 0;
	    rayCell.blue.sigmasquared = 0;

	    double rmu_diff = 0.0;
	    double gmu_diff = 0.0;
	    double bmu_diff = 0.0;

	    double discrepancy_value;
	    if (ms->config.discrepancyMode == DISCREPANCY_POINT) {
	      discrepancy_value = lm->pointDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_DOT) {
	      discrepancy_value = lm->innerProduct(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_OR) {
	      discrepancy_value = lm->noisyOrDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_AND) {
	      discrepancy_value = lm->noisyAndDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else {
	      cout << "Invalid discrepancy mode: " << ms->config.discrepancyMode << endl;
	      assert(0);
	    }
      
	    total_discrepancy = 1.0 - discrepancy_value;
	  } 
	  bright_discrepancy = total_discrepancy;
	  color_discrepancy = total_discrepancy;

	  /*
	  {
	    color_discrepancy = 
	      ( (input[base_idx + 2] - lm->red.mu)*(input[base_idx + 2] - lm->red.mu) +
	      (input[base_idx + 1] - lm->green.mu)*(input[base_idx + 1] - lm->green.mu) ) / 2.0
	    ;
	    bright_discrepancy = 
	      (input[base_idx + 0] - lm->blue.mu)*(input[base_idx + 0] - lm->blue.mu)
	    ;
	  }
	  */

	  if (color_discrepancy  < color_cast_thresh) {
	    //cout << "SK: " << color_discrepancy << " " ;
	    continue;
	  }
	  if (bright_discrepancy  < bright_cast_thresh) {
	    //cout << "SK: " << color_discrepancy << " " ;
	    continue;
	  }

          
          if ( (abhr > 0) && (abhc > 0) ) {
            if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
                 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
              continue;
            } 
          } 
          
          double x, y;
          pixelToGlobalFromCache(ms, px, py, &x, &y, &data);
          
	  // single sample update
	  int i, j;
	  maps[thread]->metersToCell(x, y, &i, &j);
	  GaussianMapCell * cell = maps[thread]->refAtCell(i, j);
	  int ri, rj;
	  ms->config.gaussian_map_register->metersToCell(x, y, &ri, &rj);
	  GaussianMapCell * register_cell = ms->config.gaussian_map_register->refAtCell(ri, rj);
	  
	  if (register_cell == NULL) {
	    continue;
	  }
	  if (z_to_use > register_cell->z.mu) {
	    //cout << "SK: " << color_discrepancy << " " ;
	    continue;
	  }
	  
	  if (cell != NULL) {
	    cell->newObservation(input[base_idx + 2], input[base_idx + 1], input[base_idx + 0], z);
	    numPixels++;
	  }
        }
      }
    }
  }
  
  #pragma omp for
  for (int y = 0; y < ms->config.scene->observed_map->height; y++) {
    for (int x = 0; x < ms->config.scene->observed_map->width; x++) {
      for (int thread = 0; thread < numThreads; thread++) {
        if (maps[thread]->refAtCell(x, y)->red.samples > 0) {
          ms->config.scene->observed_map->refAtCell(x, y)->addC(maps[thread]->refAtCell(x, y));
        }
      }
    }
  }

}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcAllLightModelRenderWithoutGlareRising)

WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcAllSelfModelRenderGlareOnly)
virtual void execute(MachineState * ms) {
  double bright_cast_thresh = 0.0;
  GET_NUMERIC_ARG(ms, bright_cast_thresh);

  double color_cast_thresh = 0.0;
  GET_NUMERIC_ARG(ms, color_cast_thresh);

  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);

  int stride = 1;
  GET_INT_ARG(ms, stride);



  Size sz = ms->config.wristViewImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 
  
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  Mat gripperMask = camera->gripperMask;
  if (isSketchyMat(gripperMask)) {
    CONSOLE_ERROR(ms, "Gripper mask is messed up.");
    return;
  }

  int numThreads = 8;


  vector<shared_ptr<GaussianMap> > maps;
  maps.resize(numThreads);


  #pragma omp parallel for
  for (int thread = 0; thread < numThreads; thread++) {
    
    maps[thread] = make_shared<GaussianMap>(ms, 
                                            ms->config.scene->observed_map->width, 
                                            ms->config.scene->observed_map->height, 
                                            ms->config.scene->observed_map->cell_width,
                                            ms->config.scene->observed_map->anchor_pose); 
    maps[thread]->zero();

    
    int thisStart = thread * (camera->streamImageBuffer.size()  / numThreads);
    int thisEnd = (thread + 1) * (camera->streamImageBuffer.size()  / numThreads); 
    stringstream buf;    
    buf << "thread: " << thread << " start: " << thisStart << " end: " << thisEnd << endl;
    cout << buf.str();
    for (int i = thisStart; i < thisEnd; i+=stride) {
      streamImage * tsi = camera->setIsbIdxNoLoadNoKick(i);
      
      
      
      if (tsi == NULL) {
        CONSOLE_ERROR(ms, "Stream image null.");
      }
      eePose tArmP, tBaseP;
      
      int success = 0;
      double z = z_to_use;
      if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
        success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
      } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
        success = 1;
        tArmP = ms->config.currentEEPose;
        z = ms->config.currentEEPose.pz + ms->config.currentTableZ;
      } else {
        assert(0);
      }
      
      
      if (success != 1) {
        CONSOLE_ERROR(ms, "Couldn't get stream pose.  Return code: " << success << " time: " << tsi->time);
        continue;
      }
      
      eePose transformed = tArmP.getPoseRelativeTo(ms->config.scene->anchor_pose);
      if (fabs(transformed.qz) > 0.01) {
        CONSOLE_ERROR(ms, "Not doing update because arm not vertical.");
        continue;
      }
      pixelToGlobalCache data;      
      computePixelToPlaneCache(ms, z, tArmP, ms->config.scene->anchor_pose, &data);  
      
      // there is a faster way to stride it but i am risk averse atm
      
      Mat wristViewYCbCr = tsi->image.clone();
      
      cvtColor(tsi->image, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
      int numPixels = 0;
      int numNulls = 0;
      
      uchar *input = (uchar*) (wristViewYCbCr.data);
      
      for (int py = topy; py <= boty; py++) {
        uchar* gripperMaskPixel = camera->gripperMask.ptr<uchar>(py); // point to first pixel in row
        for (int px = topx; px <= botx; px++) {
          if (gripperMaskPixel[px] == 0) {
            continue;
          }

	  // measure discrepancy with lighting model and throw out this pixel if it
	  // is too much like the overhead light.
	  int base_idx = py * wristViewYCbCr.cols*3 + px * 3;
          
          if ( (abhr > 0) && (abhc > 0) ) {
            if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
                 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
              continue;
            } 
          } 
          
          double x, y;
          pixelToGlobalFromCache(ms, px, py, &x, &y, &data);
          
	  // single sample update
	  int i, j;
	  maps[thread]->metersToCell(x, y, &i, &j);
	  GaussianMapCell * cell = maps[thread]->refAtCell(i, j);
	  int ri, rj;
	  ms->config.gaussian_map_register->metersToCell(x, y, &ri, &rj);
	  GaussianMapCell * register_cell = ms->config.gaussian_map_register->refAtCell(ri, rj);

	  if (register_cell == NULL) {
	    continue;
	  }
	  if (z_to_use > register_cell->z.mu) {
	    //cout << "SK: " << color_discrepancy << " " ;
	    continue;
	  }

	  // use the cell this will hit as the lighting model
	  GaussianMapCell * lm = register_cell;
	  double total_discrepancy = 0.0;
	  double color_discrepancy = 0.0;
	  double bright_discrepancy = 0.0;
	  double p_ray_samples = 100.0;
	  {
	    GaussianMapCell rayCell;
	    rayCell.red.samples = p_ray_samples;
	    rayCell.green.samples = p_ray_samples;
	    rayCell.blue.samples = p_ray_samples;
	    rayCell.red.mu = input[base_idx + 2]; 
	    rayCell.green.mu = input[base_idx + 1]; 
	    rayCell.blue.mu = input[base_idx + 0];
	    rayCell.red.sigmasquared = 0;
	    rayCell.green.sigmasquared = 0;
	    rayCell.blue.sigmasquared = 0;

	    double rmu_diff = 0.0;
	    double gmu_diff = 0.0;
	    double bmu_diff = 0.0;

	    double discrepancy_value;
	    if (ms->config.discrepancyMode == DISCREPANCY_POINT) {
	      discrepancy_value = lm->pointDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_DOT) {
	      discrepancy_value = lm->innerProduct(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_OR) {
	      discrepancy_value = lm->noisyOrDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_AND) {
	      discrepancy_value = lm->noisyAndDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else {
	      cout << "Invalid discrepancy mode: " << ms->config.discrepancyMode << endl;
	      assert(0);
	    }
      
	    total_discrepancy = 1.0 - discrepancy_value;
	  } 
	  bright_discrepancy = total_discrepancy;
	  color_discrepancy = total_discrepancy;

	  /*
	  {
	    color_discrepancy = 
	      ( (input[base_idx + 2] - lm->red.mu)*(input[base_idx + 2] - lm->red.mu) +
	      (input[base_idx + 1] - lm->green.mu)*(input[base_idx + 1] - lm->green.mu) ) / 2.0
	    ;
	    bright_discrepancy = 
	      (input[base_idx + 0] - lm->blue.mu)*(input[base_idx + 0] - lm->blue.mu)
	    ;
	  }
	  */

	  if (color_discrepancy  > color_cast_thresh) {
	    //cout << "SK: " << color_discrepancy << " " ;
	    continue;
	  }
	  if (bright_discrepancy  > bright_cast_thresh) {
	    //cout << "SK: " << color_discrepancy << " " ;
	    continue;
	  }
	  
	  if (cell != NULL) {
	    cell->newObservation(input[base_idx + 2], input[base_idx + 1], input[base_idx + 0], z);
	    numPixels++;
	  }
        }
      }
    }
  }
  
  #pragma omp for
  for (int y = 0; y < ms->config.scene->observed_map->height; y++) {
    for (int x = 0; x < ms->config.scene->observed_map->width; x++) {
      for (int thread = 0; thread < numThreads; thread++) {
        if (maps[thread]->refAtCell(x, y)->red.samples > 0) {
          ms->config.scene->observed_map->refAtCell(x, y)->addC(maps[thread]->refAtCell(x, y));
        }
      }
    }
  }

}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcAllSelfModelRenderGlareOnly)

WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcAllSelfModelRenderWithoutGlare)
virtual void execute(MachineState * ms) {
  double bright_cast_thresh = 0.0;
  GET_NUMERIC_ARG(ms, bright_cast_thresh);

  double color_cast_thresh = 0.0;
  GET_NUMERIC_ARG(ms, color_cast_thresh);

  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);

  int stride = 1;
  GET_INT_ARG(ms, stride);



  Size sz = ms->config.wristViewImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 
  
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  Mat gripperMask = camera->gripperMask;
  if (isSketchyMat(gripperMask)) {
    CONSOLE_ERROR(ms, "Gripper mask is messed up.");
    return;
  }

  int numThreads = 8;


  vector<shared_ptr<GaussianMap> > maps;
  maps.resize(numThreads);

  GaussianMapCell * lm = &(ms->config.scene->light_model);

  #pragma omp parallel for
  for (int thread = 0; thread < numThreads; thread++) {
    
    maps[thread] = make_shared<GaussianMap>(ms, 
                                            ms->config.scene->observed_map->width, 
                                            ms->config.scene->observed_map->height, 
                                            ms->config.scene->observed_map->cell_width,
                                            ms->config.scene->observed_map->anchor_pose); 
    maps[thread]->zero();

    
    int thisStart = thread * (camera->streamImageBuffer.size()  / numThreads);
    int thisEnd = (thread + 1) * (camera->streamImageBuffer.size()  / numThreads); 
    stringstream buf;    
    buf << "thread: " << thread << " start: " << thisStart << " end: " << thisEnd << endl;
    cout << buf.str();
    for (int i = thisStart; i < thisEnd; i+=stride) {
      streamImage * tsi = camera->setIsbIdxNoLoadNoKick(i);
      
      
      
      if (tsi == NULL) {
        CONSOLE_ERROR(ms, "Stream image null.");
      }
      eePose tArmP, tBaseP;
      
      int success = 0;
      double z = z_to_use;
      if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
        success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
      } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
        success = 1;
        tArmP = ms->config.currentEEPose;
        z = ms->config.currentEEPose.pz + ms->config.currentTableZ;
      } else {
        assert(0);
      }
      
      
      if (success != 1) {
        CONSOLE_ERROR(ms, "Couldn't get stream pose.  Return code: " << success << " time: " << tsi->time);
        continue;
      }
      
      eePose transformed = tArmP.getPoseRelativeTo(ms->config.scene->anchor_pose);
      if (fabs(transformed.qz) > 0.01) {
        CONSOLE_ERROR(ms, "Not doing update because arm not vertical.");
        continue;
      }
      pixelToGlobalCache data;      
      computePixelToPlaneCache(ms, z, tArmP, ms->config.scene->anchor_pose, &data);  
      
      // there is a faster way to stride it but i am risk averse atm
      
      Mat wristViewYCbCr = tsi->image.clone();
      
      cvtColor(tsi->image, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
      int numPixels = 0;
      int numNulls = 0;
      
      uchar *input = (uchar*) (wristViewYCbCr.data);
      
      for (int py = topy; py <= boty; py++) {
        uchar* gripperMaskPixel = camera->gripperMask.ptr<uchar>(py); // point to first pixel in row
        for (int px = topx; px <= botx; px++) {
          if (gripperMaskPixel[px] == 0) {
            continue;
          }

	  // measure discrepancy with lighting model and throw out this pixel if it
	  // is too much like the overhead light.
	  int base_idx = py * wristViewYCbCr.cols*3 + px * 3;

          if ( (abhr > 0) && (abhc > 0) ) {
            if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
                 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
              continue;
            } 
          } 
          
          double x, y;
          pixelToGlobalFromCache(ms, px, py, &x, &y, &data);
          
	  // single sample update
	  int i, j;
	  maps[thread]->metersToCell(x, y, &i, &j);
	  GaussianMapCell * cell = maps[thread]->refAtCell(i, j);
	  int ri, rj;
	  ms->config.gaussian_map_register->metersToCell(x, y, &ri, &rj);
	  GaussianMapCell * register_cell = ms->config.gaussian_map_register->refAtCell(ri, rj);
	  
	  if (register_cell == NULL) {
	    continue;
	  }
	  if (z_to_use > register_cell->z.mu) {
	    //cout << "SK: " << color_discrepancy << " " ;
	    continue;
	  }

	  // use the cell this will hit as the lighting model
	  GaussianMapCell * lm = register_cell;
	  double total_discrepancy = 0.0;
	  double color_discrepancy = 0.0;
	  double bright_discrepancy = 0.0;
	  double p_ray_samples = 100.0;
	  {
	    GaussianMapCell rayCell;
	    rayCell.red.samples = p_ray_samples;
	    rayCell.green.samples = p_ray_samples;
	    rayCell.blue.samples = p_ray_samples;
	    rayCell.red.mu = input[base_idx + 2]; 
	    rayCell.green.mu = input[base_idx + 1]; 
	    rayCell.blue.mu = input[base_idx + 0];
	    rayCell.red.sigmasquared = 0;
	    rayCell.green.sigmasquared = 0;
	    rayCell.blue.sigmasquared = 0;

	    double rmu_diff = 0.0;
	    double gmu_diff = 0.0;
	    double bmu_diff = 0.0;

	    double discrepancy_value;
	    if (ms->config.discrepancyMode == DISCREPANCY_POINT) {
	      discrepancy_value = lm->pointDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_DOT) {
	      discrepancy_value = lm->innerProduct(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_OR) {
	      discrepancy_value = lm->noisyOrDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_AND) {
	      discrepancy_value = lm->noisyAndDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	    } else {
	      cout << "Invalid discrepancy mode: " << ms->config.discrepancyMode << endl;
	      assert(0);
	    }
      
	    total_discrepancy = 1.0 - discrepancy_value;
	  } 
	  bright_discrepancy = total_discrepancy;
	  color_discrepancy = total_discrepancy;

	  /*
	  {
	    color_discrepancy = 
	      ( (input[base_idx + 2] - lm->red.mu)*(input[base_idx + 2] - lm->red.mu) +
	      (input[base_idx + 1] - lm->green.mu)*(input[base_idx + 1] - lm->green.mu) ) / 2.0
	    ;
	    bright_discrepancy = 
	      (input[base_idx + 0] - lm->blue.mu)*(input[base_idx + 0] - lm->blue.mu)
	    ;
	  }
	  */

	  if (color_discrepancy  > color_cast_thresh) {
	    //cout << "SK: " << color_discrepancy << " " ;
	    continue;
	  }
	  if (bright_discrepancy  > bright_cast_thresh) {
	    //cout << "SK: " << color_discrepancy << " " ;
	    continue;
	  }
	  
	  
	  if (cell != NULL) {
	    cell->newObservation(input[base_idx + 2], input[base_idx + 1], input[base_idx + 0], z);
	    numPixels++;
	  }
        }
      }
    }
  }
  
  #pragma omp for
  for (int y = 0; y < ms->config.scene->observed_map->height; y++) {
    for (int x = 0; x < ms->config.scene->observed_map->width; x++) {
      for (int thread = 0; thread < numThreads; thread++) {
        if (maps[thread]->refAtCell(x, y)->red.samples > 0) {
          ms->config.scene->observed_map->refAtCell(x, y)->addC(maps[thread]->refAtCell(x, y));
        }
      }
    }
  }

}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcAllSelfModelRenderWithoutGlare)

WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcViewSynthAll)
virtual void execute(MachineState * ms) {
// XXX not really started yet

  // distance of focal plane from anchor pose.
  double z_focal_plane = 0.388;
  GET_NUMERIC_ARG(ms, z_focal_plane);

  // distance of aperture plane from anchor pose.
  double z_aperture_plane = 0.01;
  GET_NUMERIC_ARG(ms, z_aperture_plane);

  // width in meters of circular global aperture
  double global_aperture_width_meters = 0.01;
  GET_NUMERIC_ARG(ms, global_aperture_width_meters);
  double global_aperture_radius_meters = global_aperture_width_meters / 2.0;
  double global_aperture_radius_meters_squared = global_aperture_radius_meters * global_aperture_radius_meters;
  

  int stride = 1;
  GET_INT_ARG(ms, stride);


  Size sz = ms->config.wristViewImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 

  cout << "viewSynth fp ap awm str: " << z_focal_plane << " " << z_aperture_plane << " " << global_aperture_width_meters << " " << stride << endl;
  
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  Mat gripperMask = camera->gripperMask;
  if (isSketchyMat(gripperMask)) {
    CONSOLE_ERROR(ms, "Gripper mask is messed up.");
    return;
  }


  int numThreads = 8;

  vector<shared_ptr<GaussianMap> > maps;
  maps.resize(numThreads);

  #pragma omp parallel for
  for (int thread = 0; thread < numThreads; thread++) {
    
    maps[thread] = make_shared<GaussianMap>(ms, 
                                            ms->config.scene->observed_map->width, 
                                            ms->config.scene->observed_map->height, 
                                            ms->config.scene->observed_map->cell_width,
                                            ms->config.scene->observed_map->anchor_pose); 
    maps[thread]->zero();

    
    int thisStart = thread * (camera->streamImageBuffer.size()  / numThreads);
    int thisEnd = (thread + 1) * (camera->streamImageBuffer.size()  / numThreads); 
    stringstream buf;    
    buf << "thread: " << thread << " start: " << thisStart << " end: " << thisEnd << endl;
    cout << buf.str();
    for (int i = thisStart; i < thisEnd; i+=stride) {
      //streamImage * tsi = setIsbIdxNoLoadNoKick(ms, i);
      streamImage * tsi = camera->getIsbIdxNoLoadNoKick(i);
      
      
      
      if (tsi == NULL) {
        CONSOLE_ERROR(ms, "Stream image null.");
      }
      eePose tArmP, tBaseP;
      
      int success = 0;
      // XXX
      double z = z_focal_plane;
      if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
        //success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
        success = ms->getStreamPoseAtTimeThreadSafe(tsi->time, &tArmP, &tBaseP);
      } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
        success = 1;
        tArmP = ms->config.currentEEPose;
        z = ms->config.currentEEPose.pz + ms->config.currentTableZ;
      } else {
        assert(0);
      }


      
      
      if (success != 1) {
        CONSOLE_ERROR(ms, "Couldn't get stream pose: " << success << " time: " << tsi->time);
        continue;
      }


      /*
      eePose focal_plane_anchor_pose = eePose::identity();
      focal_plane_anchor_pose.pz = -z_focal_plane;
      focal_plane_anchor_pose = focal_plane_anchor_pose.applyAsRelativePoseTo(ms->config.scene->anchor_pose);
      */
      
      eePose transformed = tArmP.getPoseRelativeTo(ms->config.scene->anchor_pose);
      if (fabs(transformed.qz) > 0.01) {
        CONSOLE_ERROR(ms, "  Not doing update because arm not vertical.");
        continue;
      }

      // XXX
      // get the pose of the image in the anchor pose frame,
      //  subtract the depth from the achnor pose focal depth.
      // if difference is positive, continue and use that z for casting
      //  but anchor focal depth as z observation
      // but the anchor pose canonically points backwards relative to the end effector that generated it
      //  so be careful with the sign
      double fp_anchor_z = -z_focal_plane;
      double ap_anchor_z = -z_aperture_plane;
      double f_cast_z = -(fp_anchor_z - transformed.pz);
      double a_cast_z = -(ap_anchor_z - transformed.pz);
      cout << transformed.pz << " " << f_cast_z << " " << a_cast_z << endl;
      if ( (f_cast_z == 0.0) || (a_cast_z == 0.0) ) {
        CONSOLE_ERROR(ms, "  Not doing update because height singular.");
	continue;
      }


      pixelToGlobalCache data_f;      
      computePixelToPlaneCache(ms, fabs(f_cast_z), tArmP, ms->config.scene->anchor_pose, &data_f);  

      double f_bc_cast_length = fabs(2.8*f_cast_z);
      pixelToGlobalCache data_2f;      
      computePixelToPlaneCache(ms, f_bc_cast_length, tArmP, ms->config.scene->anchor_pose, &data_2f);  

      pixelToGlobalCache data_a;      
      computePixelToPlaneCache(ms, fabs(a_cast_z), tArmP, ms->config.scene->anchor_pose, &data_a);  

      double a_bc_cast_length = fabs(2.8*a_cast_z);
      pixelToGlobalCache data_2a;      
      computePixelToPlaneCache(ms, a_bc_cast_length, tArmP, ms->config.scene->anchor_pose, &data_2a);  
      
      Mat wristViewYCbCr = tsi->image.clone();
      
      cvtColor(tsi->image, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
      int numPixels = 0;
      int numNulls = 0;

      int num_aperture_backcasts = 0;
      int num_focal_backcasts = 0;
      
      uchar *input = (uchar*) (wristViewYCbCr.data);
      
      for (int py = topy; py <= boty; py++) {
        uchar* gripperMaskPixel = camera->gripperMask.ptr<uchar>(py); // point to first pixel in row
        for (int px = topx; px <= botx; px++) {
          if (gripperMaskPixel[px] == 0) {
            continue;
          }
          
          if ( (abhr > 0) && (abhc > 0) ) {
            if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
                 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
              continue;
            } 
          } 

	  // XXX
	  // check to see if the ray is within the global aperture
	  //  if the start point is in front of the aperture, 
	  //  back cast it for the check
          
          double x_a, y_a;
	  if (a_cast_z > 0) {
	    pixelToGlobalFromCache(ms, px, py, &x_a, &y_a, &data_a);
	  } else {
	    //pixelToGlobalFromCacheBackCast(ms, px, py, &x_a, &y_a, &data_a);
	    num_aperture_backcasts++;
	    double x_2a, y_2a;
	    pixelToGlobalFromCache(ms, px, py, &x_a, &y_a, &data_a);
	    pixelToGlobalFromCache(ms, px, py, &x_2a, &y_2a, &data_2a);

	    x_a = x_a + 2.0*(x_a - x_2a);
	    y_a = y_a + 2.0*(y_a - y_2a);
	  } // 0 case should have been handled above
	  
	  if (x_a * x_a + y_a * y_a > global_aperture_radius_meters_squared) {
	    //cout << "  rejected: " << x_a << " " << ms->config.scene->anchor_pose.px << " " << y_a << " " << ms->config.scene->anchor_pose.py << endl;
	    continue;
	  } else {
	    //cout << "  retained: " << x_a << " " << ms->config.scene->anchor_pose.px << " " << y_a << " " << ms->config.scene->anchor_pose.py << endl;
	  } 

          double x_f, y_f;
	  if (f_cast_z > 0) {
	    pixelToGlobalFromCache(ms, px, py, &x_f, &y_f, &data_f);
	  } else {
	    //pixelToGlobalFromCacheBackCast(ms, px, py, &x_f, &y_f, &data_f);
	    num_focal_backcasts++;
	    double x_2f, y_2f;
	    pixelToGlobalFromCache(ms, px, py, &x_f, &y_f, &data_f);
	    pixelToGlobalFromCache(ms, px, py, &x_2f, &y_2f, &data_2f);

	    x_f = x_f + 2.0*(x_f - x_2f);
	    y_f = y_f + 2.0*(y_f - y_2f);
	  } // 0 case should have been handled above
          
          if (1) {
            // single sample update
            int i, j;
            maps[thread]->metersToCell(x_f, y_f, &i, &j);
            GaussianMapCell * cell = maps[thread]->refAtCell(i, j);
            
            
            if (cell != NULL) {
              int base_idx = py * wristViewYCbCr.cols*3 + px * 3;
              cell->newObservation(input[base_idx + 2], input[base_idx + 1], input[base_idx + 0], z_focal_plane);
              numPixels++;
            }
          } else {
            numNulls++;
          }
        }
      }
  
      //cout << "backcasts ap fc: " << num_aperture_backcasts << " " << num_focal_backcasts << endl;
    }
  }
  
  #pragma omp for
  for (int y = 0; y < ms->config.scene->observed_map->height; y++) {
    for (int x = 0; x < ms->config.scene->observed_map->width; x++) {
      for (int thread = 0; thread < numThreads; thread++) {
        if (maps[thread]->refAtCell(x, y)->red.samples > 0) {
          ms->config.scene->observed_map->refAtCell(x, y)->addC(maps[thread]->refAtCell(x, y));
        }
      }
    }
  }

}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcViewSynthAll)

WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcViewSynthAllGCLightModelRenderGlareOnly)
virtual void execute(MachineState * ms) {
// XXX not really started yet

  // distance of focal plane from anchor pose.
  double z_focal_plane = 0.388;
  GET_NUMERIC_ARG(ms, z_focal_plane);

  // distance of aperture plane from anchor pose.
  double z_aperture_plane = 0.01;
  GET_NUMERIC_ARG(ms, z_aperture_plane);

  // width in meters of circular global aperture
  double global_aperture_width_meters = 0.01;
  GET_NUMERIC_ARG(ms, global_aperture_width_meters);
  double global_aperture_radius_meters = global_aperture_width_meters / 2.0;
  double global_aperture_radius_meters_squared = global_aperture_radius_meters * global_aperture_radius_meters;
  

  int stride = 1;
  GET_INT_ARG(ms, stride);


  Size sz = ms->config.wristViewImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 

  cout << "viewSynth fp ap awm str: " << z_focal_plane << " " << z_aperture_plane << " " << global_aperture_width_meters << " " << stride << endl;
  
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  Mat gripperMask = camera->gripperMask;
  if (isSketchyMat(gripperMask)) {
    CONSOLE_ERROR(ms, "Gripper mask is messed up.");
    return;
  }


  int numThreads = 8;

  vector<shared_ptr<GaussianMap> > maps;
  maps.resize(numThreads);

  #pragma omp parallel for
  for (int thread = 0; thread < numThreads; thread++) {
    
    maps[thread] = make_shared<GaussianMap>(ms, 
                                            ms->config.scene->observed_map->width, 
                                            ms->config.scene->observed_map->height, 
                                            ms->config.scene->observed_map->cell_width,
                                            ms->config.scene->observed_map->anchor_pose); 
    maps[thread]->zero();

    
    int thisStart = thread * (camera->streamImageBuffer.size()  / numThreads);
    int thisEnd = (thread + 1) * (camera->streamImageBuffer.size()  / numThreads); 
    stringstream buf;    
    buf << "thread: " << thread << " start: " << thisStart << " end: " << thisEnd << endl;
    cout << buf.str();
    for (int i = thisStart; i < thisEnd; i+=stride) {
      //streamImage * tsi = setIsbIdxNoLoadNoKick(ms, i);
      streamImage * tsi = camera->getIsbIdxNoLoadNoKick(i);
      
      
      
      if (tsi == NULL) {
        CONSOLE_ERROR(ms, "Stream image null.");
      }
      eePose tArmP, tBaseP;
      
      int success = 0;
      // XXX
      double z = z_focal_plane;
      if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
        //success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
        success = ms->getStreamPoseAtTimeThreadSafe(tsi->time, &tArmP, &tBaseP);
      } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
        success = 1;
        tArmP = ms->config.currentEEPose;
        z = ms->config.currentEEPose.pz + ms->config.currentTableZ;
      } else {
        assert(0);
      }


      
      
      if (success != 1) {
        CONSOLE_ERROR(ms, "Couldn't get stream pose: " << success << " time: " << tsi->time);
        continue;
      }


      /*
      eePose focal_plane_anchor_pose = eePose::identity();
      focal_plane_anchor_pose.pz = -z_focal_plane;
      focal_plane_anchor_pose = focal_plane_anchor_pose.applyAsRelativePoseTo(ms->config.scene->anchor_pose);
      */
      
      eePose transformed = tArmP.getPoseRelativeTo(ms->config.scene->anchor_pose);
      if (fabs(transformed.qz) > 0.01) {
        CONSOLE_ERROR(ms, "  Not doing update because arm not vertical.");
        continue;
      }

      // XXX
      // get the pose of the image in the anchor pose frame,
      //  subtract the depth from the achnor pose focal depth.
      // if difference is positive, continue and use that z for casting
      //  but anchor focal depth as z observation
      // but the anchor pose canonically points backwards relative to the end effector that generated it
      //  so be careful with the sign
      double fp_anchor_z = -z_focal_plane;
      double ap_anchor_z = -z_aperture_plane;
      double f_cast_z = -(fp_anchor_z - transformed.pz);
      double a_cast_z = -(ap_anchor_z - transformed.pz);
      cout << transformed.pz << " " << f_cast_z << " " << a_cast_z << endl;
      if ( (f_cast_z == 0.0) || (a_cast_z == 0.0) ) {
        CONSOLE_ERROR(ms, "  Not doing update because height singular.");
	continue;
      }


      pixelToGlobalCache data_f;      
      computePixelToPlaneCache(ms, fabs(f_cast_z), tArmP, ms->config.scene->anchor_pose, &data_f);  

      double f_bc_cast_length = fabs(2.8*f_cast_z);
      pixelToGlobalCache data_2f;      
      computePixelToPlaneCache(ms, f_bc_cast_length, tArmP, ms->config.scene->anchor_pose, &data_2f);  

      pixelToGlobalCache data_a;      
      computePixelToPlaneCache(ms, fabs(a_cast_z), tArmP, ms->config.scene->anchor_pose, &data_a);  

      double a_bc_cast_length = fabs(2.8*a_cast_z);
      pixelToGlobalCache data_2a;      
      computePixelToPlaneCache(ms, a_bc_cast_length, tArmP, ms->config.scene->anchor_pose, &data_2a);  
      
      Mat wristViewYCbCr = tsi->image.clone();
      
      cvtColor(tsi->image, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
      int numPixels = 0;
      int numNulls = 0;

      int num_aperture_backcasts = 0;
      int num_focal_backcasts = 0;
      
      uchar *input = (uchar*) (wristViewYCbCr.data);

      GaussianMapCell * lm = &(ms->config.scene->light_model);
      
      for (int py = topy; py <= boty; py++) {
        uchar* gripperMaskPixel = camera->gripperMask.ptr<uchar>(py); // point to first pixel in row
        for (int px = topx; px <= botx; px++) {
          if (gripperMaskPixel[px] == 0) {
            continue;
          }
          
          if ( (abhr > 0) && (abhc > 0) ) {
            if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
                 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
              continue;
            } 
          } 

	  // XXX
	  // check to see if the ray is within the global aperture
	  //  if the start point is in front of the aperture, 
	  //  back cast it for the check
          
          double x_a, y_a;
	  if (a_cast_z > 0) {
	    pixelToGlobalFromCache(ms, px, py, &x_a, &y_a, &data_a);
	  } else {
	    //pixelToGlobalFromCacheBackCast(ms, px, py, &x_a, &y_a, &data_a);
	    num_aperture_backcasts++;
	    double x_2a, y_2a;
	    pixelToGlobalFromCache(ms, px, py, &x_a, &y_a, &data_a);
	    pixelToGlobalFromCache(ms, px, py, &x_2a, &y_2a, &data_2a);

	    x_a = x_a + 2.0*(x_a - x_2a);
	    y_a = y_a + 2.0*(y_a - y_2a);
	  } // 0 case should have been handled above
	  
	  if (x_a * x_a + y_a * y_a > global_aperture_radius_meters_squared) {
	    //cout << "  rejected: " << x_a << " " << ms->config.scene->anchor_pose.px << " " << y_a << " " << ms->config.scene->anchor_pose.py << endl;
	    continue;
	  } else {
	    //cout << "  retained: " << x_a << " " << ms->config.scene->anchor_pose.px << " " << y_a << " " << ms->config.scene->anchor_pose.py << endl;
	  } 

          double x_f, y_f;
	  if (f_cast_z > 0) {
	    pixelToGlobalFromCache(ms, px, py, &x_f, &y_f, &data_f);
	  } else {
	    //pixelToGlobalFromCacheBackCast(ms, px, py, &x_f, &y_f, &data_f);
	    num_focal_backcasts++;
	    double x_2f, y_2f;
	    pixelToGlobalFromCache(ms, px, py, &x_f, &y_f, &data_f);
	    pixelToGlobalFromCache(ms, px, py, &x_2f, &y_2f, &data_2f);

	    x_f = x_f + 2.0*(x_f - x_2f);
	    y_f = y_f + 2.0*(y_f - y_2f);
	  } // 0 case should have been handled above
          
          if (1) {
            // single sample update
            int i, j;
            maps[thread]->metersToCell(x_f, y_f, &i, &j);
            GaussianMapCell * cell = maps[thread]->refAtCell(i, j);
	    int base_idx = py * wristViewYCbCr.cols*3 + px * 3;
            
	    int ri, rj;
	    ms->config.gaussian_map_register->metersToCell(x_f, y_f, &ri, &rj);
	    GaussianMapCell * register_cell = ms->config.gaussian_map_register->refAtCell(ri, rj);

	    if (register_cell == NULL) {
	      continue;
	    }
	    if (z_focal_plane > register_cell->z.mu) {
	      //cout << "SK: " << color_discrepancy << " " ;
	      continue;
	    }

	    double p_color_cast_thresh = 0.5;
	    double p_bright_cast_thresh = 0.5;

	    // use the cell this will hit as the lighting model
	    double total_discrepancy = 0.0;
	    double color_discrepancy = 0.0;
	    double bright_discrepancy = 0.0;
	    double p_ray_samples = 100.0;
	    {
	      GaussianMapCell rayCell;
	      rayCell.red.samples = p_ray_samples;
	      rayCell.green.samples = p_ray_samples;
	      rayCell.blue.samples = p_ray_samples;
	      rayCell.red.mu = input[base_idx + 2]; 
	      rayCell.green.mu = input[base_idx + 1]; 
	      rayCell.blue.mu = input[base_idx + 0];
	      rayCell.red.sigmasquared = 0;
	      rayCell.green.sigmasquared = 0;
	      rayCell.blue.sigmasquared = 0;

	      double rmu_diff = 0.0;
	      double gmu_diff = 0.0;
	      double bmu_diff = 0.0;

	      double discrepancy_value;
	      if (ms->config.discrepancyMode == DISCREPANCY_POINT) {
		discrepancy_value = lm->pointDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	      } else if (ms->config.discrepancyMode == DISCREPANCY_DOT) {
		discrepancy_value = lm->innerProduct(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	      } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_OR) {
		discrepancy_value = lm->noisyOrDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	      } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_AND) {
		discrepancy_value = lm->noisyAndDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	      } else {
		cout << "Invalid discrepancy mode: " << ms->config.discrepancyMode << endl;
		assert(0);
	      }
	
	      total_discrepancy = 1.0 - discrepancy_value;
	    } 
	    bright_discrepancy = total_discrepancy;
	    color_discrepancy = total_discrepancy;

	    if (color_discrepancy  > p_color_cast_thresh) {
	      //cout << "SK: " << color_discrepancy << " " ;
	      continue;
	    }
	    if (bright_discrepancy  > p_bright_cast_thresh) {
	      //cout << "SK: " << color_discrepancy << " " ;
	      continue;
	    }
            
            if (cell != NULL) {
              cell->newObservation(input[base_idx + 2], input[base_idx + 1], input[base_idx + 0], z_focal_plane);
              numPixels++;
            }
          } else {
            numNulls++;
          }
        }
      }
  
      //cout << "backcasts ap fc: " << num_aperture_backcasts << " " << num_focal_backcasts << endl;
    }
  }
  
  #pragma omp for
  for (int y = 0; y < ms->config.scene->observed_map->height; y++) {
    for (int x = 0; x < ms->config.scene->observed_map->width; x++) {
      for (int thread = 0; thread < numThreads; thread++) {
        if (maps[thread]->refAtCell(x, y)->red.samples > 0) {
          ms->config.scene->observed_map->refAtCell(x, y)->addC(maps[thread]->refAtCell(x, y));
        }
      }
    }
  }

}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcViewSynthAllGCLightModelRenderGlareOnly)

WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcViewSynthAllGCLightModelRenderWithoutGlare)
virtual void execute(MachineState * ms) {
// XXX not really started yet

  // distance of focal plane from anchor pose.
  double z_focal_plane = 0.388;
  GET_NUMERIC_ARG(ms, z_focal_plane);

  // distance of aperture plane from anchor pose.
  double z_aperture_plane = 0.01;
  GET_NUMERIC_ARG(ms, z_aperture_plane);

  // width in meters of circular global aperture
  double global_aperture_width_meters = 0.01;
  GET_NUMERIC_ARG(ms, global_aperture_width_meters);
  double global_aperture_radius_meters = global_aperture_width_meters / 2.0;
  double global_aperture_radius_meters_squared = global_aperture_radius_meters * global_aperture_radius_meters;
  

  int stride = 1;
  GET_INT_ARG(ms, stride);


  Size sz = ms->config.wristViewImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 

  cout << "viewSynth fp ap awm str: " << z_focal_plane << " " << z_aperture_plane << " " << global_aperture_width_meters << " " << stride << endl;
  
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  Mat gripperMask = camera->gripperMask;
  if (isSketchyMat(gripperMask)) {
    CONSOLE_ERROR(ms, "Gripper mask is messed up.");
    return;
  }


  int numThreads = 8;

  vector<shared_ptr<GaussianMap> > maps;
  maps.resize(numThreads);

  #pragma omp parallel for
  for (int thread = 0; thread < numThreads; thread++) {
    
    maps[thread] = make_shared<GaussianMap>(ms, 
                                            ms->config.scene->observed_map->width, 
                                            ms->config.scene->observed_map->height, 
                                            ms->config.scene->observed_map->cell_width,
                                            ms->config.scene->observed_map->anchor_pose); 
    maps[thread]->zero();

    
    int thisStart = thread * (camera->streamImageBuffer.size()  / numThreads);
    int thisEnd = (thread + 1) * (camera->streamImageBuffer.size()  / numThreads); 
    stringstream buf;    
    buf << "thread: " << thread << " start: " << thisStart << " end: " << thisEnd << endl;
    cout << buf.str();
    for (int i = thisStart; i < thisEnd; i+=stride) {
      //streamImage * tsi = setIsbIdxNoLoadNoKick(ms, i);
      streamImage * tsi = camera->getIsbIdxNoLoadNoKick(i);
      
      
      
      if (tsi == NULL) {
        CONSOLE_ERROR(ms, "Stream image null.");
      }
      eePose tArmP, tBaseP;
      
      int success = 0;
      // XXX
      double z = z_focal_plane;
      if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
        //success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
        success = ms->getStreamPoseAtTimeThreadSafe(tsi->time, &tArmP, &tBaseP);
      } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
        success = 1;
        tArmP = ms->config.currentEEPose;
        z = ms->config.currentEEPose.pz + ms->config.currentTableZ;
      } else {
        assert(0);
      }


      
      
      if (success != 1) {
        CONSOLE_ERROR(ms, "Couldn't get stream pose: " << success << " time: " << tsi->time);
        continue;
      }


      /*
      eePose focal_plane_anchor_pose = eePose::identity();
      focal_plane_anchor_pose.pz = -z_focal_plane;
      focal_plane_anchor_pose = focal_plane_anchor_pose.applyAsRelativePoseTo(ms->config.scene->anchor_pose);
      */
      
      eePose transformed = tArmP.getPoseRelativeTo(ms->config.scene->anchor_pose);
      if (fabs(transformed.qz) > 0.01) {
        CONSOLE_ERROR(ms, "  Not doing update because arm not vertical.");
        continue;
      }

      // XXX
      // get the pose of the image in the anchor pose frame,
      //  subtract the depth from the achnor pose focal depth.
      // if difference is positive, continue and use that z for casting
      //  but anchor focal depth as z observation
      // but the anchor pose canonically points backwards relative to the end effector that generated it
      //  so be careful with the sign
      double fp_anchor_z = -z_focal_plane;
      double ap_anchor_z = -z_aperture_plane;
      double f_cast_z = -(fp_anchor_z - transformed.pz);
      double a_cast_z = -(ap_anchor_z - transformed.pz);
      cout << transformed.pz << " " << f_cast_z << " " << a_cast_z << endl;
      if ( (f_cast_z == 0.0) || (a_cast_z == 0.0) ) {
        CONSOLE_ERROR(ms, "  Not doing update because height singular.");
	continue;
      }


      pixelToGlobalCache data_f;      
      computePixelToPlaneCache(ms, fabs(f_cast_z), tArmP, ms->config.scene->anchor_pose, &data_f);  

      double f_bc_cast_length = fabs(2.8*f_cast_z);
      pixelToGlobalCache data_2f;      
      computePixelToPlaneCache(ms, f_bc_cast_length, tArmP, ms->config.scene->anchor_pose, &data_2f);  

      pixelToGlobalCache data_a;      
      computePixelToPlaneCache(ms, fabs(a_cast_z), tArmP, ms->config.scene->anchor_pose, &data_a);  

      double a_bc_cast_length = fabs(2.8*a_cast_z);
      pixelToGlobalCache data_2a;      
      computePixelToPlaneCache(ms, a_bc_cast_length, tArmP, ms->config.scene->anchor_pose, &data_2a);  
      
      Mat wristViewYCbCr = tsi->image.clone();
      
      cvtColor(tsi->image, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
      int numPixels = 0;
      int numNulls = 0;

      int num_aperture_backcasts = 0;
      int num_focal_backcasts = 0;
      
      uchar *input = (uchar*) (wristViewYCbCr.data);

      GaussianMapCell * lm = &(ms->config.scene->light_model);
      
      for (int py = topy; py <= boty; py++) {
        uchar* gripperMaskPixel = camera->gripperMask.ptr<uchar>(py); // point to first pixel in row
        for (int px = topx; px <= botx; px++) {
          if (gripperMaskPixel[px] == 0) {
            continue;
          }
          
          if ( (abhr > 0) && (abhc > 0) ) {
            if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
                 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
              continue;
            } 
          } 

	  // XXX
	  // check to see if the ray is within the global aperture
	  //  if the start point is in front of the aperture, 
	  //  back cast it for the check
          
          double x_a, y_a;
	  if (a_cast_z > 0) {
	    pixelToGlobalFromCache(ms, px, py, &x_a, &y_a, &data_a);
	  } else {
	    //pixelToGlobalFromCacheBackCast(ms, px, py, &x_a, &y_a, &data_a);
	    num_aperture_backcasts++;
	    double x_2a, y_2a;
	    pixelToGlobalFromCache(ms, px, py, &x_a, &y_a, &data_a);
	    pixelToGlobalFromCache(ms, px, py, &x_2a, &y_2a, &data_2a);

	    x_a = x_a + 2.0*(x_a - x_2a);
	    y_a = y_a + 2.0*(y_a - y_2a);
	  } // 0 case should have been handled above
	  
	  if (x_a * x_a + y_a * y_a > global_aperture_radius_meters_squared) {
	    //cout << "  rejected: " << x_a << " " << ms->config.scene->anchor_pose.px << " " << y_a << " " << ms->config.scene->anchor_pose.py << endl;
	    continue;
	  } else {
	    //cout << "  retained: " << x_a << " " << ms->config.scene->anchor_pose.px << " " << y_a << " " << ms->config.scene->anchor_pose.py << endl;
	  } 

          double x_f, y_f;
	  if (f_cast_z > 0) {
	    pixelToGlobalFromCache(ms, px, py, &x_f, &y_f, &data_f);
	  } else {
	    //pixelToGlobalFromCacheBackCast(ms, px, py, &x_f, &y_f, &data_f);
	    num_focal_backcasts++;
	    double x_2f, y_2f;
	    pixelToGlobalFromCache(ms, px, py, &x_f, &y_f, &data_f);
	    pixelToGlobalFromCache(ms, px, py, &x_2f, &y_2f, &data_2f);

	    x_f = x_f + 2.0*(x_f - x_2f);
	    y_f = y_f + 2.0*(y_f - y_2f);
	  } // 0 case should have been handled above
          
          if (1) {
            // single sample update
            int i, j;
            maps[thread]->metersToCell(x_f, y_f, &i, &j);
            GaussianMapCell * cell = maps[thread]->refAtCell(i, j);
	    int base_idx = py * wristViewYCbCr.cols*3 + px * 3;
            
	    int ri, rj;
	    ms->config.gaussian_map_register->metersToCell(x_f, y_f, &ri, &rj);
	    GaussianMapCell * register_cell = ms->config.gaussian_map_register->refAtCell(ri, rj);

	    if (register_cell == NULL) {
	      continue;
	    }
	    if (z_focal_plane > register_cell->z.mu) {
	      //cout << "SK: " << color_discrepancy << " " ;
	      continue;
	    }

	    double p_color_cast_thresh = 0.5;
	    double p_bright_cast_thresh = 0.5;

	    // use the cell this will hit as the lighting model
	    double total_discrepancy = 0.0;
	    double color_discrepancy = 0.0;
	    double bright_discrepancy = 0.0;
	    double p_ray_samples = 100.0;
	    {
	      GaussianMapCell rayCell;
	      rayCell.red.samples = p_ray_samples;
	      rayCell.green.samples = p_ray_samples;
	      rayCell.blue.samples = p_ray_samples;
	      rayCell.red.mu = input[base_idx + 2]; 
	      rayCell.green.mu = input[base_idx + 1]; 
	      rayCell.blue.mu = input[base_idx + 0];
	      rayCell.red.sigmasquared = 0;
	      rayCell.green.sigmasquared = 0;
	      rayCell.blue.sigmasquared = 0;

	      double rmu_diff = 0.0;
	      double gmu_diff = 0.0;
	      double bmu_diff = 0.0;

	      double discrepancy_value;
	      if (ms->config.discrepancyMode == DISCREPANCY_POINT) {
		discrepancy_value = lm->pointDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	      } else if (ms->config.discrepancyMode == DISCREPANCY_DOT) {
		discrepancy_value = lm->innerProduct(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	      } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_OR) {
		discrepancy_value = lm->noisyOrDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	      } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_AND) {
		discrepancy_value = lm->noisyAndDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	      } else {
		cout << "Invalid discrepancy mode: " << ms->config.discrepancyMode << endl;
		assert(0);
	      }
	
	      total_discrepancy = 1.0 - discrepancy_value;
	    } 
	    bright_discrepancy = total_discrepancy;
	    color_discrepancy = total_discrepancy;

	    if (color_discrepancy  < p_color_cast_thresh) {
	      //cout << "SK: " << color_discrepancy << " " ;
	      continue;
	    }
	    if (bright_discrepancy  < p_bright_cast_thresh) {
	      //cout << "SK: " << color_discrepancy << " " ;
	      continue;
	    }
            
            if (cell != NULL) {
              cell->newObservation(input[base_idx + 2], input[base_idx + 1], input[base_idx + 0], z_focal_plane);
              numPixels++;
            }
          } else {
            numNulls++;
          }
        }
      }
  
      //cout << "backcasts ap fc: " << num_aperture_backcasts << " " << num_focal_backcasts << endl;
    }
  }
  
  #pragma omp for
  for (int y = 0; y < ms->config.scene->observed_map->height; y++) {
    for (int x = 0; x < ms->config.scene->observed_map->width; x++) {
      for (int thread = 0; thread < numThreads; thread++) {
        if (maps[thread]->refAtCell(x, y)->red.samples > 0) {
          ms->config.scene->observed_map->refAtCell(x, y)->addC(maps[thread]->refAtCell(x, y));
        }
      }
    }
  }

}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcViewSynthAllGCLightModelRenderWithoutGlare)

WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcViewSynthAllGCSelfRenderGlareOnly)
virtual void execute(MachineState * ms) {
// XXX not really started yet

  // distance of focal plane from anchor pose.
  double z_focal_plane = 0.388;
  GET_NUMERIC_ARG(ms, z_focal_plane);

  // distance of aperture plane from anchor pose.
  double z_aperture_plane = 0.01;
  GET_NUMERIC_ARG(ms, z_aperture_plane);

  // width in meters of circular global aperture
  double global_aperture_width_meters = 0.01;
  GET_NUMERIC_ARG(ms, global_aperture_width_meters);
  double global_aperture_radius_meters = global_aperture_width_meters / 2.0;
  double global_aperture_radius_meters_squared = global_aperture_radius_meters * global_aperture_radius_meters;
  

  int stride = 1;
  GET_INT_ARG(ms, stride);


  Size sz = ms->config.wristViewImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 

  cout << "viewSynth fp ap awm str: " << z_focal_plane << " " << z_aperture_plane << " " << global_aperture_width_meters << " " << stride << endl;
  
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  Mat gripperMask = camera->gripperMask;
  if (isSketchyMat(gripperMask)) {
    CONSOLE_ERROR(ms, "Gripper mask is messed up.");
    return;
  }


  int numThreads = 8;

  vector<shared_ptr<GaussianMap> > maps;
  maps.resize(numThreads);

  #pragma omp parallel for
  for (int thread = 0; thread < numThreads; thread++) {
    
    maps[thread] = make_shared<GaussianMap>(ms, 
                                            ms->config.scene->observed_map->width, 
                                            ms->config.scene->observed_map->height, 
                                            ms->config.scene->observed_map->cell_width,
                                            ms->config.scene->observed_map->anchor_pose); 
    maps[thread]->zero();

    
    int thisStart = thread * (camera->streamImageBuffer.size()  / numThreads);
    int thisEnd = (thread + 1) * (camera->streamImageBuffer.size()  / numThreads); 
    stringstream buf;    
    buf << "thread: " << thread << " start: " << thisStart << " end: " << thisEnd << endl;
    cout << buf.str();
    for (int i = thisStart; i < thisEnd; i+=stride) {
      //streamImage * tsi = setIsbIdxNoLoadNoKick(ms, i);
      streamImage * tsi = camera->getIsbIdxNoLoadNoKick(i);
      
      
      
      if (tsi == NULL) {
        CONSOLE_ERROR(ms, "Stream image null.");
      }
      eePose tArmP, tBaseP;
      
      int success = 0;
      // XXX
      double z = z_focal_plane;
      if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
        //success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
        success = ms->getStreamPoseAtTimeThreadSafe(tsi->time, &tArmP, &tBaseP);
      } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
        success = 1;
        tArmP = ms->config.currentEEPose;
        z = ms->config.currentEEPose.pz + ms->config.currentTableZ;
      } else {
        assert(0);
      }


      
      
      if (success != 1) {
        CONSOLE_ERROR(ms, "Couldn't get stream pose: " << success << " time: " << tsi->time);
        continue;
      }


      /*
      eePose focal_plane_anchor_pose = eePose::identity();
      focal_plane_anchor_pose.pz = -z_focal_plane;
      focal_plane_anchor_pose = focal_plane_anchor_pose.applyAsRelativePoseTo(ms->config.scene->anchor_pose);
      */
      
      eePose transformed = tArmP.getPoseRelativeTo(ms->config.scene->anchor_pose);
      if (fabs(transformed.qz) > 0.01) {
        CONSOLE_ERROR(ms, "  Not doing update because arm not vertical.");
        continue;
      }

      // XXX
      // get the pose of the image in the anchor pose frame,
      //  subtract the depth from the achnor pose focal depth.
      // if difference is positive, continue and use that z for casting
      //  but anchor focal depth as z observation
      // but the anchor pose canonically points backwards relative to the end effector that generated it
      //  so be careful with the sign
      double fp_anchor_z = -z_focal_plane;
      double ap_anchor_z = -z_aperture_plane;
      double f_cast_z = -(fp_anchor_z - transformed.pz);
      double a_cast_z = -(ap_anchor_z - transformed.pz);
      cout << transformed.pz << " " << f_cast_z << " " << a_cast_z << endl;
      if ( (f_cast_z == 0.0) || (a_cast_z == 0.0) ) {
        CONSOLE_ERROR(ms, "  Not doing update because height singular.");
	continue;
      }


      pixelToGlobalCache data_f;      
      computePixelToPlaneCache(ms, fabs(f_cast_z), tArmP, ms->config.scene->anchor_pose, &data_f);  

      double f_bc_cast_length = fabs(2.8*f_cast_z);
      pixelToGlobalCache data_2f;      
      computePixelToPlaneCache(ms, f_bc_cast_length, tArmP, ms->config.scene->anchor_pose, &data_2f);  

      pixelToGlobalCache data_a;      
      computePixelToPlaneCache(ms, fabs(a_cast_z), tArmP, ms->config.scene->anchor_pose, &data_a);  

      double a_bc_cast_length = fabs(2.8*a_cast_z);
      pixelToGlobalCache data_2a;      
      computePixelToPlaneCache(ms, a_bc_cast_length, tArmP, ms->config.scene->anchor_pose, &data_2a);  
      
      Mat wristViewYCbCr = tsi->image.clone();
      
      cvtColor(tsi->image, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
      int numPixels = 0;
      int numNulls = 0;

      int num_aperture_backcasts = 0;
      int num_focal_backcasts = 0;
      
      uchar *input = (uchar*) (wristViewYCbCr.data);
      
      for (int py = topy; py <= boty; py++) {
        uchar* gripperMaskPixel = camera->gripperMask.ptr<uchar>(py); // point to first pixel in row
        for (int px = topx; px <= botx; px++) {
          if (gripperMaskPixel[px] == 0) {
            continue;
          }
          
          if ( (abhr > 0) && (abhc > 0) ) {
            if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
                 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
              continue;
            } 
          } 

	  // XXX
	  // check to see if the ray is within the global aperture
	  //  if the start point is in front of the aperture, 
	  //  back cast it for the check
          
          double x_a, y_a;
	  if (a_cast_z > 0) {
	    pixelToGlobalFromCache(ms, px, py, &x_a, &y_a, &data_a);
	  } else {
	    //pixelToGlobalFromCacheBackCast(ms, px, py, &x_a, &y_a, &data_a);
	    num_aperture_backcasts++;
	    double x_2a, y_2a;
	    pixelToGlobalFromCache(ms, px, py, &x_a, &y_a, &data_a);
	    pixelToGlobalFromCache(ms, px, py, &x_2a, &y_2a, &data_2a);

	    x_a = x_a + 2.0*(x_a - x_2a);
	    y_a = y_a + 2.0*(y_a - y_2a);
	  } // 0 case should have been handled above
	  
	  if (x_a * x_a + y_a * y_a > global_aperture_radius_meters_squared) {
	    //cout << "  rejected: " << x_a << " " << ms->config.scene->anchor_pose.px << " " << y_a << " " << ms->config.scene->anchor_pose.py << endl;
	    continue;
	  } else {
	    //cout << "  retained: " << x_a << " " << ms->config.scene->anchor_pose.px << " " << y_a << " " << ms->config.scene->anchor_pose.py << endl;
	  } 

          double x_f, y_f;
	  if (f_cast_z > 0) {
	    pixelToGlobalFromCache(ms, px, py, &x_f, &y_f, &data_f);
	  } else {
	    //pixelToGlobalFromCacheBackCast(ms, px, py, &x_f, &y_f, &data_f);
	    num_focal_backcasts++;
	    double x_2f, y_2f;
	    pixelToGlobalFromCache(ms, px, py, &x_f, &y_f, &data_f);
	    pixelToGlobalFromCache(ms, px, py, &x_2f, &y_2f, &data_2f);

	    x_f = x_f + 2.0*(x_f - x_2f);
	    y_f = y_f + 2.0*(y_f - y_2f);
	  } // 0 case should have been handled above
          
          if (1) {
            // single sample update
            int i, j;
            maps[thread]->metersToCell(x_f, y_f, &i, &j);
            GaussianMapCell * cell = maps[thread]->refAtCell(i, j);
	    int base_idx = py * wristViewYCbCr.cols*3 + px * 3;
            
	    int ri, rj;
	    ms->config.gaussian_map_register->metersToCell(x_f, y_f, &ri, &rj);
	    GaussianMapCell * register_cell = ms->config.gaussian_map_register->refAtCell(ri, rj);

	    if (register_cell == NULL) {
	      continue;
	    }
	    if (z_focal_plane > register_cell->z.mu) {
	      //cout << "SK: " << color_discrepancy << " " ;
	      continue;
	    }

	    double p_color_cast_thresh = 0.5;
	    double p_bright_cast_thresh = 0.5;

	    // use the cell this will hit as the lighting model
	    GaussianMapCell * lm = register_cell;
	    double total_discrepancy = 0.0;
	    double color_discrepancy = 0.0;
	    double bright_discrepancy = 0.0;
	    double p_ray_samples = 100.0;
	    {
	      GaussianMapCell rayCell;
	      rayCell.red.samples = p_ray_samples;
	      rayCell.green.samples = p_ray_samples;
	      rayCell.blue.samples = p_ray_samples;
	      rayCell.red.mu = input[base_idx + 2]; 
	      rayCell.green.mu = input[base_idx + 1]; 
	      rayCell.blue.mu = input[base_idx + 0];
	      rayCell.red.sigmasquared = 0;
	      rayCell.green.sigmasquared = 0;
	      rayCell.blue.sigmasquared = 0;

	      double rmu_diff = 0.0;
	      double gmu_diff = 0.0;
	      double bmu_diff = 0.0;

	      double discrepancy_value;
	      if (ms->config.discrepancyMode == DISCREPANCY_POINT) {
		discrepancy_value = lm->pointDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	      } else if (ms->config.discrepancyMode == DISCREPANCY_DOT) {
		discrepancy_value = lm->innerProduct(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	      } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_OR) {
		discrepancy_value = lm->noisyOrDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	      } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_AND) {
		discrepancy_value = lm->noisyAndDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	      } else {
		cout << "Invalid discrepancy mode: " << ms->config.discrepancyMode << endl;
		assert(0);
	      }
	
	      total_discrepancy = 1.0 - discrepancy_value;
	    } 
	    bright_discrepancy = total_discrepancy;
	    color_discrepancy = total_discrepancy;

	    if (color_discrepancy  < p_color_cast_thresh) {
	      //cout << "SK: " << color_discrepancy << " " ;
	      continue;
	    }
	    if (bright_discrepancy  < p_bright_cast_thresh) {
	      //cout << "SK: " << color_discrepancy << " " ;
	      continue;
	    }
            
            
            if (cell != NULL) {
              cell->newObservation(input[base_idx + 2], input[base_idx + 1], input[base_idx + 0], z_focal_plane);
              numPixels++;
            }
          } else {
            numNulls++;
          }
        }
      }
  
      //cout << "backcasts ap fc: " << num_aperture_backcasts << " " << num_focal_backcasts << endl;
    }
  }
  
  #pragma omp for
  for (int y = 0; y < ms->config.scene->observed_map->height; y++) {
    for (int x = 0; x < ms->config.scene->observed_map->width; x++) {
      for (int thread = 0; thread < numThreads; thread++) {
        if (maps[thread]->refAtCell(x, y)->red.samples > 0) {
          ms->config.scene->observed_map->refAtCell(x, y)->addC(maps[thread]->refAtCell(x, y));
        }
      }
    }
  }

}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcViewSynthAllGCSelfRenderGlareOnly)

WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcViewSynthAllGCSelfRenderWithoutGlare)
virtual void execute(MachineState * ms) {
// XXX not really started yet

  // distance of focal plane from anchor pose.
  double z_focal_plane = 0.388;
  GET_NUMERIC_ARG(ms, z_focal_plane);

  // distance of aperture plane from anchor pose.
  double z_aperture_plane = 0.01;
  GET_NUMERIC_ARG(ms, z_aperture_plane);

  // width in meters of circular global aperture
  double global_aperture_width_meters = 0.01;
  GET_NUMERIC_ARG(ms, global_aperture_width_meters);
  double global_aperture_radius_meters = global_aperture_width_meters / 2.0;
  double global_aperture_radius_meters_squared = global_aperture_radius_meters * global_aperture_radius_meters;
  

  int stride = 1;
  GET_INT_ARG(ms, stride);


  Size sz = ms->config.wristViewImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 

  cout << "viewSynth fp ap awm str: " << z_focal_plane << " " << z_aperture_plane << " " << global_aperture_width_meters << " " << stride << endl;
  
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  Mat gripperMask = camera->gripperMask;
  if (isSketchyMat(gripperMask)) {
    CONSOLE_ERROR(ms, "Gripper mask is messed up.");
    return;
  }


  int numThreads = 8;

  vector<shared_ptr<GaussianMap> > maps;
  maps.resize(numThreads);

  #pragma omp parallel for
  for (int thread = 0; thread < numThreads; thread++) {
    
    maps[thread] = make_shared<GaussianMap>(ms, 
                                            ms->config.scene->observed_map->width, 
                                            ms->config.scene->observed_map->height, 
                                            ms->config.scene->observed_map->cell_width,
                                            ms->config.scene->observed_map->anchor_pose); 
    maps[thread]->zero();

    
    int thisStart = thread * (camera->streamImageBuffer.size()  / numThreads);
    int thisEnd = (thread + 1) * (camera->streamImageBuffer.size()  / numThreads); 
    stringstream buf;    
    buf << "thread: " << thread << " start: " << thisStart << " end: " << thisEnd << endl;
    cout << buf.str();
    for (int i = thisStart; i < thisEnd; i+=stride) {
      //streamImage * tsi = setIsbIdxNoLoadNoKick(ms, i);
      streamImage * tsi = camera->getIsbIdxNoLoadNoKick(i);
      
      
      
      if (tsi == NULL) {
        CONSOLE_ERROR(ms, "Stream image null.");
      }
      eePose tArmP, tBaseP;
      
      int success = 0;
      // XXX
      double z = z_focal_plane;
      if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
        //success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
        success = ms->getStreamPoseAtTimeThreadSafe(tsi->time, &tArmP, &tBaseP);
      } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
        success = 1;
        tArmP = ms->config.currentEEPose;
        z = ms->config.currentEEPose.pz + ms->config.currentTableZ;
      } else {
        assert(0);
      }


      
      
      if (success != 1) {
        CONSOLE_ERROR(ms, "Couldn't get stream pose: " << success << " time: " << tsi->time);
        continue;
      }


      /*
      eePose focal_plane_anchor_pose = eePose::identity();
      focal_plane_anchor_pose.pz = -z_focal_plane;
      focal_plane_anchor_pose = focal_plane_anchor_pose.applyAsRelativePoseTo(ms->config.scene->anchor_pose);
      */
      
      eePose transformed = tArmP.getPoseRelativeTo(ms->config.scene->anchor_pose);
      if (fabs(transformed.qz) > 0.01) {
        CONSOLE_ERROR(ms, "  Not doing update because arm not vertical.");
        continue;
      }

      // XXX
      // get the pose of the image in the anchor pose frame,
      //  subtract the depth from the achnor pose focal depth.
      // if difference is positive, continue and use that z for casting
      //  but anchor focal depth as z observation
      // but the anchor pose canonically points backwards relative to the end effector that generated it
      //  so be careful with the sign
      double fp_anchor_z = -z_focal_plane;
      double ap_anchor_z = -z_aperture_plane;
      double f_cast_z = -(fp_anchor_z - transformed.pz);
      double a_cast_z = -(ap_anchor_z - transformed.pz);
      cout << transformed.pz << " " << f_cast_z << " " << a_cast_z << endl;
      if ( (f_cast_z == 0.0) || (a_cast_z == 0.0) ) {
        CONSOLE_ERROR(ms, "  Not doing update because height singular.");
	continue;
      }


      pixelToGlobalCache data_f;      
      computePixelToPlaneCache(ms, fabs(f_cast_z), tArmP, ms->config.scene->anchor_pose, &data_f);  

      double f_bc_cast_length = fabs(2.8*f_cast_z);
      pixelToGlobalCache data_2f;      
      computePixelToPlaneCache(ms, f_bc_cast_length, tArmP, ms->config.scene->anchor_pose, &data_2f);  

      pixelToGlobalCache data_a;      
      computePixelToPlaneCache(ms, fabs(a_cast_z), tArmP, ms->config.scene->anchor_pose, &data_a);  

      double a_bc_cast_length = fabs(2.8*a_cast_z);
      pixelToGlobalCache data_2a;      
      computePixelToPlaneCache(ms, a_bc_cast_length, tArmP, ms->config.scene->anchor_pose, &data_2a);  
      
      Mat wristViewYCbCr = tsi->image.clone();
      
      cvtColor(tsi->image, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
      int numPixels = 0;
      int numNulls = 0;

      int num_aperture_backcasts = 0;
      int num_focal_backcasts = 0;
      
      uchar *input = (uchar*) (wristViewYCbCr.data);
      
      for (int py = topy; py <= boty; py++) {
        uchar* gripperMaskPixel = camera->gripperMask.ptr<uchar>(py); // point to first pixel in row
        for (int px = topx; px <= botx; px++) {
          if (gripperMaskPixel[px] == 0) {
            continue;
          }
          
          if ( (abhr > 0) && (abhc > 0) ) {
            if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
                 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
              continue;
            } 
          } 

	  // XXX
	  // check to see if the ray is within the global aperture
	  //  if the start point is in front of the aperture, 
	  //  back cast it for the check
          
          double x_a, y_a;
	  if (a_cast_z > 0) {
	    pixelToGlobalFromCache(ms, px, py, &x_a, &y_a, &data_a);
	  } else {
	    //pixelToGlobalFromCacheBackCast(ms, px, py, &x_a, &y_a, &data_a);
	    num_aperture_backcasts++;
	    double x_2a, y_2a;
	    pixelToGlobalFromCache(ms, px, py, &x_a, &y_a, &data_a);
	    pixelToGlobalFromCache(ms, px, py, &x_2a, &y_2a, &data_2a);

	    x_a = x_a + 2.0*(x_a - x_2a);
	    y_a = y_a + 2.0*(y_a - y_2a);
	  } // 0 case should have been handled above
	  
	  if (x_a * x_a + y_a * y_a > global_aperture_radius_meters_squared) {
	    //cout << "  rejected: " << x_a << " " << ms->config.scene->anchor_pose.px << " " << y_a << " " << ms->config.scene->anchor_pose.py << endl;
	    continue;
	  } else {
	    //cout << "  retained: " << x_a << " " << ms->config.scene->anchor_pose.px << " " << y_a << " " << ms->config.scene->anchor_pose.py << endl;
	  } 

          double x_f, y_f;
	  if (f_cast_z > 0) {
	    pixelToGlobalFromCache(ms, px, py, &x_f, &y_f, &data_f);
	  } else {
	    //pixelToGlobalFromCacheBackCast(ms, px, py, &x_f, &y_f, &data_f);
	    num_focal_backcasts++;
	    double x_2f, y_2f;
	    pixelToGlobalFromCache(ms, px, py, &x_f, &y_f, &data_f);
	    pixelToGlobalFromCache(ms, px, py, &x_2f, &y_2f, &data_2f);

	    x_f = x_f + 2.0*(x_f - x_2f);
	    y_f = y_f + 2.0*(y_f - y_2f);
	  } // 0 case should have been handled above
          
          if (1) {
            // single sample update
            int i, j;
            maps[thread]->metersToCell(x_f, y_f, &i, &j);
            GaussianMapCell * cell = maps[thread]->refAtCell(i, j);
	    int base_idx = py * wristViewYCbCr.cols*3 + px * 3;
            
	    int ri, rj;
	    ms->config.gaussian_map_register->metersToCell(x_f, y_f, &ri, &rj);
	    GaussianMapCell * register_cell = ms->config.gaussian_map_register->refAtCell(ri, rj);

	    if (register_cell == NULL) {
	      continue;
	    }
	    if (z_focal_plane > register_cell->z.mu) {
	      //cout << "SK: " << color_discrepancy << " " ;
	      continue;
	    }

	    double p_color_cast_thresh = 0.5;
	    double p_bright_cast_thresh = 0.5;

	    // use the cell this will hit as the lighting model
	    GaussianMapCell * lm = register_cell;
	    double total_discrepancy = 0.0;
	    double color_discrepancy = 0.0;
	    double bright_discrepancy = 0.0;
	    double p_ray_samples = 100.0;
	    {
	      GaussianMapCell rayCell;
	      rayCell.red.samples = p_ray_samples;
	      rayCell.green.samples = p_ray_samples;
	      rayCell.blue.samples = p_ray_samples;
	      rayCell.red.mu = input[base_idx + 2]; 
	      rayCell.green.mu = input[base_idx + 1]; 
	      rayCell.blue.mu = input[base_idx + 0];
	      rayCell.red.sigmasquared = 0;
	      rayCell.green.sigmasquared = 0;
	      rayCell.blue.sigmasquared = 0;

	      double rmu_diff = 0.0;
	      double gmu_diff = 0.0;
	      double bmu_diff = 0.0;

	      double discrepancy_value;
	      if (ms->config.discrepancyMode == DISCREPANCY_POINT) {
		discrepancy_value = lm->pointDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	      } else if (ms->config.discrepancyMode == DISCREPANCY_DOT) {
		discrepancy_value = lm->innerProduct(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	      } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_OR) {
		discrepancy_value = lm->noisyOrDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	      } else if (ms->config.discrepancyMode == DISCREPANCY_NOISY_AND) {
		discrepancy_value = lm->noisyAndDiscrepancy(&rayCell, &rmu_diff, &gmu_diff, &bmu_diff);
	      } else {
		cout << "Invalid discrepancy mode: " << ms->config.discrepancyMode << endl;
		assert(0);
	      }
	
	      total_discrepancy = 1.0 - discrepancy_value;
	    } 
	    bright_discrepancy = total_discrepancy;
	    color_discrepancy = total_discrepancy;

	    if (color_discrepancy  > p_color_cast_thresh) {
	      //cout << "SK: " << color_discrepancy << " " ;
	      continue;
	    }
	    if (bright_discrepancy  > p_bright_cast_thresh) {
	      //cout << "SK: " << color_discrepancy << " " ;
	      continue;
	    }
            
            if (cell != NULL) {
              cell->newObservation(input[base_idx + 2], input[base_idx + 1], input[base_idx + 0], z_focal_plane);
              numPixels++;
            }
          } else {
            numNulls++;
          }
        }
      }
  
      //cout << "backcasts ap fc: " << num_aperture_backcasts << " " << num_focal_backcasts << endl;
    }
  }
  
  #pragma omp for
  for (int y = 0; y < ms->config.scene->observed_map->height; y++) {
    for (int x = 0; x < ms->config.scene->observed_map->width; x++) {
      for (int thread = 0; thread < numThreads; thread++) {
        if (maps[thread]->refAtCell(x, y)->red.samples > 0) {
          ms->config.scene->observed_map->refAtCell(x, y)->addC(maps[thread]->refAtCell(x, y));
        }
      }
    }
  }

}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcViewSynthAllGCSelfRenderWithoutGlare)

WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcSecondStageArray)
virtual void execute(MachineState * ms) {
  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  int thisIdx = camera->sibCurIdx;
  //cout << "sceneUpdateObservedFromStreamBuffer: " << thisIdx << endl;

  double arrayDimension = 25;
  GET_NUMERIC_ARG(ms, arrayDimension);

  double lens_strength = 0.5;
  GET_NUMERIC_ARG(ms, lens_strength);

  if (lens_strength == 0) {
    lens_strength = 1.0e-6;
    cout << "sceneUpdateObservedFromStreamBufferAtZNoRecalcSecondStageArray: given 0 lens_strength, using " << lens_strength << " instead." << endl;
  }

  double lens_gap = 0.01;
  GET_NUMERIC_ARG(ms, lens_gap);

  cout << "zToUse arrayDimension lens_strength: " << z_to_use << " " << arrayDimension << " " << lens_strength << endl;

  Mat bufferImage;
  eePose thisPose, tBaseP;


  int success = 1;
  if ( (thisIdx > -1) && (thisIdx < camera->streamImageBuffer.size()) ) {
    streamImage &tsi = camera->streamImageBuffer[thisIdx];
    loadStreamImage(ms, &tsi);
    bufferImage = tsi.image.clone();

    if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
      success = ms->getStreamPoseAtTime(tsi.time, &thisPose, &tBaseP);
    } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
      success = 1;
      thisPose = ms->config.currentEEPose;
      z_to_use = ms->config.currentEEPose.pz + ms->config.currentTableZ;
    } else {
      assert(0);
    }
  } else {
    CONSOLE_ERROR(ms, "No images in the buffer, returning." << endl);
    return;
  }

  if (success != 1) {
    CONSOLE_ERROR(ms, "  Not doing update because of stream buffer errors.");
    return;
  }

  eePose transformed = thisPose.getPoseRelativeTo(ms->config.scene->anchor_pose);
  if (fabs(transformed.qz) > 0.01) {
    CONSOLE_ERROR(ms, "  Not doing update because arm not vertical.");
    return;
  }

  Mat wristViewYCbCr = bufferImage.clone();

  cvtColor(bufferImage, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
  
  Size sz = bufferImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 
  
  pixelToGlobalCache data;
  pixelToGlobalCache data_gap;
  double z = z_to_use;
  double z_gap = z_to_use + lens_gap;
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  computePixelToPlaneCache(ms, z, thisPose, ms->config.scene->anchor_pose, &data);  
  computePixelToPlaneCache(ms, z_gap, thisPose, ms->config.scene->anchor_pose, &data_gap);  
  int numThreads = 8;
  // there is a faster way to stride it but i am risk averse atm


  int numPixels = 0;
  int numNulls = 0;


  int lens_row_spacing = ceil(ms->config.scene->height / arrayDimension);
  int lens_col_spacing = ceil(ms->config.scene->width / arrayDimension);

  cout << "lens_row_spacing lens_col_spacing: " << lens_row_spacing << " " << lens_col_spacing << endl;

  #pragma omp parallel for
  for (int i = 0; i < numThreads; i++) {
    //double frac = double(boty - topy) / double(numThreads);
    //double bfrac = i*frac;
    //double tfrac = (i+1)*frac;

    double frac = double(boty - topy) / double(numThreads);
    int ttopy = floor(topy + i*frac);
    int tboty = floor(topy + (i+1)*frac);

    //for (int py = topy; py <= boty; py++) 
      for (int py = ttopy; py < tboty; py++) 
      {

      //double opy = py-topy;
      // this is superior
      //if ( (bfrac <= opy) && (opy < tfrac) ) 
      //{
	for (int px = topx; px <= botx; px++) {
	  if (isInGripperMask(ms, px, py)) {
	    continue;
	  }

	  if ( (abhr > 0) && (abhc > 0) ) {
	    if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
		 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
	      continue;
	    } 
	  } 

	  double x, y;
	  pixelToGlobalFromCache(ms, px, py, &x, &y, &data);

	  double x_gap, y_gap;
	  pixelToGlobalFromCache(ms, px, py, &x_gap, &y_gap, &data_gap);
	  
	  if (1) {
	    // single sample update
	    int i, j;
	    ms->config.scene->observed_map->metersToCell(x, y, &i, &j);

	    // find the cell of the lens this ray falls into
	    int li, lj;
	    li = (i / lens_row_spacing) * lens_row_spacing; 
	    lj = (j / lens_col_spacing) * lens_col_spacing; 

	    GaussianMapCell * lcell = ms->config.scene->observed_map->refAtCell(li, lj);

	    //if (lcell != NULL) 
	    {

	      // find the physical coordinate of the lens, also the focal point
	      double mi, mj;
	      ms->config.scene->observed_map->cellToMeters(li, lj, &mi, &mj);

	      // shrink this ray towards the focal point
	      //double ri = ((1.0 - lens_strength) * x + (lens_strength) * mi) * (1.0-lens_strength);
	      //double rj = ((1.0 - lens_strength) * y + (lens_strength) * mj) * (1.0-lens_strength);
	
	      // projected angle is proportional to displacement caused by the gap
	      // the bigger the gap the more angles we get

	      double ri = ((x_gap-x)/(lens_strength) + mi);
	      double rj = ((y_gap-y)/(lens_strength) + mj);

	      // and put it in that cell
	      int cri, crj;
	      ms->config.scene->observed_map->metersToCell(ri, rj, &cri, &crj);

	      GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(cri, crj);
	      if (cell != NULL) {
		  Vec3b pixel = wristViewYCbCr.at<Vec3b>(py, px);
		  cell->newObservation(pixel, z);
		numPixels++;
	      } else {
		  numNulls++;
	      }
	    }
	  }
	}
      //}
    }
  }
  //cout << "numPixels numNulls sum apertureSize: " << numPixels << " " << numNulls << " " << numPixels + numNulls << " " << ms->config.angular_aperture_rows * ms->config.angular_aperture_cols << endl;
  //ms->config.scene->observed_map->recalculateMusAndSigmas(ms);
}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcSecondStageArray)

WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcSecondStageAsymmetricArray)
virtual void execute(MachineState * ms) {
  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);

  double lens_gap = 0.01;
  GET_NUMERIC_ARG(ms, lens_gap);


  double array_phase_y = 0.01;
  GET_NUMERIC_ARG(ms, array_phase_y);

  double array_spacing_y = 0.01;
  GET_NUMERIC_ARG(ms, array_spacing_y);

  double lens_strength_y = 0.5;
  GET_NUMERIC_ARG(ms, lens_strength_y);

  if (lens_strength_y == 0) {
    lens_strength_y = 1.0e-6;
    cout << "sceneUpdateObservedFromStreamBufferAtZNoRecalcSecondStageAsymmetricArray: given 0 lens_strength, using " << lens_strength_y << " instead." << endl;
  }


  double array_phase_x = 0.01;
  GET_NUMERIC_ARG(ms, array_phase_x);

  double array_spacing_x = 0.01;
  GET_NUMERIC_ARG(ms, array_spacing_x);

  double lens_strength_x = 0.5;
  GET_NUMERIC_ARG(ms, lens_strength_x);

  if (lens_strength_x == 0) {
    lens_strength_x = 1.0e-6;
    cout << "sceneUpdateObservedFromStreamBufferAtZNoRecalcSecondStageAsymmetricArray: given 0 lens_strength, using " << lens_strength_x << " instead." << endl;
  }

  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  int thisIdx = camera->sibCurIdx;
  //cout << "sceneUpdateObservedFromStreamBuffer: " << thisIdx << endl;

  cout << "zToUse lens_gap: " << z_to_use << " " << lens_gap << endl;
  cout << "array_spacing_x lens_strength_x: " << array_spacing_x << " " << lens_strength_x << endl;
  cout << "array_spacing_y lens_strength_y: " << array_spacing_y << " " << lens_strength_y << endl;

  Mat bufferImage;
  eePose thisPose, tBaseP;


  int success = 1;
  if ( (thisIdx > -1) && (thisIdx < camera->streamImageBuffer.size()) ) {
    streamImage &tsi = camera->streamImageBuffer[thisIdx];
    loadStreamImage(ms, &tsi);
    bufferImage = tsi.image.clone();

    if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
      success = ms->getStreamPoseAtTime(tsi.time, &thisPose, &tBaseP);
    } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
      success = 1;
      thisPose = ms->config.currentEEPose;
      z_to_use = ms->config.currentEEPose.pz + ms->config.currentTableZ;
    } else {
      assert(0);
    }
  } else {
    CONSOLE_ERROR(ms, "No images in the buffer, returning." << endl);
    return;
  }

  if (success != 1) {
    CONSOLE_ERROR(ms, "  Not doing update because of stream buffer errors.");
    return;
  }

  eePose transformed = thisPose.getPoseRelativeTo(ms->config.scene->anchor_pose);
  if (fabs(transformed.qz) > 0.01) {
    CONSOLE_ERROR(ms, "  Not doing update because arm not vertical.");
    return;
  }

  Mat wristViewYCbCr = bufferImage.clone();

  cvtColor(bufferImage, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
  
  Size sz = bufferImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 
  
  pixelToGlobalCache data;
  pixelToGlobalCache data_gap;
  double z = z_to_use;
  double z_gap = z_to_use + lens_gap;
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  computePixelToPlaneCache(ms, z, thisPose, ms->config.scene->anchor_pose, &data);  
  computePixelToPlaneCache(ms, z_gap, thisPose, ms->config.scene->anchor_pose, &data_gap);  
  int numThreads = 8;
  // there is a faster way to stride it but i am risk averse atm


  int numPixels = 0;
  int numNulls = 0;

  #pragma omp parallel for
  for (int i = 0; i < numThreads; i++) {
    //double frac = double(boty - topy) / double(numThreads);
    //double bfrac = i*frac;
    //double tfrac = (i+1)*frac;

    double frac = double(boty - topy) / double(numThreads);
    int ttopy = floor(topy + i*frac);
    int tboty = floor(topy + (i+1)*frac);

    //for (int py = topy; py <= boty; py++) 
      for (int py = ttopy; py < tboty; py++) 
      {

      //double opy = py-topy;
      // this is superior
      //if ( (bfrac <= opy) && (opy < tfrac) ) 
      //{
	for (int px = topx; px <= botx; px++) {
	  if (isInGripperMask(ms, px, py)) {
	    continue;
	  }

	  if ( (abhr > 0) && (abhc > 0) ) {
	    if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
		 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
	      continue;
	    } 
	  } 

	  double x, y;
	  pixelToGlobalFromCache(ms, px, py, &x, &y, &data);

	  double x_gap, y_gap;
	  pixelToGlobalFromCache(ms, px, py, &x_gap, &y_gap, &data_gap);
	  
	  if (1) {
	    // single sample update
	    int i, j;
	    ms->config.scene->observed_map->metersToCell(x, y, &i, &j);

	    // find the physical coordinate of the lens this ray falls into
	    double lx, ly;
	    lx = floor(x / array_spacing_x) * array_spacing_x; 
	    ly = floor(y / array_spacing_y) * array_spacing_y; 
	    lx += array_phase_x;
	    ly += array_phase_y;

	    // find the cell of the lens 
	    //double li, lj;
	    //ms->config.scene->observed_map->metersToCell(lx, ly, &li, &lj);
	    //GaussianMapCell * lcell = ms->config.scene->observed_map->refAtCell(li, lj);
	    //if (lcell != NULL) 
	    {
	      // shrink this ray towards the focal point
	      //double ri = ((1.0 - lens_strength) * x + (lens_strength) * mi) * (1.0-lens_strength);
	      //double rj = ((1.0 - lens_strength) * y + (lens_strength) * mj) * (1.0-lens_strength);
	
	      // projected angle is proportional to displacement caused by the gap
	      // the bigger the gap the more angles we get

	      double ri = ((x_gap-x)/(lens_strength_x) + lx);
	      double rj = ((y_gap-y)/(lens_strength_y) + ly);

	      // and put it in that cell
	      int cri, crj;
	      ms->config.scene->observed_map->metersToCell(ri, rj, &cri, &crj);

	      GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(cri, crj);
	      if (cell != NULL) {
		  Vec3b pixel = wristViewYCbCr.at<Vec3b>(py, px);
		  cell->newObservation(pixel, z);
		numPixels++;
	      } else {
		  numNulls++;
	      }
	    }
	  }
	}
      //}
    }
  }
  //cout << "numPixels numNulls sum apertureSize: " << numPixels << " " << numNulls << " " << numPixels + numNulls << " " << ms->config.angular_aperture_rows * ms->config.angular_aperture_cols << endl;
  //ms->config.scene->observed_map->recalculateMusAndSigmas(ms);
}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcSecondStageAsymmetricArray)

WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcEPI)
virtual void execute(MachineState * ms) {

  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);

  double lens_gap = 0.01;
  GET_NUMERIC_ARG(ms, lens_gap);


  double y_slice_meters = 0.01;
  GET_NUMERIC_ARG(ms, y_slice_meters);

  double y_aperture_meters = 0.01;
  GET_NUMERIC_ARG(ms, y_aperture_meters);


  double v_slice_unitless = 0.01;
  GET_NUMERIC_ARG(ms, v_slice_unitless);

  double v_aperture_unitless = 0.01;
  GET_NUMERIC_ARG(ms, v_aperture_unitless);
  

  double lens_strength_u = 0.5;
  GET_NUMERIC_ARG(ms, lens_strength_u);


  int stride = 1;
  GET_INT_ARG(ms, stride);

  if (lens_strength_u == 0) {
    lens_strength_u = 1.0e-6;
    cout << "sceneUpdateObservedFromStreamBufferAtZNoRecalcEPI: given 0 lens_strength, using " << lens_strength_u << " instead." << endl;
  }

  cout << "zToUse lens_gap: " << z_to_use << " " << lens_gap << endl;
  cout << "y_slice_meters y_aperture_meters: " << y_slice_meters << " " << y_aperture_meters << endl;
  cout << "v_slice_unitless v_aperture_unitless: " << v_slice_unitless << " " << v_aperture_unitless << endl;
  cout << "lens_strength_u: " << lens_strength_u << endl;
  cout << "stride: " << stride << endl;

  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  //Size sz = ms->config.wristViewImage.size();
  Size sz = ms->config.cameras[ms->config.focused_camera]->cam_img.size();

  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 

  topx = std::max(topx, 0);
  botx = std::min(botx, imW-1);

  topy = std::max(topy, 0);
  boty = std::min(boty, imH-1);

  // XXX need to bounds check on all of these variants
  // consider using a function pointer inside of a preconstructed loop or macro it.

  Mat gripperMask = camera->gripperMask;
  if (isSketchyMat(gripperMask)) {
    CONSOLE_ERROR(ms, "Gripper mask is messed up.");
    return;
  }

  int numThreads = 8;


  vector<shared_ptr<GaussianMap> > maps;
  maps.resize(numThreads);

  #pragma omp parallel for
  for (int thread = 0; thread < numThreads; thread++) {
    
    maps[thread] = make_shared<GaussianMap>(ms, 
                                            ms->config.scene->observed_map->width, 
                                            ms->config.scene->observed_map->height, 
                                            ms->config.scene->observed_map->cell_width,
                                            ms->config.scene->observed_map->anchor_pose); 
    maps[thread]->zero();

    
    int thisStart = thread * (camera->streamImageBuffer.size()  / numThreads);
    int thisEnd = (thread + 1) * (camera->streamImageBuffer.size()  / numThreads); 
    stringstream buf;    
    buf << "thread: " << thread << " start: " << thisStart << " end: " << thisEnd << endl;
    cout << buf.str();
    for (int i = thisStart; i < thisEnd; i+=stride) {
      streamImage * tsi = camera->setIsbIdxNoLoadNoKick(i);
      
      
      
      if (tsi == NULL) {
        CONSOLE_ERROR(ms, "Stream image null.");
      }
      eePose tArmP, tBaseP;
      
      int success = 0;
      double z = z_to_use;
      if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
        success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
      } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
        success = 1;
        tArmP = ms->config.currentEEPose;
        z = ms->config.currentEEPose.pz + ms->config.currentTableZ;
      } else {
        assert(0);
      }
      
      
      if (success != 1) {
        CONSOLE_ERROR(ms, "Couldn't get stream pose.  Return code: " << success << " time: " << tsi->time);
        continue;
      }
      
      eePose transformed = tArmP.getPoseRelativeTo(ms->config.scene->anchor_pose);
      if (fabs(transformed.qz) > 0.01) {
        //CONSOLE_ERROR(ms, "Not doing update because arm not vertical.");
        //continue;
        CONSOLE_ERROR(ms, "Would normally not be doing update because arm not vertical, but we are.");
      }
      pixelToGlobalCache data;      
      computePixelToGlobalFullOOPCache(ms, z, tArmP, ms->config.scene->anchor_pose, &data);  

      pixelToGlobalCache data2;      
      computePixelToGlobalFullOOPCache(ms, z + lens_gap, tArmP, ms->config.scene->anchor_pose, &data2);  

      
      // there is a faster way to stride it but i am risk averse atm
      
      Mat wristViewYCbCr = tsi->image.clone();
      
      cvtColor(tsi->image, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
      int numPixels = 0;
      int numNulls = 0;
      
      uchar *input = (uchar*) (wristViewYCbCr.data);
      
      for (int py = topy; py <= boty; py++) {
        uchar* gripperMaskPixel = camera->gripperMask.ptr<uchar>(py); // point to first pixel in row
        for (int px = topx; px <= botx; px++) {
          if (gripperMaskPixel[px] == 0) {
            continue;
          }
          
          if ( (abhr > 0) && (abhc > 0) ) {
            if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
                 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
              continue;
            } 
          } 
          
          double x, y;
	  pixelToGlobalFullFromCacheZOOP(ms, px, py, &x, &y, &data);

          double x2, y2;
	  pixelToGlobalFullFromCacheZOOP(ms, px, py, &x2, &y2, &data2);

	  // skip if not in Y aperture
	  if ( fabs( y - y_slice_meters ) >  y_aperture_meters ) {
	    continue;
	  } 
	  
	  // skip if not in V aperture
	  double v_unitless = (y2-y)/(lens_gap);
	  if ( fabs( v_unitless - v_slice_unitless ) > v_aperture_unitless ) {
	    continue;
	  }
          
	  // keep X coordinate 
	  // determine Y coordinate from U and magnification
	  double u_unitless = (x2-x)/(lens_gap);
	  double y_from_u = u_unitless * lens_strength_u;

	  // single sample update
	  int i, j;
	  maps[thread]->metersToCell(x, y_from_u, &i, &j);

	  GaussianMapCell * cell = maps[thread]->refAtCell(i, j);
	  
	  if (cell != NULL) {
	    int base_idx = py * wristViewYCbCr.cols*3 + px * 3;
	    cell->newObservation(input[base_idx + 2], input[base_idx + 1], input[base_idx + 0], z);
	    numPixels++;
	  }
        }
      }
    }
  }
  
  #pragma omp for
  for (int y = 0; y < ms->config.scene->observed_map->height; y++) {
    for (int x = 0; x < ms->config.scene->observed_map->width; x++) {
      for (int thread = 0; thread < numThreads; thread++) {
        if (maps[thread]->refAtCell(x, y)->red.samples > 0) {
          ms->config.scene->observed_map->refAtCell(x, y)->addC(maps[thread]->refAtCell(x, y));
        }
      }
    }
  }

}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcEPI)


WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcPhasedArray)
virtual void execute(MachineState * ms) {
  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);

  double lens_gap = 0.01;
  GET_NUMERIC_ARG(ms, lens_gap);

  double lambda = 0.01;
  GET_NUMERIC_ARG(ms, lambda);

  double phi = 0.01;
  GET_NUMERIC_ARG(ms, phi);

  double gain = 1.0;
  GET_NUMERIC_ARG(ms, gain);

  if (lambda == 0) {
    lambda = 1.0e-6;
    cout << "sceneUpdateObservedFromStreamBufferAtZNoRecalcPhasedArray: given 0 lambda, using " << lambda << " instead." << endl;
  }
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  int thisIdx = camera->sibCurIdx;
  //cout << "sceneUpdateObservedFromStreamBuffer: " << thisIdx << endl;

  //cout << "zToUse lens_gap: " << z_to_use << " " << lens_gap << endl;

  Mat bufferImage;
  eePose thisPose, tBaseP;


  int success = 1;
  if ( (thisIdx > -1) && (thisIdx < camera->streamImageBuffer.size()) ) {
    streamImage &tsi = camera->streamImageBuffer[thisIdx];
    loadStreamImage(ms, &tsi);
    bufferImage = tsi.image.clone();

    if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
      success = ms->getStreamPoseAtTime(tsi.time, &thisPose, &tBaseP);
    } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
      success = 1;
      thisPose = ms->config.currentEEPose;
      z_to_use = ms->config.currentEEPose.pz + ms->config.currentTableZ;
    } else {
      assert(0);
    }
  } else {
    CONSOLE_ERROR(ms, "No images in the buffer, returning." << endl);
    return;
  }

  if (success != 1) {
    CONSOLE_ERROR(ms, "  Not doing update because of stream buffer errors.");
    return;
  }

  eePose transformed = thisPose.getPoseRelativeTo(ms->config.scene->anchor_pose);
  if (fabs(transformed.qz) > 0.01) {
    CONSOLE_ERROR(ms, "  Not doing update because arm not vertical.");
    return;
  }

  Mat wristViewYCbCr = bufferImage.clone();

  cvtColor(bufferImage, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
  
  Size sz = bufferImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 
  
  pixelToGlobalCache data;
  pixelToGlobalCache data_gap;
  double z = z_to_use;
  double z_gap = z_to_use + lens_gap;
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  computePixelToPlaneCache(ms, z, thisPose, ms->config.scene->anchor_pose, &data);  
  computePixelToPlaneCache(ms, z_gap, thisPose, ms->config.scene->anchor_pose, &data_gap);  
  int numThreads = 8;
  // there is a faster way to stride it but i am risk averse atm


  int numPixels = 0;
  int numNulls = 0;

  #pragma omp parallel for
  for (int i = 0; i < numThreads; i++) {
    //double frac = double(boty - topy) / double(numThreads);
    //double bfrac = i*frac;
    //double tfrac = (i+1)*frac;

    double frac = double(boty - topy) / double(numThreads);
    int ttopy = floor(topy + i*frac);
    int tboty = floor(topy + (i+1)*frac);

    //for (int py = topy; py <= boty; py++) 
      for (int py = ttopy; py < tboty; py++) 
      {

      //double opy = py-topy;
      // this is superior
      //if ( (bfrac <= opy) && (opy < tfrac) ) 
      //{
	for (int px = topx; px <= botx; px++) {
	  if (isInGripperMask(ms, px, py)) {
	    continue;
	  }

	  if ( (abhr > 0) && (abhc > 0) ) {
	    if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
		 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
	      continue;
	    } 
	  } 

	  double x, y;
	  pixelToGlobalFromCache(ms, px, py, &x, &y, &data);

	  double x_gap, y_gap;
	  pixelToGlobalFromCache(ms, px, py, &x_gap, &y_gap, &data_gap);
	  
	  {
	    double rx = x_gap;
	    double ry = y_gap;

	    // and put it in that cell
	    int cri, crj;
	    ms->config.scene->observed_map->metersToCell(rx, ry, &cri, &crj);

	    GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(cri, crj);
	    if (cell != NULL) {
		Vec3b pixel = wristViewYCbCr.at<Vec3b>(py, px);
		Vec3d phased_pixel;

		// I wonder if there is a difference
		//double gap_path_length = sqrt( pow(x_gap-x,2.0) + pow(y_gap-y,2.0) );
		double gap_path_length_squared = ( (lens_gap*lens_gap) + (x_gap-x)*(x_gap-x) + (y_gap-y)*(y_gap-y) );
		double gap_path_length = sqrt(gap_path_length_squared);

		/*
		phased_pixel[0] = double(pixel[0]) * (1.0+cos( gap_path_length / lambda )) * 0.5;
		phased_pixel[1] = double(pixel[1]) * (1.0+cos( gap_path_length / lambda )) * 0.5;
		phased_pixel[2] = double(pixel[2]) * (1.0+cos( gap_path_length / lambda )) * 0.5;

		phased_pixel[0] = (double(pixel[0])-128.0) * cos( gap_path_length / lambda ) + 128.0;
		phased_pixel[1] = (double(pixel[1])-128.0) * cos( gap_path_length / lambda ) + 128.0;
		phased_pixel[2] = (double(pixel[2])-128.0) * cos( gap_path_length / lambda ) + 128.0;
		*/

// XXX make sure to divide magnitude by the path length, eventually, but maybe leave out as an optimization if its not much 
		phased_pixel[0] = ( gain / gap_path_length_squared ) * (double(pixel[0])/2.0) * (1.0 + cos( gap_path_length / lambda ));
		phased_pixel[1] = ( gain / gap_path_length_squared ) * (double(pixel[1])/2.0) * cos( gap_path_length / lambda ) + 128.0;
		phased_pixel[2] = ( gain / gap_path_length_squared ) * (double(pixel[2])/2.0) * cos( gap_path_length / lambda ) + 128.0;

		cell->newObservation(phased_pixel, z);
	      numPixels++;
	    } else {
		numNulls++;
	    }
	  }
	}
      //}
    }
  }
  //cout << "numPixels numNulls sum apertureSize: " << numPixels << " " << numNulls << " " << numPixels + numNulls << " " << ms->config.angular_aperture_rows * ms->config.angular_aperture_cols << endl;
  //ms->config.scene->observed_map->recalculateMusAndSigmas(ms);
}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcPhasedArray)

WORD(SceneUpdateObservedFromRegisterAtZNoRecalcPhasedArray)
virtual void execute(MachineState * ms) {

  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);

  double array_z = 0.01;
  GET_NUMERIC_ARG(ms, array_z);

  double lambda = 0.01;
  GET_NUMERIC_ARG(ms, lambda);

  double phi = 0.01;
  GET_NUMERIC_ARG(ms, phi);



  double aperture_width_cells = 11;
  GET_NUMERIC_ARG(ms, aperture_width_cells);

  double lens_x = 0.01;
  GET_NUMERIC_ARG(ms, lens_x);

  double lens_y = 0.01;
  GET_NUMERIC_ARG(ms, lens_y);




  if (lambda == 0) {
    lambda = 1.0e-6;
    cout << "sceneUpdateObservedFromRegisterAtZNoRecalcPhasedArray: given 0 lambda, using " << lambda << " instead." << endl;
  }

  shared_ptr<GaussianMap> toFill = ms->config.scene->observed_map;
  shared_ptr<GaussianMap> toLight= ms->config.gaussian_map_register;

  int t_height = toFill->height;
  int t_width = toFill->width;

  int aperture_half_width_cells = (aperture_width_cells - 1)/2;

  assert( toLight->width == t_width );
  assert( toLight->height == t_height );

  for (int y = 0; y < t_height; y++) {
    for (int x = 0; x < t_width; x++) {

      int axstart = std::max(0, x - aperture_half_width_cells);
      int axend = std::min(t_width, x + aperture_half_width_cells);
      int aystart = std::max(0, y - aperture_half_width_cells);
      int ayend = std::min(t_height, y + aperture_half_width_cells);
      for (int ay = aystart; ay <= ayend; ay++) {
	for (int ax = axstart; ax <= axend; ax++) {
	  if ( toLight->safeAt(ax,ay) ) {
	    if ( (toLight->refAtCell(ax,ay)->red.samples > ms->config.sceneCellCountThreshold) ) {

	      double delta_x = x-ax;
	      double delta_y = y-ay;
	      double delta_z = z_to_use - array_z;
	      double gap_path_length = sqrt( delta_x*delta_x + delta_y*delta_y + delta_z*delta_z );

	      GaussianMapCell * fillCell = toFill->refAtCell(x, y);
	      GaussianMapCell * lightCell = toLight->refAtCell(ax, ay);

	      Vec3d phased_pixel;
/*
	      phased_pixel[2] = (double( lightCell->red.mu )) * (1.0+cos( gap_path_length / lambda )) * 0.5;
	      phased_pixel[1] = (double( lightCell->green.mu )) * (1.0+cos( gap_path_length / lambda )) * 0.5;
	      phased_pixel[0] = (double( lightCell->blue.mu )) * (1.0+cos( gap_path_length / lambda )) * 0.5;

	      phased_pixel[2] = (double( lightCell->red.mu )-128.0) * cos( gap_path_length / lambda ) + 128.0;
	      phased_pixel[1] = (double( lightCell->green.mu )-128.0) * cos( gap_path_length / lambda ) + 128.0;
	      phased_pixel[0] = (double( lightCell->blue.mu )-128.0) * cos( gap_path_length / lambda ) + 128.0;

	      phased_pixel[2] = (lightCell->red.mu) * cos( gap_path_length / lambda ) + 128.0;
	      phased_pixel[1] = (lightCell->green.mu) * cos( gap_path_length / lambda ) + 128.0;
	      phased_pixel[0] = (lightCell->blue.mu) * cos( gap_path_length / lambda ) + 128.0;
*/


	      phased_pixel[2] = (double( lightCell->red.mu   ) - 128.0) * 2.0 * (cos( gap_path_length / lambda ));
	      phased_pixel[1] = (double( lightCell->green.mu ) - 128.0) * 2.0 * (cos( gap_path_length / lambda ));
	      phased_pixel[0] = (double( lightCell->blue.mu  )/2.0) * (1.0 + cos( gap_path_length / lambda ));


	      fillCell->newObservation(phased_pixel, z_to_use);
	    }
	  }
	}
      }

    }
  }


}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromRegisterAtZNoRecalcPhasedArray)

WORD(SceneUpdateObservedFromRegisterAtZNoRecalcPhasedArrayKernel)
virtual void execute(MachineState * ms) {

  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);

  double array_z = 0.01;
  GET_NUMERIC_ARG(ms, array_z);

  double lambda = 0.01;
  GET_NUMERIC_ARG(ms, lambda);

  double phi = 0.01;
  GET_NUMERIC_ARG(ms, phi);

  double gain = 1.0;
  GET_NUMERIC_ARG(ms, gain);




  double lens_x = 0.01;
  GET_NUMERIC_ARG(ms, lens_x);

  double lens_y = 0.01;
  GET_NUMERIC_ARG(ms, lens_y);


  double zone_kernel_width = 101;
  GET_NUMERIC_ARG(ms, zone_kernel_width);

  double vignette_scale = 1.0;
  GET_NUMERIC_ARG(ms, vignette_scale);




  if (lambda == 0) {
    lambda = 1.0e-6;
    cout << "sceneUpdateObservedFromRegisterAtZNoRecalcPhasedArray: given 0 lambda, using " << lambda << " instead." << endl;
  }

  shared_ptr<GaussianMap> toFill = ms->config.scene->observed_map;
  shared_ptr<GaussianMap> toLight= ms->config.gaussian_map_register;

  int t_height = toFill->height;
  int t_width = toFill->width;

  int zone_kernel_half_width = (zone_kernel_width - 1)/2;

  assert( toLight->width == t_width );
  assert( toLight->height == t_height );

  double cell_width = ms->config.scene->cell_width;

  // construct the kernel
  Mat zoneKernelCos(zone_kernel_width, zone_kernel_width, CV_64F);
  Mat zoneKernelSin(zone_kernel_width, zone_kernel_width, CV_64F);
  double cos_bias = 0.0;
  double sin_bias = 0.0;
  for (int ay = 0; ay < zone_kernel_width; ay++) {
    for (int ax = 0; ax < zone_kernel_width; ax++) {

      //double delta_x = (zone_kernel_half_width-ax) * cell_width * vignette_scale;
      //double delta_y = (zone_kernel_half_width-ay) * cell_width * vignette_scale;
      double delta_x = (zone_kernel_half_width-ax) * cell_width;
      double delta_y = (zone_kernel_half_width-ay) * cell_width;
      double delta_z = z_to_use - array_z;
      double gap_path_length_squared = delta_x*delta_x + delta_y*delta_y + delta_z*delta_z;
      double gap_path_length = sqrt( gap_path_length_squared );
      double gap_path_length_plane = vignette_scale * sqrt( delta_x*delta_x + delta_y*delta_y );

      //double val_cos = ( 1.0 / gap_path_length ) * (1.0 + cos( phi + gap_path_length / lambda ))/2.0;
      //double val_cos = (1.0 + cos( phi + gap_path_length / lambda ))/2.0;
      double val_cos = ( 1.0 / (gap_path_length_plane + 1.0) ) * (1.0 + cos( phi + gap_path_length / lambda ))/2.0;
      //cout << val_cos << " " << ay << " " << ax << " " << gap_path_length << endl;
      zoneKernelCos.at<double>(ay,ax) = val_cos;
      cos_bias = cos_bias + val_cos;

      //double val_sin = ( 1.0 / gap_path_length ) * (1.0 + sin( phi + gap_path_length / lambda ))/2.0;
      //double val_sin = ( 1.0 ) * (1.0 + sin( phi + gap_path_length / lambda ))/2.0;
      double val_sin = ( 1.0 / (gap_path_length_plane + 1.0) ) * (1.0 + sin( phi + gap_path_length / lambda ))/2.0;
      //cout << val_sin << " " << ay << " " << ax << " " << gap_path_length << endl;
      zoneKernelSin.at<double>(ay,ax) = val_sin;
      sin_bias = sin_bias + val_sin;
    }
  }
  double cos_norm = 0.0;
  double sin_norm = 0.0;
  sin_bias = sin_bias / (zone_kernel_width*zone_kernel_width);
  cos_bias = cos_bias / (zone_kernel_width*zone_kernel_width);
  for (int ay = 0; ay < zone_kernel_width; ay++) {
    for (int ax = 0; ax < zone_kernel_width; ax++) {
      zoneKernelCos.at<double>(ay,ax) = zoneKernelCos.at<double>(ay,ax) - cos_bias;
      zoneKernelSin.at<double>(ay,ax) = zoneKernelSin.at<double>(ay,ax) - sin_bias;

      cos_norm = cos_norm + zoneKernelCos.at<double>(ay,ax)*zoneKernelCos.at<double>(ay,ax);
      sin_norm = sin_norm + zoneKernelSin.at<double>(ay,ax)*zoneKernelSin.at<double>(ay,ax);
    }
  }

  cos_norm = std::max(EPSILON, sqrt(cos_norm));
  sin_norm = std::max(EPSILON, sqrt(sin_norm));
  for (int ay = 0; ay < zone_kernel_width; ay++) {
    for (int ax = 0; ax < zone_kernel_width; ax++) {
      zoneKernelCos.at<double>(ay,ax) = gain * zoneKernelCos.at<double>(ay,ax) / cos_norm;
      zoneKernelSin.at<double>(ay,ax) = gain * zoneKernelSin.at<double>(ay,ax) / sin_norm;
      //zoneKernelCos.at<double>(ay,ax) = gain * zoneKernelCos.at<double>(ay,ax);
      //zoneKernelSin.at<double>(ay,ax) = gain * zoneKernelSin.at<double>(ay,ax);
    }
  }


  // apply the kernel to counts
  Mat countBuffer;
  toLight->rgbCountsToMat(countBuffer);

  // XXX should subtract the actual DC component and deal with scale better...
  double bias_estimate = 0.0;
  double one_bias_estimate = 0.0;
  {
    double delta_z = z_to_use - array_z;
    double gap_path_length = sqrt( delta_z * delta_z );
    one_bias_estimate = ( gain / gap_path_length );
    bias_estimate = one_bias_estimate * zone_kernel_width * zone_kernel_width;
  }

  cout << "EPSILON clipping: " << EPSILON << endl;
  cout << "cos_bias, cos_norm: " << cos_bias << " " << cos_norm << " " << endl;
  cout << "sin_bias, sin_norm: " << sin_bias << " " << sin_norm << " " << endl;
  cout << "bias_estimate: " << bias_estimate << endl;


  Mat outputCos;
  Mat outputSin;
  filter2D(countBuffer, outputCos, -1, zoneKernelCos, Point(-1,-1), 0, BORDER_REFLECT);
  filter2D(countBuffer, outputSin, -1, zoneKernelSin, Point(-1,-1), 0, BORDER_REFLECT);

  double fake_samples = 1000.0;
  // set counts and counts squared
  for (int y = 0; y < t_height; y++) {
    for (int x = 0; x < t_width; x++) {
      // two factors of gain is too much
      double valCos = outputCos.at<Vec3d>(x,y)[2];
      //double valSin = outputSin.at<Vec3d>(x,y)[2];
      double valSin = 0.0;
      double val = (valCos * valCos + valSin * valSin);
      toFill->refAtCell(x,y)->red.counts = val*fake_samples;
      toFill->refAtCell(x,y)->red.squaredcounts = val*val*fake_samples;
      toFill->refAtCell(x,y)->red.samples = fake_samples;

      valCos = outputCos.at<Vec3d>(x,y)[1];
      //valSin = outputSin.at<Vec3d>(x,y)[1];
      val = (valCos * valCos + valSin * valSin);
      toFill->refAtCell(x,y)->green.counts = val*fake_samples;
      toFill->refAtCell(x,y)->green.squaredcounts = val*val*fake_samples;
      toFill->refAtCell(x,y)->green.samples = fake_samples;

      valCos = outputCos.at<Vec3d>(x,y)[0];
      //valSin = outputSin.at<Vec3d>(x,y)[0];
      val = (valCos * valCos + valSin * valSin);
      toFill->refAtCell(x,y)->blue.counts = val*fake_samples;
      toFill->refAtCell(x,y)->blue.squaredcounts = val*val*fake_samples;
      toFill->refAtCell(x,y)->blue.samples = fake_samples;
    }
  }
}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromRegisterAtZNoRecalcPhasedArrayKernel)



WORD(SceneUpdateObservedFromReprojectionBufferAtZNoRecalc)
virtual void execute(MachineState * ms) {
// XXX this function, which has not been started, should
// reproject the reprojection buffer to a photograph at a single depth and store
// it in the observed map, possibly to be put onto the depth stack
  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  int thisIdx = camera->sibCurIdx;
  //cout << "sceneUpdateObservedFromStreamBuffer: " << thisIdx << endl;

  double arrayDimension = 25;
  GET_NUMERIC_ARG(ms, arrayDimension);

  double lens_strength = 0.5;
  GET_NUMERIC_ARG(ms, lens_strength);

  if (lens_strength == 0) {
    lens_strength = 1.0e-6;
    cout << "sceneUpdateObservedFromStreamBufferAtZNoRecalcSecondStageArray: given 0 lens_strength, using " << lens_strength << " instead." << endl;
  }

  double lens_gap = 0.01;
  GET_NUMERIC_ARG(ms, lens_gap);

  cout << "zToUse arrayDimension lens_strength: " << z_to_use << " " << arrayDimension << " " << lens_strength << endl;

  Mat bufferImage;
  eePose thisPose, tBaseP;


  int success = 1;
  if ( (thisIdx > -1) && (thisIdx < camera->streamImageBuffer.size()) ) {
    streamImage &tsi = camera->streamImageBuffer[thisIdx];
    loadStreamImage(ms, &tsi);
    bufferImage = tsi.image.clone();

    if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
      success = ms->getStreamPoseAtTime(tsi.time, &thisPose, &tBaseP);
    } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
      success = 1;
      thisPose = ms->config.currentEEPose;
      z_to_use = ms->config.currentEEPose.pz + ms->config.currentTableZ;
    } else {
      assert(0);
    }
  } else {
    CONSOLE_ERROR(ms, "No images in the buffer, returning." << endl);
    return;
  }

  if (success != 1) {
    CONSOLE_ERROR(ms, "  Not doing update because of stream buffer errors.");
    return;
  }

  eePose transformed = thisPose.getPoseRelativeTo(ms->config.scene->anchor_pose);
  if (fabs(transformed.qz) > 0.01) {
    CONSOLE_ERROR(ms, "  Not doing update because arm not vertical.");
    return;
  }

  Mat wristViewYCbCr = bufferImage.clone();

  cvtColor(bufferImage, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
  
  Size sz = bufferImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 
  
  pixelToGlobalCache data;
  pixelToGlobalCache data_gap;
  double z = z_to_use;
  double z_gap = z_to_use + lens_gap;
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  computePixelToPlaneCache(ms, z, thisPose, ms->config.scene->anchor_pose, &data);  
  computePixelToPlaneCache(ms, z_gap, thisPose, ms->config.scene->anchor_pose, &data_gap);  
  int numThreads = 8;
  // there is a faster way to stride it but i am risk averse atm


  int numPixels = 0;
  int numNulls = 0;


  int lens_row_spacing = ceil(ms->config.scene->height / arrayDimension);
  int lens_col_spacing = ceil(ms->config.scene->width / arrayDimension);

  cout << "lens_row_spacing lens_col_spacing: " << lens_row_spacing << " " << lens_col_spacing << endl;

  #pragma omp parallel for
  for (int i = 0; i < numThreads; i++) {
    //double frac = double(boty - topy) / double(numThreads);
    //double bfrac = i*frac;
    //double tfrac = (i+1)*frac;

    double frac = double(boty - topy) / double(numThreads);
    int ttopy = floor(topy + i*frac);
    int tboty = floor(topy + (i+1)*frac);

    //for (int py = topy; py <= boty; py++) 
      for (int py = ttopy; py < tboty; py++) 
      {

      //double opy = py-topy;
      // this is superior
      //if ( (bfrac <= opy) && (opy < tfrac) ) 
      //{
	for (int px = topx; px <= botx; px++) {
	  if (isInGripperMask(ms, px, py)) {
	    continue;
	  }

	  if ( (abhr > 0) && (abhc > 0) ) {
	    if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
		 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
	      continue;
	    } 
	  } 

	  double x, y;
	  pixelToGlobalFromCache(ms, px, py, &x, &y, &data);

	  double x_gap, y_gap;
	  pixelToGlobalFromCache(ms, px, py, &x_gap, &y_gap, &data_gap);
	  
	  if (1) {
	    // single sample update
	    int i, j;
	    ms->config.scene->observed_map->metersToCell(x, y, &i, &j);

	    // find the cell of the lens this ray falls into
	    int li, lj;
	    li = (i / lens_row_spacing) * lens_row_spacing; 
	    lj = (j / lens_col_spacing) * lens_col_spacing; 

	    GaussianMapCell * lcell = ms->config.scene->observed_map->refAtCell(li, lj);

	    //if (lcell != NULL) 
	    {

	      // find the physical coordinate of the lens, also the focal point
	      double mi, mj;
	      ms->config.scene->observed_map->cellToMeters(li, lj, &mi, &mj);

	      // shrink this ray towards the focal point
	      //double ri = ((1.0 - lens_strength) * x + (lens_strength) * mi) * (1.0-lens_strength);
	      //double rj = ((1.0 - lens_strength) * y + (lens_strength) * mj) * (1.0-lens_strength);
	
	      // projected angle is proportional to displacement caused by the gap
	      // the bigger the gap the more angles we get

	      double ri = ((x_gap-x)/(lens_strength) + mi);
	      double rj = ((y_gap-y)/(lens_strength) + mj);

	      // and put it in that cell
	      int cri, crj;
	      ms->config.scene->observed_map->metersToCell(ri, rj, &cri, &crj);

	      GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(cri, crj);
	      if (cell != NULL) {
		  Vec3b pixel = wristViewYCbCr.at<Vec3b>(py, px);
		  cell->newObservation(pixel, z);
		numPixels++;
	      } else {
		  numNulls++;
	      }
	    }
	  }
	}
      //}
    }
  }
  //cout << "numPixels numNulls sum apertureSize: " << numPixels << " " << numNulls << " " << numPixels + numNulls << " " << ms->config.angular_aperture_rows * ms->config.angular_aperture_cols << endl;
  //ms->config.scene->observed_map->recalculateMusAndSigmas(ms);
}
// XXX
END_WORD
REGISTER_WORD(SceneUpdateObservedFromReprojectionBufferAtZNoRecalc)


WORD(SceneCopyObservedToReprojectionBuffer)
virtual void execute(MachineState * ms) {
// XXX
}
END_WORD
REGISTER_WORD(SceneCopyObservedToReprojectionBuffer)


CONFIG_SETTER_INT(SceneSetAngularApertureRows, ms->config.angular_aperture_rows)
CONFIG_GETTER_INT(SceneAngularApertureRows, ms->config.angular_aperture_rows)

CONFIG_GETTER_INT(SceneAngularApertureCols, ms->config.angular_aperture_cols)
CONFIG_SETTER_INT(SceneSetAngularApertureCols, ms->config.angular_aperture_cols)

CONFIG_SETTER_INT(SceneSetAngularBaffleRows, ms->config.angular_baffle_rows)
CONFIG_GETTER_INT(SceneAngularBaffleRows, ms->config.angular_baffle_rows)

CONFIG_GETTER_INT(SceneAngularBaffleCols, ms->config.angular_baffle_cols)
CONFIG_SETTER_INT(SceneSetAngularBaffleCols, ms->config.angular_baffle_cols)

CONFIG_GETTER_INT(SceneDepthPatchHalfWidth, ms->config.sceneDepthPatchHalfWidth)
CONFIG_SETTER_INT(SceneSetDepthPatchHalfWidth, ms->config.sceneDepthPatchHalfWidth)

void sceneMinIntoRegisterHelper(MachineState * ms, shared_ptr<GaussianMap> toMin) {

  int sceneDepthPatchHalfWidth = ms->config.sceneDepthPatchHalfWidth;

  int t_height = toMin->height;
  int t_width = toMin->width;

  assert( ms->config.gaussian_map_register->width == t_width );
  assert( ms->config.gaussian_map_register->height == t_height );

  for (int y = 0; y < t_height; y++) {
    for (int x = 0; x < t_width; x++) {
      if ( (toMin->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold) ) {


	/*
	  // one by one
	  double this_observed_sigma_squared = 
	  toMin->refAtCell(x,y)->red.sigmasquared +
	  toMin->refAtCell(x,y)->green.sigmasquared +
	  toMin->refAtCell(x,y)->blue.sigmasquared;

	  double this_register_sigma_squared = 
	  ms->config.gaussian_map_register->refAtCell(x,y)->red.sigmasquared +
	  ms->config.gaussian_map_register->refAtCell(x,y)->green.sigmasquared +
	  ms->config.gaussian_map_register->refAtCell(x,y)->blue.sigmasquared;
	*/

	/*
	// XXX can't do it over a patch here because you need the old neighborhoods.
	double this_observed_sigma_squared = 0;
	double this_register_sigma_squared = 0;
	int pxmin = std::max(x - sceneDepthPatchHalfWidth, 0);
	int pxmax = std::min(x + sceneDepthPatchHalfWidth, t_width-1);
	int pymin = std::max(y - sceneDepthPatchHalfWidth, 0);
	int pymax = std::min(y + sceneDepthPatchHalfWidth, t_height-1);

	double ss_r = 0;
	double ss_g = 0;
	double ss_b = 0;

	for (int py = pymin; py <= pymax; py++) {
	  for (int px = pxmin; px <= pxmax; px++) {
	    ss_r = ss_r + toMin->refAtCell(px,py)->red.sigmasquared;
	    ss_g = ss_g + toMin->refAtCell(px,py)->green.sigmasquared;
	    ss_b = ss_b + toMin->refAtCell(px,py)->blue.sigmasquared;
	  }
	}
	this_observed_sigma_squared = this_observed_sigma_squared +
	ss_r +
	ss_g +
	ss_b;

	this_register_sigma_squared = 
	ms->config.gaussian_map_register->refAtCell(x,y)->red.sigmasquared +
	ms->config.gaussian_map_register->refAtCell(x,y)->green.sigmasquared +
	ms->config.gaussian_map_register->refAtCell(x,y)->blue.sigmasquared;
	
	double thisObservedPixelArea = 1.0;
	double thisRegisterPixelArea = 1.0;
	*/
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

	double meters_scene_x, meters_scene_y;
	toMin->cellToMeters(x, y, &meters_scene_x, &meters_scene_y);


	int reg_x, reg_y;
	ms->config.gaussian_map_register->metersToCell(meters_scene_x, meters_scene_y, &reg_x, &reg_y);

	double this_observed_sigma_squared = 0;
	double this_register_sigma_squared = 0;
	int pxmin = std::max(x - sceneDepthPatchHalfWidth, 0);
	int pxmax = std::min(x + sceneDepthPatchHalfWidth, t_width-1);
	int pymin = std::max(y - sceneDepthPatchHalfWidth, 0);
	int pymax = std::min(y + sceneDepthPatchHalfWidth, t_height-1);

	double ss_r = 0;
	double ss_g = 0;
	double ss_b = 0;

	for (int py = pymin; py <= pymax; py++) {
	  for (int px = pxmin; px <= pxmax; px++) {
	    ss_r = ss_r + toMin->refAtCell(px,py)->red.sigmasquared;
	    ss_g = ss_g + toMin->refAtCell(px,py)->green.sigmasquared;
	    ss_b = ss_b + toMin->refAtCell(px,py)->blue.sigmasquared;
	  }
	}
	this_observed_sigma_squared = this_observed_sigma_squared +
	ss_r +
	ss_g +
	ss_b;

	this_register_sigma_squared = 
	  ms->config.gaussian_map_register->refAtCell(reg_x,reg_y)->red.sigmasquared +
	  ms->config.gaussian_map_register->refAtCell(reg_x,reg_y)->green.sigmasquared +
	  ms->config.gaussian_map_register->refAtCell(reg_x,reg_y)->blue.sigmasquared;
	
	double thisObservedPixelArea = 1.0;
	double thisRegisterPixelArea = 1.0;

	// checking if counts are zero allows filling in of missing data from multiple maps
	if( ms->config.gaussian_map_register->safeAt(reg_x,reg_y) ) {
	  if (
	      ( this_observed_sigma_squared/thisObservedPixelArea < this_register_sigma_squared/thisRegisterPixelArea ) ||
	      ( ms->config.gaussian_map_register->refAtCell(reg_x,reg_y)->red.samples <= 0 )
	     ) {
	    *(ms->config.gaussian_map_register->refAtCell(reg_x,reg_y)) = *(toMin->refAtCell(x,y));

	    ms->config.gaussian_map_register->refAtCell(reg_x,reg_y)->red.sigmasquared = ss_r;
	    ms->config.gaussian_map_register->refAtCell(reg_x,reg_y)->green.sigmasquared = ss_g;
	    ms->config.gaussian_map_register->refAtCell(reg_x,reg_y)->blue.sigmasquared = ss_b;
	  } 
	}

      } else {
      }
    }
  }

}

void sceneMarginalizeIntoRegisterHelper(MachineState * ms, shared_ptr<GaussianMap> toMin, Mat weights) {

// XXX 
// XXX  NOT DONE TODO
// XXX need to divide by the total probability so that z is correctly scaled. now more certain regions are brighter in z
// XXX partially addressed
// XXX 
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
	  double rescalar = 128.0;
	  double resamples = 256.0;
	  //cout << " hit " << this_observed_sigma_squared << " " ;
	  GaussianMapCell toAdd = *(toMin->refAtCell(x,y));
	  // renormalize
	  //toAdd.multS(resamples/toAdd.red.samples);
	  //  actually the map is smoother and more consistent when the sample information is kept

	  this_observed_sigma_squared = max(this_observed_sigma_squared, 1.0);
	  double thisweight = rescalar/sqrt(this_observed_sigma_squared * 2.0 * M_PI);
	  weights.at<double>(x,y) += thisweight;
	  toAdd.multS(thisweight);
	  // XXX seems like this should perform a cell to meter meter to cell like the min function.
	  ms->config.gaussian_map_register->refAtCell(x,y)->addC(&toAdd);

	  if ( ( ms->config.gaussian_map_register->refAtCell(x,y)->red.samples < 1.1 ) &&
	       ( ms->config.gaussian_map_register->refAtCell(x,y)->red.samples > 0.0 ) ) {
	    //ms->config.gaussian_map_register->refAtCell(x,y)->multS( 1.0 / ms->config.gaussian_map_register->refAtCell(x,y)->red.samples ); 
	  } else {}

	  // do this outside
	  //ms->config.gaussian_map_register->refAtCell(x,y)->recalculateMusAndSigmas(ms);
      } else {
      }
    }
  }

}

WORD(SceneFlattenUncertainZWithDepthStack)
virtual void execute(MachineState * ms) {
// XXX TODO RELEASE  this should be good for segmentation and grasp proposal
//  if the max depth estimate does not own enough of the probability share, this depth is
//  set to the maximum depth, saying if it was hazy to us consider it background.

/*
  for (int y = 0; y < t_height; y++) {
    for (int x = 0; x < t_width; x++) {
      if ( (toMin->refAtCell(x,y)->red.samples > ms->config.sceneCellCountThreshold) ) {
      }
    }
  }
*/

}
END_WORD
REGISTER_WORD(SceneFlattenUncertainZWithDepthStack)

WORD(SceneMinIntoRegister)
virtual void execute(MachineState * ms) {
  cout << "sceneMinIntoRegister: copying..." << endl;

  sceneMinIntoRegisterHelper(ms, ms->config.scene->observed_map);
}
END_WORD
REGISTER_WORD(SceneMinIntoRegister)

WORD(SceneTrimDepthWithDiscrepancy)
virtual void execute(MachineState * ms) {
  double thresh = 0.0;
  GET_NUMERIC_ARG(ms, thresh);

  double zToUse = 0;
  GET_NUMERIC_ARG(ms, zToUse);

  cout << "sceneTrimDepthWithDiscrepancy: trimming with z " << zToUse << endl;
  int t_height = ms->config.scene->observed_map->height;
  int t_width = ms->config.scene->observed_map->width;
  for (int y = 0; y < t_height; y++) {
    for (int x = 0; x < t_width; x++) {
      if (ms->config.scene->discrepancy_density.at<double>(x,y) > thresh) {
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

WORD(SceneLoadLightingModelFromGaussianMapFile)
virtual void execute(MachineState * ms) {
  GaussianMapCell * lm = &(ms->config.scene->light_model);
  shared_ptr<GaussianMap> toLoad = std::make_shared<GaussianMap>(ms, 1, 1, 1.0, eePose::identity());

// XXX NOT TESTED
  string message;
  GET_STRING_ARG(ms, message);

  stringstream ss;
  ss << ms->config.data_directory + "/maps/" + message + ".yml";
  stringstream ss_dir;
  ss_dir << ms->config.data_directory + "/maps/";
  mkdir(ss_dir.str().c_str(), 0777);

  toLoad->loadFromFile(ss.str());
  *(lm) = *(toLoad->refAtCell(0,0));

  
  cout << "sceneLoadLightingModelFromGaussianMapFile: loaded rmu gmu bmu, " << 
    lm->red.mu << " " <<
    lm->green.mu << " " <<
    lm->blue.mu << " " <<
      endl;
}
END_WORD
REGISTER_WORD(SceneLoadLightingModelFromGaussianMapFile)

WORD(SceneSaveLightingModelToGaussianMapFile)
virtual void execute(MachineState * ms) {
// XXX NOT TESTED
  GaussianMapCell * lm = &(ms->config.scene->light_model);
  shared_ptr<GaussianMap> toSave= std::make_shared<GaussianMap>(ms, 1, 1, 1.0, eePose::identity());
  *(toSave->refAtCell(0,0)) = *(lm);

  string message;
  GET_STRING_ARG(ms, message);

  stringstream ss;
  ss << ms->config.data_directory + "/maps/" + message + ".yml";
  stringstream ss_dir;
  ss_dir << ms->config.data_directory + "/maps/";
  mkdir(ss_dir.str().c_str(), 0777);

  toSave->saveToFile(ss.str());

  cout << "sceneSaveLightingModelFromGaussianMapFile: saving rmu gmu bmu, " << 
    lm->red.mu << " " <<
    lm->green.mu << " " <<
    lm->blue.mu << " " <<
      endl;
}
END_WORD
REGISTER_WORD(SceneSaveLightingModelToGaussianMapFile)

WORD(SceneSetLightingModelFromStack)
virtual void execute(MachineState * ms) {

  GaussianMapCell * lm = &(ms->config.scene->light_model);

  GET_NUMERIC_ARG(ms, lm->blue.mu);
  GET_NUMERIC_ARG(ms, lm->green.mu);
  GET_NUMERIC_ARG(ms, lm->red.mu);

  cout << "sceneSetLightModelFromStack: pushing rmu gmu bmu, " << 
    lm->red.mu << " " <<
    lm->green.mu << " " <<
    lm->blue.mu << " " <<
      endl;
}
END_WORD
REGISTER_WORD(SceneSetLightingModelFromStack)

WORD(ScenePushLightingModel)
virtual void execute(MachineState * ms) {

  GaussianMapCell * lm = &(ms->config.scene->light_model);
  cout << "scenePushLightingModel: pushing rmu gmu bmu, " << 
    lm->red.mu << " " <<
    lm->green.mu << " " <<
    lm->blue.mu << " " <<
      endl;

  ms->pushData( make_shared<DoubleWord>(lm->red.mu) );
  ms->pushData( make_shared<DoubleWord>(lm->green.mu) );
  ms->pushData( make_shared<DoubleWord>(lm->blue.mu) );
}
END_WORD
REGISTER_WORD(ScenePushLightingModel)

WORD(SceneSetLightModelFromDiscrepancy)
virtual void execute(MachineState * ms) {
  double thresh = 0.0;
  GET_NUMERIC_ARG(ms, thresh);

  GaussianMapCell * lm = &(ms->config.scene->light_model);
  lm->zero();

  int t_height = ms->config.scene->observed_map->height;
  int t_width = ms->config.scene->observed_map->width;
  for (int y = 0; y < t_height; y++) {
    for (int x = 0; x < t_width; x++) {
      if (ms->config.scene->discrepancy_density.at<double>(x,y) > thresh) {
	GaussianMapCell * mm = ms->config.scene->observed_map->refAtCell(x,y);
	lm->newObservation( Vec3b(mm->blue.mu,mm->green.mu,mm->red.mu) );
	// maybe should be weighting this by the discrepancy
      } else {
	ms->config.scene->discrepancy_density.at<double>(x,y) = 0.0;
      }
    }
  }

  lm->recalculateMusAndSigmas(ms);
  
  CONSOLE(ms, "lighting model r g b:" << lm->red.mu << " " << lm->green.mu << " " << lm->blue.mu << endl);
}
END_WORD
REGISTER_WORD(SceneSetLightModelFromDiscrepancy)

WORD(SceneSmoothSquaredCountsAndSamplesXY)
virtual void execute(MachineState * ms) {
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

      CELLREF_EQUALS_VEC3(ms->config.scene->observed_map->refAtCell(x,y), ss.at<Vec3d>(x,y), sigmasquared);

    }
  }
  //ms->config.scene->observed_map->recalculateMusAndSigmas(ms);
}
END_WORD
REGISTER_WORD(SceneSmoothSquaredCountsAndSamplesXY)

WORD(SceneSmoothDepthStackInZ)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  cout << "scenePushOntoDepthStack: " << endl;
  ms->config.depth_maps.push_back(ms->config.scene->observed_map->copy());
}
END_WORD
REGISTER_WORD(ScenePushOntoDepthStack)

WORD(ScenePushDepthStackSize)
virtual void execute(MachineState * ms) {
  cout << "scenePushDepthStackSize: " << ms->config.depth_maps.size() << endl;
  ms->pushWord( make_shared<DoubleWord>(ms->config.depth_maps.size()) );
}
END_WORD
REGISTER_WORD(ScenePushDepthStackSize)

WORD(SceneClearDepthStack)
virtual void execute(MachineState * ms) {
  cout << "sceneClearDepthStack: " << endl;
  ms->config.depth_maps.resize(0);
}
END_WORD
REGISTER_WORD(SceneClearDepthStack)

WORD(SceneMinDepthStackIntoRegister)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
  cout << "sceneMarginalizeDepthStackIntoRegister: " << endl;

  int tns = ms->config.depth_maps.size();

  Mat totalMarginalWeight;
  if (tns > 0) {
    totalMarginalWeight = Mat(ms->config.depth_maps[0]->width, ms->config.depth_maps[0]->height, CV_64F);
    //for (int y = 0; y < height; y++) 
    //  for (int x = 0; x < width; x++) 
    for (int y = 0; y < totalMarginalWeight.cols; y++) {
      for (int x = 0; x < totalMarginalWeight.rows; x++) {
	totalMarginalWeight.at<double>(x,y) = 0.0;
      }
    }
  }

  for (int i = 0; i < tns; i++) {
    cout << "  marginalizing" << i << " with max " << tns << endl;
    sceneMarginalizeIntoRegisterHelper(ms, ms->config.depth_maps[i], totalMarginalWeight);
  }

  double p_marginalize_scale = 100.0;
  if (tns > 0) {
    for (int y = 0; y < totalMarginalWeight.cols; y++) {
      for (int x = 0; x < totalMarginalWeight.rows; x++) {

	int numMapsPresented = 0;
	for (int i = 0; i < tns; i++) {
	  GaussianMapCell * toCount = (ms->config.depth_maps[i]->refAtCell(x,y));

	  if ( toCount->red.samples > ms->config.sceneCellCountThreshold)  {
	    numMapsPresented++;
	  } else {
	    break;
	  }
	}

	GaussianMapCell * toScale = (ms->config.gaussian_map_register->refAtCell(x,y));
	// trim the cells that were not present at all depths
	if (numMapsPresented < tns) {
	  toScale->zero();
	  continue;
	}
	
	double denom = 1.0;
	if ( fabs(totalMarginalWeight.at<double>(x,y)) > 0.0 ) {
	  denom = totalMarginalWeight.at<double>(x,y);
	}
	toScale->multS(p_marginalize_scale/denom);

	// recalculating here is necessary because of how the help function works
	toScale->recalculateMusAndSigmas(ms);
      }
    }
  }
}
END_WORD
REGISTER_WORD(SceneMarginalizeDepthStackIntoRegister)

WORD(SceneRecallDepthStackIndex)
virtual void execute(MachineState * ms) {
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


WORD(SceneAddDiscrepantPredictedToObserved)
virtual void execute(MachineState * ms) {
  double learning_weight = 0.0;
  GET_NUMERIC_ARG(ms, learning_weight);
  cout << "sceneAddDiscrepantPredictedToObserved: using learning rate " << learning_weight << endl;
  ms->config.scene->addPredictedObjectsToObservedMap(ms->config.scene_score_thresh, learning_weight);
}
END_WORD
REGISTER_WORD(SceneAddDiscrepantPredictedToObserved)




WORD(SceneSaveObservedMapImage)
virtual void execute(MachineState * ms) {
  string fname;
  GET_STRING_ARG(ms, fname);
  
  Mat image;
  ms->config.scene->observed_map->rgbMuToMat(image);
  Mat rgb = image.clone();  
  cvtColor(image, rgb, cv::COLOR_YCrCb2BGR);
  cout << "Writing " << fname << endl;
  imwrite(fname, rgb);

}
END_WORD
REGISTER_WORD(SceneSaveObservedMapImage)

WORD(SceneSaveDiscrepancyDensityImage)
virtual void execute(MachineState * ms) {
  string fname;
  GET_STRING_ARG(ms, fname);
  
  Mat image = ms->config.scene->discrepancy_density;
  cout << "Writing " << fname << endl;
  imwrite(fname, 255.0 * image);

}
END_WORD
REGISTER_WORD(SceneSaveDiscrepancyDensityImage)


WORD(SceneLoadMonochromeBackground)
virtual void execute(MachineState * ms) {
  int r, g, b;
  GET_INT_ARG(ms, b);
  GET_INT_ARG(ms, g);
  GET_INT_ARG(ms, r);

  //ms->config.scene->background_map = ms->config.scene->observed_map->copy();
  shared_ptr<GaussianMap> bg = ms->config.scene->background_map;

  for (int y = 0; y < bg->height; y++) {
    for (int x = 0; x < bg->width; x++) {
      bg->refAtCell(x,y)->zero();
      for (int i = 0; i < ms->config.sceneCellCountThreshold * 2; i++) {
	bg->refAtCell(x,y)->newObservation(Vec3b(b,g,r));
      }
    }
  }
  bg->recalculateMusAndSigmas(ms);
}
END_WORD
REGISTER_WORD(SceneLoadMonochromeBackground)


WORD(SceneDepthStackLoadAndPushRaw)
virtual void execute(MachineState * ms) {
// XXX 
  string depthStackFolderPath;
  GET_STRING_ARG(ms, depthStackFolderPath);

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  dpdf = opendir(depthStackFolderPath.c_str());
  if (dpdf != NULL){
    cout << "sceneDepthStackLoadAndPushRaw: checking " << depthStackFolderPath << " during snoop...";
    while (epdf = readdir(dpdf)){
      string thisFileName(epdf->d_name);

      string thisFullFileName(depthStackFolderPath.c_str());
      thisFullFileName = thisFullFileName + "/" + thisFileName;
      cout << "sceneDepthStackLoadAndPushRaw: checking " << thisFullFileName << " during snoop...";

      struct stat buf2;
      stat(thisFullFileName.c_str(), &buf2);

      int itIsADir = S_ISDIR(buf2.st_mode);
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && itIsADir) {
	cout << " is a directory." << endl;
      } else if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {
	cout << " is NOT a directory." << endl;
	shared_ptr<Scene> this_scene = Scene::createFromFile(ms, thisFullFileName);
	shared_ptr<GaussianMap> this_map = this_scene->observed_map;
	ms->config.depth_maps.push_back(this_map);
      }
    }
  } else {
    CONSOLE_ERROR(ms, "sceneDepthStackLoadAndPushRaw: could not open base class dir " << depthStackFolderPath << " ." << endl);
  } 
}
END_WORD
REGISTER_WORD(SceneDepthStackLoadAndPushRaw)

WORD(SceneDepthStackLoadAndCropRaw)
virtual void execute(MachineState * ms) {
  int x1,y1,x2,y2;
  GET_NUMERIC_ARG(ms,y2);
  GET_NUMERIC_ARG(ms,x2);
  GET_NUMERIC_ARG(ms,y1);
  GET_NUMERIC_ARG(ms,x1);

  string depthStackFolderPath;
  GET_STRING_ARG(ms, depthStackFolderPath);

  cout << "sceneDepthStackLoadAndCropRaw got path x1 y1 x2 y2: " << depthStackFolderPath << " " << x1 << " " << y1 << " " << x2 << " " << y2 << endl; 

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  dpdf = opendir(depthStackFolderPath.c_str());
  if (dpdf != NULL){
    cout << "sceneDepthStackLoadAndCropRaw: checking " << depthStackFolderPath << " during snoop...";
    while (epdf = readdir(dpdf)){
      string thisFileName(epdf->d_name);

      string thisFullFileName(depthStackFolderPath.c_str());
      thisFullFileName = thisFullFileName + "/" + thisFileName;
      cout << "sceneDepthStackLoadAndCropRaw: checking " << thisFullFileName << " during snoop...";

      struct stat buf2;
      stat(thisFullFileName.c_str(), &buf2);

      string extension;
      if (thisFullFileName.size() >=3 ) {
	extension = thisFullFileName.substr(thisFullFileName.size()-3, 3);
      } else {
      }

      int itIsADir = S_ISDIR(buf2.st_mode);
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && itIsADir) {
	cout << " is a directory." << endl;
      } else if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && !extension.compare("yml")) {
	cout << " is NOT a directory." << endl;
	shared_ptr<Scene> this_scene = Scene::createFromFile(ms, thisFullFileName);
	shared_ptr<GaussianMap> this_map = this_scene->observed_map;

	shared_ptr<GaussianMap> this_crop = this_map->copyBox(x1, y1, x2, y2);

	stringstream ss;
	ss << thisFullFileName << ".png";
	Mat toConvert;
	Mat toWrite;
	this_crop->rgbMuToMat(toConvert);
	cvtColor(toConvert, toWrite, cv::COLOR_YCrCb2BGR);
	imwrite(ss.str(), toWrite);
      }
    }
  } else {
    CONSOLE_ERROR(ms, "sceneDepthStackLoadAndCropRaw: could not open base class dir " << depthStackFolderPath << " ." << endl);
  } 
}
END_WORD
REGISTER_WORD(SceneDepthStackLoadAndCropRaw)

WORD(SceneDepthStackLoadAndMarginalizeRaw)
virtual void execute(MachineState * ms) {
  string depthStackFolderPath;
  GET_STRING_ARG(ms, depthStackFolderPath);

  cout << "sceneDepthStackLoadAndMarginalizeRaw got path: " << depthStackFolderPath << endl; 

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  dpdf = opendir(depthStackFolderPath.c_str());
  if (dpdf != NULL){
    cout << "sceneDepthStackLoadAndMarginalizeRaw: checking " << depthStackFolderPath << " during snoop...";
    while (epdf = readdir(dpdf)){
      string thisFileName(epdf->d_name);

      string thisFullFileName(depthStackFolderPath.c_str());
      thisFullFileName = thisFullFileName + "/" + thisFileName;
      cout << "sceneDepthStackLoadAndMarginalizeRaw: checking " << thisFullFileName << " during snoop...";

      struct stat buf2;
      stat(thisFullFileName.c_str(), &buf2);

      string extension;
      if (thisFullFileName.size() >=3 ) {
	extension = thisFullFileName.substr(thisFullFileName.size()-3, 3);
      } else {
      }

      int itIsADir = S_ISDIR(buf2.st_mode);
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && itIsADir) {
	cout << " is a directory." << endl;
      } else if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && !extension.compare("yml")) {
	cout << " is NOT a directory." << endl;
	shared_ptr<Scene> this_scene = Scene::createFromFile(ms, thisFullFileName);
	shared_ptr<GaussianMap> this_map = this_scene->observed_map;

	Mat totalMarginalWeight;
	totalMarginalWeight = Mat(this_map->width, this_map->height, CV_64F);
	for (int y = 0; y < totalMarginalWeight.cols; y++) {
	  for (int x = 0; x < totalMarginalWeight.rows; x++) {
	    totalMarginalWeight.at<double>(x,y) = 0.0;
	  }
	}

	sceneMarginalizeIntoRegisterHelper(ms, this_map, totalMarginalWeight);
      }
    }
  } else {
    CONSOLE_ERROR(ms, "sceneDepthStackLoadAndMarginalizeRaw: could not open base class dir " << depthStackFolderPath << " ." << endl);
  } 
}
END_WORD
REGISTER_WORD(SceneDepthStackLoadAndMarginalizeRaw)

WORD(SceneDepthStackLoadAndMinRaw)
virtual void execute(MachineState * ms) {
  string depthStackFolderPath;
  GET_STRING_ARG(ms, depthStackFolderPath);

  cout << "sceneDepthStackLoadAndMinRaw got path: " << depthStackFolderPath << endl; 

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  dpdf = opendir(depthStackFolderPath.c_str());
  if (dpdf != NULL){
    cout << "sceneDepthStackLoadAndMinRaw: checking " << depthStackFolderPath << " during snoop...";
    while (epdf = readdir(dpdf)){
      string thisFileName(epdf->d_name);

      string thisFullFileName(depthStackFolderPath.c_str());
      thisFullFileName = thisFullFileName + "/" + thisFileName;
      cout << "sceneDepthStackLoadAndMinRaw: checking " << thisFullFileName << " during snoop...";

      struct stat buf2;
      stat(thisFullFileName.c_str(), &buf2);

      string extension;
      if (thisFullFileName.size() >=3 ) {
	extension = thisFullFileName.substr(thisFullFileName.size()-3, 3);
      } else {
      }

      int itIsADir = S_ISDIR(buf2.st_mode);
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && itIsADir) {
	cout << " is a directory." << endl;
      } else if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && !extension.compare("yml")) {
	cout << " is NOT a directory." << endl;
	shared_ptr<Scene> this_scene = Scene::createFromFile(ms, thisFullFileName);
	shared_ptr<GaussianMap> this_map = this_scene->observed_map;

	sceneMinIntoRegisterHelper(ms, this_map);
      }
    }
  } else {
    CONSOLE_ERROR(ms, "sceneDepthStackLoadAndMinRaw: could not open base class dir " << depthStackFolderPath << " ." << endl);
  } 
}
END_WORD
REGISTER_WORD(SceneDepthStackLoadAndMinRaw)

WORD(SceneDepthStackSaveRaw)
virtual void execute(MachineState * ms) {
  string depthStackFolderPath;
  GET_STRING_ARG(ms, depthStackFolderPath);

  for (int i = 0; i < ms->config.depth_maps.size(); i++) {
    stringstream ss;
    ss << depthStackFolderPath << "/stack" << std::setprecision(5) << i << ".yml";
    string thisFilePath = ss.str();
    ms->config.depth_maps[i]->saveToFile(thisFilePath);
  }
}
END_WORD
REGISTER_WORD(SceneDepthStackSaveRaw)

WORD(SceneDepthStackSavePngRaw)
virtual void execute(MachineState * ms) {
  string depthStackFolderPath;
  GET_STRING_ARG(ms, depthStackFolderPath);

  for (int i = 0; i < ms->config.depth_maps.size(); i++) {
    stringstream ss;
    ss << depthStackFolderPath << "/stack" << std::setprecision(5) << i << ".png";

    Mat toConvert;
    Mat toWrite;
    ms->config.depth_maps[i]->rgbMuToMat(toConvert);
    cvtColor(toConvert, toWrite, cv::COLOR_YCrCb2BGR);
    imwrite(ss.str(), toWrite);
  }
}
END_WORD
REGISTER_WORD(SceneDepthStackSavePngRaw)


WORD(SceneRegularizeSceneL2)
virtual void execute(MachineState * ms) {
  double decay = 1.0;
  GET_NUMERIC_ARG(ms, decay);
  for (int y = 0; y < ms->config.scene->height; y++) {
    for (int x = 0; x < ms->config.scene->width; x++) {
      if (ms->config.scene->observed_map->refAtCell(x, y)->red.samples > 40) {
	ms->config.scene->observed_map->refAtCell(x,y)->multS(decay);
      }
    }
  }
  //ms->config.scene->observed_map->multS(decay);
  ms->config.scene->observed_map->recalculateMusAndSigmas(ms);
}
END_WORD
REGISTER_WORD(SceneRegularizeSceneL2)


WORD(ScenePushPixelOfMinVariance)
virtual void execute(MachineState * ms) {
  // convert map cell into global xy
  // convert global xy into pixels
  double minEnergy = DBL_MAX;
  int minEnergyX = -1;
  int minEnergyY = -1;
  double maxSamples = 0;
  for (int y = 0; y < ms->config.scene->height; y++) {
    for (int x = 0; x < ms->config.scene->width; x++) {
      if (ms->config.scene->observed_map->refAtCell(x, y)->red.samples >= maxSamples) {
        maxSamples = ms->config.scene->observed_map->refAtCell(x, y)->red.samples;
      }
      if (ms->config.scene->observed_map->refAtCell(x, y)->red.samples > 40) {
	// this is for FIXATE_STREAM so ignore Y channel
	double thisEnergy = 
	  ms->config.scene->observed_map->refAtCell(x,y)->red.sigmasquared +
	  ms->config.scene->observed_map->refAtCell(x,y)->green.sigmasquared; 
	// this is assuming image still in RGB due to FIXATE_CURRENT so use all 3 channels
	//double thisEnergy = 
	  //ms->config.scene->observed_map->refAtCell(x,y)->red.sigmasquared +
	  //ms->config.scene->observed_map->refAtCell(x,y)->green.sigmasquared +
	   //+ ms->config.scene->observed_map->refAtCell(x,y)->blue.sigmasquared;

	if (thisEnergy < minEnergy) {
	  minEnergy = thisEnergy;
	  minEnergyX = x;
	  minEnergyY = y;
	}
      }
    }
  }

  if (minEnergyX == -1 || minEnergyY == -1) {
    CONSOLE_ERROR(ms, "scenePushPixelOfMinVariance: Did not update minEnergy, were there enough samples?  minEnergyX: " 
      << minEnergyX << " minEnergyY: " << minEnergyY << " maxSamples: " << maxSamples);
    return;
  } else {
    CONSOLE(ms, "scenePushPixelOfMinVariance: Updated minEnergy, minEnergyX: " << minEnergyX << " minEnergyY: " << minEnergyY << " maxSamples: " << maxSamples);
  }
  
  double meters_scene_x, meters_scene_y;
  ms->config.scene->observed_map->cellToMeters(minEnergyX, minEnergyY, &meters_scene_x, &meters_scene_y);

  double zToUse = ms->config.currentEEPose.pz + ms->config.currentTableZ;
  int pixel_scene_x, pixel_scene_y;
  globalToPixel(ms, &pixel_scene_x, &pixel_scene_y, zToUse, meters_scene_x, meters_scene_y, ms->config.straightDown);

  CONSOLE(ms, "scenePushPixelOfMinVariance x, y: " << pixel_scene_x << " " << pixel_scene_y << endl);
  CONSOLE(ms, "scenePushPixelOfMinVariance r,g mus: " << ms->config.scene->observed_map->refAtCell(minEnergyX,minEnergyY)->red.mu << " " << ms->config.scene->observed_map->refAtCell(minEnergyX,minEnergyY)->green.mu << endl);

  ms->pushWord(make_shared<DoubleWord>(pixel_scene_x));
  ms->pushWord(make_shared<DoubleWord>(pixel_scene_y));
}
END_WORD
REGISTER_WORD(ScenePushPixelOfMinVariance)

WORD(ScenePushPixelOfMinStackVariance)
virtual void execute(MachineState * ms) {
  // convert map cell into global xy
  // convert global xy into pixels
  double minEnergy = DBL_MAX;
  int minEnergyX = -1;
  int minEnergyY = -1;

  for (int y = 0; y < ms->config.scene->height; y++) {
    for (int x = 0; x < ms->config.scene->width; x++) {
      double thisTotalEnergy = 0.0;
      double thisSamples = 0.0;
      for (int i = 0; i < ms->config.depth_maps.size(); i++) {
	if (ms->config.depth_maps[i]->refAtCell(x, y)->red.samples > 40) {
	  // this is for FIXATE_STREAM so ignore Y channel
	  thisTotalEnergy = thisTotalEnergy + 
	    ms->config.depth_maps[i]->refAtCell(x,y)->red.sigmasquared +
	    ms->config.depth_maps[i]->refAtCell(x,y)->green.sigmasquared; 
	  // this is assuming image still in RGB due to FIXATE_CURRENT so use all 3 channels
	  //thisTotalEnergy = thisTotalEnergy + 
	    //ms->config.depth_maps[i]->refAtCell(x,y)->red.sigmasquared +
	    //ms->config.depth_maps[i]->refAtCell(x,y)->green.sigmasquared +
	     //+ ms->config.depth_maps[i]->refAtCell(x,y)->blue.sigmasquared;

	  thisSamples = thisSamples + 1;
	}
      }
      // XXX this should take a running average of those equal to the current winner
      if (thisSamples > 0 ) {
	double thisEnergy = thisTotalEnergy / thisSamples;
	if (thisEnergy < minEnergy) {
	  minEnergy = thisEnergy;
	  minEnergyX = x;
	  minEnergyY = y;
	}
      }
    }
  }

  double meters_scene_x, meters_scene_y;
  ms->config.scene->observed_map->cellToMeters(minEnergyX, minEnergyY, &meters_scene_x, &meters_scene_y);

  double zToUse = ms->config.currentEEPose.pz + ms->config.currentTableZ;
  int pixel_scene_x, pixel_scene_y;
  globalToPixel(ms, &pixel_scene_x, &pixel_scene_y, zToUse, meters_scene_x, meters_scene_y, ms->config.straightDown);

  cout << "scenePushPixelOfMinStackVariance x, y: " << pixel_scene_x << " " << pixel_scene_y << endl;
  cout << "scenePushPixelOfMinStackVariance r,g mus: " << ms->config.scene->observed_map->refAtCell(minEnergyX,minEnergyY)->red.mu << " " << ms->config.scene->observed_map->refAtCell(minEnergyX,minEnergyY)->green.mu << endl;

  ms->pushWord(make_shared<DoubleWord>(pixel_scene_x));
  ms->pushWord(make_shared<DoubleWord>(pixel_scene_y));
}
END_WORD
REGISTER_WORD(ScenePushPixelOfMinStackVariance)

WORD(SceneSetVanishingPointFromPixel)
virtual void execute(MachineState * ms) {
  int pixel_scene_x, pixel_scene_y;
  GET_NUMERIC_ARG(ms, pixel_scene_x);
  GET_NUMERIC_ARG(ms, pixel_scene_y);

  cout << "sceneSetVanishingPointFromPixel x, y: " << pixel_scene_x << " " << pixel_scene_y << endl;

  // XXX optionally add a translation of the height reticles here to avoid going into "dead" configurations
  /*
  double delta_x = pixel_scene_x - camera->vanishingPointReticle.px;
  double delta_y = pixel_scene_x - camera->vanishingPointReticle.py;

  camera->heightReticles[0].px += delta_x;
  camera->heightReticles[1].px += delta_x;
  camera->heightReticles[2].px += delta_x;
  camera->heightReticles[3].px += delta_x;

  camera->heightReticles[0].py += delta_y;
  camera->heightReticles[1].py += delta_y;
  camera->heightReticles[2].py += delta_y;
  camera->heightReticles[3].py += delta_y;
  */
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  camera->vanishingPointReticle.px = pixel_scene_x;
  camera->vanishingPointReticle.py = pixel_scene_y;
}
END_WORD
REGISTER_WORD(SceneSetVanishingPointFromPixel)

WORD(SceneSetHeightReticleFromPixel)
virtual void execute(MachineState * ms) {
  int pixel_scene_x, pixel_scene_y;
  GET_NUMERIC_ARG(ms, pixel_scene_x);
  GET_NUMERIC_ARG(ms, pixel_scene_y);

  int this_height_idx = 0;
  GET_NUMERIC_ARG(ms, this_height_idx);

  cout << "sceneSetHeightReticleFromPixel x, y: " << pixel_scene_x << " " << pixel_scene_y << endl;
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  camera->heightReticles[this_height_idx].px = pixel_scene_x;
  camera->heightReticles[this_height_idx].py = pixel_scene_y;

  // XXX should really consider regressing a straight line for the reticles since if
  // they are bad the whole thing will be ill posed
}
END_WORD
REGISTER_WORD(SceneSetHeightReticleFromPixel)

WORD(SceneSetVanishingPointFromVariance)
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("scenePushPixelOfMinVariance sceneSetVanishingPointFromPixel");
}
END_WORD
REGISTER_WORD(SceneSetVanishingPointFromVariance)

WORD(SceneSetHeightReticleFromVariance)
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("scenePushPixelOfMinVariance sceneSetHeightReticleFromPixel");
}
END_WORD
REGISTER_WORD(SceneSetHeightReticleFromVariance)

WORD(ScenePushAverageCrCbSigmaSquared)
virtual void execute(MachineState * ms) {

  double total = 0;
  double samples = 0;
  double mean = 0;

  for (int y = 0; y < ms->config.scene->height; y++) {
    for (int x = 0; x < ms->config.scene->width; x++) {
      if (ms->config.scene->observed_map->refAtCell(x, y)->red.samples > 40) {
	double thisEnergy = 
	  ms->config.scene->observed_map->refAtCell(x,y)->red.sigmasquared +
	  ms->config.scene->observed_map->refAtCell(x,y)->green.sigmasquared; 
	  // + ms->config.scene->observed_map->refAtCell(x,y)->blue.sigmasquared;
	samples = samples + 1;
	total = total + thisEnergy;
      }
    }
  }

  if (samples > 0) {
    mean = total / samples;
  } else {
    // XXX if something went wrong, we don't want to choose this height. 
    // right now, only functions that want this to be low are used but they should check in the future.
    mean = DBL_MAX;
  }

  cout << "scenePushAverageCrCbSigmaSquared total samples mean: " << total << " " << samples << " " << mean << endl;

  ms->pushWord(make_shared<DoubleWord>(mean));
}
END_WORD
REGISTER_WORD(ScenePushAverageCrCbSigmaSquared)

WORD(ScenePushAnchorPose)
virtual void execute(MachineState * ms) {
  ms->pushData(make_shared<EePoseWord>(ms->config.scene->anchor_pose));
}
END_WORD
REGISTER_WORD(ScenePushAnchorPose)

/*

WORD(ScenePushDepthStackMinVarIdx)
virtual void execute(MachineState * ms) {
  // find the index of the depth stack with the least total variance
  double minEnergy = DBL_MAX;
  int minEnergyI = -1;

  for (int i = 0; i < ms->config.depth_maps.size(); i++) {
    //ms->config.depth_maps[i]->saveToFile(thisFilePath);
  }
  //ms->pushWord(make_shared<DoubleWord>());
}
END_WORD
REGISTER_WORD(ScenePushDepthStackMinVarIdx)


WORD(SceneFocusedClassDepthStackClear)
virtual void execute(MachineState * ms) {
}
END_WORD
REGISTER_WORD(SceneFocusedClassDepthStackClear)

WORD(SceneFocusedClassDepthStackPushObserved)
virtual void execute(MachineState * ms) {
}
END_WORD
REGISTER_WORD(SceneFocusedClassDepthStackPushObserved)

WORD(SceneFocusedClassDepthStackLoadAndPushRaw)
virtual void execute(MachineState * ms) {
}
END_WORD
REGISTER_WORD(SceneFocusedClassDepthStackLoadAndPushRaw)





WORD(SceneUpdateObservedFromStreamBufferAtZWithRecastThroughDepthStack)
virtual void execute(MachineState * ms) {
// updates entire stack from bottom up
}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZWithRecastThroughDepthStack)

WORD(SceneTrimDepthWithDiscrepancy)
virtual void execute(MachineState * ms) {
}
END_WORD
REGISTER_WORD(SceneTrimDepthWithDiscrepancy)

WORD(SceneUpdateObservedFromStreamBufferRecast)
virtual void execute(MachineState * ms) {
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
virtual void execute(MachineState * ms) {
// XXX
}
END_WORD
REGISTER_WORD(GaussianMapCalibrateVanishingPoint)

WORD(GaussianMapCalibrateMagnifications)
virtual void execute(MachineState * ms) {
// XXX
}
END_WORD
REGISTER_WORD(GaussianMapCalibrateMagnifications)

WORD(GaussianMapCompilePointCloud)
virtual void execute(MachineState * ms) {
// XXX
}
END_WORD
REGISTER_WORD(GaussianMapCompilePointCloud)



WORD(Scene)
virtual void execute(MachineState * ms) {
}
END_WORD
REGISTER_WORD(Scene)

WORD(GaussianMap)
virtual void execute(MachineState * ms) {
}
END_WORD
REGISTER_WORD(GaussianMap)

WORD()
virtual void execute(MachineState * ms) {
}
END_WORD
REGISTER_WORD()

WORD(TransitionTableInit)
virtual void execute(MachineState * ms) {
// zero it out
}
END_WORD
REGISTER_WORD(TransitionTableInit)

WORD(TransitionTableCount)
virtual void execute(MachineState * ms) {
}
END_WORD
REGISTER_WORD(TransitionTableCount)

WORD(PlanWithTransitionTable)
virtual void execute(MachineState * ms) {

// takes a desired state and outputs the action that best takes the prescene to that state

}
END_WORD
REGISTER_WORD(PlanWithTransitionTable)

*/
WORD(RayBufferInit)
virtual void execute(MachineState * ms) {
  ms->config.rayBuffer.resize(0);
}
END_WORD
REGISTER_WORD(RayBufferInit)

WORD(RayBufferSize)
virtual void execute(MachineState * ms) {
  ms->pushWord(make_shared<DoubleWord>(ms->config.rayBuffer.size()));
}
END_WORD
REGISTER_WORD(RayBufferSize)

WORD(RayBufferPopulateFromImageBuffer)
virtual void execute(MachineState * ms) {

  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  int thisIdx = camera->sibCurIdx;
  //cout << "sceneUpdateObservedFromStreamBuffer: " << thisIdx << endl;

  Mat bufferImage;
  eePose thisPose, tBaseP;


  int success = 1;
  if ( (thisIdx > -1) && (thisIdx < camera->streamImageBuffer.size()) ) {
    streamImage &tsi = camera->streamImageBuffer[thisIdx];
    loadStreamImage(ms, &tsi);
    bufferImage = tsi.image.clone();

    if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
      success = ms->getStreamPoseAtTime(tsi.time, &thisPose, &tBaseP);
    } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
      success = 1;
      thisPose = ms->config.currentEEPose;
      z_to_use = ms->config.currentEEPose.pz + ms->config.currentTableZ;
    } else {
      assert(0);
    }
  } else {
    CONSOLE_ERROR(ms, "No images in the buffer, returning." << endl);
    return;
  }

  if (success != 1) {
    CONSOLE_ERROR(ms, "  Not doing update because of stream buffer errors.");
    return;
  }

  eePose transformed = thisPose.getPoseRelativeTo(ms->config.scene->anchor_pose);
  if (fabs(transformed.qz) > 0.01) {
    CONSOLE_ERROR(ms, "  Not doing update because arm not vertical.");
    return;
  }

  Mat wristViewYCbCr = bufferImage.clone();

  cvtColor(bufferImage, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
  
  Size sz = bufferImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 
  
  pixelToGlobalCache data;
  pixelToGlobalCache data2;
  double z = z_to_use;
  double z2 = z_to_use-0.01;
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  computePixelToPlaneCache(ms, z, thisPose, ms->config.scene->anchor_pose, &data);  
  computePixelToPlaneCache(ms, z2, thisPose, ms->config.scene->anchor_pose, &data2);  
  int numThreads = 8;
  // there is a faster way to stride it but i am risk averse atm
  //#pragma omp parallel for
  for (int i = 0; i < numThreads; i++) {
    double frac = double(boty - topy) / double(numThreads);
    double bfrac = i*frac;
    double tfrac = (i+1)*frac;

    for (int py = topy; py < boty; py++) {
      // XXX actually this is not thread safe... 
      //if (py % numThreads == i) 
      double opy = py-topy;
      // this is superior
      if ( (bfrac <= opy) && (opy < tfrac) ) 
      {
	for (int px = topx; px < botx; px++) {
	  if (isInGripperMask(ms, px, py)) {
	    continue;
	  }

	  if ( (abhr > 0) && (abhc > 0) ) {
	    if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
		 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
	      continue;
	    } 
	  } 
	  Vec3b pixel = wristViewYCbCr.at<Vec3b>(py, px);

	  double x, y;
	  double x2, y2;
	  pixelToGlobalFromCache(ms, px, py, &x, &y, &data);
	  pixelToGlobalFromCache(ms, px, py, &x2, &y2, &data2);

	  int newIdx = ms->config.rayBuffer.size();
	  ms->config.rayBuffer.resize(newIdx+1);

	  OrientedRay * newRay =  &(ms->config.rayBuffer[newIdx]);
	  newRay->pa = eePose::identity();
	  newRay->pa.px = x;
	  newRay->pa.py = y;
	  newRay->pa.pz = z;
	  newRay->pb = eePose::identity();
	  newRay->pb.px = x2;
	  newRay->pb.py = y2;
	  newRay->pb.pz = z2;
	  newRay->r = pixel[0];
	  newRay->g = pixel[1];
	  newRay->b = pixel[2];
	  newRay->a = 0;
	  newRay->t = RAY_RGB;
	}
      }
    }
  }
  //ms->config.scene->observed_map->recalculateMusAndSigmas(ms);
}
END_WORD
REGISTER_WORD(RayBufferPopulateFromImageBuffer)

WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcAllOOP)
virtual void execute(MachineState * ms) {
  // XXX
  // here z to use is the distance from the anchor pose
  //  for best results, anchor pose should be z of camera
  double z_to_use = 0.0;
  GET_NUMERIC_ARG(ms, z_to_use);

  int stride = 1;
  GET_INT_ARG(ms, stride);


  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  //Size sz = ms->config.wristViewImage.size();
  Size sz = ms->config.cameras[ms->config.focused_camera]->cam_img.size();

  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 

  topx = std::max(topx, 0);
  botx = std::min(botx, imW-1);

  topy = std::max(topy, 0);
  boty = std::min(boty, imH-1);

  // XXX need to bounds check on all of these variants
  // consider using a function pointer inside of a preconstructed loop or macro it.
  

  Mat gripperMask = camera->gripperMask;
  if (isSketchyMat(gripperMask)) {
    CONSOLE_ERROR(ms, "Gripper mask is messed up.");
    return;
  }

  int numThreads = 8;


  vector<shared_ptr<GaussianMap> > maps;
  maps.resize(numThreads);

  #pragma omp parallel for
  for (int thread = 0; thread < numThreads; thread++) {
    
    maps[thread] = make_shared<GaussianMap>(ms, 
                                            ms->config.scene->observed_map->width, 
                                            ms->config.scene->observed_map->height, 
                                            ms->config.scene->observed_map->cell_width,
                                            ms->config.scene->observed_map->anchor_pose); 
    maps[thread]->zero();

    
    int thisStart = thread * (camera->streamImageBuffer.size()  / numThreads);
    int thisEnd = (thread + 1) * (camera->streamImageBuffer.size()  / numThreads); 
    stringstream buf;    
    buf << "thread: " << thread << " start: " << thisStart << " end: " << thisEnd << endl;
    cout << buf.str();
    for (int i = thisStart; i < thisEnd; i+=stride) {
      streamImage * tsi = camera->setIsbIdxNoLoadNoKick(i);
      
      
      
      if (tsi == NULL) {
        CONSOLE_ERROR(ms, "Stream image null.");
      }
      eePose tArmP, tBaseP;
      
      int success = 0;
      double z = z_to_use;
      if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
        success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
      } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
        success = 1;
        tArmP = ms->config.currentEEPose;
        z = ms->config.currentEEPose.pz + ms->config.currentTableZ;
      } else {
        assert(0);
      }
      
      
      if (success != 1) {
        CONSOLE_ERROR(ms, "Couldn't get stream pose.  Return code: " << success << " time: " << tsi->time);
        continue;
      }
      
      eePose transformed = tArmP.getPoseRelativeTo(ms->config.scene->anchor_pose);
      if (fabs(transformed.qz) > 0.01) {
        //CONSOLE_ERROR(ms, "Not doing update because arm not vertical.");
        //continue;
        CONSOLE_ERROR(ms, "Would normally not be doing update because arm not vertical, but we are.");
      }
      pixelToGlobalCache data;      
      computePixelToGlobalFullOOPCache(ms, z, tArmP, ms->config.scene->anchor_pose, &data);  

      
      // there is a faster way to stride it but i am risk averse atm
      
      Mat wristViewYCbCr = tsi->image.clone();
      
      cvtColor(tsi->image, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
      int numPixels = 0;
      int numNulls = 0;
      
      uchar *input = (uchar*) (wristViewYCbCr.data);
      
      for (int py = topy; py <= boty; py++) {
        uchar* gripperMaskPixel = camera->gripperMask.ptr<uchar>(py); // point to first pixel in row
        for (int px = topx; px <= botx; px++) {
          if (gripperMaskPixel[px] == 0) {
            continue;
          }
          
          if ( (abhr > 0) && (abhc > 0) ) {
            if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
                 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
              continue;
            } 
          } 
          
          double x, y;
	  pixelToGlobalFullFromCacheZOOP(ms, px, py, &x, &y, &data);
          
          if (1) {
            // single sample update
            int i, j;
            maps[thread]->metersToCell(x, y, &i, &j);
            GaussianMapCell * cell = maps[thread]->refAtCell(i, j);
            //ms->config.scene->observed_map->metersToCell(x, y, &i, &j);
            //GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(i, j);
            
            
            if (cell != NULL) {
              int base_idx = py * wristViewYCbCr.cols*3 + px * 3;
              cell->newObservation(input[base_idx + 2], input[base_idx + 1], input[base_idx + 0], z);
              numPixels++;
            }
          } else {
            numNulls++;
          }
        }
      }
    }
  }
  
  #pragma omp for
  for (int y = 0; y < ms->config.scene->observed_map->height; y++) {
    for (int x = 0; x < ms->config.scene->observed_map->width; x++) {
      for (int thread = 0; thread < numThreads; thread++) {
        if (maps[thread]->refAtCell(x, y)->red.samples > 0) {
          ms->config.scene->observed_map->refAtCell(x, y)->addC(maps[thread]->refAtCell(x, y));
        }
      }
    }
  }

}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferAtZNoRecalcAllOOP)




WORD(SceneUpdateObservedFromStreamBufferDepthMap)

virtual string description() {
  return "Updates the observed map from the stream buffer as a depth map.  Used for the Kinect 2.";
}

virtual void execute(MachineState * ms) {
  int stride = 1;
  GET_INT_ARG(ms, stride);



  Size sz = ms->config.wristViewImage.size();
  int imW = sz.width;
  int imH = sz.height;
  
  int aahr = (ms->config.angular_aperture_rows-1)/2;
  int aahc = (ms->config.angular_aperture_cols-1)/2;

  int abhr = (ms->config.angular_baffle_rows-1)/2;
  int abhc = (ms->config.angular_baffle_cols-1)/2;

  int imHoT = imH/2;
  int imWoT = imW/2;

  int topx = imWoT - aahc;
  int botx = imWoT + aahc; 
  int topy = imHoT - aahr; 
  int boty = imHoT + aahr; 
  
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  //computePixelToGlobalCache(ms, z, thisPose, &data);
  Mat gripperMask = camera->gripperMask;
  if (isSketchyMat(gripperMask)) {
    CONSOLE_ERROR(ms, "Gripper mask is messed up.");
    return;
  }

  int numThreads = 8;


  vector<shared_ptr<GaussianMap> > maps;
  maps.resize(numThreads);

  #pragma omp parallel for
  for (int thread = 0; thread < numThreads; thread++) {
    
    maps[thread] = make_shared<GaussianMap>(ms, 
                                            ms->config.scene->observed_map->width, 
                                            ms->config.scene->observed_map->height, 
                                            ms->config.scene->observed_map->cell_width,
                                            ms->config.scene->observed_map->anchor_pose); 
    maps[thread]->zero();

    
    int thisStart = thread * (camera->streamImageBuffer.size()  / numThreads);
    int thisEnd = (thread + 1) * (camera->streamImageBuffer.size()  / numThreads); 
    stringstream buf;    
    buf << "thread: " << thread << " start: " << thisStart << " end: " << thisEnd << endl;
    cout << buf.str();
    for (int i = thisStart; i < thisEnd; i+=stride) {
      streamImage * tsi = camera->setIsbIdxNoLoadNoKick(i);
      
      
      
      if (tsi == NULL) {
        CONSOLE_ERROR(ms, "Stream image null.");
      }
      loadStreamImage(ms, tsi);

      eePose tArmP, tBaseP;
      
      int success = 0;
      double z = -0.2;
      if (ms->config.currentSceneFixationMode == FIXATE_STREAM) {
        success = ms->getStreamPoseAtTime(tsi->time, &tArmP, &tBaseP);
      } else if (ms->config.currentSceneFixationMode == FIXATE_CURRENT) {
        success = 1;
        tArmP = ms->config.currentEEPose;
        z = ms->config.currentEEPose.pz + ms->config.currentTableZ;
      } else {
        assert(0);
      }
      
      
      if (success != 1) {
        CONSOLE_ERROR(ms, "Couldn't get stream pose.  Return code: " << success << " time: " << tsi->time);
        continue;
      }
      
      eePose transformed = tArmP.getPoseRelativeTo(ms->config.scene->anchor_pose);
      if (fabs(transformed.qz) > 0.01) {
        CONSOLE_ERROR(ms, "Not doing update because arm not vertical.");
        continue;
      }
      pixelToGlobalCache data;      
      computePixelToPlaneCache(ms, z, tArmP, ms->config.scene->anchor_pose, &data);  
      
      // there is a faster way to stride it but i am risk averse atm
      
      Mat wristViewYCbCr = tsi->image.clone();

      imshow("image", wristViewYCbCr);
      cvtColor(tsi->image, wristViewYCbCr, cv::COLOR_BGR2YCrCb);
      int numPixels = 0;
      int numNulls = 0;
      
      uchar *input = (uchar*) (wristViewYCbCr.data);
      
      for (int py = topy; py <= boty; py++) {
        uchar* gripperMaskPixel = camera->gripperMask.ptr<uchar>(py); // point to first pixel in row
        for (int px = topx; px <= botx; px++) {
          if (gripperMaskPixel[px] == 0) {
            continue;
          }
          
          if ( (abhr > 0) && (abhc > 0) ) {
            if ( (py > imHoT - abhr) && (py < imHoT + abhr) &&
                 (px > imWoT - abhc) && (px < imWoT + abhc) ) {
              continue;
            } 
          } 
          
          double x, y;
          pixelToGlobalFromCache(ms, px, py, &x, &y, &data);
          
          if (1) {
            // single sample update
            int i, j;
            maps[thread]->metersToCell(x, y, &i, &j);
            GaussianMapCell * cell = maps[thread]->refAtCell(i, j);
            //ms->config.scene->observed_map->metersToCell(x, y, &i, &j);
            //GaussianMapCell * cell = ms->config.scene->observed_map->refAtCell(i, j);
            
            
            if (cell != NULL) {
              int base_idx = py * wristViewYCbCr.cols*3 + px * 3;
              cell->newObservation(input[base_idx + 2], input[base_idx + 1], input[base_idx + 0], z);
              numPixels++;
            }
          } else {
            numNulls++;
          }
        }
      }
    }
  }
  
  #pragma omp for
  for (int y = 0; y < ms->config.scene->observed_map->height; y++) {
    for (int x = 0; x < ms->config.scene->observed_map->width; x++) {
      for (int thread = 0; thread < numThreads; thread++) {
        if (maps[thread]->refAtCell(x, y)->red.samples > 0) {
          ms->config.scene->observed_map->refAtCell(x, y)->addC(maps[thread]->refAtCell(x, y));
        }
      }
    }
  }

}
END_WORD
REGISTER_WORD(SceneUpdateObservedFromStreamBufferDepthMap)
}




