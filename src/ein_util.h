#ifndef _EIN_UTIL_H_
#define _EIN_UTIL_H_

#include <string>

#include <cv.h>
#include <ml.h>
#include <opencv2/gpu/gpu.hpp>

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include "word.h"

#include "base64.h"

class MachineState;

namespace ros {
  class Time;
}

using namespace std;
using namespace cv;

#define MY_FONT FONT_HERSHEY_SIMPLEX


#define RANGE_UPPER_INVALID 0.3
#define RANGE_LOWER_INVALID 0.08


typedef enum {
  UNKNOWN = -1,
  FAILURE = 0,
  SUCCESS = 1
} operationStatusType;


std::string operationStatusToString(operationStatusType mode) ;


typedef enum {
  UNIFORM_PRIOR,
  ANALYTIC_PRIOR
} priorType;

typedef enum {
  MRT,
  SPOON,
  KNIFE,
  OFT_INVALID
} orientedFilterType;






#define ORIENTATIONS 180//12 
#define O_FILTER_WIDTH 25//25
#define O_FILTER_SPOON_HEAD_WIDTH 6 
#define O_FILTER_SPOON_SHAFT_WIDTH 2


void pushGridSign(MachineState * ms, double speed);
bool isSketchyMat(Mat sketchy);

void initializeMachine(MachineState * ms);
string formatTime(ros::Time time);
bool copyDir(string src, string dest);

string readBinaryFromYaml(FileNode & fn);
void writeBinaryToYaml(unsigned char * data, int length, FileStorage & fsvO);

Mat readMatFromYaml(FileNode & fsvO);
void writeMatToYaml(Mat m, FileStorage & fsvO);

string sceneModelFile(MachineState * ms, string label);
string streamDirectory(MachineState * ms, int classIdx);

string xmlEncode(const string data);

vector<string> glob(const string& pat);

#endif /* _EIN_UTIL_H_ */
