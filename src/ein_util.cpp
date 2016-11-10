#include "ein_util.h"
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/c_local_time_adjustor.hpp>

#include <boost/filesystem.hpp>

#include "compress.h"
using namespace boost::filesystem;



Mat readMatFromYaml(FileNode & fs) {
  int rows = fs["rows"];
  int cols = fs["cols"];
  int type = fs["type"];
  
  Mat m(rows, cols, type);
  FileNode n = fs["data"];
  string stringdata = readBinaryFromYaml(n);
  uchar * data = (uchar *) stringdata.data();

  if (stringdata != "") {
    memcpy(m.data, data, m.rows * m.cols * m.elemSize());
  }
  return m;
}

void writeMatToYaml(Mat m, FileStorage & fs) {
  fs <<  "{";
  fs << "rows" << m.rows;
  fs << "cols" << m.cols;
  fs << "type" << m.type();
  fs << "data";
  writeBinaryToYaml(m.data, m.rows * m.cols * m.elemSize(), fs);
  fs << "}";
}



void writeBinaryToYaml(unsigned char * data, int length, FileStorage & fsvO) {


  string compressed_data = compress_string(data, length);
  string encoded_data = base64_encode((unsigned char *) compressed_data.data(), compressed_data.size());

  vector<string> strings;
  int current_idx = 0;
  int string_length = encoded_data.size();
  int max_string_length = 4095;
  fsvO << "[:";
  while (current_idx < string_length) {
    int end_idx = min(current_idx + max_string_length, string_length); 
    int current_length = end_idx - current_idx;
    fsvO << encoded_data.substr(current_idx, current_length);
    current_idx = end_idx;
  }

  fsvO << "]";
}



string readBinaryFromYaml(FileNode & fn) {
  stringstream result;
  for (FileNodeIterator it = fn.begin(); it != fn.end(); it++) {
    FileNode node = *it;
    result << (string) node;
  }
  string decoded_data = base64_decode(result.str());
  //cout << "decode: " << decoded_data.size() << endl;
  try { 
    return decompress_string(decoded_data);
  } catch( ... ) {
    ROS_ERROR("Exception uncompressing a binary yaml file.");
    std::exception_ptr p = std::current_exception();
    std::clog <<(p ? p.__cxa_exception_type()->name() : "null") << std::endl;
    return "";
  }

}

std::string operationStatusToString(operationStatusType mode) 
{
    string result;
  if (mode == UNKNOWN) {
    result = "Unknown";
  } else if (mode == FAILURE) {
    result = "Failure";
  } else if (mode == SUCCESS) {
    result = "Success";
  } else {
    cout << "Invalid operation status: " << mode << endl;
    assert(0);
  }
  return result;
}


void pushGridSign(MachineState * ms, double speed) {

  if (speed == NOW_THATS_COARSE) {
    ms->pushWord("setGridSizeNowThatsCoarse"); 
  } else if (speed == GRID_EVEN_COARSER) {
    ms->pushWord("setGridSizeEvenCoarser"); 
  } else  if (speed == GRID_COARSER) {
    ms->pushWord("setGridSizeCoarser"); 
  } else if (speed == GRID_COARSE) {
    ms->pushWord("setGridSizeCoarse"); 
  } else if (speed == GRID_MEDIUM) {
    ms->pushWord("setGridSizeMedium"); 
  } else if (speed == GRID_FINE) {
    ms->pushWord("setGridSizeFine"); 
  } else if (speed == GRID_VERY_FINE) {
    ms->pushWord("setGridSizeVeryFine"); 
  } else {
    ROS_ERROR_STREAM("Unknown speed: " << speed);
    assert(0);
  }

}



bool isSketchyMat(Mat sketchy) {
  return ( (sketchy.rows <= 1) || (sketchy.rows <= 1) );
}



gsl_matrix * boxMemoryToPolygon(BoxMemory b) {
  double min_x = b.top.px;
  double min_y = b.top.py;
  double max_x = b.bot.px;
  double max_y = b.bot.py;
  double width = max_x - min_x;
  double height = max_y - min_y;

  gsl_matrix *  polygon = gsl_matrix_alloc(2, 4);
  gsl_matrix_set(polygon, 0, 0, min_x);
  gsl_matrix_set(polygon, 1, 0, min_y);

  gsl_matrix_set(polygon, 0, 1, min_x + width);
  gsl_matrix_set(polygon, 1, 1, min_y);

  gsl_matrix_set(polygon, 0, 2, min_x + width);
  gsl_matrix_set(polygon, 1, 2, min_y + height);

  gsl_matrix_set(polygon, 0, 3, min_x);
  gsl_matrix_set(polygon, 1, 3, min_y + height);
  return polygon;
}


eePose rosPoseToEEPose(geometry_msgs::Pose pose) {
  eePose result;
  result.px = pose.position.x;
  result.py = pose.position.y;
  result.pz = pose.position.z;
  result.qx = pose.orientation.x;
  result.qy = pose.orientation.y;
  result.qz = pose.orientation.z;
  result.qw = pose.orientation.w;
  return result;
}



void initializeMachine(MachineState * ms) {
  ms->execute_stack = 1;

  ms->evaluateProgram("\"init\" import");
  ms->pushWord("sceneInit"); 
  ms->evaluateProgram("cameraFitHyperbolic 2 cameraSetCalibrationMode");

  stringstream s;
  s << "*** Starting Ein " << ms->config.ein_software_version << " " << ms->config.left_or_right_arm << " at " << formatTime(ros::Time::now());
  cout << "start message: " << s.str() << endl;
  ms->pushWord("print");
  ms->pushData(make_shared<StringWord>(s.str()));


  if (ms->config.currentRobotMode == PHYSICAL) {

    ms->pushWord("zeroGOff"); 
    ms->pushWord("waitUntilEndpointCallbackReceived"); 
    
    ms->pushCopies("zUp", 15);
    ms->pushWord("incrementTargetClass"); 
    ms->pushWord("synchronicServoTakeClosest");

    ms->pushWord("silenceSonar");
    ms->pushWord("exportWords");
    ms->pushWord("openGripper");
    ms->pushWord("calibrateGripper");
    ms->pushWord("shiftIntoGraspGear1"); 
    
    ms->pushWord("fillClearanceMap"); 
  }


  ms->pushWord("loadCalibration"); 
  ms->pushWord("loadIkMap"); 
  ms->pushWord("loadGripperMask"); 
  ms->pushWord("loadConfig"); 
  ms->pushWord("initializeConfig");
  ms->pushWord("guiCustom1"); 
  
}



string formatTime(ros::Time time) {
  stringstream buf;

  boost::posix_time::ptime old = time.toBoost();

  typedef boost::date_time::c_local_adjustor<boost::posix_time::ptime> local_adj;
  boost::posix_time::ptime p =  local_adj::utc_to_local(old);

  boost::posix_time::time_facet * facet = new boost::posix_time::time_facet();
  facet->format("%Y-%m-%d_%H:%M:%S %Z");

  buf.imbue(std::locale(std::cout.getloc(), facet));
  buf << p;
  return buf.str();
}



bool copyDir(string src, string dest) {


  boost::filesystem::path source(src); 
  boost::filesystem::path destination(dest);
  
  namespace fs = boost::filesystem;
  try
    {
      // Check whether the function call is valid
      if(
	 !fs::exists(source) ||
	 !fs::is_directory(source)
	 )
	{
	  std::cerr << "Source directory " << source.string()
		    << " does not exist or is not a directory." << '\n'
	    ;
	  return false;
	}
      if(fs::exists(destination))
	{
	  std::cerr << "Destination directory " << destination.string()
		    << " already exists." << '\n'
	    ;
	  return false;
	}
      // Create the destination directory
      if(!fs::create_directory(destination))
	{
	  std::cerr << "Unable to create destination directory"
		    << destination.string() << '\n'
	    ;
	  return false;
	}
    }
  catch(fs::filesystem_error const & e)
    {
      std::cerr << e.what() << '\n';
      return false;
    }
  // Iterate through the source directory
  for(
      fs::directory_iterator file(source);
      file != fs::directory_iterator(); ++file
	)
    {
      try
	{
	  fs::path current(file->path());
	  if(fs::is_directory(current))
	    {
	      // Found directory: Recursion
	      if(
		 !copyDir(
			  current.string(),
			  (destination / current.filename()).string()
			  )
		 )
		{
                    return false;
		}
	    }
	  else
	    {
	      // Found file: Copy
	      // can't use this because Boost doesn't work with c++11
	      //fs::copy_file(
	      //	    current,
	      //	    destination / current.filename()
	      //	    );

	      path newdest = destination / current.filename();
	      std::ifstream  src(current.string(), std::ios::binary);
	      std::ofstream  dst(newdest.string(),   std::ios::binary);
	      dst << src.rdbuf();

	    }
	  }
      catch(fs::filesystem_error const & e)
	{
	  std:: cerr << e.what() << '\n';
	}
    }
  return true;

}

string sceneModelFile(MachineState * ms, string label) {
  return ms->config.data_directory + "/objects/" + label + "/ein/sceneModel/model.yml";
}

string streamDirectory(MachineState * ms, int classIdx) {
  string thisLabelName = ms->config.classLabels[classIdx];
  return ms->config.data_directory + "/objects/" + thisLabelName + "/raw";
}



string xmlEncode(const string data) {
  std::string buffer;
  buffer.reserve(data.size());
  for(size_t pos = 0; pos != data.size(); ++pos) {
    switch(data[pos]) {
    case '&':  buffer.append("&amp;");       break;
    case '\"': buffer.append("&quot;");      break;
    case '\'': buffer.append("&apos;");      break;
    case '<':  buffer.append("&lt;");        break;
    case '>':  buffer.append("&gt;");        break;
    default:   buffer.append(&data[pos], 1); break;
    }
  }
  return buffer;
}
