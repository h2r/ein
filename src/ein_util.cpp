#include "ein_util.h"
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/c_local_time_adjustor.hpp>

#include <boost/filesystem.hpp>
using namespace boost::filesystem;


void writeBinaryToYaml(unsigned char * data, int length, FileStorage & fsvO) {
  int max_string_length = 3000;
  
  vector<string> strings;
  int current_idx = 0;

  while (current_idx < length) {
    int end_idx = min(current_idx + max_string_length, length); 
    int current_length = end_idx - current_idx;
    string result = base64_encode(&data[current_idx], current_length);
    //cout << "result: " << result.size() << endl;
    strings.push_back(result);
    
    current_idx = end_idx;
  }

  fsvO << "[:";
  for (int i = 0; i < strings.size(); i++) {
    fsvO << strings[i];
  }
  fsvO << "]";
}

string readBinaryFromYaml(FileNode & fn) {
  stringstream result;
  for (FileNodeIterator it = fn.begin(); it != fn.end(); it++) {
    FileNode node = *it;
    result << (string) node;
  }
  return result.str();
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


void pushGridSign(shared_ptr<MachineState> ms, double speed) {

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



void initializeMachine(shared_ptr<MachineState> ms) {
  ms->pushWord("sceneInit"); 

  if (ms->config.currentRobotMode != PHYSICAL) {
    return;
  }

  ms->pushWord("zeroGOff"); 
  ms->pushWord("waitUntilEndpointCallbackReceived"); 

  ms->pushWord("guiCustom1"); 
  ms->pushWord("printState");
  ms->pushCopies("zUp", 15);
  int devInit = 1;
  if (devInit) {
    ms->pushWord("incrementTargetClass"); 
    ms->pushWord("synchronicServoTakeClosest");
  }
  ms->pushWord("silenceSonar");
  ms->pushWord("exportWords");
  ms->pushWord("openGripper");
  ms->pushWord("calibrateGripper");
  ms->pushWord("shiftIntoGraspGear1"); 

  {
    ms->pushWord("fillClearanceMap"); 
    ms->pushWord("moveCropToProperValue"); 
    ms->pushWord("loadCalibration"); 
    ms->pushWord("loadIkMap"); 
    ms->pushWord("loadGripperMask"); 
    ms->pushWord("initializeConfig");
  }

  ms->execute_stack = 1;
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
