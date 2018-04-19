//
//  // start Tips
//  // it's dangerous to go alone. take this.
//  //               \|/ 
//  // o++{==========>*
//  //               /|\ 
//  // end Tips
//   
//  // Ein: Ein Isn't Node.
//  // Ein: A dog in Cowboy Bebop.
//  // Ein: Short for Eindecker.
//  // Ein: It's one program. 
//
//   ><>           <><
//        ><>  
//              <><
// end Header

#include "ein.h"
#include "camera.h"

#include "qtgui/mainwindow.h"
#include "qtgui/einwindow.h"
#include "qtgui/gaussianmapwindow.h"
#include "qtgui/streamviewerwindow.h"
#include "qtgui/discrepancywindow.h"
#include <QApplication>
#include <QTimer>
#include <dirent.h>
#include <sys/stat.h>
#include <signal.h>

#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <sensor_msgs/image_encodings.h>

#include <cv.h>
#include <ml.h>
#include <opencv2/gpu/gpu.hpp>


//#define DEBUG_RING_BUFFER

#define stringer(token) #token
#define stringer_value(token) stringer(token)

#include <boost/filesystem.hpp>

#include <highgui.h>

#include <boost/algorithm/string.hpp>


MainWindow * einMainWindow;
vector< MachineState * > machineStates;
MachineState * left_arm;
MachineState * right_arm;


////////////////////////////////////////////////
// start pilot includes, usings, and defines
////////////////////////////////////////////////

//
// start pilot definitions 
////////////////////////////////////////////////


int getRingRangeAtTime(MachineState * ms, ros::Time t, double &value, int drawSlack) {
  if (ms->config.rgRingBufferStart == ms->config.rgRingBufferEnd) {
#ifdef DEBUG_RING_BUFFER
    cout << "Denied request in getRingRangeAtTime(): Buffer empty." << endl;
#endif
    return 0;
  } else {
    int earliestSlot = ms->config.rgRingBufferStart;
    ros::Duration deltaTdur = t - ms->config.rgRBTimes[earliestSlot];
    // if the request comes before our earliest record, deny
    if (deltaTdur.toSec() <= 0.0) {
#ifdef DEBUG_RING_BUFFER
      cout << "Denied out of order range value in getRingRangeAtTime(): Too small." << endl;
#endif
      return -1;
    } else if (ms->config.rgRingBufferStart < ms->config.rgRingBufferEnd) {
      for (int s = ms->config.rgRingBufferStart; s < ms->config.rgRingBufferEnd; s++) {
	ros::Duration deltaTdurPre = t - ms->config.rgRBTimes[s];
	ros::Duration deltaTdurPost = t - ms->config.rgRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  double r1 = ms->config.rgRingBuffer[s];
	  double r2 = ms->config.rgRingBuffer[s+1];
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;
	  value = w1*r1 + w2*r2;

	  int newStart = s;
	  if(drawSlack) {
	    ms->config.rgRingBufferStart = newStart;
	  }
	  return 1;
	}
      }
      // if we didn't find it we should return failure
#ifdef DEBUG_RING_BUFFER
      cout << "Denied out of order range value in getRingRangeAtTime(): Too large." << endl;
#endif
      return -2;
    } else {
      for (int s = ms->config.rgRingBufferStart; s < ms->config.rgRingBufferSize-1; s++) {
	ros::Duration deltaTdurPre = t - ms->config.rgRBTimes[s];
	ros::Duration deltaTdurPost = t - ms->config.rgRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  double r1 = ms->config.rgRingBuffer[s];
	  double r2 = ms->config.rgRingBuffer[s+1];
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;
	  value = w1*r1 + w2*r2;

	  int newStart = s;
	  if(drawSlack) {
	    ms->config.rgRingBufferStart = newStart;
	  }
	  return 1;
	}
      } {
	ros::Duration deltaTdurPre = t - ms->config.rgRBTimes[ms->config.rgRingBufferSize-1];
	ros::Duration deltaTdurPost = t - ms->config.rgRBTimes[0];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  double r1 = ms->config.rgRingBuffer[ms->config.rgRingBufferSize-1];
	  double r2 = ms->config.rgRingBuffer[0];
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;
	  value = w1*r1 + w2*r2;

	  int newStart = ms->config.rgRingBufferSize-1;
	  if(drawSlack) {
	    ms->config.rgRingBufferStart = newStart;
	  }
	  return 1;
	}
      } for (int s = 0; s < ms->config.rgRingBufferEnd; s++) {
	ros::Duration deltaTdurPre = t - ms->config.rgRBTimes[s];
	ros::Duration deltaTdurPost = t - ms->config.rgRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  double r1 = ms->config.rgRingBuffer[s];
	  double r2 = ms->config.rgRingBuffer[s+1];
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;
	  value = w1*r1 + w2*r2;

	  int newStart = s;
	  if(drawSlack) {
	    ms->config.rgRingBufferStart = newStart;
	  }
	  return 1;
	}
      }
      // if we didn't find it we should return failure
#ifdef DEBUG_RING_BUFFER
      cout << "Denied out of order range value in getRingRangeAtTime(): Too large." << endl;
#endif
      return -2;
    }
  }
}


int getMostRecentRingImageAndPose(MachineState * ms, Mat * image, eePose * pose, ros::Time * time, bool debug) {
  if (ms->config.epRingBufferEnd > ms->config.epRBTimes.size()) {
    cout << "Ring buffer not yet initialized. " << ms->config.epRingBufferEnd << " times: " << ms->config.epRBTimes.size() << endl;
    assert(0);
  }
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  ros::Time poseTime = ms->config.epRBTimes[ms->config.epRingBufferEnd - 1];
  ros::Time imageTime = camera->imRBTimes[camera->imRingBufferEnd - 1];

  * time = min(poseTime, imageTime);
  geometry_msgs::Pose thisPose;
  bool error = false;
  int result = getRingPoseAtTime(ms, *time, thisPose, 0, debug);
  if (result != 1) {
    CONSOLE_ERROR(ms, "Pose ring buffer error: " << result);
    error = true;
  }
  *pose = eePose::fromGeometryMsgPose(thisPose);
  result = camera->getRingImageAtTime(*time, *image, 0, debug);
  if (result != 1) {
    CONSOLE_ERROR(ms, "Image ring buffer error: " << result);
    error = true;
  }
  if (error) {
    return -1;
  } else {
    return 1;
  }
  
}


int getRingPoseAtTime(MachineState * ms, ros::Time t, geometry_msgs::Pose &value, int drawSlack, bool debug) {
  if (ms->config.epRingBufferStart == ms->config.epRingBufferEnd) {
    if (debug) {
      cout << "Denied request in getRingPoseAtTime(): Buffer empty." << endl;
    }
    return -1;
  } else {
    int earliestSlot = ms->config.epRingBufferStart;
    ros::Duration deltaTdur = t - ms->config.epRBTimes[earliestSlot];
    // if the request comes before our earliest record, deny
    if (deltaTdur.toSec() <= 0.0) {
      if (debug) {
	cout << "Denied out of order range value in getRingPoseAtTime(): Too small." << endl;
      }
      return -1;
    } else if (ms->config.epRingBufferStart < ms->config.epRingBufferEnd) {
      for (int s = ms->config.epRingBufferStart; s < ms->config.epRingBufferEnd; s++) {
	ros::Duration deltaTdurPre = t - ms->config.epRBTimes[s];
	ros::Duration deltaTdurPost = t - ms->config.epRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Quaternionf q1 = extractQuatFromPose(ms->config.epRingBuffer[s]);
	  Quaternionf q2 = extractQuatFromPose(ms->config.epRingBuffer[s+1]);
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;

	  // XXX check this
	  Quaternionf tTerp = q1.slerp(w2, q2);
	  //Quaternionf tTerp = q1.slerp(w1, q2);

	  value.orientation.w = tTerp.w();
	  value.orientation.x = tTerp.x();
	  value.orientation.y = tTerp.y();
	  value.orientation.z = tTerp.z();
	  value.position.x = ms->config.epRingBuffer[s].position.x*w1 + ms->config.epRingBuffer[s+1].position.x*w2;
	  value.position.y = ms->config.epRingBuffer[s].position.y*w1 + ms->config.epRingBuffer[s+1].position.y*w2;
	  value.position.z = ms->config.epRingBuffer[s].position.z*w1 + ms->config.epRingBuffer[s+1].position.z*w2;
#ifdef DEBUG_RING_BUFFER
          //cout << value << endl;
          //cout << "33333c " << ms->config.epRingBuffer[s] << " " << w1 << " " << w2 << " " << totalWeight << endl;
          //cout << "44444c " << ms->config.epRingBuffer[s+1] << endl;
          //cout << "555c " << ms->config.epRBTimes[s] << " " << w1 << " " << w2 << " " << totalWeight << endl;
#endif

	  int newStart = s;
	  if(drawSlack) {
	    ms->config.epRingBufferStart = newStart;
	  }
	  return 1;
	}
	if (debug) {
	  cout << "777c " << ms->config.epRBTimes[s] << endl;
	}
      }
      // if we didn't find it we should return failure
      if (debug) {
	cout << "Denied out of order range value in getRingPoseAtTime() Upper: Too large. " << t << endl;
	cout << "rbStart: " << ms->config.epRingBufferStart << " rbEnd: " << ms->config.epRingBufferEnd << endl;
      }
      return -2;
    } else {
      for (int s = ms->config.epRingBufferStart; s < ms->config.epRingBufferSize-1; s++) {
	ros::Duration deltaTdurPre = t - ms->config.epRBTimes[s];
	ros::Duration deltaTdurPost = t - ms->config.epRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Quaternionf q1 = extractQuatFromPose(ms->config.epRingBuffer[s]);
	  Quaternionf q2 = extractQuatFromPose(ms->config.epRingBuffer[s+1]);
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;

	  Quaternionf tTerp = q1.slerp(w2, q2);
	  value.orientation.w = tTerp.w();
	  value.orientation.x = tTerp.x();
	  value.orientation.y = tTerp.y();
	  value.orientation.z = tTerp.z();
	  value.position.x = ms->config.epRingBuffer[s].position.x*w1 + ms->config.epRingBuffer[s+1].position.x*w2;
	  value.position.y = ms->config.epRingBuffer[s].position.y*w1 + ms->config.epRingBuffer[s+1].position.y*w2;
	  value.position.z = ms->config.epRingBuffer[s].position.z*w1 + ms->config.epRingBuffer[s+1].position.z*w2;
	  if (debug) {
	    cout << value << endl;
	    cout << "33333b " << ms->config.epRingBuffer[s] << " " << w1 << " " << w2 << " " << totalWeight << endl;
	    cout << "44444b " << ms->config.epRingBuffer[s+1] << endl;
	  }

	  int newStart = s;
	  if(drawSlack) {
	    ms->config.epRingBufferStart = newStart;
	  }
	  return 1;
	}
      } {
	ros::Duration deltaTdurPre = t - ms->config.epRBTimes[ms->config.epRingBufferSize-1];
	ros::Duration deltaTdurPost = t - ms->config.epRBTimes[0];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Quaternionf q1 = extractQuatFromPose(ms->config.epRingBuffer[ms->config.epRingBufferSize-1]);
	  Quaternionf q2 = extractQuatFromPose(ms->config.epRingBuffer[0]);
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;

	  Quaternionf tTerp = q1.slerp(w2, q2);
	  value.orientation.w = tTerp.w();
	  value.orientation.x = tTerp.x();
	  value.orientation.y = tTerp.y();
	  value.orientation.z = tTerp.z();
	  value.position.x = ms->config.epRingBuffer[ms->config.epRingBufferSize-1].position.x*w1 + ms->config.epRingBuffer[0].position.x*w2;
	  value.position.y = ms->config.epRingBuffer[ms->config.epRingBufferSize-1].position.y*w1 + ms->config.epRingBuffer[0].position.y*w2;
	  value.position.z = ms->config.epRingBuffer[ms->config.epRingBufferSize-1].position.z*w1 + ms->config.epRingBuffer[0].position.z*w2;
	  if (debug) {
	    cout << value << endl;
	    cout << "33333a " << ms->config.epRingBuffer[ms->config.epRingBufferSize-1] << " " << w1 << " " << w2 << " " << totalWeight << endl;
	    cout << "44444a " << ms->config.epRingBuffer[0] << endl;
	  }

	  int newStart = ms->config.epRingBufferSize-1;
	  if(drawSlack) {
	    ms->config.epRingBufferStart = newStart;
	  }
	  return 1;
	}
      } for (int s = 0; s < ms->config.epRingBufferEnd; s++) {
	ros::Duration deltaTdurPre = t - ms->config.epRBTimes[s];
	ros::Duration deltaTdurPost = t - ms->config.epRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Quaternionf q1 = extractQuatFromPose(ms->config.epRingBuffer[s]);
	  Quaternionf q2 = extractQuatFromPose(ms->config.epRingBuffer[s+1]);
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;

	  Quaternionf tTerp = q1.slerp(w2, q2);
	  value.orientation.w = tTerp.w();
	  value.orientation.x = tTerp.x();
	  value.orientation.y = tTerp.y();
	  value.orientation.z = tTerp.z();
	  value.position.x = ms->config.epRingBuffer[s].position.x*w1 + ms->config.epRingBuffer[s+1].position.x*w2;
	  value.position.y = ms->config.epRingBuffer[s].position.y*w1 + ms->config.epRingBuffer[s+1].position.y*w2;
	  value.position.z = ms->config.epRingBuffer[s].position.z*w1 + ms->config.epRingBuffer[s+1].position.z*w2;
	  if (debug) {
	    cout << value << endl;
	    cout << "33333d " << ms->config.epRingBuffer[s] << " " << w1 << " " << w2 << " " << totalWeight << endl;
	    cout << "44444d " << ms->config.epRingBuffer[s+1] << endl;
	  }
          
	  int newStart = s;
	  if(drawSlack) {
	    ms->config.epRingBufferStart = newStart;
	  }
	  return 1;
	}
      }
      // if we didn't find it we should return failure
      if (debug) {
	cout << "Denied out of order range value in getRingPoseAtTime() Lower: Too large. " << t << endl;
      }

      return -2;
    }
  }
}

void setRingRangeAtTime(MachineState * ms, ros::Time t, double rgToSet) {
#ifdef DEBUG_RING_BUFFER
  //cout << "setRingRangeAtTime() start end size: " << ms->config.rgRingBufferStart << " " << ms->config.rgRingBufferEnd << " " << ms->config.rgRingBufferSize << endl;
#endif

  // if the ring buffer is empty, always re-initialize
  if (ms->config.rgRingBufferStart == ms->config.rgRingBufferEnd) {
    ms->config.rgRingBufferStart = 0;
    ms->config.rgRingBufferEnd = 1;
    ms->config.rgRingBuffer[0] = rgToSet;
    ms->config.rgRBTimes[0] = t;
  } else {
    ros::Duration deltaTdur = t - ms->config.rgRBTimes[ms->config.rgRingBufferStart];
    if (deltaTdur.toSec() <= 0.0) {
#ifdef DEBUG_RING_BUFFER 
      //cout << "Dropped out of order range value in setRingRangeAtTime(). " << ms->config.rgRBTimes[ms->config.rgRingBufferStart].toSec() << " " << t.toSec() << " " << deltaTdur.toSec() << " " << endl;
#endif
    } else {
      int slot = ms->config.rgRingBufferEnd;
      ms->config.rgRingBuffer[slot] = rgToSet;
      ms->config.rgRBTimes[slot] = t;

      if (ms->config.rgRingBufferEnd >= (ms->config.rgRingBufferSize-1)) {
	ms->config.rgRingBufferEnd = 0;
      } else {
	ms->config.rgRingBufferEnd++;
      }

      if (ms->config.rgRingBufferEnd == ms->config.rgRingBufferStart) {
	if (ms->config.rgRingBufferStart >= (ms->config.rgRingBufferSize-1)) {
	  ms->config.rgRingBufferStart = 0;
	} else {
	  ms->config.rgRingBufferStart++;
	}
      }
    }
  }
}
void setRingPoseAtTime(MachineState * ms, ros::Time t, geometry_msgs::Pose epToSet) {
  //#define DEBUG_RING_BUFFER
#ifdef DEBUG_RING_BUFFER
  cout << "setRingPoseAtTime() start end size time: " << ms->config.epRingBufferStart << " " << ms->config.epRingBufferEnd << " " << ms->config.epRingBufferSize << " " << t << endl;
#endif

  // if the ring buffer is empty, always re-initialize
  if (ms->config.epRingBufferStart == ms->config.epRingBufferEnd) {
    ms->config.epRingBufferStart = 0;
    ms->config.epRingBufferEnd = 1;
    ms->config.epRingBuffer[0] = epToSet;
#ifdef DEBUG_RING_BUFFER
    cout << epToSet << endl;
    cout << "11111 " << ms->config.epRingBuffer[0] << endl;
#endif
    ms->config.epRBTimes[0] = t;
  } else {
    ros::Duration deltaTdur = t - ms->config.epRBTimes[ms->config.epRingBufferStart];
    if (deltaTdur.toSec() <= 0.0) {
#ifdef DEBUG_RING_BUFFER 
      cout << "Dropped out of order range value in setRingPoseAtTime(). " << ms->config.epRBTimes[ms->config.epRingBufferStart].toSec() << " " << t.toSec() << " " << deltaTdur.toSec() << " " << endl;
#endif

    } else {
      int slot = ms->config.epRingBufferEnd;
      ms->config.epRingBuffer[slot] = epToSet;
#ifdef DEBUG_RING_BUFFER
      cout << epToSet << endl;
      cout << "22222" << ms->config.epRingBuffer[slot] << endl;
#endif
      ms->config.epRBTimes[slot] = t;

      if (ms->config.epRingBufferEnd >= (ms->config.epRingBufferSize-1)) {
	ms->config.epRingBufferEnd = 0;
      } else {
	ms->config.epRingBufferEnd++;
      }

      if (ms->config.epRingBufferEnd == ms->config.epRingBufferStart) {
	if (ms->config.epRingBufferStart >= (ms->config.epRingBufferSize-1)) {
	  ms->config.epRingBufferStart = 0;
	} else {
	  ms->config.epRingBufferStart++;
	}
      }
    }
  }
}

void rgRingBufferAdvance(MachineState * ms) {
  if (ms->config.rgRingBufferEnd != ms->config.rgRingBufferStart) {
    if (ms->config.rgRingBufferStart >= (ms->config.rgRingBufferSize-1)) {
      ms->config.rgRingBufferStart = 0;
    } else {
      ms->config.rgRingBufferStart++;
    }
  }
}
void epRingBufferAdvance(MachineState * ms) {
  if (ms->config.epRingBufferEnd != ms->config.epRingBufferStart) {
    if (ms->config.epRingBufferStart >= (ms->config.epRingBufferSize-1)) {
      ms->config.epRingBufferStart = 0;
    } else {
      ms->config.epRingBufferStart++;
    }
  }
}

// advance the buffers until we have only enough
//  data to account back to time t
void allRingBuffersAdvance(MachineState * ms, ros::Time t) {

  double thisRange;
  Mat thisIm;
  geometry_msgs::Pose thisPose;

  getRingPoseAtTime(ms, t, thisPose, 1);
  for (int i = 0; i < ms->config.cameras.size(); i++) {
    ms->config.cameras[i]->getRingImageAtTime(t, thisIm, 1);
  }
  //getRingRangeAtTime(t, thisRange, 1);
}

void recordReadyRangeReadings(MachineState * ms) {
  // if we have some range readings to process
  if (ms->config.rgRingBufferEnd != ms->config.rgRingBufferStart) {

    // continue until it is empty or we don't have data for a point yet
    int IShouldContinue = 1;
    while (IShouldContinue) {
      if (ms->config.rgRingBufferEnd == ms->config.rgRingBufferStart) {
	IShouldContinue = 0; // not strictly necessary
	break; 
      }
	
      double thisRange = ms->config.rgRingBuffer[ms->config.rgRingBufferStart];
      ros::Time thisTime = ms->config.rgRBTimes[ms->config.rgRingBufferStart];

      Camera * camera  = ms->config.cameras[ms->config.focused_camera];
      geometry_msgs::Pose thisPose;
      Mat thisImage;
      int weHavePoseData = getRingPoseAtTime(ms, thisTime, thisPose);
      int weHaveImData = camera->getRingImageAtTime(thisTime, thisImage);

#ifdef DEBUG_RING_BUFFER
      cout << "  recordReadyRangeReadings()  weHavePoseData weHaveImData: " << weHavePoseData << " " << weHaveImData << endl;
#endif

      // if this request will never be serviceable then forget about it
      if (weHavePoseData == -1) {
	rgRingBufferAdvance(ms);
	IShouldContinue = 1; // not strictly necessary
	cout << "  recordReadyRangeReadings(): dropping stale packet due to epRing. consider increasing buffer size." << endl;
	cout << "  recordReadyRangeReadings() --> " << thisTime << endl; 
      }
      if (weHaveImData == -1) {
	rgRingBufferAdvance(ms);
	IShouldContinue = 1; // not strictly necessary
	cout << "  recordReadyRangeReadings(): dropping stale packet due to imRing. consider increasing buffer size." << endl;
	cout << "  recordReadyRangeReadings() --> " << thisTime << endl; 
      } 
      if ((weHavePoseData == 1) && (weHaveImData == 1)) {

	if (thisRange >= RANGE_UPPER_INVALID) {
	  //cout << "DISCARDED large range reading." << endl;
	  IShouldContinue = 1;
	  rgRingBufferAdvance(ms);
	  continue;
	}
	if (thisRange <= RANGE_LOWER_INVALID) {
	  //cout << "DISCARDED small range reading." << endl;
	  IShouldContinue = 1;
	  rgRingBufferAdvance(ms);
	  continue;
	}

	// actually storing the negative z for backwards compatibility
	double thisZmeasurement = -(thisPose.position.z - thisRange);
	double dX = 0;
	double dY = 0;
	double dZ = 0;

	Eigen::Vector3d rayDirection;

	{
	  Eigen::Quaternionf crane2quat(ms->config.straightDown.qw, ms->config.straightDown.qx, ms->config.straightDown.qy, ms->config.straightDown.qz);
          Eigen::Quaternionf irpos = crane2quat.conjugate() * camera->gear0offset * crane2quat;
	  ms->config.irGlobalPositionEEFrame[0] = irpos.w();
	  ms->config.irGlobalPositionEEFrame[1] = irpos.x();
	  ms->config.irGlobalPositionEEFrame[2] = irpos.y();
	  ms->config.irGlobalPositionEEFrame[3] = irpos.z();
	  Eigen::Quaternionf ceeQuat(thisPose.orientation.w, thisPose.orientation.x, thisPose.orientation.y, thisPose.orientation.z);
	  Eigen::Quaternionf irSensorStartLocal = ceeQuat * irpos * ceeQuat.conjugate();
	  Eigen::Quaternionf irSensorStartGlobal(
						  0.0,
						 (thisPose.position.x - irSensorStartLocal.x()),
						 (thisPose.position.y - irSensorStartLocal.y()),
						 (thisPose.position.z - irSensorStartLocal.z())
						);

	  Eigen::Quaternionf globalUnitZ(0, 0, 0, 1);
	  Eigen::Quaternionf localUnitZ = ceeQuat * globalUnitZ * ceeQuat.conjugate();

	  Eigen::Vector3d irSensorEnd(
				       (thisPose.position.x - irSensorStartLocal.x()) + thisRange*localUnitZ.x(),
				       (thisPose.position.y - irSensorStartLocal.y()) + thisRange*localUnitZ.y(),
				       (thisPose.position.z - irSensorStartLocal.z()) + thisRange*localUnitZ.z()
				      );

	  thisZmeasurement = -irSensorEnd.z();
	  // ATTN 25
	  ms->config.mostRecentUntabledZ = thisZmeasurement;
	  //mostRecentUntabledZ = ((1.0-ms->config.mostRecentUntabledZDecay)*thisZmeasurement) + (ms->config.mostRecentUntabledZDecay*mostRecentUntabledZ);
	  // ATTN 1 currently accounting for table models
	  thisZmeasurement = thisZmeasurement - ms->config.currentTableZ;

	  rayDirection = Eigen::Vector3d(localUnitZ.x(), localUnitZ.y(), localUnitZ.z());
        }
        
	rgRingBufferAdvance(ms);
	// XXX
	//allRingBuffersAdvance(ms, thisTime);
	IShouldContinue = 1; // not strictly necessary
      } else {
	IShouldContinue = 0;
	break;
      }
    }
  }
#ifdef DEBUG_RING_BUFFER
  cout << "recordReadyRangeReadings()  ms->config.rgRingBufferStart ms->config.rgRingBufferEnd: " << ms->config.rgRingBufferStart << " " << ms->config.rgRingBufferEnd << endl;
#endif
}


int classIdxForName(MachineState * ms, string name) {
  int class_idx = -1;
  
  for (int i = 0; i < ms->config.classLabels.size(); i++) {
    if (ms->config.classLabels[i] == name) {
      class_idx = i;
      break;
    }
  }
  if (class_idx == -1) {
    cout << "Could not find class " << name << endl; 
  }
  return class_idx;
}


void writeThumbnail(MachineState * ms, int idx, string thumbnail_file_path) {
  if ( (idx > -1) && (idx < ms->config.classLabels.size()) ) {
    // do nothing
  } else {
    cout << "writeThumbnail: invalid idx, not writing." << endl;
    return;
  }

  {
    string loadPath = thumbnail_file_path + "/ein/servoImages/aerialHeight0PreGradients.png";
    string outPath = thumbnail_file_path + "/thumbnail.png";
    Mat tmp = imread(loadPath);

    std::vector<int> args;
    args.push_back(CV_IMWRITE_PNG_COMPRESSION);
    args.push_back(ms->config.globalPngCompression);

    imwrite(outPath, tmp, args);
  }
}


int MachineState::getStreamPoseAtTime(double tin, eePose * outArm, eePose * outBase) {

  // if we are more than p_rejectThresh away from a measurement, reject it
  double p_rejectThresh = 1.0;
  // XXX int &thisIdx = ms->config.spbCurIdx;
  int thisIdx = ms->config.spbCurIdx;
  vector<streamEePose> &tspb = ms->config.streamPoseBuffer;

  if (tspb.size() < 2) {
    // 2 guards for the for loop that searches down, plus we only want to look it up if its between 2 measurements
    CONSOLE_ERROR(ms, "getStreamPoseAtTime:  tried to get stream pose but the buffer is too small: " << tspb.size());
    return 0;
  }

  if ( (thisIdx > -1) && (thisIdx < tspb.size()) ) {
  } else {
    thisIdx = 0;
  }

  if (tin == tspb[thisIdx].time) {
    (*outArm) = tspb[thisIdx].arm_pose;
    (*outBase) = tspb[thisIdx].base_pose;
    return 1;
  } else if (tin > tspb[thisIdx].time) {
    // checking between
    for (int j = thisIdx; j < tspb.size()-1; j++) {
      if ( (tspb[j].time < tin) && (tin < tspb[j+1].time) ) {
	double w1 = tin - tspb[j].time;
	double w2 = tspb[j+1].time - tin;
	if ( (w1 > p_rejectThresh) || (w2 > p_rejectThresh) ) {
          CONSOLE_ERROR(ms, "getStreamPoseAtTime:  w1 or w2 > p_rejectThresh.  w1: " << w1 << " w2: " << w2 << " p_rejectThresh: " << p_rejectThresh);
	  return 0;
	}
	double totalWeight = w1 + w2;
	w1 = w1 / totalWeight;
	w2 = w2 / totalWeight;
	eePose iBase = tspb[j].base_pose.getInterpolation(tspb[j+1].base_pose, w2);
	eePose iArm = tspb[j].arm_pose.getInterpolation(tspb[j+1].arm_pose, w2);
	(*outArm) = iArm;
	(*outBase) = iBase;
	return 1;
      }
    }
    CONSOLE_ERROR(ms, "getStreamPoseAtTime: didn't return from for loop, tin > tspb[thisIdx].time.  tin: " << tin << " tspb[thisIdx].time: "  << tspb[thisIdx].time);
    CONSOLE_ERROR(ms, "tspb size: " << tspb.size());
    CONSOLE_ERROR(ms, "greater than: " << (tin > tspb[thisIdx].time));
    CONSOLE_ERROR(ms, "equals than: " << (tin == tspb[thisIdx].time));
    return 0;

  } else { // tin < tspb[thisIdx].time
    // checking between
    for (int j = thisIdx-1; j > -1; j--) {
      if ( (tspb[j].time < tin) && (tin < tspb[j+1].time) ) {
	double w1 = tin - tspb[j].time;
	double w2 = tspb[j+1].time - tin;
	if ( (w1 > p_rejectThresh) || (w2 > p_rejectThresh) ) {
          CONSOLE_ERROR(ms, "getStreamPoseAtTime:  w1 or w2 > p_rejectThresh.  w1: " << w1 << " w2: " << w2 << " p_rejectThresh: " << p_rejectThresh);
	  return 0;
	}
	double totalWeight = w1 + w2;
	w1 = w1 / totalWeight;
	w2 = w2 / totalWeight;
	eePose iBase = tspb[j].base_pose.getInterpolation(tspb[j+1].base_pose, w2);
	eePose iArm = tspb[j].arm_pose.getInterpolation(tspb[j+1].arm_pose, w2);
	(*outArm) = iArm;
	(*outBase) = iBase;
	return 1;
      }
    }
    CONSOLE_ERROR(ms, "getStreamPoseAtTime: returned out of the else.");
    return 0;
  }

  assert(0);
}

int MachineState::getStreamPoseAtTimeThreadSafe(double tin, eePose * outArm, eePose * outBase) {

  // if we are more than p_rejectThresh away from a measurement, reject it
  double p_rejectThresh = 1.0;
  // XXX int &thisIdx = ms->config.spbCurIdx;
  int thisIdx = 0;
  vector<streamEePose> &tspb = ms->config.streamPoseBuffer;

  if (tspb.size() < 2) {
    // 2 guards for the for loop that searches down, plus we only want to look it up if its between 2 measurements
    cout << "getStreamPoseAtTime:  tried to get stream pose but the buffer is too small: " << tspb.size() << endl;
    return 0;
  }

  if ( (thisIdx > -1) && (thisIdx < tspb.size()) ) {
  } else {
    thisIdx = 0;
  }

  if (tin == tspb[thisIdx].time) {
    (*outArm) = tspb[thisIdx].arm_pose;
    (*outBase) = tspb[thisIdx].base_pose;
    return 1;
  } else if (tin > tspb[thisIdx].time) {
    // checking between
    for (int j = thisIdx; j < tspb.size()-1; j++) {
      if ( (tspb[j].time < tin) && (tin < tspb[j+1].time) ) {
	double w1 = tin - tspb[j].time;
	double w2 = tspb[j+1].time - tin;
	if ( (w1 > p_rejectThresh) || (w2 > p_rejectThresh) ) {
          cout << "getStreamPoseAtTime:  w1 or w2 > p_rejectThresh.  w1: " << w1 << " w2: " << w2 << " p_rejectThresh: " << p_rejectThresh << endl;
	  return 0;
	} else {
	}
	double totalWeight = w1 + w2;
	w1 = w1 / totalWeight;
	w2 = w2 / totalWeight;
	eePose iBase = tspb[j].base_pose.getInterpolation(tspb[j+1].base_pose, w2);
	eePose iArm = tspb[j].arm_pose.getInterpolation(tspb[j+1].arm_pose, w2);
	(*outArm) = iArm;
	(*outBase) = iBase;
	return 1;
      }
    }
  } else { // tin < tspb[thisIdx].time
    // checking between
    for (int j = thisIdx-1; j > -1; j--) {
      if ( (tspb[j].time < tin) && (tin < tspb[j+1].time) ) {
	double w1 = tin - tspb[j].time;
	double w2 = tspb[j+1].time - tin;
	if ( (w1 > p_rejectThresh) || (w2 > p_rejectThresh) ) {
          cout << "getStreamPoseAtTime:  w1 or w2 > p_rejectThresh.  w1: " << w1 << " w2: " << w2 << " p_rejectThresh: " << p_rejectThresh << endl;
	  return 0;
	}
	double totalWeight = w1 + w2;
	w1 = w1 / totalWeight;
	w2 = w2 / totalWeight;
	eePose iBase = tspb[j].base_pose.getInterpolation(tspb[j+1].base_pose, w2);
	eePose iArm = tspb[j].arm_pose.getInterpolation(tspb[j+1].arm_pose, w2);
	(*outArm) = iArm;
	(*outBase) = iBase;
	return 1;
      }
    }
    cout << "bottomed out of the if." << endl;
    return 0;
  }
}

// casts ray of length thisRange from end effector position thisPose to obtain castPointOut in direction rayDirectionOut 
void castRangeRay(MachineState * ms, double thisRange, eePose thisPose, Vector3d * rayDirectionOut) {

  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  Eigen::Quaternionf crane2quat(ms->config.straightDown.qw, ms->config.straightDown.qx, ms->config.straightDown.qy, ms->config.straightDown.qz);
  Eigen::Quaternionf irpos = crane2quat.conjugate() * camera->gear0offset * crane2quat;
  ms->config.irGlobalPositionEEFrame[0] = irpos.w();
  ms->config.irGlobalPositionEEFrame[1] = irpos.x();
  ms->config.irGlobalPositionEEFrame[2] = irpos.y();
  ms->config.irGlobalPositionEEFrame[3] = irpos.z();

  Eigen::Quaternionf ceeQuat(thisPose.qw, thisPose.qx, thisPose.qy, thisPose.qz);
  Eigen::Quaternionf irSensorStartLocal = ceeQuat * irpos * ceeQuat.conjugate();
  Eigen::Quaternionf irSensorStartGlobal(
					  0.0,
					 (thisPose.px - irSensorStartLocal.x()),
					 (thisPose.py - irSensorStartLocal.y()),
					 (thisPose.pz - irSensorStartLocal.z())
					);

  Eigen::Quaternionf globalUnitZ(0, 0, 0, 1);
  Eigen::Quaternionf localUnitZ = ceeQuat * globalUnitZ * ceeQuat.conjugate();

  Eigen::Vector3d irSensorEnd(
			       (thisPose.px - irSensorStartLocal.x()) + thisRange*localUnitZ.x(),
			       (thisPose.py - irSensorStartLocal.y()) + thisRange*localUnitZ.y(),
			       (thisPose.pz - irSensorStartLocal.z()) + thisRange*localUnitZ.z()
			      );


  (*rayDirectionOut) = Eigen::Vector3d(localUnitZ.x(), localUnitZ.y(), localUnitZ.z());
}


bool streamRangeComparator (streamRange i, streamRange j) {
 return (i.time < j.time);
}
bool streamPoseComparator (streamEePose i, streamEePose j) {
 return (i.time < j.time);
}
bool streamImageComparator (streamImage i, streamImage j) {
 return (i.time < j.time);
}
bool streamJointsComparator(streamJoints i, streamJoints j) {
 return (i.time < j.time);
}
bool streamWordComparator(streamWord i, streamWord j) {
 return (i.time < j.time);
}
bool streamLabelComparator(streamLabel i, streamLabel j) {
 return (i.time < j.time);
}



void populateStreamJointsBuffer(MachineState * ms) {
// XXX TODO
}

void streamJointAsClass(MachineState * ms, int classToStreamIdx, double now) {
// XXX TODO
}

void writeJointsBatchAsClass(MachineState * ms, int classToStreamIdx) {
// XXX TODO
}


void populateStreamWordBuffer(MachineState * ms) {
// XXX TODO
  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");
  string dotyml(".yml");

  int classToStreamIdx = ms->config.focusedClass;
  if (ms->config.focusedClass == -1) {
    return;
  }
  string this_word_path = streamDirectory(ms, classToStreamIdx) + "/word/";
  dpdf = opendir(this_word_path.c_str());
  cout << "Populating stream word buffer from " << this_word_path << endl;
  if (dpdf != NULL) {
    while (epdf = readdir(dpdf)) {

      string fname(epdf->d_name);
      string fextension;
      string fnoextension;
      if (fname.length() > 4) {
	fextension = fname.substr(fname.length() - 4, 4);
	fnoextension = fname.substr(0, fname.length() - 4);
      } else {
      } // do nothing

      if (!dotyml.compare(fextension) && dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

	string inFileName = this_word_path + fname;
	cout << "Streaming words from " << inFileName << " ...";
	FileStorage fsvI;
	fsvI.open(inFileName, FileStorage::READ);

	{
	  FileNode anode = fsvI["words"];
	  {
	    FileNode bnode = anode["size"];
	    FileNodeIterator itb = bnode.begin(), itb_end = bnode.end();
	    int tnp = -1;
	    if (itb != itb_end) {
	      tnp = *(itb++);
	    } else {
	    }

	    FileNode cnode = anode["streamWords"];
	    FileNodeIterator itc = cnode.begin(), itc_end = cnode.end();
	    int numLoadedWords = 0;
	    for ( ; itc != itc_end; itc++, numLoadedWords++) {
	      streamWord toAdd;
	      int loaded = 1;
	      {
		FileNode dnode = (*itc)["word"];
		FileNodeIterator itd = dnode.begin(), itd_end = dnode.end();
		if (itd != itd_end) {
		  toAdd.word= (string)(*itd);
// remove cout
		  cout << "Read word: " << toAdd.word<< " ." << endl;
		} else {
		  loaded = 0;
		  cout << "Word not found :P" << endl;
		}
	      }
	      {
		FileNode dnode = (*itc)["command"];
		FileNodeIterator itd = dnode.begin(), itd_end = dnode.end();
		if (itd != itd_end) {
		  toAdd.command= (string)(*itd);
// remove cout
		  cout << "Read command: " << toAdd.command<< " ." << endl;
		} else {
		  loaded = 0;
		  cout << "Word not found :P" << endl;
		}
	      }
	      {
		FileNode dnode = (*itc)["time"];
		FileNodeIterator itd = dnode.begin(), itd_end = dnode.end();
		if (itd != itd_end) {
		  toAdd.time = (*itd);
// remove cout
		  cout << "Read time: " << toAdd.time << " ." << endl;
		} else {
		  loaded = 0;
		  cout << "Time not found :P" << endl;
		}
	      }
	      if (loaded) {
		ms->config.streamWordBuffer.push_back(toAdd);
	      } else {
		cout << "failed :P" << endl;
	      }
	    }
	    if (numLoadedWords != tnp) {
	      CONSOLE_ERROR(ms, "Did not load the expected number of words.");
	    }
	    cout << " Expected to load " << tnp << " words, loaded " << numLoadedWords << " ..." << endl; cout.flush();
	  }
	}
      }
    }
  }
}


void checkAndStreamWord(MachineState * ms, string wordIn, string commandIn) {
  //cout << "checkAndStreamWord: " << wordIn << " " << commandIn << endl;

  int cfClass = ms->config.focusedClass;
  if ((cfClass > -1) && (cfClass < ms->config.classLabels.size()) && (ms->config.sensorStreamOn) && (ms->config.sisWord)) {
    ros::Time rNow = ros::Time::now();
    double thisNow = rNow.toSec();
    streamWordAsClass(ms, wordIn, commandIn, cfClass, thisNow);

    //for (int i = 0; i < ms->config.streamWordBuffer.size(); i++) {
      //cout << "  streamWordBuffer[" << i << "] = " << ms->config.streamWordBuffer[i].word << " " << ms->config.streamWordBuffer[i].command << " " << ms->config.streamWordBuffer[i].time << endl;;
    //}
  } else {
    //cout << "  streamWord failed " << wordIn << endl;
    //cout << " XXX " << (cfClass > -1)  << (cfClass < ms->config.classLabels.size()) << (ms->config.sensorStreamOn) << (ms->config.sisWord) << endl;
  } // do nothing
}

void writeSideAndSerialToFileStorage(MachineState * ms, FileStorage& fsvO) {
  fsvO << "serial" <<  ms->config.robot_serial;
  fsvO << "side" << ms->config.left_or_right_arm;
}

void readSideAndSerialFromFileStorage(MachineState * ms, FileStorage fsvI, string * serial, string * side) {
  FileNode anode = fsvI["words"];

  {
    FileNode bnode = anode["serial"];
    FileNodeIterator itb = bnode.begin(), itb_end = bnode.end();
    if (itb != itb_end) {
      (*serial) = (string)(*(itb++));
    } else {
    }
  }
  {
    FileNode bnode = anode["side"];
    FileNodeIterator itb = bnode.begin(), itb_end = bnode.end();
    if (itb != itb_end) {
      (*side) = (string)(*(itb++));
    } else {
    }
  }
}

string appendSideAndSerial(MachineState * ms, string root) {
  string toReturn = root + "_" + ms->config.robot_serial + "_" + ms->config.left_or_right_arm;
  return toReturn;
}

void streamWordAsClass(MachineState * ms, string wordIn, string commandIn, int classToStreamIdx, double now) {
  if (didSensorStreamTimeout(ms)) {
    return;
  } else {
  }

  int cfClass = ms->config.focusedClass;
  if ((cfClass > -1) && (cfClass < ms->config.classLabels.size())) {
    streamWord toAdd;
    toAdd.word = wordIn;
    toAdd.command = commandIn;
    toAdd.time = now;
    ms->config.streamWordBuffer.push_back(toAdd);
    cout << "streamWordAsClass pushed back " << toAdd.word << " " << toAdd.command << endl;
  } else {
    cout << "streamWordAsClass: invalid focused class, deactivating streaming." << endl;
    ms->config.sensorStreamOn = 0;
    return;
  } 


  if (ms->config.diskStreamingEnabled) {
    if (ms->config.streamWordBuffer.size() >= ms->config.streamWordBatchSize) {
      writeWordBatchAsClass(ms, classToStreamIdx);	
      ms->config.streamWordBuffer.resize(0);
    } else {
    } // do nothing
  } else {
    if ((ms->config.streamWordBuffer.size() % ms->config.streamWordBatchSize) == 0) {
      cout << "streamWordAsClass: disk streaming not enabled, buffer size: " << ms->config.streamWordBuffer.size() << endl;
    } else {
    }
  }
}

void writeWordBatchAsClass(MachineState * ms, int classToStreamIdx) {
  if (ms->config.streamWordBuffer.size() > 0) {
  } else {
    cout << "writeWordBatchAsClass: buffer empty, returning." << endl;
    return;
  }

  if ((classToStreamIdx > -1) && (classToStreamIdx < ms->config.classLabels.size())) {
    // do nothing
  } else {
    cout << "writeWordBatchAsClass: invalid class, not writing." << endl;
    return;
  }

  string this_image_path = streamDirectory(ms, classToStreamIdx) + "/word/";
  ros::Time thisNow = ros::Time::now();
  char buf[1024];
  sprintf(buf, "%s%f", this_image_path.c_str(), thisNow.toSec());
  string root_path(buf); 
  root_path = appendSideAndSerial(ms, root_path);


  string yaml_path = root_path + ".yml";
  // XXX take this cout out
  cout << "Streaming current word batch to " << yaml_path << endl;

  // may want to save additional camera parameters
  FileStorage fsvO;
  fsvO.open(yaml_path, FileStorage::WRITE);

  fsvO << "words" << "{";
  {
	writeSideAndSerialToFileStorage(ms, fsvO);

    int tng = ms->config.streamWordBuffer.size();
    fsvO << "size" <<  tng;
    fsvO << "streamWords" << "[" ;
    for (int i = 0; i < tng; i++) {
      fsvO << "{:";
	fsvO << "word" << ms->config.streamWordBuffer[i].word;
	fsvO << "command" << ms->config.streamWordBuffer[i].command;
	fsvO << "time" << ms->config.streamWordBuffer[i].time;
      fsvO << "}";
      // XXX take this cout out
      //cout << " wrote word: " << ms->config.streamWordBuffer[i].word << " and time: " << ms->config.streamWordBuffer[i].time << endl;
    }
    fsvO << "]";
  }
  fsvO << "}";
}


void populateStreamLabelBuffer(MachineState * ms) {
  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");
  string dotyml(".yml");

  int classToStreamIdx = ms->config.focusedClass;
  if (ms->config.focusedClass == -1) {
    return;
  }
  string this_label_path = streamDirectory(ms, classToStreamIdx) + "/label/";
  dpdf = opendir(this_label_path.c_str());
  cout << "Populating stream label buffer from " << this_label_path << endl;
  if (dpdf != NULL) {
    while (epdf = readdir(dpdf)) {

      string fname(epdf->d_name);
      string fextension;
      string fnoextension;
      if (fname.length() > 4) {
	fextension = fname.substr(fname.length() - 4, 4);
	fnoextension = fname.substr(0, fname.length() - 4);
      } else {
      } // do nothing

      if (!dotyml.compare(fextension) && dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

	string inFileName = this_label_path + fname;
	cout << "Streaming labels from " << inFileName << " ...";
	FileStorage fsvI;
	fsvI.open(inFileName, FileStorage::READ);

	{
	  FileNode anode = fsvI["labels"];
	  {
	    FileNode bnode = anode["size"];
	    FileNodeIterator itb = bnode.begin(), itb_end = bnode.end();
	    int tnp = -1;
	    if (itb != itb_end) {
	      tnp = *(itb++);
	    } else {
	    }

	    FileNode cnode = anode["streamLabels"];
	    FileNodeIterator itc = cnode.begin(), itc_end = cnode.end();
	    int numLoadedLabels = 0;
	    for ( ; itc != itc_end; itc++, numLoadedLabels++) {
	      streamLabel toAdd;
	      int loaded = 1;
	      {
		FileNode dnode = (*itc)["label"];
		FileNodeIterator itd = dnode.begin(), itd_end = dnode.end();
		if (itd != itd_end) {
		  toAdd.label= (string)(*itd);
// remove cout
		  cout << "Read label: " << toAdd.label<< " ." << endl;
		} else {
		  loaded = 0;
		  cout << "Label not found :P" << endl;
		}
	      }
	      {
		FileNode dnode = (*itc)["time"];
		FileNodeIterator itd = dnode.begin(), itd_end = dnode.end();
		if (itd != itd_end) {
		  toAdd.time = (*itd);
// remove cout
		  cout << "Read time: " << toAdd.time << " ." << endl;
		} else {
		  loaded = 0;
		  cout << "Time not found :P" << endl;
		}
	      }
	      if (loaded) {
		ms->config.streamLabelBuffer.push_back(toAdd);
	      } else {
		cout << "failed :P" << endl;
	      }
	    }
	    if (numLoadedLabels != tnp) {
	      CONSOLE_ERROR(ms, "Did not load the expected number of labels.");
	    }
	    cout << " Expected to load " << tnp << " labels, loaded " << numLoadedLabels << " ..." << endl; cout.flush();
	  }
	}
      }
    }
  }
}

void streamLabelAsClass(MachineState * ms, string labelIn, int classToStreamIdx, double now) {

  if (didSensorStreamTimeout(ms)) {
    return;
  } else {
  }

  int cfClass = ms->config.focusedClass;
  if ((cfClass > -1) && (cfClass < ms->config.classLabels.size())) {
    streamLabel toAdd;
    toAdd.label = labelIn;
    toAdd.time = now;
    ms->config.streamLabelBuffer.push_back(toAdd);
  } else {
    cout << "streamLabelAsClass: invalid focused class, deactivating streaming." << endl;
    ms->config.sensorStreamOn = 0;
    return;
  } 


  if (ms->config.diskStreamingEnabled) {
    if (ms->config.streamLabelBuffer.size() >= ms->config.streamLabelBatchSize) {
      writeLabelBatchAsClass(ms, classToStreamIdx);	
      ms->config.streamLabelBuffer.resize(0);
    } else {
    } // do nothing
  } else {
    if ((ms->config.streamLabelBuffer.size() % ms->config.streamLabelBatchSize) == 0) {
      cout << "streamLabelAsClass: disk streaming not enabled, buffer size: " << ms->config.streamLabelBuffer.size() << endl;
    } else {
    }
  }
}

void writeLabelBatchAsClass(MachineState * ms, int classToStreamIdx) {
// XXX TODO

  if (ms->config.streamLabelBuffer.size() <= 0) {
    cout << "writeLabelBatchAsClass: buffer empty, returning." << endl;
    return;
  }

  if ((classToStreamIdx > -1) && (classToStreamIdx < ms->config.classLabels.size())) {
    // do nothing
  } else {
    cout << "writeLabelBatchAsClass: invalid class, not writing." << endl;
    return;
  }

  string this_image_path = streamDirectory(ms, classToStreamIdx) + "/label/";
  ros::Time thisNow = ros::Time::now();
  char buf[1024];
  sprintf(buf, "%s%f", this_image_path.c_str(), thisNow.toSec());
  string root_path(buf); 
  root_path = appendSideAndSerial(ms, root_path);


  string yaml_path = root_path + ".yml";
  // XXX take this cout out
  cout << "Streaming current label batch to " << yaml_path << endl;

  // may want to save additional camera parameters
  FileStorage fsvO;
  fsvO.open(yaml_path, FileStorage::WRITE);

  fsvO << "labels" << "{";
  {
	writeSideAndSerialToFileStorage(ms, fsvO);

    int tng = ms->config.streamLabelBuffer.size();
    fsvO << "size" <<  tng;
    fsvO << "streamLabels" << "[" ;
    for (int i = 0; i < tng; i++) {
      fsvO << "{:";
	fsvO << "label" << ms->config.streamLabelBuffer[i].label;
	fsvO << "time" << ms->config.streamLabelBuffer[i].time;
      fsvO << "}";
      // XXX take this cout out
      //cout << " wrote label: " << ms->config.streamLabelBuffer[i].label << " and time: " << ms->config.streamLabelBuffer[i].time << endl;
    }
    fsvO << "]";
  }
  fsvO << "}";
}



void populateStreamRangeBuffer(MachineState * ms) {
  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");
  string dotyml(".yml");

  int classToStreamIdx = ms->config.focusedClass;
  if (ms->config.focusedClass == -1) {
    return;
  }
  string this_range_path = streamDirectory(ms, classToStreamIdx) + "/range/";
  dpdf = opendir(this_range_path.c_str());
  cout << "Populating stream range buffer from " << this_range_path << endl;
  if (dpdf != NULL) {
    while (epdf = readdir(dpdf)) {

      string fname(epdf->d_name);
      string fextension;
      string fnoextension;
      if (fname.length() > 4) {
	fextension = fname.substr(fname.length() - 4, 4);
	fnoextension = fname.substr(0, fname.length() - 4);
      } else {
      } // do nothing

      if (!dotyml.compare(fextension) && dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

	string inFileName = this_range_path + fname;
	cout << "Streaming ranges from " << inFileName << " ...";
	FileStorage fsvI;
	fsvI.open(inFileName, FileStorage::READ);

	{
	  FileNode anode = fsvI["ranges"];
	  {
	    FileNode bnode = anode["size"];
	    FileNodeIterator itb = bnode.begin(), itb_end = bnode.end();
	    int tnp = -1;
	    if (itb != itb_end) {
	      tnp = *(itb++);
	    } else {
	    }

	    FileNode cnode = anode["streamRanges"];
	    FileNodeIterator itc = cnode.begin(), itc_end = cnode.end();
	    int numLoadedRanges = 0;
	    for ( ; itc != itc_end; itc++, numLoadedRanges++) {
	      streamRange toAdd;
	      int loaded = 1;
	      {
		FileNode dnode = (*itc)["range"];
		FileNodeIterator itd = dnode.begin(), itd_end = dnode.end();
		if (itd != itd_end) {
		  toAdd.range= (*itd);
		  //cout << "Read range: " << toAdd.range<< " ." << endl;
		} else {
		  loaded = 0;
		  cout << "Range not found :P" << endl;
		}
	      }
	      {
		FileNode dnode = (*itc)["time"];
		FileNodeIterator itd = dnode.begin(), itd_end = dnode.end();
		if (itd != itd_end) {
		  toAdd.time = (*itd);
		  //cout << "Read time: " << toAdd.time << " ." << endl;
		} else {
		  loaded = 0;
		  cout << "Time not found :P" << endl;
		}
	      }
	      if (loaded) {
		ms->config.streamRangeBuffer.push_back(toAdd);
	      } else {
		cout << "failed :P" << endl;
	      }
	    }
	    if (numLoadedRanges != tnp) {
	      CONSOLE_ERROR(ms, "Did not load the expected number of ranges.");
	    }
	    cout << " Expected to load " << tnp << " ranges, loaded " << numLoadedRanges << " ..." << endl; cout.flush();
	  }
	}
      }
    }
  }
}

void populateStreamPoseBuffer(MachineState * ms) {
  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");
  string dotyml(".yml");

  int classToStreamIdx = ms->config.focusedClass;
  if (ms->config.focusedClass == -1) {
    return;
  }
  string this_pose_path = streamDirectory(ms, classToStreamIdx) + "/pose/";
  dpdf = opendir(this_pose_path.c_str());
  cout << "Populating stream pose buffer from " << this_pose_path << endl;
  if (dpdf != NULL) {
    while (epdf = readdir(dpdf)) {

      string fname(epdf->d_name);
      string fextension;
      string fnoextension;
      if (fname.length() > 4) {
	fextension = fname.substr(fname.length() - 4, 4);
	fnoextension = fname.substr(0, fname.length() - 4);
      } else {
      } // do nothing

      if (!dotyml.compare(fextension) && dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

	string inFileName = this_pose_path + fname;
	cout << "Streaming poses from " << inFileName << " ...";
	FileStorage fsvI;
	fsvI.open(inFileName, FileStorage::READ);

	{
	  FileNode anode = fsvI["poses"];
	  {
	    FileNode bnode = anode["size"];
	    FileNodeIterator itb = bnode.begin(), itb_end = bnode.end();
	    int tnp = -1;
	    if (itb != itb_end) {
	      tnp = *(itb++);
	    } else {
	    }

	    FileNode cnode = anode["streamPoses"];
	    FileNodeIterator itc = cnode.begin(), itc_end = cnode.end();
	    int numLoadedPoses = 0;
	    for ( ; itc != itc_end; itc++, numLoadedPoses++) {
	      streamEePose toAdd;
	      int loaded = 1;
	      {
		{
		  FileNode dnode = (*itc)["arm_pose"];
		  toAdd.arm_pose.readFromFileNode(dnode);
		  //cout << "Read arm pose " << toAdd.arm_pose << endl;
		}
		{
		  FileNode dnode = (*itc)["base_pose"];
		  toAdd.base_pose.readFromFileNode(dnode);
		  //cout << "Read base pose " << toAdd.base_pose << endl;
		}
	      }
	      {
		//cout << "about to get time..." << endl;
		FileNode dnode = (*itc)["time"];
		FileNodeIterator itd = dnode.begin(), itd_end = dnode.end();
		if (itd != itd_end) {
		  toAdd.time = (*itd);
		  //cout << "Read time: " << toAdd.time << " ." << endl;
		} else {
		  loaded = 0;
		  //cout << "Arm pose not found :P" << endl;
		}
	      }
	      if (loaded) {
		ms->config.streamPoseBuffer.push_back(toAdd);
	      } else {
		cout << "failed :P" << endl;
	      }
	    }
	    if (numLoadedPoses != tnp) {
	      CONSOLE_ERROR(ms, "Did not load the expected number of poses.");
	    }
	    cout << " Expected to load " << tnp << " poses, loaded " << numLoadedPoses << " ..." << endl; cout.flush();
	  }
	}
      }
    }
  }
}

void activateSensorStreaming(MachineState * ms) {
  ros::NodeHandle n("~");

  int cfClass = ms->config.focusedClass;
  if ((cfClass > -1) && (cfClass < ms->config.classLabels.size())) {
    string this_label_name = ms->config.classLabels[cfClass]; 

    string this_raw_path = streamDirectory(ms, cfClass);
    string this_image_path = this_raw_path + "/images/";
    string this_pose_path = this_raw_path + "/pose/";
    string this_range_path = this_raw_path +  "/range/";
    string this_joints_path = this_raw_path +  "/joints/";
    string this_word_path = this_raw_path + "/word/";
    string this_label_path = this_raw_path + "/label/";
    string this_calibration_path = ms->config.data_directory + "/objects/" + this_label_name + "/ein/calibration/";
    mkdir(this_raw_path.c_str(), 0777);
    mkdir(this_image_path.c_str(), 0777);
    mkdir(this_pose_path.c_str(), 0777);
    mkdir(this_range_path.c_str(), 0777);
    mkdir(this_joints_path.c_str(), 0777);
    mkdir(this_word_path.c_str(), 0777);
    mkdir(this_label_path.c_str(), 0777);
    mkdir(this_calibration_path.c_str(), 0777);
    ms->config.sensorStreamOn = 1;

    robotActivateSensorStreaming(ms);

    for (int i = 0; i < ms->config.cameras.size(); i++) {
      ms->config.cameras[i]->activateSensorStreaming();
    }
    CONSOLE(ms, "Activating sensor streaming.");
    ros::Time thisTime = ros::Time::now();
    ms->config.sensorStreamLastActivated = thisTime.toSec();
  } else {
    CONSOLE_ERROR(ms, "Cannot activate sensor stream: invalid focused class.");
  } 
}

void deactivateSensorStreaming(MachineState * ms) {
  cout << "deactivateSensorStreaming: Making node handle." << endl;
  ros::NodeHandle n("~");
  CONSOLE(ms, "Deactivating sensor streaming.");

  cout << "deactivateSensorStreaming: Making image transport." << endl;
  ms->config.sensorStreamOn = 0;
  // restore those queue sizes to defaults.

  robotDeactivateSensorStreaming(ms);

  for (int i = 0; i < ms->config.cameras.size(); i++) {
    ms->config.cameras[i]->deactivateSensorStreaming();
  }
  if (ms->config.diskStreamingEnabled) {
    cout << "deactivateSensorStreaming: About to write batches... ";
    int cfClass = ms->config.focusedClass;
    if ((cfClass > -1) && (cfClass < ms->config.classLabels.size())) {
      writeRangeBatchAsClass(ms, cfClass);	
      writePoseBatchAsClass(ms, cfClass);	
      writeJointsBatchAsClass(ms, cfClass);	
      writeWordBatchAsClass(ms, cfClass);	
      writeLabelBatchAsClass(ms, cfClass);	

      ms->config.streamPoseBuffer.resize(0);
      ms->config.streamRangeBuffer.resize(0);
      ms->config.streamJointsBuffer.resize(0);
      ms->config.streamWordBuffer.resize(0);
      ms->config.streamLabelBuffer.resize(0);
      

      cout << "Wrote batches." << endl;
    } else {
      cout << "Did not write batches, invalid focused class." << endl;
    } 
  } else {
    cout << "deactivateSensorStreaming: Disk streaming not enabled, keeping range and pose stream buffers populated." << endl;
  }
}


int didSensorStreamTimeout(MachineState * ms) {
  ros::Time safetyNow =  ros::Time::now();
  double sNow = safetyNow.toSec();
  if (sNow - ms->config.sensorStreamLastActivated > ms->config.sensorStreamTimeout) {
    cout << "Whoops, sensor stream timed out, clearing stack and deactivating." << endl;
    ms->clearStack();
    deactivateSensorStreaming(ms);
    return 1;
  } else {
    return 0;
  }
}


void streamRangeAsClass(MachineState * ms, double rangeIn, int classToStreamIdx, double now) {

  if (didSensorStreamTimeout(ms)) {
    return;
  } else {
  }

  int cfClass = ms->config.focusedClass;
  if ((cfClass > -1) && (cfClass < ms->config.classLabels.size())) {
    streamRange toAdd;
    toAdd.range = rangeIn;
    toAdd.time = now;
    ms->config.streamRangeBuffer.push_back(toAdd);
  } else {
    cout << "streamRangeAsClass: invalid focused class, deactivating streaming." << endl;
    ms->config.sensorStreamOn = 0;
    return;
  } 


  if (ms->config.diskStreamingEnabled) {
    if (ms->config.streamRangeBuffer.size() >= ms->config.streamRangeBatchSize) {
      writeRangeBatchAsClass(ms, classToStreamIdx);	
      ms->config.streamRangeBuffer.resize(0);
    } else {
    } // do nothing
  } else {
    if ((ms->config.streamRangeBuffer.size() % ms->config.streamRangeBatchSize) == 0) {
      cout << "streamRangeAsClass: disk streaming not enabled, buffer size: " << ms->config.streamRangeBuffer.size() << endl;
    } else {
    }
  }
}

void streamPoseAsClass(MachineState * ms, eePose poseIn, int classToStreamIdx, double now) {

  if (didSensorStreamTimeout(ms)) {
    return;
  } else {
  }

  int cfClass = ms->config.focusedClass;
  if ((cfClass > -1) && (cfClass < ms->config.classLabels.size())) {
    streamEePose toAdd;
    toAdd.arm_pose = poseIn;
    toAdd.base_pose = ms->config.c3dPoseBase;
    toAdd.time = now;

    ms->config.streamPoseBuffer.push_back(toAdd);
  } else {
    cout << "streamPoseAsClass: invalid focused class, deactivating streaming." << endl;
    ms->config.sensorStreamOn = 0;
  } 

  if (ms->config.diskStreamingEnabled) {
    if (ms->config.streamPoseBuffer.size() >= ms->config.streamPoseBatchSize) {
	writePoseBatchAsClass(ms, classToStreamIdx);	
      ms->config.streamPoseBuffer.resize(0);
    } else {
    } // do nothing
  } else {
    if ((ms->config.streamPoseBuffer.size() % ms->config.streamPoseBatchSize) == 0) {
      cout << "streamPoseAsClass: disk streaming not enabled, buffer size: " << ms->config.streamPoseBuffer.size() << endl;
    } else {
    }
  }
}

void writeRangeBatchAsClass(MachineState * ms, int classToStreamIdx) {
  if (ms->config.streamRangeBuffer.size() > 0) {
  } else {
    cout << "writeRangeBatchAsClass: buffer empty, returning." << endl;
    return;
  }

  if ((classToStreamIdx > -1) && (classToStreamIdx < ms->config.classLabels.size())) {
    // do nothing
  } else {
    cout << "writeRangeBatchAsClass: invalid class, not writing." << endl;
    return;
  }
  string this_image_path = streamDirectory(ms, classToStreamIdx) + "/range/";
  ros::Time thisNow = ros::Time::now();
  char buf[1024];
  sprintf(buf, "%s%f", this_image_path.c_str(), thisNow.toSec());
  string root_path(buf); 
  root_path = appendSideAndSerial(ms, root_path);


  string yaml_path = root_path + ".yml";
  // XXX take this cout out
  cout << "Streaming current range batch to " << yaml_path << endl;

  // may want to save additional camera parameters
  FileStorage fsvO;
  fsvO.open(yaml_path, FileStorage::WRITE);

  fsvO << "ranges" << "{";
  {
	writeSideAndSerialToFileStorage(ms, fsvO);

    int tng = ms->config.streamRangeBuffer.size();
    fsvO << "size" <<  tng;
    fsvO << "streamRanges" << "[" ;
    for (int i = 0; i < tng; i++) {
      fsvO << "{:";
	fsvO << "range" << ms->config.streamRangeBuffer[i].range;
	fsvO << "time" << ms->config.streamRangeBuffer[i].time;
      fsvO << "}";
      // XXX take this cout out
      //cout << " wrote range: " << ms->config.streamRangeBuffer[i].range << " and time: " << ms->config.streamRangeBuffer[i].time << endl;
    }
    fsvO << "]";
  }
  fsvO << "}";
}

void writePoseBatchAsClass(MachineState * ms, int classToStreamIdx) {
  if (ms->config.streamPoseBuffer.size() > 0) {
  } else {
    cout << "writePoseBatchAsClass: buffer empty, returning." << endl;
    return;
  }

  if ((classToStreamIdx > -1) && (classToStreamIdx < ms->config.classLabels.size())) {
    // do nothing
  } else {
    cout << "writePoseBatchAsClass: invalid class, not writing." << endl;
    return;
  }
  string this_image_path = streamDirectory(ms, classToStreamIdx) + "/pose/";
  ros::Time thisNow = ros::Time::now();
  char buf[1024];
  sprintf(buf, "%s%f", this_image_path.c_str(), thisNow.toSec());
  string root_path(buf); 
  root_path = appendSideAndSerial(ms, root_path);

  string yaml_path = root_path + ".yml";
  // XXX take this cout out
  cout << "Streaming current pose batch to " << yaml_path << endl;

  // may want to save additional camera parameters
  FileStorage fsvO;
  fsvO.open(yaml_path, FileStorage::WRITE);

  fsvO << "poses" << "{";
  {
	writeSideAndSerialToFileStorage(ms, fsvO);

    int tng = ms->config.streamPoseBuffer.size();
    fsvO << "size" <<  tng;
    fsvO << "streamPoses" << "[" ;
    for (int i = 0; i < tng; i++) {
      fsvO << "{:";
	fsvO << "arm_pose";
	  ms->config.streamPoseBuffer[i].arm_pose.writeToFileStorage(fsvO);
	fsvO << "base_pose";
	  ms->config.streamPoseBuffer[i].base_pose.writeToFileStorage(fsvO);
	fsvO << "time" << ms->config.streamPoseBuffer[i].time;
      fsvO << "}";
      // XXX take this cout out
      //cout << " wrote arm_pose: " << ms->config.streamPoseBuffer[i].arm_pose << " base pose: base_pose: " << ms->config.streamPoseBuffer[i].base_pose << " and time: " << ms->config.streamPoseBuffer[i].time << endl;
    }
    fsvO << "]";
  }
  fsvO << "}";
  fsvO.release();
}

void write3dGrasps(MachineState * ms, int idx, string this_grasp_path) {
  if ((idx > -1) && (idx < ms->config.class3dGrasps.size())) {
    // do nothing
  } else {
    cout << "write3dGrasps: invalid idx, not writing." << endl;
    return;
  }

  FileStorage fsvO;
  cout << "write3dGrasps: Writing: " << this_grasp_path << endl;
  fsvO.open(this_grasp_path, FileStorage::WRITE);

  fsvO << "grasps" << "{";
  {
    int tng = ms->config.class3dGrasps[idx].size();
    fsvO << "size" <<  tng;
    fsvO << "graspPoses" << "[" ;
    for (int i = 0; i < tng; i++) {
      ms->config.class3dGrasps[idx][i].writeToFileStorage(fsvO);
      cout << " wrote pose: " << ms->config.class3dGrasps[idx][i] << endl;
    }
    fsvO << "]";
  }
  fsvO << "}";

  fsvO << "placeUnderPoints" << "{";
  {
    int tng = ms->config.classPlaceUnderPoints[idx].size();
    fsvO << "size" <<  tng;
    fsvO << "pupPoses" << "[" ;
    for (int i = 0; i < tng; i++) {
      ms->config.classPlaceUnderPoints[idx][i].writeToFileStorage(fsvO);
      cout << " wrote pup pose: " << ms->config.classPlaceUnderPoints[idx][i] << endl;
    }
    fsvO << "]";
  }
  fsvO << "}";

  fsvO << "placeOverPoints" << "{";
  {
    int tng = ms->config.classPlaceOverPoints[idx].size();
    fsvO << "size" <<  tng;
    fsvO << "popPoses" << "[" ;
    for (int i = 0; i < tng; i++) {
      ms->config.classPlaceOverPoints[idx][i].writeToFileStorage(fsvO);
      cout << " wrote pop pose: " << ms->config.classPlaceOverPoints[idx][i] << endl;
    }
    fsvO << "]";
  }

  fsvO.release();
}


void writeSceneModel(MachineState * ms, int idx, string this_scene_path) {
  // initialize this if we need to
  guardSceneModels(ms);

  if ((idx > -1) && (idx < ms->config.class_scene_models.size())) {
    // do nothing
  } else {
    cout << "writeSceneModel: invalid idx, not writing." << endl;
    return;
  }

  ms->config.class_scene_models[idx]->saveToFile(this_scene_path);
}

void initStreamFolders(MachineState * ms, string folderName) {
  
}

void initClassFolders(MachineState * ms, string folderName) {
  string item = folderName + "/";
  string raw = item + "raw/";
  string images = raw + "images/";
  string poseBatches = raw + "pose/";
  string rangeBatches = raw + "range/";
  string ein = item + "ein/";
  string d3dGrasps = ein + "3dGrasps/";
  string detectionCrops = ein + "detectionCrops/";
  string sceneModel = ein + "sceneModel/";
  string ir2d = ein + "ir2d/";
  string pickMemories = ein + "pickMemories/";
  string servoCrops = ein + "servoCrops/";
  string servoImages = ein + "servoImages/";
  string knn = ein + "knn/";
  string calibration = ein + "calibration/";
  
  mkdir(item.c_str(), 0777);
  mkdir(raw.c_str(), 0777);
  mkdir(images.c_str(), 0777);
  mkdir(poseBatches.c_str(), 0777);
  mkdir(rangeBatches.c_str(), 0777);
  mkdir(ein.c_str(), 0777);
  mkdir(d3dGrasps.c_str(), 0777);
  mkdir(detectionCrops.c_str(), 0777);
  mkdir(sceneModel.c_str(), 0777);
  mkdir(ir2d.c_str(), 0777);
  mkdir(pickMemories.c_str(), 0777);
  mkdir(servoCrops.c_str(), 0777);
  mkdir(servoImages.c_str(), 0777);
  mkdir(knn.c_str(), 0777);
  mkdir(calibration.c_str(), 0777);
}

void writeClassToFolder(MachineState * ms, int idx, string folderName) {

  if ((idx > -1) && (idx < ms->config.classLabels.size())) {
    // do nothing
  } else {
    CONSOLE_ERROR(ms, "writeClassToFolder: invalid idx, not writing.");
    return;
  }

  string item = folderName + "/";
    string raw = item + "raw/";
      string images = raw + "images/";
      string poseBatches = raw + "poseBatches/";
      string rangeBatches = raw + "rangeBatches/";
    string ein = item + "ein/";
      string d3dGrasps = ein + "3dGrasps/";
      string detectionCrops = ein + "detectionCrops/";
      string sceneModel = ein + "sceneModel/";
      string ir2d = ein + "ir2d/";
      string pickMemories = ein + "pickMemories/";
      string servoCrops = ein + "servoCrops/";
      string servoImages = ein + "servoImages/";

  initClassFolders(ms, folderName);
  CONSOLE(ms, "Writing class " << idx << " to folder " << folderName);

  string d3d_grasp_file_path = d3dGrasps + "3dGrasps.yml";
  write3dGrasps(ms, idx, d3d_grasp_file_path);
  
  string thumbnail_file_path = item;
  writeThumbnail(ms, idx, thumbnail_file_path);

  string scene_model_file_path = sceneModel + "model.yml";
  writeSceneModel(ms, idx, scene_model_file_path);
}

void writeClassGraspsToFolder(MachineState * ms, int idx, string folderName) {

  if ((idx > -1) && (idx < ms->config.classLabels.size())) {
    // do nothing
  } else {
    CONSOLE_ERROR(ms, "writeClassGraspsToFolder: invalid idx, not writing.");
    return;
  }

  string item = folderName + "/";
    string raw = item + "raw/";
      string images = raw + "images/";
      string poseBatches = raw + "poseBatches/";
      string rangeBatches = raw + "rangeBatches/";
    string ein = item + "ein/";
      string d3dGrasps = ein + "3dGrasps/";
      string detectionCrops = ein + "detectionCrops/";
      string sceneModel = ein + "sceneModel/";
      string ir2d = ein + "ir2d/";
      string pickMemories = ein + "pickMemories/";
      string servoCrops = ein + "servoCrops/";
      string servoImages = ein + "servoImages/";

  initClassFolders(ms, folderName);
  CONSOLE(ms, "Writing grasps for class " << idx << " to folder " << folderName);

  string d3d_grasp_file_path = d3dGrasps + "3dGrasps.yml";
  write3dGrasps(ms, idx, d3d_grasp_file_path);

}



void MachineState::moveEndEffectorCommandCallback(const geometry_msgs::Pose& msg) {
  cout << "moveEndEffectorCommandCallback" << endl << msg.position << msg.orientation << endl;
  MachineState * ms = this;
  if (ms->config.currentRobotMode == PHYSICAL) {
    return;
  } else if (ms->config.currentRobotMode == SIMULATED) {
    ms->config.currentEEPose.px = msg.position.x;
    ms->config.currentEEPose.py = msg.position.y;
    ms->config.currentEEPose.pz = msg.position.z;
  } else if (ms->config.currentRobotMode == SNOOP) {
    return;
  } else {
    assert(0);
  }
}


void MachineState::pickObjectUnderEndEffectorCommandCallback(const std_msgs::Empty& msg) {
  MachineState * ms = this;
  if (ms->config.currentRobotMode == PHYSICAL) {
    return;
  } else if (ms->config.currentRobotMode == SIMULATED) {
    if (ms->config.objectInHandLabel == -1) {
      // this is a fake box to test intersection
      int probeBoxHalfWidthPixels = 10;
      BoxMemory box;
      Camera * camera  = ms->config.cameras[ms->config.focused_camera];

      box.bTop.x = camera->vanishingPointReticle.px-probeBoxHalfWidthPixels;
      box.bTop.y = camera->vanishingPointReticle.py-probeBoxHalfWidthPixels;
      box.bBot.x = camera->vanishingPointReticle.px+probeBoxHalfWidthPixels;
      box.bBot.y = camera->vanishingPointReticle.py+probeBoxHalfWidthPixels;
      box.cameraPose = ms->config.currentEEPose;
      box.top = pixelToGlobalEEPose(ms, box.bTop.x, box.bTop.y, ms->config.trueEEPose.position.z + ms->config.currentTableZ);
      box.bot = pixelToGlobalEEPose(ms, box.bBot.x, box.bBot.y, ms->config.trueEEPose.position.z + ms->config.currentTableZ);
      box.centroid.px = (box.top.px + box.bot.px) * 0.5;
      box.centroid.py = (box.top.py + box.bot.py) * 0.5;
      box.centroid.pz = (box.top.pz + box.bot.pz) * 0.5;
      box.cameraTime = ros::Time::now();
      box.labeledClassIndex = 0;

      vector<BoxMemory> newMemories;
      bool foundOne = false;
      int foundClassIndex = -1;
      for (int i = 0; i < ms->config.blueBoxMemories.size(); i++) {
	if ( (!foundOne) && (boxMemoryIntersectCentroid(box, ms->config.blueBoxMemories[i])) ) {
	  foundOne = true;
	  foundClassIndex = ms->config.blueBoxMemories[i].labeledClassIndex;
	} else {
	  newMemories.push_back(ms->config.blueBoxMemories[i]);
	}
      }
      ms->config.blueBoxMemories = newMemories;
      ms->config.objectInHandLabel = foundClassIndex;
      if (ms->config.objectInHandLabel >= 0) {
	cout << "pickObjectUnderEndEffectorCommandCallback: The " << ms->config.classLabels[ms->config.objectInHandLabel] << " you found is now in your hand." << endl;
      } else {
	cout << "pickObjectUnderEndEffectorCommandCallback: Alas, nothing to be found." << endl;
      }
    } else {
      if (ms->config.objectInHandLabel >= 0) {
	cout << "pickObjectUnderEndEffectorCommandCallback: Not picking because of the " << ms->config.classLabels[ms->config.objectInHandLabel] << " you already hold." << endl;
      } else {
	cout << "pickObjectUnderEndEffectorCommandCallback: Not picking because objectInHandLabel is " << ms->config.objectInHandLabel << "." << endl;
      }
    }
  } else if (ms->config.currentRobotMode == SNOOP) {
    return;
  } else {
    assert(0);
  }
}

void MachineState::placeObjectInEndEffectorCommandCallback(const std_msgs::Empty& msg) {
  MachineState * ms = this;
  if (ms->config.currentRobotMode == PHYSICAL) {
    return;
  } else if (ms->config.currentRobotMode == SIMULATED) {
    if (ms->config.objectInHandLabel >= 0) {
      BoxMemory box;
      Camera * camera  = ms->config.cameras[ms->config.focused_camera];

      box.bTop.x = camera->vanishingPointReticle.px-ms->config.simulatedObjectHalfWidthPixels;
      box.bTop.y = camera->vanishingPointReticle.py-ms->config.simulatedObjectHalfWidthPixels;
      box.bBot.x = camera->vanishingPointReticle.px+ms->config.simulatedObjectHalfWidthPixels;
      box.bBot.y = camera->vanishingPointReticle.py+ms->config.simulatedObjectHalfWidthPixels;
      box.cameraPose = ms->config.currentEEPose;
      box.top = pixelToGlobalEEPose(ms, box.bTop.x, box.bTop.y, ms->config.trueEEPose.position.z + ms->config.currentTableZ);
      box.bot = pixelToGlobalEEPose(ms, box.bBot.x, box.bBot.y, ms->config.trueEEPose.position.z + ms->config.currentTableZ);
      box.centroid.px = (box.top.px + box.bot.px) * 0.5;
      box.centroid.py = (box.top.py + box.bot.py) * 0.5;
      box.centroid.pz = (box.top.pz + box.bot.pz) * 0.5;
      box.cameraTime = ros::Time::now();
      box.labeledClassIndex = ms->config.objectInHandLabel;
      
      mapBox(ms, box);
      vector<BoxMemory> newMemories;
      for (int i = 0; i < ms->config.blueBoxMemories.size(); i++) {
	newMemories.push_back(ms->config.blueBoxMemories[i]);
      }
      newMemories.push_back(box);
      ms->config.blueBoxMemories = newMemories;
      cout << "placeObjectInEndEffectorCommandCallback: You dropped the " << ms->config.classLabels[ms->config.objectInHandLabel] << "." << endl;
    } else {
      cout << "placeObjectInEndEffectorCommandCallback: Not placing because objectInHandLabel is " << ms->config.objectInHandLabel << "." << endl;
    }
    ms->config.objectInHandLabel = -1;
  } else if (ms->config.currentRobotMode == SNOOP) {
    return;
  } else {
    assert(0);
  }
}

void MachineState::forthCommandCallback(const std_msgs::String::ConstPtr& msg) {
  MachineState * ms = this;
  cout << "Received " << ms->config.forthCommand << endl;
  ms->config.forthCommand = msg->data;
  evaluateProgram(msg->data);

}



bool isGripperGripping(MachineState * ms) {
  //return (ms->config.gripperPosition >= ms->config.gripperThresh);
  return ms->config.gripperGripping; 
}

bool isGripperMoving(MachineState * ms) {
  //return (ms->config.gripperPosition >= ms->config.gripperThresh);
  return ms->config.gripperMoving; 
}


int getColorReticleX(MachineState * ms) {
  // rounding
  //int tcri = int(round((eeRange - camera->firstCReticleIndexDepth)/camera->cReticleIndexDelta));
  //tcri = min(max(tcri,0),camera->numCReticleIndexes-1);
  //return camera->xCR[tcri];

  // interpolating
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  int tcriL = int(floor((ms->config.eeRange - camera->firstCReticleIndexDepth)/camera->cReticleIndexDelta));
  int tcriH = int(ceil((ms->config.eeRange - camera->firstCReticleIndexDepth)/camera->cReticleIndexDelta));
  tcriL = min(max(tcriL,0),camera->numCReticleIndexes-1);
  tcriH = min(max(tcriH,0),camera->numCReticleIndexes-1);

  double tcrwL = ((ms->config.eeRange - camera->firstCReticleIndexDepth)/camera->cReticleIndexDelta) - floor((ms->config.eeRange - camera->firstCReticleIndexDepth)/camera->cReticleIndexDelta);
  double tcrwH = ceil((ms->config.eeRange - camera->firstCReticleIndexDepth)/camera->cReticleIndexDelta) - ((ms->config.eeRange - camera->firstCReticleIndexDepth)/camera->cReticleIndexDelta);

  if (tcriL == tcriH)
    return camera->xCR[tcriL];
  else
    return int(round(tcrwL*double(camera->xCR[tcriL]) + tcrwH*double(camera->xCR[tcriH])));
}

int getColorReticleY(MachineState * ms) {
  // rounding
  //int tcri = int(round((ms->config.eeRange - camera->firstCReticleIndexDepth)/camera->cReticleIndexDelta));
  //tcri = min(max(tcri,0),camera->numCReticleIndexes-1);
  //return camera->yCR[tcri];
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  // interpolating
  int tcriL = int(floor((ms->config.eeRange - camera->firstCReticleIndexDepth)/camera->cReticleIndexDelta));
  int tcriH = int(ceil((ms->config.eeRange - camera->firstCReticleIndexDepth)/camera->cReticleIndexDelta));
  tcriL = min(max(tcriL,0),camera->numCReticleIndexes-1);
  tcriH = min(max(tcriH,0),camera->numCReticleIndexes-1);

  double tcrwL = ((ms->config.eeRange - camera->firstCReticleIndexDepth)/camera->cReticleIndexDelta) - floor((ms->config.eeRange - camera->firstCReticleIndexDepth)/camera->cReticleIndexDelta);
  double tcrwH = ceil((ms->config.eeRange - camera->firstCReticleIndexDepth)/camera->cReticleIndexDelta) - ((ms->config.eeRange - camera->firstCReticleIndexDepth)/camera->cReticleIndexDelta);

  if (tcriL == tcriH)
    return camera->yCR[tcriL];
  else
    return int(round(tcrwL*double(camera->yCR[tcriL]) + tcrwH*double(camera->yCR[tcriH])));
}

cv::Vec3b getCRColor(MachineState * ms) {
  cv::Vec3b toReturn(0,0,0);
  if (ms->config.wristCamInit) {
    int crX = getColorReticleX(ms);
    int crY = getColorReticleY(ms);

    if ((crX < ms->config.wristCamImage.cols) && (crY < ms->config.wristCamImage.rows))
      toReturn = ms->config.wristCamImage.at<cv::Vec3b>(crY,crX); 
  }
  return toReturn;
}

// XXX TODO this should really use a buffered ms->config.eeRange
cv::Vec3b getCRColor(MachineState * ms, Mat im) {
  cv::Vec3b toReturn(0,0,0);

  int crX = getColorReticleX(ms);
  int crY = getColorReticleY(ms);

  if ((crX < im.cols) && (crY < im.rows))
    toReturn = im.at<cv::Vec3b>(crY,crX); 

  return toReturn;
}

Quaternionf extractQuatFromPose(geometry_msgs::Pose poseIn) {
  return Quaternionf(poseIn.orientation.w, poseIn.orientation.x, poseIn.orientation.y, poseIn.orientation.z);
}





Eigen::Quaternionf getGGRotation(MachineState * ms, int givenGraspGear) {
  Eigen::Vector3f localUnitX;
  {
    Eigen::Quaternionf qin(0, 1, 0, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    //Eigen::Quaternionf eeqform(ms->config.eepReg2.qw, ms->config.eepReg2.qx, ms->config.eepReg2.qy, ms->config.eepReg2.qz);
    Eigen::Quaternionf eeqform(0, 0, 1.0, 0);
    //Eigen::Quaternionf eeqform(ms->config.currentEEPose.qw, ms->config.currentEEPose.qx, ms->config.currentEEPose.qy, ms->config.currentEEPose.qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitX.x() = qout.x();
    localUnitX.y() = qout.y();
    localUnitX.z() = qout.z();
  }

  Eigen::Vector3f localUnitY;
  {
    Eigen::Quaternionf qin(0, 0, 1, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    //Eigen::Quaternionf eeqform(ms->config.eepReg2.qw, ms->config.eepReg2.qx, ms->config.eepReg2.qy, ms->config.eepReg2.qz);
    Eigen::Quaternionf eeqform(0, 0, 1.0, 0);
    //Eigen::Quaternionf eeqform(ms->config.currentEEPose.qw, ms->config.currentEEPose.qx, ms->config.currentEEPose.qy, ms->config.currentEEPose.qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitY.x() = qout.x();
    localUnitY.y() = qout.y();
    localUnitY.z() = qout.z();
  }

  Eigen::Vector3f localUnitZ;
  {
    Eigen::Quaternionf qin(0, 0, 0, 1);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    //Eigen::Quaternionf eeqform(ms->config.eepReg2.qw, ms->config.eepReg2.qx, ms->config.eepReg2.qy, ms->config.eepReg2.qz);
    Eigen::Quaternionf eeqform(0, 0, 1.0, 0);
    //Eigen::Quaternionf eeqform(ms->config.currentEEPose.qw, ms->config.currentEEPose.qx, ms->config.currentEEPose.qy, ms->config.currentEEPose.qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitZ.x() = qout.x();
    localUnitZ.y() = qout.y();
    localUnitZ.z() = qout.z();
  }

  double deltaTheta = double(givenGraspGear)*2.0*3.1415926/double(ms->config.totalGraspGears);
  double sinBuff = 0.0;
  double angleRate = 1.0;
  //Eigen::Quaternionf eeBaseQuat(ms->config.eepReg2.qw, ms->config.eepReg2.qx, ms->config.eepReg2.qy, ms->config.eepReg2.qz);
  Eigen::Quaternionf eeBaseQuat(0, 0, 1, 0);
  sinBuff = sin(angleRate*0.0/2.0);
  Eigen::Quaternionf eeRotatorX(cos(angleRate*0.0/2.0), localUnitX.x()*sinBuff, localUnitX.y()*sinBuff, localUnitX.z()*sinBuff);
  sinBuff = sin(angleRate*0.0/2.0);
  Eigen::Quaternionf eeRotatorY(cos(angleRate*0.0/2.0), localUnitY.x()*sinBuff, localUnitY.y()*sinBuff, localUnitY.z()*sinBuff);
  sinBuff = sin(angleRate*deltaTheta/2.0);
  Eigen::Quaternionf eeRotatorZ(cos(angleRate*deltaTheta/2.0), localUnitZ.x()*sinBuff, localUnitZ.y()*sinBuff, localUnitZ.z()*sinBuff);
  eeRotatorX.normalize();
  eeRotatorY.normalize();
  eeRotatorZ.normalize();

  eeBaseQuat = eeRotatorX * eeRotatorY * eeRotatorZ * eeBaseQuat;
  eeBaseQuat.normalize();

  return eeBaseQuat;
}

void setGGRotation(MachineState * ms, int thisGraspGear) {
  Eigen::Quaternionf eeBaseQuat = getGGRotation(ms, thisGraspGear);

  ms->config.currentEEPose.qx = eeBaseQuat.x();
  ms->config.currentEEPose.qy = eeBaseQuat.y();
  ms->config.currentEEPose.qz = eeBaseQuat.z();
  ms->config.currentEEPose.qw = eeBaseQuat.w();
}

Eigen::Quaternionf getCCRotation(MachineState * ms, int givenGraspGear, double angle) {
  Eigen::Vector3f localUnitX;
  {
    Eigen::Quaternionf qin(0, 1, 0, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    //Eigen::Quaternionf eeqform(ms->config.eepReg2.qw, ms->config.eepReg2.qx, ms->config.eepReg2.qy, ms->config.eepReg2.qz);
    Eigen::Quaternionf eeqform(0, 0, 1.0, 0);
    //Eigen::Quaternionf eeqform(ms->config.currentEEPose.qw, ms->config.currentEEPose.qx, ms->config.currentEEPose.qy, ms->config.currentEEPose.qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitX.x() = qout.x();
    localUnitX.y() = qout.y();
    localUnitX.z() = qout.z();
  }

  Eigen::Vector3f localUnitY;
  {
    Eigen::Quaternionf qin(0, 0, 1, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    //Eigen::Quaternionf eeqform(ms->config.eepReg2.qw, ms->config.eepReg2.qx, ms->config.eepReg2.qy, ms->config.eepReg2.qz);
    Eigen::Quaternionf eeqform(0, 0, 1.0, 0);
    //Eigen::Quaternionf eeqform(ms->config.currentEEPose.qw, ms->config.currentEEPose.qx, ms->config.currentEEPose.qy, ms->config.currentEEPose.qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitY.x() = qout.x();
    localUnitY.y() = qout.y();
    localUnitY.z() = qout.z();
  }

  Eigen::Vector3f localUnitZ;
  {
    Eigen::Quaternionf qin(0, 0, 0, 1);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    //Eigen::Quaternionf eeqform(ms->config.eepReg2.qw, ms->config.eepReg2.qx, ms->config.eepReg2.qy, ms->config.eepReg2.qz);
    Eigen::Quaternionf eeqform(0, 0, 1.0, 0);
    //Eigen::Quaternionf eeqform(ms->config.currentEEPose.qw, ms->config.currentEEPose.qx, ms->config.currentEEPose.qy, ms->config.currentEEPose.qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitZ.x() = qout.x();
    localUnitZ.y() = qout.y();
    localUnitZ.z() = qout.z();
  }

  double deltaTheta = angle + (double(givenGraspGear)*2.0*3.1415926/double(ms->config.totalGraspGears));
  double sinBuff = 0.0;
  double angleRate = 1.0;
  //Eigen::Quaternionf eeBaseQuat(ms->config.eepReg2.qw, ms->config.eepReg2.qx, ms->config.eepReg2.qy, ms->config.eepReg2.qz);
  //Eigen::Quaternionf eeBaseQuat(0, 0, 1, 0);
  Eigen::Quaternionf eeBaseQuat(ms->config.bestOrientationEEPose.qw, ms->config.bestOrientationEEPose.qx, ms->config.bestOrientationEEPose.qy, ms->config.bestOrientationEEPose.qz);
  sinBuff = sin(angleRate*0.0/2.0);
  Eigen::Quaternionf eeRotatorX(cos(angleRate*0.0/2.0), localUnitX.x()*sinBuff, localUnitX.y()*sinBuff, localUnitX.z()*sinBuff);
  sinBuff = sin(angleRate*0.0/2.0);
  Eigen::Quaternionf eeRotatorY(cos(angleRate*0.0/2.0), localUnitY.x()*sinBuff, localUnitY.y()*sinBuff, localUnitY.z()*sinBuff);
  sinBuff = sin(angleRate*deltaTheta/2.0);
  Eigen::Quaternionf eeRotatorZ(cos(angleRate*deltaTheta/2.0), localUnitZ.x()*sinBuff, localUnitZ.y()*sinBuff, localUnitZ.z()*sinBuff);
  eeRotatorX.normalize();
  eeRotatorY.normalize();
  eeRotatorZ.normalize();

  eeBaseQuat = eeRotatorX * eeRotatorY * eeRotatorZ * eeBaseQuat;
  eeBaseQuat.normalize();

  return eeBaseQuat;
}

void setCCRotation(MachineState * ms, int thisGraspGear) {
  //Eigen::Quaternionf eeBaseQuat = getCCRotation(ms, thisGraspGear, -ms->config.bestOrientationAngle);
  Eigen::Quaternionf eeBaseQuat = getCCRotation(ms, thisGraspGear, 0.0);

  ms->config.currentEEPose.qx = eeBaseQuat.x();
  ms->config.currentEEPose.qy = eeBaseQuat.y();
  ms->config.currentEEPose.qz = eeBaseQuat.z();
  ms->config.currentEEPose.qw = eeBaseQuat.w();
}


void MachineState::accelerometerCallback(const sensor_msgs::Imu& moment) {
  MachineState * ms = this;
  ms->config.lastAccelerometerCallbackReceived = ros::Time::now();
  ms->config.eeLinearAcceleration[0] = moment.linear_acceleration.x;
  ms->config.eeLinearAcceleration[1] = moment.linear_acceleration.y;
  ms->config.eeLinearAcceleration[2] = moment.linear_acceleration.z;
}

void MachineState::rangeCallback(const sensor_msgs::Range& range) {
  MachineState * ms = this;
  //cout << "range frame_id: " << range.header.frame_id << endl;
  setRingRangeAtTime(ms, range.header.stamp, range.range);
  //double thisRange;
  //int weHaveRangeData = getRingRangeAtTime(range.header.stamp, thisRange);

  int cfClass = ms->config.focusedClass;
  if ((cfClass > -1) && (cfClass < ms->config.classLabels.size()) && (ms->config.sensorStreamOn) && (ms->config.sisRange)) {
    double thisNow = range.header.stamp.toSec();
    streamRangeAsClass(ms, range.range, cfClass, thisNow); 
  } else {
  } // do nothing

  time(&ms->config.thisTimeRange);
  double deltaTimeRange = difftime(ms->config.thisTimeRange, ms->config.firstTimeRange);
  ms->config.timeMassRange = ms->config.timeMassRange + 1;

  if (deltaTimeRange > ms->config.timeIntervalRange) {
    deltaTimeRange = 0;
    ms->config.timeMassRange = 0;
    time(&ms->config.firstTimeRange);
  }

  if (ms->config.timeMassRange > 0.0) {
    ms->config.aveTimeRange = deltaTimeRange / ms->config.timeMassRange;
  }

  if (deltaTimeRange > 0.0) {
    ms->config.aveFrequencyRange = ms->config.timeMassRange / deltaTimeRange;
  }

  ms->config.eeRange = range.range;
  ms->config.rangeHistory[ms->config.currentRangeHistoryIndex] = ms->config.eeRange;
  ms->config.currentRangeHistoryIndex++;
  ms->config.currentRangeHistoryIndex = ms->config.currentRangeHistoryIndex % ms->config.totalRangeHistoryLength;


  for (int rr = ms->config.currentRangeHistoryIndex-1; rr <= ms->config.currentRangeHistoryIndex; rr++) {
    int r = 0;
    if (rr == -1)
      r = ms->config.totalRangeHistoryLength-1;
    else
      r = rr;

    cv::Scalar fillColor(0,0,0);
    cv::Scalar backColor(0,0,0);
    int topY = 0;
    if (r == ms->config.currentRangeHistoryIndex) {
      fillColor = cv::Scalar(0,0,255);
      topY = 0;
    } else {
      fillColor = cv::Scalar(0,64,0);
      double thisHeight = floor(ms->config.rangeHistory[r]*ms->config.rggHeight);
      thisHeight = min(thisHeight,double(ms->config.rggHeight));
      topY = thisHeight;
      //cout << " " << ms->config.rangeHistory[r] << " " << thisHeight << " " << ms->config.rggHeight << " " << topY << endl;
    }
    int truH = ms->config.rggHeight-topY;
    {
      cv::Point outTop = cv::Point(r*ms->config.rggStride, 0);
      cv::Point outBot = cv::Point((r+1)*ms->config.rggStride, ms->config.rggHeight);
      Mat vCrop = ms->config.rangeogramImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
      vCrop = backColor;
    }{
      cv::Point outTop = cv::Point(r*ms->config.rggStride, topY);
      cv::Point outBot = cv::Point((r+1)*ms->config.rggStride, ms->config.rggHeight);
      Mat vCrop = ms->config.rangeogramImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
      vCrop += fillColor;
    }
    if (r != ms->config.currentRangeHistoryIndex) {
      {
	cv::Point outTop = cv::Point(r*ms->config.rggStride, topY);
	cv::Point outBot = cv::Point((r+1)*ms->config.rggStride, topY+truH/8);
	Mat vCrop = ms->config.rangeogramImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
	vCrop += fillColor;
      }{
	cv::Point outTop = cv::Point(r*ms->config.rggStride, topY);
	cv::Point outBot = cv::Point((r+1)*ms->config.rggStride, topY+truH/16);
	Mat vCrop = ms->config.rangeogramImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
	vCrop += fillColor;
      }
    }
  }

  {
    cv::Point text_anchor = cv::Point(0,ms->config.rangeogramImage.rows-1);
    {
      cv::Scalar backColor(0,0,0);
      cv::Point outTop = cv::Point(text_anchor.x,text_anchor.y+1-35);
      cv::Point outBot = cv::Point(text_anchor.x+350,text_anchor.y+1);
      Mat vCrop = ms->config.rangeogramImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
      vCrop = backColor;
    }
    {
      char buff[256];
      sprintf(buff, "            Hz: %.2f", ms->config.aveFrequency);
      string fpslabel(buff);
      putText(ms->config.rangeogramImage, fpslabel, text_anchor, MY_FONT, 1.0, Scalar(160,0,0), 1.0);
    }
    {
      char buff[256];
      sprintf(buff, "Hz: %.2f", ms->config.aveFrequencyRange);
      string fpslabel(buff);
      putText(ms->config.rangeogramImage, fpslabel, text_anchor, MY_FONT, 1.0, Scalar(0,0,160), 1.0);
    }
  }

  if (ms->config.showgui) {
    ms->config.rangeogramWindow->updateImage(ms->config.rangeogramImage);
  }

  if (!ms->config.shouldIRangeCallback) {
    return;
  }

  if (ms->config.shouldIRender && ms->config.showgui) {
    ms->config.objectViewerWindow->updateImage(ms->config.objectViewerImage);

    ms->config.objectMapViewerWindow->updateImage(ms->config.objectMapViewerImage);

    ms->config.densityViewerWindow->updateImage(ms->config.densityViewerImage);

    ms->config.mapBackgroundViewWindow->updateImage(ms->config.mapBackgroundImage);
  }
}

void endEffectorAngularUpdate(eePose *givenEEPose, eePose *deltaEEPose) {

  /* end effector local angular update */
  Eigen::Vector3f localUnitX;
  {
    Eigen::Quaternionf qin(0, 1, 0, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(givenEEPose->qw, givenEEPose->qx, givenEEPose->qy, givenEEPose->qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitX.x() = qout.x();
    localUnitX.y() = qout.y();
    localUnitX.z() = qout.z();
  }

  Eigen::Vector3f localUnitY;
  {
    Eigen::Quaternionf qin(0, 0, 1, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(givenEEPose->qw, givenEEPose->qx, givenEEPose->qy, givenEEPose->qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitY.x() = qout.x();
    localUnitY.y() = qout.y();
    localUnitY.z() = qout.z();
  }

  Eigen::Vector3f localUnitZ;
  {
    Eigen::Quaternionf qin(0, 0, 0, 1);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(givenEEPose->qw, givenEEPose->qx, givenEEPose->qy, givenEEPose->qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitZ.x() = qout.x();
    localUnitZ.y() = qout.y();
    localUnitZ.z() = qout.z();
  }

  double sinBuff = 0.0;
  double angleRate = 1.0;
  Eigen::Quaternionf eeBaseQuat(givenEEPose->qw, givenEEPose->qx, givenEEPose->qy, givenEEPose->qz);
  sinBuff = sin(angleRate*deltaEEPose->px/2.0);
  Eigen::Quaternionf eeRotatorX(cos(angleRate*deltaEEPose->px/2.0), localUnitX.x()*sinBuff, localUnitX.y()*sinBuff, localUnitX.z()*sinBuff);
  sinBuff = sin(angleRate*deltaEEPose->py/2.0);
  Eigen::Quaternionf eeRotatorY(cos(angleRate*deltaEEPose->py/2.0), localUnitY.x()*sinBuff, localUnitY.y()*sinBuff, localUnitY.z()*sinBuff);
  sinBuff = sin(angleRate*deltaEEPose->pz/2.0);
  Eigen::Quaternionf eeRotatorZ(cos(angleRate*deltaEEPose->pz/2.0), localUnitZ.x()*sinBuff, localUnitZ.y()*sinBuff, localUnitZ.z()*sinBuff);
  deltaEEPose->px = 0;
  deltaEEPose->py = 0;
  deltaEEPose->pz = 0;


  eeRotatorX.normalize();
  eeRotatorY.normalize();
  eeRotatorZ.normalize();

  eeBaseQuat = eeRotatorX * eeRotatorY * eeRotatorZ * eeBaseQuat;
  eeBaseQuat.normalize();

  givenEEPose->qx = eeBaseQuat.x();
  givenEEPose->qy = eeBaseQuat.y();
  givenEEPose->qz = eeBaseQuat.z();
  givenEEPose->qw = eeBaseQuat.w();
}

void endEffectorAngularUpdateOuter(eePose *givenEEPose, eePose *deltaEEPose) {

  /* end effector local angular update */
  Eigen::Vector3f localUnitX;
  {
    Eigen::Quaternionf qin(0, 1, 0, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(givenEEPose->qw, givenEEPose->qx, givenEEPose->qy, givenEEPose->qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitX.x() = qout.x();
    localUnitX.y() = qout.y();
    localUnitX.z() = qout.z();
  }

  Eigen::Vector3f localUnitY;
  {
    Eigen::Quaternionf qin(0, 0, 1, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(givenEEPose->qw, givenEEPose->qx, givenEEPose->qy, givenEEPose->qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitY.x() = qout.x();
    localUnitY.y() = qout.y();
    localUnitY.z() = qout.z();
  }

  Eigen::Vector3f localUnitZ;
  {
    Eigen::Quaternionf qin(0, 0, 0, 1);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(givenEEPose->qw, givenEEPose->qx, givenEEPose->qy, givenEEPose->qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitZ.x() = qout.x();
    localUnitZ.y() = qout.y();
    localUnitZ.z() = qout.z();
  }

  double sinBuff = 0.0;
  double angleRate = 1.0;
  Eigen::Quaternionf eeBaseQuat(givenEEPose->qw, givenEEPose->qx, givenEEPose->qy, givenEEPose->qz);
  sinBuff = sin(angleRate*deltaEEPose->px/2.0);
  Eigen::Quaternionf eeRotatorX(cos(angleRate*deltaEEPose->px/2.0), localUnitX.x()*sinBuff, localUnitX.y()*sinBuff, localUnitX.z()*sinBuff);
  sinBuff = sin(angleRate*deltaEEPose->py/2.0);
  Eigen::Quaternionf eeRotatorY(cos(angleRate*deltaEEPose->py/2.0), localUnitY.x()*sinBuff, localUnitY.y()*sinBuff, localUnitY.z()*sinBuff);
  sinBuff = sin(angleRate*deltaEEPose->pz/2.0);
  Eigen::Quaternionf eeRotatorZ(cos(angleRate*deltaEEPose->pz/2.0), localUnitZ.x()*sinBuff, localUnitZ.y()*sinBuff, localUnitZ.z()*sinBuff);
  deltaEEPose->px = 0;
  deltaEEPose->py = 0;
  deltaEEPose->pz = 0;


  eeRotatorX.normalize();
  eeRotatorY.normalize();
  eeRotatorZ.normalize();
  //eeBaseQuat = eeRotatorX * eeRotatorY * eeRotatorZ * 
		//eeBaseQuat * 
	      //eeRotatorZ.conjugate() * eeRotatorY.conjugate() * eeRotatorX.conjugate();
  eeBaseQuat = eeBaseQuat * eeRotatorX * eeRotatorY * eeRotatorZ;
  eeBaseQuat.normalize();

  givenEEPose->qx = eeBaseQuat.x();
  givenEEPose->qy = eeBaseQuat.y();
  givenEEPose->qz = eeBaseQuat.z();
  givenEEPose->qw = eeBaseQuat.w();
}






void MachineState::timercallback1(const ros::TimerEvent&) {


  ros::NodeHandle n("~");

  MachineState * ms = this;


  int c = -1;
  if (ms->config.shouldIMiscCallback) {
    QApplication::instance()->processEvents();
    c = ms->config.last_key;
    ms->config.last_key = -1;
  } else if ((ms->config.heartBeatCounter % ms->config.heartBeatPeriod) == 0) {
    QApplication::instance()->processEvents();
    c = ms->config.last_key;
    ms->config.last_key = -1;
    ms->config.heartBeatCounter = 0;

  }
  ms->config.heartBeatCounter++;

  if (ms->config.armWidget) {
    ms->config.armWidget->update();
    ms->config.renderedWristViewWindow->updateImage(ms->config.wristViewImage);
  }
  if (einMainWindow != NULL) {
    einMainWindow->update();
  }

  // XXX is heartBeatCounter even used?

  if (c != -1) {
    // don't print for capslock, shift, alt (for alt-tab)
    if (!(c == 65509 || c == 196581 || c == 196577 || c == 65505 ||
          c == 65513 || c == 196578)) {
      //cout << "You pressed " << c << "." << endl;

      if (character_code_to_word.count(c) > 0) {
        shared_ptr<Word> keycode_word = character_code_to_word[c];
        ms->execute(keycode_word);

      } else {
        //cout  << "Could not find word for " << c << endl;
      }
    }
  }


  // always call execute stack, whether or not we are paused.
  if (ms->call_stack.size() > 0 && ms->call_stack.back()->name() == "executeStack") {
    shared_ptr<Word> execute_stack_word = ms->popWord();
    ms->execute(execute_stack_word);
  }


  ms->config.endThisStackCollapse = ms->config.endCollapse;
  while (1) {
    std::shared_ptr<Word> word = NULL;
    time(&ms->config.thisTime);
    double deltaTime = difftime(ms->config.thisTime, ms->config.firstTime);
    ms->config.timeMass = ms->config.timeMass + 1;

    if (deltaTime > ms->config.timeInterval) {
      deltaTime = 0;
      ms->config.timeMass = 0;
      time(&ms->config.firstTime);
    }

    if (ms->config.timeMass > 0.0) {
      ms->config.aveTime = deltaTime / ms->config.timeMass;
    }

    if (deltaTime > 0.0) {
      ms->config.aveFrequency = ms->config.timeMass / deltaTime;
    }

    // deal with the stack

    if (ms->execute_stack) {
      if (ms->call_stack.size() > 0) {
	word = ms->popWord();
      } else {
	ms->execute_stack = 0;
	ms->config.endThisStackCollapse = 1;
      }
    } else {
      ms->config.endThisStackCollapse = 1;
    }


    if (word != NULL) {
      ms->execute(word);
    }

    if (ms->execution_mode == STEP) {
      ms->execute_stack = 0;
    }

    if (ms->config.endThisStackCollapse || (ms->call_stack.size() == 0)) {
      break;
    }
    
  }


  if (ros::Time::now() - ms->config.lastStatePubTime > ros::Duration(0.1)) {
    EinState state;
    fillEinStateMsg(ms, &state);
    ms->config.einStatePub.publish(state);
    ms->config.lastStatePubTime = ros::Time::now();
  }

  endEffectorAngularUpdate(&ms->config.currentEEPose, &ms->config.currentEEDeltaRPY);

  if (!ms->config.zero_g_toggle) {
    robotUpdate(ms);
  } else {
    ms->config.currentEEPose.px = ms->config.trueEEPose.position.x;
    ms->config.currentEEPose.py = ms->config.trueEEPose.position.y;
    ms->config.currentEEPose.pz = ms->config.trueEEPose.position.z;
    ms->config.currentEEPose.qx = ms->config.trueEEPose.orientation.x;
    ms->config.currentEEPose.qy = ms->config.trueEEPose.orientation.y;
    ms->config.currentEEPose.qz = ms->config.trueEEPose.orientation.z;
    ms->config.currentEEPose.qw = ms->config.trueEEPose.orientation.w;


    robotSetCurrentJointPositions(ms);
  }

  if (ms->config.showgui) {
    if (ms->config.coreViewWindow->isVisible()) {
      renderCoreView(ms);
      ms->config.coreViewWindow->updateImage(ms->config.coreViewImage);
    }
    
    if (ms->config.rangeogramWindow->isVisible()) {
      renderRangeogramView(ms);
    }
    
    if (ms->config.shouldIRender && ms->config.showgui) { // && ms->config.objectMapViewerWindow->isVisible()) {
      renderObjectMapView(left_arm, right_arm);
    }
  }
}

void MachineState::publishConsoleMessage(string msg) {
  MachineState * ms = this;
  EinConsole consoleMsg;
  consoleMsg.msg = msg;
  ms->config.einConsolePub.publish(consoleMsg);
  ROS_INFO_STREAM("Console: " << msg);
}



int renderInit(MachineState * ms,  Camera * camera) {

  ms->config.renderInit = 1;
  
  ms->config.shouldIRender = ms->config.shouldIRenderDefault;
  
  ms->config.wristCamInit = 1;

  ms->config.faceViewImage = camera->cam_bgr_img.clone();
  ms->config.wristViewImage = camera->cam_bgr_img.clone();
  cout << "Camera Image: " << camera->cam_bgr_img.rows << ", " << camera->cam_bgr_img.cols << endl;
  ms->config.accumulatedImage = Mat(camera->cam_bgr_img.rows, camera->cam_bgr_img.cols, CV_64FC3);
  ms->config.accumulatedImageMass = Mat(camera->cam_bgr_img.rows, camera->cam_bgr_img.cols, CV_64F);
  
  ms->config.densityViewerImage = camera->cam_bgr_img.clone();
  ms->config.densityViewerImage *= 0;
  ms->config.objectViewerImage = camera->cam_bgr_img.clone();
  
  
  int imW = camera->cam_bgr_img.cols;
  int imH = camera->cam_bgr_img.rows;
  
  // determine table edges, i.e. the gray boxes
  ms->config.lGO = ms->config.gBoxW*(ms->config.lGO/ms->config.gBoxW);
  ms->config.rGO = ms->config.gBoxW*(ms->config.rGO/ms->config.gBoxW);
  ms->config.tGO = ms->config.gBoxH*(ms->config.tGO/ms->config.gBoxH);
  ms->config.bGO = ms->config.gBoxH*(ms->config.bGO/ms->config.gBoxH);
  
  
  ms->config.grayTop = cv::Point(ms->config.lGO, ms->config.tGO);
  ms->config.grayBot = cv::Point(imW-ms->config.rGO-1, imH-ms->config.bGO-1);
  
  if (ms->config.all_range_mode) {
    ms->config.grayTop = ms->config.armTop;
    ms->config.grayBot = ms->config.armBot;
  }
  
  if (ms->config.integralDensity != NULL) {
    delete ms->config.integralDensity;
  }
  ms->config.integralDensity = new double[imW*imH];


  if (ms->config.density != NULL) {
    delete ms->config.density;
  }
  ms->config.density = new double[imW*imH];


  if (ms->config.preDensity != NULL) {
    delete ms->config.preDensity;
  }
  ms->config.preDensity = new double[imW*imH];

  if (ms->config.temporalDensity != NULL) {
    delete ms->config.temporalDensity;
  }

  ms->config.temporalDensity = new double[imW*imH];
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      ms->config.temporalDensity[y*imW + x] = 0;
    }
  }

  initializeViewers(ms);

  return 0;
}

void saveConfig(MachineState * ms, string outFileName) {
  CONSOLE(ms, "Saving config file from " << outFileName);
  ros::Time savedTime = ros::Time::now();

  FileStorage fsvO;

  fsvO.open(outFileName, FileStorage::WRITE);

  if (! fsvO.isOpened()) {
    CONSOLE_ERROR(ms, "Couldn't open config file " << outFileName);
    return;
  }

  fsvO << "savedTime" << "[" 
    << savedTime.toSec() 
  << "]";

  fsvO << "currentTableZ" << "[" 
    << ms->config.currentTableZ 
  << "]";

  fsvO.release();

}

void loadConfig(MachineState * ms, string filename) {

  CONSOLE(ms, "Loading config file from " << filename);
  FileStorage fsvI;
  fsvI.open(filename, FileStorage::READ);

  if (!fsvI.isOpened()) {
    CONSOLE_ERROR(ms, "Couldn't open config file " << filename);
    return;
  }

  FileNode anode = fsvI["currentTableZ"];
  FileNodeIterator it = anode.begin(), it_end = anode.end();
  ms->config.currentTableZ = *(it++);

  
}


void accumulateImage(MachineState * ms) {
  Size sz = ms->config.accumulatedImage.size();
  int imW = sz.width;
  int imH = sz.height;
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  for (int y = 0; y < imH; y++) {

    cv::Vec3d* pixel = ms->config.accumulatedImage.ptr<cv::Vec3d>(y); // point to first pixel in row

    cv::Vec3b* wpixel = camera->cam_bgr_img.ptr<cv::Vec3b>(y); // point to first pixel in row
    double * mass = ms->config.accumulatedImageMass.ptr<double>(y);
    for (int x = 0; x < imW; x++) {
      pixel[x][0] += wpixel[x][0];
      pixel[x][1] += wpixel[x][1];
      pixel[x][2] += wpixel[x][2];
      mass[x] += 1.0;
    }
  }
}

void renderWristViewImage(MachineState * ms) {

  // paint gripper reticle centerline
  if (1) {
    eePose teePose;
    teePose.px = ms->config.trueEEPose.position.x;
    teePose.py = ms->config.trueEEPose.position.y;
    teePose.pz = ms->config.trueEEPose.position.z;
    cv::Scalar theColor(192, 64, 64);
    cv::Scalar THEcOLOR(64, 192, 192);
    
    double zStart = ms->config.minHeight;
    double zEnd = ms->config.maxHeight; 
    double deltaZ = 0.005;
    
    for (double zCounter = zStart; zCounter < zEnd; zCounter += deltaZ) {
      int pX = 0, pY = 0;  
      double zToUse = zCounter;
      
      globalToPixel(ms, &pX, &pY, zToUse, teePose.px, teePose.py);
      Point pt1(pX, pY);
      
      zToUse = zCounter+deltaZ;
      globalToPixel(ms, &pX, &pY, zToUse, teePose.px, teePose.py);
      Point pt2(pX, pY);
      
      line(ms->config.wristViewImage, pt1, pt2, theColor, 3);
    }
    for (double zCounter = zStart; zCounter < zEnd; zCounter += deltaZ) {
      int pX = 0, pY = 0;  
      double zToUse = zCounter;
      
      globalToPixel(ms, &pX, &pY, zToUse, teePose.px, teePose.py);
      Point pt1(pX, pY);
      
      zToUse = zCounter+deltaZ;
      globalToPixel(ms, &pX, &pY, zToUse, teePose.px, teePose.py);
      Point pt2(pX, pY);
      
      line(ms->config.wristViewImage, pt1, pt2, THEcOLOR, 1);
    }
    
  }
  
  // paint transform reticles
  if (ms->config.paintEEandReg1OnWrist) {
    paintEEPoseOnWrist(ms, ms->config.trueEEPoseEEPose, cv::Scalar(0,0,255));
    paintEEPoseOnWrist(ms, ms->config.eepReg1, cv::Scalar(0,255,0));
    
    paintEEPoseOnWrist(ms, ms->config.currentEEPose, cv::Scalar(255,255,0));
    
    {
      eePose irPose;
      {
        Camera * camera  = ms->config.cameras[ms->config.focused_camera];

	Eigen::Quaternionf crane2quat(ms->config.straightDown.qw, ms->config.straightDown.qx, ms->config.straightDown.qy, ms->config.straightDown.qz);
        Eigen::Quaternionf irpos = crane2quat.conjugate() * camera->gear0offset * crane2quat;
        ms->config.irGlobalPositionEEFrame[0] = irpos.w();
        ms->config.irGlobalPositionEEFrame[1] = irpos.x();
        ms->config.irGlobalPositionEEFrame[2] = irpos.y();
        ms->config.irGlobalPositionEEFrame[3] = irpos.z();

	geometry_msgs::Pose thisPose = ms->config.trueEEPose;
	Eigen::Quaternionf ceeQuat(thisPose.orientation.w, thisPose.orientation.x, thisPose.orientation.y, thisPose.orientation.z);
	Eigen::Quaternionf irSensorStartLocal = ceeQuat * irpos * ceeQuat.conjugate();
	Eigen::Quaternionf irSensorStartGlobal(
                                               0.0,
					       (thisPose.position.x - irSensorStartLocal.x()),
					       (thisPose.position.y - irSensorStartLocal.y()),
					       (thisPose.position.z - irSensorStartLocal.z())
                                               );
        
	Eigen::Quaternionf globalUnitZ(0, 0, 0, 1);
	Eigen::Quaternionf localUnitZ = ceeQuat * globalUnitZ * ceeQuat.conjugate();

	Eigen::Vector3d irSensorEnd(
				     (thisPose.position.x - irSensorStartLocal.x()) + ms->config.eeRange*localUnitZ.x(),
				     (thisPose.position.y - irSensorStartLocal.y()) + ms->config.eeRange*localUnitZ.y(),
				     (thisPose.position.z - irSensorStartLocal.z()) + ms->config.eeRange*localUnitZ.z()
				    );
	irPose.px = irSensorEnd.x();
	irPose.py = irSensorEnd.y();
	irPose.pz = irSensorEnd.z();
      }
      if (fabs(ms->config.eeRange - ms->config.eeRangeMaxValue) > 0.0001) {
        paintEEPoseOnWrist(ms, irPose, cv::Scalar(255,0,0));
      }
    }
  }

  // draw vanishing point reticle
  {
    int vanishingPointReticleRadius = 15;
    Camera * camera  = ms->config.cameras[ms->config.focused_camera];

    int x0 = camera->vanishingPointReticle.px;
    int y0 = camera->vanishingPointReticle.py;
    Point pt1(x0, y0);

    cv::Scalar theColor(192, 64, 64);
    cv::Scalar THEcOLOR(64, 192, 192);

    circle(ms->config.wristViewImage, pt1, vanishingPointReticleRadius, theColor, 1);
    circle(ms->config.wristViewImage, pt1, vanishingPointReticleRadius+1, THEcOLOR, 1);
    circle(ms->config.wristViewImage, pt1, vanishingPointReticleRadius+3, theColor, 1);
  }

  // draw ms->config.currentMovementState indicator
  {
    // XXX TODO this should be guarded 
    int movementIndicatorInnerHalfWidth = 7;
    int movementIndicatorOuterHalfWidth = 10;
    Camera * camera  = ms->config.cameras[ms->config.focused_camera];

    int x0 = camera->vanishingPointReticle.px;
    int y0 = camera->vanishingPointReticle.py+3*movementIndicatorOuterHalfWidth;
    Point pt1(x0, y0);
    Mat innerCrop = ms->config.wristViewImage(cv::Rect(pt1.x-movementIndicatorInnerHalfWidth, pt1.y-movementIndicatorInnerHalfWidth, 
					  2*movementIndicatorInnerHalfWidth, 2*movementIndicatorInnerHalfWidth) );
    Mat outerCrop = ms->config.wristViewImage(cv::Rect(pt1.x-movementIndicatorOuterHalfWidth, pt1.y-movementIndicatorOuterHalfWidth, 
					  2*movementIndicatorOuterHalfWidth, 2*movementIndicatorOuterHalfWidth) );
    int icMag = 64;
    Scalar indicatorColor = CV_RGB(icMag,icMag,icMag);
    if (ms->config.currentMovementState == STOPPED) {
      indicatorColor = CV_RGB(icMag,0,0); 
    } else if (ms->config.currentMovementState == HOVERING) {
      indicatorColor = CV_RGB(0,0,icMag); 
    } else if (ms->config.currentMovementState == MOVING) {
      indicatorColor = CV_RGB(0,icMag,0); 
    } else if (ms->config.currentMovementState == BLOCKED) {
      indicatorColor = CV_RGB(icMag,0,0); 
    } else if (ms->config.currentMovementState == ARMED) {
      indicatorColor = CV_RGB(icMag,0,0); 
    }
    outerCrop += indicatorColor;
    
    if (ms->config.currentMovementState == STOPPED) {
      indicatorColor = CV_RGB(icMag,2*icMag,0); 
    } else if (ms->config.currentMovementState == ARMED) {
      indicatorColor = CV_RGB(0,0,2*icMag); 
    }
    innerCrop += indicatorColor;
  }

  // draw probe reticle
  {
    Camera * camera  = ms->config.cameras[ms->config.focused_camera];

    int probeReticleHalfWidth = 7;
    int x0 = camera->probeReticle.px;
    int y0 = camera->probeReticle.py;

    int x1 = max(int(camera->probeReticle.px-probeReticleHalfWidth), 0);
    int x2 = min(int(camera->probeReticle.px+probeReticleHalfWidth), ms->config.wristViewImage.cols);
    int y1 = max(int(camera->probeReticle.py-probeReticleHalfWidth), 0);
    int y2 = min(int(camera->probeReticle.py+probeReticleHalfWidth), ms->config.wristViewImage.rows);

    int probeReticleShortHalfWidth = 3;
    int x1s = max(int(camera->probeReticle.px-probeReticleShortHalfWidth), 0);
    int x2s = min(int(camera->probeReticle.px+probeReticleShortHalfWidth), ms->config.wristViewImage.cols);
    int y1s = max(int(camera->probeReticle.py-probeReticleShortHalfWidth), 0);
    int y2s = min(int(camera->probeReticle.py+probeReticleShortHalfWidth), ms->config.wristViewImage.rows);

    cv::Scalar theColor(255, 0, 0);
    cv::Scalar THEcOLOR(0, 255, 255);
    {
      int xs = x0;
      int xf = x0;
      int ys = y1;
      int yf = y1s;
      {
	Point pt1(xs, ys);
	Point pt2(xf, yf);
	line(ms->config.wristViewImage, pt1, pt2, theColor, 2.0);
      }
      {
	Point pt1(xs, ys+1);
	Point pt2(xf, yf-1);
	line(ms->config.wristViewImage, pt1, pt2, THEcOLOR, 1.0);
      }
    }
    {
      int xs = x0;
      int xf = x0;
      int ys = y2s;
      int yf = y2;
      {
	Point pt1(xs, ys);
	Point pt2(xf, yf);
	line(ms->config.wristViewImage, pt1, pt2, theColor, 2.0);
      }
      {
	Point pt1(xs, ys+1);
	Point pt2(xf, yf-1);
	line(ms->config.wristViewImage, pt1, pt2, THEcOLOR, 1.0);
      }
    }
    {
      int xs = x1;
      int xf = x1s;
      int ys = y0;
      int yf = y0;
      {
	Point pt1(xs, ys);
	Point pt2(xf, yf);
	line(ms->config.wristViewImage, pt1, pt2, theColor, 2.0);
      }
      {
	Point pt1(xs+1, ys);
	Point pt2(xf-1, yf);
	line(ms->config.wristViewImage, pt1, pt2, THEcOLOR, 1.0);
      }
    }
    {
      int xs = x2s;
      int xf = x2;
      int ys = y0;
      int yf = y0;
      {
	Point pt1(xs, ys);
	Point pt2(xf, yf);
	line(ms->config.wristViewImage, pt1, pt2, theColor, 2.0);
      }
      {
	Point pt1(xs+1, ys);
	Point pt2(xf-1, yf);
	line(ms->config.wristViewImage, pt1, pt2, THEcOLOR, 1.0);
      }
    }
  
  }

  // draw color reticle
  /*
  {
    for (int cr = 0; cr < camera->numCReticleIndexes; cr++) {
      cv::Point outTop = cv::Point(camera->xCR[cr]-3, camera->yCR[cr]-3);
      cv::Point outBot = cv::Point(camera->xCR[cr]+3, camera->yCR[cr]+3);
      cv::Point inTop = cv::Point(outTop.x+1, outTop.y+1);
      cv::Point inBot = cv::Point(outBot.x-1, outBot.y-1);
      rectangle(ms->config.wristViewImage, outTop, outBot, cv::Scalar(0,192,0)); 
      rectangle(ms->config.wristViewImage, inTop, inBot, cv::Scalar(0,64,0)); 
    }
    {
      int tcrx = getColorReticleX(ms);
      int tcry = getColorReticleY(ms);
      cv::Point outTop = cv::Point(tcrx-5, tcry-5);
      cv::Point outBot = cv::Point(tcrx+5, tcry+5);
      cv::Point inTop = cv::Point(outTop.x+1, outTop.y+1);
      cv::Point inBot = cv::Point(outBot.x-1, outBot.y-1);
      rectangle(ms->config.wristViewImage, outTop, outBot, cv::Scalar(227,104,193)); 
      rectangle(ms->config.wristViewImage, inTop, inBot, cv::Scalar(133,104,109)); 
    }
    }*/

  // ATTN 16
  if (1) {
    for (int hri = 0; hri < 4; hri++) {
      if (hri != ms->config.currentThompsonHeightIdx)
	continue;
      Camera * camera  = ms->config.cameras[ms->config.focused_camera];
      eePose thisReticle = camera->heightReticles[hri];
      int param_reticleHalfWidth = 18;
      int thisReticleHalfWidth = int(  ceil( double(param_reticleHalfWidth) / double(1+hri) )  );
      cv::Point outTop = cv::Point(thisReticle.px-thisReticleHalfWidth, thisReticle.py-thisReticleHalfWidth);
      cv::Point outBot = cv::Point(thisReticle.px+thisReticleHalfWidth, thisReticle.py+thisReticleHalfWidth);
      cv::Point inTop = cv::Point(thisReticle.px+1-thisReticleHalfWidth,thisReticle.py+1-thisReticleHalfWidth);
      cv::Point inBot = cv::Point(thisReticle.px-1+thisReticleHalfWidth,thisReticle.py-1+thisReticleHalfWidth);

      if (hri == ms->config.currentThompsonHeightIdx) {
	rectangle(ms->config.wristViewImage, outTop, outBot, cv::Scalar(22,70,82)); 
	rectangle(ms->config.wristViewImage, inTop, inBot, cv::Scalar(68,205,239));
      } else {
	rectangle(ms->config.wristViewImage, outTop, outBot, cv::Scalar(82,70,22)); // RGB: 22 70 82
	rectangle(ms->config.wristViewImage, inTop, inBot, cv::Scalar(239,205,68)); // RGB: 68 205 239
      }
    }
  }

  {

    Size sz = ms->config.wristViewImage.size();
    int imW = sz.width;
    int imH = sz.height;
    
    int param_pilotTargetHalfWidth = 15;
    cv::Point outTop = cv::Point(ms->config.pilotTarget.px-param_pilotTargetHalfWidth, ms->config.pilotTarget.py-param_pilotTargetHalfWidth);
    cv::Point outBot = cv::Point(ms->config.pilotTarget.px+param_pilotTargetHalfWidth, ms->config.pilotTarget.py+param_pilotTargetHalfWidth);
    cv::Point inTop = cv::Point(ms->config.pilotTarget.px+1-param_pilotTargetHalfWidth,ms->config.pilotTarget.py+1-param_pilotTargetHalfWidth);
    cv::Point inBot = cv::Point(ms->config.pilotTarget.px-1+param_pilotTargetHalfWidth,ms->config.pilotTarget.py-1+param_pilotTargetHalfWidth);
    if ( (outTop.x > 0) && (outTop.y > 0) && (outBot.x < imW) && (outBot.y < imH) ) {
      rectangle(ms->config.wristViewImage, outTop, outBot, cv::Scalar(53,10,97)); // RGB: 97 10 53
      rectangle(ms->config.wristViewImage, inTop, inBot, cv::Scalar(142,31,255)); // RGB: 255 31 142
    }
  }
  if (ms->config.mask_gripper) {
    Camera * camera  = ms->config.cameras[ms->config.focused_camera];
    for (int y = 0; y < camera->gripperMask.rows; y++) {
      uchar* gripperMaskPixel = camera->gripperMask.ptr<uchar>(y); // point to first pixel in row
      for (int x = 0; x < camera->gripperMask.cols; x++) {
        if (gripperMaskPixel[x] == 0) {
          ms->config.wristViewImage.at<Vec3b>(y,x)[0] = 255;
	}
      }
    }
  }
}

void MachineState::imageCallback(Camera * camera) {
  if (camera != ms->config.cameras[ms->config.focused_camera]) {
    return;
  }

  if (!ms->config.renderInit) {
    if (renderInit(ms, camera) == -1) {
      ROS_ERROR("Couldn't initialize rendering system.");
      return;
    }
  }

  if (ms->config.castRecentRangeRay) {
    recordReadyRangeReadings(ms);
  }

  ms->config.wristCamImage = camera->cam_img.clone();
  ms->config.wristCamInit = 1;

  ms->config.wristViewImage = camera->cam_bgr_img.clone();

  ms->config.faceViewImage = camera->cam_bgr_img.clone();

  accumulateImage(ms);

  renderWristViewImage(ms);
  if (ms->config.shouldIRender && ms->config.showgui) {
    //QMetaObject::invokeMethod(qtTestWindow, "updateImage", Qt::QueuedConnection, Q_ARG(Mat, (Mat) ms->config.wristViewImage));
    //QMetaObject::invokeMethod(ms-.config.wristViewWindow, "updateImage", Qt::QueuedConnection, Q_ARG(Mat, (Mat) ms->config.wristViewImage));
    ms->config.wristViewWindow->updateImage(camera->cam_img);

    if (ms->config.wristViewBrightnessScalar == 1.0) {
      ms->config.wristViewWindow->updateImage(camera->cam_img);
    } else {
      Mat scaledWristViewImage = ms->config.wristViewBrightnessScalar * camera->cam_img;
      ms->config.wristViewWindow->updateImage(scaledWristViewImage);
    }
    //Mat firstYCBCR;  cvtColor(ms->config.wristViewImage, firstYCBCR, CV_BGR2YCrCb);
    //ms->config.wristViewWindow->updateImage(firstYCBCR);
  }
}


cv::Point worldToMapPixel(Mat mapImage, double xMin, double xMax, double yMin, double yMax, double x, double y) {
  double pxMin = 0;
  double pxMax = mapImage.rows;
  double pyMin = 0;
  double pyMax = mapImage.cols;
  cv::Point center = cv::Point(pxMax/2, pyMax/2);

  cv::Point out = cv::Point((pyMax - pyMin) / (yMax - yMin) * y + center.y,
                            (pxMax - pxMin) / (xMax - xMin) * x + center.x);

  return out;
}



void mapPixelToWorld(Mat mapImage, double xMin, double xMax, double yMin, double yMax, int px, int py, double &x, double &y) {
  double pxMin = 0;
  double pxMax = mapImage.rows;
  double pyMin = 0;
  double pyMax = mapImage.cols;
  cv::Point center = cv::Point(pxMax/2, pyMax/2);

  y = (px - center.y) * (yMax - yMin) / (pyMax - pyMin);
  x = (py - center.x) * (xMax - xMin) / (pxMax - pxMin);
}


void renderObjectMapView(MachineState * leftArm, MachineState * rightArm) {
  if (leftArm != NULL && leftArm->config.objectMapViewerImage.rows <= 0 ) {
    //ms->config.objectMapViewerImage = Mat(600, 600, CV_8UC3);
    //ms->config.objectMapViewerImage = Mat(400, 400, CV_8UC3);
    leftArm->config.objectMapViewerImage = Mat(300, 480, CV_8UC3);

    if (rightArm != NULL) {
      rightArm->config.objectMapViewerImage = leftArm->config.objectMapViewerImage;
    }
  } else if (rightArm != NULL && rightArm->config.objectMapViewerImage.rows <= 0 ) {
    //ms->config.objectMapViewerImage = Mat(600, 600, CV_8UC3);
    //ms->config.objectMapViewerImage = Mat(400, 400, CV_8UC3);
    rightArm->config.objectMapViewerImage = Mat(300, 480, CV_8UC3);
    if (leftArm != NULL) {
      leftArm->config.objectMapViewerImage = rightArm->config.objectMapViewerImage;
    }
  } else {
    // no need to recreate the image
  }

  if (leftArm != NULL) {
    leftArm->config.objectMapViewerImage = CV_RGB(0, 0, 0);
  } else if (rightArm != NULL) {
    rightArm->config.objectMapViewerImage = CV_RGB(0, 0, 0);
  } else {
    assert(0);
  }

  renderObjectMapViewOneArm(leftArm);
  renderObjectMapViewOneArm(rightArm);
}

void renderObjectMapViewOneArm(MachineState * ms) {
  if (ms == NULL) {
    return;
  }
  double pxMin = 0;
  double pxMax = ms->config.objectMapViewerImage.cols;
  double pyMin = 0;
  double pyMax = ms->config.objectMapViewerImage.rows;

  cv::Point center = cv::Point(pxMax/2, pyMax/2);

  double fadeBias = 0.50;
  double fadeLast = 30.0;

  for (int i = 0; i < ms->config.mapWidth; i++) {
    for (int j = 0; j < ms->config.mapHeight; j++) {

      //ros::Duration longAgo = ros::Time::now() - ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime;
      double longAgoSec = ros::Time::now().sec - ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime.sec; // faster than above
      double fadeFraction = (1.0-fadeBias)*(1.0-(min(max(longAgoSec, 0.0), fadeLast) / fadeLast)) + fadeBias;
      fadeFraction = min(max(fadeFraction, 0.0), 1.0);
      if (!ms->config.useGlow) {
	fadeFraction = 1.0;
      }
      if (!ms->config.useFade) {
	fadeFraction = 1.0;
      }

      double x, y;
      mapijToxy(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, i, j, &x, &y);

      if (ms->config.objectMap[i + ms->config.mapWidth * j].detectedClass != -1) {
        cv::Point outTop = worldToMapPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, x, y);
        cv::Point outBot = worldToMapPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, x + ms->config.mapStep, y + ms->config.mapStep);
        cv::Scalar color = CV_RGB((int) (ms->config.objectMap[i + ms->config.mapWidth * j].r / ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount),
                                  (int) (ms->config.objectMap[i + ms->config.mapWidth * j].g / ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount),
                                  (int) (ms->config.objectMap[i + ms->config.mapWidth * j].b / ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount) );
	color = color*fadeFraction;
        rectangle(ms->config.objectMapViewerImage, outTop, outBot, 
                  color,
                  CV_FILLED);
      }
    }
  }


  double glowBias = 0.15;
  double glowLast = 30.0;
  ros::Time now = ros::Time::now();
  if (ms->config.drawIKMap || ms->config.drawClearanceMap) { // draw ikMap and clearance map
    int ikMapRenderStride = 1;
    for (int i = 0; i < ms->config.mapWidth; i+=ikMapRenderStride) {
      for (int j = 0; j < ms->config.mapHeight; j+=ikMapRenderStride) {
	if ( cellIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, 
                                ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, i, j) ) {
          //ros::Duration longAgo = ros::Time::now() - ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime;
          double longAgoSec = now.sec - ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime.sec; // faster than above
          double glowFraction = (1.0-glowBias)*(1.0-(min(max(longAgoSec, 0.0), glowLast) / glowLast)) + glowBias;
          glowFraction = min(max(glowFraction, 0.0), 1.0);
          if (!ms->config.useGlow) {
            glowFraction = 1.0;
          }
          double x=-1, y=-1;
          mapijToxy(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, i, j, &x, &y);
          cv::Point cvp1 = worldToMapPixel(ms->config.objectMapViewerImage, 
                                           ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, x, y);
          
          if (ms->config.drawIKMap) { // draw ikMap 
            if ( (ms->config.ikMap[i + ms->config.mapWidth * j] == IK_GOOD) ) {
              // do not draw when it's good
            } else if ( (ms->config.ikMap[i + ms->config.mapWidth * j] == IK_FAILED) ) {
              Scalar tColor = CV_RGB(192, 32, 32);
              cv::Vec3b cColor;
              cColor[0] = tColor[0]*glowFraction;
              cColor[1] = tColor[1]*glowFraction;
              cColor[2] = tColor[2]*glowFraction;
              //gsl_matrix * mapcell = mapCellToPolygon(ms, i, j);
              //drawMapPolygon(mapcell, tColor);
              //gsl_matrix_free(mapcell);
              //line(ms->config.objectMapViewerImage, cvp1, cvp1, tColor);
              ms->config.objectMapViewerImage.at<cv::Vec3b>(cvp1.y, cvp1.x) = 
                ms->config.objectMapViewerImage.at<cv::Vec3b>(cvp1.y, cvp1.x) + cColor;
            } else if ( (ms->config.ikMap[i + ms->config.mapWidth * j] == IK_LIKELY_IN_COLLISION) ) {
              Scalar tColor = CV_RGB(224, 64, 64);
              cv::Vec3b cColor;
              cColor[0] = tColor[0]*glowFraction;
              cColor[1] = tColor[1]*glowFraction;
              cColor[2] = tColor[2]*glowFraction;
              //gsl_matrix * mapcell = mapCellToPolygon(ms, i, j);
              //drawMapPolygon(mapcell, tColor);
              //gsl_matrix_free(mapcell);
              //line(ms->config.objectMapViewerImage, cvp1, cvp1, tColor);
              ms->config.objectMapViewerImage.at<cv::Vec3b>(cvp1.y, cvp1.x) = 
                ms->config.objectMapViewerImage.at<cv::Vec3b>(cvp1.y, cvp1.x) + cColor;
            } else {
              cout << "Bad IK value: " << ms->config.ikMap[i + ms->config.mapWidth * j] << endl;
              assert(0);
            }
          }
          if (ms->config.drawClearanceMap) { // draw clearanceMap 
            if ( (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 0 ) ) {
              // do not draw
            } else if ( (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 1) ) {
              Scalar tColor = CV_RGB(224, 224, 0);
              cv::Vec3b cColor;
              cColor[0] = tColor[0]*glowFraction;
              cColor[1] = tColor[1]*glowFraction;
              cColor[2] = tColor[2]*glowFraction;
              //gsl_matrix * mapcell = mapCellToPolygon(ms, i, j);
              //drawMapPolygon(mapcell, CV_RGB(128, 128, 0));
              //gsl_matrix_free(mapcell);
              //line(ms->config.objectMapViewerImage, cvp1, cvp1, tColor);
              ms->config.objectMapViewerImage.at<cv::Vec3b>(cvp1.y, cvp1.x) = 
                  ms->config.objectMapViewerImage.at<cv::Vec3b>(cvp1.y, cvp1.x) + cColor;
            } else if ( (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 2) ) {
              Scalar tColor = CV_RGB(0, 224, 0);
              cv::Vec3b cColor;
              cColor[0] = tColor[0]*glowFraction;
              cColor[1] = tColor[1]*glowFraction;
              cColor[2] = tColor[2]*glowFraction;
              //gsl_matrix * mapcell = mapCellToPolygon(ms, i, j);
              //drawMapPolygon(mapcell, CV_RGB(32, 128, 32));
              //gsl_matrix_free(mapcell);
              //line(ms->config.objectMapViewerImage, cvp1, cvp1, tColor);
              ms->config.objectMapViewerImage.at<cv::Vec3b>(cvp1.y, cvp1.x) = 
                ms->config.objectMapViewerImage.at<cv::Vec3b>(cvp1.y, cvp1.x) + cColor;
            } else  {
                cout << "Bad clearance value: " << ms->config.clearanceMap[i + ms->config.mapWidth * j] << endl;
                assert(0);
            }
          }
	}
      }
    }
  }
  

  { // drawMapSearchFence
    
    cv::Point outTop = worldToMapPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
                                    ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceYMin);
    cv::Point outBot = worldToMapPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
                                    ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMax);

    rectangle(ms->config.objectMapViewerImage, outTop, outBot, 
              CV_RGB(255, 255, 0));
  }

  { // drawMapRejectFence
    
    cv::Point outTop = worldToMapPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
                                    ms->config.mapRejectFenceXMin, ms->config.mapRejectFenceYMin);
    cv::Point outBot = worldToMapPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
                                    ms->config.mapRejectFenceXMax, ms->config.mapRejectFenceYMax);

    rectangle(ms->config.objectMapViewerImage, outTop, outBot, 
              CV_RGB(255, 0, 0));
  }

  // draw sprites
  if (ms->config.currentRobotMode == SIMULATED) {
    for (int s = 0; s < ms->config.instanceSprites.size(); s++) {
      Sprite sprite = ms->config.instanceSprites[s];
      
      double cx, cy;
      
      cx = sprite.pose.px;
      cy = sprite.pose.py;
      
      cv::Point objectPoint = worldToMapPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
					   cx, cy);
      objectPoint.x += 15;

      cv::Point outTop = worldToMapPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
				      sprite.top.px, sprite.top.py);
      cv::Point outBot = worldToMapPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
				      sprite.bot.px, sprite.bot.py);

      int halfHeight = (outBot.y - outTop.y)/2;
      int halfWidth = (outBot.x - outTop.x)/2;
      if ((halfHeight < 0) || (halfWidth < 0)) {
	// really either both or neither should be true
	cv::Point tmp = outTop;
	outTop = outBot;
	outBot = tmp;
	halfHeight = (outBot.y - outTop.y)/2;
	halfWidth = (outBot.x - outTop.x)/2;
      }

      rectangle(ms->config.objectMapViewerImage, outTop, outBot, 
		CV_RGB(0, 255, 0));

      putText(ms->config.objectMapViewerImage, sprite.name, objectPoint, MY_FONT, 0.5, CV_RGB(196, 255, 196), 2.0);
    }
  }
  // draw blue boxes
  for (int i = 0; i < ms->config.blueBoxMemories.size(); i++) {
    BoxMemory memory = ms->config.blueBoxMemories[i];
    string class_name = ms->config.classLabels[memory.labeledClassIndex];
    
    double cx, cy;
    
    cx = memory.centroid.px;
    cy = memory.centroid.py;
    
    cv::Point objectPoint = worldToMapPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
                                         cx, cy);
    objectPoint.x += 15;

    cv::Point outTop = worldToMapPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
                                    memory.top.px, memory.top.py);
    cv::Point outBot = worldToMapPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
                                    memory.bot.px, memory.bot.py);

    int halfHeight = (outBot.y - outTop.y)/2;
    int halfWidth = (outBot.x - outTop.x)/2;
    if ((halfHeight < 0) || (halfWidth < 0)) {
      // really either both or neither should be true
      cv::Point tmp = outTop;
      outTop = outBot;
      outBot = tmp;
      halfHeight = (outBot.y - outTop.y)/2;
      halfWidth = (outBot.x - outTop.x)/2;
    }

    rectangle(ms->config.objectMapViewerImage, outTop, outBot, 
              CV_RGB(0, 0, 255));

    if (memory.lockStatus == POSE_LOCK ||
        memory.lockStatus == POSE_REPORTED) {
      double lockRenderPeriod1 = 3.0;
      double lockRenderPeriod2 = 2.0;
      //ros::Duration timeSince = ros::Time::now() - memory.cameraTime;
      ros::Duration timeSince = now - ros::Time(0);

      double lockArg1 = 2.0 * 3.1415926 * timeSince.toSec() / lockRenderPeriod1;
      double lockArg2 = 2.0 * 3.1415926 * timeSince.toSec() / lockRenderPeriod2;
      cv::Point outTopLock;
      cv::Point outBotLock;
      
      int widthShim = floor((halfWidth)  * 0.5 * (1.0 + sin(lockArg1)));
      int heightShim = floor((halfHeight)  * 0.5 * (1.0 + sin(lockArg1)));
      widthShim = min(max(1,widthShim),halfWidth-1);
      heightShim = min(max(1,heightShim),halfHeight-1);

      outTopLock.x = outTop.x + widthShim;
      outTopLock.y = outTop.y + heightShim;
      outBotLock.x = outBot.x - widthShim;
      outBotLock.y = outBot.y - heightShim;

      int nonBlueAmount = 0.0;
      if (memory.lockStatus == POSE_REPORTED) {
	nonBlueAmount = floor(128 * 0.5 * (1.0 + cos(lockArg2+memory.cameraTime.sec)));
      }
      
      rectangle(ms->config.objectMapViewerImage, outTopLock, outBotLock, 
              CV_RGB(nonBlueAmount, nonBlueAmount, 128+nonBlueAmount));
    }

    putText(ms->config.objectMapViewerImage, class_name, objectPoint, MY_FONT, 0.5, CV_RGB(196, 196, 255), 2.0);
  }

  cv::Scalar color;
  if (ms->config.lastIkWasSuccessful) {
    color = cv::Scalar(255, 255, 255);
  } else {
    color = cv::Scalar(0, 0, 255);
  }

  
  { // drawRobot
    double radius = 20;
    cv::Point orientation_point = cv::Point(pxMax/2, pyMax/2 + radius);
    
    circle(ms->config.objectMapViewerImage, center, radius, color);
    line(ms->config.objectMapViewerImage, center, orientation_point, color);
  }
  { // drawHand
    eePose tp = rosPoseToEEPose(ms->config.trueEEPose);
    double radius = 10;
    cv::Point handPoint = worldToMapPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
                                       tp.px, tp.py);
    
    Eigen::Quaternionf handQuat(tp.qw, tp.qx, tp.qy, tp.qz);

    double rotated_magnitude = radius / sqrt(pow(pxMax - pxMin, 2) +  pow(pyMax - pyMin, 2)) * sqrt(pow(ms->config.mapXMax - ms->config.mapXMin, 2) + pow(ms->config.mapYMax - ms->config.mapYMin, 2));
    Eigen::Vector3f point(rotated_magnitude, 0, 0);
    Eigen::Vector3f rotated = handQuat * point;
    
    cv::Point orientation_point = worldToMapPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax,
                                               tp.px + rotated[0], 
                                               tp.py + rotated[1]);


    circle(ms->config.objectMapViewerImage, handPoint, radius, color);

    line(ms->config.objectMapViewerImage, handPoint, orientation_point, color);

  }

  if (0) { // drawBoxMemoryIntersectTests

    for (int i = 0; i < ms->config.mapWidth; i++) {
      for (int j = 0; j < ms->config.mapHeight; j++) {
        gsl_matrix * mapcell = mapCellToPolygon(ms, i, j);

        for (int b_i = 0; b_i < ms->config.blueBoxMemories.size(); b_i++) {
          BoxMemory b = ms->config.blueBoxMemories[b_i];
          gsl_matrix * poly = boxMemoryToPolygon(b);
          cv::Scalar color;
          if (boxMemoryIntersectsMapCell(ms, b, i, j)) {
            ros::Duration diff = ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime - b.cameraTime;
            cout << "box time: " << b.cameraTime << endl;
            cout << "cell time: " << ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime << endl;
            cout << "diff: " << diff << endl;
            if (diff < ros::Duration(2.0)) {
              drawMapPolygon(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
                             poly, color);
              drawMapPolygon(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
                             mapcell, CV_RGB(255, 255, 0));
            }
          }
     
          
          gsl_matrix_free(poly);
        }
        gsl_matrix_free(mapcell);
      }
    }
  }

  if (ms->config.shouldIRender && ms->config.showgui) {
    ms->config.objectMapViewerWindow->updateImage(ms->config.objectMapViewerImage);
  }


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



void drawMapPolygon(Mat mapImage, double mapXMin, double mapXMax, double mapYMin, double mapYMax, gsl_matrix * polygon_xy, cv::Scalar color) {
  for (size_t i = 0; i < polygon_xy->size2; i++) {
    int j = (i + 1) % polygon_xy->size2;
    gsl_vector_view p1 = gsl_matrix_column(polygon_xy, i);
    gsl_vector_view p2 = gsl_matrix_column(polygon_xy, j);
    double x1 = gsl_vector_get(&p1.vector, 0);
    double y1 = gsl_vector_get(&p1.vector, 1);


    double x2 = gsl_vector_get(&p2.vector, 0);
    double y2 = gsl_vector_get(&p2.vector, 1);
    double px1, px2, py1, py2;
    cv::Point cvp1 = worldToMapPixel(mapImage, mapXMin, mapXMax, mapYMin, mapYMax, 
                                  x1, y1);
    cv::Point cvp2 = worldToMapPixel(mapImage, mapXMin, mapXMax, mapYMin, mapYMax, 
                                  x2, y2);
    line(mapImage, cvp1, cvp2, color);
  }

}



void renderRangeogramView(MachineState * ms) {
 if ( (ms->config.rangeogramImage.rows > 0) && (ms->config.rangeogramImage.cols > 0) ) {
    cv::Point text_anchor = cv::Point(0,ms->config.rangeogramImage.rows-1);
    {
      cv::Scalar backColor(0,0,0);
      cv::Point outTop = cv::Point(text_anchor.x,text_anchor.y+1-35);
      cv::Point outBot = cv::Point(text_anchor.x+400,text_anchor.y+1);
      Mat vCrop = ms->config.rangeogramImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
      vCrop = backColor;
    }
    {
      char buff[256];
      sprintf(buff, "            Hz: %.2f", ms->config.aveFrequency);
      string fpslabel(buff);
      putText(ms->config.rangeogramImage, fpslabel, text_anchor, MY_FONT, 1.0, Scalar(160,0,0), 1.0);
    }
    {
      char buff[256];
      sprintf(buff, "Hz: %.2f", ms->config.aveFrequencyRange);
      string fpslabel(buff);
      putText(ms->config.rangeogramImage, fpslabel, text_anchor, MY_FONT, 1.0, Scalar(0,0,160), 1.0);
    }
  }
  ms->config.rangeogramWindow->updateImage(ms->config.rangeogramImage);
}

void MachineState::targetCallback(const geometry_msgs::Point& point) {

  MachineState * ms = this;

  if (!ms->config.shouldIMiscCallback) {
    return;
  }

}

void pilotCallbackFunc(int event, int x, int y, int flags, void* userdata) {
  MachineState * ms = ((MachineState *) userdata);
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  //if (!ms->config.shouldIMiscCallback) {
    //return;
  //}

  if ( event == EIN_EVENT_LBUTTONDOWN ) {
    cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    Camera * camera  = ms->config.cameras[ms->config.focused_camera];

    camera->probeReticle.px = x;
    camera->probeReticle.py = y;
    cout << "x: " << x << " y: " << y << " eeRange: " << ms->config.eeRange << endl;

    // form a rotation about the vanishing point, measured from positive x axis
    // window is inverted
    double thisTheta = vectorArcTan(ms, camera->vanishingPointReticle.py - y, x - camera->vanishingPointReticle.px);

    ms->pushWord("pixelServoA");
    ms->pushWord(std::make_shared<DoubleWord>(thisTheta));
    ms->pushWord(std::make_shared<IntegerWord>(camera->vanishingPointReticle.py));
    ms->pushWord(std::make_shared<IntegerWord>(camera->vanishingPointReticle.px));
    ms->execute_stack = 1;
  } else if ( event == EIN_EVENT_RBUTTONDOWN ) {
    //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    ms->pushWord("pixelServoA");
    ms->pushWord(std::make_shared<DoubleWord>(0));
    ms->pushWord(std::make_shared<IntegerWord>(y));
    ms->pushWord(std::make_shared<IntegerWord>(x));
    ms->execute_stack = 1;
  } else if  ( event == EIN_EVENT_MBUTTONDOWN ) {
    //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    ms->evaluateProgram("tenthImpulse ( zUp ) 10 replicateWord waitUntilAtCurrentPosition 1 changeToHeight");
    ms->pushWord("waitUntilGripperNotMoving");
    
    ms->pushWord("closeGripper");
    ms->pushWord("pressUntilEffortOrTwist");

    ms->pushWord("setTwistThresh");
    ms->pushWord("0.015");

    ms->pushWord("setEffortThresh");
    ms->pushWord("20.0");

    ms->pushWord("setSpeed");
    ms->pushWord("0.03");

    ms->pushWord("pressUntilEffortOrTwistInit");
    ms->pushWord("comeToStop");
    ms->pushWord("setMovementStateToMoving");
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition");

    ms->pushWord("setSpeed");
    ms->pushWord("0.05");

    ms->pushWord("setGridSizeCoarse");
    ms->pushWord("pixelServoPutVanishingPointUnderGripper");
    ms->execute_stack = 1;
  } else if ( event == EIN_EVENT_MOUSEMOVE ) {
    //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
  }
}

void mapCallbackFunc(int event, int x, int y, int flags, void* userdata) {
  MachineState * ms = ((MachineState *) userdata);
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  //if (!ms->config.shouldIMiscCallback) {
    //return;
  //}

  if ( event == EIN_EVENT_LBUTTONDOWN ) {
    cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

    eePose clickPoseInLocal = eePose::identity();
    ms->config.scene->cellToMeters(y, x, &clickPoseInLocal.px, &clickPoseInLocal.py);
    eePose clickPoseInBase = clickPoseInLocal.applyAsRelativePoseTo(ms->config.scene->anchor_pose);

    eePose clickPoseToPush = clickPoseInBase;
    clickPoseToPush.copyQ(ms->config.currentEEPose);
    clickPoseToPush.pz = ms->config.currentEEPose.pz;

    ms->pushData(make_shared<EePoseWord>(clickPoseToPush));

    ms->pushWord("assumePose");
    ms->execute_stack = 1;
  } else if ( event == EIN_EVENT_RBUTTONDOWN ) {
    cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if  ( event == EIN_EVENT_MBUTTONDOWN ) {
    cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if ( event == EIN_EVENT_MOUSEMOVE ) {
    cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
  }
}

void objectMapCallbackFunc(int event, int x, int y, int flags, void* userdata) {
  vector<MachineState * > machineStates = *((vector<MachineState * > *) userdata);
  for(int i = 0; i < machineStates.size(); i++) {
    doObjectMapCallbackFunc(event, x, y, flags, machineStates[i]);
  }

  

}
void doObjectMapCallbackFunc(int event, int x, int y, int flags, MachineState * ms) {

  

  if ( event == EIN_EVENT_LBUTTONDBLCLK ) {
    double worldX, worldY;
    mapPixelToWorld(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, x, y, worldX, worldY);
    ms->config.currentEEPose.px = worldX;  
    ms->config.currentEEPose.py = worldY;  


    for (int i = 0; i < ms->config.blueBoxMemories.size(); i++) {
      BoxMemory memory = ms->config.blueBoxMemories[i];
      string class_name = ms->config.classLabels[memory.labeledClassIndex];
       
      cv::Point outTop = worldToMapPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
				      memory.top.px, memory.top.py);
      cv::Point outBot = worldToMapPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
				      memory.bot.px, memory.bot.py);
      
      cout <<" Top: " << outTop.x << ", " << outTop.y << endl;
      cout <<" Bot: " << outBot.x << ", " << outBot.y << endl;
      if ((outBot.x <= x && x <= outTop.x) && 
	  (outBot.y <= y && y <= outTop.y)) {

	cout << "Got: " << memory.labeledClassIndex << ": " << ms->config.classLabels[memory.labeledClassIndex] << endl;
	targetBoxMemory(ms, i);
	ms->pushWord("deliverTargetBoxMemory");
	ms->execute_stack = 1;

	
      }
    }
  }
}


int shouldIPick(MachineState * ms, int classToPick) {

  int toReturn = 0;

  // 6 is black tape
  // 12 is green block
  // 2 is greenPepper 
  // 9 is chile
  // 10 is cucumber
//  if (
//      (classToPick == 2) ||
//      (classToPick == 9) ||
//      (classToPick == 10)||
//      (classToPick == 12)||
//      (classToPick == 6)
//    ) {
//    toReturn = 1;
//  }
  
  toReturn = (classToPick == ms->config.targetClass);

  cout << classToPick << " " << ms->config.targetClass << " " << toReturn;

  return toReturn;
}

void changeCamera(MachineState * ms, int newCamera) {
  ms->config.focused_camera = newCamera;
  ms->config.renderInit = 0;
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
}

void changeTargetClass(MachineState * ms, int newTargetClass) {

  if ( (newTargetClass < 0) || (newTargetClass >= ms->config.classLabels.size()) ) {
    CONSOLE_ERROR(ms, "changeTargetClass: tried to change to an invalid class. setting class to 0." << endl); 
    newTargetClass = 0;
    ms->config.targetClass = 0;
    ms->config.focusedClass = 0;
    return;
  } else {
  }

  ms->config.targetClass = newTargetClass;
  ms->config.focusedClass = ms->config.targetClass;
  ms->config.focusedClassLabel = ms->config.classLabels[ms->config.focusedClass];
  cout << "class " << ms->config.targetClass << " " << ms->config.classLabels[ms->config.targetClass] << endl;
  ms->execute_stack = 1;	

}


void guard3dGrasps(MachineState * ms) {
  if (ms->config.class3dGrasps.size() < ms->config.numClasses) {
    ms->config.class3dGrasps.resize(ms->config.numClasses);
    ms->config.classPlaceUnderPoints.resize(ms->config.numClasses);
    ms->config.classPlaceOverPoints.resize(ms->config.numClasses);
  }
}

void guardSceneModels(MachineState * ms) {
  if (ms->config.class_scene_models.size() < ms->config.numClasses) {
    ms->config.class_scene_models.resize(ms->config.numClasses);
  }
}


int calibrateGripper(MachineState * ms) {
  if (ms->config.currentRobotMode == SIMULATED) {
    return 0;
  } else if (ms->config.currentRobotMode == SNOOP) {
    return 0;
  } else if (ms->config.currentRobotMode == PHYSICAL) {
    for (int i = 0; i < 10; i++) {
      int return_value = doCalibrateGripper(ms);
      if (return_value == 0) {
	return return_value;
      }
    }
    CONSOLE_ERROR(ms, "Gripper could not calibrate!  Try running from the command line, because this often means your Baxter SDK is set up incorrectly. 'rosrun baxter_examples gripper_keyboard.py'");
    ms->pushWord("pauseStackExecution"); // pause stack execution
    ms->pushCopies("beep", 15); // beep
    return -1;
  } else {
    cout << "Bad mode: " << ms->config.currentRobotMode << endl;
    assert(0);
  }
}
int doCalibrateGripper(MachineState * ms) {
  int return_value;
  if (0 == ms->config.left_or_right_arm.compare("left")) {
    return_value =system("bash -c \"echo -e \'c\003\' | rosrun baxter_examples gripper_keyboard.py\"");
  } else if (0 == ms->config.left_or_right_arm.compare("right")) {
    return_value = system("bash -c \"echo -e \'C\003\' | rosrun baxter_examples gripper_keyboard.py\"");
  }
  if (return_value != 0) {
    cout << "Error running calibrate: " << return_value << endl;
  }
  return return_value;
}

double convertHeightIdxToGlobalZ(MachineState * ms, int heightIdx) {
  double tabledMaxHeight = ms->config.maxHeight - ms->config.currentTableZ;
  double tabledMinHeight = ms->config.minHeight - ms->config.currentTableZ;

  double scaledHeight = (double(heightIdx)/double(ms->config.hmWidth-1)) * (tabledMaxHeight - tabledMinHeight);
  double scaledTranslatedHeight = scaledHeight + tabledMinHeight;
  return scaledTranslatedHeight;
}

double convertHeightIdxToLocalZ(MachineState * ms, int heightIdx) {
  double unTabledMaxHeight = ms->config.maxHeight;
  double unTabledMinHeight = ms->config.minHeight;

  double scaledHeight = (double(heightIdx)/double(ms->config.hmWidth-1)) * (unTabledMaxHeight - unTabledMinHeight);
  double scaledTranslatedHeight = scaledHeight + unTabledMinHeight;
  return scaledTranslatedHeight;
}

void convertHeightGlobalZToIdx(MachineState * ms, double globalZ) {
  double tabledMaxHeight = ms->config.maxHeight - ms->config.currentTableZ;
  double tabledMinHeight = ms->config.minHeight - ms->config.currentTableZ;

  double scaledHeight = (globalZ - ms->config.currentTableZ) / (tabledMaxHeight - tabledMinHeight);
  int heightIdx = floor(scaledHeight * (ms->config.hmWidth - 1));
}

void testHeightConversion(MachineState * ms) {
  for (int i = 0; i < ms->config.hmWidth; i++) {
    double height = convertHeightIdxToGlobalZ(ms, i);
    convertHeightGlobalZToIdx(ms, height);
    cout << "i: " << i << " height: " << height << endl;
    //assert(newIdx == i);
  }
}


void moveCurrentGripperRayToCameraVanishingRay(MachineState * ms) {
  bool useLaser = 0;
  if (useLaser) {
    //double d_y = -0.04;
    //double d_x = 0.018;
    // this is really the right way of doing it... the question is,
    // which of the three ways gives the best result?
    double pTermY = -0.018;
    double pTermX = 0.04;
    Eigen::Vector3f localUnitX;
    {
      Eigen::Quaternionf qin(0, 1, 0, 0);
      Eigen::Quaternionf qout(0, 1, 0, 0);
      Eigen::Quaternionf eeqform(ms->config.trueEEPose.orientation.w, ms->config.trueEEPose.orientation.x, ms->config.trueEEPose.orientation.y, ms->config.trueEEPose.orientation.z);
      qout = eeqform * qin * eeqform.conjugate();
      localUnitX.x() = qout.x();
      localUnitX.y() = qout.y();
      localUnitX.z() = qout.z();
    }
      
    Eigen::Vector3f localUnitY;
    {
      Eigen::Quaternionf qin(0, 0, 1, 0);
      Eigen::Quaternionf qout(0, 1, 0, 0);
      Eigen::Quaternionf eeqform(ms->config.trueEEPose.orientation.w, ms->config.trueEEPose.orientation.x, ms->config.trueEEPose.orientation.y, ms->config.trueEEPose.orientation.z);
      qout = eeqform * qin * eeqform.conjugate();
      localUnitY.x() = qout.x();
      localUnitY.y() = qout.y();
      localUnitY.z() = qout.z();
    }
    cout << "moveCurrentGripperRayToCameraVanishingRay localUnitY localUnitX: " << localUnitY << " " << localUnitX << endl;
    cout << "moveCurrentGripperRayToCameraVanishingRay localUnitY localUnitX: " << localUnitY.norm() << " " << localUnitX.norm() << endl;
    double xToAdd = (pTermX*localUnitY.x() - pTermY*localUnitX.x());
    double yToAdd = (pTermX*localUnitY.y() - pTermY*localUnitX.y());
    cout << "moveCurrentGripperRayToCameraVanishingRay xToAdd yToAdd: " << xToAdd << " " << yToAdd << endl;
    ms->config.currentEEPose.px += xToAdd;
    ms->config.currentEEPose.py += yToAdd;
  } else {
    double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;
    Camera * camera  = ms->config.cameras[ms->config.focused_camera];
    pixelToGlobal(ms, camera->vanishingPointReticle.px, camera->vanishingPointReticle.py, zToUse, &(ms->config.currentEEPose.px), &(ms->config.currentEEPose.py));
  }
  { // yet another way to do this
    // 0 assumes no rotation 
    //eePose finalGlobalTarget = analyticServoPixelToReticle(ms->config.pilotTarget, thisGripperReticle, 0);
    //ms->config.currentEEPose.px = finalGlobalTarget.px;
    //ms->config.currentEEPose.py = finalGlobalTarget.py;
  }
}

Mat makeGCrop(MachineState * ms, int etaX, int etaY) {
  Size sz = ms->config.objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;

  int crows = ms->config.aerialGradientReticleWidth;
  int ccols = ms->config.aerialGradientReticleWidth;
  int maxDim = max(crows, ccols);
  int tRy = (maxDim-crows)/2;
  int tRx = (maxDim-ccols)/2;

  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  int topCornerX = etaX + camera->reticle.px - (ms->config.aerialGradientReticleWidth/2);
  int topCornerY = etaY + camera->reticle.py - (ms->config.aerialGradientReticleWidth/2);

  Mat gCrop(maxDim, maxDim, CV_64F);
  Size toBecome(ms->config.aerialGradientWidth, ms->config.aerialGradientWidth);


  
  for (int x = 0; x < maxDim; x++) {
    for (int y = 0; y < maxDim; y++) {
      int tx = x - tRx;
      int ty = y - tRy;
      int tCtx = topCornerX + tx;
      int tCty = topCornerY + ty;
      if ( (tx >= 0 && ty >= 0 && ty < crows && tx < ccols) &&
           (tCtx > 0) && (tCty > 0) && (tCtx < imW) && (tCty < imH) ) {
        gCrop.at<double>(y, x) = ms->config.frameGraySobel.at<double>(topCornerY + ty, topCornerX + tx);
      } else {
        gCrop.at<double>(y, x) = 0.0;
      }
    }
  }
  
  cv::resize(gCrop, gCrop, toBecome);
  
  processSaliency(gCrop, gCrop);
  
  {
    double mean = gCrop.dot(Mat::ones(ms->config.aerialGradientWidth, ms->config.aerialGradientWidth, gCrop.type())) / double(ms->config.aerialGradientWidth*ms->config.aerialGradientWidth);
    gCrop = gCrop - mean;
    double l2norm = gCrop.dot(gCrop);
    l2norm = sqrt(l2norm);
    // ATTN 17
    // removed normalization for discriminative servoing
    // ATTN 15
    // normalization hoses rejection
    if (l2norm <= EPSILON) {
      l2norm = 1.0;
    }
    gCrop = gCrop / l2norm;
  }
  return gCrop;
}


void prepareForCrossCorrelation(MachineState * ms, Mat input, Mat& output, int thisOrient, int numOrientations, double thisScale, Size toBecome) {
  Point center = Point(ms->config.aerialGradientWidth/2, ms->config.aerialGradientWidth/2);
  double angle = thisOrient*360.0/numOrientations;
  
  //double scale = 1.0;
  double scale = thisScale;

  // Get the rotation matrix with the specifications above
  Mat rot_mat = getRotationMatrix2D(center, angle, scale);
  warpAffine(input, output, rot_mat, toBecome);
  
  processSaliency(output, output);

  double mean = output.dot(Mat::ones(ms->config.aerialGradientWidth, ms->config.aerialGradientWidth, output.type())) / double(ms->config.aerialGradientWidth*ms->config.aerialGradientWidth);
  output = output - mean;
  double l2norm = output.dot(output);
  l2norm = sqrt(l2norm);
  if (l2norm <= EPSILON) {
    l2norm = 1.0;
  }
  output = output / l2norm;

}

void normalizeForCrossCorrelation(MachineState * ms, Mat input, Mat& output) {
  processSaliency(input, output);

  double mean = output.dot(Mat::ones(output.rows, output.cols, output.type())) / double(output.rows*output.cols);
  output = output - mean;
  double l2norm = output.dot(output);
  l2norm = sqrt(l2norm);
  if (l2norm <= EPSILON) {
    l2norm = 1.0;
  }
  output = output / l2norm;
}

double computeSimilarity(MachineState * ms, Mat im1, Mat im2) {

  vector<Mat> rotatedAerialGrads;
  int gradientServoScale = 1;//11;
  int numOrientations = 37;
  double gradientServoScaleStep = 1.02;
  int etaS = 0;
  Size toBecome(ms->config.aerialGradientWidth, ms->config.aerialGradientWidth);
  rotatedAerialGrads.resize(gradientServoScale*numOrientations);
  double startScale = pow(gradientServoScaleStep, -(gradientServoScale-1)/2);  
  double thisScale = startScale * pow(gradientServoScaleStep, etaS);

  Mat preparedClass1;
  {
    int thisOrient = 0;
    prepareForCrossCorrelation(ms, im1, preparedClass1, thisOrient, numOrientations, thisScale, toBecome);
  }

  double globalMax = 0.0;
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

    prepareForCrossCorrelation(ms, im2, rotatedAerialGrads[thisOrient + etaS*numOrientations], thisOrient, numOrientations, thisScale, toBecome);

    Mat output = preparedClass1.clone(); 
    filter2D(preparedClass1, output, -1, rotatedAerialGrads[thisOrient + etaS*numOrientations], Point(-1,-1), 0, BORDER_CONSTANT);
    double minValue, maxValue;
    minMaxLoc(output, &minValue, &maxValue);
    globalMax = max(maxValue, globalMax);
  }

  cout << globalMax << endl;
  return globalMax;

}

double computeSimilarity(MachineState * ms, int class1, int class2) {

  cout << "computeSimilarity on classes " << class1 << " " << class2 << endl;

  int tnc = ms->config.classLabels.size();
  if ( (class1 >=0) && (class1 < tnc) &&
       (class2 >=0) && (class2 < tnc) ) {
  } else {
    cout << "  unable to computeSimilarity: invalid class." << endl;
    assert(0);
  }

  return computeSimilarity(ms, ms->config.classHeight1AerialGradients[class1], ms->config.classHeight1AerialGradients[class2]);
}


void faceServo(MachineState * ms, vector<Rect> faces) {

  if (faces.size() == 0) {
    cout << "no faces, servoing more. " << ms->config.faceServoIterations << " " << ms->config.faceServoTimeout << endl;
    ms->pushWord("faceServoA");
    return;
  }

  eePose bestFacePose;
  double distance = VERYBIGNUMBER;
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  for (int i = 0; i < faces.size(); i++) {
    eePose faceImagePose = eePose::fromRectCentroid(faces[i]);
    double thisDistance = eePose::squareDistance(camera->vanishingPointReticle, faceImagePose);
    if (thisDistance < distance) {
      distance = thisDistance;
      bestFacePose = faceImagePose;
    }
  }

  double heightFactor = 1 / ms->config.minHeight;

  camera->reticle = camera->vanishingPointReticle;
  ms->config.pilotTarget.px = bestFacePose.px;
  ms->config.pilotTarget.py = bestFacePose.py;

  double Px = camera->reticle.px - ms->config.pilotTarget.px;
  double Py = camera->reticle.py - ms->config.pilotTarget.py;

  //double thisKp = ms->config.faceKp * heightFactor;
  double yScale = 1.0;
  double thisKp = ms->config.faceKp;
  double pTermX = thisKp*Px;
  double pTermY = thisKp*Py;

  // invert the current eePose orientation to decide which direction to move from POV
  //  of course this needs to be from the pose that corresponds to the frame this was taken from 
  Eigen::Vector3f localUnitX;
  {
    Eigen::Quaternionf qin(0, 1, 0, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(ms->config.trueEEPose.orientation.w, ms->config.trueEEPose.orientation.x, ms->config.trueEEPose.orientation.y, ms->config.trueEEPose.orientation.z);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitX.x() = qout.x();
    localUnitX.y() = qout.y();
    localUnitX.z() = qout.z();
  }

  Eigen::Vector3f localUnitY;
  {
    Eigen::Quaternionf qin(0, 0, 1, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(ms->config.trueEEPose.orientation.w, ms->config.trueEEPose.orientation.x, ms->config.trueEEPose.orientation.y, ms->config.trueEEPose.orientation.z);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitY.x() = qout.x();
    localUnitY.y() = qout.y();
    localUnitY.z() = qout.z();
  }

  //double newx = ms->config.currentEEPose.px + pTermX*localUnitY.x() - pTermY*localUnitX.x();
  //double newy = ms->config.currentEEPose.py + pTermX*localUnitY.y() - pTermY*localUnitX.y();
  //ms->config.currentEEPose.px = newx;
  //ms->config.currentEEPose.py = newy;
  ms->config.currentEEDeltaRPY.px = -pTermY;
  endEffectorAngularUpdate(&ms->config.currentEEPose, &ms->config.currentEEDeltaRPY);
  ms->config.currentEEDeltaRPY.py = pTermX*yScale;
  endEffectorAngularUpdate(&ms->config.currentEEPose, &ms->config.currentEEDeltaRPY);

  if ((fabs(Px) < ms->config.faceServoPixelThresh) && (fabs(Py) < ms->config.faceServoPixelThresh)) {
    cout << "face reached, continuing." << endl;
  } else {
    cout << "face not reached, servoing more. " << ms->config.faceServoIterations << " " << ms->config.faceServoTimeout << endl;
    ms->pushWord("faceServoA");
  }
}



void pixelToGlobalFullFromCacheZNotBuilt(MachineState * ms, int pX, int pY, double * gX, double * gY, pixelToGlobalCache * cache, double z) {
  Eigen::Vector4f pixelVector;
  float centralizedX = pX - cache->cx;
  float centralizedY = pY - cache->cy;

  double uncorrectedX = centralizedX;
  double uncorrectedY = centralizedY;

  double correctedX = uncorrectedX;
  double correctedY = uncorrectedY;

  double reuncorrectedX = uncorrectedX;
  double reuncorrectedY = uncorrectedY;

/*
cout << "gX gY gZ uncX uncY: " << gX << " " << gY << " " << gZ << " " << uncorrectedX << " " << uncorrectedY << endl;
cout << cache->apInv << endl << cache->g2pComposedZNotBuilt << endl << "pV: " << endl << pixelVector << endl << "gV: " << endl << globalVector << endl;
cout << "cx cy: " << cache->cx << " " << cache->cy << endl;

cout <<
cache->ccpRot << endl <<
cache->ccpRotInv << endl <<
cache->ccpTrans << endl <<
cache->ccpTransInv << endl <<
endl;
*/
  
  // the point of this procedure is to find a distortion lambda which undoes
  // kappa. This is impossible in principle and so lambda becomes both a
  // function of x, the undistorted (and sought) point, and y, the known
  // distorted point. Fortunately we can substitute our best approximation of x
  // (in the beginning, y) to obtain an approximate lambda to obtain an
  // approximate x which can be fed back into the formula.  convergence is
  // rapid for values of kappa close to what we are likely to encounter and
  // lenses with larger values will likely require more firepower to
  // approximate accurately.
  int p_g2p_cubic_max = 3;
  double skx = sqrt(cache->kappa_x);
  double sky = sqrt(cache->kappa_y);
  for (int i = 0; i < p_g2p_cubic_max; i++) {

    if (cache->kappa_x != 0.0) {
      //double sub_term_x = (1.0/skx + skx * uncorrectedX * uncorrectedX);
      //double lambdaX = -1.0 / ( sub_term_x * sub_term_x + uncorrectedX * uncorrectedX );
      double sub_term_x = (1.0/skx + skx * correctedX * correctedX);
      double lambdaX = -1.0 / ( sub_term_x * sub_term_x + uncorrectedX * uncorrectedX );
      //double lambdaX = -cache->kappa_x;
      correctedX = uncorrectedX * ( 1.0 + lambdaX * uncorrectedX * uncorrectedX);
    }

    if (cache->kappa_y != 0.0) {
      //double sub_term_y = (1.0/sky + sky * uncorrectedY * uncorrectedY);
      //double lambdaY = -1.0 / ( sub_term_y * sub_term_y + uncorrectedY * uncorrectedY );
      double sub_term_y = (1.0/sky + sky * correctedY * correctedY);
      double lambdaY = -1.0 / ( sub_term_y * sub_term_y + uncorrectedY * uncorrectedY );
      //double lambdaY = -cache->kappa_y;
      correctedY = uncorrectedY * ( 1.0 + lambdaY * uncorrectedY * uncorrectedY);
    }


    //reuncorrectedX = correctedX * ( 1.0 + cache->kappa_x * correctedX * correctedX);
    //reuncorrectedY = correctedY * ( 1.0 + cache->kappa_y * correctedY * correctedY);

/*
    cout << "iteration " << i << endl 
	  << "    corrected x: " << correctedX << "   y: " <<   correctedY << endl
	  << "  uncorrected x: " << uncorrectedX << " y: " << uncorrectedY << endl
	  << "reuncorrected x: " << reuncorrectedX << " y: " << reuncorrectedY << endl;
*/

    //uncorrectedX = correctedX;
    //uncorrectedY = correctedY;
  }
  
/*
  cout << "final values" << endl
	  << "    corrected  x: " << correctedX << "    y: " <<   correctedY << endl
	  << "  uncorrected x0: " << uncorrectedX0 << " y0: " << uncorrectedY0 << endl
	  << "reuncorrected  x: " << reuncorrectedX << "  y: " << reuncorrectedY << endl;
*/


  pixelVector <<
     z * correctedX,
     z * correctedY,
     z,
    1.0;

  Eigen::Vector4f globalVector;
  globalVector = cache->p2gComposedZNotBuilt * pixelVector;

  *gX = globalVector(0);
  *gY = globalVector(1);
}

void pixelToGlobalFullFromCacheZBuilt(MachineState * ms, int pX, int pY, double * gX, double * gY, pixelToGlobalCache * cache) {
  Eigen::Vector4f pixelVector;
  float centralizedX = pX - cache->cx;
  float centralizedY = pY - cache->cy;

  double uncorrectedX = centralizedX;
  double uncorrectedY = centralizedY;

  double correctedX = uncorrectedX;
  double correctedY = uncorrectedY;

  double reuncorrectedX = uncorrectedX;
  double reuncorrectedY = uncorrectedY;

/*
cout << "gX gY gZ uncX uncY: " << gX << " " << gY << " " << gZ << " " << uncorrectedX << " " << uncorrectedY << endl;
cout << cache->apInv << endl << cache->g2pComposedZNotBuilt << endl << "pV: " << endl << pixelVector << endl << "gV: " << endl << globalVector << endl;
cout << "cx cy: " << cache->cx << " " << cache->cy << endl;

cout <<
cache->ccpRot << endl <<
cache->ccpRotInv << endl <<
cache->ccpTrans << endl <<
cache->ccpTransInv << endl <<
endl;
*/
  
  // the point of this procedure is to find a distortion lambda which undoes
  // kappa. This is impossible in principle and so lambda becomes both a
  // function of x, the undistorted (and sought) point, and y, the known
  // distorted point. Fortunately we can substitute our best approximation of x
  // (in the beginning, y) to obtain an approximate lambda to obtain an
  // approximate x which can be fed back into the formula.  convergence is
  // rapid for values of kappa close to what we are likely to encounter and
  // lenses with larger values will likely require more firepower to
  // approximate accurately.
  int p_g2p_cubic_max = 3;
  double skx = sqrt(cache->kappa_x);
  double sky = sqrt(cache->kappa_y);
  for (int i = 0; i < p_g2p_cubic_max; i++) {

    if (cache->kappa_x != 0.0) {
      //double sub_term_x = (1.0/skx + skx * uncorrectedX * uncorrectedX);
      //double lambdaX = -1.0 / ( sub_term_x * sub_term_x + uncorrectedX * uncorrectedX );
      double sub_term_x = (1.0/skx + skx * correctedX * correctedX);
      double lambdaX = -1.0 / ( sub_term_x * sub_term_x + uncorrectedX * uncorrectedX );
      //double lambdaX = -cache->kappa_x;
      correctedX = uncorrectedX * ( 1.0 + lambdaX * uncorrectedX * uncorrectedX);
    }

    if (cache->kappa_y != 0.0) {
      //double sub_term_y = (1.0/sky + sky * uncorrectedY * uncorrectedY);
      //double lambdaY = -1.0 / ( sub_term_y * sub_term_y + uncorrectedY * uncorrectedY );
      double sub_term_y = (1.0/sky + sky * correctedY * correctedY);
      double lambdaY = -1.0 / ( sub_term_y * sub_term_y + uncorrectedY * uncorrectedY );
      //double lambdaY = -cache->kappa_y;
      correctedY = uncorrectedY * ( 1.0 + lambdaY * uncorrectedY * uncorrectedY);
    }


    //reuncorrectedX = correctedX * ( 1.0 + cache->kappa_x * correctedX * correctedX);
    //reuncorrectedY = correctedY * ( 1.0 + cache->kappa_y * correctedY * correctedY);

/*
    cout << "iteration " << i << endl 
	  << "    corrected x: " << correctedX << "   y: " <<   correctedY << endl
	  << "  uncorrected x: " << uncorrectedX << " y: " << uncorrectedY << endl
	  << "reuncorrected x: " << reuncorrectedX << " y: " << reuncorrectedY << endl;
*/

    //uncorrectedX = correctedX;
    //uncorrectedY = correctedY;
  }
  
/*
  cout << "final values" << endl
	  << "    corrected  x: " << correctedX << "    y: " <<   correctedY << endl
	  << "  uncorrected x0: " << uncorrectedX0 << " y0: " << uncorrectedY0 << endl
	  << "reuncorrected  x: " << reuncorrectedX << "  y: " << reuncorrectedY << endl;
*/


  pixelVector <<
     correctedX,
     correctedY,
    1.0,
    1.0;

  Eigen::Vector4f globalVector;
  globalVector = cache->p2gComposedZBuilt * pixelVector;

  *gX = globalVector(0);
  *gY = globalVector(1);

//cout << "pX pY gX gY: " << pX << " " << pY << " " << *gX << " " << *gY << endl;
}

void globalToPixelFullFromCache(MachineState * ms, int * pX, int * pY, double gX, double gY, double gZ, pixelToGlobalCache * cache) {
  Eigen::Vector4f globalVector;
  globalVector <<
    gX,
    gY,
    gZ,
    1.0 ;

  Eigen::Vector4f pixelVector;
  pixelVector = cache->g2pComposedZNotBuilt * globalVector;

  // divide by z
  // 
  double uncorrectedX = pixelVector(0) / gZ;
  double uncorrectedY = pixelVector(1) / gZ;

  double correctedX = uncorrectedX * (1.0 + cache->kappa_x * uncorrectedX * uncorrectedX);
  double correctedY = uncorrectedY * (1.0 + cache->kappa_y * uncorrectedY * uncorrectedY);

  *pX = correctedX + cache->cx;
  *pY = correctedY + cache->cy;

/*
cout << "pX pY: " << *pX << " " << *pY << endl;
*/

//  float centralizedX = pX - cache->cx;
//  float centralizedY = pY - cache->cy;
//  pixelVector <<
//    z * centralizedX * (1.0 + cache->kappa_x * centralizedX * centralizedX),
//    z * centralizedY * (1.0 + cache->kappa_y * centralizedY * centralizedY),
//    z,
//    1.0;
}

// the givenEEPose is the end effector pose and so we have to obtain the camera pose relative to that
void computePixelToGlobalFullCache(MachineState * ms, double gZ, eePose givenEEPose, pixelToGlobalCache * cache) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  cache->mu_x = camera->mu_x;
  cache->mu_y = camera->mu_y;
  cache->kappa_x = camera->kappa_x;
  cache->kappa_y = camera->kappa_y;

  // the old calibration solved for the principle point, but for now we are  
  // making the assumption that it is at the center of the image 
  cache->cx = camera->centerX - camera->cropUpperLeftCorner.px;
  cache->cy = camera->centerY - camera->cropUpperLeftCorner.py;

  // one magnification and one distortion coefficient per axis
  // XXX once depth is in line we can calibrate this over a range of depths.
  // XXX we might need to calibrate the depth camera's return range using the IR sensor.

  // construct the axis permutation matrix and its inverse
  cache->ap <<
    camera->r_00, camera->r_01,   0,	0,
    camera->r_10, camera->r_11,   0,	0,
	       0,	     0, 1.0,	0,
	       0,	     0,	  0,  1.0 ;
  cache->apInv = cache->ap.inverse();

  // construct the current camera pose affine matrix and its inverse
  eePose thisCameraPose;

  // fill out thisCameraPose
  thisCameraPose = camera->handCameraOffset.applyAsRelativePoseTo(givenEEPose);
/*
cout << "cache: " << 
givenEEPose << endl <<
camera->handCameraOffset << endl <<
thisCameraPose << endl <<
endl;
*/


  Eigen::Quaternionf quat(thisCameraPose.qw, thisCameraPose.qx, thisCameraPose.qy, thisCameraPose.qz);
  Eigen::Matrix3f rot3matrix = quat.toRotationMatrix();
  cache->ccpRot << 
    rot3matrix(0,0) , rot3matrix(0,1) , rot3matrix(0,2) , 0.0 , 
    rot3matrix(1,0) , rot3matrix(1,1) , rot3matrix(1,2) , 0.0 , 
    rot3matrix(2,0) , rot3matrix(2,1) , rot3matrix(2,2) , 0.0 , 
    0.0 , 0.0 , 0.0 , 1.0 ;
  cache->ccpRotInv = cache->ccpRot.inverse();
  cache->ccpTrans << 
    1.0 , 0.0 , 0.0 , thisCameraPose.px , 
    0.0 , 1.0 , 0.0 , thisCameraPose.py , 
    0.0 , 0.0 , 1.0 , thisCameraPose.pz , 
    0.0 , 0.0 , 0.0 , 1.0 ;
  cache->ccpTransInv = cache->ccpTrans.inverse();

  {
    // construct the pixel to global matrix origin matrix
    // note that we multiply by z during the vector construction 
    //  so that this one matrix applies to many z values.
    Eigen::Matrix4f p2gOriginZNotBuilt;
    p2gOriginZNotBuilt << 
      cache->mu_x , 0 , 0 , 0 ,
      0 , cache->mu_y , 0 , 0 ,
      0 , 0 , 1, 0 , 
      0 , 0 , 0, 1 ;

    // construt the global to pixel matrix origin matrix
    // note that we multiply by z during the vector construction 
    //  so that this one matrix applies to many z values.
    Eigen::Matrix4f g2pOriginZNotBuilt;
    g2pOriginZNotBuilt <<
      1.0/(cache->mu_x) , 0 , 0 , 0 ,
      0 , 1.0/(cache->mu_y) , 0 , 0 ,
      0 , 0 , 1, 0 , 
      0 , 0 , 0, 1 ;

    // construct the composed pixel to global matrix
    cache->p2gComposedZNotBuilt = cache->ccpTrans * cache->ccpRot * cache->ap * p2gOriginZNotBuilt;

    // construct the composed global to pixel matrix
    cache->g2pComposedZNotBuilt = g2pOriginZNotBuilt * cache->apInv * cache->ccpRotInv * cache->ccpTransInv;
  }
  {
    // construct the pixel to global matrix origin matrix
    // note that we will NOT mulitply by z during the vector construction 
    //  so that this one matrix applies to one z, and is thus faster for such batches.
    Eigen::Matrix4f p2gOriginZBuilt;
    p2gOriginZBuilt << 
      gZ * cache->mu_x , 0 , 0 , 0 ,
      0 , gZ * cache->mu_y , 0 , 0 ,
      0 , 0 , gZ, 0 , 
      0 , 0 , 0, 1 ;

    // construt the global to pixel matrix origin matrix
    // note that we multiply by z during the vector construction 
    //  so that this one matrix applies to many z values.
    Eigen::Matrix4f g2pOriginZBuilt;
    g2pOriginZBuilt <<
      1.0/(gZ*cache->mu_x) , 0 , 0 , 0 ,
      0 , 1.0/(gZ*cache->mu_y) , 0 , 0 ,
      0 , 0 , 1, 0 , 
      0 , 0 , 0, 1 ;

    // construct the composed pixel to global matrix
    cache->p2gComposedZBuilt = cache->ccpTrans * cache->ccpRot * cache->ap * p2gOriginZBuilt;

    // construct the composed global to pixel matrix
    cache->g2pComposedZBuilt = g2pOriginZBuilt * cache->apInv * cache->ccpRotInv * cache->ccpTransInv;
  }
}

void pixelToGlobalFullFromCacheZOOP(MachineState * ms, int pX, int pY, double * gX, double * gY, pixelToGlobalCache * cache) {
  // determine z using cache->target_plane
  double maggedCentralizedX = cache->mu_x * (pX - cache->cx);
  double maggedCentralizedY = cache->mu_y * (pY - cache->cy);

  double permutedCoordinate0 = cache->ap(0,0) * maggedCentralizedX + cache->ap(0,1) * maggedCentralizedY;
  double permutedCoordinate1 = cache->ap(1,0) * maggedCentralizedX + cache->ap(1,1) * maggedCentralizedY;
  double planed_z = cache->target_plane[3] / (permutedCoordinate0 * cache->target_plane[0] + permutedCoordinate1 * cache->target_plane[1] + cache->target_plane[2]);

  /*
  if (pX % 10 == 0 && pY % 10 == 0) {
    cout << planed_z << " ";

    cout << 
    cache->target_plane[0] << " " <<
    cache->target_plane[1] << " " <<
    cache->target_plane[2] << " " <<
    cache->target_plane[3] << " " << endl;
  }
  */
  // XXX all of this can be substantially optimized by building another matrix by shooting the unit vectors through this transformation. 

  // cast with pixelToGlobalFullFromCacheZNotBuilt
  pixelToGlobalFullFromCacheZNotBuilt(ms, pX, pY, gX, gY, cache, planed_z);
}

void computePixelToGlobalFullOOPCache(MachineState * ms, double gZ, eePose givenEEPose, eePose otherPlane, pixelToGlobalCache * cache) {
  // other plane is the target photographic plane, usually the scene's anchor pose
  // anchor pose tends to be the pose of the end effector upon calling
  computePixelToPlaneCache(ms, gZ, givenEEPose, otherPlane, cache);

  // fill out thisCameraPose
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  eePose thisCameraPose = camera->handCameraOffset.applyAsRelativePoseTo(givenEEPose);
  eePose transformed_anchor = otherPlane.getPoseRelativeTo(thisCameraPose);

  // the plane is facing towards the arm
  eePose tempZUnit = eePose(0,0,1,0,0,0,1);
  eePose tempPlaneNormal = tempZUnit.applyAsRelativePoseTo(transformed_anchor);

  cache->target_plane[0] = tempPlaneNormal.px - transformed_anchor.px;
  cache->target_plane[1] = tempPlaneNormal.py - transformed_anchor.py;
  cache->target_plane[2] = tempPlaneNormal.pz - transformed_anchor.pz;

  // take a point known to be on the plane (the transformed anchor pose) and
  // find the dot product with the determined normal of the plane, then add the target plane distance
  cache->target_plane[3] = cache->target_plane[0]*transformed_anchor.px + cache->target_plane[1]*transformed_anchor.py + cache->target_plane[2]*transformed_anchor.pz - gZ;
}

eePose pixelToGlobalEEPose(MachineState * ms, int pX, int pY, double gZ) {
  eePose result;
  pixelToGlobal(ms, pX, pY, gZ, &result.px, &result.py);
  result.pz = ms->config.trueEEPose.position.z - ms->config.currentTableZ;
  result.qx = 0;
  result.qy = 0;
  result.qz = 0;
  return result;
}

string pixelToGlobalCacheToString(const pixelToGlobalCache &cache)
{
  stringstream buf;

  buf << "givenEEPose: " << cache.givenEEPose << endl;
  buf << "gZ: " << cache.gZ << endl;
  buf << "x1: " << cache.x1 << endl; 
  buf << "x2: " << cache.x2 << endl;
  buf << "x3: " << cache.x3 << endl;
  buf << "x4: " << cache.x4 << endl;

  buf << "y1: " << cache.y1 << endl;
  buf << "y2: " << cache.y2 << endl;
  buf << "y3: " << cache.y3 << endl;
  buf << "y4: " << cache.y4 << endl;

  buf << "z1: " << cache.z1 << endl;
  buf << "z2: " << cache.z2 << endl;
  buf << "z3: " << cache.z3 << endl;
  buf << "z4: " << cache.z4 << endl;

  buf << "reticlePixelX: " << cache.reticlePixelX << endl;
  buf << "reticlePixelY: " << cache.reticlePixelY << endl;
  buf << "reticlePixelXOffset: " << cache.reticlePixelXOffset << endl;
  buf << "reticlePixelYOffset: " << cache.reticlePixelYOffset << endl;

  buf << "x_thisZ: " << cache.x_thisZ << endl;
  buf << "y_thisZ: " << cache.y_thisZ << endl;

  buf << "gXFactor: " << cache.gXFactor << endl;
  buf << "gYFactor: " << cache.gYFactor << endl;
  buf << "finalXOffset: " << cache.finalXOffset << endl;
  buf << "finalYOffset: " << cache.finalYOffset << endl;

  buf << "un_rot_mat: " << cache.un_rot_mat << endl;

  buf << "rotx[3]: " << cache.rotx[0] << ", " << cache.rotx[1] << ", " << cache.rotx[2] << endl;
  buf << "roty[3]: " << cache.roty[0] << ", " << cache.roty[1] << ", " << cache.roty[2] << endl;


  buf << "dx: " << cache.dx << endl;
  buf << "cx: " << cache.cx << endl;
  buf << "b42x: " << cache.b42x << endl;
  buf << "b31x: " << cache.b31x << endl;
  buf << "bDiffx: " << cache.bDiffx << endl;
  buf << "bx: " << cache.bx << endl;


  buf << "dy: " << cache.dy << endl;
  buf << "cy: " << cache.cy << endl;
  buf << "b42y: " << cache.b42y << endl;
  buf << "b31y: " << cache.b31y << endl;
  buf << "bDiffy: " << cache.bDiffy << endl;
  buf << "by: " << cache.by << endl;
  return buf.str();

}

void interpolateM_xAndM_yFromZ(MachineState * ms, double dZ, double * m_x, double * m_y) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  if (camera->currentCameraCalibrationMode == CAMCAL_LINBOUNDED) {
    double bBZ[4];
    bBZ[0] = convertHeightIdxToGlobalZ(ms, 0) + ms->config.currentTableZ;
    bBZ[1] = convertHeightIdxToGlobalZ(ms, 1) + ms->config.currentTableZ;
    bBZ[2] = convertHeightIdxToGlobalZ(ms, 2) + ms->config.currentTableZ;
    bBZ[3] = convertHeightIdxToGlobalZ(ms, 3) + ms->config.currentTableZ;

    if (dZ <= bBZ[0]) {
      *m_x = camera->m_x_h[0];
      *m_y = camera->m_y_h[0];
    } else if (dZ <= bBZ[1]) {
      double gap = bBZ[1] - bBZ[0];
      double c0 = 1.0 - ((dZ - bBZ[0])/gap);
      double c1 = 1.0 - ((bBZ[1] - dZ)/gap);
      *m_x = c0*camera->m_x_h[0] + c1*camera->m_x_h[1];
      *m_y = c0*camera->m_y_h[0] + c1*camera->m_y_h[1];
    } else if (dZ <= bBZ[2]) {
      double gap = bBZ[2] - bBZ[1];
      double c1 = 1.0 - ((dZ - bBZ[1])/gap);
      double c2 = 1.0 - ((bBZ[2] - dZ)/gap);
      *m_x = c1*camera->m_x_h[1] + c2*camera->m_x_h[2];
      *m_y = c1*camera->m_y_h[1] + c2*camera->m_y_h[2];
    } else if (dZ <= bBZ[3]) {
      double gap = bBZ[3] - bBZ[2];
      double c2 = 1.0 - ((dZ - bBZ[2])/gap);
      double c3 = 1.0 - ((bBZ[3] - dZ)/gap);
      *m_x = c2*camera->m_x_h[2] + c3*camera->m_x_h[3];
      *m_y = c2*camera->m_y_h[2] + c3*camera->m_y_h[3];
    } else if (dZ > bBZ[3]) {
      *m_x = camera->m_x_h[3];
      *m_y = camera->m_y_h[3];
    } else {
      assert(0); // my my
    }
    //cout << camera->m_x_h[0] << " " << camera->m_x_h[1] << " " << camera->m_x_h[2] << " " << camera->m_x_h[3] << " " << *m_x << endl;
    //cout << m_y_h[0] << " " << camera->m_y_h[1] << " " << camera->m_y_h[2] << " " << camera->m_y_h[3] << " " << *m_y << endl;
  } else if (camera->currentCameraCalibrationMode == CAMCAL_QUADRATIC) {
    *(m_y) = camera->m_YQ[0] + (dZ * camera->m_YQ[1]) + (dZ * dZ * camera->m_YQ[2]);
    *(m_x) = camera->m_XQ[0] + (dZ * camera->m_XQ[1]) + (dZ * dZ * camera->m_XQ[2]);
  } else if (camera->currentCameraCalibrationMode == CAMCAL_HYPERBOLIC) {
    
    double ooDZ = dZ;
    if (dZ == 0) {
      cout << "oops, magnification interpolate failed." << endl;
    } else {
      ooDZ = 1.0/dZ;
    } 
    *(m_y) = camera->m_YQ[0] + (ooDZ * camera->m_YQ[1]) + (ooDZ * ooDZ * camera->m_YQ[2]);
    *(m_x) = camera->m_XQ[0] + (ooDZ * camera->m_XQ[1]) + (ooDZ * ooDZ * camera->m_XQ[2]);
  } else {
    ROS_ERROR_STREAM("Invalid camera calibration mode: " << camera->currentCameraCalibrationMode);
    assert(0);
  }
}

void pixelToGlobal(MachineState * ms, int pX, int pY, double gZ, double * gX, double * gY) {
  pixelToGlobal(ms, pX, pY, gZ, gX, gY, ms->config.trueEEPoseEEPose);
}

void pixelToPlane(MachineState * ms, int pX, int pY, double gZ, double * gX, double * gY, eePose givenEEPose, eePose referenceFrame) {

  eePose transformedPose = givenEEPose.getPoseRelativeTo(referenceFrame);

  pixelToGlobal(ms, pX, pY, gZ, gX, gY, transformedPose);
  
}

void computePixelToPlaneCache(MachineState * ms, double gZ, eePose givenEEPose, eePose referenceFrame, pixelToGlobalCache * cache) {
  eePose transformedPose = givenEEPose.getPoseRelativeTo(referenceFrame);
  computePixelToGlobalCache(ms, gZ, transformedPose, cache);
}

void computePixelToGlobalCache(MachineState * ms, double gZ, eePose givenEEPose, pixelToGlobalCache * cache) {
  computePixelToGlobalFullCache(ms, gZ, givenEEPose, cache);
/*
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  interpolateM_xAndM_yFromZ(ms, gZ, &camera->m_x, &camera->m_y);
  cache->givenEEPose = givenEEPose;
  cache->gZ = gZ;
  cache->x1 = camera->heightReticles[0].px;
  cache->x2 = camera->heightReticles[1].px;
  cache->x3 = camera->heightReticles[2].px;
  cache->x4 = camera->heightReticles[3].px;

  cache->y1 = camera->heightReticles[0].py;
  cache->y2 = camera->heightReticles[1].py;
  cache->y3 = camera->heightReticles[2].py;
  cache->y4 = camera->heightReticles[3].py;

  cache->z1 = convertHeightIdxToGlobalZ(ms, 0) + ms->config.currentTableZ;
  cache->z2 = convertHeightIdxToGlobalZ(ms, 1) + ms->config.currentTableZ;
  cache->z3 = convertHeightIdxToGlobalZ(ms, 2) + ms->config.currentTableZ;
  cache->z4 = convertHeightIdxToGlobalZ(ms, 3) + ms->config.currentTableZ;

  cache->reticlePixelX = 0.0;
  cache->reticlePixelY = 0.0;
  {
    double d = camera->handCameraOffset.py;
    double c = ((cache->z4*cache->x4-cache->z2*cache->x2)*(cache->x3-cache->x1)-(cache->z3*cache->x3-cache->z1*cache->x1)*(cache->x4-cache->x2))/((cache->z1-cache->z3)*(cache->x4-cache->x2)-(cache->z2-cache->z4)*(cache->x3-cache->x1));

    double b42 = (cache->z4*cache->x4-cache->z2*cache->x2+(cache->z2-cache->z4)*c)/(cache->x4-cache->x2);
    double b31 = (cache->z3*cache->x3-cache->z1*cache->x1+(cache->z1-cache->z3)*c)/(cache->x3-cache->x1);

    double bDiff = b42-b31;
    double b = (b42+b31)/2.0;

    double zFraction = (gZ); // (gZ-b)
    // taking out other singularity
    double x_thisZ = c + ( (cache->x1-c)*(cache->z1) )/zFraction;
    //int x_thisZ = c + ( (cache->x1-c)*(cache->z1-b) )/zFraction;
    //x_thisZ = c + ( (d)*(x_thisZ-c) )/(d);
    // removed the above correction
    cache->reticlePixelX = x_thisZ;

    //cout << "bDiff = " << bDiff << ", c = " << c << " b42, b31: " << b42 << " " << b31 << " " << endl;
  }
  {
    double d = -camera->handCameraOffset.px;
    double c = ((cache->z4*cache->y4-cache->z2*cache->y2)*(cache->y3-cache->y1)-(cache->z3*cache->y3-cache->z1*cache->y1)*(cache->y4-cache->y2))/((cache->z1-cache->z3)*(cache->y4-cache->y2)-(cache->z2-cache->z4)*(cache->y3-cache->y1));

    double b42 = (cache->z4*cache->y4-cache->z2*cache->y2+(cache->z2-cache->z4)*c)/(cache->y4-cache->y2);
    double b31 = (cache->z3*cache->y3-cache->z1*cache->y1+(cache->z1-cache->z3)*c)/(cache->y3-cache->y1);

    double bDiff = b42-b31;
    double b = (b42+b31)/2.0;

    double zFraction = (gZ); // (gZ-b)
    // taking out other singularity
    double y_thisZ = c + ( (cache->y1-c)*(cache->z1) )/zFraction;
    //y_thisZ = c + ( (d)*(y_thisZ-c) )/(d);
    // removed the above correction
    cache->reticlePixelY = y_thisZ;
    //cout << "bDiff = " << bDiff << ", c = " << c << " b42, b31: " << b42 << " " << b31 << " " << endl;
  }

  cache->reticlePixelYOffset = cache->reticlePixelY - ms->config.offY - cache->reticlePixelX;
  cache->reticlePixelXOffset = cache->reticlePixelX - ms->config.offX - cache->reticlePixelY;

  // account for rotation of the end effector 
  Quaternionf eeqform(givenEEPose.qw, givenEEPose.qx, givenEEPose.qy, givenEEPose.qz);
  Quaternionf crane2Orient(0, 1, 0, 0);
  Quaternionf rel = eeqform * crane2Orient.inverse();
  Quaternionf ex(0,1,0,0);
  Quaternionf zee(0,0,0,1);
	
  Quaternionf result = rel * ex * rel.conjugate();
  double aY = result.y();
  double aX = result.x();

  // ATTN 22
  //double angle = atan2(aY, aX)*180.0/3.1415926;
  double angle = vectorArcTan(ms, aY, aX)*180.0/3.1415926;
  angle = (angle);
  double scale = 1.0;
  Point center = Point(cache->reticlePixelX, cache->reticlePixelY);

  cache->un_rot_mat = getRotationMatrix2D( center, angle, scale );
  cache->rotx[0] = cache->un_rot_mat.at<double>(0, 0);
  cache->rotx[1] = cache->un_rot_mat.at<double>(0, 1);
  cache->rotx[2] = cache->un_rot_mat.at<double>(0, 2);
  cache->roty[0] = cache->un_rot_mat.at<double>(1, 0);
  cache->roty[1] = cache->un_rot_mat.at<double>(1, 1);
  cache->roty[2] = cache->un_rot_mat.at<double>(1, 2);

  double tmp[4];

  tmp[0] = cache->rotx[0] * camera->transform_matrix[0] + cache->rotx[1]*camera->transform_matrix[2];
  tmp[1] = cache->rotx[0] * camera->transform_matrix[1] + cache->rotx[1]*camera->transform_matrix[3];
  tmp[2] = cache->roty[0] * camera->transform_matrix[0] + cache->roty[1]*camera->transform_matrix[2];
  tmp[3] = cache->roty[0] * camera->transform_matrix[1] + cache->roty[1]*camera->transform_matrix[3];
  cache->rotx[0] = tmp[0];
  cache->rotx[1] = tmp[1];
  cache->roty[0] = tmp[2];
  cache->roty[1] = tmp[3];


  cache->dx = camera->handCameraOffset.py/camera->m_x;
  cache->cx = ((cache->z4*cache->x4-cache->z2*cache->x2)*(cache->x3-cache->x1)-(cache->z3*cache->x3-cache->z1*cache->x1)*(cache->x4-cache->x2))/((cache->z1-cache->z3)*(cache->x4-cache->x2)-(cache->z2-cache->z4)*(cache->x3-cache->x1));
  cache->b42x = (cache->z4*cache->x4-cache->z2*cache->x2+(cache->z2-cache->z4)*cache->cx)/(cache->x4-cache->x2);
  cache->b31x = (cache->z3*cache->x3-cache->z1*cache->x1+(cache->z1-cache->z3)*cache->cx)/(cache->x3-cache->x1);

  cache->bDiffx = cache->b42x-cache->b31x;
  cache->bx = (cache->b42x+cache->b31x)/2.0;


  cache->dy = -camera->handCameraOffset.px/camera->m_y;
  cache->cy = ((cache->z4*cache->y4-cache->z2*cache->y2)*(cache->y3-cache->y1)-(cache->z3*cache->y3-cache->z1*cache->y1)*(cache->y4-cache->y2))/((cache->z1-cache->z3)*(cache->y4-cache->y2)-(cache->z2-cache->z4)*(cache->y3-cache->y1));

  cache->b42y = (cache->z4*cache->y4-cache->z2*cache->y2+(cache->z2-cache->z4)*cache->cy)/(cache->y4-cache->y2);
  cache->b31y = (cache->z3*cache->y3-cache->z1*cache->y1+(cache->z1-cache->z3)*cache->cy)/(cache->y3-cache->y1);

  cache->bDiffy = cache->b42y-cache->b31y;
    //cout << "x1 x2 x3 x4: " << x1 << " " << x2 << " " << x3 << " " << x4 << endl;
    //cout << "y1 y2 y3 y4: " << y1 << " " << y2 << " " << y3 << " " << y4 << endl;
    //cout << "z1 z2 z3 z4: " << z1 << " " << z2 << " " << z3 << " " << z4 << endl;
    //cout << "bDiff = " << bDiff << ", c = " << c << " b42, b31: " << b42 << " " << b31 << " " << endl;
  cache->by = (cache->b42y+cache->b31y)/2.0;

  
  cache->x_thisZ = cache->cx + ( (cache->x1-cache->cx)*(cache->z1) )/(cache->gZ);
  cache->y_thisZ = cache->cy + ( (cache->y1-cache->cy)*(cache->z1) )/(cache->gZ);
  cache->gXFactor = (cache->dx) / (cache->x_thisZ-cache->cx) ;
  cache->gYFactor = (cache->dy) / (cache->y_thisZ-cache->cy) ;

  cache->finalXOffset = cache->givenEEPose.px - cache->dx - cache->cx*cache->gXFactor;
  cache->finalYOffset = cache->givenEEPose.py - cache->dy - cache->cy*cache->gYFactor;
*/
}


void pixelToGlobal(MachineState * ms, int pX, int pY, double gZ, double * gX, double * gY, eePose givenEEPose) {
  pixelToGlobalCache data;
  computePixelToGlobalCache(ms, gZ, givenEEPose, &data);
  pixelToGlobalFromCache(ms, pX, pY, gX, gY, &data);
}

void pixelToGlobalFromCache(MachineState * ms, int pX, int pY, double * gX, double * gY, pixelToGlobalCache * cache) {

  pixelToGlobalFullFromCacheZBuilt(ms, pX, pY, gX, gY, cache);

/*
  double rotatedPX = (cache->rotx[0] * pX +
                      cache->rotx[1] * pY +
                      cache->rotx[2]);
  double rotatedPY = (cache->roty[0] * pX +
                      cache->roty[1] * pY +
                      cache->roty[2]);
  //assert(0);

  pX = cache->reticlePixelXOffset + rotatedPY;
  pY = cache->reticlePixelYOffset + rotatedPX;

  //double x_thisZ = cache->cx + ( (cache->x1-cache->cx)*(cache->z1-cache->bx) )/(cache->gZ-cache->bx);
  //*gX = cache->givenEEPose.px - cache->dx + ( (pX-cache->cx)*(cache->dx) )/( (x_thisZ-cache->cx) ) ;

  //double y_thisZ = cache->cy + ( (cache->y1-cache->cy)*(cache->z1-cache->by) )/(cache->gZ-cache->by);
  //*gY = cache->givenEEPose.py - cache->dy + ( (pY-cache->cy)*(cache->dy) )/( (y_thisZ-cache->cy) ) ;
  // taking out other singularity

  //double x_thisZ = cache->cx + ( (cache->x1-cache->cx)*(cache->z1-cache->bx) )/(cache->gZ);
  //double x_thisZ = cache->cx + ( (cache->x1-cache->cx)*(cache->z1) )/(cache->gZ);
  //*gX = cache->givenEEPose.px - cache->dx + (pX-cache->cx)*cache->gXFactor;
  *gX = cache->finalXOffset + pX * cache->gXFactor;

  //double y_thisZ = cache->cy + ( (cache->y1-cache->cy)*(cache->z1-cache->by) )/(cache->gZ);
  //double y_thisZ = cache->cy + ( (cache->y1-cache->cy)*(cache->z1) )/(cache->gZ);
  //*gY = cache->givenEEPose.py - cache->dy + (pY-cache->cy)*cache->gYFactor;
  *gY = cache->finalYOffset + pY * cache->gYFactor;

*/
}

void pixelToGlobalFromCacheBackCast(MachineState * ms, int pX, int pY, double * gX, double * gY, pixelToGlobalCache * cache) {
// XXX TODO adapt to matrix framework
  double rotatedPX = (cache->rotx[0] * pX +
                      cache->rotx[1] * pY +
                      cache->rotx[2]);
  double rotatedPY = (cache->roty[0] * pX +
                      cache->roty[1] * pY +
                      cache->roty[2]);
  //assert(0);

  pX = cache->reticlePixelXOffset + rotatedPY;
  pY = cache->reticlePixelYOffset + rotatedPX;

/*
  double x_thisZ = cache->cx + ( (cache->x1-cache->cx)*(cache->z1-cache->bx) )/(cache->gZ-cache->bx);
  *gX = cache->givenEEPose.px - cache->dx + ( (pX-cache->cx)*(cache->dx) )/( (x_thisZ-cache->cx) ) ;

  double y_thisZ = cache->cy + ( (cache->y1-cache->cy)*(cache->z1-cache->by) )/(cache->gZ-cache->by);
  *gY = cache->givenEEPose.py - cache->dy + ( (pY-cache->cy)*(cache->dy) )/( (y_thisZ-cache->cy) ) ;
*/
  // taking out other singularity

  //double x_thisZ = cache->cx + ( (cache->x1-cache->cx)*(cache->z1-cache->bx) )/(cache->gZ);
  //double x_thisZ = cache->cx + ( (cache->x1-cache->cx)*(cache->z1) )/(cache->gZ);
  //*gX = cache->givenEEPose.px - cache->dx + (pX-cache->cx)*cache->gXFactor;
  *gX = cache->finalXOffset - pX * cache->gXFactor;

  //double y_thisZ = cache->cy + ( (cache->y1-cache->cy)*(cache->z1-cache->by) )/(cache->gZ);
  //double y_thisZ = cache->cy + ( (cache->y1-cache->cy)*(cache->z1) )/(cache->gZ);
  //*gY = cache->givenEEPose.py - cache->dy + (pY-cache->cy)*cache->gYFactor;
  *gY = cache->finalYOffset - pY * cache->gYFactor;

}


void globalToPixel(MachineState * ms, int * pX, int * pY, double gZ, double gX, double gY) {
  globalToPixel(ms, pX, pY, gZ, gX, gY, ms->config.trueEEPoseEEPose);
}

void globalToPixel(MachineState * ms, int * pX, int * pY, double gZ, double gX, double gY, eePose givenEEPose) {

  pixelToGlobalCache cache;
  computePixelToGlobalFullCache(ms, gZ, givenEEPose, &cache);
  globalToPixelFullFromCache(ms, pX, pY, gX, gY, gZ, &cache);
/*
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  interpolateM_xAndM_yFromZ(ms, gZ, &camera->m_x, &camera->m_y);

  int x1 = camera->heightReticles[0].px;
  int x2 = camera->heightReticles[1].px;
  int x3 = camera->heightReticles[2].px;
  int x4 = camera->heightReticles[3].px;

  int y1 = camera->heightReticles[0].py;
  int y2 = camera->heightReticles[1].py;
  int y3 = camera->heightReticles[2].py;
  int y4 = camera->heightReticles[3].py;

  double z1 = convertHeightIdxToGlobalZ(ms, 0) + ms->config.currentTableZ;
  double z2 = convertHeightIdxToGlobalZ(ms, 1) + ms->config.currentTableZ;
  double z3 = convertHeightIdxToGlobalZ(ms, 2) + ms->config.currentTableZ;
  double z4 = convertHeightIdxToGlobalZ(ms, 3) + ms->config.currentTableZ;

  double reticlePixelX = 0.0;
  double reticlePixelY = 0.0;
  {
    //double d = camera->handCameraOffset.py;
    double d = camera->handCameraOffset.py/camera->m_x;
    double c = ((z4*x4-z2*x2)*(x3-x1)-(z3*x3-z1*x1)*(x4-x2))/((z1-z3)*(x4-x2)-(z2-z4)*(x3-x1));

    double b42 = (z4*x4-z2*x2+(z2-z4)*c)/(x4-x2);
    double b31 = (z3*x3-z1*x1+(z1-z3)*c)/(x3-x1);

    double bDiff = b42-b31;
    //cout << "x1 x2 x3 x4: " << x1 << " " << x2 << " " << x3 << " " << x4 << endl;
    //cout << "y1 y2 y3 y4: " << y1 << " " << y2 << " " << y3 << " " << y4 << endl;
    //cout << "z1 z2 z3 z4: " << z1 << " " << z2 << " " << z3 << " " << z4 << endl;
    //cout << "bDiff = " << bDiff << ", c = " << c << " b42, b31: " << b42 << " " << b31 << " " << endl;
    double b = (b42+b31)/2.0;

    double zFraction = (gZ); // (gZ-b)
    // taking out other singularity
    double x_thisZ = c + ( (x1-c)*(z1) )/zFraction;
    //int x_thisZ = c + ( (x1-c)*(z1-b) )/zFraction;
    //int x_thisZ = c + ( camera->m_x*(x1-c)*(z1-b) )/zFraction;
    //*pX = c + ( (gX-d)*(x1-c) )/(ms->config.currentEEPose.px-d);
    //*pX = c + ( (gX-d)*(x_thisZ-c) )/(ms->config.currentEEPose.px-d);
    //*pX = c + ( camera->m_x*(gX-givenEEPose.px+d)*(x_thisZ-c) )/(d);
    *pX = c + ( (gX-givenEEPose.px+d)*(x_thisZ-c) )/(d);
    // need to set this again so things match up if gX is truEEpose
    //x_thisZ = c + ( camera->m_x*(x1-c)*(z1-b) )/zFraction;
    //x_thisZ = c + ( (d)*(x_thisZ-c) )/(d);
    // removed the above correction
    reticlePixelX = x_thisZ;
  }
  {
    //double d = -camera->handCameraOffset.px;
    double d = -camera->handCameraOffset.px/camera->m_y;
    double c = ((z4*y4-z2*y2)*(y3-y1)-(z3*y3-z1*y1)*(y4-y2))/((z1-z3)*(y4-y2)-(z2-z4)*(y3-y1));

    double b42 = (z4*y4-z2*y2+(z2-z4)*c)/(y4-y2);
    double b31 = (z3*y3-z1*y1+(z1-z3)*c)/(y3-y1);

    double bDiff = b42-b31;
    //cout << "x1 x2 x3 x4: " << x1 << " " << x2 << " " << x3 << " " << x4 << endl;
    //cout << "y1 y2 y3 y4: " << y1 << " " << y2 << " " << y3 << " " << y4 << endl;
    //cout << "z1 z2 z3 z4: " << z1 << " " << z2 << " " << z3 << " " << z4 << endl;
    //cout << "bDiff = " << bDiff << ", c = " << c << " b42, b31: " << b42 << " " << b31 << " " << endl;
    double b = (b42+b31)/2.0;

    double zFraction = (gZ); // (gZ-b)
    // taking out other singularity
    double y_thisZ = c + ( (y1-c)*(z1) )/zFraction;
    //int y_thisZ = c + ( (y1-c)*(z1-b) )/zFraction;
    //int y_thisZ = c + ( camera->m_y*(y1-c)*(z1-b) )/zFraction;
    //*pY = c + ( (gY-d)*(y1-c) )/(ms->config.currentEEPose.py-d);
    //*pY = c + ( (gY-d)*(y_thisZ-c) )/(ms->config.currentEEPose.py-d);
    //*pY = c + ( camera->m_y*(gY-givenEEPose.py+d)*(y_thisZ-c) )/(d);
    *pY = c + ( (gY-givenEEPose.py+d)*(y_thisZ-c) )/(d);
    // need to set this again so things match up if gX is truEEpose
    //y_thisZ = c + ( camera->m_y*(y1-c)*(z1-b) )/zFraction;
    //y_thisZ = c + ( (d)*(y_thisZ-c) )/(d);
    // removed the above correction
    reticlePixelY = y_thisZ;
  }

  //cout << "reticlePixelX, reticlePixelY: " << reticlePixelX << " " << reticlePixelY << endl;

  // account for rotation of the end effector 
  Quaternionf eeqform(ms->config.trueEEPose.orientation.w, ms->config.trueEEPose.orientation.x, ms->config.trueEEPose.orientation.y, ms->config.trueEEPose.orientation.z);
  Quaternionf crane2Orient(0, 1, 0, 0);
  Quaternionf rel = eeqform * crane2Orient.inverse();
  Quaternionf ex(0,1,0,0);
  Quaternionf zee(0,0,0,1);
	
  Quaternionf result = rel * ex * rel.conjugate();
  Quaternionf thumb = rel * zee * rel.conjugate();
  double aY = result.y();
  double aX = result.x();

  // ATTN 22
  //double angle = atan2(aY, aX)*180.0/3.1415926;
  double angle = vectorArcTan(ms, aY, aX)*180.0/3.1415926;
  angle = angle;
  double scale = 1.0;
  Point center = Point(reticlePixelX, reticlePixelY);

  Mat un_rot_mat = getRotationMatrix2D( center, angle, scale );
  double rotx[3];
  double roty[3];
  rotx[0] = un_rot_mat.at<double>(0, 0);
  rotx[1] = un_rot_mat.at<double>(0, 1);
  rotx[2] = un_rot_mat.at<double>(0, 2);
  roty[0] = un_rot_mat.at<double>(1, 0);
  roty[1] = un_rot_mat.at<double>(1, 1);
  roty[2] = un_rot_mat.at<double>(1, 2);

  
//  Mat toUn(3,1,CV_64F);
//  toUn.at<double>(0,0)=*pX;
//  toUn.at<double>(1,0)=*pY;
//  toUn.at<double>(2,0)=1.0;
//  Mat didUn = un_rot_mat*toUn;
//  *pX = didUn.at<double>(0,0);
//  *pY = didUn.at<double>(1,0);

  double rotatedPX = rotx[0] * *pX + rotx[1] * *pY + rotx[2];
  double rotatedPY = roty[0] * *pX + roty[1] * *pY + roty[2];
  *pX = rotatedPX;
  *pY = rotatedPY;

  double oldPx = *pX;
  double oldPy = *pY;
  //*pX = reticlePixelX + camera->m_y*(oldPy - reticlePixelY) + ms->config.offX;
  //*pY = reticlePixelY + camera->m_x*(oldPx - reticlePixelX) + ms->config.offY;
  *pX = round(reticlePixelX + (oldPy - reticlePixelY) + ms->config.offX);
  *pY = round(reticlePixelY + (oldPx - reticlePixelX) + ms->config.offY);
*/
}

void paintEEPoseOnWrist(MachineState * ms, eePose toPaint, cv::Scalar theColor) {
  cv::Scalar THEcOLOR(255-theColor[0], 255-theColor[1], 255-theColor[2]);
  int lineLength = 5;
  int pXo = 0, pYo = 0;  
  int pX = 0, pY = 0;  
  double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;

  globalToPixel(ms, &pXo, &pYo, zToUse, toPaint.px, toPaint.py);
  pX = pXo - lineLength;
  pY = pYo - lineLength;
  //cout << "paintEEPoseOnWrist pX pY zToUse: " << pX << " " << pY << " " << zToUse << endl;
  if ( (pX > 0+lineLength) && (pX < ms->config.wristViewImage.cols-lineLength) && (pY > 0+lineLength) && (pY < ms->config.wristViewImage.rows-lineLength) ) {
    {
      Point pt1(pX+lineLength, pY);
      Point pt2(pX+lineLength, pY+lineLength*2);
      line(ms->config.wristViewImage, pt1, pt2, theColor);
    }
    {
      Point pt1(pX, pY+lineLength);
      Point pt2(pX+lineLength*2, pY+lineLength);
      line(ms->config.wristViewImage, pt1, pt2, theColor);
    }
  }

  // draw the test pattern for the inverse transformation 
  if (1) {
    double gX = 0, gY = 0;
    int pXb = 0, pYb = 0;  
    pixelToGlobal(ms, pXo, pYo, zToUse, &gX, &gY);
    globalToPixel(ms, &pXb, &pYb, zToUse, gX, gY);
    pX = pXb - lineLength;
    pY = pYb - lineLength;
    //cout << "PAINTeepOSEoNwRIST pX pY gX gY: " << pX << " " << pY << " " << gX << " " << gY << endl;
    if ( (pX > 0+lineLength) && (pX < ms->config.wristViewImage.cols-lineLength) && (pY > 0+lineLength) && (pY < ms->config.wristViewImage.rows-lineLength) ) {
      {
	Point pt1(pX+lineLength, pY);
	Point pt2(pX+lineLength, pY+lineLength*2);
	line(ms->config.wristViewImage, pt1, pt2, THEcOLOR);
      }
      {
	Point pt1(pX, pY+lineLength);
	Point pt2(pX+lineLength*2, pY+lineLength);
	line(ms->config.wristViewImage, pt1, pt2, THEcOLOR);
      }
    }
  }

  //guardedImshow(ms->config.objectViewerName, ms->config.objectViewerImage);
}

double vectorArcTan(MachineState * ms, double y, double x) {
  int maxVaSlot = 0;
  double maxVaDot = -INFINITY;
  for (int vaSlot = 0; vaSlot < ms->config.vaNumAngles; vaSlot++) {
    double product = ms->config.vaX[vaSlot]*x + ms->config.vaY[vaSlot]*y;
    if (product > maxVaDot) {
      maxVaDot = product;
      maxVaSlot = vaSlot;
    }
  } 
  // return value in interval [-pi, pi]

  double angleZeroTwopi = (maxVaSlot * ms->config.vaDelta);
  if (angleZeroTwopi <= 3.1415926) {
    return angleZeroTwopi;
  } else {
    return ( angleZeroTwopi - (2.0*3.1415926) );
  }
}

void initVectorArcTan(MachineState * ms) {
  for (int vaSlot = 0; vaSlot < ms->config.vaNumAngles; vaSlot++) {
    ms->config.vaX[vaSlot] = cos(vaSlot*ms->config.vaDelta);
    ms->config.vaY[vaSlot] = sin(vaSlot*ms->config.vaDelta);
  }
  /* smoke test
  for (int vaSlot = 0; vaSlot < ms->config.vaNumAngles; vaSlot++) {
    cout << "atan2 vectorArcTan: " << 
      vectorArcTan(ms->p->config.vaY[vaSlot], ms->config.vaX[vaSlot]) << " " << 
      atan2(ms->config.vaY[vaSlot], ms->config.vaX[vaSlot]) << endl; 
  }
  */
}

void mapBlueBox(MachineState * ms, cv::Point tbTop, cv::Point tbBot, int detectedClass, ros::Time timeToMark) {
  Size sz = ms->config.objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  for (double px = tbTop.x-ms->config.mapBlueBoxPixelSkirt; px <= tbBot.x+ms->config.mapBlueBoxPixelSkirt; px++) {
    for (double py = tbTop.y-ms->config.mapBlueBoxPixelSkirt; py <= tbBot.y+ms->config.mapBlueBoxPixelSkirt; py++) {
      double x, y;
      double z = ms->config.trueEEPose.position.z + ms->config.currentTableZ;

      pixelToGlobal(ms, px, py, z, &x, &y);
      int i, j;
      mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, x, y, &i, &j);

      if (i >= 0 && i < ms->config.mapWidth && j >= 0 && j < ms->config.mapHeight) {
	ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime = timeToMark;
	ms->config.objectMap[i + ms->config.mapWidth * j].detectedClass = detectedClass;

  //      if (timeToMark - ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime > mapMemoryTimeout) {
  //        ms->config.objectMap[i + ms->config.mapWidth * j].b = 0;
  //        ms->config.objectMap[i + ms->config.mapWidth * j].g = 0;
  //        ms->config.objectMap[i + ms->config.mapWidth * j].r = 0;
  //        ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount = 0;
  //      }

	double blueBoxWeight = 0.1;
	if ( (camera->cam_bgr_img.rows != 0 && camera->cam_bgr_img.cols != 0) &&
	     ((px >=0) && (px < imW)) &&
	     ((py >=0) && (py < imH)) ) {
	  ms->config.objectMap[i + ms->config.mapWidth * j].b = (camera->cam_bgr_img.at<cv::Vec3b>(py, px)[0] * blueBoxWeight);
	  ms->config.objectMap[i + ms->config.mapWidth * j].g = (camera->cam_bgr_img.at<cv::Vec3b>(py, px)[1] * blueBoxWeight);
	  ms->config.objectMap[i + ms->config.mapWidth * j].r = (camera->cam_bgr_img.at<cv::Vec3b>(py, px)[2] * blueBoxWeight);
	  ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount = blueBoxWeight;
	}
      }
    }
  }
}

void mapBox(MachineState * ms, BoxMemory boxMemory) {
  mapBlueBox(ms, boxMemory.bTop, boxMemory.bBot, boxMemory.labeledClassIndex, ros::Time::now());
}


void globalToMapBackground(MachineState * ms, double gX, double gY, double zToUse, int * mapGpPx, int * mapGpPy) {
  double msfWidth = ms->config.mapBackgroundXMax - ms->config.mapBackgroundXMin;
  double msfHeight = ms->config.mapBackgroundYMax - ms->config.mapBackgroundYMin;

  double mapGpFractionWidth = (gX - ms->config.mapBackgroundXMin) / msfWidth;
  double mapGpFractionHeight = (gY - ms->config.mapBackgroundYMin) / msfHeight;
  *mapGpPx = floor(mapGpFractionWidth * ms->config.mbiWidth);
  *mapGpPy = floor(mapGpFractionHeight * ms->config.mbiHeight);
  *mapGpPx = min(max(0, *mapGpPx), ms->config.mbiWidth-1);
  *mapGpPy = min(max(0, *mapGpPy), ms->config.mbiHeight-1);
}

void MachineState::einStateCallback(const EinState & msg) {
  cout << "Received state msg." << endl;
}

void MachineState::rosoutCallback(const rosgraph_msgs::Log & msg) {

  if (msg.name == "/baxter_cams") {
    MachineState * ms = this;
    //cout << "Received cam msg." << msg.name << " " << msg.line << " " << msg.msg << endl;
    size_t loc = msg.msg.find(':');
    Camera * camera  = ms->config.cameras[ms->config.focused_camera];

    if (loc != std::string::npos) {
      string key = msg.msg.substr(0, loc);
      string strvalue = msg.msg.substr(loc + 2, msg.msg.length());
      if (key == "mirror" || key == "flip") {
	bool value;
	if (strvalue == "true") {
	  value = true;
	} else if (strvalue == "false") {
	  value = false;
	} else {
	  CONSOLE_ERROR(ms, "Bad message: " << msg);
	}
	//cout << "boolean key: " << key << " value: " << value << endl;
	if (key == "mirror") {
	  camera->observedCameraMirror = value;
	} else if (key == "flip") {
	  camera->observedCameraFlip = value;
	} else {
	  assert(0);
	}
      } else {
	int value;
	stringstream ss(strvalue);
	ss >> std::skipws >>  value;
	//cout << "key: " << key << " value: " << value << endl;
	if (key == "exposure") {
	  camera->observedCameraExposure = value;
	} else if (key == "gain") {
	  camera->observedCameraGain = value;
	} else if (key == "white balance red") {
	  camera->observedCameraWhiteBalanceRed = value;
	} else if (key == "white balance green") {
	  camera->observedCameraWhiteBalanceGreen = value;
	} else if (key == "white balance blue") {
	  camera->observedCameraWhiteBalanceBlue = value;
	} else if (key == "window x") {
	  camera->observedCameraWindowX = value;
	} else if (key == "window y") {
	  camera->observedCameraWindowY = value;
	} else {
	  // ignoring keys for now for now.
	}
      }


    }
  }
}

void MachineState::simulatorCallback(const ros::TimerEvent&) {

  MachineState * ms = this;

  {
    sensor_msgs::Range myRange;
    myRange.range = 0.1;
    myRange.header.stamp = ros::Time::now();
    rangeCallback(myRange);
  }
  robotEndPointCallback(ms);
  {
    double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;

    ms->config.mapBackgroundImage = ms->config.originalMapBackgroundImage.clone();
    // draw sprites on background
    if (1) {
      for (int s = 0; s < ms->config.instanceSprites.size(); s++) {
	Sprite sprite = ms->config.instanceSprites[s];
	
	int topX=0, topY=0, botX=0, botY=0;
        globalToMapBackground(ms, sprite.bot.px, sprite.bot.py, zToUse, &topX, &topY);
        globalToMapBackground(ms, sprite.top.px, sprite.top.py, zToUse, &botX, &botY);

	//cout << topX << " " << topY << " " << botX << " " << botY << endl; cout.flush();

	int localTopX = min(topX, botX);
	int localTopY = min(topY, botY);
	int localBotX = max(topX, botX);
	int localBotY = max(topY, botY);

	Mat backCrop = ms->config.mapBackgroundImage(cv::Rect(localTopX, localTopY, localBotX-localTopX, localBotY-localTopY));
	resize(sprite.image, backCrop, backCrop.size(), 0, 0, CV_INTER_LINEAR);
      }
    }

    int imW = 640;
    int imH = 400;
    Mat dummyImage(imH, imW, CV_8UC3);
    //cv::resize(ms->config.mapBackgroundImage, dummyImage, cv::Size(imW,imH));
    {
      double msfWidth = ms->config.mapBackgroundXMax - ms->config.mapBackgroundXMin;
      double msfHeight = ms->config.mapBackgroundYMax - ms->config.mapBackgroundYMin;

      double topLx = 0.0;
      double topLy = 0.0;
      pixelToGlobal(ms, 0, 0, zToUse, &topLx, &topLy);
      double botLx = 0.0;
      double botLy = 0.0;
      pixelToGlobal(ms, imW-1, imH-1, zToUse, &botLx, &botLy);
      topLx = min(max(ms->config.mapBackgroundXMin, topLx), ms->config.mapBackgroundXMax);
      topLy = min(max(ms->config.mapBackgroundYMin, topLy), ms->config.mapBackgroundYMax);
      botLx = min(max(ms->config.mapBackgroundXMin, botLx), ms->config.mapBackgroundXMax);
      botLy = min(max(ms->config.mapBackgroundYMin, botLy), ms->config.mapBackgroundYMax);
      //cout << zToUse << " z: " << endl;
      //cout << topLx << " " << topLy << " " << botLx << " " << botLy << endl;

      // account for rotation of the end effector 
      double mapGpFractionWidth = (ms->config.currentEEPose.px - ms->config.mapBackgroundXMin) / msfWidth;
      double mapGpFractionHeight = (ms->config.currentEEPose.py - ms->config.mapBackgroundYMin) / msfHeight;
      int mapGpPx = floor(mapGpFractionWidth * ms->config.mbiWidth);
      int mapGpPy = floor(mapGpFractionHeight * ms->config.mbiHeight);
      mapGpPx = min(max(0, mapGpPx), ms->config.mbiWidth-1);
      mapGpPy = min(max(0, mapGpPy), ms->config.mbiHeight-1);

      Quaternionf eeqform(ms->config.currentEEPose.qw, ms->config.currentEEPose.qx, ms->config.currentEEPose.qy, ms->config.currentEEPose.qz);
      Quaternionf crane2Orient(0, 1, 0, 0);
      Quaternionf rel = eeqform * crane2Orient.inverse();
      Quaternionf ex(0,1,0,0);
      Quaternionf zee(0,0,0,1);
	    
      Quaternionf result = rel * ex * rel.conjugate();
      Quaternionf thumb = rel * zee * rel.conjugate();
      double aY = result.y();
      double aX = result.x();

      // ATTN 22
      //double angle = atan2(aY, aX)*180.0/3.1415926;
      double angle = vectorArcTan(ms, aY, aX)*180.0/3.1415926;
      angle = (angle);
      double scale = 1.0;
      Point center = Point(mapGpPx, mapGpPy);

      Mat un_rot_mat = getRotationMatrix2D( center, angle, scale );

      if (0) {
	// un_rot_mat = [[0 1 0]' [1 0 0]' [0 0 1]'] * un_rot_mat
	cout << un_rot_mat.size() << endl;
	float tmp = 0.0;
	tmp = un_rot_mat.at<float>(1,0);
	un_rot_mat.at<float>(1,0) = un_rot_mat.at<float>(0,0);
	un_rot_mat.at<float>(0,0) = tmp;

	tmp = un_rot_mat.at<float>(1,1);
	un_rot_mat.at<float>(1,1) = un_rot_mat.at<float>(0,1);
	un_rot_mat.at<float>(0,1) = tmp;
      }

      double mapStartFractionWidth = (topLx - ms->config.mapBackgroundXMin) / msfWidth;
      double mapStartFractionHeight = (topLy - ms->config.mapBackgroundYMin) / msfHeight;
      double mapEndFractionWidth = (botLx - ms->config.mapBackgroundXMin) / msfWidth;
      double mapEndFractionHeight = (botLy - ms->config.mapBackgroundYMin) / msfHeight;

      //cout << "iii: " << mapStartFractionWidth << " " << mapStartFractionHeight << " " << mapEndFractionWidth << " " << mapEndFractionHeight << endl; cout.flush();

      int mapStartPx = floor(mapStartFractionWidth * ms->config.mbiWidth);
      int mapStartPy = floor(mapStartFractionHeight * ms->config.mbiHeight);
      int mapEndPx = floor(mapEndFractionWidth * ms->config.mbiWidth);
      int mapEndPy = floor(mapEndFractionHeight * ms->config.mbiHeight);
      mapStartPx = min(max(0, mapStartPx), ms->config.mbiWidth-1);
      mapStartPy = min(max(0, mapStartPy), ms->config.mbiHeight-1);
      mapEndPx = min(max(0, mapEndPx), ms->config.mbiWidth-1);
      mapEndPy = min(max(0, mapEndPy), ms->config.mbiHeight-1);

      //cout << "jjj: " << mapStartPx << " " << mapStartPy << " " << mapEndPx << " " << mapEndPy << endl; cout.flush();

      int rotTopPx = 0.0;
      int rotTopPy = 0.0;
      int rotBotPx = 0.0;
      int rotBotPy = 0.0;
      {
	Mat toUn(3,1,CV_64F);
	toUn.at<double>(0,0)=mapStartPx;
	toUn.at<double>(1,0)=mapStartPy;
	toUn.at<double>(2,0)=1.0;
	Mat didUn = un_rot_mat*toUn;
	rotTopPx = floor(didUn.at<double>(0,0));
	rotTopPy = floor(didUn.at<double>(1,0));
      }
      {
	Mat toUn(3,1,CV_64F);
	toUn.at<double>(0,0)=mapEndPx;
	toUn.at<double>(1,0)=mapEndPy;
	toUn.at<double>(2,0)=1.0;
	Mat didUn = un_rot_mat*toUn;
	rotBotPx = floor(didUn.at<double>(0,0));
	rotBotPy = floor(didUn.at<double>(1,0));
      }
      rotTopPx = min(max(0, rotTopPx), ms->config.mbiWidth-1);
      rotTopPy = min(max(0, rotTopPy), ms->config.mbiHeight-1);
      rotBotPx = min(max(0, rotBotPx), ms->config.mbiWidth-1);
      rotBotPy = min(max(0, rotBotPy), ms->config.mbiHeight-1);

      int topPx = 0.0;
      int topPy = 0.0;
      globalToPixel(ms, &topPx, &topPy, zToUse, topLx, topLy);
      int botPx = 0.0;
      int botPy = 0.0;
      globalToPixel(ms, &botPx, &botPy, zToUse, botLx, botLy);
      topPx = min(max(0, topPx), imW-1);
      topPy = min(max(0, topPy), imH-1);
      botPx = min(max(0, botPx), imW-1);
      botPy = min(max(0, botPy), imH-1);

      //cout << "jja: " << rotTopPx << " " << rotTopPy << " " << rotBotPx << " " << rotBotPy << endl; cout.flush();
      //cout << "jjb: " << topPx << " " << topPy << " " << botPx << " " << botPy << endl; cout.flush();

      // this switch happens due to the default handedness and rotation of the camera
      if (0) {
	int tmp;
	tmp = rotTopPx;
	rotTopPx = rotBotPx;
	rotBotPx = tmp;
	//tmp = rotTopPy;
	//rotTopPy = rotBotPy;
	//rotBotPy = tmp;
      }

      int localRotTopPx = min(rotTopPx, rotBotPx);
      int localRotTopPy = min(rotTopPy, rotBotPy);
      int localRotBotPx = max(rotTopPx, rotBotPx);
      int localRotBotPy = max(rotTopPy, rotBotPy);

      //cout << "jjc: " << rotTopPx << " " << rotTopPy << " " << rotBotPx << " " << rotBotPy << endl; cout.flush();
      //cout << "jjd: " << localRotTopPx << " " << localRotTopPy << " " << localRotBotPx << " " << localRotBotPy << endl; cout.flush();
      //eePose::print(ms->config.currentEEPose);

      Mat rotatedBackMapImage;
      warpAffine(ms->config.mapBackgroundImage, rotatedBackMapImage, un_rot_mat, ms->config.mapBackgroundImage.size(), INTER_LINEAR, BORDER_REPLICATE);

      //Mat backgroundMapCrop = rotatedBackMapImage(cv::Rect(rotTopPx, rotTopPy, rotBotPx-rotTopPx, rotBotPy-rotTopPy));
      Mat backgroundMapCrop = rotatedBackMapImage(cv::Rect(localRotTopPx, localRotTopPy, localRotBotPx-localRotTopPx, localRotBotPy-localRotTopPy));
      transpose(backgroundMapCrop, backgroundMapCrop);
      //Mat backgroundMapCrop = rotatedBackMapImage(cv::Rect(localRotTopPy, localRotTopPx, localRotBotPy-localRotTopPy, localRotBotPx-localRotTopPx));


      //Mat backgroundMapCrop = rotatedBackMapImage(cv::Rect(localRotTopPx, localRotTopPy, localRotBotPx-localRotTopPx, localRotBotPy-localRotTopPy));
      Mat screenCrop = dummyImage(cv::Rect(topPx, topPy, botPx-topPx, botPy-topPy));
      resize(backgroundMapCrop, screenCrop, screenCrop.size(), 0, 0, CV_INTER_LINEAR);


      // XXX rotate the background so that it is aligned with the camera
      // resize the relevant crop into the image crop

//      if (0) {
//	int localMapStartPx = min(mapStartPx, mapEndPx);
//	int localMapStartPy = min(mapStartPy, mapEndPy);
//	int localMapEndPx = max(mapStartPx, mapEndPx);
//	int localMapEndPy = max(mapStartPy, mapEndPy);
//	
//	Mat backgroundMapCrop = ms->config.mapBackgroundImage(cv::Rect(localMapStartPx, localMapStartPy, localMapEndPx-localMapStartPx, localMapEndPy-localMapStartPy));
//	Mat screenCrop = dummyImage(cv::Rect(topPx, topPy, botPx-topPx, botPy-topPy));
//	resize(backgroundMapCrop, screenCrop, screenCrop.size(), 0, 0, CV_INTER_LINEAR);
//      }

//      Size sz = screenCrop.size();
//      int cropW = sz.width;
//      int cropH = sz.height;
//      Vec3b thisColor = cv::Vec<uchar, 3>(128,128,128);
//      //screenCrop += thisColor;
//      for (int y = 0; y < cropH; y++) {
//	for (int x = 0; x < cropW; x++) {
//	  screenCrop.at<Vec3b>(y,x) = thisColor;
//	}
//      }
    }


    sensor_msgs::ImagePtr myImagePtr(new sensor_msgs::Image());
    myImagePtr->header.stamp = ros::Time::now();
    myImagePtr->width = dummyImage.cols;
    myImagePtr->height = dummyImage.rows;
    myImagePtr->step = dummyImage.cols * dummyImage.elemSize();
    myImagePtr->is_bigendian = false;
    myImagePtr->encoding = sensor_msgs::image_encodings::BGR8;
    myImagePtr->data.assign(dummyImage.data, dummyImage.data + size_t(dummyImage.rows * myImagePtr->step));
    for (int i = 0; i < ms->config.cameras.size(); i++) {
      ms->config.cameras[i]->imageCallback(myImagePtr);
    }
  }


}

bool isInGripperMaskBlocks(MachineState * ms, int x, int y) {
  if ( (x >= ms->config.g1xs && x <= ms->config.g1xe && y >= ms->config.g1ys && y <= ms->config.g1ye) ||
       (x >= ms->config.g2xs && x <= ms->config.g2xe && y >= ms->config.g2ys && y <= ms->config.g2ye) ) {
    return true;
  } else {
    return false;
  }
}

bool isInGripperMask(MachineState * ms, int x, int y) {
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  if (ms->config.mask_gripper) {
    if (isSketchyMat(camera->gripperMask)) {
      return false;
    } else {
      return (( camera->gripperMask.at<uchar>(y,x) == 0 ));
    }
  } else if (ms->config.mask_gripper_blocks) {
    return isInGripperMaskBlocks(ms, x,y);
  } else {
    return false;
  }
}

void findDarkness(MachineState * ms, int * xout, int * yout) {
  //*xout = vanishingPointReticle.px;
  //*yout = vanishingPointReticle.py;
  findOptimum(ms, xout, yout, -1);
}

void findLight(MachineState * ms, int * xout, int * yout) {
  findOptimum(ms, xout, yout, 1);
}

void findOptimum(MachineState * ms, int * xout, int * yout, int sign) {

  if (isSketchyMat(ms->config.accumulatedImage)) {
    ROS_ERROR("Whoops, accumulatedImage is sketchy, returning vanishing point to findOptimum.");
    Camera * camera  = ms->config.cameras[ms->config.focused_camera];
    *xout = camera->vanishingPointReticle.px;
    *yout = camera->vanishingPointReticle.py;
    return;
  }

  Size sz = ms->config.accumulatedImage.size();
  int imW = sz.width;
  int imH = sz.height;

  int maxX = 0;
  int maxY = 0;

  // this should be -INF regardless of sign because we
  //  always use > to compare
  double maxVal = -INFINITY;

  Mat accToBlur;
  accToBlur.create(ms->config.accumulatedImage.size(), CV_64F);

  //int xmin = ms->config.grayTop.x;
  //int ymin = ms->config.grayTop.y;
  //int xmax = ms->config.grayBot.x;
  //int ymax = ms->config.grayBot.y;
  //int xmin = 0;
  //int ymin = 0;
  //int xmax = imW;
  //int ymax = imH;
  int findPadX = 100;
  int findPadY = 50;
  int xmin = 0+findPadX;
  int ymin = 0+findPadY;
  int xmax = imW-findPadX;
  int ymax = imH-findPadY;

  xmax = min(max(0, xmax), imW); 
  xmin = min(max(0, xmin), imW); 
  ymax = min(max(0, ymax), imH); 
  ymin = min(max(0, ymin), imH); 

  for (int x = xmin; x < xmax; x++) {
    for (int y = ymin; y < ymax; y++) {
      double thisVal = 
      ( (ms->config.accumulatedImage.at<Vec3d>(y,x)[0]*
         ms->config.accumulatedImage.at<Vec3d>(y,x)[0])+
        (ms->config.accumulatedImage.at<Vec3d>(y,x)[1]*
         ms->config.accumulatedImage.at<Vec3d>(y,x)[1])+
        (ms->config.accumulatedImage.at<Vec3d>(y,x)[2]*
         ms->config.accumulatedImage.at<Vec3d>(y,x)[2]) );
      accToBlur.at<double>(y,x) = thisVal;
    }
  }
  // XXX TODO
  // This should probably be in YCbCr space with the Y channel
  // blurring helps with a noisy image but can kill off small targets
  //double darkSigma = 1.0;
  //GaussianBlur(accToBlur, accToBlur, cv::Size(0,0), darkSigma, BORDER_REFLECT);
  for (int x = xmin; x < xmax; x++) {
    for (int y = ymin; y < ymax; y++) {
      double thisVal = sign * accToBlur.at<double>(y,x);
      if ((!isInGripperMask(ms, x,y)) && (thisVal > maxVal)) {
	maxVal = thisVal;
	maxX = x;
	maxY = y;
      }
    }
  }

  maxX = min(max(xmin, maxX), xmax); 
  maxY = min(max(ymin, maxY), ymax); 

  *xout = maxX;
  *yout = maxY;
}

void fillLocalUnitBasis(eePose localFrame, Vector3d * localUnitX, Vector3d * localUnitY, Vector3d * localUnitZ) {
  {
    Eigen::Quaternionf qin(0, 1, 0, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(localFrame.qw, localFrame.qx, localFrame.qy, localFrame.qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitX->x() = qout.x();
    localUnitX->y() = qout.y();
    localUnitX->z() = qout.z();
  }
  {
    Eigen::Quaternionf qin(0, 0, 1, 0);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(localFrame.qw, localFrame.qx, localFrame.qy, localFrame.qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitY->x() = qout.x();
    localUnitY->y() = qout.y();
    localUnitY->z() = qout.z();
  }
  {
    Eigen::Quaternionf qin(0, 0, 0, 1);
    Eigen::Quaternionf qout(0, 1, 0, 0);
    Eigen::Quaternionf eeqform(localFrame.qw, localFrame.qx, localFrame.qy, localFrame.qz);
    qout = eeqform * qin * eeqform.conjugate();
    localUnitZ->x() = qout.x();
    localUnitZ->y() = qout.y();
    localUnitZ->z() = qout.z();
  }
}

////////////////////////////////////////////////
// end pilot definitions 
//
// start node definitions 
////////////////////////////////////////////////

int doubleToByte(double in) {
  return min(max(round(in),0.0),255.0);
}



void gridKeypoints(MachineState * ms, int gImW, int gImH, cv::Point top, cv::Point bot, int strideX, int strideY, vector<KeyPoint>& keypoints, int period) {
  keypoints.resize(0);

  // make sure feature pad is a multiple of the stride
  int featurePadX = strideX;
  int featurePadY = strideY;

  cv::Point sTop(featurePadX, featurePadY);
  cv::Point sBot(strideX*(((bot.x-top.x-featurePadX)/strideX)-2), strideY*(((bot.y-top.y-featurePadY)/strideY)-2));

  int responseIndex = 1;
  
  int mX = ms->config.gBoxW / 2;
  int mY = ms->config.gBoxH / 2;


  for (int y = sTop.y; y <= sBot.y; y+=strideX*period) {
    for (int x = sTop.x; x <= sBot.x; x+=strideY*period) {
      KeyPoint thisKeypoint;
      thisKeypoint.angle = -1;
      thisKeypoint.class_id = -1;
      thisKeypoint.octave = 0;
      thisKeypoint.pt.x = x+mX;
      thisKeypoint.pt.y = y+mY;
      thisKeypoint.response = responseIndex;
      thisKeypoint.size = min(ms->config.gBoxW, ms->config.gBoxH);
      responseIndex++;

      double param_kpGreenThresh = 0;
      //const double param_kpProb = 0.1;
      const double param_kpProb = 1.0;

      if (gImW > 0) {
	if ((ms->config.pBoxIndicator[(top.y+y)*gImW+(top.x+x)] >= param_kpGreenThresh) && (drand48() < param_kpProb)) {
	  keypoints.push_back(thisKeypoint);
	}
      } else {
	if (drand48() < param_kpProb) {
	  keypoints.push_back(thisKeypoint);
	}
      }
      
      //if ((ms->config.pBoxIndicator[(top.y+y)*gImW+(top.x+x)] >= kpGreenThresh))
	//keypoints.push_back(thisKeypoint);
    }
  }

}


bool isFiniteNumber(double x) {
    return (x <= DBL_MAX && x >= -DBL_MAX); 
} 

void appendColorHist(Mat& yCrCb_image, vector<KeyPoint>& keypoints, Mat& descriptors, Mat& descriptors2) {
  
  // paramaters for the color histogram feature
  // ATTN 25
  //const double colorHistNumBins = 8;
  double colorHistNumBins = 16;
  double colorHistBinWidth = 256/colorHistNumBins;
  double colorHistLambda = 1.0;//0.5;
  double colorHistThresh = 0.1;
  // ATTN 25
  //int colorHistBoxHalfWidth = 1;
  int colorHistBoxHalfWidth = 2;


  Size sz = yCrCb_image.size();
  int imW = sz.width;
  int imH = sz.height;

  Mat colorHist(1, colorHistNumBins*colorHistNumBins, descriptors.type());
  for (int i = 0; i < colorHistNumBins*colorHistNumBins; i++) 
    colorHist.at<float>(i) = 0;

  double numPix = 0;
  // traverse all of the keypoints
  for (int kk = 0; kk < keypoints.size(); kk++) {
    // count pixels in a neighborhood of the keypoint
    int yMin = max(int(keypoints[kk].pt.y)-colorHistBoxHalfWidth,0);
    int yMax = min(int(keypoints[kk].pt.y)+colorHistBoxHalfWidth,imH);
    int xMin = max(int(keypoints[kk].pt.x)-colorHistBoxHalfWidth,0);
    int xMax = min(int(keypoints[kk].pt.x)+colorHistBoxHalfWidth,imW);
    for (int y = yMin; y < yMax; y++) {
      for (int x = xMin; x < xMax; x++) {

	cv::Vec3b thisColor = yCrCb_image.at<cv::Vec3b>(y,x);

	int CrBin = thisColor[1]/colorHistBinWidth;
	int CbBin = thisColor[2]/colorHistBinWidth;

	colorHist.at<float>(CrBin + CbBin*colorHistNumBins)++;

	numPix++;
      }
    }
  }
  // normalize the histogram
  for (int i = 0; i < colorHistNumBins*colorHistNumBins; i++) 
    colorHist.at<float>(i) = colorHist.at<float>(i) / numPix;

  descriptors2 = Mat(1, descriptors.size().width + colorHistNumBins*colorHistNumBins, descriptors.type());
  for (int i = 0; i < descriptors.size().width; i++) 
    descriptors2.at<float>(i) = descriptors.at<float>(i);
  for (int i = 0; i < colorHistNumBins*colorHistNumBins; i++) {
    colorHist.at<float>(i) = min(colorHist.at<float>(i), float(colorHistThresh));
    descriptors2.at<float>(i+descriptors.size().width) = colorHistLambda * colorHist.at<float>(i);
  }
}

void processImage(Mat &image, Mat& gray_image, Mat& yCrCb_image, double sigma) {
  cvtColor(image, gray_image, CV_BGR2GRAY);
  cvtColor(image, yCrCb_image, CV_BGR2YCrCb);
  GaussianBlur(gray_image, gray_image, cv::Size(0,0), sigma);
  GaussianBlur(yCrCb_image, yCrCb_image, cv::Size(0,0), sigma);
}

void bowGetFeatures(MachineState * ms, std::string classDir, const char *className, double sigma, int keypointPeriod, int * grandTotalDescriptors, DescriptorExtractor * extractor, BOWKMeansTrainer * bowTrainer) {

  int totalDescriptors = 0;
  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  char buf[1024];
  sprintf(buf, "%s%s/ein/detectionCrops/", classDir.c_str(), className);
  dpdf = opendir(buf);
  if (dpdf != NULL){
    while (epdf = readdir(dpdf)){
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

        vector<KeyPoint> keypoints1;
        vector<KeyPoint> keypoints2;
        Mat descriptors;

        char filename[1024];
        sprintf(filename, "%s%s/ein/detectionCrops/%s", classDir.c_str(), className, epdf->d_name);
        Mat image;
        image = imread(filename);
	Size sz = image.size();
	int cropW = sz.width;
	int cropH = sz.height;
	cv::Point bot(cropW, cropH);

        Mat gray_image;
        Mat yCrCb_image;
	processImage(image, gray_image, yCrCb_image, sigma);


        // if you add an immense number of examples or some new classes and
        //   you begin having discriminative problems (confusion), you can
        //   increase the number of words.
        // ATTN 25
        //const double bowSubSampleFactor = 0.05;//0.02;//0.01;
        double param_bowSubSampleFactor = 0.10;
        int param_bowOverSampleFactor = 1;
	for (int i = 0; i < param_bowOverSampleFactor; i++) {
	  //ms->config.detector->detect(gray_image, keypoints1);
	  gridKeypoints(ms, 0, 0, cv::Point(0,0), bot, ms->config.gBoxStrideX, ms->config.gBoxStrideY, keypoints1, keypointPeriod);
	  for (int kp = 0; kp < keypoints1.size(); kp++) {
	    if (drand48() < param_bowSubSampleFactor)
	      keypoints2.push_back(keypoints1[kp]);
	  }
	  extractor->compute(gray_image, keypoints2, descriptors);

	  totalDescriptors += int(descriptors.rows);
	  *grandTotalDescriptors += int(descriptors.rows);
	  cout << className << ":  "  << epdf->d_name << "  " << descriptors.size() << " total descriptors: " << totalDescriptors << endl;

	  if (!descriptors.empty() && !keypoints2.empty())
	    bowTrainer->add(descriptors);
	}
      }
    }
  }
}

void kNNGetFeatures(MachineState * ms, std::string classDir, const char *className, int label, double sigma, Mat &kNNfeatures, Mat &kNNlabels, double sobel_sigma) {

  int param_kNNOverSampleFactor = 1;

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  char buf[1024];
  sprintf(buf, "%s%s/ein/detectionCrops/", classDir.c_str(), className);
  dpdf = opendir(buf);
  if (dpdf != NULL){
    while (epdf = readdir(dpdf)){
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

        vector<KeyPoint> keypoints;
        Mat descriptors;
        Mat descriptors2;

        char filename[1024];
        sprintf(filename, "%s%s/ein/detectionCrops/%s", classDir.c_str(), className, epdf->d_name);
        Mat image;
        image = imread(filename);
	Size sz = image.size();
	int cropW = sz.width;
	int cropH = sz.height;
	cv::Point bot(cropW, cropH);

        Mat gray_image;
        Mat yCrCb_image;

	//if ((ms->config.chosen_feature == SIFTBOW_GLOBALCOLOR_HIST) || (ms->config.chosen_feature == OPPONENTSIFTBOW_GLOBALCOLOR_HIST))
	if (ms->config.chosen_feature == SIFTBOW_GLOBALCOLOR_HIST) 
	{
	  processImage(image, gray_image, yCrCb_image, sigma);
	  for (int i = 0; i < param_kNNOverSampleFactor; i++) {
	    //ms->config.detector->detect(gray_image, keypoints);
	    gridKeypoints(ms, 0, 0, cv::Point(0,0), bot, ms->config.gBoxStrideX, ms->config.gBoxStrideY, keypoints, ms->config.keypointPeriod);
	    ms->config.bowExtractor->compute(gray_image, keypoints, descriptors);

	    cout << className << ":  "  << epdf->d_name << "  " << descriptors.size() << " type: " << descriptors.type() << " tot: " << kNNfeatures.size() << endl;

	    if (!descriptors.empty() && !keypoints.empty()) {
	      appendColorHist(yCrCb_image, keypoints, descriptors, descriptors2);

	      kNNfeatures.push_back(descriptors2);
	      kNNlabels.push_back(label);
	    }
	  }
	} else if (ms->config.chosen_feature == OPPONENTSIFTBOW_GLOBALCOLOR_HIST) {
	  processImage(image, gray_image, yCrCb_image, sigma);
	  for (int i = 0; i < param_kNNOverSampleFactor; i++) {
	    //ms->config.detector->detect(gray_image, keypoints);
	    gridKeypoints(ms, 0, 0, cv::Point(0,0), bot, ms->config.gBoxStrideX, ms->config.gBoxStrideY, keypoints, ms->config.keypointPeriod);
	    //ms->config.bowExtractor->compute(gray_image, keypoints, descriptors);

	    Mat tmpC;
	    image.convertTo(tmpC, CV_32FC3);
	    ms->config.bowExtractor->compute(tmpC, keypoints, descriptors);

	    cout << className << ":  "  << epdf->d_name << "  " << descriptors.size() << " type: " << descriptors.type() << " tot: " << kNNfeatures.size() << endl;

	    if (!descriptors.empty() && !keypoints.empty()) {
	      appendColorHist(yCrCb_image, keypoints, descriptors, descriptors2);

	      kNNfeatures.push_back(descriptors2);
	      kNNlabels.push_back(label);
	    }
	  }
	} else if (ms->config.chosen_feature == GRADIENT) {
	  processImage(image, gray_image, yCrCb_image, sobel_sigma);

	  Mat totalGraySobel;
	  {
	    Mat grad_x, grad_y;
	    int sobelScale = 1;
	    int sobelDelta = 0;
	    int sobelDepth = CV_32F;
	    /// Gradient X
	    Sobel(gray_image, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
	    /// Gradient Y
	    Sobel(gray_image, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

	    grad_x = grad_x.mul(grad_x);
	    grad_y = grad_y.mul(grad_y);
	    totalGraySobel = grad_x + grad_y;
	    // now totalGraySobel is gradient magnitude squared
	  }

	  // grow to the max dimension to avoid distortion
	  // find the dimensions that pad the sobel image up to a square
	  // raster scan a 'virtual image' to generate the 1D vector, adding 0's when on the pad

	  int crows = totalGraySobel.rows;
	  int ccols = totalGraySobel.cols;
	  int maxDim = max(crows, ccols);
	  int tRy = (maxDim-crows)/2;
	  int tRx = (maxDim-ccols)/2;
	  Mat gCrop(maxDim, maxDim, totalGraySobel.type());

	  float totalMass = 0.0;

	  for (int x = 0; x < maxDim; x++) {
	    for (int y = 0; y < maxDim; y++) {
	      int tx = x - tRx;
	      int ty = y - tRy;
	      if (tx >= 0 && ty >= 0 && ty < crows && tx < ccols) {
		gCrop.at<float>(y, x) = totalGraySobel.at<float>(ty, tx);
		//totalMass += gCrop.at<float>(y, x);
		totalMass += gCrop.at<float>(y, x) * gCrop.at<float>(y, x);
	      } else {
		gCrop.at<float>(y, x) = 0.0;
	      }
	    }
	  }
	  totalMass = sqrt(totalMass);
	  Mat descriptorsG = Mat(1, ms->config.gradientFeatureWidth*ms->config.gradientFeatureWidth, CV_32F);
	  for (int y = 0; y < ms->config.gradientFeatureWidth; y++) {
	    for (int x = 0; x < ms->config.gradientFeatureWidth; x++) {
	      int tranX = floor(float(x)*float(maxDim)/float(ms->config.gradientFeatureWidth));
	      int tranY = floor(float(y)*float(maxDim)/float(ms->config.gradientFeatureWidth));
	      //descriptorsG.at<float>(x + y*ms->config.gradientFeatureWidth) = gCrop.at<float>(y,x);
	      descriptorsG.at<float>(x + y*ms->config.gradientFeatureWidth) = gCrop.at<float>(y,x)/totalMass;
	    }
	  }
	  kNNfeatures.push_back(descriptorsG);
	  kNNlabels.push_back(label);
	} else if (ms->config.chosen_feature == OPPONENT_COLOR_GRADIENT) {
	  processImage(image, gray_image, yCrCb_image, sobel_sigma);

	  Mat totalGraySobel;
	  {
	    Mat grad_x, grad_y;
	    int sobelScale = 1;
	    int sobelDelta = 0;
	    int sobelDepth = CV_32F;
	    /// Gradient X
	    Sobel(gray_image, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
	    /// Gradient Y
	    Sobel(gray_image, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

	    grad_x = grad_x.mul(grad_x);
	    grad_y = grad_y.mul(grad_y);
	    totalGraySobel = grad_x + grad_y;
	    // now totalGraySobel is gradient magnitude squared
	  }
	  Mat totalCrSobel = totalGraySobel.clone();
	  {
	    for (int y = 0; y < image.rows; y++) {
	      for (int x = 0; x < image.cols; x++) {
		cv::Vec3b thisColor = yCrCb_image.at<cv::Vec3b>(y,x);
		totalCrSobel.at<float>(y,x) = thisColor[1];
	      }
	    }
	    Mat grad_x, grad_y;
	    int sobelScale = 1;
	    int sobelDelta = 0;
	    int sobelDepth = CV_32F;
	    /// Gradient X
	    Sobel(totalCrSobel, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
	    /// Gradient Y
	    Sobel(totalCrSobel, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

	    grad_x = grad_x.mul(grad_x);
	    grad_y = grad_y.mul(grad_y);
	    totalCrSobel = grad_x + grad_y;
	  }
	  Mat totalCbSobel = totalGraySobel.clone();
	  {
	    for (int y = 0; y < image.rows; y++) {
	      for (int x = 0; x < image.cols; x++) {
		cv::Vec3b thisColor = yCrCb_image.at<cv::Vec3b>(y,x);
		totalCbSobel.at<float>(y,x) = thisColor[2];
	      }
	    }
	    Mat grad_x, grad_y;
	    int sobelScale = 1;
	    int sobelDelta = 0;
	    int sobelDepth = CV_32F;
	    /// Gradient X
	    Sobel(totalCbSobel, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
	    /// Gradient Y
	    Sobel(totalCbSobel, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

	    grad_x = grad_x.mul(grad_x);
	    grad_y = grad_y.mul(grad_y);
	    totalCbSobel = grad_x + grad_y;
	  }

	  // grow to the max dimension to avoid distortion
	  // find the dimensions that pad the sobel image up to a square
	  // raster scan a 'virtual image' to generate the 1D vector, adding 0's when on the pad

	  int crows = totalGraySobel.rows;
	  int ccols = totalGraySobel.cols;
	  int maxDim = max(crows, ccols);
	  int tRy = (maxDim-crows)/2;
	  int tRx = (maxDim-ccols)/2;
	  Mat gCrop(maxDim, maxDim, totalGraySobel.type());
	  Mat crCrop(maxDim, maxDim, totalGraySobel.type());
	  Mat cbCrop(maxDim, maxDim, totalGraySobel.type());

	  float totalGMass = 0.0;
	  float totalCrMass = 0.0;
	  float totalCbMass = 0.0;

	  for (int x = 0; x < maxDim; x++) {
	    for (int y = 0; y < maxDim; y++) {
	      int tx = x - tRx;
	      int ty = y - tRy;
	      if (tx >= 0 && ty >= 0 && ty < crows && tx < ccols) {

		// ATTN 24
		// XXX
		crCrop.at<float>(y,x) = yCrCb_image.at<Vec3b>(y,x)[1];
		cbCrop.at<float>(y,x) = yCrCb_image.at<Vec3b>(y,x)[2];

		gCrop.at<float>(y, x) = totalGraySobel.at<float>(ty, tx);
		crCrop.at<float>(y, x) = totalCrSobel.at<float>(ty, tx);
		cbCrop.at<float>(y, x) = totalCbSobel.at<float>(ty, tx);
		//totalGMass += gCrop.at<float>(y, x);
		totalGMass += gCrop.at<float>(y, x) * gCrop.at<float>(y, x);
		//totalCrMass += gCrop.at<float>(y, x) * gCrop.at<float>(y, x);
		//totalCbMass += gCrop.at<float>(y, x) * gCrop.at<float>(y, x);
		totalCrMass += crCrop.at<float>(y, x) * crCrop.at<float>(y, x);
		totalCbMass += cbCrop.at<float>(y, x) * cbCrop.at<float>(y, x);
	      } else {
		gCrop.at<float>(y, x) = 0.0;
	      }
	    }
	  }
	  totalGMass = sqrt(totalGMass);
	  totalCrMass = sqrt(totalCrMass);
	  totalCbMass = sqrt(totalCbMass);
	  double totalColorMass = totalCrMass + totalCbMass;
	  //Mat descriptorsG = Mat(1, ms->config.gradientFeatureWidth*ms->config.gradientFeatureWidth, CV_32F);
	  Mat descriptorsCbCr = Mat(1, 2*ms->config.gradientFeatureWidth*ms->config.gradientFeatureWidth, CV_32F);
	  for (int y = 0; y < ms->config.gradientFeatureWidth; y++) {
	    for (int x = 0; x < ms->config.gradientFeatureWidth; x++) {
	      int tranX = floor(float(x)*float(maxDim)/float(ms->config.gradientFeatureWidth));
	      int tranY = floor(float(y)*float(maxDim)/float(ms->config.gradientFeatureWidth));
	      //descriptorsG.at<float>(x + y*ms->config.gradientFeatureWidth) = gCrop.at<float>(y,x);
	      //descriptorsCbCr.at<float>(x + y*ms->config.gradientFeatureWidth) = crCrop.at<float>(y,x)/totalCrMass;
	      //descriptorsCbCr.at<float>(x + y*ms->config.gradientFeatureWidth + ms->config.gradientFeatureWidth*ms->config.gradientFeatureWidth) = cbCrop.at<float>(y,x)/totalCbMass;
	      //descriptorsCbCr.at<float>(x + y*ms->config.gradientFeatureWidth) = crCrop.at<float>(y,x);
	      //descriptorsCbCr.at<float>(x + y*ms->config.gradientFeatureWidth + ms->config.gradientFeatureWidth*ms->config.gradientFeatureWidth) = cbCrop.at<float>(y,x);

	      descriptorsCbCr.at<float>(x + y*ms->config.gradientFeatureWidth) = crCrop.at<float>(y,x)/totalColorMass;
	      descriptorsCbCr.at<float>(x + y*ms->config.gradientFeatureWidth + ms->config.gradientFeatureWidth*ms->config.gradientFeatureWidth) = cbCrop.at<float>(y,x)/totalColorMass;
	    }
	  }
	  cout << descriptorsCbCr << endl;

	  kNNfeatures.push_back(descriptorsCbCr);
	  kNNlabels.push_back(label);
	}
      }
    }
  }
}

void posekNNGetFeatures(MachineState * ms, std::string classDir, const char *className, double sigma, Mat &kNNfeatures, Mat &kNNlabels,
                        vector< cv::Vec<double,4> >& classQuaternions, int keypointPeriod, BOWImgDescriptorExtractor *bowExtractor, int lIndexStart) {

  string sClassName(className);

  int label = 0;

  int lIndex = lIndexStart;

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");
  string png(".png");

  char buf[1024];
  sprintf(buf, "%s%s/rgbPose", classDir.c_str(), className);
  dpdf = opendir(buf);
  if (dpdf != NULL){
    while (epdf = readdir(dpdf)){
      string fileName(epdf->d_name);
      cout << fileName << " " << endl;
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

	string fext = fileName.substr(fileName.size()-4, 4);
	if (fext.compare(png))
	  continue;

	//string poseIndex = fileName.substr(sClassName.size()+1, string::npos);
	//poseIndex = poseIndex.substr(0,  poseIndex.length()-4);
	//label = std::atoi(poseIndex.c_str());

	// remove .png to form key
	string thisCropLabel = fileName.substr(0,fileName.size()-4);
	string poseLabelsPath =  classDir + className + "/poseLabels.yml";

	cv::Vec<double,4> tLQ;

	FileStorage fsfI;
	fsfI.open(poseLabelsPath, FileStorage::READ);
	fsfI[thisCropLabel] >> tLQ; 
	fsfI.release();

        vector<KeyPoint> keypoints;
        Mat descriptors;
	Mat descriptors2;

        char filename[1024];
        sprintf(filename, "%s%s/rgbPose/%s", classDir.c_str(), className, epdf->d_name);
        Mat image;
        image = imread(filename);
	Size sz = image.size();
	int cropW = sz.width;
	int cropH = sz.height;
	cv::Point bot(cropW, cropH);
	
        Mat gray_image;
        Mat yCrCb_image;
	processImage(image, gray_image, yCrCb_image, sigma);
        int param_poseOverSampleFactor = 1;
	for (int i = 0; i < param_poseOverSampleFactor; i++) {
	  //ms->config.detector->detect(gray_image, keypoints);
	  gridKeypoints(ms, 0, 0, cv::Point(0,0), bot, ms->config.gBoxStrideX, ms->config.gBoxStrideY, keypoints, keypointPeriod);
	  bowExtractor->compute(gray_image, keypoints, descriptors);

	  cout << className << ":  "  << epdf->d_name << "  "  << fileName << " " << descriptors.size() << 
	    " type: " << descriptors.type() << " label: " << label << endl;

	  if (!descriptors.empty() && !keypoints.empty()) {
	    appendColorHist(yCrCb_image, keypoints, descriptors, descriptors2);
	    kNNfeatures.push_back(descriptors);
	    //kNNlabels.push_back(label);
	    kNNlabels.push_back(lIndex);
	    classQuaternions.push_back(tLQ);
	    lIndex++;
	  }
	}
      }
    }
  }
}

void goFindBlueBoxes(MachineState * ms) {
  Size sz = ms->config.objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  ms->config.gBoxIndicator = new double[imW*imH];
  double *gBoxGrayNodes = new double[imW*imH];
  double *gBoxComponentLabels = new double[imW*imH];
  if (ms->config.pBoxIndicator == NULL)
    ms->config.pBoxIndicator = new double[imW*imH];

  vector<int> parentX;
  vector<int> parentY;
  vector<int> parentD;
  
  ms->config.cTops.resize(0);
  ms->config.cBots.resize(0);

  const int directionX[] = {1, 0, -1,  0};
  const int directionY[] = {0, 1,  0, -1};

  int total_components = 0;

  // make sure that green boxes stay within the grey
  // box and stay on the canonical green matter grid
  int xS = ms->config.gBoxW*(ms->config.grayTop.x/ms->config.gBoxW);
  int xF = min(ms->config.grayBot.x-ms->config.gBoxW, imW-ms->config.gBoxW);
  int yS = ms->config.gBoxH*(ms->config.grayTop.y/ms->config.gBoxH);
  int yF = min(ms->config.grayBot.y-ms->config.gBoxH, imH-ms->config.gBoxH);

  // fine tune
  //double adjusted_canny_lo_thresh = ms->config.canny_lo_thresh * (1.0 + (double(ms->config.loTrackbarVariable-50) / 50.0));
  //double adjusted_canny_hi_thresh = ms->config.canny_hi_thresh * (1.0 + (double(ms->config.hiTrackbarVariable-50) / 50.0));
  // broad tune
  double adjusted_canny_lo_thresh = ms->config.canny_lo_thresh * double(ms->config.loTrackbarVariable)/100.0;
  double adjusted_canny_hi_thresh = ms->config.canny_hi_thresh * double(ms->config.hiTrackbarVariable)/100.0;

//cout << "Here 1" << endl;
  for (int x = xS; x <= xF; x+=ms->config.gBoxStrideX) {
    for (int y = yS; y <= yF; y+=ms->config.gBoxStrideY) {

      int xt = x;
      int yt = y;
      int xb = x+ms->config.gBoxW;
      int yb = y+ms->config.gBoxH;
      cv::Point thisTop(xt,yt);
      cv::Point thisBot(xb,yb);

      gBoxComponentLabels[y*imW+x] = -1;
      gBoxGrayNodes[y*imW+x] = 0;
      ms->config.gBoxIndicator[y*imW+x] = 0;
      ms->config.pBoxIndicator[y*imW+x] = 0;

      double thisIntegral = ms->config.integralDensity[yb*imW+xb]-ms->config.integralDensity[yb*imW+xt]-
	ms->config.integralDensity[yt*imW+xb]+ms->config.integralDensity[yt*imW+xt];

//cout << thisIntegral << " ";

      if (thisIntegral > adjusted_canny_lo_thresh) {
	      ms->config.gBoxIndicator[y*imW+x] = 1;
	      if (ms->config.drawGreen)
		rectangle(ms->config.objectViewerImage, thisTop, thisBot, cv::Scalar(0,128,0));
      }
      if (thisIntegral > adjusted_canny_hi_thresh) {
	      ms->config.gBoxIndicator[y*imW+x] = 2;
	      if (ms->config.drawGreen)
		rectangle(ms->config.objectViewerImage, thisTop, thisBot, cv::Scalar(0,255,0));
      }
      ms->config.pBoxIndicator[y*imW+x] = thisIntegral;

    }
  }
//cout << "Here 2" << endl;

  // canny will start on a hi and spread on a lo or hi.
  //{for (int x = 0; x < imW-ms->config.gBoxW; x+=ms->config.gBoxStrideX)}
    //{for (int y = 0; y < imH-ms->config.gBoxH; y+=ms->config.gBoxStrideY)}
  for (int x = xS; x <= xF; x+=ms->config.gBoxStrideX) {
    for (int y = yS; y <= yF; y+=ms->config.gBoxStrideY) {
  
      if (ms->config.gBoxIndicator[y*imW+x] == 2 && gBoxGrayNodes[y*imW+x] == 0) {

      	gBoxGrayNodes[y*imW+x] = 1;
      	parentX.push_back(x);
      	parentY.push_back(y);
      	parentD.push_back(0);

      	gBoxComponentLabels[y*imW+x] = total_components;
      	total_components++;

      	int xt = x;
      	int yt = y;
      	int xb = x+ms->config.gBoxW;
      	int yb = y+ms->config.gBoxH;
      	cv::Point thisTop(xt,yt);
      	cv::Point thisBot(xb,yb);
      	ms->config.cTops.push_back(thisTop);
      	ms->config.cBots.push_back(thisBot);

      	while( parentX.size() > 0 ) {
      	  int index = parentX.size()-1;
      	  int direction = parentD[index];
      	  parentD[index]++;
      	  int nextX = parentX[index] + ms->config.gBoxStrideX*directionX[direction];
      	  int nextY = parentY[index] + ms->config.gBoxStrideY*directionY[direction];

      	  // if we have no more directions, then pop this parent 
      	  if (direction > 3) {
      	    parentX.pop_back();
      	    parentY.pop_back();
      	    parentD.pop_back();
      	  } 
      	  // if the next direction is valid, push it on to the stack and increment direction counter
      	  //else if(nextX > -1 && nextX < imW && nextY > -1 && nextY < imH && 
	    //ms->config.gBoxIndicator[nextY*imW+nextX] >= 1 && gBoxGrayNodes[nextY*imW+nextX] == 0) 
      	  else if(nextX >= xS && nextX <= xF && nextY >= yS && nextY <= yF && 
	    ms->config.gBoxIndicator[nextY*imW+nextX] >= 1 && gBoxGrayNodes[nextY*imW+nextX] == 0) 
	    {

      	    gBoxGrayNodes[nextY*imW+nextX] = 1;
      	    gBoxComponentLabels[nextY*imW+nextX] = gBoxComponentLabels[parentY[index]*imW+parentX[index]];

      	    int nxt = nextX;
      	    int nyt = nextY;
      	    int nxb = nextX+ms->config.gBoxW;
      	    int nyb = nextY+ms->config.gBoxH;
      	    ms->config.cTops[gBoxComponentLabels[nextY*imW+nextX]].x = min(ms->config.cTops[gBoxComponentLabels[nextY*imW+nextX]].x, nxt);
      	    ms->config.cTops[gBoxComponentLabels[nextY*imW+nextX]].y = min(ms->config.cTops[gBoxComponentLabels[nextY*imW+nextX]].y, nyt);
      	    ms->config.cBots[gBoxComponentLabels[nextY*imW+nextX]].x = max(ms->config.cBots[gBoxComponentLabels[nextY*imW+nextX]].x, nxb);
      	    ms->config.cBots[gBoxComponentLabels[nextY*imW+nextX]].y = max(ms->config.cBots[gBoxComponentLabels[nextY*imW+nextX]].y, nyb);

      	    parentX.push_back(nextX);
      	    parentY.push_back(nextY);
      	    parentD.push_back(0);
      	  } 
      	}

      }
    }
  }
//cout << "Here 3" << endl;

  ms->config.bTops.resize(0);
  ms->config.bBots.resize(0);
  ms->config.bCens.resize(0);

  ms->config.lARM = ms->config.gBoxW*(ms->config.lARM/ms->config.gBoxW);
  ms->config.rARM = ms->config.gBoxW*(ms->config.rARM/ms->config.gBoxW);
  ms->config.tARM = ms->config.gBoxH*(ms->config.tARM/ms->config.gBoxH);
  ms->config.bARM = ms->config.gBoxH*(ms->config.bARM/ms->config.gBoxH);
  ms->config.armTop = cv::Point(ms->config.lARM, ms->config.tARM);
  ms->config.armBot = cv::Point(imW-ms->config.rARM, imH-ms->config.bARM);

  int biggestBB = -1;
  int biggestBBArea = 0;

  int closestBBToReticle = -1;
  double closestBBDistance = VERYBIGNUMBER;

  // this should be -1 if we don't find the target class 
  ms->config.pilotTarget.px = -1;
  ms->config.pilotTarget.py = -1;
  ms->config.pilotClosestTarget.px = -1;
  ms->config.pilotClosestTarget.py = -1;

  if (!ms->config.all_range_mode) {
    double rejectArea = ms->config.rejectAreaScale*ms->config.gBoxW*ms->config.gBoxH;
    for (int c = 0; c < total_components; c++) {

      ms->config.cTops[c].x = max(0,min(imW-1, ms->config.cTops[c].x));
      ms->config.cTops[c].y = max(0,min(imH-1, ms->config.cTops[c].y));
      ms->config.cBots[c].x = max(0,min(imW-1, ms->config.cBots[c].x));
      ms->config.cBots[c].y = max(0,min(imH-1, ms->config.cBots[c].y));

      int allow = 1;
      if (ms->config.cBots[c].x - ms->config.cTops[c].x < ms->config.rejectScale*ms->config.gBoxW || ms->config.cBots[c].y - ms->config.cTops[c].y < ms->config.rejectScale*ms->config.gBoxH)
	allow = 0;
      if ((ms->config.cBots[c].x - ms->config.cTops[c].x)*(ms->config.cBots[c].y - ms->config.cTops[c].y) < rejectArea)
	allow = 0;
      //if (ms->config.cTops[c].y > rejectLow || ms->config.cBots[c].y < rejectHigh)
	//allow = 0;
      
      // XXX for some reason there were spurious blue boxes outside of the gray box, with no green boxes,
      //  so we reject them here for now
//      if ( (ms->config.cTops[c].x < max(ms->config.grayTop.x-ms->config.gBoxW, 0)) || (ms->config.cBots[c].x > min(ms->config.grayBot.x+ms->config.gBoxW, imW-1)) ||
//	   (ms->config.cTops[c].y < max(ms->config.grayTop.y-ms->config.gBoxW, 0)) || (ms->config.cBots[c].y > min(ms->config.grayBot.y+ms->config.gBoxW, imH-1)) )
//	allow = 0;

      // ATTN 5
      // check for overlap and fuse
      cv::Point thisCen = cv::Point((ms->config.cTops[c].x+ms->config.cBots[c].x)/2, (ms->config.cTops[c].y+ms->config.cBots[c].y)/2);
      if (ms->config.fuseBlueBoxes) {
	if (allow) {
	  for (int fuseIter = 0; fuseIter < ms->config.fusePasses; fuseIter++) {
	    for (int cbc = 0; cbc < ms->config.bTops.size(); cbc++) {

	      int smallWidth = min(ms->config.bCens[cbc].x-ms->config.bTops[cbc].x, thisCen.x-ms->config.cTops[c].x);
	      int bigWidth = max(ms->config.bCens[cbc].x-ms->config.bTops[cbc].x, thisCen.x-ms->config.cTops[c].x);

	      // this tests overlap
	      //if ( fabs(thisCen.x - ms->config.bCens[cbc].x) < fabs(ms->config.bCens[cbc].x-ms->config.bTops[cbc].x+thisCen.x-ms->config.cTops[c].x) && 
		   //fabs(thisCen.y - ms->config.bCens[cbc].y) < fabs(ms->config.bCens[cbc].y-ms->config.bTops[cbc].y+thisCen.y-ms->config.cTops[c].y) ) 
	      //this tests containment
	      if ( fabs(thisCen.x - ms->config.bCens[cbc].x) < fabs(bigWidth - smallWidth) && 
		   fabs(thisCen.y - ms->config.bCens[cbc].y) < fabs(bigWidth - smallWidth) ) 
	      {
		allow = 0;
		ms->config.bTops[cbc].x = min(ms->config.bTops[cbc].x, ms->config.cTops[c].x);
		ms->config.bTops[cbc].y = min(ms->config.bTops[cbc].y, ms->config.cTops[c].y);
		ms->config.bBots[cbc].x = max(ms->config.bBots[cbc].x, ms->config.cBots[c].x);
		ms->config.bBots[cbc].y = max(ms->config.bBots[cbc].y, ms->config.cBots[c].y);

		// gotta do this and continue searching to fuse everything, need a better algorithm in the future
		ms->config.cTops[c].x = ms->config.bTops[cbc].x;
		ms->config.cTops[c].y = ms->config.bTops[cbc].y;
		ms->config.cBots[c].x = ms->config.bBots[cbc].x;
		ms->config.cBots[c].y = ms->config.bBots[cbc].y;
	      }
	    }
	  }
	}
      }

      if (allow == 1) {
	ms->config.bTops.push_back(ms->config.cTops[c]);
	ms->config.bBots.push_back(ms->config.cBots[c]);
	ms->config.bCens.push_back(thisCen);
	int t = ms->config.bTops.size()-1;

	int thisArea = (ms->config.cBots[c].x - ms->config.cTops[c].x)*(ms->config.cBots[c].y - ms->config.cTops[c].y);
	if (thisArea > biggestBBArea) {
	  biggestBBArea = thisArea;
	  biggestBB = t;
	}

	double thisDistance = sqrt((ms->config.bCens[t].x-camera->reticle.px)*(ms->config.bCens[t].x-camera->reticle.px) + (ms->config.bCens[t].y-camera->reticle.py)*(ms->config.bCens[t].y-camera->reticle.py));
	//cout << "   (density) Distance for box " << t << " : " << thisDistance << endl;
	if (thisDistance < closestBBDistance) {
	  closestBBDistance = thisDistance;
	  closestBBToReticle = t;
	}
      }
    }
  } else {
    ms->config.bTops.push_back(ms->config.armTop);
    ms->config.bBots.push_back(ms->config.armBot);
    ms->config.bCens.push_back(cv::Point((ms->config.armTop.x+ms->config.armBot.x)/2, (ms->config.armTop.y+ms->config.armBot.y)/2));
  }

  if ((ms->config.bTops.size() > 0) && (biggestBB > -1)) {
    geometry_msgs::Point p;
    p.x = ms->config.bCens[biggestBB].x;
    p.y = ms->config.bCens[biggestBB].y;
    p.z = 0.0;
    
      //ms->config.ee_target_pub.publish(p);
    ms->config.pilotTarget.px = p.x;
    ms->config.pilotTarget.py = p.y;
    ms->config.pilotTarget.pz = p.z;
    
    ms->config.pilotTargetBlueBoxNumber = biggestBB;
  }
  if (closestBBToReticle > -1) {
    geometry_msgs::Point p;
    p.x = ms->config.bCens[closestBBToReticle].x;
    p.y = ms->config.bCens[closestBBToReticle].y;
    p.z = 0.0;
  
    //ms->config.ee_target_pub.publish(p);
    ms->config.pilotClosestTarget.px = p.x;
    ms->config.pilotClosestTarget.py = p.y;
    ms->config.pilotClosestTarget.pz = p.z;

    ms->config.pilotClosestBlueBoxNumber = closestBBToReticle;
  } else {
    ms->config.pilotClosestBlueBoxNumber = -1;
  }

  if (ms->config.bTops.size() > 0) {
    geometry_msgs::Point p;
    p.x = ms->config.bCens[biggestBB].x;
    p.y = ms->config.bCens[biggestBB].y;
    p.z = 0.0;
  
    //ms->config.ee_target_pub.publish(p);
    //ms->config.pilotTarget.px = p.x;
    //ms->config.pilotTarget.py = p.y;
    //ms->config.pilotTarget.pz = p.z;
  }

  if (ms->config.drawBlue) {
    for (int c = ms->config.bTops.size()-1; c >= 0; c--) {
      cv::Point outTop = cv::Point(ms->config.bTops[c].x, ms->config.bTops[c].y);
      cv::Point outBot = cv::Point(ms->config.bBots[c].x, ms->config.bBots[c].y);
      cv::Point inTop = cv::Point(ms->config.bTops[c].x+1,ms->config.bTops[c].y+1);
      cv::Point inBot = cv::Point(ms->config.bBots[c].x-1,ms->config.bBots[c].y-1);
      rectangle(ms->config.objectViewerImage, outTop, outBot, cv::Scalar(255,0,0));
      rectangle(ms->config.objectViewerImage, inTop, inBot, cv::Scalar(255,192,192));
    }
  }

//cout << "Here 4" << endl;

  if (ms->config.shouldIRender && ms->config.showgui) {
    ms->config.objectViewerWindow->updateImage(ms->config.objectViewerImage);
  }

  delete ms->config.gBoxIndicator;
  delete gBoxGrayNodes;
  delete gBoxComponentLabels;
}


void goClassifyBlueBoxes(MachineState * ms) {
  //cout << "entered gCBB()" << endl; cout.flush();
  Size sz = ms->config.objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;
  //cout << imW << " " << imH << endl; cout.flush();
  int param_numNeighbors = 4;


  vector< vector<int> > pIoCbuffer;

  // classify the crops
  ms->config.bKeypoints.resize(ms->config.bTops.size());
  ms->config.bWords.resize(ms->config.bTops.size());
  ms->config.bYCrCb.resize(ms->config.bTops.size());
  ms->config.bLabels.resize(ms->config.bTops.size());

  int biggestBB = -1;
  int biggestBBArea = 0;

  int closestBBToReticle = -1;
  double closestBBDistance = VERYBIGNUMBER;

  double label = -1;

  if (ms->config.kNN == NULL) {
    CONSOLE_ERROR(ms, "Oops, kNN is NULL, so we better stop here... but we'll continue, setting all labels to 0." << endl);
    for (int i = 0; i < ms->config.bLabels.size(); i++) {
      ms->config.bLabels[i] = 0;
    }
    return;
    //assert(0);
  } else {
  }

  if (ms->config.kNN->get_sample_count() < 1) {
    CONSOLE_ERROR(ms, "Oops, kNN has no samples, so we better stop here... but we'll continue, setting all labels to 0." << endl);
    for (int i = 0; i < ms->config.bLabels.size(); i++) {
      ms->config.bLabels[i] = 0;
    }
    return;
    //assert(0);
  }
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  for (int c = 0; c < ms->config.bTops.size(); c++) {
    vector<KeyPoint>& keypoints = ms->config.bKeypoints[c];
    Mat descriptors;
    Mat descriptors2;

    Mat original_cam_img = camera->cam_bgr_img;
    Mat crop = original_cam_img(cv::Rect(ms->config.bTops[c].x, ms->config.bTops[c].y, ms->config.bBots[c].x-ms->config.bTops[c].x, ms->config.bBots[c].y-ms->config.bTops[c].y));
    Mat gray_image;
    Mat& yCrCb_image = ms->config.bYCrCb[c];

    //if ((ms->config.chosen_feature == SIFTBOW_GLOBALCOLOR_HIST) || (ms->config.chosen_feature == OPPONENTSIFTBOW_GLOBALCOLOR_HIST))
    if (ms->config.chosen_feature == SIFTBOW_GLOBALCOLOR_HIST) 
    {
      processImage(crop, gray_image, yCrCb_image, ms->config.grayBlur);

      //ms->config.detector->detect(gray_image, keypoints);
      gridKeypoints(ms, imW, imH, ms->config.bTops[c], ms->config.bBots[c], ms->config.gBoxStrideX, ms->config.gBoxStrideY, keypoints, ms->config.keypointPeriod);

      ms->config.bowExtractor->compute(gray_image, keypoints, descriptors, &pIoCbuffer);

      // save the word assignments for the keypoints so we can use them for red boxes

      ms->config.bWords[c].resize(keypoints.size());
      if ((pIoCbuffer.size() > 0) && (keypoints.size() > 0)) {
	for (int w = 0; w < ms->config.vocabNumWords; w++) {
	  int numDescrOfWord = pIoCbuffer[w].size();

	  for (int w2 = 0; w2 < numDescrOfWord; w2++) {
	    ms->config.bWords[c][pIoCbuffer[w][w2]] = w;
	  }
	}
    
	if (ms->config.drawBlueKP) {
	  for (int kp = 0; kp < keypoints.size(); kp++) {
	    int tX = keypoints[kp].pt.x;
	    int tY = keypoints[kp].pt.y;
	    cv::Point kpTop = cv::Point(ms->config.bTops[c].x+tX-1,ms->config.bTops[c].y+tY-1);
	    cv::Point kpBot = cv::Point(ms->config.bTops[c].x+tX,ms->config.bTops[c].y+tY);
	    if(
	      (kpTop.x >= 1) &&
	      (kpBot.x <= imW-2) &&
	      (kpTop.y >= 1) &&
	      (kpBot.y <= imH-2) 
	      ) {
	      rectangle(ms->config.objectViewerImage, kpTop, kpBot, cv::Scalar(255,0,0));
	    }
	  }
	}

      }
      
      if (!descriptors.empty() && !keypoints.empty()) {
      
	appendColorHist(yCrCb_image, keypoints, descriptors, descriptors2);
	label = ms->config.kNN->find_nearest(descriptors2, param_numNeighbors);
	ms->config.bLabels[c] = label;
      }
    } else if (ms->config.chosen_feature == OPPONENTSIFTBOW_GLOBALCOLOR_HIST) {
      processImage(crop, gray_image, yCrCb_image, ms->config.grayBlur);

      //ms->config.detector->detect(gray_image, keypoints);
      gridKeypoints(ms, imW, imH, ms->config.bTops[c], ms->config.bBots[c], ms->config.gBoxStrideX, ms->config.gBoxStrideY, keypoints, ms->config.keypointPeriod);

      //ms->config.bowExtractor->compute(gray_image, keypoints, descriptors, &pIoCbuffer);

      Mat tmpC;
      crop.convertTo(tmpC, CV_32FC3);
      ms->config.bowExtractor->compute(tmpC, keypoints, descriptors);

      // save the word assignments for the keypoints so we can use them for red boxes

      ms->config.bWords[c].resize(keypoints.size());
      if ((pIoCbuffer.size() > 0) && (keypoints.size() > 0)) {
	for (int w = 0; w < ms->config.vocabNumWords; w++) {
	  int numDescrOfWord = pIoCbuffer[w].size();

	  for (int w2 = 0; w2 < numDescrOfWord; w2++) {
	    ms->config.bWords[c][pIoCbuffer[w][w2]] = w;
	  }
	}
    
	if (ms->config.drawBlueKP) {
	  for (int kp = 0; kp < keypoints.size(); kp++) {
	    int tX = keypoints[kp].pt.x;
	    int tY = keypoints[kp].pt.y;
	    cv::Point kpTop = cv::Point(ms->config.bTops[c].x+tX-1,ms->config.bTops[c].y+tY-1);
	    cv::Point kpBot = cv::Point(ms->config.bTops[c].x+tX,ms->config.bTops[c].y+tY);
	    if(
	      (kpTop.x >= 1) &&
	      (kpBot.x <= imW-2) &&
	      (kpTop.y >= 1) &&
	      (kpBot.y <= imH-2) 
	      ) {
	      rectangle(ms->config.objectViewerImage, kpTop, kpBot, cv::Scalar(255,0,0));
	    }
	  }
	}

      }
      
      if (!descriptors.empty() && !keypoints.empty()) {
      
	//appendColorHist(yCrCb_image, keypoints, descriptors, descriptors2);
	//label = kNN->find_nearest(descriptors2,k);
	label = ms->config.kNN->find_nearest(descriptors, param_numNeighbors);
	ms->config.bLabels[c] = label;
      }
    } else if (ms->config.chosen_feature == GRADIENT) {
      processImage(crop, gray_image, yCrCb_image, ms->config.sobel_sigma);

      Mat totalGraySobel;
      {
	Mat grad_x, grad_y;
	int sobelScale = 1;
	int sobelDelta = 0;
	int sobelDepth = CV_32F;
	/// Gradient X
	Sobel(gray_image, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
	/// Gradient Y
	Sobel(gray_image, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

	grad_x = grad_x.mul(grad_x);
	grad_y = grad_y.mul(grad_y);
	totalGraySobel = grad_x + grad_y;
	// now totalGraySobel is gradient magnitude squared
      }

      // grow to the max dimension to avoid distortion
      // find the dimensions that pad the sobel image up to a square
      // raster scan a 'virtual image' to generate the 1D vector, adding 0's when on the pad

      int crows = totalGraySobel.rows;
      int ccols = totalGraySobel.cols;
      int maxDim = max(crows, ccols);
      int tRy = (maxDim-crows)/2;
      int tRx = (maxDim-ccols)/2;
      Mat gCrop(maxDim, maxDim, totalGraySobel.type());

      float totalMass = 0.0;

      for (int x = 0; x < maxDim; x++) {
	for (int y = 0; y < maxDim; y++) {
	  int tx = x - tRx;
	  int ty = y - tRy;
	  if (tx >= 0 && ty >= 0 && ty < crows && tx < ccols) {
	    gCrop.at<float>(y, x) = totalGraySobel.at<float>(ty, tx);
	    //totalMass += gCrop.at<float>(y, x);
	    totalMass += gCrop.at<float>(y, x) * gCrop.at<float>(y, x);
	  } else {
	    gCrop.at<float>(y, x) = 0.0;
	  }
	}
      }
      totalMass = sqrt(totalMass);
      Mat descriptorsG = Mat(1, ms->config.gradientFeatureWidth*ms->config.gradientFeatureWidth, CV_32F);
      for (int y = 0; y < ms->config.gradientFeatureWidth; y++) {
	for (int x = 0; x < ms->config.gradientFeatureWidth; x++) {
	  int tranX = floor(float(x)*float(maxDim)/float(ms->config.gradientFeatureWidth));
	  int tranY = floor(float(y)*float(maxDim)/float(ms->config.gradientFeatureWidth));
	  //descriptorsG.at<float>(x + y*ms->config.gradientFeatureWidth) = gCrop.at<float>(y,x);
	  descriptorsG.at<float>(x + y*ms->config.gradientFeatureWidth) = gCrop.at<float>(y,x)/totalMass;
	}
      }

      label = ms->config.kNN->find_nearest(descriptorsG, param_numNeighbors);
      ms->config.bLabels[c] = label;
    } else if (ms->config.chosen_feature == OPPONENT_COLOR_GRADIENT) {
      processImage(crop, gray_image, yCrCb_image, ms->config.sobel_sigma);

      Mat totalGraySobel;
      {
	Mat grad_x, grad_y;
	int sobelScale = 1;
	int sobelDelta = 0;
	int sobelDepth = CV_32F;
	/// Gradient X
	Sobel(gray_image, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
	/// Gradient Y
	Sobel(gray_image, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

	grad_x = grad_x.mul(grad_x);
	grad_y = grad_y.mul(grad_y);
	totalGraySobel = grad_x + grad_y;
	// now totalGraySobel is gradient magnitude squared
      }
      Mat totalCrSobel = totalGraySobel.clone();
      {
	for (int y = 0; y < crop.rows; y++) {
	  for (int x = 0; x < crop.cols; x++) {
	    cv::Vec3b thisColor = yCrCb_image.at<cv::Vec3b>(y,x);
	    totalCrSobel.at<float>(y,x) = thisColor[1];
	  }
	}
	Mat grad_x, grad_y;
	int sobelScale = 1;
	int sobelDelta = 0;
	int sobelDepth = CV_32F;
	/// Gradient X
	Sobel(totalCrSobel, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
	/// Gradient Y
	Sobel(totalCrSobel, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

	grad_x = grad_x.mul(grad_x);
	grad_y = grad_y.mul(grad_y);
	totalCrSobel = grad_x + grad_y;
      }
      Mat totalCbSobel = totalGraySobel.clone();
      {
	for (int y = 0; y < crop.rows; y++) {
	  for (int x = 0; x < crop.cols; x++) {
	    cv::Vec3b thisColor = yCrCb_image.at<cv::Vec3b>(y,x);
	    totalCbSobel.at<float>(y,x) = thisColor[2];
	  }
	}
	Mat grad_x, grad_y;
	int sobelScale = 1;
	int sobelDelta = 0;
	int sobelDepth = CV_32F;
	/// Gradient X
	Sobel(totalCbSobel, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
	/// Gradient Y
	Sobel(totalCbSobel, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

	grad_x = grad_x.mul(grad_x);
	grad_y = grad_y.mul(grad_y);
	totalCbSobel = grad_x + grad_y;
      }

      // grow to the max dimension to avoid distortion
      // find the dimensions that pad the sobel image up to a square
      // raster scan a 'virtual image' to generate the 1D vector, adding 0's when on the pad

      int crows = totalGraySobel.rows;
      int ccols = totalGraySobel.cols;
      int maxDim = max(crows, ccols);
      int tRy = (maxDim-crows)/2;
      int tRx = (maxDim-ccols)/2;
      Mat gCrop(maxDim, maxDim, totalGraySobel.type());
      Mat crCrop(maxDim, maxDim, totalGraySobel.type());
      Mat cbCrop(maxDim, maxDim, totalGraySobel.type());

      float totalGMass = 0.0;
      float totalCrMass = 0.0;
      float totalCbMass = 0.0;

      for (int x = 0; x < maxDim; x++) {
	for (int y = 0; y < maxDim; y++) {
	  int tx = x - tRx;
	  int ty = y - tRy;
	  if (tx >= 0 && ty >= 0 && ty < crows && tx < ccols) {

	    // ATTN 24
	    // XXX
	    crCrop.at<float>(y,x) = yCrCb_image.at<Vec3b>(y,x)[1];
	    cbCrop.at<float>(y,x) = yCrCb_image.at<Vec3b>(y,x)[2];

	    gCrop.at<float>(y, x) = totalGraySobel.at<float>(ty, tx);
	    crCrop.at<float>(y, x) = totalCrSobel.at<float>(ty, tx);
	    cbCrop.at<float>(y, x) = totalCbSobel.at<float>(ty, tx);
	    //totalGMass += gCrop.at<float>(y, x);
	    totalGMass += gCrop.at<float>(y, x) * gCrop.at<float>(y, x);
	    //totalCrMass += gCrop.at<float>(y, x) * gCrop.at<float>(y, x);
	    //totalCbMass += gCrop.at<float>(y, x) * gCrop.at<float>(y, x);
	    totalCrMass += crCrop.at<float>(y, x) * crCrop.at<float>(y, x);
	    totalCbMass += cbCrop.at<float>(y, x) * cbCrop.at<float>(y, x);
	  } else {
	    gCrop.at<float>(y, x) = 0.0;
	  }
	}
      }
      totalGMass = sqrt(totalGMass);
      totalCrMass = sqrt(totalCrMass);
      totalCbMass = sqrt(totalCbMass);
      double totalColorMass = totalCrMass + totalCbMass;
      //Mat descriptorsG = Mat(1, ms->config.gradientFeatureWidth*ms->config.gradientFeatureWidth, CV_32F);
      Mat descriptorsCbCr = Mat(1, 2*ms->config.gradientFeatureWidth*ms->config.gradientFeatureWidth, CV_32F);
      for (int y = 0; y < ms->config.gradientFeatureWidth; y++) {
	for (int x = 0; x < ms->config.gradientFeatureWidth; x++) {
	  int tranX = floor(float(x)*float(maxDim)/float(ms->config.gradientFeatureWidth));
	  int tranY = floor(float(y)*float(maxDim)/float(ms->config.gradientFeatureWidth));
	  //descriptorsG.at<float>(x + y*ms->config.gradientFeatureWidth) = gCrop.at<float>(y,x);
	  //descriptorsCbCr.at<float>(x + y*ms->config.gradientFeatureWidth) = crCrop.at<float>(y,x)/totalCrMass;
	  //descriptorsCbCr.at<float>(x + y*ms->config.gradientFeatureWidth + ms->config.gradientFeatureWidth*ms->config.gradientFeatureWidth) = cbCrop.at<float>(y,x)/totalCbMass;
	  //descriptorsCbCr.at<float>(x + y*ms->config.gradientFeatureWidth) = crCrop.at<float>(y,x);
	  //descriptorsCbCr.at<float>(x + y*ms->config.gradientFeatureWidth + ms->config.gradientFeatureWidth*ms->config.gradientFeatureWidth) = cbCrop.at<float>(y,x);

	  descriptorsCbCr.at<float>(x + y*ms->config.gradientFeatureWidth) = crCrop.at<float>(y,x)/totalColorMass;
	  descriptorsCbCr.at<float>(x + y*ms->config.gradientFeatureWidth + ms->config.gradientFeatureWidth*ms->config.gradientFeatureWidth) = cbCrop.at<float>(y,x)/totalColorMass;
	}
      }

      label = ms->config.kNN->find_nearest(descriptorsCbCr, param_numNeighbors);
      ms->config.bLabels[c] = label;
    }

    string labelName; 
    string augmentedLabelName;
    double poseIndex = -1;
    int winningO = -1;

    if (label == -1)
      labelName = "VOID";
    else
      labelName = ms->config.classLabels[label];
    augmentedLabelName = labelName;

    cv::Point text_anchor(ms->config.bTops[c].x+1, ms->config.bBots[c].y-2);
    cv::Point text_anchor2(ms->config.bTops[c].x+1, ms->config.bBots[c].y-2);
    putText(ms->config.objectViewerImage, augmentedLabelName, text_anchor, MY_FONT, 0.5, Scalar(255,192,192), 2.0);
    putText(ms->config.objectViewerImage, augmentedLabelName, text_anchor2, MY_FONT, 0.5, Scalar(255,0,0), 1.0);



    vector<cv::Point> pointCloudPoints;

    if (label >= 0) {

      int thisArea = (ms->config.bBots[c].x - ms->config.bTops[c].x)*(ms->config.bBots[c].y - ms->config.bTops[c].y);
      if ((thisArea > biggestBBArea) && (label == ms->config.targetClass)) 
      //if ((thisArea > biggestBBArea) && (shouldIPick(label))) 
      {
	biggestBBArea = thisArea;
	biggestBB = c;
      }

      //int thisDistance = int(fabs(ms->config.bCens[c].x-reticle.px) + fabs(ms->config.bCens[c].y-reticle.py));
      double thisDistance = sqrt((ms->config.bCens[c].x-camera->reticle.px)*(ms->config.bCens[c].x-camera->reticle.px) + (ms->config.bCens[c].y-camera->reticle.py)*(ms->config.bCens[c].y-camera->reticle.py));
      cout << "   Distance for box " << c << " : " << thisDistance << endl;
      if (thisDistance < closestBBDistance) {
	closestBBDistance = thisDistance;
	closestBBToReticle = c;
      }
    }
  }

  if ((ms->config.bTops.size() > 0) && (biggestBB > -1)) {
    geometry_msgs::Point p;
    p.x = ms->config.bCens[biggestBB].x;
    p.y = ms->config.bCens[biggestBB].y;
    p.z = 0.0;
    
      //ms->config.ee_target_pub.publish(p);
    ms->config.pilotTarget.px = p.x;
    ms->config.pilotTarget.py = p.y;
    ms->config.pilotTarget.pz = p.z;
    
    ms->config.pilotTargetBlueBoxNumber = biggestBB;
  }
  if (closestBBToReticle > -1) {
    geometry_msgs::Point p;
    p.x = ms->config.bCens[closestBBToReticle].x;
    p.y = ms->config.bCens[closestBBToReticle].y;
    p.z = 0.0;
  
    //ms->config.ee_target_pub.publish(p);
    ms->config.pilotClosestTarget.px = p.x;
    ms->config.pilotClosestTarget.py = p.y;
    ms->config.pilotClosestTarget.pz = p.z;

    ms->config.pilotClosestBlueBoxNumber = closestBBToReticle;
  } else {
    ms->config.pilotClosestBlueBoxNumber = -1;
  }

  if (ms->config.shouldIRender && ms->config.showgui) {
    ms->config.objectViewerWindow->updateImage(ms->config.objectViewerImage);
  }

}



void loadROSParamsFromArgs(MachineState * ms) {
  ros::NodeHandle nh("~");


  //cout << "nh namespace: " << nh.getNamespace() << endl;


  nh.getParam("/robot_description", ms->config.robot_description);
  nh.getParam("/manifest/robot_serial", ms->config.robot_serial);
  nh.getParam("/rethink/software_version", ms->config.robot_software_version);

  if (ms->config.robot_mode == "simulated") {
    ms->config.currentRobotMode = SIMULATED;

    std::ifstream ifs("src/ein/baxter.urdf");
    std::string content( (std::istreambuf_iterator<char>(ifs) ),
			 (std::istreambuf_iterator<char>()    ) );
    ms->config.robot_description = content;
    ms->config.robot_serial = "simulatedserial";
    for (int i = 0; i < ms->config.cameras.size(); i++) {
      ms->config.cameras[i]->currentCameraCalibrationMode = CAMCAL_LINBOUNDED;
    }
  } 

  ms->config.config_directory = "/config_" + ms->config.robot_serial + "/";
  ms->config.config_filename = ms->config.data_directory + "/" + ms->config.config_directory + "config.yml";
}


void irInit(MachineState * ms) {

  {
    //gear0offset = Eigen::Quaternionf(0.0, 0.023, 0.023, 0.0167228); // z is from TF, good for depth alignment
    //if (0 == ms->config.left_or_right_arm.compare("left")) {
      //camera->gear0offset = Eigen::Quaternionf(0.0, 0.03, 0.023, 0.0167228); // z is from TF, good for depth alignment
    //} else if (0 == ms->config.left_or_right_arm.compare("right")) {
      //camera->gear0offset = Eigen::Quaternionf(0.0, 0.023, 0.023, 0.0167228); // z is from TF, good for depth alignment
    //}

    // invert the transformation
    Camera * camera  = ms->config.cameras[ms->config.focused_camera];

    Eigen::Quaternionf crane2quat(ms->config.straightDown.qw, ms->config.straightDown.qx, ms->config.straightDown.qy, ms->config.straightDown.qz);
    Eigen::Quaternionf irpos = crane2quat.conjugate() * camera->gear0offset * crane2quat;
    ms->config.irGlobalPositionEEFrame[0] = irpos.w();
    ms->config.irGlobalPositionEEFrame[1] = irpos.x();
    ms->config.irGlobalPositionEEFrame[2] = irpos.y();
    ms->config.irGlobalPositionEEFrame[3] = irpos.z();

  }


  ms->config.epRingBuffer.resize(ms->config.epRingBufferSize);
  ms->config.rgRingBuffer.resize(ms->config.rgRingBufferSize);

  ms->config.epRBTimes.resize(ms->config.epRingBufferSize);
  ms->config.rgRBTimes.resize(ms->config.rgRingBufferSize);


  for (int r = 0; r < ms->config.totalRangeHistoryLength; r++) {
    ms->config.rangeHistory[r] = 0;
  }
  ms->config.rangeogramImage = Mat(ms->config.rggHeight, ms->config.rggWidth, CV_8UC3);
}


void nodeInit(MachineState * ms) {
  ms->config.gBoxStrideX = ms->config.gBoxW / 2.0;
  ms->config.gBoxStrideY = ms->config.gBoxH / 2.0;
  ms->config.cropCounter = 0;

}


void detectorsInit(MachineState * ms) {

  // XXX TODO this function should reinit the structures if this function is to be called multiple times

  // SIFT 
  //ms->config.detector = new SiftFeatureDetector(0, 3, 0.04, 10, 1.6);
  //cout << "ms->config.chosen_feature: " << ms->config.chosen_feature << endl;
  if (ms->config.detector == NULL)
    ms->config.detector = new FastFeatureDetector(4);

#ifdef __OPENCV_NONFREE_HPP__
  if (ms->config.extractor == NULL) {
    if (ms->config.chosen_feature == SIFTBOW_GLOBALCOLOR_HIST)
      ms->config.extractor = new SiftDescriptorExtractor();
    else if (ms->config.chosen_feature == OPPONENTSIFTBOW_GLOBALCOLOR_HIST)
      ms->config.extractor = DescriptorExtractor::create("OpponentSIFT");
    else {
      ms->config.extractor = new SiftDescriptorExtractor();
    }
  }
#endif
  if ( (ms->config.chosen_feature == GRADIENT) || 
       (ms->config.chosen_feature == OPPONENT_COLOR_GRADIENT) ||
       (ms->config.chosen_feature == CBCR_HISTOGRAM) ){
    ms->config.retrain_vocab = 0;
  }

  // BOW time
  ms->config.bowTrainer = new BOWKMeansTrainer(ms->config.vocabNumWords);

  // read the class image data
  string dot(".");
  string dotdot("..");

  char vocabularyPath[1024];
  char featuresPath[1024];
  char labelsPath[1024];
  sprintf(vocabularyPath, "%s/objects/%s", ms->config.data_directory.c_str(), ms->config.vocab_file.c_str());
  sprintf(featuresPath, "%s/objects/%s", ms->config.data_directory.c_str(), ms->config.knn_file.c_str());
  sprintf(labelsPath, "%s/objects/%s", ms->config.data_directory.c_str(), ms->config.label_file.c_str());
  cout << "vocabularyPath: " << vocabularyPath << endl;
  cout << "featuresPath: " << featuresPath << endl;
  cout << "labelsPath: " << labelsPath << endl;

  string bufstr; // Have a buffer string

  int numCachedClasses = 0;

  if (ms->config.rewrite_labels) {
    // load cached labels 
    vector<string> classCacheLabels;
    vector<string> classCachePoseModels;
    if (ms->config.cache_prefix.size() > 0) {
      string labelsCacheFile = ms->config.data_directory + "/objects/" + ms->config.cache_prefix + "labels.yml";

      FileStorage fsvI;
      cout<<"Reading CACHED labels and pose models from " << labelsCacheFile << " ...";
      fsvI.open(labelsCacheFile, FileStorage::READ);
      fsvI["labels"] >> classCacheLabels;
      fsvI["poseModels"] >> classCachePoseModels;
      //ms->config.classLabels.insert(ms->config.classLabels.end(), classCacheLabels.begin(), classCacheLabels.end());
      //ms->config.classPoseModels.insert(ms->config.classPoseModels.end(), classCachePoseModels.begin(), classCachePoseModels.end());
      cout << "done." << endl << "classCacheLabels size: " << classCacheLabels.size() << " classCachePoseModels size: " << classCachePoseModels.size() << endl;
      numCachedClasses = classCacheLabels.size();

      classCacheLabels.insert(classCacheLabels.end(), ms->config.classLabels.begin(), ms->config.classLabels.end());
      classCachePoseModels.insert(classCachePoseModels.end(), ms->config.classPoseModels.begin(), ms->config.classPoseModels.end());
      ms->config.classLabels = classCacheLabels;
      ms->config.classPoseModels = classCachePoseModels;
      cout << "classLabels size: " << ms->config.classLabels.size() << " classPoseModels size: " << ms->config.classPoseModels.size() << endl;
    }

    FileStorage fsvO;
    cout<<"Writing labels and pose models... " << labelsPath << " ...";
    fsvO.open(labelsPath, FileStorage::WRITE);
    fsvO << "labels" << ms->config.classLabels;
    fsvO << "poseModels" << ms->config.classPoseModels;
    fsvO.release();
    cout << "done." << endl;
  } else {
    FileStorage fsvI;
    cout<<"Reading labels and pose models... "<< labelsPath << " ...";
    fsvI.open(labelsPath, FileStorage::READ);
    fsvI["labels"] >> ms->config.classLabels;
    fsvI["poseModels"] >> ms->config.classPoseModels;
    cout << "done. classLabels size: " << ms->config.classLabels.size() << " classPoseModels size: " << ms->config.classPoseModels.size() << endl;
  }

  for (unsigned int i = 0; i < ms->config.classLabels.size(); i++) {
    cout << ms->config.classLabels[i] << " " << ms->config.classPoseModels[i] << endl;
  }

  // this is the total number of classes, so it is counted after the cache is dealt with
  ms->config.numClasses = ms->config.classLabels.size();

  Mat vocabulary;

  ms->config.grandTotalDescriptors = 0;
  if (ms->config.retrain_vocab) {
    for (unsigned int i = 0; i < ms->config.classLabels.size(); i++) {
      cout << "Getting BOW features for class " << ms->config.classLabels[i] 
	   << " with pose model " << ms->config.classPoseModels[i] << " index " << i << endl;
      bowGetFeatures(ms, ms->config.class_crops_path, ms->config.classLabels[i].c_str(), ms->config.grayBlur, ms->config.keypointPeriod, &ms->config.grandTotalDescriptors,
                     ms->config.extractor, ms->config.bowTrainer);
      if (ms->config.classPoseModels[i].compare("G") == 0) {
	string thisPoseLabel = ms->config.classLabels[i] + "Poses";
        bowGetFeatures(ms, ms->config.class_crops_path, thisPoseLabel.c_str(), ms->config.grayBlur, ms->config.keypointPeriod, &ms->config.grandTotalDescriptors,
                       ms->config.extractor, ms->config.bowTrainer);
      }
    }

    if (ms->config.grandTotalDescriptors < ms->config.vocabNumWords) {
      cout << "Fewer descriptors than words in the vocab!?... This will never work, cease training. Duplicate RGB images if you must." << endl;
      cout << "Label file may now be corrupt!" << endl;
      // TODO XXX we shouldn't write any files until we know it will succeed
      return;
    }

    cout << "Clustering features... ";
    cout.flush();
    vocabulary = ms->config.bowTrainer->cluster();
    cout << "done." << endl;

    FileStorage fsvO;
    cout << "Writing vocab... " << vocabularyPath << " ...";
    fsvO.open(vocabularyPath, FileStorage::WRITE);
    fsvO << "vocab" << vocabulary;
    fsvO.release();
    cout << "done." << endl;
  } else {
    FileStorage fsvI;
    cout << "Reading vocab... " << vocabularyPath << " ...";
    fsvI.open(vocabularyPath, FileStorage::READ);
    fsvI["vocab"] >> vocabulary;
    cout << "done. vocabulary size: " << vocabulary.size() << endl;
  }

  if (ms->config.matcher == NULL)
    ms->config.matcher = new BFMatcher(NORM_L2);
  if (ms->config.bowExtractor == NULL)
    ms->config.bowExtractor = new BOWImgDescriptorExtractor(ms->config.extractor, ms->config.matcher);
  ms->config.bowExtractor->setVocabulary(vocabulary);

  Mat kNNfeatures;
  Mat kNNlabels;

  ms->config.classPosekNNs.resize(ms->config.numClasses);
  ms->config.classPosekNNfeatures.resize(ms->config.numClasses);
  ms->config.classPosekNNlabels.resize(ms->config.numClasses);
  ms->config.classQuaternions.resize(ms->config.numClasses);

  if (ms->config.reextract_knn) {
    //for (int i = 0; i < numNewClasses; i++) 
    for (int i = numCachedClasses; i < ms->config.numClasses; i++) 
    {
      cout << "Getting kNN features for class " << ms->config.classLabels[i] 
	   << " with pose model " << ms->config.classPoseModels[i] << " index " << i << endl;
      kNNGetFeatures(ms, ms->config.class_crops_path, ms->config.classLabels[i].c_str(), i, ms->config.grayBlur, kNNfeatures, kNNlabels, ms->config.sobel_sigma);
      if (ms->config.classPoseModels[i].compare("G") == 0) {
	string thisPoseLabel = ms->config.classLabels[i] + "Poses";
      posekNNGetFeatures(ms, ms->config.class_crops_path, thisPoseLabel.c_str(), ms->config.grayBlur, ms->config.classPosekNNfeatures[i], ms->config.classPosekNNlabels[i],
                         ms->config.classQuaternions[i], ms->config.keypointPeriod, ms->config.bowExtractor);
      }
    }

    // load cached kNN features 
    // XXX experimental handling of G pose models
    Mat kNNCachefeatures;
    Mat kNNCachelabels;
    if (ms->config.cache_prefix.size() > 0) {
      string knnCacheFile = ms->config.data_directory + "/objects/" + ms->config.cache_prefix + "knn.yml";

      FileStorage fsfI;
      cout<<"Reading CACHED features... " << knnCacheFile << " ..." << endl;
      fsfI.open(knnCacheFile, FileStorage::READ);
      fsfI["features"] >> kNNCachefeatures;
      fsfI["labels"] >> kNNCachelabels;
      kNNfeatures.push_back(kNNCachefeatures);
      kNNlabels.push_back(kNNCachelabels);

      for (int i = 0; i < ms->config.numClasses; i++) {
	if (ms->config.classPoseModels[i].compare("G") == 0) {
	  string fnIn = "features" + ms->config.classLabels[i];
	  string lnIn = "labels" + ms->config.classLabels[i];
	  string qnIn = "quaternions" + ms->config.classLabels[i];
	  cout << "G: " << ms->config.classLabels[i] << " " << fnIn << " " << lnIn << endl;
	  fsfI[fnIn] >> ms->config.classPosekNNfeatures[i];
	  fsfI[lnIn] >> ms->config.classPosekNNlabels[i];
	  fsfI[qnIn] >> ms->config.classQuaternions[i];
	}
      }

      cout << "done." << kNNCachefeatures.size() << " " << kNNCachelabels.size() << endl;
    }

    FileStorage fsfO;
    cout<<"Writing features and labels... " << featuresPath << " ..." << endl;
    fsfO.open(featuresPath, FileStorage::WRITE);
    fsfO << "features" << kNNfeatures;
    fsfO << "labels" << kNNlabels;

    // TODO also cache the features for the pose models

    for (int i = 0; i < ms->config.numClasses; i++) {
      if (ms->config.classPoseModels[i].compare("G") == 0) {
	string fnOut = "features" + ms->config.classLabels[i];
	string lnOut = "labels" + ms->config.classLabels[i];
	string qnOut = "quaternions" + ms->config.classLabels[i];
	cout << "G: " << ms->config.classLabels[i] << " " << fnOut << " " << lnOut << endl;
	fsfO << fnOut << ms->config.classPosekNNfeatures[i];
	fsfO << lnOut << ms->config.classPosekNNlabels[i];
	fsfO << qnOut << ms->config.classQuaternions[i];
      }
    }
    fsfO.release();
    cout << "done." << endl;
  } else { 
    FileStorage fsfI;
    cout<<"Reading features and labels... " << featuresPath << " ..." << endl;
    fsfI.open(featuresPath, FileStorage::READ);
    if (!fsfI.isOpened()) {
      CONSOLE_ERROR(ms, "Could not find file " << featuresPath << endl);
    }

    fsfI["features"] >> kNNfeatures;
    fsfI["labels"] >> kNNlabels;
    for (int i = 0; i < ms->config.numClasses; i++) {
      if (ms->config.classPoseModels[i].compare("G") == 0) {
	string fnIn = "features" + ms->config.classLabels[i];
	string lnIn = "labels" + ms->config.classLabels[i];
	string qnIn = "quaternions" + ms->config.classLabels[i];
	cout << "G: " << ms->config.classLabels[i] << " " << fnIn << " " << lnIn << endl;
	fsfI[fnIn] >> ms->config.classPosekNNfeatures[i];
	fsfI[lnIn] >> ms->config.classPosekNNlabels[i];
	fsfI[qnIn] >> ms->config.classQuaternions[i];
      }
    }
    cout << "done. knnFeatures size: " << kNNfeatures.size() << " kNNlabels size: " << kNNlabels.size() << endl;
  }

  cout << "kNNlabels dimensions: " << kNNlabels.size().height << " by " << kNNlabels.size().width << endl;
  cout << "kNNfeatures dimensions: " << kNNfeatures.size().height << " by " << kNNfeatures.size().width << endl;

  cout << "Main kNN...";

  if ( (kNNfeatures.data == NULL) || (kNNfeatures.rows < 1) || (kNNfeatures.cols < 1) ) {
    // seeing this can be distressing
    //cout << "There is a problem with kNN features, cannot initialize detector and files may be corrupt." << endl;
  } else {
    ms->config.kNN = new CvKNearest(kNNfeatures, kNNlabels);
    cout << "done." << endl;
    for (int i = 0; i < ms->config.numClasses; i++) {
      if (ms->config.classPoseModels[i].compare("G") == 0) {
	cout << "Class " << i << " kNN..." << ms->config.classPosekNNfeatures[i].size() << ms->config.classPosekNNlabels[i].size() << endl;
	ms->config.classPosekNNs[i] = new CvKNearest(ms->config.classPosekNNfeatures[i], ms->config.classPosekNNlabels[i]);
	cout << "Done" << endl;
      }
    }
  }
}


void tryToLoadRangeMap(MachineState * ms, std::string classDir, const char *className, int i) {

  string thisLabelName(className);

  string objectDir = ms->config.data_directory + "/objects/" + thisLabelName;
  if (! boost::filesystem::exists(objectDir)) {
    CONSOLE_ERROR(ms, "Could not find " << objectDir << "... loading default empty class.");
  }


  {
    string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/ir2d/";
    string this_range_path = dirToMakePath + "ir2d.yml";

    cout << "  tryToLoadRangeMap: " << this_range_path << endl;

    FileStorage fsfI;
    fsfI.open(this_range_path, FileStorage::READ);
    if (fsfI.isOpened()) {

      {
	FileNode anode = fsfI["graspZ"];

	if (anode.type() == cv::FileNode::SEQ){
	  cout << anode.type() << " Loading  classGraspZs from " << this_range_path;
	  FileNodeIterator it = anode.begin(), it_end = anode.end();
	  ms->config.currentGraspZ = *(it++);
	  ms->config.classGraspZs[i] = ms->config.currentGraspZ;
	  ms->config.classGraspZsSet[i] = 1;
	  cout << " ...done " << ms->config.currentGraspZ << " ." << endl;
	} else {
	  cout << anode.type() << " Failed to load classGraspZs from " << this_range_path << endl;
	  ms->config.currentGraspZ = 0;
	  ms->config.classGraspZs[i] = ms->config.currentGraspZ;
	  ms->config.classGraspZsSet[i] = 0;
	}
      }

      fsfI["rangeMap"] >> ms->config.classRangeMaps[i]; 

      fsfI.release();
      cout << "Loaded rangeMap from " << this_range_path << ms->config.classRangeMaps[i].size() << endl; 

    } else {
      ms->config.classRangeMaps[i] = Mat(1, 1, CV_64F);

      cout << "Failed to load rangeMap from " << this_range_path << endl; 
    }
  }

  {
    string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/pickMemories/";
    string this_grasp_path = dirToMakePath + "graspMemories.yml";

    cout << "  tryToLoadRangeMap: " << this_grasp_path << endl;

    FileStorage fsfI;
    fsfI.open(this_grasp_path, FileStorage::READ);
    if (fsfI.isOpened()) {

      fsfI["graspMemoryTries1"] >> ms->config.classGraspMemoryTries1[i];
      fsfI["graspMemoryPicks1"] >> ms->config.classGraspMemoryPicks1[i];
      fsfI["graspMemoryTries2"] >> ms->config.classGraspMemoryTries2[i];
      fsfI["graspMemoryPicks2"] >> ms->config.classGraspMemoryPicks2[i];
      fsfI["graspMemoryTries3"] >> ms->config.classGraspMemoryTries3[i];
      fsfI["graspMemoryPicks3"] >> ms->config.classGraspMemoryPicks3[i];
      fsfI["graspMemoryTries4"] >> ms->config.classGraspMemoryTries4[i];
      fsfI["graspMemoryPicks4"] >> ms->config.classGraspMemoryPicks4[i];

      fsfI["heightMemoryTries"] >> ms->config.classHeightMemoryTries[i];
      fsfI["heightMemoryPicks"] >> ms->config.classHeightMemoryPicks[i];

      fsfI.release();
      cout << "Loaded classGraspMemoryTries1 from " << this_grasp_path << ms->config.classGraspMemoryTries1[i].size() << endl; 
      cout << "Loaded classGraspMemoryPicks1 from " << this_grasp_path << ms->config.classGraspMemoryPicks1[i].size() << endl; 
      cout << "Loaded classGraspMemoryTries2 from " << this_grasp_path << ms->config.classGraspMemoryTries2[i].size() << endl; 
      cout << "Loaded classGraspMemoryPicks2 from " << this_grasp_path << ms->config.classGraspMemoryPicks2[i].size() << endl; 
      cout << "Loaded classGraspMemoryTries3 from " << this_grasp_path << ms->config.classGraspMemoryTries3[i].size() << endl; 
      cout << "Loaded classGraspMemoryPicks3 from " << this_grasp_path << ms->config.classGraspMemoryPicks3[i].size() << endl; 
      cout << "Loaded classGraspMemoryTries4 from " << this_grasp_path << ms->config.classGraspMemoryTries4[i].size() << endl; 
      cout << "Loaded classGraspMemoryPicks4 from " << this_grasp_path << ms->config.classGraspMemoryPicks4[i].size() << endl; 

      cout << "Loaded classHeightMemoryTries from " << this_grasp_path << ms->config.classHeightMemoryTries[i].size() << endl;
      cout << "Loaded classHeightMemoryPicks from " << this_grasp_path << ms->config.classHeightMemoryPicks[i].size() << endl;
    } else {
      ms->config.classGraspMemoryTries1[i] = Mat(1, 1, CV_64F);
      ms->config.classGraspMemoryPicks1[i] = Mat(1, 1, CV_64F);
      ms->config.classGraspMemoryTries2[i] = Mat(1, 1, CV_64F);
      ms->config.classGraspMemoryPicks2[i] = Mat(1, 1, CV_64F);
      ms->config.classGraspMemoryTries3[i] = Mat(1, 1, CV_64F);
      ms->config.classGraspMemoryPicks3[i] = Mat(1, 1, CV_64F);
      ms->config.classGraspMemoryTries4[i] = Mat(1, 1, CV_64F);
      ms->config.classGraspMemoryPicks4[i] = Mat(1, 1, CV_64F);

      ms->config.classHeightMemoryTries[i] = Mat(1, 1, CV_64F);
      ms->config.classHeightMemoryPicks[i] = Mat(1, 1, CV_64F);

      cout << "Failed to load grasp memories from " << this_grasp_path << endl; 
    }
  }

  {
    {
      string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/servoCrops/";
      string this_ag_path = dirToMakePath + "aerialHeight0Gradients.yml";

      FileStorage fsfI;
      fsfI.open(this_ag_path, FileStorage::READ);
      if (fsfI.isOpened()) {
	fsfI["aerialHeight0Gradients"] >> ms->config.classHeight0AerialGradients[i]; 
	fsfI.release();
	cout << "Loaded aerial height 0 gradient from " << this_ag_path << ms->config.classHeight0AerialGradients[i].size() << endl;
      } else {
	ms->config.classHeight0AerialGradients[i] = Mat(1, 1, CV_64F);
	cout << "Failed to load aerialHeight0Gradients from " << this_ag_path << endl; 
      }
    }
    {
      string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/servoCrops/";
      string this_ag_path = dirToMakePath + "aerialHeight1Gradients.yml";

      FileStorage fsfI;
      fsfI.open(this_ag_path, FileStorage::READ);
      if (fsfI.isOpened()) {
	fsfI["aerialHeight1Gradients"] >> ms->config.classHeight1AerialGradients[i]; 
	fsfI.release();
	cout << "Loaded aerial height 1 gradient from " << this_ag_path << ms->config.classHeight1AerialGradients[i].size() << endl;
      } else {
	ms->config.classHeight1AerialGradients[i] = Mat(1, 1, CV_64F);
	cout << "Failed to load aerialHeight1Gradients from " << this_ag_path << endl; 
      }
    }
    {
      string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/servoCrops/";
      string this_ag_path = dirToMakePath + "aerialHeight2Gradients.yml";

      FileStorage fsfI;
      fsfI.open(this_ag_path, FileStorage::READ);
      if (fsfI.isOpened()) {
	fsfI["aerialHeight2Gradients"] >> ms->config.classHeight2AerialGradients[i]; 
	fsfI.release();
	cout << "Loaded aerial height 2 gradient from " << this_ag_path << ms->config.classHeight2AerialGradients[i].size() << endl;
      } else {
	ms->config.classHeight2AerialGradients[i] = Mat(1, 1, CV_64F);
	cout << "Failed to load aerialHeight2Gradients from " << this_ag_path << endl; 
      }
    }
    {
      string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/servoCrops/";
      string this_ag_path = dirToMakePath + "aerialHeight3Gradients.yml";

      FileStorage fsfI;
      fsfI.open(this_ag_path, FileStorage::READ);
      if (fsfI.isOpened()) {
	fsfI["aerialHeight3Gradients"] >> ms->config.classHeight3AerialGradients[i]; 
	fsfI.release();
	cout << "Loaded aerial height 3 gradient from " << this_ag_path << ms->config.classHeight3AerialGradients[i].size() << endl;
      } else {
	ms->config.classHeight3AerialGradients[i] = Mat(1, 1, CV_64F);
	cout << "Failed to load aerialHeight3Gradients from " << this_ag_path << endl; 
      }
    }
    cout << "Initializing classAerialGradients with classAerialHeight0Gradients." << endl;
    ms->config.classAerialGradients[i] = ms->config.classHeight0AerialGradients[i];
    {
      guard3dGrasps(ms);
      string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/ein/3dGrasps/";
      string this_grasp_path = dirToMakePath + "3dGrasps.yml";

      FileStorage fsvI;
      cout << "Reading grasp information from " << this_grasp_path << " ..."; cout.flush();
      fsvI.open(this_grasp_path, FileStorage::READ);

      {
	FileNode anode = fsvI["grasps"];
	{
	  FileNode bnode = anode["size"];
	  FileNodeIterator itb = bnode.begin();
	  int tng = *itb;
	  ms->config.class3dGrasps[i].resize(0);

	  FileNode cnode = anode["graspPoses"];
	  FileNodeIterator itc = cnode.begin(), itc_end = cnode.end();
	  int numLoadedPoses = 0;
	  for ( ; itc != itc_end; itc++, numLoadedPoses++) {
	    Grasp buf;
	    buf.readFromFileNodeIterator(itc);
	    cout << " read 3d pose: " << buf; cout.flush();
	    ms->config.class3dGrasps[i].push_back(buf);
	  }
	  if (numLoadedPoses != tng) {
	    CONSOLE_ERROR(ms, "Did not load the expected number of poses.");
	  }
	  cout << "Expected to load " << tng << " 3d poses, loaded " << numLoadedPoses << " ..." << endl; cout.flush();
	}
      }
      {
	FileNode anode = fsvI["placeUnderPoints"];
	{
	  FileNode bnode = anode["size"];
	  FileNodeIterator itb = bnode.begin();
	  int tng = *itb;
	  ms->config.classPlaceUnderPoints[i].resize(0);

	  FileNode cnode = anode["pupPoses"];
	  FileNodeIterator itc = cnode.begin(), itc_end = cnode.end();
	  int numLoadedPoses = 0;
	  for ( ; itc != itc_end; itc++, numLoadedPoses++) {
	    eePose buf;
	    buf.readFromFileNodeIterator(itc);
	    cout << " read pup pose: " << buf; cout.flush();
	    ms->config.classPlaceUnderPoints[i].push_back(buf);
	  }
	  if (numLoadedPoses != tng) {
	    CONSOLE_ERROR(ms, "Did not load the expected number of poses.");
	  }
	  cout << "Expected to load " << tng << " pup poses, loaded " << numLoadedPoses << " ..." << endl; cout.flush();
	}
      }
      {
	FileNode anode = fsvI["placeOverPoints"];
	{
	  FileNode bnode = anode["size"];
	  FileNodeIterator itb = bnode.begin();
	  int tng = *itb;
	  ms->config.classPlaceOverPoints[i].resize(0);

	  FileNode cnode = anode["popPoses"];
	  FileNodeIterator itc = cnode.begin(), itc_end = cnode.end();
	  int numLoadedPoses = 0;
	  for ( ; itc != itc_end; itc++, numLoadedPoses++) {
	    eePose buf;
	    buf.readFromFileNodeIterator(itc);
	    cout << " read pop pose: " << buf; cout.flush();
	    ms->config.classPlaceOverPoints[i].push_back(buf);
	  }
	  if (numLoadedPoses != tng) {
	    CONSOLE_ERROR(ms, "Did not load the expected number of poses.");
	  }
	  cout << "Expected to load " << tng << " pop poses, loaded " << numLoadedPoses << " ..." << endl; cout.flush();
	}
      }

      cout << "done.";
    }
  }

  {
    guardSceneModels(ms);
    string fname = sceneModelFile(ms, thisLabelName);
    if (!boost::filesystem::exists(fname)) {
      ms->config.class_scene_models[i] = Scene::createEmptyScene(ms);
    } else {
      ms->config.class_scene_models[i] = Scene::createFromFile(ms, fname);
    }

  }
}

void processSaliency(Mat in, Mat out) {
//  out = in.clone();
//
//  double saliencyPreSigma = 4.0;//0.5;//2.0;
//  GaussianBlur(out, out, cv::Size(0,0), saliencyPreSigma);
//  
//  double tMax = -INFINITY;
//  double tMin =  INFINITY;
//  for (int x = 0; x < in.cols; x++) {
//    for (int y = 0; y < in.rows; y++) {
//      tMax = max(tMax, out.at<double>(y,x));
//      tMin = min(tMin, out.at<double>(y,x));
//    }
//  }
//
//  double saliencyThresh = 0.33*(tMax-tMin) + tMin;
//  for (int x = 0; x < in.cols; x++) {
//    for (int y = 0; y < in.rows; y++) {
//      if (out.at<double>(y,x) >= saliencyThresh)
//	out.at<double>(y,x) = 1;
//      else
//	out.at<double>(y,x) = 0;
//    }
//  }
//
//  double saliencyPostSigma = 0.5;//4.0;//0.5;//2.0;
//  GaussianBlur(out, out, cv::Size(0,0), saliencyPostSigma);
  out = in.clone();

  double saliencyPreSigma = 4.0;//0.5;//2.0;
  GaussianBlur(out, out, cv::Size(0,0), saliencyPreSigma);
  
  double tMax = -INFINITY;
  double tMin =  INFINITY;
  for (int x = 0; x < in.cols; x++) {
    for (int y = 0; y < in.rows; y++) {
      tMax = max(tMax, out.at<double>(y,x));
      tMin = min(tMin, out.at<double>(y,x));
    }
  }

  double saliencyThresh = 0.1*(tMax-tMin) + tMin;
  for (int x = 0; x < in.cols; x++) {
    for (int y = 0; y < in.rows; y++) {
      if (out.at<double>(y,x) >= saliencyThresh)
	out.at<double>(y,x) = 1;
      else
	out.at<double>(y,x) = 0;
    }
  }

  double saliencyPostSigma = 4.0;//0.5;//4.0;//0.5;//2.0;
  GaussianBlur(out, out, cv::Size(0,0), saliencyPostSigma);
}


void mapxyToij(double xmin, double ymin, double mapStep, double x, double y, int * i, int * j) 
{
  *i = round((x - xmin) / mapStep);
  *j = round((y - ymin) / mapStep);
}
void mapijToxy(double xmin, double ymin, double mapStep, int i, int j, double * x, double * y) 
{
  *x = xmin + i * mapStep;
  *y = ymin + j * mapStep;
}
bool cellIsSearched(double fenceXMin, double fenceXMax, double fenceYMin, double fenceYMax, double xmin, double ymin, double mapStep, int i, int j) {
  double x, y;
  mapijToxy(xmin, ymin, mapStep, i, j, &x, &y);
  return positionIsSearched(fenceXMin, fenceXMax, fenceYMin, fenceYMax, x, y);
}

bool positionIsSearched(double fenceXMin, double fenceXMax, double fenceYMin, double fenceYMax, double x, double y) {
  if ((fenceXMin <= x && x <= fenceXMax) &&
      (fenceYMin <= y && y <= fenceYMax)) {
    return true;
  } else {
    return false;
  }

}


gsl_matrix * mapCellToPolygon(MachineState * ms, int map_i, int map_j) {
  
  double min_x, min_y;
  mapijToxy(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, map_i, map_j, &min_x, &min_y);
  double max_x = min_x + ms->config.mapStep;
  double max_y = min_y + ms->config.mapStep;
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

bool isBoxMemoryIkPossible(MachineState * ms, BoxMemory b) {
  int toReturn = 1;
  {
    int i, j;
    mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, b.top.px, b.top.py, &i, &j);
    toReturn &= isCellIkPossible(ms, i, j);
  }
  {
    int i, j;
    mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, b.bot.px, b.bot.py, &i, &j);
    toReturn &= isCellIkPossible(ms, i, j);
  }
  {
    int i, j;
    mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, b.bot.px, b.top.py, &i, &j);
    toReturn &= isCellIkPossible(ms, i, j);
  }
  {
    int i, j;
    mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, b.top.px, b.bot.py, &i, &j);
    toReturn &= isCellIkPossible(ms, i, j);
  }
  return toReturn;
}

bool isBlueBoxIkPossible(MachineState * ms, cv::Point tbTop, cv::Point tbBot) {
  double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;
  int toReturn = 1;
  {
    double tbx, tby;
    int tbi, tbj;
    pixelToGlobal(ms, tbTop.x, tbTop.y, zToUse, &tbx, &tby);
    mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, tbx, tby, &tbi, &tbj);
    toReturn &= isCellIkPossible(ms, tbi, tbj);
  }
  {
    double tbx, tby;
    int tbi, tbj;
    pixelToGlobal(ms, tbBot.x, tbBot.y, zToUse, &tbx, &tby);
    mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, tbx, tby, &tbi, &tbj);
    toReturn &= isCellIkPossible(ms, tbi, tbj);
  }
  {
    double tbx, tby;
    int tbi, tbj;
    pixelToGlobal(ms, tbTop.x, tbBot.y, zToUse, &tbx, &tby);
    mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, tbx, tby, &tbi, &tbj);
    toReturn &= isCellIkPossible(ms, tbi, tbj);
  }
  {
    double tbx, tby;
    int tbi, tbj;
    pixelToGlobal(ms, tbBot.x, tbTop.y, zToUse, &tbx, &tby);
    mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, tbx, tby, &tbi, &tbj);
    toReturn &= isCellIkPossible(ms, tbi, tbj);
  }
  return toReturn;
}

bool boxMemoryIntersectsMapCell(MachineState * ms, BoxMemory b, int map_i, int map_j) {
  gsl_matrix * bpolygon = boxMemoryToPolygon(b);

  gsl_matrix * map_cell = mapCellToPolygon(ms, map_i, map_j);

  bool result = math2d_overlaps(bpolygon, map_cell);
  gsl_matrix_free(bpolygon);
  gsl_matrix_free(map_cell);

  return result;
}

bool boxMemoryIntersectPolygons(BoxMemory b1, BoxMemory b2) {
  gsl_matrix * p1 = boxMemoryToPolygon(b1);
  gsl_matrix * p2 = boxMemoryToPolygon(b2);

  bool result = math2d_overlaps(p1, p2);
  
  gsl_matrix_free(p1);
  gsl_matrix_free(p2);

  return result;
}

bool boxMemoryIntersectCentroid(BoxMemory b1, BoxMemory b2) {
  gsl_matrix * p1 = boxMemoryToPolygon(b1);
  gsl_matrix * p2 = boxMemoryToPolygon(b2);
  gsl_vector * p1_center = math2d_point(b1.centroid.px, b1.centroid.py);
  gsl_vector * p2_center = math2d_point(b2.centroid.px, b2.centroid.py);

  bool result;
  if (math2d_is_interior_point(p2_center, p1) || 
      math2d_is_interior_point(p1_center, p2)) {
    result = true;
  } else {
    result = false;
  }
    
  gsl_matrix_free(p1);
  gsl_matrix_free(p2);
  gsl_vector_free(p1_center);
  gsl_vector_free(p2_center);

  return result;
}


vector<BoxMemory> memoriesForClass(MachineState * ms, int classIdx) {
  int unused = 0;
  vector<BoxMemory> results = memoriesForClass(ms, classIdx, &unused);
  return results;
}

vector<BoxMemory> memoriesForClass(MachineState * ms, int classIdx, int * memoryIdxOfFirst) {
  vector<BoxMemory> results;
  int haventFoundFirst = 1;
  for (int j = 0; j < ms->config.blueBoxMemories.size(); j++) {
    if (ms->config.blueBoxMemories[j].labeledClassIndex == classIdx) {
      results.push_back(ms->config.blueBoxMemories[j]);
      if ( haventFoundFirst && (ms->config.blueBoxMemories[j].lockStatus == POSE_REPORTED) ) {
		*memoryIdxOfFirst = j;
		haventFoundFirst = 0;
      }
    }
  }
  return results;
}

int getBoxMemoryOfLabel(MachineState * ms, string label, int * idxOfLabel, BoxMemory * out) {
  // note that this function does not check for a lock
  int class_idx = classIdxForName(ms, label);
  if (class_idx != -1) {
    vector<BoxMemory> focusedClassMemories = memoriesForClass(ms, class_idx);
    if (focusedClassMemories.size() > 0) {
      (*out) = focusedClassMemories[0];
      (*idxOfLabel) = class_idx;
      return 1;
    } else {
      return 0;
    }
  } else {
    return 0;
  }
}

int placementPoseLabel1BetweenLabel2AndLabel3(MachineState * ms, string label1, 
  // XXX guard affPXPs
  // XXX guard affPXPs
  // XXX guard affPXPs
  string label2, string label3, eePose * out) {

  eePose label1Pose;
  int success = 0;
  int label1Idx = -1;
  BoxMemory label1Mem;
  success = getBoxMemoryOfLabel(ms, label1, &label1Idx, &label1Mem);
  if (success) {
    int label2Idx = -1;
    BoxMemory label2Mem;
    success = getBoxMemoryOfLabel(ms, label2, &label2Idx, &label2Mem);
    if (success) {
      int label3Idx = -1;
      BoxMemory label3Mem;
      success = getBoxMemoryOfLabel(ms, label3, &label3Idx, &label3Mem);
      if (success) {

	eePose label1PickOffset = label1Mem.aimedPose.minusP(label1Mem.affPlaceUnderPoses[0]);
	
	eePose betweenL2AndL3PlaceOver = label2Mem.affPlaceOverPoses[0].plusP(label3Mem.affPlaceOverPoses[0]);
	betweenL2AndL3PlaceOver = betweenL2AndL3PlaceOver.multP(0.5);

	label1Pose = betweenL2AndL3PlaceOver.plusP(label1PickOffset);
	double thisPickZ = 0.0;
	double label2TipZAtPick = 0;
	if ( (ms->config.classGraspZsSet.size() > label2Idx) && 
	     (ms->config.classGraspZs.size() > label2Idx) &&
	     (ms->config.classGraspZsSet[label2Idx] == 1) ) {
	  label2TipZAtPick = -ms->config.classGraspZs[label2Idx] - ms->config.pickFlushFactor;
	} else {
	  label2TipZAtPick = (-label2Mem.trZ) - ms->config.pickFlushFactor;
	}

	double totalZOffset = label2TipZAtPick;
	thisPickZ = -ms->config.currentTableZ + totalZOffset;

	label1Pose.pz = thisPickZ;
	label1Pose.copyQ(ms->config.beeHome);
	(*out) = label1Pose;
	return 1;
      } else {
	cout << label2 << " not found, exiting and clearing stack." << endl;
	ms->clearStack();
	return 0;
      }
    } else {
      cout << label2 << " not found, exiting and clearing stack." << endl;
      ms->clearStack();
      return 0;
    }
  } else {
    cout << label1 << " not found, exiting and clearing stack." << endl;
    ms->clearStack();
    return 0;
  }
}

int placementPoseLabel1AboveLabel2By3dFirst(MachineState * ms, string label1, string label2, double zAbove, eePose * out) {
// XXX this is not correct
  // XXX guard affPXPs
  // XXX guard affPXPs
  // XXX guard affPXPs
  eePose label1Out;
  int success = 0;
  int label1Idx = -1;
  BoxMemory label1Mem;
  success = getBoxMemoryOfLabel(ms, label1, &label1Idx, &label1Mem);
  if (success) {
    int label2Idx = -1;
    BoxMemory label2Mem;
    success = getBoxMemoryOfLabel(ms, label2, &label2Idx, &label2Mem);
    if (success) {

      eePose deltaXY = label2Mem.affPlaceOverPoses[0].minusP(label1Mem.affPlaceUnderPoses[0]);
      label1Out = ms->config.lastPickPose.plusP(deltaXY);

      double label2DeltaZ = 0;
      label2DeltaZ = (label2Mem.affPlaceOverPoses[0].pz - (-ms->config.currentTableZ)) - ms->config.pickFlushFactor + zAbove;
      label1Out.pz = ms->config.lastPickPose.pz + label2DeltaZ;

      label1Out.copyQ(ms->config.lastPickPose);

      (*out) = label1Out;
      return 1;
    } else {
      cout << label2 << " not found, exiting and clearing stack." << endl;
      ms->clearStack();
      return 0;
    }
  } else {
    cout << label1 << " not found, exiting and clearing stack." << endl;
    ms->clearStack();
    return 0;
  }
}

int placementPoseLabel1AboveLabel2By(MachineState * ms, string label1, string label2, double zAbove, eePose * out) {
  // XXX guard affPXPs
  // XXX guard affPXPs
  // XXX guard affPXPs
  eePose label2Pose;
  int success = 0;
  int label1Idx = -1;
  BoxMemory label1Mem;
  success = getBoxMemoryOfLabel(ms, label1, &label1Idx, &label1Mem);
  if (success) {
    int label2Idx = -1;
    BoxMemory label2Mem;
    success = getBoxMemoryOfLabel(ms, label2, &label2Idx, &label2Mem);
    if (success) {
      eePose label1PickOffset = label1Mem.aimedPose.minusP(label1Mem.affPlaceUnderPoses[0]);
      label2Pose = label2Mem.affPlaceOverPoses[0].plusP(label1PickOffset);
      double thisPickZ = 0.0;
      //. label2TipZAtPick is the height of the place over point above the table
      double label2TipZAtPick = 0;
      label2TipZAtPick = (label2Mem.affPlaceOverPoses[0].pz - (-ms->config.currentTableZ)) - ms->config.pickFlushFactor;

      double totalZOffset = zAbove + label2TipZAtPick;
      if ( (ms->config.classGraspZsSet.size() > label1Idx) && 
	   (ms->config.classGraspZs.size() > label1Idx) &&
	   (ms->config.classGraspZsSet[label1Idx] == 1) ) {
//cout << "YYY cGZ: " << -ms->config.classGraspZs[label1Idx] << endl;
//cout <<  "YYY : " << label2Mem.affPlaceOverPoses[0] << ms->config.pickFlushFactor << endl;
	thisPickZ = -ms->config.currentTableZ + -ms->config.classGraspZs[label1Idx] + totalZOffset;
      } else {
	thisPickZ = -ms->config.currentTableZ + (-label1Mem.trZ) + totalZOffset;
      }



      label2Pose.pz = thisPickZ;
      label2Pose.copyQ(ms->config.beeHome);
cout << "ZZZ currentTableZ: " << ms->config.currentTableZ << " thisPickZ: " << thisPickZ << endl; 
      (*out) = label2Pose;
      return 1;
    } else {
      cout << label2 << " not found, exiting and clearing stack." << endl;
      ms->clearStack();
      return 0;
    }
  } else {
    cout << label1 << " not found, exiting and clearing stack." << endl;
    ms->clearStack();
    return 0;
  }
}

int placementPoseHeldAboveLabel2By(MachineState * ms, string label2, double zAbove, eePose * out) {
  // XXX guard affPXPs
  // XXX guard affPXPs
  // XXX guard affPXPs
  eePose label1Out;
  int success = 0;
  int label1Idx = -1;
  BoxMemory labelHeldMem;

  int tbb = ms->config.targetBlueBox;
  if (tbb < ms->config.blueBoxMemories.size()) {
    labelHeldMem = ms->config.blueBoxMemories[tbb];  
    success = 1;  
  } else {
    success = 0;  
  }

  eePose heldPickedPose = labelHeldMem.pickedPose;

  if (success) {
    int label2Idx = -1;
    BoxMemory label2Mem;
    success = getBoxMemoryOfLabel(ms, label2, &label2Idx, &label2Mem);
    if (success) {

      eePose deltaXY = label2Mem.affPlaceOverPoses[0].minusP(labelHeldMem.affPlaceUnderPoses[0]);
      label1Out = heldPickedPose.plusP(deltaXY);


      double label2DeltaZ = 0;
      label2DeltaZ = (label2Mem.affPlaceOverPoses[0].pz - (-ms->config.currentTableZ)) - ms->config.pickFlushFactor + zAbove;
      label1Out.pz = heldPickedPose.pz + label2DeltaZ;

      label1Out.copyQ(heldPickedPose);

      cout << "placementPoseHeldAboveLabel2By   heldPickedPose, deltaXY, label2DeltaZ: " << heldPickedPose << " " << deltaXY << " " << label2DeltaZ << endl;

      (*out) = label1Out;
      return 1;
    } else {
      cout << label2 << " not found, exiting and clearing stack." << endl;
      ms->clearStack();
      return 0;
    }
  } else {
    cout << "held object (target blue box)  not found, exiting and clearing stack." << endl;
    ms->clearStack();
    return 0;
  }
}

void recordBlueBoxInHistogram(MachineState * ms, int idx) {
  if ( (idx > -1) && (idx < ms->config.bLabels.size()) ) {
    int thisLabel = ms->config.bLabels[idx];
    ms->config.chHistogram.at<double>(0,thisLabel)++;
  }
}

void computeClassificationDistributionFromHistogram(MachineState * ms) {
  int thisNC = ms->config.chHistogram.cols; 
  assert( thisNC == ms->config.chDistribution.cols );
  double total = 0.0;
  for (int i = 0; i < thisNC; i++) {
    total = total + ms->config.chHistogram.at<double>(0,i);
  }
  if (total < EPSILON) {
    total = 1.0;
  } else {
  } // do nothing
  for (int i = 0; i < thisNC; i++) {
    ms->config.chDistribution.at<double>(0,i) = ms->config.chHistogram.at<double>(0,i) / total;
  }
}

bool cellIsMapped(MachineState * ms, int i, int j) {
  double x, y;
  mapijToxy(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, i, j, &x, &y);
  return positionIsMapped(ms, x, y);
}
bool positionIsMapped(MachineState * ms, double x, double y) {
  if ((ms->config.mapRejectFenceXMin <= x && x <= ms->config.mapRejectFenceXMax) &&
      (ms->config.mapRejectFenceYMin <= y && y <= ms->config.mapRejectFenceYMax)) {
    return true;
  } else {
    return false;
  }
}

// TODO XXX make clearance status enum
bool isCellInPursuitZone(MachineState * ms, int i, int j) {
  return ( (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 1) ||
	   (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 2) );
} 
bool isCellInPatrolZone(MachineState * ms, int i, int j) {
  return (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 2);
} 

bool isCellInteresting(MachineState * ms, int i, int j) {
  if ( (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 1) ||
       (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 2) ) {
    return ( ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime < ms->config.lastScanStarted );
  } else {
    return false;
  }
} 
void markCellAsInteresting(MachineState * ms, int i, int j) {
  if ( (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 1) ||
       (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 2) ) {
    ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime = ros::Time(0.001);
    return;
  } else {
    return;
  }
} 
void markCellAsNotInteresting(MachineState * ms, int i, int j) {
  if ( (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 1) ||
       (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 2) ) {
    ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime = ros::Time::now() + ros::Duration(VERYBIGNUMBER);
    return;
  } else {
    return;
  }
} 

bool isCellIkColliding(MachineState * ms, int i, int j) {
  return (ms->config.ikMap[i + ms->config.mapWidth * j] == IK_LIKELY_IN_COLLISION);
} 
bool isCellIkPossible(MachineState * ms, int i, int j) {
  return (ms->config.ikMap[i + ms->config.mapWidth * j] == IK_GOOD);
} 
bool isCellIkImpossible(MachineState * ms, int i, int j) {
  return (ms->config.ikMap[i + ms->config.mapWidth * j] == IK_FAILED);
} 


int blueBoxForPixel(MachineState * ms, int px, int py)
{
  for (int c = 0; c < ms->config.bTops.size(); c++) {
    if ((ms->config.bTops[c].x <= px && px <= ms->config.bBots[c].x) &&
        (ms->config.bTops[c].y <= py && py <= ms->config.bBots[c].y)) {
      return c;
    }
  }
  return -1;
}

int skirtedBlueBoxForPixel(MachineState * ms, int px, int py, int skirtPixels) {
  vector<cv::Point> newBTops;
  vector<cv::Point> newBBots;
  newBTops.resize(ms->config.bBots.size());
  newBBots.resize(ms->config.bTops.size()); 
  for (int c = 0; c < ms->config.bTops.size(); c++) {
    newBTops[c].x = ms->config.bTops[c].x-skirtPixels;
    newBTops[c].y = ms->config.bTops[c].y-skirtPixels;
    newBBots[c].x = ms->config.bBots[c].x+skirtPixels;
    newBBots[c].y = ms->config.bBots[c].y+skirtPixels;
  }

  for (int c = 0; c < newBTops.size(); c++) {
    if ((newBTops[c].x <= px && px <= newBBots[c].x) &&
        (newBTops[c].y <= py && py <= newBBots[c].y)) {
      return c;
    }
  }
  return -1;
}

void randomizeNanos(MachineState * ms, ros::Time * time) {
  double nanoseconds = rk_double(&ms->config.random_state) * 1000;
  time->nsec = nanoseconds;
}

void voidMapRegion(MachineState * ms, double xc, double yc) {
  double voidRegionWidth = 0.1;
  double voidTimeGap = 60.0;

  int mxs=0,mxe=0,mys=0,mye=0;
  mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, xc-voidRegionWidth, yc-voidRegionWidth, &mxs, &mys);
  mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, xc+voidRegionWidth, yc+voidRegionWidth, &mxe, &mye);
  mxs = max(0,min(mxs, (int) ms->config.mapWidth));
  mxe = max(0,min(mxe, (int) ms->config.mapWidth));
  mys = max(0,min(mys, (int) ms->config.mapWidth));
  mye = max(0,min(mye, (int) ms->config.mapWidth));
  ros::Time startTime = ros::Time::now();
  for (int i = mxs; i < mxe; i++) {
    for(int j = mys; j < mye; j++) {
      ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime = startTime - ros::Duration(ms->config.mapBlueBoxCooldown) - ros::Duration(voidTimeGap);
      ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime.nsec = 0.0;
      ms->config.objectMap[i + ms->config.mapWidth * j].detectedClass = -1;
      ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount = 0;
      ms->config.objectMap[i + ms->config.mapWidth * j].r = 0;
      ms->config.objectMap[i + ms->config.mapWidth * j].g = 0;
      ms->config.objectMap[i + ms->config.mapWidth * j].b = 0;

      int goAgain = 1;
      while (goAgain) {
	goAgain = 0;
	vector<BoxMemory> newMemories;
	for (int k = 0; k < ms->config.blueBoxMemories.size(); k++) {
	  BoxMemory b = ms->config.blueBoxMemories[k];
	  if (boxMemoryIntersectsMapCell(ms, b,i,j)) {
	    // if we remove one, go again!
	    goAgain = 1;
	  } else {
	    newMemories.push_back(b);
	  }
	}
	ms->config.blueBoxMemories = newMemories;
      }

    }
  }
}

void markMapAsCompleted(MachineState * ms) {
  double completionGap = 10.0;
  for (int i = 0; i < ms->config.mapWidth; i++) {
    for(int j = 0; j < ms->config.mapHeight; j++) {
      ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime = ms->config.lastScanStarted + ros::Duration(completionGap);

      ms->config.objectMap[i + ms->config.mapWidth * j].detectedClass = -1;
      ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount = 10;
      ms->config.objectMap[i + ms->config.mapWidth * j].r = 64;
      ms->config.objectMap[i + ms->config.mapWidth * j].g = 64;
      ms->config.objectMap[i + ms->config.mapWidth * j].b = 64;
    }
  }
}

void clearMapForPatrol(MachineState * ms) {
  ros::Time startTime = ros::Time::now();
  for (int i = 0; i < ms->config.mapWidth; i++) {
    for(int j = 0; j < ms->config.mapHeight; j++) {
      ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime = startTime - ros::Duration(ms->config.mapBlueBoxCooldown);
      // make the search go in order but strided
      if ((j % 10) == 0) {
	if ((i % 10) == 0) {
	  ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime.nsec = 1000.0*(double(j + i*ms->config.mapHeight)/double(ms->config.mapHeight*ms->config.mapWidth));
	} else {
	  ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime.nsec = 1000.0;
	}
      } else {
	ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime.nsec = 1000.0;
      }

      ms->config.objectMap[i + ms->config.mapWidth * j].detectedClass = -1;
      ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount = 0;
      ms->config.objectMap[i + ms->config.mapWidth * j].r = 0;
      ms->config.objectMap[i + ms->config.mapWidth * j].g = 0;
      ms->config.objectMap[i + ms->config.mapWidth * j].b = 0;
    }
  }
  ms->config.lastScanStarted = ros::Time::now();
}

void initializeMap(MachineState * ms) {
  ros::Time startTime = ros::Time::now();
  for (int i = 0; i < ms->config.mapWidth; i++) {
    for(int j = 0; j < ms->config.mapHeight; j++) {
      ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime = startTime - ros::Duration(ms->config.mapBlueBoxCooldown);
      // make the search more random
      randomizeNanos(ms, &ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime);


      ms->config.objectMap[i + ms->config.mapWidth * j].detectedClass = -1;
      ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount = 0;
      ms->config.objectMap[i + ms->config.mapWidth * j].r = 0;
      ms->config.objectMap[i + ms->config.mapWidth * j].g = 0;
      ms->config.objectMap[i + ms->config.mapWidth * j].b = 0;

      ms->config.ikMap[i + ms->config.mapWidth * j] = IK_GOOD;
      ms->config.clearanceMap[i + ms->config.mapWidth * j] = 0;
    }
  }

  for (int i = 0; i < ms->config.mapWidth; i++) {
    for (int j = 0; j < ms->config.mapHeight; j++) {
      for (int heightIdx = 0; heightIdx < ms->config.numIkMapHeights; heightIdx++) {
	ms->config.ikMapAtHeight[i  + ms->config.mapWidth * j + ms->config.mapWidth * ms->config.mapHeight * heightIdx] = IK_GOOD;
      }
    }
  }

  ms->config.lastScanStarted = ros::Time::now();
  ms->config.ikMapStartHeight = -ms->config.currentTableZ + ms->config.pickFlushFactor;
  ms->config.ikMapEndHeight = convertHeightIdxToGlobalZ(ms, ms->config.mappingHeightIdx);
}


void initializeViewers(MachineState * ms) {

  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  ms->config.objectViewerYCbCrBlur = Mat(camera->cam_bgr_img.rows, camera->cam_bgr_img.cols, CV_64FC3);


  ms->config.objectViewerGrayBlur = Mat(camera->cam_bgr_img.rows, camera->cam_bgr_img.cols, CV_64FC3);


  ms->config.accumulatedImage = Mat(camera->cam_bgr_img.rows, camera->cam_bgr_img.cols, CV_64FC3);


  ms->config.accumulatedImageMass = Mat(camera->cam_bgr_img.rows, camera->cam_bgr_img.cols, CV_64F);


  ms->config.objectViewerImage = camera->cam_bgr_img.clone();

}




////////////////////////////////////////////////
// end node definitions 
//
// start ein 
////////////////////////////////////////////////

int findClosestBlueBoxMemory(MachineState * ms, eePose targetPose, int classToSearch) {
  int closest_idx = -1;
  double min_square_dist = VERYBIGNUMBER;

  // either constrain to a class or do not
  if (classToSearch == -1) {
    for (int j = 0; j < ms->config.blueBoxMemories.size(); j++) {
      if (ms->config.blueBoxMemories[j].lockStatus == POSE_LOCK ||
	   ms->config.blueBoxMemories[j].lockStatus == POSE_REPORTED) {
	  double square_dist = 
	    eePose::squareDistance(targetPose, ms->config.blueBoxMemories[j].centroid);
	if (square_dist < min_square_dist) {
	  min_square_dist = square_dist;
	  closest_idx = j;
	}
      }
    }
  } else {
    for (int j = 0; j < ms->config.blueBoxMemories.size(); j++) {
      if (ms->config.blueBoxMemories[j].labeledClassIndex == classToSearch &&
	  (ms->config.blueBoxMemories[j].lockStatus == POSE_LOCK ||
	   ms->config.blueBoxMemories[j].lockStatus == POSE_REPORTED)) {
	  double square_dist = 
	    eePose::squareDistance(targetPose, ms->config.blueBoxMemories[j].centroid);
	if (square_dist < min_square_dist) {
	  min_square_dist = square_dist;
	  closest_idx = j;
	}
      }
    }
  }

  return closest_idx;
}

void fillRecognizedObjectArrayFromBlueBoxMemory(MachineState * ms, object_recognition_msgs::RecognizedObjectArray * roa) {
  roa->objects.resize(0);

  roa->header.stamp = ros::Time::now();
  roa->header.frame_id = "/base";


  // this sets all locked or reported boxes to lock
  for (int j = 0; j < ms->config.blueBoxMemories.size(); j++) {
    if (ms->config.blueBoxMemories[j].lockStatus == POSE_LOCK ||
	ms->config.blueBoxMemories[j].lockStatus == POSE_REPORTED) {
      ms->config.blueBoxMemories[j].lockStatus = POSE_LOCK;
    }
  }

  // for each class, this finds the centroid of the blue boxes of that class and
  //   selects the nearest one to report
  for (int class_i = 0; class_i < ms->config.classLabels.size(); class_i++) {
    string class_label = ms->config.classLabels[class_i];
    if (class_label != "background") {
      eePose centroid;
      centroid.px = 0;
      centroid.py = 0;
      centroid.pz = 0;
      int class_count = 0;
      for (int j = 0; j < ms->config.blueBoxMemories.size(); j++) {
	if (ms->config.blueBoxMemories[j].labeledClassIndex == class_i &&
	    (ms->config.blueBoxMemories[j].lockStatus == POSE_LOCK ||
	     ms->config.blueBoxMemories[j].lockStatus == POSE_REPORTED)) {
	  centroid.px += ms->config.blueBoxMemories[j].centroid.px;
	  centroid.py += ms->config.blueBoxMemories[j].centroid.py;
	  centroid.pz += ms->config.blueBoxMemories[j].centroid.pz;
	  class_count += 1;
	}
      }
      if (class_count == 0) {
	continue;
      }
      centroid.px = centroid.px / class_count;
      centroid.py = centroid.py / class_count;
      centroid.pz = centroid.pz / class_count;
/*
      int closest_idx = -1;
      double min_square_dist = VERYBIGNUMBER;

      for (int j = 0; j < ms->config.blueBoxMemories.size(); j++) {
	if (ms->config.blueBoxMemories[j].labeledClassIndex == class_i &&
	    (ms->config.blueBoxMemories[j].lockStatus == POSE_LOCK ||
	     ms->config.blueBoxMemories[j].lockStatus == POSE_REPORTED)) {
	  double square_dist = 
	    eePose::squareDistance(centroid, ms->config.blueBoxMemories[j].centroid);
	  if (square_dist < min_square_dist) {
	    min_square_dist = square_dist;
	    closest_idx = j;
	  }
	}
      }
*/
      int closest_idx = findClosestBlueBoxMemory(ms, centroid);


      if (closest_idx != -1) {
	ms->config.blueBoxMemories[closest_idx].lockStatus = POSE_REPORTED;

	geometry_msgs::Pose pose;
	int aI = roa->objects.size();
	roa->objects.resize(roa->objects.size() + 1);

	pose.position.x = ms->config.blueBoxMemories[closest_idx].centroid.px;
	pose.position.y = ms->config.blueBoxMemories[closest_idx].centroid.py;
	pose.position.z = ms->config.blueBoxMemories[closest_idx].centroid.pz;

	//cout << "blueBoxMemories: " << ms->config.blueBoxMemories[closest_idx].centroid.px << endl;
	//cout << "pose: " << pose.position.x << endl;

	roa->objects[aI].pose.pose.pose.position = pose.position;

	//cout << "roa objects x: " << roa->objects[aI].pose.pose.pose.position.x << endl;
	roa->objects[aI].type.key = class_label;

	roa->objects[aI].header = roa->header;
      }
    }
  }
}

// set exactly one blue box of each class to be reported
void promoteBlueBoxes(MachineState * ms) {
  // this sets all locked or reported boxes to lock
  for (int j = 0; j < ms->config.blueBoxMemories.size(); j++) {
    if (ms->config.blueBoxMemories[j].lockStatus == POSE_LOCK ||
	ms->config.blueBoxMemories[j].lockStatus == POSE_REPORTED) {
      ms->config.blueBoxMemories[j].lockStatus = POSE_LOCK;
    }
  }

  // for each class, this finds the centroid of the blue boxes of that class and
  //   selects the nearest one to report
  for (int class_i = 0; class_i < ms->config.classLabels.size(); class_i++) {
    string class_label = ms->config.classLabels[class_i];
    if (class_label != "background") {
      eePose centroid;
      centroid.px = 0;
      centroid.py = 0;
      centroid.pz = 0;
      int class_count = 0;
      for (int j = 0; j < ms->config.blueBoxMemories.size(); j++) {
	    if (ms->config.blueBoxMemories[j].labeledClassIndex == class_i &&
	        (ms->config.blueBoxMemories[j].lockStatus == POSE_LOCK ||
	         ms->config.blueBoxMemories[j].lockStatus == POSE_REPORTED)) {
	      centroid.px += ms->config.blueBoxMemories[j].centroid.px;
	      centroid.py += ms->config.blueBoxMemories[j].centroid.py;
	      centroid.pz += ms->config.blueBoxMemories[j].centroid.pz;
	      class_count += 1;
	    }
      }
      if (class_count == 0) {
		cout << "promoteBlueBoxes: none to report of class " << class_i << endl;
	    continue;
      }
      centroid.px = centroid.px / class_count;
      centroid.py = centroid.py / class_count;
      centroid.pz = centroid.pz / class_count;

      int closest_idx = findClosestBlueBoxMemory(ms, centroid);
      /*
      int closest_idx = -1;
      double min_square_dist = VERYBIGNUMBER;

      for (int j = 0; j < ms->config.blueBoxMemories.size(); j++) {
	    if (ms->config.blueBoxMemories[j].labeledClassIndex == class_i &&
	        (ms->config.blueBoxMemories[j].lockStatus == POSE_LOCK ||
	         ms->config.blueBoxMemories[j].lockStatus == POSE_REPORTED)) {
	      double square_dist = 
	        eePose::squareDistance(centroid, ms->config.blueBoxMemories[j].centroid);
	      if (square_dist < min_square_dist) {
	        min_square_dist = square_dist;
	        closest_idx = j;
	      }
	    }
      }
      */


      if (closest_idx != -1) {
		ms->config.blueBoxMemories[closest_idx].lockStatus = POSE_REPORTED;
		cout << "promoteBlueBoxes: promoting index " << closest_idx << " for class "  << class_i << endl;
	  }
    }
  }
}



void fillEinStateMsg(MachineState * ms, EinState * stateOut) {
  stateOut->zero_g = ms->config.zero_g_toggle;

  stateOut->movement_state = ms->config.currentMovementState;
  stateOut->patrol_state = ms->config.currentPatrolState;
  stateOut->patrol_mode = ms->config.currentPatrolMode;
  stateOut->place_mode = ms->config.currentPlaceMode;
  stateOut->idle_mode = ms->config.currentIdleMode;
  for (int i = 0; i < ms->call_stack.size(); i ++) {
    shared_ptr<Word> w = ms->call_stack[i];
    stateOut->call_stack.push_back(w->name());
  }

  for (int i = 0; i < ms->data_stack.size(); i ++) {
    shared_ptr<Word> w = ms->data_stack[i];
    stateOut->data_stack.push_back(w->repr());
  }


  object_recognition_msgs::RecognizedObjectArray roa;
  fillRecognizedObjectArrayFromBlueBoxMemory(ms, &roa);

  for (int i = 0; i < roa.objects.size(); i++) {
    stateOut->objects.push_back(roa.objects[i]);
  }

  std::map<std::string, shared_ptr<Word> >::iterator iter;

  for (iter = ms->variables.begin(); iter != ms->variables.end(); ++iter) {
    stateOut->words.push_back(iter->first);
  }
  std::vector<std::shared_ptr<Word> > words = register_word(NULL);
  for (int i = 0; i < words.size(); i++) {
    vector<string> names = words[i]->names();
    for (int j = 0; j < names.size(); j++) {
      stateOut->words.push_back(names[j]);
    }
  }

  stateOut->state_string = ms->currentState();
}

bool isFocusedClassValid(MachineState * ms) {
  if ((ms->config.focusedClass > -1) && (ms->config.focusedClass < ms->config.classLabels.size())) {
    return true;
  } else {
    return false;
  }
}

void initializeArm(MachineState * ms, string left_or_right_arm) {

  ros::NodeHandle n("~");

  time(&ms->config.firstTime);
  time(&ms->config.firstTimeRange);


#ifdef USE_ROBOT_BAXTER
  ms->config.robot_type = "baxter";
#elif defined(USE_ROBOT_AIBO)
  ms->config.robot_type = "aibo123";
#elif defined(USE_ROBOT_PIDRONE)
  ms->config.robot_type = "pidrone";
#else
  ms->config.robot_type = "none";
#endif

  ms->config.left_or_right_arm = left_or_right_arm;

  if (left_or_right_arm == "left") {
    ms->config.other_arm = "right";
  } else if (left_or_right_arm == "right") {
    ms->config.other_arm = "left";
  } else {
    ms->config.other_arm = "none";
  }

  //cout << "n namespace: " << n.getNamespace() << endl;
  ms->config.data_directory = ros::package::getPath("ein") + "/default";

  string console_topic = "/ein/" + ms->config.left_or_right_arm + "/console";
  ms->config.einConsolePub = n.advertise<EinConsole>(console_topic, 10);

  loadROSParamsFromArgs(ms);
  //cout << "mask_gripper: " << ms->config.mask_gripper << endl;
  //cout << "all_range_mode: " << ms->config.all_range_mode << endl;
  cout << "data_directory: " << ms->config.data_directory << endl;
  //<< "run_prefix: " << ms->config.run_prefix << endl << endl 
  //<< "vocab_file: " << ms->config.vocab_file << endl 
  //<< "knn_file: " << ms->config.knn_file << endl << "label_file: " << ms->config.label_file << endl
  //<< endl;

  ms->config.class_crops_path = ms->config.data_directory + "/objects/";

  unsigned long seed = 1;
  rk_seed(seed, &ms->config.random_state);

  //Camera * k2rgb = new Camera(ms, "left_kinect2_color_qhd", "/kinect2/qhd/image_color", ms->config.left_or_right_arm + "_hand", "k2rgb_tf_link");
  //Camera * k2rgb = new Camera(ms, "left_kinect2_color_hd", "/kinect2/hd/image_color", ms->config.left_or_right_arm + "_hand", "kinect2_link");
  //ms->config.cameras.push_back(k2rgb);

  //Camera * k2ir = new Camera(ms, "left_kinect2_ir", "/kinect2/sd/image_ir", ms->config.left_or_right_arm + "_hand", "k2ir_tf_link");
  //ms->config.cameras.push_back(k2ir);

  //Camera * k2depth = new Camera(ms, "left_kinect2_depth", "/kinect2/sd/image_depth", ms->config.left_or_right_arm + "_hand", "k2ir_tf_link");
  //ms->config.cameras.push_back(k2depth);


  ms->config.rec_objs_blue_memory = n.advertise<object_recognition_msgs::RecognizedObjectArray>("blue_memory_objects", 10);
  ms->config.markers_blue_memory = n.advertise<visualization_msgs::MarkerArray>("blue_memory_markers", 10);

  ms->config.ee_target_pub = n.advertise<geometry_msgs::Point>("pilot_target_" + ms->config.left_or_right_arm, 10);

  robotInitializeConfig(ms);

  ms->config.pickObjectUnderEndEffectorCommandCallbackSub = n.subscribe("/ein/eePickCommand", 1, &MachineState::pickObjectUnderEndEffectorCommandCallback, ms);
  ms->config.placeObjectInEndEffectorCommandCallbackSub = n.subscribe("/ein/eePlaceCommand", 1, &MachineState::placeObjectInEndEffectorCommandCallback, ms);
  ms->config.moveEndEffectorCommandCallbackSub = n.subscribe("/ein/eeMoveCommand", 1, &MachineState::moveEndEffectorCommandCallback, ms);


  if (ms->config.currentRobotMode == PHYSICAL || ms->config.currentRobotMode == SIMULATED) {
    ms->config.forthCommandSubscriber = n.subscribe("/ein/" + ms->config.left_or_right_arm + "/forth_commands", 1, 
						    &MachineState::forthCommandCallback, ms);
    ms->config.forthCommandPublisher = n.advertise<std_msgs::String>("/ein/" + ms->config.other_arm + "/forth_commands", 10);
  } else if (ms->config.currentRobotMode == SNOOP) {
    ms->config.forthCommandPublisher = n.advertise<std_msgs::String>("/ein/" + ms->config.left_or_right_arm + "/forth_commands", 10);
    ms->config.einSub = n.subscribe("/ein_" + ms->config.left_or_right_arm + "/state", 1, &MachineState::einStateCallback, ms);
  } else {
    assert(0);
  }


  ms->config.tfListener = new tf::TransformListener();
  ms->config.tfListener->setUsingDedicatedThread(true);

  string state_topic = "/ein/" + ms->config.left_or_right_arm + "/state";
  ms->config.einStatePub = n.advertise<EinState>(state_topic, 10);


  ms->config.frameGraySobel = Mat(1,1,CV_64F);

  initializeMap(ms);


  nodeInit(ms);
  detectorsInit(ms);
  irInit(ms);



  ms->config.lastMovementStateSet = ros::Time::now();

  {
    for (int i = 0; i < ms->config.numCornellTables; i++) {
      double yDelta = (ms->config.mapSearchFenceYMax - ms->config.mapSearchFenceXMin) / (double(i));
      eePose thisTablePose = ms->config.beeHome;
      thisTablePose.px = 0.75*(ms->config.mapSearchFenceXMax - ms->config.mapSearchFenceXMin) + ms->config.mapSearchFenceXMin; 
      thisTablePose.py = ms->config.mapSearchFenceYMin + (double(i) + 0.5)*yDelta;
      thisTablePose.pz = ms->config.currentTableZ; 
      ms->config.cornellTables.push_back(thisTablePose);
    }
  } 


  initializeMachine(ms);

}

void initializeArmGui(MachineState * ms, MainWindow * einMainWindow) {

//  ms->config.gripperMaskFirstContrastWindow = new EinWindow(NULL, ms);
//  ms->config.gripperMaskFirstContrastWindow->setWindowTitle("Gripper Mask First Contrast " + ms->config.left_or_right_arm);
//  einMainWindow->addWindow(ms->config.gripperMaskFirstContrastWindow);
//
//  ms->config.gripperMaskSecondContrastWindow = new EinWindow(NULL, ms);
//  ms->config.gripperMaskSecondContrastWindow->setWindowTitle("Gripper Mask Second Contrast " + ms->config.left_or_right_arm);
//  einMainWindow->addWindow(ms->config.gripperMaskSecondContrastWindow);
//
//  ms->config.gripperMaskDifferenceWindow = new EinWindow(NULL, ms);
//  ms->config.gripperMaskDifferenceWindow->setWindowTitle("Gripper Mask Difference " + ms->config.left_or_right_arm);
//  einMainWindow->addWindow(ms->config.gripperMaskDifferenceWindow);
//
//  ms->config.gripperMaskVarianceWindow = new EinWindow(NULL, ms);
//  ms->config.gripperMaskVarianceWindow->setWindowTitle("Gripper Mask Variance " + ms->config.left_or_right_arm);
//  einMainWindow->addWindow(ms->config.gripperMaskVarianceWindow);
//
//  ms->config.gripperMaskMeanWindow = new EinWindow(NULL, ms);
//  ms->config.gripperMaskMeanWindow->setWindowTitle("Gripper Mask Mean " + ms->config.left_or_right_arm);
//  einMainWindow->addWindow(ms->config.gripperMaskMeanWindow);
//
//  ms->config.gripperMaskSquaresWindow = new EinWindow(NULL, ms);
//  ms->config.gripperMaskSquaresWindow->setWindowTitle("Gripper Mask Squares " + ms->config.left_or_right_arm);
//  einMainWindow->addWindow(ms->config.gripperMaskSquaresWindow);

  ms->config.dogSnoutViewWindow = new EinWindow(NULL, ms);
  ms->config.dogSnoutViewWindow->setWindowTitle("Dog Snout View " + ms->config.left_or_right_arm);
  einMainWindow->addWindow(ms->config.dogSnoutViewWindow);
  ms->config.dogSnoutViewWindow->setVisible(true);


  ms->config.rangeogramWindow = new EinWindow(NULL, ms);
  ms->config.rangeogramWindow->setWindowTitle("Rangeogram View " + ms->config.left_or_right_arm);
  einMainWindow->addWindow(ms->config.rangeogramWindow);

  ms->config.wristViewWindow = new EinWindow(NULL, ms);
  ms->config.wristViewWindow->setWindowTitle("Wrist View " + ms->config.left_or_right_arm);
  einMainWindow->addWindow(ms->config.wristViewWindow);
  ms->config.wristViewWindow->setMouseCallBack(pilotCallbackFunc, ms);

  ms->config.renderedWristViewWindow = new EinWindow(NULL, ms);
  ms->config.renderedWristViewWindow->setWindowTitle("Rendered Wrist View " + ms->config.left_or_right_arm);
  einMainWindow->addWindow(ms->config.renderedWristViewWindow);
  ms->config.renderedWristViewWindow->setMouseCallBack(pilotCallbackFunc, ms);



  ms->config.coreViewWindow = new EinWindow(NULL, ms);
  ms->config.coreViewWindow->setWindowTitle("Core View " + ms->config.left_or_right_arm);
  einMainWindow->addWindow(ms->config.coreViewWindow);
  
  
  ms->config.faceViewWindow = new EinWindow(NULL, ms);
  ms->config.faceViewWindow->setWindowTitle("Face View " + ms->config.left_or_right_arm);
  einMainWindow->addWindow(ms->config.faceViewWindow);
  

  

  ms->config.mapBackgroundViewWindow = new EinWindow(NULL, ms);
  ms->config.mapBackgroundViewWindow->setWindowTitle("Hi Color Range Map View " + ms->config.left_or_right_arm);
  einMainWindow->addWindow(ms->config.mapBackgroundViewWindow);


  ms->config.densityViewerWindow = new EinWindow(NULL, ms);
  ms->config.densityViewerWindow->setWindowTitle("Density Viewer " + ms->config.left_or_right_arm);
  einMainWindow->addWindow(ms->config.densityViewerWindow);

  ms->config.objectViewerWindow = new EinWindow(NULL, ms);
  ms->config.objectViewerWindow->setWindowTitle("Object Viewer " + ms->config.left_or_right_arm);
  einMainWindow->addWindow(ms->config.objectViewerWindow);

  ms->config.objectMapViewerWindow = new EinWindow(NULL, ms);
  ms->config.objectMapViewerWindow->setWindowTitle("Object Map Viewer " + ms->config.left_or_right_arm);
  einMainWindow->addWindow(ms->config.objectMapViewerWindow);


  ms->config.stereoViewerWindow = new EinWindow(NULL, ms);
  ms->config.stereoViewerWindow->setWindowTitle("Stereo Viewer " + ms->config.left_or_right_arm);
  einMainWindow->addWindow(ms->config.stereoViewerWindow);


  ms->config.backgroundWindow = new EinWindow(NULL, ms);
  ms->config.backgroundWindow->setWindowTitle("Background Mean View " + ms->config.left_or_right_arm);
  ms->config.backgroundWindow->setVisible(true);
  einMainWindow->addWindow(ms->config.backgroundWindow);

  ms->config.backgroundMapWindow = new GaussianMapWindow(NULL, ms);
  ms->config.backgroundMapWindow->setWindowTitle("Background View " + ms->config.left_or_right_arm);
  ms->config.backgroundMapWindow->setVisible(true);
  einMainWindow->addWindow(ms->config.backgroundMapWindow);

  ms->config.observedWindow = new EinWindow(NULL, ms);
  ms->config.observedWindow->setWindowTitle("Observed Mean View " + ms->config.left_or_right_arm);
  ms->config.observedWindow->setMouseCallBack(mapCallbackFunc, ms);
  ms->config.observedWindow->setVisible(true);
  einMainWindow->addWindow(ms->config.observedWindow);

  ms->config.observedStdDevWindow = new EinWindow(NULL, ms);
  ms->config.observedStdDevWindow->setWindowTitle("Observed Std Dev View " + ms->config.left_or_right_arm);
  ms->config.observedStdDevWindow->setVisible(true);
  einMainWindow->addWindow(ms->config.observedStdDevWindow);


  ms->config.observedMapWindow = new GaussianMapWindow(NULL, ms);
  ms->config.observedMapWindow->setWindowTitle("Observed View " + ms->config.left_or_right_arm);
  ms->config.observedMapWindow->setVisible(true);
  einMainWindow->addWindow(ms->config.observedMapWindow);





  ms->config.predictedWindow = new EinWindow(NULL, ms);
  ms->config.predictedWindow->setWindowTitle("Predicted Mean View " + ms->config.left_or_right_arm);
  ms->config.predictedWindow->setVisible(true);
  einMainWindow->addWindow(ms->config.predictedWindow);


  ms->config.predictedStdDevWindow = new EinWindow(NULL, ms);
  ms->config.predictedStdDevWindow->setWindowTitle("Predicted Std Dev View " + ms->config.left_or_right_arm);
  ms->config.predictedStdDevWindow->setVisible(false);
  einMainWindow->addWindow(ms->config.predictedStdDevWindow);


  ms->config.predictedMapWindow = new GaussianMapWindow(NULL, ms);
  ms->config.predictedMapWindow->setWindowTitle("Predicted View " + ms->config.left_or_right_arm);
  ms->config.predictedMapWindow->setVisible(true);
  einMainWindow->addWindow(ms->config.predictedMapWindow);


  ms->config.streamViewerWindow = new StreamViewerWindow(NULL, ms);
  ms->config.streamViewerWindow->setWindowTitle("Stream Viewer " + ms->config.left_or_right_arm);
  ms->config.streamViewerWindow->setVisible(true);
  einMainWindow->addWindow(ms->config.streamViewerWindow);

  ms->config.discrepancyViewerWindow = new DiscrepancyWindow(NULL, ms);
  ms->config.discrepancyViewerWindow->setWindowTitle("Discrepancy " + ms->config.left_or_right_arm);
  ms->config.discrepancyViewerWindow->setVisible(true);
  einMainWindow->addWindow(ms->config.discrepancyViewerWindow);

  ms->config.discrepancyWindow = new EinWindow(NULL, ms);
  ms->config.discrepancyWindow->setWindowTitle("Discrepancy RGB View " + ms->config.left_or_right_arm);
  ms->config.discrepancyWindow->setVisible(false);
  einMainWindow->addWindow(ms->config.discrepancyWindow);


  ms->config.discrepancyDensityWindow = new EinWindow(NULL, ms);
  ms->config.discrepancyDensityWindow->setWindowTitle("Discrepancy Density View " + ms->config.left_or_right_arm);
  ms->config.discrepancyDensityWindow->setVisible(false);
  einMainWindow->addWindow(ms->config.discrepancyDensityWindow);


  ms->config.zWindow = new EinWindow(NULL, ms);
  ms->config.zWindow->setWindowTitle("Gaussian Map Z View " + ms->config.left_or_right_arm);
  ms->config.zWindow->setVisible(false);
  einMainWindow->addWindow(ms->config.zWindow);




  //createTrackbar("post_density_sigma", ms->config.densityViewerName, &ms->config.postDensitySigmaTrackbarVariable, 40);
  //createTrackbar("canny_lo", ms->config.densityViewerName, &ms->config.loTrackbarVariable, 100);
  //createTrackbar("canny_hi", ms->config.densityViewerName, &ms->config.hiTrackbarVariable, 100);


}

int opencvError (int status, const char *func_name, const char *err_msg, const char *file_name, int line, void *userdata) {
  cout << "OpenCV error: " << func_name << " with message " << err_msg << endl;
  cout << "File: " << file_name << " line: " << line << endl;
  assert(0);
}


void signalHandler( int signo )
{
  cout << "SIGNAL!  Shutting down: " << signo << endl;
  ros::shutdown();
}


int main(int argc, char **argv) {

  initializeWords();

  srand(time(NULL));

  if (argc < 4) {
    cout << "Must pass at least four arguments.  Received " << argc;
    ROS_ERROR("ein <physical|simulated|snoop> <left|right|both> <gui|nogui>");
    return -1;
  }

  string robot_mode = argv[1];
  if (robot_mode != "simulated" && robot_mode != "physical" && robot_mode != "snoop")  {
    cout << "Invalid mode: " << robot_mode << endl;
    ROS_ERROR("Must pass ein <physical|simulated|snoop> <left|right|both> <gui|nogui>");
    return -1;
  }

  string left_or_right_arm = argv[2];

  vector<string> arm_names;

  if (left_or_right_arm == "both") {
    arm_names.push_back("left");
    arm_names.push_back("right");
  } else if (left_or_right_arm == "left") {
    arm_names.push_back("left");
  } else if (left_or_right_arm == "right") {
    arm_names.push_back("right");
  } else {
    ROS_ERROR("Must pass left, right, or both.");
  }
  bool showgui;
  string gui_or_nogui = argv[3];
  if (gui_or_nogui == "gui") {
    showgui = true;
  } else if (gui_or_nogui == "nogui") {
    showgui = false;
  } else {
    ROS_ERROR("Must pass gui or nogui");
  }

  QCoreApplication * a;

  if (showgui) {
    a = new QApplication(argc, argv);
  } else {
    a = new QCoreApplication(argc, argv);
  }



  string programName;
  if (argc > 1) {
    programName = string(PROGRAM_NAME) + "_" + left_or_right_arm;
    cout << "programName: " << programName << endl;
  }
  else {
    programName = string(PROGRAM_NAME);
  }

  if (robot_mode == "snoop" || robot_mode == "simulated") {
    ros::init(argc, argv, programName, ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  } else {
    ros::init(argc, argv, programName, ros::init_options::NoSigintHandler);
  }
  ros::NodeHandle n("~");


  std::ifstream ifs("src/ein/VERSION");
  std::string ein_software_version( (std::istreambuf_iterator<char>(ifs) ),
                                    (std::istreambuf_iterator<char>()    ) );
  boost::trim(ein_software_version);


  for(int i = 0; i < arm_names.size(); i++) {
    string left_or_right = arm_names[i];
    MachineState * ms = new MachineState();
    ms->config.ein_software_version = ein_software_version;
    ms->config.robot_mode = robot_mode;
    if (ms->config.robot_mode == "simulated") {
      ms->config.currentRobotMode = SIMULATED;
    } else if (ms->config.robot_mode == "physical") {
      ms->config.currentRobotMode = PHYSICAL;
    } else if (ms->config.robot_mode == "snoop") {
      ms->config.currentRobotMode = SNOOP;
    } else {
      cout << "bad mode: " << ms->config.robot_mode << endl;
      assert(0);
    }
    
    machineStates.push_back(ms);
    if (left_or_right == "left") {
      left_arm = ms;
    } else if (left_or_right == "right") {
      right_arm = ms;
    } else {
      assert(0);
    }

    initVectorArcTan(ms);

    initializeArm(ms, left_or_right);

    ms->config.timer1 = n.createTimer(ros::Duration(0.01), &MachineState::timercallback1, ms);
    ms->config.showgui = showgui;
  }

  if (showgui) {
    einMainWindow = new MainWindow(NULL, right_arm, left_arm);

    for(int i = 0; i < machineStates.size(); i++) {
      initializeArmGui(machineStates[i], einMainWindow);
    }

    einMainWindow->show();
    einMainWindow->setObjectMapViewMouseCallBack(objectMapCallbackFunc, &machineStates);
    einMainWindow->setWindowTitle(QString::fromStdString("Ein " + ein_software_version + " Main Window (" + robot_mode + " " + left_or_right_arm + ")"));
  }


  //timer->start(0);
  qRegisterMetaType<Mat>("Mat");

  int cudaCount = gpu::getCudaEnabledDeviceCount();
  cout << "cuda count: " << cudaCount << endl;;

  cv::redirectError(opencvError, NULL, NULL);

  //a.exec();
  signal(SIGINT, signalHandler);
  signal(SIGHUP, signalHandler);
  signal(SIGTERM, signalHandler);

  
  ros::spin();

  return 0;
}
 
