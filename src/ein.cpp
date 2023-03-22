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


#include <sensor_msgs/image_encodings.hpp>


//#define DEBUG_RING_BUFFER

#define stringer(token) #token
#define stringer_value(token) stringer(token)

#include <boost/filesystem.hpp>

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


int getRingRangeAtTime(MachineState * ms, rclcpp::Time t, double &value, int drawSlack) {
  if (ms->config.rgRingBufferStart == ms->config.rgRingBufferEnd) {
#ifdef DEBUG_RING_BUFFER
    cout << "Denied request in getRingRangeAtTime(): Buffer empty." << endl;
#endif
    return 0;
  } else {
    int earliestSlot = ms->config.rgRingBufferStart;
    rclcpp::Duration deltaTdur = t - ms->config.rgRBTimes[earliestSlot];
    // if the request comes before our earliest record, deny
    if (deltaTdur.seconds() <= 0.0) {
#ifdef DEBUG_RING_BUFFER
      cout << "Denied out of order range value in getRingRangeAtTime(): Too small." << endl;
#endif
      return -1;
    } else if (ms->config.rgRingBufferStart < ms->config.rgRingBufferEnd) {
      for (int s = ms->config.rgRingBufferStart; s < ms->config.rgRingBufferEnd; s++) {
	rclcpp::Duration deltaTdurPre = t - ms->config.rgRBTimes[s];
	rclcpp::Duration deltaTdurPost = t - ms->config.rgRBTimes[s+1];
	if ((deltaTdurPre.seconds() >= 0.0) && (deltaTdurPost.seconds() <= 0)) {
	  double r1 = ms->config.rgRingBuffer[s];
	  double r2 = ms->config.rgRingBuffer[s+1];
	  double w1 = deltaTdurPre.seconds();
	  double w2 = -deltaTdurPost.seconds();
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
	rclcpp::Duration deltaTdurPre = t - ms->config.rgRBTimes[s];
	rclcpp::Duration deltaTdurPost = t - ms->config.rgRBTimes[s+1];
	if ((deltaTdurPre.seconds() >= 0.0) && (deltaTdurPost.seconds() <= 0)) {
	  double r1 = ms->config.rgRingBuffer[s];
	  double r2 = ms->config.rgRingBuffer[s+1];
	  double w1 = deltaTdurPre.seconds();
	  double w2 = -deltaTdurPost.seconds();
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
	rclcpp::Duration deltaTdurPre = t - ms->config.rgRBTimes[ms->config.rgRingBufferSize-1];
	rclcpp::Duration deltaTdurPost = t - ms->config.rgRBTimes[0];
	if ((deltaTdurPre.seconds() >= 0.0) && (deltaTdurPost.seconds() <= 0)) {
	  double r1 = ms->config.rgRingBuffer[ms->config.rgRingBufferSize-1];
	  double r2 = ms->config.rgRingBuffer[0];
	  double w1 = deltaTdurPre.seconds();
	  double w2 = -deltaTdurPost.seconds();
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
	rclcpp::Duration deltaTdurPre = t - ms->config.rgRBTimes[s];
	rclcpp::Duration deltaTdurPost = t - ms->config.rgRBTimes[s+1];
	if ((deltaTdurPre.seconds() >= 0.0) && (deltaTdurPost.seconds() <= 0)) {
	  double r1 = ms->config.rgRingBuffer[s];
	  double r2 = ms->config.rgRingBuffer[s+1];
	  double w1 = deltaTdurPre.seconds();
	  double w2 = -deltaTdurPost.seconds();
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
  cerr << "END OF FUNCTION" << endl;
  return -2;
}


int getMostRecentRingImageAndPose(MachineState * ms, Mat * image, eePose * pose, rclcpp::Time * time, bool debug) {
  if (ms->config.epRingBufferEnd > ms->config.epRBTimes.size()) {
    cout << "Ring buffer not yet initialized. " << ms->config.epRingBufferEnd << " times: " << ms->config.epRBTimes.size() << endl;
    assert(0);
  }
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
  rclcpp::Time poseTime = ms->config.epRBTimes[ms->config.epRingBufferEnd - 1];
  rclcpp::Time imageTime = camera->imRBTimes[camera->imRingBufferEnd - 1];

  * time = min(poseTime, imageTime);
  geometry_msgs::msg::Pose thisPose;
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


int getRingPoseAtTime(MachineState * ms, rclcpp::Time t, geometry_msgs::msg::Pose &value, int drawSlack, bool debug) {
  if (ms->config.epRingBufferStart == ms->config.epRingBufferEnd) {
    if (debug) {
      cout << "Denied request in getRingPoseAtTime(): Buffer empty." << endl;
    }
    return -1;
  } else {
    int earliestSlot = ms->config.epRingBufferStart;
    rclcpp::Duration deltaTdur = t - ms->config.epRBTimes[earliestSlot];
    // if the request comes before our earliest record, deny
    if (deltaTdur.seconds() <= 0.0) {
      if (debug) {
	cout << "Denied out of order range value in getRingPoseAtTime(): Too small." << endl;
      }
      return -1;
    } else if (ms->config.epRingBufferStart < ms->config.epRingBufferEnd) {
      for (int s = ms->config.epRingBufferStart; s < ms->config.epRingBufferEnd; s++) {
	rclcpp::Duration deltaTdurPre = t - ms->config.epRBTimes[s];
	rclcpp::Duration deltaTdurPost = t - ms->config.epRBTimes[s+1];
	if ((deltaTdurPre.seconds() >= 0.0) && (deltaTdurPost.seconds() <= 0)) {
	  Quaternionf q1 = extractQuatFromPose(ms->config.epRingBuffer[s]);
	  Quaternionf q2 = extractQuatFromPose(ms->config.epRingBuffer[s+1]);
	  double w1 = deltaTdurPre.seconds();
	  double w2 = -deltaTdurPost.seconds();
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
      }
      return -2;
    } else {
      for (int s = ms->config.epRingBufferStart; s < ms->config.epRingBufferSize-1; s++) {
	rclcpp::Duration deltaTdurPre = t - ms->config.epRBTimes[s];
	rclcpp::Duration deltaTdurPost = t - ms->config.epRBTimes[s+1];
	if ((deltaTdurPre.seconds() >= 0.0) && (deltaTdurPost.seconds() <= 0)) {
	  Quaternionf q1 = extractQuatFromPose(ms->config.epRingBuffer[s]);
	  Quaternionf q2 = extractQuatFromPose(ms->config.epRingBuffer[s+1]);
	  double w1 = deltaTdurPre.seconds();
	  double w2 = -deltaTdurPost.seconds();
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

	  int newStart = s;
	  if(drawSlack) {
	    ms->config.epRingBufferStart = newStart;
	  }
	  return 1;
	}
      } {
	rclcpp::Duration deltaTdurPre = t - ms->config.epRBTimes[ms->config.epRingBufferSize-1];
	rclcpp::Duration deltaTdurPost = t - ms->config.epRBTimes[0];
	if ((deltaTdurPre.seconds() >= 0.0) && (deltaTdurPost.seconds() <= 0)) {
	  Quaternionf q1 = extractQuatFromPose(ms->config.epRingBuffer[ms->config.epRingBufferSize-1]);
	  Quaternionf q2 = extractQuatFromPose(ms->config.epRingBuffer[0]);
	  double w1 = deltaTdurPre.seconds();
	  double w2 = -deltaTdurPost.seconds();
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

	  int newStart = ms->config.epRingBufferSize-1;
	  if(drawSlack) {
	    ms->config.epRingBufferStart = newStart;
	  }
	  return 1;
	}
      } for (int s = 0; s < ms->config.epRingBufferEnd; s++) {
	rclcpp::Duration deltaTdurPre = t - ms->config.epRBTimes[s];
	rclcpp::Duration deltaTdurPost = t - ms->config.epRBTimes[s+1];
	if ((deltaTdurPre.seconds() >= 0.0) && (deltaTdurPost.seconds() <= 0)) {
	  Quaternionf q1 = extractQuatFromPose(ms->config.epRingBuffer[s]);
	  Quaternionf q2 = extractQuatFromPose(ms->config.epRingBuffer[s+1]);
	  double w1 = deltaTdurPre.seconds();
	  double w2 = -deltaTdurPost.seconds();
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
          
	  int newStart = s;
	  if(drawSlack) {
	    ms->config.epRingBufferStart = newStart;
	  }
	  return 1;
	}
      }

      return -2;
    }
  }
  cout << "Bottomed out" << endl;
  return -1;
}

void setRingRangeAtTime(MachineState * ms, rclcpp::Time t, double rgToSet) {
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
    rclcpp::Duration deltaTdur = t - ms->config.rgRBTimes[ms->config.rgRingBufferStart];
    if (deltaTdur.seconds() <= 0.0) {
#ifdef DEBUG_RING_BUFFER 
      //cout << "Dropped out of order range value in setRingRangeAtTime(). " << ms->config.rgRBTimes[ms->config.rgRingBufferStart].seconds() << " " << t.seconds() << " " << deltaTdur.seconds() << " " << endl;
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
void setRingPoseAtTime(MachineState * ms, rclcpp::Time t, geometry_msgs::msg::Pose epToSet) {
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
    rclcpp::Duration deltaTdur = t - ms->config.epRBTimes[ms->config.epRingBufferStart];
    if (deltaTdur.seconds() <= 0.0) {
#ifdef DEBUG_RING_BUFFER 
      cout << "Dropped out of order range value in setRingPoseAtTime(). " << ms->config.epRBTimes[ms->config.epRingBufferStart].seconds() << " " << t.seconds() << " " << deltaTdur.seconds() << " " << endl;
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
void allRingBuffersAdvance(MachineState * ms, rclcpp::Time t) {

  double thisRange;
  Mat thisIm;
  geometry_msgs::msg::Pose thisPose;

  getRingPoseAtTime(ms, t, thisPose, 1);
  for (int i = 0; i < ms->config.cameras.size(); i++) {
    ms->config.cameras[i]->getRingImageAtTime(t, thisIm, 1);
  }
  //getRingRangeAtTime(t, thisRange, 1);
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
    cout << "bottomed out of the if." << endl;
    return 0;
  
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

void MachineState::moveEndEffectorCommandCallback(const geometry_msgs::msg::Pose& msg) {

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


void MachineState::forthCommandCallback(const std_msgs::msg::String& msg) {
  MachineState * ms = this;
  cout << "Received " << ms->config.forthCommand << endl;
  ms->config.forthCommand = msg.data;
  evaluateProgram(msg.data);
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

Quaternionf extractQuatFromPose(geometry_msgs::msg::Pose poseIn) {
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


void MachineState::accelerometerCallback(const sensor_msgs::msg::Imu& moment) {
  MachineState * ms = this;
  ms->config.lastAccelerometerCallbackReceived = rclcpp::Clock{}.now();
  ms->config.eeLinearAcceleration[0] = moment.linear_acceleration.x;
  ms->config.eeLinearAcceleration[1] = moment.linear_acceleration.y;
  ms->config.eeLinearAcceleration[2] = moment.linear_acceleration.z;
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






void MachineState::timercallback1() {

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


  if (rclcpp::Clock{}.now() - ms->config.lastStatePubTime > rclcpp::Duration(0.1, 0)) {
    ein::msg::EinState state;
    fillEinStateMsg(ms, &state);
    //cout << "Publishing state." << endl;
    ms->config.einStatePub->publish(state);
    ms->config.lastStatePubTime = rclcpp::Clock{}.now();
  }

  endEffectorAngularUpdate(&ms->config.currentEEPose, &ms->config.currentEEDeltaRPY);

  if (!ms->config.zero_g_toggle) {
    robotUpdate(ms);
  } else {
    ms->config.currentEEPose = ms->config.trueEEPoseEEPose;
    robotSetCurrentJointPositions(ms);
  }

  if (ms->config.showgui) {
    if (ms->config.coreViewWindow->isVisible()) {
      renderCoreView(ms);
      ms->config.coreViewWindow->updateImage(ms->config.coreViewImage);
    }
    
  }
}

void MachineState::publishConsoleMessage(string msg) {
  MachineState * ms = this;
  ein::msg::EinConsole consoleMsg;
  consoleMsg.msg = msg;
  ms->config.einConsolePub->publish(consoleMsg);
  cout << "Console: " << msg << endl;
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
  rclcpp::Time savedTime = rclcpp::Clock{}.now();

  FileStorage fsvO;

  fsvO.open(outFileName, FileStorage::WRITE);

  if (! fsvO.isOpened()) {
    CONSOLE_ERROR(ms, "Couldn't open config file " << outFileName);
    return;
  }

  fsvO << "savedTime" << "[" 
    << savedTime.seconds() 
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

void resetAccumulatedImageAndMass(MachineState * ms) {

  Size sz = ms->config.accumulatedImageMass.size();
  int imW = sz.width;
  int imH = sz.height;
  cout << "Reading denom:" << endl;
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      double denom = ms->config.accumulatedImageMass.at<double>(y,x);
      cout << "Denom is denom: " << denom << endl;
    }
  }

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      ms->config.accumulatedImageMass.at<double>(y,x) = 0;
      ms->config.accumulatedImage.at<Vec3d>(y,x)[0] = 0;
      ms->config.accumulatedImage.at<Vec3d>(y,x)[1] = 0;
      ms->config.accumulatedImage.at<Vec3d>(y,x)[2] = 0;
    }
  }

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      double denom = ms->config.accumulatedImageMass.at<double>(y,x);
      cout << "Set denom: " << denom << endl;
    }
  }

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
    teePose.px = ms->config.trueEEPoseEEPose.px;
    teePose.py = ms->config.trueEEPoseEEPose.py;
    teePose.pz = ms->config.trueEEPoseEEPose.pz;
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

	eePose thisPose = ms->config.trueEEPoseEEPose;
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
				     (thisPose.px - irSensorStartLocal.x()) + ms->config.eeRange*localUnitZ.x(),
				     (thisPose.py - irSensorStartLocal.y()) + ms->config.eeRange*localUnitZ.y(),
				     (thisPose.pz - irSensorStartLocal.z()) + ms->config.eeRange*localUnitZ.z()
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
      CONSOLE_ERROR(ms, "Couldn't initialize rendering system.");
      return;
    }
  }

  ms->config.wristCamImage = camera->cam_img.clone();
  ms->config.wristCamInit = 1;

  ms->config.wristViewImage = camera->cam_bgr_img.clone();

  ms->config.faceViewImage = camera->cam_bgr_img.clone();

  //accumulateImage(ms);

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




void changeCamera(MachineState * ms, int newCamera) {
  ms->config.focused_camera = newCamera;
  ms->config.renderInit = 0;
  Camera * camera  = ms->config.cameras[ms->config.focused_camera];
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
  result.pz = ms->config.trueEEPoseEEPose.pz - ms->config.currentTableZ;
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
    CONSOLE_ERROR(ms, "Invalid camera calibration mode: " << camera->currentCameraCalibrationMode);
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
  Quaternionf eeqform(ms->config.trueEEPoseEEPose.qw, ms->config.trueEEPoseEEPose.qx, ms->config.trueEEPoseEEPose.qy, ms->config.trueEEPoseEEPose.qz);
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
  double zToUse = ms->config.trueEEPoseEEPose.pz+ms->config.currentTableZ;

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

void MachineState::einStateCallback(const ein::msg::EinState & msg) {
  cout << "Received state msg." << endl;
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
    CONSOLE_ERROR(ms, "Whoops, accumulatedImage is sketchy, returning vanishing point to findOptimum.");
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

void loadROSParamsFromArgs(MachineState * ms) {



  //nh.getParam("/robot_description", ms->config.robot_description);
  robotInitializeSerial(ms);

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


void initializeViewers(MachineState * ms) {

  Camera * camera  = ms->config.cameras[ms->config.focused_camera];

  ms->config.objectViewerYCbCrBlur = Mat(camera->cam_bgr_img.rows, camera->cam_bgr_img.cols, CV_64FC3);


  ms->config.objectViewerGrayBlur = Mat(camera->cam_bgr_img.rows, camera->cam_bgr_img.cols, CV_64FC3);


  ms->config.accumulatedImage = Mat(camera->cam_bgr_img.rows, camera->cam_bgr_img.cols, CV_64FC3);


  ms->config.accumulatedImageMass = Mat(camera->cam_bgr_img.rows, camera->cam_bgr_img.cols, CV_64F);


  ms->config.objectViewerImage = camera->cam_bgr_img.clone();

}



void fillEinStateMsg(MachineState * ms, ein::msg::EinState * stateOut) {
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


void initializeArm(rclcpp::Node::SharedPtr node, MachineState * ms, string left_or_right_arm) {

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
  ms->config.data_directory = "default";

  string console_topic = "/ein/" + ms->config.left_or_right_arm + "/console";
  ms->config.einConsolePub = node->create_publisher<ein::msg::EinConsole>(console_topic, 10);

  loadROSParamsFromArgs(ms);
  //cout << "mask_gripper: " << ms->config.mask_gripper << endl;
  //cout << "all_range_mode: " << ms->config.all_range_mode << endl;
  cout << "data_directory: " << ms->config.data_directory << endl;
  //<< "run_prefix: " << ms->config.run_prefix << endl << endl 
  //<< "vocab_file: " << ms->config.vocab_file << endl 
  //<< "knn_file: " << ms->config.knn_file << endl << "label_file: " << ms->config.label_file << endl
  //<< endl;

  unsigned long seed = 1;
  rk_seed(seed, &ms->config.random_state);

  //Camera * k2rgb = new Camera(ms, "left_kinect2_color_qhd", "/kinect2/qhd/image_color", ms->config.left_or_right_arm + "_hand", "k2rgb_tf_link");
  //Camera * k2rgb = new Camera(ms, "left_kinect2_color_hd", "/kinect2/hd/image_color", ms->config.left_or_right_arm + "_hand", "kinect2_link");
  //ms->config.cameras.push_back(k2rgb);

  //Camera * k2ir = new Camera(ms, "left_kinect2_ir", "/kinect2/sd/image_ir", ms->config.left_or_right_arm + "_hand", "k2ir_tf_link");
  //ms->config.cameras.push_back(k2ir);

  //Camera * k2depth = new Camera(ms, "left_kinect2_depth", "/kinect2/sd/image_depth", ms->config.left_or_right_arm + "_hand", "k2ir_tf_link");
  //ms->config.cameras.push_back(k2depth);


  ms->config.ee_target_pub = node->create_publisher<geometry_msgs::msg::Point>("pilot_target_" + ms->config.left_or_right_arm, 10);

  ms->config.it =  new image_transport::ImageTransport(node);


  robotInitializeConfig(ms);

  ms->config.moveEndEffectorCommandCallbackSub = node->create_subscription<geometry_msgs::msg::Pose>("/ein/eeMoveCommand", 1, std::bind(&MachineState::moveEndEffectorCommandCallback, ms, std::placeholders::_1));


  if (ms->config.currentRobotMode == PHYSICAL || ms->config.currentRobotMode == SIMULATED) {
    ms->config.forthCommandSubscriber = node->create_subscription<std_msgs::msg::String>("/ein/" + ms->config.left_or_right_arm + "/forth_commands", 1, 
								  std::bind(&MachineState::forthCommandCallback, ms, std::placeholders::_1));
    ms->config.forthCommandPublisher = node->create_publisher<std_msgs::msg::String>("/ein/" + ms->config.other_arm + "/forth_commands", 10);
  } else if (ms->config.currentRobotMode == SNOOP) {
    ms->config.forthCommandPublisher = node->create_publisher<std_msgs::msg::String>("/ein/" + ms->config.left_or_right_arm + "/forth_commands", 10);
    ms->config.einSub = node->create_subscription<ein::msg::EinState>("/ein_" + ms->config.left_or_right_arm + "/state", 1, std::bind(&MachineState::einStateCallback, ms, std::placeholders::_1));
  } else {
    assert(0);
  }



  ms->config.tf_buffer =
      std::make_unique<tf2_ros::Buffer>(node->get_clock());
  ms->config.tfListener = std::make_shared<tf2_ros::TransformListener>(*ms->config.tf_buffer);

  string state_topic = "/ein/" + ms->config.left_or_right_arm + "/state";
  ms->config.einStatePub = node->create_publisher<ein::msg::EinState>(state_topic, 10);


  ms->config.frameGraySobel = Mat(1,1,CV_64F);


  nodeInit(ms);
  irInit(ms);



  ms->config.lastMovementStateSet = rclcpp::Clock{}.now();

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


  ms->config.renderedWristViewWindow = new EinWindow(NULL, ms);
  ms->config.renderedWristViewWindow->setWindowTitle("Rendered Wrist View " + ms->config.left_or_right_arm);
  einMainWindow->addWindow(ms->config.renderedWristViewWindow);


  ms->config.coreViewWindow = new EinWindow(NULL, ms);
  ms->config.coreViewWindow->setWindowTitle("Core View " + ms->config.left_or_right_arm);
  einMainWindow->addWindow(ms->config.coreViewWindow);
  
  
  ms->config.faceViewWindow = new EinWindow(NULL, ms);
  ms->config.faceViewWindow->setWindowTitle("Face View " + ms->config.left_or_right_arm);
  einMainWindow->addWindow(ms->config.faceViewWindow);
  

  

  ms->config.mapBackgroundViewWindow = new EinWindow(NULL, ms);
  ms->config.mapBackgroundViewWindow->setWindowTitle("Hi Color Range Map View " + ms->config.left_or_right_arm);
  einMainWindow->addWindow(ms->config.mapBackgroundViewWindow);


  ms->config.meanViewerWindow = new EinWindow(NULL, ms);
  ms->config.meanViewerWindow->setWindowTitle("Mean Viewer " + ms->config.left_or_right_arm);
  einMainWindow->addWindow(ms->config.meanViewerWindow);

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
  rclcpp::shutdown();
}

int main(int argc, char **argv) {

  initializeWords();

  srand(time(NULL));

  if (argc < 4) {
    cout << "Must pass at least four arguments.  Received " << argc;
    return -1;
  }

  string robot_mode = argv[1];
  if (robot_mode != "simulated" && robot_mode != "physical" && robot_mode != "snoop")  {
    cout << "Invalid mode: " << robot_mode << endl;
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
    cerr<< "Must pass left, right, or both." << endl;
  }
  bool showgui;
  string gui_or_nogui = argv[3];
  if (gui_or_nogui == "gui") {
    showgui = true;
  } else if (gui_or_nogui == "nogui") {
    showgui = false;
  } else {
    cerr << "Must pass gui or nogui" << endl;
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

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ein", options);



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

    initializeArm(node, ms, left_or_right);

    ms->config.timer1 = node->create_wall_timer(std::chrono::milliseconds(1),
						std::bind(&MachineState::timercallback1, ms));
    ms->config.showgui = showgui;
  }

  if (showgui) {
    einMainWindow = new MainWindow(NULL, right_arm, left_arm);

    for(int i = 0; i < machineStates.size(); i++) {
      initializeArmGui(machineStates[i], einMainWindow);
    }

    einMainWindow->show();

    einMainWindow->setWindowTitle(QString::fromStdString("Ein " + ein_software_version + " Main Window (" + robot_mode + " " + left_or_right_arm + ")"));
  }


  //timer->start(0);
  qRegisterMetaType<Mat>("Mat");

  cv::redirectError(opencvError, NULL, NULL);

  //a.exec();
  signal(SIGINT, signalHandler);
  signal(SIGHUP, signalHandler);
  signal(SIGTERM, signalHandler);

  cout << "Spinning ." << endl;
  rclcpp::spin(node);

  return 0;
}


 
