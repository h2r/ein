
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
//        ><>  
//
// end Header

#include "ein.h"

MachineState machineState;
shared_ptr<MachineState> pMachineState;


////////////////////////////////////////////////
// start pilot includes, usings, and defines
////////////////////////////////////////////////

//
// start pilot definitions 
////////////////////////////////////////////////

void happy(shared_ptr<MachineState> ms) {
  std_msgs::Int32 msg;
  msg.data = 0;
  ms->config.facePub.publish(msg);
}

void sad(shared_ptr<MachineState> ms) {
  std_msgs::Int32 msg;
  msg.data = 99;
  ms->config.facePub.publish(msg);
}

void neutral(shared_ptr<MachineState> ms) {
  std_msgs::Int32 msg;
  msg.data = 50;
  ms->config.facePub.publish(msg);
}


int getRingImageAtTime(shared_ptr<MachineState> ms, ros::Time t, Mat& value, int drawSlack) {
  if (ms->config.imRingBufferStart == ms->config.imRingBufferEnd) {
    
#ifdef DEBUG_RING_BUFFER
    cout << "Denied request in getRingImageAtTime(): Buffer empty." << endl;
#endif
    return 0;
  } else {
    int earliestSlot = ms->config.imRingBufferStart;
    ros::Duration deltaTdur = t - ms->config.imRBTimes[earliestSlot];
    // if the request comes before our earliest record, deny
    if (deltaTdur.toSec() <= 0.0) {
#ifdef DEBUG_RING_BUFFER
      cout << "Denied out of order range value in getRingImageAtTime(): Too small." << endl;
      cout << "  getRingImageAtTime() ms->config.imRingBufferStart ms->config.imRingBufferEnd t ms->config.imRBTimes[earliestSlot]: " << 
	ms->config.imRingBufferStart << " " << ms->config.imRingBufferEnd << " " << t << " " << ms->config.imRBTimes[earliestSlot] << endl;
#endif
      return -1;
    } else if (ms->config.imRingBufferStart < ms->config.imRingBufferEnd) {
      for (int s = ms->config.imRingBufferStart; s < ms->config.imRingBufferEnd; s++) {
	ros::Duration deltaTdurPre = t - ms->config.imRBTimes[s];
	ros::Duration deltaTdurPost = t - ms->config.imRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Mat m1 = ms->config.imRingBuffer[s];
	  Mat m2 = ms->config.imRingBuffer[s+1];
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;
	  if (w1 >= w2)
	    //value = m1;
	    value = m1*w1 + m2*w2;
	  else
	    //value = m2;
	    value = m1*w1 + m2*w2;

	  int newStart = s;
	  if(drawSlack) {
	    ms->config.imRingBufferStart = newStart;
	  }
	  return 1;
	}
      }
      // if we didn't find it we should return failure
#ifdef DEBUG_RING_BUFFER
      cout << "Denied out of order range value in getRingImageAtTime(): Too large." << endl;
#endif
      return -2;
    } else {
      for (int s = ms->config.imRingBufferStart; s < ms->config.imRingBufferSize-1; s++) {
	ros::Duration deltaTdurPre = t - ms->config.imRBTimes[s];
	ros::Duration deltaTdurPost = t - ms->config.imRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Mat m1 = ms->config.imRingBuffer[s];
	  Mat m2 = ms->config.imRingBuffer[s+1];
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;
	  if (w1 >= w2)
	    value = m1;
	  else
	    value = m2;

	  int newStart = s;
	  if(drawSlack) {
	    ms->config.imRingBufferStart = newStart;
	  }
	  return 1;
	}
      } {
	ros::Duration deltaTdurPre = t - ms->config.imRBTimes[ms->config.imRingBufferSize-1];
	ros::Duration deltaTdurPost = t - ms->config.imRBTimes[0];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Mat m1 = ms->config.imRingBuffer[ms->config.imRingBufferSize-1];
	  Mat m2 = ms->config.imRingBuffer[0];
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;
	  if (w1 >= w2)
	    value = m1;
	  else
	    value = m2;

	  int newStart = ms->config.imRingBufferSize-1;
	  if(drawSlack) {
	    ms->config.imRingBufferStart = newStart;
	  }
	  return 1;
	}
      } for (int s = 0; s < ms->config.imRingBufferEnd; s++) {
	ros::Duration deltaTdurPre = t - ms->config.imRBTimes[s];
	ros::Duration deltaTdurPost = t - ms->config.imRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Mat m1 = ms->config.imRingBuffer[s];
	  Mat m2 = ms->config.imRingBuffer[s+1];
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;
	  if (w1 >= w2)
	    value = m1;
	  else
	    value = m2;

	  int newStart = s;
	  if(drawSlack) {
	    ms->config.imRingBufferStart = newStart;
	  }
	  return 1;
	}
      }
      // if we didn't find it we should return failure
#ifdef DEBUG_RING_BUFFER
      cout << "Denied out of order range value in getRingImageAtTime(): Too large." << endl;
#endif
      return -2;
    }
  }
}
int getRingRangeAtTime(shared_ptr<MachineState> ms, ros::Time t, double &value, int drawSlack) {
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
int getRingPoseAtTime(shared_ptr<MachineState> ms, ros::Time t, geometry_msgs::Pose &value, int drawSlack) {
  if (ms->config.epRingBufferStart == ms->config.epRingBufferEnd) {
#ifdef DEBUG_RING_BUFFER
    cout << "Denied request in getRingPoseAtTime(): Buffer empty." << endl;
#endif
    return 0;
  } else {
    int earliestSlot = ms->config.epRingBufferStart;
    ros::Duration deltaTdur = t - ms->config.epRBTimes[earliestSlot];
    // if the request comes before our earliest record, deny
    if (deltaTdur.toSec() <= 0.0) {
#ifdef DEBUG_RING_BUFFER
      cout << "Denied out of order range value in getRingPoseAtTime(): Too small." << endl;
#endif
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

	  Quaternionf tTerp = q1.slerp(w2, q2);
	  value.orientation.w = tTerp.w();
	  value.orientation.x = tTerp.x();
	  value.orientation.y = tTerp.y();
	  value.orientation.z = tTerp.z();
	  value.position.x = ms->config.epRingBuffer[s].position.x*w1 + ms->config.epRingBuffer[s+1].position.x*w2;
	  value.position.y = ms->config.epRingBuffer[s].position.y*w1 + ms->config.epRingBuffer[s+1].position.y*w2;
	  value.position.z = ms->config.epRingBuffer[s].position.z*w1 + ms->config.epRingBuffer[s+1].position.z*w2;
#ifdef DEBUG_RING_BUFFER
          cout << value << endl;
          cout << "33333c " << ms->config.epRingBuffer[s] << " " << w1 << " " << w2 << " " << totalWeight << endl;
          cout << "44444c " << ms->config.epRingBuffer[s+1] << endl;
#endif

	  int newStart = s;
	  if(drawSlack) {
	    ms->config.epRingBufferStart = newStart;
	  }
	  return 1;
	}
      }
      // if we didn't find it we should return failure
#ifdef DEBUG_RING_BUFFER
      cout << "Denied out of order range value in getRingPoseAtTime(): Too large." << endl;
#endif
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
#ifdef DEBUG_RING_BUFFER
          cout << value << endl;
          cout << "33333b " << ms->config.epRingBuffer[s] << " " << w1 << " " << w2 << " " << totalWeight << endl;
          cout << "44444b " << ms->config.epRingBuffer[s+1] << endl;
#endif

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
#ifdef DEBUG_RING_BUFFER
          cout << value << endl;
          cout << "33333a " << ms->config.epRingBuffer[ms->config.epRingBufferSize-1] << " " << w1 << " " << w2 << " " << totalWeight << endl;
          cout << "44444a " << ms->config.epRingBuffer[0] << endl;
#endif

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
#ifdef DEBUG_RING_BUFFER
          cout << value << endl;
          cout << "33333d " << ms->config.epRingBuffer[s] << " " << w1 << " " << w2 << " " << totalWeight << endl;
          cout << "44444d " << ms->config.epRingBuffer[s+1] << endl;
#endif
          
	  int newStart = s;
	  if(drawSlack) {
	    ms->config.epRingBufferStart = newStart;
	  }
	  return 1;
	}
      }
      // if we didn't find it we should return failure
#ifdef DEBUG_RING_BUFFER
      cout << "Denied out of order range value in getRingPoseAtTime(): Too large." << endl;
#endif
      return -2;
    }
  }
}

void setRingImageAtTime(shared_ptr<MachineState> ms, ros::Time t, Mat& imToSet) {
#ifdef DEBUG_RING_BUFFER
  cout << "setRingImageAtTime() start end size: " << ms->config.imRingBufferStart << " " << ms->config.imRingBufferEnd << " " << ms->config.imRingBufferSize << endl;
#endif

  // if the ring buffer is empty, always re-initialize
  if (ms->config.imRingBufferStart == ms->config.imRingBufferEnd) {
    ms->config.imRingBufferStart = 0;
    ms->config.imRingBufferEnd = 1;
    ms->config.imRingBuffer[0] = imToSet;
    ms->config.imRBTimes[0] = t;
  } else {
    ros::Duration deltaTdur = t - ms->config.imRBTimes[ms->config.imRingBufferStart];
    if (deltaTdur.toSec() <= 0.0) {
#ifdef DEBUG_RING_BUFFER 
      cout << "Dropped out of order range value in setRingImageAtTime(). " << ms->config.imRBTimes[ms->config.ms->config.imRingBufferStart].toSec() << " " << t.toSec() << " " << deltaTdur.toSec() << " " << endl;
#endif
    } else {
      int slot = ms->config.imRingBufferEnd;
      ms->config.imRingBuffer[slot] = imToSet;
      ms->config.imRBTimes[slot] = t;

      if (ms->config.imRingBufferEnd >= (ms->config.imRingBufferSize-1)) {
	ms->config.imRingBufferEnd = 0;
      } else {
	ms->config.imRingBufferEnd++;
      }

      if (ms->config.imRingBufferEnd == ms->config.imRingBufferStart) {
	if (ms->config.imRingBufferStart >= (ms->config.imRingBufferSize-1)) {
	  ms->config.imRingBufferStart = 0;
	} else {
	  ms->config.imRingBufferStart++;
	}
      }
    }
  }
}
void setRingRangeAtTime(shared_ptr<MachineState> ms, ros::Time t, double rgToSet) {
#ifdef DEBUG_RING_BUFFER
  cout << "setRingRangeAtTime() start end size: " << ms->config.rgRingBufferStart << " " << ms->config.rgRingBufferEnd << " " << ms->config.rgRingBufferSize << endl;
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
      cout << "Dropped out of order range value in setRingRangeAtTime(). " << ms->config.rgRBTimes[ms->config.rgRingBufferStart].toSec() << " " << t.toSec() << " " << deltaTdur.toSec() << " " << endl;
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
void setRingPoseAtTime(shared_ptr<MachineState> ms, ros::Time t, geometry_msgs::Pose epToSet) {
#ifdef DEBUG_RING_BUFFER
  cout << "setRingPoseAtTime() start end size: " << ms->config.epRingBufferStart << " " << ms->config.epRingBufferEnd << " " << ms->config.epRingBufferSize << endl;
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

void imRingBufferAdvance(shared_ptr<MachineState> ms) {
  if (ms->config.imRingBufferEnd != ms->config.imRingBufferStart) {
    if (ms->config.imRingBufferStart >= (ms->config.imRingBufferSize-1)) {
      ms->config.imRingBufferStart = 0;
    } else {
      ms->config.imRingBufferStart++;
    }
  }
}
void rgRingBufferAdvance(shared_ptr<MachineState> ms) {
  if (ms->config.rgRingBufferEnd != ms->config.rgRingBufferStart) {
    if (ms->config.rgRingBufferStart >= (ms->config.rgRingBufferSize-1)) {
      ms->config.rgRingBufferStart = 0;
    } else {
      ms->config.rgRingBufferStart++;
    }
  }
}
void epRingBufferAdvance(shared_ptr<MachineState> ms) {
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
void allRingBuffersAdvance(shared_ptr<MachineState> ms, ros::Time t) {

  double thisRange;
  Mat thisIm;
  geometry_msgs::Pose thisPose;

  getRingPoseAtTime(ms, t, thisPose, 1);
  getRingImageAtTime(ms, t, thisIm, 1);
  //getRingRangeAtTime(t, thisRange, 1);
}

void recordReadyRangeReadings(shared_ptr<MachineState> ms) {
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
    
      geometry_msgs::Pose thisPose;
      Mat thisImage;
      int weHavePoseData = getRingPoseAtTime(ms, thisTime, thisPose);
      int weHaveImData = getRingImageAtTime(ms, thisTime, thisImage);

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
	  ms->config.irGlobalPositionEEFrame = crane2quat.conjugate() * ms->config.gear0offset * crane2quat;
	  Eigen::Quaternionf ceeQuat(thisPose.orientation.w, thisPose.orientation.x, thisPose.orientation.y, thisPose.orientation.z);
	  Eigen::Quaternionf irSensorStartLocal = ceeQuat * ms->config.irGlobalPositionEEFrame * ceeQuat.conjugate();
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

	  dX = (irSensorEnd.x() - ms->config.rmcX); //(thisPose.position.x - ms->config.drX) - rmcX;
	  dY = (irSensorEnd.y() - ms->config.rmcY); //(thisPose.position.y - ms->config.drY) - rmcY;
	  dZ = (irSensorEnd.z() - ms->config.rmcZ); //(thisPose.position.y - ms->config.drY) - rmcY;

	  double eX = (irSensorEnd.x() - ms->config.rmcX) / ms->config.hrmDelta;
	  double eY = (irSensorEnd.y() - ms->config.rmcY) / ms->config.hrmDelta;
	  int eeX = (int)round(eX + ms->config.hrmHalfWidth);
	  int eeY = (int)round(eY + ms->config.hrmHalfWidth);

#ifdef DEBUG
	  cout << "irSensorEnd w x y z: " << irSensorEnd.w() << " " << 
	    irSensorEnd.x() << " " << irSensorEnd.y() << " " << irSensorEnd.z() << endl;
	  cout << "irSensorStartGlobal w x y z: " << irSensorStartGlobal.w() << " " << 
	    irSensorStartGlobal.x() << " " << irSensorStartGlobal.y() << " " << irSensorStartGlobal.z() << endl;
	  cout << "Corrected x y: " << (thisPose.position.x - ms->config.drX) << " " << (thisPose.position.y - ms->config.drY) << endl;
	  cout.flush();
#endif

	  //cout << thisPose.orientation << thisPose.position << " " << eX << " " << eY << " " << thisRange << endl;
	  if ((fabs(eX) <= ms->config.hrmHalfWidth) && (fabs(eY) <= ms->config.hrmHalfWidth))
	    ms->config.hiRangemapImage.at<cv::Vec3b>(eeX,eeY) += cv::Vec3b(0,0,128);
	  // ATTN 0 this is negative because it used to be range and not Z but we have to chase the min / max switches to correct it
	  thisZmeasurement = -irSensorEnd.z();
	  // ATTN 25
	  ms->config.mostRecentUntabledZ = thisZmeasurement;
	  //mostRecentUntabledZ = ((1.0-ms->config.mostRecentUntabledZDecay)*thisZmeasurement) + (ms->config.mostRecentUntabledZDecay*mostRecentUntabledZ);
	  // ATTN 1 currently accounting for table models
	  thisZmeasurement = thisZmeasurement - ms->config.currentTableZ;

	  rayDirection = Eigen::Vector3d(localUnitZ.x(), localUnitZ.y(), localUnitZ.z());
	}

	double iX = dX / ms->config.rmDelta;
	double iY = dY / ms->config.rmDelta;

	double hiX = dX / ms->config.hrmDelta;
	double hiY = dY / ms->config.hrmDelta;
	double hiZ = dZ / ms->config.hrmDelta;

	if (ms->config.recordRangeMap) {
	  // draw new cell
	  if ((fabs(hiX) <= ms->config.hrmHalfWidth) && (fabs(hiY) <= ms->config.hrmHalfWidth)) {
	    int hiiX = (int)round(hiX + ms->config.hrmHalfWidth);
	    int hiiY = (int)round(hiY + ms->config.hrmHalfWidth);
	    int hiiZ = (int)round(hiZ + ms->config.hrmHalfWidth);

	    // the wrong point without pose correction
	    //double upX = ((ms->config.trueEEPose.position.x - ms->config.drX) - rmcX)/ms->config.hrmDelta;
	    //double upY = ((ms->config.trueEEPose.position.y - ms->config.drY) - ms->config.rmcY)/ms->config.hrmDelta;
	    //int iupX = (int)round(upX + ms->config.hrmHalfWidth);
	    //int iupY = (int)round(upY + ms->config.hrmHalfWidth);
	    //if ((fabs(upX) <= ms->config.hrmHalfWidth) && (fabs(upY) <= ms->config.hrmHalfWidth)) 
	      //ms->config.hiRangemapImage.at<cv::Vec3b>(iupX,iupY) += cv::Vec3b(0,128,0);

	    // 2D map
	    {
	      int pxMin = max(0, hiiX-ms->config.parzenKernelHalfWidth);
	      int pxMax = min(ms->config.hrmWidth-1, hiiX+ms->config.parzenKernelHalfWidth);
	      int pyMin = max(0, hiiY-ms->config.parzenKernelHalfWidth);
	      int pyMax = min(ms->config.hrmWidth-1, hiiY+ms->config.parzenKernelHalfWidth);
	      // correct loop order for cache coherency
	      for (int py = pyMin; py <= pyMax; py++) {
		for (int px = pxMin; px <= pxMax; px++) {
		  int kpx = px - (hiiX - ms->config.parzenKernelHalfWidth);
		  int kpy = py - (hiiY - ms->config.parzenKernelHalfWidth);

		  cv::Vec3b thisSample = getCRColor(ms, thisImage); 
		  ms->config.hiColorRangeMapAccumulator[px + py*ms->config.hrmWidth + 0*ms->config.hrmWidth*ms->config.hrmWidth] += thisSample[0]*ms->config.parzenKernel[kpx + kpy*ms->config.parzenKernelWidth];
		  ms->config.hiColorRangeMapAccumulator[px + py*ms->config.hrmWidth + 1*ms->config.hrmWidth*ms->config.hrmWidth] += thisSample[1]*ms->config.parzenKernel[kpx + kpy*ms->config.parzenKernelWidth];
		  ms->config.hiColorRangeMapAccumulator[px + py*ms->config.hrmWidth + 2*ms->config.hrmWidth*ms->config.hrmWidth] += thisSample[2]*ms->config.parzenKernel[kpx + kpy*ms->config.parzenKernelWidth];
		  ms->config.hiColorRangeMapMass[px + py*ms->config.hrmWidth] += ms->config.parzenKernel[kpx + kpy*ms->config.parzenKernelWidth];

		  double denomC = max(ms->config.hiColorRangeMapMass[px + py*ms->config.hrmWidth], EPSILON);
		  int tRed = min(255, max(0,int(round(ms->config.hiColorRangeMapAccumulator[px + py*ms->config.hrmWidth + 2*ms->config.hrmWidth*ms->config.hrmWidth] / denomC))));
		  int tGreen = min(255, max(0,int(round(ms->config.hiColorRangeMapAccumulator[px + py*ms->config.hrmWidth + 1*ms->config.hrmWidth*ms->config.hrmWidth] / denomC))));
		  int tBlue = min(255, max(0,int(round(ms->config.hiColorRangeMapAccumulator[px + py*ms->config.hrmWidth + 0*ms->config.hrmWidth*ms->config.hrmWidth] / denomC))));

		  ms->config.hiColorRangemapImage.at<cv::Vec3b>(px,py) = cv::Vec3b(tBlue, tGreen, tRed);

		  ms->config.hiRangeMapAccumulator[px + py*ms->config.hrmWidth] += thisZmeasurement*ms->config.parzenKernel[kpx + kpy*ms->config.parzenKernelWidth];
		  ms->config.hiRangeMapMass[px + py*ms->config.hrmWidth] += ms->config.parzenKernel[kpx + kpy*ms->config.parzenKernelWidth];
		  // nonexperimental
		  //double denom = max(ms->config.hiRangeMapMass[px + py*ms->config.hrmWidth], EPSILON);
		  // XXX experimental
		  double denom = 1.0;
		  if (ms->config.hiRangeMapMass[px + py*ms->config.hrmWidth] > 0)
		    denom = ms->config.hiRangeMapMass[px + py*ms->config.hrmWidth];
		  ms->config.hiRangeMap[px + py*ms->config.hrmWidth] = ms->config.hiRangeMapAccumulator[px + py*ms->config.hrmWidth] / denom;
		}
	      }
	    }
	    // record the point in the 3D maps
	    // positive surface observation
	    {
	      int pxMin = max(0, hiiX-ms->config.parzen3DKernelHalfWidth);
	      int pxMax = min(ms->config.vmWidth-1, hiiX+ms->config.parzen3DKernelHalfWidth);
	      int pyMin = max(0, hiiY-ms->config.parzen3DKernelHalfWidth);
	      int pyMax = min(ms->config.vmWidth-1, hiiY+ms->config.parzen3DKernelHalfWidth);
	      int pzMin = max(0, hiiZ-ms->config.parzen3DKernelHalfWidth);
	      int pzMax = min(ms->config.vmWidth-1, hiiZ+ms->config.parzen3DKernelHalfWidth);
	      // correct loop order for cache coherency
	      for (int pz = pzMin; pz <= pzMax; pz++) {
		for (int py = pyMin; py <= pyMax; py++) {
		  for (int px = pxMin; px <= pxMax; px++) {
		    int kpx = px - (hiiX - ms->config.parzen3DKernelHalfWidth);
		    int kpy = py - (hiiY - ms->config.parzen3DKernelHalfWidth);
		    int kpz = pz - (hiiZ - ms->config.parzen3DKernelHalfWidth);

		    cv::Vec3b thisSample = getCRColor(ms, thisImage); 
		    ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 0*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] += thisSample[0]*ms->config.parzen3DKernel[kpx + kpy*ms->config.parzen3DKernelWidth + kpz*ms->config.parzen3DKernelWidth*ms->config.parzen3DKernelWidth];
		    ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 1*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] += thisSample[1]*ms->config.parzen3DKernel[kpx + kpy*ms->config.parzen3DKernelWidth + kpz*ms->config.parzen3DKernelWidth*ms->config.parzen3DKernelWidth];
		    ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 2*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] += thisSample[2]*ms->config.parzen3DKernel[kpx + kpy*ms->config.parzen3DKernelWidth + kpz*ms->config.parzen3DKernelWidth*ms->config.parzen3DKernelWidth];
		    ms->config.vmColorRangeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] += ms->config.parzen3DKernel[kpx + kpy*ms->config.parzen3DKernelWidth + kpz*ms->config.parzen3DKernelWidth*ms->config.parzen3DKernelWidth];

		    //double denomC = max(ms->config.vmColorRangeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth], EPSILON);
		    //int tRed = min(255, max(0,int(round(ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 2*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] / denomC))));
		    //int tGreen = min(255, max(0,int(round(ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 1*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] / denomC))));
		    //int tBlue = min(255, max(0,int(round(ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 0*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] / denomC))));

		    // slightly different than 2D
		    ms->config.volumeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] += ms->config.parzen3DKernel[kpx + kpy*ms->config.parzen3DKernelWidth + kpz*ms->config.parzen3DKernelWidth*ms->config.parzen3DKernelWidth];
		    //ms->config.volumeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] += 1.0;
		    ms->config.volumeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] += ms->config.parzen3DKernel[kpx + kpy*ms->config.parzen3DKernelWidth + kpz*ms->config.parzen3DKernelWidth*ms->config.parzen3DKernelWidth];

		    double denom = max(ms->config.volumeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth], 1e-99); // XXX should be epsilon but there is clipping...
		    ms->config.volumeMap[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] = ms->config.volumeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] / denom;
		  }
		}
	      }
	    }
	    double negativeSpacing = 1.0*ms->config.parzen3DKernelSigma*ms->config.vmDelta;
	    //int numCastPoints = int(ceil(thisRange / negativeSpacing));
	    int numCastPoints = 10;
	    // negative surface observations
	    for (int castPoint = 1; castPoint <= numCastPoints; castPoint++) {
	      double piX = (dX - negativeSpacing*castPoint*rayDirection.x())/ ms->config.hrmDelta;
	      double piY = (dY - negativeSpacing*castPoint*rayDirection.y()) / ms->config.hrmDelta;
	      double piZ = (dZ - negativeSpacing*castPoint*rayDirection.z()) / ms->config.hrmDelta;

	      int piiX = (int)round(piX + ms->config.hrmHalfWidth);
	      int piiY = (int)round(piY + ms->config.hrmHalfWidth);
	      int piiZ = (int)round(piZ + ms->config.hrmHalfWidth);
	      

	      int pxMin = max(0, piiX-ms->config.parzen3DKernelHalfWidth);
	      int pxMax = min(ms->config.vmWidth-1, piiX+ms->config.parzen3DKernelHalfWidth);
	      int pyMin = max(0, piiY-ms->config.parzen3DKernelHalfWidth);
	      int pyMax = min(ms->config.vmWidth-1, piiY+ms->config.parzen3DKernelHalfWidth);
	      int pzMin = max(0, piiZ-ms->config.parzen3DKernelHalfWidth);
	      int pzMax = min(ms->config.vmWidth-1, piiZ+ms->config.parzen3DKernelHalfWidth);
	      // correct loop order for cache coherency
	      for (int pz = pzMin; pz <= pzMax; pz++) {
		for (int py = pyMin; py <= pyMax; py++) {
		  for (int px = pxMin; px <= pxMax; px++) {
		    int kpx = px - (piiX - ms->config.parzen3DKernelHalfWidth);
		    int kpy = py - (piiY - ms->config.parzen3DKernelHalfWidth);
		    int kpz = pz - (piiZ - ms->config.parzen3DKernelHalfWidth);

		    cv::Vec3b thisSample = getCRColor(ms, thisImage); 
		    ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 0*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] += thisSample[0]*ms->config.parzen3DKernel[kpx + kpy*ms->config.parzen3DKernelWidth + kpz*ms->config.parzen3DKernelWidth*ms->config.parzen3DKernelWidth];
		    ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 1*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] += thisSample[1]*ms->config.parzen3DKernel[kpx + kpy*ms->config.parzen3DKernelWidth + kpz*ms->config.parzen3DKernelWidth*ms->config.parzen3DKernelWidth];
		    ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 2*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] += thisSample[2]*ms->config.parzen3DKernel[kpx + kpy*ms->config.parzen3DKernelWidth + kpz*ms->config.parzen3DKernelWidth*ms->config.parzen3DKernelWidth];
		    ms->config.vmColorRangeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] += ms->config.parzen3DKernel[kpx + kpy*ms->config.parzen3DKernelWidth + kpz*ms->config.parzen3DKernelWidth*ms->config.parzen3DKernelWidth];

		    //double denomC = max(ms->config.vmColorRangeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth], EPSILON);
		    //int tRed = min(255, max(0,int(round(ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 2*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] / denomC))));
		    //int tGreen = min(255, max(0,int(round(ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 1*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] / denomC))));
		    //int tBlue = min(255, max(0,int(round(ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 0*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] / denomC))));

		    // slightly different than 2D
		    //ms->config.volumeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] += 0.0;
		    ms->config.volumeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] += ms->config.parzen3DKernel[kpx + kpy*ms->config.parzen3DKernelWidth + kpz*ms->config.parzen3DKernelWidth*ms->config.parzen3DKernelWidth];

		    double denom = max(ms->config.volumeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth], 1e-99); // XXX should be epsilon but there is clipping...
		    ms->config.volumeMap[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] = ms->config.volumeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] / denom;
		  }
		}
	      }
	    }
	  }
	  if ((fabs(ms->config.thisiX) <= ms->config.rmHalfWidth) && (fabs(ms->config.thisiY) <= ms->config.rmHalfWidth)) {
	    int iiX = (int)round(ms->config.thisiX + ms->config.rmHalfWidth);
	    int iiY = (int)round(ms->config.thisiY + ms->config.rmHalfWidth);
	    
	    {
	      ms->config.rangeMapMass[iiX + iiY*ms->config.rmWidth] += 1;
	      //ms->config.rangeMapAccumulator[iiX + iiY*ms->config.ms->config.rmWidth] += eeRange;
	      ms->config.rangeMapAccumulator[iiX + iiY*ms->config.rmWidth] += thisZmeasurement;
	      double denom = max(ms->config.rangeMapMass[iiX + iiY*ms->config.rmWidth], EPSILON);
	      ms->config.rangeMap[iiX + iiY*ms->config.rmWidth] = ms->config.rangeMapAccumulator[iiX + iiY*ms->config.rmWidth] / denom;
	    }
	    
	    double minDepth = VERYBIGNUMBER;
	    double maxDepth = 0;
	    for (int rx = 0; rx < ms->config.rmWidth; rx++) {
	      for (int ry = 0; ry < ms->config.rmWidth; ry++) {
		minDepth = min(minDepth, ms->config.rangeMap[rx + ry*ms->config.rmWidth]);
		maxDepth = max(maxDepth, ms->config.rangeMap[rx + ry*ms->config.rmWidth]);
	      }
	    }
	    double denom2 = max(EPSILON,maxDepth-minDepth);
	    if (denom2 <= EPSILON)
	      denom2 = VERYBIGNUMBER;
	    double intensity = 255 * (maxDepth - ms->config.rangeMap[iiX + iiY*ms->config.rmWidth]) / denom2;
	    cv::Scalar backColor(0,0,ceil(intensity));
	    cv::Point outTop = cv::Point(iiY*ms->config.rmiCellWidth,iiX*ms->config.rmiCellWidth);
	    cv::Point outBot = cv::Point((iiY+1)*ms->config.rmiCellWidth,(iiX+1)*ms->config.rmiCellWidth);
	    Mat vCrop = ms->config.rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
	    vCrop = backColor;
	    // draw border
	    {
	      cv::Point outTop = cv::Point(iiY*ms->config.rmiCellWidth+1,iiX*ms->config.rmiCellWidth+1);
	      cv::Point outBot = cv::Point((iiY+1)*ms->config.rmiCellWidth-1,(iiX+1)*ms->config.rmiCellWidth-1);
	      cv::Point inTop = cv::Point(outTop.x+1, outTop.y+1);
	      cv::Point inBot = cv::Point(outBot.x-1, outBot.y-1);
	      rectangle(ms->config.rangemapImage, outTop, outBot, cv::Scalar(0,192,0)); 
	      rectangle(ms->config.rangemapImage, inTop, inBot, cv::Scalar(0,64,0)); 
	    }
	  }
	}
    
	rgRingBufferAdvance(ms);
	allRingBuffersAdvance(ms, thisTime);
	IShouldContinue = 1; // not strictly necessary
      } else {
	IShouldContinue = 0;
	break;
      }
    }
  }
#ifdef DEBUG_RING_BUFFER
  cout << "recordReadyRangeReadings()  ms->config.rgRingBufferStart ms->config.rgRingBufferEnd: " << rgRingBufferStart << " " << ms->config.rgRingBufferEnd << endl;
#endif
}

void jointCallback(const sensor_msgs::JointState& js) {

//  if (!ms->config.shouldIMiscCallback) {
//    return;
//  }
  shared_ptr<MachineState> ms = pMachineState;
  if (ms->config.jointNamesInit) {
    int limit = js.position.size();
    for (int i = 0; i < limit; i++) {
      for (int j = 0; j < NUM_JOINTS; j++) {
	if (0 == js.name[i].compare(ms->config.jointNames[j]))
	  ms->config.trueJointPositions[j] = js.position[i];
	//cout << "tJP[" << j << "]: " << trueJointPositions[j] << endl;
      }
    }
  }
}

int classIdxForName(shared_ptr<MachineState> ms, string name) {
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

void fetchCommandCallback(const std_msgs::String::ConstPtr& msg) {
  shared_ptr<MachineState> ms = pMachineState;

  if (ros::Time::now() - ms->config.fetchCommandTime < ros::Duration(ms->config.fetchCommandCooldown)) {
    cout << "Received a fetch command but the fetchCommandCooldown hasn't expired so returning." << endl;
    return;
  }

  if (!ms->config.acceptingFetchCommands) {
    cout << "Received a fetch command but not accepting fetch commands so returning." << endl;
    return;
  }
  //ms->config.acceptingFetchCommands = 0;

  ms->config.fetchCommand = msg->data;
  ms->config.fetchCommandTime = ros::Time::now();
  ROS_INFO_STREAM("Received " << ms->config.fetchCommand << endl);

  int class_idx = classIdxForName(ms, ms->config.fetchCommand);

  if (class_idx == -1) {
    cout << "Could not find class " << ms->config.fetchCommand << endl; 
    // ATTN 25
    ms->pushWord("clearStackAcceptFetchCommands"); 
  } else {
    ms->clearStack();

    changeTargetClass(ms, class_idx);
    // ATTN 25
    //ms->pushWord("mappingPatrol");
    ms->pushWord("deliverTargetObject");
    ms->execute_stack = 1;
    ms->config.acceptingFetchCommands = 0;
  }
}

vector<string> split(const char *str, char c = ' ')
{
    vector<string> result;
    do
    {
      const char *begin = str;
      
      while(*str != c && *str)
        str++;
      
      result.push_back(string(begin, str));
    } while (0 != *str++);
    
    return result;
}


void moveEndEffectorCommandCallback(const geometry_msgs::Pose& msg) {
  cout << "moveEndEffectorCommandCallback" << endl << msg.position << msg.orientation << endl;
  shared_ptr<MachineState> ms = pMachineState;
  if (ms->config.currentRobotMode == PHYSICAL) {
    return;
  } else if (ms->config.currentRobotMode == SIMULATED) {
    ms->config.currentEEPose.px = msg.position.x;
    ms->config.currentEEPose.py = msg.position.y;
    ms->config.currentEEPose.pz = msg.position.z;
  } else {
    assert(0);
  }
}

void pickObjectUnderEndEffectorCommandCallback(const std_msgs::Empty& msg) {
  shared_ptr<MachineState> ms = pMachineState;
  if (ms->config.currentRobotMode == PHYSICAL) {
    return;
  } else if (ms->config.currentRobotMode == SIMULATED) {
    if (ms->config.objectInHandLabel == -1) {
      // this is a fake box to test intersection
      int probeBoxHalfWidthPixels = 10;
      BoxMemory box;
      box.bTop.x = ms->config.vanishingPointReticle.px-probeBoxHalfWidthPixels;
      box.bTop.y = ms->config.vanishingPointReticle.py-probeBoxHalfWidthPixels;
      box.bBot.x = ms->config.vanishingPointReticle.px+probeBoxHalfWidthPixels;
      box.bBot.y = ms->config.vanishingPointReticle.py+probeBoxHalfWidthPixels;
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
  } else {
    assert(0);
  }
}

void placeObjectInEndEffectorCommandCallback(const std_msgs::Empty& msg) {
  shared_ptr<MachineState> ms = pMachineState;
  if (ms->config.currentRobotMode == PHYSICAL) {
    return;
  } else if (ms->config.currentRobotMode == SIMULATED) {
    if (ms->config.objectInHandLabel >= 0) {
      BoxMemory box;
      box.bTop.x = ms->config.vanishingPointReticle.px-ms->config.simulatedObjectHalfWidthPixels;
      box.bTop.y = ms->config.vanishingPointReticle.py-ms->config.simulatedObjectHalfWidthPixels;
      box.bBot.x = ms->config.vanishingPointReticle.px+ms->config.simulatedObjectHalfWidthPixels;
      box.bBot.y = ms->config.vanishingPointReticle.py+ms->config.simulatedObjectHalfWidthPixels;
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
  } else {
    assert(0);
  }
}

void forthCommandCallback(const std_msgs::String::ConstPtr& msg) {

  // disabling this would be unwise
  shared_ptr<MachineState> ms = pMachineState;
  ms->config.forthCommand = msg->data;
  ROS_INFO_STREAM("Received " << ms->config.forthCommand << endl);
  vector<string> tokens = split(ms->config.forthCommand.c_str(), ' ');
  for (unsigned int i = 0; i < tokens.size(); i++) {
    trim(tokens[i]);
    if (!ms->pushWord(tokens[i])) {
      cout << "Warning, ignoring unknown word from the forth topic: " << tokens[i] << endl;
    }
  }
}

void endpointCallback(const baxter_core_msgs::EndpointState& eps) {
  doEndpointCallback(pMachineState, eps);
}

void doEndpointCallback(shared_ptr<MachineState> ms, const baxter_core_msgs::EndpointState& eps) {

//  if (!ms->config.shouldIMiscCallback) {
//    return;
//  }

  ms->config.lastEndpointCallbackReceived = ros::Time::now();

  // note that the quaternion field holds a vector3!
  ms->config.trueEEWrench.px = eps.wrench.force.x;
  ms->config.trueEEWrench.py = eps.wrench.force.y;
  ms->config.trueEEWrench.pz = eps.wrench.force.z;
  ms->config.trueEEWrench.qx = eps.wrench.torque.x;
  ms->config.trueEEWrench.qy = eps.wrench.torque.y;
  ms->config.trueEEWrench.qz = eps.wrench.torque.z;

  //cout << "endpoint frame_id: " << eps.header.frame_id << endl;
  ms->config.trueEEPose = eps.pose;
  ms->config.trueEEPoseEEPose.px = eps.pose.position.x;
  ms->config.trueEEPoseEEPose.py = eps.pose.position.y;
  ms->config.trueEEPoseEEPose.pz = eps.pose.position.z;
  ms->config.trueEEPoseEEPose.qx = eps.pose.orientation.x;
  ms->config.trueEEPoseEEPose.qy = eps.pose.orientation.y;
  ms->config.trueEEPoseEEPose.qz = eps.pose.orientation.z;
  ms->config.trueEEPoseEEPose.qw = eps.pose.orientation.w;

  setRingPoseAtTime(ms, eps.header.stamp, eps.pose);
  geometry_msgs::Pose thisPose;
  int weHavePoseData = getRingPoseAtTime(ms, eps.header.stamp, thisPose);

  {
    double distance = eePose::squareDistance(ms->config.trueEEPoseEEPose, ms->config.lastTrueEEPoseEEPose);
    double distance2 = eePose::squareDistance(ms->config.trueEEPoseEEPose, ms->config.currentEEPose);

    if (ms->config.currentMovementState == ARMED ) {
      if (distance2 > ms->config.armedThreshold*ms->config.armedThreshold) {
	cout << "armedThreshold crossed so leaving armed state into MOVING." << endl;
	ms->config.currentMovementState = MOVING;
	ms->config.lastTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
	ms->config.lastMovementStateSet = ros::Time::now();
      } else {
	//cout << "ms->config.currentMovementState is ARMED." << endl;
      }
    } else if (distance > ms->config.movingThreshold*ms->config.movingThreshold) {
      ms->config.currentMovementState = MOVING;
      ms->config.lastTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
      ms->config.lastMovementStateSet = ros::Time::now();
    } else if (distance > ms->config.hoverThreshold*ms->config.hoverThreshold) {
      if (distance2 > ms->config.hoverThreshold) {
	ms->config.currentMovementState = MOVING;
	ms->config.lastTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
	ms->config.lastMovementStateSet = ros::Time::now();
      } else {
	ms->config.currentMovementState = HOVERING;
	ms->config.lastTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
	ms->config.lastMovementStateSet = ros::Time::now();
      }

    } else {
      ros::Duration deltaT = ros::Time::now() - ms->config.lastMovementStateSet;
      if ( (deltaT.sec) > ms->config.stoppedTimeout ) {
	if (distance2 > ms->config.hoverThreshold*ms->config.hoverThreshold) {
	  ms->config.currentMovementState = BLOCKED;
	  ms->config.lastMovementStateSet = ros::Time::now();
	  ms->config.lastTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
	} else {
	  ms->config.currentMovementState = STOPPED;
	  ms->config.lastMovementStateSet = ros::Time::now();
	  ms->config.lastTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
	}
      }
    }
  }
}

void gripStateCallback(const baxter_core_msgs::EndEffectorState& ees) {

  shared_ptr<MachineState> ms = pMachineState;
  ms->config.lastGripperCallbackReceived = ros::Time::now();
  ms->config.gripperLastUpdated = ros::Time::now();
  ms->config.gripperPosition  = ees.position;
  ms->config.gripperMoving = ees.moving;
  ms->config.gripperGripping = ees.gripping;
}

bool isGripperGripping(shared_ptr<MachineState> ms) {
  //return (ms->config.gripperPosition >= ms->config.gripperThresh);
  return ms->config.gripperGripping; 
}

void initialize3DParzen(shared_ptr<MachineState> ms) {
  for (int kx = 0; kx < ms->config.parzen3DKernelWidth; kx++) {
    for (int ky = 0; ky < ms->config.parzen3DKernelWidth; ky++) {
      for (int kz = 0; kz < ms->config.parzen3DKernelWidth; kz++) {
	double pkx = kx - ms->config.parzen3DKernelHalfWidth;
	double pky = ky - ms->config.parzen3DKernelHalfWidth;
	double pkz = ky - ms->config.parzen3DKernelHalfWidth;
	ms->config.parzen3DKernel[kx + ky*ms->config.parzen3DKernelWidth + kz*ms->config.parzen3DKernelWidth*ms->config.parzen3DKernelWidth] = exp(-(pkx*pkx + pky*pky + pkz*pkz)/(2.0*ms->config.parzen3DKernelSigma*ms->config.parzen3DKernelSigma));
      }
    }
  }
}

void l2Normalize3DParzen(shared_ptr<MachineState> ms) {
  double norm = 0;
  for (int kx = 0; kx < ms->config.parzen3DKernelWidth; kx++) {
    for (int ky = 0; ky < ms->config.parzen3DKernelWidth; ky++) {
      for (int kz = 0; kz < ms->config.parzen3DKernelWidth; kz++) {
	double pkx = kx - ms->config.parzen3DKernelHalfWidth;
	double pky = ky - ms->config.parzen3DKernelHalfWidth;
	double pkz = ky - ms->config.parzen3DKernelHalfWidth;
	norm += ms->config.parzen3DKernel[kx + ky*ms->config.parzen3DKernelWidth + kz*ms->config.parzen3DKernelWidth*ms->config.parzen3DKernelWidth];
      }
    }
  }
  if (fabs(norm) < ms->config.fEpsilon)
    norm = 1;
  for (int kx = 0; kx < ms->config.parzen3DKernelWidth; kx++) {
    for (int ky = 0; ky < ms->config.parzen3DKernelWidth; ky++) {
      for (int kz = 0; kz < ms->config.parzen3DKernelWidth; kz++) {
	double pkx = kx - ms->config.parzen3DKernelHalfWidth;
	double pky = ky - ms->config.parzen3DKernelHalfWidth;
	double pkz = ky - ms->config.parzen3DKernelHalfWidth;

	ms->config.parzen3DKernel[kx + ky*ms->config.parzen3DKernelWidth + kz*ms->config.parzen3DKernelWidth*ms->config.parzen3DKernelWidth] /= norm;
#ifdef DEBUG_RING_BUFFER
	cout << "Parzen3D: " << ms->config.parzenKernel[kx + ky*ms->config.parzenKernelWidth] << endl;
#endif
      }
    }
  }
}

void initializeParzen(shared_ptr<MachineState> ms) {
  for (int kx = 0; kx < ms->config.parzenKernelWidth; kx++) {
    for (int ky = 0; ky < ms->config.parzenKernelWidth; ky++) {
      double pkx = kx - ms->config.parzenKernelHalfWidth;
      double pky = ky - ms->config.parzenKernelHalfWidth;
      ms->config.parzenKernel[kx + ky*ms->config.parzenKernelWidth] = exp(-(pkx*pkx + pky*pky)/(2.0*ms->config.parzenKernelSigma*ms->config.parzenKernelSigma));
    }
  }
}


void l2NormalizeParzen(shared_ptr<MachineState> ms) {
  double norm = 0;
  for (int kx = 0; kx < ms->config.parzenKernelWidth; kx++) {
    for (int ky = 0; ky < ms->config.parzenKernelWidth; ky++) {
      double pkx = kx - ms->config.parzenKernelHalfWidth;
      double pky = ky - ms->config.parzenKernelHalfWidth;
      norm += ms->config.parzenKernel[kx + ky*ms->config.parzenKernelWidth];
    }
  }
  if (fabs(norm) < ms->config.fEpsilon)
    norm = 1;
  for (int kx = 0; kx < ms->config.parzenKernelWidth; kx++) {
    for (int ky = 0; ky < ms->config.parzenKernelWidth; ky++) {
      double pkx = kx - ms->config.parzenKernelHalfWidth;
      double pky = ky - ms->config.parzenKernelHalfWidth;
      ms->config.parzenKernel[kx + ky*ms->config.parzenKernelWidth] /= norm;
#ifdef DEBUG_RING_BUFFER
      cout << "Parzen: " << ms->config.parzenKernel[kx + ky*ms->config.parzenKernelWidth] << endl;
#endif
    }
  }
}

void l2NormalizeFilter(shared_ptr<MachineState> ms) {
  double norm = 0;
  for (int fx = 0; fx < 9; fx++) {
    norm += ms->config.filter[fx]*ms->config.filter[fx];
  }
  if (fabs(norm) < ms->config.fEpsilon)
    norm = 1;
  for (int fx = 0; fx < 9; fx++) {
    ms->config.filter[fx] /= norm;
  }
}


int getColorReticleX(shared_ptr<MachineState> ms) {
  // rounding
  //int tcri = int(round((eeRange - ms->config.firstCReticleIndexDepth)/ms->config.cReticleIndexDelta));
  //tcri = min(max(tcri,0),ms->config.numCReticleIndeces-1);
  //return ms->config.xCR[tcri];

  // interpolating
  int tcriL = int(floor((ms->config.eeRange - ms->config.firstCReticleIndexDepth)/ms->config.cReticleIndexDelta));
  int tcriH = int(ceil((ms->config.eeRange - ms->config.firstCReticleIndexDepth)/ms->config.cReticleIndexDelta));
  tcriL = min(max(tcriL,0),ms->config.numCReticleIndeces-1);
  tcriH = min(max(tcriH,0),ms->config.numCReticleIndeces-1);

  double tcrwL = ((ms->config.eeRange - ms->config.firstCReticleIndexDepth)/ms->config.cReticleIndexDelta) - floor((ms->config.eeRange - ms->config.firstCReticleIndexDepth)/ms->config.cReticleIndexDelta);
  double tcrwH = ceil((ms->config.eeRange - ms->config.firstCReticleIndexDepth)/ms->config.cReticleIndexDelta) - ((ms->config.eeRange - ms->config.firstCReticleIndexDepth)/ms->config.cReticleIndexDelta);

  if (tcriL == tcriH)
    return ms->config.xCR[tcriL];
  else
    return int(round(tcrwL*double(ms->config.xCR[tcriL]) + tcrwH*double(ms->config.xCR[tcriH])));
}

int getColorReticleY(shared_ptr<MachineState> ms) {
  // rounding
  //int tcri = int(round((ms->config.eeRange - ms->config.firstCReticleIndexDepth)/ms->config.cReticleIndexDelta));
  //tcri = min(max(tcri,0),ms->config.numCReticleIndeces-1);
  //return ms->config.yCR[tcri];

  // interpolating
  int tcriL = int(floor((ms->config.eeRange - ms->config.firstCReticleIndexDepth)/ms->config.cReticleIndexDelta));
  int tcriH = int(ceil((ms->config.eeRange - ms->config.firstCReticleIndexDepth)/ms->config.cReticleIndexDelta));
  tcriL = min(max(tcriL,0),ms->config.numCReticleIndeces-1);
  tcriH = min(max(tcriH,0),ms->config.numCReticleIndeces-1);

  double tcrwL = ((ms->config.eeRange - ms->config.firstCReticleIndexDepth)/ms->config.cReticleIndexDelta) - floor((ms->config.eeRange - ms->config.firstCReticleIndexDepth)/ms->config.cReticleIndexDelta);
  double tcrwH = ceil((ms->config.eeRange - ms->config.firstCReticleIndexDepth)/ms->config.cReticleIndexDelta) - ((ms->config.eeRange - ms->config.firstCReticleIndexDepth)/ms->config.cReticleIndexDelta);

  if (tcriL == tcriH)
    return ms->config.yCR[tcriL];
  else
    return int(round(tcrwL*double(ms->config.yCR[tcriL]) + tcrwH*double(ms->config.yCR[tcriH])));
}

cv::Vec3b getCRColor(shared_ptr<MachineState> ms) {
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
cv::Vec3b getCRColor(shared_ptr<MachineState> ms, Mat im) {
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




void scanXdirection(shared_ptr<MachineState> ms, double speedOnLines, double speedBetweenLines) {
// XXX TODO work this out so that it scans from -ms->config.rmHalfWidth*ms->config.rmDelta to ms->config.rmHalfWidth*ms->config.rmDelta

// XXX TODO right now we need to exit after every increment to set a new position in case there was an IK error

  double onLineGain = ms->config.rmDelta / speedOnLines;
  double betweenLineGain = ms->config.rmDelta / speedBetweenLines;

  int scanPadding = int(floor(1 * onLineGain));

  ms->pushWord("waitUntilAtCurrentPosition"); 
  for (int g = 0; g < ((ms->config.rmWidth*onLineGain)-(ms->config.rmHalfWidth*onLineGain))+scanPadding; g++) {
    ms->pushWord('a');
    ms->pushWord("endStackCollapseNoop");
  }
  for (int g = 0; g < ms->config.rmHalfWidth*onLineGain+scanPadding; g++) {
    ms->pushWord('e');
    ms->pushWord("endStackCollapseNoop");
  }

  ms->pushWord("waitUntilAtCurrentPosition"); 
  //int gLimit = 1+((ms->config.rmWidth*betweenLineGain+2*scanPadding)/2);
  int gLimit = ((ms->config.rmWidth*betweenLineGain+2*scanPadding));
  for (int g = 0; g < gLimit; g++) {
    ms->pushWord("fullRender"); 
    ms->pushWord("waitUntilAtCurrentPosition"); 
    ms->pushWord('d');
    ms->pushWord("waitUntilAtCurrentPosition");
    for (int gg = 0; gg < ms->config.rmWidth*onLineGain+2*scanPadding; gg++) {
      ms->pushWord('q');
      ms->pushWord("endStackCollapseNoop");
    }
    ms->pushWord("waitUntilAtCurrentPosition"); 
    for (int gg = 0; gg < ms->config.rmWidth*onLineGain+2*scanPadding; gg++) {
      ms->pushWord('e');
      ms->pushWord("endStackCollapseNoop");
    }
  }

  ms->pushWord("waitUntilAtCurrentPosition"); 
  for (int g = 0; g < ms->config.rmHalfWidth*onLineGain+scanPadding; g++) {
    ms->pushWord('q');
    ms->pushWord("endStackCollapseNoop");
  }
  for (int g = 0; g < ms->config.rmHalfWidth*onLineGain+scanPadding; g++) {
    ms->pushWord('a');
    ms->pushWord("endStackCollapseNoop");
  }
}


void scanYdirection(shared_ptr<MachineState> ms, double speedOnLines, double speedBetweenLines) {

  double onLineGain = ms->config.rmDelta / speedOnLines;
  double betweenLineGain = ms->config.rmDelta / speedBetweenLines;

  int scanPadding = int(floor(1 * onLineGain));

  for (int g = 0; g < ((ms->config.rmWidth*onLineGain)-(ms->config.rmHalfWidth*onLineGain))+scanPadding; g++) {
    // ATTN 2
    //ms->pushWord(1048677);
    ms->pushWord("waitUntilAtCurrentPosition"); 
    ms->pushWord('q');
  }
  for (int g = 0; g < ms->config.rmHalfWidth*onLineGain+scanPadding; g++) {
    //ms->pushWord(1048677);
    ms->pushWord("waitUntilAtCurrentPosition"); 
    ms->pushWord('d');
  }
  pushSpeedSign(ms, speedOnLines);

  //int gLimit = 1+((ms->config.rmWidth*betweenLineGain+2*scanPadding)/2);
  int gLimit = ((ms->config.rmWidth*betweenLineGain+2*scanPadding));
  for (int g = 0; g < gLimit; g++) {
    ms->pushWord("fullRender"); // full render
    //ms->pushWord(1048677);
    ms->pushWord("waitUntilAtCurrentPosition"); 
    pushSpeedSign(ms, speedOnLines);
    ms->pushWord('e');
    pushSpeedSign(ms, speedBetweenLines);
    for (int gg = 0; gg < ms->config.rmWidth*onLineGain+2*scanPadding; gg++) {
      //ms->pushWord(1048677);
      ms->pushWord("waitUntilAtCurrentPosition"); 
      ms->pushWord('a');
    }
    //pushSpeedSign(ms, speedOnLines);
    //ms->pushWord('e');
    //pushSpeedSign(ms, speedBetweenLines);
    for (int gg = 0; gg < ms->config.rmWidth*onLineGain+2*scanPadding; gg++) {
      //ms->pushWord(1048677);
      ms->pushWord("waitUntilAtCurrentPosition"); 
      ms->pushWord('d');
    }
  }

  for (int g = 0; g < ms->config.rmHalfWidth*onLineGain+scanPadding; g++) {
    //ms->pushWord(1048677);
    ms->pushWord("waitUntilAtCurrentPosition"); 
    ms->pushWord('q');
  }
  for (int g = 0; g < ms->config.rmHalfWidth*onLineGain+scanPadding; g++) {
    //ms->pushWord(1048677);
    ms->pushWord("waitUntilAtCurrentPosition"); 
    ms->pushWord('a');
  }
  pushSpeedSign(ms, speedOnLines);
}

Eigen::Quaternionf getGGRotation(shared_ptr<MachineState> ms, int givenGraspGear) {
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

void setGGRotation(shared_ptr<MachineState> ms, int thisGraspGear) {
  Eigen::Quaternionf eeBaseQuat = getGGRotation(ms, thisGraspGear);

  ms->config.currentEEPose.qx = eeBaseQuat.x();
  ms->config.currentEEPose.qy = eeBaseQuat.y();
  ms->config.currentEEPose.qz = eeBaseQuat.z();
  ms->config.currentEEPose.qw = eeBaseQuat.w();
}

Eigen::Quaternionf getCCRotation(shared_ptr<MachineState> ms, int givenGraspGear, double angle) {
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

void setCCRotation(shared_ptr<MachineState> ms, int thisGraspGear) {
  //Eigen::Quaternionf eeBaseQuat = getCCRotation(ms, thisGraspGear, -ms->config.bestOrientationAngle);
  Eigen::Quaternionf eeBaseQuat = getCCRotation(ms, thisGraspGear, 0.0);

  ms->config.currentEEPose.qx = eeBaseQuat.x();
  ms->config.currentEEPose.qy = eeBaseQuat.y();
  ms->config.currentEEPose.qz = eeBaseQuat.z();
  ms->config.currentEEPose.qw = eeBaseQuat.w();
}

void accelerometerCallback(const sensor_msgs::Imu& moment) {
  shared_ptr<MachineState> ms = pMachineState;
  ms->config.lastAccelerometerCallbackReceived = ros::Time::now();
  ms->config.eeLinearAcceleration = Vector3d(
    moment.linear_acceleration.x,
    moment.linear_acceleration.y,
    moment.linear_acceleration.z );
}

void rangeCallback(const sensor_msgs::Range& range) {
  shared_ptr<MachineState> ms = pMachineState;
  //cout << "range frame_id: " << range.header.frame_id << endl;
  setRingRangeAtTime(ms, range.header.stamp, range.range);
  //double thisRange;
  //int weHaveRangeData = getRingRangeAtTime(range.header.stamp, thisRange);


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
  guardedImshow(ms->config.rangeogramViewName, ms->config.rangeogramImage, ms->config.sirRangeogram);

  if (!ms->config.shouldIRangeCallback) {
    return;
  }


  if (ms->config.recordRangeMap) {
    // actually storing the negative z for backwards compatibility
    //double thisZmeasurement = -(ms->config.currentEEPose.pz - ms->config.eeRange);
    double thisZmeasurement = -(ms->config.trueEEPose.position.z - ms->config.eeRange);
    double dX = 0;
    double dY = 0;

    {
      Eigen::Quaternionf crane2quat(ms->config.straightDown.qw, ms->config.straightDown.qx, ms->config.straightDown.qy, ms->config.straightDown.qz);
      ms->config.irGlobalPositionEEFrame = crane2quat.conjugate() * ms->config.gear0offset * crane2quat;
      Eigen::Quaternionf ceeQuat(ms->config.trueEEPose.orientation.w, ms->config.trueEEPose.orientation.x, ms->config.trueEEPose.orientation.y, ms->config.trueEEPose.orientation.z);
      Eigen::Quaternionf irSensorStartLocal = ceeQuat * ms->config.irGlobalPositionEEFrame * ceeQuat.conjugate();
      Eigen::Quaternionf irSensorStartGlobal(
					      0.0,
					     (ms->config.trueEEPose.position.x - irSensorStartLocal.x()),
					     (ms->config.trueEEPose.position.y - irSensorStartLocal.y()),
					     (ms->config.trueEEPose.position.z - irSensorStartLocal.z())
					    );

      Eigen::Quaternionf globalUnitZ(0, 0, 0, 1);
      Eigen::Quaternionf localUnitZ = ceeQuat * globalUnitZ * ceeQuat.conjugate();

      Eigen::Vector3d irSensorEnd(
				   (ms->config.trueEEPose.position.x - irSensorStartLocal.x()) + ms->config.eeRange*localUnitZ.x(),
				   (ms->config.trueEEPose.position.y - irSensorStartLocal.y()) + ms->config.eeRange*localUnitZ.y(),
				   (ms->config.trueEEPose.position.z - irSensorStartLocal.z()) + ms->config.eeRange*localUnitZ.z()
				  );

      dX = (irSensorEnd.x() - ms->config.rmcX); 
      dY = (irSensorEnd.y() - ms->config.rmcY); 

      double eX = (irSensorEnd.x() - ms->config.rmcX) / ms->config.hrmDelta;
      double eY = (irSensorEnd.y() - ms->config.rmcY) / ms->config.hrmDelta;
      int eeX = (int)round(eX + ms->config.hrmHalfWidth);
      int eeY = (int)round(eY + ms->config.hrmHalfWidth);

      if ((fabs(eX) <= ms->config.hrmHalfWidth) && (fabs(eY) <= ms->config.hrmHalfWidth)) {
	ms->config.hiRangemapImage.at<cv::Vec3b>(eeX,eeY) += cv::Vec3b(128,0,0);
      }
      // XXX
      thisZmeasurement = -irSensorEnd.z();
    }

    // find current rangemap slot
    // check to see if it falls in our mapped region
    // if so, update the arrays and draw the slot
    // XXX
    //double dX = (ms->config.trueEEPose.position.x - ms->config.drX) - rmcX;
    //double dY = (ms->config.trueEEPose.position.y - ms->config.drY) - ms->config.rmcY;

    double iX = dX / ms->config.rmDelta;
    double iY = dY / ms->config.rmDelta;

    double hiX = dX / ms->config.hrmDelta;
    double hiY = dY / ms->config.hrmDelta;

    ms->config.lastiX = ms->config.thisiX;
    ms->config.lastiY = ms->config.thisiY;
    ms->config.thisiX = iX;
    ms->config.thisiY = iY;

    
  //cout << rmcX << " " << ms->config.trueEEPose.position.x << " " << dX << " " << iX << " " << ms->config.rmHalfWidth << endl;

    // erase old cell
    if ((fabs(ms->config.lastiX) <= ms->config.rmHalfWidth) && (fabs(ms->config.lastiY) <= ms->config.rmHalfWidth)) {
      int iiX = (int)round(ms->config.lastiX + ms->config.rmHalfWidth);
      int iiY = (int)round(ms->config.lastiY + ms->config.rmHalfWidth);

      double minDepth = VERYBIGNUMBER;
      double maxDepth = 0;
      for (int rx = 0; rx < ms->config.rmWidth; rx++) {
	for (int ry = 0; ry < ms->config.rmWidth; ry++) {
	  minDepth = min(minDepth, ms->config.rangeMap[rx + ry*ms->config.rmWidth]);
	  maxDepth = max(maxDepth, ms->config.rangeMap[rx + ry*ms->config.rmWidth]);
	}
      }
      double denom2 = max(EPSILON,maxDepth-minDepth);
      if (denom2 <= EPSILON)
	denom2 = VERYBIGNUMBER;
      double intensity = 255 * (maxDepth - ms->config.rangeMap[iiX + iiY*ms->config.rmWidth]) / denom2;
      cv::Scalar backColor(0,0,ceil(intensity));
      cv::Point outTop = cv::Point(iiY*ms->config.rmiCellWidth,iiX*ms->config.rmiCellWidth);
      cv::Point outBot = cv::Point((iiY+1)*ms->config.rmiCellWidth,(iiX+1)*ms->config.rmiCellWidth);
      Mat vCrop = ms->config.rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
      vCrop = backColor;
    }



    // draw new cell
    if ((fabs(hiX) <= ms->config.hrmHalfWidth) && (fabs(hiY) <= ms->config.hrmHalfWidth)) {
      int hiiX = (int)round(hiX + ms->config.hrmHalfWidth);
      int hiiY = (int)round(hiY + ms->config.hrmHalfWidth);

      // the wrong point without pose correction
      //double upX = ((ms->config.trueEEPose.position.x - ms->config.drX) - rmcX)/ms->config.hrmDelta;
      //double upY = ((ms->config.trueEEPose.position.y - ms->config.drY) - ms->config.rmcY)/ms->config.hrmDelta;
      //int iupX = (int)round(upX + ms->config.hrmHalfWidth);
      //int iupY = (int)round(upY + ms->config.hrmHalfWidth);
      //if ((fabs(upX) <= ms->config.hrmHalfWidth) && (fabs(upY) <= ms->config.hrmHalfWidth)) 
	//ms->config.hiRangemapImage.at<cv::Vec3b>(iupX,iupY) += cv::Vec3b(0,128,0);

      int pxMin = max(0, hiiX-ms->config.parzenKernelHalfWidth);
      int pxMax = min(ms->config.hrmWidth-1, hiiX+ms->config.parzenKernelHalfWidth);
      int pyMin = max(0, hiiY-ms->config.parzenKernelHalfWidth);
      int pyMax = min(ms->config.hrmWidth-1, hiiY+ms->config.parzenKernelHalfWidth);
      for (int px = pxMin; px <= pxMax; px++) {
	for (int py = pyMin; py <= pyMax; py++) {
	  int kpx = px - (hiiX - ms->config.parzenKernelHalfWidth);
	  int kpy = py - (hiiY - ms->config.parzenKernelHalfWidth);

	  cv::Vec3b thisSample = getCRColor(ms); 
//	  ms->config.hiColorRangeMapAccumulator[px + py*ms->config.hrmWidth + 0*ms->config.hrmWidth*ms->config.hrmWidth] += thisSample[0]*parzenKernel[kpx + kpy*ms->config.parzenKernelWidth];
//	  ms->config.hiColorRangeMapAccumulator[px + py*ms->config.hrmWidth + 1*ms->config.hrmWidth*ms->config.hrmWidth] += thisSample[1]*parzenKernel[kpx + kpy*ms->config.parzenKernelWidth];
//	  ms->config.hiColorRangeMapAccumulator[px + py*ms->config.hrmWidth + 2*ms->config.hrmWidth*ms->config.hrmWidth] += thisSample[2]*parzenKernel[kpx + kpy*ms->config.parzenKernelWidth];
//	  ms->config.hiColorRangeMapMass[px + py*ms->config.hrmWidth] += parzenKernel[kpx + kpy*ms->config.parzenKernelWidth];
//
//	  double denomC = max(ms->config.hiColorRangeMapMass[px + py*ms->config.hrmWidth], EPSILON);
//	  int tRed = min(255, max(0,int(round(ms->config.hiColorRangeMapAccumulator[px + py*ms->config.hrmWidth + 2*ms->config.hrmWidth*ms->config.hrmWidth] / denomC))));
//	  int tGreen = min(255, max(0,int(round(ms->config.hiColorRangeMapAccumulator[px + py*ms->config.hrmWidth + 1*ms->config.hrmWidth*ms->config.hrmWidth] / denomC))));
//	  int tBlue = min(255, max(0,int(round(ms->config.hiColorRangeMapAccumulator[px + py*ms->config.hrmWidth + 0*ms->config.hrmWidth*ms->config.hrmWidth] / denomC))));
//
//	  ms->config.hiColorRangemapImage.at<cv::Vec3b>(px,py) = cv::Vec3b(tBlue, tGreen, tRed);

	  //ms->config.hiRangeMapAccumulator[px + py*ms->config.hrmWidth] += ms->config.eeRange*parzenKernel[kpx + kpy*ms->config.parzenKernelWidth];
	  //ms->config.hiRangeMapAccumulator[px + py*ms->config.hrmWidth] += thisZmeasurement*parzenKernel[kpx + kpy*ms->config.parzenKernelWidth];
	  //ms->config.hiRangeMapMass[px + py*ms->config.hrmWidth] += parzenKernel[kpx + kpy*ms->config.parzenKernelWidth];
	  //double denom = max(ms->config.hiRangeMapMass[px + py*ms->config.hrmWidth], EPSILON);
	  //ms->config.hiRangeMap[px + py*ms->config.hrmWidth] = ms->config.hiRangeMapAccumulator[px + py*ms->config.hrmWidth] / denom;
	}
      }
    }
    if ((fabs(ms->config.thisiX) <= ms->config.rmHalfWidth) && (fabs(ms->config.thisiY) <= ms->config.rmHalfWidth)) {
      int iiX = (int)round(ms->config.thisiX + ms->config.rmHalfWidth);
      int iiY = (int)round(ms->config.thisiY + ms->config.rmHalfWidth);
      
      {
	//ms->config.rangeMapMass[iiX + iiY*rmWidth] += 1;
	//ms->config.rangeMapAccumulator[iiX + iiY*rmWidth] += thisZmeasurement;
	//double denom = max(ms->config.rangeMapMass[iiX + iiY*rmWidth], EPSILON);
	//ms->config.rangeMap[iiX + iiY*rmWidth] = ms->config.rangeMapAccumulator[iiX + iiY*rmWidth] / denom;
      }
      
      double minDepth = VERYBIGNUMBER;
      double maxDepth = 0;
      for (int rx = 0; rx < ms->config.rmWidth; rx++) {
	for (int ry = 0; ry < ms->config.rmWidth; ry++) {
	  minDepth = min(minDepth, ms->config.rangeMap[rx + ry*ms->config.rmWidth]);
	  maxDepth = max(maxDepth, ms->config.rangeMap[rx + ry*ms->config.rmWidth]);
	}
      }
      double denom2 = max(EPSILON,maxDepth-minDepth);
      if (denom2 <= EPSILON)
	denom2 = VERYBIGNUMBER;
      double intensity = 255 * (maxDepth - ms->config.rangeMap[iiX + iiY*ms->config.rmWidth]) / denom2;
      cv::Scalar backColor(0,0,ceil(intensity));
      cv::Point outTop = cv::Point(iiY*ms->config.rmiCellWidth,iiX*ms->config.rmiCellWidth);
      cv::Point outBot = cv::Point((iiY+1)*ms->config.rmiCellWidth,(iiX+1)*ms->config.rmiCellWidth);
      Mat vCrop = ms->config.rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
      vCrop = backColor;
      // draw border
      {
	cv::Point outTop = cv::Point(iiY*ms->config.rmiCellWidth+1,iiX*ms->config.rmiCellWidth+1);
	cv::Point outBot = cv::Point((iiY+1)*ms->config.rmiCellWidth-1,(iiX+1)*ms->config.rmiCellWidth-1);
	cv::Point inTop = cv::Point(outTop.x+1, outTop.y+1);
	cv::Point inBot = cv::Point(outBot.x-1, outBot.y-1);
	rectangle(ms->config.rangemapImage, outTop, outBot, cv::Scalar(0,192,0)); 
	rectangle(ms->config.rangemapImage, inTop, inBot, cv::Scalar(0,64,0)); 
      }
    }
  }

  if (ms->config.shouldIRender) {
    guardedImshow(ms->config.rangemapViewName, ms->config.rangemapImage, ms->config.sirRangemap);
    guardedImshow(ms->config.graspMemoryViewName, ms->config.graspMemoryImage, ms->config.sirGraspMemory);
    guardedImshow(ms->config.graspMemorySampleViewName, ms->config.graspMemorySampleImage, ms->config.sirGraspMemorySample);
    guardedImshow(ms->config.heightMemorySampleViewName, ms->config.heightMemorySampleImage, ms->config.sirHeightMemorySample);
    Mat hRIT;
    cv::resize(ms->config.hiRangemapImage, hRIT, cv::Size(0,0), 2, 2);
    guardedImshow(ms->config.hiRangemapViewName, hRIT, ms->config.sirHiRangemap);
    Mat hCRIT;
    cv::resize(ms->config.hiColorRangemapImage, hCRIT, cv::Size(0,0), 2, 2);
    guardedImshow(ms->config.hiColorRangemapViewName, hCRIT, ms->config.sirHiColorRangemap);

    guardedImshow(ms->config.objectViewerName, ms->config.objectViewerImage, ms->config.sirObject);
    guardedImshow(ms->config.objectMapViewerName, ms->config.objectMapViewerImage, ms->config.sirObjectMap);
    //cv::moveWindow(ms->config.objectMapViewerName, 0, 0);

    guardedImshow(ms->config.densityViewerName, ms->config.densityViewerImage, ms->config.sirDensity);
    guardedImshow(ms->config.gradientViewerName, ms->config.gradientViewerImage, ms->config.sirGradient);

    guardedImshow(ms->config.mapBackgroundViewName, ms->config.mapBackgroundImage, ms->config.sirMapBackground);
    
    if (ms->config.targetClass > -1) {
      if (ms->config.classHeight0AerialGradients[ms->config.targetClass].rows == ms->config.aerialGradientWidth) {
	Mat crop0 = ms->config.aerialGradientViewerImage(cv::Rect(0, 3*ms->config.aerialGradientWidth, ms->config.aerialGradientWidth, ms->config.aerialGradientWidth));
	double min0 = 0;
	double max0 = 0;
	minMaxLoc(ms->config.classHeight0AerialGradients[ms->config.targetClass], &min0, &max0);
	double denom0 = max0-min0;
	if (fabs(denom0) < EPSILON)
	  denom0 = 1;
	crop0 = (ms->config.classHeight0AerialGradients[ms->config.targetClass] - min0) / denom0;

	Mat crop1 = ms->config.aerialGradientViewerImage(cv::Rect(0, 2*ms->config.aerialGradientWidth, ms->config.aerialGradientWidth, ms->config.aerialGradientWidth));
	double min1 = 0;
	double max1 = 0;
	minMaxLoc(ms->config.classHeight1AerialGradients[ms->config.targetClass], &min1, &max1);
	double denom1 = max1-min1;
	if (fabs(denom1) < EPSILON)
	  denom1 = 1;
	crop1 = (ms->config.classHeight1AerialGradients[ms->config.targetClass] - min1) / denom1;

	Mat crop2 = ms->config.aerialGradientViewerImage(cv::Rect(0, 1*ms->config.aerialGradientWidth, ms->config.aerialGradientWidth, ms->config.aerialGradientWidth));
	double min2 = 0;
	double max2 = 0;
	minMaxLoc(ms->config.classHeight2AerialGradients[ms->config.targetClass], &min2, &max2);
	double denom2 = max2-min2;
	if (fabs(denom2) < EPSILON)
	  denom2 = 1;
	crop2 = (ms->config.classHeight2AerialGradients[ms->config.targetClass] - min2) / denom2;

	Mat crop3 = ms->config.aerialGradientViewerImage(cv::Rect(0, 0*ms->config.aerialGradientWidth, ms->config.aerialGradientWidth, ms->config.aerialGradientWidth));
	double min3 = 0;
	double max3 = 0;
	minMaxLoc(ms->config.classHeight3AerialGradients[ms->config.targetClass], &min3, &max3);
	double denom3 = max3-min3;
	if (fabs(denom3) < EPSILON)
	  denom3 = 1;
	crop3 = (ms->config.classHeight3AerialGradients[ms->config.targetClass] - min3) / denom3;

	guardedImshow(ms->config.aerialGradientViewerName, ms->config.aerialGradientViewerImage, ms->config.sirAerialGradient);
      }
    }
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
  //eeBaseQuat = eeRotatorX * eeRotatorY * eeRotatorZ * 
		//eeBaseQuat * 
	      //eeRotatorZ.conjugate() * eeRotatorY.conjugate() * eeRotatorX.conjugate();
  eeBaseQuat = eeRotatorX * eeRotatorY * eeRotatorZ * eeBaseQuat;
  eeBaseQuat.normalize();

  givenEEPose->qx = eeBaseQuat.x();
  givenEEPose->qy = eeBaseQuat.y();
  givenEEPose->qz = eeBaseQuat.z();
  givenEEPose->qw = eeBaseQuat.w();
}

void fillIkRequest(eePose * givenEEPose, baxter_core_msgs::SolvePositionIK * givenIkRequest) {
  givenIkRequest->request.pose_stamp.resize(1);

  givenIkRequest->request.pose_stamp[0].header.seq = 0;
  givenIkRequest->request.pose_stamp[0].header.stamp = ros::Time::now();
  givenIkRequest->request.pose_stamp[0].header.frame_id = "/base";

  
  givenIkRequest->request.pose_stamp[0].pose.position.x = givenEEPose->px;
  givenIkRequest->request.pose_stamp[0].pose.position.y = givenEEPose->py;
  givenIkRequest->request.pose_stamp[0].pose.position.z = givenEEPose->pz;

//  Eigen::Quaternionf normalizer(givenEEPose->qw, givenEEPose->qx, givenEEPose->qy, givenEEPose->qz);
//  normalizer.normalize();
//  givenEEPose->qx = normalizer.x();
//  givenEEPose->qy = normalizer.y();
//  givenEEPose->qz = normalizer.z();
//  givenEEPose->qw = normalizer.w();

  givenIkRequest->request.pose_stamp[0].pose.orientation.x = givenEEPose->qx;
  givenIkRequest->request.pose_stamp[0].pose.orientation.y = givenEEPose->qy;
  givenIkRequest->request.pose_stamp[0].pose.orientation.z = givenEEPose->qz;
  givenIkRequest->request.pose_stamp[0].pose.orientation.w = givenEEPose->qw;
}

void reseedIkRequest(shared_ptr<MachineState> ms, eePose *givenEEPose, baxter_core_msgs::SolvePositionIK * givenIkRequest, int it, int itMax) {

  double jointSeedAmplitude = (3.1415926 * double(it) / double(itMax));
  double jointSeedAmplitudeMin = 0.02;
  jointSeedAmplitude = max(jointSeedAmplitude, jointSeedAmplitudeMin);

  if (ms->config.goodIkInitialized) {
    givenIkRequest->request.seed_mode = 1; // SEED_USER
    givenIkRequest->request.seed_angles.resize(1);
    givenIkRequest->request.seed_angles[0].position.resize(NUM_JOINTS);
    givenIkRequest->request.seed_angles[0].name.resize(NUM_JOINTS);
    for (int j = 0; j < NUM_JOINTS; j++) {
      givenIkRequest->request.seed_angles[0].name[j] = ms->config.lastGoodIkRequest.response.joints[0].name[j];
      givenIkRequest->request.seed_angles[0].position[j] = ms->config.lastGoodIkRequest.response.joints[0].position[j] + 
	((drand48() - 0.5)*2.0*jointSeedAmplitude);
    }
  } else {
    ROS_WARN_STREAM("_______**__________");
    ROS_WARN_STREAM("_____*____*________");
    ROS_ERROR_STREAM("Uh oh, tried to reseed ik before it was initialized, so returning with no action.");
    ROS_WARN_STREAM("_____*____*________");
    ROS_WARN_STREAM("_______**__________");
    return;
  }
}

bool willIkResultFail(shared_ptr<MachineState> ms, baxter_core_msgs::SolvePositionIK thisIkRequest, int thisIkCallResult, bool * likelyInCollision) {
  bool thisIkResultFailed = 0;
  *likelyInCollision = 0;

  if (thisIkCallResult && thisIkRequest.response.isValid[0]) {
    thisIkResultFailed = 0;
  } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].position.size() != NUM_JOINTS)) {
    thisIkResultFailed = 1;
    //cout << "Initial IK result appears to be truly invalid, not enough positions." << endl;
  } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].name.size() != NUM_JOINTS)) {
    thisIkResultFailed = 1;
    //cout << "Initial IK result appears to be truly invalid, not enough names." << endl;
  } else if (thisIkRequest.response.joints.size() == 1) {
    if( ms->config.usePotentiallyCollidingIK ) {
      //cout << "WARNING: using ik even though result was invalid under presumption of false collision..." << endl;
      //cout << "Received enough positions and names for ikPose: " << thisIkRequest.request.pose_stamp[0].pose << endl;
      thisIkResultFailed = 0;
      *likelyInCollision = 1;
    } else {
      thisIkResultFailed = 1;
      *likelyInCollision = 1;
    }
  } else {
    thisIkResultFailed = 1;
    //cout << "Initial IK result appears to be truly invalid, incorrect joint field." << endl;
  }

  return thisIkResultFailed;
}

void update_baxter(ros::NodeHandle &n) {

  shared_ptr<MachineState> ms = pMachineState;
  ms->config.bfc = ms->config.bfc % ms->config.bfc_period;
  if (!ms->config.shouldIDoIK) {
    return;
  }

  if (ms->config.currentRobotMode == SIMULATED) {
    return;
  }

  baxter_core_msgs::SolvePositionIK thisIkRequest;
  fillIkRequest(&ms->config.currentEEPose, &thisIkRequest);

  int ikResultFailed = 0;
  eePose originalCurrentEEPose = ms->config.currentEEPose;

  // do not start in a state with ikShare 
  if ((drand48() <= ms->config.ikShare) || !ms->config.ikInitialized) {

    int numIkRetries = 100; //5000;//100;
    double ikNoiseAmplitude = 0.01;//0.1;//0.03;
    double useZOnly = 1;
    double ikNoiseAmplitudeQuat = 0;
    for (int ikRetry = 0; ikRetry < numIkRetries; ikRetry++) {
      // ATTN 24
      //int ikCallResult = ms->config.ikClient.call(thisIkRequest);
      int ikCallResult = 0;
      queryIK(ms, &ikCallResult, &thisIkRequest);

      if (ikCallResult && thisIkRequest.response.isValid[0]) {
	// set this here in case noise was added
	ms->config.currentEEPose.px = thisIkRequest.request.pose_stamp[0].pose.position.x;
	ms->config.currentEEPose.py = thisIkRequest.request.pose_stamp[0].pose.position.y;
	ms->config.currentEEPose.pz = thisIkRequest.request.pose_stamp[0].pose.position.z;
	ikResultFailed = 0;
	if (ikRetry > 0) {
	  ROS_WARN_STREAM("___________________");
	  ROS_ERROR_STREAM("Accepting perturbed IK result.");
	  cout << "ikRetry: " << ikRetry << endl;
	  eePose::print(originalCurrentEEPose);
	  eePose::print(ms->config.currentEEPose);
	  ROS_WARN_STREAM("___________________");
	}
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].position.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough positions." << endl;
      } else if ((thisIkRequest.response.joints.size() == 1) && (thisIkRequest.response.joints[0].name.size() != NUM_JOINTS)) {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, not enough names." << endl;
      } else if (thisIkRequest.response.joints.size() == 1) {
	if( ms->config.usePotentiallyCollidingIK ) {
	  cout << "WARNING: using ik even though result was invalid under presumption of false collision..." << endl;
	  cout << "Received enough positions and names for ikPose: " << thisIkRequest.request.pose_stamp[0].pose << endl;

	  ikResultFailed = 0;
	  ms->config.currentEEPose.px = thisIkRequest.request.pose_stamp[0].pose.position.x;
	  ms->config.currentEEPose.py = thisIkRequest.request.pose_stamp[0].pose.position.y;
	  ms->config.currentEEPose.pz = thisIkRequest.request.pose_stamp[0].pose.position.z;
	} else {
	  ikResultFailed = 1;
	  cout << "ik result was reported as colliding and we are sensibly rejecting it..." << endl;
	}
      } else {
	ikResultFailed = 1;
	cout << "Initial IK result appears to be truly invalid, incorrect joint field." << endl;
      }

      if (!ikResultFailed) {
	break;
      }

      ROS_WARN_STREAM("Initial IK result invalid... adding noise and retrying.");
      cout << thisIkRequest.request.pose_stamp[0].pose << endl;

      //eePose noisedCurrentEEPose = ms->config.currentEEPose;
      //noisedCurrentEEPose.px = ms->config.currentEEPose.px + (drand48() - 0.5)*2.0*ikNoiseAmplitude*(1.0-useZOnly);
      //noisedCurrentEEPose.py = ms->config.currentEEPose.py + (drand48() - 0.5)*2.0*ikNoiseAmplitude*(1.0-useZOnly);
      //noisedCurrentEEPose.pz = ms->config.currentEEPose.pz + (drand48() - 0.5)*2.0*ikNoiseAmplitude*useZOnly;

      //noisedCurrentEEPose.qx = ms->config.currentEEPose.qx + (drand48() - 0.5)*2.0*ikNoiseAmplitudeQuat;
      //noisedCurrentEEPose.qy = ms->config.currentEEPose.qy + (drand48() - 0.5)*2.0*ikNoiseAmplitudeQuat;
      //noisedCurrentEEPose.qz = ms->config.currentEEPose.qz + (drand48() - 0.5)*2.0*ikNoiseAmplitudeQuat;
      //noisedCurrentEEPose.qw = ms->config.currentEEPose.qw + (drand48() - 0.5)*2.0*ikNoiseAmplitudeQuat;
      //fillIkRequest(&noisedCurrentEEPose, &thisIkRequest);

      reseedIkRequest(ms, &ms->config.currentEEPose, &thisIkRequest, ikRetry, numIkRetries);
      fillIkRequest(&ms->config.currentEEPose, &thisIkRequest);
    }
  }

  /*
    if ( ms->config.ikClient.waitForExistence(ros::Duration(1, 0)) ) {
  //cout << "block6.1" << endl;
      ikResultFailed = (!ms->config.ikClient.call(thisIkRequest) || !thisIkRequest.response.isValid[0]);
    } else {
      cout << "waitForExistence timed out" << endl;
      ikResultFailed = 1;
    }
  */


    if (ikResultFailed) 
    {
      ROS_ERROR_STREAM("ikClient says pose request is invalid.");
      ms->config.ik_reset_counter++;

      cout << "ik_reset_counter, ik_reset_thresh: " << ms->config.ik_reset_counter << " " << ms->config.ik_reset_thresh << endl;
      if (ms->config.ik_reset_counter > ms->config.ik_reset_thresh) {
	ms->config.ik_reset_counter = 0;
	ms->config.currentEEPose = ms->config.ik_reset_eePose;
	ms->pushWord("pauseStackExecution"); // pause stack execution
	ms->pushCopies("beep", 15); // beep
	cout << "target position denied by ik, please reset the object.";
      }
      else {
	cout << "This pose was rejected by ikClient:" << endl;
	cout << "Current EE Position (x,y,z): " << ms->config.currentEEPose.px << " " << ms->config.currentEEPose.py << " " << ms->config.currentEEPose.pz << endl;
	cout << "Current EE Orientation (x,y,z,w): " << ms->config.currentEEPose.qx << " " << ms->config.currentEEPose.qy << " " << ms->config.currentEEPose.qz << " " << ms->config.currentEEPose.qw << endl;

	ms->config.currentEEPose = ms->config.lastGoodEEPose;
      }

      return;
    }

    ms->config.ik_reset_counter = max(ms->config.ik_reset_counter-1, 0);

    ms->config.lastGoodEEPose = ms->config.currentEEPose;
    ms->config.ikRequest = thisIkRequest;
    ms->config.ikInitialized = 1;
  

  // but in theory we can bypass the joint controllers by publishing to this topic
  // /robot/limb/left/joint_command

  baxter_core_msgs::JointCommand myCommand;

  if (!ms->config.jointNamesInit) {
    ms->config.jointNames.resize(NUM_JOINTS);
    for (int j = 0; j < NUM_JOINTS; j++) {
      ms->config.jointNames[j] = ms->config.ikRequest.response.joints[0].name[j];
    }
    ms->config.jointNamesInit = 1;
  }

  if (ms->config.driveVelocities) {

    double l2Gravity = 0.0;

    myCommand.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;
    myCommand.command.resize(NUM_JOINTS);
    myCommand.names.resize(NUM_JOINTS);

    ros::Time theNow = ros::Time::now();
    ros::Duration howLong = theNow - ms->config.oscilStart;
    double spiralEta = 1.25;
    double rapidJointGlobalOmega[NUM_JOINTS] = {4, 0, 0, 4, 4, 4, 4};
    double rapidJointLocalOmega[NUM_JOINTS] = {.2, 0, 0, 2, 2, .2, 2};
    double rapidJointLocalBias[NUM_JOINTS] = {0, 0, 0, 0.7, 0, 0, 0};
    int rapidJointMask[NUM_JOINTS] = {1, 0, 0, 1, 1, 1, 1};
    double rapidJointScales[NUM_JOINTS] = {.10, 0, 0, 1.0, 2.0, .20, 3.1415926};


    for (int j = 0; j < NUM_JOINTS; j++) {
      myCommand.names[j] = ms->config.ikRequest.response.joints[0].name[j];
      //myCommand.command[j] = 0.0;
      myCommand.command[j] = spiralEta*rapidJointScales[j]*(ms->config.ikRequest.response.joints[0].position[j] - ms->config.trueJointPositions[j]);
      //myCommand.command[j] = sin(rapidJointGlobalOmega[j]*howLong.toSec());
    }
    {
      double tim = howLong.toSec();
      double rapidAmp1 = 0.00; //0.3 is great
      myCommand.command[4] += -rapidAmp1*rapidJointScales[4]*sin(rapidJointLocalBias[4] + (rapidJointLocalOmega[4]*rapidJointGlobalOmega[4]*tim));
      myCommand.command[3] +=  rapidAmp1*rapidJointScales[3]*cos(rapidJointLocalBias[3] + (rapidJointLocalOmega[3]*rapidJointGlobalOmega[3]*tim));

      //myCommand.command[5] += -rapidAmp1*rapidJointScales[4]*sin(rapidJointLocalBias[4] + (rapidJointLocalOmega[4]*rapidJointGlobalOmega[4]*tim));
      //myCommand.command[0] +=  rapidAmp1*rapidJointScales[3]*cos(rapidJointLocalBias[3] + (rapidJointLocalOmega[3]*rapidJointGlobalOmega[3]*tim));

      double rapidAmp2 = 0.00;
      myCommand.command[5] += -rapidAmp2*rapidJointScales[5]*sin(rapidJointLocalBias[5] + (rapidJointLocalOmega[5]*rapidJointGlobalOmega[5]*tim));
      myCommand.command[0] +=  rapidAmp2*rapidJointScales[0]*cos(rapidJointLocalBias[0] + (rapidJointLocalOmega[0]*rapidJointGlobalOmega[0]*tim));

      //myCommand.command[6] +=  rapidAmp1*rapidJointScales[6]*2.0*((0 < cos(0.1*3.1415926*tim))-0.5);
    }
  } else {
    myCommand.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    myCommand.command.resize(NUM_JOINTS);
    myCommand.names.resize(NUM_JOINTS);
    ms->config.lastGoodIkRequest.response.joints.resize(1);
    ms->config.lastGoodIkRequest.response.joints[0].name.resize(NUM_JOINTS);
    ms->config.lastGoodIkRequest.response.joints[0].position.resize(NUM_JOINTS);

    for (int j = 0; j < NUM_JOINTS; j++) {
      myCommand.names[j] = ms->config.ikRequest.response.joints[0].name[j];
      myCommand.command[j] = ms->config.ikRequest.response.joints[0].position[j];
      ms->config.lastGoodIkRequest.response.joints[0].name[j] = ms->config.ikRequest.response.joints[0].name[j];
      ms->config.lastGoodIkRequest.response.joints[0].position[j] = ms->config.ikRequest.response.joints[0].position[j];
    }
    ms->config.goodIkInitialized = 1;
  }

  std_msgs::Float64 speedCommand;
  speedCommand.data = ms->config.currentEESpeedRatio;
  int param_resend_times = 1;
  for (int r = 0; r < param_resend_times; r++) {
    ms->config.joint_mover.publish(myCommand);
    ms->config.moveSpeedPub.publish(speedCommand);
  }

  ms->config.bfc++;
}







void timercallback1(const ros::TimerEvent&) {

  ros::NodeHandle n("~");

  shared_ptr<MachineState> ms = pMachineState;


  int c = -1;
  if (ms->config.shouldIMiscCallback) {
    c = cvWaitKey(1);
  } else if ((ms->config.heartBeatCounter % ms->config.heartBeatPeriod) == 0) {
    c = cvWaitKey(1);
    ms->config.heartBeatCounter = 0;
  }
  ms->config.heartBeatCounter++;
  // XXX is heartBeatCounter even used?

  if (c != -1) {
    // don't print for capslock, shift, alt (for alt-tab)
    if (!(c == 65509 || c == 196581 || c == 196577 || c == 65505 ||
          c == 65513 || c == 196578)) {
      cout << "You pressed " << c << "." << endl;

      if (character_code_to_word.count(c) > 0) {
        shared_ptr<Word> keycode_word = character_code_to_word[c];
        ms->execute(keycode_word);

      } else {
        cout  << "Could not find word for " << c << endl;
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
      if (ms->call_stack.size() > 0 && 
	  !ms->call_stack[ms->call_stack.size() - 1]->is_value()) {
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

    if (ms->config.endThisStackCollapse || (ms->call_stack.size() == 0)) {
      break;
    }
    
  }

  {
    EinState state;
    fillEinStateMsg(ms, &state);
    ms->config.einPub.publish(state);
  }

  endEffectorAngularUpdate(&ms->config.currentEEPose, &ms->config.currentEEDeltaRPY);

  if (!ms->config.zero_g_toggle) {
    update_baxter(n);
  }
  else {
    ms->config.currentEEPose.px = ms->config.trueEEPose.position.x;
    ms->config.currentEEPose.py = ms->config.trueEEPose.position.y;
    ms->config.currentEEPose.pz = ms->config.trueEEPose.position.z;
    ms->config.currentEEPose.qx = ms->config.trueEEPose.orientation.x;
    ms->config.currentEEPose.qy = ms->config.trueEEPose.orientation.y;
    ms->config.currentEEPose.qz = ms->config.trueEEPose.orientation.z;
    ms->config.currentEEPose.qw = ms->config.trueEEPose.orientation.w;
  }

  if (ms->config.sirCore) {
    renderCoreView(ms, ms->config.coreViewName);
  }
  renderRangeogramView(ms);

  if (ms->config.shouldIRender) {
    renderObjectMapView(ms);
  }
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg){

  shared_ptr<MachineState> ms = pMachineState;
  ms->config.lastImageCallbackReceived = ros::Time::now();


  if (!ms->config.shouldIImageCallback) {
    return;
  }

  if (!ms->config.renderInit) {
    ms->config.renderInit = 1;
    ms->config.shouldIRender = ms->config.shouldIRenderDefault;

    try{
      ms->config.cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      ms->config.cam_img = ms->config.cv_ptr->image.clone();
    }catch(cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    ms->config.wristCamImage = ms->config.cv_ptr->image.clone();
    ms->config.wristCamInit = 1;
    ms->config.wristViewImage = ms->config.cv_ptr->image.clone();
    ms->config.faceViewImage = ms->config.cv_ptr->image.clone();

    ms->config.accumulatedImage = Mat(ms->config.cv_ptr->image.rows, ms->config.cv_ptr->image.cols, CV_64FC3);
    ms->config.accumulatedImageMass = Mat(ms->config.cv_ptr->image.rows, ms->config.cv_ptr->image.cols, CV_64F);

    ms->config.densityViewerImage = ms->config.cv_ptr->image.clone();
    ms->config.densityViewerImage *= 0;
    ms->config.gradientViewerImage = Mat(2*ms->config.cv_ptr->image.rows, ms->config.cv_ptr->image.cols, ms->config.cv_ptr->image.type());
    ms->config.aerialGradientViewerImage = Mat(4*ms->config.aerialGradientWidth, ms->config.aerialGradientWidth, CV_64F);
    ms->config.objectViewerImage = ms->config.cv_ptr->image.clone();
  }

  try{
    ms->config.cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    ms->config.cam_img = ms->config.cv_ptr->image.clone();
  }catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  ms->config.wristCamImage = ms->config.cv_ptr->image.clone();
  ms->config.wristCamInit = 1;
  ms->config.wristViewImage = ms->config.cv_ptr->image.clone();
  ms->config.faceViewImage = ms->config.cv_ptr->image.clone();


  guardViewers(ms);

  Size sz = ms->config.accumulatedImage.size();
  int imW = sz.width;
  int imH = sz.height;

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      ms->config.accumulatedImage.at<Vec3d>(y,x)[0] = ms->config.accumulatedImage.at<Vec3d>(y,x)[0] + ms->config.wristCamImage.at<Vec3b>(y,x)[0];
      ms->config.accumulatedImage.at<Vec3d>(y,x)[1] = ms->config.accumulatedImage.at<Vec3d>(y,x)[1] + ms->config.wristCamImage.at<Vec3b>(y,x)[1];
      ms->config.accumulatedImage.at<Vec3d>(y,x)[2] = ms->config.accumulatedImage.at<Vec3d>(y,x)[2] + ms->config.wristCamImage.at<Vec3b>(y,x)[2];
      ms->config.accumulatedImageMass.at<double>(y,x) += 1.0;
    }
  }

  setRingImageAtTime(ms, msg->header.stamp, ms->config.wristCamImage);
  Mat thisImage;
  int weHaveImData = getRingImageAtTime(ms, msg->header.stamp, thisImage);

  //if (ms->config.recordRangeMap) 
  recordReadyRangeReadings(ms);

  // publish volumetric representation to a marker array
  {
    int aI = 0;
    int vmSubsampleStride = 10;
    visualization_msgs::MarkerArray ma_to_send; 
    for (int pz = 0; pz < ms->config.vmWidth; pz+=vmSubsampleStride) {
      for (int py = 0; py < ms->config.vmWidth; py+=vmSubsampleStride) {
	for (int px = 0; px < ms->config.vmWidth; px+=vmSubsampleStride) {
	  if (ms->config.volumeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] > 0) {
	    aI++;
	  }
	}
      }
    }
    int numCubesToShow = aI;

    /*
    ma_to_send.markers.resize(aI);
    aI = 0;
    for (int pz = 0; pz < ms->config.vmWidth; pz+=vmSubsampleStride) {
      for (int py = 0; py < ms->config.vmWidth; py+=vmSubsampleStride) {
	for (int px = 0; px < ms->config.vmWidth; px+=vmSubsampleStride) {
	  if (ms->config.volumeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] > 0) {
	    double denomC = max(ms->config.vmColorRangeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth], EPSILON);
	    int tRed = min(255, max(0,int(round(ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 2*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] / denomC))));
	    int tGreen = min(255, max(0,int(round(ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 1*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] / denomC))));
	    int tBlue = min(255, max(0,int(round(ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 0*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] / denomC))));

//cout << tBlue << " " << ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 0*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] << " " <<  denomC << endl;
	    ma_to_send.markers[aI].pose.position.x = rmcX + (px - ms->config.vmHalfWidth)*ms->config.vmDelta;
	    ma_to_send.markers[aI].pose.position.y = ms->config.rmcY + (py - ms->config.vmHalfWidth)*ms->config.vmDelta;
	    ma_to_send.markers[aI].pose.position.z = ms->config.rmcZ + (pz - ms->config.vmHalfWidth)*ms->config.vmDelta;
	    ma_to_send.markers[aI].pose.orientation.w = 1.0;
	    ma_to_send.markers[aI].pose.orientation.x = 0.0;
	    ma_to_send.markers[aI].pose.orientation.y = 0.0;
	    ma_to_send.markers[aI].pose.orientation.z = 0.0;
	    ma_to_send.markers[aI].type =  visualization_msgs::Marker::CUBE;
	    ma_to_send.markers[aI].scale.x = ms->config.vmDelta*vmSubsampleStride;
	    ma_to_send.markers[aI].scale.y = ms->config.vmDelta*vmSubsampleStride;
	    ma_to_send.markers[aI].scale.z = ms->config.vmDelta*vmSubsampleStride;
	    ma_to_send.markers[aI].color.a = volumeMap[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth];
	    ma_to_send.markers[aI].color.r = double(tRed)/255.0;
	    ma_to_send.markers[aI].color.g = double(tGreen)/255.0;
	    ma_to_send.markers[aI].color.b = double(tBlue)/255.0;

	    ma_to_send.markers[aI].header.stamp = ros::Time::now();
	    ma_to_send.markers[aI].header.frame_id = "/base";
	    ma_to_send.markers[aI].action = visualization_msgs::Marker::ADD;
	    ma_to_send.markers[aI].id = aI;
	    ma_to_send.markers[aI].lifetime = ros::Duration(1.0);

	    aI++;
	  }
	}
      }
    }
    */

    ma_to_send.markers.resize(1);

    ma_to_send.markers[0].pose.orientation.w = 1.0;
    ma_to_send.markers[0].pose.orientation.x = 0.0;
    ma_to_send.markers[0].pose.orientation.y = 0.0;
    ma_to_send.markers[0].pose.orientation.z = 0.0;
    ma_to_send.markers[0].type =  visualization_msgs::Marker::CUBE_LIST;
    ma_to_send.markers[0].scale.x = ms->config.vmDelta*vmSubsampleStride;
    ma_to_send.markers[0].scale.y = ms->config.vmDelta*vmSubsampleStride;
    ma_to_send.markers[0].scale.z = ms->config.vmDelta*vmSubsampleStride;

    ma_to_send.markers[0].header.stamp = ros::Time::now();
    ma_to_send.markers[0].header.frame_id = "/base";
    ma_to_send.markers[0].action = visualization_msgs::Marker::ADD;
    ma_to_send.markers[0].id = 0;
    ma_to_send.markers[0].lifetime = ros::Duration(1.0);

    double volumeRenderThresh = 0.333;

    for (int pz = 0; pz < ms->config.vmWidth; pz+=vmSubsampleStride) {
      for (int py = 0; py < ms->config.vmWidth; py+=vmSubsampleStride) {
	for (int px = 0; px < ms->config.vmWidth; px+=vmSubsampleStride) {
	  if (ms->config.volumeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] > 0) {
	    double denomC = max(ms->config.vmColorRangeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth], EPSILON);
	    int tRed = min(255, max(0,int(round(ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 2*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] / denomC))));
	    int tGreen = min(255, max(0,int(round(ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 1*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] / denomC))));
	    int tBlue = min(255, max(0,int(round(ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + 0*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] / denomC))));

	    std_msgs::ColorRGBA p;
	    //p.a = volumeMap[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] > 0;
	    //p.a = volumeMap[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] > 0.5;
	    p.a = ms->config.volumeMap[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] > volumeRenderThresh;
	    //p.a = 4.0*volumeMap[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth];
	    p.r = double(tRed)/255.0;
	    p.g = double(tGreen)/255.0;
	    p.b = double(tBlue)/255.0;
	    ma_to_send.markers[0].colors.push_back(p);

	    geometry_msgs::Point temp;
	    temp.x = ms->config.rmcX + (px - ms->config.vmHalfWidth)*ms->config.vmDelta;
	    temp.y = ms->config.rmcY + (py - ms->config.vmHalfWidth)*ms->config.vmDelta;
	    temp.z = ms->config.rmcZ + (pz - ms->config.vmHalfWidth)*ms->config.vmDelta;
	    ma_to_send.markers[0].points.push_back(temp);
	  }
	}
      }
    }
    ms->config.vmMarkerPublisher.publish(ma_to_send);
  }

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
    eePose teePose;
    teePose.px = ms->config.trueEEPose.position.x;
    teePose.py = ms->config.trueEEPose.position.y;
    teePose.pz = ms->config.trueEEPose.position.z;
    paintEEPoseOnWrist(ms, teePose, cv::Scalar(0,0,255));
    paintEEPoseOnWrist(ms, ms->config.eepReg1, cv::Scalar(0,255,0));

    {
      eePose irPose;
      {
	Eigen::Quaternionf crane2quat(ms->config.straightDown.qw, ms->config.straightDown.qx, ms->config.straightDown.qy, ms->config.straightDown.qz);
	ms->config.irGlobalPositionEEFrame = crane2quat.conjugate() * ms->config.gear0offset * crane2quat;
	geometry_msgs::Pose thisPose = ms->config.trueEEPose;
	Eigen::Quaternionf ceeQuat(thisPose.orientation.w, thisPose.orientation.x, thisPose.orientation.y, thisPose.orientation.z);
	Eigen::Quaternionf irSensorStartLocal = ceeQuat * ms->config.irGlobalPositionEEFrame * ceeQuat.conjugate();
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
      paintEEPoseOnWrist(ms, irPose, cv::Scalar(255,0,0));
    }
  }

  // draw vanishing point reticle
  {
    int vanishingPointReticleRadius = 15;

    int x0 = ms->config.vanishingPointReticle.px;
    int y0 = ms->config.vanishingPointReticle.py;
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
    int x0 = ms->config.vanishingPointReticle.px;
    int y0 = ms->config.vanishingPointReticle.py+3*movementIndicatorOuterHalfWidth;
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
    int probeReticleHalfWidth = 7;
    int x0 = ms->config.probeReticle.px;
    int y0 = ms->config.probeReticle.py;

    int x1 = max(int(ms->config.probeReticle.px-probeReticleHalfWidth), 0);
    int x2 = min(int(ms->config.probeReticle.px+probeReticleHalfWidth), ms->config.wristViewImage.cols);
    int y1 = max(int(ms->config.probeReticle.py-probeReticleHalfWidth), 0);
    int y2 = min(int(ms->config.probeReticle.py+probeReticleHalfWidth), ms->config.wristViewImage.rows);

    int probeReticleShortHalfWidth = 3;
    int x1s = max(int(ms->config.probeReticle.px-probeReticleShortHalfWidth), 0);
    int x2s = min(int(ms->config.probeReticle.px+probeReticleShortHalfWidth), ms->config.wristViewImage.cols);
    int y1s = max(int(ms->config.probeReticle.py-probeReticleShortHalfWidth), 0);
    int y2s = min(int(ms->config.probeReticle.py+probeReticleShortHalfWidth), ms->config.wristViewImage.rows);

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
  {
    for (int cr = 0; cr < ms->config.numCReticleIndeces; cr++) {
      cv::Point outTop = cv::Point(ms->config.xCR[cr]-3, ms->config.yCR[cr]-3);
      cv::Point outBot = cv::Point(ms->config.xCR[cr]+3, ms->config.yCR[cr]+3);
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
  }

  // ATTN 16
  if (1) {
    for (int hri = 0; hri < 4; hri++) {
      if (hri != ms->config.currentThompsonHeightIdx)
	continue;
      eePose thisReticle = ms->config.heightReticles[hri];
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

  if (ms->config.shouldIRender) {
    guardedImshow(ms->config.wristViewName, ms->config.wristViewImage, ms->config.sirWrist);
  }
}

cv::Point worldToPixel(Mat mapImage, double xMin, double xMax, double yMin, double yMax, double x, double y) {
  double pxMin = 0;
  double pxMax = mapImage.cols;
  double pyMin = 0;
  double pyMax = mapImage.rows;
  cv::Point center = cv::Point(pxMax/2, pyMax/2);

  cv::Point out = cv::Point((pyMax - pyMin) / (yMax - yMin) * y + center.y,
                            (pxMax - pxMin) / (xMax - xMin) * x + center.x);
  return out;
}

void renderObjectMapView(shared_ptr<MachineState> ms) {
  if (ms->config.objectMapViewerImage.rows <= 0 ) {
    ms->config.objectMapViewerImage = Mat(800, 800, CV_8UC3);
  }

  if (0) { // drawGrid
    for (int i = 0; i < ms->config.mapWidth; i++) {
      for (int j = 0; j < ms->config.mapHeight; j++) {
        gsl_matrix * mapcell = mapCellToPolygon(ms, i, j);
        drawMapPolygon(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
                       mapcell, CV_RGB(0, 255, 0));
        gsl_matrix_free(mapcell);
      }
    }
  }

  ms->config.objectMapViewerImage = CV_RGB(0, 0, 0);
  double pxMin = 0;
  double pxMax = ms->config.objectMapViewerImage.cols;
  double pyMin = 0;
  double pyMax = ms->config.objectMapViewerImage.rows;

  cv::Point center = cv::Point(pxMax/2, pyMax/2);

  double fadeBias = 0.50;
  double fadeLast = 30.0;

  for (int i = 0; i < ms->config.mapWidth; i++) {
    for (int j = 0; j < ms->config.mapHeight; j++) {

      ros::Duration longAgo = ros::Time::now() - ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime;
      double fadeFraction = (1.0-fadeBias)*(1.0-(min(max(longAgo.toSec(), 0.0), fadeLast) / fadeLast)) + fadeBias;
      fadeFraction = min(max(fadeFraction, 0.0), 1.0);
      if (!ms->config.useGlow) {
	fadeFraction = 1.0;
      }
      if (!ms->config.useFade) {
	fadeFraction = 1.0;
      }

      double x, y;
      mapijToxy(ms, i, j, &x, &y);

      if (ms->config.objectMap[i + ms->config.mapWidth * j].detectedClass != -1) {
        cv::Point outTop = worldToPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, x, y);
        cv::Point outBot = worldToPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, x + ms->config.mapStep, y + ms->config.mapStep);
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

  if (ms->config.drawIKMap) { // draw ikMap
    int ikMapRenderStride = 1;
    for (int i = 0; i < ms->config.mapWidth; i+=ikMapRenderStride) {
      for (int j = 0; j < ms->config.mapHeight; j+=ikMapRenderStride) {
	if ( cellIsSearched(ms, i, j) ) {
	    ros::Duration longAgo = ros::Time::now() - ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime;
	    double glowFraction = (1.0-glowBias)*(1.0-(min(max(longAgo.toSec(), 0.0), glowLast) / glowLast)) + glowBias;
	    glowFraction = min(max(glowFraction, 0.0), 1.0);
	    if (!ms->config.useGlow) {
	      glowFraction = 1.0;
	    }
	    double x=-1, y=-1;
	    mapijToxy(ms, i, j, &x, &y);
	    cv::Point cvp1 = worldToPixel(ms->config.objectMapViewerImage, 
	      ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, x, y);
	    if ( (ms->config.ikMap[i + ms->config.mapWidth * j] == 1) ) {
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
	    } else if ( (ms->config.ikMap[i + ms->config.mapWidth * j] == 2) ) {
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
	    }
	}
      }
    }
  }

  if (ms->config.drawClearanceMap) { // draw clearanceMap 
    int ikMapRenderStride = 1;
    for (int i = 0; i < ms->config.mapWidth; i+=ikMapRenderStride) {
      for (int j = 0; j < ms->config.mapHeight; j+=ikMapRenderStride) {
	if ( cellIsSearched(ms, i, j) ) {
	    ros::Duration longAgo = ros::Time::now() - ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime;
	    double glowFraction = (1.0-glowBias)*(1.0-(min(max(longAgo.toSec(), 0.0), glowLast) / glowLast)) + glowBias;
	    if (!ms->config.useGlow) {
	      glowFraction = 1.0;
	    }
	    double x=-1, y=-1;
	    mapijToxy(ms, i, j, &x, &y);
	    cv::Point cvp1 = worldToPixel(ms->config.objectMapViewerImage, 
	      ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, x, y);
	    if ( (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 1) ) {
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
	    }
	}
      }
    }
  }

  { // drawMapSearchFence
    
    cv::Point outTop = worldToPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
                                    ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceYMin);
    cv::Point outBot = worldToPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
                                    ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMax);

    rectangle(ms->config.objectMapViewerImage, outTop, outBot, 
              CV_RGB(255, 255, 0));
  }

  { // drawMapRejectFence
    
    cv::Point outTop = worldToPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
                                    ms->config.mapRejectFenceXMin, ms->config.mapRejectFenceYMin);
    cv::Point outBot = worldToPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
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
      
      cv::Point objectPoint = worldToPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
					   cx, cy);
      objectPoint.x += 15;

      cv::Point outTop = worldToPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
				      sprite.top.px, sprite.top.py);
      cv::Point outBot = worldToPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
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
    
    cv::Point objectPoint = worldToPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
                                         cx, cy);
    objectPoint.x += 15;

    cv::Point outTop = worldToPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
                                    memory.top.px, memory.top.py);
    cv::Point outBot = worldToPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
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
      ros::Duration timeSince = ros::Time::now() - ros::Time(0);
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
  
  { // drawRobot
    double radius = 20;
    cv::Point orientation_point = cv::Point(pxMax/2, pyMax/2 + radius);
    
    circle(ms->config.objectMapViewerImage, center, radius, cv::Scalar(0, 0, 255));
    line(ms->config.objectMapViewerImage, center, orientation_point, cv::Scalar(0, 0, 255));
  }
  { // drawHand
    eePose tp = rosPoseToEEPose(ms->config.trueEEPose);
    double radius = 10;
    cv::Point handPoint = worldToPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax, 
                                       tp.px, tp.py);
    
    Eigen::Quaternionf handQuat(tp.qw, tp.qx, tp.qy, tp.qz);

    double rotated_magnitude = radius / sqrt(pow(pxMax - pxMin, 2) +  pow(pyMax - pyMin, 2)) * sqrt(pow(ms->config.mapXMax - ms->config.mapXMin, 2) + pow(ms->config.mapYMax - ms->config.mapYMin, 2));
    Eigen::Vector3f point(rotated_magnitude, 0, 0);
    Eigen::Vector3f rotated = handQuat * point;
    
    cv::Point orientation_point = worldToPixel(ms->config.objectMapViewerImage, ms->config.mapXMin, ms->config.mapXMax, ms->config.mapYMin, ms->config.mapYMax,
                                               tp.px + rotated[0], 
                                               tp.py + rotated[1]);


    circle(ms->config.objectMapViewerImage, handPoint, radius, cv::Scalar(0, 0, 255));

    line(ms->config.objectMapViewerImage, handPoint, orientation_point, cv::Scalar(0, 0, 255));

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

  if (ms->config.shouldIRender) {
    guardedImshow(ms->config.objectMapViewerName, ms->config.objectMapViewerImage, ms->config.sirObjectMap);
  }


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
    cv::Point cvp1 = worldToPixel(mapImage, mapXMin, mapXMax, mapYMin, mapYMax, 
                                  x1, y1);
    cv::Point cvp2 = worldToPixel(mapImage, mapXMin, mapXMax, mapYMin, mapYMax, 
                                  x2, y2);
    line(mapImage, cvp1, cvp2, color);
  }

}



void renderRangeogramView(shared_ptr<MachineState> ms) {
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
  guardedImshow(ms->config.rangeogramViewName, ms->config.rangeogramImage, ms->config.sirRangeogram);
}

void targetCallback(const geometry_msgs::Point& point) {

  shared_ptr<MachineState> ms = pMachineState;

  if (!ms->config.shouldIMiscCallback) {
    return;
  }

}

void pilotCallbackFunc(int event, int x, int y, int flags, void* userdata) {

  //if (!ms->config.shouldIMiscCallback) {
    //return;
  //}
  shared_ptr<MachineState> ms = pMachineState;

  if ( event == EVENT_LBUTTONDOWN ) {
    cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    ms->config.probeReticle.px = x;
    ms->config.probeReticle.py = y;
    cout << "x: " << x << " y: " << y << " eeRange: " << ms->config.eeRange << endl;
  } else if ( event == EVENT_RBUTTONDOWN ) {
    //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if  ( event == EVENT_MBUTTONDOWN ) {
    //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
  } else if ( event == EVENT_MOUSEMOVE ) {
    //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
  }
}

void graspMemoryCallbackFunc(int event, int x, int y, int flags, void* userdata) {

  shared_ptr<MachineState> ms = pMachineState;

  if (!ms->config.shouldIMiscCallback) {
    return;
  }

  if ( event == EVENT_LBUTTONDOWN ) {
    int bigX = x / ms->config.rmiCellWidth;
    int bigY = y / ms->config.rmiCellWidth;
    if ((bigX >= ms->config.rmWidth) && (bigX < 2*ms->config.rmWidth) && (bigY < ms->config.rmWidth)) {
      // weight the grasp at a single point
      ms->config.gmTargetY = (bigX-ms->config.rmWidth);
      ms->config.gmTargetX = bigY;

      // ATTN 5
      // XXX no check
//      for (int delX = -1; delX <= 1; delX++) {
//	for (int delY = -1; delY <= 1; delY++) {
//	  ms->config.graspMemoryTries[(ms->config.gmTargetX+delX) + (ms->config.gmTargetY+delY)*ms->config.rmWidth] = 1;
//	  ms->config.graspMemoryPicks[(ms->config.gmTargetX+delX) + (ms->config.gmTargetY+delY)*ms->config.rmWidth] = 1;
//	}
//      }
      ms->config.graspMemoryTries[ms->config.gmTargetX + ms->config.gmTargetY*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*ms->config.currentGraspGear] += 1;
      ms->config.graspMemoryPicks[ms->config.gmTargetX + ms->config.gmTargetY*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*ms->config.currentGraspGear] += 1;
    }
    ms->pushWord("paintReticles"); // render reticle
    ms->pushWord("drawMapRegisters"); // render register 1
    ms->execute_stack = 1;

    cout << "Grasp Memory Left Click x: " << x << " y: " << y << " eeRange: " << ms->config.eeRange << 
      " bigX: " << bigX << " bigY: " << bigY << " gmTargetX gmTargetY: " << ms->config.gmTargetX << " " << ms->config.gmTargetY << endl;
  } else if ( event == EVENT_RBUTTONDOWN ) {
    int bigX = x / ms->config.rmiCellWidth;
    int bigY = y / ms->config.rmiCellWidth;
    if ((bigX >= ms->config.rmWidth) && (bigX < 2*ms->config.rmWidth) && (bigY < ms->config.rmWidth)) {
      ms->config.graspMemoryTries[ms->config.gmTargetX + ms->config.gmTargetY*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*ms->config.currentGraspGear] += 1;
    }
    ms->pushWord("paintReticles"); // render reticle
    ms->pushWord("drawMapRegisters"); // render register 1
    ms->execute_stack = 1;

    cout << "Grasp Memory Left Click x: " << x << " y: " << y << " eeRange: " << ms->config.eeRange << 
      " bigX: " << bigX << " bigY: " << bigY << " gmTargetX gmTargetY: " << ms->config.gmTargetX << " " << ms->config.gmTargetY << endl;
  } else if  ( event == EVENT_MBUTTONDOWN ) {
    int bigX = x / ms->config.rmiCellWidth;
    int bigY = y / ms->config.rmiCellWidth;
    if ((bigX >= ms->config.rmWidth) && (bigX < 2*ms->config.rmWidth) && (bigY < ms->config.rmWidth)) {
      // reset to uniform failure
      for (int rx = 0; rx < ms->config.rmWidth; rx++) {
	for (int ry = 0; ry < ms->config.rmWidth; ry++) {
	  ms->config.graspMemoryTries[rx + ry*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*ms->config.currentGraspGear] = 10;
	  ms->config.graspMemoryPicks[rx + ry*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*ms->config.currentGraspGear] = 0;
	}
      }
    }
    ms->pushWord("paintReticles"); // render reticle
    ms->pushWord("drawMapRegisters"); // render register 1
    ms->execute_stack = 1;

    cout << "Grasp Memory Left Click x: " << x << " y: " << y << " eeRange: " << ms->config.eeRange << 
      " bigX: " << bigX << " bigY: " << bigY << " gmTargetX gmTargetY: " << ms->config.gmTargetX << " " << ms->config.gmTargetY << endl;
  } else if ( event == EVENT_MOUSEMOVE ) {
    //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
  }
}

void loadCalibration(shared_ptr<MachineState> ms, string inFileName) {
  FileStorage fsvI;
  cout << "Reading calibration information from " << inFileName << " ...";
  fsvI.open(inFileName, FileStorage::READ);

  {
    FileNode anode = fsvI["currentTableZ"];
    FileNodeIterator it = anode.begin(), it_end = anode.end();
    ms->config.currentTableZ = *(it++);
  }

  {
    FileNode anode = fsvI["cropUpperLeftCorner"];
    FileNodeIterator it = anode.begin(), it_end = anode.end();
    ms->config.cropUpperLeftCorner.px = *(it++);
    ms->config.cropUpperLeftCorner.py = *(it++);
  }

  {
    FileNode anode = fsvI["vanishingPointReticle"];
    FileNodeIterator it = anode.begin(), it_end = anode.end();
    ms->config.vanishingPointReticle.px = *(it++);
    ms->config.vanishingPointReticle.py = *(it++);
  }

  {
    FileNode anode = fsvI["heightReticles"];
    FileNodeIterator it = anode.begin(), it_end = anode.end();
    ms->config.heightReticles[3].px = *(it++);
    ms->config.heightReticles[2].px = *(it++);
    ms->config.heightReticles[1].px = *(it++);
    ms->config.heightReticles[0].px = *(it++);

    ms->config.heightReticles[3].py = *(it++);
    ms->config.heightReticles[2].py = *(it++);
    ms->config.heightReticles[1].py = *(it++);
    ms->config.heightReticles[0].py = *(it++);
  }

  {
    FileNode anode = fsvI["colorReticles"];
    FileNodeIterator it = anode.begin(), it_end = anode.end();
    ms->config.xCR[0]  = *(it++);
    ms->config.xCR[1]  = *(it++);
    ms->config.xCR[2]  = *(it++);
    ms->config.xCR[3]  = *(it++);
    ms->config.xCR[4]  = *(it++);
    ms->config.xCR[5]  = *(it++);
    ms->config.xCR[6]  = *(it++);
    ms->config.xCR[7]  = *(it++);
    ms->config.xCR[8]  = *(it++);
    ms->config.xCR[9]  = *(it++);
    ms->config.xCR[10] = *(it++);
    ms->config.xCR[11] = *(it++);
    ms->config.xCR[12] = *(it++);
    ms->config.xCR[13] = *(it++);

    ms->config.yCR[0]  = *(it++);
    ms->config.yCR[1]  = *(it++);
    ms->config.yCR[2]  = *(it++);
    ms->config.yCR[3]  = *(it++);
    ms->config.yCR[4]  = *(it++);
    ms->config.yCR[5]  = *(it++);
    ms->config.yCR[6]  = *(it++);
    ms->config.yCR[7]  = *(it++);
    ms->config.yCR[8]  = *(it++);
    ms->config.yCR[9]  = *(it++);
    ms->config.yCR[10] = *(it++);
    ms->config.yCR[11] = *(it++);
    ms->config.yCR[12] = *(it++);
    ms->config.yCR[13] = *(it++);
  }

  {
    FileNode anode = fsvI["lensCorrections"];
    FileNodeIterator it = anode.begin(), it_end = anode.end();
    ms->config.m_x_h[0] = *(it++);
    ms->config.m_x_h[1] = *(it++);
    ms->config.m_x_h[2] = *(it++);
    ms->config.m_x_h[3] = *(it++);

    ms->config.m_y_h[0] = *(it++);
    ms->config.m_y_h[1] = *(it++);
    ms->config.m_y_h[2] = *(it++);
    ms->config.m_y_h[3] = *(it++);
  }

  {
    FileNode anode = fsvI["gear0offset"];
    FileNodeIterator it = anode.begin(), it_end = anode.end();
    ms->config.gear0offset.x() = *(it++);
    ms->config.gear0offset.y() = *(it++);
    ms->config.gear0offset.z() = *(it++);
    ms->config.gear0offset.w() = *(it++);
  }

  cout << "done." << endl;
}

void saveCalibration(shared_ptr<MachineState> ms, string outFileName) {

  /* this works
  for (int i = 0; i < 5; i++) {
    char buf[256];
    sprintf(buf, "%d", i);
    string testString(buf);
    testString = "test"+testString;
    fsvO << testString << ms->config.classLabels;
  }
  */

  FileStorage fsvO;
  cout << "Writing calibration information to " << outFileName << " ...";
  fsvO.open(outFileName, FileStorage::WRITE);

  fsvO << "currentTableZ" << "[" 
    << ms->config.currentTableZ 
  << "]";

  fsvO << "cropUpperLeftCorner" << "[" 
    << ms->config.cropUpperLeftCorner.px 
    << ms->config.cropUpperLeftCorner.py 
  << "]";

  fsvO << "vanishingPointReticle" << "[" 
    << ms->config.vanishingPointReticle.px 
    << ms->config.vanishingPointReticle.py 
  << "]";

  fsvO << "heightReticles" << "[" 
    << ms->config.heightReticles[3].px
    << ms->config.heightReticles[2].px
    << ms->config.heightReticles[1].px
    << ms->config.heightReticles[0].px

    << ms->config.heightReticles[3].py
    << ms->config.heightReticles[2].py
    << ms->config.heightReticles[1].py
    << ms->config.heightReticles[0].py
  << "]";

  fsvO << "colorReticles" << "[" 
    << ms->config.xCR[0]
    << ms->config.xCR[1]
    << ms->config.xCR[2]
    << ms->config.xCR[3]
    << ms->config.xCR[4]
    << ms->config.xCR[5]
    << ms->config.xCR[6]
    << ms->config.xCR[7]
    << ms->config.xCR[8]
    << ms->config.xCR[9]
    << ms->config.xCR[10]
    << ms->config.xCR[11]
    << ms->config.xCR[12]
    << ms->config.xCR[13]

    << ms->config.yCR[0] 
    << ms->config.yCR[1] 
    << ms->config.yCR[2] 
    << ms->config.yCR[3] 
    << ms->config.yCR[4] 
    << ms->config.yCR[5] 
    << ms->config.yCR[6] 
    << ms->config.yCR[7] 
    << ms->config.yCR[8] 
    << ms->config.yCR[9] 
    << ms->config.yCR[10]
    << ms->config.yCR[11]
    << ms->config.yCR[12]
    << ms->config.yCR[13]
  << "]";

  fsvO << "lensCorrections" << "[" 
    << ms->config.m_x_h[0]
    << ms->config.m_x_h[1]
    << ms->config.m_x_h[2]
    << ms->config.m_x_h[3]

    << ms->config.m_y_h[0]
    << ms->config.m_y_h[1]
    << ms->config.m_y_h[2]
    << ms->config.m_y_h[3]
  << "]";

  fsvO << "gear0offset" << "["
    << ms->config.gear0offset.x()
    << ms->config.gear0offset.y()
    << ms->config.gear0offset.z()
    << ms->config.gear0offset.w()
  << "]";

  fsvO.release();
  cout << "done." << endl;
}

void pilotInit(shared_ptr<MachineState> ms) {

  if (0 == ms->config.left_or_right_arm.compare("left")) {
    cout << "Possessing left arm..." << endl;
    ms->config.beeHome = {.px = 0.334217, .py = 0.75386, .pz = 0.0362593,
                          .qx = -0.00125253, .qy = 0.999999, .qz = -0.000146851, .qw = 0.000236656};
    
    ms->config.eepReg4 = ms->config.beeHome;
    ms->config.defaultReticle = {.px = 334, .py = 100, .pz = 0.0,
                      .qx = 0.0, .qy = 0.0, .qz = 0.0, .qw = 0.0};
    ms->config.reticle = ms->config.defaultReticle;

    ms->config.crane1 = {.px = -0.0155901, .py = 0.981296, .pz = 0.71078,
		     .qx = 0.709046, .qy = -0.631526, .qz = -0.226613, .qw = -0.216967};

    double ystart = 0.1;
    double yend = 0.7;
    int numposes = 4;
    double ystep = (yend - ystart) / numposes;
    eePose pose1 = {.px = 0.65, .py = 0.0544691, .pz = -0.0582791,
                       .qx = 0, .qy = 1, .qz = 0, .qw = 0};
    for (int i = 0; i < numposes; i++) {
      ms->config.deliveryPoses.push_back(pose1);
    }
    for (int i = 0; i < numposes; i++) {
      ms->config.deliveryPoses[i].py = ystart + i * ystep;
    }

    ms->config.ik_reset_eePose = {.px = 0.334217, .py = 0.75386, .pz = 0.0362593,
                                  .qx = -0.00125253, .qy = 0.999999, .qz = -0.000146851, .qw = 0.000236656};

    ms->config.currentTableZ = ms->config.leftTableZ;
    ms->config.bagTableZ = ms->config.leftTableZ;
    ms->config.counterTableZ = ms->config.leftTableZ;
    ms->config.pantryTableZ  = ms->config.leftTableZ;

    ms->config.eepReg1 = ms->config.beeHome; 
    ms->config.eepReg2 = ms->config.beeHome; 

    ms->config.mapSearchFenceXMin = -0.75;
    ms->config.mapSearchFenceXMax = 1.0;
    ms->config.mapSearchFenceYMin = -1.25;
    ms->config.mapSearchFenceYMax = 1.25;

    ms->config.mapRejectFenceXMin = ms->config.mapSearchFenceXMin;
    ms->config.mapRejectFenceXMax = ms->config.mapSearchFenceXMax;
    ms->config.mapRejectFenceYMin = ms->config.mapSearchFenceYMin;
    ms->config.mapRejectFenceYMax = ms->config.mapSearchFenceYMax;

    ms->config.mapBackgroundXMin = ms->config.mapSearchFenceXMin - ms->config.mapBackgroundBufferMeters;
    ms->config.mapBackgroundXMax = ms->config.mapSearchFenceXMax + ms->config.mapBackgroundBufferMeters;
    ms->config.mapBackgroundYMin = ms->config.mapSearchFenceYMin - ms->config.mapBackgroundBufferMeters;
    ms->config.mapBackgroundYMax = ms->config.mapSearchFenceYMax + ms->config.mapBackgroundBufferMeters;

    // left arm
    // (313, 163)
    ms->config.vanishingPointReticle.px = 313;
    ms->config.vanishingPointReticle.py = 163;
    ms->config.probeReticle = ms->config.vanishingPointReticle;

    // ATTN 16
    ms->config.heightReticles[0] = ms->config.defaultReticle;
    ms->config.heightReticles[1] = ms->config.defaultReticle;
    ms->config.heightReticles[2] = ms->config.defaultReticle;
    ms->config.heightReticles[3] = ms->config.defaultReticle;

    ms->config.heightReticles[3].px = 323;
    ms->config.heightReticles[2].px = 326;
    ms->config.heightReticles[1].px = 329;
    ms->config.heightReticles[0].px = 336;

    ms->config.heightReticles[3].py = 135;
    ms->config.heightReticles[2].py = 128;
    ms->config.heightReticles[1].py = 117;
    ms->config.heightReticles[0].py = 94;

    /* color reticle init */
    /* XXX TODO needs recalibrating */
    //const int ms->config.xCR[ms->config.numCReticleIndeces] = {462, 450, 439, 428, 419, 410, 405, 399, 394, 389, 383, 381, 379, 378};
    ms->config.xCR[0] = 462;
    ms->config.xCR[1] = 450;
    ms->config.xCR[2] = 439;
    ms->config.xCR[3] = 428;
    ms->config.xCR[4] = 419;
    ms->config.xCR[5] = 410;
    ms->config.xCR[6] = 405;
    ms->config.xCR[7] = 399;
    ms->config.xCR[8] = 394;
    ms->config.xCR[9] = 389;
    ms->config.xCR[10] = 383;
    ms->config.xCR[11] = 381;
    ms->config.xCR[12] = 379;
    ms->config.xCR[13] = 378;

    /* left arm */
    //const int ms->config.yCR[ms->config.numCReticleIndeces] = {153, 153, 153, 153, 153, 154, 154, 154, 154, 154, 155, 155, 155, 155};
    ms->config.yCR[0] = 153;
    ms->config.yCR[1] = 153;
    ms->config.yCR[2] = 153;
    ms->config.yCR[3] = 153;
    ms->config.yCR[4] = 153;
    ms->config.yCR[5] = 154;
    ms->config.yCR[6] = 154;
    ms->config.yCR[7] = 154;
    ms->config.yCR[8] = 154;
    ms->config.yCR[9] = 154;
    ms->config.yCR[10] = 155;
    ms->config.yCR[11] = 155;
    ms->config.yCR[12] = 155;
    ms->config.yCR[13] = 155;

    /* lens correction */
    ms->config.m_x_h[0] = 1.2;
    ms->config.m_x_h[1] = 1.06;
    ms->config.m_x_h[2] = 0.98;
    ms->config.m_x_h[3] = 0.94;

    ms->config.m_y_h[0] = 0.95;
    ms->config.m_y_h[1] = 0.93;
    ms->config.m_y_h[2] = 0.92;
    ms->config.m_y_h[3] = 0.92;

    ms->config.handingPose = {.px = 0.955119, .py = 0.0466243, .pz = 0.20442,
                   .qx = 0.538769, .qy = -0.531224, .qz = 0.448211, .qw = -0.476063};
    ms->config.eepReg3 = ms->config.handingPose;

    // ir offset
    ms->config.gear0offset = Eigen::Quaternionf(0.0, 0.03, 0.023, 0.0167228); // z is from TF, good for depth alignment

    ms->config.calibrationPose = {.px = 0.434176, .py = 0.633423, .pz = 0.48341,
                      .qx = 0.000177018, .qy = 1, .qz = -0.000352912, .qw = -0.000489087};
    ms->config.shrugPose = {.px = 0.0354772, .py = 1.20633, .pz = 0.150562,
                 .qx = -0.370521, .qy = 0.381345, .qz = 0.578528, .qw = 0.618544};
  } else if (0 == ms->config.left_or_right_arm.compare("right")) {
    cout << "Possessing right arm..." << endl;

    ms->config.beeHome = {.px = 0.525866, .py = -0.710611, .pz = 0.0695764,
                          .qx = -0.00122177, .qy = 0.999998, .qz = 0.00116169, .qw = -0.001101};

    ms->config.eepReg4 = ms->config.beeHome;
    ms->config.defaultReticle = {.px = 325, .py = 127, .pz = 0.0,
                      .qx = 0.0, .qy = 0.0, .qz = 0.0, .qw = 0.0};
    ms->config.reticle = ms->config.defaultReticle;

    ms->config.crane1 = {.px = 0.0448714, .py = -1.04476, .pz = 0.698522,
              .qx = 0.631511, .qy = 0.68929, .qz = -0.25435, .qw = 0.247748};

    double ystart = -0.7;
    double yend = -0.1;
    int numposes = 4;
    double ystep = (yend - ystart) / numposes;
    eePose pose1 = {.px = 0.65, .py = 0.0544691, .pz = -0.0582791,
                       .qx = 0, .qy = 1, .qz = 0, .qw = 0};
    for (int i = 0; i < numposes; i++) {
      ms->config.deliveryPoses.push_back(pose1);
    }
    for (int i = 0; i < numposes; i++) {
      ms->config.deliveryPoses[i].py = ystart + i * ystep;
    }


    ms->config.ik_reset_eePose = ms->config.beeHome;

    ms->config.currentTableZ = ms->config.rightTableZ;
    ms->config.bagTableZ = ms->config.rightTableZ;
    ms->config.counterTableZ = ms->config.rightTableZ;
    ms->config.pantryTableZ  = ms->config.rightTableZ;


    ms->config.eepReg1 = ms->config.beeHome;
    ms->config.eepReg2 = ms->config.beeHome;

    // raw fence values (from John estimating arm limits)
    // True EE Position (x,y,z): -0.329642 -0.77571 0.419954
    // True EE Position (x,y,z): 0.525236 -0.841226 0.217111

    // full workspace
    ms->config.mapSearchFenceXMin = -0.75;
    ms->config.mapSearchFenceXMax = 1.00;
    ms->config.mapSearchFenceYMin = -1.25;
    ms->config.mapSearchFenceYMax = 1.25;
    ms->config.mapRejectFenceXMin = ms->config.mapSearchFenceXMin;
    ms->config.mapRejectFenceXMax = ms->config.mapSearchFenceXMax;
    ms->config.mapRejectFenceYMin = ms->config.mapSearchFenceYMin;
    ms->config.mapRejectFenceYMax = ms->config.mapSearchFenceYMax;

    ms->config.mapBackgroundXMin = ms->config.mapSearchFenceXMin - ms->config.mapBackgroundBufferMeters;
    ms->config.mapBackgroundXMax = ms->config.mapSearchFenceXMax + ms->config.mapBackgroundBufferMeters;
    ms->config.mapBackgroundYMin = ms->config.mapSearchFenceYMin - ms->config.mapBackgroundBufferMeters;
    ms->config.mapBackgroundYMax = ms->config.mapSearchFenceYMax + ms->config.mapBackgroundBufferMeters;

    // right arm
    ms->config.vanishingPointReticle.px = 313;
    ms->config.vanishingPointReticle.py = 185;
    ms->config.probeReticle = ms->config.vanishingPointReticle;

    // ATTN 16
    ms->config.heightReticles[0] = ms->config.defaultReticle;
    ms->config.heightReticles[1] = ms->config.defaultReticle;
    ms->config.heightReticles[2] = ms->config.defaultReticle;
    ms->config.heightReticles[3] = ms->config.defaultReticle;
    
    ms->config.heightReticles[3].px = 314;
    ms->config.heightReticles[2].px = 317;
    ms->config.heightReticles[1].px = 320;
    ms->config.heightReticles[0].px = 328;

    ms->config.heightReticles[3].py = 154;
    ms->config.heightReticles[2].py = 149;
    ms->config.heightReticles[1].py = 139;
    ms->config.heightReticles[0].py = 120;

    /* color reticle init */
    /* XXX TODO needs recalibrating */
    //const int ms->config.xCR[ms->config.numCReticleIndeces] = {462, 450, 439, 428, 419, 410, 405, 399, 394, 389, 383, 381, 379, 378};
    ms->config.xCR[0] = 462;
    ms->config.xCR[1] = 450;
    ms->config.xCR[2] = 439;
    ms->config.xCR[3] = 428;
    ms->config.xCR[4] = 419;
    ms->config.xCR[5] = 410;
    ms->config.xCR[6] = 405;
    ms->config.xCR[7] = 399;
    ms->config.xCR[8] = 394;
    ms->config.xCR[9] = 389;
    ms->config.xCR[10] = 383;
    ms->config.xCR[11] = 381;
    ms->config.xCR[12] = 379;
    ms->config.xCR[13] = 378;

    /* right arm */
    //const int ms->config.yCR[ms->config.numCReticleIndeces] = {153, 153, 153, 153, 153, 154, 154, 154, 154, 154, 155, 155, 155, 155};
    ms->config.yCR[0] = 153;
    ms->config.yCR[1] = 153;
    ms->config.yCR[2] = 153;
    ms->config.yCR[3] = 153;
    ms->config.yCR[4] = 153;
    ms->config.yCR[5] = 154;
    ms->config.yCR[6] = 154;
    ms->config.yCR[7] = 154;
    ms->config.yCR[8] = 154;
    ms->config.yCR[9] = 154;
    ms->config.yCR[10] = 155;
    ms->config.yCR[11] = 155;
    ms->config.yCR[12] = 155;
    ms->config.yCR[13] = 155;

    /* lens correction */
    ms->config.m_x_h[0] = 1.18;
    ms->config.m_x_h[1] = 1.12;
    ms->config.m_x_h[2] = 1.09;
    ms->config.m_x_h[3] = 1.08;

    ms->config.m_y_h[0] = 1.16;
    ms->config.m_y_h[1] = 1.17;
    ms->config.m_y_h[2] = 1.16;
    ms->config.m_y_h[3] = 1.2;

    ms->config.handingPose = {.px = 0.879307, .py = -0.0239328, .pz = 0.223839,
                      .qx = 0.459157, .qy = 0.527586, .qz = 0.48922, .qw = 0.521049};
    ms->config.eepReg3 = ms->config.handingPose;

    // ir offset
    ms->config.gear0offset = Eigen::Quaternionf(0.0, 0.023, 0.023, 0.0167228); // z is from TF, good for depth alignment

    ms->config.calibrationPose = {.px = 0.562169, .py = -0.348055, .pz = 0.493231,
                                  .qx = 0.00391311, .qy = 0.999992, .qz = -0.00128095, .qw = 8.18951e-05};
    ms->config.shrugPose = {.px = 0.0558937, .py = -1.12849, .pz = 0.132171,
                            .qx = 0.392321, .qy = 0.324823, .qz = -0.555039, .qw = 0.657652};


  } else {
    cout << "Invalid chirality: " << ms->config.left_or_right_arm << ".  Exiting." << endl;
    exit(0);
  }
  ms->config.pilotTarget = ms->config.beeHome;
  ms->config.lastGoodEEPose = ms->config.beeHome;
  ms->config.currentEEPose = ms->config.beeHome;

  for (int r = 0; r < ms->config.totalRangeHistoryLength; r++) {
    ms->config.rangeHistory[r] = 0;
  }

  for (int rx = 0; rx < ms->config.rmWidth; rx++) {
    for (int ry = 0; ry < ms->config.rmWidth; ry++) {
      ms->config.rangeMap[rx + ry*ms->config.rmWidth] = 0;
      ms->config.rangeMapReg1[rx + ry*ms->config.rmWidth] = 0;
      ms->config.rangeMapReg2[rx + ry*ms->config.rmWidth] = 0;
      ms->config.rangeMapMass[rx + ry*ms->config.rmWidth] = 0;
      ms->config.rangeMapAccumulator[rx + ry*ms->config.rmWidth] = 0;

      // ATTN 6 change initialization to determine speed of learning
      for (int tGG = 0; tGG < ms->config.totalGraspGears/2; tGG++) {
	ms->config.graspMemoryTries[rx + ry*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tGG] = 1;
	ms->config.graspMemoryPicks[rx + ry*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tGG] = 1;
      }
    }
  }

  ms->config.rangemapImage = Mat(ms->config.rmiHeight, 3*ms->config.rmiWidth, CV_8UC3);
  ms->config.graspMemoryImage = Mat(ms->config.rmiHeight, 2*ms->config.rmiWidth, CV_8UC3);
  ms->config.graspMemorySampleImage = Mat(2*ms->config.rmiHeight, 2*ms->config.rmiWidth, CV_8UC3);
  ms->config.heightMemorySampleImage = Mat(ms->config.hmiHeight, 2*ms->config.hmiWidth, CV_8UC3);

  for (int rx = 0; rx < ms->config.hrmWidth; rx++) {
    for (int ry = 0; ry < ms->config.hrmWidth; ry++) {
      ms->config.hiRangeMap[rx + ry*ms->config.hrmWidth] = 0;
      ms->config.hiRangeMapReg1[rx + ry*ms->config.hrmWidth] = 0;
      ms->config.hiRangeMapReg2[rx + ry*ms->config.hrmWidth] = 0;
      ms->config.hiRangeMapMass[rx + ry*ms->config.hrmWidth] = 0;
      ms->config.hiRangeMapAccumulator[rx + ry*ms->config.hrmWidth] = 0;
    }
  }
  ms->config.hiRangemapImage = Mat(ms->config.hrmiHeight, 3*ms->config.hrmiWidth, CV_8UC3);

  ms->config.hiColorRangemapImage = Mat(ms->config.hrmiHeight, ms->config.hrmiWidth, CV_8UC3);

  ms->config.rangeogramImage = Mat(ms->config.rggHeight, ms->config.rggWidth, CV_8UC3);

  ms->config.rmcX = 0;
  ms->config.rmcY = 0;
  ms->config.rmcZ = 0;

  for (int g = 0; g < ms->config.totalGraspGears; g++) {
    ms->config.ggX[g] = 0;
    ms->config.ggY[g] = 0;
    ms->config.ggT[g] = double(g)*2.0*3.1415926/double(ms->config.totalGraspGears);
  }
  // old orientation
  //ms->config.ggX[0] =  0.03;
  //ggY[0] =  0.02;
  //ms->config.ggX[1] =  0.04;
  //ms->config.ggY[1] =  0.00;
  //ms->config.ggX[2] =  0.03;
  //ms->config.ggY[2] = -0.02;
  //ms->config.ggX[3] =  0.00;
  //ms->config.ggY[3] = -0.03; //-0.04

  // new orientation
  // verticle calibration
  ms->config.ggX[0] =  0.02;
  ms->config.ggY[0] =  0.02;
  ms->config.ggX[1] =  0.03;
  ms->config.ggY[1] =  0.00;
  ms->config.ggX[2] =  0.02;
  ms->config.ggY[2] = -0.02;
  ms->config.ggX[3] =  0.00;
  ms->config.ggY[3] = -0.03;//-0.03; //-0.04

  ms->config.ggX[4] = -0.02;
  ms->config.ggY[4] = -0.02;
  ms->config.ggX[5] = -0.03;
  ms->config.ggY[5] = -0.00;
  ms->config.ggX[6] = -0.02;
  ms->config.ggY[6] =  0.02;
  ms->config.ggX[7] = -0.00;
  ms->config.ggY[7] =  0.03;//-0.03; //-0.04

  // XXX set this to be arm-generic
  // XXX add symbols to change register sets
  //ms->config.eepReg3 = crane4right;

  initializeParzen(ms);
  //l2NormalizeParzen();
  initialize3DParzen(ms);
  //l2Normalize3DParzen();

  {
    //gear0offset = Eigen::Quaternionf(0.0, 0.023, 0.023, 0.0167228); // z is from TF, good for depth alignment
    //if (0 == ms->config.left_or_right_arm.compare("left")) {
      //ms->config.gear0offset = Eigen::Quaternionf(0.0, 0.03, 0.023, 0.0167228); // z is from TF, good for depth alignment
    //} else if (0 == ms->config.left_or_right_arm.compare("right")) {
      //ms->config.gear0offset = Eigen::Quaternionf(0.0, 0.023, 0.023, 0.0167228); // z is from TF, good for depth alignment
    //}

    // invert the transformation
    Eigen::Quaternionf crane2quat(ms->config.straightDown.qw, ms->config.straightDown.qx, ms->config.straightDown.qy, ms->config.straightDown.qz);
    ms->config.irGlobalPositionEEFrame = crane2quat.conjugate() * ms->config.gear0offset * crane2quat;

    cout << "irGlobalPositionEEFrame w x y z: " << ms->config.irGlobalPositionEEFrame.w() << " " << 
      ms->config.irGlobalPositionEEFrame.x() << " " << ms->config.irGlobalPositionEEFrame.y() << " " << ms->config.irGlobalPositionEEFrame.z() << endl;
  }

  for (int h = 0; h < ms->config.hrmWidth; h++) {
    for (int i = 0; i < ms->config.hrmWidth; i++) {
      ms->config.hiColorRangeMapMass[h + i*ms->config.hrmWidth] = 0;
      for (int j = 0; j < 3; j++) {
	ms->config.hiColorRangeMapAccumulator[h + i*ms->config.hrmWidth + j*ms->config.hrmWidth*ms->config.hrmWidth] = 0;
      }
    }
  }
  
  ms->config.imRingBuffer.resize(ms->config.imRingBufferSize);
  ms->config.epRingBuffer.resize(ms->config.epRingBufferSize);
  ms->config.rgRingBuffer.resize(ms->config.rgRingBufferSize);

  ms->config.imRBTimes.resize(ms->config.imRingBufferSize);
  ms->config.epRBTimes.resize(ms->config.epRingBufferSize);
  ms->config.rgRBTimes.resize(ms->config.rgRingBufferSize);

  for (int pz = 0; pz < ms->config.vmWidth; pz++) {
    for (int py = 0; py < ms->config.vmWidth; py++) {
      for (int px = 0; px < ms->config.vmWidth; px++) {
	ms->config.volumeMap[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] = 0;
	ms->config.volumeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] = 0;
	ms->config.volumeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] = 0;
	ms->config.vmColorRangeMapMass[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth] = 0;
	for (int pc = 0; pc < 3; pc++) {
	  ms->config.vmColorRangeMapAccumulator[px + py*ms->config.vmWidth + pz*ms->config.vmWidth*ms->config.vmWidth + pc*ms->config.vmWidth*ms->config.vmWidth*ms->config.vmWidth] = 0;
	}
      }
    }
  }
}

void spinlessPilotMain(shared_ptr<MachineState> ms) {
  cout << endl << endl << "Pilot main begin..." << endl;
  
  pilotInit(ms);
}

int shouldIPick(shared_ptr<MachineState> ms, int classToPick) {

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

int getLocalGraspGear(shared_ptr<MachineState> ms, int globalGraspGearIn) {
  // ATTN 7
  // diagnostic line
  //Quaternionf eeqform(ms->config.currentEEPose.qw, ms->config.currentEEPose.qx, ms->config.currentEEPose.qy, ms->config.currentEEPose.qz);
  // correct line
  Quaternionf eeqform(ms->config.bestOrientationEEPose.qw, ms->config.bestOrientationEEPose.qx, ms->config.bestOrientationEEPose.qy, ms->config.bestOrientationEEPose.qz);

  Quaternionf gear1Orient = getGGRotation(ms, 0);
  Quaternionf rel = eeqform * gear1Orient.inverse();
  Quaternionf ex(0,1,0,0);
  Quaternionf zee(0,0,0,1);
  

  Quaternionf result = rel * ex * rel.conjugate();
  Quaternionf thumb = rel * zee * rel.conjugate();
  double aY = result.y();
  double aX = result.x();

  // ATTN 1
  // this is here to get the signs right
  aX = -aX;

  //double angle = atan2(aY, aX)*180.0/3.1415926;
  // no degrees here
  // ATTN 22
  //double angle = atan2(aY, aX);
  double angle = vectorArcTan(ms, aY, aX);
  // no inversion necessary
  //angle = -angle;
  
  double deltaGG = floor(angle * ms->config.totalGraspGears / (2.0 * 3.1415926));
  int ggToReturn = (ms->config.totalGraspGears + globalGraspGearIn + int(deltaGG)) % (ms->config.totalGraspGears / 2);

  //cout << "getLocalGraspGear angle deltaGG ggToReturn: " << angle << " " << deltaGG << " " << ggToReturn << endl;

  assert(getGlobalGraspGear(ms, ggToReturn) == globalGraspGearIn);

  return ggToReturn;
}

int getGlobalGraspGear(shared_ptr<MachineState> ms, int localGraspGearIn) {
  // ATTN 7
  // diagnostic line
  //Quaternionf eeqform(ms->config.currentEEPose.qw, ms->config.currentEEPose.qx, ms->config.currentEEPose.qy, ms->config.currentEEPose.qz);
  // correct line
  Quaternionf eeqform(ms->config.bestOrientationEEPose.qw, ms->config.bestOrientationEEPose.qx, ms->config.bestOrientationEEPose.qy, ms->config.bestOrientationEEPose.qz);

  Quaternionf gear1Orient = getGGRotation(ms, 0);
  Quaternionf rel = eeqform * gear1Orient.inverse();
  Quaternionf ex(0,1,0,0);
  Quaternionf zee(0,0,0,1);
  

  Quaternionf result = rel * ex * rel.conjugate();
  Quaternionf thumb = rel * zee * rel.conjugate();
  double aY = result.y();
  double aX = result.x();

  // ATTN 1
  // this is here to get the signs right
  aX = -aX;

  //double angle = atan2(aY, aX)*180.0/3.1415926;
  // no degrees here
  // ATTN 22
  //double angle = atan2(aY, aX);
  double angle = vectorArcTan(ms, aY, aX);
  // inversion to convert to global
  angle = -angle;
  
  double deltaGG = floor(angle * ms->config.totalGraspGears / (2.0 * 3.1415926));
  // we are doing ceiling by taking the floor and then adding one, the inverse of getLocalGraspGear.
  int ggToReturn = (ms->config.totalGraspGears + localGraspGearIn + 1 + int(deltaGG)) % (ms->config.totalGraspGears / 2);

  //assert(getLocalGraspGear(ms, ggToReturn) == localGraspGearIn);

  return ggToReturn;
}

void changeTargetClass(shared_ptr<MachineState> ms, int newTargetClass) {
  ms->config.targetClass = newTargetClass;
  ms->config.focusedClass = ms->config.targetClass;
  ms->config.focusedClassLabel = ms->config.classLabels[ms->config.focusedClass];
  cout << "class " << ms->config.targetClass << " " << ms->config.classLabels[ms->config.targetClass] << endl;
  ms->execute_stack = 1;	


  ms->pushWord("loadMarginalHeightMemory"); 


  ms->pushWord("drawMapRegisters"); // render register 1
  // ATTN 10
  //ms->pushWord(196360); // loadPriorGraspMemory
  //ms->pushWord(1179721); // set graspMemories from classGraspMemories
  switch (ms->config.currentPickMode) {
  case STATIC_PRIOR:
    {
      ms->pushWord(196360); // loadPriorGraspMemory
    }
    break;
  case LEARNING_ALGORITHMC:
  case LEARNING_SAMPLING:
    {
      ms->pushWord(1179721); // set graspMemories from classGraspMemories
    }
    break;
  case STATIC_MARGINALS:
    {
      ms->pushWord(1179721); // set graspMemories from classGraspMemories
    }
    break;
  default:
    {
      assert(0);
    }
    break;
  }
  
  switch (ms->config.currentBoundingBoxMode) {
  case STATIC_PRIOR:
    {
      ms->pushWord(1244936); // loadPriorHeightMemory
    }
    break;
  case LEARNING_ALGORITHMC:
  case LEARNING_SAMPLING:
    {
      ms->pushWord(1245289); // set heightMemories from classHeightMemories
    }
    break;
  case STATIC_MARGINALS:
    {
      //cout << "Pushing set heightMemories from classHeightMemories" << endl;
      ms->pushWord(1245289); // set heightMemories from classHeightMemories
    }
    break;
  case MAPPING:
    {
    }
    break;
  default:
    {
      assert(0);
    }
    break;
  }
}


int ARE_GENERIC_PICK_LEARNING(shared_ptr<MachineState> ms) {
  return ( (ms->config.currentPickMode == LEARNING_SAMPLING) ||
	   (ms->config.currentPickMode == LEARNING_ALGORITHMC) );
}


int ARE_GENERIC_HEIGHT_LEARNING(shared_ptr<MachineState> ms) {
  return ( (ms->config.currentBoundingBoxMode == LEARNING_SAMPLING) ||
	   (ms->config.currentBoundingBoxMode == LEARNING_ALGORITHMC) );
}



void guard3dGrasps(shared_ptr<MachineState> ms) {
  if (ms->config.class3dGrasps.size() < ms->config.numClasses) {
    ms->config.class3dGrasps.resize(ms->config.numClasses);
  }
}

void guardGraspMemory(shared_ptr<MachineState> ms) {

  {
    if (ms->config.classGraspMemoryTries1.size() <= ms->config.focusedClass) {
      ms->config.classGraspMemoryTries1.resize(ms->config.focusedClass + 1);
    }
    if (ms->config.classGraspMemoryPicks1.size() <= ms->config.focusedClass) {
      ms->config.classGraspMemoryPicks1.resize(ms->config.focusedClass + 1);
    }

    if (ms->config.classGraspMemoryTries2.size() <= ms->config.focusedClass) {
      ms->config.classGraspMemoryTries2.resize(ms->config.focusedClass + 1);
    }
    if (ms->config.classGraspMemoryPicks2.size() <= ms->config.focusedClass) {
      ms->config.classGraspMemoryPicks2.resize(ms->config.focusedClass + 1);
    }

    if (ms->config.classGraspMemoryTries3.size() <= ms->config.focusedClass) {
      ms->config.classGraspMemoryTries3.resize(ms->config.focusedClass + 1);
    }
    if (ms->config.classGraspMemoryPicks3.size() <= ms->config.focusedClass) {
      ms->config.classGraspMemoryPicks3.resize(ms->config.focusedClass + 1);
    }

    if (ms->config.classGraspMemoryTries4.size() <= ms->config.focusedClass) {
      ms->config.classGraspMemoryTries4.resize(ms->config.focusedClass + 1);
    }
    if (ms->config.classGraspMemoryPicks4.size() <= ms->config.focusedClass) {
      ms->config.classGraspMemoryPicks4.resize(ms->config.focusedClass + 1);
    }

  }

  {
    bool loadPrior = false;
    if (!((ms->config.classGraspMemoryTries1[ms->config.focusedClass].rows > 1) && (ms->config.classGraspMemoryTries1[ms->config.focusedClass].cols > 1) &&
	(ms->config.classGraspMemoryPicks1[ms->config.focusedClass].rows > 1) && (ms->config.classGraspMemoryPicks1[ms->config.focusedClass].cols > 1) )) {
      ms->config.classGraspMemoryTries1[ms->config.focusedClass] = Mat(ms->config.rmWidth, ms->config.rmWidth, CV_64F);
      ms->config.classGraspMemoryPicks1[ms->config.focusedClass] = Mat(ms->config.rmWidth, ms->config.rmWidth, CV_64F);
      loadPrior = true;
    }
    if (!((ms->config.classGraspMemoryTries2[ms->config.focusedClass].rows > 1) && (ms->config.classGraspMemoryTries2[ms->config.focusedClass].cols > 1) &&
	(ms->config.classGraspMemoryPicks2[ms->config.focusedClass].rows > 1) && (ms->config.classGraspMemoryPicks2[ms->config.focusedClass].cols > 1) )) {
      ms->config.classGraspMemoryTries2[ms->config.focusedClass] = Mat(ms->config.rmWidth, ms->config.rmWidth, CV_64F);
      ms->config.classGraspMemoryPicks2[ms->config.focusedClass] = Mat(ms->config.rmWidth, ms->config.rmWidth, CV_64F);
      loadPrior = true;
    }
    if (!((ms->config.classGraspMemoryTries3[ms->config.focusedClass].rows > 1) && (ms->config.classGraspMemoryTries3[ms->config.focusedClass].cols > 1) &&
	(ms->config.classGraspMemoryPicks3[ms->config.focusedClass].rows > 1) && (ms->config.classGraspMemoryPicks3[ms->config.focusedClass].cols > 1) )) {
      ms->config.classGraspMemoryTries3[ms->config.focusedClass] = Mat(ms->config.rmWidth, ms->config.rmWidth, CV_64F);
      ms->config.classGraspMemoryPicks3[ms->config.focusedClass] = Mat(ms->config.rmWidth, ms->config.rmWidth, CV_64F);
      loadPrior = true;
    }
    if (!((ms->config.classGraspMemoryTries4[ms->config.focusedClass].rows > 1) && (ms->config.classGraspMemoryTries4[ms->config.focusedClass].cols > 1) &&
	(ms->config.classGraspMemoryPicks4[ms->config.focusedClass].rows > 1) && (ms->config.classGraspMemoryPicks4[ms->config.focusedClass].cols > 1) )) {
      ms->config.classGraspMemoryTries4[ms->config.focusedClass] = Mat(ms->config.rmWidth, ms->config.rmWidth, CV_64F);
      ms->config.classGraspMemoryPicks4[ms->config.focusedClass] = Mat(ms->config.rmWidth, ms->config.rmWidth, CV_64F);
      loadPrior = true;
    }
    if (loadPrior) {
      loadPriorGraspMemory(ms, ANALYTIC_PRIOR);
    }
  }

}

void guardHeightMemory(shared_ptr<MachineState> ms) {
  if (ms->config.focusedClass == -1) {
    ROS_ERROR_STREAM("Focused class not initialized! " << ms->config.focusedClass);
  }
  if (ms->config.classHeightMemoryTries.size() <= ms->config.focusedClass) {
    ms->config.classHeightMemoryTries.resize(ms->config.focusedClass + 1);
  }
  if (ms->config.classHeightMemoryPicks.size() <= ms->config.focusedClass) {
    ms->config.classHeightMemoryPicks.resize(ms->config.focusedClass + 1);
  }
  if (!((ms->config.classHeightMemoryTries[ms->config.focusedClass].rows > 1) && (ms->config.classHeightMemoryTries[ms->config.focusedClass].cols == 1) &&
	(ms->config.classHeightMemoryPicks[ms->config.focusedClass].rows > 1) && (ms->config.classHeightMemoryPicks[ms->config.focusedClass].cols == 1) )) {
    ms->config.classHeightMemoryTries[ms->config.focusedClass] = Mat(ms->config.hmWidth, 1, CV_64F);
    ms->config.classHeightMemoryPicks[ms->config.focusedClass] = Mat(ms->config.hmWidth, 1, CV_64F);
    loadPriorHeightMemory(ms, ANALYTIC_PRIOR);
  }
}

int calibrateGripper(shared_ptr<MachineState> ms) {
  if (ms->config.currentRobotMode == SIMULATED) {
    return 0;
  }
  for (int i = 0; i < 10; i++) {
    int return_value = doCalibrateGripper(ms);
    if (return_value == 0) {
      return return_value;
    }
  }
  cout << "Gripper could not calibrate!" << endl;
  ms->pushWord("pauseStackExecution"); // pause stack execution
  ms->pushCopies("beep", 15); // beep
  return -1;
}
int doCalibrateGripper(shared_ptr<MachineState> ms) {
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

void convertGlobalGraspIdxToLocal(shared_ptr<MachineState> ms, const int rx, const int ry, 
                                  int * localX, int * localY) {
  // COMPLETELY UNTESTED
  assert(0);
  // find global coordinate of current point
  double thX = (rx-ms->config.rmHalfWidth) * ms->config.rmDelta;
  double thY = (ry-ms->config.rmHalfWidth) * ms->config.rmDelta;
  // transform it into local coordinates
  double unangle = -ms->config.bestOrientationAngle;
  double unscale = 1.0;
  Point uncenter = Point(0, 0);
  Mat un_rot_mat = getRotationMatrix2D(uncenter, unangle, unscale);
  Mat toUn(3,1,CV_64F);
  toUn.at<double>(0,0)=thX;
  toUn.at<double>(1,0)=thY;
  toUn.at<double>(2,0)=1.0;
  Mat didUn = un_rot_mat*toUn;
  double localThX = didUn.at<double>(0,0);
  double localThY = didUn.at<double>(1,0);
  *localX = ((localThX)/ms->config.rmDelta) + ms->config.rmHalfWidth; 
  *localY = ((localThY)/ms->config.rmDelta) + ms->config.rmHalfWidth; 

}

void convertLocalGraspIdxToGlobal(shared_ptr<MachineState> ms, const int localX, const int localY,
                                  int * rx, int * ry) {
  // find local coordinate of current point
  double thX = (localX-ms->config.rmHalfWidth) * ms->config.rmDelta;
  double thY = (localY-ms->config.rmHalfWidth) * ms->config.rmDelta;
  // transform it into local coordinates
  double unangle = ms->config.bestOrientationAngle;
  double unscale = 1.0;
  Point uncenter = Point(0, 0);
  Mat un_rot_mat = getRotationMatrix2D(uncenter, unangle, unscale);
  Mat toUn(3,1,CV_64F);
  toUn.at<double>(0,0)=thX;
  toUn.at<double>(1,0)=thY;
  toUn.at<double>(2,0)=1.0;
  Mat didUn = un_rot_mat*toUn;
  double localThX = didUn.at<double>(0,0);
  double localThY = didUn.at<double>(1,0);
  *rx = (int) round(((localThX)/ms->config.rmDelta) + ms->config.rmHalfWidth); 
  *ry = (int) round(((localThY)/ms->config.rmDelta) + ms->config.rmHalfWidth); 
}


void loadSampledGraspMemory(shared_ptr<MachineState> ms) {
  ROS_INFO("Loading sampled grasp memory.");
  for (int tGG = 0; tGG < ms->config.totalGraspGears/2; tGG++) {
    for (int rx = 0; rx < ms->config.rmWidth; rx++) {
      for (int ry = 0; ry < ms->config.rmWidth; ry++) {
        

	// ATTN 19 this isn't quite Thompson sampling...
	//   regularization.
        int i = rx + ry * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tGG;
        double nsuccess = ms->config.pickEccentricity * (ms->config.graspMemoryPicks[i]);
        double nfailure = ms->config.pickEccentricity * (ms->config.graspMemoryTries[i] - ms->config.graspMemoryPicks[i]);
        ms->config.graspMemorySample[i] = rk_beta(&ms->config.random_state, 
                                       nsuccess + 1, 
                                       nfailure + 1);
      }
    }
  }
}


void loadMarginalGraspMemory(shared_ptr<MachineState> ms) {
  ROS_INFO("Loading marginal grasp memory.");
  for (int tGG = 0; tGG < ms->config.totalGraspGears/2; tGG++) {
    for (int rx = 0; rx < ms->config.rmWidth; rx++) {
      for (int ry = 0; ry < ms->config.rmWidth; ry++) {
        int i = rx + ry * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tGG;
        double nsuccess = ms->config.graspMemoryPicks[i];
        double nfailure = ms->config.graspMemoryTries[i] - ms->config.graspMemoryPicks[i];
        ms->config.graspMemorySample[i] = (nsuccess + 1) / (nsuccess + nfailure + 2);
      }
    }
  }
}

void loadPriorGraspMemory(shared_ptr<MachineState> ms, priorType prior) {
  ROS_INFO("Loading prior grasp memory.");
  double max_range_value = -VERYBIGNUMBER;
  double min_range_value = VERYBIGNUMBER;


  for (int tGG = 0; tGG < ms->config.totalGraspGears/2; tGG++) {
    prepareGraspFilter(ms, tGG);
    loadLocalTargetClassRangeMap(ms, ms->config.rangeMapReg3, ms->config.rangeMapReg4);
    applyGraspFilter(ms, ms->config.rangeMapReg3, ms->config.rangeMapReg4);

    for (int rx = 0; rx < ms->config.rmWidth; rx++) {
      for (int ry = 0; ry < ms->config.rmWidth; ry++) {
        int i = rx + ry * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tGG;
        ms->config.graspMemoryReg1[i] = ms->config.rangeMapReg3[rx + ry * ms->config.rmWidth];
        if (ms->config.graspMemoryReg1[i] < min_range_value) {
          min_range_value = ms->config.graspMemoryReg1[i];
        }
        if (ms->config.graspMemoryReg1[i] > max_range_value) {
          max_range_value = ms->config.graspMemoryReg1[i];
        }
      }
    }
  }

  // ATTN 18
  // make the grasp gears symmetric so we can reliably 
  // populate a fixed number of grasps.
  int symmetrizeGraspGears = 0;
  if (symmetrizeGraspGears) {
    for (int rx = 0; rx < ms->config.rmWidth; rx++) {
      for (int ry = 0; ry < ms->config.rmWidth; ry++) {
	double minAtThisXY = INFINITY;
	for (int tGG = 0; tGG < ms->config.totalGraspGears/2; tGG++) {
	  int i = rx + ry * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tGG;
	  if (ms->config.graspMemoryReg1[i] < minAtThisXY) {
	    minAtThisXY = ms->config.graspMemoryReg1[i];
	  }
	}
	for (int tGG = 0; tGG < ms->config.totalGraspGears/2; tGG++) {
	  int i = rx + ry * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tGG;
	  ms->config.graspMemoryReg1[i] = minAtThisXY; 
	}
      }
    }
  }

  for (int tGG = 0; tGG < ms->config.totalGraspGears/2; tGG++) {
    for (int rx = 0; rx < ms->config.rmWidth; rx++) {
      for (int ry = 0; ry < ms->config.rmWidth; ry++) {
        int i = rx + ry * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tGG;
        ms->config.graspMemoryReg1[i] = (max_range_value - ms->config.graspMemoryReg1[i]) / (max_range_value - min_range_value);
      }
    }
  }

  // make everything peakier
  // for (int tGG = 0; tGG < ms->config.totalGraspGears/2; tGG++) {
  //   for (int rx = 0; rx < ms->config.rmWidth; rx++) {
  //     for (int ry = 0; ry < ms->config.rmWidth; ry++) {
  //       int i = rx + ry * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tGG;
  //       ms->config.graspMemoryReg1[i] = pow(ms->config.graspMemoryReg1[i], 4);
  //     }
  //   }
  // }
  
  std::vector<double> sorted;
  for (int tGG = 0; tGG < ms->config.totalGraspGears/2; tGG++) {
    for (int rx = 0; rx < ms->config.rmWidth; rx++) {
      for (int ry = 0; ry < ms->config.rmWidth; ry++) {
        int i = rx + ry * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tGG;
        sorted.push_back(ms->config.graspMemoryReg1[i]);
      }
    }
  }
  
  std::sort (sorted.begin(), sorted.end());

  int numLocationsToTry = 10;
  double threshold = sorted[sorted.size() - 4*numLocationsToTry];

  for (int tGG = 0; tGG < ms->config.totalGraspGears/2; tGG++) {
    for (int rx = 0; rx < ms->config.rmWidth; rx++) {
      for (int ry = 0; ry < ms->config.rmWidth; ry++) {
        int i = rx + ry * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tGG;
        if (ms->config.graspMemoryReg1[i] < threshold && prior == UNIFORM_PRIOR) {
          ms->config.graspMemoryReg1[i] = 0;
        }
      }
    }
  }


  for (int tGG = 0; tGG < ms->config.totalGraspGears/2; tGG++) {
    for (int rx = 0; rx < ms->config.rmWidth; rx++) {
      for (int ry = 0; ry < ms->config.rmWidth; ry++) {
        int i = rx + ry * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tGG;
        double mu = ms->config.graspMemoryReg1[i];
        double nfailure;
        //double eccentricity = 3.0;//100;
	// ATTN 19
        //double nsuccess = (eccentricity * mu);
        double nsuccess = mu;

        if (mu == 0) {
          //nfailure = VERYBIGNUMBER;
          nfailure = 1;
        } else {
          if (prior == UNIFORM_PRIOR) {
            nfailure = 0;
          } else if (prior == ANALYTIC_PRIOR) {
	    // ATTN 19
            //nfailure = round(eccentricity * (1 - mu));
            //nfailure = eccentricity * (1 - mu);
            nfailure = (1 - mu);
          } else {
            cout << "Invalid prior: " << prior << endl;
            assert(0);
          }
        }

	if (prior == UNIFORM_PRIOR) {
	  nsuccess = 0;
	}
	ms->config.graspMemoryPicks[i] = nsuccess;
	ms->config.graspMemoryTries[i] = nsuccess + nfailure;
      }
    }
  }
}

void loadMarginalHeightMemory(shared_ptr<MachineState> ms) {
  //ROS_INFO("Loading marginal height memory.");
  for (int i = 0; i < ms->config.hmWidth; i++) {
    double nsuccess = ms->config.heightMemoryPicks[i];
    double nfailure = ms->config.heightMemoryTries[i] - ms->config.heightMemoryPicks[i];
    ms->config.heightMemorySample[i] = (nsuccess + 1) / (nsuccess + nfailure + 2);
  }
}
 
void loadSampledHeightMemory(shared_ptr<MachineState> ms) {
  ROS_INFO("Loading sampled height memory.");
  for (int i = 0; i < ms->config.hmWidth; i++) {
    double nsuccess = ms->config.heightEccentricity * (ms->config.heightMemoryPicks[i]);
    double nfailure = ms->config.heightEccentricity * (ms->config.heightMemoryTries[i] - ms->config.heightMemoryPicks[i]);
    ms->config.heightMemorySample[i] = rk_beta(&ms->config.random_state, 
                                    nsuccess + 1, 
                                    nfailure + 1);
  }
  drawHeightMemorySample(ms);
}

double convertHeightIdxToGlobalZ(shared_ptr<MachineState> ms, int heightIdx) {
  double tabledMaxHeight = ms->config.maxHeight - ms->config.currentTableZ;
  double tabledMinHeight = ms->config.minHeight - ms->config.currentTableZ;

  double scaledHeight = (double(heightIdx)/double(ms->config.hmWidth-1)) * (tabledMaxHeight - tabledMinHeight);
  double scaledTranslatedHeight = scaledHeight + tabledMinHeight;
  return scaledTranslatedHeight;
}

int convertHeightGlobalZToIdx(shared_ptr<MachineState> ms, double globalZ) {
  double tabledMaxHeight = ms->config.maxHeight - ms->config.currentTableZ;
  double tabledMinHeight = ms->config.minHeight - ms->config.currentTableZ;

  double scaledHeight = (globalZ - ms->config.currentTableZ) / (tabledMaxHeight - tabledMinHeight);
  int heightIdx = floor(scaledHeight * (ms->config.hmWidth - 1));
}

void testHeightConversion(shared_ptr<MachineState> ms) {
  for (int i = 0; i < ms->config.hmWidth; i++) {
    double height = convertHeightIdxToGlobalZ(ms, i);
    int newIdx = convertHeightGlobalZToIdx(ms, height);
    cout << "i: " << i << " height: " << height << " newIdx: " << newIdx << endl;
    //assert(newIdx == i);
  }
}

void loadPriorHeightMemory(shared_ptr<MachineState> ms, priorType prior) {
  for (int i = 0; i < ms->config.hmWidth; i++) {
    ms->config.heightMemoryPicks[i] = 1;
    ms->config.heightMemoryTries[i] = 1;
  }
  if (prior == ANALYTIC_PRIOR) {
    ms->config.heightMemoryPicks[1] = 1;
    ms->config.heightMemoryTries[1] = 1;
  }
}

void drawHeightMemorySample(shared_ptr<MachineState> ms) {
  
  {
    double max_value = -VERYBIGNUMBER;
    int max_i=0, max_ry=0, max_rx=0;
    for (int i = 0; i < ms->config.hmWidth; i++) {
      if (ms->config.heightMemorySample[i] > max_value) {
	max_value = ms->config.heightMemorySample[i];
	max_i = i;
	max_rx = ms->config.hmWidth - 1 - max_i;
	max_ry = 0;
      }
      {
	int ry = 0;
	int rx = ms->config.hmWidth - 1 - i;
	double blueIntensity = 255 * ms->config.heightMemorySample[i];
	double greenIntensity = 255 * ms->config.heightMemorySample[i];
	double redIntensity = 255 * ms->config.heightMemorySample[i];
	//cout << "Height Memory Sample: " << "rx: " << rx << " ry: " << ry << " tGG:" << tGG << "sample: " << ms->config.heightMemorySample[i] << endl;
	cv::Scalar color(ceil(blueIntensity),ceil(greenIntensity),ceil(redIntensity));
	
	cv::Point outTop = cv::Point((ry)*ms->config.hmiCellWidth,(rx)*ms->config.hmiCellWidth);
	cv::Point outBot = cv::Point(((ry)+1)*ms->config.hmiCellWidth,((rx)+1)*ms->config.hmiCellWidth);
	Mat vCrop = ms->config.heightMemorySampleImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
	vCrop = color;
      }
    }
    {
      // draw the max
      char buff[256];
      cv::Point text_anchor = cv::Point((max_ry) * ms->config.hmiCellWidth - 5, 
					(max_rx + 1) * ms->config.hmiCellWidth);
      sprintf(buff, "x");
      putText(ms->config.heightMemorySampleImage, buff, text_anchor, MY_FONT, 7, 
	      Scalar(0,0,255), 2);
    }
  }
  {
    double max_value = -VERYBIGNUMBER;
    int max_i=0, max_ry=0, max_rx=0;
    for (int i = 0; i < ms->config.hmWidth; i++) {
      double thisMarginal = (ms->config.heightMemoryPicks[i]+1)/(ms->config.heightMemoryTries[i]+2);
      if (thisMarginal > max_value) {
	max_value = thisMarginal;
	max_i = i;
	max_rx = ms->config.hmWidth - 1 - max_i;
	max_ry = 0;
      }
      {
	int ry = 0;
	int rx = ms->config.hmWidth - 1 - i;
	double blueIntensity = 255 * thisMarginal;
	double greenIntensity = 255 * thisMarginal;
	double redIntensity = 255 * thisMarginal;
	//cout << "Height Memory Marginal: " << "rx: " << rx << " ry: " << ry << " tGG:" << tGG << "sample: " << thisMarginal << endl;
	cv::Scalar color(ceil(blueIntensity),ceil(greenIntensity),ceil(redIntensity));
	
	cv::Point outTop = cv::Point((ry+1)*ms->config.hmiCellWidth,(rx)*ms->config.hmiCellWidth);
	cv::Point outBot = cv::Point(((ry+1)+1)*ms->config.hmiCellWidth,((rx)+1)*ms->config.hmiCellWidth);
	Mat vCrop = ms->config.heightMemorySampleImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
	vCrop = color;
      }
    }
    {
      // draw the max
      char buff[256];
      cv::Point text_anchor = cv::Point((max_ry+1) * ms->config.hmiCellWidth, 
					(max_rx + 1) * ms->config.hmiCellWidth);
      sprintf(buff, "x");
      putText(ms->config.heightMemorySampleImage, buff, text_anchor, MY_FONT, 7, 
	      Scalar(0,0,255), 2);
    }
  }
}

void copyHeightMemoryTriesToClassHeightMemoryTries(shared_ptr<MachineState> ms) {
  guardHeightMemory(ms);
  for (int i = 0; i < ms->config.hmWidth; i++) {
    ms->config.classHeightMemoryTries[ms->config.focusedClass].at<double>(i,0) = ms->config.heightMemoryTries[i];
    ms->config.classHeightMemoryPicks[ms->config.focusedClass].at<double>(i,0) = ms->config.heightMemoryPicks[i];
  }
}

void estimateGlobalGraspGear(shared_ptr<MachineState> ms) {
  ROS_INFO("Estimating global grasp gear.");
  double max_range_value = -VERYBIGNUMBER;
  double min_range_value = VERYBIGNUMBER;
  int eMinGG = 0;

  for (int tGG = 0; tGG < ms->config.totalGraspGears/2; tGG++) {
    prepareGraspFilter(ms, tGG);
    loadGlobalTargetClassRangeMap(ms, ms->config.rangeMapReg3, ms->config.rangeMapReg4);
    applyGraspFilter(ms, ms->config.rangeMapReg3, ms->config.rangeMapReg4);

    int rx = ms->config.maxX;
    int ry = ms->config.maxY;

    int i = rx + ry * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tGG;
    ms->config.graspMemoryReg1[i] = ms->config.rangeMapReg3[rx + ry * ms->config.rmWidth];
    if (ms->config.graspMemoryReg1[i] < min_range_value) {
      min_range_value = ms->config.graspMemoryReg1[i];
      eMinGG = tGG;
    }
    if (ms->config.graspMemoryReg1[i] > max_range_value) {
      max_range_value = ms->config.graspMemoryReg1[i];
    }
  }

  ms->config.maxGG = eMinGG;
  ms->config.localMaxGG = getLocalGraspGear(ms, eMinGG);
}

void drawMapRegisters(shared_ptr<MachineState> ms) {
  {
    double minDepth = VERYBIGNUMBER;
    double maxDepth = 0;
    for (int rx = 0; rx < ms->config.rmWidth; rx++) {
      for (int ry = 0; ry < ms->config.rmWidth; ry++) {
        minDepth = min(minDepth, ms->config.rangeMapReg1[rx + ry*ms->config.rmWidth]);
        maxDepth = max(maxDepth, ms->config.rangeMapReg1[rx + ry*ms->config.rmWidth]);
      }
    }
    for (int rx = 0; rx < ms->config.rmWidth; rx++) {
      for (int ry = 0; ry < ms->config.rmWidth; ry++) {
        double denom = max(EPSILON,maxDepth-minDepth);
        if (denom <= EPSILON)
          denom = VERYBIGNUMBER;
        double intensity = 255 * (maxDepth - ms->config.rangeMapReg1[rx + ry*ms->config.rmWidth]) / denom;
        //cout << denom << " " << maxDepth << " " << ms->config.rangeMapReg1[rx + ry*ms->config.rmWidth] << " " << (maxDepth - ms->config.rangeMapReg1[rx + ry*ms->config.rmWidth]) << " " << endl;
        cv::Scalar backColor(0,0,ceil(intensity));
        cv::Point outTop = cv::Point((ry+ms->config.rmWidth)*ms->config.rmiCellWidth,rx*ms->config.rmiCellWidth);
        cv::Point outBot = cv::Point(((ry+ms->config.rmWidth)+1)*ms->config.rmiCellWidth,(rx+1)*ms->config.rmiCellWidth);
        Mat vCrop = ms->config.rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
        vCrop = backColor;
      }
    }
  }
  {
    double minDepth = VERYBIGNUMBER;
    double maxDepth = 0;
    for (int rx = 0; rx < ms->config.rmWidth; rx++) {
      for (int ry = 0; ry < ms->config.rmWidth; ry++) {
        minDepth = min(minDepth, ms->config.rangeMapReg2[rx + ry*ms->config.rmWidth]);
        maxDepth = max(maxDepth, ms->config.rangeMapReg2[rx + ry*ms->config.rmWidth]);
      }
    }
    for (int rx = 0; rx < ms->config.rmWidth; rx++) {
      for (int ry = 0; ry < ms->config.rmWidth; ry++) {
        double denom = max(EPSILON,maxDepth-minDepth);
        if (denom <= EPSILON)
          denom = VERYBIGNUMBER;
        double intensity = 255 * (maxDepth - ms->config.rangeMapReg2[rx + ry*ms->config.rmWidth]) / denom;
        cv::Scalar backColor(0,0,ceil(intensity));
        cv::Point outTop = cv::Point((ry+2*ms->config.rmWidth)*ms->config.rmiCellWidth,rx*ms->config.rmiCellWidth);
        cv::Point outBot = cv::Point(((ry+2*ms->config.rmWidth)+1)*ms->config.rmiCellWidth,(rx+1)*ms->config.rmiCellWidth);
        Mat vCrop = ms->config.rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
        vCrop = backColor;
      }
    }
  }
  {
    double minDepth = VERYBIGNUMBER;
    double maxDepth = 0;
    for (int rx = 0; rx < ms->config.hrmWidth; rx++) {
      for (int ry = 0; ry < ms->config.hrmWidth; ry++) {
        minDepth = min(minDepth, ms->config.hiRangeMap[rx + ry*ms->config.hrmWidth]);
        maxDepth = max(maxDepth, ms->config.hiRangeMap[rx + ry*ms->config.hrmWidth]);
      }
    }
    for (int rx = 0; rx < ms->config.hrmWidth; rx++) {
      for (int ry = 0; ry < ms->config.hrmWidth; ry++) {
        double denom = max(EPSILON,maxDepth-minDepth);
        if (denom <= EPSILON)
          denom = VERYBIGNUMBER;
        double intensity = 255 * (maxDepth - ms->config.hiRangeMap[rx + ry*ms->config.hrmWidth]) / denom;
        ms->config.hiRangemapImage.at<cv::Vec3b>(rx,ry) = cv::Vec3b(0,0,ceil(intensity));
      }
    }
  }
  {
    double minDepth = VERYBIGNUMBER;
    double maxDepth = 0;
    for (int rx = 0; rx < ms->config.hrmWidth; rx++) {
      for (int ry = 0; ry < ms->config.hrmWidth; ry++) {
        minDepth = min(minDepth, ms->config.hiRangeMapReg1[rx + ry*ms->config.hrmWidth]);
        maxDepth = max(maxDepth, ms->config.hiRangeMapReg1[rx + ry*ms->config.hrmWidth]);
      }
    }
    for (int rx = 0; rx < ms->config.hrmWidth; rx++) {
      for (int ry = 0; ry < ms->config.hrmWidth; ry++) {
        double denom = max(EPSILON,maxDepth-minDepth);
        if (denom <= EPSILON)
          denom = VERYBIGNUMBER;
        double intensity = 255 * (maxDepth - ms->config.hiRangeMapReg1[rx + ry*ms->config.hrmWidth]) / denom;
        ms->config.hiRangemapImage.at<cv::Vec3b>(rx,ry+ms->config.hrmWidth) = cv::Vec3b(0,0,ceil(intensity));
      }
    }
  }
  {
    double minDepth = VERYBIGNUMBER;
    double maxDepth = 0;
    for (int rx = 0; rx < ms->config.hrmWidth; rx++) {
      for (int ry = 0; ry < ms->config.hrmWidth; ry++) {
        minDepth = min(minDepth, ms->config.hiRangeMapReg2[rx + ry*ms->config.hrmWidth]);
        maxDepth = max(maxDepth, ms->config.hiRangeMapReg2[rx + ry*ms->config.hrmWidth]);
      }
    }
    for (int rx = 0; rx < ms->config.hrmWidth; rx++) {
      for (int ry = 0; ry < ms->config.hrmWidth; ry++) {
        double denom = max(EPSILON,maxDepth-minDepth);
        if (denom <= EPSILON)
          denom = VERYBIGNUMBER;
        double intensity = 255 * (maxDepth - ms->config.hiRangeMapReg2[rx + ry*ms->config.hrmWidth]) / denom;
        ms->config.hiRangemapImage.at<cv::Vec3b>(rx,ry+2*ms->config.hrmWidth) = cv::Vec3b(0,0,ceil(intensity));
      }
    }
  }

  // draw grasp memory window
  {
    {
      double minDepth = VERYBIGNUMBER;
      double maxDepth = 0;
      for (int rx = 0; rx < ms->config.rmWidth; rx++) {
        for (int ry = 0; ry < ms->config.rmWidth; ry++) {
          minDepth = min(minDepth, ms->config.graspMemoryTries[rx + ry*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*ms->config.currentGraspGear]);
          maxDepth = max(maxDepth, ms->config.graspMemoryTries[rx + ry*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*ms->config.currentGraspGear]);
        }
      }
      for (int rx = 0; rx < ms->config.rmWidth; rx++) {
        for (int ry = 0; ry < ms->config.rmWidth; ry++) {
          double denom = max(1.0,maxDepth);
          if (denom <= EPSILON)
            denom = VERYBIGNUMBER;
          double blueIntensity = 128 * (ms->config.graspMemoryPicks[rx + ry*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*ms->config.currentGraspGear]) / denom;
          double redIntensity = 128 * (ms->config.graspMemoryTries[rx + ry*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*ms->config.currentGraspGear] - ms->config.graspMemoryPicks[rx + ry*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*ms->config.currentGraspGear]) / denom;
          cv::Scalar backColor(ceil(blueIntensity),0,ceil(redIntensity));
          cv::Point outTop = cv::Point((ry)*ms->config.rmiCellWidth,rx*ms->config.rmiCellWidth);
          cv::Point outBot = cv::Point(((ry)+1)*ms->config.rmiCellWidth,(rx+1)*ms->config.rmiCellWidth);
          Mat vCrop = ms->config.graspMemoryImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
          vCrop = backColor;
        }
      }
    }
    if ((ms->config.targetClass > -1) && (ms->config.classRangeMaps[ms->config.targetClass].rows > 1) && (ms->config.classRangeMaps[ms->config.targetClass].cols > 1)) {
      double minDepth = VERYBIGNUMBER;
      double maxDepth = 0;
      for (int rx = 0; rx < ms->config.rmWidth; rx++) {
        for (int ry = 0; ry < ms->config.rmWidth; ry++) {

          minDepth = min(minDepth, ms->config.classRangeMaps[ms->config.targetClass].at<double>(ry,rx));
          maxDepth = max(maxDepth, ms->config.classRangeMaps[ms->config.targetClass].at<double>(ry,rx));
        }
      }
      for (int rx = 0; rx < ms->config.rmWidth; rx++) {
        for (int ry = 0; ry < ms->config.rmWidth; ry++) {
          double denom = max(EPSILON,maxDepth-minDepth);
          if (denom <= EPSILON)
            denom = VERYBIGNUMBER;
          double greenIntensity = 255 * (maxDepth - ms->config.classRangeMaps[ms->config.targetClass].at<double>(ry,rx)) / denom;
          {
            cv::Scalar backColor(0,ceil(greenIntensity),0);
            cv::Point outTop = cv::Point((ry+ms->config.rmWidth)*ms->config.rmiCellWidth,rx*ms->config.rmiCellWidth);
            cv::Point outBot = cv::Point(((ry+ms->config.rmWidth)+1)*ms->config.rmiCellWidth,(rx+1)*ms->config.rmiCellWidth);
            Mat vCrop = ms->config.graspMemoryImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
            vCrop = backColor;
          }
          {
            cv::Scalar backColor(0,ceil(greenIntensity/2),0);
            cv::Point outTop = cv::Point((ry)*ms->config.rmiCellWidth,rx*ms->config.rmiCellWidth);
            cv::Point outBot = cv::Point(((ry)+1)*ms->config.rmiCellWidth,(rx+1)*ms->config.rmiCellWidth);
            Mat vCrop = ms->config.graspMemoryImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
            vCrop = vCrop + backColor;
          }
        }
      }
    }
  }
  // draw grasp memory sample window
  {
    double max_value = -VERYBIGNUMBER;
    int max_rx=0, max_ry=0, max_tGG=0;
    int dy[4] = {0,1,0,1};
    int dx[4] = {0,0,1,1};
    
    for (int tGG = 0; tGG < ms->config.totalGraspGears/2; tGG++) {

      for (int rx = 0; rx < ms->config.rmWidth; rx++) {
        for (int ry = 0; ry < ms->config.rmWidth; ry++) {
          int i = rx + ry*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tGG;
          if (ms->config.graspMemorySample[i] > max_value) {
            max_value = ms->config.graspMemorySample[i];
            max_rx = rx;
            max_ry = ry;
            max_tGG = tGG;
          }

          
          {
            double blueIntensity = 255 * ms->config.graspMemorySample[i];
            double greenIntensity = 255 * ms->config.graspMemorySample[i];
            double redIntensity = 255 * ms->config.graspMemorySample[i];
            //cout << "Grasp Memory Sample: " << "rx: " << rx << " ry: " << ry << " tGG:" << tGG << "sample: " << ms->config.graspMemorySample[i] << endl;

            cv::Scalar color(ceil(blueIntensity),ceil(greenIntensity),ceil(redIntensity));

            cv::Point outTop = cv::Point((ry + dy[tGG]*ms->config.rmWidth)*ms->config.rmiCellWidth,(rx + dx[tGG]*ms->config.rmWidth)*ms->config.rmiCellWidth);
            cv::Point outBot = cv::Point(((ry + dy[tGG]*ms->config.rmWidth)+1)*ms->config.rmiCellWidth,((rx + dx[tGG]*ms->config.rmWidth)+1)*ms->config.rmiCellWidth);
            Mat vCrop = ms->config.graspMemorySampleImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
            vCrop = color;
          }
        }
      }
    }

    
    for (int tGG = 0; tGG < ms->config.totalGraspGears/2; tGG++) {
      {
        char buff[256];
        cv::Point text_anchor = cv::Point((dy[tGG]*ms->config.rmWidth)*ms->config.rmiCellWidth, 
                                          (dx[tGG]*ms->config.rmWidth + 1)*ms->config.rmiCellWidth);
        sprintf(buff, "%d", tGG+1);
        putText(ms->config.graspMemorySampleImage, buff, text_anchor, MY_FONT, 1, Scalar(192,192,192), 2);
      }
    }

    {
      // draw the max
      char buff[256];
      cv::Point text_anchor = cv::Point((max_ry + dy[max_tGG]*ms->config.rmWidth) * ms->config.rmiCellWidth, 
                                        (max_rx + dx[max_tGG]*ms->config.rmWidth + 1) * ms->config.rmiCellWidth);
      sprintf(buff, "x");
      putText(ms->config.graspMemorySampleImage, buff, text_anchor, MY_FONT, 1, 
              Scalar(0,0,255), 2);
    }
  }
}


void applyGraspFilter(shared_ptr<MachineState> ms, double * rangeMapRegA, double * rangeMapRegB) {
  cout << "Applying filter to rangeMapRegA and storing result in rangeMapRegA." << endl;

  // ATTN 2
  int dx[9] = { -1,  0,  1, 
                -1,  0,  1, 
                -1,  0,  1};
  int dy[9] = { -1, -1, -1, 
                0,  0,  0, 
                1,  1,  1};
  //	int dx[9] = { -2,  0,  2, 
  //		      -2,  0,  2, 
  //		      -2,  0,  2};
  //	int dy[9] = { -2, -2, -2, 
  //		       0,  0,  0, 
  //		       2,  2,  2};
  // ATTN 2
  int transformPadding = 2;
  //int transformPadding = 4;

  for (int rx = 0; rx < ms->config.rmWidth; rx++) {
    for (int ry = 0; ry < ms->config.rmWidth; ry++) {
      rangeMapRegB[rx + ry*ms->config.rmWidth] = 0.0;
    }
  }
  for (int rx = transformPadding; rx < ms->config.rmWidth-transformPadding; rx++) {
    for (int ry = transformPadding; ry < ms->config.rmWidth-transformPadding; ry++) {
      for (int fx = 0; fx < 9; fx++)
        rangeMapRegB[rx + ry*ms->config.rmWidth] += ms->config.filter[fx] * rangeMapRegA[(rx+dx[fx]) + (ry+dy[fx])*ms->config.rmWidth];
    }
  }
  for (int rx = 0; rx < ms->config.rmWidth; rx++) {
    for (int ry = 0; ry < ms->config.rmWidth; ry++) {
      rangeMapRegA[rx + ry*ms->config.rmWidth] = rangeMapRegB[rx + ry*ms->config.rmWidth];
    }
  }

  // XXX TODO Consider: 
  // Push boundary to deepest point...
  double minDepth = VERYBIGNUMBER;
  double maxDepth = 0;
  for (int rx = 0; rx < ms->config.rmWidth; rx++) {
    for (int ry = 0; ry < ms->config.rmWidth; ry++) {
      minDepth = min(minDepth, rangeMapRegA[rx + ry*ms->config.rmWidth]);
      maxDepth = max(maxDepth, rangeMapRegA[rx + ry*ms->config.rmWidth]);
    }
  }
  for (int rx = 0; rx < ms->config.rmWidth; rx++) {
    for (int ry = 0; ry < transformPadding; ry++) {
      rangeMapRegA[rx + ry*ms->config.rmWidth] = maxDepth;
      rangeMapRegB[rx + ry*ms->config.rmWidth] = maxDepth;
    }
    for (int ry = ms->config.rmWidth-transformPadding; ry < ms->config.rmWidth; ry++) {
      rangeMapRegA[rx + ry*ms->config.rmWidth] = maxDepth;
      rangeMapRegB[rx + ry*ms->config.rmWidth] = maxDepth;
    }
  }
  for (int ry = 0; ry < ms->config.rmWidth; ry++) {
    for (int rx = 0; rx < transformPadding; rx++) {
      rangeMapRegA[rx + ry*ms->config.rmWidth] = maxDepth;
      rangeMapRegB[rx + ry*ms->config.rmWidth] = maxDepth;
    }
    for (int rx = ms->config.rmWidth-transformPadding; rx < ms->config.rmWidth; rx++) {
      rangeMapRegA[rx + ry*ms->config.rmWidth] = maxDepth;
      rangeMapRegB[rx + ry*ms->config.rmWidth] = maxDepth;
    }
  }
}
void copyRangeMapRegister(shared_ptr<MachineState> ms, double * src, double * target) {
  for (int ry = 0; ry < ms->config.rmWidth; ry++) {
    for (int rx = 0; rx < ms->config.rmWidth; rx++) {
      target[rx + ry*ms->config.rmWidth] = src[rx + ry * ms->config.rmWidth];
    }
  }
}

void copyGraspMemoryRegister(shared_ptr<MachineState> ms, double * src, double * target) {
  for (int tGG = 0; tGG < ms->config.totalGraspGears/2; tGG++) {
    for (int ry = 0; ry < ms->config.rmWidth; ry++) {
      for (int rx = 0; rx < ms->config.rmWidth; rx++) {
        target[rx + ry*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tGG] = src[rx + ry * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*tGG];
      }
    }
  }
}

void loadGlobalTargetClassRangeMap(shared_ptr<MachineState> ms, double * rangeMapRegA, double * rangeMapRegB) {
  //Quaternionf eeqform(ms->config.currentEEPose.qw, ms->config.currentEEPose.qx, ms->config.currentEEPose.qy, ms->config.currentEEPose.qz);
  Quaternionf eeqform(ms->config.bestOrientationEEPose.qw, ms->config.bestOrientationEEPose.qx, ms->config.bestOrientationEEPose.qy, ms->config.bestOrientationEEPose.qz);
  Quaternionf crane2Orient(0, 1, 0, 0);
  Quaternionf rel = eeqform * crane2Orient.inverse();
  Quaternionf ex(0,1,0,0);
  Quaternionf zee(0,0,0,1);
	
    
  Quaternionf result = rel * ex * rel.conjugate();
  Quaternionf thumb = rel * zee * rel.conjugate();
  double aY = result.y();
  double aX = result.x();

  // ATTN 1
  // this is here to get the signs right
  aX = -aX;

  // ATTN 22
  //double angle = atan2(aY, aX)*180.0/3.1415926;
  double angle = vectorArcTan(ms, aY, aX)*180.0/3.1415926;
  double scale = 1.0;
  Point center = Point(ms->config.rmWidth/2, ms->config.rmWidth/2);
  Size toBecome(ms->config.rmWidth, ms->config.rmWidth);

  cout << "load target class range map angle result eeqform thumb: " << angle << " | " << result.x() << " "  << result.y() << " "  << result.z() << " "  << result.w() << " | " << eeqform.x() << " "  << eeqform.y() << " "  << eeqform.z() << " "  << eeqform.w() << " | " << thumb.x() << " "  << thumb.y() << " "  << thumb.z() << " "  << thumb.w() << endl;

  // Get the rotation matrix with the specifications above
  Mat rotatedClassRangeMap;
  Mat rot_mat = getRotationMatrix2D(center, angle, scale);
  warpAffine(ms->config.classRangeMaps[ms->config.targetClass], rotatedClassRangeMap, rot_mat, toBecome, INTER_LINEAR, BORDER_REPLICATE);

  ms->config.bestOrientationAngle = angle;

  if ((ms->config.targetClass < ms->config.numClasses) && (ms->config.targetClass >= 0)) {
    for (int y = 0; y < ms->config.rmWidth; y++) {
      for (int x = 0; x < ms->config.rmWidth; x++) {
        // unrotated
        //rangeMap[x + y*ms->config.rmWidth] = ms->config.classRangeMaps[ms->config.targetClass].at<double>(y,x);
        //ms->config.rangeMapReg1[x + y*ms->config.rmWidth] = ms->config.classRangeMaps[ms->config.targetClass].at<double>(y,x);
        // rotated
        rangeMapRegA[x + y*ms->config.rmWidth] = rotatedClassRangeMap.at<double>(y,x);
        rangeMapRegB[x + y*ms->config.rmWidth] = rotatedClassRangeMap.at<double>(y,x);
      } 
    } 
  } 
}


void loadLocalTargetClassRangeMap(shared_ptr<MachineState> ms, double * rangeMapRegA, double * rangeMapRegB) {
  if ((ms->config.targetClass < ms->config.numClasses) && (ms->config.targetClass >= 0)) {
    for (int y = 0; y < ms->config.rmWidth; y++) {
      for (int x = 0; x < ms->config.rmWidth; x++) {
        rangeMapRegA[x + y*ms->config.rmWidth] = ms->config.classRangeMaps[ms->config.targetClass].at<double>(y,x);
        rangeMapRegB[x + y*ms->config.rmWidth] = ms->config.classRangeMaps[ms->config.targetClass].at<double>(y,x);
      } 
    } 
  } 
}


void prepareGraspFilter(shared_ptr<MachineState> ms, int i) {
  if (i == 0) {
    prepareGraspFilter1(ms);
  } else if (i == 1) {
    prepareGraspFilter2(ms);
  } else if (i == 2) {
    prepareGraspFilter3(ms);
  } else if (i == 3) {
    prepareGraspFilter4(ms);
  }
}
void prepareGraspFilter1(shared_ptr<MachineState> ms) {
  double tfilter[9]    = {   0, -1,  0, 
                             0,  2,  0, 
                             0, -1,  0};
  for (int fx = 0; fx < 9; fx++)
    ms->config.filter[fx] = tfilter[fx];
  l2NormalizeFilter(ms);
  for (int fx = 0; fx < 9; fx++) {
    cout << ms->config.filter[fx] << endl;
  }

}

void prepareGraspFilter2(shared_ptr<MachineState> ms) {
  double tfilter[9]    = {  -1,  0,  0, 
                            0,  2,  0, 
                            0,  0, -1};
  //double tfilter[9]    = {  -1,  0,  0, 
  //0,  2-diagonalKappa,  0, 
  //0,  0, -1};
  for (int fx = 0; fx < 9; fx++)
    ms->config.filter[fx] = tfilter[fx];
  l2NormalizeFilter(ms);
  for (int fx = 0; fx < 9; fx++) {
    cout << ms->config.filter[fx] << " ";
    ms->config.filter[fx] *= ms->config.diagonalKappa;
    cout << ms->config.filter[fx] << endl;
  }
}
void prepareGraspFilter3(shared_ptr<MachineState> ms) {
  double tfilter[9]    = {   0,  0,  0, 
                             -1,  2, -1, 
                             0,  0,  0};
  for (int fx = 0; fx < 9; fx++)
    ms->config.filter[fx] = tfilter[fx];
  l2NormalizeFilter(ms);
  for (int fx = 0; fx < 9; fx++) {
    cout << ms->config.filter[fx] << endl;
  }
}
void prepareGraspFilter4(shared_ptr<MachineState> ms) {
  double tfilter[9]    = {   0,  0, -1, 
                             0,  2,  0, 
                             -1,  0,  0};
  //double tfilter[9]    = {   0,  0, -1, 
  //0,  2-diagonalKappa,  0, 
  //-1,  0,  0};
  for (int fx = 0; fx < 9; fx++)
    ms->config.filter[fx] = tfilter[fx];
  l2NormalizeFilter(ms);
  for (int fx = 0; fx < 9; fx++) {
    cout << ms->config.filter[fx] << " ";
    ms->config.filter[fx] *= ms->config.diagonalKappa;
    cout << ms->config.filter[fx] << endl;
  }

}

void copyClassGraspMemoryTriesToGraspMemoryTries(shared_ptr<MachineState> ms) {
  if ((ms->config.classGraspMemoryTries1[ms->config.targetClass].rows > 1) && (ms->config.classGraspMemoryTries1[ms->config.targetClass].cols > 1) &&
      (ms->config.classGraspMemoryPicks1[ms->config.targetClass].rows > 1) && (ms->config.classGraspMemoryPicks1[ms->config.targetClass].cols > 1) ) {
    cout << "graspMemoryTries[] = classGraspMemoryTries1" << endl;
    //cout << "ms->config.classGraspMemoryTries1 " << ms->config.classGraspMemoryTries1[ms->config.targetClass] << endl; 
    //cout << "classGraspMemoryPicks1 " << ms->config.classGraspMemoryPicks1[ms->config.targetClass] << endl; 
    for (int y = 0; y < ms->config.rmWidth; y++) {
      for (int x = 0; x < ms->config.rmWidth; x++) {
        ms->config.graspMemoryTries[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*0] = ms->config.classGraspMemoryTries1[ms->config.targetClass].at<double>(y,x);
      } 
    } 
    for (int y = 0; y < ms->config.rmWidth; y++) {
      for (int x = 0; x < ms->config.rmWidth; x++) {
        ms->config.graspMemoryPicks[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*0] = ms->config.classGraspMemoryPicks1[ms->config.targetClass].at<double>(y,x);
      } 
    } 
  } else {
    cout << "Whoops, tried to set grasp memories 1 but they don't exist for this class." << ms->config.targetClass << " " << ms->config.classLabels[ms->config.targetClass] << endl;
  }
  if ((ms->config.classGraspMemoryTries2[ms->config.targetClass].rows > 1) && (ms->config.classGraspMemoryTries2[ms->config.targetClass].cols > 1) &&
      (ms->config.classGraspMemoryPicks2[ms->config.targetClass].rows > 1) && (ms->config.classGraspMemoryPicks2[ms->config.targetClass].cols > 1) ) {
    cout << "graspMemoryTries[] = classGraspMemoryTries2" << endl;
    //cout << "classGraspMemoryTries2 " << ms->config.classGraspMemoryTries2[ms->config.targetClass] << endl; 
    //cout << "classGraspMemoryPicks2 " << ms->config.classGraspMemoryPicks2[ms->config.targetClass] << endl; 
    for (int y = 0; y < ms->config.rmWidth; y++) {
      for (int x = 0; x < ms->config.rmWidth; x++) {
        ms->config.graspMemoryTries[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*1] = ms->config.classGraspMemoryTries2[ms->config.targetClass].at<double>(y,x);
      } 
    } 
    for (int y = 0; y < ms->config.rmWidth; y++) {
      for (int x = 0; x < ms->config.rmWidth; x++) {
        ms->config.graspMemoryPicks[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*1] = ms->config.classGraspMemoryPicks2[ms->config.targetClass].at<double>(y,x);
      } 
    } 
  } else {
    cout << "Whoops, tried to set grasp memories 2 but they don't exist for this class." << ms->config.targetClass << " " << ms->config.classLabels[ms->config.targetClass] << endl;
  }
  if ((ms->config.classGraspMemoryTries3[ms->config.targetClass].rows > 1) && (ms->config.classGraspMemoryTries3[ms->config.targetClass].cols > 1) &&
      (ms->config.classGraspMemoryPicks3[ms->config.targetClass].rows > 1) && (ms->config.classGraspMemoryPicks3[ms->config.targetClass].cols > 1) ) {
    cout << "graspMemoryTries[] = classGraspMemoryTries3" << endl;
    //cout << "classGraspMemoryTries3 " << ms->config.classGraspMemoryTries3[ms->config.targetClass] << endl; 
    //cout << "classGraspMemoryPicks3 " << ms->config.classGraspMemoryPicks3[ms->config.targetClass] << endl; 
    for (int y = 0; y < ms->config.rmWidth; y++) {
      for (int x = 0; x < ms->config.rmWidth; x++) {
        ms->config.graspMemoryTries[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*2] = ms->config.classGraspMemoryTries3[ms->config.targetClass].at<double>(y,x);
      } 
    } 
    for (int y = 0; y < ms->config.rmWidth; y++) {
      for (int x = 0; x < ms->config.rmWidth; x++) {
        ms->config.graspMemoryPicks[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*2] = ms->config.classGraspMemoryPicks3[ms->config.targetClass].at<double>(y,x);
      } 
    } 
  } else {
    cout << "Whoops, tried to set grasp memories 3 but they don't exist for this class." << ms->config.targetClass << " " << ms->config.classLabels[ms->config.targetClass] << endl;
  }
  if ((ms->config.classGraspMemoryTries4[ms->config.targetClass].rows > 1) && (ms->config.classGraspMemoryTries4[ms->config.targetClass].cols > 1) &&
      (ms->config.classGraspMemoryPicks4[ms->config.targetClass].rows > 1) && (ms->config.classGraspMemoryPicks4[ms->config.targetClass].cols > 1) ) {
    cout << "graspMemoryTries[] = classGraspMemoryTries4" << endl;
    //cout << "classGraspMemoryTries4 " << ms->config.classGraspMemoryTries4[ms->config.targetClass] << endl; 
    //cout << "classGraspMemoryPicks4 " << ms->config.classGraspMemoryPicks4[ms->config.targetClass] << endl; 
    for (int y = 0; y < ms->config.rmWidth; y++) {
      for (int x = 0; x < ms->config.rmWidth; x++) {
        ms->config.graspMemoryTries[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*3] = ms->config.classGraspMemoryTries4[ms->config.targetClass].at<double>(y,x);
      } 
    } 
    for (int y = 0; y < ms->config.rmWidth; y++) {
      for (int x = 0; x < ms->config.rmWidth; x++) {
        ms->config.graspMemoryPicks[x + y*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*3] = ms->config.classGraspMemoryPicks4[ms->config.targetClass].at<double>(y,x);
      } 
    } 
  } else {
    cout << "Whoops, tried to set grasp memories 4 but they don't exist for this class." << ms->config.targetClass << " " << ms->config.classLabels[ms->config.targetClass] << endl;
  }
        
  cout << "class " << ms->config.classLabels[ms->config.targetClass] << " number ";

}

void copyGraspMemoryTriesToClassGraspMemoryTries(shared_ptr<MachineState> ms) {
  guardGraspMemory(ms);
  for (int y = 0; y < ms->config.rmWidth; y++) {
    for (int x = 0; x < ms->config.rmWidth; x++) {
      ms->config.classGraspMemoryTries1[ms->config.focusedClass].at<double>(y,x) = ms->config.graspMemoryTries[x + y*ms->config.rmWidth + 0*ms->config.rmWidth*ms->config.rmWidth];
    } 
  }
  for (int y = 0; y < ms->config.rmWidth; y++) {
    for (int x = 0; x < ms->config.rmWidth; x++) {
      ms->config.classGraspMemoryPicks1[ms->config.focusedClass].at<double>(y,x) = ms->config.graspMemoryPicks[x + y*ms->config.rmWidth + 0*ms->config.rmWidth*ms->config.rmWidth];
    } 
  } 

  for (int y = 0; y < ms->config.rmWidth; y++) {
    for (int x = 0; x < ms->config.rmWidth; x++) {
      ms->config.classGraspMemoryTries2[ms->config.focusedClass].at<double>(y,x) = ms->config.graspMemoryTries[x + y*ms->config.rmWidth + 1*ms->config.rmWidth*ms->config.rmWidth];
    } 
  } 
  for (int y = 0; y < ms->config.rmWidth; y++) {
    for (int x = 0; x < ms->config.rmWidth; x++) {
      ms->config.classGraspMemoryPicks2[ms->config.focusedClass].at<double>(y,x) = ms->config.graspMemoryPicks[x + y*ms->config.rmWidth + 1*ms->config.rmWidth*ms->config.rmWidth];
    } 
  } 

  for (int y = 0; y < ms->config.rmWidth; y++) {
    for (int x = 0; x < ms->config.rmWidth; x++) {
      ms->config.classGraspMemoryTries3[ms->config.focusedClass].at<double>(y,x) = ms->config.graspMemoryTries[x + y*ms->config.rmWidth + 2*ms->config.rmWidth*ms->config.rmWidth];
    } 
  }
  for (int y = 0; y < ms->config.rmWidth; y++) {
    for (int x = 0; x < ms->config.rmWidth; x++) {
      ms->config.classGraspMemoryPicks3[ms->config.focusedClass].at<double>(y,x) = ms->config.graspMemoryPicks[x + y*ms->config.rmWidth + 2*ms->config.rmWidth*ms->config.rmWidth];
    } 
  }

  for (int y = 0; y < ms->config.rmWidth; y++) {
    for (int x = 0; x < ms->config.rmWidth; x++) {
      ms->config.classGraspMemoryTries4[ms->config.focusedClass].at<double>(y,x) = ms->config.graspMemoryTries[x + y*ms->config.rmWidth + 3*ms->config.rmWidth*ms->config.rmWidth];
    } 
  } 
  for (int y = 0; y < ms->config.rmWidth; y++) {
    for (int x = 0; x < ms->config.rmWidth; x++) {
      ms->config.classGraspMemoryPicks4[ms->config.focusedClass].at<double>(y,x) = ms->config.graspMemoryPicks[x + y*ms->config.rmWidth + 3*ms->config.rmWidth*ms->config.rmWidth];
    } 
  }
}

void selectMaxTarget(shared_ptr<MachineState> ms, double minDepth) {
  // ATTN 10
  //selectMaxTargetLinearFilter(minDepth);
  //selectMaxTargetThompsonRotated(minDepth);
  //selectMaxTargetThompsonRotated2(minDepth);
  // ATTN 19
  //selectMaxTargetThompson(minDepth);
  //selectMaxTargetThompsonContinuous(minDepth);
  selectMaxTargetThompsonContinuous2(ms, minDepth);
}

void selectMaxTargetLinearFilter(shared_ptr<MachineState> ms, double minDepth) {
  // ATTN 2
  int maxSearchPadding = 3;
  //int maxSearchPadding = 4;

  for (int rx = maxSearchPadding; rx < ms->config.rmWidth-maxSearchPadding; rx++) {
    for (int ry = maxSearchPadding; ry < ms->config.rmWidth-maxSearchPadding; ry++) {

      // ATTN 5
      double graspMemoryWeight = 0.0;
      double graspMemoryBias = VERYBIGNUMBER;
      int localIntThX = -1; 
      int localIntThY = -1; 
      {
        // find global coordinate of current point
        double thX = (rx-ms->config.rmHalfWidth) * ms->config.rmDelta;
        double thY = (ry-ms->config.rmHalfWidth) * ms->config.rmDelta;
        // transform it into local coordinates
        double unangle = -ms->config.bestOrientationAngle;
        double unscale = 1.0;
        Point uncenter = Point(0, 0);
        Mat un_rot_mat = getRotationMatrix2D(uncenter, unangle, unscale);
        Mat toUn(3,1,CV_64F);
        toUn.at<double>(0,0)=thX;
        toUn.at<double>(1,0)=thY;
        toUn.at<double>(2,0)=1.0;
        Mat didUn = un_rot_mat*toUn;
        double localThX = didUn.at<double>(0,0);
        double localThY = didUn.at<double>(1,0);
        localIntThX = ((localThX)/ms->config.rmDelta) + ms->config.rmHalfWidth; 
        localIntThY = ((localThY)/ms->config.rmDelta) + ms->config.rmHalfWidth; 
        // retrieve its value
        double mDenom = max(ms->config.graspMemoryTries[localIntThX + localIntThY*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*getLocalGraspGear(ms, ms->config.currentGraspGear)], 1.0);
        if ((localIntThX < ms->config.rmWidth) && (localIntThY < ms->config.rmWidth)) {

          // Thompson
          //graspMemoryWeight = (graspMemorySample[localIntThX + localIntThY*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*getLocalGraspGear(ms, ms->config.currentGraspGear)]) * -1;  

          // Original
          //graspMemoryWeight = ms->config.graspMemoryPicks[localIntThX + localIntThY*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*getLocalGraspGear(ms, ms->config.currentGraspGear)] / mDenom;
          //graspMemoryWeight = graspMemoryWeight * ms->config.rangeMapReg1[rx + ry*ms->config.rmWidth]);  
           
          // No memory; just linear filter
          graspMemoryWeight = ms->config.rangeMapReg1[rx + ry*ms->config.rmWidth];
          graspMemoryBias = 0;
        } else {
          graspMemoryWeight = 0;
        }
      }


      //cout << "graspMemory Incorporation rx ry lthx lthy gmw: " << rx << " " << ry << " LL: " << localIntThX << " " << localIntThY << " " << graspMemoryWeight << endl;
      //cout << "  gmTargetX gmTargetY eval: " << ms->config.gmTargetX << " " << ms->config.gmTargetY << " " << ms->config.graspMemoryPicks[ms->config.gmTargetX + ms->config.gmTargetY*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*getLocalGraspGear(ms, ms->config.currentGraspGear)] << endl;
	    
      // 
      if (graspMemoryBias + graspMemoryWeight < minDepth) 
      //if (graspMemoryBias + graspMemoryWeight < minDepth)  // thompson
      {
	minDepth = ms->config.rangeMapReg1[rx + ry*ms->config.rmWidth];
	ms->config.maxX = rx;
        ms->config.maxY = ry;
	ms->config.localMaxX = localIntThX;
	ms->config.localMaxY = localIntThY;
	ms->config.localMaxGG = getLocalGraspGear(ms, ms->config.currentGraspGear);
	ms->config.maxD = ms->config.rangeMapReg1[rx + ry*ms->config.rmWidth];
	ms->config.maxGG = ms->config.currentGraspGear;
	ms->config.useContinuousGraspTransform = 0;
      }
    }
  }
  cout << "non-cumulative maxX: " << ms->config.maxX << " maxY: " << ms->config.maxY <<  " maxD: " << ms->config.maxD << " maxGG: " << ms->config.maxGG << endl;
}

void selectMaxTargetThompson(shared_ptr<MachineState> ms, double minDepth) {
  // ATTN 2
  int maxSearchPadding = 3;
  //int maxSearchPadding = 4;

  for (int localX = 0; localX < ms->config.rmWidth; localX++) {
    for (int localY = 0; localY < ms->config.rmWidth; localY++) {
      // ATTN 5
      double graspMemoryWeight = 0.0;
      double graspMemoryBias = VERYBIGNUMBER;
      int rx, ry;
      convertLocalGraspIdxToGlobal(ms, localX, localY, &rx, &ry);
      if ((rx < ms->config.rmWidth) && (ry < ms->config.rmWidth)) {
        
        // Thompson
        graspMemoryWeight = (ms->config.graspMemorySample[localX + localY*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*getLocalGraspGear(ms, ms->config.currentGraspGear)]) * -1;  
        
        graspMemoryBias = 0;
      } else {
        graspMemoryWeight = 0;
      }
      
      //cout << "graspMemory Thompson incorporation rx ry lthx lthy gmw: " << rx << " " << ry << " LL: " << localX << " " << localY << " " << graspMemoryWeight << endl;
      
      // ATTN 19
      int i = localX + localY * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*getLocalGraspGear(ms, ms->config.currentGraspGear);
      int maxedOutTries = isThisGraspMaxedOut(ms, i);

      //if (graspMemoryBias + graspMemoryWeight < minDepth) 
      if ((graspMemoryBias + graspMemoryWeight < minDepth) && !maxedOutTries) {
	minDepth = graspMemoryWeight;
	ms->config.maxX = rx;
        ms->config.maxY = ry;
	ms->config.localMaxX = localX;
	ms->config.localMaxY = localY;
	ms->config.localMaxGG = getLocalGraspGear(ms, ms->config.currentGraspGear);
	ms->config.maxD = graspMemoryWeight;
	ms->config.maxGG = ms->config.currentGraspGear;
	ms->config.useContinuousGraspTransform = 0;
      }
    }
  }
  cout << "non-cumulative Thompson maxX: " << ms->config.maxX << " ms->config.maxY: " << ms->config.maxY <<  " maxD: " << ms->config.maxD << " maxGG: " << ms->config.maxGG << endl;
}

void selectMaxTargetThompsonContinuous(shared_ptr<MachineState> ms, double minDepth) {
  // ATTN 2
  int maxSearchPadding = 3;
  //int maxSearchPadding = 4;

  for (int rx = maxSearchPadding; rx < ms->config.rmWidth-maxSearchPadding; rx++) {
    for (int ry = maxSearchPadding; ry < ms->config.rmWidth-maxSearchPadding; ry++) {

      // ATTN 5
      double graspMemoryWeight = 0.0;
      double graspMemoryBias = VERYBIGNUMBER;
      int localIntThX = -1; 
      int localIntThY = -1; 
      {
        // find global coordinate of current point
        double thX = (rx-ms->config.rmHalfWidth) * ms->config.rmDelta;
        double thY = (ry-ms->config.rmHalfWidth) * ms->config.rmDelta;
        // transform it into local coordinates
        double unangle = -ms->config.bestOrientationAngle;
        double unscale = 1.0;
        Point uncenter = Point(0, 0);
        Mat un_rot_mat = getRotationMatrix2D(uncenter, unangle, unscale);
        Mat toUn(3,1,CV_64F);
        toUn.at<double>(0,0)=thX;
        toUn.at<double>(1,0)=thY;
        toUn.at<double>(2,0)=1.0;
        Mat didUn = un_rot_mat*toUn;
        double localThX = didUn.at<double>(0,0);
        double localThY = didUn.at<double>(1,0);
        localIntThX = ((localThX)/ms->config.rmDelta) + ms->config.rmHalfWidth; 
        localIntThY = ((localThY)/ms->config.rmDelta) + ms->config.rmHalfWidth; 
        // retrieve its value
        double mDenom = max(ms->config.graspMemoryTries[localIntThX + localIntThY*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*getLocalGraspGear(ms, ms->config.currentGraspGear)], 1.0);
        if ((localIntThX < ms->config.rmWidth) && (localIntThY < ms->config.rmWidth)) {

          // Thompson
          graspMemoryWeight = (ms->config.graspMemorySample[localIntThX + localIntThY*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*getLocalGraspGear(ms, ms->config.currentGraspGear)]) * -1;  

          graspMemoryBias = 0;
        } else {
          graspMemoryWeight = 0;
        }
      }

      //cout << "graspMemory Incorporation rx ry lthx lthy gmw: " << rx << " " << ry << " LL: " << localIntThX << " " << localIntThY << " " << graspMemoryWeight << endl;
      //cout << "  gmTargetX gmTargetY eval: " << ms->config.gmTargetX << " " << ms->config.gmTargetY << " " << ms->config.graspMemoryPicks[ms->config.gmTargetX + ms->config.gmTargetY*rmWidth + ms->config.rmWidth*ms->config.rmWidth*getLocalGraspGear(ms, ms->config.currentGraspGear)] << endl;
	    
      // ATTN 19
      int i = localIntThX + localIntThY * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*getLocalGraspGear(ms, ms->config.currentGraspGear);
      int maxedOutTries = isThisGraspMaxedOut(ms, i);

      //if (graspMemoryBias + graspMemoryWeight < minDepth) 
      if ((graspMemoryBias + graspMemoryWeight < minDepth) && !maxedOutTries) {
	minDepth = graspMemoryWeight;
	ms->config.maxX = rx;
	ms->config.maxY = ry;
	ms->config.localMaxX = localIntThX;
	ms->config.localMaxY = localIntThY;
	//ms->config.localMaxGG = ms->config.currentGraspGear;
	ms->config.localMaxGG = getLocalGraspGear(ms, ms->config.currentGraspGear);
	ms->config.maxD = graspMemoryWeight;
	//ms->config.maxGG = ms->config.currentGraspGear;
	ms->config.maxGG = getLocalGraspGear(ms, ms->config.currentGraspGear);
	ms->config.useContinuousGraspTransform = 1;
	//ms->config.useContinuousGraspTransform = 0;
      }
    }
  }
  cout << "non-cumulative maxX: " << ms->config.maxX << " maxY: " << ms->config.maxY <<  " maxD: " << ms->config.maxD << " maxGG: " << ms->config.maxGG << endl;
}

void selectMaxTargetThompsonContinuous2(shared_ptr<MachineState> ms, double minDepth) {
  // ATTN 2
  int maxSearchPadding = 3;
  //int maxSearchPadding = 4;

  for (int rx = maxSearchPadding; rx < ms->config.rmWidth-maxSearchPadding; rx++) {
    for (int ry = maxSearchPadding; ry < ms->config.rmWidth-maxSearchPadding; ry++) {

      // ATTN 5
      double graspMemoryWeight = 0.0;
      double graspMemoryBias = VERYBIGNUMBER;
      int localIntThX = -1; 
      int localIntThY = -1; 
      double localThX = 0.0;
      double localThY = 0.0;
      {
        // find local coordinate of current point
        double thX = (rx-ms->config.rmHalfWidth) * ms->config.rmDelta;
        double thY = (ry-ms->config.rmHalfWidth) * ms->config.rmDelta;
        // transform it into global coordinates
        double angle = ms->config.bestOrientationAngle;
        double unscale = 1.0;
        Point uncenter = Point(0, 0);
        Mat un_rot_mat = getRotationMatrix2D(uncenter, angle, unscale);
        Mat toUn(3,1,CV_64F);
        toUn.at<double>(0,0)=thX;
        toUn.at<double>(1,0)=thY;
        toUn.at<double>(2,0)=1.0;
        Mat didUn = un_rot_mat*toUn;
        localThX = didUn.at<double>(0,0);
        localThY = didUn.at<double>(1,0);
        localIntThX = ((localThX)/ms->config.rmDelta) + ms->config.rmHalfWidth; 
        localIntThY = ((localThY)/ms->config.rmDelta) + ms->config.rmHalfWidth; 
        // retrieve its value
        double mDenom = max(ms->config.graspMemoryTries[rx + ry*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*(ms->config.currentGraspGear)], 1.0);
        if ((rx < ms->config.rmWidth) && (ry < ms->config.rmWidth)) {

          // Thompson
          graspMemoryWeight = (ms->config.graspMemorySample[rx + ry*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*(ms->config.currentGraspGear)]) * -1;  

          graspMemoryBias = 0;
        } else {
          graspMemoryWeight = 0;
        }
      }


      //cout << "graspMemory Incorporation rx ry lthx lthy gmw: " << rx << " " << ry << " LL: " << localIntThX << " " << localIntThY << " " << graspMemoryWeight << endl;
      //cout << "  gmTargetX gmTargetY eval: " << ms->config.gmTargetX << " " << ms->config.gmTargetY << " " << ms->config.graspMemoryPicks[ms->config.gmTargetX + ms->config.gmTargetY*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*(ms->config.currentGraspGear)] << endl;
	    
      // ATTN 19
      int i = rx + ry * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*(ms->config.currentGraspGear);
      int maxedOutTries = isThisGraspMaxedOut(ms, i);

      //if (graspMemoryBias + graspMemoryWeight < minDepth) 
      if ((graspMemoryBias + graspMemoryWeight < minDepth) && !maxedOutTries) {
          minDepth = graspMemoryWeight;
          ms->config.maxX = localIntThX;
          ms->config.maxY = localIntThY;
          ms->config.localMaxX = rx;
          ms->config.localMaxY = ry;
          ms->config.localMaxGG = (ms->config.currentGraspGear);
          ms->config.maxD = graspMemoryWeight;
          //ms->config.maxGG = getGlobalGraspGear(ms, ms->config.currentGraspGear);
          ms->config.maxGG = (ms->config.currentGraspGear);
	  //ms->config.useContinuousGraspTransform = 0;
	  ms->config.useContinuousGraspTransform = 1;
	  cout << "ZZZ ZZZ ZZZ" << endl;
        }
    }
  }
  cout << "non-cumulative maxX: " << ms->config.maxX << " ms->config.maxY: " << ms->config.maxY <<  " maxD: " << 
    ms->config.maxD << " maxGG: " << ms->config.maxGG << " localMaxGG: " << ms->config.localMaxGG << endl;
}

void selectMaxTargetThompsonRotated(shared_ptr<MachineState> ms, double minDepth) {
  // ATTN 2
  int maxSearchPadding = 3;
  //int maxSearchPadding = 4;

  for (int rx = maxSearchPadding; rx < ms->config.rmWidth-maxSearchPadding; rx++) {
    for (int ry = maxSearchPadding; ry < ms->config.rmWidth-maxSearchPadding; ry++) {

      // ATTN 5
      double graspMemoryWeight = 0.0;
      double graspMemoryBias = VERYBIGNUMBER;
      int localIntThX = -1; 
      int localIntThY = -1; 
      {
        // find global coordinate of current point
        double thX = (rx-ms->config.rmHalfWidth) * ms->config.rmDelta;
        double thY = (ry-ms->config.rmHalfWidth) * ms->config.rmDelta;
        // transform it into local coordinates
        double unangle = -ms->config.bestOrientationAngle;
        double unscale = 1.0;
        Point uncenter = Point(0, 0);
        Mat un_rot_mat = getRotationMatrix2D(uncenter, unangle, unscale);
        Mat toUn(3,1,CV_64F);
        toUn.at<double>(0,0)=thX;
        toUn.at<double>(1,0)=thY;
        toUn.at<double>(2,0)=1.0;
        Mat didUn = un_rot_mat*toUn;
        double localThX = didUn.at<double>(0,0);
        double localThY = didUn.at<double>(1,0);
        localIntThX = ((localThX)/ms->config.rmDelta) + ms->config.rmHalfWidth; 
        localIntThY = ((localThY)/ms->config.rmDelta) + ms->config.rmHalfWidth; 
        // retrieve its value
        double mDenom = max(ms->config.graspMemoryTries[localIntThX + localIntThY*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*getLocalGraspGear(ms, ms->config.currentGraspGear)], 1.0);
        if ((localIntThX < ms->config.rmWidth) && (localIntThY < ms->config.rmWidth)) {

          // Thompson
          graspMemoryWeight = (ms->config.graspMemorySample[localIntThX + localIntThY*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*getLocalGraspGear(ms, ms->config.currentGraspGear)]) * -1;  

          // Original
          //graspMemoryWeight = ms->config.graspMemoryPicks[localIntThX + localIntThY*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*getLocalGraspGear(ms, ms->config.currentGraspGear)] / mDenom;
          //graspMemoryWeight = graspMemoryWeight * ms->config.rangeMapReg1[rx + ry*ms->config.rmWidth]);  
           
          // No memory; just linear filter
          // graspMemoryWeight = ms->config.rangeMapReg1[rx + ry*ms->config.rmWidth];
          graspMemoryBias = 0;
        } else {
          graspMemoryWeight = 0;
        }
      }


      //cout << "graspMemory Incorporation rx ry lthx lthy gmw: " << rx << " " << ry << " LL: " << localIntThX << " " << localIntThY << " " << graspMemoryWeight << endl;
      //cout << "  gmTargetX gmTargetY eval: " << ms->config.gmTargetX << " " << ms->config.gmTargetY << " " << ms->config.graspMemoryPicks[ms->config.gmTargetX + ms->config.gmTargetY*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*getLocalGraspGear(ms, ms->config.currentGraspGear)] << endl;
	    
      // ATTN 19
      int i = localIntThX + localIntThY * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*getLocalGraspGear(ms, ms->config.currentGraspGear);
      int maxedOutTries = isThisGraspMaxedOut(ms, i);

      //if (graspMemoryBias + graspMemoryWeight < minDepth) 
      if ((graspMemoryBias + graspMemoryWeight < minDepth) && !maxedOutTries) {
          minDepth = graspMemoryWeight;
          ms->config.maxX = rx;
          ms->config.maxY = ry;
          ms->config.localMaxX = localIntThX;
          ms->config.localMaxY = localIntThY;
          ms->config.localMaxGG = getLocalGraspGear(ms, ms->config.currentGraspGear);
          ms->config.maxD = graspMemoryWeight;
          ms->config.maxGG = ms->config.currentGraspGear;
	  ms->config.useContinuousGraspTransform = 0;
        }
    }
  }
  cout << "non-cumulative maxX: " << ms->config.maxX << " maxY: " << ms->config.maxY <<  " maxD: " << ms->config.maxD << " maxGG: " << ms->config.maxGG << endl;
}

void selectMaxTargetThompsonRotated2(shared_ptr<MachineState> ms, double minDepth) {
  // ATTN 2
  int maxSearchPadding = 3;
  //int maxSearchPadding = 4;

  for (int rx = maxSearchPadding; rx < ms->config.rmWidth-maxSearchPadding; rx++) {
    for (int ry = maxSearchPadding; ry < ms->config.rmWidth-maxSearchPadding; ry++) {

      // ATTN 5
      double graspMemoryWeight = 0.0;
      double graspMemoryBias = VERYBIGNUMBER;
      int localIntThX = -1; 
      int localIntThY = -1; 
      double localThX = 0.0;
      double localThY = 0.0;
      {
        // find local coordinate of current point
        double thX = (rx-ms->config.rmHalfWidth) * ms->config.rmDelta;
        double thY = (ry-ms->config.rmHalfWidth) * ms->config.rmDelta;
        // transform it into global coordinates
        double angle = ms->config.bestOrientationAngle;
        double unscale = 1.0;
        Point uncenter = Point(0, 0);
        Mat un_rot_mat = getRotationMatrix2D(uncenter, angle, unscale);
        Mat toUn(3,1,CV_64F);
        toUn.at<double>(0,0)=thX;
        toUn.at<double>(1,0)=thY;
        toUn.at<double>(2,0)=1.0;
        Mat didUn = un_rot_mat*toUn;
        localThX = didUn.at<double>(0,0);
        localThY = didUn.at<double>(1,0);
        localIntThX = ((localThX)/ms->config.rmDelta) + ms->config.rmHalfWidth; 
        localIntThY = ((localThY)/ms->config.rmDelta) + ms->config.rmHalfWidth; 
        // retrieve its value
        double mDenom = max(ms->config.graspMemoryTries[rx + ry*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*(ms->config.currentGraspGear)], 1.0);
        if ((rx < ms->config.rmWidth) && (ry < ms->config.rmWidth)) {

          // Thompson
          graspMemoryWeight = (ms->config.graspMemorySample[rx + ry*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*(ms->config.currentGraspGear)]) * -1;  

          graspMemoryBias = 0;
        } else {
          graspMemoryWeight = 0;
        }
      }


      //cout << "graspMemory Incorporation rx ry lthx lthy gmw: " << rx << " " << ry << " LL: " << localIntThX << " " << localIntThY << " " << graspMemoryWeight << endl;
      //cout << "  gmTargetX gmTargetY eval: " << ms->config.gmTargetX << " " << ms->config.gmTargetY << " " << ms->config.graspMemoryPicks[ms->config.gmTargetX + ms->config.gmTargetY*ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*(ms->config.currentGraspGear)] << endl;
	    
      // ATTN 19
      int i = rx + ry * ms->config.rmWidth + ms->config.rmWidth*ms->config.rmWidth*(ms->config.currentGraspGear);
      int maxedOutTries = isThisGraspMaxedOut(ms, i);

      //if (graspMemoryBias + graspMemoryWeight < minDepth) 
      if ((graspMemoryBias + graspMemoryWeight < minDepth) && !maxedOutTries) {
          minDepth = graspMemoryWeight;
          ms->config.maxX = localIntThX;
          ms->config.maxY = localIntThY;
          ms->config.localMaxX = rx;
          ms->config.localMaxY = ry;
          ms->config.localMaxGG = (ms->config.currentGraspGear);
          ms->config.maxD = graspMemoryWeight;
          ms->config.maxGG = getGlobalGraspGear(ms, ms->config.currentGraspGear);
	  ms->config.useContinuousGraspTransform = 0;
        }
    }
  }
  cout << "non-cumulative maxX: " << ms->config.maxX << " maxY: " << ms->config.maxY <<  " maxD: " << ms->config.maxD << " maxGG: " << ms->config.maxGG << endl;
}


void recordBoundingBoxSuccess(shared_ptr<MachineState> ms) {
  ms->config.heightMemoryTries[ms->config.currentThompsonHeightIdx]++;
  ms->config.heightMemoryPicks[ms->config.currentThompsonHeightIdx]++;
  ms->config.heightSuccessCounter++;
  ms->config.heightAttemptCounter++;
  cout << "Successful bounding box on floor " << ms->config.currentThompsonHeightIdx << endl;
  cout << "Tries: " << ms->config.heightMemoryTries[ms->config.currentThompsonHeightIdx] << endl;
  cout << "Picks: " << ms->config.heightMemoryPicks[ms->config.currentThompsonHeightIdx] << endl;
  int ttotalTries = 0;
  int ttotalPicks = 0;
  for (int i = 0; i < ms->config.hmWidth; i++) {
    ttotalTries += ms->config.heightMemoryTries[i];
    ttotalPicks += ms->config.heightMemoryPicks[i];
  }
  cout << "Total Tries: " << ttotalTries << endl;
  cout << "Total Picks: " << ttotalPicks << endl;

  double thisPickRate = double(ms->config.heightMemoryPicks[ms->config.currentThompsonHeightIdx]) / double(ms->config.heightMemoryTries[ms->config.currentThompsonHeightIdx]);
  int thisNumTries = ms->config.heightMemoryTries[ms->config.currentThompsonHeightIdx];
  cout << "Thompson Early Out: thisPickrate = " << thisPickRate << ", thisNumTries = " << thisNumTries << endl;
  if (ms->config.currentBoundingBoxMode == LEARNING_SAMPLING) {
    if ( (thisNumTries >= ms->config.thompsonMinTryCutoff) && 
	 (thisPickRate >= ms->config.thompsonMinPassRate) ) {
      ms->config.thompsonHeightHaltFlag = 1;
    }
  }

  // ATTN 20
  {
    double successes = ms->config.heightMemoryPicks[ms->config.currentThompsonHeightIdx];
    double failures =  ms->config.heightMemoryTries[ms->config.currentThompsonHeightIdx] - ms->config.heightMemoryPicks[ms->config.currentThompsonHeightIdx];
    // returns probability that mu <= d given successes and failures.
    double presult = cephes_incbet(successes + 1, failures + 1, ms->config.algorithmCTarget);
    // we want probability that mu > d
    double result = 1.0 - presult;

    double presult2a = cephes_incbet(successes + 1, failures + 1, ms->config.algorithmCTarget + ms->config.algorithmCEPS);
    double presult2b = cephes_incbet(successes + 1, failures + 1, ms->config.algorithmCTarget - ms->config.algorithmCEPS);
    // we want probability that 
    //  ms->config.algorithmCTarget - ms->config.algorithmCEPS < mu < ms->config.algorithmCTarget + ms->config.algorithmCEPS
    double result2 = presult2a - presult2b;

    cout << "prob that mu > d: " << result << " algorithmCAT: " << ms->config.algorithmCAT << endl;
    if (ms->config.currentBoundingBoxMode == LEARNING_ALGORITHMC) {
      ms->config.thompsonHeightHaltFlag = (result > ms->config.algorithmCAT);
      if (result2 > ms->config.algorithmCAT) {
	ms->config.thompsonHeightHaltFlag = 1;
      }
    }
  }
}

void recordBoundingBoxFailure(shared_ptr<MachineState> ms) {
  ms->config.heightMemoryTries[ms->config.currentThompsonHeightIdx]++;
  ms->config.heightAttemptCounter++;
  cout << "Failed to learn bounding box on floor " << ms->config.currentThompsonHeightIdx << endl;
  cout << "Tries: " << ms->config.heightMemoryTries[ms->config.currentThompsonHeightIdx] << endl;
  cout << "Picks: " << ms->config.heightMemoryPicks[ms->config.currentThompsonHeightIdx] << endl;
  int ttotalTries = 0;
  int ttotalPicks = 0;
  for (int i = 0; i < ms->config.hmWidth; i++) {
    ttotalTries += ms->config.heightMemoryTries[i];
    ttotalPicks += ms->config.heightMemoryPicks[i];
  }
  cout << "Total Tries: " << ttotalTries << endl;
  cout << "Total Picks: " << ttotalPicks << endl;
}

void restartBBLearning(shared_ptr<MachineState> ms) {
  recordBoundingBoxFailure(ms);
  ms->clearStack();
  ms->pushWord("continueHeightLearning"); // continue bounding box learning
}


void moveCurrentGripperRayToCameraVanishingRay(shared_ptr<MachineState> ms) {
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
    pixelToGlobal(ms, ms->config.vanishingPointReticle.px, ms->config.vanishingPointReticle.py, zToUse, &(ms->config.currentEEPose.px), &(ms->config.currentEEPose.py));
  }
  { // yet another way to do this
    // 0 assumes no rotation 
    //eePose finalGlobalTarget = analyticServoPixelToReticle(ms->config.pilotTarget, thisGripperReticle, 0);
    //ms->config.currentEEPose.px = finalGlobalTarget.px;
    //ms->config.currentEEPose.py = finalGlobalTarget.py;
  }
}

void gradientServo(shared_ptr<MachineState> ms) {
  Size sz = ms->config.objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;

  // ATTN 23
  //reticle = ms->config.heightReticles[ms->config.currentThompsonHeightIdx];
  eePose thisGripperReticle;
  double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;
  int xOut=-1, yOut=-1;
  globalToPixel(ms, &xOut, &yOut, zToUse, ms->config.trueEEPoseEEPose.px, ms->config.trueEEPoseEEPose.py);
  thisGripperReticle.px = xOut;
  thisGripperReticle.py = yOut;
  ms->config.reticle = ms->config.vanishingPointReticle;

  // ATTN 12
  //        if ((ms->config.synServoLockFrames > ms->config.heightLearningServoTimeout) && (ms->config.currentBoundingBoxMode == LEARNING_SAMPLING)) {
  //          cout << "bbLearning: synchronic servo timed out, early outting." << endl;
  //          restartBBLearning(ms);
  //        }

  cout << "entered gradient servo... iteration " << ms->config.currentGradientServoIterations << endl;
  if (ms->config.targetClass < 0 || ms->config.targetClass >= ms->config.numClasses) {
    cout << "bad target class, not servoing." << endl;
    return;
  }
  if ((ms->config.classAerialGradients[ms->config.targetClass].rows <= 1) && (ms->config.classAerialGradients[ms->config.targetClass].cols <= 1)) {
    cout << "no aerial gradients for this class, not servoing." << endl;
    return;
  }

  {
    int i, j;
    mapxyToij(ms, ms->config.currentEEPose.px, ms->config.currentEEPose.py, &i, &j);
    int doWeHaveClearance = (ms->config.clearanceMap[i + ms->config.mapWidth * j] != 0);
    if (!doWeHaveClearance) {
      //ms->pushWord("clearStackIntoMappingPatrol"); 
      cout << ">>>> Gradient servo strayed out of clearance area during mapping. <<<<" << endl;
      ms->pushWord("endStackCollapseNoop");
      return;
    }
  }

  // ATTN 16
  switch (ms->config.currentThompsonHeightIdx) {
  case 0:
    {
      ms->config.classAerialGradients[ms->config.targetClass] = ms->config.classHeight0AerialGradients[ms->config.targetClass];
    }
    break;
  case 1:
    {
      ms->config.classAerialGradients[ms->config.targetClass] = ms->config.classHeight1AerialGradients[ms->config.targetClass];
    }
    break;
  case 2:
    {
      ms->config.classAerialGradients[ms->config.targetClass] = ms->config.classHeight2AerialGradients[ms->config.targetClass];
    }
    break;
  case 3:
    {
      ms->config.classAerialGradients[ms->config.targetClass] = ms->config.classHeight3AerialGradients[ms->config.targetClass];
    }
    break;
  default:
    {
      assert(0);
    }
    break;
  }

  double Px = 0;
  double Py = 0;

  double Ps = 0;

  //cout << "computing scores... ";

  Size toBecome(ms->config.aerialGradientWidth, ms->config.aerialGradientWidth);

  int numOrientations = 37;
  vector<Mat> rotatedAerialGrads;

  // ATTN 3
  // gradientServoScale should be even
  int gradientServoScale = 3;//11;
  double gradientServoScaleStep = 1.02;
  if (ms->config.orientationCascade) {
    if (ms->config.lastPtheta < ms->config.lPTthresh) {
      //gradientServoScale = 1;
      //gradientServoScaleStep = 1.0;
    }
  }
  double startScale = pow(gradientServoScaleStep, -(gradientServoScale-1)/2);

  //rotatedAerialGrads.resize(numOrientations);
  rotatedAerialGrads.resize(gradientServoScale*numOrientations);

  if ((ms->config.lastPtheta < ms->config.lPTthresh) && ms->config.orientationCascade) {
    cout << "orientation cascade activated" << endl;
  }

  for (int etaS = 0; etaS < gradientServoScale; etaS++) {
    double thisScale = startScale * pow(gradientServoScaleStep, etaS);
    for (int thisOrient = 0; thisOrient < numOrientations; thisOrient++) {
      // orientation cascade
      if (ms->config.orientationCascade) {
        if (ms->config.lastPtheta < ms->config.lPTthresh) {
          if (thisOrient < ms->config.orientationCascadeHalfWidth) {
            //cout << "skipping orientation " << thisOrient << endl;
            continue;
          }
          if (thisOrient > numOrientations - ms->config.orientationCascadeHalfWidth) {
            //cout << "skipping orientation " << thisOrient << endl;
            continue;
          }
        }
      }
      
      // rotate the template and L1 normalize it
      Point center = Point(ms->config.aerialGradientWidth/2, ms->config.aerialGradientWidth/2);
      double angle = thisOrient*360.0/numOrientations;
      
      //double scale = 1.0;
      double scale = thisScale;
      
      // Get the rotation matrix with the specifications above
      Mat rot_mat = getRotationMatrix2D(center, angle, scale);
      warpAffine(ms->config.classAerialGradients[ms->config.targetClass], rotatedAerialGrads[thisOrient + etaS*numOrientations], rot_mat, toBecome);
      
      processSaliency(rotatedAerialGrads[thisOrient + etaS*numOrientations], rotatedAerialGrads[thisOrient + etaS*numOrientations]);
      
      //double l1norm = rotatedAerialGrads[thisOrient + etaS*numOrientations].dot(Mat::ones(ms->config.aerialGradientWidth, ms->config.aerialGradientWidth, rotatedAerialGrads[thisOrient + etaS*numOrientations].type()));
      //if (l1norm <= EPSILON)
      //l1norm = 1.0;
      //rotatedAerialGrads[thisOrient + etaS*numOrientations] = rotatedAerialGrads[thisOrient + etaS*numOrientations] / l1norm;
      //cout << "classOrientedGradients[ms->config.targetClass]: " << ms->config.classAerialGradients[ms->config.targetClass] << "rotatedAerialGrads[thisOrient + etaS*numOrientations] " << rotatedAerialGrads[thisOrient + etaS*numOrientations] << endl;
      
      double mean = rotatedAerialGrads[thisOrient + etaS*numOrientations].dot(Mat::ones(ms->config.aerialGradientWidth, ms->config.aerialGradientWidth, rotatedAerialGrads[thisOrient + etaS*numOrientations].type())) / double(ms->config.aerialGradientWidth*ms->config.aerialGradientWidth);
      rotatedAerialGrads[thisOrient + etaS*numOrientations] = rotatedAerialGrads[thisOrient + etaS*numOrientations] - mean;
      double l2norm = rotatedAerialGrads[thisOrient + etaS*numOrientations].dot(rotatedAerialGrads[thisOrient + etaS*numOrientations]);
      if (l2norm <= EPSILON)
        l2norm = 1.0;
      rotatedAerialGrads[thisOrient + etaS*numOrientations] = rotatedAerialGrads[thisOrient + etaS*numOrientations] / l2norm;
    }
  }

  int bestOrientation = -1;
  double bestOrientationScore = -INFINITY;
  double bestCropNorm = 1.0;
  int bestX = -1;
  int bestY = -1;
  int bestS = -1;

  int crows = ms->config.aerialGradientReticleWidth;
  int ccols = ms->config.aerialGradientReticleWidth;
  int maxDim = max(crows, ccols);
  int tRy = (maxDim-crows)/2;
  int tRx = (maxDim-ccols)/2;

  //int gradientServoTranslation = 40;
  //int gsStride = 2;
  int gradientServoTranslation = 40;
  int gsStride = 2;
  if (ms->config.orientationCascade) {
    if (ms->config.lastPtheta < ms->config.lPTthresh) {
      //int gradientServoTranslation = 20;
      //int gsStride = 2;
      int gradientServoTranslation = 40;
      int gsStride = 2;
    }
  }
  
  //rotatedAerialGrads.resize(gradientServoScale*numOrientations);
  int gSTwidth = 2*gradientServoTranslation + 1;
  double allScores[gSTwidth][gSTwidth][gradientServoScale][numOrientations];
  
  for (int etaS = 0; etaS < gradientServoScale; etaS++) {
#pragma omp parallel for
    for (int etaY = -gradientServoTranslation; etaY < gradientServoTranslation; etaY += gsStride) {
      for (int etaX = -gradientServoTranslation; etaX < gradientServoTranslation; etaX += gsStride) {
        // get the patch
        int topCornerX = etaX + ms->config.reticle.px - (ms->config.aerialGradientReticleWidth/2);
        int topCornerY = etaY + ms->config.reticle.py - (ms->config.aerialGradientReticleWidth/2);
        Mat gCrop(maxDim, maxDim, CV_64F);
        
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
          // ATTN 17
          // removed normalization for discriminative servoing
          // ATTN 15
          // normalization hoses rejection
          if (ms->config.useGradientServoThresh) {
          } else {
            if (l2norm <= EPSILON)
              l2norm = 1.0;
            gCrop = gCrop / l2norm;
          }
        }
        
        for (int thisOrient = 0; thisOrient < numOrientations; thisOrient++) {
          // orientation cascade
          if (ms->config.orientationCascade) {
            if (ms->config.lastPtheta < ms->config.lPTthresh) {
              if (thisOrient < ms->config.orientationCascadeHalfWidth) {
                //cout << "skipping orientation " << thisOrient << endl;
                continue;
              }
              if (thisOrient > numOrientations - ms->config.orientationCascadeHalfWidth) {
                //cout << "skipping orientation " << thisOrient << endl;
                continue;
              }
            }
          }
  // ATTN 25
  if ( (ms->config.currentGradientServoIterations > (ms->config.softMaxGradientServoIterations-1)) &&
       (thisOrient != 0) ) {
    continue;
  }
          
          // compute the score
          double thisScore = 0;
          thisScore = rotatedAerialGrads[thisOrient + etaS*numOrientations].dot(gCrop);
          
          int tEtaX = etaX+gradientServoTranslation;
          int tEtaY = etaY+gradientServoTranslation;
          allScores[tEtaX][tEtaY][etaS][thisOrient] = thisScore;
        }
      }
    }
  }
  
  // perform max
  for (int etaS = 0; etaS < gradientServoScale; etaS++) {
    for (int etaY = -gradientServoTranslation; etaY < gradientServoTranslation; etaY += gsStride) {
      for (int etaX = -gradientServoTranslation; etaX < gradientServoTranslation; etaX += gsStride) {
        // get the patch
        int topCornerX = etaX + ms->config.reticle.px - (ms->config.aerialGradientReticleWidth/2);
        int topCornerY = etaY + ms->config.reticle.py - (ms->config.aerialGradientReticleWidth/2);
        Mat gCrop(maxDim, maxDim, CV_64F);
        
        // throw it out if it isn't contained in the image
        //    if ( (topCornerX+ms->config.aerialGradientWidth >= imW) || (topCornerY+ms->config.aerialGradientWidth >= imH) )
        //      continue;
        //    if ( (topCornerX < 0) || (topCornerY < 0) )
        //      continue;
        
        for (int thisOrient = 0; thisOrient < numOrientations; thisOrient++) {
          // orientation cascade
          if (ms->config.orientationCascade) {
            if (ms->config.lastPtheta < ms->config.lPTthresh) {
              if (thisOrient < ms->config.orientationCascadeHalfWidth) {
                //cout << "skipping orientation " << thisOrient << endl;
                continue;
              }
              if (thisOrient > numOrientations - ms->config.orientationCascadeHalfWidth) {
                //cout << "skipping orientation " << thisOrient << endl;
                continue;
              }
            }
          }
  // ATTN 25
  if ( (ms->config.currentGradientServoIterations > (ms->config.softMaxGradientServoIterations-1)) &&
       (thisOrient != 0) ) {
    continue;
  }
          
          int tEtaX = etaX+gradientServoTranslation;
          int tEtaY = etaY+gradientServoTranslation;
          double thisScore = allScores[tEtaX][tEtaY][etaS][thisOrient];
          
          if (thisScore > bestOrientationScore) {
            bestOrientation = thisOrient;
            bestOrientationScore = thisScore;
            bestCropNorm = sqrt(gCrop.dot(gCrop));
            bestX = etaX;
            bestY = etaY;
            bestS = etaS;
          }
          //cout << " this best: " << thisScore << " " << bestOrientationScore << " " << bestX << " " << bestY << endl;
        } 
      }
    }
  }

  // set the target reticle
  ms->config.pilotTarget.px = ms->config.reticle.px + bestX;
  ms->config.pilotTarget.py = ms->config.reticle.py + bestY;
  
  ms->config.bestOrientationEEPose = ms->config.currentEEPose;
  
  int oneToDraw = bestOrientation;
  Px = -bestX;
  Py = -bestY;
  
  //Ps = bestS - ((gradientServoScale-1)/2);
  Ps = 0;
  
  Mat toShow;
  Size toUnBecome(maxDim, maxDim);
  //cv::resize(ms->config.classAerialGradients[ms->config.targetClass], toShow, toUnBecome);
  //cv::resize(rotatedAerialGrads[oneToDraw], toShow, toUnBecome);
  cv::resize(rotatedAerialGrads[bestOrientation + bestS*numOrientations], toShow, toUnBecome);
  //cout << rotatedAerialGrads[oneToDraw];
  
  double maxTS = -INFINITY;
  double minTS = INFINITY;
  for (int x = 0; x < maxDim; x++) {
    for (int y = 0; y < maxDim; y++) {
      maxTS = max(maxTS, toShow.at<double>(y, x));
      minTS = min(minTS, toShow.at<double>(y, x));
    }
  }
  
  // draw the winning score in place
  for (int x = 0; x < maxDim; x++) {
    for (int y = 0; y < maxDim; y++) {
      //int tx = x - tRx;
      //int ty = y - tRy;
      int tx = x - tRx;
      int ty = y - tRy;
      if (tx >= 0 && ty >= 0 && ty < crows && tx < ccols) {
        Vec3b thisColor = Vec3b(0,0,min(255, int(floor(255.0*8*(toShow.at<double>(y, x)-minTS)/(maxTS-minTS)))));
        //Vec3b thisColor = Vec3b(0,0,min(255, int(floor(100000*toShow.at<double>(y, x)))));
        //Vec3b thisColor = Vec3b(0,0,min(255, int(floor(0.2*sqrt(toShow.at<double>(y, x))))));
        //cout << thisColor;
        int thisTopCornerX = bestX + ms->config.reticle.px - (ms->config.aerialGradientReticleWidth/2);
        int thisTopCornerY = bestY + ms->config.reticle.py - (ms->config.aerialGradientReticleWidth/2);
        
        int tgX = thisTopCornerX + tx;
        int tgY = thisTopCornerY + ty;
        if ((tgX > 0) && (tgX < imW) && (tgY > 0) && (tgY < imH)) {
          ms->config.gradientViewerImage.at<Vec3b>(tgY, tgX) += thisColor;
        }
      }
    }
  }
  
  oneToDraw = oneToDraw % numOrientations;
  double Ptheta = min(bestOrientation, numOrientations - bestOrientation);
  ms->config.lastPtheta = Ptheta;
  
  
  // update after
  ms->config.currentGradientServoIterations++;
  
  // XXX this still might miss if it nails the correct orientation on the last try
  // TODO but we could set the bestOrientationEEPose here according to what it would have been`
  // but we don't want to move because we want all the numbers to be consistent
  if (ms->config.currentGradientServoIterations > (ms->config.hardMaxGradientServoIterations-1)) {
    //cout << "LAST ITERATION indefinite orientation ";
  } else {
    double kPtheta = 0.0;
    if (Ptheta < ms->config.kPThresh)
      kPtheta = ms->config.kPtheta2;
    else
      kPtheta = ms->config.kPtheta1;
    
    if (bestOrientation <= numOrientations/2) {
      ms->config.currentEEDeltaRPY.pz -= kPtheta * bestOrientation*2.0*3.1415926/double(numOrientations);
      
    } else {
      ms->config.currentEEDeltaRPY.pz -= kPtheta * (-(numOrientations - bestOrientation))*2.0*3.1415926/double(numOrientations);
    }
  }
  
  double doublePtheta =   ms->config.currentEEDeltaRPY.pz;

  //cout << "gradient servo Px Py Ps bestOrientation Ptheta doublePtheta: " << Px << " " << Py << " " << Ps << " : " << reticle.px << " " << 
  //ms->config.pilotTarget.px << " " << reticle.py << " " << ms->config.pilotTarget.py << " " <<
  //bestOrientation << " " << Ptheta << " " << doublePtheta << endl;
  
  double dx = (ms->config.currentEEPose.px - ms->config.trueEEPose.position.x);
  double dy = (ms->config.currentEEPose.py - ms->config.trueEEPose.position.y);
  double dz = (ms->config.currentEEPose.pz - ms->config.trueEEPose.position.z);
  double distance = dx*dx + dy*dy + dz*dz;
  
  // ATTN 15
  // return to synchronic if the match is poor
  if (ms->config.useGradientServoThresh) {
    cout << "ATTN score, thresh, norm, product: " << bestOrientationScore << " " << ms->config.gradientServoResetThresh << " " << bestCropNorm << " " << (ms->config.gradientServoResetThresh * bestCropNorm) << endl;
    if (bestOrientationScore < (ms->config.gradientServoResetThresh * bestCropNorm) ) {
      ms->pushWord("synchronicServo"); 
      cout << " XXX BAD GRADIENT SERVO SCORE, RETURN TO SYNCHRONIC XXX" << endl;
      return;
    }
  }

  if (distance > ms->config.w1GoThresh*ms->config.w1GoThresh) {
    
    ms->pushWord("gradientServo"); 
    // ATTN 8
    //ms->pushCopies("density", ms->config.densityIterationsForGradientServo); 
    //ms->pushCopies("accumulateDensity", ms->config.densityIterationsForGradientServo); 
    //ms->pushCopies("resetTemporalMap", 1); 
    //ms->pushWord("resetAerialGradientTemporalFrameAverage"); 
    //ms->pushCopies("density", 1); 
    //ms->pushCopies("waitUntilAtCurrentPosition", 5); 
    cout << " XXX deprecated code path, gradient servo should not be responsible for enforcing distance 73825" << endl;
    ms->pushCopies("waitUntilAtCurrentPosition", 1); 
    
  } else {
    // ATTN 5
    // cannot proceed unless Ptheta = 0, since our best eePose is determined by our current pose and not where we WILL be after adjustment
    if (((fabs(Px) < ms->config.gradServoPixelThresh) && (fabs(Py) < ms->config.gradServoPixelThresh) && (fabs(Ptheta) < ms->config.gradServoThetaThresh)) ||
        (ms->config.currentGradientServoIterations > (ms->config.hardMaxGradientServoIterations-1)))
      {
	// ATTN 23
	// move from vanishing point reticle to gripper reticle
	//moveCurrentGripperRayToCameraVanishingRay();
	//ms->config.bestOrientationEEPose = ms->config.currentEEPose;

        // ATTN 12
        if (ARE_GENERIC_HEIGHT_LEARNING(ms)) {
          cout << "bbLearning: gradient servo succeeded. gradientServoDuringHeightLearning: " << ms->config.gradientServoDuringHeightLearning << endl;
          cout << "bbLearning: returning from gradient servo." << endl;
          return;
        }
        
        // ATTN 17
        if (ms->config.bailAfterGradient) {
          cout << "gradient servo set to bail. returning." << endl;
          return;
        }
        
        //cout << "got within thresh, returning." << endl;
        cout << "got within thresh, fetching." << endl;
        ms->config.lastPtheta = INFINITY;
        cout << "resetting lastPtheta: " << ms->config.lastPtheta << endl;
    
        if (ms->config.synchronicTakeClosest) {
          if (ms->config.gradientTakeClosest) {
            if ((ms->config.classRangeMaps[ms->config.targetClass].rows > 1) && (ms->config.classRangeMaps[ms->config.targetClass].cols > 1))
              ms->pushWord("prepareForAndExecuteGraspFromMemoryLearning"); // prepare for and execute the best grasp from memory at the current location and target
            else {
              ROS_ERROR_STREAM("Cannot pick object with incomplete map.");
            }
          } else {
            return;
          }
        } else {
          if ((ms->config.classRangeMaps[ms->config.targetClass].rows > 1) && (ms->config.classRangeMaps[ms->config.targetClass].cols > 1)) {
            ms->pushWord("prepareForAndExecuteGraspFromMemoryLearning"); 
          } else {
            ROS_ERROR_STREAM("Cannot pick object with incomplete map.");
          }
        }
        
        return;
      } else {
      
      ms->pushWord("gradientServo"); 
      
      double pTermX = ms->config.gradKp*Px;
      double pTermY = ms->config.gradKp*Py;
      
      double pTermS = Ps * .005;
      ms->config.currentEEPose.pz += pTermS;
      
      // invert the current eePose orientation to decide which direction to move from POV
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
    
    // ATTN 21
    //double newx = ms->config.currentEEPose.px + pTermX*localUnitY.x() - pTermY*localUnitX.x();
    //double newy = ms->config.currentEEPose.py + pTermX*localUnitY.y() - pTermY*localUnitX.y();
    double newx = 0;
    double newy = 0;
    // first analytic
    //double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;
    //pixelToGlobal(ms->config.pilotTarget.px, ms->config.pilotTarget.py, zToUse, &newx, &newy);
    // old PID
    //ms->config.currentEEPose.py += pTermX*localUnitY.y() - pTermY*localUnitX.y();
    //ms->config.currentEEPose.px += pTermX*localUnitY.x() - pTermY*localUnitX.x();
    // ATTN 23
    // second analytic
    eePose newGlobalTarget = analyticServoPixelToReticle(ms, ms->config.pilotTarget, ms->config.reticle, ms->config.currentEEDeltaRPY.pz);
    newx = newGlobalTarget.px;
    newy = newGlobalTarget.py;
    ms->config.currentEEPose.px = newx;
    ms->config.currentEEPose.py = newy;
    
    // ATTN 8
    //ms->pushWord("visionCycle"); // vision cycle
    //ms->pushWord(196721); // vision cycle no classify
    //ms->pushCopies("density", ms->config.densityIterationsForGradientServo); // density
    //ms->pushCopies("accumulateDensity", ms->config.densityIterationsForGradientServo); 
    //ms->pushCopies("resetTemporalMap", 1); // reset temporal map
    //ms->pushWord("resetAerialGradientTemporalFrameAverage"); // reset aerialGradientTemporalFrameAverage
    //ms->pushCopies("density", 1); // density
    //ms->pushWord("waitUntilAtCurrentPosition"); 
    
    // ATTN 7
    // if you don't wait multiple times, it could get triggered early by weird ik or latency could cause a loop
    // this is a very aggressive choice and we should also be using the ring buffers for Ode calls
    //ms->pushCopies("waitUntilAtCurrentPosition", 40); 
    //ms->pushCopies("waitUntilAtCurrentPosition", 5); 
    ms->pushCopies("waitUntilAtCurrentPosition", 1); 
    
    // ATTN 16
    //	    { // prepare to servo
    //	      ms->config.currentEEPose.pz = wholeFoodsCounter1.pz+.1;
    //	    }
    }
  }
}

// given pixel is the pixel in the current frame that you want to be at the vanishing point
//  after undergoing a rotaion of ozAngle about the end effector Z axis
eePose analyticServoPixelToReticle(shared_ptr<MachineState> ms, eePose givenPixel, eePose givenReticle, double ozAngle) {
  eePose toReturn = ms->config.trueEEPoseEEPose;
  eePose grGlobalPostRotation = ms->config.trueEEPoseEEPose;
  eePose grGlobalPreRotation = ms->config.trueEEPoseEEPose;
  eePose gpGlobalPreRotation = ms->config.trueEEPoseEEPose;
  {
    double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;
    pixelToGlobal(ms, givenPixel.px, givenPixel.py, zToUse, &(gpGlobalPreRotation.px), &(gpGlobalPreRotation.py));
  }
  {
    double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;
    pixelToGlobal(ms, givenReticle.px, givenReticle.py, zToUse, &(grGlobalPreRotation.px), &(grGlobalPreRotation.py));
  }

  eePose fakeEndEffector = ms->config.currentEEPose;
  eePose fakeEndEffectorDeltaRPY = eePose::zero();
  fakeEndEffectorDeltaRPY.pz = ozAngle;
  endEffectorAngularUpdate(&fakeEndEffector, &fakeEndEffectorDeltaRPY);
  {
    double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;
    pixelToGlobal(ms, givenReticle.px, givenReticle.py, zToUse, &(grGlobalPostRotation.px), &(grGlobalPostRotation.py), fakeEndEffector);
  }
  double  postRotationTranslationX = (gpGlobalPreRotation.px - grGlobalPostRotation.px);
  double  postRotationTranslationY = (gpGlobalPreRotation.py - grGlobalPostRotation.py);

  toReturn.px += postRotationTranslationX;
  toReturn.py += postRotationTranslationY;
  return toReturn;
}

void synchronicServo(shared_ptr<MachineState> ms) {
  ROS_WARN_STREAM("___________________ Synchronic Servo");
  ms->config.synServoLockFrames++;

  // ATTN 23
  //reticle = ms->config.heightReticles[ms->config.currentThompsonHeightIdx];
  eePose thisGripperReticle;
  double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;
  int xOut=-1, yOut=-1;
  globalToPixel(ms, &xOut, &yOut, zToUse, ms->config.trueEEPoseEEPose.px, ms->config.trueEEPoseEEPose.py);
  thisGripperReticle.px = xOut;
  thisGripperReticle.py = yOut;
  ms->config.reticle = ms->config.vanishingPointReticle;

  // ATTN 17
  ms->config.currentGradientServoIterations = 0;

  // ATTN 12
  // if we time out, reset the bblearning program
  if ( ((ms->config.synServoLockFrames > ms->config.heightLearningServoTimeout) || (ms->config.bTops.size() <= 0)) && 
	(ARE_GENERIC_HEIGHT_LEARNING(ms)) ) {
    cout << "bbLearning: synchronic servo early outting: ";
    if (ms->config.bTops.size() <= 0) {
      cout << "NO BLUE BOXES ";
    }
    if ((ms->config.synServoLockFrames > ms->config.heightLearningServoTimeout) && (ARE_GENERIC_HEIGHT_LEARNING(ms))) {
      cout << "TIMED OUT ";
    }
    cout << endl;
    restartBBLearning(ms);
    return;
  }

  if ( ((ms->config.synServoLockFrames > ms->config.mappingServoTimeout) || (ms->config.bTops.size() <= 0)) && 
	(ms->config.currentBoundingBoxMode == MAPPING) ) {
    cout << ">>>> Synchronic servo timed out or no blue boxes during mapping. <<<<" << endl;
    return;
  }

  {
    int i, j;
    mapxyToij(ms, ms->config.currentEEPose.px, ms->config.currentEEPose.py, &i, &j);
    int doWeHaveClearance = (ms->config.clearanceMap[i + ms->config.mapWidth * j] != 0);
    if (!doWeHaveClearance) {
      cout << ">>>> Synchronic servo strayed out of clearance area during mapping. <<<<" << endl;
      ms->pushWord("endStackCollapseNoop");
      return;
    }
  }

  // ATTN 19
  // if we time out, reset the pick learning 
  if ( ((ms->config.synServoLockFrames > ms->config.heightLearningServoTimeout)) && 
	ARE_GENERIC_PICK_LEARNING(ms) ) {
    // record a failure
    cout << ">>>> Synchronic servo timed out.  Going back on patrol. <<<<" << endl;
    ms->config.thisGraspPicked = FAILURE; 
    ms->pushWord("shiftIntoGraspGear1"); 
    ms->pushCopies("beep", 15); 
    ms->pushWord("countGrasp"); 
    return; 
  }

  if (ms->config.bTops.size() <= 0) {
    cout << ">>>> HELP,  I CAN'T SEE!!!!! Going back on patrol. <<<<" << endl;
    ms->pushWord("visionCycle"); 
    ms->pushWord("waitUntilAtCurrentPosition"); 
    return;
  }

  if (ms->config.synchronicTakeClosest) {
    if ((ms->config.pilotClosestTarget.px != -1) && (ms->config.pilotClosestTarget.py != -1)) {
      ms->config.pilotTarget.px = ms->config.pilotClosestTarget.px;
      ms->config.pilotTarget.py = ms->config.pilotClosestTarget.py;
      ms->config.pilotTarget.pz = ms->config.pilotClosestTarget.pz;
      ms->config.pilotTargetBlueBoxNumber = ms->config.pilotClosestBlueBoxNumber;
    } else {
      return;
    }
  }

  // target the closest blue box that hasn't been mapped since
  //  the last mapping started
  if (ms->config.currentBoundingBoxMode == MAPPING) {
    int foundAnUnmappedTarget = 0;
    int closestUnmappedBBToReticle = -1;
    double closestBBDistance = VERYBIGNUMBER;
    for (int c = 0; c < ms->config.bTops.size(); c++) {
      double tbx, tby;
      int tbi, tbj;
      double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;
      pixelToGlobal(ms, ms->config.bCens[c].x, ms->config.bCens[c].y, zToUse, &tbx, &tby);
      mapxyToij(ms, tbx, tby, &tbi, &tbj);
      
      ros::Time thisLastMappedTime = ms->config.objectMap[tbi + ms->config.mapWidth * tbj].lastMappedTime;
      ros::Time thisNow = ros::Time::now();

      // ATTN 23
      //int isUnmapped = (thisLastMappedTime < ms->config.lastScanStarted);
      int isCooldownComplete = (thisNow.sec - thisLastMappedTime.sec) > ms->config.mapBlueBoxCooldown;

      int isOutOfReach = ( !positionIsSearched(ms, tbx, tby) || 
                           !isBlueBoxIkPossible(ms, ms->config.bTops[c], ms->config.bBots[c]) ); 

      double thisDistance = sqrt((ms->config.bCens[c].x-ms->config.reticle.px)*(ms->config.bCens[c].x-ms->config.reticle.px) + (ms->config.bCens[c].y-ms->config.reticle.py)*(ms->config.bCens[c].y-ms->config.reticle.py));
      cout << "   Servo CUB distance for box " << c << " : " << thisDistance << ", isCooldownComplete isOutOfReach: " <<
	      isCooldownComplete << " " << isOutOfReach << endl;
      cout << "      (thisNow - thisLastMappedTime) mapBlueBoxCooldown:" << 
	      thisNow.sec - thisLastMappedTime.sec << " " << ms->config.mapBlueBoxCooldown << " " <<  endl;

      if (isOutOfReach) {
	mapBlueBox(ms, ms->config.bTops[c], ms->config.bBots[c], 0, ros::Time::now()+ros::Duration(ms->config.mapBlueBoxCooldown));
      }

      if ( isCooldownComplete  && 
	   !isOutOfReach ) {
	if (thisDistance < closestBBDistance) {
	  closestBBDistance = thisDistance;
	  closestUnmappedBBToReticle = c;
	  foundAnUnmappedTarget = 1;
	}
      } 
    }

    if (foundAnUnmappedTarget) {
      ms->config.pilotClosestBlueBoxNumber = closestUnmappedBBToReticle;
      ms->config.pilotTarget.px = ms->config.bCens[ms->config.pilotClosestBlueBoxNumber].x;
      ms->config.pilotTarget.py = ms->config.bCens[ms->config.pilotClosestBlueBoxNumber].y;
      ms->config.pilotTarget.pz = 0;
      ms->config.pilotClosestTarget = ms->config.pilotTarget;
    } else {
      // this prevents gradient servo 
      ms->config.pilotClosestBlueBoxNumber = -1;
      ms->config.bTops.resize(0);
      ms->config.bBots.resize(0);
      ms->config.bCens.resize(0);
      ms->config.bLabels.resize(0);
      return;
    }
  }


  double Px = ms->config.reticle.px - ms->config.pilotTarget.px;
  double Py = ms->config.reticle.py - ms->config.pilotTarget.py;

  double dx = (ms->config.currentEEPose.px - ms->config.trueEEPose.position.x);
  double dy = (ms->config.currentEEPose.py - ms->config.trueEEPose.position.y);
  double dz = (ms->config.currentEEPose.pz - ms->config.trueEEPose.position.z);
  double distance = dx*dx + dy*dy + dz*dz;

  // if we are not there yet, continue
  if (distance > ms->config.w1GoThresh*ms->config.w1GoThresh) {
    cout << " XXX deprecated code path, synchronci servo should not be responsible for enforcing distance 4812675" << endl;
    ms->pushCopies("waitUntilAtCurrentPosition", 1); 
    ms->config.synServoLockFrames = 0;
    ms->pushWord("synchronicServo"); 
  } else {
    if ((fabs(Px) < ms->config.synServoPixelThresh) && (fabs(Py) < ms->config.synServoPixelThresh)) {
      // ATTN 12
      if (ARE_GENERIC_HEIGHT_LEARNING(ms)) {
	cout << "bbLearning: synchronic servo succeeded. gradientServoDuringHeightLearning: " << ms->config.gradientServoDuringHeightLearning << endl;
	if (ms->config.gradientServoDuringHeightLearning) {
	  cout << "bbLearning: proceeding to gradient servo." << endl;
	} else {
	  cout << "bbLearning: returning from synchronic servo." << endl;
	  return;
	}
      }

      // ATTN 17
      if (ms->config.bailAfterSynchronic) {
	cout << "synchronic servo set to bail. returning." << endl;
	return;
      }

      cout << "got within thresh. ";
      if ((ms->config.classAerialGradients[ms->config.targetClass].rows > 1) && (ms->config.classAerialGradients[ms->config.targetClass].cols > 1)) {
        ms->pushWord("gradientServo"); 
        cout << "Queuing gradient servo." << endl;
        //ms->pushCopies("density", ms->config.densityIterationsForGradientServo); 
	//ms->pushCopies("accumulateDensity", ms->config.densityIterationsForGradientServo); 
        //ms->pushCopies("resetTemporalMap", 1); 
        //ms->pushWord("resetAerialGradientTemporalFrameAverage"); 
        //ms->pushCopies("density", 1); 
        //ms->pushCopies("waitUntilAtCurrentPosition", 5); 
        ms->pushCopies("waitUntilAtCurrentPosition", 1); 
        
      } else {
        ROS_ERROR_STREAM("No gradient map for class " << ms->config.targetClass << endl);
        ms->clearStack();
      }

      return;	
    } else {

      double thisKp = ms->config.synKp;
      double pTermX = thisKp*Px;
      double pTermY = thisKp*Py;

      // invert the current eePose orientation to decide which direction to move from POV
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

      // ATTN 21
      // old PID
      //double newx = ms->config.currentEEPose.px + pTermX*localUnitY.x() - pTermY*localUnitX.x();
      //double newy = ms->config.currentEEPose.py + pTermX*localUnitY.y() - pTermY*localUnitX.y();
      double newx = 0;
      double newy = 0;
      // first analytic
      //double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;
      //pixelToGlobal(ms, ms->config.pilotTarget.px, ms->config.pilotTarget.py, zToUse, &newx, &newy);
      // ATTN 23
      eePose newGlobalTarget = analyticServoPixelToReticle(ms, ms->config.pilotTarget, ms->config.reticle, 0);
      newx = newGlobalTarget.px;
      newy = newGlobalTarget.py;

      if (!positionIsMapped(ms, newx, newy)) {
        cout << "Returning because position is out of map bounds." << endl;
        return;
      } else {
        ms->pushWord("synchronicServo"); 
	// ATTN 21
        //ms->config.currentEEPose.px += pTermX*localUnitY.x() - pTermY*localUnitX.x();
        //ms->config.currentEEPose.py += pTermX*localUnitY.y() - pTermY*localUnitX.y();
        ms->config.currentEEPose.px = newx;
        ms->config.currentEEPose.py = newy;
      }
    }
  }
}

void darkServo(shared_ptr<MachineState> ms) {

  // remember, ms->config.currentTableZ is inverted so this is like minus
  double heightAboveTable = ms->config.currentEEPose.pz + ms->config.currentTableZ;

  double heightFactor = heightAboveTable / ms->config.minHeight;

  int darkX = 0;
  int darkY = 0;
  findDarkness(ms, &darkX, &darkY);

  cout << "darkServo darkX darkY heightAboveTable: " << darkX << " " << darkY << " " << heightAboveTable << endl;

  ms->config.reticle = ms->config.vanishingPointReticle;
  ms->config.pilotTarget.px = darkX;
  ms->config.pilotTarget.py = darkY;

  double Px = ms->config.reticle.px - ms->config.pilotTarget.px;
  double Py = ms->config.reticle.py - ms->config.pilotTarget.py;

  double thisKp = ms->config.darkKp * heightFactor;
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

  double newx = ms->config.currentEEPose.px + pTermX*localUnitY.x() - pTermY*localUnitX.x();
  double newy = ms->config.currentEEPose.py + pTermX*localUnitY.y() - pTermY*localUnitX.y();

  ms->config.currentEEPose.px = newx;
  ms->config.currentEEPose.py = newy;

  if ((fabs(Px) < ms->config.darkServoPixelThresh) && (fabs(Py) < ms->config.darkServoPixelThresh)) {
    cout << "darkness reached, continuing." << endl;
  } else {
    cout << "darkness not reached, servoing more. " << ms->config.darkServoIterations << " " << ms->config.darkServoTimeout << endl;
    ms->pushWord("darkServoA");
  }
}

void faceServo(shared_ptr<MachineState> ms, vector<Rect> faces) {

  if (faces.size() == 0) {
    cout << "no faces, servoing more. " << ms->config.faceServoIterations << " " << ms->config.faceServoTimeout << endl;
    ms->pushWord("faceServoA");
    return;
  }

  eePose bestFacePose;
  double distance = VERYBIGNUMBER;
  for (int i = 0; i < faces.size(); i++) {
    eePose faceImagePose = eePose::fromRectCentroid(faces[i]);
    double thisDistance = eePose::squareDistance(ms->config.vanishingPointReticle, faceImagePose);
    if (thisDistance < distance) {
      distance = thisDistance;
      bestFacePose = faceImagePose;
    }
  }

  double heightFactor = 1 / ms->config.minHeight;

  ms->config.reticle = ms->config.vanishingPointReticle;
  ms->config.pilotTarget.px = bestFacePose.px;
  ms->config.pilotTarget.py = bestFacePose.py;

  double Px = ms->config.reticle.px - ms->config.pilotTarget.px;
  double Py = ms->config.reticle.py - ms->config.pilotTarget.py;

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


void initRangeMaps(shared_ptr<MachineState> ms) {
  ms->config.classRangeMaps.resize(ms->config.numClasses);
  ms->config.classGraspMemoryTries1.resize(ms->config.numClasses);
  ms->config.classGraspMemoryPicks1.resize(ms->config.numClasses);
  ms->config.classGraspMemoryTries2.resize(ms->config.numClasses);
  ms->config.classGraspMemoryPicks2.resize(ms->config.numClasses);
  ms->config.classGraspMemoryTries3.resize(ms->config.numClasses);
  ms->config.classGraspMemoryPicks3.resize(ms->config.numClasses);
  ms->config.classGraspMemoryTries4.resize(ms->config.numClasses);
  ms->config.classGraspMemoryPicks4.resize(ms->config.numClasses);
  ms->config.classAerialGradients.resize(ms->config.numClasses);

  // ATTN 16
  ms->config.classHeight0AerialGradients.resize(ms->config.numClasses);
  ms->config.classHeight1AerialGradients.resize(ms->config.numClasses);
  ms->config.classHeight2AerialGradients.resize(ms->config.numClasses);
  ms->config.classHeight3AerialGradients.resize(ms->config.numClasses);

  ms->config.classGraspZs.resize(ms->config.numClasses);
  ms->config.classGraspZsSet.resize(ms->config.numClasses);
  ms->config.class3dGrasps.resize(ms->config.numClasses);
  for(int i = 0; i < ms->config.numClasses; i++) {
    ms->config.class3dGrasps[i].resize(0);
  }

  ms->config.classHeightMemoryTries.resize(ms->config.numClasses);
  ms->config.classHeightMemoryPicks.resize(ms->config.numClasses);
  for (int i = 0; i < ms->config.classLabels.size(); i++) {
    tryToLoadRangeMap(ms, ms->config.class_crops_path, ms->config.classLabels[i].c_str(), i);
  }
}

int isThisGraspMaxedOut(shared_ptr<MachineState> ms, int i) {
  int toReturn = 0;

  if (ms->config.currentPickMode == LEARNING_SAMPLING) {
    toReturn = ( (ms->config.graspMemoryTries[i] >= ms->config.graspLearningMaxTries) );
  } else if (ms->config.currentPickMode == LEARNING_ALGORITHMC) {
    // ATTN 20
    double successes = ms->config.graspMemoryPicks[i];
    double failures = ms->config.graspMemoryTries[i] - ms->config.graspMemoryPicks[i];
    cout << "YYY failures, successes: " << failures << " " << successes << endl;
    successes = round(successes);
    failures = round(failures);
    cout << "XXX failures, successes: " << failures << " " << successes << endl;
    // returns probability that mu <= d given successes and failures.
    double result = cephes_incbet(successes + 1, failures + 1, ms->config.algorithmCTarget);
    toReturn = (result > ms->config.algorithmCRT);
  } else if (ms->config.currentPickMode == STATIC_MARGINALS) {
    //toReturn = (ms->config.graspMemoryTries[i] <= 1);
  }

  return toReturn;
}

eePose pixelToGlobalEEPose(shared_ptr<MachineState> ms, int pX, int pY, double gZ) {
  eePose result;
  pixelToGlobal(ms, pX, pY, gZ, &result.px, &result.py);
  result.pz = ms->config.trueEEPose.position.z - ms->config.currentTableZ;
  result.qx = 0;
  result.qy = 0;
  result.qz = 0;
  return result;
}

void interpolateM_xAndM_yFromZ(shared_ptr<MachineState> ms, double dZ, double * m_x, double * m_y) {

  // XXX disabling for calibration
  //return;

  double bBZ[4];
  bBZ[0] = convertHeightIdxToGlobalZ(ms, 0) + ms->config.currentTableZ;
  bBZ[1] = convertHeightIdxToGlobalZ(ms, 1) + ms->config.currentTableZ;
  bBZ[2] = convertHeightIdxToGlobalZ(ms, 2) + ms->config.currentTableZ;
  bBZ[3] = convertHeightIdxToGlobalZ(ms, 3) + ms->config.currentTableZ;

  if (dZ <= bBZ[0]) {
    *m_x = ms->config.m_x_h[0];
    *m_y = ms->config.m_y_h[0];
  } else if (dZ <= bBZ[1]) {
    double gap = bBZ[1] - bBZ[0];
    double c0 = 1.0 - ((dZ - bBZ[0])/gap);
    double c1 = 1.0 - ((bBZ[1] - dZ)/gap);
    *m_x = c0*ms->config.m_x_h[0] + c1*ms->config.m_x_h[1];
    *m_y = c0*ms->config.m_y_h[0] + c1*ms->config.m_y_h[1];
  } else if (dZ <= bBZ[2]) {
    double gap = bBZ[2] - bBZ[1];
    double c1 = 1.0 - ((dZ - bBZ[1])/gap);
    double c2 = 1.0 - ((bBZ[2] - dZ)/gap);
    *m_x = c1*ms->config.m_x_h[1] + c2*ms->config.m_x_h[2];
    *m_y = c1*ms->config.m_y_h[1] + c2*ms->config.m_y_h[2];
  } else if (dZ <= bBZ[3]) {
    double gap = bBZ[3] - bBZ[2];
    double c2 = 1.0 - ((dZ - bBZ[2])/gap);
    double c3 = 1.0 - ((bBZ[3] - dZ)/gap);
    *m_x = c2*ms->config.m_x_h[2] + c3*ms->config.m_x_h[3];
    *m_y = c2*ms->config.m_y_h[2] + c3*ms->config.m_y_h[3];
  } else if (dZ > bBZ[3]) {
    *m_x = ms->config.m_x_h[3];
    *m_y = ms->config.m_y_h[3];
  } else {
    assert(0); // my my
  }
  //cout << ms->config.m_x_h[0] << " " << ms->config.m_x_h[1] << " " << ms->config.m_x_h[2] << " " << ms->config.m_x_h[3] << " " << *m_x << endl;
  //cout << m_y_h[0] << " " << ms->config.m_y_h[1] << " " << ms->config.m_y_h[2] << " " << ms->config.m_y_h[3] << " " << *m_y << endl;
}

void pixelToGlobal(shared_ptr<MachineState> ms, int pX, int pY, double gZ, double * gX, double * gY) {
  pixelToGlobal(ms, pX, pY, gZ, gX, gY, ms->config.trueEEPoseEEPose);
}

void pixelToGlobal(shared_ptr<MachineState> ms, int pX, int pY, double gZ, double * gX, double * gY, eePose givenEEPose) {
  interpolateM_xAndM_yFromZ(ms, gZ, &ms->config.m_x, &ms->config.m_y);

  int x1 = ms->config.heightReticles[0].px;
  int x2 = ms->config.heightReticles[1].px;
  int x3 = ms->config.heightReticles[2].px;
  int x4 = ms->config.heightReticles[3].px;

  int y1 = ms->config.heightReticles[0].py;
  int y2 = ms->config.heightReticles[1].py;
  int y3 = ms->config.heightReticles[2].py;
  int y4 = ms->config.heightReticles[3].py;

  double z1 = convertHeightIdxToGlobalZ(ms, 0) + ms->config.currentTableZ;
  double z2 = convertHeightIdxToGlobalZ(ms, 1) + ms->config.currentTableZ;
  double z3 = convertHeightIdxToGlobalZ(ms, 2) + ms->config.currentTableZ;
  double z4 = convertHeightIdxToGlobalZ(ms, 3) + ms->config.currentTableZ;

  double reticlePixelX = 0.0;
  double reticlePixelY = 0.0;
  {
    double d = ms->config.d_x;
    double c = ((z4*x4-z2*x2)*(x3-x1)-(z3*x3-z1*x1)*(x4-x2))/((z1-z3)*(x4-x2)-(z2-z4)*(x3-x1));

    double b42 = (z4*x4-z2*x2+(z2-z4)*c)/(x4-x2);
    double b31 = (z3*x3-z1*x1+(z1-z3)*c)/(x3-x1);

    double bDiff = b42-b31;
    double b = (b42+b31)/2.0;

    int x_thisZ = c + ( (x1-c)*(z1-b) )/(gZ-b);
    x_thisZ = c + ( (d)*(x_thisZ-c) )/(d);
    reticlePixelX = x_thisZ;
  }
  {
    double d = ms->config.d_y;
    double c = ((z4*y4-z2*y2)*(y3-y1)-(z3*y3-z1*y1)*(y4-y2))/((z1-z3)*(y4-y2)-(z2-z4)*(y3-y1));

    double b42 = (z4*y4-z2*y2+(z2-z4)*c)/(y4-y2);
    double b31 = (z3*y3-z1*y1+(z1-z3)*c)/(y3-y1);

    double bDiff = b42-b31;
    double b = (b42+b31)/2.0;

    int y_thisZ = c + ( (y1-c)*(z1-b) )/(gZ-b);
    y_thisZ = c + ( (d)*(y_thisZ-c) )/(d);
    reticlePixelY = y_thisZ;
  }

  // account for rotation of the end effector 
  Quaternionf eeqform(givenEEPose.qw, givenEEPose.qx, givenEEPose.qy, givenEEPose.qz);
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
  Point center = Point(reticlePixelX, reticlePixelY);

  Mat un_rot_mat = getRotationMatrix2D( center, angle, scale );

  Mat toUn(3,1,CV_64F);
  toUn.at<double>(0,0)=pX;
  toUn.at<double>(1,0)=pY;
  toUn.at<double>(2,0)=1.0;
  Mat didUn = un_rot_mat*toUn;
  pX = didUn.at<double>(0,0);
  pY = didUn.at<double>(1,0);

  double oldPx = pX;
  double oldPy = pY;
  pX = reticlePixelX + (oldPy - reticlePixelY) - ms->config.offX;
  pY = reticlePixelY + (oldPx - reticlePixelX) - ms->config.offY;

  {
    double d = ms->config.d_x/ms->config.m_x;
    double c = ((z4*x4-z2*x2)*(x3-x1)-(z3*x3-z1*x1)*(x4-x2))/((z1-z3)*(x4-x2)-(z2-z4)*(x3-x1));

    double b42 = (z4*x4-z2*x2+(z2-z4)*c)/(x4-x2);
    double b31 = (z3*x3-z1*x1+(z1-z3)*c)/(x3-x1);

    double bDiff = b42-b31;
    //cout << "x1 x2 x3 x4: " << x1 << " " << x2 << " " << x3 << " " << x4 << endl;
    //cout << "y1 y2 y3 y4: " << y1 << " " << y2 << " " << y3 << " " << y4 << endl;
    //cout << "z1 z2 z3 z4: " << z1 << " " << z2 << " " << z3 << " " << z4 << endl;
    //cout << "bDiff = " << bDiff << ", c = " << c << " b42, b31: " << b42 << " " << b31 << " " << endl;
    double b = (b42+b31)/2.0;

    int x_thisZ = c + ( (x1-c)*(z1-b) )/(gZ-b);
    //int x_thisZ = c + ( ms->config.m_x*(x1-c)*(z1-b) )/(gZ-b);
    //*gX = d + ( (pX-c)*(ms->config.currentEEPose.px-d) )/(x1-c) ;
    //*gX = givenEEPose.px - d + ( (pX-c)*(d) )/( (x_thisZ-c)*ms->config.m_x ) ;
    *gX = givenEEPose.px - d + ( (pX-c)*(d) )/( (x_thisZ-c) ) ;
    x_thisZ = c + ( (d)*(x_thisZ-c) )/(d);
  }
  {
    double d = ms->config.d_y/ms->config.m_y;
    double c = ((z4*y4-z2*y2)*(y3-y1)-(z3*y3-z1*y1)*(y4-y2))/((z1-z3)*(y4-y2)-(z2-z4)*(y3-y1));

    double b42 = (z4*y4-z2*y2+(z2-z4)*c)/(y4-y2);
    double b31 = (z3*y3-z1*y1+(z1-z3)*c)/(y3-y1);

    double bDiff = b42-b31;
    //cout << "x1 x2 x3 x4: " << x1 << " " << x2 << " " << x3 << " " << x4 << endl;
    //cout << "y1 y2 y3 y4: " << y1 << " " << y2 << " " << y3 << " " << y4 << endl;
    //cout << "z1 z2 z3 z4: " << z1 << " " << z2 << " " << z3 << " " << z4 << endl;
    //cout << "bDiff = " << bDiff << ", c = " << c << " b42, b31: " << b42 << " " << b31 << " " << endl;
    double b = (b42+b31)/2.0;

    int y_thisZ = c + ( (y1-c)*(z1-b) )/(gZ-b);
    //int y_thisZ = c + ( ms->config.m_y*(y1-c)*(z1-b) )/(gZ-b);
    //*gY = d + ( (pY-c)*(ms->config.currentEEPose.py-d) )/(y1-c) ;
    //*gY = givenEEPose.py - d + ( (pY-c)*(d) )/( (y_thisZ-c)*ms->config.m_y ) ;
    *gY = givenEEPose.py - d + ( (pY-c)*(d) )/( (y_thisZ-c) ) ;
    y_thisZ = c + ( (d)*(y_thisZ-c) )/(d);
  }
}

void globalToPixelPrint(shared_ptr<MachineState> ms, int * pX, int * pY, double gZ, double gX, double gY) {
  interpolateM_xAndM_yFromZ(ms, gZ, &ms->config.m_x, &ms->config.m_y);

  int x1 = ms->config.heightReticles[0].px;
  int x2 = ms->config.heightReticles[1].px;
  int x3 = ms->config.heightReticles[2].px;
  int x4 = ms->config.heightReticles[3].px;

  int y1 = ms->config.heightReticles[0].py;
  int y2 = ms->config.heightReticles[1].py;
  int y3 = ms->config.heightReticles[2].py;
  int y4 = ms->config.heightReticles[3].py;

  double z1 = convertHeightIdxToGlobalZ(ms, 0) + ms->config.currentTableZ;
  double z2 = convertHeightIdxToGlobalZ(ms, 1) + ms->config.currentTableZ;
  double z3 = convertHeightIdxToGlobalZ(ms, 2) + ms->config.currentTableZ;
  double z4 = convertHeightIdxToGlobalZ(ms, 3) + ms->config.currentTableZ;

  double reticlePixelX = 0.0;
  double reticlePixelY = 0.0;
  {
    //double d = ms->config.d_x;
    double d = ms->config.d_x/ms->config.m_x;
    double c = ((z4*x4-z2*x2)*(x3-x1)-(z3*x3-z1*x1)*(x4-x2))/((z1-z3)*(x4-x2)-(z2-z4)*(x3-x1));

    double b42 = (z4*x4-z2*x2+(z2-z4)*c)/(x4-x2);
    double b31 = (z3*x3-z1*x1+(z1-z3)*c)/(x3-x1);

    double bDiff = b42-b31;
    //cout << "x1 x2 x3 x4: " << x1 << " " << x2 << " " << x3 << " " << x4 << endl;
    //cout << "y1 y2 y3 y4: " << y1 << " " << y2 << " " << y3 << " " << y4 << endl;
    //cout << "z1 z2 z3 z4: " << z1 << " " << z2 << " " << z3 << " " << z4 << endl;
    //cout << "bDiff = " << bDiff << ", c = " << c << " b42, b31: " << b42 << " " << b31 << " " << endl;
    double b = (b42+b31)/2.0;

    int x_thisZ = c + ( (x1-c)*(z1-b) )/(gZ-b);
    //int x_thisZ = c + ( ms->config.m_x*(x1-c)*(z1-b) )/(gZ-b);
    //*pX = c + ( (gX-d)*(x1-c) )/(ms->config.currentEEPose.px-d);
    //*pX = c + ( (gX-d)*(x_thisZ-c) )/(ms->config.currentEEPose.px-d);
    //*pX = c + ( ms->config.m_x*(gX-ms->config.trueEEPose.position.x+d)*(x_thisZ-c) )/(d);
    *pX = c + ( (gX-ms->config.trueEEPose.position.x+d)*(x_thisZ-c) )/(d);
    // need to set this again so things match up if gX is truEEpose
    //x_thisZ = c + ( ms->config.m_x*(x1-c)*(z1-b) )/(gZ-b);
    x_thisZ = c + ( (d)*(x_thisZ-c) )/(d);
    reticlePixelX = x_thisZ;

    cout << "(x pass) d c b42 b31 bDiff b x_thisZ m_x: " << endl 
	 << d << " " << c << " " << b42 << " " << b31 << " " << bDiff << " " << b << " " << x_thisZ << " "  << ms->config.m_x << " " << endl;
  }
  {
    //double d = ms->config.d_y;
    double d = ms->config.d_y/ms->config.m_y;
    double c = ((z4*y4-z2*y2)*(y3-y1)-(z3*y3-z1*y1)*(y4-y2))/((z1-z3)*(y4-y2)-(z2-z4)*(y3-y1));

    double b42 = (z4*y4-z2*y2+(z2-z4)*c)/(y4-y2);
    double b31 = (z3*y3-z1*y1+(z1-z3)*c)/(y3-y1);

    double bDiff = b42-b31;
    //cout << "x1 x2 x3 x4: " << x1 << " " << x2 << " " << x3 << " " << x4 << endl;
    //cout << "y1 y2 y3 y4: " << y1 << " " << y2 << " " << y3 << " " << y4 << endl;
    //cout << "z1 z2 z3 z4: " << z1 << " " << z2 << " " << z3 << " " << z4 << endl;
    //cout << "bDiff = " << bDiff << ", c = " << c << " b42, b31: " << b42 << " " << b31 << " " << endl;
    double b = (b42+b31)/2.0;

    int y_thisZ = c + ( (y1-c)*(z1-b) )/(gZ-b);
    //int y_thisZ = c + ( ms->config.m_y*(y1-c)*(z1-b) )/(gZ-b);
    //*pY = c + ( (gY-d)*(y1-c) )/(ms->config.currentEEPose.py-d);
    //*pY = c + ( (gY-d)*(y_thisZ-c) )/(ms->config.currentEEPose.py-d);
    //*pY = c + ( ms->config.m_y*(gY-ms->config.trueEEPose.position.y+d)*(y_thisZ-c) )/(d);
    *pY = c + ( (gY-ms->config.trueEEPose.position.y+d)*(y_thisZ-c) )/(d);
    // need to set this again so things match up if gX is truEEpose
    //y_thisZ = c + ( ms->config.m_y*(y1-c)*(z1-b) )/(gZ-b);
    y_thisZ = c + ( (d)*(y_thisZ-c) )/(d);
    reticlePixelY = y_thisZ;

    cout << "(y pass) d c b42 b31 bDiff b y_thisZ m_y: " << endl 
	 << d << " " << c << " " << b42 << " " << b31 << " " << bDiff << " " << b << " " << y_thisZ << " "  << ms->config.m_y << " " << endl;
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

  Mat toUn(3,1,CV_64F);
  toUn.at<double>(0,0)=*pX;
  toUn.at<double>(1,0)=*pY;
  toUn.at<double>(2,0)=1.0;
  Mat didUn = un_rot_mat*toUn;
  *pX = didUn.at<double>(0,0);
  *pY = didUn.at<double>(1,0);

  double oldPx = *pX;
  double oldPy = *pY;
  //*pX = reticlePixelX + m_y*(oldPy - reticlePixelY) + ms->config.offX;
  //*pY = reticlePixelY + m_x*(oldPx - reticlePixelX) + ms->config.offY;
  *pX = reticlePixelX + (oldPy - reticlePixelY) + ms->config.offX;
  *pY = reticlePixelY + (oldPx - reticlePixelX) + ms->config.offY;
}
void globalToPixel(shared_ptr<MachineState> ms, int * pX, int * pY, double gZ, double gX, double gY) {
  interpolateM_xAndM_yFromZ(ms, gZ, &ms->config.m_x, &ms->config.m_y);

  int x1 = ms->config.heightReticles[0].px;
  int x2 = ms->config.heightReticles[1].px;
  int x3 = ms->config.heightReticles[2].px;
  int x4 = ms->config.heightReticles[3].px;

  int y1 = ms->config.heightReticles[0].py;
  int y2 = ms->config.heightReticles[1].py;
  int y3 = ms->config.heightReticles[2].py;
  int y4 = ms->config.heightReticles[3].py;

  double z1 = convertHeightIdxToGlobalZ(ms, 0) + ms->config.currentTableZ;
  double z2 = convertHeightIdxToGlobalZ(ms, 1) + ms->config.currentTableZ;
  double z3 = convertHeightIdxToGlobalZ(ms, 2) + ms->config.currentTableZ;
  double z4 = convertHeightIdxToGlobalZ(ms, 3) + ms->config.currentTableZ;

  double reticlePixelX = 0.0;
  double reticlePixelY = 0.0;
  {
    //double d = ms->config.d_x;
    double d = ms->config.d_x/ms->config.m_x;
    double c = ((z4*x4-z2*x2)*(x3-x1)-(z3*x3-z1*x1)*(x4-x2))/((z1-z3)*(x4-x2)-(z2-z4)*(x3-x1));

    double b42 = (z4*x4-z2*x2+(z2-z4)*c)/(x4-x2);
    double b31 = (z3*x3-z1*x1+(z1-z3)*c)/(x3-x1);

    double bDiff = b42-b31;
    //cout << "x1 x2 x3 x4: " << x1 << " " << x2 << " " << x3 << " " << x4 << endl;
    //cout << "y1 y2 y3 y4: " << y1 << " " << y2 << " " << y3 << " " << y4 << endl;
    //cout << "z1 z2 z3 z4: " << z1 << " " << z2 << " " << z3 << " " << z4 << endl;
    //cout << "bDiff = " << bDiff << ", c = " << c << " b42, b31: " << b42 << " " << b31 << " " << endl;
    double b = (b42+b31)/2.0;

    int x_thisZ = c + ( (x1-c)*(z1-b) )/(gZ-b);
    //int x_thisZ = c + ( ms->config.m_x*(x1-c)*(z1-b) )/(gZ-b);
    //*pX = c + ( (gX-d)*(x1-c) )/(ms->config.currentEEPose.px-d);
    //*pX = c + ( (gX-d)*(x_thisZ-c) )/(ms->config.currentEEPose.px-d);
    //*pX = c + ( ms->config.m_x*(gX-ms->config.trueEEPose.position.x+d)*(x_thisZ-c) )/(d);
    *pX = c + ( (gX-ms->config.trueEEPose.position.x+d)*(x_thisZ-c) )/(d);
    // need to set this again so things match up if gX is truEEpose
    //x_thisZ = c + ( ms->config.m_x*(x1-c)*(z1-b) )/(gZ-b);
    x_thisZ = c + ( (d)*(x_thisZ-c) )/(d);
    reticlePixelX = x_thisZ;
  }
  {
    //double d = ms->config.d_y;
    double d = ms->config.d_y/ms->config.m_y;
    double c = ((z4*y4-z2*y2)*(y3-y1)-(z3*y3-z1*y1)*(y4-y2))/((z1-z3)*(y4-y2)-(z2-z4)*(y3-y1));

    double b42 = (z4*y4-z2*y2+(z2-z4)*c)/(y4-y2);
    double b31 = (z3*y3-z1*y1+(z1-z3)*c)/(y3-y1);

    double bDiff = b42-b31;
    //cout << "x1 x2 x3 x4: " << x1 << " " << x2 << " " << x3 << " " << x4 << endl;
    //cout << "y1 y2 y3 y4: " << y1 << " " << y2 << " " << y3 << " " << y4 << endl;
    //cout << "z1 z2 z3 z4: " << z1 << " " << z2 << " " << z3 << " " << z4 << endl;
    //cout << "bDiff = " << bDiff << ", c = " << c << " b42, b31: " << b42 << " " << b31 << " " << endl;
    double b = (b42+b31)/2.0;

    int y_thisZ = c + ( (y1-c)*(z1-b) )/(gZ-b);
    //int y_thisZ = c + ( ms->config.m_y*(y1-c)*(z1-b) )/(gZ-b);
    //*pY = c + ( (gY-d)*(y1-c) )/(ms->config.currentEEPose.py-d);
    //*pY = c + ( (gY-d)*(y_thisZ-c) )/(ms->config.currentEEPose.py-d);
    //*pY = c + ( ms->config.m_y*(gY-ms->config.trueEEPose.position.y+d)*(y_thisZ-c) )/(d);
    *pY = c + ( (gY-ms->config.trueEEPose.position.y+d)*(y_thisZ-c) )/(d);
    // need to set this again so things match up if gX is truEEpose
    //y_thisZ = c + ( ms->config.m_y*(y1-c)*(z1-b) )/(gZ-b);
    y_thisZ = c + ( (d)*(y_thisZ-c) )/(d);
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

  Mat toUn(3,1,CV_64F);
  toUn.at<double>(0,0)=*pX;
  toUn.at<double>(1,0)=*pY;
  toUn.at<double>(2,0)=1.0;
  Mat didUn = un_rot_mat*toUn;
  *pX = didUn.at<double>(0,0);
  *pY = didUn.at<double>(1,0);

  double oldPx = *pX;
  double oldPy = *pY;
  //*pX = reticlePixelX + ms->config.m_y*(oldPy - reticlePixelY) + ms->config.offX;
  //*pY = reticlePixelY + ms->config.m_x*(oldPx - reticlePixelX) + ms->config.offY;
  *pX = reticlePixelX + (oldPy - reticlePixelY) + ms->config.offX;
  *pY = reticlePixelY + (oldPx - reticlePixelX) + ms->config.offY;
}

void paintEEPoseOnWrist(shared_ptr<MachineState> ms, eePose toPaint, cv::Scalar theColor) {
  cv::Scalar THEcOLOR(255-theColor[0], 255-theColor[1], 255-theColor[2]);
  int lineLength = 5;
  int pX = 0, pY = 0;  
  double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;

  globalToPixel(ms, &pX, &pY, zToUse, toPaint.px, toPaint.py);
  pX = pX - lineLength;
  pY = pY - lineLength;
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
  if (0) {
    double gX = 0, gY = 0;
    pixelToGlobal(ms, pX, pY, zToUse, &gX, &gY);
    globalToPixel(ms, &pX, &pY, zToUse, gX, gY);
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

double vectorArcTan(shared_ptr<MachineState> ms, double y, double x) {
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

void initVectorArcTan(shared_ptr<MachineState> ms) {
  for (int vaSlot = 0; vaSlot < ms->config.vaNumAngles; vaSlot++) {
    ms->config.vaX[vaSlot] = cos(vaSlot*ms->config.vaDelta);
    ms->config.vaY[vaSlot] = sin(vaSlot*ms->config.vaDelta);
  }
  /* smoke test
  for (int vaSlot = 0; vaSlot < ms->config.vaNumAngles; vaSlot++) {
    cout << "atan2 vectorArcTan: " << 
      vectorArcTan(ms->config.vaY[vaSlot], ms->config.vaX[vaSlot]) << " " << 
      atan2(ms->config.vaY[vaSlot], ms->config.vaX[vaSlot]) << endl; 
  }
  */
}

void mapBlueBox(shared_ptr<MachineState> ms, cv::Point tbTop, cv::Point tbBot, int detectedClass, ros::Time timeToMark) {
  Size sz = ms->config.objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;

  for (double px = tbTop.x-ms->config.mapBlueBoxPixelSkirt; px <= tbBot.x+ms->config.mapBlueBoxPixelSkirt; px++) {
    for (double py = tbTop.y-ms->config.mapBlueBoxPixelSkirt; py <= tbBot.y+ms->config.mapBlueBoxPixelSkirt; py++) {
      double x, y;
      double z = ms->config.trueEEPose.position.z + ms->config.currentTableZ;

      pixelToGlobal(ms, px, py, z, &x, &y);
      int i, j;
      mapxyToij(ms, x, y, &i, &j);

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
	if ( (ms->config.cam_img.rows != 0 && ms->config.cam_img.cols != 0) &&
	     ((px >=0) && (px < imW)) &&
	     ((py >=0) && (py < imH)) ) {
	  ms->config.objectMap[i + ms->config.mapWidth * j].b = (ms->config.cam_img.at<cv::Vec3b>(py, px)[0] * blueBoxWeight);
	  ms->config.objectMap[i + ms->config.mapWidth * j].g = (ms->config.cam_img.at<cv::Vec3b>(py, px)[1] * blueBoxWeight);
	  ms->config.objectMap[i + ms->config.mapWidth * j].r = (ms->config.cam_img.at<cv::Vec3b>(py, px)[2] * blueBoxWeight);
	  ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount = blueBoxWeight;
	}
      }
    }
  }
}

void mapBox(shared_ptr<MachineState> ms, BoxMemory boxMemory) {
  mapBlueBox(ms, boxMemory.bTop, boxMemory.bBot, boxMemory.labeledClassIndex, ros::Time::now());
}

void queryIK(shared_ptr<MachineState> ms, int * thisResult, baxter_core_msgs::SolvePositionIK * thisRequest) {
  if (ms->config.currentRobotMode == PHYSICAL) {
    *thisResult = ms->config.ikClient.call(*thisRequest);
  } else if (ms->config.currentRobotMode == SIMULATED) {
    *thisResult = 1;
  } else {
    assert(0);
  }
}

void globalToMapBackground(shared_ptr<MachineState> ms, double gX, double gY, double zToUse, int * mapGpPx, int * mapGpPy) {
  double msfWidth = ms->config.mapBackgroundXMax - ms->config.mapBackgroundXMin;
  double msfHeight = ms->config.mapBackgroundYMax - ms->config.mapBackgroundYMin;

  double mapGpFractionWidth = (gX - ms->config.mapBackgroundXMin) / msfWidth;
  double mapGpFractionHeight = (gY - ms->config.mapBackgroundYMin) / msfHeight;
  *mapGpPx = floor(mapGpFractionWidth * ms->config.mbiWidth);
  *mapGpPy = floor(mapGpFractionHeight * ms->config.mbiHeight);
  *mapGpPx = min(max(0, *mapGpPx), ms->config.mbiWidth-1);
  *mapGpPy = min(max(0, *mapGpPy), ms->config.mbiHeight-1);
}

void simulatorCallback(const ros::TimerEvent&) {
    //imageCallback
    //rangeCallback
    //endpointCallback

    //gripStateCallback

    //jointCallback
    //targetCallback
  shared_ptr<MachineState> ms = pMachineState;

  {
    sensor_msgs::Range myRange;
    myRange.range = 0.1;
    myRange.header.stamp = ros::Time::now();
    rangeCallback(myRange);
  }

  {
    baxter_core_msgs::EndpointState myEPS;

    myEPS.header.stamp = ros::Time::now();
    myEPS.pose.position.x = ms->config.currentEEPose.px;
    myEPS.pose.position.y = ms->config.currentEEPose.py;
    myEPS.pose.position.z = ms->config.currentEEPose.pz;
    myEPS.pose.orientation.x = ms->config.currentEEPose.qx;
    myEPS.pose.orientation.y = ms->config.currentEEPose.qy;
    myEPS.pose.orientation.z = ms->config.currentEEPose.qz;
    myEPS.pose.orientation.w = ms->config.currentEEPose.qw;

    endpointCallback(myEPS);
  }
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
    imageCallback(myImagePtr);
  }


}

bool isInGripperMaskBlocks(shared_ptr<MachineState> ms, int x, int y) {
  if ( (x >= ms->config.g1xs && x <= ms->config.g1xe && y >= ms->config.g1ys && y <= ms->config.g1ye) ||
       (x >= ms->config.g2xs && x <= ms->config.g2xe && y >= ms->config.g2ys && y <= ms->config.g2ye) ) {
    return true;
  } else {
    return false;
  }
}

bool isInGripperMask(shared_ptr<MachineState> ms, int x, int y) {
  if (ms->config.mask_gripper) {
    if (isSketchyMat(ms->config.gripperMask)) {
      return false;
    } else {
      return (( ms->config.gripperMask.at<uchar>(y,x) == 0 ));
    }
  } else if (ms->config.mask_gripper_blocks) {
    return isInGripperMaskBlocks(ms, x,y);
  } else {
    return false;
  }
}

void findDarkness(shared_ptr<MachineState> ms, int * xout, int * yout) {
  //*xout = vanishingPointReticle.px;
  //*yout = vanishingPointReticle.py;
  findOptimum(ms, xout, yout, -1);
}

void findLight(shared_ptr<MachineState> ms, int * xout, int * yout) {
  findOptimum(ms, xout, yout, 1);
}

void findOptimum(shared_ptr<MachineState> ms, int * xout, int * yout, int sign) {

  if (isSketchyMat(ms->config.accumulatedImage)) {
    ROS_ERROR("Whoops, accumulatedImage is sketchy, returning vanishing point to findOptimum.");
    *xout = ms->config.vanishingPointReticle.px;
    *yout = ms->config.vanishingPointReticle.py;
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



void gridKeypoints(shared_ptr<MachineState> ms, int gImW, int gImH, cv::Point top, cv::Point bot, int strideX, int strideY, vector<KeyPoint>& keypoints, int period) {
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

void bowGetFeatures(shared_ptr<MachineState> ms, std::string classDir, const char *className, double sigma, int keypointPeriod, int * grandTotalDescriptors, DescriptorExtractor * extractor, BOWKMeansTrainer * bowTrainer) {

  int totalDescriptors = 0;
  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  char buf[1024];
  sprintf(buf, "%s%s/rgb", classDir.c_str(), className);
  dpdf = opendir(buf);
  if (dpdf != NULL){
    while (epdf = readdir(dpdf)){
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

        vector<KeyPoint> keypoints1;
        vector<KeyPoint> keypoints2;
        Mat descriptors;

        char filename[1024];
        sprintf(filename, "%s%s/rgb/%s", classDir.c_str(), className, epdf->d_name);
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

void kNNGetFeatures(shared_ptr<MachineState> ms, std::string classDir, const char *className, int label, double sigma, Mat &kNNfeatures, Mat &kNNlabels, double sobel_sigma) {

  int param_kNNOverSampleFactor = 1;

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");

  char buf[1024];
  sprintf(buf, "%s%s/rgb", classDir.c_str(), className);
  dpdf = opendir(buf);
  if (dpdf != NULL){
    while (epdf = readdir(dpdf)){
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

        vector<KeyPoint> keypoints;
        Mat descriptors;
        Mat descriptors2;

        char filename[1024];
        sprintf(filename, "%s%s/rgb/%s", classDir.c_str(), className, epdf->d_name);
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

void posekNNGetFeatures(shared_ptr<MachineState> ms, std::string classDir, const char *className, double sigma, Mat &kNNfeatures, Mat &kNNlabels,
                        vector< cv::Vec<double,4> >& classQuaternions, int keypointPeriod, int lIndexStart, BOWImgDescriptorExtractor *bowExtractor) {

  string sClassName(className);

  int label = 0;

  int lIndex = lIndexStart;

  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");
  string ppm(".ppm");

  char buf[1024];
  sprintf(buf, "%s%s/rgbPose", classDir.c_str(), className);
  dpdf = opendir(buf);
  if (dpdf != NULL){
    while (epdf = readdir(dpdf)){
      string fileName(epdf->d_name);
      cout << fileName << " " << endl;
      if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

	string fext = fileName.substr(fileName.size()-4, 4);
	if (fext.compare(ppm))
	  continue;

	//string poseIndex = fileName.substr(sClassName.size()+1, string::npos);
	//poseIndex = poseIndex.substr(0,  poseIndex.length()-4);
	//label = std::atoi(poseIndex.c_str());

	// remove .ppm to form key
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






void resetAccumulatedImageAndMass(shared_ptr<MachineState> ms) {
  Size sz = ms->config.accumulatedImageMass.size();
  int imW = sz.width;
  int imH = sz.height;

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      ms->config.accumulatedImageMass.at<double>(y,x) = 0;
      ms->config.accumulatedImage.at<Vec3d>(y,x)[0] = 0;
      ms->config.accumulatedImage.at<Vec3d>(y,x)[1] = 0;
      ms->config.accumulatedImage.at<Vec3d>(y,x)[2] = 0;
    }
  }
}

void renderAccumulatedImageAndDensity(shared_ptr<MachineState> ms) {
  /*
  // copy the density map to the rendered image
  for (int x = 0; x < imW; x++) {
  for (int y = 0; y < imH; y++) {
  //uchar val = uchar(min( 1*255.0 *  (totalGraySobel.at<double>(y,x) - minGraySob) / sobGrayRange, 255.0));
  uchar val = uchar(min( 1*255.0 *  (ms->config.frameGraySobel.at<double>(y,x) - minAerTemp) / aerTempRange, 255.0));
  ms->config.gradientViewerImage.at<cv::Vec3b>(y,x) = cv::Vec<uchar, 3>(0,val,0);

  ms->config.gradientViewerImage.at<cv::Vec3b>(y+imH,x) = convertedYCbCrGradientImage.at<cv::Vec3b>(y,x);
  }
  }
  */
  Size sz = ms->config.objectViewerYCbCrBlur.size();
  int imW = sz.width;
  int imH = sz.height;

  int YConstant = 128;
  Mat oviToConstantize = ms->config.objectViewerYCbCrBlur.clone();


  Mat oviInBGR;
  cvtColor(oviToConstantize, oviInBGR, CV_YCrCb2BGR);

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      ms->config.gradientViewerImage.at<cv::Vec3b>(y+imH,x) = oviInBGR.at<cv::Vec3b>(y,x);
     }
  }

  if (ms->config.shouldIRender) {
    guardedImshow(ms->config.gradientViewerName, ms->config.gradientViewerImage, ms->config.sirGradient);
  }

}

void substituteAccumulatedImageQuantities(shared_ptr<MachineState> ms) {
  double param_aerialGradientDecayImageAverage = 0.0;
  ms->config.aerialGradientDecay = param_aerialGradientDecayImageAverage;

  double param_sobel_sigma_substitute_accumulated = 4.0;//2.0; reflections are a problem for low sigma...
  ms->config.sobel_sigma = param_sobel_sigma_substitute_accumulated;
  Size sz = ms->config.accumulatedImage.size();
  int imW = sz.width;
  int imH = sz.height;

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      double denom = ms->config.accumulatedImageMass.at<double>(y,x);
      if (denom <= 1.0) {
	denom = 1.0;
      }
      ms->config.objectViewerImage.at<Vec3b>(y,x)[0] = doubleToByte(ms->config.accumulatedImage.at<Vec3d>(y,x)[0] / denom);
      ms->config.objectViewerImage.at<Vec3b>(y,x)[1] = doubleToByte(ms->config.accumulatedImage.at<Vec3d>(y,x)[1] / denom);
      ms->config.objectViewerImage.at<Vec3b>(y,x)[2] = doubleToByte(ms->config.accumulatedImage.at<Vec3d>(y,x)[2] / denom);
    }
  }
}

void substituteLatestImageQuantities(shared_ptr<MachineState> ms) {
  double param_aerialGradientDecayIteratedDensity = 0.9;
  ms->config.aerialGradientDecay = param_aerialGradientDecayIteratedDensity;

  double param_sobel_sigma_substitute_latest = 4.0;
  ms->config.sobel_sigma = param_sobel_sigma_substitute_latest;
  if (ms->config.cv_ptr == NULL) {
    ROS_ERROR("Not receiving camera data, clearing call stack.");
    ms->clearStack();
    return;
  }

  ms->config.objectViewerImage = ms->config.cv_ptr->image.clone();
}

void goCalculateDensity(shared_ptr<MachineState> ms) {
  Size sz = ms->config.objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;

  // XXX TODO might be able to pick up some time here if their allocation is slow
  // by making these global
  ms->config.densityViewerImage = ms->config.objectViewerImage.clone();
  Mat tmpImage = ms->config.objectViewerImage.clone();

  Mat yCbCrGradientImage = ms->config.objectViewerImage.clone();

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

  // Sobel business
  Mat sobelGrayBlur;
  Mat sobelYCrCbBlur;
  processImage(tmpImage, sobelGrayBlur, sobelYCrCbBlur, ms->config.sobel_sigma);
  ms->config.objectViewerYCbCrBlur = sobelYCrCbBlur;
  ms->config.objectViewerGrayBlur = sobelYCrCbBlur;
  
  Mat totalGraySobel;
  {
    Mat grad_x, grad_y;
    int sobelScale = 1;
    int sobelDelta = 0;
    int sobelDepth = CV_64F;
    /// Gradient X
    Sobel(sobelGrayBlur, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
    /// Gradient Y
    Sobel(sobelGrayBlur, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

    grad_x = grad_x.mul(grad_x);
    grad_y = grad_y.mul(grad_y);
    totalGraySobel = grad_x + grad_y;
    // now totalGraySobel is gradient magnitude squared
  }

  Mat totalCrSobel = totalGraySobel.clone();
  Mat totalCrSobelMag;
  {
    for (int y = 0; y < imH; y++) {
      for (int x = 0; x < imW; x++) {
	cv::Vec3b thisColor = sobelYCrCbBlur.at<cv::Vec3b>(y,x);
	totalCrSobel.at<double>(y,x) = thisColor[1];
      }
    }
    Mat grad_x, grad_y;
    int sobelScale = 1;
    int sobelDelta = 0;
    int sobelDepth = CV_64F;
    /// Gradient X
    Sobel(totalCrSobel, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
    /// Gradient Y
    Sobel(totalCrSobel, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

    totalCrSobel = grad_x + grad_y;
    grad_x = grad_x.mul(grad_x);
    grad_y = grad_y.mul(grad_y);
    totalCrSobelMag = grad_x + grad_y;
  }

  Mat totalCbSobel = totalGraySobel.clone();
  Mat totalCbSobelMag;
  {
    for (int y = 0; y < imH; y++) {
      for (int x = 0; x < imW; x++) {
	cv::Vec3b thisColor = sobelYCrCbBlur.at<cv::Vec3b>(y,x);
	totalCbSobel.at<double>(y,x) = thisColor[2];
      }
    }
    Mat grad_x, grad_y;
    int sobelScale = 1;
    int sobelDelta = 0;
    int sobelDepth = CV_64F;
    /// Gradient X
    Sobel(totalCbSobel, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
    /// Gradient Y
    Sobel(totalCbSobel, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

    totalCbSobel = grad_x + grad_y;
    grad_x = grad_x.mul(grad_x);
    grad_y = grad_y.mul(grad_y);
    totalCbSobelMag = grad_x + grad_y;
  }

  Mat totalYSobel = totalGraySobel.clone();
  {
    for (int y = 0; y < imH; y++) {
      for (int x = 0; x < imW; x++) {
	cv::Vec3b thisColor = sobelYCrCbBlur.at<cv::Vec3b>(y,x);
	totalYSobel.at<double>(y,x) = thisColor[0];
      }
    }
    Mat grad_x, grad_y;
    int sobelScale = 1;
    int sobelDelta = 0;
    int sobelDepth = CV_64F;
    /// Gradient X
    Sobel(totalYSobel, grad_x, sobelDepth, 1, 0, 5, sobelScale, sobelDelta, BORDER_DEFAULT);
    /// Gradient Y
    Sobel(totalYSobel, grad_y, sobelDepth, 0, 1, 5, sobelScale, sobelDelta, BORDER_DEFAULT);

    grad_x = grad_x.mul(grad_x);
    grad_y = grad_y.mul(grad_y);
    totalYSobel = grad_x + grad_y;
  }

  // total becomes sum of Cr and Cb
  int totalBecomes = 1;
  if (totalBecomes) {
    totalGraySobel = totalCrSobelMag + totalCbSobelMag;
  }

  // truncate the Sobel image outside the gray box
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < ms->config.grayTop.y; y++) {
      totalGraySobel.at<double>(y,x) = 0;
      totalCrSobel.at<double>(y,x) = 0;
      totalCbSobel.at<double>(y,x) = 0;
      totalYSobel.at<double>(y,x) = 0;
    }
  }

  for (int x = 0; x < ms->config.grayTop.x; x++) {
    for (int y = ms->config.grayTop.y; y < ms->config.grayBot.y; y++) {
      totalGraySobel.at<double>(y,x) = 0;
      totalCrSobel.at<double>(y,x) = 0;
      totalCbSobel.at<double>(y,x) = 0;
      totalYSobel.at<double>(y,x) = 0;
    }
  }

  for (int x = ms->config.grayBot.x; x < imW; x++) {
    for (int y = ms->config.grayTop.y; y < ms->config.grayBot.y; y++) {
      totalGraySobel.at<double>(y,x) = 0;
      totalCrSobel.at<double>(y,x) = 0;
      totalCbSobel.at<double>(y,x) = 0;
      totalYSobel.at<double>(y,x) = 0;
    }
  }

  for (int x = 0; x < imW; x++) {
    for (int y = ms->config.grayBot.y; y < imH; y++) {
      totalGraySobel.at<double>(y,x) = 0;
      totalCrSobel.at<double>(y,x) = 0;
      totalCbSobel.at<double>(y,x) = 0;
      totalYSobel.at<double>(y,x) = 0;
    }
  }

  // input image is noisy so blurring is a good idea
  //GaussianBlur(ms->config.densityViewerImage, ms->config.densityViewerImage, cv::Size(0,0), 1.0);

  if (ms->config.integralDensity == NULL)
    ms->config.integralDensity = new double[imW*imH];
  if (ms->config.density == NULL)
    ms->config.density = new double[imW*imH];
  if (ms->config.preDensity == NULL)
    ms->config.preDensity = new double[imW*imH];
  if (ms->config.temporalDensity == NULL) {
    ms->config.temporalDensity = new double[imW*imH];
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	ms->config.temporalDensity[y*imW + x] = 0;
      }
    }
  }

  int replaceDensityWithGrad = 1;
  if (replaceDensityWithGrad) {
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	ms->config.density[y*imW+x] = totalGraySobel.at<double>(y,x);
      }
    }
  }

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      ms->config.preDensity[y*imW+x] = totalGraySobel.at<double>(y,x);
    }
  }

  // now update the exponential average of the density
  // and set the density to be a thresholded version of this
  ms->config.maxDensity = 0;
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      ms->config.maxDensity = max(ms->config.maxDensity, ms->config.density[y*imW+x]);
    }
  }
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      ms->config.temporalDensity[y*imW+x] = ms->config.densityDecay*ms->config.temporalDensity[y*imW+x] + (1.0-ms->config.densityDecay)*ms->config.density[y*imW+x];
    }
  }

  // optionally feed it back in
  int sobelBecomesDensity = 0;
  if (sobelBecomesDensity) {
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	totalGraySobel.at<double>(y,x) = ms->config.density[y*imW+x];
      }
    }
  }

  double maxGsob = -INFINITY;
  double maxYsob = -INFINITY;
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      maxGsob = max(maxGsob, totalGraySobel.at<double>(y,x));
      maxYsob = max(maxYsob, totalYSobel.at<double>(y,x));
    }
  }
  
  // ATTN 11
  // experimental
  int combineYandGray = 1;
  double yWeight = 1.0;
  if (combineYandGray) {
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	double thisY2G = min(maxYsob, yWeight * totalYSobel.at<double>(y,x));
	totalGraySobel.at<double>(y,x) += maxGsob * thisY2G * thisY2G / (maxYsob * maxYsob);
      }
    }
  }

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      if (ms->config.density[y*imW+x] < ms->config.maxDensity* ms->config.threshFraction)
	ms->config.density[y*imW+x] = 0;
    }
  }

  // smooth the density
  int smoothDensity = 1;
  double densitySigma = max(0.5, double(ms->config.postDensitySigmaTrackbarVariable));//3.0;
  Mat denTemp = totalGraySobel.clone();
  if (smoothDensity) {
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	denTemp.at<double>(y,x) = ms->config.density[y*imW+x];
      }
    }

    GaussianBlur(denTemp, denTemp, cv::Size(0,0), densitySigma);

    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	ms->config.density[y*imW+x] = denTemp.at<double>(y,x);
      }
    }
  }

  // inject some of the Y gradient map back in AFTER feeding back to totalGraySobel
  //   so that Y contributes to objectness to help catch objects with poor color contrast,
  //   but not to pose since it is corrupted by shadows.
  int injectYGrad = 1;
  double yThresh = 0.9*maxGsob;
  if (injectYGrad) {
    // truncate again after reinjection
    ms->config.maxDensity = 0;
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	ms->config.maxDensity = max(ms->config.maxDensity, ms->config.density[y*imW+x]);
      }
    }
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	if (totalYSobel.at<double>(y,x) > yThresh) {
	  ms->config.density[y*imW+x] += 0.5*ms->config.maxDensity;
	}
      }
    }

    // truncate again after reinjection
    ms->config.maxDensity = 0;
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	ms->config.maxDensity = max(ms->config.maxDensity, ms->config.density[y*imW+x]);
      }
    }
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	if (ms->config.density[y*imW+x] < ms->config.maxDensity* ms->config.threshFraction)
	  ms->config.density[y*imW+x] = 0;
      }
    }
  }

  if (ms->config.drawGray) {
    cv::Point outTop = cv::Point(ms->config.grayTop.x, ms->config.grayTop.y);
    cv::Point outBot = cv::Point(ms->config.grayBot.x, ms->config.grayBot.y);
    cv::Point inTop = cv::Point(ms->config.grayTop.x+1,ms->config.grayTop.y+1);
    cv::Point inBot = cv::Point(ms->config.grayBot.x-1,ms->config.grayBot.y-1);
    rectangle(ms->config.objectViewerImage, outTop, outBot, cv::Scalar(128,128,128));
    rectangle(ms->config.objectViewerImage, inTop, inBot, cv::Scalar(32,32,32));
  }

  if (ms->config.mask_gripper) {
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	if ( isInGripperMask(ms, x, y) ) {
	  ms->config.density[y*imW+x] = 0;
	  totalGraySobel.at<double>(y,x) = 0;
	  if (!isSketchyMat(ms->config.objectViewerImage)) {
	    ms->config.objectViewerImage.at<Vec3b>(y,x)[0] = 255;
	  }
	}
      }
    }
  }

  if (ms->config.mask_gripper_blocks) {
    int xs = ms->config.g1xs;
    int xe = ms->config.g1xe;
    int ys = ms->config.g1ys;
    int ye = ms->config.g1ye;
    for (int x = xs; x < xe; x++) {
      for (int y = ys; y < ye; y++) {
	ms->config.density[y*imW+x] = 0;
	totalGraySobel.at<double>(y,x) = 0;
      }
    }
    if (!isSketchyMat(ms->config.objectViewerImage)) {
      Mat vCrop = ms->config.objectViewerImage(cv::Rect(xs, ys, xe-xs, ye-ys));
      vCrop = vCrop/2;
    }
    xs = ms->config.g2xs;
    xe = ms->config.g2xe;
    ys = ms->config.g2ys;
    ye = ms->config.g2ye;
    for (int x = xs; x < xe; x++) {
      for (int y = ys; y < ye; y++) {
	ms->config.density[y*imW+x] = 0;
	totalGraySobel.at<double>(y,x) = 0;
      }
    }
    Mat vCrop2 = ms->config.objectViewerImage(cv::Rect(xs, ys, xe-xs, ye-ys));
    vCrop2 = vCrop2/2;
  }

  // truncate the density outside the gray box
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < ms->config.grayTop.y; y++) {
      ms->config.density[y*imW+x] = 0;
    }
  }

  for (int x = 0; x < ms->config.grayTop.x; x++) {
    for (int y = ms->config.grayTop.y; y < ms->config.grayBot.y; y++) {
      ms->config.density[y*imW+x] = 0;
    }
  }

  for (int x = ms->config.grayBot.x; x < imW; x++) {
    for (int y = ms->config.grayTop.y; y < ms->config.grayBot.y; y++) {
      ms->config.density[y*imW+x] = 0;
    }
  }

  for (int x = 0; x < imW; x++) {
    for (int y = ms->config.grayBot.y; y < imH; y++) {
      ms->config.density[y*imW+x] = 0;
    }
  }

  // integrate the density into the integral density
  //double maxIntegralDensity = 0;
  ms->config.integralDensity[0] = ms->config.density[0];
  for (int x = 1; x < imW; x++) {
    int y = 0;
    ms->config.integralDensity[y*imW+x] = ms->config.integralDensity[y*imW+(x-1)] + ms->config.density[y*imW + x];
  }
  for (int y = 1; y < imH; y++) {
    int x = 0;
    ms->config.integralDensity[y*imW+x] = ms->config.integralDensity[(y-1)*imW+x] + ms->config.density[y*imW + x];
  }
  for (int x = 1; x < imW; x++) {
    for (int y = 1; y < imH; y++) {
      ms->config.integralDensity[y*imW+x] = 
	ms->config.integralDensity[(y-1)*imW+x]+ms->config.integralDensity[y*imW+(x-1)]-ms->config.integralDensity[(y-1)*imW+(x-1)]+ms->config.density[y*imW + x];
    }
  }

  // copy the density map to the rendered image
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      uchar val = uchar(min( 255.0 * ms->config.density[y*imW+x] / ms->config.maxDensity, 255.0));
      ms->config.densityViewerImage.at<cv::Vec3b>(y,x) = cv::Vec<uchar, 3>(0,val,0);
    }
  }

  // masked this too
  ms->config.frameGraySobel = totalGraySobel.clone();
  ms->config.preFrameGraySobel = totalGraySobel.clone();

  { // temporal averaging of aerial gradient
    if ( (ms->config.aerialGradientTemporalFrameAverage.rows < ms->config.aerialGradientReticleWidth) ||
	 (ms->config.aerialGradientTemporalFrameAverage.cols < ms->config.aerialGradientReticleWidth) ) {
      ms->config.aerialGradientTemporalFrameAverage = Mat(imH,imW,ms->config.frameGraySobel.type()); 
    }

    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
	ms->config.aerialGradientTemporalFrameAverage.at<double>(y, x) = 
	  ms->config.aerialGradientDecay*ms->config.aerialGradientTemporalFrameAverage.at<double>(y, x) + 
	  (1.0 - ms->config.aerialGradientDecay)*ms->config.frameGraySobel.at<double>(y, x);
      }
    }
  }

  ms->config.frameGraySobel = ms->config.aerialGradientTemporalFrameAverage;


  double minGraySob = INFINITY;
  double maxGraySob = -INFINITY;
  double minCrSob = INFINITY;
  double maxCrSob = -INFINITY;
  double minCbSob = INFINITY;
  double maxCbSob = -INFINITY;
  double minYSob = INFINITY;
  double maxYSob = -INFINITY;
  double minAerTemp = INFINITY;
  double maxAerTemp = -INFINITY;
  for (int y = 0; y < imH; y++) {
    for (int x = 0; x < imW; x++) {
      minGraySob = min(minGraySob, double(totalGraySobel.at<double>(y,x)));
      maxGraySob = max(maxGraySob, double(totalGraySobel.at<double>(y,x)));

      minCrSob = min(minCrSob, double(totalCrSobel.at<double>(y,x)));
      maxCrSob = max(maxCrSob, double(totalCrSobel.at<double>(y,x)));

      minCbSob = min(minCbSob, double(totalCbSobel.at<double>(y,x)));
      maxCbSob = max(maxCbSob, double(totalCbSobel.at<double>(y,x)));

      minYSob = min(minYSob, double(totalYSobel.at<double>(y,x)));
      maxYSob = max(maxYSob, double(totalYSobel.at<double>(y,x)));
      
      minAerTemp = min(minAerTemp, double(ms->config.frameGraySobel.at<double>(y,x)));
      maxAerTemp = max(maxAerTemp, double(ms->config.frameGraySobel.at<double>(y,x)));
    }
  }

  double sobGrayRange = maxGraySob - minGraySob;
  double sobCrRange = maxCrSob - minCrSob;
  double sobCbRange = maxCbSob - minCbSob;
  double sobYRange = maxYSob - minYSob;
  double aerTempRange = maxAerTemp - minAerTemp;

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      yCbCrGradientImage.at<cv::Vec3b>(y,x) = cv::Vec<uchar, 3>(
	uchar(max(0.0, min((128+255.0*(totalYSobel.at<double>(y,x) - minYSob - (sobYRange/2.0)) / sobYRange), 255.0))) ,
	uchar(max(0.0, min((128+255.0*(totalCrSobel.at<double>(y,x) - minCrSob - (sobCrRange/2.0)) / sobCrRange), 255.0))) ,
	uchar(max(0.0, min((128+255.0*(totalCbSobel.at<double>(y,x) - minCbSob - (sobCbRange/2.0)) / sobCbRange), 255.0))) );
    }
  }
  Mat convertedYCbCrGradientImage;
  cvtColor(yCbCrGradientImage, convertedYCbCrGradientImage, CV_YCrCb2BGR);

  // copy the density map to the rendered image
  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      //uchar val = uchar(min( 1*255.0 *  (totalGraySobel.at<double>(y,x) - minGraySob) / sobGrayRange, 255.0));
      uchar val = uchar(min( 1*255.0 *  (ms->config.frameGraySobel.at<double>(y,x) - minAerTemp) / aerTempRange, 255.0));
      ms->config.gradientViewerImage.at<cv::Vec3b>(y,x) = cv::Vec<uchar, 3>(0,val,0);

      ms->config.gradientViewerImage.at<cv::Vec3b>(y+imH,x) = convertedYCbCrGradientImage.at<cv::Vec3b>(y,x);
    }
  }

  if (ms->config.shouldIRender) {
    guardedImshow(ms->config.densityViewerName, ms->config.densityViewerImage, ms->config.sirDensity);
    guardedImshow(ms->config.gradientViewerName, ms->config.gradientViewerImage, ms->config.sirGradient);
  }
}

void goFindBlueBoxes(shared_ptr<MachineState> ms) {
  Size sz = ms->config.objectViewerImage.size();
  int imW = sz.width;
  int imH = sz.height;

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

	double thisDistance = sqrt((ms->config.bCens[t].x-ms->config.reticle.px)*(ms->config.bCens[t].x-ms->config.reticle.px) + (ms->config.bCens[t].y-ms->config.reticle.py)*(ms->config.bCens[t].y-ms->config.reticle.py));
	cout << "   (density) Distance for box " << t << " : " << thisDistance << endl;
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

  if (ms->config.shouldIRender) {
    guardedImshow(ms->config.objectViewerName, ms->config.objectViewerImage, ms->config.sirObject);
  }

  delete ms->config.gBoxIndicator;
  delete gBoxGrayNodes;
  delete gBoxComponentLabels;
}


void goClassifyBlueBoxes(shared_ptr<MachineState> ms) {
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

  for (int c = 0; c < ms->config.bTops.size(); c++) {
    vector<KeyPoint>& keypoints = ms->config.bKeypoints[c];
    Mat descriptors;
    Mat descriptors2;

    Mat original_cam_img = ms->config.cam_img;
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
      double thisDistance = sqrt((ms->config.bCens[c].x-ms->config.reticle.px)*(ms->config.bCens[c].x-ms->config.reticle.px) + (ms->config.bCens[c].y-ms->config.reticle.py)*(ms->config.bCens[c].y-ms->config.reticle.py));
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

  if (ms->config.shouldIRender) {
    guardedImshow(ms->config.objectViewerName, ms->config.objectViewerImage, ms->config.sirObject);
  }

}


// TODO probably don't need two separate functions for this
void loadROSParamsFromArgs(shared_ptr<MachineState> ms) {
  ros::NodeHandle nh("~");


  cout << "nh namespace: " << nh.getNamespace() << endl;

  nh.getParam("vocab_file", ms->config.vocab_file);
  nh.getParam("knn_file", ms->config.knn_file);
  nh.getParam("label_file", ms->config.label_file);

  nh.getParam("data_directory", ms->config.data_directory);

  nh.getParam("run_prefix", ms->config.run_prefix);

  nh.getParam("all_range_mode", ms->config.all_range_mode);

  nh.getParam("gray_box_top", ms->config.tGO);
  nh.getParam("gray_box_bot", ms->config.bGO);
  nh.getParam("gray_box_left", ms->config.lGO);
  nh.getParam("gray_box_right", ms->config.rGO);

  nh.getParam("arm_box_top", ms->config.tARM);
  nh.getParam("arm_box_bot", ms->config.bARM);
  nh.getParam("arm_box_left", ms->config.lARM);
  nh.getParam("arm_box_right", ms->config.rARM);

  nh.getParam("image_topic", ms->config.image_topic);

  nh.getParam("retrain_vocab", ms->config.retrain_vocab);
  nh.getParam("reextract_knn", ms->config.reextract_knn);
  nh.getParam("rewrite_labels", ms->config.rewrite_labels);

  nh.getParam("cache_prefix", ms->config.cache_prefix);

  nh.getParam("mask_gripper", ms->config.mask_gripper);

  nh.getParam("left_or_right_arm", ms->config.left_or_right_arm);

  //nh.getParam("ms->config.chosen_feature", cfi);
  //ms->config.chosen_feature = static_cast<featureType>(cfi);


  nh.getParam("use_simulator", ms->config.use_simulator);
  if (ms->config.use_simulator) {
    ms->config.currentRobotMode = SIMULATED;
  } else {
    ms->config.currentRobotMode = PHYSICAL;
  } 
}

void loadROSParams(shared_ptr<MachineState> ms) {
  ros::NodeHandle nh("~");

  nh.getParam("threshold_fraction", ms->config.threshFraction);
  nh.getParam("reject_scale", ms->config.rejectScale);
  nh.getParam("reject_area_scale", ms->config.rejectAreaScale);
  nh.getParam("density_decay", ms->config.densityDecay);

  nh.getParam("data_directory", ms->config.data_directory);
  nh.getParam("run_prefix", ms->config.run_prefix);
  nh.getParam("all_range_mode", ms->config.all_range_mode);

  nh.getParam("gray_box_top", ms->config.tGO);
  nh.getParam("gray_box_bot", ms->config.bGO);
  nh.getParam("gray_box_left", ms->config.lGO);
  nh.getParam("gray_box_right", ms->config.rGO);

  nh.getParam("arm_box_top", ms->config.tARM);
  nh.getParam("arm_box_bot", ms->config.bARM);
  nh.getParam("arm_box_left", ms->config.lARM);
  nh.getParam("arm_box_right", ms->config.rARM);

  nh.getParam("image_topic", ms->config.image_topic);

  nh.getParam("retrain_vocab", ms->config.retrain_vocab);
  nh.getParam("reextract_knn", ms->config.reextract_knn);
  nh.getParam("rewrite_labels", ms->config.rewrite_labels);

  nh.getParam("sobel_sigma", ms->config.sobel_sigma);
  nh.getParam("canny_hi_thresh",ms->config.canny_hi_thresh);
  nh.getParam("canny_lo_thresh",ms->config.canny_lo_thresh);
  nh.getParam("sobel_scale_factor",ms->config.sobel_scale_factor);

  nh.getParam("mask_gripper", ms->config.mask_gripper);

  nh.getParam("left_or_right_arm", ms->config.left_or_right_arm);

}

void saveROSParams(shared_ptr<MachineState> ms) {
  ros::NodeHandle nh("~");

  nh.setParam("threshold_fraction", ms->config.threshFraction);
  nh.setParam("reject_scale", ms->config.rejectScale);
  nh.setParam("reject_area_scale", ms->config.rejectAreaScale);
  nh.setParam("density_decay", ms->config.densityDecay);

  nh.setParam("data_directory", ms->config.data_directory);
  nh.setParam("run_prefix", ms->config.run_prefix);
  nh.setParam("all_range_mode", ms->config.all_range_mode);


  nh.setParam("gray_box_top", ms->config.tGO);
  nh.setParam("gray_box_bot", ms->config.bGO);
  nh.setParam("gray_box_left", ms->config.lGO);
  nh.setParam("gray_box_right", ms->config.rGO);

  nh.setParam("arm_box_top", ms->config.tARM);
  nh.setParam("arm_box_bot", ms->config.bARM);
  nh.setParam("arm_box_left", ms->config.lARM);
  nh.setParam("arm_box_right", ms->config.rARM);

  nh.setParam("image_topic", ms->config.image_topic);

  nh.setParam("retrain_vocab", ms->config.retrain_vocab);
  nh.setParam("reextract_knn", ms->config.reextract_knn);
  nh.setParam("rewrite_labels", ms->config.rewrite_labels);

  nh.setParam("sobel_sigma", ms->config.sobel_sigma);
  nh.setParam("canny_hi_thresh",ms->config.canny_hi_thresh);
  nh.setParam("canny_lo_thresh",ms->config.canny_lo_thresh);
  nh.setParam("sobel_scale_factor",ms->config.sobel_scale_factor);

  nh.setParam("mask_gripper", ms->config.mask_gripper);

  nh.setParam("left_or_right_arm", ms->config.left_or_right_arm);

  //nh.setParam("ms->config.chosen_feature", cfi);
  //ms->config.chosen_feature = static_cast<featureType>(cfi);

}

void spinlessNodeMain(shared_ptr<MachineState> ms) {
  cout << endl << endl << "Node main begin..." << endl;
  nodeInit(ms);
  detectorsInit(ms);
}

void nodeInit(shared_ptr<MachineState> ms) {
  ms->config.gBoxStrideX = ms->config.gBoxW / 2.0;
  ms->config.gBoxStrideY = ms->config.gBoxH / 2.0;
  ms->config.cropCounter = 0;

}

void detectorsInit(shared_ptr<MachineState> ms) {

  // XXX TODO this function should reinit the structures if this function is to be called multiple times

  // SIFT 
  //ms->config.detector = new SiftFeatureDetector(0, 3, 0.04, 10, 1.6);
  cout << "ms->config.chosen_feature: " << ms->config.chosen_feature << endl;
  if (ms->config.detector == NULL)
    ms->config.detector = new FastFeatureDetector(4);

  if (ms->config.extractor == NULL) {
    if (ms->config.chosen_feature == SIFTBOW_GLOBALCOLOR_HIST)
      ms->config.extractor = new SiftDescriptorExtractor();
    else if (ms->config.chosen_feature == OPPONENTSIFTBOW_GLOBALCOLOR_HIST)
      ms->config.extractor = DescriptorExtractor::create("OpponentSIFT");
    else {
      ms->config.extractor = new SiftDescriptorExtractor();
    }
  }
  
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

  initRangeMaps(ms);


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
                         ms->config.classQuaternions[i], 0, ms->config.keypointPeriod, ms->config.bowExtractor);
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
      ROS_ERROR_STREAM("Could not find file " << featuresPath << endl);
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


void tryToLoadRangeMap(shared_ptr<MachineState> ms, std::string classDir, const char *className, int i) {
  {
    string thisLabelName(className);

    string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/ir2D/";
    string this_range_path = dirToMakePath + "xyzRange.yml";

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
      cout << "Loaded rangeMap from " << this_range_path << ms->config.classRangeMaps[i].size() << endl; 
      cout << "Loaded classGraspMemoryTries1 from " << this_range_path << ms->config.classGraspMemoryTries1[i].size() << endl; 
      cout << "Loaded classGraspMemoryPicks1 from " << this_range_path << ms->config.classGraspMemoryPicks1[i].size() << endl; 
      cout << "Loaded classGraspMemoryTries2 from " << this_range_path << ms->config.classGraspMemoryTries2[i].size() << endl; 
      cout << "Loaded classGraspMemoryPicks2 from " << this_range_path << ms->config.classGraspMemoryPicks2[i].size() << endl; 
      cout << "Loaded classGraspMemoryTries3 from " << this_range_path << ms->config.classGraspMemoryTries3[i].size() << endl; 
      cout << "Loaded classGraspMemoryPicks3 from " << this_range_path << ms->config.classGraspMemoryPicks3[i].size() << endl; 
      cout << "Loaded classGraspMemoryTries4 from " << this_range_path << ms->config.classGraspMemoryTries4[i].size() << endl; 
      cout << "Loaded classGraspMemoryPicks4 from " << this_range_path << ms->config.classGraspMemoryPicks4[i].size() << endl; 

      cout << "Loaded classHeightMemoryTries from " << this_range_path << ms->config.classHeightMemoryTries[i].size() << endl;
      cout << "Loaded classHeightMemoryPicks from " << this_range_path << ms->config.classHeightMemoryPicks[i].size() << endl;
    } else {
      ms->config.classRangeMaps[i] = Mat(1, 1, CV_64F);
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

      cout << "Failed to load rangeMap from " << this_range_path << endl; 
    }
  }
  {
    {
      string thisLabelName(className);

      string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/aerialGradient/";
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
      string thisLabelName(className);

      string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/aerialGradient/";
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
      string thisLabelName(className);

      string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/aerialGradient/";
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
      string thisLabelName(className);

      string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/aerialGradient/";
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
      string thisLabelName(className);
      string dirToMakePath = ms->config.data_directory + "/objects/" + thisLabelName + "/3dGrasps/";
      string this_grasp_path = dirToMakePath + "3dGrasps.yml";

      FileStorage fsvI;
      cout << "Reading grasp information from " << this_grasp_path << " ..."; cout.flush();
      fsvI.open(this_grasp_path, FileStorage::READ);

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
	  eePose buf;
	  buf.readFromFileNodeIterator(itc);
	  cout << " read pose: " << buf; cout.flush();
	  ms->config.class3dGrasps[i].push_back(buf);
	}
	if (numLoadedPoses != tng) {
	  ROS_ERROR_STREAM("Did not load the expected number of poses.");
	}
	cout << "Expected to load " << tng << " poses, loaded " << numLoadedPoses << " ..." << endl; cout.flush();
      }
      cout << "done.";
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


void mapxyToij(shared_ptr<MachineState> ms, double x, double y, int * i, int * j) 
{
  *i = round((x - ms->config.mapXMin) / ms->config.mapStep);
  *j = round((y - ms->config.mapYMin) / ms->config.mapStep);
}
void mapijToxy(shared_ptr<MachineState> ms, int i, int j, double * x, double * y) 
{
  *x = ms->config.mapXMin + i * ms->config.mapStep;
  *y = ms->config.mapYMin + j * ms->config.mapStep;
}
bool cellIsSearched(shared_ptr<MachineState> ms, int i, int j) {
  double x, y;
  mapijToxy(ms, i, j, &x, &y);
  return positionIsSearched(ms, x, y);
}

bool positionIsSearched(shared_ptr<MachineState> ms, double x, double y) {
  if ((ms->config.mapSearchFenceXMin <= x && x <= ms->config.mapSearchFenceXMax) &&
      (ms->config.mapSearchFenceYMin <= y && y <= ms->config.mapSearchFenceYMax)) {
    return true;
  } else {
    return false;
  }

}


gsl_matrix * mapCellToPolygon(shared_ptr<MachineState> ms, int map_i, int map_j) {
  
  double min_x, min_y;
  mapijToxy(ms, map_i, map_j, &min_x, &min_y);
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

bool isBoxMemoryIkPossible(shared_ptr<MachineState> ms, BoxMemory b) {
  int toReturn = 1;
  {
    int i, j;
    mapxyToij(ms, b.top.px, b.top.py, &i, &j);
    toReturn &= isCellIkPossible(ms, i, j);
  }
  {
    int i, j;
    mapxyToij(ms, b.bot.px, b.bot.py, &i, &j);
    toReturn &= isCellIkPossible(ms, i, j);
  }
  {
    int i, j;
    mapxyToij(ms, b.bot.px, b.top.py, &i, &j);
    toReturn &= isCellIkPossible(ms, i, j);
  }
  {
    int i, j;
    mapxyToij(ms, b.top.px, b.bot.py, &i, &j);
    toReturn &= isCellIkPossible(ms, i, j);
  }
  return toReturn;
}

bool isBlueBoxIkPossible(shared_ptr<MachineState> ms, cv::Point tbTop, cv::Point tbBot) {
  double zToUse = ms->config.trueEEPose.position.z+ms->config.currentTableZ;
  int toReturn = 1;
  {
    double tbx, tby;
    int tbi, tbj;
    pixelToGlobal(ms, tbTop.x, tbTop.y, zToUse, &tbx, &tby);
    mapxyToij(ms, tbx, tby, &tbi, &tbj);
    toReturn &= isCellIkPossible(ms, tbi, tbj);
  }
  {
    double tbx, tby;
    int tbi, tbj;
    pixelToGlobal(ms, tbBot.x, tbBot.y, zToUse, &tbx, &tby);
    mapxyToij(ms, tbx, tby, &tbi, &tbj);
    toReturn &= isCellIkPossible(ms, tbi, tbj);
  }
  {
    double tbx, tby;
    int tbi, tbj;
    pixelToGlobal(ms, tbTop.x, tbBot.y, zToUse, &tbx, &tby);
    mapxyToij(ms, tbx, tby, &tbi, &tbj);
    toReturn &= isCellIkPossible(ms, tbi, tbj);
  }
  {
    double tbx, tby;
    int tbi, tbj;
    pixelToGlobal(ms, tbBot.x, tbTop.y, zToUse, &tbx, &tby);
    mapxyToij(ms, tbx, tby, &tbi, &tbj);
    toReturn &= isCellIkPossible(ms, tbi, tbj);
  }
  return toReturn;
}

bool boxMemoryIntersectsMapCell(shared_ptr<MachineState> ms, BoxMemory b, int map_i, int map_j) {
  gsl_matrix * bpolygon = boxMemoryToPolygon(b);

  gsl_matrix * map_cell = mapCellToPolygon(ms, map_i, map_j);

  bool result = math2d_overlaps(bpolygon, map_cell);
  gsl_matrix_free(bpolygon);
  gsl_matrix_free(map_cell);

  return result;
}

bool boxMemoryTooOld(BoxMemory b) {
  
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


vector<BoxMemory> memoriesForClass(shared_ptr<MachineState> ms, int classIdx) {
  vector<BoxMemory> results;
  for (int j = 0; j < ms->config.blueBoxMemories.size(); j++) {
    if (ms->config.blueBoxMemories[j].labeledClassIndex == ms->config.focusedClass) {
      results.push_back(ms->config.blueBoxMemories[j]);
    }
  }
  return results;
}

vector<BoxMemory> memoriesForClass(shared_ptr<MachineState> ms, int classIdx, int * memoryIdxOfFirst) {
  vector<BoxMemory> results;
  int haventFoundFirst = 1;
  for (int j = 0; j < ms->config.blueBoxMemories.size(); j++) {
    if (ms->config.blueBoxMemories[j].labeledClassIndex == ms->config.focusedClass) {
      results.push_back(ms->config.blueBoxMemories[j]);
      if ( haventFoundFirst && (ms->config.blueBoxMemories[j].lockStatus == POSE_REPORTED) ) {
	*memoryIdxOfFirst = j;
	haventFoundFirst = 0;
      }
    }
  }
  return results;
}

bool cellIsMapped(shared_ptr<MachineState> ms, int i, int j) {
  double x, y;
  mapijToxy(ms, i, j, &x, &y);
  return positionIsMapped(ms, x, y);
}
bool positionIsMapped(shared_ptr<MachineState> ms, double x, double y) {
  if ((ms->config.mapRejectFenceXMin <= x && x <= ms->config.mapRejectFenceXMax) &&
      (ms->config.mapRejectFenceYMin <= y && y <= ms->config.mapRejectFenceYMax)) {
    return true;
  } else {
    return false;
  }
}

// TODO XXX make clearance status enum
bool isCellInPursuitZone(shared_ptr<MachineState> ms, int i, int j) {
  return ( (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 1) ||
	   (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 2) );
} 
bool isCellInPatrolZone(shared_ptr<MachineState> ms, int i, int j) {
  return (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 2);
} 

bool isCellInteresting(shared_ptr<MachineState> ms, int i, int j) {
  if ( (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 1) ||
       (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 2) ) {
    return ( ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime < ms->config.lastScanStarted );
  } else {
    return false;
  }
} 
void markCellAsInteresting(shared_ptr<MachineState> ms, int i, int j) {
  if ( (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 1) ||
       (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 2) ) {
    ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime = ros::Time(0.001);
    return;
  } else {
    return;
  }
} 
void markCellAsNotInteresting(shared_ptr<MachineState> ms, int i, int j) {
  if ( (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 1) ||
       (ms->config.clearanceMap[i + ms->config.mapWidth * j] == 2) ) {
    ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime = ros::Time::now() + ros::Duration(VERYBIGNUMBER);
    return;
  } else {
    return;
  }
} 

bool isCellIkColliding(shared_ptr<MachineState> ms, int i, int j) {
  return (ms->config.ikMap[i + ms->config.mapWidth * j] == 2);
} 
bool isCellIkPossible(shared_ptr<MachineState> ms, int i, int j) {
  return (ms->config.ikMap[i + ms->config.mapWidth * j] == 0);
} 
bool isCellIkImpossible(shared_ptr<MachineState> ms, int i, int j) {
  return (ms->config.ikMap[i + ms->config.mapWidth * j] == 1);
} 


int blueBoxForPixel(shared_ptr<MachineState> ms, int px, int py)
{
  for (int c = 0; c < ms->config.bTops.size(); c++) {
    if ((ms->config.bTops[c].x <= px && px <= ms->config.bBots[c].x) &&
        (ms->config.bTops[c].y <= py && py <= ms->config.bBots[c].y)) {
      return c;
    }
  }
  return -1;
}

int skirtedBlueBoxForPixel(shared_ptr<MachineState> ms, int px, int py, int skirtPixels) {
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

void randomizeNanos(shared_ptr<MachineState> ms, ros::Time * time) {
  double nanoseconds = rk_double(&ms->config.random_state) * 1000;
  time->nsec = nanoseconds;
}

void voidMapRegion(shared_ptr<MachineState> ms, double xc, double yc) {
  double voidRegionWidth = 0.1;
  double voidTimeGap = 60.0;

  int mxs=0,mxe=0,mys=0,mye=0;
  mapxyToij(ms, xc-voidRegionWidth, yc-voidRegionWidth, &mxs, &mys);
  mapxyToij(ms, xc+voidRegionWidth, yc+voidRegionWidth, &mxe, &mye);
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

void markMapAsCompleted(shared_ptr<MachineState> ms) {
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

void clearMapForPatrol(shared_ptr<MachineState> ms) {
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

void initializeMap(shared_ptr<MachineState> ms) {
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

      ms->config.ikMap[i + ms->config.mapWidth * j] = 0;
      ms->config.clearanceMap[i + ms->config.mapWidth * j] = 0;
    }
  }
  ms->config.lastScanStarted = ros::Time::now();
}


void guardViewers(shared_ptr<MachineState> ms) {
  if ( isSketchyMat(ms->config.objectViewerYCbCrBlur) ) {
    ms->config.objectViewerYCbCrBlur = Mat(ms->config.cv_ptr->image.rows, ms->config.cv_ptr->image.cols, CV_64FC3);
  }
  if ( isSketchyMat(ms->config.objectViewerGrayBlur) ) {
    ms->config.objectViewerGrayBlur = Mat(ms->config.cv_ptr->image.rows, ms->config.cv_ptr->image.cols, CV_64FC3);
  }
  if ( isSketchyMat(ms->config.densityViewerImage) ) {
    ms->config.densityViewerImage = ms->config.cv_ptr->image.clone();
    ms->config.densityViewerImage *= 0;
  }
  if ( isSketchyMat(ms->config.accumulatedImage) ) {
    ms->config.accumulatedImage = Mat(ms->config.cv_ptr->image.rows, ms->config.cv_ptr->image.cols, CV_64FC3);
  }
  if ( isSketchyMat(ms->config.accumulatedImageMass) ) {
    ms->config.accumulatedImageMass = Mat(ms->config.cv_ptr->image.rows, ms->config.cv_ptr->image.cols, CV_64F);
  }
  if ( isSketchyMat(ms->config.gradientViewerImage) ) {
    ms->config.gradientViewerImage = Mat(2*ms->config.cv_ptr->image.rows, ms->config.cv_ptr->image.cols, ms->config.cv_ptr->image.type());
  }
  if ( isSketchyMat(ms->config.aerialGradientViewerImage) ) {
    ms->config.aerialGradientViewerImage = Mat(4*ms->config.aerialGradientWidth, ms->config.aerialGradientWidth, CV_64F);
  }
  if ( isSketchyMat(ms->config.objectViewerImage) ) {
    ms->config.objectViewerImage = ms->config.cv_ptr->image.clone();
  }
}




////////////////////////////////////////////////
// end node definitions 
//
// start ein 
////////////////////////////////////////////////

void fillRecognizedObjectArrayFromBlueBoxMemory(shared_ptr<MachineState> ms, object_recognition_msgs::RecognizedObjectArray * roa) {
  roa->objects.resize(0);

  roa->header.stamp = ros::Time::now();
  roa->header.frame_id = "/base";

  for (int j = 0; j < ms->config.blueBoxMemories.size(); j++) {
    if (ms->config.blueBoxMemories[j].lockStatus == POSE_LOCK ||
	ms->config.blueBoxMemories[j].lockStatus == POSE_REPORTED) {
      ms->config.blueBoxMemories[j].lockStatus = POSE_LOCK;
    }
  }

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

void fillEinStateMsg(shared_ptr<MachineState> ms, EinState * stateOut) {
  stateOut->zero_g = ms->config.zero_g_toggle;

  stateOut->movement_state = ms->config.currentMovementState;
  stateOut->patrol_state = ms->config.currentPatrolState;
  stateOut->patrol_mode = ms->config.currentPatrolMode;
  stateOut->place_mode = ms->config.currentPlaceMode;
  stateOut->idle_mode = ms->config.currentIdleMode;

  object_recognition_msgs::RecognizedObjectArray roa;
  fillRecognizedObjectArrayFromBlueBoxMemory(ms, &roa);

  for (int i = 0; i < roa.objects.size(); i++) {
    stateOut->objects.push_back(roa.objects[i]);
  }
}

int main(int argc, char **argv) {

  initializeWords();
  pMachineState = std::make_shared<MachineState>(machineState);
  shared_ptr<MachineState> ms = pMachineState;
  initializeMachine(ms);

  initVectorArcTan(ms);
  srand(time(NULL));
  time(&ms->config.firstTime);
  time(&ms->config.firstTimeRange);

  cout << "argc: " << argc << endl;
  for (int ccc = 0; ccc < argc; ccc++) {
    cout << argv[ccc] << endl;
  }
  cout << endl << endl;

  string programName;
  if (argc > 1) {
    programName = string(PROGRAM_NAME) + "_" + argv[argc-1];
    cout << "programName: " << programName << endl;
  }
  else {
    programName = string(PROGRAM_NAME);
  }

  ros::init(argc, argv, programName);
  ros::NodeHandle n("~");

  cout << "n namespace: " << n.getNamespace() << endl;

  loadROSParamsFromArgs(ms);
  cout << "mask_gripper: " << ms->config.mask_gripper << endl;
  cout << "all_range_mode: " << ms->config.all_range_mode << endl;
  cout << "data_directory: " << ms->config.data_directory << endl 
       << "run_prefix: " << ms->config.run_prefix << endl << endl 
       << "vocab_file: " << ms->config.vocab_file << endl 
       << "knn_file: " << ms->config.knn_file << endl << "label_file: " << ms->config.label_file << endl
       << endl;

  ms->config.class_crops_path = ms->config.data_directory + "/objects/";

  unsigned long seed = 1;
  rk_seed(seed, &ms->config.random_state);

  if ( (ms->config.left_or_right_arm.compare("right") == 0) || (ms->config.left_or_right_arm.compare("left") == 0) ) {
    ms->config.image_topic = "/cameras/" + ms->config.left_or_right_arm + "_hand_camera/image";
  }

  image_transport::ImageTransport it(n);
  image_transport::Subscriber image_sub;

  ros::Subscriber epState;
  ros::Subscriber gripState;
  ros::Subscriber eeAccelerator;
  ros::Subscriber eeRanger;
  ros::Subscriber eeTarget;
  ros::Subscriber jointSubscriber;


  ros::Subscriber pickObjectUnderEndEffectorCommandCallbackSub;
  ros::Subscriber placeObjectInEndEffectorCommandCallbackSub;
  ros::Subscriber moveEndEfffectorCommandCallbackSub;

  ms->config.rec_objs_blue_memory = n.advertise<object_recognition_msgs::RecognizedObjectArray>("blue_memory_objects", 10);
  ms->config.markers_blue_memory = n.advertise<visualization_msgs::MarkerArray>("blue_memory_markers", 10);

  ms->config.ee_target_pub = n.advertise<geometry_msgs::Point>("pilot_target_" + ms->config.left_or_right_arm, 10);

  ms->config.densityViewerName = "Density Viewer " + ms->config.left_or_right_arm;
  ms->config.objectViewerName = "Object Viewer " + ms->config.left_or_right_arm;
  ms->config.gradientViewerName = "Gradient Viewer " + ms->config.left_or_right_arm;
  ms->config.aerialGradientViewerName = "Aerial Gradient Viewer " + ms->config.left_or_right_arm;
  ms->config.stereoViewerName = "Stereo Viewer " + ms->config.left_or_right_arm;

  cv::namedWindow(ms->config.gradientViewerName);
  cv::namedWindow(ms->config.aerialGradientViewerName);
  cv::namedWindow(ms->config.densityViewerName);

  createTrackbar("post_density_sigma", ms->config.densityViewerName, &ms->config.postDensitySigmaTrackbarVariable, 40);
  createTrackbar("canny_lo", ms->config.densityViewerName, &ms->config.loTrackbarVariable, 100);
  createTrackbar("canny_hi", ms->config.densityViewerName, &ms->config.hiTrackbarVariable, 100);

  ros::Timer simulatorCallbackTimer;

  if (ms->config.currentRobotMode == PHYSICAL) {
    epState =   n.subscribe("/robot/limb/" + ms->config.left_or_right_arm + "/endpoint_state", 1, endpointCallback);
    gripState = n.subscribe("/robot/end_effector/" + ms->config.left_or_right_arm + "_gripper/state", 1, gripStateCallback);
    eeAccelerator =  n.subscribe("/robot/accelerometer/" + ms->config.left_or_right_arm + "_accelerometer/state", 1, accelerometerCallback);
    eeRanger =  n.subscribe("/robot/range/" + ms->config.left_or_right_arm + "_hand_range/state", 1, rangeCallback);
    eeTarget =  n.subscribe("/ein_" + ms->config.left_or_right_arm + "/pilot_target_" + ms->config.left_or_right_arm, 1, targetCallback);
    jointSubscriber = n.subscribe("/robot/joint_states", 1, jointCallback);
    image_sub = it.subscribe(ms->config.image_topic, 1, imageCallback);
  } else if (ms->config.currentRobotMode == SIMULATED) {
    cout << "SIMULATION mode enabled." << endl;
    simulatorCallbackTimer = n.createTimer(ros::Duration(1.0/ms->config.simulatorCallbackFrequency), simulatorCallback);


    { // load sprites
      // snoop data/sprites folder
      //   loop through subfolders
      //     load image.ppm for now, default everything else
      vector<string> spriteLabels;
      spriteLabels.resize(0);
      ms->config.masterSprites.resize(0);
      ms->config.instanceSprites.resize(0);
      DIR *dpdf;
      struct dirent *epdf;
      string dot(".");
      string dotdot("..");

      char buf[1024];
      sprintf(buf, "%s/simulator/sprites", ms->config.data_directory.c_str());
      dpdf = opendir(buf);
      if (dpdf != NULL){
	while (epdf = readdir(dpdf)){
	  string thisFileName(epdf->d_name);

	  string thisFullFileName(buf);
	  thisFullFileName = thisFullFileName + "/" + thisFileName;
	  cout << "checking " << thisFullFileName << " during sprite snoop...";

	  struct stat buf2;
	  stat(thisFullFileName.c_str(), &buf2);

	  int itIsADir = S_ISDIR(buf2.st_mode);
	  if (dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name) && itIsADir) {
	    spriteLabels.push_back(thisFileName);
	    cout << " is a directory." << endl;
	  } else {
	    cout << " is NOT a directory." << endl;
	  }
	}
      }

      ms->config.masterSprites.resize(spriteLabels.size());
      for (int s = 0; s < ms->config.masterSprites.size(); s++) {
	ms->config.masterSprites[s].name = spriteLabels[s];
	string filename = ms->config.data_directory + "/simulator/sprites/" + ms->config.masterSprites[s].name + "/image.ppm";
	cout << "loading sprite from " << filename << " ... ";

	Mat tmp = imread(filename);
	ms->config.masterSprites[s].image = tmp;
	ms->config.masterSprites[s].scale = 15/.01;

	ms->config.masterSprites[s].top = eePose::zero();
	ms->config.masterSprites[s].bot = eePose::zero();
	ms->config.masterSprites[s].pose = eePose::zero();
	cout << "loaded " << ms->config.masterSprites[s].name << " as masterSprites[" << s << "] scale " << ms->config.masterSprites[s].scale << " image size " << ms->config.masterSprites[s].image.size() << endl;
      }
    }

    // load background
    int tileBackground = 1;
    if (tileBackground) {
      string filename;
      filename = ms->config.data_directory + "/simulator/tableTile.png";
      cout << "loading mapBackgroundImage from " << filename << " "; cout.flush();
      Mat tmp = imread(filename);
      cout << "done. Tiling " << tmp.size() << " "; cout.flush();
      //cout << "downsampling... "; cout.flush();
      //cv::resize(tmp, tmp, cv::Size(tmp.cols/2,tmp.rows/2));
      cv::resize(tmp, ms->config.mapBackgroundImage, cv::Size(ms->config.mbiWidth,ms->config.mbiHeight));

      int tilesWidth = ms->config.mbiWidth / tmp.cols;
      int tilesHeight = ms->config.mbiHeight / tmp.rows;

      for (int tx = 0; tx < tilesWidth; tx++) {
	for (int ty = 0; ty < tilesHeight; ty++) {
	  Mat crop = ms->config.mapBackgroundImage(cv::Rect(tx*tmp.cols, ty*tmp.rows, tmp.cols, tmp.rows));
	  resize(tmp, crop, crop.size(), 0, 0, CV_INTER_LINEAR);
	  if (tx % 2) {
	    flip(crop, crop, 1);
	  }
	  if ((ty) % 2) {
	    flip(crop, crop, 0);
	  }
	}
      }

      cout << "done. " << ms->config.mapBackgroundImage.size() << endl; cout.flush();
    } else {
      string filename;
      //filename = ms->config.data_directory + "/mapBackground.ppm";
      filename = ms->config.data_directory + "/simulator/carpetBackground.jpg";
      cout << "loading mapBackgroundImage from " << filename << " "; cout.flush();
      Mat tmp = imread(filename);
      cout << "done. Resizing " << tmp.size() << " "; cout.flush();
      cv::resize(tmp, ms->config.mapBackgroundImage, cv::Size(ms->config.mbiWidth,ms->config.mbiHeight));
      cout << "done. " << ms->config.mapBackgroundImage.size() << endl; cout.flush();
    }
    ms->config.originalMapBackgroundImage = ms->config.mapBackgroundImage.clone();
  }  else {
    assert(0);
  }

  pickObjectUnderEndEffectorCommandCallbackSub = n.subscribe("/ein/eePickCommand", 1, pickObjectUnderEndEffectorCommandCallback);
  placeObjectInEndEffectorCommandCallbackSub = n.subscribe("/ein/eePlaceCommand", 1, placeObjectInEndEffectorCommandCallback);
  moveEndEfffectorCommandCallbackSub = n.subscribe("/ein/eeMoveCommand", 1, moveEndEffectorCommandCallback);

  ms->config.wristViewName = "Wrist View " + ms->config.left_or_right_arm;
  ms->config.coreViewName = "Core View " + ms->config.left_or_right_arm;
  ms->config.faceViewName = "Face View " + ms->config.left_or_right_arm;
  ms->config.rangeogramViewName = "Rangeogram View " + ms->config.left_or_right_arm;
  ms->config.rangemapViewName = "Range Map View " + ms->config.left_or_right_arm;
  ms->config.graspMemoryViewName = "Grasp Memory View " + ms->config.left_or_right_arm;
  ms->config.graspMemorySampleViewName = "Grasp Memory Sample View " + ms->config.left_or_right_arm;
  ms->config.heightMemorySampleViewName = "Height Memory Sample View " + ms->config.left_or_right_arm;
  ms->config.hiRangemapViewName = "Hi Range Map View " + ms->config.left_or_right_arm;
  ms->config.hiColorRangemapViewName = "Hi Color Range Map View " + ms->config.left_or_right_arm;
  ms->config.mapBackgroundViewName = "Map Background Viewer " + ms->config.left_or_right_arm;

  cv::namedWindow(ms->config.wristViewName);
  cv::setMouseCallback(ms->config.wristViewName, pilotCallbackFunc, NULL);
  cv::namedWindow(ms->config.graspMemoryViewName);
  cv::setMouseCallback(ms->config.graspMemoryViewName, graspMemoryCallbackFunc, NULL);
  cv::namedWindow(ms->config.coreViewName);
  cv::namedWindow(ms->config.rangeogramViewName);
  cv::namedWindow(ms->config.mapBackgroundViewName);
  cv::namedWindow(ms->config.faceViewName);

  ros::Subscriber fetchCommandSubscriber;
  fetchCommandSubscriber = n.subscribe("/fetch_commands", 1, 
                                       fetchCommandCallback);

  ros::Subscriber forthCommandSubscriber;
  forthCommandSubscriber = n.subscribe("/ein/" + ms->config.left_or_right_arm + "/forth_commands", 1, 
                                       forthCommandCallback);

  ros::Timer timer1 = n.createTimer(ros::Duration(0.0001), timercallback1);

  ms->config.tfListener = new tf::TransformListener();

  ms->config.ikClient = n.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/" + ms->config.left_or_right_arm + "/PositionKinematicsNode/IKService");
  ms->config.cameraClient = n.serviceClient<baxter_core_msgs::OpenCamera>("/cameras/open");

  ms->config.joint_mover = n.advertise<baxter_core_msgs::JointCommand>("/robot/limb/" + ms->config.left_or_right_arm + "/joint_command", 10);
  ms->config.gripperPub = n.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/" + ms->config.left_or_right_arm + "_gripper/command",10);
  ms->config.moveSpeedPub = n.advertise<std_msgs::Float64>("/robot/limb/" + ms->config.left_or_right_arm + "/set_speed_ratio",10);
  ms->config.sonarPub = n.advertise<std_msgs::UInt16>("/robot/sonar/head_sonar/set_sonars_enabled",10);
  ms->config.headPub = n.advertise<baxter_core_msgs::HeadPanCommand>("/robot/head/command_head_pan",10);
  ms->config.nodPub = n.advertise<std_msgs::Bool>("/robot/head/command_head_nod",10);

  ms->config.currentHeadPanCommand.target = 0;
  ms->config.currentHeadPanCommand.speed = 50;
  ms->config.currentHeadNodCommand.data = 0;
  ms->config.currentSonarCommand.data = 0;


  ms->config.facePub = n.advertise<std_msgs::Int32>("/confusion/target/command", 10);
  ms->config.einPub = n.advertise<EinState>("state", 10);

  ms->config.vmMarkerPublisher = n.advertise<visualization_msgs::MarkerArray>("volumetric_rgb_map", 10);

  {
    baxter_core_msgs::EndEffectorCommand command;
    command.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
    command.id = 65538;
    ms->config.gripperPub.publish(command);
  }

  ms->config.frameGraySobel = Mat(1,1,CV_64F);

  initializeMap(ms);

  spinlessNodeMain(ms);
  spinlessPilotMain(ms);

  saveROSParams(ms);






  int cudaCount = gpu::getCudaEnabledDeviceCount();
  cout << "cuda count: " << cudaCount << endl;;

  cvWaitKey(1); // this might be good to init cv gui stuff
  ms->config.lastImageCallbackReceived = ros::Time::now();

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

  ms->config.lastMovementStateSet = ros::Time::now();

  ros::spin();

  return 0;
}
 
