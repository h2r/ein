#include "ein.h"
#include "camera.h"
#include <vector>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

Camera::Camera(MachineState * m, string topic) {
  image_topic = topic;
  ms = m;
  ros::NodeHandle n("~");
  it = make_shared<image_transport::ImageTransport>(n);
  image_sub = it->subscribe(image_topic, 1, &Camera::imageCallback, this);
  imRingBuffer.resize(imRingBufferSize);
  imRBTimes.resize(imRingBufferSize);
  lastImageCallbackReceived = ros::Time::now();


}
void Camera::deactivateSensorStreaming() {
  cout << "deactivateSensorStreaming: Subscribe to image." << image_topic << endl;
  image_sub = it->subscribe(image_topic, 1, &Camera::imageCallback, this);
  cout << "Subscribed to image." << endl;

}

void Camera::activateSensorStreaming() {
  image_sub = it->subscribe(image_topic, 30, &Camera::imageCallback, this);
}



void Camera::imageCallback(const sensor_msgs::ImageConstPtr& msg){
  lastImageCallbackReceived = ros::Time::now();

  lastImageStamp = msg->header.stamp;


  if((ms->config.sensorStreamOn) && (ms->config.sisImage)) {
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      int cfClass = ms->config.focusedClass;
      if ((cfClass > -1) && (cfClass < ms->config.classLabels.size())) {
	double thisNow = msg->header.stamp.toSec();
	streamImageAsClass(ms, cv_ptr->image, cfClass, thisNow); 
      }
    } catch(cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  if (!ms->config.shouldIImageCallback) {
    //cout << "Early exit image callback." << endl;
    return;
  }



  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cam_img = cv_ptr->image.clone();
  }catch(cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }



  setRingImageAtTime(msg->header.stamp, cam_img);

  ms->imageCallback(this);
}



int Camera::getRingImageAtTime(ros::Time t, Mat& value, int drawSlack, bool debug) {
  if (imRingBufferStart == imRingBufferEnd) {
    
    if (debug) {
      cout << "Denied request in getRingImageAtTime(): Buffer empty." << endl;
    }
    return 0;
  } else {
    int earliestSlot = imRingBufferStart;
    ros::Duration deltaTdur = t - imRBTimes[earliestSlot];
    // if the request comes before our earliest record, deny
    if (deltaTdur.toSec() <= 0.0) {
      if (debug) {
	cout << "Denied out of order range value in getRingImageAtTime(): Too small." << endl;
	cout << "  getRingImageAtTime() ms->config.imRingBufferStart ms->config.imRingBufferEnd t ms->config.imRBTimes[earliestSlot]: " << 
	  imRingBufferStart << " " << imRingBufferEnd << " " << t << " " << imRBTimes[earliestSlot] << endl;
      }
      return -1;
    } else if (imRingBufferStart < imRingBufferEnd) {
      for (int s = imRingBufferStart; s < imRingBufferEnd; s++) {
	ros::Duration deltaTdurPre = t - imRBTimes[s];
	ros::Duration deltaTdurPost = t - imRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Mat m1 = imRingBuffer[s];
	  Mat m2 = imRingBuffer[s+1];
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
	    imRingBufferStart = newStart;
	  }
	  return 1;
	}
      }
      // if we didn't find it we should return failure
      if (debug) {
	cout << "Denied out of order range value in getRingImageAtTime(): Too large." << endl;
      }
      return -2;
    } else {
      for (int s = imRingBufferStart; s < imRingBufferSize-1; s++) {
	ros::Duration deltaTdurPre = t - imRBTimes[s];
	ros::Duration deltaTdurPost = t - imRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Mat m1 = imRingBuffer[s];
	  Mat m2 = imRingBuffer[s+1];
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
	    imRingBufferStart = newStart;
	  }
	  return 1;
	}
      } {
	ros::Duration deltaTdurPre = t - imRBTimes[imRingBufferSize-1];
	ros::Duration deltaTdurPost = t - imRBTimes[0];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Mat m1 = imRingBuffer[imRingBufferSize-1];
	  Mat m2 = imRingBuffer[0];
	  double w1 = deltaTdurPre.toSec();
	  double w2 = -deltaTdurPost.toSec();
	  double totalWeight = w1 + w2;
	  w1 = w1 / totalWeight;
	  w2 = w2 / totalWeight;
	  if (w1 >= w2)
	    value = m1;
	  else
	    value = m2;

	  int newStart = imRingBufferSize-1;
	  if(drawSlack) {
	    imRingBufferStart = newStart;
	  }
	  return 1;
	}
      } for (int s = 0; s < imRingBufferEnd; s++) {
	ros::Duration deltaTdurPre = t - imRBTimes[s];
	ros::Duration deltaTdurPost = t - imRBTimes[s+1];
	if ((deltaTdurPre.toSec() >= 0.0) && (deltaTdurPost.toSec() <= 0)) {
	  Mat m1 = imRingBuffer[s];
	  Mat m2 = imRingBuffer[s+1];
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
	    imRingBufferStart = newStart;
	  }
	  return 1;
	}
      }
      // if we didn't find it we should return failure
      if (debug) {
	cout << "Denied out of order range value in getRingImageAtTime(): Too large." << endl;
      }

      return -2;
    }
  }
}






void Camera::setRingImageAtTime(ros::Time t, Mat& imToSet) {
#ifdef DEBUG_RING_BUFFER
  //cout << "setRingImageAtTime() start end size: " << ms->config.imRingBufferStart << " " << ms->config.imRingBufferEnd << " " << ms->config.imRingBufferSize << endl;
#endif

  // if the ring buffer is empty, always re-initialize
  if (imRingBufferStart == imRingBufferEnd) {
    imRingBufferStart = 0;
    imRingBufferEnd = 1;
    imRingBuffer[0] = imToSet;
    imRBTimes[0] = t;
  } else {
    ros::Duration deltaTdur = t - imRBTimes[imRingBufferStart];
    if (deltaTdur.toSec() <= 0.0) {
#ifdef DEBUG_RING_BUFFER 
      //cout << "Dropped out of order range value in setRingImageAtTime(). " << ms->config.imRBTimes[ms->config.imRingBufferStart].toSec() << " " << t.toSec() << " " << deltaTdur.toSec() << " " << endl;
#endif
    } else {
      int slot = imRingBufferEnd;
      imRingBuffer[slot] = imToSet;
      imRBTimes[slot] = t;

      if (imRingBufferEnd >= (imRingBufferSize-1)) {
	imRingBufferEnd = 0;
      } else {
	imRingBufferEnd++;
      }

      if (imRingBufferEnd == imRingBufferStart) {
	if (imRingBufferStart >= (imRingBufferSize-1)) {
	  imRingBufferStart = 0;
	} else {
	  imRingBufferStart++;
	}
      }
    }
  }
}


void Camera::imRingBufferAdvance() {
  if (imRingBufferEnd != imRingBufferStart) {
    if (imRingBufferStart >= (imRingBufferSize-1)) {
      imRingBufferStart = 0;
    } else {
      imRingBufferStart++;
    }
  }
}
