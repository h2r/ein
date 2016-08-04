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
	streamImageAsClass(cv_ptr->image, cfClass, thisNow); 
      }
    } catch(cv_bridge::Exception& e) {
      ROS_ERROR_STREAM("cv_bridge exception " << __FILE__ ":" << __LINE__ << ": " << e.what());
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
    ROS_ERROR_STREAM("cv_bridge exception " << __FILE__ ":" << __LINE__ << ": " << e.what());
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




streamImage * Camera::setIsbIdxNoLoadNoKick(int idx) {
  if ( (idx > -1) && (idx < streamImageBuffer.size()) ) {
    streamImage &tsi = streamImageBuffer[idx];
    int lastIdx = sibCurIdx;
    if ( (lastIdx > -1) && (lastIdx < streamImageBuffer.size()) && (lastIdx != idx) ) {
      //cout << "setIsbIdx: last was valid and different." << endl;
    } else {
      //cout << "setIsbIdx: last was invalid or the same." << endl;
    }

    if (tsi.loaded) {
    } else {
      tsi.loaded = 0;
    } 

    sibCurIdx = idx;
  } else {
    cout << "Tried to set ISB index out of bounds: " << idx << endl;
    return NULL;
  }

  return &(streamImageBuffer[idx]);
}




streamImage * Camera::getIsbIdxNoLoadNoKick(int idx) {
  if ( (idx > -1) && (idx < streamImageBuffer.size()) ) {
    return &(streamImageBuffer[idx]);
  } else {
    cout << "Tried to set ISB index out of bounds: " << idx << endl;
    return NULL;
  }
}



streamImage * Camera::setIsbIdxNoLoad(int idx) {
  if ( (idx > -1) && (idx < streamImageBuffer.size()) ) {
    streamImage &tsi = streamImageBuffer[idx];
    int lastIdx = sibCurIdx;
    if ( (lastIdx > -1) && (lastIdx < streamImageBuffer.size()) && (lastIdx != idx) ) {
      streamImage &lsi = streamImageBuffer[lastIdx];
      lsi.image.create(1, 1, CV_8UC3);
      lsi.loaded = 0;
      //cout << "setIsbIdx: last was valid and different." << endl;
    } else {
      //cout << "setIsbIdx: last was invalid or the same." << endl;
    }

    if (tsi.loaded) {
    } else {
      tsi.loaded = 0;
    } 

    sibCurIdx = idx;
  } else {
    cout << "Tried to set ISB index out of bounds: " << idx << endl;
    return NULL;
  }

  return &(streamImageBuffer[idx]);
}

streamImage * Camera::setIsbIdxYesLoadNoKick(int idx) {
  if ( (idx > -1) && (idx < streamImageBuffer.size()) ) {
    streamImage &tsi = streamImageBuffer[idx];
    int lastIdx = sibCurIdx;
    if ( (lastIdx > -1) && (lastIdx < streamImageBuffer.size()) && (lastIdx != idx) ) {
      //streamImage &lsi = streamImageBuffer[lastIdx];
      //lsi.image.create(1, 1, CV_8UC3);
      //lsi.loaded = 0;
      //cout << "setIsbIdx: last was valid and different." << endl;
    } else {
      //cout << "setIsbIdx: last was invalid or the same." << endl;
    }

    if (tsi.loaded) {
      //cout << "setIsbIdx: this was loaded." << endl;
    } else {
      //cout << "setIsbIdx: this was not loaded." << endl;
      tsi.image = imread(tsi.filename);
      if (tsi.image.data == NULL) {
	tsi.loaded = 0;
	cout << "Tried to set ISB index but image failed to load: " << tsi.filename << endl;
	return NULL;
      } else {
	tsi.loaded = 1;
	sibCurIdx = idx;
      }
    } 

    sibCurIdx = idx;
  } else {
    cout << "Tried to set ISB index out of bounds: " << idx << endl;
    return NULL;
  }

  return &(streamImageBuffer[sibCurIdx]);
}

streamImage * Camera::setIsbIdx(int idx) {
  if ( (idx > -1) && (idx < streamImageBuffer.size()) ) {
    streamImage &tsi = streamImageBuffer[idx];
    int lastIdx = sibCurIdx;
    if ( (lastIdx > -1) && (lastIdx < streamImageBuffer.size()) && (lastIdx != idx) ) {
      streamImage &lsi = streamImageBuffer[lastIdx];
      lsi.image.create(1, 1, CV_8UC3);
      lsi.loaded = 0;
      //cout << "setIsbIdx: last was valid and different." << endl;
    } else {
      //cout << "setIsbIdx: last was invalid or the same." << endl;
    }

    if (tsi.loaded) {
      //cout << "setIsbIdx: this was loaded." << endl;
    } else {
      //cout << "setIsbIdx: this was not loaded." << endl;
      tsi.image = imread(tsi.filename);
      if (tsi.image.data == NULL) {
	tsi.loaded = 0;
	cout << "Tried to set ISB index but image failed to load: " << tsi.filename << endl;
	return NULL;
      } else {
	tsi.loaded = 1;
	sibCurIdx = idx;
      }
    } 

    sibCurIdx = idx;
  } else {
    cout << "Tried to set ISB index out of bounds: " << idx << endl;
    return NULL;
  }

  return &(streamImageBuffer[idx]);
}

void Camera::resetAccumulatedStreamImage() {
  Size sz = ms->config.accumulatedStreamImage.size();
  int imW = sz.width;
  int imH = sz.height;

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      ms->config.accumulatedStreamImage.at<Vec3d>(y,x)[0] = 0.0;
      ms->config.accumulatedStreamImage.at<Vec3d>(y,x)[1] = 0.0;
      ms->config.accumulatedStreamImage.at<Vec3d>(y,x)[2] = 0.0;
      ms->config.accumulatedStreamImageMass.at<double>(y,x) = 0.0;
    }
  }
}



void Camera::populateStreamImageBuffer() {
  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");
  string dotpng(".png");

  int classToStreamIdx = ms->config.focusedClass;
  string thisLabelName = ms->config.classLabels[classToStreamIdx];
  string this_image_path = ms->config.data_directory + "/objects/" + thisLabelName + "/raw/images/";
  dpdf = opendir(this_image_path.c_str());
  cout << "Populating stream image buffer from " << this_image_path << endl;
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

      if (!dotpng.compare(fextension) && dot.compare(epdf->d_name) && dotdot.compare(epdf->d_name)) {

	int loaded = 1;

        char filename[1024];
        sprintf(filename, "%s%s", this_image_path.c_str(), epdf->d_name);
	string imfilename(filename);

	/* this is slow
        Mat image;
        image = imread(imfilename);

	if (image.data != NULL) {
	} else {
	  loaded = 0;
	}
	*/

        sprintf(filename, "%s%s.yml", this_image_path.c_str(), fnoextension.c_str());
	string inFileName(filename);
	FileStorage fsvI;
	cout << "Streaming image from " << inFileName << " ...";
	fsvI.open(inFileName, FileStorage::READ);

	double time = 0.0;
	{
	  FileNode anode = fsvI["time"];
	  FileNodeIterator it = anode.begin(), it_end = anode.end();
	  if (it != it_end) {
	    time = *(it++);
	  } else {
	    loaded = 0;
	  }
	}

	if (loaded) {
	  streamImage toAdd;
	  toAdd.time = time;
	  toAdd.loaded = 0;
	  toAdd.filename = imfilename;
	  streamImageBuffer.push_back(toAdd);
	  cout << "done." << endl;
	} else {
	  cout << "failed :P" << endl;
	}
      }
    }
  }

  sort(streamImageBuffer.begin(), streamImageBuffer.end(), streamImageComparator);

}

void Camera::clearStreamBuffer() {
  streamImageBuffer.resize(0);
}



void Camera::streamImageAsClass(Mat im, int classToStreamIdx, double now) {

  if (didSensorStreamTimeout(ms)) {
    return;
  } else {
  }

  if (ms->config.diskStreamingEnabled) {
    string thisLabelName = ms->config.classLabels[classToStreamIdx];
    string this_image_path = ms->config.data_directory + "/objects/" + thisLabelName + "/raw/images/";
    char buf[1024];
    sprintf(buf, "%s%f", this_image_path.c_str(), now);
    string root_path(buf); 
	root_path = appendSideAndSerial(ms, root_path);

    string png_path = root_path + ".png";
    string yaml_path = root_path + ".yml";
    //cout << "streamImageAsClass: Streaming current frame to " << png_path << " " << yaml_path << endl;
    // no compression!
    std::vector<int> args;
    args.push_back(CV_IMWRITE_PNG_COMPRESSION);
    args.push_back(ms->config.globalPngCompression);
    imwrite(png_path, im, args);

    // may want to save additional camera parameters
    FileStorage fsvO;
    fsvO.open(yaml_path, FileStorage::WRITE);

    writeSideAndSerialToFileStorage(ms, fsvO);

    fsvO << "time" <<  now;
    fsvO.release();
  } else {
    streamImage toAdd;
    toAdd.image = im;
    toAdd.time = now;
    toAdd.loaded = 1;
    toAdd.filename = "CAMERA";
    streamImageBuffer.push_back(toAdd);

    //cout << "streamImageAsClass: WARNING disk streaming not enabled, there are " << ms->config.streamImageBuffer.size() << " images in the buffer and growing..." << endl;
  }
}
