#include "config.h"
#include "camera.h"
#include <vector>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>


#include <dirent.h>

#include "ein.h"
using namespace boost::filesystem;
using namespace boost::system;


Camera::Camera(MachineState * m, string iname, string topic, string _tf_ee_link, string _tf_camera_link) {
  image_topic = topic;
  name = iname;
  ms = m;
  tf_ee_link = _tf_ee_link;
  tf_camera_link = _tf_camera_link;

  image_sub = m->config.it->subscribe(image_topic, 1, &Camera::imageCallback, this);
  imRingBuffer.resize(imRingBufferSize);
  imRBTimes.resize(imRingBufferSize);
  lastImageCallbackReceived = rclcpp::Clock{}.now();
  calibrationDirectory = ms->config.data_directory + ms->config.config_directory + name;

  // call after ROS is all set up.
  boost::system::error_code ec;
  create_directories(calibrationDirectory, ec);
  if (ec) {
    CONSOLE_ERROR(ms, "Unable to create camera calibration directory " << calibrationDirectory << " boost error code: " << ec);
  }  
  calibrationFilename = calibrationDirectory + "/cameraCalibration.yml";
  gripperMaskFilename = calibrationDirectory + "/gripperMask.bmp";
  loadCalibration();
  loadGripperMask();

  mu_x = 0.01;
  mu_y = 0.01;
  kappa_x = 0.000;
  kappa_y = 0.000;

  r_00 = 1;
  r_01 = 0;
  r_10 = 0;
  r_11 = 1;

  centerX = 640;
  centerY = 400;

  setDefaultHandCameraOffset();
}



void Camera::deactivateSensorStreaming() {
  cout << "deactivateSensorStreaming: Subscribe to image." << image_topic << endl;
  image_sub = it->subscribe(image_topic, 1, &Camera::imageCallback, this);
  cout << "Subscribed to image." << endl;

}

void Camera::activateSensorStreaming() {
  image_sub = it->subscribe(image_topic, 30, &Camera::imageCallback, this);
}



void Camera::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg){
  lastImageCallbackReceived = rclcpp::Clock{}.now();

  lastImageStamp = msg->header.stamp;
  cv_bridge::CvImageConstPtr cv_ptr = NULL;

  try {
    //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::msg::image_encodings::BGR8);
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
  } catch(cv_bridge::Exception& e) {
    CONSOLE_ERROR(ms, "cv_bridge exception " << __FILE__ ":" << __LINE__ << " camera " << name << ": " << e.what());
    return;
  }

  if (!ms->config.shouldIImageCallback) {
    //cout << "Early exit image callback." << endl;
    return;
  }
  //`7cout << "Topic: " << image_topic << " ";
  //cout << "Type: " << cv_ptr->image.type() << endl;
  //cout << "type: " << CV_16U << endl;

  cam_img = cv_ptr->image.clone();

  if (cam_img.type() == CV_16UC1) {
    Mat graybgr;
    cvtColor(cam_img, graybgr, cv::COLOR_GRAY2BGR);  
    graybgr.convertTo(cam_bgr_img, CV_8U, 1.0/256.0);
  } else if (cam_img.type() == CV_8UC4) {
    cvtColor(cam_img, cam_bgr_img, cv::COLOR_BGRA2BGR);
  } else {
    cam_bgr_img = cam_img.clone();
  }
  cvtColor(cam_bgr_img, cam_ycrcb_img, cv::COLOR_BGR2YCrCb);

  setRingImageAtTime(msg->header.stamp, cam_bgr_img);

  ms->imageCallback(this);
}



int Camera::getRingImageAtTime(rclcpp::Time t, Mat& value, int drawSlack, bool debug) {
  if (imRingBufferStart == imRingBufferEnd) {
    
    if (debug) {
      cout << "Denied request in getRingImageAtTime(): Buffer empty." << endl;
    }
    return 0;
  } else {
    int earliestSlot = imRingBufferStart;
    rclcpp::Duration deltaTdur = t - imRBTimes[earliestSlot];
    // if the request comes before our earliest record, deny
    if (deltaTdur.seconds() <= 0.0) {
      if (debug) {
	cout << "Denied out of order range value in getRingImageAtTime(): Too small." << endl;
	//	cout << "  getRingImageAtTime() ms->config.imRingBufferStart ms->config.imRingBufferEnd t ms->config.imRBTimes[earliestSlot]: " << 
	//	  imRingBufferStart << " " << imRingBufferEnd << " " << t << " " << imRBTimes[earliestSlot] << endl;
      }
      return -1;
    } else if (imRingBufferStart < imRingBufferEnd) {
      for (int s = imRingBufferStart; s < imRingBufferEnd; s++) {
	rclcpp::Duration deltaTdurPre = t - imRBTimes[s];
	rclcpp::Duration deltaTdurPost = t - imRBTimes[s+1];
	if ((deltaTdurPre.seconds() >= 0.0) && (deltaTdurPost.seconds() <= 0)) {
	  Mat m1 = imRingBuffer[s];
	  Mat m2 = imRingBuffer[s+1];
	  double w1 = deltaTdurPre.seconds();
	  double w2 = -deltaTdurPost.seconds();
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
	rclcpp::Duration deltaTdurPre = t - imRBTimes[s];
	rclcpp::Duration deltaTdurPost = t - imRBTimes[s+1];
	if ((deltaTdurPre.seconds() >= 0.0) && (deltaTdurPost.seconds() <= 0)) {
	  Mat m1 = imRingBuffer[s];
	  Mat m2 = imRingBuffer[s+1];
	  double w1 = deltaTdurPre.seconds();
	  double w2 = -deltaTdurPost.seconds();
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
	rclcpp::Duration deltaTdurPre = t - imRBTimes[imRingBufferSize-1];
	rclcpp::Duration deltaTdurPost = t - imRBTimes[0];
	if ((deltaTdurPre.seconds() >= 0.0) && (deltaTdurPost.seconds() <= 0)) {
	  Mat m1 = imRingBuffer[imRingBufferSize-1];
	  Mat m2 = imRingBuffer[0];
	  double w1 = deltaTdurPre.seconds();
	  double w2 = -deltaTdurPost.seconds();
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
	rclcpp::Duration deltaTdurPre = t - imRBTimes[s];
	rclcpp::Duration deltaTdurPost = t - imRBTimes[s+1];
	if ((deltaTdurPre.seconds() >= 0.0) && (deltaTdurPost.seconds() <= 0)) {
	  Mat m1 = imRingBuffer[s];
	  Mat m2 = imRingBuffer[s+1];
	  double w1 = deltaTdurPre.seconds();
	  double w2 = -deltaTdurPost.seconds();
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
  return -10;
}






void Camera::setRingImageAtTime(rclcpp::Time t, Mat& imToSet) {
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
    rclcpp::Duration deltaTdur = t - imRBTimes[imRingBufferStart];
    if (deltaTdur.seconds() <= 0.0) {
#ifdef DEBUG_RING_BUFFER 
      //cout << "Dropped out of order range value in setRingImageAtTime(). " << ms->config.imRBTimes[ms->config.imRingBufferStart].seconds() << " " << t.seconds() << " " << deltaTdur.seconds() << " " << endl;
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


streamImage * Camera::currentImage() {
  if (sibCurIdx < streamImageBuffer.size()) {
    streamImage * tsi = &(streamImageBuffer[sibCurIdx]);
    return tsi;
  } else {
    return NULL;
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



void Camera::clearStreamBuffer() {
  streamImageBuffer.resize(0);
}



void Camera::loadCalibration(string inFileName) {
  CONSOLE(ms, "Loading calibration file from " << inFileName);
  FileStorage fsvI;
  fsvI.open(inFileName, FileStorage::READ);

  if (!fsvI.isOpened()) {
    CONSOLE_ERROR(ms, "Couldn't open calibration file " << inFileName);
    return;
  }


  FileNode hand_camera_offset_node = fsvI["hand_camera_offset"];
  handCameraOffset.readFromFileNode(hand_camera_offset_node);

  {
    FileNode anode = fsvI["cropUpperLeftCorner"];
    FileNodeIterator it = anode.begin(), it_end = anode.end();
    cropUpperLeftCorner.px = *(it++);
    cropUpperLeftCorner.py = *(it++);
  }


  {
    FileNode anode = fsvI["mu"];
    FileNodeIterator it = anode.begin(), it_end = anode.end();
    mu_x = *(it++);
    mu_y = *(it++);
  }

  {
    FileNode anode = fsvI["kappa"];
    FileNodeIterator it = anode.begin(), it_end = anode.end();
    kappa_x = *(it++);
    kappa_y = *(it++);
  }


  {
    FileNode anode = fsvI["vanishingPointReticle"];
    FileNodeIterator it = anode.begin(), it_end = anode.end();
    vanishingPointReticle.px = *(it++);
    vanishingPointReticle.py = *(it++);
    probeReticle = vanishingPointReticle;

  }

  {
    FileNode anode = fsvI["heightReticles"];
    FileNodeIterator it = anode.begin(), it_end = anode.end();
    heightReticles[3].px = *(it++);
    heightReticles[2].px = *(it++);
    heightReticles[1].px = *(it++);
    heightReticles[0].px = *(it++);

    heightReticles[3].py = *(it++);
    heightReticles[2].py = *(it++);
    heightReticles[1].py = *(it++);
    heightReticles[0].py = *(it++);
  }

  {
    FileNode anode = fsvI["colorReticles"];
    FileNodeIterator it = anode.begin(), it_end = anode.end();
    xCR[0]  = *(it++);
    xCR[1]  = *(it++);
    xCR[2]  = *(it++);
    xCR[3]  = *(it++);
    xCR[4]  = *(it++);
    xCR[5]  = *(it++);
    xCR[6]  = *(it++);
    xCR[7]  = *(it++);
    xCR[8]  = *(it++);
    xCR[9]  = *(it++);
    xCR[10] = *(it++);
    xCR[11] = *(it++);
    xCR[12] = *(it++);
    xCR[13] = *(it++);

    yCR[0]  = *(it++);
    yCR[1]  = *(it++);
    yCR[2]  = *(it++);
    yCR[3]  = *(it++);
    yCR[4]  = *(it++);
    yCR[5]  = *(it++);
    yCR[6]  = *(it++);
    yCR[7]  = *(it++);
    yCR[8]  = *(it++);
    yCR[9]  = *(it++);
    yCR[10] = *(it++);
    yCR[11] = *(it++);
    yCR[12] = *(it++);
    yCR[13] = *(it++);
  }

  {
    FileNode anode = fsvI["lensCorrections"];
    FileNodeIterator it = anode.begin(), it_end = anode.end();
    m_x_h[0] = *(it++);
    m_x_h[1] = *(it++);
    m_x_h[2] = *(it++);
    m_x_h[3] = *(it++);

    m_y_h[0] = *(it++);
    m_y_h[1] = *(it++);
    m_y_h[2] = *(it++);
    m_y_h[3] = *(it++);
  }

  {
    FileNode anode = fsvI["gear0offset"];
    FileNodeIterator it = anode.begin(), it_end = anode.end();
    gear0offset.x() = *(it++);
    gear0offset.y() = *(it++);
    gear0offset.z() = *(it++);
    gear0offset.w() = *(it++);
  }
  {
    cameraExposure = (int) fsvI["cameraExposure"];
    cameraGain = (int) fsvI["cameraGain"];
    cameraWhiteBalanceRed = (int) fsvI["cameraWhiteBalanceRed"];
    cameraWhiteBalanceGreen = (int) fsvI["cameraWhiteBalanceGreen"];
    cameraWhiteBalanceBlue = (int) fsvI["cameraWhiteBalanceBlue"];

  }
  if (name.find("hand_camera") != std::string::npos) {
    ms->pushWord("moveCropToProperValue"); 
  }
}

void Camera::saveCalibration() {
  saveCalibration(calibrationFilename);
}

void Camera::loadCalibration() {
  loadCalibration(calibrationFilename);
}
void Camera::saveCalibration(string outFileName) {
  CONSOLE(ms, "Saving calibration file from " << outFileName);
  rclcpp::Time savedTime = rclcpp::Clock{}.now();

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
  fsvO.open(outFileName, FileStorage::WRITE);

  if (! fsvO.isOpened()) {
    CONSOLE_ERROR(ms, "Couldn't open calibration file " << outFileName);
    return;
  }

  fsvO << "savedTime" << "[" 
    << savedTime.seconds() 
  << "]";

  fsvO << "hand_camera_offset";
  handCameraOffset.writeToFileStorage(fsvO);


  fsvO << "cropUpperLeftCorner" << "[" 
    << cropUpperLeftCorner.px 
    << cropUpperLeftCorner.py 
  << "]";

  fsvO << "mu" << "[" 
    << mu_x 
    << mu_y 
  << "]";

  fsvO << "kappa" << "[" 
    << kappa_x 
    << kappa_y 
  << "]";

  fsvO << "vanishingPointReticle" << "[" 
    << vanishingPointReticle.px 
    << vanishingPointReticle.py 
  << "]";

  fsvO << "heightReticles" << "[" 
    << heightReticles[3].px
    << heightReticles[2].px
    << heightReticles[1].px
    << heightReticles[0].px

    << heightReticles[3].py
    << heightReticles[2].py
    << heightReticles[1].py
    << heightReticles[0].py
  << "]";

  fsvO << "colorReticles" << "[" 
    << xCR[0]
    << xCR[1]
    << xCR[2]
    << xCR[3]
    << xCR[4]
    << xCR[5]
    << xCR[6]
    << xCR[7]
    << xCR[8]
    << xCR[9]
    << xCR[10]
    << xCR[11]
    << xCR[12]
    << xCR[13]

    << yCR[0] 
    << yCR[1] 
    << yCR[2] 
    << yCR[3] 
    << yCR[4] 
    << yCR[5] 
    << yCR[6] 
    << yCR[7] 
    << yCR[8] 
    << yCR[9] 
    << yCR[10]
    << yCR[11]
    << yCR[12]
    << yCR[13]
  << "]";

  fsvO << "lensCorrections" << "[" 
    << m_x_h[0]
    << m_x_h[1]
    << m_x_h[2]
    << m_x_h[3]

    << m_y_h[0]
    << m_y_h[1]
    << m_y_h[2]
    << m_y_h[3]
  << "]";

  fsvO << "gear0offset" << "["
    << gear0offset.x()
    << gear0offset.y()
    << gear0offset.z()
    << gear0offset.w()
  << "]";
  fsvO << "cameraExposure" << cameraExposure;
  fsvO << "cameraGain" << cameraGain;
  fsvO << "cameraWhiteBalanceRed" << cameraWhiteBalanceRed;
  fsvO << "cameraWhiteBalanceGreen" << cameraWhiteBalanceGreen;
  fsvO << "cameraWhiteBalanceBlue" << cameraWhiteBalanceBlue;

  fsvO.release();
  cout << "done." << endl;
}
void Camera::saveGripperMask() {
  saveGripperMask(gripperMaskFilename);
}
void Camera::saveGripperMask(string filename) {
  CONSOLE(ms, "Saving gripper mask to " << filename);
  bool result = imwrite(filename, 255*gripperMask);
  if (! result) {
    CONSOLE_ERROR(ms, "Could not save gripper mask.");
  }
}

void Camera::loadGripperMask() {
  loadGripperMask(gripperMaskFilename);
}
void Camera::loadGripperMask(string filename) {
  CONSOLE(ms, "Loading gripper mask from " << filename << "...");
  Mat tmpMask = imread(filename, cv::ImreadModes::IMREAD_GRAYSCALE);
  if (tmpMask.data == NULL) {
    CONSOLE_ERROR(ms, "Could not load gripper mask; will use empty one.");
  }

  gripperMask.create(tmpMask.size(), CV_8U);
  Size sz = gripperMask.size();
  int imW = sz.width;
  int imH = sz.height;

  for (int x = 0; x < imW; x++) {
    for (int y = 0; y < imH; y++) {
      if (tmpMask.at<uchar>(y,x) > 0) {
	gripperMask.at<uchar>(y,x) = 1;
      } else {
	gripperMask.at<uchar>(y,x) = 0;
      }
    }
  }
}


void Camera::updateTrueCameraPoseFromTf(rclcpp::Time time) {
  // string tflink = ms->config.left_or_right_arm + "_hand_camera"
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = 0;
  pose.pose.orientation.w = 1;

  //pose.header.stamp = rclcpp::Time(0);
  pose.header.stamp = time;
  pose.header.frame_id =  tf_camera_link;
  
  geometry_msgs::msg::PoseStamped transformed_pose;
  if (ms->config.currentRobotMode != SIMULATED) {    
    try {
      //ms->config.tfListener->waitForTransform("base", tf_camera_link, pose.header.stamp, rclcpp::Duration(1.0, 0));
      //ms->config.tfListener->transformPose("base", pose.header.stamp, pose, tf_camera_link, transformed_pose);
      } catch (tf2::TransformException ex){
        cout << "Tf error (a few at startup are normal; worry if you see a lot!): " << __FILE__ << ":" << __LINE__ << endl;
        cout << "link: " << tf_camera_link << endl;
        //cout << "p1: " << pose << " p2: " << transformed_pose << endl;
        cout << ex.what();
        //throw;
      }
    }

    truePose.px = transformed_pose.pose.position.x;
    truePose.py = transformed_pose.pose.position.y;
    truePose.pz = transformed_pose.pose.position.z;
    truePose.qx = transformed_pose.pose.orientation.x;
    truePose.qy = transformed_pose.pose.orientation.y;
    truePose.qz = transformed_pose.pose.orientation.z;
    truePose.qw = transformed_pose.pose.orientation.w;
  }



void Camera::setDefaultHandCameraOffset() {
  eePose p = {0.03815,0.01144,0.01589, 0,0,0,1};
  handCameraOffset = p;
}




void Camera::initializeConfig(int rows, int cols)
{
  if (!isSketchyMat(cam_img)) {
    if (cam_img.cols != cols || cam_img.rows != rows) {
      CONSOLE_ERROR(ms, "Given rows and cols does not match image!");
    }
  }

  cropUpperLeftCorner = eePose(cols, rows, 0.0,
                               0.0, 1.0, 0.0, 0.0);
  centerReticle = eePose(cols/2, rows/2, 0.0,
                         0.0, 0.0, 0.0, 0.0);
  
  defaultReticle = centerReticle;
  probeReticle = centerReticle;
  vanishingPointReticle = centerReticle;
}
