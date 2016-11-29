#include "ein.h"
#include "camera.h"
#include <vector>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>

using namespace boost::filesystem;
using namespace boost::system;


Camera::Camera(MachineState * m, string iname, string topic, string _tf_ee_link, string _tf_camera_link) {
  image_topic = topic;
  name = iname;
  ms = m;
  tf_ee_link = _tf_ee_link;
  tf_camera_link = _tf_camera_link;
  ros::NodeHandle n("~");
  it = make_shared<image_transport::ImageTransport>(n);
  image_sub = it->subscribe(image_topic, 1, &Camera::imageCallback, this);
  imRingBuffer.resize(imRingBufferSize);
  imRBTimes.resize(imRingBufferSize);
  lastImageCallbackReceived = ros::Time::now();
  calibrationDirectory = ms->config.data_directory + ms->config.config_directory + name;

  // call after ROS is all set up.
  error_code ec;
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
  cv_bridge::CvImageConstPtr cv_ptr = NULL;

  try {
    //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv_ptr = cv_bridge::toCvShare(msg);
  } catch(cv_bridge::Exception& e) {
    CONSOLE_ERROR(ms, "cv_bridge exception " << __FILE__ ":" << __LINE__ << " camera " << name << ": " << e.what());
    return;
  }

  if((ms->config.sensorStreamOn) && (ms->config.sisImage)) {
    int cfClass = ms->config.focusedClass;
    if ((cfClass > -1) && (cfClass < ms->config.classLabels.size())) {
      double thisNow = msg->header.stamp.toSec();
      streamImageAsClass(cv_ptr->image, cfClass, thisNow); 
    }
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
    cvtColor(cam_img, graybgr, CV_GRAY2BGR);  
    graybgr.convertTo(cam_bgr_img, CV_8U, 1.0/256.0);
  } else if (cam_img.type() == CV_8UC4) {
    cvtColor(cam_img, cam_bgr_img, CV_BGRA2BGR);
  } else {
    cam_bgr_img = cam_img.clone();
  }
  cvtColor(cam_bgr_img, cam_ycrcb_img, CV_BGR2YCrCb);

  setRingImageAtTime(msg->header.stamp, cam_bgr_img);

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



void Camera::populateStreamImageBuffer() {
  DIR *dpdf;
  struct dirent *epdf;
  string dot(".");
  string dotdot("..");
  string dotpng(".png");

  int classToStreamIdx = ms->config.focusedClass;
  if (ms->config.focusedClass == -1) {
    return;
  }
  string root_path = createStreamImagePath(classToStreamIdx);
  cout << "Populating stream image buffer from " << root_path << endl;

  vector<string> files = glob(root_path + "*.png");

  for (int i = 0; i < files.size(); i++) {
    string fname(files[i]);
    int loaded = 1;
    string fnoextension = fname.substr(0, fname.size() - 4);

    string imfilename(fname);
    string ymlfilename = fnoextension + ".yml";

    FileStorage fsvI;
    cout << "Streaming image from " << ymlfilename << " ...";
    fsvI.open(ymlfilename, FileStorage::READ);

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

  sort(streamImageBuffer.begin(), streamImageBuffer.end(), streamImageComparator);

}

void Camera::clearStreamBuffer() {
  streamImageBuffer.resize(0);
}


string Camera::createStreamImagePath(int classToStreamIdx) {

  string this_image_path = streamDirectory(ms, classToStreamIdx) + "/images/";

  stringstream buf;
  buf << this_image_path << "/";
  buf << ms->config.robot_serial << "_" << ms->config.left_or_right_arm << "_" << name;
  return buf.str();
}

void Camera::writeImage(Mat im, int classToStreamIdx, double now) {
  string root_path = createStreamImagePath(classToStreamIdx);
  string formattedTime = formatTime(ros::Time(now));
  root_path += "_" + formattedTime;


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
}

void Camera::writeImageBatchAsClass(int classToStreamIdx) {

  FileStorage fsvO;
  string root_path = createStreamImagePath(classToStreamIdx);

  ros::Time time;
  if (streamImageBuffer.size() != 0) {
    time = ros::Time(streamImageBuffer[0].time);
  } else {
    time = ros::Time::now();
  }

  string formattedTime = formatTime(time);
  root_path += "_" + formattedTime + "_batch";
  string yaml_path = root_path + ".yml";

  fsvO.open(yaml_path, FileStorage::WRITE);
  fsvO << "camera_name"  << name;
  fsvO << "camera_topic"  << image_topic;
  fsvO << "camera_ee_link"  << tf_ee_link;
  fsvO << "camera_camera_link"  << tf_camera_link;
  fsvO.release();

  saveCalibration(root_path + "_calibration.yml");


  int tng = streamImageBuffer.size();
  for (int i = 0; i < tng; i++) {
    streamImage &tsi = streamImageBuffer[i];
    if (tsi.image.data == NULL) {
      tsi.image = imread(tsi.filename);
      if (tsi.image.data == NULL) {
        cout << " Failed to load " << tsi.filename << endl;
        tsi.loaded = 0;
        return;
      } else {
        tsi.loaded = 1;
      }
    }
    writeImage(tsi.image, classToStreamIdx, tsi.time);
  }
}

void Camera::streamImageAsClass(Mat im, int classToStreamIdx, double now) {

  if (didSensorStreamTimeout(ms)) {
    return;
  } else {
  }

  if (ms->config.diskStreamingEnabled) {
    writeImage(im, classToStreamIdx, now);
  } else {
    streamImage toAdd;
    toAdd.image = im.clone();
    toAdd.time = now;
    toAdd.loaded = 1;
    toAdd.filename = "CAMERA";
    streamImageBuffer.push_back(toAdd);

    //cout << "streamImageAsClass: WARNING disk streaming not enabled, there are " << ms->config.streamImageBuffer.size() << " images in the buffer and growing..." << endl;
  }
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
  ros::Time savedTime = ros::Time::now();

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
    << savedTime.toSec() 
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
  Mat tmpMask = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
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


void Camera::updateTrueCameraPoseFromTf(ros::Time time) {
  // string tflink = ms->config.left_or_right_arm + "_hand_camera"
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = 0;
  pose.pose.orientation.w = 1;

  //pose.header.stamp = ros::Time(0);
  pose.header.stamp = time;
  pose.header.frame_id =  tf_camera_link;
  
  geometry_msgs::PoseStamped transformed_pose;
  if (ms->config.currentRobotMode != SIMULATED) {    
    try {
      ms->config.tfListener->waitForTransform("base", tf_camera_link, pose.header.stamp, ros::Duration(1.0));
        ms->config.tfListener->transformPose("base", pose.header.stamp, pose, tf_camera_link, transformed_pose);
      } catch (tf::TransformException ex){
        cout << "Tf error (a few at startup are normal; worry if you see a lot!): " << __FILE__ << ":" << __LINE__ << endl;
        cout << "link: " << tf_camera_link << endl;
        cout << "p1: " << pose << " p2: " << transformed_pose << endl;
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


void Camera::updateTrueCameraPoseWithHandCameraOffset(ros::Time time) {
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = handCameraOffset.px;
  pose.pose.position.y = handCameraOffset.py;
  pose.pose.position.z = handCameraOffset.pz;
  pose.pose.orientation.x = handCameraOffset.qx;
  pose.pose.orientation.y = handCameraOffset.qy;
  pose.pose.orientation.z = handCameraOffset.qz;
  pose.pose.orientation.w = handCameraOffset.qw;
  
  //pose.header.stamp = ros::Time(0);
  pose.header.stamp = time;
  pose.header.frame_id =  tf_ee_link;
  
  geometry_msgs::PoseStamped transformed_pose;
  if (ms->config.currentRobotMode != SIMULATED) {    
    try {
      ms->config.tfListener->waitForTransform("base", tf_ee_link, pose.header.stamp, ros::Duration(1.0));
      ms->config.tfListener->transformPose("base", pose.header.stamp, pose, tf_ee_link, transformed_pose);
    } catch (tf::TransformException ex){
      cout << "Tf error (a few at startup are normal; worry if you see a lot!): " << __FILE__ << ":" << __LINE__ << endl;
      cout << "link: " << tf_ee_link << endl;
      cout << "p1: " << pose << " p2: " << transformed_pose << endl;
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


void Camera::setHandCameraOffsetFromTf(ros::Time time)
{
  
}
