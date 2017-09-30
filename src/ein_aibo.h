#ifndef _EIN_AIBO_H_
#define _EIN_AIBO_H_

#include <ros/ros.h>

#include<arpa/inet.h> 

#include "word.h"

class EinAiboJoints {
  public:
  double legLF1;
  double legLF2;
  double legLF3;
  double legLH1;
  double legLH2;
  double legLH3;
  double legRH1;
  double legRH2;
  double legRH3;
  double legRF1;
  double legRF2;
  double legRF3;
  double neck;
  double headPan;
  double headTilt;
  double tailPan;
  double tailTilt;
  double mouth;

  string toString();

  double dist(EinAiboJoints & other);
};

class EinAiboIndicators {
  public:
  // indicator lights
  double ledBFC;
  double ledBFW;
  double ledBMC;
  double ledBMW;
  double ledBRC;
  double ledBRW;
  double ledF1;
  double ledF2;
  double ledF3;
  double ledF4;
  double ledF5;
  double ledF6;
  double ledF7;
  double ledF8;
  double ledF9;
  double ledF10;
  double ledF11;
  double ledF12;
  double ledF13;
  double ledF14;
  double ledHC;
  double modeR;
  double modeG;
  double modeB;

  // indicator motors
  double earL;
  double earR;
};

class EinAiboSensors {
  public:
  // IR sensors
  double distanceNearSnout = 0;
  double distanceFarSnout = 0;
  double distanceChest = 0;

  // 3-axis accelerometer
  Vector3d accelerometer;

  // pressure sensors
  double pawLF;
  double pawLH;
  double pawRF;
  double pawRH;
  double chinSensor;

  // continuous pressure sensors
  double headTouch;
  double backTouchR;
  double backTouchM;
  double backTouchF;
};

class EinAiboConfig {

  public:
  ros::Publisher aibo_snout_pub;
  ros::Time lastSensoryMotorUpdateTime;

  EinAiboJoints targetJoints;
  EinAiboJoints trueJoints;

  // socket stuff for AIBO
  string ip_string;
  int dog_needs_reinit = 0;
  int aibo_socket_desc;
  int aibo_socket_did_connect = 0;
  struct sockaddr_in aibo_server;
  const static int aibo_sock_buf_size = 1024*1024*3;
  char aibo_sock_buf[aibo_sock_buf_size];
  int aibo_sock_buf_valid_bytes = 0;


  // parameter for relative movements
  double dogPoseGridSize = 2.0;
  double dogGainGridSize = 0.1;

  EinAiboJoints intendedPose;
  EinAiboJoints truePose;

  // copies for P, I, and D in that order
  EinAiboJoints intendedGain[3];
  EinAiboJoints trueGain[3];

  // indicator lights and motors
  EinAiboIndicators intendedIndicators;
  EinAiboIndicators trueIndicators;

  EinAiboSensors trueSensors;
  Mat snoutImage;
  Mat snoutCamImage;

  double * voice_buffer = NULL;
  int voice_buffer_size = -1;
};





#endif /* _EIN_AIBO_H_*/
