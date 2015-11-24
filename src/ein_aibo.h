#ifndef _EIN_AIBO_H_
#define _EIN_AIBO_H_

#include <ros/ros.h>

#include<sys/socket.h>
#include<arpa/inet.h> 
#include <sys/poll.h>

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

/*
class AiboPoseWord: public Word
{
private:
  EinAiboJoints pose;

public:
  EinAiboJoints value() {
    return pose;
  }

  virtual bool is_value() {
    return true;
  }

  static std::shared_ptr<AiboPoseWord> parse(string token) {
    EinAiboJoints pose;
    return std::make_shared<AiboPoseWord>(pose);
  }
  virtual bool is_static() {
    return false;
  }
  static bool isInteger(string token) {
    if (token.substr(0,5) == "EinAiboJoints") {
      return true;
    } else {
      return false;
    }
  }
  
  AiboPoseWord(EinAiboJoints _pose) {
    pose = _pose;
  }

  virtual string repr();

  string name() {
    stringstream ss;
    ss << pose;
    return ss.str();
    // XXX need to overload << and make repr, creator
  }

  bool equals(shared_ptr<Word> word) {
    shared_ptr<AiboPoseWord> w1 = dynamic_pointer_cast<AiboPoseWord>(word);
    if (w1 == NULL) {
      return false;
    } else {
      return w1->value().equals(this->value());
    }
  }
  
  virtual bool to_bool() {
    return true;
  }
  virtual int to_int() {
    return 1;
  }
};
*/

class EinAiboConfig {

  public:
  ros::Publisher aibo_snout_pub;


  EinAiboJoints targetJoints;
  EinAiboJoints trueJoints;

  // socket stuff for AIBO
  int aibo_socket_desc;
  struct sockaddr_in aibo_server;
  const static int aibo_sock_buf_size = 1024*1024*3;
  char aibo_sock_buf[aibo_sock_buf_size];
  int aibo_sock_buf_valid_bytes = 0;


  // parameter for relative movements
  double dogNormGridSize = 0.02;

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
};





#endif /* _EIN_AIBO_H_*/
