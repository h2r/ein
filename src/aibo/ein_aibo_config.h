#ifndef _EIN_AIBO_CONFIG_H_
#define _EIN_AIBO_CONFIG_H_

#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sys/stat.h>
#include <dirent.h>
#include <signal.h>
#include <arpa/inet.h>
#include <sys/poll.h>
#include <sys/socket.h>


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

class EinAiboDog {

  public:
  EinAiboDog();
  ros::Publisher aibo_snout_pub;
  ros::Publisher joint_state_pub;
  sensor_msgs::JointState joint_state;

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

  void jointsDeg2Rad();
  void publishJoints();
  void publishMapToBaseLink();
  void robotUpdate();
};


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
    //ss << pose;
    return ss.str();
    // XXX need to overload << 
  }

  bool equals(shared_ptr<Word> word) {
    shared_ptr<AiboPoseWord> w1 = dynamic_pointer_cast<AiboPoseWord>(word);
    if (w1 == NULL) {
      return false;
    } else {
      //return w1->value().equals(this->value());
      // XXX not done
      return false;
    }
  }
  
  virtual bool to_bool() {
    return true;
  }
  virtual int to_int() {
    return 1;
  }
};

class EinAiboConfig {
 public:

  MachineState * ms;

  EinAiboConfig(MachineState * ms);

  int focusedMember = 0;
  std::vector<EinAiboDog*> pack;

  ros::Time aiboStoppedTime;
  EinAiboJoints * stoppedJoints;
  ros::Time aiboComeToStopTime;
};

#endif /* _EIN_AIBO_CONFIG_H_ */
