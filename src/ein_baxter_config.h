#ifndef _EIN_BAXTER_CONFIG_H_
#define _EIN_BAXTER_CONFIG_H_


#include <Eigen/Geometry> 
using namespace Eigen;

#include <ros/package.h>
#include <ros/ros.h>


#include <baxter_core_msgs/CameraControl.h>
#include <baxter_core_msgs/OpenCamera.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/CollisionDetectionState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/HeadPanCommand.h>
#include <baxter_core_msgs/SEAJointState.h>
#include <baxter_core_msgs/DigitalIOState.h>
#include <baxter_core_msgs/AnalogIOState.h>
#include <baxter_core_msgs/DigitalOutputCommand.h>
#include <baxter_core_msgs/AnalogOutputCommand.h>



#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>


class EinBaxterConfig {

 public:
  MachineState * ms;

  EinBaxterConfig(MachineState * ms);
  void endpointCallback(const baxter_core_msgs::EndpointState& eps);

  void collisionDetectionStateCallback(const baxter_core_msgs::CollisionDetectionState& cds);
  void gripStateCallback(const baxter_core_msgs::EndEffectorState& ees);
  void jointCallback(const sensor_msgs::JointState& js);
  void gravityCompCallback(const baxter_core_msgs::SEAJointState& seaJ) ;
  void cuffGraspCallback(const baxter_core_msgs::DigitalIOState& cuffDIOS) ;
  void cuffOkCallback(const baxter_core_msgs::DigitalIOState& cuffDIOS) ;
  void torsoFanCallback(const baxter_core_msgs::AnalogIOState& dios) ;
  void shoulderCallback(const baxter_core_msgs::DigitalIOState& shoulderDIOS) ;
  void armShowButtonCallback(const baxter_core_msgs::DigitalIOState& shoulderDIOS) ;
  void armBackButtonCallback(const baxter_core_msgs::DigitalIOState& shoulderDIOS) ;
  void armOkButtonCallback(const baxter_core_msgs::DigitalIOState& shoulderDIOS) ;
  void activateSensorStreaming();
  void deactivateSensorStreaming(); 
  void update_baxter();

  baxter_core_msgs::HeadPanCommand currentHeadPanCommand;
  baxter_core_msgs::SolvePositionIK currentJointPositions;
  baxter_core_msgs::SolvePositionIK ikRequest;
  baxter_core_msgs::SolvePositionIK lastGoodIkRequest;

  std_msgs::Bool currentHeadNodCommand;
  std_msgs::UInt16 currentSonarCommand;

  ros::ServiceClient ikClient;

  ros::Subscriber gravity_comp_sub;
  ros::Subscriber torso_fan_sub;
  ros::Subscriber arm_button_back_sub;
  ros::Subscriber arm_button_ok_sub;
  ros::Subscriber arm_button_show_sub;
  ros::Subscriber cuff_grasp_sub;
  ros::Subscriber cuff_ok_sub;
  ros::Subscriber shoulder_sub;

  ros::Subscriber eeRanger;
  ros::Subscriber epState;

  ros::Subscriber collisionDetectionState;
  ros::Subscriber gripState;
  ros::Subscriber eeAccelerator;
  ros::Subscriber eeTarget;
  ros::Subscriber jointSubscriber;

  ros::Publisher sonarPub;
  ros::Publisher headPub;
  ros::Publisher nodPub;
  ros::Publisher joint_mover;
  ros::Publisher digital_io_pub;
  ros::Publisher analog_io_pub;

  ros::Publisher sonar_pub;
  ros::Publisher red_halo_pub;
  ros::Publisher green_halo_pub;

  ros::Publisher face_screen_pub;
  ros::Publisher gripperPub;
  ros::Publisher facePub;
  ros::Publisher moveSpeedPub;
  ros::Publisher stiffPub;

  std_msgs::UInt32 currentStiffnessCommand;
  int sonar_led_state = 0;
  double red_halo_state = 100.0;
  double green_halo_state = 100.0;
  int repeat_halo = 1;

  ros::NodeHandle n;


  EinBaxterConfig();
};

#endif /* _EIN_BAXTER_CONFIG_H_ */
