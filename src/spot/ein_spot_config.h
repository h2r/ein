#ifndef _EIN_SPOT_CONFIG_H_
#define _EIN_SPOT_CONFIG_H_


#include <Eigen/Geometry> 
using namespace Eigen;

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <spot_msgs/srv/set_locomotion.hpp>

#include <spot_msgs/srv/get_inverse_kinematic_solutions.hpp>

#include <spot_msgs/action/robot_command.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include "sensor_msgs/msg/joint_state.hpp"


class EinSpotConfig {

 public:
  MachineState * ms;

  EinSpotConfig(MachineState * ms);

  std::map<string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> trigger_clients;


  std::map<string, bool> trigger_responses;
  //  std::map<string, rclcpp::Client<std_srvs::srv::Trigger>::FutureAndRequestId> trigger_responses;
	   //std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>::FutureAndRequestId> > trigger_responses;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr claim_client;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr sit_client;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stand_client;

  rclcpp::Client<spot_msgs::srv::GetInverseKinematicSolutions>::SharedPtr ik_client;
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub;


  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSub;

  geometry_msgs::msg::Twist twistMsg;

  sensor_msgs::msg::JointState jointMsg;
  
  virtual void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr);

};

#endif /* _EIN_SPOT_CONFIG_H_ */
