#ifndef _EIN_SPOT_CONFIG_H_
#define _EIN_SPOT_CONFIG_H_


#include <Eigen/Geometry> 
using namespace Eigen;

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <spot_msgs/srv/set_locomotion.hpp>


class EinSpotConfig {

 public:
  MachineState * ms;

  EinSpotConfig(MachineState * ms);

  std::map<string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> trigger_clients;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr claim_client;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr sit_client;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stand_client;

  

};

#endif /* _EIN_SPOT_CONFIG_H_ */
