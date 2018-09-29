#ifndef _EIN_KUKA_CONFIG_H_
#define _EIN_KUKA_CONFIG_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>


#include <iiwa_msgs/CartesianEulerPose.h>

#include <moveit/move_group_interface/move_group.h>
using namespace moveit;
using namespace planning_interface;


#include "ein_words.h"
#include "config.h"
#include "camera.h"



class EinKukaConfig {
 public:
  MachineState * ms;
  EinKukaConfig(MachineState * ms);
  ros::NodeHandle n;

  
  MoveGroup * arm;
  vector<MoveGroup *> endEffectors;
  int focused_ee = 0;

  ros::Subscriber moveitStatusSubscriber;
  actionlib_msgs::GoalStatusArray moveitStatusMsg;
  void moveitStatusCallback(const actionlib_msgs::GoalStatusArray&m);
  map<string, actionlib_msgs::GoalStatus> goals;
  ros::Time lastMoveitCallTime;
  eePose lastMoveitCallPose;



  const static unsigned int num_joints=7;  
  const vector<double> homedJoints{0,0,0,0,0,0,0};
  const vector<std::string> upperBodyJoints{"a1",
					    "a2",
					    "a3",
					    "a4",
					    "a5",
					    "a6",
					    "a7"};


  double gridSize=0.01;  

  eePose armPose;
  eePose armTargetPose;

  void changeFocusedEndEffector(int idx);

};

#endif /* _EIN_KUKA_CONFIG_H_ */
