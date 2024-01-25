#include "ein_spot.h"
#include "config.h"
#include "camera.h"
#include "ein_words.h"
#include "ein_spot_config.h"

#define MC ms->config.spotConfig

void robotInitializeSerial(MachineState * ms) {
}
void robotInitializeConfig(MachineState * ms) {
  
  Camera * c = new Camera(ms, "hand_camera", "spottopic", "hand_camera", "hand_camera");
  ms->config.cameras.push_back(c);
  ms->config.focused_camera = 0;

  MC = new EinSpotConfig(ms);

  MC->trigger_clients["sit"] = ms->config.node->create_client<std_srvs::srv::Trigger>("sit");

  MC->trigger_clients["stand"] = ms->config.node->create_client<std_srvs::srv::Trigger>("stand");

  MC->trigger_clients["claim"] = ms->config.node->create_client<std_srvs::srv::Trigger>("claim");

  MC->trigger_clients["release"] = ms->config.node->create_client<std_srvs::srv::Trigger>("release");


  MC->trigger_clients["power_on"] = ms->config.node->create_client<std_srvs::srv::Trigger>("power_on");

  MC->trigger_clients["power_off"] = ms->config.node->create_client<std_srvs::srv::Trigger>("power_off");

  MC->trigger_clients["arm_stow"] = ms->config.node->create_client<std_srvs::srv::Trigger>("arm_stow");
  MC->trigger_clients["arm_unstow"] = ms->config.node->create_client<std_srvs::srv::Trigger>("arm_unstow");      


  MC->trigger_clients["gripper_open"] = ms->config.node->create_client<std_srvs::srv::Trigger>("gripper_open");

  MC->trigger_clients["gripper_close"] = ms->config.node->create_client<std_srvs::srv::Trigger>("gripper_close");


  MC->trigger_clients["arm_carry"] = ms->config.node->create_client<std_srvs::srv::Trigger>("arm_carry");            

  for (std::map<string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr>::iterator it=MC->trigger_clients.begin(); it!=MC->trigger_clients.end(); ++it) {
    it->second->wait_for_service(2s);
    if (!rclcpp::ok()) {
      CONSOLE_ERROR(ms, "Interrupted while waiting for the service " << it->first << ".");
    }
  }


  MC->cmdVelPub = ms->config.node->create_publisher<geometry_msgs::msg::Twist>("/movo/cmd_vel", 10);
  

  
}
void robotInitializeMachine(MachineState * ms) {


}

void robotSetCurrentJointPositions(MachineState * ms) {
}

void robotEndPointCallback(MachineState * ms) {
}

void robotActivateSensorStreaming(MachineState * ms){}
void robotDeactivateSensorStreaming(MachineState * ms){}
void robotUpdate(MachineState * ms){}


EinSpotConfig::EinSpotConfig(MachineState * ms)  {


}
  
namespace ein_words {


WORD(WaitTrigger) 
virtual string description() {
  return "Pop the name of a trigger from the stack; wait until it completes.";
}
virtual void execute(MachineState * ms) { 

  string trigger_name;
  GET_STRING_ARG(ms, trigger_name);
  if (MC->trigger_responses.find(trigger_name) !=
      MC->trigger_responses.end()) {
    if (MC->trigger_responses[trigger_name]) {
      ms->pushData(trigger_name);		      
      ms->pushWord("waitTrigger");       
    } else {
      CONSOLE_ERROR(ms, "Finished " << trigger_name);
    }
  } else {
    CONSOLE_ERROR(ms, "No response for trigger " << trigger_name << ".");
  }
  
}
END_WORD 
REGISTER_WORD(WaitTrigger) 
  

#define CONFIG_TRIGGER_WORD(wordName, wordStringName, waitName, stringName, desc) \
WORD(wordName) \
 virtual string description() {\
  return desc;\
}\
virtual void execute(MachineState * ms) { \
 MC->trigger_responses[stringName] = true; \
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();\
 CONSOLE_ERROR(ms, "Putting " << stringName << " into trigger responses."); \
 using TriggerResponseFuture = rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture; \
 auto response_received_callback = [ms](TriggerResponseFuture future) { \
   auto result = future.get(); \
   CONSOLE_ERROR(ms, "Got " << stringName << " result."); \
   MC->trigger_responses[stringName] = false;   \
 };		\
 auto result = MC->trigger_clients[stringName]->async_send_request(request, response_received_callback); \
} \
END_WORD \
REGISTER_WORD(wordName)\
WORD(waitName) \
 virtual string description() {\
  return desc  " Wait until action has completed.";\
}\
virtual void execute(MachineState * ms) { \
  ms->pushWord(wordStringName);   \
  ms->pushData(stringName);  \
  ms->pushWord("waitTrigger");  \
} \
END_WORD \
REGISTER_WORD(waitName) 


CONFIG_TRIGGER_WORD(SpotSitAsync, "spotSitAsync", SpotSit, "sit", "Sit the robot.");
CONFIG_TRIGGER_WORD(SpotStandAsync, "spotStandAsync", SpotStand, "stand", "Stand the robot.");
  //CONFIG_TRIGGER_WORD(SpotClaimAsync, SpotClaim, "claim", "Claim the robot.");
  //CONFIG_TRIGGER_WORD(SpotReleaseAsync, SpotRelease, "release", "Release the robot.");
  //CONFIG_TRIGGER_WORD(SpotPowerOnAsync, SpotPowerOn, "power_on", "Power on the robot.");
  //CONFIG_TRIGGER_WORD(SpotPowerOffAsync, SpotPowerOff, "power_off", "Power off the robot.");
  //CONFIG_TRIGGER_WORD(SpotArmStowAsync, SpotArmStow, "arm_stow", "Stow the arm.");
  //CONFIG_TRIGGER_WORD(SpotArmUnstowAsync, SpotArmUnstow, "arm_unstow", "Unstow the arm.");
  //CONFIG_TRIGGER_WORD(SpotGripperOpenAsync, SpotGripperOpen, "gripper_open", "Open the gripper.");
  //CONFIG_TRIGGER_WORD(SpotGripperCloseAsync, SpotGripperClose, "gripper_close", "Close the gripper.");
  //CONFIG_TRIGGER_WORD(SpotArmCarryAsync, SpotArmCarry, "arm_carry", "Put the arm in carry position.");




WORD(BaseStopTwist)
virtual string description() {
  return "Send a zero twist command.";
}
virtual void execute(MachineState * ms) {
  MC->twistMsg.linear.x = 0;
  MC->twistMsg.linear.y = 0;
  MC->twistMsg.linear.z = 0;
  MC->twistMsg.angular.x = 0;
  MC->twistMsg.angular.y = 0;
  MC->twistMsg.angular.z = 0;
  MC->cmdVelPub->publish(MC->twistMsg);
}
END_WORD
REGISTER_WORD(BaseStopTwist)




WORD(BaseSendOZVel)
virtual string description() {
  return "Rotate the base at a velocity.";
}
virtual void execute(MachineState * ms) {
  double angularspeed;
  GET_NUMERIC_ARG(ms, angularspeed);

  MC->twistMsg.linear.x = 0;
  MC->twistMsg.linear.y = 0;
  MC->twistMsg.linear.z = 0;
  MC->twistMsg.angular.x = 0;
  MC->twistMsg.angular.y = 0;
  MC->twistMsg.angular.z = angularspeed;
  MC->cmdVelPub->publish(MC->twistMsg);
}
END_WORD
REGISTER_WORD(BaseSendOZVel)

WORD(BaseSendXVel)
virtual string description() {
  return "Move the base in X at a velocity.";
}
virtual void execute(MachineState * ms) {
  double speed;
  GET_NUMERIC_ARG(ms, speed);

  MC->twistMsg.linear.x = speed;
  MC->twistMsg.linear.y = 0;
  MC->twistMsg.linear.z = 0;
  MC->twistMsg.angular.x = 0;
  MC->twistMsg.angular.y = 0;
  MC->twistMsg.angular.z = 0;
  MC->cmdVelPub->publish(MC->twistMsg);
}
END_WORD
REGISTER_WORD(BaseSendXVel)


WORD(BaseSendYVel)
virtual string description() {
  return "Move the base in Y at a velocity.";
}
virtual void execute(MachineState * ms) {
  double speed;
  GET_NUMERIC_ARG(ms, speed);

  MC->twistMsg.linear.x = 0;
  MC->twistMsg.linear.y = speed;
  MC->twistMsg.linear.z = 0;
  MC->twistMsg.angular.x = 0;
  MC->twistMsg.angular.y = 0;
  MC->twistMsg.angular.z = 0;
  MC->cmdVelPub->publish(MC->twistMsg);
}
END_WORD
REGISTER_WORD(BaseSendYVel)

  


  }
