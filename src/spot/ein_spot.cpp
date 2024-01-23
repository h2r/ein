#include "ein_spot.h"
#include "config.h"
#include "camera.h"
#include "ein_words.h"
#include "ein_spot_config.h"



void robotInitializeSerial(MachineState * ms) {
}
void robotInitializeConfig(MachineState * ms) {
  
  Camera * c = new Camera(ms, "hand_camera", "spottopic", "hand_camera", "hand_camera");
  ms->config.cameras.push_back(c);
  ms->config.focused_camera = 0;

  ms->config.spotConfig = new EinSpotConfig(ms);

  ms->config.spotConfig->trigger_clients["sit"] = ms->config.node->create_client<std_srvs::srv::Trigger>("sit");

  ms->config.spotConfig->trigger_clients["stand"] = ms->config.node->create_client<std_srvs::srv::Trigger>("stand");

  ms->config.spotConfig->trigger_clients["claim"] = ms->config.node->create_client<std_srvs::srv::Trigger>("claim");

  ms->config.spotConfig->trigger_clients["release"] = ms->config.node->create_client<std_srvs::srv::Trigger>("release");


  ms->config.spotConfig->trigger_clients["power_on"] = ms->config.node->create_client<std_srvs::srv::Trigger>("power_on");

  ms->config.spotConfig->trigger_clients["power_off"] = ms->config.node->create_client<std_srvs::srv::Trigger>("power_off");

  ms->config.spotConfig->trigger_clients["arm_stow"] = ms->config.node->create_client<std_srvs::srv::Trigger>("arm_stow");
  ms->config.spotConfig->trigger_clients["arm_unstow"] = ms->config.node->create_client<std_srvs::srv::Trigger>("arm_unstow");      


  ms->config.spotConfig->trigger_clients["gripper_open"] = ms->config.node->create_client<std_srvs::srv::Trigger>("gripper_open");

  ms->config.spotConfig->trigger_clients["gripper_close"] = ms->config.node->create_client<std_srvs::srv::Trigger>("gripper_close");


  ms->config.spotConfig->trigger_clients["arm_carry"] = ms->config.node->create_client<std_srvs::srv::Trigger>("arm_carry");            

  for (std::map<string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr>::iterator it=ms->config.spotConfig->trigger_clients.begin(); it!=ms->config.spotConfig->trigger_clients.end(); ++it) {
    it->second->wait_for_service(2s);
    if (!rclcpp::ok()) {
      CONSOLE_ERROR(ms, "Interrupted while waiting for the service " << it->first << ".");
    }
  }

  
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
  
  
}
END_WORD \
REGISTER_WORD(WaitTrigger) 
  

#define CONFIG_TRIGGER_WORD(wordName, waitName, stringName, desc) \
WORD(wordName) \
 virtual string description() {\
  return desc;\
}\
virtual void execute(MachineState * ms) { \
 auto request = std::make_shared<std_srvs::srv::Trigger::Request>();\
  auto result = ms->config.spotConfig->trigger_clients[stringName]->async_send_request(request);\
} \
END_WORD \
REGISTER_WORD(wordName)\
WORD(waitName) \
 virtual string description() {\
  return desc  " Wait until action has completed.";\
}\
virtual void execute(MachineState * ms) { \
 auto request = std::make_shared<std_srvs::srv::Trigger::Request>();\
 auto result = ms->config.spotConfig->trigger_clients[stringName]->async_send_request(request);\
 ms->pushData(stringName); \
 ms->pushWord("waitTrigger"); \
} \
END_WORD \
REGISTER_WORD(waitName) 


CONFIG_TRIGGER_WORD(SpotSitAsync, SpotSit, "sit", "Sit the robot.");
CONFIG_TRIGGER_WORD(SpotStandAsync, SpotStand, "stand", "Stand the robot.");
CONFIG_TRIGGER_WORD(SpotClaimAsync, SpotClaim, "claim", "Claim the robot.");
CONFIG_TRIGGER_WORD(SpotReleaseAsync, SpotRelease, "release", "Release the robot.");
CONFIG_TRIGGER_WORD(SpotPowerOnAsync, SpotPowerOn, "power_on", "Power on the robot.");
CONFIG_TRIGGER_WORD(SpotPowerOffAsync, SpotPowerOff, "power_off", "Power off the robot.");
CONFIG_TRIGGER_WORD(SpotArmStowAsync, SpotArmStow, "arm_stow", "Stow the arm.");
CONFIG_TRIGGER_WORD(SpotArmUnstowAsync, SpotArmUnstow, "arm_unstow", "Unstow the arm.");
CONFIG_TRIGGER_WORD(SpotGripperOpenAsync, SpotGripperOpen, "gripper_open", "Open the gripper.");
CONFIG_TRIGGER_WORD(SpotGripperCloseAsync, SpotGripperClose, "gripper_close", "Close the gripper.");
CONFIG_TRIGGER_WORD(SpotArmCarryAsync, SpotArmCarry, "arm_carry", "Put the arm in carry position.");

}
