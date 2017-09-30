
#include "ein_words.h"
#include "ein.h"
#include "camera.h"
#include "eigen_util.h"

namespace ein_words {

CONFIG_GETTER_DOUBLE(GridSize, ms->config.bDelta)
CONFIG_SETTER_DOUBLE(SetGridSize, ms->config.bDelta)


CONFIG_GETTER_DOUBLE(W1GoThresh, ms->config.w1GoThresh)
CONFIG_SETTER_DOUBLE(SetW1GoThresh, ms->config.w1GoThresh)
CONFIG_GETTER_DOUBLE(W1AngleThresh, ms->config.w1AngleThresh)
CONFIG_SETTER_DOUBLE(SetW1AngleThresh, ms->config.w1AngleThresh)

CONFIG_GETTER_INT(CurrentIKBoundaryMode, ms->config.currentIKBoundaryMode)
CONFIG_SETTER_ENUM(SetCurrentIKBoundaryMode, ms->config.currentIKBoundaryMode, (ikBoundaryMode))

CONFIG_GETTER_INT(CurrentIKFastMode, ms->config.currentIKFastMode)
CONFIG_SETTER_ENUM(SetCurrentIKFastMode, ms->config.currentIKFastMode, (ikFastMode))

WORD(AssumeAimedPose)
virtual void execute(MachineState * ms) {
  int idxToRemove = ms->config.targetBlueBox;
  BoxMemory memory = ms->config.blueBoxMemories[idxToRemove];
  ms->config.currentEEPose = memory.aimedPose;
  ms->pushWord("waitUntilAtCurrentPosition");
}
END_WORD
REGISTER_WORD(AssumeAimedPose)

WORD(AssumePose)
virtual void execute(MachineState * ms) {
  shared_ptr<EePoseWord> word ;
  GET_WORD_ARG(ms, EePoseWord, word);

  ms->config.currentEEPose = word->value();
  ms->pushWord("waitUntilAtCurrentPosition");
}
END_WORD
REGISTER_WORD(AssumePose)


WORD(AssumeBackScanningPose)
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose = ms->config.backScanningPose;
  ms->pushWord("waitUntilAtCurrentPosition");
}
END_WORD
REGISTER_WORD(AssumeBackScanningPose)

WORD(WaitUntilAtCurrentPositionCollapse)
virtual void execute(MachineState * ms) {

  ms->config.currentMovementState = MOVING;
  ms->config.lastTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
  ms->config.lastMovementStateSet = ros::Time::now();

  ms->config.waitUntilAtCurrentPositionCounter = 0;
  ms->config.waitUntilAtCurrentPositionStart = ros::Time::now();
  double dx = (ms->config.currentEEPose.px - ms->config.trueEEPose.position.x);
  double dy = (ms->config.currentEEPose.py - ms->config.trueEEPose.position.y);
  double dz = (ms->config.currentEEPose.pz - ms->config.trueEEPose.position.z);
  double distance = dx*dx + dy*dy + dz*dz;
  
  double qx = (fabs(ms->config.currentEEPose.qx) - fabs(ms->config.trueEEPose.orientation.x));
  double qy = (fabs(ms->config.currentEEPose.qy) - fabs(ms->config.trueEEPose.orientation.y));
  double qz = (fabs(ms->config.currentEEPose.qz) - fabs(ms->config.trueEEPose.orientation.z));
  double qw = (fabs(ms->config.currentEEPose.qw) - fabs(ms->config.trueEEPose.orientation.w));
  double angleDistance = qx*qx + qy*qy + qz*qz + qw*qw;
  
  if ((distance > ms->config.w1GoThresh*ms->config.w1GoThresh) || (angleDistance > ms->config.w1AngleThresh*ms->config.w1AngleThresh)) {
    ms->pushWord("waitUntilAtCurrentPositionB"); 
    ms->config.endThisStackCollapse = 1;
    ms->config.shouldIDoIK = 1;
  } else {
  }

  if (ms->config.currentControlMode == VELOCITY) {
  } else if (ms->config.currentControlMode == EEPOSITION) {
  } else if (ms->config.currentControlMode == ANGLES) {
    ms->pushWord("setCurrentPoseFromJoints"); 
  } else {
    assert(0);
  }
}
END_WORD
REGISTER_WORD(WaitUntilAtCurrentPositionCollapse)

WORD(WaitUntilAtCurrentPosition)
CODE(131154)    // capslock + r
virtual void execute(MachineState * ms) {

  ms->config.currentMovementState = MOVING;
  ms->config.lastTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
  ms->config.lastMovementStateSet = ros::Time::now();

  ms->config.waitUntilAtCurrentPositionCounter = 0;
  ms->config.waitUntilAtCurrentPositionStart = ros::Time::now();

  double distance, angleDistance;
  eePose::distanceXYZAndAngle(ms->config.currentEEPose, ms->config.trueEEPoseEEPose, &distance, &angleDistance);

  if ((distance > ms->config.w1GoThresh*ms->config.w1GoThresh) || (angleDistance > ms->config.w1AngleThresh*ms->config.w1AngleThresh)) {
    ms->pushWord("waitUntilAtCurrentPositionB"); 
    ms->config.endThisStackCollapse = 1;
    ms->config.shouldIDoIK = 1;
  } else {
    // try not collapsing if you are where you want to be
    ms->config.endThisStackCollapse = 1;
  }

  if (ms->config.currentControlMode == VELOCITY) {
  } else if (ms->config.currentControlMode == EEPOSITION) {
  } else if (ms->config.currentControlMode == ANGLES) {
    ms->pushWord("setCurrentPoseFromJoints"); 
  } else {
    assert(0);
  }
}
END_WORD
REGISTER_WORD(WaitUntilAtCurrentPosition)


WORD(IsAtCurrentPosition)
virtual void execute(MachineState * ms) {

  ms->config.currentMovementState = MOVING;
  ms->config.lastTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
  ms->config.lastMovementStateSet = ros::Time::now();

  ms->config.waitUntilAtCurrentPositionCounter = 0;

  double distance, angleDistance;
  eePose::distanceXYZAndAngle(ms->config.currentEEPose, ms->config.trueEEPoseEEPose, &distance, &angleDistance);

  std::shared_ptr<IntegerWord> newWord;
  if ((distance > ms->config.w1GoThresh*ms->config.w1GoThresh) || (angleDistance > ms->config.w1AngleThresh*ms->config.w1AngleThresh)) {
    newWord = std::make_shared<IntegerWord>(0);
  } else {
    newWord = std::make_shared<IntegerWord>(1);
  }
  ms->pushWord(newWord);
}
END_WORD
REGISTER_WORD(IsAtCurrentPosition)

CONFIG_GETTER_INT(WaitGetCurrentWaitMode, ms->config.currentWaitMode)
CONFIG_SETTER_ENUM(WaitSetCurrentWaitMode, ms->config.currentWaitMode, (waitMode))


WORD(WaitUntilAtCurrentPositionB)
virtual void execute(MachineState * ms) {
  //if (ms->config.waitUntilAtCurrentPositionCounter < ms->config.waitUntilAtCurrentPositionCounterTimeout) 
  if ( ros::Time::now().toSec() - ms->config.waitUntilAtCurrentPositionStart.toSec() < ms->config.waitUntilAtCurrentPositionTimeout )
  {


    ms->config.waitUntilAtCurrentPositionCounter++;

    double distance, angleDistance;
    eePose::distanceXYZAndAngle(ms->config.currentEEPose, ms->config.trueEEPoseEEPose, &distance, &angleDistance);

    if ((distance > ms->config.w1GoThresh*ms->config.w1GoThresh) || (angleDistance > ms->config.w1AngleThresh*ms->config.w1AngleThresh)) {
      if ( (ms->config.currentMovementState == STOPPED) ||
	   (ms->config.currentMovementState == BLOCKED) ) {

	if (ms->config.currentMovementState == STOPPED) {
	  //cout << "Warning: waitUntilAtCurrentPosition ms->config.currentMovementState = STOPPED, moving on." << endl;
	  ms->config.endThisStackCollapse = ms->config.endCollapse;
	}
	if (ms->config.currentMovementState == BLOCKED) {
	  //cout << "Warning: waitUntilAtCurrentPosition ms->config.currentMovementState = BLOCKED, moving on." << endl;
	  ms->config.endThisStackCollapse = ms->config.endCollapse;
	}
	
	if (ms->config.currentWaitMode == WAIT_KEEP_ON) {
	  //cout << "waitUntilAtCurrentPositionB: currentWaitMode WAIT_KEEP_ON, so doing nothing...";
	} else if (ms->config.currentWaitMode == WAIT_BACK_UP) {
	  cout << "waitUntilAtCurrentPositionB: currentWaitMode WAIT_BACK_UP, so...";
	  ms->config.currentEEPose.pz = ms->config.trueEEPose.position.z + 0.001;
	  cout << "  backing up just a little to dislodge, then waiting again." << endl;
	} else {
	  assert(0);
	}

	ms->evaluateProgram("0.1 waitForSeconds waitUntilAtCurrentPositionB");
	ms->config.endThisStackCollapse = 1;
	return;
      }

      ms->pushWord("waitUntilAtCurrentPositionB"); 
      ms->config.endThisStackCollapse = 1;
      ms->config.shouldIDoIK = 1;
    } else {
      ms->config.endThisStackCollapse = ms->config.endCollapse;
    }
  } else {
    cout << "Warning: waitUntilAtCurrentPosition timed out, moving on." << endl;
    ms->config.endThisStackCollapse = 1;
  }
}
END_WORD
REGISTER_WORD(WaitUntilAtCurrentPositionB)

WORD(WaitUntilGripperNotMoving)
virtual void execute(MachineState * ms) {
  ms->config.waitUntilGripperNotMovingCounter = 0;
  ms->config.lastGripperCallbackRequest = ros::Time::now();
  ms->pushWord("waitUntilGripperNotMovingB"); 
  ms->config.endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(WaitUntilGripperNotMoving)

WORD(WaitUntilGripperNotMovingB)
virtual void execute(MachineState * ms) {
  if (ms->config.lastGripperCallbackRequest >= ms->config.lastGripperCallbackReceived) {
    ms->pushWord("waitUntilGripperNotMovingB"); 
  } else {
    ms->config.lastGripperCallbackRequest = ros::Time::now();
    if (ms->config.waitUntilGripperNotMovingCounter < ms->config.waitUntilGripperNotMovingTimeout) {
      if (ms->config.gripperMoving) {
	ms->config.waitUntilGripperNotMovingCounter++;
	ms->pushWord("waitUntilGripperNotMovingB"); 
      } else {
	ms->pushWord("waitUntilGripperNotMovingC"); 
	ms->config.waitUntilGripperNotMovingStamp = ros::Time::now();
	ms->config.waitUntilGripperNotMovingCounter = 0;
      }
    } else {
      cout << "Warning: waitUntilGripperNotMovingB timed out, moving on." << endl;
    }
  }
  ms->config.endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(WaitUntilGripperNotMovingB)

WORD(WaitUntilGripperNotMovingC)
virtual void execute(MachineState * ms) {
// waits until gripper has not been moving for gripperNotMovingConfirmTime
  if (ms->config.lastGripperCallbackRequest >= ms->config.lastGripperCallbackReceived) {
    ms->pushWord("waitUntilGripperNotMovingC"); 
  } else {
    ms->config.lastGripperCallbackRequest = ros::Time::now();
    if (ms->config.waitUntilGripperNotMovingCounter < ms->config.waitUntilGripperNotMovingTimeout) {
      ros::Duration deltaSinceUpdate = ms->config.gripperLastUpdated - ms->config.waitUntilGripperNotMovingStamp;
      if (deltaSinceUpdate.toSec() <= ms->config.gripperNotMovingConfirmTime) {
	ms->config.waitUntilGripperNotMovingCounter++;
	ms->pushWord("waitUntilGripperNotMovingC"); 
      }
    } else {
      cout << "Warning: waitUntilGripperNotMovingC timed out, moving on." << endl;
    }
  }
  ms->config.endThisStackCollapse = 1;
}
END_WORD
REGISTER_WORD(WaitUntilGripperNotMovingC)

WORD(PerturbPositionScale)
virtual void execute(MachineState * ms) {
  double param_perturbScale = 0.05;//0.1;
  GET_NUMERIC_ARG(ms, param_perturbScale);

  double noX = param_perturbScale * ((drand48() - 0.5) * 2.0);
  double noY = param_perturbScale * ((drand48() - 0.5) * 2.0);
  double noTheta = 3.1415926 * ((drand48() - 0.5) * 2.0);
  
  ms->config.currentEEPose.px += noX;
  ms->config.currentEEPose.py += noY;

  ms->config.currentEEDeltaRPY.pz += noTheta;
  endEffectorAngularUpdate(&ms->config.currentEEPose, &ms->config.currentEEDeltaRPY);
}
END_WORD
REGISTER_WORD(PerturbPositionScale)


WORD(PerturbPosition)
CODE(1048623)     // numlock + /
virtual void execute(MachineState * ms) {

  stringstream cmd;
  cmd << "0.05 perturbPositionScale";
  ms->evaluateProgram(cmd.str());
}
END_WORD
REGISTER_WORD(PerturbPosition)

WORD(OYDown)
CODE('w'+65504) 
virtual void execute(MachineState * ms) {
  ms->config.currentEEDeltaRPY.py -= ms->config.bDelta;
  endEffectorAngularUpdate(&ms->config.currentEEPose, &ms->config.currentEEDeltaRPY);
}
END_WORD
REGISTER_WORD(OYDown)

WORD(OYUp)
CODE('s'+65504) 
virtual void execute(MachineState * ms) {
  ms->config.currentEEDeltaRPY.py += ms->config.bDelta;
  endEffectorAngularUpdate(&ms->config.currentEEPose, &ms->config.currentEEDeltaRPY);
}
END_WORD
REGISTER_WORD(OYUp)

WORD(OZDown)
CODE('q'+65504) 
virtual void execute(MachineState * ms) {
  ms->config.currentEEDeltaRPY.pz -= ms->config.bDelta;
  endEffectorAngularUpdate(&ms->config.currentEEPose, &ms->config.currentEEDeltaRPY);
}
END_WORD
REGISTER_WORD(OZDown)

WORD(OZUp)
CODE('e'+65504) 
virtual void execute(MachineState * ms) {
  ms->config.currentEEDeltaRPY.pz += ms->config.bDelta;
  endEffectorAngularUpdate(&ms->config.currentEEPose, &ms->config.currentEEDeltaRPY);
}
END_WORD
REGISTER_WORD(OZUp)

WORD(OXDown)
CODE('a'+65504) 
virtual void execute(MachineState * ms) {
  ms->config.currentEEDeltaRPY.px -= ms->config.bDelta;
  endEffectorAngularUpdate(&ms->config.currentEEPose, &ms->config.currentEEDeltaRPY);
}
END_WORD
REGISTER_WORD(OXDown)

WORD(OXUp)
CODE('d'+65504) 
virtual void execute(MachineState * ms) {
  ms->config.currentEEDeltaRPY.px += ms->config.bDelta;
  endEffectorAngularUpdate(&ms->config.currentEEPose, &ms->config.currentEEDeltaRPY);
}
END_WORD
REGISTER_WORD(OXUp)


WORD(CurrentPose)
virtual void execute(MachineState * ms) {
  shared_ptr<EePoseWord> word = std::make_shared<EePoseWord>(ms->config.currentEEPose);
  ms->pushWord(word);
}
END_WORD
REGISTER_WORD(CurrentPose)

WORD(SaveRegister1)
CODE(65568+1) // ! 
virtual void execute(MachineState * ms) {
  ms->config.eepReg1 = ms->config.currentEEPose;
}
END_WORD
REGISTER_WORD(SaveRegister1)

WORD(SaveRegister2)
CODE(65600) // @
virtual void execute(MachineState * ms) {
  ms->config.eepReg2 = ms->config.currentEEPose;
}
END_WORD
REGISTER_WORD(SaveRegister2)

WORD(SaveRegister3)
CODE(65568+3) // # 
virtual void execute(MachineState * ms) {
  ms->config.eepReg3 = ms->config.currentEEPose;
}
END_WORD
REGISTER_WORD(SaveRegister3)

WORD(SaveRegister4)
CODE( 65568+4) // $ 
virtual void execute(MachineState * ms) {
  ms->config.eepReg4 = ms->config.currentEEPose;
}
END_WORD
REGISTER_WORD(SaveRegister4)


WORD(MoveToRegister)
virtual void execute(MachineState * ms) {
  int register_num = 0;
  GET_ARG(ms, IntegerWord, register_num);

  if (register_num == 1) {
    ms->config.currentEEPose = ms->config.eepReg1;
  } else if (register_num == 2) {
    ms->config.currentEEPose = ms->config.eepReg2;
  } else if (register_num == 3) {
    ms->config.currentEEPose = ms->config.eepReg3;
  } else if (register_num == 4) {
    ms->config.currentEEPose = ms->config.eepReg4;
  } else if (register_num == 5) {
    ms->config.currentEEPose = ms->config.eepReg5;
  } else if (register_num == 6) {
    ms->config.currentEEPose = ms->config.eepReg6;
  } else {
    cout << "Bad register number: " << " int: " << register_num << endl;
  }
}
END_WORD
REGISTER_WORD(MoveToRegister)

WORD(MoveToRegister1)
CODE('1') 
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose = ms->config.eepReg1;
}
END_WORD
REGISTER_WORD(MoveToRegister1)

WORD(MoveToRegister2)
CODE('2') 
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose = ms->config.eepReg2;
}
END_WORD
REGISTER_WORD(MoveToRegister2)

WORD(MoveToRegister3)
CODE('3') 
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose = ms->config.eepReg3;
}
END_WORD
REGISTER_WORD(MoveToRegister3)

WORD(MoveToRegister4)
CODE('4') 
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose = ms->config.eepReg4;
}
END_WORD
REGISTER_WORD(MoveToRegister4)

WORD(MoveToRegister5)
CODE('5') 
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose = ms->config.eepReg5;
}
END_WORD
REGISTER_WORD(MoveToRegister5)


WORD(MoveToRegister6)
CODE('6') 
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose = ms->config.eepReg6;
}
END_WORD
REGISTER_WORD(MoveToRegister6)

WORD(LocalXDown)
virtual void execute(MachineState * ms) {
  Vector3d localUnitX;
  Vector3d localUnitY;
  Vector3d localUnitZ;
  fillLocalUnitBasis(ms->config.currentEEPose, &localUnitX, &localUnitY, &localUnitZ);
  ms->config.currentEEPose = eePoseMinus(ms->config.currentEEPose, ms->config.bDelta * localUnitX);
}
END_WORD
REGISTER_WORD(LocalXDown)


WORD(LocalXUp)
virtual void execute(MachineState * ms) {
  Vector3d localUnitX;
  Vector3d localUnitY;
  Vector3d localUnitZ;
  fillLocalUnitBasis(ms->config.currentEEPose, &localUnitX, &localUnitY, &localUnitZ);
  ms->config.currentEEPose = eePosePlus(ms->config.currentEEPose, ms->config.bDelta * localUnitX);
}
END_WORD
REGISTER_WORD(LocalXUp)

WORD(LocalYDown)
virtual void execute(MachineState * ms) {
  Vector3d localUnitX;
  Vector3d localUnitY;
  Vector3d localUnitZ;
  fillLocalUnitBasis(ms->config.currentEEPose, &localUnitX, &localUnitY, &localUnitZ);
  ms->config.currentEEPose = eePoseMinus(ms->config.currentEEPose, ms->config.bDelta * localUnitY);
}
END_WORD
REGISTER_WORD(LocalYDown)


WORD(LocalYUp)
virtual void execute(MachineState * ms) {
  Vector3d localUnitX;
  Vector3d localUnitY;
  Vector3d localUnitZ;
  fillLocalUnitBasis(ms->config.currentEEPose, &localUnitX, &localUnitY, &localUnitZ);
  ms->config.currentEEPose = eePosePlus(ms->config.currentEEPose, ms->config.bDelta * localUnitY);
}
END_WORD
REGISTER_WORD(LocalYUp)


WORD(LocalZUp)
virtual void execute(MachineState * ms)
{
  Vector3d localUnitX;
  Vector3d localUnitY;
  Vector3d localUnitZ;
  fillLocalUnitBasis(ms->config.currentEEPose, &localUnitX, &localUnitY, &localUnitZ);
  ms->config.currentEEPose = eePosePlus(ms->config.currentEEPose, ms->config.bDelta * localUnitZ);
}
END_WORD
REGISTER_WORD(LocalZUp)

WORD(LocalZDown)
virtual void execute(MachineState * ms)
{
  Vector3d localUnitX;
  Vector3d localUnitY;
  Vector3d localUnitZ;
  fillLocalUnitBasis(ms->config.currentEEPose, &localUnitX, &localUnitY, &localUnitZ);
  ms->config.currentEEPose = eePoseMinus(ms->config.currentEEPose, ms->config.bDelta * localUnitZ);
}
END_WORD
REGISTER_WORD(LocalZDown)

WORD(XDown)
CODE('q') 
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose.px -= ms->config.bDelta;
}
END_WORD
REGISTER_WORD(XDown)


WORD(XUp)
CODE('e') 
virtual string description() {
  return "Move end effector up in the x dimension one unit.  Change unit size with setGridSize.  Usage:  xUp";
}
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose.px += ms->config.bDelta;
}
END_WORD
REGISTER_WORD(XUp)

WORD(YDown)
CODE('a') 
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose.py -= ms->config.bDelta;
}
END_WORD
REGISTER_WORD(YDown)


WORD(YUp)
CODE('d') 
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose.py += ms->config.bDelta;
}
END_WORD
REGISTER_WORD(YUp)


WORD(ZUp)
CODE('w')
virtual void execute(MachineState * ms)
{
  ms->config.currentEEPose.pz += ms->config.bDelta;
}
END_WORD
REGISTER_WORD(ZUp)

WORD(ZDown)
CODE('s')
virtual void execute(MachineState * ms)
{
    ms->config.currentEEPose.pz -= ms->config.bDelta;
}
END_WORD
REGISTER_WORD(ZDown)

WORD(SetGripperThresh)
CODE(1179713)     // capslock + numlock + a
virtual void execute(MachineState * ms) {
  double param_lastMeasuredBias = 1;
  ms->config.gripperThresh = ms->config.lastMeasuredClosed + param_lastMeasuredBias;
  cout << "lastMeasuredClosed: " << ms->config.lastMeasuredClosed << " lastMeasuredBias: " << param_lastMeasuredBias << endl;
  cout << "gripperThresh = " << ms->config.gripperThresh << endl;
}
END_WORD
REGISTER_WORD(SetGripperThresh)

WORD(CalibrateGripper)
CODE('i') 
virtual void execute(MachineState * ms) {
  //baxter_core_msgs::EndEffectorCommand command;
  //command.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
  //command.id = 65538;
  //ms->config.gripperPub.publish(command);
  calibrateGripper(ms);
}
END_WORD
REGISTER_WORD(CalibrateGripper)

WORD(TuckArms)
virtual void execute(MachineState * ms) {
  int return_value;
  return_value = system("rosrun baxter_tools tuck_arms.py -t");
}
END_WORD
REGISTER_WORD(TuckArms)

WORD(UntuckArms)
virtual void execute(MachineState * ms) {
  int return_value;
  return_value = system("rosrun baxter_tools tuck_arms.py -u");
}
END_WORD
REGISTER_WORD(UntuckArms)



WORD(SetGridSizeNowThatsCoarse)
CODE(1114193)    // numlock + Q
virtual void execute(MachineState * ms) {
  ms->config.bDelta = NOW_THATS_COARSE;
}
END_WORD
REGISTER_WORD(SetGridSizeNowThatsCoarse)

WORD(SetGridSizeEvenCoarser)
CODE(1114199)     // numlock + W
virtual void execute(MachineState * ms) {
  ms->config.bDelta = GRID_EVEN_COARSER;
}
END_WORD
REGISTER_WORD(SetGridSizeEvenCoarser)


WORD(SetGridSizeCoarser)
CODE(1114181)  // numlock + E
virtual void execute(MachineState * ms) {
  ms->config.bDelta = GRID_COARSER;
}
END_WORD
REGISTER_WORD(SetGridSizeCoarser)

WORD(SetGridSizeCoarse)
CODE(1048674)     // numlock + b
virtual void execute(MachineState * ms)  {
  ms->config.bDelta = GRID_COARSE;
}
END_WORD
REGISTER_WORD(SetGridSizeCoarse)

WORD(SetGridSizeMedium)
CODE(1048686)   // numlock + n
virtual void execute(MachineState * ms) {
  ms->config.bDelta = GRID_MEDIUM;
}
END_WORD
REGISTER_WORD(SetGridSizeMedium)

WORD(SetGridSizeFine)
CODE(1114190) // numlock + N
virtual void execute(MachineState * ms) {
  ms->config.bDelta = GRID_FINE;
}
END_WORD
REGISTER_WORD(SetGridSizeFine)

WORD(SetGridSizeVeryFine)
CODE(1114178) // numlock + B
virtual void execute(MachineState * ms) {
	ms->config.bDelta = GRID_VERY_FINE;
}
END_WORD
REGISTER_WORD(SetGridSizeVeryFine)

WORD(ChangeToHeight)
virtual void execute(MachineState * ms) {
  int nextHeight = 0;
  GET_ARG(ms, IntegerWord, nextHeight);

  if ( (nextHeight > -1) && (nextHeight < ms->config.hmWidth) ) {
    ms->config.currentThompsonHeightIdx = nextHeight;
    cout << "changeToHeight changing to height " << nextHeight << endl;
  } else {
    cout << "changeToHeight received invalid height, clearing stack." << endl;
    ms->clearStack();
  }

  ms->config.currentThompsonHeight = convertHeightIdxToGlobalZ(ms, ms->config.currentThompsonHeightIdx);
  ms->config.currentEEPose.pz = ms->config.currentThompsonHeight;
  // ATTN 23
  for (int i = 0; i < ms->config.cameras.size(); i++) {
    ms->config.cameras[i]->reticle = ms->config.cameras[i]->vanishingPointReticle;
    ms->config.cameras[i]->m_x = ms->config.cameras[i]->m_x_h[ms->config.currentThompsonHeightIdx];
    ms->config.cameras[i]->m_y = ms->config.cameras[i]->m_y_h[ms->config.currentThompsonHeightIdx];
  }
  //camera->reticle = heightReticles[ms->config.currentThompsonHeightIdx];
}
END_WORD
REGISTER_WORD(ChangeToHeight)

WORD(ChangeToHeight0)
CODE(1245217) // capslock + numlock + !
virtual void execute(MachineState * ms) {
  ms->config.currentThompsonHeightIdx = 0;
  ms->config.currentThompsonHeight = convertHeightIdxToGlobalZ(ms, ms->config.currentThompsonHeightIdx);
  ms->config.currentEEPose.pz = ms->config.currentThompsonHeight;
  // ATTN 23
  for (int i = 0; i < ms->config.cameras.size(); i++) {
    ms->config.cameras[i]->reticle = ms->config.cameras[i]->vanishingPointReticle;
    ms->config.cameras[i]->m_x = ms->config.cameras[i]->m_x_h[ms->config.currentThompsonHeightIdx];
    ms->config.cameras[i]->m_y = ms->config.cameras[i]->m_y_h[ms->config.currentThompsonHeightIdx];
  }
}
END_WORD
REGISTER_WORD(ChangeToHeight0)

WORD(ChangeToHeight1)
CODE(1245248)     // capslock + numlock + @
virtual void execute(MachineState * ms) {
  ms->config.currentThompsonHeightIdx = 1;
  ms->config.currentThompsonHeight = convertHeightIdxToGlobalZ(ms, ms->config.currentThompsonHeightIdx);
  ms->config.currentEEPose.pz = ms->config.currentThompsonHeight;
  // ATTN 23
  for (int i = 0; i < ms->config.cameras.size(); i++) {
    ms->config.cameras[i]->reticle = ms->config.cameras[i]->vanishingPointReticle;
    ms->config.cameras[i]->m_x = ms->config.cameras[i]->m_x_h[ms->config.currentThompsonHeightIdx];
    ms->config.cameras[i]->m_y = ms->config.cameras[i]->m_y_h[ms->config.currentThompsonHeightIdx];
  }
}
END_WORD
REGISTER_WORD(ChangeToHeight1)

WORD(ChangeToHeight2)
CODE(1245219)  // capslock + numlock + #
virtual void execute(MachineState * ms)  {
  ms->config.currentThompsonHeightIdx = 2;
  ms->config.currentThompsonHeight = convertHeightIdxToGlobalZ(ms, ms->config.currentThompsonHeightIdx);
  ms->config.currentEEPose.pz = ms->config.currentThompsonHeight;
  // ATTN 23
  for (int i = 0; i < ms->config.cameras.size(); i++) {
    ms->config.cameras[i]->reticle = ms->config.cameras[i]->vanishingPointReticle;
    ms->config.cameras[i]->m_x = ms->config.cameras[i]->m_x_h[ms->config.currentThompsonHeightIdx];
    ms->config.cameras[i]->m_y = ms->config.cameras[i]->m_y_h[ms->config.currentThompsonHeightIdx];
  }
}
END_WORD
REGISTER_WORD(ChangeToHeight2)

WORD(ChangeToHeight3)
CODE(1245220) // capslock + numlock + $
virtual void execute(MachineState * ms) {
  ms->config.currentThompsonHeightIdx = 3;
  ms->config.currentThompsonHeight = convertHeightIdxToGlobalZ(ms, ms->config.currentThompsonHeightIdx);
  ms->config.currentEEPose.pz = ms->config.currentThompsonHeight;
  // ATTN 23
  for (int i = 0; i < ms->config.cameras.size(); i++) {
    ms->config.cameras[i]->reticle = ms->config.cameras[i]->vanishingPointReticle;
    ms->config.cameras[i]->m_x = ms->config.cameras[i]->m_x_h[ms->config.currentThompsonHeightIdx];
    ms->config.cameras[i]->m_y = ms->config.cameras[i]->m_y_h[ms->config.currentThompsonHeightIdx];
  }
}
END_WORD
REGISTER_WORD(ChangeToHeight3)

WORD(HundredthImpulse)
virtual void execute(MachineState * ms) {
  ms->config.currentEESpeedRatio = 0.01;
}
END_WORD
REGISTER_WORD(HundredthImpulse)

WORD(TenthImpulse)
virtual void execute(MachineState * ms) {
  ms->config.currentEESpeedRatio = 0.1;
}
END_WORD
REGISTER_WORD(TenthImpulse)

WORD(QuarterImpulse)
virtual void execute(MachineState * ms) {
  ms->config.currentEESpeedRatio = 0.25;
}
END_WORD
REGISTER_WORD(QuarterImpulse)

WORD(HalfImpulse)
virtual void execute(MachineState * ms) {
  ms->config.currentEESpeedRatio = 0.5;
}
END_WORD
REGISTER_WORD(HalfImpulse)

WORD(FullImpulse)
virtual void execute(MachineState * ms) {
  ms->config.currentEESpeedRatio = 1.0;
}
END_WORD
REGISTER_WORD(FullImpulse)

WORD(CruisingSpeed)
virtual void execute(MachineState * ms) {
  //w1GoThresh = 0.40;
  //ms->config.currentEESpeedRatio = 0.75;
  ms->config.currentEESpeedRatio = 1.0;
}
END_WORD
REGISTER_WORD(CruisingSpeed)

WORD(ApproachSpeed)
virtual void execute(MachineState * ms) {
  //w1GoThresh = 0.01;
  ms->config.currentEESpeedRatio = 0.05;//0.035;//0.07;//0.05;
}
END_WORD
REGISTER_WORD(ApproachSpeed)

WORD(DepartureSpeed)
virtual void execute(MachineState * ms) {
  //ms->config.currentEESpeedRatio = 0.5;
  ms->config.currentEESpeedRatio = 0.05;
}
END_WORD
REGISTER_WORD(DepartureSpeed)

WORD(ResetW1ThreshToDefault)
virtual void execute(MachineState * ms) {
  ms->config.w1GoThresh = 0.03;
}
END_WORD
REGISTER_WORD(ResetW1ThreshToDefault)

WORD(RasterScanningSpeed)
virtual void execute(MachineState * ms) {
  //w1GoThresh = 0.05;
  ms->config.currentEESpeedRatio = 0.1;//0.025;//0.02;
}
END_WORD
REGISTER_WORD(RasterScanningSpeed)

WORD(StreamImageSpeed)
virtual void execute(MachineState * ms) {
  ms->config.currentEESpeedRatio = 0.05;
}
END_WORD
REGISTER_WORD(StreamImageSpeed)

WORD(FasterRasterScanningSpeed)
virtual void execute(MachineState * ms) {
  ms->config.currentEESpeedRatio = 0.1;
}
END_WORD
REGISTER_WORD(FasterRasterScanningSpeed)

WORD(IRCalibrationSpeed)
virtual void execute(MachineState * ms) {
  ms->config.currentEESpeedRatio = 0.04;
}
END_WORD
REGISTER_WORD(IRCalibrationSpeed)

CONFIG_GETTER_DOUBLE(GetSpeed, ms->config.currentEESpeedRatio)
WORD(SetSpeed)
virtual void execute(MachineState * ms) {
  double v1;
  GET_NUMERIC_ARG(ms, v1);

  double newspeed = min( max(0.0, v1), 1.0);

  cout << "setSpeed got " << v1 << " setting " << newspeed << endl;
  
  ms->config.currentEESpeedRatio = newspeed;
}
END_WORD
REGISTER_WORD(SetSpeed)

WORD(Hover)
virtual void execute(MachineState * ms) {
  ms->config.lastHoverTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
  ms->pushWord("hoverA");
  ms->config.endThisStackCollapse = 1;
  ms->config.shouldIDoIK = 1;
  ms->config.lastHoverRequest = ros::Time::now();
  ms->config.lastEndpointCallbackRequest = ms->config.lastHoverRequest;
}
END_WORD
REGISTER_WORD(Hover)

WORD(HoverA)
virtual void execute(MachineState * ms) {
  if (ms->config.lastEndpointCallbackRequest >= ms->config.lastEndpointCallbackReceived) {
    ms->pushWord("hoverA");
    cout << "hoverA waiting for endpointCallback." << endl;
    ms->config.endThisStackCollapse = 1;
  } else {
    double dx = (ms->config.lastHoverTrueEEPoseEEPose.px - ms->config.trueEEPoseEEPose.px);
    double dy = (ms->config.lastHoverTrueEEPoseEEPose.py - ms->config.trueEEPoseEEPose.py);
    double dz = (ms->config.lastHoverTrueEEPoseEEPose.pz - ms->config.trueEEPoseEEPose.pz);
    double distance = dx*dx + dy*dy + dz*dz;
    
    double qx = (fabs(ms->config.lastHoverTrueEEPoseEEPose.qx) - fabs(ms->config.trueEEPoseEEPose.qx));
    double qy = (fabs(ms->config.lastHoverTrueEEPoseEEPose.qy) - fabs(ms->config.trueEEPoseEEPose.qy));
    double qz = (fabs(ms->config.lastHoverTrueEEPoseEEPose.qz) - fabs(ms->config.trueEEPoseEEPose.qz));
    double qw = (fabs(ms->config.lastHoverTrueEEPoseEEPose.qw) - fabs(ms->config.trueEEPoseEEPose.qw));
    double angleDistance = qx*qx + qy*qy + qz*qz + qw*qw;
  
    if ( ros::Time::now() - ms->config.lastHoverRequest < ros::Duration(ms->config.hoverTimeout) ) {
      if ((distance > ms->config.hoverGoThresh*ms->config.hoverGoThresh) || (angleDistance > ms->config.hoverAngleThresh*ms->config.hoverAngleThresh)) {
	ms->pushWord("hoverA"); 
	ms->config.endThisStackCollapse = 1;
	ms->config.shouldIDoIK = 1;
	cout << "hoverA distance requirement not met, distance angleDistance: " << distance << " " << angleDistance << endl;
	ms->config.lastHoverTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
      } else {
	ms->config.endThisStackCollapse = ms->config.endCollapse;
      }
    } else {
      cout << "Warning: hover timed out, moving on." << endl;
      ms->config.endThisStackCollapse = ms->config.endCollapse;
    }
  }
}
END_WORD
REGISTER_WORD(HoverA)

WORD(SpawnTargetClassAtEndEffector)
CODE(65379) // insert
virtual void execute(MachineState * ms) {
  cout << "SpawnTargetClassAtEndEffector called." << endl;
  if (ms->config.targetClass < 0) {
    cout << "Not spawning because targetClass is " << ms->config.targetClass << endl;
    return;
  }

  if (ms->config.currentRobotMode == PHYSICAL) {
    return;
  } else if (ms->config.currentRobotMode == SIMULATED) {
    Camera * camera  = ms->config.cameras[ms->config.focused_camera];
    BoxMemory box;
    box.bTop.x = camera->vanishingPointReticle.px-ms->config.simulatedObjectHalfWidthPixels;
    box.bTop.y = camera->vanishingPointReticle.py-ms->config.simulatedObjectHalfWidthPixels;
    box.bBot.x = camera->vanishingPointReticle.px+ms->config.simulatedObjectHalfWidthPixels;
    box.bBot.y = camera->vanishingPointReticle.py+ms->config.simulatedObjectHalfWidthPixels;
    box.cameraPose = ms->config.currentEEPose;
    box.top = pixelToGlobalEEPose(ms, box.bTop.x, box.bTop.y, ms->config.trueEEPose.position.z + ms->config.currentTableZ);
    box.bot = pixelToGlobalEEPose(ms, box.bBot.x, box.bBot.y, ms->config.trueEEPose.position.z + ms->config.currentTableZ);
    box.centroid.px = (box.top.px + box.bot.px) * 0.5;
    box.centroid.py = (box.top.py + box.bot.py) * 0.5;
    box.centroid.pz = (box.top.pz + box.bot.pz) * 0.5;
    box.cameraTime = ros::Time::now();
    box.labeledClassIndex = ms->config.targetClass;
  
    mapBox(ms, box);
    vector<BoxMemory> newMemories;
    for (int i = 0; i < ms->config.blueBoxMemories.size(); i++) {
      newMemories.push_back(ms->config.blueBoxMemories[i]);
    }
    newMemories.push_back(box);
    ms->config.blueBoxMemories = newMemories;
  } else {
    assert(0);
  }
}
END_WORD
REGISTER_WORD(SpawnTargetClassAtEndEffector)

WORD(DestroyObjectInEndEffector)
CODE(65535) // delete
virtual void execute(MachineState * ms) {
  if (ms->config.objectInHandLabel >= 0) {
    cout << "destroyObjectInEndEffector: The " << ms->config.classLabels[ms->config.objectInHandLabel] << " in your hand simply vanished." << endl;
    ms->config.objectInHandLabel = -1;
  } else {
    cout << "destroyObjectInEndEffector: There is nothing in your hand so there is nothing to destroy." << ms->config.objectInHandLabel << endl;
  }
}
END_WORD
REGISTER_WORD(DestroyObjectInEndEffector)

WORD(PickObjectUnderEndEffector)
CODE(65365) // page up
virtual void execute(MachineState * ms) {
  std_msgs::Empty msg;
  ms->pickObjectUnderEndEffectorCommandCallback(msg);
}
END_WORD
REGISTER_WORD(PickObjectUnderEndEffector)

WORD(PlaceObjectInEndEffector)
CODE(65366) // page down
virtual void execute(MachineState * ms) {
  std_msgs::Empty msg;
  ms->placeObjectInEndEffectorCommandCallback(msg);
}
END_WORD
REGISTER_WORD(PlaceObjectInEndEffector)

WORD(SetCurrentCornellTableToZero)
virtual void execute(MachineState * ms) {
  cout << "Setting currentCornellTableIndex to " << "0" << " out of " << ms->config.numCornellTables << "." << endl;
  ms->config.currentCornellTableIndex = 0;
}
END_WORD
REGISTER_WORD(SetCurrentCornellTableToZero)

WORD(IncrementCurrentCornellTable)
virtual void execute(MachineState * ms) {
  ms->config.currentCornellTableIndex = (ms->config.currentCornellTableIndex + 1 + ms->config.numCornellTables) % ms->config.numCornellTables;
  cout << "Incrementing currentCornellTableIndex to " << ms->config.currentCornellTableIndex << " out of " << ms->config.numCornellTables << "." << endl;
}
END_WORD
REGISTER_WORD(IncrementCurrentCornellTable)

WORD(DecrementCurrentCornellTable)
virtual void execute(MachineState * ms) {
  ms->config.currentCornellTableIndex = (ms->config.currentCornellTableIndex - 1 + ms->config.numCornellTables) % ms->config.numCornellTables;
  cout << "Decrementing currentCornellTableIndex to " << ms->config.currentCornellTableIndex << " out of " << ms->config.numCornellTables << "." << endl;
}
END_WORD
REGISTER_WORD(DecrementCurrentCornellTable)

WORD(MoveToCurrentCornellTable)
virtual void execute(MachineState * ms) {
  if (ms->config.currentCornellTableIndex >= 0) {
    ms->config.currentEEPose.px = ms->config.cornellTables[ms->config.currentCornellTableIndex].px;
    ms->config.currentEEPose.py = ms->config.cornellTables[ms->config.currentCornellTableIndex].py;
  }
}
END_WORD
REGISTER_WORD(MoveToCurrentCornellTable)

WORD(SpawnTargetMasterSpriteAtEndEffector)
CODE(130915) // shift + insert
virtual void execute(MachineState * ms) {
  cout << "SpawnTargetMasterSpriteAtEndEffector called." << endl;
  if (ms->config.targetMasterSprite < 0) {
    cout << "Not spawning because targetMasterSprite is " << ms->config.targetMasterSprite << endl;
    return;
  }

  for (int s = 0; s < ms->config.masterSprites.size(); s++) {
    cout << "checked " << ms->config.masterSprites[s].name << " as masterSprites[" << s << "] scale " << ms->config.masterSprites[s].scale << " image size " << ms->config.masterSprites[s].image.size() << endl;
  }
  
  if (ms->config.currentRobotMode == PHYSICAL) {
    return;
  } else if (ms->config.currentRobotMode == SIMULATED) {
    Sprite sprite;
    sprite.image = ms->config.masterSprites[ms->config.targetMasterSprite].image.clone();
    sprite.name = ms->config.masterSprites[ms->config.targetMasterSprite].name;
    sprite.scale = ms->config.masterSprites[ms->config.targetMasterSprite].scale;
    sprite.creationTime = ros::Time::now();
    sprite.pose = ms->config.currentEEPose;
    sprite.top = sprite.pose;
    sprite.bot = sprite.pose;

    // rotate image and grow size
    {
      Size sz = sprite.image.size();
      int deltaWidth = 0;
      int deltaHeight = 0;
      if (sz.width < sz.height) {
	deltaWidth = sz.height - sz.width;
	sz.width = sz.height;
      }
      if (sz.width > sz.height) {
	deltaHeight = sz.width - sz.height;
	sz.height = sz.width;
      }

      Quaternionf eeqform(sprite.pose.qw, sprite.pose.qx, sprite.pose.qy, sprite.pose.qz);
      Quaternionf crane2Orient(0, 1, 0, 0);
      Quaternionf rel = eeqform * crane2Orient.inverse();
      Quaternionf ex(0,1,0,0);
      Quaternionf zee(0,0,0,1);
	    
      Quaternionf result = rel * ex * rel.conjugate();
      Quaternionf thumb = rel * zee * rel.conjugate();
      double aY = result.y();
      double aX = result.x();

      double angle = vectorArcTan(ms, aY, aX)*180.0/3.1415926;
      angle = (angle);
      double scale = 1.0;
      Point center = Point(sz.width/2, sz.height/2);

      Mat un_rot_mat = getRotationMatrix2D( center, angle, scale );

      Mat trans_mat = getRotationMatrix2D( center, angle, scale );
      trans_mat.at<double>(0,0) = 1;
      trans_mat.at<double>(1,1) = 1;
      trans_mat.at<double>(0,1) = 0;
      trans_mat.at<double>(1,0) = 0;
      trans_mat.at<double>(0,2) = double(deltaWidth) / 2.0;
      trans_mat.at<double>(1,2) = double(deltaHeight) / 2.0;

      //Mat rotatedSpriteImage;
      //warpAffine(sprite.image, rotatedSpriteImage, un_rot_mat, sz, INTER_LINEAR, BORDER_REPLICATE);
      Mat translatedSpriteImage;
      warpAffine(sprite.image, translatedSpriteImage, trans_mat, sz, INTER_LINEAR, BORDER_REPLICATE);
      Mat rotatedSpriteImage;
      warpAffine(translatedSpriteImage, rotatedSpriteImage, un_rot_mat, sz, INTER_LINEAR, BORDER_REPLICATE);
      sprite.image = rotatedSpriteImage;
    }

    double halfWidthMeters = sprite.image.cols / (2.0 * sprite.scale);
    double halfHeightMeters = sprite.image.rows / (2.0 * sprite.scale);

    sprite.top.px -= halfWidthMeters;
    sprite.top.py -= halfHeightMeters;
    sprite.bot.px += halfWidthMeters;
    sprite.bot.py += halfHeightMeters;

    ms->config.instanceSprites.push_back(sprite);
  } else {
    assert(0);
  }
  cout << "Now instanceSprites.size() is " << ms->config.instanceSprites.size() << "." << endl;
}
END_WORD
REGISTER_WORD(SpawnTargetMasterSpriteAtEndEffector)

WORD(DestroyTargetInstanceSprite)
CODE(131071) // shift + delete
virtual void execute(MachineState * ms) {
  cout << "DestroyTargetInstanceSprite called." << endl;
  if ((ms->config.targetInstanceSprite < 0) ||
      (ms->config.targetInstanceSprite >= ms->config.instanceSprites.size())) {
    cout << "Not destoying because targetInstanceSprite is " << ms->config.targetInstanceSprite << " out of " << ms->config.instanceSprites.size() << endl;
    return;
  }
  
  if (ms->config.currentRobotMode == PHYSICAL) {
    return;
  } else if (ms->config.currentRobotMode == SIMULATED) {
    vector<Sprite> newInstanceSprites;
    for (int s = 0; s < ms->config.instanceSprites.size(); s++) {
      if (s != ms->config.targetInstanceSprite) {
	newInstanceSprites.push_back(ms->config.instanceSprites[s]);
      }
    }
    ms->config.instanceSprites = newInstanceSprites;
  } else {
    assert(0);
  }
  cout << "Now instanceSprites.size() is " << ms->config.instanceSprites.size() << "." << endl;
}
END_WORD
REGISTER_WORD(DestroyTargetInstanceSprite)

WORD(IncrementTargetInstanceSprite)
CODE(130901) // shift + page up
virtual void execute(MachineState * ms) {
  if (ms->config.currentRobotMode == PHYSICAL) {
    return;
  } else if (ms->config.currentRobotMode == SIMULATED) {
    int base = ms->config.instanceSprites.size();
    ms->config.targetInstanceSprite = (ms->config.targetInstanceSprite + 1 + base) % max(base, 1);
    cout << "Incrementing targetInstanceSprite to " << ms->config.targetInstanceSprite << " out of " << base << "." << endl;
  } else {
    assert(0);
  }
}
END_WORD
REGISTER_WORD(IncrementTargetInstanceSprite)

WORD(DecrementTargetInstanceSprite)
CODE(130902) // shift + page down
virtual void execute(MachineState * ms) {
  if (ms->config.currentRobotMode == PHYSICAL) {
    return;
  } else if (ms->config.currentRobotMode == SIMULATED) {
    int base = ms->config.instanceSprites.size();
    ms->config.targetInstanceSprite = (ms->config.targetInstanceSprite - 1 + base) % max(base, 1);
    cout << "Decrementing targetInstanceSprite to " << ms->config.targetInstanceSprite << " out of " << base << "." << endl;
  } else {
    assert(0);
  }
}
END_WORD
REGISTER_WORD(DecrementTargetInstanceSprite)

WORD(IncrementTargetMasterSprite)
CODE(130896) // shift + home 
virtual void execute(MachineState * ms) {
  if (ms->config.currentRobotMode == PHYSICAL) {
    return;
  } else if (ms->config.currentRobotMode == SIMULATED) {
    int base = ms->config.masterSprites.size();
    ms->config.targetMasterSprite = (ms->config.targetMasterSprite + 1 + base) % max(base, 1);
    cout << "Incrementing targetMasterSprite to " << ms->config.targetMasterSprite << " out of " << base << "." << endl;
  } else {
    assert(0);
  }
}
END_WORD
REGISTER_WORD(IncrementTargetMasterSprite)

WORD(DecrementTargetMasterSprite)
CODE(130903) // shift + end 
virtual void execute(MachineState * ms) {
  if (ms->config.currentRobotMode == PHYSICAL) {
    return;
  } else if (ms->config.currentRobotMode == SIMULATED) {
    int base = ms->config.masterSprites.size();
    ms->config.targetMasterSprite = (ms->config.targetMasterSprite - 1 + base) % max(base, 1);
    cout << "Decrementing targetMasterSprite to " << ms->config.targetMasterSprite << " out of " << base << "." << endl;
  } else {
    assert(0);
  }
}
END_WORD
REGISTER_WORD(DecrementTargetMasterSprite)

CONFIG_GETTER_DOUBLE(ArmedThreshold, ms->config.armedThreshold)
CONFIG_SETTER_DOUBLE(SetArmedThreshold, ms->config.armedThreshold)
CONFIG_GETTER_DOUBLE(MovingThreshold, ms->config.movingThreshold)
CONFIG_SETTER_DOUBLE(SetMovingThreshold, ms->config.movingThreshold)
CONFIG_GETTER_DOUBLE(HoverThreshold, ms->config.hoverThreshold)
CONFIG_SETTER_DOUBLE(SetHoverThreshold, ms->config.hoverThreshold)

WORD(ComeToStop)
virtual void execute(MachineState * ms) {
  //ms->config.currentEEPose = ms->config.trueEEPoseEEPose;
  ms->pushWord("comeToStopA");
  ms->config.comeToStopStart = ros::Time::now();
  cout << "Waiting to come to a stop..." << endl;
}
END_WORD
REGISTER_WORD(ComeToStop)

WORD(ComeToStopA)
virtual void execute(MachineState * ms) {
  if ( (ms->config.currentMovementState == STOPPED) ||
       (ms->config.currentMovementState == BLOCKED) ) {
    // do nothing
    cout << "Came to a stop, moving on." << endl;
  } else {
    ros::Duration timeSinceCTS = ros::Time::now() - ms->config.comeToStopStart;
    if (timeSinceCTS.toSec() < ms->config.comeToStopTimeout) {
      ms->pushWord("comeToStopA");
    } else {
      ROS_WARN_STREAM("_____*____*________");
      ROS_ERROR_STREAM("comeToStop timeout reached, moving on.");
      ROS_WARN_STREAM("_____*____*________");

      // waitUntilCurrentPosition will time out, make sure that there will
      //  be no cycles introduced
      ms->config.currentEEPose.pz = ms->config.trueEEPose.position.z + 0.001;
      cout << "  backing up just a little to dislodge from failed hover, then waiting." << endl;
      ms->pushWord("waitUntilAtCurrentPosition"); 
    }
    ms->config.endThisStackCollapse = 1;
  }
}
END_WORD
REGISTER_WORD(ComeToStopA)

WORD(ComeToHover)
virtual void execute(MachineState * ms) {
  //ms->config.currentEEPose = ms->config.trueEEPoseEEPose;
  ms->pushWord("comeToHoverA");
  ms->config.comeToHoverStart = ros::Time::now();
  cout << "Waiting to come to a hover..." << endl;
}
END_WORD
REGISTER_WORD(ComeToHover)

WORD(ComeToHoverA)
virtual void execute(MachineState * ms) {
  if ( (ms->config.currentMovementState == HOVERING) ) {
    // do nothing
    cout << "Came to a hover, moving on." << endl;
  } else {
    ros::Duration timeSinceCTH = ros::Time::now() - ms->config.comeToHoverStart;
    if (timeSinceCTH.toSec() < ms->config.comeToHoverTimeout) {
      ms->pushWord("comeToHoverA");
    } else {
      ROS_WARN_STREAM("_____*____*________");
      ROS_ERROR_STREAM("comeToHover timeout reached, moving on.");
      ROS_WARN_STREAM("_____*____*________");
    }
    ms->config.endThisStackCollapse = 1;
  }
}
END_WORD
REGISTER_WORD(ComeToHoverA)

WORD(WaitForTugThenOpenGripper)
virtual void execute(MachineState * ms) {
  ms->pushWord("waitForTugThenOpenGripperA");
  ms->pushWord("waitUntilEndpointCallbackReceived");
  //ms->pushWord("comeToStop");
  //ms->pushWord("comeToHover");
  //ms->pushWord("waitUntilAtCurrentPosition");
  ms->config.waitForTugStart = ros::Time::now();
  cout << "Waiting to feel a tug... " << ARMED << " " << ms->config.currentMovementState << endl;
}
END_WORD
REGISTER_WORD(WaitForTugThenOpenGripper)

WORD(WaitForTugThenOpenGripperA)
virtual void execute(MachineState * ms) {
  ms->config.endThisStackCollapse = 1;
  if (0) { // position based
    if ( ( ms->config.currentMovementState == MOVING ) ||
	 ( !ms->config.gripperGripping ) ) {
      if ( !ms->config.gripperGripping ) {
	cout << "There is nothing in the gripper so we should move on..." << endl;
      }
      if (ms->config.currentMovementState == MOVING) {
	cout << "Felt a tug, opening gripper." << endl;
      }
      ms->pushWord("openGripper");
    } else {
      ms->config.currentMovementState = ARMED;
      ros::Duration timeSinceWFT = ros::Time::now() - ms->config.waitForTugStart;
      if (timeSinceWFT.toSec() < ms->config.waitForTugTimeout) {
	ms->pushWord("waitForTugThenOpenGripperA");
      } else {
	ROS_WARN_STREAM("_____*____*________");
	ROS_ERROR_STREAM("waitForTugThenOpenGripper timeout reached, moving on.");
	ROS_WARN_STREAM("_____*____*________");
      }
    }
  } else if (0) { // wrench based
    double wrenchNorm = sqrt( eePose::squareDistance(eePose::zero(), ms->config.trueEEWrench) );
    bool wrenchOverThresh = ( wrenchNorm > ms->config.wrenchThresh );

    if ( wrenchOverThresh || 
	 ( !ms->config.gripperGripping ) ) {
      if ( !ms->config.gripperGripping ) {
	cout << "There is nothing in the gripper so we should move on..." << endl;
      }
      if ( wrenchOverThresh ) {
	cout << "Felt a tug, opening gripper; wrenchNorm wrenchThresh: " << wrenchNorm << " " << ms->config.wrenchThresh << " " << endl;
      }
      ms->pushWord("openGripper");
    } else {
      ms->config.currentMovementState = ARMED;
      ros::Duration timeSinceWFT = ros::Time::now() - ms->config.waitForTugStart;
      if (timeSinceWFT.toSec() < ms->config.waitForTugTimeout) {
	ms->pushWord("waitForTugThenOpenGripperA");
      } else {
	ROS_WARN_STREAM("_____*____*________");
	ROS_ERROR_STREAM("waitForTugThenOpenGripper timeout reached, moving on.");
	ROS_WARN_STREAM("_____*____*________");
      }
    }
  } else {
    // triple trigger, position enabled
    double wrenchNorm = sqrt( eePose::squareDistance(eePose::zero(), ms->config.trueEEWrench) );
    bool wrenchOverThresh = ( wrenchNorm > ms->config.wrenchThresh );


    bool effortOverThresh = false;
    {
      double totalDiff = 0.0;
      for (int i = 0; i < NUM_JOINTS; i++) {
	double thisDiff = (ms->config.target_joint_actual_effort[i] - ms->config.last_joint_actual_effort[i]);
	//cout << ms->config.target_joint_actual_effort[i] << " " << ms->config.last_joint_actual_effort[i] << " " << thisDiff << " ";
	totalDiff = totalDiff + (thisDiff * thisDiff);
      }

      //cout << endl << "  totalDiff: " << totalDiff << "   actual_effort_thresh: " << ms->config.actual_effort_thresh << endl;
      effortOverThresh = (totalDiff > ms->config.actual_effort_thresh);
    }

    bool positionTrigger = ( ms->config.currentMovementState == MOVING );
    //bool positionTrigger = false;

    if ( wrenchOverThresh || effortOverThresh || positionTrigger ||
	 ( !ms->config.gripperGripping ) ) {
      if ( !ms->config.gripperGripping ) {
	cout << "There is nothing in the gripper so we should move on..." << endl;
      }
      if ( wrenchOverThresh ) {
	cout << "Felt a tug, opening gripper; wrenchNorm wrenchThresh: " << wrenchNorm << " " << ms->config.wrenchThresh << " " << endl;
      }
      cout << "position trigger disabled --> wrenchOverThresh, effortOverThresh, positionTrigger: " << wrenchOverThresh << " " << effortOverThresh << " " << positionTrigger << endl;
      ms->pushWord("openGripper");
    } else {
      ms->config.currentMovementState = ARMED;
      ros::Duration timeSinceWFT = ros::Time::now() - ms->config.waitForTugStart;
      if (timeSinceWFT.toSec() < ms->config.waitForTugTimeout) {
	ms->pushWord("waitForTugThenOpenGripperA");
      } else {
	ROS_WARN_STREAM("_____*____*________");
	ROS_ERROR_STREAM("waitForTugThenOpenGripper timeout reached, moving on.");
	ROS_WARN_STREAM("_____*____*________");
      }
    }
  }
}
END_WORD
REGISTER_WORD(WaitForTugThenOpenGripperA)

WORD(Idler)
virtual void execute(MachineState * ms) {
  if (ms->config.currentIdleMode == EMPTY) {
    // empty
  } else if (ms->config.currentIdleMode == STOPCLEAR) {
    ms->pushWord("clearStack"); 
  } else if (ms->config.currentIdleMode == PATROL) {
    ms->pushWord("clearStackIntoMappingPatrol"); 
  } else if (ms->config.currentIdleMode == CRANE) {
    ms->pushWord("idler"); 
    ms->pushWord("publishRecognizedObjectArrayFromBlueBoxMemory");
    ms->pushWord("assumeCrane1"); 
  } else if (ms->config.currentIdleMode == SHRUG) {
    ms->pushWord("publishRecognizedObjectArrayFromBlueBoxMemory");
    ms->pushWord("assumeShrugPose"); 
  } else {
    assert(0);
  }
  ms->pushWord("setPatrolStateToIdling");
}
END_WORD
REGISTER_WORD(Idler)

WORD(SetMovementStateToMoving)
virtual void execute(MachineState * ms) {
  ms->config.currentMovementState = MOVING;
  ms->config.lastTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
  ms->config.lastMovementStateSet = ros::Time::now();
}
END_WORD
REGISTER_WORD(SetMovementStateToMoving)

WORD(AssumeCrane1)
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose = ms->config.crane1;
  ms->pushWord("waitUntilAtCurrentPosition");
}
END_WORD
REGISTER_WORD(AssumeCrane1)

CONFIG_GETTER_POSE(BeeHome, ms->config.beeHome)
CONFIG_SETTER_POSE(SetBeeHome, ms->config.beeHome)

WORD(AssumeBeeHome)
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose = ms->config.beeHome;
  // XXX consider changing the standard
  //ms->pushWord("waitUntilAtCurrentPosition");
}
END_WORD
REGISTER_WORD(AssumeBeeHome)

WORD(AssumeShrugPose)
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose = ms->config.shrugPose;
  ms->pushWord("waitUntilAtCurrentPosition");
}
END_WORD
REGISTER_WORD(AssumeShrugPose)

WORD(AssumeHandingPose)
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose = ms->config.handingPose;
  ms->pushWord("waitUntilAtCurrentPosition");
}
END_WORD
REGISTER_WORD(AssumeHandingPose)

WORD(SetPatrolStateToIdling)
virtual void execute(MachineState * ms) {
  ms->config.currentPatrolState = IDLING;
}
END_WORD
REGISTER_WORD(SetPatrolStateToIdling)

WORD(SetPatrolStateToPatrolling)
virtual void execute(MachineState * ms) {
  ms->config.currentPatrolState = PATROLLING;
}
END_WORD
REGISTER_WORD(SetPatrolStateToPatrolling)

WORD(SetPatrolStateToPicking)
virtual void execute(MachineState * ms) {
  ms->config.currentPatrolState = PICKING;
}
END_WORD
REGISTER_WORD(SetPatrolStateToPicking)

WORD(SetPatrolStateToPlacing)
virtual void execute(MachineState * ms) {
  ms->config.currentPatrolState = PLACING;
}
END_WORD
REGISTER_WORD(SetPatrolStateToPlacing)

WORD(SetPatrolStateToHanding)
virtual void execute(MachineState * ms) {
  ms->config.currentPatrolState = HANDING;
}
END_WORD
REGISTER_WORD(SetPatrolStateToHanding)

WORD(SetPatrolModeToLoop)
virtual void execute(MachineState * ms) {
  ms->config.currentPatrolMode = LOOP;
}
END_WORD
REGISTER_WORD(SetPatrolModeToLoop)

WORD(SetPatrolModeToOnce)
virtual void execute(MachineState * ms) {
  ms->config.currentPatrolMode = ONCE;
}
END_WORD
REGISTER_WORD(SetPatrolModeToOnce)

WORD(SetIdleModeToCrane)
virtual void execute(MachineState * ms) {
  ms->config.currentIdleMode = CRANE;
}
END_WORD
REGISTER_WORD(SetIdleModeToCrane)

WORD(SetIdleModeToShrug)
virtual void execute(MachineState * ms) {
  ms->config.currentIdleMode = SHRUG;
}
END_WORD
REGISTER_WORD(SetIdleModeToShrug)

WORD(SetIdleModeToEmpty)
virtual void execute(MachineState * ms) {
  ms->config.currentIdleMode = EMPTY;
}
END_WORD
REGISTER_WORD(SetIdleModeToEmpty)

WORD(SetIdleModeToStopClear)
virtual void execute(MachineState * ms) {
  ms->config.currentIdleMode = STOPCLEAR;
}
END_WORD
REGISTER_WORD(SetIdleModeToStopClear)

WORD(SetIdleModeToPatrol)
virtual void execute(MachineState * ms) {
  ms->config.currentIdleMode = PATROL;
}
END_WORD
REGISTER_WORD(SetIdleModeToPatrol)

WORD(SetCurrentPoseToTruePose)
virtual void execute(MachineState * ms) {
  cout << "Setting current position to true position." << endl;
  ms->config.endThisStackCollapse = 1;
  ms->config.currentEEPose = ms->config.trueEEPoseEEPose;
}
END_WORD
REGISTER_WORD(SetCurrentPoseToTruePose)

WORD(DislodgeEndEffectorFromTable)
virtual void execute(MachineState * ms) {
  cout << "Dislodging end effector from table 1cm..." << endl;
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("zUp"); 
}
END_WORD
REGISTER_WORD(DislodgeEndEffectorFromTable)
	
WORD(MoveToEEPose)
virtual void execute(MachineState * ms) {
  // get 7 strings, make them into doubles, move there
  // order px py pz qx qy qz qw
  double vals[7]; 

  for (int i = 6; i >= 0; i--) { 
    GET_NUMERIC_ARG(ms, vals[i]);
  }

  // make them actually into an eepose
  _eePose pose = eePose(vals[0], vals[1], vals[2],
                        vals[3], vals[4], vals[5], vals[6]);
  ms->config.currentEEPose = pose; 
}
END_WORD
REGISTER_WORD(MoveToEEPose)

WORD(CreateEEPose)
virtual void execute(MachineState * ms) {
  double vals[7]; 

  for (int i = 6; i >= 0; i--) { 
    GET_NUMERIC_ARG(ms, vals[i]);
  }

  // make them actually into an eepose
  _eePose pose = eePose(vals[0], vals[1], vals[2],
                        vals[3], vals[4], vals[5], vals[6]);

  ms->pushWord(std::make_shared<EePoseWord>(pose));
}
END_WORD
REGISTER_WORD(CreateEEPose)

WORD(CreateArmPose)
virtual void execute(MachineState * ms) {
  double vals[7]; 
  for (int i = 6; i >= 0; i--) { 
    GET_NUMERIC_ARG(ms, vals[i]);
  }

  _armPose pose(vals[0],vals[1],vals[2],vals[3],vals[4],vals[5],vals[6]);
  ms->pushWord(std::make_shared<ArmPoseWord>(pose));
}
END_WORD
REGISTER_WORD(CreateArmPose)

WORD(WaitForSeconds)
virtual void execute(MachineState * ms) {
  //cout << "waitForSeconds: ";
  double secondsToWait = 0;
  GET_NUMERIC_ARG(ms, secondsToWait);

  ms->config.waitForSecondsTarget = ros::Time::now() + ros::Duration(secondsToWait);
  //cout << "waiting " << secondsToWait << " seconds until " << ms->config.waitForSecondsTarget << endl;
  ms->pushWord("waitForSecondsA");
}
END_WORD
REGISTER_WORD(WaitForSeconds)

WORD(WaitForSecondsA)
virtual void execute(MachineState * ms) {
  //cout << "waitForSecondsA: ";
  ros::Time thisNow = ros::Time::now();
  if (thisNow.toSec() > ms->config.waitForSecondsTarget.toSec()) {
    //cout << "PASSED at time, target, delta: " << thisNow.toSec() << " " << ms->config.waitForSecondsTarget.toSec() << " " << thisNow.toSec() - ms->config.waitForSecondsTarget.toSec() << endl;
  } else {
    //cout << "HELD at time, target, delta: " << thisNow.toSec() << " " << ms->config.waitForSecondsTarget.toSec() << " " << thisNow.toSec() - ms->config.waitForSecondsTarget.toSec() << endl;
    ms->pushWord("waitForSecondsA");
    ms->config.endThisStackCollapse = 1;
  }
}
END_WORD
REGISTER_WORD(WaitForSecondsA)

WORD(SpinForSeconds)
virtual void execute(MachineState * ms) {
  //cout << "spinForSeconds: ";
  //cout << "spinning " << secondsToSpin << " seconds until " << ms->config.spinForSecondsTarget << endl;
  ms->pushWord("spinForSecondsA");
  ms->pushWord("spinForSecondsInit");
}
END_WORD
REGISTER_WORD(SpinForSeconds)

WORD(SpinForSecondsInit)
virtual void execute(MachineState * ms) {
  double secondsToSpin = 0;
  GET_NUMERIC_ARG(ms, secondsToSpin);
  ros::Time spinForSecondsTarget = ros::Time::now() + ros::Duration(secondsToSpin);
  ms->pushData(make_shared<DoubleWord>(spinForSecondsTarget.toSec())); 
}
END_WORD
REGISTER_WORD(SpinForSecondsInit)

WORD(SpinForSecondsA)
virtual void execute(MachineState * ms) {
  //cout << "spinForSecondsA: ";
  double spinForSecondsTargetDouble = 0;
  GET_NUMERIC_ARG(ms, spinForSecondsTargetDouble);
  ros::Time thisNow = ros::Time::now();
  if (thisNow.toSec() > spinForSecondsTargetDouble) {
    //cout << "PASSED at time, target, delta: " << thisNow.toSec() << " " << ms->config.spinForSecondsTarget.toSec() << " " << thisNow.toSec() - ms->config.spinForSecondsTarget.toSec() << endl;
  } else {
    //cout << "HELD at time, target, delta: " << thisNow.toSec() << " " << ms->config.spinForSecondsTarget.toSec() << " " << thisNow.toSec() - ms->config.spinForSecondsTarget.toSec() << endl;
    ms->pushWord("spinForSecondsA");
    ms->pushData(make_shared<DoubleWord>(spinForSecondsTargetDouble)); 
    // does not end stack collapse
  }
}
END_WORD
REGISTER_WORD(SpinForSecondsA)
		
WORD(CurrentPoseToWord)
virtual void execute(MachineState * ms) {
  ms->pushWord(std::make_shared<EePoseWord>(ms->config.currentEEPose));
}
END_WORD
REGISTER_WORD(CurrentPoseToWord)

WORD(MoveEeToPoseWord)
virtual void execute(MachineState * ms) {
  eePose destPose;
  GET_ARG(ms,EePoseWord,destPose);
  ms->config.currentEEPose = destPose;
}
END_WORD
REGISTER_WORD(MoveEeToPoseWord)

WORD(DiagnosticRelativePose)
virtual void execute(MachineState * ms) {
  cout << "diagnosticRelativePose: Applying eepReg2 relative to eepReg1 to currentEEPose." << endl;
  eePose oldCurrent = ms->config.currentEEPose;
  eePose reg2RelReg1 = ms->config.eepReg2.getPoseRelativeTo(ms->config.eepReg1);
  ms->config.currentEEPose = reg2RelReg1.applyAsRelativePoseTo(oldCurrent);
}
END_WORD
REGISTER_WORD(DiagnosticRelativePose)




WORD(MeasureTimeStart)
virtual void execute(MachineState * ms) {
  ros::Time tNow = ros::Time::now();
  ms->config.measureTimeTarget = tNow;
  ms->config.measureTimeStart = tNow;
  ms->config.measureTimePeriod = 1.0;
}
END_WORD
REGISTER_WORD(MeasureTimeStart)

WORD(MeasureTimeEnd)
virtual void execute(MachineState * ms) {
  // this word is a marker for the end of a sequence
}
END_WORD
REGISTER_WORD(MeasureTimeEnd)

WORD(MeasureTimeSetPeriod)
virtual void execute(MachineState * ms) {
  double periodin = 0;
  GET_NUMERIC_ARG(ms, periodin);
  ms->config.measureTimePeriod = periodin;
}
END_WORD
REGISTER_WORD(MeasureTimeSetPeriod)

WORD(MeasureTime)
virtual void execute(MachineState * ms) {
  ms->pushWord("measureTimeA");
  ms->pushWord("measureTimeInit");
}
END_WORD
REGISTER_WORD(MeasureTime)

WORD(MeasureTimeSinceStart)
virtual void execute(MachineState * ms) {
  ms->pushWord("measureTimeA");
  ms->pushWord("measureTimeInitSinceStart");
}
END_WORD
REGISTER_WORD(MeasureTimeSinceStart)

WORD(MeasureTimeInit)
virtual void execute(MachineState * ms) {
  double secondsToMeasure = 0;
  GET_NUMERIC_ARG(ms, secondsToMeasure);
  ms->config.measureTimeTarget = ms->config.measureTimeTarget + ros::Duration(secondsToMeasure * ms->config.measureTimePeriod);
}
END_WORD
REGISTER_WORD(MeasureTimeInit)

WORD(MeasureTimeInitSinceStart)
virtual void execute(MachineState * ms) {
  double secondsToMeasure = 0;
  GET_NUMERIC_ARG(ms, secondsToMeasure);
  ms->config.measureTimeTarget = ms->config.measureTimeStart + ros::Duration(secondsToMeasure * ms->config.measureTimePeriod);
}
END_WORD
REGISTER_WORD(MeasureTimeInitSinceStart)

WORD(MeasureTimeA)
virtual void execute(MachineState * ms) {
  //cout << "measureTimeA: ";
  ros::Time thisNow = ros::Time::now();
  if (thisNow.toSec() > ms->config.measureTimeTarget.toSec()) {
    //cout << "PASSED at time, target, delta: " << thisNow.toSec() << " " << ms->config.measureTimeTarget.toSec() << " " << thisNow.toSec() - ms->config.measureTimeTarget.toSec() << endl;
  } else {
    //cout << "HELD at time, target, delta: " << thisNow.toSec() << " " << ms->config.measureTimeTarget.toSec() << " " << thisNow.toSec() - ms->config.measureTimeTarget.toSec() << endl;
    ms->pushWord("measureTimeA");
    // does not end stack collapse
  }
}
END_WORD
REGISTER_WORD(MeasureTimeA)

WORD(AboutFace)
virtual string description() {
  return "Rotate the gripper in oZ by 180 degrees.";
}
virtual void execute(MachineState * ms) {
  ms->config.currentEEDeltaRPY.pz = ( M_PI );
  endEffectorAngularUpdate( &ms->config.currentEEPose, &ms->config.currentEEDeltaRPY );
}
END_WORD
REGISTER_WORD(AboutFace)

WORD(QuarterTurn)
virtual void execute(MachineState * ms) {
  ms->config.currentEEDeltaRPY.pz = ( M_PI/2.0 );
  endEffectorAngularUpdate( &ms->config.currentEEPose, &ms->config.currentEEDeltaRPY );
}
END_WORD
REGISTER_WORD(QuarterTurn)

WORD(EighthTurn)
virtual void execute(MachineState * ms) {
  ms->config.currentEEDeltaRPY.pz = ( M_PI/4.0 );
  endEffectorAngularUpdate( &ms->config.currentEEPose, &ms->config.currentEEDeltaRPY );
}
END_WORD
REGISTER_WORD(EighthTurn)

WORD(TouchDown)
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("currentPose  0 currentTableZ - pickFlushFactor +  setEEPosePZ assumePose");
}
END_WORD
REGISTER_WORD(TouchDown)

/*
WORD(PressDown)
virtual void execute(MachineState * ms) {
  ms->evaluateProgram("currentPose  0 currentTableZ - pickFlushFactor + 0.045 + setEEPosePZ assumePose pressUntilEffortInit 0.04 setSpeed pressUntilEffortCombo");
}
END_WORD
REGISTER_WORD(PressDown)
*/

WORD(SetControlModeEePosition)
virtual void execute(MachineState * ms) {
  ms->config.currentControlMode = EEPOSITION;
}
END_WORD
REGISTER_WORD(SetControlModeEePosition)

WORD(SetControlModeAngles)
virtual void execute(MachineState * ms) {
  ms->config.currentControlMode = ANGLES;
}
END_WORD
REGISTER_WORD(SetControlModeAngles)



WORD(FollowPath)
virtual void execute(MachineState * ms) {
  eePose destPose;
  CONSUME_EEPOSE(destPose,ms);

  ms->pushWord("followPath");

  ms->pushWord("waitUntilAtCurrentPositionCollapse");
  ms->pushWord("moveEeToPoseWord");
  ms->pushWord(std::make_shared<EePoseWord>(destPose));
}
END_WORD
REGISTER_WORD(FollowPath)

WORD(InterpolatePath)
virtual void execute(MachineState * ms) {
  vector<eePose> pathPoints;
  GET_WORD_ARG_VALUE_LIST(ms, EePoseWord, pathPoints);

  double position_tolerance;
  GET_NUMERIC_ARG(ms, position_tolerance);

  double angle_tolerance;
  GET_NUMERIC_ARG(ms, angle_tolerance);

  int numPathPoints = pathPoints.size();
  cout << "interpolatePath: ok, interpolating a path with " << numPathPoints << " points to have at most " << 
    position_tolerance << " meters and at most " << angle_tolerance << " radians between points." << endl;
 
  double p_undershoot = 0.99;

  vector<eePose> newPathPoints;
  for (int i = 0; i < numPathPoints-1; i++) {
    newPathPoints.push_back(pathPoints[i]);

    // enforce tolerance
    double pos_d = eePose::distance( pathPoints[i], pathPoints[i+1]);
    double pos_q = eePose::distanceQ(pathPoints[i], pathPoints[i+1]);

    double minFraction = std::min( position_tolerance / pos_d, angle_tolerance / pos_q ); 
    double interpStep = minFraction * p_undershoot;

    for (double mu = interpStep; mu < 1.0; mu += interpStep) {
      newPathPoints.push_back(pathPoints[i].getInterpolation(pathPoints[i+1], mu));
    }
  }
  newPathPoints.push_back((pathPoints[numPathPoints-1]));

  for (int i = 0; i < newPathPoints.size(); i++) {
    ms->pushWord(std::make_shared<EePoseWord>(newPathPoints[i]));
  }
}
END_WORD
REGISTER_WORD(InterpolatePath)

WORD(ReversePath)
virtual void execute(MachineState * ms) {
  vector<eePose> pathPoints;
  GET_WORD_ARG_VALUE_LIST(ms, EePoseWord, pathPoints);

  int numPathPoints = pathPoints.size();
  cout << "reversePath: ok, reversing a path with " << numPathPoints << " points." << endl;

  for (int i = 0; i < numPathPoints; i++) {
    ms->pushWord(std::make_shared<EePoseWord>(pathPoints[numPathPoints-1-i]));
  }
}
END_WORD
REGISTER_WORD(ReversePath)

WORD(ReverseCompound)
virtual void execute(MachineState * ms) {
// XXX 
  shared_ptr<CompoundWord> toReverse;
  GET_WORD_ARG(ms, CompoundWord, toReverse);

  shared_ptr<CompoundWord> reversedWord = std::make_shared<CompoundWord>();

  std::shared_ptr<Word> word = toReverse->popWord();
  while (word != NULL) {
    reversedWord->pushWord(word);
    word = toReverse->popWord();
  }
  
  ms->pushData(reversedWord);
}
END_WORD
REGISTER_WORD(ReverseCompound)

WORD(TransformPath)
virtual void execute(MachineState * ms) {
  // maintains order
  // use:
  // ( eePoseBase eePoseApplyRelativePoseTo ) endArgs toApply1 toApply2 ... transformPath
  // ( eePoseToApply swap eePoseApplyRelativePoseTo ) endArgs eePoseBase1 eePoseBase2 ... transformPath
  vector<eePose> pathPoints;
  GET_WORD_ARG_VALUE_LIST(ms, EePoseWord, pathPoints);

  shared_ptr<CompoundWord> transformation;
  GET_WORD_ARG(ms, CompoundWord, transformation);

  int numPathPoints = pathPoints.size();
  cout << "transformPath: ok, transforming a path with " << numPathPoints << " points." << endl;

  for (int i = 0; i < numPathPoints; i++) {
    ms->pushWord(transformation);
    ms->pushWord(std::make_shared<EePoseWord>(pathPoints[i]));
  }
}
END_WORD
REGISTER_WORD(TransformPath)


WORD(WaitUntilOnSideOfPlane)
virtual void execute(MachineState * ms) {
  eePose normalIn;
  GET_ARG(ms, EePoseWord, normalIn);
  normalIn = normalIn.getNormalized();

  eePose anchorIn;
  GET_ARG(ms, EePoseWord, anchorIn);

  ms->config.currentMovementState = MOVING;
  ms->config.lastTrueEEPoseEEPose = ms->config.trueEEPoseEEPose;
  ms->config.lastMovementStateSet = ros::Time::now();

  ms->config.waitUntilAtCurrentPositionCounter = 0;
  ms->config.waitUntilAtCurrentPositionStart = ros::Time::now();

  eePose anchorDiff = ms->config.trueEEPoseEEPose.minusP(anchorIn);
  double distFromPlane = eePose::dotP(anchorDiff, normalIn);

  if (distFromPlane > ms->config.w1GoThresh) {
    ms->pushWord(make_shared<EePoseWord>(anchorIn));
    ms->pushWord(make_shared<EePoseWord>(normalIn));
    ms->pushWord("waitUntilOnSideOfPlaneB"); 
    ms->config.endThisStackCollapse = 1;
    ms->config.shouldIDoIK = 1;
  } else {
    // try not collapsing if you are where you want to be
    ms->config.endThisStackCollapse = 1;
  }

  if (ms->config.currentControlMode == VELOCITY) {
  } else if (ms->config.currentControlMode == EEPOSITION) {
  } else if (ms->config.currentControlMode == ANGLES) {
    ms->pushWord("setCurrentPoseFromJoints"); 
  } else {
    assert(0);
  }
}
END_WORD
REGISTER_WORD(WaitUntilOnSideOfPlane)

WORD(WaitUntilOnSideOfPlaneB)
virtual void execute(MachineState * ms) {
  eePose normalIn;
  GET_ARG(ms, EePoseWord, normalIn);

  eePose anchorIn;
  GET_ARG(ms, EePoseWord, anchorIn);

  if ( ros::Time::now().toSec() - ms->config.waitUntilAtCurrentPositionStart.toSec() < ms->config.waitUntilAtCurrentPositionTimeout )
  {
    if ( (ms->config.currentMovementState == STOPPED) ||
	 (ms->config.currentMovementState == BLOCKED) ) {

      if (ms->config.currentMovementState == STOPPED) {
	cout << "Warning: waitUntilOnSideOfPlane ms->config.currentMovementState = STOPPED, moving on." << endl;
	ms->config.endThisStackCollapse = ms->config.endCollapse;
      }
      if (ms->config.currentMovementState == BLOCKED) {
	cout << "Warning: waitUntilOnSideOfPlane ms->config.currentMovementState = BLOCKED, moving on." << endl;
	ms->config.endThisStackCollapse = ms->config.endCollapse;
      }
      
      if (ms->config.currentWaitMode == WAIT_KEEP_ON) {
	cout << "waitUntilOnSideOfPlaneB: currentWaitMode WAIT_KEEP_ON, so doing nothing...";
      } else if (ms->config.currentWaitMode == WAIT_BACK_UP) {
	cout << "waitUntilOnSideOfPlaneB: currentWaitMode WAIT_BACK_UP, so...";
	cout << " doing nothing!" << endl;
	//ms->config.currentEEPose.pz = ms->config.trueEEPose.position.z + 0.001;
	//cout << "  backing up just a little to dislodge, then waiting again." << endl;
      } else {
	assert(0);
      }

      ms->pushWord(make_shared<EePoseWord>(anchorIn));
      ms->pushWord(make_shared<EePoseWord>(normalIn));
      ms->pushWord("waitUntilOnSideOfPlane"); 
      return;
    }

    ms->config.waitUntilAtCurrentPositionCounter++;

    eePose anchorDiff = ms->config.trueEEPoseEEPose.minusP(anchorIn);
    double distFromPlane = eePose::dotP(anchorDiff, normalIn);

    if (distFromPlane > ms->config.w1GoThresh) {
      ms->pushWord(make_shared<EePoseWord>(anchorIn));
      ms->pushWord(make_shared<EePoseWord>(normalIn));
      ms->pushWord("waitUntilOnSideOfPlaneB"); 
      ms->config.endThisStackCollapse = 1;
      ms->config.shouldIDoIK = 1;
    } else {
      ms->config.endThisStackCollapse = ms->config.endCollapse;
    }
  } else {
    cout << "Warning: waitUntilOnSideOfPlane timed out, moving on." << endl;
    ms->config.endThisStackCollapse = 1;
  }
}
END_WORD
REGISTER_WORD(WaitUntilOnSideOfPlaneB)

WORD(InterlaceTop)
virtual void execute(MachineState * ms) {
  // XXX perhaps this is in fact "map"
  //  at any rate, a generalized TransformPath in the style of ReverseCompound
  vector< shared_ptr<Word> > theseWords;
  GET_WORD_ARG_LIST(ms, Word, theseWords);

  shared_ptr<CompoundWord> transformation;
  GET_WORD_ARG(ms, CompoundWord, transformation);

  int numTheseWords = theseWords.size();
  cout << "interlaceTop: ok, transforming a path with " << numTheseWords << " points." << endl;

  for (int i = 0; i < numTheseWords; i++) {
    ms->pushData(transformation);
    ms->pushData(theseWords[i]);
  }
}
END_WORD
REGISTER_WORD(InterlaceTop)

WORD(InterlaceBottom)
virtual void execute(MachineState * ms) {
  // XXX perhaps this is in fact "map"
  //  at any rate, a generalized TransformPath in the style of ReverseCompound
  vector< shared_ptr<Word> > theseWords;
  GET_WORD_ARG_LIST(ms, Word, theseWords);

  shared_ptr<CompoundWord> transformation;
  GET_WORD_ARG(ms, CompoundWord, transformation);

  int numTheseWords = theseWords.size();
  cout << "interlaceBottom: ok, transforming a path with " << numTheseWords << " points." << endl;

  for (int i = 0; i < numTheseWords; i++) {
    ms->pushData(theseWords[i]);
    ms->pushData(transformation);
  }
}
END_WORD
REGISTER_WORD(InterlaceBottom)

WORD(PlanToPointCraneFourStroke)
virtual void execute(MachineState * ms) {
  eePose targetPoseIn;
  GET_ARG(ms, EePoseWord, targetPoseIn);

  if ( (ms->config.bDelta <= 0) || (ms->config.bDelta > 0.5*ms->config.w1GoThresh) ) {
    cout << "planToPointCraneFourStroke: Oops, there's a problem with bDelta and w1GoThresh, " << ms->config.bDelta << " " << ms->config.w1GoThresh << endl;
    cout << "bDelta needs to be < 0.5 * w1GoThresh but greater than 0." << endl;
  }

  double current_r_coordinate = sqrt( pow(ms->config.currentEEPose.px, 2.0) + pow(ms->config.currentEEPose.py, 2.0) );
  double current_theta_coordinate = atan2(ms->config.currentEEPose.py, ms->config.currentEEPose.px);

  double p_inter_target_z = convertHeightIdxToGlobalZ(ms, 1);
  double p_inter_target_r = 0.8;

  double target_plane_distance = sqrt( pow(ms->config.currentEEPose.px - targetPoseIn.px, 2.0) + pow(ms->config.currentEEPose.py - targetPoseIn.py, 2.0) );
  double target_z_difference = targetPoseIn.pz - ms->config.currentEEPose.pz;
  double target_z_distance = fabs(target_z_difference);
  double target_r_coordinate = sqrt( pow(targetPoseIn.px, 2.0) + pow(targetPoseIn.py, 2.0) );
  double target_r_difference = target_r_coordinate - current_r_coordinate;
  double target_r_distance = fabs( target_r_difference );
  double target_theta_coordinate = atan2(targetPoseIn.py, targetPoseIn.px);
  double target_theta_difference = target_theta_coordinate - current_theta_coordinate;
  double target_theta_distance = fabs(target_theta_difference);

  double inter_r_distance = fabs( current_r_coordinate - p_inter_target_r);
  double inter_z_difference = p_inter_target_z - ms->config.currentEEPose.pz;
  double inter_z_distance = fabs(inter_z_difference);

  cout << "target plane_d: " << target_plane_distance << " z_dist: " << target_z_distance << " r_c: " 
  << target_r_coordinate << " r_diff: " << target_r_difference << " r_dist: " << target_r_distance << " theta_c: " 
  << target_theta_coordinate << " theta_diff: " << target_theta_difference << " theta_dist: " << target_theta_distance << endl;
  cout << "inter r_dist: " << inter_r_distance << " z_dist: " << inter_z_distance << endl;

  eePose nextPose = ms->config.currentEEPose;
  if (target_plane_distance > ms->config.w1GoThresh) { 
    cout << "A" << endl;
    if (inter_z_distance > ms->config.w1GoThresh) {
      double tSign = 0.0;
      if (inter_z_distance > 0.0) {
	tSign = inter_z_difference / inter_z_distance;
      }

      nextPose.pz += ms->config.bDelta * tSign;
      cout << "AA" << endl;

      ms->pushWord("planToPointCraneFourStroke");
      ms->pushWord(make_shared<EePoseWord>(targetPoseIn));
      ms->pushWord("waitUntilAtCurrentPosition");
      ms->pushWord("moveEeToPoseWord");
      ms->pushWord(make_shared<EePoseWord>(nextPose));
      ms->evaluateProgram("0 setCurrentIKFastMode");
    } else {
      cout << "AB" << endl;
      if (target_r_distance > ms->config.w1GoThresh) {
	cout << "ABA" << endl;
	double tSign = 0.0;
	if (target_r_distance > 0.0) {
	  tSign = target_r_difference / target_r_distance;
	}
	nextPose.px += ms->config.bDelta * tSign * ms->config.currentEEPose.px / current_r_coordinate;
	nextPose.py += ms->config.bDelta * tSign * ms->config.currentEEPose.py / current_r_coordinate;
      } else {
	cout << "ABB" << endl;
	//[ cos -sin 
	//  sin  cos ]
	double tSign = 0.0;
	if (target_theta_distance > 0.0) {
	  tSign = target_theta_difference / target_theta_distance;
	}
	
	double t_rotation_theta = ms->config.bDelta * tSign / current_r_coordinate;
  
	nextPose.px = nextPose.px * cos(t_rotation_theta) - nextPose.py * sin(t_rotation_theta);
	nextPose.py = nextPose.px * sin(t_rotation_theta) + nextPose.py * cos(t_rotation_theta);
	
	nextPose.px = target_r_coordinate * nextPose.px / current_r_coordinate;
	nextPose.py = target_r_coordinate * nextPose.py / current_r_coordinate;
      }

      ms->pushWord("planToPointCraneFourStroke");
      ms->pushWord(make_shared<EePoseWord>(targetPoseIn));
      ms->pushWord("waitUntilAtCurrentPosition");
      ms->pushWord("moveEeToPoseWord");
      ms->pushWord(make_shared<EePoseWord>(nextPose));
      ms->evaluateProgram("1 setCurrentIKFastMode");
    }
  } else if (target_z_distance > ms->config.w1GoThresh) {
    cout << "B" << endl;
    if (target_z_distance > ms->config.bDelta) {
      cout << "BA" << endl;

      double tSign = 0.0;
      if (target_z_distance > 0.0) {
	tSign = target_z_difference / target_z_distance;
      }

      nextPose.pz += ms->config.bDelta * tSign;

      ms->pushWord("planToPointCraneFourStroke");
      ms->pushWord(make_shared<EePoseWord>(targetPoseIn));
      ms->pushWord("waitUntilAtCurrentPosition");
      ms->pushWord("moveEeToPoseWord");
      ms->pushWord(make_shared<EePoseWord>(nextPose));
      ms->evaluateProgram("0 setCurrentIKFastMode");
    } else {
      cout << "BB" << endl;
      nextPose = targetPoseIn;

      ms->pushWord("waitUntilAtCurrentPosition");
      ms->pushWord("moveEeToPoseWord");
      ms->pushWord(make_shared<EePoseWord>(nextPose));
      ms->evaluateProgram("2 setCurrentIKFastMode");
    }

  } else {
    cout << "C" << endl;
    //cout << "planToPointCraneFourStroke: done." << endl;
  }
}
END_WORD
REGISTER_WORD(PlanToPointCraneFourStroke)

WORD(PlanToPointCraneThreeStroke)
virtual void execute(MachineState * ms) {
  eePose targetPoseIn;
  GET_ARG(ms, EePoseWord, targetPoseIn);

  if ( (ms->config.bDelta <= 0) || (ms->config.bDelta > 0.5*ms->config.w1GoThresh) ) {
    cout << "planToPointCraneThreeStroke: Oops, there's a problem with bDelta and w1GoThresh, " << ms->config.bDelta << " " << ms->config.w1GoThresh << endl;
    cout << "bDelta needs to be < 0.5 * w1GoThresh but greater than 0." << endl;
  }

  double current_r_coordinate = sqrt( pow(ms->config.currentEEPose.px, 2.0) + pow(ms->config.currentEEPose.py, 2.0) );
  double current_theta_coordinate = atan2(ms->config.currentEEPose.py, ms->config.currentEEPose.px);

  double p_inter_target_z = convertHeightIdxToGlobalZ(ms, 1);
  double p_inter_target_r = 0.8;

  double target_plane_distance = sqrt( pow(ms->config.currentEEPose.px - targetPoseIn.px, 2.0) + pow(ms->config.currentEEPose.py - targetPoseIn.py, 2.0) );
  double target_z_difference = targetPoseIn.pz - ms->config.currentEEPose.pz;
  double target_z_distance = fabs(target_z_difference);
  double target_r_coordinate = sqrt( pow(targetPoseIn.px, 2.0) + pow(targetPoseIn.py, 2.0) );
  double target_r_difference = target_r_coordinate - current_r_coordinate;
  double target_r_distance = fabs( target_r_difference );
  double target_theta_coordinate = atan2(targetPoseIn.py, targetPoseIn.px);
  double target_theta_difference = target_theta_coordinate - current_theta_coordinate;
  double target_theta_distance = fabs(target_theta_difference);

  double inter_r_distance = fabs( current_r_coordinate - p_inter_target_r);
  double inter_z_difference = p_inter_target_z - ms->config.currentEEPose.pz;
  double inter_z_distance = fabs(inter_z_difference);

  cout << "target plane_d: " << target_plane_distance << " z_dist: " << target_z_distance << " r_c: " 
  << target_r_coordinate << " r_diff: " << target_r_difference << " r_dist: " << target_r_distance << " theta_c: " 
  << target_theta_coordinate << " theta_diff: " << target_theta_difference << " theta_dist: " << target_theta_distance << endl;
  cout << "inter r_dist: " << inter_r_distance << " z_dist: " << inter_z_distance << endl;

  eePose nextPose = ms->config.currentEEPose;
  if (target_plane_distance > ms->config.w1GoThresh) { 
    cout << "A" << endl;
    if (inter_z_distance > ms->config.w1GoThresh) {
      double tSign = 0.0;
      if (inter_z_distance > 0.0) {
	tSign = inter_z_difference / inter_z_distance;
      }

      nextPose.pz += ms->config.bDelta * tSign;
      cout << "AA" << endl;

      ms->pushWord("planToPointCraneThreeStroke");
      ms->pushWord(make_shared<EePoseWord>(targetPoseIn));
      ms->pushWord("waitUntilAtCurrentPosition");
      ms->pushWord("moveEeToPoseWord");
      ms->pushWord(make_shared<EePoseWord>(nextPose));

      ms->evaluateProgram("0 setCurrentIKFastMode");
    } else {
      cout << "AB" << endl;
      {
	cout << "ABB" << endl;
	//[ cos -sin 
	//  sin  cos ]
	double tSign = 0.0;
	if (target_theta_distance > 0.0) {
	  tSign = target_theta_difference / target_theta_distance;
	}
	
	double t_rotation_theta = ms->config.bDelta * tSign / current_r_coordinate;
  
	nextPose.px = nextPose.px * cos(t_rotation_theta) - nextPose.py * sin(t_rotation_theta);
	nextPose.py = nextPose.px * sin(t_rotation_theta) + nextPose.py * cos(t_rotation_theta);
	
	//nextPose.px = target_r_coordinate * nextPose.px / current_r_coordinate;
	//nextPose.py = target_r_coordinate * nextPose.py / current_r_coordinate;
      }
      if (target_r_distance > ms->config.w1GoThresh) {
	cout << "ABA" << endl;
	double tSign = 0.0;
	if (target_r_distance > 0.0) {
	  tSign = target_r_difference / target_r_distance;
	}
	double radius_steps = target_r_coordinate * M_PI * target_theta_distance / ms->config.bDelta;
	double r_plan_steps = std::max(1.0, radius_steps/2.0);
	double r_delta = target_r_distance / r_plan_steps;
	nextPose.px += r_delta * tSign * ms->config.currentEEPose.px / current_r_coordinate;
	nextPose.py += r_delta * tSign * ms->config.currentEEPose.py / current_r_coordinate;
	//nextPose.px += ms->config.bDelta * tSign * ms->config.currentEEPose.px / current_r_coordinate;
	//nextPose.py += ms->config.bDelta * tSign * ms->config.currentEEPose.py / current_r_coordinate;
      }

      ms->pushWord("planToPointCraneThreeStroke");
      ms->pushWord(make_shared<EePoseWord>(targetPoseIn));
      ms->pushWord("waitUntilAtCurrentPosition");
      ms->pushWord("moveEeToPoseWord");
      ms->pushWord(make_shared<EePoseWord>(nextPose));
      ms->evaluateProgram("1 setCurrentIKFastMode");
    }
  } else if (target_z_distance > ms->config.w1GoThresh) {
    cout << "B" << endl;
    if (target_z_distance > ms->config.bDelta) {
      cout << "BA" << endl;

      double tSign = 0.0;
      if (target_z_distance > 0.0) {
	tSign = target_z_difference / target_z_distance;
      }

      nextPose.pz += ms->config.bDelta * tSign;

      ms->pushWord("planToPointCraneThreeStroke");
      ms->pushWord(make_shared<EePoseWord>(targetPoseIn));
      ms->pushWord("waitUntilAtCurrentPosition");
      ms->pushWord("moveEeToPoseWord");
      ms->pushWord(make_shared<EePoseWord>(nextPose));
      ms->evaluateProgram("0 setCurrentIKFastMode");
    } else {
      cout << "BB" << endl;
      nextPose = targetPoseIn;

      ms->pushWord("waitUntilAtCurrentPosition");
      ms->pushWord("moveEeToPoseWord");
      ms->pushWord(make_shared<EePoseWord>(nextPose));
      ms->evaluateProgram("2 setCurrentIKFastMode");
    }

  } else {
    cout << "C" << endl;
    //cout << "planToPointCraneThreeStroke: done." << endl;
  }
}
END_WORD
REGISTER_WORD(PlanToPointCraneThreeStroke)


WORD(PlanToPointCraneThreeStrokeOpenJointHyperPlanner)
virtual void execute(MachineState * ms) {
  eePose startPoseIn;
  GET_ARG(ms, EePoseWord, startPoseIn);

  eePose targetPoseIn;
  GET_ARG(ms, EePoseWord, targetPoseIn);

  if ( (ms->config.bDelta <= 0) || (ms->config.bDelta > 0.5*ms->config.w1GoThresh) ) {
    cout << "planToPointCraneThreeStroke: Oops, there's a problem with bDelta and w1GoThresh, " << ms->config.bDelta << " " << ms->config.w1GoThresh << endl;
    cout << "bDelta needs to be < 0.5 * w1GoThresh but greater than 0." << endl;
  }

  double current_r_coordinate = sqrt( pow(startPoseIn.px, 2.0) + pow(startPoseIn.py, 2.0) );
  double current_theta_coordinate = atan2(startPoseIn.py, startPoseIn.px);

  double p_inter_target_z = convertHeightIdxToGlobalZ(ms, 1);
  double p_inter_target_r = 0.8;

  double target_plane_distance = sqrt( pow(startPoseIn.px - targetPoseIn.px, 2.0) + pow(startPoseIn.py - targetPoseIn.py, 2.0) );
  double target_z_difference = targetPoseIn.pz - startPoseIn.pz;
  double target_z_distance = fabs(target_z_difference);
  double target_r_coordinate = sqrt( pow(targetPoseIn.px, 2.0) + pow(targetPoseIn.py, 2.0) );
  double target_r_difference = target_r_coordinate - current_r_coordinate;
  double target_r_distance = fabs( target_r_difference );
  double target_theta_coordinate = atan2(targetPoseIn.py, targetPoseIn.px);
  double target_theta_difference = target_theta_coordinate - current_theta_coordinate;
  double target_theta_distance = fabs(target_theta_difference);

  double inter_r_distance = fabs( current_r_coordinate - p_inter_target_r);
  double inter_z_difference = p_inter_target_z - startPoseIn.pz;
  double inter_z_distance = fabs(inter_z_difference);

  cout << "target plane_d: " << target_plane_distance << " z_dist: " << target_z_distance << " r_c: " 
  << target_r_coordinate << " r_diff: " << target_r_difference << " r_dist: " << target_r_distance << " theta_c: " 
  << target_theta_coordinate << " theta_diff: " << target_theta_difference << " theta_dist: " << target_theta_distance << endl;
  cout << "inter r_dist: " << inter_r_distance << " z_dist: " << inter_z_distance << endl;

  eePose nextPose = startPoseIn;
  if (target_plane_distance > ms->config.w1GoThresh) { 
    cout << "A" << endl;
    if (inter_z_distance > ms->config.w1GoThresh) {
      double tSign = 0.0;
      if (inter_z_distance > 0.0) {
	tSign = inter_z_difference / inter_z_distance;
      }

      nextPose.pz += ms->config.bDelta * tSign;
      cout << "AA" << endl;

      ms->pushWord(this->name());
      ms->pushWord(make_shared<EePoseWord>(nextPose));
      ms->pushWord(make_shared<EePoseWord>(targetPoseIn));
      ms->pushData("eePoseToArmPose");
      ms->pushData(make_shared<EePoseWord>(nextPose));
	ms->pushData("setCurrentIKFastMode");
	ms->pushData(make_shared<IntegerWord>(0));
    } else {
      cout << "AB" << endl;
      {
	cout << "ABB" << endl;
	//[ cos -sin 
	//  sin  cos ]
	double tSign = 0.0;
	if (target_theta_distance > 0.0) {
	  tSign = target_theta_difference / target_theta_distance;
	}
	
	double t_rotation_theta = ms->config.bDelta * tSign / current_r_coordinate;
  
	nextPose.px = nextPose.px * cos(t_rotation_theta) - nextPose.py * sin(t_rotation_theta);
	nextPose.py = nextPose.px * sin(t_rotation_theta) + nextPose.py * cos(t_rotation_theta);
	
	//nextPose.px = target_r_coordinate * nextPose.px / current_r_coordinate;
	//nextPose.py = target_r_coordinate * nextPose.py / current_r_coordinate;
      }
      if (target_r_distance > ms->config.w1GoThresh) {
	cout << "ABA" << endl;
	double tSign = 0.0;
	if (target_r_distance > 0.0) {
	  tSign = target_r_difference / target_r_distance;
	}
	double radius_steps = target_r_coordinate * M_PI * target_theta_distance / ms->config.bDelta;
	double r_plan_steps = std::max(1.0, radius_steps/2.0);
	double r_delta = target_r_distance / r_plan_steps;
	nextPose.px += r_delta * tSign * startPoseIn.px / current_r_coordinate;
	nextPose.py += r_delta * tSign * startPoseIn.py / current_r_coordinate;
	//nextPose.px += ms->config.bDelta * tSign * startPoseIn.px / current_r_coordinate;
	//nextPose.py += ms->config.bDelta * tSign * startPoseIn.py / current_r_coordinate;
      }

      ms->pushWord(this->name());
      ms->pushWord(make_shared<EePoseWord>(nextPose));
      ms->pushWord(make_shared<EePoseWord>(targetPoseIn));
      ms->pushData("eePoseToArmPose");
      ms->pushData(make_shared<EePoseWord>(nextPose));
	ms->pushData("setCurrentIKFastMode");
	ms->pushData(make_shared<IntegerWord>(1));
    }
  } else if (target_z_distance > ms->config.w1GoThresh) {
    cout << "B" << endl;
    if (target_z_distance > ms->config.bDelta) {
      cout << "BA" << endl;

      double tSign = 0.0;
      if (target_z_distance > 0.0) {
	tSign = target_z_difference / target_z_distance;
      }

      nextPose.pz += ms->config.bDelta * tSign;

      ms->pushWord(this->name());
      ms->pushWord(make_shared<EePoseWord>(nextPose));
      ms->pushWord(make_shared<EePoseWord>(targetPoseIn));
      ms->pushData("eePoseToArmPose");
      ms->pushData(make_shared<EePoseWord>(nextPose));
	ms->pushData("setCurrentIKFastMode");
	ms->pushData(make_shared<IntegerWord>(0));
    } else {
      cout << "BB" << endl;
      nextPose = targetPoseIn;

      ms->pushData("eePoseToArmPose");
      ms->pushData(make_shared<EePoseWord>(nextPose));
	ms->pushData("setCurrentIKFastMode");
	ms->pushData(make_shared<IntegerWord>(2));
    }

  } else {
    cout << "C" << endl;
    //cout << "planToPointCraneThreeStroke: done." << endl;
  }
}
END_WORD
REGISTER_WORD(PlanToPointCraneThreeStrokeOpenJointHyperPlanner)


WORD(PlanCommandJointsAtRateSpin)
virtual void execute(MachineState * ms) {
//  13 "frames" store 2 "unframes" store 0.015 "gap" store fullImpulse 2 waitForSeconds ( setControlModeAngles jointPathFive ( moveArmToPoseWord armPublishJointPositionCommand gap spinForSeconds ) frames replicateWord ( pop ) unframes replicateWord setControlModeAngles quarterImpulse 2 waitForSeconds jointPathFiveReverse ( pop ) unframes replicateWord ( moveArmToPoseWord armPublishJointPositionCommand 0.002 spinForSeconds ) frames replicateWord fullImpulse 5.0 waitForSeconds ) 7 replicateWord 

  // XXX perhaps this is in fact "map"
  //  at any rate, a generalized TransformPath in the style of ReverseCompound
  vector< shared_ptr<Word> > theseWords;
  GET_WORD_ARG_LIST(ms, Word, theseWords);

  double gap = 0.05;
  GET_NUMERIC_ARG(ms, gap);

  int numTheseWords = theseWords.size();
  cout << "planCommandJointsAtRateSpin: ok, commanding path with " << numTheseWords << " points with " << gap << " seconds in between." << endl;

  for (int i = 0; i < numTheseWords; i++) {
    ms->pushWord("armPublishJointPositionCommand");
    ms->pushWord("moveArmToPoseWord");
    ms->pushWord(theseWords[i]);
    ms->pushWord("spinForSeconds");
    ms->pushWord(make_shared<DoubleWord>(gap));
  }






}
END_WORD
REGISTER_WORD(PlanCommandJointsAtRateSpin)

WORD(TwistWords)
virtual void execute(MachineState * ms) {
// XXX 
  vector< shared_ptr<Word> > theseWords;
  GET_WORD_ARG_LIST(ms, Word, theseWords);

  vector< shared_ptr<Word> > thoseWords;
  GET_WORD_ARG_LIST(ms, Word, thoseWords);

  int numTheseWords = theseWords.size();
  int numThoseWords = thoseWords.size();
  cout << "twistWords: ok, twisting lists with " << numTheseWords << " and " << numThoseWords << " points." << endl;
  int numTwists = std::max(numTheseWords, numThoseWords);

  for (int i = 0; i < numTwists; i++) {
    if (i < numTheseWords) {
      ms->pushData(theseWords[i]);
    }
    if ( i < numThoseWords) {
      ms->pushData(thoseWords[i]);
    }
  }
}
END_WORD
REGISTER_WORD(TwistWords)

WORD(GaitAnchoredTranslation)
virtual void execute(MachineState * ms) {
  eePose startPoseIn;
  GET_ARG(ms, EePoseWord, startPoseIn);

/*
  shared_ptr<CompoundWord> qwW;
  GET_WORD_ARG(ms, CompoundWord, qwW);
  
  shared_ptr<CompoundWord> qzW;
  GET_WORD_ARG(ms, CompoundWord, qzW);
  
  shared_ptr<CompoundWord> qyW;
  GET_WORD_ARG(ms, CompoundWord, qyW);
  
  shared_ptr<CompoundWord> qxW;
  GET_WORD_ARG(ms, CompoundWord, qxW);
*/

  shared_ptr<CompoundWord> pzW;
  GET_WORD_ARG(ms, CompoundWord, pzW);
  
  shared_ptr<CompoundWord> pyW;
  GET_WORD_ARG(ms, CompoundWord, pyW);
  
  shared_ptr<CompoundWord> pxW;
  GET_WORD_ARG(ms, CompoundWord, pxW);
  

  ms->evaluateProgram("currentPose eePosePZ plus currentPose swap setEEPosePZ moveEeToPoseWord");
  ms->pushWord(pzW);

  ms->evaluateProgram("currentPose eePosePY plus currentPose swap setEEPosePY moveEeToPoseWord");
  ms->pushWord(pyW);

  ms->evaluateProgram("currentPose eePosePX plus currentPose swap setEEPosePX moveEeToPoseWord");
  ms->pushWord(pxW);

  ms->pushWord("moveEeToPoseWord");
  ms->pushWord(make_shared<EePoseWord>(startPoseIn));
}
END_WORD
REGISTER_WORD(GaitAnchoredTranslation)

/*
currentPose "placeA" store
currentPose "placeB" store

[ 0.01 0.01 endArgs placeA placeB interpolatePath ] "placeEePath" store

(might be reversed now)

[ ( eePoseToArmPose ) endArgs placeEePath interlaceBottom ] "placeHyperPlanner" store

[ placeHyperPlanner ] "placeJointPath" store

0.03 "placeSpeed" store setControlModeAngles

placeSpeed endArgs placeJointPath planCommandJointsAtRateSpin
placeSpeed endArgs [ placeJointPath ] reverseCompound slip planCommandJointsAtRateSpin


*/

/*
WORD(PlanKeepPushingDemonstratedEePoses)
virtual void execute(MachineState * ms) {

  double p_tol = 0.01;
  GET_NUMERIC_ARG(ms, p_tol);

  double q_tol = 0.01;
  GET_NUMERIC_ARG(ms, q_tol);

  eePose topPose;
  GET_ARG(ms,EePoseWord,topPose);
  

}
END_WORD
REGISTER_WORD(PlanKeepPushingDemonstratedEePoses)

WORD(PlanStopPushingDemonstratedEePoses)
virtual void execute(MachineState * ms) {
}
END_WORD
REGISTER_WORD(PlanStopPushingDemonstratedEePoses)

WORD(PlanStartPushingDemonstratedEePoses)
virtual void execute(MachineState * ms) {

  double p_tol = 0.01;
  GET_NUMERIC_ARG(ms, p_tol);

  double q_tol = 0.01;
  GET_NUMERIC_ARG(ms, q_tol);

}
END_WORD
REGISTER_WORD(PlanStartPushingDemonstratedEePoses)


WORD(PlanExecuteHyperPlanner)
virtual void execute(MachineState * ms) {
  // uses hyper planner to make a JIT planner (eePose plan with style info embedded in the eePose stream)
  //   this allows changes to ik policies to be made between executions of the JIT planner while it remains the same.
  // the output of a JIT planner is a joint trajectory.
//XXX needs a couple of little touches like priming the ik and resetting the ik upon return
// slide ( placeC placeB planToPointCraneThreeStrokeOpenJointHyperPlanner 1 |S ) reverseCompound "eePathTwo" store
// slide ( eePathTwo 1 |S ) reverseCompound "jointPathTwo" store
// 0.525866 -0.710611 -0.146919 -0.001222 0.999998 0.001162 -0.001101 createEEPose
// 0.525866 -0.710611 0.279748 -0.001222 0.999998 0.001162 -0.001101 createEEPose
// 0.525866 -0.710611 -0.066919 -0.001222 0.999998 0.001162 -0.001101 createEEPose
// 5 waitForSeconds ( setControlModeAngles jointPathTwoReverse jointPathTwo ( moveArmToPoseWord armPublishJointPositionCommand 0.02 spinForSeconds ) 80 replicateWord setControlModeEePosition placeC moveEeToPoseWord 3.0 waitForSeconds setControlModeAngles 0.5 spinForSeconds ) 10 replicateWord

//  13 "frames" store 2 "unframes" store 0.015 "gap" store fullImpulse 2 waitForSeconds ( setControlModeAngles jointPathFive ( moveArmToPoseWord armPublishJointPositionCommand gap spinForSeconds ) frames replicateWord ( pop ) unframes replicateWord setControlModeAngles quarterImpulse 2 waitForSeconds jointPathFiveReverse ( pop ) unframes replicateWord ( moveArmToPoseWord armPublishJointPositionCommand 0.002 spinForSeconds ) frames replicateWord fullImpulse 5.0 waitForSeconds ) 7 replicateWord 
}
END_WORD
REGISTER_WORD(PlanExecuteHyperPlanner)

WORD(PlanFollowPathWithWaits)
virtual void execute(MachineState * ms) {
  // markov planner takes a position on the stack, pushes and dups the next point in the sequence, and pushes itself (or another reentrant)
  //  onto the stack until the plan is done. So takes a start point and returns a list of points, which is the plan backwards.

  // this function list of points and pushes to the data stack a compound word program that executes the plan from the current positin

// XXX need waitless planners
// XXX test interleaving 
}
END_WORD
REGISTER_WORD(PlanFollowPathWithWaits)

WORD(PlanCommandJointsAtRateWait)
virtual void execute(MachineState * ms) {
}
END_WORD
REGISTER_WORD(PlanCommandJointsAtRateWait)

WORD(ScaleMeasures)
virtual void execute(MachineState * ms) {
  // parse a compound word and rewrite the measureTime statements
  // XXX OR this should be written into hardInterlace vs softInterlace
}
END_WORD
REGISTER_WORD(ScaleMeasures)

// twistWords takes two compound words A and B as arguments.
// A and B must start with MeasureTimeStart

WORD(TwistWords)
virtual void execute(MachineState * ms) {
// XXX 
}
END_WORD
REGISTER_WORD(TwistWords)
*/


}
