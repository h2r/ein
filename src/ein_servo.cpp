
#include "ein_words.h"
#include "ein.h"
#include "camera.h"

namespace ein_words {

CONFIG_SETTER_ENUM(SetGradientServoMode, ms->config.currentGradientServoMode, (gradientServoMode))
CONFIG_GETTER_INT(GradientServoMode, ms->config.currentGradientServoMode)

CONFIG_GETTER_DOUBLE(PickFlushFactor, ms->config.pickFlushFactor)
CONFIG_SETTER_DOUBLE(SetPickFlushFactor, ms->config.pickFlushFactor)

WORD(SetSnapToFlushGrasp)
virtual void execute(MachineState * ms)       {
  int valToSet = 0;
  GET_ARG(ms, IntegerWord, valToSet);

  cout << "setSnapToFlushGrasp: was " << ms->config.snapToFlushGrasp << " will be " << valToSet << endl;
  ms->config.snapToFlushGrasp = valToSet;
}
END_WORD
REGISTER_WORD(SetSnapToFlushGrasp)

WORD(MoveToTargetZAndGrasp)
virtual void execute(MachineState * ms)       {
  //ms->pushWord("moveToTargetZAndGraspA");

  cout << "moveToTargetZAndGrasp: snapToFlushGrasp is " << ms->config.snapToFlushGrasp << endl;
  if (ms->config.snapToFlushGrasp) {
    if (0) {
      // using position
      ms->pushWord("pressAndGrasp");
    } else if (0) {
      // using effort only 
      ms->pushWord("closeGripper");
      ms->pushWord("pressUntilEffort");

      ms->pushWord("setEffortThresh");
      ms->pushWord("8.0");


      ms->pushWord("setSpeed");
      ms->pushWord("0.03");

      ms->pushWord("pressUntilEffortInit");
      ms->pushWord("comeToStop");
      ms->pushWord("setMovementStateToMoving");
      ms->pushWord("comeToStop");
      ms->pushWord("waitUntilAtCurrentPosition");

      ms->pushWord("setSpeed");
      ms->pushWord("0.05");

      ms->pushWord("replicateWord");
      ms->pushWord("16");
      ms->pushData("localZUp");

      ms->pushWord("setGridSizeCoarse");
    } else {
      // using twist and effort
      ms->pushWord("closeGripper");
      ms->pushWord("pressUntilEffortOrTwist");

      ms->pushWord("setTwistThresh");
      ms->pushWord("0.015");

      ms->pushWord("setEffortThresh");
      ms->pushWord("20.0");

      ms->pushWord("setSpeed");
      ms->pushWord("0.03");

      ms->pushWord("pressUntilEffortOrTwistInit");
      ms->pushWord("comeToStop");
      ms->pushWord("setMovementStateToMoving");
      ms->pushWord("comeToStop");
      ms->pushWord("waitUntilAtCurrentPosition");

      ms->pushWord("setSpeed");
      ms->pushWord("0.05");

      ms->pushWord("replicateWord");
      ms->pushWord("16");
      ms->pushData("localZUp");

      ms->pushWord("setGridSizeCoarse");
    
    }
  } else {
    ms->pushWord("moveToTargetZAndGraspA");
  }
}
END_WORD
REGISTER_WORD(MoveToTargetZAndGrasp)

WORD(MoveToTargetZAndGraspA)
CODE(1048682)     // numlock + j
virtual void execute(MachineState * ms)       {
  ms->pushWord("closeGripper"); 

  cout << this->name() <<": "  << ms->config.classGraspZsSet.size() << " " << ms->config.classGraspZs.size() << endl;
  double pickZ;

/* Don't use for now.
  if ( (ms->config.classGraspZsSet.size() > ms->config.targetClass) && 
       (ms->config.classGraspZs.size() > ms->config.targetClass) &&
       (ms->config.classGraspZsSet[ms->config.targetClass] == 1)) {
    // -(-trZ - graspDepthOffset)
    // trZ holds the height above the table that we want the end effector to be. 
    pickZ = -ms->config.currentTableZ + -ms->config.classGraspZs[ms->config.targetClass];
    cout << "delivering class " << ms->config.classLabels[ms->config.targetClass] << " with classGraspZ " << ms->config.classGraspZs[ms->config.targetClass] << endl;
  }  
  else 
*/
  {
    double threshedZ = min(ms->config.trZ, 0.0);
    
    // trZ is the height of the object above the table. 
    // pickFlushFactor is distance from the end effector Z to the tip of the gripper. 
    double worldZOfObjectTop = -(threshedZ + ms->config.currentTableZ) ;
    double pickZpre = worldZOfObjectTop + ms->config.pickFlushFactor + ms->config.graspDepthOffset;
    double flushZ = -(ms->config.currentTableZ) + ms->config.pickFlushFactor;
    pickZ = max(flushZ, pickZpre);
    cout << "moveToTargetZAndGrasp trZ pickZ flushZ pickZpre: " << ms->config.trZ << " " << pickZ << " " << flushZ << " " << pickZpre << " " << endl;    
  }

  cout << "pickZ: " << pickZ << endl;
  bool useHybridPick = 1;
  
  double deltaZ = pickZ - ms->config.currentEEPose.pz;
  ms->config.lastPickPose = ms->config.currentEEPose;
  ms->config.lastPickPose.pz = pickZ;

  int tbb = ms->config.targetBlueBox;
  if (tbb < ms->config.blueBoxMemories.size()) {
    ms->config.blueBoxMemories[tbb].pickedPose = ms->config.lastPickPose;  
  } else {
    assert(0);
  }

  if (useHybridPick) {
    int pickNoops = 20;
    int increments = 0.1/GRID_COARSE;
    ms->config.currentEEPose.pz = pickZ+increments*GRID_COARSE;
    ms->pushWord("comeToStop");
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushCopies('s', increments);
    ms->pushWord("setGridSizeCoarse");
    ms->pushWord("approachSpeed");
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    //ms->pushWord("quarterImpulse");
    ms->pushWord("approachSpeed");
  } else {
    ms->config.currentEEPose.pz = pickZ;
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  }
  
}
END_WORD
REGISTER_WORD(MoveToTargetZAndGraspA)

WORD(ShakeItUpAndDown)
CODE(131081)   // capslock + tab
virtual void execute(MachineState * ms) {
  ms->config.eepReg5 = ms->config.currentEEPose;

  ms->config.eepReg6 = ms->config.currentEEPose;
  ms->config.eepReg6.pz += 0.2;

  pushGridSign(ms, GRID_COARSE);    
  if (isGripperGripping(ms)) {
    ms->pushWord("happyFace");
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord('5');  // assume pose at register 5
      
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord('6'); // assume pose at register 6
      
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord('5'); // assume pose at register 5
      
    ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
    ms->pushWord('6');

  }

  pushGridSign(ms, NOW_THATS_COARSE);
  ms->pushWord("fullImpulse");

}
END_WORD
REGISTER_WORD(ShakeItUpAndDown)

WORD(TryToMoveToTheLastPrePickHeight)
virtual void execute(MachineState * ms) {
  ms->config.currentEEPose.pz = ms->config.lastPrePickPose.pz;
  ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
  cout << "trying to move to the last pre pick height..." << endl;
}
END_WORD
REGISTER_WORD(TryToMoveToTheLastPrePickHeight)

WORD(TryToMoveToTheLastPickHeight)
CODE( 262241)     // ctrl + a
virtual void execute(MachineState * ms) {
  double deltaZ = (ms->config.lastPickPose.pz) - ms->config.currentEEPose.pz;
  double zTimes = fabs(floor(deltaZ / ms->config.bDelta)); 
  int numNoOps = 2;
  int useIncrementalPlace = 0;
  bool useHybridPlace = 1;
  if (useIncrementalPlace) {
    if (deltaZ > 0) {
      for (int zc = 0; zc < zTimes; zc++) {
	for (int cc = 0; cc < numNoOps; cc++) {
	  ms->pushWord("endStackCollapseNoop");
	}
	ms->pushWord('w');
      }
    }
    if (deltaZ < 0) {
      for (int zc = 0; zc < zTimes; zc++) {
	for (int cc = 0; cc < numNoOps; cc++) {
	  ms->pushWord("endStackCollapseNoop");
	}
	ms->pushWord('s');
      }
    }
  } else {
    if (useHybridPlace) {
      int pickNoops = 20;
      int increments = 0.1/GRID_COARSE;
      ms->config.currentEEPose.pz = ms->config.lastPickPose.pz+increments*GRID_COARSE;

      //ms->pushCopies("endStackCollapseNoop", pickNoops);
      ms->pushWord("waitUntilAtCurrentPosition"); 
      ms->pushCopies('s', increments);
      ms->pushWord("setGridSizeCoarse");
      ms->pushWord("approachSpeed");
      ms->pushWord("waitUntilAtCurrentPosition"); 
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      ms->pushWord("waitUntilAtCurrentPosition"); // w1 wait until at current position
      //ms->pushWord("quarterImpulse");
      ms->pushWord("approachSpeed");
    } else {
      ms->config.currentEEPose.pz = ms->config.lastPickPose.pz;
      ms->pushWord("waitUntilAtCurrentPosition"); 
    }
  }
  cout << "trying to move to the last pick height..." << endl;
}
END_WORD
REGISTER_WORD(TryToMoveToTheLastPickHeight)


WORD(IfNoGrasp)
virtual void execute(MachineState * ms) {
  if (isGripperGripping(ms))  {
    ms->popWord();
  }
}
END_WORD
REGISTER_WORD(IfNoGrasp)

WORD(IfGrasp)
virtual void execute(MachineState * ms) {
  if (!isGripperGripping(ms))  {
    ms->popWord();
  }
}
END_WORD
REGISTER_WORD(IfGrasp)




WORD(ShakeItOff1)
CODE( 131151)     // capslock + o
  virtual void execute(MachineState * ms)       {
  int depthToPlunge = 24;
  int flexThisFar = 80;
  cout << "SHAKING IT OFF!!!" << endl;

  ms->pushNoOps(60);
  //ms->pushWord("moveToRegister2"); // assume pose at register 2
  ms->pushWord("closeGripper"); // close gripper
  ms->pushNoOps(20);
  ms->pushWord("openGripper"); // open gripper
  ms->pushWord("closeGripper"); // close gripper
  ms->pushNoOps(20);
  //ms->pushCopies('w', depthToPlunge); // move up 
  ms->pushWord("openGripper"); // open gripper
  ms->pushWord("closeGripper"); // close gripper
  //ms->pushNoOps(20);
  //ms->pushCopies("zDown"+65504, flexThisFar); // rotate forward
  //ms->pushCopies('e', 5); // move forward
  //ms->pushCopies('s', depthToPlunge); // move down
  ms->pushWord("openGripper"); // open gripper
  ms->pushNoOps(50);
  //ms->pushCopies('w'+65504, flexThisFar); // rotate forward

  //ms->pushWord("moveToRegister2"); // assume pose at register 2
  pushGridSign(ms, GRID_COARSE);

  // resets the gripper server
  //int sis = system("bash -c \"echo -e \'cC\003\' | rosrun baxter_examples gripper_keyboard.py\"");
  //calibrateGripper();
}
END_WORD
REGISTER_WORD(ShakeItOff1)



WORD(CheckGrasp)
CODE(196718)     // capslock + N 
virtual void execute(MachineState * ms)       {
  if (ms->config.gripperMoving) {
    ms->pushWord("checkGrasp"); // check grasp
  } else {
    cout << "gripperPosition: " << ms->config.gripperPosition << " gripperThresh: " << ms->config.gripperThresh << endl;
    cout << "gripperGripping: " << ms->config.gripperGripping << endl;
    if (!isGripperGripping(ms)) {
      cout << "Failed to pick." << endl;
      ms->config.thisGraspPicked = FAILURE;
      ms->pushWord("sadFace");
      ms->pushCopies("beep", 15); // beep
    } else {
      cout << "Successful pick." << endl;
      ms->config.thisGraspPicked = SUCCESS;
      ms->pushWord("happyFace");
    }
  }
}
END_WORD
REGISTER_WORD(CheckGrasp)

WORD(CheckIfJammed)
virtual void execute(MachineState * ms)       {
  REQUIRE_FOCUSED_CLASS(ms,tfc);

  if (ms->config.gripperMoving) {
    ms->pushWord("checkIfJammed"); // check grasp
  } else {
    cout << "gripperPosition: " << ms->config.gripperPosition << " gripperThresh: " << ms->config.gripperThresh << endl;
    cout << "gripperGripping: " << ms->config.gripperGripping << endl;
    if (isGripperGripping(ms)) {
      cout << "STUCK!!!!!!" << endl;
      ms->pushWord("sadFace");

      ms->pushWord("pauseStackExecution");
      ms->pushCopies("beep", 15); // beep

      int t3dgi = ms->config.current3dGraspIndex;
      if ( (t3dgi > -1) && (t3dgi < ms->config.class3dGrasps.size()) ) {
	cout << "checkIfJammed: using t3dgi " << t3dgi << endl;
	Grasp *thisGrasp = &(ms->config.class3dGrasps[tfc][t3dgi]);
	thisGrasp->jams++;
	// NOTE: true successes is successes - jams, this seems like it is more error tolerant.
      } else {
	cout << "checkIfJammed: bad t3dgi, doing nothing... " << t3dgi << endl;
      }

    } else {
      cout << "Not stuck :)" << endl;
      ms->pushWord("happyFace");
    }
  }
}
END_WORD
REGISTER_WORD(CheckIfJammed)

WORD(SetGraspModeToCrane)
virtual void execute(MachineState * ms) {
  cout << "Setting grasp mode to GRASP_CRANE." << endl;
  ms->config.currentGraspMode = GRASP_CRANE;
}
END_WORD
REGISTER_WORD(SetGraspModeToCrane)

WORD(SetGraspModeTo3D)
virtual void execute(MachineState * ms) {
  cout << "Setting grasp mode to GRASP_3D." << endl;
  ms->config.currentGraspMode = GRASP_3D;
}
END_WORD
REGISTER_WORD(SetGraspModeTo3D)


WORD(Blur)
CODE(1048697)  // numlock + y
virtual void execute(MachineState * ms) {
  double tfilter[9] = { 1.0/16.0, 1.0/8.0, 1.0/16.0, 
                        1.0/8.0, 1.0/4.0, 1.0/8.0, 
                        1.0/16.0, 1.0/8.0, 1.0/16.0};
  for (int fx = 0; fx < 9; fx++) {
    ms->config.filter[fx] = tfilter[fx];
  }
}
END_WORD
REGISTER_WORD(Blur)

}
