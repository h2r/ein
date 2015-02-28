WORD(IncrementGraspGear)
CODE(196712)     // capslock + H
virtual void execute()       {
  cout << "increment currentGraspGear was is: " << currentGraspGear << " ";
  int thisGraspGear = (currentGraspGear + 1) % totalGraspGears;
  
  //   set drX
  drX = ggX[thisGraspGear];
  drY = ggY[thisGraspGear];
  
  //   rotate
  setGGRotation(thisGraspGear);
  currentGraspGear = thisGraspGear;
  
  cout << currentGraspGear << endl;
}
END_WORD





WORD(ShiftGraspGear)
CODE(1114155)     // numlock + +
virtual void execute() {
  pushNoOps(50);
  int thisGraspGear = (currentGraspGear+4) % totalGraspGears;
  
  //   set drX
  drX = ggX[thisGraspGear];
  drY = ggY[thisGraspGear];
  
  //   rotate
  setGGRotation(thisGraspGear);
  
  //   set currentGraspGear;
  currentGraspGear = thisGraspGear;
}
END_WORD

WORD(PrepareToApplyGraspFilterFor1)
CODE(1048681)     // numlock + i
virtual void execute() {
  prepareGraspFilter1();
}
END_WORD

WORD(PrepareToApplyGraspFilterFor2)
CODE(1048687)     // numlock + o
virtual void execute() {
  prepareGraspFilter2();
}
END_WORD


WORD(PrepareToApplyGraspFilterFor3)
CODE(1048693)     // numlock + u
virtual void execute() {
  prepareGraspFilter3();
}
END_WORD

WORD(PrepareToApplyGraspFilterFor4)
CODE(1048688)     // numlock + p
virtual void execute() {
  prepareGraspFilter4();
}
END_WORD


WORD(SelectBestAvailableGrasp)
CODE(1048630)  // numlock + 6
virtual void execute() {
  cout << "Selecting best of 4 grasps... numlock + 6" << endl;
  // select max target cumulative
  pilot_call_stack.push_back(1114195);
  // apply grasp filter for 4
  pilot_call_stack.push_back(1048673); // drawMapRegisters
  pilot_call_stack.push_back(1048692);
  pilot_call_stack.push_back(1048688);
  // blur
  pilot_call_stack.push_back(1048673); // drawMapRegisters
  pilot_call_stack.push_back(1048692);
  pilot_call_stack.push_back(1048697);
  // load reg1
  pilot_call_stack.push_back(1048690);
  // change gear to 4
  pilot_call_stack.push_back(1048628);

  // select max target cumulative
  pilot_call_stack.push_back(1114195);
  // apply grasp filter for 3
  pilot_call_stack.push_back(1048673); // drawMapRegisters
  pilot_call_stack.push_back(1048692);
  pilot_call_stack.push_back(1048693);
  // blur
  pilot_call_stack.push_back(1048673); // drawMapRegisters
  pilot_call_stack.push_back(1048692);
  pilot_call_stack.push_back(1048697);
  // load reg1
  pilot_call_stack.push_back(1048690);
  // change gear to 3
  pilot_call_stack.push_back(1048627);

  // select max target cumulative
  pilot_call_stack.push_back(1114195);
  // apply grasp filter for 2
  pilot_call_stack.push_back(1048673); // drawMapRegisters
  pilot_call_stack.push_back(1048692);
  pilot_call_stack.push_back(1048687);
  // blur
  pilot_call_stack.push_back(1048673); // drawMapRegisters
  pilot_call_stack.push_back(1048692);
  pilot_call_stack.push_back(1048697);
  // load reg1
  pilot_call_stack.push_back(1048690);
  // change gear to 2
  pilot_call_stack.push_back(1048626);

  // select max target NOT cumulative
  pilot_call_stack.push_back(1048691);
  // apply grasp filter for 1
  pilot_call_stack.push_back(1048673); // drawMapRegisters
  pilot_call_stack.push_back(1048692);
  pilot_call_stack.push_back(1048681);
  // blur
  pilot_call_stack.push_back(1048673); // drawMapRegisters
  pilot_call_stack.push_back(1048692);
  pilot_call_stack.push_back(1048697);
  // load reg1
  pilot_call_stack.push_back(1048690);
  // change gear to 1
  pilot_call_stack.push_back(1048625);
  pilot_call_stack.push_back(1048684); // turn off scanning
}
END_WORD

WORD(SelectMaxTargetNotCumulative)
CODE(1048691)     // numlock + s
virtual void execute() {
  selectMaxTarget(VERYBIGNUMBER);
}
END_WORD

WORD(SelectMaxTargetCumulative)
CODE(1114195)     // numlock + S
virtual void execute() {
  selectMaxTarget(maxD);
}
END_WORD

WORD(ApplyGraspFilter)
CODE(1048692)  // numlock + t
virtual void execute() {
  applyGraspFilter(rangeMapReg1, rangeMapReg2);
}
END_WORD


WORD(Blur)
CODE(1048697)  // numlock + y
virtual void execute() {
  double tfilter[9] = { 1.0/16.0, 1.0/8.0, 1.0/16.0, 
                        1.0/8.0, 1.0/4.0, 1.0/8.0, 
                        1.0/16.0, 1.0/8.0, 1.0/16.0};
  for (int fx = 0; fx < 9; fx++) {
    filter[fx] = tfilter[fx];
  }
}
END_WORD

WORD(ShiftIntoGraspGear1)
CODE(1048625)      // numlock + 1
virtual void execute() {
  int thisGraspGear = 0;
  
  //   set drX
  drX = ggX[thisGraspGear];
  drY = ggY[thisGraspGear];
  
  //   rotate
  setGGRotation(thisGraspGear);
  
  //   set currentGraspGear;
  currentGraspGear = thisGraspGear;
}
END_WORD

WORD(ShiftIntoGraspGear2)
CODE(1048626)     // numlock + 2
virtual void execute() {
  int thisGraspGear = 1;
  //   set drX
  drX = ggX[thisGraspGear];
  drY = ggY[thisGraspGear];
  
  //   rotate
  setGGRotation(thisGraspGear);
  
  //   set currentGraspGear;
  currentGraspGear = thisGraspGear;
}
END_WORD

WORD(ShiftIntoGraspGear3)
CODE(1048627)     // numlock + 3
virtual void execute() {
  int thisGraspGear = 2;
  
  //   set drX
  drX = ggX[thisGraspGear];
  drY = ggY[thisGraspGear];
  
  //   rotate
  setGGRotation(thisGraspGear);
  
  //   set currentGraspGear;
  currentGraspGear = thisGraspGear;

}
END_WORD


WORD(ShiftIntoGraspGear4)
CODE(1048628)      // numlock + 4
virtual void execute() {
  int thisGraspGear = 3;
  //   set drX
  drX = ggX[thisGraspGear];
  drY = ggY[thisGraspGear];
  //   rotate
  setGGRotation(thisGraspGear);
  //   set currentGraspGear;
  currentGraspGear = thisGraspGear;
}
END_WORD


WORD(TurnOffScanning)
CODE(1048684)     // numlock + l
virtual void execute() {
  recordRangeMap = 0;
}
END_WORD


WORD(ResetAerialGradientTemporalFrameAverage)
CODE(262237)      // ctrl + ]
virtual void execute() {
  cout << "resetting aerialGradientTemporalFrameAverage." << endl;
  aerialGradientTemporalFrameAverage *= 0.0;
}
END_WORD


WORD(SynchronicServoDoNotTakeClosest)
CODE(131139)  // capslock + c
virtual void execute() {
  synchronicTakeClosest = 0;
  cout << "synchronicTakeClosest = 0" << endl;
  synServoLockFrames = 0;
}
END_WORD


WORD(SynchronicServoTakeClosest)
CODE(196707)     // capslock + C
virtual void execute() {
  synchronicTakeClosest = 1;
  cout << "synchronicTakeClosest = 1" << endl;
  synServoLockFrames = 0;
}
END_WORD





WORD(TwoDPatrolContinue)
CODE(131141) // capslock + e
virtual void execute() {
  thisGraspPicked = UNKNOWN;
  thisGraspReleased = UNKNOWN;
  neutral();
  
  if (ARE_GENERIC_PICK_LEARNING()) {
    if (thompsonHardCutoff) {
      if (graspAttemptCounter >= thompsonTries) {
        cout << "Clearing call stack because we did " << graspAttemptCounter << " tries." << endl;
        pilot_call_stack.resize(0);
        pushCopies(1245308, 15); // beep
        return;
      }
    }
    
    if (thompsonAdaptiveCutoff) {
      if ( (thompsonPickHaltFlag) ||
           (graspAttemptCounter >= thompsonTries) ) {
        cout << "Clearing call stack. thompsonPickHaltFlag = " << thompsonPickHaltFlag << 
          " and we did " << graspAttemptCounter << " tries." << endl;
        pilot_call_stack.resize(0);
        pushCopies(1245308, 15); // beep
        return;
      }
    }
  }
  
  synServoLockFrames = 0;
  currentGradientServoIterations = 0;
  
  ros::Duration delta = (ros::Time::now() - oscilStart) + accumulatedTime;
  
  currentEEPose.px = oscCenX + oscAmpX*sin(2.0*3.1415926*oscFreqX*delta.toSec());
  currentEEPose.py = oscCenY + oscAmpY*sin(2.0*3.1415926*oscFreqY*delta.toSec());
  
  // check to see if the target class is around, or take closest
  if ( ((pilotTarget.px != -1) && (pilotTarget.py != -1)) ||
       (synchronicTakeClosest && ((pilotClosestTarget.px != -1) && (pilotClosestTarget.py != -1))) )
    {
      // if so, push servoing command and set lock frames to 0
      pilot_call_stack.push_back(131156); // synchronic servo
      pilot_call_stack.push_back(131146); // turn survey on
      
      if (targetClass != -1)
        cout << "Found the target " << classLabels[targetClass] << ". " << endl;
      // grab the last bit of accumulated time
      accumulatedTime = accumulatedTime + (ros::Time::now() - oscilStart);
    } else {
    // if not, potentially do vision and continue the 2D patrol
    
    pilot_call_stack.push_back(131141); // 2D patrol continue
    // check and push vision cycle 
    ros::Duration timeSinceLast = ros::Time::now() - lastVisionCycle;
    if (timeSinceLast.toSec() > visionCycleInterval) {
      if (collectBackgroundInstances) {
        pilot_call_stack.push_back(131152); // save all blue boxes as focused class
      }
      pilot_call_stack.push_back(131153); // vision cycle
      // grab the last bit of accumulated time
      accumulatedTime = accumulatedTime + (ros::Time::now() - oscilStart);
    }
  }
  // if you are static_prior, this does nothing and defaults to the usual height
  pilot_call_stack.push_back(1245247); // sample height
}
END_WORD

WORD(SynchronicServo)
CODE(131156)    // capslock + t
virtual void execute() { 
  synchronicServo();
}
END_WORD

WORD(GradientServo)
CODE(196728)   // capslock + X
virtual void execute() {
  gradientServo();
}
END_WORD



WORD(GradientServoTakeClosest)
// capslock + numlock + h
CODE(1179720)
  virtual void execute()
{
    gradientTakeClosest = 1;
    cout << "gradientTakeClosest = " << gradientTakeClosest << endl;
}
END_WORD

