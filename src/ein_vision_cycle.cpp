#include <object_recognition_msgs/RecognizedObjectArray.h>

#include "ein_words.h"
#include "ein.h"
#include "qtgui/einwindow.h"
#include "camera.h"


namespace ein_words {

WORD(ClearStackIntoMappingPatrol)
virtual void execute(MachineState * ms) {
  ms->clearStack();
  ms->pushWord("mappingPatrol");
  ms->execute_stack = 1;
}
END_WORD
REGISTER_WORD(ClearStackIntoMappingPatrol)


WORD(ToggleShouldIDoIK)
virtual void execute(MachineState * ms) {
  ms->config.shouldIDoIK = !ms->config.shouldIDoIK;
}
END_WORD
REGISTER_WORD(ToggleShouldIDoIK)

WORD(ToggleShouldIRender)
virtual void execute(MachineState * ms) {
  ms->config.shouldIRender = !ms->config.shouldIRender;
}
END_WORD
REGISTER_WORD(ToggleShouldIRender)

WORD(ToggleDrawClearanceMap)
virtual void execute(MachineState * ms) {
  ms->config.drawClearanceMap = !ms->config.drawClearanceMap;
}
END_WORD
REGISTER_WORD(ToggleDrawClearanceMap)

WORD(ToggleDrawIKMap)
virtual void execute(MachineState * ms) {
  ms->config.drawIKMap = !ms->config.drawIKMap;
}
END_WORD
REGISTER_WORD(ToggleDrawIKMap)

WORD(ToggleUseGlow)
virtual void execute(MachineState * ms) {
  ms->config.useGlow = !ms->config.useGlow;
}
END_WORD
REGISTER_WORD(ToggleUseGlow)

WORD(ToggleUseFade)
virtual void execute(MachineState * ms) {
  ms->config.useFade = !ms->config.useFade;
}
END_WORD
REGISTER_WORD(ToggleUseFade)

CONFIG_GETTER_INT(PursuitProximity, ms->config.pursuitProximity)
CONFIG_SETTER_INT(SetPursuitProximity, ms->config.pursuitProximity)

CONFIG_GETTER_INT(SearchProximity, ms->config.searchProximity)
CONFIG_SETTER_INT(SetSearchProximity, ms->config.searchProximity)


WORD(PublishRecognizedObjectArrayFromBlueBoxMemory)
virtual void execute(MachineState * ms) {
  object_recognition_msgs::RecognizedObjectArray roaO;
  fillRecognizedObjectArrayFromBlueBoxMemory(ms, &roaO);
  ms->config.rec_objs_blue_memory.publish(roaO);
}
END_WORD
REGISTER_WORD(PublishRecognizedObjectArrayFromBlueBoxMemory)


WORD(RecordAllBlueBoxes)
virtual void execute(MachineState * ms) {
  cout << "Recording blue boxes: " << ms->config.bTops.size() << endl;
  for (int c = 0; c < ms->config.bTops.size(); c++) {
    BoxMemory box;
    box.bTop = ms->config.bTops[c];
    box.bBot = ms->config.bBots[c];
    box.cameraPose = ms->config.currentEEPose;
    box.top = pixelToGlobalEEPose(ms, box.bTop.x, box.bTop.y, ms->config.trueEEPoseEEPose.pz + ms->config.currentTableZ);
    box.bot = pixelToGlobalEEPose(ms, box.bBot.x, box.bBot.y, ms->config.trueEEPoseEEPose.pz + ms->config.currentTableZ);
    box.centroid.px = (box.top.px + box.bot.px) * 0.5;
    box.centroid.py = (box.top.py + box.bot.py) * 0.5;
    box.centroid.pz = (box.top.pz + box.bot.pz) * 0.5;
    box.cameraTime = ros::Time::now();
    box.labeledClassIndex = ms->config.bLabels[c];
    ms->config.blueBoxMemories.push_back(box);
  }

}
END_WORD
REGISTER_WORD(RecordAllBlueBoxes)

WORD(VoidCurrentMapRegion)
virtual void execute(MachineState * ms) {
  voidMapRegion(ms, ms->config.currentEEPose.px, ms->config.currentEEPose.py);
  cout << "Voiding the region of the map around ms->config.currentEEPose." << endl;
}
END_WORD
REGISTER_WORD(VoidCurrentMapRegion)

WORD(ClearMapForPatrol)
virtual void execute(MachineState * ms) {
  clearMapForPatrol(ms);
  cout << "Clearing the map for a new patrol." << endl;
}
END_WORD
REGISTER_WORD(ClearMapForPatrol)

WORD(MarkMapAsCompleted)
virtual void execute(MachineState * ms) {
  markMapAsCompleted(ms);
  cout << "Marking whole map as completed." << endl;
}
END_WORD
REGISTER_WORD(MarkMapAsCompleted)

WORD(InitializeMap)
virtual void execute(MachineState * ms) {
  initializeMap(ms);
}
END_WORD
REGISTER_WORD(InitializeMap)

CONFIG_GETTER_INT(MapGrayBoxPixelSkirtCols, ms->config.mapGrayBoxPixelSkirtCols)
CONFIG_SETTER_INT(SetMapGrayBoxPixelSkirtCols, ms->config.mapGrayBoxPixelSkirtCols)

CONFIG_GETTER_INT(MapGrayBoxPixelSkirtRows, ms->config.mapGrayBoxPixelSkirtRows)
CONFIG_SETTER_INT(SetMapGrayBoxPixelSkirtRows, ms->config.mapGrayBoxPixelSkirtRows)

CONFIG_GETTER_INT(MapGrayBoxPixelWaistCols, ms->config.mapGrayBoxPixelWaistCols)
CONFIG_SETTER_INT(SetMapGrayBoxPixelWaistCols, ms->config.mapGrayBoxPixelWaistCols)

CONFIG_GETTER_INT(MapGrayBoxPixelWaistRows, ms->config.mapGrayBoxPixelWaistRows)
CONFIG_SETTER_INT(SetMapGrayBoxPixelWaistRows, ms->config.mapGrayBoxPixelWaistRows)

CONFIG_GETTER_INT(MapFreeSpacePixelSkirt, ms->config.mapFreeSpacePixelSkirt)
CONFIG_SETTER_INT(SetMapFreeSpacePixelSkirt, ms->config.mapFreeSpacePixelSkirt)

WORD(MapEmptySpace)
virtual void execute(MachineState * ms) {
  Camera * camera = ms->config.cameras[ms->config.focused_camera];
  
  for (int px = ms->config.grayTop.x+ms->config.mapGrayBoxPixelSkirtCols; px < ms->config.grayBot.x-ms->config.mapGrayBoxPixelSkirtCols; px++) {
    for (int py = ms->config.grayTop.y+ms->config.mapGrayBoxPixelSkirtRows; py < ms->config.grayBot.y-ms->config.mapGrayBoxPixelSkirtRows; py++) {
      
      if (isInGripperMask(ms, px, py)) {
	continue;
      }
      
      //int blueBoxIdx = blueBoxForPixel(px, py);
      int blueBoxIdx = skirtedBlueBoxForPixel(ms, px, py, ms->config.mapFreeSpacePixelSkirt);
      
      if (blueBoxIdx == -1) {
        double x, y;
        double z = ms->config.trueEEPoseEEPose.pz + ms->config.currentTableZ;

        pixelToGlobal(ms, px, py, z, &x, &y);
        int i, j;
        mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, x, y, &i, &j);
	
	//        if (ros::Time::now() - ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime > mapMemoryTimeout) {
	//          ms->config.objectMap[i + ms->config.mapWidth * j].b = 0;
	//          ms->config.objectMap[i + ms->config.mapWidth * j].g = 0;
	//          ms->config.objectMap[i + ms->config.mapWidth * j].r = 0;
	//          ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount = 0;
	//        }
	
	// if we are mapping too close to the table these can exceed bounds
	if ( (i < 0) || (j < 0) || (i >= ms->config.mapWidth) || (j >= ms->config.mapHeight) ) {
	  continue;
	}


        ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime = ros::Time::now();
        randomizeNanos(ms, &ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime);
        
        ms->config.objectMap[i + ms->config.mapWidth * j].detectedClass = -2;

//	{
//	  ms->config.objectMap[i + ms->config.mapWidth * j].b += (int) camera->cam_img.at<cv::Vec3b>(py, px)[0];
//	  ms->config.objectMap[i + ms->config.mapWidth * j].g += (int) camera->cam_img.at<cv::Vec3b>(py, px)[1];
//	  ms->config.objectMap[i + ms->config.mapWidth * j].r += (int) camera->cam_img.at<cv::Vec3b>(py, px)[2];
//        ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount += 1.0;
//	}
	//const double spaceDecay = 0.996; // 0.7 ^ 0.01
	const double spaceDecay = 0.99821; // 0.7 ^ 0.005
	{
	  ms->config.objectMap[i + ms->config.mapWidth * j].b = 
	    ( spaceDecay*double(ms->config.objectMap[i + ms->config.mapWidth * j].b) + 
		    (1.0-spaceDecay)*double(camera->cam_img.at<cv::Vec3b>(py, px)[0]) );
	  ms->config.objectMap[i + ms->config.mapWidth * j].g = 
	    ( spaceDecay*double(ms->config.objectMap[i + ms->config.mapWidth * j].g) + 
		    (1.0-spaceDecay)*double(camera->cam_img.at<cv::Vec3b>(py, px)[1]) );
	  ms->config.objectMap[i + ms->config.mapWidth * j].r = 
	    ( spaceDecay*double(ms->config.objectMap[i + ms->config.mapWidth * j].r) + 
		    (1.0-spaceDecay)*double(camera->cam_img.at<cv::Vec3b>(py, px)[2]) );
	  ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount = 
	    ( spaceDecay*double(ms->config.objectMap[i + ms->config.mapWidth * j].pixelCount) + 
		    (1.0-spaceDecay)*double(1.0) );
	}

      }
    }
  }
}
END_WORD
REGISTER_WORD(MapEmptySpace)





WORD(MapClosestBlueBox)
virtual void execute(MachineState * ms) {

  if (ms->config.pilotClosestBlueBoxNumber == -1) {
    cout << "Not mapping closest bbox since it is " << ms->config.pilotClosestBlueBoxNumber << endl;
    return;
  } else {
    cout << "Mapping closest blue box, number " << ms->config.pilotClosestBlueBoxNumber << endl;
  }

  int c = ms->config.pilotClosestBlueBoxNumber;
  BoxMemory box;
  box.bTop = ms->config.bTops[c];
  box.bBot = ms->config.bBots[c];
  box.cameraPose = ms->config.currentEEPose;
  box.top = pixelToGlobalEEPose(ms, box.bTop.x, box.bTop.y, ms->config.trueEEPoseEEPose.pz + ms->config.currentTableZ);
  box.bot = pixelToGlobalEEPose(ms, box.bBot.x, box.bBot.y, ms->config.trueEEPoseEEPose.pz + ms->config.currentTableZ);
  box.centroid.px = (box.top.px + box.bot.px) * 0.5;
  box.centroid.py = (box.top.py + box.bot.py) * 0.5;
  box.centroid.pz = (box.top.pz + box.bot.pz) * 0.5;
  box.cameraTime = ros::Time::now();
  box.labeledClassIndex = ms->config.bLabels[c];
  box.lockStatus = CENTROID_LOCK;
  
  int i, j;
  mapxyToij(ms->config.mapXMin, ms->config.mapYMin, ms->config.mapStep, box.centroid.px, box.centroid.py, &i, &j);

  // this only does the timestamp to avoid obsessive behavior
  mapBox(ms, box);
  
  //if ( !positionIsSearched(box.centroid.px, box.centroid.py) && 
       //!isCellInPursuitZone(i, j) ) 
  //if (!positionIsSearched(box.centroid.px, box.centroid.py)) 
  if ( !positionIsSearched(ms->config.mapSearchFenceXMin, ms->config.mapSearchFenceXMax, ms->config.mapSearchFenceYMin, ms->config.mapSearchFenceYMax, box.centroid.px, box.centroid.py) || 
       !isBoxMemoryIkPossible(ms, box) ) 
  {
    return;
  } else {
    vector<BoxMemory> newMemories;
    for (int i = 0; i < ms->config.blueBoxMemories.size(); i++) {
      if (!boxMemoryIntersectCentroid(box, ms->config.blueBoxMemories[i])) {
	newMemories.push_back(ms->config.blueBoxMemories[i]);
      }
    }
    newMemories.push_back(box);
    ms->config.blueBoxMemories = newMemories;
  }
}
END_WORD
REGISTER_WORD(MapClosestBlueBox)


WORD(FilterBoxMemories)
virtual void execute(MachineState * ms) {
  set<int> boxMemoryIndexesToKeep;

  for (int b_i = 0; b_i < ms->config.blueBoxMemories.size(); b_i++) {
    int keep = 0;
    BoxMemory b = ms->config.blueBoxMemories[b_i];
    for (int i = 0; i < ms->config.mapWidth; i++) {
      for (int j = 0; j < ms->config.mapHeight; j++) {
        if (boxMemoryIntersectsMapCell(ms, b, i, j)) {
          //if (b.cameraTime.sec > ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime.sec) {
          ros::Duration diff = ms->config.objectMap[i + ms->config.mapWidth * j].lastMappedTime - b.cameraTime;

          if (diff < ros::Duration(2.0)) {
            boxMemoryIndexesToKeep.insert(b_i);
	    keep = 1;
          }
        }
      }
    }
    if (!keep) {
      cout << "filterBoxMemories rejecting box " << b_i << endl;
    }
  }

  vector<BoxMemory> newMemories;

  for (std::set<int>::iterator it=boxMemoryIndexesToKeep.begin(); it!=boxMemoryIndexesToKeep.end(); ++it) {
    newMemories.push_back(ms->config.blueBoxMemories[*it]);
  }
  ms->config.blueBoxMemories = newMemories;
}
END_WORD
REGISTER_WORD(FilterBoxMemories)

WORD(ClearBlueBoxMemories)
CODE(196709) // capslock + E
virtual void execute(MachineState * ms) {
  cout << "Clearing blue box memory: " << ms->config.blueBoxMemories.size() << endl;
  ms->config.blueBoxMemories.resize(0);
}
END_WORD
REGISTER_WORD(ClearBlueBoxMemories)


WORD(ResetTemporalMap)
CODE(1179737) // capslock + numlock + y
virtual void execute(MachineState * ms) {
  if (ms->config.temporalDensity != NULL && ms->config.preDensity != NULL) {
    //cout << "ms->config.preDensity<<<<***" << endl;
    Size sz = ms->config.objectViewerImage.size();
    int imW = sz.width;
    int imH = sz.height;
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
        ms->config.temporalDensity[y*imW+x] = ms->config.preDensity[y*imW+x];
      }
    }
  }
}
END_WORD
REGISTER_WORD(ResetTemporalMap)

WORD(GoFindBlueBoxes)
CODE(131122) // capslock + 2
virtual void execute(MachineState * ms) {
  goFindBlueBoxes(ms);
}
END_WORD
REGISTER_WORD(GoFindBlueBoxes)

WORD(GoClassifyBlueBoxes)
CODE(131123) // capslock + 3
virtual void execute(MachineState * ms) {
  ms->config.lastVisionCycle = ros::Time::now();
  ms->config.oscilStart = ros::Time::now();
  goClassifyBlueBoxes(ms);
}
END_WORD
REGISTER_WORD(GoClassifyBlueBoxes)

WORD(AssumeFacePose)
virtual void execute(MachineState * ms) {

  //eePose facePose = {.px = 1.07226, .py = 0.564963, .pz = 0.287997,
  //                   .qx = -0.234838, .qy = 0.75433, .qz = 0.106368, .qw = 0.603757};      
  eePose facePose = eePose(0.85838, 0.56957, 0.163187,
                           -0.153116, 0.717486, 0.0830483, 0.674442);

  ms->config.currentEEPose = facePose;
  ms->pushWord("waitUntilAtCurrentPosition");
}
END_WORD
REGISTER_WORD(AssumeFacePose)


WORD(MapWaypoints)
virtual string description() {
  return "Maps objects at locations specified by EePoseWords underneath.";
}
virtual void execute(MachineState * ms) {

  cout << "Entering mapWaypoints" << endl;

  eePose destPose;
  CONSUME_EEPOSE(destPose, ms);

  ms->pushWord("mapWaypoints");

  ms->pushWord("mapLocal");
  ms->pushWord("waitUntilAtCurrentPosition");
  ms->pushWord("shiftIntoGraspGear1");
  ms->pushWord("changeToHeight");
  ms->pushWord(std::make_shared<IntegerWord>(1));

  ms->pushWord("moveEeToPoseWord");
  ms->pushWord(std::make_shared<EePoseWord>(destPose));

}
END_WORD
REGISTER_WORD(MapWaypoints)

WORD(SetTrackbarLoHi)
virtual void execute(MachineState * ms)
{
  int newLo = 0;
  int newHi = 0;
  GET_ARG(ms, IntegerWord, newHi);
  GET_ARG(ms, IntegerWord, newLo);
  cout << "setTrackbarLoHi oldLo oldHi newLo newHi: " << ms->config.loTrackbarVariable << " " << ms->config.hiTrackbarVariable << " " << newLo << " " << newHi << endl;
  ms->config.loTrackbarVariable = newLo;
  ms->config.hiTrackbarVariable = newHi;
}
END_WORD
REGISTER_WORD(SetTrackbarLoHi)

}
