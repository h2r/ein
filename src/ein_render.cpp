
#include "ein_words.h"
#include "ein.h"
#include "qtgui/einwindow.h"


namespace ein_words {

WORD(PaintReticles)
CODE(1048679)     // numlock + g
virtual void execute(std::shared_ptr<MachineState> ms)       {
  double ddrX = (ms->config.drX)/ms->config.rmDelta;
  double ddrY = (ms->config.drY)/ms->config.rmDelta;
  double ttrX = (ms->config.trX-ms->config.rmcX)/ms->config.rmDelta;
  double ttrY = (ms->config.trY-ms->config.rmcY)/ms->config.rmDelta;
  if ((fabs(ddrX) <= ms->config.rmHalfWidth) && (fabs(ddrY) <= ms->config.rmHalfWidth)) {
    int iiX = (int)round(ddrX + ms->config.rmHalfWidth);
    int iiY = (int)round(ddrY + ms->config.rmHalfWidth);

    double intensity = 128;
    cv::Scalar backColor(ceil(intensity),0,0);
    cv::Point outTop = cv::Point((iiY+ms->config.rmWidth)*ms->config.rmiCellWidth,iiX*ms->config.rmiCellWidth);
    cv::Point outBot = cv::Point(((iiY+ms->config.rmWidth)+1)*ms->config.rmiCellWidth,(iiX+1)*ms->config.rmiCellWidth);
    Mat vCrop = ms->config.rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
    vCrop += backColor;
  }
  if ((fabs(ttrX) <= ms->config.rmHalfWidth) && (fabs(ttrY) <= ms->config.rmHalfWidth)) {
    int iiX = (int)round(ttrX + ms->config.rmHalfWidth);
    int iiY = (int)round(ttrY + ms->config.rmHalfWidth);

    double intensity = 128;
    cv::Scalar backColor(0,ceil(intensity),0);
    cv::Point outTop = cv::Point((iiY+ms->config.rmWidth)*ms->config.rmiCellWidth,iiX*ms->config.rmiCellWidth);
    cv::Point outBot = cv::Point(((iiY+ms->config.rmWidth)+1)*ms->config.rmiCellWidth,(iiX+1)*ms->config.rmiCellWidth);
    Mat vCrop = ms->config.rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
    vCrop += backColor;

    cv::Point text_anchor = cv::Point(outTop.x+4, outBot.y-4);
    char buff[256];
    sprintf(buff, "%d", ms->config.maxGG+1);
    string reticleLabel(buff);
    putText(ms->config.rangemapImage, reticleLabel, text_anchor, MY_FONT, 0.5, Scalar(192,192,192), 1.0);
  }
  for (int gg = 0; gg < ms->config.totalGraspGears; gg++){
    double gggX = (ms->config.ggX[gg])/ms->config.rmDelta;
    double gggY = (ms->config.ggY[gg])/ms->config.rmDelta;
    if ((fabs(gggX) <= ms->config.rmHalfWidth) && (fabs(gggY) <= ms->config.rmHalfWidth)) {
      int iiX = (int)round(gggX + ms->config.rmHalfWidth);
      int iiY = (int)round(gggY + ms->config.rmHalfWidth);

      cv::Point outTop = cv::Point((iiY+ms->config.rmWidth)*ms->config.rmiCellWidth,iiX*ms->config.rmiCellWidth);
      cv::Point outBot = cv::Point(((iiY+ms->config.rmWidth)+1)*ms->config.rmiCellWidth-1,(iiX+1)*ms->config.rmiCellWidth-1);
      cv::Point inTop = cv::Point(outTop.x+1, outTop.y+1);
      cv::Point inBot = cv::Point(outBot.x-1, outBot.y-1);
      rectangle(ms->config.rangemapImage, outTop, outBot, cv::Scalar(192,0,0)); 
      rectangle(ms->config.rangemapImage, inTop, inBot, cv::Scalar(64,0,0)); 

      cv::Point text_anchor = cv::Point(outTop.x+4, outBot.y-4);
      char buff[256];
      sprintf(buff, "%d", gg+1);
      string reticleLabel(buff);
      putText(ms->config.rangemapImage, reticleLabel, text_anchor, MY_FONT, 0.5, Scalar(192,192,192), 1.0);
    }
  }
  double httrX = (ms->config.trX-ms->config.rmcX)/ms->config.hrmDelta;
  double httrY = (ms->config.trY-ms->config.rmcY)/ms->config.hrmDelta;
  int hiCellWidth = 5;
  if ((fabs(httrX) <= ms->config.hrmHalfWidth-hiCellWidth) && (fabs(httrY) <= ms->config.hrmHalfWidth-hiCellWidth)) {
    int hiiX = (int)round(httrX + ms->config.hrmHalfWidth);
    int hiiY = (int)round(httrY + ms->config.hrmHalfWidth);

    double intensity = 128;
    cv::Scalar backColor(0,ceil(intensity),0);
    cv::Point l1p1 = cv::Point(hiiY+ms->config.hrmWidth-hiCellWidth,hiiX);
    cv::Point l1p2 = cv::Point((hiiY+ms->config.hrmWidth)+hiCellWidth,hiiX);
    cv::Point l2p1 = cv::Point(hiiY+ms->config.hrmWidth,hiiX-hiCellWidth);
    cv::Point l2p2 = cv::Point((hiiY+ms->config.hrmWidth),hiiX+hiCellWidth);
    line(ms->config.hiRangemapImage, l1p1, l1p2, backColor);
    line(ms->config.hiRangemapImage, l2p1, l2p2, backColor);
  }
  double cttrX = ms->config.curseReticleX - ms->config.hrmHalfWidth;
  double cttrY = ms->config.curseReticleY - ms->config.hrmHalfWidth;
  if ((fabs(cttrX) <= ms->config.hrmHalfWidth-hiCellWidth) && (fabs(cttrY) <= ms->config.hrmHalfWidth-hiCellWidth)) {
    int ciiX = (int)round(cttrX + ms->config.hrmHalfWidth);
    int ciiY = (int)round(cttrY + ms->config.hrmHalfWidth);

    double intensity = 128;
    cv::Scalar backColor(0,ceil(intensity),ceil(intensity));
    cv::Point l1p1 = cv::Point(ciiY+ms->config.hrmWidth-hiCellWidth,ciiX);
    cv::Point l1p2 = cv::Point((ciiY+ms->config.hrmWidth)+hiCellWidth,ciiX);
    cv::Point l2p1 = cv::Point(ciiY+ms->config.hrmWidth,ciiX-hiCellWidth);
    cv::Point l2p2 = cv::Point((ciiY+ms->config.hrmWidth),ciiX+hiCellWidth);
    line(ms->config.hiRangemapImage, l1p1, l1p2, backColor);
    line(ms->config.hiRangemapImage, l2p1, l2p2, backColor);
#ifdef DEBUG4
    cout << "printing curseReticle xy globalz: " << ms->config.curseReticleX << " " << ms->config.curseReticleY << " " << ms->config.hiRangeMap[ciiX + ciiY*ms->config.hrmWidth] << endl;
#endif
  }
  {
    double intensity = 128;
    cv::Scalar backColor(0,ceil(intensity),0);
    circle(ms->config.hiRangemapImage, cv::Point(ms->config.hrmHalfWidth+ms->config.hrmWidth, ms->config.hrmHalfWidth), hiCellWidth, backColor);
  }
  int localCenterMaxX = ms->config.localMaxX-ms->config.rmHalfWidth;
  int localCenterMaxY = ms->config.localMaxY-ms->config.rmHalfWidth;
  cout << "localMaxX localMaxY localCenterMaxX localCenterMaxY: " << ms->config.localMaxX << " " << ms->config.localMaxY << " " << localCenterMaxX << " " << localCenterMaxY << endl;
  if ((fabs(localCenterMaxX) <= ms->config.rmHalfWidth) && (fabs(localCenterMaxY) <= ms->config.rmHalfWidth)) {
    int liiX = (int)round(localCenterMaxX + ms->config.rmHalfWidth);
    int liiY = (int)round(localCenterMaxY + ms->config.rmHalfWidth);

    double intensity = 255;
    cv::Scalar backColor(ceil(intensity),ceil(intensity),0);
    cv::Point outTop = cv::Point((liiY+ms->config.rmWidth)*ms->config.rmiCellWidth,liiX*ms->config.rmiCellWidth);
    cv::Point outBot = cv::Point(((liiY+ms->config.rmWidth)+1)*ms->config.rmiCellWidth,(liiX+1)*ms->config.rmiCellWidth);
    Mat vCrop = ms->config.graspMemoryImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
    vCrop += backColor;

    cv::Point text_anchor = cv::Point(outTop.x+4, outBot.y-4);
    char buff[256];
    sprintf(buff, "%d", ms->config.localMaxGG + 1);
    string reticleLabel(buff);
    putText(ms->config.graspMemoryImage, reticleLabel, text_anchor, MY_FONT, 0.5, Scalar(64,64,192), 1.0);
  }
}
END_WORD
REGISTER_WORD(PaintReticles)


WORD(FullRender)
CODE(1114183)     // numlock + G 
virtual void execute(std::shared_ptr<MachineState> ms) {
  if (!ms->config.shouldIRender) {
    ms->pushWord(1114177); // manual render
  }
  ms->pushWord("paintReticles"); // render reticle
  ms->pushWord("drawMapRegisters"); 
  ms->pushWord("downsampleIrScan"); // load map to register 1
}
END_WORD
REGISTER_WORD(FullRender)

WORD(DrawMapRegisters)
// numlock + a 
CODE(1048673)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  drawMapRegisters(ms);
}
END_WORD
REGISTER_WORD(DrawMapRegisters)

WORD(GuiShowAll)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  destroyAllWindows();
  ms->config.rangeogramWindow->setVisible(true);

  ms->config.sirRangemap = 1;
  ms->config.sirGraspMemory = 1;
  ms->config.sirGraspMemorySample = 1;
  ms->config.sirHeightMemorySample = 1;
  ms->config.sirHiRangemap = 1;
  ms->config.sirHiColorRangemap = 1;
  ms->config.sirObject = 1;
  ms->config.sirObjectMap = 1;
  ms->config.sirDensity = 1;
  ms->config.sirGradient = 1;
  ms->config.sirObjectness = 1;
  ms->config.sirMapBackground = 1;
  ms->config.sirAerialGradient = 1;
  ms->config.sirWrist = 1;
  ms->config.sirCore = 1;
  cv::namedWindow(ms->config.objectViewerName);
  cv::namedWindow(ms->config.graspMemoryViewName);
  cv::namedWindow(ms->config.wristViewName);
  cv::setMouseCallback(ms->config.wristViewName, pilotCallbackFunc, NULL);
  cv::setMouseCallback(ms->config.graspMemoryViewName, graspMemoryCallbackFunc, NULL);
}
END_WORD
REGISTER_WORD(GuiShowAll)

WORD(GuiHideAll)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  destroyAllWindows();
  ms->config.rangeogramWindow->setVisible(false);

  ms->config.sirRangemap = 0;
  ms->config.sirGraspMemory = 0;
  ms->config.sirGraspMemorySample = 0;
  ms->config.sirHeightMemorySample = 0;
  ms->config.sirHiRangemap = 0;
  ms->config.sirHiColorRangemap = 0;
  ms->config.sirObject = 0;
  ms->config.sirObjectMap = 0;
  ms->config.sirDensity = 0;
  ms->config.sirGradient = 0;
  ms->config.sirObjectness = 0;
  ms->config.sirMapBackground = 0;
  ms->config.sirAerialGradient = 0;
  ms->config.sirWrist = 0;
  ms->config.sirCore = 0;
}
END_WORD
REGISTER_WORD(GuiHideAll)

WORD(GuiCustom1)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ROS_WARN_STREAM("___________________");
  ROS_ERROR_STREAM("Called word guiCustom1.");
  ROS_WARN_STREAM("___________________");
  destroyAllWindows();
  ms->config.rangeogramWindow->setVisible(false);

  ms->config.sirRangemap = 0;
  ms->config.sirGraspMemory = 0;
  ms->config.sirGraspMemorySample = 0;
  ms->config.sirHeightMemorySample = 0;
  ms->config.sirHiRangemap = 0;
  ms->config.sirHiColorRangemap = 0;
  ms->config.sirObject = 1;
  ms->config.sirObjectMap = 1;
  ms->config.sirDensity = 0;
  ms->config.sirGradient = 1;
  ms->config.sirObjectness = 0;
  ms->config.sirMapBackground = 0;
  ms->config.sirAerialGradient = 0;
  ms->config.sirWrist = 1;
  ms->config.sirCore = 1;

  //cv::namedWindow(ms->config.objectViewerName);
  //cv::namedWindow(ms->config.graspMemoryViewName);
  cv::namedWindow(ms->config.wristViewName);
  cv::setMouseCallback(ms->config.wristViewName, pilotCallbackFunc, NULL);
}
END_WORD
REGISTER_WORD(GuiCustom1)



}
