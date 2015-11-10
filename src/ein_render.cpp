
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
    //ms->pushWord(1114177); // manual render
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
  ms->config.rangeogramWindow->setVisible(true);

  ms->config.rangemapWindow->setVisible(true);

  ms->config.graspMemoryWindow->setVisible(true);

  ms->config.graspMemorySampleWindow->setVisible(true);
  ms->config.heightMemorySampleWindow->setVisible(true);

  ms->config.hiRangemapWindow->setVisible(true);
  ms->config.hiColorRangemapWindow->setVisible(true);

  ms->config.objectViewerWindow->setVisible(true);

  ms->config.objectMapViewerWindow->setVisible(true);

  ms->config.densityViewerWindow->setVisible(true);
  ms->config.gradientViewerWindow->setVisible(true);

  ms->config.mapBackgroundViewWindow->setVisible(true);
  ms->config.aerialGradientViewerWindow->setVisible(true);

  ms->config.wristViewWindow->setVisible(true);
  ms->config.coreViewWindow->setVisible(true);
}
END_WORD
REGISTER_WORD(GuiShowAll)

WORD(GuiHideAll)
virtual void execute(std::shared_ptr<MachineState> ms)
{

  ms->config.rangeogramWindow->setVisible(false);

  ms->config.rangeogramWindow->setVisible(false);

  ms->config.rangemapWindow->setVisible(false);

  ms->config.graspMemoryWindow->setVisible(false);

  ms->config.graspMemorySampleWindow->setVisible(false);
  ms->config.heightMemorySampleWindow->setVisible(false);

  ms->config.hiRangemapWindow->setVisible(false);
  ms->config.hiColorRangemapWindow->setVisible(false);

  ms->config.objectViewerWindow->setVisible(false);

  ms->config.objectMapViewerWindow->setVisible(false);

  ms->config.densityViewerWindow->setVisible(false);
  ms->config.gradientViewerWindow->setVisible(false);

  ms->config.mapBackgroundViewWindow->setVisible(false);
  ms->config.aerialGradientViewerWindow->setVisible(false);

  ms->config.wristViewWindow->setVisible(false);
  ms->config.coreViewWindow->setVisible(false);
}
END_WORD
REGISTER_WORD(GuiHideAll)

WORD(GuiCustom1)
virtual void execute(std::shared_ptr<MachineState> ms)
{
  ROS_WARN_STREAM("___________________");
  ROS_ERROR_STREAM("Called word guiCustom1.");
  ROS_WARN_STREAM("___________________");
  ms->config.rangeogramWindow->setVisible(false);


  ms->config.rangemapWindow->setVisible(false);
  ms->config.graspMemoryWindow->setVisible(false);
  ms->config.graspMemorySampleWindow->setVisible(false);
  ms->config.heightMemorySampleWindow->setVisible(false);
  ms->config.hiRangemapWindow->setVisible(false);
  ms->config.hiColorRangemapWindow->setVisible(false);
  ms->config.objectViewerWindow->setVisible(true);
  ms->config.objectMapViewerWindow->setVisible(false);
  ms->config.densityViewerWindow->setVisible(false);
  ms->config.gradientViewerWindow->setVisible(true);
  ms->config.mapBackgroundViewWindow->setVisible(false);
  ms->config.aerialGradientViewerWindow->setVisible(false);
  ms->config.wristViewWindow->setVisible(false);
  ms->config.coreViewWindow->setVisible(false);

  //ms->config.gripperMaskFirstContrastWindow->setVisible(true);
  //ms->config.gripperMaskSecondContrastWindow->setVisible(true);
  //ms->config.gripperMaskDifferenceWindow->setVisible(true);
  //ms->config.gripperMaskVarianceWindow->setVisible(true);
  //ms->config.gripperMaskMeanWindow->setVisible(true);
  //ms->config.gripperMaskSquaresWindow->setVisible(true);


  
}

END_WORD
REGISTER_WORD(GuiCustom1)

WORD(PublishWristViewToFace)
virtual void execute(std::shared_ptr<MachineState> ms) {
  Size toBecome(1024,600);
  sensor_msgs::Image msg;

  Mat toresize = ms->config.wristViewImage.clone();
  Mat topub;
  cv::resize(toresize, topub, toBecome);

  msg.header.stamp = ros::Time::now();
  msg.width = topub.cols;
  msg.height = topub.rows;
  msg.step = topub.cols * topub.elemSize();
  msg.is_bigendian = false;
  msg.encoding = sensor_msgs::image_encodings::BGR8;
  msg.data.assign(topub.data, topub.data + size_t(topub.rows * msg.step));
  ms->config.face_screen_pub.publish(msg);
}
END_WORD
REGISTER_WORD(PublishWristViewToFace)

WORD(PublishImageFileToFace)
virtual void execute(std::shared_ptr<MachineState> ms) {
  string imfilename_post;
  GET_STRING_ARG(ms, imfilename_post);
  
  string imfilename = "src/ein/images/" + imfilename_post;
  Mat topub = imread(imfilename);

  if (isSketchyMat(topub)) {
    cout << "publishImageFileToFace: cannot load file " << imfilename << endl;
    return;
  } else {
    sensor_msgs::Image msg;
    msg.header.stamp = ros::Time::now();
    msg.width = topub.cols;
    msg.height = topub.rows;
    msg.step = topub.cols * topub.elemSize();
    msg.is_bigendian = false;
    msg.encoding = sensor_msgs::image_encodings::BGR8;
    msg.data.assign(topub.data, topub.data + size_t(topub.rows * msg.step));
    ms->config.face_screen_pub.publish(msg);
  }
}
END_WORD
REGISTER_WORD(PublishImageFileToFace)


WORD(BlankFace)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::stringstream program;
  program << "\"black.tif\" publishImageFileToFace";
  ms->evaluateProgram(program.str());  
}
END_WORD
REGISTER_WORD(BlankFace)


WORD(HappyFace)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::stringstream program;
  program << "\"ursula_yes.tif\" publishImageFileToFace";
  ms->evaluateProgram(program.str());  
}
END_WORD
REGISTER_WORD(HappyFace)


WORD(SadFace)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::stringstream program;
  program << "\"ursula_no.tif\" publishImageFileToFace";
  ms->evaluateProgram(program.str());  
}
END_WORD
REGISTER_WORD(SadFace)


WORD(NeutralFace)
virtual void execute(std::shared_ptr<MachineState> ms) {
  std::stringstream program;
  program << "\"ursula_neutral.tif\" publishImageFileToFace";
  ms->evaluateProgram(program.str());  
}
END_WORD
REGISTER_WORD(NeutralFace)




}
