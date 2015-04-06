
WORD(PaintReticles)
CODE(1048679)     // numlock + g
virtual void execute()       {
  double ddrX = (drX)/rmDelta;
  double ddrY = (drY)/rmDelta;
  double ttrX = (trX-rmcX)/rmDelta;
  double ttrY = (trY-rmcY)/rmDelta;
  if ((fabs(ddrX) <= rmHalfWidth) && (fabs(ddrY) <= rmHalfWidth)) {
    int iiX = (int)round(ddrX + rmHalfWidth);
    int iiY = (int)round(ddrY + rmHalfWidth);

    double intensity = 128;
    cv::Scalar backColor(ceil(intensity),0,0);
    cv::Point outTop = cv::Point((iiY+rmWidth)*rmiCellWidth,iiX*rmiCellWidth);
    cv::Point outBot = cv::Point(((iiY+rmWidth)+1)*rmiCellWidth,(iiX+1)*rmiCellWidth);
    Mat vCrop = rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
    vCrop += backColor;
  }
  if ((fabs(ttrX) <= rmHalfWidth) && (fabs(ttrY) <= rmHalfWidth)) {
    int iiX = (int)round(ttrX + rmHalfWidth);
    int iiY = (int)round(ttrY + rmHalfWidth);

    double intensity = 128;
    cv::Scalar backColor(0,ceil(intensity),0);
    cv::Point outTop = cv::Point((iiY+rmWidth)*rmiCellWidth,iiX*rmiCellWidth);
    cv::Point outBot = cv::Point(((iiY+rmWidth)+1)*rmiCellWidth,(iiX+1)*rmiCellWidth);
    Mat vCrop = rangemapImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
    vCrop += backColor;

    cv::Point text_anchor = cv::Point(outTop.x+4, outBot.y-4);
    char buff[256];
    sprintf(buff, "%d", maxGG+1);
    string reticleLabel(buff);
    putText(rangemapImage, reticleLabel, text_anchor, MY_FONT, 0.5, Scalar(192,192,192), 1.0);
  }
  for (int gg = 0; gg < totalGraspGears; gg++){
    double gggX = (ggX[gg])/rmDelta;
    double gggY = (ggY[gg])/rmDelta;
    if ((fabs(gggX) <= rmHalfWidth) && (fabs(gggY) <= rmHalfWidth)) {
      int iiX = (int)round(gggX + rmHalfWidth);
      int iiY = (int)round(gggY + rmHalfWidth);

      cv::Point outTop = cv::Point((iiY+rmWidth)*rmiCellWidth,iiX*rmiCellWidth);
      cv::Point outBot = cv::Point(((iiY+rmWidth)+1)*rmiCellWidth-1,(iiX+1)*rmiCellWidth-1);
      cv::Point inTop = cv::Point(outTop.x+1, outTop.y+1);
      cv::Point inBot = cv::Point(outBot.x-1, outBot.y-1);
      rectangle(rangemapImage, outTop, outBot, cv::Scalar(192,0,0)); 
      rectangle(rangemapImage, inTop, inBot, cv::Scalar(64,0,0)); 

      cv::Point text_anchor = cv::Point(outTop.x+4, outBot.y-4);
      char buff[256];
      sprintf(buff, "%d", gg+1);
      string reticleLabel(buff);
      putText(rangemapImage, reticleLabel, text_anchor, MY_FONT, 0.5, Scalar(192,192,192), 1.0);
    }
  }
  double httrX = (trX-rmcX)/hrmDelta;
  double httrY = (trY-rmcY)/hrmDelta;
  int hiCellWidth = 5;
  if ((fabs(httrX) <= hrmHalfWidth-hiCellWidth) && (fabs(httrY) <= hrmHalfWidth-hiCellWidth)) {
    int hiiX = (int)round(httrX + hrmHalfWidth);
    int hiiY = (int)round(httrY + hrmHalfWidth);

    double intensity = 128;
    cv::Scalar backColor(0,ceil(intensity),0);
    cv::Point l1p1 = cv::Point(hiiY+hrmWidth-hiCellWidth,hiiX);
    cv::Point l1p2 = cv::Point((hiiY+hrmWidth)+hiCellWidth,hiiX);
    cv::Point l2p1 = cv::Point(hiiY+hrmWidth,hiiX-hiCellWidth);
    cv::Point l2p2 = cv::Point((hiiY+hrmWidth),hiiX+hiCellWidth);
    line(hiRangemapImage, l1p1, l1p2, backColor);
    line(hiRangemapImage, l2p1, l2p2, backColor);
  }
  double cttrX = curseReticleX - hrmHalfWidth;
  double cttrY = curseReticleY - hrmHalfWidth;
  if ((fabs(cttrX) <= hrmHalfWidth-hiCellWidth) && (fabs(cttrY) <= hrmHalfWidth-hiCellWidth)) {
    int ciiX = (int)round(cttrX + hrmHalfWidth);
    int ciiY = (int)round(cttrY + hrmHalfWidth);

    double intensity = 128;
    cv::Scalar backColor(0,ceil(intensity),ceil(intensity));
    cv::Point l1p1 = cv::Point(ciiY+hrmWidth-hiCellWidth,ciiX);
    cv::Point l1p2 = cv::Point((ciiY+hrmWidth)+hiCellWidth,ciiX);
    cv::Point l2p1 = cv::Point(ciiY+hrmWidth,ciiX-hiCellWidth);
    cv::Point l2p2 = cv::Point((ciiY+hrmWidth),ciiX+hiCellWidth);
    line(hiRangemapImage, l1p1, l1p2, backColor);
    line(hiRangemapImage, l2p1, l2p2, backColor);
#ifdef DEBUG4
    cout << "printing curseReticle xy globalz: " << curseReticleX << " " << curseReticleY << " " << hiRangeMap[ciiX + ciiY*hrmWidth] << endl;
#endif
  }
  {
    double intensity = 128;
    cv::Scalar backColor(0,ceil(intensity),0);
    circle(hiRangemapImage, cv::Point(hrmHalfWidth+hrmWidth, hrmHalfWidth), hiCellWidth, backColor);
  }
  int localCenterMaxX = localMaxX-rmHalfWidth;
  int localCenterMaxY = localMaxY-rmHalfWidth;
  cout << "localMaxX localMaxY localCenterMaxX localCenterMaxY: " << localMaxX << " " << localMaxY << " " << localCenterMaxX << " " << localCenterMaxY << endl;
  if ((fabs(localCenterMaxX) <= rmHalfWidth) && (fabs(localCenterMaxY) <= rmHalfWidth)) {
    int liiX = (int)round(localCenterMaxX + rmHalfWidth);
    int liiY = (int)round(localCenterMaxY + rmHalfWidth);

    double intensity = 255;
    cv::Scalar backColor(ceil(intensity),ceil(intensity),0);
    cv::Point outTop = cv::Point((liiY+rmWidth)*rmiCellWidth,liiX*rmiCellWidth);
    cv::Point outBot = cv::Point(((liiY+rmWidth)+1)*rmiCellWidth,(liiX+1)*rmiCellWidth);
    Mat vCrop = graspMemoryImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
    vCrop += backColor;

    cv::Point text_anchor = cv::Point(outTop.x+4, outBot.y-4);
    char buff[256];
    sprintf(buff, "%d", localMaxGG + 1);
    string reticleLabel(buff);
    putText(graspMemoryImage, reticleLabel, text_anchor, MY_FONT, 0.5, Scalar(64,64,192), 1.0);
  }
}
END_WORD



WORD(FullRender)
CODE(1114183)     // numlock + G 
virtual void execute() {
  if (!shouldIRender) {
    pushWord(1114177); // manual render
  }
  pushWord("paintReticles"); // render reticle
  pushWord("drawMapRegisters"); 
  pushWord("downsampleIrScan"); // load map to register 1
}
END_WORD


WORD(DrawMapRegisters)
// numlock + a 
CODE(1048673)
virtual void execute()
{
  drawMapRegisters();
}
END_WORD

WORD(GuiShowAll)
virtual void execute()
{
  destroyAllWindows();
  sirRangeogram = 1;
  sirRangemap = 1;
  sirGraspMemory = 1;
  sirGraspMemorySample = 1;
  sirHeightMemorySample = 1;
  sirHiRangmap = 1;
  sirHiColorRangemap = 1;
  sirObject = 1;
  sirObjectMap = 1;
  sirDensity = 1;
  sirGradient = 1;
  sirObjectness = 1;
  sirMapBackground = 1;
  sirAerialGradient = 1;
  sirWrist = 1;
  sirCore = 1;
  cv::namedWindow(objectViewerName);
  cv::namedWindow(graspMemoryViewName);
  cv::namedWindow(wristViewName);
  cv::setMouseCallback(wristViewName, pilotCallbackFunc, NULL);
  cv::setMouseCallback(graspMemoryViewName, graspMemoryCallbackFunc, NULL);
  cv::setMouseCallback(objectViewerName, nodeCallbackFunc, NULL);
}
END_WORD

WORD(GuiHideAll)
virtual void execute()
{
  destroyAllWindows();
  sirRangeogram = 0;
  sirRangemap = 0;
  sirGraspMemory = 0;
  sirGraspMemorySample = 0;
  sirHeightMemorySample = 0;
  sirHiRangmap = 0;
  sirHiColorRangemap = 0;
  sirObject = 0;
  sirObjectMap = 0;
  sirDensity = 0;
  sirGradient = 0;
  sirObjectness = 0;
  sirMapBackground = 0;
  sirAerialGradient = 0;
  sirWrist = 0;
  sirCore = 0;
}
END_WORD

WORD(GuiCustom1)
virtual void execute()
{
  ROS_WARN_STREAM("___________________");
  ROS_ERROR_STREAM("Called word guiCustom1.");
  ROS_WARN_STREAM("___________________");
  destroyAllWindows();
  sirRangeogram = 0;
  sirRangemap = 0;
  sirGraspMemory = 0;
  sirGraspMemorySample = 0;
  sirHeightMemorySample = 0;
  sirHiRangmap = 0;
  sirHiColorRangemap = 0;
  sirObject = 0;
  sirObjectMap = 1;
  sirDensity = 0;
  sirGradient = 0;
  sirObjectness = 0;
  sirMapBackground = 0;
  sirAerialGradient = 0;
  sirWrist = 1;
  sirCore = 0;

  //cv::namedWindow(objectViewerName);
  //cv::namedWindow(graspMemoryViewName);
  cv::namedWindow(wristViewName);
  cv::setMouseCallback(wristViewName, pilotCallbackFunc, NULL);
}
END_WORD




