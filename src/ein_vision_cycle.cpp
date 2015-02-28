WORD(VisionCycle)
CODE(131153)  // capslock + q
virtual void execute() {
  pilot_call_stack.push_back(131123); // classify
  pilot_call_stack.push_back(131122); // blue boxes
  pushCopies(131121, 4); // density
  pushCopies(1179737, 1); // reset temporal map
  pushCopies(131121, 1); // density
}
END_WORD

WORD(Density)
CODE(131121)     // capslock + 1
virtual void execute() {
  goCalculateDensity();
  goCalculateObjectness();
}
END_WORD


WORD(ResetTemporalMap)
CODE(1179737) // capslock + numlock + y
virtual void execute() {
  if (temporalDensity != NULL && preDensity != NULL) {
    //cout << "preDensity<<<<***" << endl;
    Size sz = objectViewerImage.size();
    int imW = sz.width;
    int imH = sz.height;
    for (int x = 0; x < imW; x++) {
      for (int y = 0; y < imH; y++) {
        temporalDensity[y*imW+x] = preDensity[y*imW+x];
      }
    }
  }
}
END_WORD

WORD(GoFindBlueBoxes)
CODE(131122) // capslock + 2
virtual void execute() {
  goFindBlueBoxes();
}
END_WORD


WORD(GoClassifyBlueBoxes)
CODE(131123) // capslock + 3
virtual void execute() {
  lastVisionCycle = ros::Time::now();
  oscilStart = ros::Time::now();
  goClassifyBlueBoxes();
}
END_WORD
