#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "ein_util.h"

typedef enum {
  ARMED,
  BLOCKED,
  STOPPED,
  HOVERING,
  MOVING
} movementState;

typedef enum {
  SIFTBOW_GLOBALCOLOR_HIST = 1,
  OPPONENTSIFTBOW_GLOBALCOLOR_HIST = 2, // this has not been sufficiently tested
  SIFTCOLORBOW_HIST = 3, // unimplemented, calculate color histograms at each keypoint and augment each SIFT feature before clustering
  GRADIENT = 4,
  OPPONENT_COLOR_GRADIENT = 5,
  CBCR_HISTOGRAM = 6
} featureType;

typedef enum {
  PHYSICAL,
  SIMULATED
} robotMode;



class EinConfig {
 public:
  int zero_g_toggle = 0;
  int currentGraspGear = -1;
  const int imRingBufferSize = 300;
  const int epRingBufferSize = 100;
  const int rgRingBufferSize = 100;

  // we make use of a monotonicity assumption
  // if the current index passes the last recorded index, then we just proceed
  //  and lose the ranges we skipped over. alert when this happens
  // first valid entries
  int imRingBufferStart = 0;
  int epRingBufferStart = 0;
  int rgRingBufferStart = 0;
  
  // first free entries
  int imRingBufferEnd = 0;
  int epRingBufferEnd = 0;
  int rgRingBufferEnd = 0;

  movementState currentMovementState = STOPPED;

  // set color reticles iterator
  int scrI = 0;

  double currentGraspZ = 0;
  vector<double> classGraspZs;
  vector<double> classGraspZsSet;

  double movingThreshold = 0.02;
  double hoverThreshold = 0.003; 
  double stoppedTimeout = 0.25;


  featureType chosen_feature = SIFTBOW_GLOBALCOLOR_HIST;
  int cfi = 1;

  int gradientFeatureWidth = 50;

  robotMode chosen_mode = PHYSICAL;


  // config variables that don't seem to be used
  //double rapidAmp1 = 0.00; //0.3 is great
  //double rapidAmp1Delta = 0.01;

  //double rapidAmp2 = 0.00;
  //double rapidAmp2Delta = 0.03;

};

class Word;

class MachineState: public std::enable_shared_from_this<MachineState> {
 private:
 public:
  std::vector<std::shared_ptr<Word> > call_stack;
  std::shared_ptr<Word> current_instruction = NULL;
  EinConfig config;

  int execute_stack = 0;
  bool pushWord(int code);
  bool pushWord(string name);
  bool pushWord(std::shared_ptr<Word> word);
  std::shared_ptr<Word> popWord();
  void clearStack();
  void pushNoOps(int n);
  void pushCopies(int symbol, int times);
  void pushCopies(string symbol, int times);

  void execute(std::shared_ptr<Word> word);
};


#endif /* _CONFIG_H_ */
