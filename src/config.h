#ifndef _CONFIG_H_
#define _CONFIG_H_

typedef enum {
  ARMED,
  BLOCKED,
  STOPPED,
  HOVERING,
  MOVING
} movementState;


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

};

#endif /* _CONFIG_H_ */
