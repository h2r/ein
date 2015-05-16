
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <memory>

#include "word.h"

#include "ein.h"



// Preprocessor macro to automatically create class structure for words
// extracted from the case statement of evil (despair)
// WORD(gName)
// // code here
// CODE(123)

std::vector<std::shared_ptr<Word> > words;


#define WORD(gName) \
class gName: public Word \
{ \
public: \
  virtual string name() { \
    string str = #gName; \
    str[0] = tolower(str[0]); \
    return str; \
  } 

#define CODE(code) \
  virtual int character_code() { \
    return code; \
  } 

#define END_WORD };

#define REGISTER_WORD(gName) \
  int gName ## _register = register_word(std::make_shared<gName>());


int register_word(std::shared_ptr<Word> word) {
  words.push_back(word);
  return 0;
}


// overall:  30s
// no words;  12s
// only vision, scanning, misc: 22s
// only bandit, render, movement, servo: 22s

#include "config.h"

namespace ein_words
{
#include "ein_bandit.cpp"
#include "ein_render.cpp"
#include "ein_movement.cpp"
#include "ein_servo.cpp"
#include "ein_vision_cycle.cpp"
#include "ein_scanning.cpp"
#include "ein_misc.cpp"
}


