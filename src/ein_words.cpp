
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <memory>

#include <object_recognition_msgs/RecognizedObjectArray.h>



#include "ein.h"

#include "ein_words.h"

std::vector<std::shared_ptr<Word> > words;



int register_word(std::shared_ptr<Word> word) {
  words.push_back(word);
  return 0;
}


// overall:  30s
// no words;  12s
// only vision, scanning, misc: 22s
// only bandit, render, movement, servo: 22s



