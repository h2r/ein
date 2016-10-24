#ifndef _GAUSSIAN_MAP_WORDS_H_
#define _GAUSSIAN_MAP_WORDS_H_

#include "gaussian_map.h"
#include "word.h"

class GaussianMapWord: public Word
{
  //private:


public:

  shared_ptr<GaussianMap> map;
  GaussianMapWord(shared_ptr<GaussianMap> map);
  string name() {
    stringstream ss;
    ss << "GaussianMap";
    return ss.str();
  }

  string to_string() {
    return name();
  }
};

#endif /* _GAUSSIAN_MAP_WORDS_H_ */
