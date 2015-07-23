#ifndef _EIN_WORDS_H_
#define _EIN_WORDS_H_

#include <vector>
#include <string>
#include <memory>
#include "word.h"


using namespace std;

extern vector<shared_ptr<Word> > words;


// Preprocessor macro to automatically create class structure for words
// extracted from the case statement of evil (despair)
// WORD(gName)
// // code here
// CODE(123)


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
  int gName ## _register = register_word(make_shared<gName>());

#define GET_ARG_INT(x,ms) \
{\
  shared_ptr<Word> hWord = ms->popWord();\
  if (hWord == NULL) {\
    cout << "oops, GET_ARG_INT " << #x << " " << #ms << " found no argument..." << endl;\
    ms->clearStack();\
    return;\
  } else {\
  }\
  std::shared_ptr<IntegerWord> hIntWord = std::dynamic_pointer_cast<IntegerWord>(hWord);\
  x =  hIntWord->value();\
}\


int register_word(shared_ptr<Word> word);


#endif /* _EIN_WORDS_H_ */
