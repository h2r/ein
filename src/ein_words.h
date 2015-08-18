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
    cout << "Oops, GET_ARG_INT " << #x << " " << #ms << " found no argument..." << endl;\
    cout << "  Must pass an integer word as an argument to " << this->name() << endl;\
    cout << "  Pausing." << endl;\
    ms->pushWord("pauseStackExecution");\
    return;\
  } else {\
  }\
  std::shared_ptr<IntegerWord> hIntWord = std::dynamic_pointer_cast<IntegerWord>(hWord);\
\
  if (hIntWord == NULL) {\
    cout << "Oops, GET_ARG_INT " << #x << " " << #ms << " found an argument, but not an integer..." << endl;\
    cout << "  Must pass an integer word as an argument to " << this->name() << endl;\
    cout << "  Instead got word: " << hWord->name() << " repr: " << hWord->repr() << endl;\
    cout << "  Pausing." << endl;\
    ms->pushWord("pauseStackExecution");\
    return;\
  }\
  x =  hIntWord->value();\
}\

#define CONSUME_EEPOSE(x,ms) \
{\
  shared_ptr<Word> word = ms->popWord();\
  if (word == NULL) {\
    cout << "Stack empty, returning." << endl;\
    return;\
  } else {\
  }\
\
  std::shared_ptr<EePoseWord> destWord = std::dynamic_pointer_cast<EePoseWord>(word);\
  if (destWord == NULL) {\
    cout << "Must pass a pose as an argument to " << this->name() << endl;\
    cout << " Instead got word: " << word->name() << " repr: " << word->repr() << endl;\
    cout << " Pushing it back on the stack." << endl;\
    ms->pushWord(word);\
    return;\
  } else {\
  }\
\
  x = destWord->value();\
}\

#define GET_ARG(type,x,ms) \
{\
  shared_ptr<Word> hWord = ms->popWord();\
  if (hWord == NULL) {\
    cout << "Oops, GET_ARG " << #type << " " << #x << " " << #ms << " found no argument..." << endl;\
    cout << "  Must pass " << #type << " as an argument to " << this->name() << endl;\
    cout << "  Pausing." << endl;\
    ms->pushWord("pauseStackExecution");\
    return;\
  } else {\
  }\
  std::shared_ptr<type> hTypeWord = std::dynamic_pointer_cast<type>(hWord);\
\
  if (hTypeWord == NULL) {\
    cout << "Oops, GET_ARG " << #type << " " << #x << " " << #ms << " found an argument, but not " << #type << "..." << endl;\
    cout << "  Must pass " << #type << " as an argument to " << this->name() << endl;\
    cout << "  Instead got word: " << hWord->name() << " repr: " << hWord->repr() << endl;\
    cout << "  Pausing." << endl;\
    ms->pushWord("pauseStackExecution");\
    return;\
  }\
  x =  hTypeWord->value();\
}\

int register_word(shared_ptr<Word> word);


#endif /* _EIN_WORDS_H_ */
