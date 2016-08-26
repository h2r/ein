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

#define CONSUME_EEPOSE(x,ms) \
{\
  shared_ptr<Word> word = ms->popData();\
  if (word == NULL) {\
    CONSOLE_ERROR(ms, "Stack empty, returning.");   \
    return;\
  } else {\
    if (word->name().compare("endArgs") == 0) {\
      CONSOLE_ERROR(ms, " found endArgs, discarding and returning.");       \
      return;\
    } else {\
    }\
  }\
\
  std::shared_ptr<EePoseWord> destWord = std::dynamic_pointer_cast<EePoseWord>(word);\
  if (destWord == NULL) {\
    CONSOLE_ERROR(ms, "Must pass a pose as an argument to " << this->name()); \
    CONSOLE_ERROR(ms, " Instead got word: " << word->name() << " repr: " << word->repr()); \
    CONSOLE_ERROR(ms, " Pushing it back on the stack.");                    \
    ms->pushWord(word);\
    return;\
  } else {\
  }\
\
  x = destWord->value();\
}\

#define GET_ARG(ms,type,x) \
{\
  shared_ptr<Word> hWord = ms->popData();\
  if (hWord == NULL) {\
    CONSOLE_ERROR(ms, "Oops, GET_ARG " << #type << " " << #x << " found no argument..."); \
    CONSOLE_ERROR(ms, "  Must pass " << #type << " as an argument to " << this->name()); \
    ms->pushWord("pauseStackExecution");\
    return;\
  } else {\
  }\
  std::shared_ptr<type> hTypeWord = std::dynamic_pointer_cast<type>(hWord);\
\
  if (hTypeWord == NULL) {\
    CONSOLE_ERROR(ms, "Oops, GET_ARG " << #type << " " << #x << " found an argument, but not " << #type << "..."); \
    CONSOLE_ERROR(ms, "  Must pass " << #type << " as an argument to " << this->name()); \
    CONSOLE_ERROR(ms, "  Instead got word: " << hWord->name() << " repr: " << hWord->repr()); \
    ms->pushWord("pauseStackExecution");\
    CONSOLE_ERROR(ms, "Pushing the bad word back on the data stack.");      \
    ms->pushData(hWord);\
    return;\
  }\
  x =  hTypeWord->value();\
}\


#define GET_NUMERIC_ARG(ms,x) \
{\
  shared_ptr<Word> hWord = ms->popData();\
  if (hWord == NULL) {\
    CONSOLE_ERROR(ms, "Oops, GET_NUMERIC_ARG " << " " << #x << " found no argument..."); \
    CONSOLE_ERROR(ms, "  Must pass a numeric argument to " << this->name()); \
    ms->pushWord("pauseStackExecution");\
    return;\
  } else {\
  std::shared_ptr<IntegerWord> intWord = std::dynamic_pointer_cast<IntegerWord>(hWord);\
\
  if (intWord != NULL) {\
    x = intWord->value();			\
  } else { \
    std::shared_ptr<DoubleWord> doubleWord = std::dynamic_pointer_cast<DoubleWord>(hWord);\
    if (doubleWord != NULL) {\
      x =  doubleWord->value();\
    } else { \
      CONSOLE_ERROR(ms, "Oops, GET_NUMERIC_ARG " << #x << " found an argument, but not a number..."); \
      CONSOLE_ERROR(ms, "  Must pass a number as an argument to " << this->name()); \
      CONSOLE_ERROR(ms, "  Instead got word: " << hWord->name() << " repr: " << hWord->repr()); \
      ms->pushWord("pauseStackExecution");	\
      CONSOLE_ERROR(ms, "Pushing the bad word back on the data stack.");    \
      ms->pushData(hWord);\
      return;\
    }	     \
  }	     \
  }	     \
   }




#define GET_BOOLEAN_ARG(ms,x) \
{\
  shared_ptr<Word> hWord = ms->popData();\
  if (hWord == NULL) {\
  CONSOLE_ERROR(ms, "Oops, GET_BOOLEAN_ARG " << " " << #x << " found no argument..."); \
  CONSOLE_ERROR(ms, "  Must pass a boolean argument to " << this->name());  \
  ms->pushWord("pauseStackExecution");                                  \
  return;                                                               \
  }  \
  x =  hWord->to_bool();			\
}

#define GET_STRING_ARG(ms,x) \
{\
  shared_ptr<Word> hWord = ms->popData();\
  if (hWord == NULL) {\
    CONSOLE_ERROR(ms, "Oops, GET_STRING_ARG " << " " << #x << " " << #ms << " found no argument..."); \
    CONSOLE_ERROR(ms, "  Must pass a string argument to " << this->name()); \
    CONSOLE_ERROR(ms, "  Pausing.");                                    \
    ms->pushWord("pauseStackExecution");\
    return;\
  }  \
  x =  hWord->to_string();			\
 }


#define GET_INT_ARG(ms,x) \
{\
  shared_ptr<Word> hWord = ms->popData();\
  if (hWord == NULL) {\
    CONSOLE_ERROR(ms, "Oops, GET_INT_ARG " << " " << #x <<  " found no argument..."); \
    CONSOLE_ERROR(ms, "  Must pass a numeric argument to " << this->name()); \
    ms->pushWord("pauseStackExecution");\
    return;\
  }  \
  x =  hWord->to_int();			\
 }



#define GET_WORD_ARG(ms,type,x) \
{\
  shared_ptr<Word> hWord = ms->popData();\
  if (hWord == NULL) {\
    CONSOLE_ERROR(ms, "Oops, GET_WORD_ARG " << #type << " " << #x << " found no argument..."); \
    CONSOLE_ERROR(ms, "  Must pass " << #type << " as an argument to " << this->name()); \
    ms->pushWord("pauseStackExecution");\
    return;\
  } else {\
  }\
  x = std::dynamic_pointer_cast<type>(hWord);\
\
  if (x == NULL) {\
    CONSOLE_ERROR(ms, "Oops, GET_WORD_ARG " << #type << " " << #x << " " << #ms << " found an argument, but not " << #type << "..."); \
    CONSOLE_ERROR(ms, "  Must pass " << #type << " as an argument to " << this->name()); \
    CONSOLE_ERROR(ms, "  Instead got word: " << hWord->name() << " repr: " << hWord->repr()); \
    ms->pushWord("pauseStackExecution");\
    CONSOLE_ERROR(ms, "Pushing the bad word back on the data stack.");      \
    ms->pushData(hWord);\
    return;\
  }\
}\

#define CONFIG_GETTER_INT(backName, configName)	\
WORD(backName) \
virtual void execute(MachineState * ms) { \
  ms->pushWord(make_shared<IntegerWord>(configName)); \
} \
END_WORD \
REGISTER_WORD(backName) 

#define CONFIG_SETTER_INT(backName, configName)	\
WORD(backName) \
virtual void execute(MachineState * ms) { \
  int value; \
  GET_INT_ARG(ms, value); \
  configName = value;	\
} \
END_WORD \
REGISTER_WORD(backName) 


#define CONFIG_SETTER_ENUM(backName, configName, cast)	\
WORD(backName) \
virtual void execute(MachineState * ms) { \
  int value; \
  GET_INT_ARG(ms, value); \
  configName = cast value;	\
} \
END_WORD \
REGISTER_WORD(backName) 

#define CONFIG_GETTER_DOUBLE(backName, configName)	\
WORD(backName) \
virtual void execute(MachineState * ms) { \
  ms->pushWord(make_shared<DoubleWord>(configName)); \
} \
END_WORD \
REGISTER_WORD(backName) 

#define CONFIG_SETTER_DOUBLE(backName, configName)	\
WORD(backName) \
virtual void execute(MachineState * ms) { \
  double value; \
  GET_NUMERIC_ARG(ms, value); \
  configName = value;	\
} \
END_WORD \
REGISTER_WORD(backName) 

#define CONFIG_GETTER_STRING(backName, configName)	\
WORD(backName) \
virtual void execute(MachineState * ms) { \
  ms->pushWord(make_shared<StringWord>(configName)); \
} \
END_WORD \
REGISTER_WORD(backName) 

#define CONFIG_SETTER_STRING(backName, configName)	\
WORD(backName) \
virtual void execute(MachineState * ms) { \
  string value; \
  GET_ARG(ms, StringWord, value); \
  configName = value;	\
} \
END_WORD \
REGISTER_WORD(backName) 

#define CONFIG_GETTER_POSE(backName, configName)	\
WORD(backName) \
virtual void execute(MachineState * ms) { \
  ms->pushWord(make_shared<EePoseWord>(configName)); \
} \
END_WORD \
REGISTER_WORD(backName) 

#define CONFIG_SETTER_POSE(backName, configName)	\
WORD(backName) \
virtual void execute(MachineState * ms) { \
  eePose value; \
  GET_ARG(ms, EePoseWord, value); \
  configName = value;	\
} \
END_WORD \
REGISTER_WORD(backName) 




#define REQUIRE_FOCUSED_CLASS(ms, tfc) \
int tfc = ms->config.focusedClass;\
if ( (tfc > -1) && (tfc < ms->config.classLabels.size()) ) {\
} else {\
  CONSOLE_ERROR(ms, this->name() << ": Invalid focused class, not grabbing..."); \
  return;\
}\

#define REQUIRE_VALID_CLASS(ms, tfc) \
if ( (tfc > -1) && (tfc < ms->config.classLabels.size()) ) {\
} else {\
  CONSOLE_ERROR(ms, "Invalid focused class, not grabbing..."); \
  return;\
}\

#define REQUIRE_VALID_SCENE_OBJECT(ms, tfc) \
if ( (tfc > -1) && (tfc < ms->config.scene->predicted_objects.size()) ) {\
} else {\
  CONSOLE_ERROR(ms, this->name() << ": Invalid scene object, class, not grabbing..."); \
  return;\
}\


#define GET_WORD_ARG_VALUE_LIST(ms,type,x)\
{\
  int hMore_args = 1;\
  while(hMore_args) {\
    shared_ptr<Word> hWord = ms->popData();\
\
    if (hWord == NULL) {\
      CONSOLE_ERROR(ms, "oops, GET_WORD_ARG_VALUE_LIST expects endArgs");   \
      ms->clearStack();\
      return;\
\
      CONSOLE_ERROR(ms, "Oops, GET_WORD_ARG_VALUE_LIST " << #type << " " << #x << " found no argument..."); \
      CONSOLE_ERROR(ms, "  Must pass: endArgs ( list of " << #type << " ) as an argument to " << this->name()); \
      ms->pushWord("pauseStackExecution");\
      return;\
    } else {\
    }\
\
    if (hWord->name().compare("endArgs") == 0) {\
      cout << " found endArgs" << endl;\
      hMore_args = 0;\
      break;\
    } else {\
    }\
\
    std::shared_ptr<type> hTypeWord = std::dynamic_pointer_cast<type>(hWord);\
\
    if (hTypeWord == NULL) {\
      CONSOLE_ERROR(ms, "Oops, GET_WORD_ARG_VALUE_LIST " << #type << " " << #x << " " << #ms << " found an argument, but not " << #type << "..."); \
      CONSOLE_ERROR(ms, "  Must pass " << #type << " as an argument to " << this->name()); \
      CONSOLE_ERROR(ms, "  Instead got word: " << hWord->name() << " repr: " << hWord->repr()); \
      ms->pushWord("pauseStackExecution");\
      CONSOLE_ERROR(ms, "Pushing the bad word back on the data stack.");    \
      ms->pushData(hWord);\
      return;\
    } else {\
      x.push_back(hTypeWord->value());\
    }\
  }\
}\
\

#define GET_WORD_ARG_LIST(ms,type,x)\
{\
  int hMore_args = 1;\
  while(hMore_args) {\
    shared_ptr<Word> hWord = ms->popData();\
\
    if (hWord == NULL) {\
      CONSOLE_ERROR(ms, "oops, GET_WORD_ARG_LIST expects endArgs"); \
      ms->clearStack();\
      return;\
\
      CONSOLE_ERROR(ms, "Oops, GET_WORD_ARG_LIST " << #type << " " << #x << " found no argument..."); \
      CONSOLE_ERROR(ms, "  Must pass: endArgs ( list of " << #type << " ) as an argument to " << this->name()); \
      ms->pushWord("pauseStackExecution");\
      return;\
    } else {\
    }\
\
    if (hWord->name().compare("endArgs") == 0) {\
      hMore_args = 0;\
      break;\
    } else {\
    }\
\
    std::shared_ptr<type> hTypeWord = std::dynamic_pointer_cast<type>(hWord);\
\
    if (hTypeWord == NULL) {\
      CONSOLE_ERROR(ms, "Oops, GET_WORD_ARG_LIST " << #type << " " << #x << " " << #ms << " found an argument, but not " << #type << "..."); \
      CONSOLE_ERROR(ms, "  Must pass " << #type << " as an argument to " << this->name()); \
      CONSOLE_ERROR(ms, "  Instead got word: " << hWord->name() << " repr: " << hWord->repr()); \
      ms->pushWord("pauseStackExecution");\
      CONSOLE_ERROR(ms, "Pushing the bad word back on the data stack.");    \
      ms->pushData(hWord);\
      return;\
    } else {\
      x.push_back(hTypeWord);\
    }\
  }\
}\
\

#define GET_WORD_AS_STRING(ms,x)\
  std::shared_ptr<Word> word = ms->popData();\
  std::shared_ptr<StringWord> s = std::dynamic_pointer_cast<StringWord>(word);\
  if (s != NULL) {\
    x = s->value();\
  } else if (word != NULL) {\
    x = word->repr();\
  } else {\
    x = "";\
  }

int register_word(shared_ptr<Word> word);


#endif /* _EIN_WORDS_H_ */
