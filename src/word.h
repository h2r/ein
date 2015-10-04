#ifndef _WORD_H_
#define _WORD_H_

#include "ein_util.h"

using namespace std;
#include <sstream>
#include <map>
#include <boost/algorithm/string.hpp>


class MachineState;

using namespace boost::algorithm;

class Word:  public enable_shared_from_this<Word> {
  
public:

  virtual void execute(std::shared_ptr<MachineState> ms);
  virtual string name() = 0;

  virtual string description() {
    return "No help for this word.";
  }

  virtual vector<string> names() {
    vector<string> result;
    result.push_back(name());
    return result;
  }

  virtual bool is_value() {
    return false;
  }
     

  virtual int character_code() {
    return -1;
  }

  virtual bool equals(std::shared_ptr<Word> word) {
    if (word.get() == this) {
      return true;
    } else {
      return false;
    }
  }

  virtual bool to_bool() {
    return true;
  }
  virtual int to_int() {
    return 1;
  }

  /**
   * Returns it as a string in the repl (e.g., quotes, back ticks for strings and symbols)
   */
  virtual string repr() {
    return name();
  }

  /**
   * Converts to a string (e.g., no quotes)
   */
  virtual string to_string() {
    return name();
  }

};

class DoubleWord: public Word
{
private:
  double n;

public:

  double value() {
    return n;
  }

  virtual bool is_value() {
    return true;
  }

  static std::shared_ptr<DoubleWord> parse(string token);

  virtual bool is_static() {
    return false;
  }
  static bool isDouble(string token) {
    try {
      parse(token);
      return true;
    } catch (...) {
      return false;
    }
  }
  
  DoubleWord(double _n) {
    n = _n;
  }
  string name() {
    stringstream ss;
    ss << scientific;
    ss << n;
    return ss.str();
  }

  bool equals(shared_ptr<Word> word) {
    shared_ptr<DoubleWord> w1 = dynamic_pointer_cast<DoubleWord>(word);
    if (w1 == NULL) {
      return false;
    } else {
      return w1->value() == this->value();
    }
  }
  
  virtual bool to_bool() {
    if (n == 0) {
      return false;
    } else {
      return true;
    }
  }
  virtual double to_double() {
    return n;
  }
};


class IntegerWord: public Word
{
private:
  int n;

public:

  int value() {
    return n;
  }

  virtual bool is_value() {
    return true;
  }

  static std::shared_ptr<IntegerWord> parse(string token);
  virtual bool is_static() {
    return false;
  }
  static bool isInteger(string token) {
    try {
      parse(token);
      return true;
    } catch (...) {
      return false;
    }
  }
  
  IntegerWord(int _n) {
    n = _n;
  }
  string name() {
    stringstream ss;
    ss << n;
    return ss.str();
  }

  bool equals(shared_ptr<Word> word) {
    shared_ptr<IntegerWord> w1 = dynamic_pointer_cast<IntegerWord>(word);
    if (w1 == NULL) {
      return false;
    } else {
      return w1->value() == this->value();
    }
  }
  
  virtual bool to_bool() {
    if (n == 0) {
      return false;
    } else {
      return true;
    }
  }
  virtual int to_int() {
    return n;
  }
};


class StringWord: public Word
{
private:
  string s;

public:

  string value() {
    return s;
  }

  virtual bool is_value() {
    return true;
  }

  virtual bool is_static() {
    return false;
  }

  static std::shared_ptr<StringWord> parse(string token) {
    if (token[0] == '\"' && token[token.size() - 1] == '\"') {
      return std::make_shared<StringWord>(token.substr(1, token.size() - 2));      
    } else {
      return NULL;
    }

  }
  static bool isString(string token) {
    try {
      if (parse(token) != NULL) {
        return true;
      } else {
        return false;
      }
    } catch (...) {
      return false;
    }
  }
  
  StringWord(string _s) {
    s = _s;
  }

  string name() {
    stringstream ss;
    ss << "\"" << s << "\"";
    return ss.str();
  }

  string to_string() {
    return s;
  }

  bool equals(shared_ptr<Word> word);

  
};


class CommentWord: public Word
{
private:
  string s;

public:

  string value() {
    return s;
  }

  virtual bool is_value() {
    return true;
  }

  virtual bool is_static() {
    return false;
  }

  virtual void execute(std::shared_ptr<MachineState> ms) {
    
  }

  static std::shared_ptr<CommentWord> parse(string token) {
    if (token.size() < 4) {
      return NULL;
    }
    if (token[0] == '/' && token[1] == '*' && token[token.size() - 2] == '*'  && token[token.size() - 1] == '/') {
      return std::make_shared<CommentWord>(token.substr(2, token.size() - 4));      
    } else {
      return NULL;
    }

  }
  static bool isComment(string token) {
    try {
      if (parse(token) != NULL) {
        return true;
      } else {
        return false;
      }
    } catch (...) {
      return false;
    }
  }
  
  CommentWord(string _s) {
    s = _s;
  }

  string name() {
    stringstream ss;
    ss << "/*" << s << "*/";
    return ss.str();
  }

  string to_string() {
    return s;
  }
  
};




class SymbolWord: public Word
{
private:
  string s;

public:

  virtual bool is_value() {
    return true;
  }

  virtual bool is_static() {
    return false;
  }

  static std::shared_ptr<SymbolWord> parse(string token) {
    if (token.size() != 0) {
      return std::make_shared<SymbolWord>(token);
    } else {
      return NULL;
    }
  }

  static bool isSymbol(string token);
  
  SymbolWord(string _s) {
    s = _s;
  }
  virtual void execute(std::shared_ptr<MachineState> ms);

  string name() {
    stringstream ss;
    ss << "'" << s;
    return ss.str();
  }

  string to_string() {
    return s;
  }
  
};


class EePoseWord: public Word
{
private:
  eePose pose;

public:
  eePose value() {
    return pose;
  }

  virtual bool is_value() {
    return true;
  }

  static std::shared_ptr<EePoseWord> parse(string token) {
    
    eePose pose;
    return std::make_shared<EePoseWord>(pose);
  }
  virtual bool is_static() {
    return false;
  }
  static bool isInteger(string token) {
    if (token.substr(0,5) == "eePose") {
      return true;
    } else {
      return false;
    }
  }
  
  EePoseWord(eePose _pose) {
    pose = _pose;
  }

  virtual string repr();

  string name() {
    stringstream ss;
    ss << pose;
    return ss.str();
  }

  bool equals(shared_ptr<Word> word) {
    shared_ptr<EePoseWord> w1 = dynamic_pointer_cast<EePoseWord>(word);
    if (w1 == NULL) {
      return false;
    } else {
      return w1->value().equals(this->value());
    }
  }
  
  virtual bool to_bool() {
    return true;
  }
  virtual int to_int() {
    return 1;
  }
};




class CompoundWord : public Word {
 private:
  vector<std::shared_ptr<Word> > stack;
 public:
  CompoundWord() {
  }
  void pushWord(shared_ptr<Word> word) {
    stack.push_back(word);
  }
  void pushWord(shared_ptr<MachineState> ms, string word);

  int size();
  shared_ptr<Word> popWord();
  shared_ptr<Word> getWord(int i);

  virtual void execute(std::shared_ptr<MachineState> ms);
  string repr();
  string name();
  string to_string();
};


std::map<int, std::shared_ptr<Word> > create_character_code_to_word(std::vector<std::shared_ptr<Word> > words);
std::map<string, std::shared_ptr<Word> > create_name_to_word(std::vector<std::shared_ptr<Word> > words);

std::shared_ptr<Word> parseToken(std::shared_ptr<MachineState> ms, string token);
std::shared_ptr<Word> nameToWord(string name);

extern std::vector<std::shared_ptr<Word> > words;
extern std::map<int, std::shared_ptr<Word> > character_code_to_word;
extern std::map<string, std::shared_ptr<Word> > name_to_word;

void initializeWords();

void renderCoreView(shared_ptr<MachineState> ms);

/*
WORD()
virtual void execute(std::shared_ptr<MachineState> ms)
{
    ms->pushWord("");
}
END_WORD
REGISTER_WORD()
*/

#endif /* _WORD_H_ */
