#include <assert.h>
#include <iostream>
#include <vector>
#include <exception>
#include <sstream>
#include <stdexcept>

using namespace std;


class Word {
public:
  virtual void execute() = 0;
  virtual string to_string() = 0;
};


vector<Word *> stack;


void push(Word * word) {
  stack.push_back(word);
  word->execute();
}

Word * pop() {
  Word * result = stack.back();
  stack.pop_back();
  return result;
}



class Integer: public Word
{
private:
public:
  int n;

  static Integer * parse(string token) {
    return new Integer(stoi(token));
  }
  static bool isInteger(string token) {
    try {
      parse(token);
      return true;
    } catch (...) {
      return false;
    }
  }
  
  Integer(int _n) {
    n = _n;
  }
  void execute() {
  }
  string to_string() {
    stringstream ss;
    ss << n;
    return ss.str();
  }
};

class Plus: public Word
{
public: 
  static Plus * parse(string token) {
    if (token == "+") {
      return new Plus();
    } else {
      throw runtime_error("not +");
    }
  }
  static bool isPlus(string token) {
    try {
      parse(token);
      return true;
    } catch (...) {
      return false;
    }
  }

  virtual void execute() {
    pop(); // pop yourself
    Integer * n1 = (Integer *) pop();
    Integer * n2 = (Integer *) pop();

    Integer * result = new Integer(n1->n + n2->n);
    push(result);
  }
  virtual string to_string() {
    return "+";
  }
};


class Minus: public Word
{
public: 
  static Minus * parse(string token) {
    if (token == "-") {
      return new Minus();
    } else {
      throw runtime_error("not -");
    }
  }
  static bool isMinus(string token) {
    try {
      parse(token);
      return true;
    } catch (...) {
      return false;
    }
  }

  virtual void execute() {
    pop(); // pop yourself
    Integer * n1 = (Integer *) pop();
    Integer * n2 = (Integer *) pop();

    Integer * result = new Integer(n1->n - n2->n);
    push(result);
  }
  virtual string to_string() {
    return "-";
  }
};

vector<string> split(const char *str, char c = ' ')
{
    vector<string> result;
    do
    {
      const char *begin = str;
      
      while(*str != c && *str)
        str++;
      
      result.push_back(string(begin, str));
    } while (0 != *str++);
    
    return result;
}

void printStack() {
  cout << "Stack: " << endl;
  for (int i = 0; i < stack.size(); i++) {
    cout << i << ": ";
    cout << stack[i]->to_string() << endl;
  }

}

Word * parse(string token) {
  if (Integer::isInteger(token)) {
    return Integer::parse(token);
  }
  if (Plus::isPlus(token)) {
    return Plus::parse(token);
  }
  if (Minus::isMinus(token)) {
    return Minus::parse(token);
  }
  throw runtime_error("No value for token: " + token);
}

string read() {
  string line;
  cout << ">>> ";
  getline(cin, line);
  cout << "ok" << endl;
  vector<string> tokens = split(line.c_str(), ' ');
  for (int i = 0; i < tokens.size(); i++) {
    Word * word = parse(tokens[i]);
    push(word);
  }
  printStack();

  return line;
}

int main(int argc, char **argv) {

  cout << "Hello world." << endl;

  while (true) {
    string line = read();
  }
}
