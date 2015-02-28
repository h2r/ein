class PauseWord: public Word 
{
public:
  virtual void execute() {
    if (auto_pilot || (holding_pattern != 0)) {
      pilot_call_stack.push_back('C');
    } else {
      holding_pattern = 0;
      auto_pilot = 0;
      go_on_lock = 0;
    }
  }
  virtual string name() {
    return "pause";
  }
  virtual int character_code() {
    return 'C';
  }
};

class ZUpWord: public Word
{
public:
  virtual void execute() {
    currentEEPose.pz += bDelta;
  }
  virtual string name() {
    return "zup";
  }

  virtual int character_code() {
    return 'w';
  }
};

class ZDownWord: public Word
{
public:
  virtual void execute() {
    currentEEPose.pz -= bDelta;
  }
  virtual string name() {
    return "zdown";
  }

  virtual int character_code() {
    return 's';
  }
};

std::vector<Word *> create_words() {
  std::vector<Word *> words;
  words.push_back(new ZUpWord());
  words.push_back(new ZDownWord());
  words.push_back(new PauseWord());
  return words;
}

std::map<int, Word *> create_character_code_to_word(std::vector<Word *> words) {
  std::map<int, Word *> character_code_to_word;
  for (unsigned int i = 0; i < words.size(); i++) {
    character_code_to_word[words[i]->character_code()] = words[i];
  }
  return character_code_to_word;
}

std::map<string, Word *> create_name_to_word(std::vector<Word *> words) {
  std::map<string, Word *> name_to_word;
  for (unsigned int i = 0; i < words.size(); i++) {
    name_to_word[words[i]->name()] = words[i];
  }
  return name_to_word;
}


