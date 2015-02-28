using namespace std;

class Word {
public:
  virtual void execute() = 0;
  virtual string name() = 0;
  virtual int character_code() = 0;
};

std::map<int, Word *> create_character_code_to_word(std::vector<Word *> words);
std::map<string, Word *> create_name_to_word(std::vector<Word *> words);
std::vector<Word *> create_words();
