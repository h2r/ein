#include "word.h"

void CompoundWord::execute(std::shared_ptr<MachineState> ms) {
  for (unsigned int i = 0; i < stack.size(); i++) {
    ms->pushWord(stack[i]);
  }
}

std::map<int, std::shared_ptr<Word> > character_code_to_word;
std::map<string, std::shared_ptr<Word> > name_to_word;

std::map<int, std::shared_ptr<Word> > create_character_code_to_word(std::vector<std::shared_ptr<Word> > words) {
  std::map<int, std::shared_ptr<Word> > character_code_to_word;
  for (unsigned int i = 0; i < words.size(); i++) {
    if (words[i]->character_code() != -1) {
      if (character_code_to_word.count(words[i]->character_code()) > 0) {
        cout << "Two words with the same code." << endl;
        cout << "Word 1: " << character_code_to_word[words[i]->character_code()]->name() << endl;
        cout << "Word 2: " << words[i]->name() << endl;
        cout << "Code: " << words[i]->character_code() << endl;
        assert(0);
      } else {
        character_code_to_word[words[i]->character_code()] = words[i];
      }
    }
  }
  return character_code_to_word;
}

std::map<string, std::shared_ptr<Word> > create_name_to_word(std::vector<std::shared_ptr<Word> > words) {
  std::map<string, std::shared_ptr<Word> > name_to_word;
  for (unsigned int i = 0; i < words.size(); i++) {
    vector<string> names = words[i]->names();
    for (unsigned int j = 0; j < names.size(); j++) {
      name_to_word[names[j]] = words[i];      
    }
  }
  return name_to_word;
}




bool MachineState::pushWord(int code) {
  if (character_code_to_word.count(code) > 0) {
    std::shared_ptr<Word> word = character_code_to_word[code];      
    return pushWord(word);
  } else {
    cout << "No word for code " << code << endl;
    return false;
  }
}

bool MachineState::pushWord(string token) {
  std:shared_ptr<Word> word = forthletParse(token);
  cout << "Pushing token " << token << " as " << word << endl;
  if (word != NULL) {
    return pushWord(word);
  }
}


bool MachineState::pushWord(std::shared_ptr<Word> word) {
  cout << "Pushing " << word->name() << endl;
  call_stack.push_back(word);
  return true;
}

void MachineState::clearStack() {
  call_stack.resize(0);
}

std::shared_ptr<Word> MachineState::popWord() {
  if (call_stack.size() > 0) {
    std::shared_ptr<Word> word = call_stack.back();
    call_stack.pop_back();
    return word; 
  } else {
    return NULL;
  }
}

void MachineState::pushNoOps(int n) {
  for (int i = 0; i < n; i++)
    pushWord("noop"); 
}

void MachineState::pushCopies(int symbol, int times) {
  for (int i = 0; i < times; i++) {
    pushWord(symbol); 
  }
}

void MachineState::pushCopies(string symbol, int times) {
  for (int i = 0; i < times; i++) {
    pushWord(symbol); 
  }
}




std::shared_ptr<Word> forthletParse(string token) {
  if (IntegerWord::isInteger(token)) {
    return IntegerWord::parse(token);
  } else if (StringWord::isString(token)) {
    return StringWord::parse(token);
  } else if (name_to_word.count(token) > 0) {
    std::shared_ptr<Word> word = name_to_word[token];
    return word;
  } else if (SymbolWord::isSymbol(token)) {
    std::shared_ptr<Word> word = SymbolWord::parse(token);
    return word;
  } else {
    cout << "Cannot parse " << token << endl;
    return NULL;
  }
}


std::shared_ptr<Word> nameToWord(string name) {
  if (name_to_word.count(name) > 0) {
    std::shared_ptr<Word> word = name_to_word[name];
    return word;
  } else  {
    cout << "Could not find word with name " << name << endl;
    assert(0);
  }
}



void initializeWords() {
  cout << "Making words: " << words.size() << endl;
  character_code_to_word = create_character_code_to_word(words);
  name_to_word = create_name_to_word(words);
}
