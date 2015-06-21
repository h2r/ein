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
  if (word != NULL) {
    return pushWord(word);
  }
}


bool MachineState::pushWord(std::shared_ptr<Word> word) {
  call_stack.push_back(word);
  return true;
}

void MachineState::clearStack() {
  call_stack.resize(0);
}

void MachineState::execute(shared_ptr<Word> word) {
  if (word != NULL) {
    current_instruction = word;
    word->execute(shared_from_this());
  }
}

shared_ptr<Word> MachineState::popWord() {
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

void MachineState::pushCopies(std::shared_ptr<Word> word, int times) {
  for (int i = 0; i < times; i++) {
    pushWord(word); 
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


void renderCoreView(shared_ptr<MachineState> ms) {

  Mat coreImage(800, 800, CV_64F);
  coreImage = 0.0*coreImage;

  cv::Scalar dataColor(192,192,192);
  cv::Scalar labelColor(160,160,160);

  cv::Point ciAnchor(10,50);
  putText(coreImage, "Current Registers: ", ciAnchor, MY_FONT, 0.5, labelColor, 1.0);

  char buf[256];
  cv::Point lAnchor(170,50);
  string lText = "";
  lText += "CI: ";
  if (ms->current_instruction != NULL) {
    lText += ms->current_instruction->name();
  } else {
    lText += "NULL";
  }
  putText(coreImage, lText, lAnchor, MY_FONT, 0.5, dataColor, 2.0);

  lAnchor.y += 20;
  lText = "";
  lText += "ZG: ";
  sprintf(buf, "%d", ms->config.zero_g_toggle);
  lText += buf;
  lText += " GG: ";
  sprintf(buf, "%d", ms->config.currentGraspGear);
  lText += buf;
  putText(coreImage, lText, lAnchor, MY_FONT, 0.5, dataColor, 2.0);

  lAnchor.y += 20;
  lText = "";
  lText += "rgRB: ";
  sprintf(buf, "%+.02d/%d", ms->config.rgRingBufferEnd-ms->config.rgRingBufferStart, ms->config.rgRingBufferSize);
  lText += buf;
  lText += " epRB: ";
  sprintf(buf, "%+.02d/%d", ms->config.epRingBufferEnd-ms->config.epRingBufferStart, ms->config.epRingBufferSize);
  lText += buf;
  lText += " imRB: ";
  sprintf(buf, "%+.02d/%d", ms->config.imRingBufferEnd-ms->config.imRingBufferStart, ms->config.imRingBufferSize);
  lText += buf;
  putText(coreImage, lText, lAnchor, MY_FONT, 0.5, dataColor, 2.0);

  int stackRowY = lAnchor.y + 40; 
  cv::Point csAnchor(10,stackRowY);
  putText(coreImage, "Call Stack: ", csAnchor, MY_FONT, 0.5, labelColor, 1.0);
  
  int instructionsPerRow = 1;
  int rowAnchorStep = 25;
  cv::Point rowAnchor(120,stackRowY);
  int insCount = 0; 

  int numCommandsToShow = 400;
  int lowerBound = max(int(ms->call_stack.size() - numCommandsToShow), 0);
  insCount = lowerBound;

  while (insCount < ms->call_stack.size()) {
    string outRowText;

    for (int rowCount = 0; (insCount < ms->call_stack.size()) && (rowCount < instructionsPerRow); insCount++, rowCount++) {
      outRowText += ms->call_stack[max(int(ms->call_stack.size() - (insCount - lowerBound) - 1),0)]->name();
      outRowText += " ";
    }

    putText(coreImage, outRowText, rowAnchor, MY_FONT, 0.5, dataColor, 2.0);
    rowAnchor.y += rowAnchorStep;
    if (rowAnchor.y >= coreImage.rows) {
      break;
    }
  }

//  {
//    cv::Point text_anchor = cv::Point(0,coreImage.rows-1);
//    {
//      cv::Scalar backColor(0,0,0);
//      cv::Point outTop = cv::Point(text_anchor.x,text_anchor.y+1-35);
//      cv::Point outBot = cv::Point(text_anchor.x+200,text_anchor.y+1);
//      Mat vCrop = coreImage(cv::Rect(outTop.x, outTop.y, outBot.x-outTop.x, outBot.y-outTop.y));
//      vCrop = backColor;
//    }
//    char buff[256];
//    sprintf(buff, "Hz: %.2f", aveFrequency);
//    string fpslabel(buff);
//    putText(coreImage, fpslabel, text_anchor, MY_FONT, 1.0, Scalar(0,0,160), 1.0);
//  }
   ms->config.coreViewImage = coreImage;
}
