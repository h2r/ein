
#include "ein_words.h"
#include "ein.h"
#include <boost/filesystem.hpp>

#ifdef USE_OPENCL
using namespace boost::filesystem;
int runFunction(int argc, char** argv) ;

namespace ein_words {

WORD(OpenClNbodyDemo)
virtual void execute(std::shared_ptr<MachineState> ms) {
  char * programName = "einOpenClNbodyDemo";
  runFunction(1, &programName);
}
END_WORD
REGISTER_WORD(OpenClNbodyDemo)

}
#endif
