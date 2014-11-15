#define PROGRAM_NAME "train_classifier"

#define TRAIN_ONLY
#define RUN_INFERENCE

// if you add more examples, you must reload the data for them to 
//   be taken into account.
#define RELOAD_DATA
// if you add a class or many new examples, it would be wise to
//   recalculate the vocab.
#define RELEARN_VOCAB
// similarly you must rewrite the labels.
#define REWRITE_LABELS

#include "node_main.cpp"
