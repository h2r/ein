#define PROGRAM_NAME "train_classifier"
// if you add a class or many new examples, it would be wise to
//   recalculate the vocab.
// similarly you must rewrite the labels.
// if you add more examples, you must reload the data for them to 
//   be taken into account.

int trainOnly = 1;

int retrain_vocab = 1;
int rewrite_labels = 1;
int reextract_knn = 1;

int runInference = 1;
int publishObjects = 0;

int saveAnnotatedBoxes = 0;
int captureHardClass = 0;
int captureOnly = 0;
int saveBoxes = 0;

#include "node_main.cpp"
