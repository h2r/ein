#define PROGRAM_NAME "capture_object"






int trainOnly = 0;

int retrain_vocab = 0;
int rewrite_labels = 0;
int reextract_knn = 0;

int runInference = 0;
int publishObjects = 0;

int saveAnnotatedBoxes = 0;
int captureHardClass = 0;
int captureOnly = 1;
int saveBoxes = 1;

#include "node_main.cpp"
