#define PROGRAM_NAME "publish_detections"






int trainOnly = 0;

int retrain_vocab = 0;
int rewrite_labels = 0;
int reextract_knn = 0;

int runInference = 1;
int publishObjects = 1;

int saveAnnotatedBoxes = 0;
int captureHardClass = 0;
int captureOnly = 0;
int saveBoxes = 0;

#include "node_main.cpp"
