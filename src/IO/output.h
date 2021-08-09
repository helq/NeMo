#ifndef NEMO_OUTPUT_H
#define NEMO_OUTPUT_H

#include "../globals.h"

void initOutFiles();
void closeFiles();
void saveNeuronFire(tw_stime timestamp, id_type core, id_type local, tw_lpid destGID, long destCore,
                    long destLocal, unsigned int isOutput);
void saveNeuronFireDebug(tw_stime timestamp, id_type core, id_type local, tw_lpid destGID, long destCore,
                         long destLocal, unsigned int isOutput);

#endif
