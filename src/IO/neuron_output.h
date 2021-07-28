#ifndef NEMO_IO_NEURON_OUTPUT_H
#define NEMO_IO_NEURON_OUTPUT_H

#include "../neuro/tn_neuron_struct.h"

/** @defgroup spout Spike Output Functions @{ */
void openOutputFiles(char *outputFileName);
void initDataStructures(int simSize);
void closeOutputFiles();
void saveIndNeuron(void *n);
void saveNetworkStructure();
/** @} */

void saveNeuronNetworkStructure(void *n);
void saveNetworkStructureMPI();
void saveNeuronPreRun();

void debug_neuron_connections(struct NeuronState *n,tw_lp *lp);

#endif  // ifndef NEMO_IO_NEURON_OUTPUT_H
