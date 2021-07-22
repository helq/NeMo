#ifndef LAYER_MAP_LIBRARY_H
#define LAYER_MAP_LIBRARY_H

#include "../neuro/tn_neuron_struct.h"

void setupGrid(int showMapping);
void configureNeuronInLayer(tn_neuron_state *s, tw_lp *lp);
void displayConfig();

#endif // LAYER_MAP_LIBRARY_H
