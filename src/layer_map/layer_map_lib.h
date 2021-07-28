#ifndef LAYER_MAP_LIBRARY_H
#define LAYER_MAP_LIBRARY_H

#include "../neuro/tn_neuron_struct.h"

enum LayerTypes {
  NON_LAYER = 0,
  GRID_LAYER = 1 << 1,
  CONVOLUTIONAL_LAYER = 1 << 2,
  OUTPUT_UNQ = 1 << 3,
  OUTPUT_RND = 1 << 4
};

extern unsigned int NUM_LAYERS_IN_SIM;
extern enum LayerTypes LAYER_NET_MODE;
extern unsigned int CORES_PER_LAYER;
extern unsigned int CHIPS_PER_LAYER;
extern unsigned int GRID_ENABLE;
extern unsigned int GRID_MODE;
extern unsigned int RND_GRID;
extern unsigned int RND_UNIQ;

void setupGrid(int showMapping);
void configureNeuronInLayer(tn_neuron_state *s, tw_lp *lp);
void displayConfig();

#endif // LAYER_MAP_LIBRARY_H
