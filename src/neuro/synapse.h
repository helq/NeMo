// Created by Mark Plagge on 5/25/16.
//

#ifndef NEMO_SYNAPSE_H
#define NEMO_SYNAPSE_H
#include "../globals.h"
#include "../message.h"

typedef struct SynapseState {
  uint64_t msgSent;
  tw_stime lastBigTick;
  id_type myCore;
  tw_bf neuronBF[NEURONS_IN_CORE];
  unsigned long randCount[NEURONS_IN_CORE];
  bool connectionGrid[NEURONS_IN_CORE][NEURONS_IN_CORE];
} synapseState;

void synapse_init(synapseState *s, tw_lp *lp);
void synapse_event(synapseState *s, tw_bf *bf, struct messageData *M, tw_lp *lp);
void synapse_reverse(synapseState *s, tw_bf *bf, struct messageData *M, tw_lp *lp);
void synapse_final(synapseState *s, tw_lp *lp);
void synapse_pre_run(synapseState *s, tw_lp *lp);

#endif //NEMO_SYNAPSE_H
