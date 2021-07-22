//
// Created by Mark Plagge on 9/4/17.
//

#ifndef SUPERNEMO_TOMACS_EXP_H
#define SUPERNEMO_TOMACS_EXP_H
#include "../globals.h"
#include "../neuro/tn_neuron.h"

extern int connectedWeight;

/**
 * using the connectivity probability, this function returns a connectivity grid for a neuron.
 * Using a bucket method instead of true random will help with the reandomness.
 * @param synapticConGrid
 *
 */
void getSynapticConnectivity(bool *synapticConGrid, tw_lp *lp);

void clearBucket();

#endif //SUPERNEMO_TOMACS_EXP_H
