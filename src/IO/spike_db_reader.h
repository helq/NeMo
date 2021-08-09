//
// Created by Mark Plagge on 11/28/17.
//

#ifndef SUPERNEMO_SPIKE_DB_READER_H
#define SUPERNEMO_SPIKE_DB_READER_H

#include "../globals.h"

extern char NEMO_SPIKE_FILE_PATH[512];

/** @defgroup spin Spike Input Functions @{ */

/* Spike Loading Functions (generics) */
/** Not used in NeMo2 */
int getSpikesFromAxon(void *timeList, id_type core, id_type axonID);
/**
 * Core-wise spike loading. Synapse calls this function with its core and and inited list. returns num of
 * spikes and a populated list.
 * @param timeList A list populated with interleaved time->axonID structs
 * @param core the coreID of the synapse
 * @return the number of spikes in the input file.
 */
int getSpikesFromSynapse(void *timeList, id_type core);
/**
 * Core-wise spike counter. Synapse calls this function to see if there are spikes in the input file for this
 * particular core.
 * @param core The local coreID of the synapse.
 * @return number of spikes
 */
int getNumSpikesForCore(id_type core);

//int spikeFromSynapseComplete(void *timeList);
void spikeFromAxonComplete(void *timeList);
//int loadSpikesFromFile(char *filename);
//int getSpikeCount();
//void testSpikes();

/** @}**/

int openSpikeFile();
int closeSpikeFile();

#endif //SUPERNEMO_SPIKE_DB_READER_H
