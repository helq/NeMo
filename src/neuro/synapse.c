#include "synapse.h"

#include "../../../external/simclist/simclist.h"
#include "../../external/itrlve.h"
#include "../IO/energy_stats.h"
#include "../IO/spike_db_reader.h"
#include "../mapping.h"

uint64_t numScheduledEvents = 0;
void scheduleSp(tw_stime time, id_type axonID, tw_lp *lp) {

  tw_event *synapsePrime = tw_event_new(lp->gid, time, lp);
  messageData *outData = tw_event_data(synapsePrime);
  outData->axonID = axonID;
  outData->eventType = AXON_OUT;
  tw_event_send(synapsePrime);
  numScheduledEvents++;
}

/**
 * Loads all input spikes from the SQLite database that are going to this neurosynaptic core.
 *
 * @param s My State
 * @param lp My LP
 *
 */
void loadSynapseSpikesFile(synapseState *s, tw_lp *lp) {
  list_t spikelist;
  id_type myCore = s->myCore;

  //query the spike database for the number of spikes scheduled for this core
  int nspikes = getNumSpikesForCore(myCore);

  //if there are spikes to read in, do so:
  if (nspikes) {
    //Store the spikes in a linked list
    list_init(&spikelist);
    list_attributes_copy(&spikelist, list_meter_int64_t, 1);

    //This is the call to the SQLite wrapper functions.
    getSpikesFromSynapse(&spikelist, myCore);

    /** Macro wrappers for interleaving time/axonids */
    #define EXTIME(x) EXHIGH(x)
    #define EXAXON(x) EXLOW(x)

    list_iterator_start(&spikelist);
    while (list_iterator_hasnext(&spikelist)) {

      uint64_t ilv = *(uint64_t *) list_iterator_next(&spikelist);
      uint32_t time = EXTIME(ilv);
      uint32_t axid = EXAXON(ilv);

      //This call schedules the input spike - for all
      //input spikes in the input spike file (that are supposed to
      //go to this core), scheduleSP needs to be called.
      //time and axid are the pulled data fields from the config file,
      // lp is the LP variable (passed in at the top of the function
      scheduleSp(time, axid, lp);

    }
    list_iterator_stop(&spikelist);

    spikeFromAxonComplete(&spikelist);
  }
}

void synapse_init(synapseState *s, tw_lp *lp) {
  s->msgSent = 0;
  s->lastBigTick = 0;
  s->myCore = getCoreFromGID(lp->gid);

  if (FILE_IN) {
    loadSynapseSpikesFile(s, lp);
  }

  if (DEBUG_MODE) {
    printf("Super Synapse Created - GID is %llu\n", lp->gid);
  }
}

/**
 * Gets the next event time for synapse internal heartbeats -
 */
tw_stime getNextSynapseHeartbeat(tw_lp *lp) {
  return JITTER + littleTick;
}

void sendSynapseHB(synapseState *s, tw_bf *bf, messageData *M, tw_lp *lp, unsigned long count) {

  tw_event *synapseHB = tw_event_new(lp->gid, getNextSynapseHeartbeat(lp), lp);
  messageData *outData = tw_event_data(synapseHB);
  outData->synapseCounter = count - 1;
  outData->axonID = M->axonID;
  outData->localID = M->localID;
  outData->eventType = SYNAPSE_HEARTBEAT;

  tw_event_send(synapseHB);

}
void reverseSynapseHB(synapseState *s, tw_bf *bf, messageData *M, tw_lp *lp) {
  M->synapseCounter++;
}

void synapse_event(synapseState *s, tw_bf *bf, messageData *M, tw_lp *lp) {
  unsigned long randCount = lp->rng->count;

  if (M->axonID > AXONS_IN_CORE)
    tw_error(TW_LOC, "Invalid AXON value within synapse system.");

  if (M->eventType==SYNAPSE_HEARTBEAT) {
    //Heartbeat message
    if (M->synapseCounter!=0) {
      //unsigned long sc = M->synapseCounter - 1;
      sendSynapseHB(s, bf, M, lp, M->synapseCounter);
    }

    tw_lpid neuron = getNeuronGlobal(s->myCore, M->synapseCounter);
    tw_event *sout = tw_event_new(neuron, getNextEventTime(lp), lp);
    messageData *outData = tw_event_data(sout);
    outData->axonID = M->axonID;
    outData->localID = M->axonID;
    outData->eventType = SYNAPSE_OUT;
    tw_event_send(sout);
    ++s->msgSent;

  } else if (M->eventType==AXON_OUT) {
    sendSynapseHB(s, bf, M, lp, NEURONS_IN_CORE);
  }

  M->rndCallCount = lp->rng->count - randCount;

}

void synapse_reverse(synapseState *s, tw_bf *bf, messageData *M, tw_lp *lp) {

  if (M->eventType==AXON_OUT) {

  } else if (M->eventType==SYNAPSE_HEARTBEAT) {
    --s->msgSent;
  }
  unsigned long count = M->rndCallCount;
  while (count--) {
    tw_rand_reverse_unif(lp->rng);
  }
}

void synapse_final(synapseState *s, tw_lp *lp) {
  //do some stats here if needed.
  static int announce = 0;
  if (!announce && g_tw_mynode==0) {
    tw_printf(TW_LOC, "Scheduled %llu events from file.\n", numScheduledEvents);
    announce = 1;
  }
  if (g_tw_synchronization_protocol==OPTIMISTIC_DEBUG) {
    char *shdr = "Synapse Error\n";

    if (s->msgSent!=0) {
      printf("%s ", shdr);
    }
  }

}
void synapse_pre_run(synapseState *s, tw_lp *lp) {
  static int should_close = 1;
  if (should_close) {
    closeSpikeFile();
    should_close = 0;
  }
}

