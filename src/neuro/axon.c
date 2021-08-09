//
//  axon.c
//  ROSS_TOP
//
//  Created by Mark Plagge on 6/18/15.
//
//

#include <simclist/simclist.h>
#include "../IO/spike_db_reader.h"
#include "../dumpi.h"
#include "../mapping.h"
#include "axon.h"

//static list_t spikeList;
//static FILE *spikeFile;
static FILE *dumpi_out1;

//static void scheduleSpike(long time, id_type axonID, tw_lp *lp) {
//  tw_stime sched_event = time + JITTER;
//  tw_event *axevt = tw_event_new(lp->gid, sched_event, lp);
//  struct messageData *data = tw_event_data(axevt);
//  data->axonID = axonID;
//  data->originGID = lp->gid;
//  data->eventType = AXON_OUT;
//  tw_event_send(axevt);
//}

//static int axonct = 0;

/* Not used in NeMo2 */
//static void axonSpikeReader(struct AxonState *s, tw_lp *lp) {
//  static char announce = 1;
//  // static int axonct = 0;
//  if (g_tw_mynode == 0) {
//    if (announce) {
//      printf("Starting axon loading of spikes.\n");
//      announce = 0;
//    }
//    if (axonct % 100 == 0) {
//      printf("Axon %i - %i checking in.\n ", s->axonID, axonct);
//      axonct++;
//    }
//  }
//  list_t spikeList;
//  list_init(&spikeList);
//  //    if(getCoreFromGID(lp->gid) > 511){
//  //        printf("high order here.\n");
//  //    }
//  int numSpikes =
//      getSpikesFromAxon(&spikeList, getCoreFromGID(lp->gid), s->axonID);
//  if (numSpikes > 0) {
//    list_iterator_start(&spikeList);
//    while (list_iterator_hasnext(&spikeList)) {
//      scheduleSpike(*(long *)list_iterator_next(&spikeList), s->axonID, lp);
//    }
//    list_iterator_stop(&spikeList);
//  }
//  spikeFromAxonComplete(&spikeList);
//}

void axon_init(struct AxonState *s, tw_lp *lp) {
  static int fileInit = 0;
  ///// DUMPI FILE
  if (DO_DUMPI && !fileInit) {
    char *fn = calloc(sizeof(char), 256);
    sprintf(fn, "dumpi_virt-%i_rnk%li-rcvr.txt", getCoreFromGID(lp->gid),
            g_tw_mynode);
    dumpi_out1 = fopen(fn, "w");
    free(fn);
    fileInit = 1;
  }

  // TODO: Maybe switch this to a switch/case later, since it's going to get
  // big.
  static int specAxons = 0;
  s->axtype = "NORM";

  if (FILE_IN) { // FILE_IN){

    s->sendMsgCount = 0;
    s->axonID = getAxonLocal(lp->gid);
    s->destSynapse = getSynapseFromAxon(lp->gid);
    // Moved this functionality to synapses.
    // axonSpikeReader(s,lp);

    specAxons++;

  } else if (PHAS_VAL) { // one phasic axon:
    if (specAxons == 0) {
      // crPhasicAxon(s, lp);
      specAxons++;
    } else {
      s->sendMsgCount = 0;
      s->axonID = (lp->gid);
      s->destSynapse = getSynapseFromAxon(lp->gid);
    }
  } else { // else this is a random network for benchmarking.
    s->sendMsgCount = 0;
    s->axonID = getAxonLocal(lp->gid);
    s->destSynapse = getSynapseFromAxon(lp->gid);
    tw_stime r = getNextBigTick(lp, 0);
    tw_event *axe = tw_event_new(lp->gid, r, lp);
    struct messageData *data = (struct messageData *)tw_event_data(axe);
    data->eventType = AXON_OUT;
    data->axonID = s->axonID;
    tw_event_send(axe);
  }

  if (DEBUG_MODE) {

    printf("Axon type - %s, #%llu checking in with dest synapse %llu\n",
           s->axtype, lp->gid, s->destSynapse);
  }

  // printf("message ready at %f",r);
}

void axon_event(struct AxonState *s, tw_bf *CV, struct messageData *M, tw_lp *lp) {
  // generate new message
  // enum evtType mt = M->eventType;
  long rc = lp->rng->count;
  tw_event *axonEvent = tw_event_new(s->destSynapse, getNextEventTime(lp), lp);
  struct messageData *data = (struct messageData *)tw_event_data(axonEvent);

  data->localID = lp->gid;
  data->eventType = AXON_OUT;
  data->axonID = s->axonID;

  tw_event_send(axonEvent);

  // End message generation - add message count and check random times
  s->sendMsgCount++;
  M->rndCallCount = lp->rng->count - rc;
}

void axon_reverse(struct AxonState *s, tw_bf *CV, struct messageData *M, tw_lp *lp) {
  --s->sendMsgCount;

  long count = M->rndCallCount;
  while (count--) {
    tw_rand_reverse_unif(lp->rng);
  }
}

void axon_final(struct AxonState *s, tw_lp *lp) {
  static int fileOpen = 1;
  if (DO_DUMPI && fileOpen) {
    fclose(dumpi_out1);
    fileOpen = 0;
  }
}

void axon_commit(struct AxonState *s, tw_bf *CV, struct messageData *M, tw_lp *lp) {
  if (DO_DUMPI && M->isRemote) {
    // saveMPIMessage(s->myCoreID, getCoreFromGID(s->outputGID), tw_now(lp),
    //			   dumpi_out);
    setrnd(lp);
    saveRecvMessage(M->isRemote, getCoreFromGID(lp->gid), tw_now(lp), 0,
                    dumpi_out1);
  }
}
