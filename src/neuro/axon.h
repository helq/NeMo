#ifndef __NEMO_axon__
#define __NEMO_axon__
#include "../globals.h"
#include "../message.h"

extern bool PHAS_VAL;

typedef struct AxonState {
  uint64_t sendMsgCount;
  tw_lpid destSynapse;
  id_type axonID;

  //inputSimulatorState *sim;

  char *axtype;
} axonState;

void axon_init(axonState *s, tw_lp *lp);
void axon_event(axonState *s, tw_bf *CV, struct messageData *M, tw_lp *lp);
void axon_reverse(axonState *s, tw_bf *CV, struct messageData *M, tw_lp *lp);
void axon_commit(axonState *s, tw_bf *CV, struct messageData *M, tw_lp *lp);
void axon_final(axonState *s, tw_lp *lp);
#endif
