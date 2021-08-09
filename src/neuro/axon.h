#ifndef __NEMO_axon__
#define __NEMO_axon__
#include "../message.h"

extern bool PHAS_VAL;

struct AxonState {
  uint64_t sendMsgCount;
  tw_lpid destSynapse;
  id_type axonID;
  char *axtype;
};

void axon_init(struct AxonState *s, tw_lp *lp);
void axon_event(struct AxonState *s, tw_bf *CV, struct messageData *M, tw_lp *lp);
void axon_reverse(struct AxonState *s, tw_bf *CV, struct messageData *M, tw_lp *lp);
void axon_commit(struct AxonState *s, tw_bf *CV, struct messageData *M, tw_lp *lp);
void axon_final(struct AxonState *s, tw_lp *lp);
#endif
