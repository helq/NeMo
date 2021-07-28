//
// Created by Mark Plagge on 5/25/16.
//
//
/** @file globals.h */

#ifndef __NEMO_GLOBALS_H__
#define __NEMO_GLOBALS_H__

#define BGQ 1
#define NET_IO_DEBUG 1

#include <stdbool.h>
#include <nemo_config.h>
#include "ross.h"

#define MAX_RANKS_FILES  65535

typedef int_fast64_t id_type;  //!< id type is used for local mapping functions - there should be $n$ of them depending on CORE_SIZE

/** @defgroup timeFuncts Time Functions
  * Functions that manage big tick and little ticks
  */
/** @{ */

/** Macro for use within globals.
 Assumes that there is a tw_lp pointer called lp in the function it is used.
 */
#define JITTER (tw_rand_unif(lp->rng) / 10000)

/* Global Timing Variables */
/** little tick rate - controls little tick timing */
#define littleTick 0.0001
/** bigTickRate is the rate of simulation - neurons synchronize based on this value. */
#define bigTickRate 1

/**
 *  Gets the next event time, based on a random function. Moved here to allow for
 *  easier abstraction, and random function replacement.
 *
 *
 *  @param lp Reference to the current LP so that the function can see the RNG
 *
 *  @return a tw_stime value, such that \f$ 0 < t < 1 \f$. A delta for the next
 *  time slice.
 */
static inline tw_stime getNextEventTime(tw_lp *lp) {
  return JITTER + littleTick;
}

/**
 *  @brief  Given a tw_stime, returns the current big tick.
 *
 *  @param now current time
 *
 *  @return the current big tick time.
 */
static inline tw_stime getCurrentBigTick(tw_stime now) {
  return floor(now);
}

/**
 *  @brief  Given a tw_stime, returns the next big-tick that will happen
 *
 *  @param lp the lp asking for the next big tick.
 *  @param neuronID Currently unused. Reserved for future fine-grained neuron tick management.
 *
 *  @return Next big tick time.
 */
static inline tw_stime getNextBigTick(tw_lp *lp, tw_lpid neuronID) {
  return JITTER + bigTickRate;
}

/**@}*/

/** Global Variables */
extern uint64_t SIM_SIZE;

extern uint64_t CORES_IN_SIM;
extern uint64_t SYNAPSES_IN_CORE;
extern unsigned int CORES_IN_CHIP;

extern bool DEBUG_MODE;
extern bool SAVE_SPIKE_EVTS; //!< Toggles saving spike events
extern bool SAVE_NETWORK_STRUCTURE;
extern unsigned int SAVE_OUTPUT_NEURON_EVTS;

extern bool FILE_IN;

#endif // __NEMO_GLOBALS_H__
