//
// Created by Mark Plagge on 5/25/16.
//

#include "../IO/model_reader.h"
#include "../IO/neuron_output.h"
#include "../IO/output.h"
#include "../dumpi.h"
#include "../layer_map/layer_map_lib.h"
#include "../mapping.h"
#include "tn_neuron.h"

#define IABS(a) (((a) < 0) ? (-a) : (a)) //!< Typeless integer absolute value function
// TODO: See if there is a non-branching version of the signum function, maybe in MAth libs and use that.
#define SGN(x) ((x > 0) - (x < 0)) //!< Signum function
#define DT(x) !(x) //!< Kronecker Delta function.

#define BINCOMP(s, p) IABS((s)) >= (p) //!< binary comparison for conditional stochastic evaluation
#define iBINCOMP(s, p) iIABS((s)) >= (p) //!< binary comparison for conditional stochastic evaluation (integer only)

/** 32bit X86 Assembler IABS: */
static inline int32_t iIABS(int32_t vals) {
#ifdef X86ASM
  int result;
  asm ("movl  %[valI], %%eax;"
          "cdq;"
          "xor %%edx, %%eax;"
          "sub %%edx, %%eax;"
          "movl %%eax, %[resI];"
  : [resI] "=r" (result)
  : [valI] "r" (vals)
  : "cc","%eax", "%ebx");
  return result;
#endif
  return IABS(vals);
}

// File output handle.
static FILE *dumpi_out;
/** \defgroup TN_Function_hdrs True North Function headers
 * TrueNorth Neuron leak, integrate, and fire function forward decs.
 * @{ */

static void TNFire(struct NeuronState *st, void *l);

/**
 *  @brief  function that adds a synapse's value to the current neuron's
 * membrane potential.
 *
 *  @param synapseID localID of the synapse sending the message.
 */
static void TNIntegrate(id_type synapseID, struct NeuronState *st, void *lp);
/**
*  @brief  handles incomming synapse messages. In this model, the neurons send
messages to axons during "big tick" intervals.
This is done through an event sent upon receipt of the first synapse message of
the current big-tick.
*
*  @param st   current neuron state
*  @param m    event message data
*  @param lp   lp.
*/

static bool TNReceiveMessage(struct NeuronState *st, struct messageData *m, tw_lp *lp,
                             tw_bf *bf);

/**
 * @brief handels reverse computation and state messages.
 * @param st current neuron state
 * @param M reverse message
 * @param lp the lp
 * @param bf the reverse computation bitfield.
 */

static void TNReceiveReverseMessage(struct NeuronState *st, struct messageData *M,
                                    tw_lp *lp, tw_bf *bf);

/**
 *  @brief  Checks to see if a neuron should fire.
 *  @todo check to see if this is needed, since it looks like just a simple if
 * statement is in order.
 *
 *  @param st neuron state
 *
 *  @return true if the neuron is ready to fire.
 */
static bool TNShouldFire(struct NeuronState *st, tw_lp *lp);

/**
 * @brief New firing system using underflow/overflow and reset.
 * @return true if neuron is ready to fire. Membrane potential is set
 * regardless.
 */
static bool TNfireFloorCelingReset(struct NeuronState *ns, tw_lp *lp);

/**
 *  @brief  Neuron stochastic integration function - for use with stochastic
 * leaks and synapse messages.
 *
 *  @param weight weight of selected leak or synapse
 *  @param st     the neuron state
 */
static void TNstochasticIntegrate(int64_t weight, struct NeuronState *st);

/**
 *  @brief NumericLeakCalc - uses formula from the TrueNorth paper to calculate
 * leak.
 *  @details Will run $n$ times, where $n$ is the number of big-ticks that have
 * occured since
 *  the last integrate. Handles stochastic and regular integrations.
 *
 *  @TODO: self-firing neurons will not properly send messages currently - if
 * the leak is divergent, the flag needs to be set upon neuron init.
 *  @TODO: does not take into consideration thresholds. Positive thresholds
 * would fall under self-firing neurons, but negative thresholds should be
 * reset.
 *  @TODO: test leaking functions
 */
static void TNNumericLeakCalc(struct NeuronState *st, tw_stime now);

static void TNSendHeartbeat(struct NeuronState *st, tw_stime time, void *lp);

static void tn_create_neuron_encoded_rv(
    id_type coreID, id_type nID, bool synapticConnectivity[NEURONS_IN_CORE],
    short G_i[NEURONS_IN_CORE], short sigma[4], short S[4], bool b[4],
    bool epsilon, short sigma_l, short lambda, bool c, uint32_t alpha,
    uint32_t beta, short TM, short VR, short sigmaVR, short gamma, bool kappa,
    struct NeuronState *n, int signalDelay, uint64_t destGlobalID, int destAxonID);

/** From Neuron Behavior Reference - checks to make sure that there is no
 "ringing".
 The specs state that the leak stores the voltage in a temporary variable. Here,
 we store the leak voltage in the membrane potential, and override it with a new
 value. */

static void ringing(void *nsv, int32_t oldVoltage);

/** @} */

static bool fireTimingCheck(struct NeuronState *s, tw_lp *lp) {
  // check to see if the fire event should occur.
  // Neurons can output one spike per big tick.
  // in Debug mode, we set the last fired time and update the count.
  // If we are in the next big tick then everything is okay.
  tw_stime now = tw_now(lp);
  unsigned long currentBigTick = getCurrentBigTick(now);
  unsigned long prevBigTick = getCurrentBigTick(s->lastFire);

  if (currentBigTick == 0) { // big tick is zero so we are at first
    s->firecount++;
    return true;
  }
  // check to see if we are on a new tick:
  if (currentBigTick > prevBigTick) {
    // we are in a new big tick. This is okay.
    s->firecount = 1;
    s->lastFire = now;
    return true;
  } else if (currentBigTick ==
             prevBigTick) { // fire request on the same tick - error condition 1
    s->firecount++;

    //#ifdef DEBUG
    //    tw_error(TW_LOC, "Neuron fire rate error. Current big tick: %lu \t
    //    FireCount: %i. Neuron Core: %i Local: %i ",
    //             currentBigTick, s->firecount, s->myCoreID, s->myLocalID);
    //#endif
    return false;
  } else {
    // Unknown error state - BigTick < prevBigTick.
    // tw_error(TW_LOC, "Out of order big ticks! PrevActive: %lu, CurrentTick:
    // %lu", prevBigTick, currentBigTick); Reverse computation dude.
    return true;
  }
}

/**
 * \ingroup TN_RESET True North Reset
 * True North Leak, Integrate and Fire Functions
 * Reset function defs. Neuron reset functions will
 * change the neuron state without saving the previous state. All voltage state
 saving
 * must be handled in the neuron event function neuronReceiveMessage().
 * These functions operate based on the table presented in \cite Cass13_1000 .
 * Currently, there are three functions, one for linear reset (resetLinear()),
 * "normal" reset (resetNormal()), and non-reset (resetNone()).

 From the paper:
 | \f$𝛾_j\f$ | \f$𝜘_j\f$| Reset Mode               |     Positive Reset     |
 Negative Reset    |
 |----|----|--------------------------|:----------------------:|:---------------------:|
 | 0  | 0  | normal                   |          \f$R_j\f$         |
 \f$-R_j\f$        |
 | 0  | 1  | normal - neg saturation  |          \f$R_j\f$         |
 \f$-𝛽_j\f$        |
 | 1  | 0  | Linear                   | \f$V_j - (𝛼_j  + 𝜂_j)\f$ | \f$V_j + (𝛽_j
 + 𝜂_j)\f$ |
 | 1  | 1  | linear -neg saturation   |  \f$V_j - (𝛼_j,+ 𝜂_j)\f$ |
 \f$-𝛽_j\f$        |
 | 2  | 0  | non-reset                |          \f$V_j\f$         |
 \f$V_j\f$         |
 | 2  | 1  | non-reset net saturation |          \f$V_j\f$         |
 \f$-𝛽_j\f$        |

 * @todo: Check that reverse reset functions are needed, since previous voltage
 is stored in the neuron.
 * @{ */

/**
 * negative saturation reset function (common to all reset modes, called if
 * 𝛾 is true. Simply sets the value of the membrane potential to $-𝛽_j$.
 **/
static void negThresholdReset(struct NeuronState *s) {
  s->membranePotential = -s->negThreshold;
}

/**

 * Normal reset function.
 */
static void resetNormal(void *neuronState) {
  struct NeuronState *s = (struct NeuronState *)neuronState;
  if (s->membranePotential < s->negThreshold) {
    if (s->kappa)
      negThresholdReset(s);
    else
      s->membranePotential = -(s->resetVoltage);
  } else {
    s->membranePotential = s->resetVoltage; // set current voltage to \f$R\f$.
  }
}

/**
 *   Linear reset mode - ignores \f$R\f$, and sets the membrane potential
 *  to the difference between the threshold and the potential. *
 */
static void resetLinear(void *neuronState) {
  struct NeuronState *s = (struct NeuronState *)neuronState;

  if (s->membranePotential < s->negThreshold) {
    if (s->kappa)
      negThresholdReset(s);
    else {
      s->membranePotential =
          s->membranePotential - (s->negThreshold + s->drawnRandomNumber);
    }
  } else {
    s->membranePotential =
        s->membranePotential - (s->posThreshold + s->drawnRandomNumber);
  }
}

/**
 *   non-reset handler function - does non-reset style reset. Interestingly,
 *  even non-reset functions follow the negative saturation parameter from the
 * paper.
 */
static void resetNone(void *neuronState) {
  struct NeuronState *s = (struct NeuronState *)neuronState;

  if (s->kappa && s->membranePotential < s->negThreshold) {
    negThresholdReset(s);
  }
}

/**@} */
/** @defgroup TN_fire_reset True North Reset Fire
 *   True North reset and fire functions
 * @{ */

/** From Neuron Behavior Reference - checks to make sure that there is no
 "ringing".
 The specs state that the leak stores the voltage in a temporary variable. Here,
 we store the leak voltage in the membrane potential, and override it with a new
 value. */
static void TNFire(struct NeuronState *st, void *l) {

  // tw_lp *lp = (tw_lp *) l;

  // DEBUG
  //	tw_lpid outid = st->dendriteGlobalDest;
  //	tw_lp *destLP = tw_getlp(outid);
  //	printf("Sending message to %llu\n", destLP->gid);

  //} else {
  tw_lp *lp = (tw_lp *)l;
  // DEBUG
  //	tw_lp *destLP = tw_getlp(outid);
  //	tw_lpid outid = st->dendriteGlobalDest;
  //	printf("Sending message to %llu\n", destLP->gid);

  // DEBUG
  tw_stime nextHeartbeat = getNextBigTick(lp, st->myLocalID);
  tw_event *newEvent = tw_event_new(st->outputGID, nextHeartbeat, lp);
  struct messageData *data = (struct messageData *)tw_event_data(newEvent);

  data->eventType = NEURON_OUT;
  data->localID = st->myLocalID;
  if (isDestInterchip(st->myCoreID, getCoreFromGID(st->outputGID))) {
    data->isRemote = st->myCoreID;
  }
  tw_event_send(newEvent);
  st->firedLast = true;
  //}
}

static bool TNReceiveMessage(struct NeuronState *st, struct messageData *m, tw_lp *lp,
                             tw_bf *bf) {
  /** @todo Replace these state saving values with reverse computation. */
  m->neuronVoltage = st->membranePotential;
  m->neuronLastLeakTime = st->lastLeakTime;
  m->neuronDrawnRandom = st->drawnRandomNumber;
  // m->neuronFireCount = st->fireCount;

  // bf->c14 = st->heartbeatOut; //C14 indicates the old heartbeat state.
  // state management
  bool willFire = false;
  // Next big tick:

  //@todo: see if this is used still and remove
  // int num = st->myLocalID;

  switch (m->eventType) {
    /// @TODO: possibly need to aggregate inputs on the same channel? If
    /// validation isn't working check this.

  case SYNAPSE_OUT:
    st->drawnRandomNumber = tw_rand_integer(
        lp->rng, 0,
        st->largestRandomValue); //!<- @BUG This might be creating
    //! non-deterministic errors
    TNIntegrate(m->axonID, st, lp);
    // next, we will check if a heartbeat message should be sent
    if (st->heartbeatOut == false) {
      tw_stime time = getNextBigTick(lp, st->myLocalID);
      st->heartbeatOut = true;
      bf->c13 = 1; // C13 indicates that the heartbeatout flag has been changed.
      TNSendHeartbeat(st, time, lp);

      // set message flag indicating that the heartbeat msg has been sent
    }
    break;

  case NEURON_HEARTBEAT:
    st->heartbeatOut = false;
    // set message flag indicating that the heartbeat msg has been sent
    bf->c13 = 1; // C13 indicates that the heartbeatout flag has been changed.

    // Currently operates - leak->fire->(reset)
    st->drawnRandomNumber = tw_rand_integer(lp->rng, 0, st->largestRandomValue);

    TNNumericLeakCalc(st, tw_now(lp));
    // linearLeak( st, tw_now(lp));
    ringing(st, m->neuronVoltage);

    // willFire = neuronShouldFire(st, lp); //removed and replaced with
    // fireFloorCelingReset
    willFire = TNfireFloorCelingReset(st, lp);
    willFire = (willFire && fireTimingCheck(st, lp));
    bf->c0 = bf->c0 || willFire;

    if (willFire) {
      if (!st->isOutputNeuron) {
        TNFire(st, lp);
        // check for intra-core communications -
        // setting bit 31 as toggle for send communication

        // st->fireCount++;
      } else {
        /** bf->c10 is an output neuron fire state checker. True means the
         * neuron fired this turn. */
        bf->c10 = 1;
      }
      if (isDestInterchip(st->myCoreID, getCoreFromGID(st->outputGID))) {
        bf->c31 = 1;
        // m->dumpiID = tw_rand_ulong(lp->rng,0,ULONG_MAX - 1);
      } else {
        bf->c31 = 0;
      }
    }

    st->lastActiveTime = tw_now(lp);

    // do we still have more than the threshold volts left? if so,
    // send a heartbeat out that will ensure the neuron fires again.
    // Or if we are as self-firing neuron.
    ///@TODO: Add detection of self-firing neuron state.
    ///@TODO: Ensure bf-c13 state validity here for reverse computations
    if (TNShouldFire(st, lp) && st->heartbeatOut == false) {
      tw_stime const time = getNextBigTick(lp, st->myLocalID);
      st->heartbeatOut = true;
      // set message flag indicating that the heartbeat msg has been sent
      bf->c13 = 1; // C13 indicates that the heartbeatout flag has been changed.
      TNSendHeartbeat(st, time, lp);
    }
    break;
  default:
    // Error condition - non-valid input.
#ifdef DEBUG
    tw_printf(TW_LOC,
              "Invalid message type received. Dumping information..."
              "\n message source: %li "
              "\n message o gid %lu:\n"
              "---",
              m->localID, m->originGID);
#endif
    tw_error(TW_LOC, "Neuron (%i,%i) received invalid message type, %i \n ",
             st->myCoreID, st->myLocalID, m->eventType);

    break;
  }
  // self-firing neuron (spont.)
  if (st->isSelfFiring && st->heartbeatOut == false) {
    tw_stime const time = getNextBigTick(lp, st->myLocalID);
    st->heartbeatOut = true;
    // set message flag indicating that the heartbeat msg has been sent
    bf->c13 = 1; // C13 indicates that the heartbeatout flag has been changed.
    TNSendHeartbeat(st, time, lp);
  }
  return willFire;
}

static void TNReceiveReverseMessage(struct NeuronState *st, struct messageData *M,
                                    tw_lp *lp, tw_bf *bf) {
  if (M->eventType == NEURON_HEARTBEAT) {
    // reverse heartbeat message
  }
  if (bf->c0) { // c0 flags firing state
    // reverse computation of fire and reset functions here.
    /**@todo implement neuron fire/reset reverse computation functions */
    st->firedLast = false;
  }
  if (bf->c13) {
    st->heartbeatOut = !st->heartbeatOut;
  }
  /**@todo remove this once neuron reverse computation functions are built. */
  st->membranePotential = M->neuronVoltage;
  st->lastLeakTime = M->neuronLastLeakTime;
  st->lastActiveTime = M->neuronLastActiveTime;
  st->drawnRandomNumber = M->neuronDrawnRandom;
}

/**
 * @brief      From Neuron Behavior Reference - checks to make sure that there
 is no "ringing".
 The specs state that the leak stores the voltage in a temporary variable. Here,
 we store the leak voltage in the membrane potential, and override it with a new
 value.
 *
 * @param      nsv         The neuron state
 * @param[in]  oldVoltage  The old voltage
 */
static void ringing(void *nsv, int32_t oldVoltage) {
  struct NeuronState *ns = (struct NeuronState *)nsv;
  if (ns->epsilon && (SGN(ns->membranePotential) != SGN(oldVoltage))) {
    ns->membranePotential = 0;
  }
}

static void TNIntegrate(id_type synapseID, struct NeuronState *st, void *lp) {
  // tw_lp *l = (tw_lp *) lp;
  // int at = st->axonTypes[synapseID];
  bool const con = st->synapticConnectivity[synapseID];
  // DEBUG CODE REMOVE FOR PRODUCTION:
  // id_type myid = st->myLocalID;

  if (con == 0)
    return;
  // printf("id-%llu, sid-%llu, connect: %i\n",myid, synapseID,con);
  // int64_t stw = st->synapticWeight[at];
  int64_t const weight = st->synapticWeight[st->axonTypes[synapseID]] &&
                         st->synapticConnectivity[synapseID];
  //!!!! DEBUG CHECK FOR WEIGHT ISSUES:
  // weight = 0;
  //!!! REMOVE previous FOR PRODUCTION

  if (st->weightSelection[st->axonTypes[synapseID]]) { // zero if this is
    // normal, else

    TNstochasticIntegrate(weight, st);
  } else {
    st->membranePotential += weight;
  }
}

static void TNSendHeartbeat(struct NeuronState *st, tw_stime time, void *lp) {
  tw_lp *l = (tw_lp *)lp;
  tw_event *newEvent =
      tw_event_new(l->gid, getNextBigTick(l, st->myLocalID), l);
  // tw_event *newEvent = tw_event_new(l->gid, (0.1 + (tw_rand_unif(l->rng) /
  // 1000)),l);
  struct messageData *data = (struct messageData *)tw_event_data(newEvent);
  data->localID = st->myLocalID;
  data->eventType = NEURON_HEARTBEAT;
  tw_event_send(newEvent);
  if (st->heartbeatOut == false) {
    tw_error(TW_LOC,
             "Error - neuron sent heartbeat without setting HB to true\n");
  }
}

static bool overUnderflowCheck(void *ns) {
  struct NeuronState *n = (struct NeuronState *)ns;

  int ceiling = 393216;
  int floor = -393216;
  bool spike = false;
  if (n->membranePotential > ceiling) {
    spike = true;
    n->membranePotential = ceiling;
  } else if (n->membranePotential < floor) {
    n->membranePotential = floor;
  }
  return spike;
}

static bool TNShouldFire(struct NeuronState *st, tw_lp *lp) {
  // check negative threshold values:
  int32_t threshold = st->posThreshold;
  return (st->membranePotential >= threshold &&
          fireTimingCheck(st, lp)); // + (st->drawnRandomNumber));
}

static bool TNfireFloorCelingReset(struct NeuronState *ns, tw_lp *lp) {
  bool shouldFire = false;
  ///@TODO remove this line later for performacne - check that neuron init
  /// handles this properly.
  ns->drawnRandomNumber = 0;
  // Sanity variables - remove if we need that .01% performance increase:
  int32_t Vrst = ns->resetVoltage;
  int32_t alpha = ns->posThreshold;
  int32_t beta = ns->negThreshold;
  // DEBUG DELTA GAMMA
  //	short deltaGamma = DT(ns->resetMode);
  //	short dg1 = DT(ns->resetMode -1);
  //	short dg2 = DT(ns->resetMode -2);
  //	short tg2 = !(ns->resetMode -2);
  short gamma = ns->resetMode;

  ///@TODO: might not need this random generator, if it is called in the event
  /// handler here
  if (ns->c)
    ns->drawnRandomNumber =
        (tw_rand_integer(lp->rng, 0, ns->largestRandomValue));
  // check if neuron is overflowing/underflowing:
  if (overUnderflowCheck((void *)ns)) {
    return true;
  } else if (ns->membranePotential >=
             ns->posThreshold + ns->drawnRandomNumber) {
    // reset:
    ns->membranePotential =
        ((DT(gamma)) * Vrst) +
        ((DT(gamma - 1)) *
         (ns->membranePotential - (alpha + ns->drawnRandomNumber))) +
        ((DT(gamma - 2)) * ns->membranePotential);
    // int32_t mp = ns->membranePotential;
    shouldFire = true;
  } else if (ns->membranePotential <
             (-1 * (beta * ns->kappa +
                    (beta + ns->drawnRandomNumber) * (1 - ns->kappa)))) {
    // int32_t x = ns->membranePotential;
    // x = ((-1 * beta) * ns->kappa);
    //        int32_t s1,s2,s3,s4;
    //        s1 = (-1*beta) * ns->kappa;
    //        s2 = (-1*(DT(gamma))) * Vrst;
    //        s3 = (DT((gamma - 1)) * (ns->membranePotential + (beta +
    //        ns->drawnRandomNumber)));
    //        s4 = (DT((gamma - 2)) * ns->membranePotential) * (1 - ns->kappa);
    //		//x = s1 + (s2 + s3 + s4);

    ns->membranePotential =
        (((-1 * beta) * ns->kappa) +
         (((-1 * (DT(gamma))) * Vrst) +
          ((DT((gamma - 1))) *
           (ns->membranePotential + (beta + ns->drawnRandomNumber))) +
          ((DT((gamma - 2))) * ns->membranePotential)) *
             (1 - ns->kappa));
  }
  return shouldFire;
}

static void TNstochasticIntegrate(int64_t weight, struct NeuronState *st) {
  if (BINCOMP(weight, st->drawnRandomNumber)) {
    st->membranePotential += 1;
  }
}

static void TNNumericLeakCalc(struct NeuronState *st, tw_stime now) {
  // shortcut for calcuation - neurons do not leak if:
  // lambda is zero:
  if (st->lambda == 0)
    return;
  // calculate current time since last leak --- LEAK IS TERRIBLE FOR THIS:
  uint_fast32_t numberOfBigTicksSinceLastLeak =
      getCurrentBigTick(now) - getCurrentBigTick(st->lastLeakTime);
  // then run the leak function until we've caught up:
  int32_t newMP = st->membranePotential;
  short lamb = st->lambda;
  short drawnRandom = st->drawnRandomNumber;
  short c = st->c;
  int64_t omega = st->sigma_l * (1 - st->epsilon) +
                  SGN(st->membranePotential) * st->sigma_l * st->epsilon;
  for (; numberOfBigTicksSinceLastLeak > 0; numberOfBigTicksSinceLastLeak--) {
    //    int64_t omega = st->sigma_l * (1 - st->epsilon) +
    //        SGN(st->membranePotential) * st->sigma_l * st->epsilon;

    //    st->membranePotential =
    //        st->membranePotential + (omega * ((1 - st->c) * st->lambda)) +
    //            (st->c & (BINCOMP(st->lambda, st->drawnRandomNumber)));
    // st->membranePotential =
    newMP +=
        (omega * ((1 - st->c) * lamb)) + (c & (BINCOMP(lamb, drawnRandom)));
  }
  st->membranePotential = newMP;
  st->lastLeakTime = now;
}

/** @} ******************************************************/

/**
 * \defgroup TN_REVERSE TN Reverse
 * True North Reverse Leak, Integrate, and Fire Functions
 * @{
 */

/** @} */

/** \defgroup TNParams TN Parameters
 * TrueNorth Neuron Parameter setting functions. Used as helper functions for
 * init
 * @{ */

static void TN_set_neuron_dest(int signalDelay, uint64_t gid,
                               struct NeuronState *n) {
  n->delayVal = signalDelay;
  n->outputGID = gid;
}

/** @} */

//*********************************************************************************
/** \defgroup TNNeuronInit TrueNorth Init
 *  TrueNorth Neuron initialization functions
 * @{ */
/** Constructor / Init a new neuron. assumes that the reset voltage is NOT
 * encoded (i.e.,
 * a reset value of -5 is allowed. Sets reset voltage sign from input reset
 * voltage).*/
static void tn_create_neuron(id_type coreID, id_type nID,
                             bool synapticConnectivity[NEURONS_IN_CORE],
                             short G_i[NEURONS_IN_CORE], short sigma[4],
                             short S[4], bool b[4], bool epsilon, short sigma_l,
                             short lambda, bool c, uint32_t alpha,
                             uint32_t beta, short TM, short VR, short sigmaVR,
                             short gamma, bool kappa, struct NeuronState *n,
                             int signalDelay, uint64_t destGlobalID,
                             int destAxonID) {
  for (int i = 0; i < 4; i++) {
    n->synapticWeight[i] = sigma[i] * S[i];
    n->weightSelection[i] = b[i];
  }
  for (int i = 0; i < NEURONS_IN_CORE; i++) {
    n->synapticConnectivity[i] = synapticConnectivity[i];
    n->axonTypes[i] = G_i[i];
  }

  // set up other parameters
  n->myCoreID = coreID;
  n->myLocalID = nID;
  n->epsilon = epsilon;
  n->sigma_l = sigma_l;
  n->lambda = lambda;
  n->c = c;
  n->posThreshold = alpha;
  n->negThreshold = beta;
  // n->thresholdMaskBits = TM;
  // n->thresholdPRNMask = getBitMask(n->thresholdMaskBits);
  n->sigmaVR = SGN(VR);
  n->encodedResetVoltage = VR;
  n->resetVoltage = VR; //* sigmaVR;

  n->resetMode = gamma;
  n->kappa = kappa;

  //! @TODO: perhaps calculate if a neuron is self firing or not.
  n->firedLast = false;
  n->heartbeatOut = false;
  // n->isSelfFiring = false;
  // n->receivedSynapseMsgs = 0;

  TN_set_neuron_dest(signalDelay, destGlobalID, n);

  // synaptic neuron setup:
  n->largestRandomValue = n->thresholdPRNMask;
  if (n->largestRandomValue > 256) {
    tw_error(TW_LOC, "Error - neuron (%i,%i) has a PRN Max greater than 256\n ",
             n->myCoreID, n->myLocalID);
  }
  // just using this rather than bit shadowing.

  n->dendriteLocal = destAxonID;
  n->outputGID = destGlobalID;

  // Check to see if we are a self-firing neuron. If so, we need to send
  // heartbeats every big tick.
  n->isSelfFiring =
      false; //!@TODO: Add logic to support self-firing (spontanious) neurons
}

static void tn_create_neuron_encoded_rv_non_global(
    int coreID, int nID, bool synapticConnectivity[NEURONS_IN_CORE],
    short G_i[NEURONS_IN_CORE], short sigma[4], short S[4], bool b[4],
    bool epsilon, int sigma_l, int lambda, bool c, int alpha, int beta, int TM,
    int VR, int sigmaVR, int gamma, bool kappa, struct NeuronState *n,
    int signalDelay, int destCoreID, int destAxonID) {
  uint64_t dest_global = getNeuronGlobal(destCoreID, destAxonID);
  tn_create_neuron_encoded_rv(coreID, nID, synapticConnectivity, G_i, sigma, S,
                              b, epsilon, sigma_l, lambda, c, alpha, beta, TM,
                              VR, sigmaVR, gamma, kappa, n, signalDelay,
                              dest_global, destAxonID);
}

static void tn_create_neuron_encoded_rv(
    id_type coreID, id_type nID, bool synapticConnectivity[NEURONS_IN_CORE],
    short G_i[NEURONS_IN_CORE], short sigma[4], short S[4], bool b[4],
    bool epsilon, short sigma_l, short lambda, bool c, uint32_t alpha,
    uint32_t beta, short TM, short VR, short sigmaVR, short gamma, bool kappa,
    struct NeuronState *n, int signalDelay, uint64_t destGlobalID,
    int destAxonID) {
  tn_create_neuron(coreID, nID, synapticConnectivity, G_i, sigma, S, b, epsilon,
                   sigma_l, lambda, c, alpha, beta, TM, VR, sigmaVR, gamma,
                   kappa, n, signalDelay, destGlobalID, destAxonID);
  n->sigmaVR = sigmaVR;
  n->encodedResetVoltage = VR;
  n->resetVoltage = (n->sigmaVR * (pow(2, n->encodedResetVoltage) - 1));
}

/**
 * @brief      Creates a simple neuron for a identity matrix benchmark.
 *  Weights are set up such that axon $n$ has weight 1, where $n$ is the
 *  neuron local id. Other axons have weight 0. Leak is set to zero as well.
 *  The output axon is a randomly selected axon.
 *
 *
 * @param      s     { parameter_description }
 * @param      lp    The pointer to a
 */
static void TN_create_simple_neuron(struct NeuronState *s, tw_lp *lp) {
  // Rewrote this function to have a series of variables that are easier to
  // read.
  // Since init time is not so important, readability wins here.
  // AutoGenerated test neurons:
  static int created = 0;
  bool synapticConnectivity[NEURONS_IN_CORE];
  short G_i[NEURONS_IN_CORE];
  short sigma[4];
  short S[4] = {[0] = 3};
  bool b[4];
  bool const epsilon = 0;
  bool const sigma_l = 0;
  short const lambda = 0;
  bool const c = false;
  short const TM = 0;
  short const VR = 0;
  short const sigmaVR = 1;
  short const gamma = 0;
  bool const kappa = 0;
  int const signalDelay = 0; // tw_rand_integer(lp->rng, 0,5);

  for (int i = 0; i < NEURONS_IN_CORE; i++) {
    // s->synapticConnectivity[i] = tw_rand_integer(lp->rng, 0, 1);
    s->axonTypes[i] = 1;
    G_i[i] = 0;
    synapticConnectivity[i] = 0;
    // synapticConnectivity[i] = tw_rand_integer(lp->rng, 0, 1)
  }

  id_type const myLocalID = getNeuronLocalFromGID(lp->gid);

  synapticConnectivity[myLocalID] = 1;

  //(creates an "ident. matrix" of neurons.
  for (int i = 0; i < 4; i++) {
    // int ri = tw_rand_integer(lp->rng, -1, 0);
    // unsigned int mk = tw_rand_integer(lp->rng, 0, 1);

    // sigma[i] = (!ri * 1) + (-1 & ri))
    // sigma[i] = (mk ^ (mk - 1)) * 1;
    sigma[i] = 1;
    b[i] = 0;
  }

  // int64_t alpha = tw_rand_integer(lp->rng, THRESHOLD_MIN, THRESHOLD_MAX);
  // int64_t beta = tw_rand_integer(lp->rng, (NEG_THRESH_SIGN *
  // NEG_THRESHOLD_MIN), NEG_THRESHOLD_MAX);
  int32_t const alpha = 1;
  int32_t const beta = -1;
  // DEBUG LINE

  tn_create_neuron_encoded_rv(
      getCoreFromGID(lp->gid), getNeuronLocalFromGID(lp->gid),
      synapticConnectivity, G_i, sigma, S, b, epsilon, sigma_l, lambda, c,
      alpha, beta, TM, VR, sigmaVR, gamma, kappa, s, signalDelay, 0, 0);
  // we re-define the destination axons here, rather than use the constructor.

  float const remoteCoreProbability = .905;
  long int dendriteCore = s->myCoreID;
  // This neuron's core is X. There is a 90.5% chance that my destination will
  // be X - and a 10% chance it will be a different core.
  if (tw_rand_unif(lp->rng) < remoteCoreProbability) {
    //		long dendriteCore = s->myCoreID;
    //		dendriteCore = tw_rand_integer(lp->rng, 0, CORES_IN_SIM - 1);
    dendriteCore = tw_rand_integer(lp->rng, 0, CORES_IN_SIM - 1);
  }

  /**@note This random setup will create neurons that have an even chance of
   * getting an axon inside thier own core
   * vs an external core. The paper actually capped this value at something like
   * 20%. @todo - make this match the
   * paper if performance is slow. * */
  // s->dendriteCore = tw_rand_integer(lp->rng, 0, CORES_IN_SIM - 1);
  s->dendriteLocal =
      s->myLocalID; // tw_rand_integer(lp->rng, 0, AXONS_IN_CORE - 1);
  //     if (tnMapping == LLINEAR) {
  s->outputGID = getAxonGlobal(dendriteCore, s->dendriteLocal);
  created++;
}

/**
 * Details - must be called from TN_init after TN_create_simple_neuron() has
 * been called.
 * @param s
 * @param lp
 */
static void TN_create_saturation_neuron(struct NeuronState *s, tw_lp *lp) {

  static uint64_t numCreated = 0;
  bool synapticConnectivity[NEURONS_IN_CORE];
  getSynapticConnectivity(synapticConnectivity, lp);
  for (int i = 0; i < NEURONS_IN_CORE; i++) {
    // s->synapticConnectivity[i] = tw_rand_integer(lp->rng, 0, 1);
    s->axonTypes[i] = 1;
    s->synapticConnectivity[i] = synapticConnectivity[i];
  }
  for (int i = 0; i < NUM_NEURON_WEIGHTS; i++) {
    s->synapticWeight[i] = connectedWeight;
  }
  s->lambda = SAT_NET_LEAK;
  s->c = SAT_NET_STOC;
  s->posThreshold = SAT_NET_THRESH;
  s->negThreshold = (0 - SAT_NET_THRESH);
  numCreated++;
  if (numCreated >= NEURONS_IN_CORE * CORES_IN_SIM) {
    printf("SAT network finished init \n");
    clearBucket();
  }
}

static void modelErr(char *element, char *filename, id_type coreID, id_type lID,
                     char *tp, char *fnDat, int codeline) {
  char *em = calloc(4096, sizeof(char));
  char *linebreak = calloc(128, sizeof(char));
  int i = 0;
  for (; i < 100; i++) {
    linebreak[i] = '-';
  }
  linebreak[i + 1] = '\n';
  sprintf(em, "%s\n%s", em, linebreak);
  char *emp1 =
      "Error while loading NeMo Model config file. Possible bad option.\n";
  sprintf(em, "%s", emp1);
  sprintf(em, "%s\n Filename: %s", em, filename);
  char loc[128];
  sprintf(loc, "%i%s%i", coreID, tp, lID);
  sprintf(em, "%s \n NeuronID: %s", em, loc);
  sprintf(em, "%s \n While loading parameter %s ", em, element);
  sprintf(em, "%s\n%s", em, linebreak);
  sprintf(em, "%s \n -- Error type: %s \n", em, fnDat);
  sprintf(em, "%s\n%s", em, linebreak);
  sprintf(em, "%s%s", em, "Config File Details: \n");
  getModelErrorInfo(coreID, lID, tp, element, 0);
  printf("%s", em);
  tw_error(TW_LOC, em);
}

static int safeGetArr(int direct, char *lutName, char *dirName, long vars[],
                      int expectedParams, int cid, int lid, char *ntype) {
  char *errtps[4] = {"0 - No Parameters Loaded From File",
                     "1 - Too Few Parameters Loaded From File",
                     "2 - Parameter Not Found - Assuming 0 values.",
                     "3 - Too Many Values Found"};
  int validation = 0;
  char *pn;
  if (direct) {
    validation = lGetAndPushParam(dirName, 1, vars);
    pn = dirName;
  } else {
    validation = lGetAndPushParam(luT(lutName), 1, vars);
    pn = lutName;
  }
  ++validation;
  if (validation == expectedParams) {
    return validation;
  } else if (validation == 0) {
    modelErr(pn, NEMO_MODEL_FILE_PATH, cid, lid, ntype, errtps[0], 917);
  } else if (validation < expectedParams && validation > 0) {
    modelErr(pn, NEMO_MODEL_FILE_PATH, cid, lid, ntype, errtps[1], 917);
  } else if (validation > expectedParams) {
    modelErr(pn, NEMO_MODEL_FILE_PATH, cid, lid, ntype, errtps[3], 917);
  } else {
    modelErr(pn, NEMO_MODEL_FILE_PATH, cid, lid, ntype, errtps[2], 917);
  }
  return validation;
}
//**** Some loader helper macros.
//! Debug network
static FILE *core_connectivity_map;
//! @TODO: Move these helper macros somewhere better! .
#define LGT(V) (lGetAndPushParam(luT((V)), 0, NULL))
//#define GA(N, T) (getArray( (#N) , &(N), (T) ))
#define TID core, nid, "TN"
static long num_neg_found = 0;
static void TNPopulateFromFile(struct NeuronState *st, tw_lp *lp) {

  int const extraParamCache = 32;
  // Set up neuron - first non array params:
  tw_lpid outputGID = 0;
  id_type const core = getCoreFromGID(lp->gid);
  id_type const nid = getNeuronLocalFromGID(lp->gid);

  long const outputCore = lGetAndPushParam("destCore", 0, NULL);
  long const outputLID = lGetAndPushParam("destLocal", 0, NULL);
#ifdef DEBUG
  static int found_one = 0;

  if (outputCore < -100) {
    if (found_one == 0) {
      tw_printf(TW_LOC, "Found a negative core! Good! %lli", outputCore);
      found_one = 1;
    }
    num_neg_found++;
  }
#endif
  if (outputCore < 0 || outputLID < 0) {
    // st->outputGID = 0;
    st->isOutputNeuron = true;

    st->outputCoreDest = outputCore;
    st->outputNeuronDest = outputLID;

  } else {
    outputGID = getAxonGlobal(outputCore, outputLID);
    st->outputGID = outputGID;
    st->outputCoreDest = outputCore;
    st->outputNeuronDest = outputLID;
  }
  luT("lambda");

  short const lambda = LGT("lambda");
  short const resetMode = LGT("resetMode");
  LGT("resetVoltage");
  short const sigmaVR = LGT("sigmaVR");
  LGT("encodedResetVoltage");
  short const sigma_l = LGT("sigma_l");
  bool const isOutputNeuron = LGT("isOutputNeuron");
  bool const epsilon = LGT("epsilon");
  bool const c = LGT("c");
  bool const kappa = LGT("kappa");
  // bool isActiveNeuron = 		LGT("isActiveNeuron");
  bool const isActiveNeuron = true;
  int32_t const alpha = lGetAndPushParam("alpha", 0, NULL);
  int32_t const beta = lGetAndPushParam("beta", 0, NULL);

  short const TM = lGetAndPushParam("TM", 0, NULL);
  short const VR = lGetAndPushParam("VR", 0, NULL);
  short const gamma = lGetAndPushParam("gamma", 0, NULL);

  bool synapticConnectivity[NEURONS_IN_CORE];
  short axonTypes[NEURONS_IN_CORE];
  short sigma[NUM_NEURON_WEIGHTS];
  short S[NUM_NEURON_WEIGHTS];
  bool b[NUM_NEURON_WEIGHTS];

  long vars[NEURONS_IN_CORE + extraParamCache];
  long validation = 0;

  //@TODO: move the string types to a lookup table

  validation =
      safeGetArr(0, "synapticConnectivity", NULL, vars, NEURONS_IN_CORE, TID);
  //	validation = lGetAndPushParam(luT("synapticConnectivity"), 1, vars);
  if (validation > 0 && validation == NEURONS_IN_CORE) {
    for (int i = 0; i < validation; i++)
      synapticConnectivity[i] = vars[i];
  }
  validation = safeGetArr(0, "axonTypes", NULL, vars, NEURONS_IN_CORE, TID);
  //	validation = lGetAndPushParam(luT("axonTypes"), 1, vars);
  if (validation > 0 && validation == NEURONS_IN_CORE) {
    for (int i = 0; i < validation; i++)
      axonTypes[i] = vars[i];
  }
  validation = safeGetArr(0, "sigma", NULL, vars, NUM_NEURON_WEIGHTS, TID);
  //	validation = lGetAndPushParam(luT("sigma"), 1, vars);
  if (validation > 0 && validation == NUM_NEURON_WEIGHTS) {
    for (int i = 0; i < validation; i++)
      sigma[i] = vars[i];
  }
  validation = safeGetArr(1, NULL, "S", vars, NUM_NEURON_WEIGHTS, TID);
  //	validation = lGetAndPushParam("S", 1, vars);
  if (validation > 0 && validation == NUM_NEURON_WEIGHTS) {
    for (int i = 0; i < validation; i++)
      S[i] = vars[i];
  }
  validation = safeGetArr(1, NULL, "b", vars, NUM_NEURON_WEIGHTS, TID);
  //	validation = lGetAndPushParam(luT("b"), 1, vars);
  if (validation > 0 && validation == NUM_NEURON_WEIGHTS) {
    for (int i = 0; i < validation; i++)
      b[i] = vars[i];
  }
  tn_create_neuron_encoded_rv(core, nid, synapticConnectivity, axonTypes, sigma,
                              S, b, epsilon, sigma_l, lambda, c, alpha, beta,
                              TM, VR, sigmaVR, gamma, kappa, st, 0, outputGID,
                              outputLID);
  st->resetMode = resetMode;
  st->isActiveNeuron = isActiveNeuron;
  st->isOutputNeuron = isOutputNeuron;
  //    if(g_tw_mynode == 0){
  //    	if(st->myLocalID==NEURONS_IN_CORE - 1 || st->myLocalID == 0){
  //		printf("Completed loading neurons in core %i", st->myCoreID);
  //	}
  //    }
  if (outputGID >= SIM_SIZE) {
    // printf("err cond 1.\n");
    // st->isActiveNeuron = false;
  }

  clearNeuron(core, nid);
  clearStack();
}

/** @} */
/** attempts to find this neuron's def. from the file input.
 Files are assumed to be inited during main().


 */
static void TNCreateFromFile(struct NeuronState *s, tw_lp *lp) {

  static int needannounce = 1;

  // first, get our Neuron IDs:
  id_type core = getCoreFromGID(lp->gid);
  id_type nid = getNeuronLocalFromGID(lp->gid);
  s->myCoreID = core;
  s->myLocalID = nid;
  char *nt = "TN";

  // If we are using JSON, call the JSON loader lib instead of the LUA stack
  // if(NEMO_MODEL_IS_TN_JSON){
  //  loadNeuronFromJSON(core,nid,s);
  //  if(s->outputCoreDest < 0 ){
  //    s->outputGID = 0;
  //    s->isOutputNeuron = 1;

  //  }else{
  //    s->outputGID = getGIDFromLocalIDs(core,nid);
  //  }
  //}
  if (NEMO_MODEL_IS_BINARY) {
    // load the binary file info.
    bool found = loadNeuronFromBIN(core, nid, s);
    if (!found) {
      s->isActiveNeuron = false;
    }

  } else {
    int nNotFound = lookupAndPrimeNeuron(core, nid, nt);
    //printf("Found status: %i \n ", nNotFound);
    if (nNotFound) {
      s->isActiveNeuron = false;

    } else {
      TNPopulateFromFile(s, lp);
    }
  }

  if (needannounce && (g_tw_mynode == 0) && s->isOutputNeuron) {
    printf("output neuron created.\n");
    printf("Neuron CORE: %lli - LID: %lli - Dest Core: %li  Local: %li \n",
           s->myCoreID, s->myLocalID, s->outputCoreDest, s->outputNeuronDest);
    needannounce = 0;
  }
}

void TN_pre_run(struct NeuronState *s, tw_lp *me) {

  static bool clean = false;
  static bool core_con_open = false;

  /** @todo: remove this once debugging connections is done
    //DUMB CSV DEBUG
     */
  debug_neuron_connections(s, me);
  //    if(!clean){
  //        debug_init_neuron_json();
  //    }
  //    debug_add_neuron_to_json(s,me);
  /////////////////////////

  if (clean == false) {
#ifdef DEBUG
    tw_printf(TW_LOC, "Lua cleanup from neuron.\n");
    tw_printf(TW_LOC, " Found %lli debug cores.\n", num_neg_found);
#endif
    closeLua();
    clean = true;
  }
  //  if (SAVE_NETWORK_STRUCTURE) {
  //    saveNeuronPreRun();
  //  }
  if (SAVE_NETWORK_STRUCTURE) {
    if (core_con_open == false) {
#ifdef DEBUG
      tw_printf(TW_LOC, "Core Connectivity Map Init.\n");
      // saveNetworkStructureMPI();
#endif
      char *fn = calloc(128, sizeof(char));
      sprintf(fn, "core_con_r%li.csv", g_tw_mynode);
      core_connectivity_map = fopen(fn, "w");
      free(fn);
      core_con_open = true;
      if (g_tw_mynode == 0) {
        fprintf(core_connectivity_map, "from_core,to_core\n");
      }
    }

    fprintf(core_connectivity_map, "%llu,%llu\n", s->myCoreID,
            getCoreFromGID(s->outputGID));
    saveNeuronPreRun();
  }
}

/** /defgroup TN_ROSS_HANDLERS
 * Implementations of TN functions that are called by ross. Forward, reverse,
 * init, etc.
 * @{
 * */

void TN_init(struct NeuronState *s, tw_lp *lp) {
#ifdef DEBUG
  static int an = 0;
  if (!an && g_tw_mynode == 0) {
    if (SAVE_NETWORK_STRUCTURE) {
      tw_printf(TW_LOC, "SAVING NETWORK STRUCTURE\n");
    } else {
      tw_printf(TW_LOC, "NOT SAVING NETWORK STRUCTURE\n");
    }
    an = 1;
  }

#endif
  static u_int8_t fileInit = 0;
  if (DO_DUMPI) {
    if (!fileInit) {
      char *fn = calloc(sizeof(char), 256);
      sprintf(fn, "dumpi_virt-%i_rnk%li.txt", getCoreFromGID(lp->gid),
              g_tw_mynode);
      dumpi_out = fopen(fn, "w");
      free(fn);
      fileInit = 1;
    }
  }
  if (fileInit == 0 || fileInit == 1) {
    printf("Starting tn network init\n");
    fileInit = 3;
  }
  static bool announced = false;
  // s->neuronTypeDesc = "SIMPLE";
  if (DEBUG_MODE && !announced) {
    printf("Creating neurons\n");
    announced = true;
  }
  if (FILE_IN) {
    TNCreateFromFile(s, lp);

  } else {
    TN_create_simple_neuron(s, lp);
    // This if statement should not be needed, due to check in setupGrud.
    if ((LAYER_NET_MODE & GRID_LAYER) ||
        (LAYER_NET_MODE & CONVOLUTIONAL_LAYER)) {
      configureNeuronInLayer(s, lp);
    } else if (IS_SAT_NET) {
    }
    TN_create_saturation_neuron(s, lp);
  }
  if (SAVE_NETWORK_STRUCTURE) {
    // tw_printf(TW_LOC,"Network structure saving...\n");
    saveNeuronNetworkStructure(s);
  } else {
    //#ifdef DEBUG
    //    tw_printf(TW_LOC, "Not saving network structure?\n");
    //#endif
  }

  s->energy_stat.spike_count = 0;
  s->energy_stat.rng_count = 0;
  s->energy_stat.sops_count = 0;
  s->energy_stat.output_count = 0;

  s->energy_stat.dest_core = s->outputCoreDest;
  s->energy_stat.dest_neuron = s->outputNeuronDest;
  s->energy_stat.my_neuron = s->myLocalID;
  s->energy_stat.my_core = s->myCoreID;
}

void TN_forward_event(struct NeuronState *s, tw_bf *CV, struct messageData *m,
                      tw_lp *lp) {
  long const start_count = lp->rng->count;
  // This is the primary entry point to the neuron behavior.
  // Add metrics and stats around this function.
  bool const fired = TNReceiveMessage(s, m, lp, CV);

  s->energy_stat.sops_count++;
  if (fired) {
    s->energy_stat.spike_count++;
    s->energy_stat.output_count++;
  }
  /**@todo save message trace here: */

  CV->c0 = fired; // save fired information for reverse computation.

  m->rndCallCount = lp->rng->count - start_count;
  s->energy_stat.rng_count += m->rndCallCount;
}

void TN_reverse_event(struct NeuronState *s, tw_bf *CV, struct messageData *m,
                      tw_lp *lp) {
  if (!s->isActiveNeuron) {
    return;
  }
  long count = m->rndCallCount;
  //    tw_snapshot_restore(lp, lp->type->state_sz);

  TNReceiveReverseMessage(s, m, lp, CV);
  s->energy_stat.sops_count--;
  if (CV->c0) {
    s->energy_stat.spike_count--;
    s->energy_stat.output_count--;
  }
  while (count--)
    tw_rand_reverse_unif(lp->rng);
}

/* Debug - special case - core 0 seems to be hyper-active */
#ifdef DEBUG
static FILE *debug_core;
static int debug_core_open = 0;
#endif

/**
 * TN_save_events - Saves an event to the output file.
 * Moved this functionality into it's own function to make testing easier.
 * Will call the neuron event save function if this neuron sent a spike AND:
 * if SAVE_SPIKE_EVTS is set,
 * or if isOutputNeuron is true AND SAVE_OUTPUT_NEURON_EVTS is set.
 * Default output core/neuron is -909 to catch possible invalid values.
 * @param s
 * @param csv
 * @param m
 * @param lp
 */
static void TN_save_events(struct NeuronState *s, tw_bf *cv, struct messageData *m,
                           tw_lp *lp) {

  unsigned int fired = cv->c0 | cv->c31 | cv->c10;
  if (fired > 0) {
    if (SAVE_SPIKE_EVTS || SAVE_OUTPUT_NEURON_EVTS) {
      saveNeuronFire(tw_now(lp), getCoreFromGID(lp->gid),
                     getLocalFromGID(lp->gid), s->outputGID,
                     getCoreFromGID(s->outputGID),
                     getLocalFromGID(s->outputGID), s->isOutputNeuron);
    }
  }
}

/** TN_commit is a function called on commit. This is used for management of
 * neurons! */
void TN_commit(struct NeuronState *s, tw_bf *cv, struct messageData *m, tw_lp *lp) {
  // if neuron has fired and save neuron fire events is enabled, save this
  // event.
  // if (SAVE_SPIKE_EVTS && cv->c0) {
  //  saveNeuronFire(tw_now(lp), s->myCoreID, s->myLocalID,
  //  s->outputGID,getCoreFromGID(s->outputGID),getLocalFromGID(s->outputGID),0);
  //}
#ifdef DEBUG
  static int displayFlag = 0;
  if (displayFlag == 0) {
    if (s->myCoreID == 4041) {
      printf("-----------------------------------------------------------------"
             "-------------------------------\n");
      printf("Neuron 4041 commit fn. Dest neuron is %i\n", s->outputNeuronDest);
      displayFlag = 1;
    }
  }
#endif
  /// Save output neuron events
  // can save either all spike even
  TN_save_events(s, cv, m, lp);
  // save simulated dumpi trace if inter core and dumpi trace is on
  if (cv->c31 && DO_DUMPI) {
    // saveMPIMessage(s->myCoreID, getCoreFromGID(s->outputGID), tw_now(lp),

    //			   dumpi_out);
    setrnd(lp);
    saveSendMessage(s->myCoreID, getCoreFromGID(s->outputGID), tw_now(lp), 0,
                    dumpi_out);
  }
  // save energy stats:
  int my_gid = g_tw_mynode;
  save_energy_stats(&s->energy_stat, my_gid);

#ifdef DEBUG
  // Debug - special log case for neuron 0
  if (!debug_core_open) {
    debug_core_open = 1;
    debug_core = fopen("debug_core_0.csv", "w");
    fprintf(debug_core, "TYPE,CORE_ID,NEURON_ID,SEND_GID,SEND_CORE,SEND_AXON,"
                        "TIME,RCV_FROM_AXON\n");
  }
  if (getCoreFromGID(lp->gid) == 0) { //&& cv->c0){
    // core 0 fired
    char tp;
    int recv_axon = -1;
    if (cv->c0) {
      tp = 'S';
      //      fprintf(debug_core,"%c,%llu,%llu,%llu,%li,%li,%f,%i\n",tp,s->myCoreID,s->myLocalID,s->outputGID,
      //              s->outputCoreDest,s->outputNeuronDest,tw_now(lp),recv_axon);
    } else {
      tp = 'R';
      recv_axon = m->localID;
    }
    fprintf(debug_core, "%c,%llu,%llu,%llu,%li,%li,%f,%i\n", tp, s->myLocalID,
            s->myCoreID, s->outputGID, s->outputCoreDest, s->outputNeuronDest,
            tw_now(lp), recv_axon);
  }
#endif
}

void TN_final(struct NeuronState *s, tw_lp *lp) {
  static int fileOpen = 1;
  if (fileOpen) {
    /////////// DEBUG CODE REMOVE WHEN DONE /////////////
    // debug_close_neuron_json();
    if (DO_DUMPI) {
      fclose(dumpi_out);
      fileOpen = 0;
    }
    if (SAVE_NETWORK_STRUCTURE) {
      fclose(core_connectivity_map);
    }
  }
  if (DO_DUMPI && fileOpen) {
  }
  if (g_tw_synchronization_protocol == OPTIMISTIC_DEBUG) {
    // Alpha, SOPS should be zero. HeartbeatOut should be false.
    char *em = (char *)calloc(1024, sizeof(char));
    char *hdr = "------ Neuron Optimistic Debug Check -----";
    sprintf(em, "%s\n Core: %i Local: %i \n", hdr, s->myCoreID, s->myLocalID);
  }
  if (SAVE_NETWORK_STRUCTURE) {
    saveIndNeuron(s);
  }
}

inline struct NeuronState *TN_convert(void *lpstate) {
  return (struct NeuronState *)lpstate;
}

/**@}*/
/** RIO Functions for neuron config  @{ **/
size_t tn_size(struct NeuronState *s, tw_lp *lp) {
  size_t neuronSize = sizeof(struct NeuronState);
  return neuronSize;
}
void tn_serialize(struct NeuronState *s, void *buffer, tw_lp *lp) {
  memcpy(buffer, s, sizeof(struct NeuronState));
}
void tn_deserialize(struct NeuronState *s, void *buffer, tw_lp *lp) {
  memcpy(s, buffer, sizeof(struct NeuronState));
}
