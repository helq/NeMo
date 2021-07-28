//
// Created by Mark Plagge on 5/25/16.
//

#ifndef __NEMO_TN_NEURON_H__
#define __NEMO_TN_NEURON_H__
#include "tn_neuron_struct.h"
#include "../tests/tomacs_exp.h"
#include "../message.h"

#ifdef NET_IO_DEBUG
#include <stdarg.h>
#endif

/** DUMPI FILE */


/**
 * @brief      True North Forward Event handler
 *
 * @param      s  The tn neuron state
 * @param      CV               flags for message flow
 * @param      m      The message data
 * @param      lp               The pointer to a LP
 */
void TN_forward_event(tn_neuron_state *s, tw_bf *CV, struct messageData *m, tw_lp *lp);

/**
 * @brief      True North Reverse Event Handler
 *
 * @param      s  The tn neuron state
 * @param      CV               flags for message flow
 * @param      m      The message data
 * @param      lp               The pointer to a
 */
void TN_reverse_event(tn_neuron_state *s, tw_bf *CV, struct messageData *m, tw_lp *lp);

void TN_commit(tn_neuron_state *s, tw_bf *cv, struct messageData *m, tw_lp *lp);

/**
 * @brief      Initialize a TrueNorth neuron
 *
 * @param      s     The TN State
 * @param      lp    The pointer to the LP
 */
void TN_init(tn_neuron_state *s, tw_lp *lp);

/**
 * TN_pre_run cleans up the input files after an initialization/
 * @param s
 * @param me
 */
void TN_pre_run(tn_neuron_state *s, tw_lp *me);
/**
 * @brief      The TN neuron final function
 *
 * @param      s     TN State
 * @param      lp    The pointer to an LP
 */
void TN_final(tn_neuron_state *s, tw_lp *lp);

/**
 * @brief	This takes a void pointer and returns this neuron's struct.
 * This is used for managing super synapse direct communication functionality.
 */

//inline tn_neuron_state *TN_convert(void *lpstate);

size_t tn_size(tn_neuron_state *s, tw_lp *lp);
void tn_serialize(tn_neuron_state *s, void *buffer, tw_lp *lp);
void tn_deserialize(tn_neuron_state *s, void *buffer, tw_lp *lp);

#endif  // NEMO_TN_NEURON_H
