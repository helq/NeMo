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

/** @defgroup tempConfig Temporary configuration globals
 *	These global defines are stored here before I migrate them into either a run-time
 *	or compile-time option
 * @{ */
#define SAVE_NEURON_STATS true
/**@}*/

/**
 * Neuron file read buffer size - for reading CSV files.
 */
#define NEURON_BUFFER_SZ  32

#define N_FIRE_BUFF_SIZE 4096
#define TXT_HEADER "****************************************************************************\n"
#define MAX_RANKS_FILES  65535

#define DBG_MODEL_MSGS 0

/** @defgroup types Typedef Vars
 * Typedefs to ensure proper types for the neuron parameters/mapping calculations
 */
/**@{  */

typedef int_fast64_t id_type;  //!< id type is used for local mapping functions - there should be $n$ of them depending on CORE_SIZE
typedef int32_t volt_type;     //!< volt_type stores voltage values for membrane potential calculations
typedef uint64_t size_type;    //!< size_type holds sizes of the sim - core size, neurons per core, etc.
typedef uint64_t stat_type;
/**@}*/


/**
 * @defgroup gmacros Global Macros and Related Functions
 *Global Macros */
/**@{ */

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
/** @} */

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

/** @defgroup global_structs_enums Global Structs and Enums
  * Global structs and enums, including event types, lp types, and the message structure
  */
/** @{ */

/** evtType is a message/event identifier flag */
enum evtType {
  AXON_OUT, //!< Message originates from an axon
  AXON_HEARTBEAT, //!< Axon heartbeat message - big clock synchronization.
  SYNAPSE_OUT, //!< Message originates from a synapse
  SYNAPSE_HEARTBEAT, //!< Message is a synapse heartbeat message.
  NEURON_OUT, //!< Message originates from a neuron, and is going to an axion.
  NEURON_HEARTBEAT, //!< Neuron heartbeat messages - for big clock syncronization.
  NEURON_SETUP, //!< Message that contains a neuron's setup information for the synapse - connectivity info
  GEN_HEARTBEAT //!< Signal generator messages -- used to simulate input for benchmarking.
};

typedef enum LayerTypes {
  NON_LAYER = 0,
  GRID_LAYER = 1 << 1,
  CONVOLUTIONAL_LAYER = 1 << 2,
  OUTPUT_UNQ = 1 << 3,
  OUTPUT_RND = 1 << 4
} layerTypes;
/**@} */

//Message Structure (Used Globally so placed here)
typedef struct Ms {
  enum evtType eventType;
  unsigned long rndCallCount;
  id_type localID; //!< Sender's local (within a core) id - used for weight lookups.
  unsigned long long isRemote;
  //long double remoteRcvTime;
  union {
    unsigned long synapseCounter;
    struct {
      volt_type neuronVoltage;
      tw_stime neuronLastActiveTime;
      tw_stime neuronLastLeakTime;
      uint16_t neuronDrawnRandom;
    };
  };

  // union{
  id_type axonID; //!< Axon ID for neuron value lookups.
  bool *neuronConn;
  //};
  //message tracking values:
#ifdef SAVE_MSGS
  union {
    uint64_t uuid;
    struct {
      uint16_t idp1;
      uint16_t idp2;
      uint32_t idp3;
    };
  };
  tw_lpid originGID;
  char originComponent;

  //tw_stime msgCreationTime;
#endif
} messageData;
/**@}*/

/**
 * \defgroup Globals Global Variables
 * @{
 */
extern size_type LPS_PER_PE;
extern size_type SIM_SIZE;
extern size_type LP_PER_KP;

extern size_type CORES_IN_SIM;
extern size_type CORE_SIZE;
extern size_type SYNAPSES_IN_CORE;

extern bool BULK_MODE;
extern bool DEBUG_MODE;
extern bool SAVE_MEMBRANE_POTS;
extern bool SAVE_SPIKE_EVTS; //!< Toggles saving spike events
extern bool SAVE_NETWORK_STRUCTURE;
extern unsigned int SAVE_OUTPUT_NEURON_EVTS;
extern bool BINARY_OUTPUT;

extern bool PHAS_VAL;
extern bool TONIC_BURST_VAL;
extern bool PHASIC_BURST_VAL;
extern bool VALIDATION;

extern bool FILE_OUT;
extern bool FILE_IN;

/** value for the size of a processor. Defaults to 4096 cores per proc */
extern unsigned int CORES_IN_CHIP;
extern unsigned int NUM_CHIPS_IN_SIM;
extern unsigned int CHIPS_PER_RANK;

/** @defgroup ctime Compute Time Parameters
 * Variables that change DUMPI compute time / send time @{
 */
extern long double COMPUTE_TIME;
extern long double SEND_TIME_MIN;
extern long double SEND_TIME_MAX;
extern unsigned int DO_DUMPI;

/**
 * clock random value adjuster.
 */
extern tw_stime CLOCK_RANDOM_ADJ;
/** @} */

/** @defgroup satnet Saturation network flags / settings @{ */
extern unsigned int SAT_NET_PERCENT;
extern unsigned int SAT_NET_COREMODE;
extern unsigned int SAT_NET_THRESH;
extern unsigned int SAT_NET_LEAK;
extern unsigned int SAT_NET_STOC;
extern unsigned int IS_SAT_NET;

/** @} @defgroup laynet Basic Layer Network Settings @{ */
//typedef struct LParams {
//    unsigned long numLayersInSim;
//    layerTypes LAYER_NET_MODE;
//    unsigned int layerSizes[4096];
//};
extern unsigned int NUM_LAYERS_IN_SIM;
extern layerTypes LAYER_NET_MODE;
extern unsigned int LAYER_SIZES[4096];
extern unsigned int CORES_PER_LAYER;
extern unsigned int CHIPS_PER_LAYER;
extern unsigned int GRID_ENABLE;
extern unsigned int GRID_MODE;
extern unsigned int RND_GRID;
extern unsigned int RND_UNIQ;
extern unsigned int UNEVEN_LAYERS;
extern char *LAYER_LAYOUT;
/**@} @defgroup iocfg File buffer settings
 * @{
 * */

extern char *luaConfigFile; //!< Stores LUA configuration file in memory for performance increase.
extern long isBin;

/** POSIX Neuron Fire record line buffer size.
 * For text mode only, sets the length of strings stored in the neuron fire buffer*/
extern int N_FIRE_LINE_SIZE;


/** dumpi noise/squash generation toggle - for reading in model files that ignore
 * spike files or for squashing spike times.
 * Mode 1: Will read in the spike CSV file to determine the
 * input axons, then will make those axons spike every tick to generate traffic.
 * Mode 2: Will read in the spike CSV file as normal, but it will compress the input
 * to be sequentially incrementing spike times. For example, if an axon is supposed
 * to fire at time 1, 4, 10, this axon will spike at times 0,1,2.
 *
 * Default is 0: No spike modifications.s
 */
extern char DUMPI_NOISE;

//#define N_FIRE_LINE_SIZE 128
/** @} */



/** @defgroup fileNames File Names
 * Vars that manage file names for IO.
 * These variables include the paths to the model file,
 * the path to the spike input file,
 * and file loading options.
 * @{*/
extern char NEMO_MODEL_FILE_PATH[512];
extern char NEMO_SPIKE_FILE_PATH[512];
//extern bool NEMO_MODEL_IS_TN_JSON;
extern bool NEMO_MODEL_IS_BINARY;
/** @} */

#endif // __NEMO_GLOBALS_H__
