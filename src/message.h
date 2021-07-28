#ifndef SRC_MESSAGE_H
#define SRC_MESSAGE_H

#include "globals.h"

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

//Message Structure (Used Globally so placed here)
struct messageData {
  enum evtType eventType;
  unsigned long rndCallCount;
  id_type localID; //!< Sender's local (within a core) id - used for weight lookups.
  unsigned long long isRemote;
  //long double remoteRcvTime;
  union {
    unsigned long synapseCounter;
    struct {
      int32_t neuronVoltage;
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
};

#endif /* end of include guard */
