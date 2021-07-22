#ifndef NEMO_MODEL_READER_H
#define NEMO_MODEL_READER_H

#include "../globals.h"
#include "../neuro/tn_neuron_struct.h"

#define SAVE_ALL_NEURON_PARAMS 1
enum modelReadMode {
  START_READ,
  MODEL_HDR,
  N_TYPE,
  N_CORE,
  N_LOCAL,
  N_CONNECTIVITY,
  N_AXONTYPES,
  N_SGI,
  N_SP,
  N_BV,
  N_PARAMS
};

/** @defgroup modelReading @{ */
/** lookupNeuron finds a neuron in the config file, and pushes it's table to the top of the lua stack.
 * Call this function first, then call lPushParam() followed by lGetParam()
 * @param coreID
 * @param localID
 * @param nt NeuronType: can be "TN"
 * @return -1 if not found, 0 if found.
 */
int lookupAndPrimeNeuron(long coreID, long localID, char *nt);

/**
 * Once a neuron is selected, this pushes a parameter from the table to the top of the stack .
 * Run this after lookupNeuron()
 * @param paramName
 */
void lPushParam(char *paramName);

/**
 * onece a parameter is pushed (see lpushParam()), this extracts the parameter.
 * If parameter is an array, returns the number of elements placed in the array specified by arrayParam.
 * Otherwise, returns the value from the config file (that was specified in the lPushParam() function.
 * @param isArray
 * @param arrayParam
 * @return
 */
long long int lGetParam(int isArray, long *arrayParam);

/**
 * a helper function that calls lPushParam then lGetParam.
 * @param paramName
 * @param isArray
 * @param arrayParam
 * @return If no neuron is found -1 - otherwise it returns the value, or the length of values found in the array.
 */
long lGetAndPushParam(char *paramName, int isArray, long *arrayParam);

/**
 * A model read error helper function */

void getModelErrorInfo(int ncore, int nlocal, char *ntype, char *paramName, int errorno);

/**
 * Prepares the model file for loadiing. Loads the file into memory and stores it in a list.
 * @param filename the model filename, must be a NeMo CSV.
 * @param maxNeurons The maximum number of neurons in the model file. Is an upper bound on the list size. If set to -1,
 * will be estimated from the number of lines in the file.
 */
void initModelInput(unsigned long maxNeurons);

void clearNeuron(int curCoreID, int curLocalID);
void clearStack();
void closeLua();
/** @} */

/**
 * for reading in TN neuron structs saved to file. Currently this binary system only handles TN LPs saved from
 * the JSON parser.
 * @param binFileName
 * @return
 */
int openBinaryModelFile(char * binFileName);
/**
 * Initializes the library of neuron structs from the binary file.
 * @return the number of neurons loaded.
 */
long setupBinaryNeurons();
/**
 * initializes the given neuron, if it exists in the neuron library.
 * @param neuronCore
 * @param neuronLocal
 * @param n
 */
bool loadNeuronFromBIN(id_type neuronCore, id_type neuronLocal, tn_neuron_state *n);
/**@}*/

/**
 * Closes the binary file and frees the library of neurons.
 */
void closeBinaryModelFile();

/**
 * uses a lua table to lookup the nemo config variable name from a TN Name
 * @param nemoName
 * @return
 */
char *luT(char *nemoName);

#endif  // ifndef NEMO_MODEL_READER_H
