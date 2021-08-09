//
// Created by Mark Plagge on 8/2/16.
//
#include <rqueue.h>
#include "energy_stats.h"
#include "output.h"
#include <assert.h>

#define BINARY_OUTPUT false
#define N_FIRE_BUFF_SIZE 4096

static FILE *neuronFireFile;
static FILE *neuronFireFileBinary;

// Names of output files
static char *neuronFireFinalFN;
static char *neuronRankFN;
static char *NEURON_FIRE_R_FN =
    "fire_record"; // <-- Global variable sets the name of the fire record.

// Global Memory Pool Position Counters
static int neuronFirePoolPos = 0;

/* Text Mode Buffers */
static char **neuronFireBufferTXT;

/** @defgroup THDIO threaded fileIO for sim performance @{ */

/**
 * The ringbuffer queue that handles data to be saved to disk.
 */
static rqueue_t *outputDataQ;

const char poison = '!';
void thOutput(char *data) {
  fprintf(neuronFireFile, "%s\n", neuronFireBufferTXT[neuronFirePoolPos]);
}
char *dequeue(void *queue) { return (char *)rqueue_read((rqueue_t *)queue); }
int enqueue(void *queue, char *data) {
  return rqueue_write((rqueue_t *)queue, data);
}
void *outputWorker() {
  bool working = true;
  while (working) {
    char *data = rqueue_read(outputDataQ);
    if (data) {
      if (data[0] == poison) {
        working = false;
        free(data);
      } else {
        thOutput(data);
        free(data);
      }
    }
  }
  return NULL;
}
/**
 * initializes threads and sets up queue for file io
 */
void initThreading() {}

/** @} */

/** neuronFireStruct contains the in-memory representation of a neuron spike
 * event. Used for binary data saving of neuron events. This struct will be used
 * both in MPI and in POSIX file IO. Currently, MPI-IO is not implemented and
 * neruron spikes are saved as text, but in the future spikes will be saved as
 * binary.
 */
typedef struct NeuronFireStruct {
  tw_stime timestamp;
  id_type core;
  id_type local;
  tw_lpid dest;
  char ne;
} neuronFireStruct;

/** Binary buffer container for POSIX write */
neuronFireStruct *neuronFireBufferBIN;

/** @defgroup MPI_IO_WRITE MPI File IO output data.
 * Currently this is not used, but this is a skeleton for
 * collaborative IO if the POSIX multi process multi file technique kills
 * performance.
 * @{
 */

// Neuron Fire Record MPI pointers and datatypes

MPI_File *neuronFireFileMPI;

char *mpiFileName = "spike_events_mpi.dat";

// not sure if we need to buffer MPI writes, but if so this is the buffer for
// MPI.
neuronFireStruct *neuronFireBufferMPI;

/**@}*/

/** @defgroup neuronFireRecord Neuron Fire Record Functions
 * These functions manage the neuron fire event record. These functions
 * currently use POSIX file IO, and generate one file per rank during runs. The
 * buffers and files are initialized inside the initFiles() function, contained
 * in the more file output handler group.
 * @see{outputFileHandler}.
 * @{
 */

void flushNeuron() {
  if (BINARY_OUTPUT) {
    // handle binary output
    fwrite(neuronFireBufferBIN, neuronFirePoolPos, sizeof(neuronFireStruct),
           neuronFireFileBinary);
  } else {
    while (--neuronFirePoolPos > -1) {

      fprintf(neuronFireFile, "%s\n", neuronFireBufferTXT[neuronFirePoolPos]);
    }
  }
  // reset the memory pool counter.
  neuronFirePoolPos = 0;
}

void setNeuronNetFileName() {

  char *ext = BINARY_OUTPUT ? ".dat" : ".csv";
  neuronRankFN = (char *)calloc(128, sizeof(char));
  sprintf(neuronRankFN, "%s_rank_%li%s", NEURON_FIRE_R_FN, g_tw_mynode, ext);
  neuronFireFinalFN = (char *)calloc(128, sizeof(char));
  sprintf(neuronFireFinalFN, "%s_final.csv", NEURON_FIRE_R_FN);
}
//! @todo remove this once done debugging.
unsigned long num_save_ops = 0;
void saveNeuronFire(tw_stime timestamp, id_type core, id_type local,
                    tw_lpid destGID, long destCore, long destLocal,
                    unsigned int isOutput) {
  num_save_ops++;
  if (neuronFirePoolPos >= N_FIRE_BUFF_SIZE) {
    flushNeuron();
  }
  if (BINARY_OUTPUT) {

    neuronFireBufferBIN[neuronFirePoolPos].core = core;
    neuronFireBufferBIN[neuronFirePoolPos].dest = destGID;
    neuronFireBufferBIN[neuronFirePoolPos].timestamp = timestamp;
    neuronFireBufferBIN[neuronFirePoolPos].local = local;
    neuronFireBufferBIN[neuronFirePoolPos].ne = '|';

  } else {

    sprintf(neuronFireBufferTXT[neuronFirePoolPos],
            "%f,%lli,%lli,%llu,%li,%li,%u", timestamp, core, local, destGID,
            destCore, destLocal, isOutput);
  }

  ++neuronFirePoolPos;
}
FILE *neuron_output_dbg;
void saveNeuronFireDebug(tw_stime timestamp, id_type core, id_type local,
                         tw_lpid destGID, long destCore, long destLocal,
                         unsigned int isOutput) {
  static int file_open = 0;
  if (!file_open) {
    if (g_tw_mynode == 0) {
      tw_printf(TW_LOC, "init debug spike file\n");
    }
    char fn[1024] = {'\0'};
    sprintf(fn, "d_fire_record_rank_%li.csv", g_tw_mynode);
    neuron_output_dbg = fopen(fn, "w");
    file_open = 1;
    fprintf(neuron_output_dbg,
            "timestamp,srcCore,srcNeuron,destGID,destCore,destAxon,isOutput\n");
  }
  fprintf(neuron_output_dbg, "%f,%lli,%lli,%llu,%li,%li,%u\n", timestamp, core,
          local, destGID, destCore, destLocal, isOutput);
}
/** @} */

void setFileNames() {
  // if (SAVE_SPIKE_EVTS || SAVE_OUTPUT_NEURON_EVTS) {
  setNeuronNetFileName();
  //}
}

void initOutFiles() {
  setFileNames();
  int tv = N_FIRE_BUFF_SIZE;
  if (SAVE_SPIKE_EVTS || SAVE_OUTPUT_NEURON_EVTS) {
    if (BINARY_OUTPUT) {
      neuronFireBufferBIN = (neuronFireStruct *)tw_calloc(
          TW_LOC, "OUTPUT", tv, sizeof(neuronFireStruct));
      neuronFireFile = fopen(neuronRankFN, "wb");

    } else {
      neuronFireBufferTXT =
          (char **)tw_calloc(TW_LOC, "OUTPUT", tv, sizeof(char *));

      for (int i = 0; i < N_FIRE_BUFF_SIZE; i++) {
        neuronFireBufferTXT[i] =
            (char *)tw_calloc(TW_LOC, "OUTPUT", tv, sizeof(char *));
      }
      neuronFireFile = fopen(neuronRankFN, "w");
      if (g_tw_mynode == 0) {
        fprintf(neuronFireFile,
                "timestamp,core,local,destGID,destCore,destNeuron,isOutput?\n");
      }
    }
    //
    //        MPI_File_open(MPI_COMM_WORLD,mpiFileName,
    //                      MPI_MODE_CREATE|MPI_MODE_WRONLY,MPI_INFO_NULL,neuronFireFileMPI);
    //
    //        MPI_File_set_atomicity(neuronFireFileMPI,1);
  }
}
void closeFiles() {
  close_energy_stat_file();
  flushNeuron();
  // if (neuronFireFile != NULL) {
  //  fclose(neuronFireFile);
  //}
  MPI_Barrier(MPI_COMM_WORLD); // wait for everyone to catch up.
  if (g_tw_mynode == 0) {
    FILE *finalout = fopen("neuron_spike_evts.csv", "w");
    fprintf(finalout, "timestamp,neuron_core,neuron_local,destGID\n");
    assert(num_save_ops <= N_FIRE_BUFF_SIZE);
    for (size_t i = 0; i < num_save_ops && neuronFireBufferTXT[i] != NULL;
         i++) {
      fprintf(finalout, "%s\n", neuronFireBufferTXT[i]);
    }
    fclose(finalout);
    printf("Attempted to write %lu spikes to file \n", num_save_ops);
  }
}

/** @} */
