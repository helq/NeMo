//
// Created by Mark Plagge on 5/25/16.
//

#include "IO/model_reader.h"
#include "IO/neuron_output.h"
#include "IO/output.h"
#include "IO/spike_db_reader.h"
#include "dumpi.h"
#include "layer_map/layer_map_lib.h"
#include "mapping.h"
#include "message.h"
#include "ross.h"
#include <nemo_config.h>
#include <stdio.h>

uint64_t CORES_IN_SIM = 16;  // user defined option
uint64_t SIM_SIZE = 1025;  // user defined option
uint64_t SYNAPSES_IN_CORE = 0;
uint64_t CORE_SIZE = 0;

static bool IS_RAND_NETWORK = false;  // user defined option
bool SAVE_SPIKE_EVTS = false;  // user defined option
bool SAVE_NETWORK_STRUCTURE = false;  // user defined option
bool PHAS_VAL = false;  // user defined option
bool DEBUG_MODE = false;  // user defined option
unsigned int DO_DUMPI = false;  // user defined option

unsigned int CORES_IN_CHIP = 4096;  // user defined option

long double COMPUTE_TIME = 0.0000000002;  // user defined option
long double SEND_TIME_MIN = 0.0000000001;  // user defined option
long double SEND_TIME_MAX = 0.000000002;  // user defined option

unsigned int SAT_NET_PERCENT = 2; // user defined option
unsigned int SAT_NET_COREMODE = false; // user defined option
unsigned int SAT_NET_THRESH = 2; // user defined option
unsigned int SAT_NET_LEAK = 1; // user defined option
unsigned int SAT_NET_STOC = false; // user defined option
unsigned int IS_SAT_NET = false; // user defined option
unsigned int SAVE_OUTPUT_NEURON_EVTS = false; // user defined option

// Used solely by layer_map/layer_map_lib.c
enum LayerTypes LAYER_NET_MODE = NON_LAYER;
unsigned int NUM_LAYERS_IN_SIM;  // user defined option
unsigned int GRID_ENABLE = 0;  // user defined option
unsigned int GRID_MODE = 0;  // user defined option
unsigned int RND_GRID = 0;  // user defined option
unsigned int RND_UNIQ = 0;  // user defined option

static bool FILE_OUT;
bool FILE_IN = false; // user defined option

char NEMO_MODEL_FILE_PATH[512] = {'\0'}; // user defined option
char NEMO_SPIKE_FILE_PATH[512] = {'\0'}; // user defined option
bool NEMO_MODEL_IS_BINARY = false; // user defined option

char *luaConfigFile = NULL;
long isBin = 0;

/**
 * app_opt - Application Options. Manages the options for NeMo's run.
 */
static tw_optdef const app_opt[] = {
    TWOPT_GROUP("Input Parameters"),
    TWOPT_FLAG("netin", FILE_IN,
               "Load network information from a file. If set,"
               "a network file name must be specified.\n"
               "This will set random network, sat network, and layer network "
               "modes off. "),
    TWOPT_CHAR("model", NEMO_MODEL_FILE_PATH,
               "Path to NeMo model file, binary neuron file, or TN JSON File."),
    TWOPT_CHAR("spikes", NEMO_SPIKE_FILE_PATH, "Path to NeMo Spike File"),
    TWOPT_FLAG("tn_bin", NEMO_MODEL_IS_BINARY,
               "Is the input model file in processed TN binary format?"),

    TWOPT_GROUP("General Network Parameters"),
    TWOPT_FLAG("rand", IS_RAND_NETWORK,
               "Generate a random network? Alternatively, "
               "you need to specify config files."),
    TWOPT_FLAG("sat", IS_SAT_NET,
               "Generate a SAT network with n% core/neuron connectivity"),
    TWOPT_UINT("sp", SAT_NET_PERCENT,
               "SAT network connectivity percentage in core mode, "
               "or percentage chance of connected axon"),
    TWOPT_UINT(
        "sc", SAT_NET_COREMODE,
        "SAT network mode: 0: axon probability, 1: Core pool, 2: Neuron pool."),
    TWOPT_UINT("st", SAT_NET_THRESH, "Sat network neuron threshold"),
    TWOPT_UINT("sl", SAT_NET_LEAK, "Sat network per-neuron leak value"),
    TWOPT_FLAG("ss", SAT_NET_STOC, "Sat network stochastic weight mode "),

    TWOPT_GROUP("Randomized (ID Matrix) Network Parameters"),
    TWOPT_UINT("chip", CORES_IN_CHIP,
               "The number of neurosynaptic cores contained in one chip"),
    TWOPT_ULONGLONG("cores", CORES_IN_SIM, "number of cores in simulation"),

    TWOPT_GROUP("Data Gathering Settings"),
    TWOPT_FLAG("dbg", DEBUG_MODE, "Debug message printing"),
    TWOPT_FLAG("svnet", SAVE_NETWORK_STRUCTURE,
               "Save neuron output axon IDs on creation - Creates a map of the "
               "neural network."),
    TWOPT_FLAG("svs", SAVE_SPIKE_EVTS,
               "Save neuron spike event times and info (all spike events, not "
               "just output)"),
    TWOPT_FLAG("svouts", SAVE_OUTPUT_NEURON_EVTS, "Save output neuron spikes"),

    TWOPT_GROUP("Integrated Bio Model Testing"),
    TWOPT_FLAG("phval", PHAS_VAL, "Phasic Neuron Validation"),

    TWOPT_GROUP("DUMPI Timing Parameters - All Parameters are in the scale of "
                "seconds."),
    TWOPT_FLAG("dmp", DO_DUMPI,
               "Save simulated DUMPI files. Note: For consistent operation, "
               "run one rank per sim chip."),
    TWOPT_STIME("ct", COMPUTE_TIME,
                "The time between message sends."
                "\n Used if a collision in TW_NOW is detected when generating "
                "DUMPI cmds"),
    TWOPT_STIME("stmin", SEND_TIME_MIN, "The send time min value"),
    TWOPT_STIME("stmax", SEND_TIME_MAX, "The maximum send time value"),

    TWOPT_GROUP("Layer/Grid Network Benchmark Parameters"),
    TWOPT_FLAG("gd", GRID_ENABLE, "Enable Grid Mode"),
    TWOPT_UINT("gm", GRID_MODE, "Grid Mode: 0 is Linear, 1 is Convolutional"),
    TWOPT_FLAG("gr", RND_GRID, "Enable random grid mode"),
    TWOPT_FLAG("gru", RND_UNIQ,
               "For random grids, are connections unique (no neurons attached "
               "to the same axon)"),
    TWOPT_UINT("lnum", NUM_LAYERS_IN_SIM,
               "Number of layers in simulation. Ignored if grid mode."),
    TWOPT_UINT("gridsize", CHIPS_PER_LAYER,
               "Number of chips in a layer. Grid mode only. Must be evenly "
               "distributable"),
    TWOPT_END()
};

/**
 * @brief      Displays NeMo's initial run size configuration.
 */
void displayModelSettings() {
#define TXT_HEADER "****************************************************************************\n"

  printf(TXT_HEADER);
  double cores_per_node = CORES_IN_SIM / tw_nnodes();
  char *netMode = FILE_IN ? "file defined" : "random benchmark";
  printf("\n");
#ifdef DEBUG
  printf(TXT_HEADER);
  printf("* \tDebug Mode Enabled.\n");
  printf("* \t\t Save Network Structure? %i\n", SAVE_NETWORK_STRUCTURE);
#endif
  printf(TXT_HEADER);
  printf("* \t %i Neurons per core (cmake defined), %lu cores in sim.\n",
         NEURONS_IN_CORE, CORES_IN_SIM);
  printf("* \t %f cores per PE, giving %lu LPs per pe.\n", cores_per_node,
         g_tw_nlp);
  printf("* \t Neurons have %i axon types (cmake defined)\n",
         NUM_NEURON_WEIGHTS);
  printf("* \t Network is a %s network.\n", netMode);
  printf("* \t Neuron stats:\n");
  printf("* \tCalculated sim_size is %lu\n", SIM_SIZE);

  printf(TXT_HEADER);

  printf("* \tChip Sim Info:\n");
  printf("* \tCores per chip: %i\n", CORES_IN_CHIP);
  printf("* \tReported chips in sim: %ld\n", (long)coreToChip(CORES_IN_SIM));
  printf(TXT_HEADER);
  if (FILE_IN) {
    printf("* \tNeuron model input file: %s\n", NEMO_MODEL_FILE_PATH);
    printf("* \tNeuron spike input file: %s\n", NEMO_SPIKE_FILE_PATH);
    // char * inputMode = NEMO_MODEL_IS_TN_JSON ? "TN JSON format" : "NeMo NFG
    // Format";
    char *inputMode = "NeMo NFG Format";
    printf("* \tModel input mode is %s\n", inputMode);
    printf(TXT_HEADER);
  } else {
    printf("* \tSAT NET ENABLED: %i\n", IS_SAT_NET);
    printf("* \tSAT net stoc. mode: %i\n", SAT_NET_STOC);
    printf("* \tSAT NET Weight: %u %%\n", SAT_NET_PERCENT);
    printf("* \tSAT mode set to %u\n", SAT_NET_COREMODE);
    printf("* \tModes: (0) - Neuron %%, (1) - Core Pool, (2) Neuron Pool \n");
    printf(TXT_HEADER);
    printf("* \tLayer Network Parameters \n");
    displayConfig();
    printf("\n");
  }
}

char *luaMemLoader(char *luaConfigFilePath) {
  char *binVersion = alloca(sizeof(char) * 5192);
  strcpy(binVersion, luaConfigFilePath);
  char *ext = strstr(binVersion, ".nfg1");
  FILE *luaFile;

  if (ext != NULL) {
    strncpy(ext, ".nbin", 6);
    printf("\n----\n%s\n", binVersion);
  }

  if (0 && access(binVersion, F_OK) != -1) {
    luaFile = fopen(binVersion, "rb");
    isBin = 1;
    tw_printf(TW_LOC, "Loaded binary file at %s\n", binVersion);
  } else {
    luaFile = fopen(luaConfigFilePath, "r");
    tw_printf(TW_LOC, "Loaded text file at %s\n", luaConfigFilePath);
  }
  if (luaFile != NULL) {
    fseek(luaFile, 0, SEEK_END);
    long numbytes = ftell(luaFile);
    fseek(luaFile, 0L, SEEK_SET);
    char *luaData = calloc(numbytes, sizeof(char));
    if (luaData == NULL) {
      tw_error(TW_LOC, "Could not allocate %li bytes for config file.",
               numbytes);
    }
    if (isBin) {
      fread(luaData, numbytes, 1, luaFile);
      isBin = numbytes;
    } else {
      fread(luaData, sizeof(char), numbytes, luaFile);
    }

    fclose(luaFile);
    return luaData;
  } else {
    tw_error(TW_LOC, "Could not read config file %s", luaConfigFilePath);
  }
}

/**
 * @brief New method for loading LUA config file.
 *
 * Function runs
 * @param luaConfigFilePath
 */
void luaLoader(char *luaConfigFilePath) {
  // Loads LUA file into memory. Will operate either 1 to 1 with number of ranks
  // if the number of ranks are below a threshold, otherwise it will load the
  // file from rank 0 and MPI_ISEND the rank data.

  if (1 <= MAX_RANKS_FILES) {
    char *luaData = luaMemLoader(luaConfigFilePath);
    luaConfigFile = luaData;
    if (strlen(luaData) < 30 && isBin == 0) {
      tw_error(TW_LOC, "Error loading config file into memory. ");
    } else if (isBin == 1) {
      if (g_tw_mynode == 0) {
        tw_printf(TW_LOC, "Loaded binary file with %li elements.",
                  strlen(luaData));
      }
    } else {

      if (g_tw_mynode == 0) {
        tw_printf(TW_LOC, "Loaded config file - total size %li",
                  strlen(luaData));
      }
    }
  }
}

void luaLoaderClean() {
  if (1 <= MAX_RANKS_FILES) {
    free(luaConfigFile);
  }
}
/**
 * @brief      Initializes NeMo
 *
 * First, this function checks for potential file IO, and creates file handles
 * for use.
 *
 * Based on the file_in option, the function then sets the neuron, axon, and
 * synapse
 * function pointers to the proper values. Default is the IBM TrueNorth neuron
 * model.
 * If NeMo is reading a file, then we set the size of the sim based on the model
 * config file.
 *
 * The rest of this function manages ROSS initialization and setup. When done,
 *
 */
void init_nemo() {
  /// SANTIY CHECKS:
  if (FILE_IN && (IS_SAT_NET || IS_RAND_NETWORK || GRID_ENABLE)) {
    printf("Found file in option: %i and sat mode option %i, rand network %i, "
           "grid mode %i\n",
           FILE_IN, IS_SAT_NET, IS_RAND_NETWORK, GRID_ENABLE);
    tw_error(TW_LOC, "Multiple modes implemented - Can not load file and have "
                     "random network.");
  }

  if (GRID_ENABLE && IS_RAND_NETWORK)
    tw_error(TW_LOC, "Can not have random network and grid/layer network ");
  if (SAT_NET_COREMODE > 2) {
    tw_error(TW_LOC, "Please choose a valid SAT mode if using SAT. \n"
                     "Can be 0,1,2");
  }
#ifdef DEBUG
  if (SAVE_NETWORK_STRUCTURE)
    tw_printf(TW_LOC, "Saving network structure\n");
#endif
  FILE_OUT = SAVE_SPIKE_EVTS || SAVE_NETWORK_STRUCTURE ||
             PHAS_VAL;
  // FILE_IN = !IS_RAND_NETWORK;
  if (FILE_OUT) {
    // Init file output handles
    initOutFiles();
    if (SAVE_NETWORK_STRUCTURE)
#ifdef DEBUG
      tw_printf(TW_LOC, "Saving network structure for sure.\n");
#endif
    openOutputFiles("network_def.csv");
    initDataStructures(g_tw_nlp);
    if (g_tw_mynode == 0) {

      printf("Output Files Init.\n");
    }
  }

  if (FILE_IN) {
    tw_printf(TW_LOC, "File input starting.");
    // Init File Input Handles
    if (g_tw_mynode == 0) {
      printf("Network Input Active - ");
      printf("Config Filename specified: %s\n", NEMO_MODEL_FILE_PATH);
      printf("Spike file: %s\n", NEMO_SPIKE_FILE_PATH);
    }
    // SPIKE_IN_FN = SPIKE_FILE;
    // INPUT Model file init here:
    ///////////////////////////////////////////////
    // Call the new JSON input function
    // if(NEMO_MODEL_IS_TN_JSON) {
    //  initJSON(NEMO_MODEL_FILE_PATH);
    //}else
    if (NEMO_MODEL_IS_BINARY) {
#ifdef DEBUG
      if (g_tw_mynode == 0) {
        tw_printf(TW_LOC, "Loading pre-processesd binary file from %s \n",
                  NEMO_MODEL_FILE_PATH);
      }
#endif
      openBinaryModelFile(NEMO_MODEL_FILE_PATH);
      setupBinaryNeurons();
    } else {
      luaLoader(NEMO_MODEL_FILE_PATH);
      initModelInput(CORES_IN_SIM);
    }

    // INPUT SPIKE FILE init HERE:
    ////////////////////////
    int num_spikes = openSpikeFile();
    tw_printf(TW_LOC, "SQLITE Spike DB loaded, found %i spikes.\n", num_spikes);
    // connectToDB(SPIKE_FILE);
    // int spkCT = loadSpikesFromFile(SPIKE_FILE);
    // printf("Read %i spikes\n", spkCT);
    GRID_ENABLE = false;
    IS_RAND_NETWORK = false;
    IS_SAT_NET = false;
  }

  SYNAPSES_IN_CORE = 1;

  CORE_SIZE = SYNAPSES_IN_CORE + NEURONS_IN_CORE + AXONS_IN_CORE;
  SIM_SIZE = CORE_SIZE * CORES_IN_SIM;

  g_tw_nlp = SIM_SIZE / tw_nnodes();
  g_tw_lookahead = 0.001;
  g_tw_lp_types = model_lps;
  g_tw_lp_typemap = lpTypeMapper;

  /// EVENTS PER PE SETTING
  g_tw_events_per_pe = NEURONS_IN_CORE * AXONS_IN_CORE * 128; // magic number

  // Layer / Grid Mode setup:
  if (GRID_ENABLE)
    setupGrid(0);
}

/**
 * @brief  NeMo Main entry point
 */
int main(int argc, char *argv[]) {
  tw_opt_add(app_opt);
  tw_init(&argc, &argv);

  // call nemo init
  init_nemo();
  printf("\n Completed initial setup and model loading.\n");
  if (nonC11 == 1)
    printf("Non C11 compliant compiler detected.\n");

  // Define LPs:
  tw_define_lps(g_tw_nlp, sizeof(struct messageData));
  tw_lp_setup_types();

  if (g_tw_mynode == 0) {
    displayModelSettings();
  }
  tw_run();
  if (SAVE_NETWORK_STRUCTURE) {
    closeOutputFiles();
  }
  if (FILE_OUT) {
    closeFiles();
  }
  tw_end();
}
