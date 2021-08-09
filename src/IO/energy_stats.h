#ifndef IO_ENERGY_STATS
#define IO_ENERGY_STATS

#include "../globals.h"

typedef struct {
  uint64_t rng_count;
  uint64_t sops_count;
  uint64_t output_count;
  uint64_t spike_count;
  int my_core;
  int my_neuron;
  int dest_core;
  int dest_neuron;
} tn_energy;

void close_energy_stat_file();
void save_energy_stats(tn_energy *energy_data, int source_rank);

#endif  // IO_ENERGY_STATS
