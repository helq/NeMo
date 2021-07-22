#ifndef IO_ENERGY_STATS
#define IO_ENERGY_STATS

#include "../globals.h"

typedef struct {
  stat_type rng_count;
  stat_type sops_count;
  stat_type output_count;
  stat_type spike_count;
  int my_core;
  int my_neuron;
  int dest_core;
  int dest_neuron;
} tn_energy;

void close_energy_stat_file();
void save_energy_stats(tn_energy *energy_data, int source_rank);

#endif  // IO_ENERGY_STATS
