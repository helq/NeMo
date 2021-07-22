//
// Created by Mark Plagge on 8/18/17.
//

#include <printf.h>
#include "dumpi.h"
#include "mapping.h"
/*
 * Set up datatypes for the MPI send/rcv cdoe
 */
int COUNT = 2; // One count for the COREid, one for the neuron local id.
int DTYPE = 11; // dtype  for the DUMPI file.
int COMM = 4;
int TAG = 0;
//unsigned int NUM_CHIPS_IN_SIM = CORES_IN_SIM / CORES_IN_CHIP;
//unsigned int CHIPS_PER_RANK; = g_tw_npe; //! Sets the number of chips per MPI rank for sim




const long double WALL_OFFSET = 0.0000002; //! Some sort of offset for the wall clock time - what is a good value?
const long double CPU_OFFSET = 0.0000002; //! The CPU time for recv. messages.
const long double NEURO_CORE_CLOCK = 1000; //! Neuromorphic core speed (cycles / second).
//new constansts for wall clock time
const long double JITTER_MAX = 0.0000005;
const long double JITTER_MIN = 0.0000001;

// extern long double COMPUTE_TIME;  //= 0.000005;
// extern long double SEND_TIME_MIN; //= 0.00000005;
// extern long double SEND_TIME_MAX;// = 0.00000010;



long double RAND_COMP_TIME = 0.0;
long double RAND_JITTER = 0.0;
long double RAND_SEND_TIME = 0.0;

long double LAST_END_TIME_WC = 0;

long CURRENT_TICK = 0;
/**
 * Converts the chip ID to an MPI rank.
 * @param chipID
 * @return
 */
size_type chipToRank(size_type chipID) {
  return chipID/CHIPS_PER_RANK;
}

size_type coreToChip(size_type coreID) {
  return coreID/CORES_IN_CHIP;

}

size_type coreToRank(size_type coreID) {
  return chipToRank(coreToChip(coreID));
}

long double ld_rand(long double min, long double max, tw_rng_stream *g) {

  long double scale = tw_rand_unif(g);
  return min + scale*(max - min);

}

long double jitter() {
  return RAND_JITTER;
  //long double jit = ld_rand(JITTER_MIN, JITTER_MAX);
  //return jit;
}
long double compTime() {
  return RAND_COMP_TIME;
  //not truly random due to maths - but fast
  return COMPUTE_TIME + jitter();
}
int getCurrentTick(double now) {
  return (int) now;
}
long double sendTime() {
//return ld_rand(SEND_TIME_MIN, SEND_TIME_MAX);
  return RAND_SEND_TIME;
}

/**@{ */
/**
 * getWallStart returns the wallStart param for dumpi
 * @param twSendTime
 * @return
 */

long double getWallStart(long double twSendTime) {

//	struct timespec start;
//	clock_gettime(CLOCK_REALTIME, &start);
//
//	double ctime = start.tv_sec / 100.0;
//	ctime = ctime + twSendTime;
//    return ctime;// + (arc4random() * WALL_OFFSET);

  int ct = getCurrentTick(twSendTime);

  long double wctime = twSendTime/100;

  if (ct!=CURRENT_TICK) {

    CURRENT_TICK = ct;
    //LAST_END_TIME_WC = twSendTime;
    twSendTime = ((long double) ct)/(long double) 1000.0;
    LAST_END_TIME_WC = twSendTime;
    wctime = twSendTime;
  }
//	}else if (twSendTime <= LAST_END_TIME_WC ||
//			(twSendTime - LAST_END_TIME_WC > COMPUTE_TIME)){
//        wctime = LAST_END_TIME_WC + compTime();
//    }
  wctime = LAST_END_TIME_WC + COMPUTE_TIME;

  LAST_END_TIME_WC = wctime;
  return wctime;
}

/**
 * getWallEnd returns the  wallEnd param for dumpi.
 * @param startTime
 * @return
 */
long double getWallEnd(long double startTime) {
  LAST_END_TIME_WC = startTime + sendTime();
  return LAST_END_TIME_WC;// + jitter();
  long double endTime = LAST_END_TIME_WC + sendTime() + jitter();
  LAST_END_TIME_WC = endTime;
  return endTime;
}

/**
 * getCPUStart returns the cpuStart param for dumpi.
 * @param twSendTime
 * @return
 */
double getCPUStart(double twSendTime) {
  return twSendTime;
  struct timespec start;
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
  double ctime = start.tv_sec;
  ctime = ctime*CPU_OFFSET;
  return ((arc4random()%1000)*CPU_OFFSET) + twSendTime;
  return ctime;
  //return (twSendTime * CPU_OFFSET ) + (arc4random() * CPU_OFFSET);
}

/**
 * getCPUEnd returns the cpuEnd param for dumpi.
 * @param twSendTime
 * @return
 */
double getCPUEnd(double cpuStart) {
  //return cpuStart + CPU_OFFSET + (( arc4random() % 1000) * CPU_OFFSET);
  return cpuStart;
}

char *generateMsg(size_type sourceChip, size_type destChip, double twTimeSend, char *type, unsigned long dumpID) {

  char *outStr = calloc(sizeof(char), 1024); // alloc new string - using this instead of buffer for the time being.
  long double wallStart = getWallStart(twTimeSend);
  long double wallEnd = getWallEnd(wallStart);
  long double cpuStart = getCPUStart(twTimeSend);
  long double cpuEnd = getCPUEnd(cpuStart);
  long t = sourceChip;
  if (type[5]=='r') {
    sourceChip = destChip;
    destChip = t;
  }
  sprintf(outStr, "%li,%s %llu %llu %.21Lf %.21Lf %.17Lf %.17Lf %i %i %i %i\n",
          t, type, sourceChip, destChip, wallStart,
          wallEnd, cpuStart, cpuEnd, COUNT, DTYPE, COMM, TAG);

  return outStr;
}
char *generateIsend(size_type sourceChip, size_type destChip, double twTimeSend, unsigned long dumpID) {

  return generateMsg(sourceChip, destChip, twTimeSend, "MPI_Isend", dumpID);
}
char *generateIrecv(size_type sourceChip, size_type destChip, double twTimeSend, unsigned long dumpID) {
  return generateMsg(sourceChip, destChip, twTimeSend + (3*CPU_OFFSET), "MPI_Irecv", dumpID);
}

bool isDestInterchip(id_type core1, id_type core2) {

  return coreToRank(core1)!=coreToRank(core2);
}

void saveMPIMessage(id_type sourceCore, id_type destCore, double twTimeSend,
                    FILE *outputFile) {
  long sourceChip = coreToRank(sourceCore);
  long destChip = coreToRank(destCore);
  char *isend = generateIsend(sourceChip, destChip, twTimeSend, 0);
  char *ircv = generateIrecv(destChip, sourceChip, twTimeSend, 0);

  fprintf(outputFile, "%s", isend);
  fprintf(outputFile, "%s", ircv);

  free(isend);
  free(ircv);

}
void saveSendMessage(unsigned long long sourceCore, unsigned long long destCore, double twTime, unsigned long dumpID,
                     FILE *outputFile) {
  size_type sourceChip = coreToRank(sourceCore);
  size_type destChip = coreToRank(destCore);
  char *isend = generateIsend(sourceChip, destChip, twTime, dumpID);
  fprintf(outputFile, "%s", isend);
  free(isend);
}
void saveRecvMessage(unsigned long long sourceCore, unsigned long long destCore, double twTime, unsigned long dumpID,
                     FILE *outputFile) {
  long sourceChip = coreToRank(sourceCore);
  long destChip = coreToRank(destCore);
  char *ircv = generateIrecv(destChip, sourceChip, twTime, dumpID);
  fprintf(outputFile, "%s", ircv);
  free(ircv);
}

void setrnd(tw_lp *lp) {
  RAND_JITTER = ld_rand(JITTER_MIN, JITTER_MAX, lp->rng);
  RAND_COMP_TIME = COMPUTE_TIME + RAND_JITTER;
  RAND_SEND_TIME = ld_rand(SEND_TIME_MIN, SEND_TIME_MAX, lp->rng);
}

/**@}*/
