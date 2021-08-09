//
// Created by Mark Plagge on 8/18/17.
//

#ifndef SUPERNEMO_DUMPI_H
#define SUPERNEMO_DUMPI_H

#include "globals.h"
#include <stdlib.h>

//Linux / BSD issue
#ifndef arc4random
#define arc4random random
#endif

// Dumpi variables
extern long double COMPUTE_TIME;
extern long double SEND_TIME_MIN;
extern long double SEND_TIME_MAX;
extern unsigned int DO_DUMPI;

/** DUMPI TEXT FILE PERMS
 * <mpiType> <src> <dst> <wallStart> <wallStop> <cpuStart>
 * <cpuStop> <count> <dataType> <comm> <tag>
 */

/**
 * returns an Isend in dumpi format.
 * @param sourceChip The soruce chip - will be converted to an mpi rank
 * @param destChip The destination chip - will be converted to an mpi rank
 * @param twTimeSend -DUMPI start
 * @return A char (dynamic mem) containing the full dumpi line.
 */
char *generateIsend(uint64_t sourceChip, uint64_t destChip, double twTimeSend, unsigned long dumpID);

/**
 * returns an Irecv in dumpi format.
 * @param sourceChip The soruce chip - will be converted to an mpi rank
 * @param destChip The destination chip - will be converted to an mpi rank
 * @param start -DUMPI start
 * @param stop -DUMPI end
 * @return A char (dynamic mem) containing the full dumpi line.
 */
char *generateIrecv(uint64_t sourceChip, uint64_t destChip, double twTimeSbend, unsigned long dumpID);

void saveMPIMessage(id_type sourceCore, id_type destCore, double twTimeSend,
                    FILE *outputFile);

uint64_t coreToChip(uint64_t coreID);

uint64_t coreToRank(uint64_t coreID);

bool isDestInterchip(id_type core1, id_type core2);

void saveSendMessage(unsigned long long sourceCore, unsigned long long destCore, double twTime, unsigned long dumpID,
                     FILE *outputFile);
void saveRecvMessage(unsigned long long sourceCore, unsigned long long destCore, double twTime, unsigned long dumpID,
                     FILE *outputFile);

void setrnd(tw_lp *lp);

#endif //SUPERNEMO_DUMPI_H
