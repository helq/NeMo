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
char *generateIsend(size_type sourceChip, size_type destChip, double twTimeSend, unsigned long dumpID);

/**
 * returns an Irecv in dumpi format.
 * @param sourceChip The soruce chip - will be converted to an mpi rank
 * @param destChip The destination chip - will be converted to an mpi rank
 * @param start -DUMPI start
 * @param stop -DUMPI end
 * @return A char (dynamic mem) containing the full dumpi line.
 */
char *generateIrecv(size_type sourceChip, size_type destChip, double twTimeSbend, unsigned long dumpID);

void saveMPIMessage(id_type sourceCore, id_type destCore, double twTimeSend,
                    FILE *outputFile);

size_type coreToChip(size_type coreID);

size_type coreToRank(size_type coreID);

bool isDestInterchip(id_type core1, id_type core2);

void saveSendMessage(unsigned long long sourceCore, unsigned long long destCore, double twTime, unsigned long dumpID,
                     FILE *outputFile);
void saveRecvMessage(unsigned long long sourceCore, unsigned long long destCore, double twTime, unsigned long dumpID,
                     FILE *outputFile);

void setrnd(tw_lp *lp);

#endif //SUPERNEMO_DUMPI_H
