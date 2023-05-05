/* sequence.h
   ==========
   Author: E.G.Thomas
*/


#ifndef _SEQUENCE_H
#define _SEQUENCE_H

struct sequence {
  int mppul;
  int mplgs;
  int mpinc;
  int mplgexs;
  int *ptab;
  int (*lags)[2];
};

struct sequence *OpsSequenceMake();

int OpsBuild7pulse(struct sequence *ptr);
int OpsBuild8pulse(struct sequence *ptr);
int OpsBuild16pulse(struct sequence *ptr);
int OpsBuildTauscan(struct sequence *ptr);
int OpsBuildTauscan11(struct sequence *ptr);

void OpsBuildPcode(int nbaud, int mppul, int *pcode);

#endif

