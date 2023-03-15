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

int OpsBuild7pulse(struct sequence *seq);
int OpsBuild8pulse(struct sequence *seq);
int OpsBuild16pulse(struct sequence *seq);
int OpsBuildTauscan(struct sequence *seq);
int OpsBuildTauscan11(struct sequence *seq);

#endif

