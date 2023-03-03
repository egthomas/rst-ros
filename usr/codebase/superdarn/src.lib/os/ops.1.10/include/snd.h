/* snd.h
   =====
   Author: E.G.Thomas
*/


#ifndef _SND_H
#define _SND_H

extern struct SndData *snd;

int OpsSndStart();
void OpsBuildSnd(struct RadarParm *prm, struct FitData *fit);
void write_snd_record(int sock, char *name, struct SndData *snd, char *ststr);

#endif

