/* snd.h
   =====
   Author: E.G.Thomas
*/


#ifndef _SND_H
#define _SND_H

#define MAX_SND_FREQS 12

extern int snd_freqs_tot;
extern int snd_freqs[MAX_SND_FREQS];

extern struct SndData *snd;

void OpsLoadSndFreqs(char *ststr);
int OpsSndStart();
void OpsBuildSnd(struct RadarParm *prm, struct FitData *fit);
void OpsWriteSnd(int sock, char *name, struct SndData *snd, char *ststr);

#endif

