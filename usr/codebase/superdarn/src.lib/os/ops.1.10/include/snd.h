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
void OpsFindSndSkip(char *ststr,int *snd_bms,int snd_bms_tot,int *snd_bm_cnt,int *odd_beams);
void OpsBuildSnd(struct SndData *snd, struct RadarParm *prm, struct FitData *fit);
void OpsWriteSnd(int sock, char *name, struct SndData *snd, char *ststr);
void OpsWriteSndRaw(int sock, char *progname, struct RadarParm *prm,
                    struct RawData *raw, char *ststr);
void OpsWriteSndIQ(int sock, char *progname, struct RadarParm *prm,
                   struct IQ *iq, unsigned int *badtr, char *ststr);

#endif
