/* snd.c
   =====
   Author: E.G.Thomas
*/


#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <string.h>
#include <time.h>
#include <zlib.h>
#include "rtypes.h"
#include "limit.h"
#include "dmap.h"
#include "rprm.h"
#include "fitdata.h"
#include "snddata.h"
#include "fitsnd.h"
#include "sndwrite.h"
#include "errlog.h"
#include "global.h"


struct SndData *snd;


int OpsSndStart() {
  snd=SndMake();
  fprintf(stderr,"Leaving OpsSndStart\n");
  fflush(stderr);
  return 0;
}


void OpsBuildSnd(struct RadarParm *prm, struct FitData *fit) {
  SndSetOriginTime(snd,prm->origin.time);
  SndSetOriginCommand(snd,(char *)command);
  SndSetCombf(snd,combf);
  FitToSnd(snd,prm,fit,prm->scan);
}


void write_snd_record(int sock, char *progname, struct SndData *snd, char *ststr) {

  char data_path[100], data_filename[50], filename[80];

  char *snd_dir;
  FILE *out;

  char logtxt[1024]="";
  int status;

  /* set up the data directory */
  /* get the snd data dir */
  snd_dir = getenv("SD_SND_PATH");
  if (snd_dir == NULL)
    sprintf(data_path,"/data/ros/snd/");
  else {
    memcpy(data_path,snd_dir,strlen(snd_dir));
    data_path[strlen(snd_dir)] = '/';
    data_path[strlen(snd_dir)+1] = 0;
  }

  /* make up the filename */
  /* YYYYMMDD.HH.rad.snd */
  sprintf(data_filename, "%04d%02d%02d.%02d.%s", prm->time.yr, prm->time.mo, prm->time.dy,
                                                 (prm->time.hr/2)*2, ststr);

  /* finally make the filename */
  sprintf(filename, "%s%s.snd", data_path, data_filename);

  /* open the output file */
  fprintf(stderr,"Sounding Data File: %s\n",filename);
  out = fopen(filename,"a");
  if (out == NULL) {
    /* crap. might as well go home */
    sprintf(logtxt,"Unable to open sounding file:%s",filename);
    ErrLog(sock,progname,logtxt);
    return;
  }

  /* write the sounding record */
  status = SndFwrite(out, snd);
  if (status == -1) {
    ErrLog(sock,progname,"Error writing sounding record.");
  } else {
    ErrLog(sock,progname,"Sounding record successfully written.");
  }

  fclose(out);
}

