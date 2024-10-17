/* iq_fix.c
   ========
   Author: E.G.Thomas
*/

/*
  Copyright (c) 2012 The Johns Hopkins University/Applied Physics Laboratory
 
This file is part of the Radar Software Toolkit (RST).

RST is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

Modifications:
*/ 

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <zlib.h>
#include "rtypes.h"
#include "dmap.h"
#include "rtime.h"
#include "option.h"
#include "rprm.h"
#include "iq.h"
#include "version.h"

#include "iqread.h"
#include "iqwrite.h"
#include "iqindex.h"
#include "iqseek.h"

#include "errstr.h"
#include "hlpstr.h"

struct RadarParm *prm=NULL;
struct IQ *iq=NULL;
unsigned int *badtr=NULL;
int16 *samples=NULL;

struct IQ *oiq=NULL;
struct timespec *otime=NULL;
int *oatten=NULL;
float *onoise=NULL;
int *ooffset=NULL;
int *osize=NULL;
int *otbadtr=NULL;
unsigned int *obadtr=NULL;
int16 *osamples=NULL;

struct OptionData opt;

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: iq_fix --help\n");
  return(-1);
}
 
int main (int argc,char *argv[]) {

  int arg=0;

  unsigned char vb=0;
  unsigned char help=0;
  unsigned char option=0;
  unsigned char version=0;

  int chnnum=0;

  FILE *fp=NULL;

  time_t ctime;
  int n=0;
  char command[128];
  char tmstr[40];

  int offset1=0;
  int offset2=0;
  int badtrnum=0;
  int size=0;

  prm=RadarParmMake();
  iq=IQMake();

  oiq=IQMake();

  OptionAdd(&opt,"-help",'x',&help);
  OptionAdd(&opt,"-option",'x',&option);
  OptionAdd(&opt,"-version",'x',&version);
  OptionAdd(&opt,"vb",'x',&vb);
  OptionAdd(&opt,"chnnum",'i',&chnnum);

  arg=OptionProcess(1,argc,argv,&opt,rst_opterr);

  if (arg==-1) {
    exit(-1);
  }

  if (help==1) {
    OptionPrintInfo(stdout,hlpstr);
    exit(0);
  }

  if (option==1) {
    OptionDump(stdout,&opt);
    exit(0);
  }

  if (version==1) {
    OptionVersion(stdout);
    exit(0);
  }

  if (arg==argc) fp=stdin;
  else fp=fopen(argv[arg],"r");

  if (fp==NULL) {
    fprintf(stderr,"File not found.\n");
    exit(-1);
  }


  command[0]=0;
  for (int c=0; c<argc; c++) {
    n+=strlen(argv[c])+1;
    if (n>127) break;
    if (c !=0) strcat(command," ");
    strcat(command, argv[c]);
  }

  while (IQFread(fp,prm,iq,&badtr,&samples) !=-1) {

    if (vb==1) {
      fprintf(stderr,"%d-%d-%d %d:%d:%d beam=%d nave=%d\n",prm->time.yr,prm->time.mo,
              prm->time.dy,prm->time.hr,prm->time.mt,prm->time.sc,prm->bmnum,prm->nave);
    }

    prm->origin.code=1;
    ctime= time((time_t) 0);
    RadarParmSetOriginCommand(prm,command);
    strcpy(tmstr,asctime(gmtime(&ctime)));
    tmstr[24]=0;
    RadarParmSetOriginTime(prm,tmstr);

    if ((prm->scan == 1) && (prm->nave > 1)) {
      size = 0;
      badtrnum = 0;
      for (n=1; n<iq->seqnum; n++) {
        size += iq->size[n];
        badtrnum += iq->badtr[n];
      }
      offset2 = iq->offset[1];

      prm->nave -= 1;
      oiq->revision.major = iq->revision.major;
      oiq->revision.minor = iq->revision.minor;
      if (chnnum > 0) oiq->chnnum = chnnum;
      else            oiq->chnnum = iq->chnnum;
      oiq->smpnum = iq->smpnum;
      oiq->skpnum = iq->skpnum;
      oiq->seqnum = iq->seqnum - 1;
      oiq->tbadtr = badtrnum;

      if (oiq->tval == NULL) otime=malloc(oiq->seqnum*sizeof(struct timespec));
      else otime=realloc(oiq->tval,oiq->seqnum*sizeof(struct timespec));
      memcpy(otime,&iq->tval[1],oiq->seqnum*sizeof(struct timespec));
      oiq->tval = otime;

      if (oiq->atten == NULL) oatten=malloc(oiq->seqnum*sizeof(int));
      else oatten=realloc(oiq->atten,oiq->seqnum*sizeof(int));
      memcpy(oatten,&iq->atten[1],oiq->seqnum*sizeof(int));
      oiq->atten = oatten;

      if (oiq->noise == NULL) onoise=malloc(oiq->seqnum*sizeof(float));
      else onoise=realloc(oiq->noise,oiq->seqnum*sizeof(float));
      memcpy(onoise,&iq->noise[1],oiq->seqnum*sizeof(float));
      oiq->noise = onoise;

      if (oiq->offset == NULL) ooffset=malloc(oiq->seqnum*sizeof(int));
      else ooffset=realloc(oiq->offset,oiq->seqnum*sizeof(int));
      memcpy(ooffset,&iq->offset[1],oiq->seqnum*sizeof(int));
      oiq->offset = ooffset;
      for (n=0; n<oiq->seqnum; n++) oiq->offset[n] -= offset2;

      if (oiq->size == NULL) osize=malloc(oiq->seqnum*sizeof(int));
      else osize=realloc(oiq->size,oiq->seqnum*sizeof(int));
      memcpy(osize,&iq->size[1],oiq->seqnum*sizeof(int));
      oiq->size = osize;

      if (oiq->badtr == NULL) otbadtr=malloc(oiq->seqnum*sizeof(int));
      else otbadtr=realloc(oiq->badtr,oiq->seqnum*sizeof(int));
      memcpy(otbadtr,&iq->badtr[1],oiq->seqnum*sizeof(int));
      oiq->badtr = otbadtr;
      offset1 = 2*iq->badtr[0];

      if (obadtr == NULL) obadtr=malloc(badtrnum*2*sizeof(unsigned int));
      else obadtr=realloc(obadtr,badtrnum*2*sizeof(unsigned int));
      memcpy(obadtr,&badtr[offset1],badtrnum*2*sizeof(unsigned int));

      if (osamples == NULL) osamples=malloc(size*sizeof(int16));
      else osamples=realloc(osamples,size*sizeof(int16));
      memcpy(osamples,samples+offset2,size*sizeof(int16));

      IQFwrite(stdout,prm,oiq,obadtr,osamples);
    } else {
      if (chnnum > 0) iq->chnnum=chnnum;
      IQFwrite(stdout,prm,iq,badtr,samples);
    }

  }

  if (fp !=stdin) fclose(fp);

  return 0;

}
