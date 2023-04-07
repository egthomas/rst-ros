/* oldsndtosnd.c
   =============
   Author: E.G.Thomas
*/

/*
 Copyright (C) <year>  <name of author>
 
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
#include <sys/types.h>
#include <ctype.h>
#include <zlib.h>
#include "rtypes.h"
#include "dmap.h"
#include "rtime.h"
#include "option.h"
#include "dmap.h"
#include "snddata.h"

#include "sndread.h"
#include "sndwrite.h"
#include "sndseek.h"

#include "errstr.h"
#include "hlpstr.h"


#define SND_NRANG 75

struct SndData *snd;
struct OptionData opt;

struct header_struct {
  int stime;
  short site_id;
  short beam_no;
  short freq;
  short noise;
  short frange;
  short rsep;
  short gsct[SND_NRANG];
  short qflg[SND_NRANG];
  char program_name[40];
} header;

struct data_struct {
  short pwr;
  short vel;
  short width;
  short AOA;
} data;

int rst_opterr (char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: oldsndtosnd --help\n");
  return(-1);
}

int main (int argc,char *argv[]) {

  int arg;

  int i,status=0;
  FILE *fp=NULL;

  int yr,mo,dy,hr,mt;
  double sc;
 
  unsigned char vb=0;
  unsigned char help=0;
  unsigned char option=0;
  unsigned char version=0;

  time_t ctime;
  int c,n;
  char command[128];
  char tmstr[40];

  OptionAdd(&opt,"-help",'x',&help);
  OptionAdd(&opt,"-option",'x',&option);
  OptionAdd(&opt,"-version",'x',&version);

  OptionAdd(&opt,"vb",'x',&vb);

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


  command[0]=0;
  n=0;
  for (c=0;c<argc;c++) {
    n+=strlen(argv[c])+1;
    if (n>127) break;
    if (c !=0) strcat(command," ");
    strcat(command,argv[c]);
  }

  snd=SndMake();

  if (arg==argc) fp=stdin;
  else fp=fopen(argv[arg],"r");

  if (fp==NULL) {
    fprintf(stderr,"File not found.\n");
    exit(-1);
  }

  while(fread(&header,sizeof(header),1,fp) == 1) {

    snd->origin.code=1;
    ctime = time((time_t) 0);
    SndSetOriginCommand(snd,command);
    strcpy(tmstr,asctime(gmtime(&ctime)));
    tmstr[24]=0;
    SndSetOriginTime(snd,tmstr);

    TimeEpochToYMDHMS(header.stime,&yr,&mo,&dy,&hr,&mt,&sc);

    snd->stid = header.site_id;
    snd->time.yr = yr;
    snd->time.mo = mo;
    snd->time.dy = dy;
    snd->time.hr = hr;
    snd->time.mt = mt;
    snd->time.sc = (int) sc;
    snd->noise.mean = header.noise;
    snd->bmnum = header.beam_no;
    snd->nrang = SND_NRANG;
    snd->frang = header.frange;
    snd->rsep = header.rsep;
    snd->tfreq = header.freq;
    snd->combf = header.program_name;
    snd->snd_revision.major = SND_MAJOR_REVISION;
    snd->snd_revision.minor = SND_MINOR_REVISION;

    SndSetRng(snd,SND_NRANG);

    for (i=0;i<SND_NRANG;i++) {
      snd->rng[i].qflg = header.qflg[i];
      snd->rng[i].gsct = header.gsct[i];
      if (header.qflg[i] == 1) {
        status=fread(&data,sizeof(data),1,fp);
        if (status != 1) break;
        snd->rng[i].v = data.vel;
        snd->rng[i].p_l = data.pwr;
        snd->rng[i].w_l = data.width;
      } else {
        snd->rng[i].v = 0;
        snd->rng[i].p_l = 0;
        snd->rng[i].w_l = 0;
      }
    }

    status=SndFwrite(stdout,snd);

    if (status==-1) break;

    if (vb) fprintf(stderr,"%.4d-%.2d-%.2d %.2d:%.2d:%.2d\n",snd->time.yr,
                    snd->time.mo,snd->time.dy,snd->time.hr,snd->time.mt,
                    snd->time.sc);
  }

  if (fp !=stdin) fclose(fp);

  return 0;
}

