/* sync.c
   ====== 
   Author: R.J.Barnes
*/

/*
 (c) 2010 JHU/APL & Others - Please Consult LICENSE.superdarn-rst.3.1-beta-18-gf704e97.txt for more information.
*/

#include <math.h>
#include <time.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <zlib.h>
#include "rtime.h"
#include "rtypes.h"
#include "dmap.h"
#include "limit.h"
#include "rprm.h"
#include "rawdata.h"
#include "fitblk.h"
#include "fitdata.h"
#include "radar.h"
#include "global.h"

#define USEC 1000000.0

int OpsDayNight() {
  if (day < night) {
    if ((day <= hr) && (hr < night)) return 1;
    else return 0;
  } else {
    if ((night <= hr) && (hr < day)) return 0;
  }
  return 1;
}


/*int OpsFindSkip(int bsc,int bus)*/
int OpsFindSkip(int scnsc, int scnus, int synsc, int synus, int nbm)
{
  unsigned tv,bv,iv;
  int skip;
  /*int nbm;*/

  /* option to pass in number of beams for camping mode */
/*  if (nbm <= 0) nbm = fabs(ebm-sbm) + 1;*/  /* # of bms to get thru in scan */
  if (nbm <= 0) nbm = abs(ebm-sbm); /* Evan doesn't like the +1 ... */
  TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
  /* all times converted to microseconds */
  iv   = synsc*1000000 + synus;  /* sync time for one beam */
  bv   = scnsc*1000000 + scnus;  /* time for a full scan */
  tv   = (mt*60 + sc)*1000000 + us + iv/2 - 100000; /* time from start of hour
                                                       add half a beam & some
                                                       rando buffer for FCLR? */
  skip = (tv % bv)/iv;
  if (skip > nbm || skip < 0) skip = 0;

  return skip;
  /*
  nbm=fabs(ebm-sbm);
  TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
  iv=intsc*1000000+intus;
  bv=bsc*1000000+bus;
  tv=(mt*60+sc)*1000000+us+iv/2-100000;
  skip=(tv % bv)/iv;
  if (skip>nbm) skip=0;
  if (skip<0) skip=0;
  return skip;
  */
}

