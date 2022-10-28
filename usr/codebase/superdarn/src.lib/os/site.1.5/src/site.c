/* site.c
   ====== 
   Author R.J.Barnes
*/

/*
 (c) 2010 JHU/APL & Others - Please Consult LICENSE.superdarn-rst.3.1-beta-18-gf704e97.txt for more information.
 
 
 
*/


#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>
#include "rtypes.h"
#include "limit.h"
#include "tsg.h"
#include "maketsg.h"
#include "acf.h"
#include "tcpipmsg.h"
#include "rosmsg.h"
#include "shmem.h"
#include "global.h"
#include "site.h"

int sock;
char server[256];
int port;
int num_transmitters;
/*struct timeval tock;*/
struct ControlPRM rprm;
struct RosData rdata;
struct DataPRM dprm;
struct TRTimes badtrdat;
struct TXStatus txstatus;
struct SiteLibrary sitelib;
int cancel_count=0;

struct TCPIPMsgHost errlog={"127.0.0.1",44100,-1};
struct TCPIPMsgHost shell={"127.0.0.1",44101,-1};
struct TCPIPMsgHost task[4]={
  {"127.0.0.1",1,-1}, /* iqwrite */
  {"127.0.0.1",2,-1}, /* rawacfwrite */
  {"127.0.0.1",3,-1}, /* fitacfwrite */
  {"127.0.0.1",4,-1}  /* rtserver */
};
int baseport=44100;


int SiteStart(char *host) {
  rdata.main=NULL;
  rdata.back=NULL;
  badtrdat.start_usec=NULL;
  badtrdat.duration_usec=NULL; 
  return (sitelib.start)(host);
}

int SiteSetupRadar() {
  return (sitelib.setupradar)();
}
 
int SiteStartScan() {
  return (sitelib.startscan)();
}

int SiteStartIntt(int sec,int usec) {
  return (sitelib.startintt)(sec,usec);
}

int SiteFCLR(int stfreq,int edfreq) {
  return (sitelib.fclr)(stfreq,edfreq);
}

int SiteTimeSeq(int *ptab) {
  return (sitelib.tmseq)(ptab);
}

int SiteIntegrate(int (*lags)[2]) {
  return (sitelib.integrate)(lags);
}

int SiteEndScan(int bsc,int bus, unsigned sleepus) {
  return (sitelib.endscan)(bsc,bus, sleepus);
}

void SiteExit(int signo) {
  (sitelib.exit)(signo);
}

