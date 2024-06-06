/* pcppcodescan.c
   ============
   Author: J.Spaleta & R.J.Barnes
*/

/*
 ${license}
*/

/*

Modified how PCPBEAM is defined to make it a schedule line argument. Under
old method of using define, it would not be possible to run two different
beams on a dual radar site. The default of beam 9 is kept in the variable
declaration.

2012/01/05 09:46:00 EST  - KTS

*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <zlib.h>
#include "rtypes.h"
#include "option.h"
#include "rtime.h"
#include "dmap.h"
#include "limit.h"
#include "radar.h"
#include "rprm.h"
#include "iq.h"
#include "rawdata.h"
#include "fitblk.h"
#include "fitdata.h"
#include "fitacf.h"

#include "errlog.h"
#include "freq.h"
#include "tcpipmsg.h"

#include "rmsg.h"
#include "rmsgsnd.h"

#include "radarshell.h"

#include "build.h"
#include "global.h"
#include "reopen.h"
#include "sequence.h"
#include "setup.h"
#include "sync.h"

#include "site.h"
#include "sitebuild.h"
#include "siteglobal.h"

/* #define PCPBEAM 9 
Not possible to set two different beams for dual sites.
*/

#define PCPFNUM 8
#define PCPCPID 9213    /* never used. SGS */
int pcpfreqs[PCPFNUM]={10200, 10800, 11800, 12500, 13500, 14500, 15500, 16500};
int pcpcnt;

char *ststr=NULL;
char *dfststr="tst";
char *libstr="ros";

void *tmpbuf;
size_t tmpsze;

char progid[80]={"pcppcodescan"};
char progname[256];

int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: pcppcodescan --help\n");
  return(-1);
}

int main(int argc,char *argv[]) {

  char logtxt[1024];

  int scnsc=60;
  int scnus=0;
  int skip;
/*  int cnt=0; */

  unsigned char fast=0;
  unsigned char discretion=0;

  unsigned char option=0;
  unsigned char version=0;

  int status=0,n;

  int PCPBEAM=9;    /* Set PCPBEAM to default to beam 9 -KTS */

  struct sequence *seq;

  seq=OpsSequenceMake();
  OpsBuild8pulse(seq);

  cp=PCPCPID;
  intsc=2;          /* Why is this set to 2 seconds and changed later? SGS */
  intus=0;
  mppul=seq->mppul;
  mplgs=seq->mplgs;
  mpinc=1560;
  nbaud=13;

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */

  OptionAdd(&opt,"di",'x',&discretion);

  OptionAdd(&opt,"frang",'i',&frang);
  OptionAdd(&opt,"rsep",'i',&rsep);

  OptionAdd( &opt, "dt", 'i', &day);
  OptionAdd( &opt, "nt", 'i', &night);
  OptionAdd( &opt, "sf", 'i', &stfrq);
  OptionAdd( &opt, "df", 'i', &dfrq);
  OptionAdd( &opt, "nf", 'i', &nfrq);
  OptionAdd( &opt, "xcf", 'i', &xcnt);
  OptionAdd( &opt, "baud", 'i', &nbaud);
  OptionAdd( &opt, "tau", 'i', &mpinc);
  OptionAdd( &opt, "rangeres", 'i', &rsep);
  OptionAdd( &opt, "ranges", 'i', &nrang);
  OptionAdd( &opt, "PCPBEAM", 'i', &PCPBEAM);

/* Added by KTS to enable changing of beams used for FoV scan  21Dec2011 */

  OptionAdd(&opt, "sb",  'i', &sbm);
  OptionAdd(&opt, "eb",  'i', &ebm);
  OptionAdd(&opt, "ep",  'i', &errlog.port);
  OptionAdd(&opt, "sp",  'i', &shell.port);
  OptionAdd(&opt, "bp",  'i', &baseport);
  OptionAdd(&opt, "stid",'t', &ststr);
  OptionAdd(&opt, "fast",'x', &fast);
  OptionAdd(&opt, "c",   'i', &cnum);
  OptionAdd(&opt, "ros", 't', &roshost);  /* Set the roshost IP address */
  OptionAdd(&opt, "debug",'x', &debug);
  OptionAdd(&opt, "-option",'x', &option);
  OptionAdd(&opt, "-version",'x', &version);

  arg=OptionProcess(1,argc,argv,&opt,rst_opterr);

  if (arg==-1) {
    exit(-1);
  }

  if (option==1) {
    OptionDump(stdout,&opt);
    exit(0);
  }

  if (version==1) {
    OptionVersion(stdout);
    exit(0);
  }
 
  if (ststr==NULL) ststr=dfststr;

  channel = cnum;

  pcode=(int *)malloc((size_t)sizeof(int)*seq->mppul*nbaud);
  OpsBuildPcode(nbaud,seq->mppul,pcode);

  printf("Station String: %s\n",ststr);
  OpsStart(ststr);

  status=SiteBuild(libstr);

  if (status==-1) {
    fprintf(stderr,"Could not load site library.\n");
    exit(1);
  }

  status = SiteStart(roshost,ststr);    /* sbm and ebm are reset by this function. SGS */
  if (status==-1) {
    fprintf(stderr,"Error reading site configuration file.\n");
    exit(1);
  }

  /* non-standard nrang and rsep for this mode */
  nrang=565;        /* 3390 km range at 6 km range separation */
  /*nrang=750;*/    /* 4500 km range at 6 km range separation */
  rsep=6;           /* [this exceeds MAX_RANGE (700) in limit.h] */

  arg=OptionProcess(1,argc,argv,&opt,NULL);  /* this fixes it... SGS */

  sprintf(progname,"pcppcodescan");

  strncpy(combf,progid,80);

  if ((errlog.sock=TCPIPMsgOpen(errlog.host,errlog.port))==-1) {
    fprintf(stderr,"Error connecting to error log.\n");
  }

  if ((shell.sock=TCPIPMsgOpen(shell.host,shell.port))==-1) {
    fprintf(stderr,"Error connecting to shell.\n");
  }

  for (n=0;n<tnum;n++) task[n].port+=baseport;

  OpsSetupCommand(argc,argv);
  OpsSetupShell();

  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l frqrng l xcnt l",
                  &sbm,&ebm,
                  &dfrq,&nfrq,
                  &frqrng,&xcnt);

  scnsc=60;
  scnus=0;
  intsc=2;
  intus=0;

  txpl=(nbaud*rsep*20)/3;

  /* Attempt to adjust mpinc to be a multiple of 10 and a multiple of txpl */
  if ((mpinc % txpl) || (mpinc % 10)) {
    ErrLog(errlog.sock,progname,"Error: mpinc not multiple of txpl, checking to see if it can be adjusted.");
    sprintf(logtxt,"Initial: mpinc: %d  txpl: %d  nbaud: %d  rsep: %d",mpinc,txpl,nbaud,rsep);
    ErrLog(errlog.sock,progname,logtxt);

    if ((txpl % 10)==0) {
      ErrLog(errlog.sock,progname,"Attempting to adjust mpinc.");
      if (mpinc < txpl) mpinc = txpl;
      int minus_remain = mpinc % txpl;
      int plus_remain  = txpl - (mpinc % txpl);
      if (plus_remain > minus_remain) {
        mpinc = mpinc - minus_remain;
      } else {
        mpinc = mpinc + plus_remain;
      }
      if (mpinc==0) mpinc = mpinc + plus_remain;

      sprintf(logtxt,"Adjusted: mpinc: %d  txpl: %d  nbaud: %d  rsep: %d",mpinc,txpl,nbaud,rsep);
      ErrLog(errlog.sock,progname,logtxt);
    } else {
      ErrLog(errlog.sock,progname,"Cannot adjust mpinc.");
    }
  }

  /* Check mpinc and if still invalid, exit with error */
  if ((mpinc % txpl) || (mpinc % 10) || (mpinc==0)) {
    sprintf(logtxt,"Error: mpinc: %d  txpl: %d  nbaud: %d  rsep: %d",mpinc,txpl,nbaud,rsep);
    ErrLog(errlog.sock,progname,logtxt);
    SiteExit(0);
  }

  OpsSetupIQBuf(intsc,intus,mppul,mpinc,nbaud);

  status=SiteSetupRadar();
  if (status !=0) {
    ErrLog(errlog.sock,progname,"Error locating hardware.");
    exit(1);
  }

  cp=PCPCPID;    /* why do we need this block? SGS */

  if (discretion) cp= -cp;

  OpsLogStart(errlog.sock,progname,argc,argv);
  OpsSetupTask(tnum,task,errlog.sock,progname);

  for (n=0;n<tnum;n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen( (char *) command),command);
  }

  OpsFitACFStart();

  tsgid=SiteTimeSeq(seq->ptab);
  if (tsgid !=0) {
    if (tsgid==-2) {
      ErrLog(errlog.sock,progname,"Error registering timing sequence.");
    } else if (tsgid==-1) {
      ErrLog(errlog.sock,progname,"TSGMake error code: 0 (tsgbuff==NULL)");
    } else {
      sprintf(logtxt,"TSGMake error code: %d",tsgid);
      ErrLog(errlog.sock,progname,logtxt);
    }
    exit(-1);
  }

  do {

     cp=PCPCPID;    /* does the cp ever change? SGS */
     scnsc=60;      /* this never changes and is defined 3 times... SGS */
     scnus=0;       /* this one too... SGS */
     intsc=2;
     intus=500000;

    if (SiteStartScan() !=0) continue;

    TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
    if (OpsReOpen(2,0,0) !=0) {
      ErrLog(errlog.sock,progname,"Opening new files.");
      for (n=0;n<tnum;n++) {
        RMsgSndClose(task[n].sock);
        RMsgSndOpen(task[n].sock,strlen( (char *) command),command);
      }
    }

    scan=1;

    ErrLog(errlog.sock,progname,"Starting scan.");

    xcf=1;      /*  FHR unable to do xcf at this time  21Dec2011  */
                /* who cannot do xcfs? should this be turned on? SGS */

    skip=OpsFindSkip(scnsc,scnus,intsc,intus,0);

    if (backward) {
      bmnum=sbm-skip;
      if (bmnum<ebm) bmnum=sbm;
    } else {
      bmnum=sbm+skip;
      if (bmnum>ebm) bmnum=sbm;
    }

    do {

      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

      if (OpsDayNight()==1) {
        stfrq=dfrq;
      } else {
        stfrq=nfrq;
      }

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%02d:%02d:%02d:%06d)",bmnum,
                      intsc,intus,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);
      ErrLog(errlog.sock,progname,"Starting Integration.");

      SiteStartIntt(intsc,intus);

      ErrLog(errlog.sock,progname,"Doing clear frequency search.");
      sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
      ErrLog(errlog.sock,progname, logtxt);

      tfreq=SiteFCLR(stfrq,stfrq+frqrng);

      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog.sock,progname,logtxt);

      nave=SiteIntegrate(seq->lags);
      if (nave<0) {
        sprintf(logtxt,"Integration error: %d",nave);
        ErrLog(errlog.sock,progname,logtxt);
        continue;
      }
      sprintf(logtxt,"Number of sequences: %d",nave);
      ErrLog(errlog.sock,progname,logtxt);

      OpsBuildPrm(prm,seq->ptab,seq->lags);
      OpsBuildIQ(iq,&badtr);
      OpsBuildRaw(raw);
      FitACF(prm,raw,fblk,fit,site,tdiff,-999);
      FitSetAlgorithm(fit,"fitacf2");

      /* write out data here */
      msg.num=0;
      msg.tsize=0;
      tmpbuf=RadarParmFlatten(prm,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf, PRM_TYPE,0);

      tmpbuf=IQFlatten(iq,prm->nave,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,IQ_TYPE,0);

      RMsgSndAdd(&msg,sizeof(unsigned int)*2*iq->tbadtr,
                 (unsigned char *) badtr,BADTR_TYPE,0);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,(unsigned char *) sharedmemory,
                 IQS_TYPE,0);

      tmpbuf=RawFlatten(raw,prm->nrang,prm->mplgs,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,RAW_TYPE,0);

      tmpbuf=FitFlatten(fit,prm->nrang,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,FIT_TYPE,0);
      for (n=0;n<tnum;n++) RMsgSndSend(task[n].sock,&msg);
      for (n=0;n<msg.num;n++) {
        if (msg.data[n].type==PRM_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==IQ_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==RAW_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==FIT_TYPE) free(msg.ptr[n]);
      }

      RadarShell(shell.sock,&rstable);

      scan=0;
      if (bmnum==ebm) break;        /* it seems that it is critical that the
                                       number of beams must fit within the
                                       integration time, no? SGS */
      if (backward) bmnum--;
      else bmnum++;

    } while (1);
    /* ** Single beam sounding to fill remaining time ******************* */
    scan=-2;
    bmnum=PCPBEAM;
    intsc=1;
    intus=0;
    for (pcpcnt=0;pcpcnt<PCPFNUM;pcpcnt++) {
      stfrq=pcpfreqs[pcpcnt];
      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

      sprintf(logtxt,"Sounding beam:%d intt:%ds.%dus (%02d:%02d:%02d:%06d)",bmnum,
                     intsc,intus,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);
      ErrLog(errlog.sock,progname,"Starting Integration.");

      SiteStartIntt(intsc,intus);

      ErrLog(errlog.sock,progname,"Doing clear frequency search.");
      sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
      ErrLog(errlog.sock,progname, logtxt);

      tfreq=SiteFCLR(stfrq,stfrq+frqrng);

      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog.sock,progname,logtxt);

      nave=SiteIntegrate(seq->lags);
      if (nave<0) {
        sprintf(logtxt,"Integration error: %d",nave);
        ErrLog(errlog.sock,progname,logtxt);
        continue;
      }
      sprintf(logtxt,"Number of sequences: %d",nave);
      ErrLog(errlog.sock,progname,logtxt);

      OpsBuildPrm(prm,seq->ptab,seq->lags);
      OpsBuildIQ(iq,&badtr);
      OpsBuildRaw(raw);
      FitACF(prm,raw,fblk,fit,site,tdiff,-999);

      /* write out data here */
      msg.num=0;
      msg.tsize=0;
      tmpbuf=RadarParmFlatten(prm,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf, PRM_TYPE,0);

      tmpbuf=IQFlatten(iq,prm->nave,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,IQ_TYPE,0);

      RMsgSndAdd(&msg,sizeof(unsigned int)*2*iq->tbadtr,
                 (unsigned char *) badtr,BADTR_TYPE,0);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,(unsigned char *) sharedmemory,
                 IQS_TYPE,0);

      tmpbuf=RawFlatten(raw,prm->nrang,prm->mplgs,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,RAW_TYPE,0);

      tmpbuf=FitFlatten(fit,prm->nrang,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,FIT_TYPE,0);
      for (n=0;n<tnum;n++) RMsgSndSend(task[n].sock,&msg);
      for (n=0;n<msg.num;n++) {
        if (msg.data[n].type==PRM_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==IQ_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==RAW_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==FIT_TYPE) free(msg.ptr[n]);
      }
    }
    /* ************************** END Single beam sounding to fill remaining time ******************* */
    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");
    SiteEndScan(scnsc,scnus,5000);

  } while (1);

  for (n=0;n<tnum;n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  return 0;
} 

