/* soundingscan.c
   ============
   Adapted from icescan: K.T. Sterne, originally normalscan by:
   Author: R.J.Barnes & J.Spaleta
*/

/* $License */

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
#include "rosmsg.h"
#include "tsg.h"

char *ststr=NULL;
char *dfststr="tst";
char *libstr="ros";

void *tmpbuf;
size_t tmpsze;

char progid[80]={"soundingscan"};
char progname[256];

int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: soundingscan --help\n");
  return(-1);
}

int main(int argc,char *argv[]) {

  char logtxt[1024];

  int scannowait=0;

  int scnsc=120; /* Default of 1 minute scan time. */
  int scnus=0;
  int skip;
  int cnt=0;

  unsigned char fast=0;
  unsigned char discretion=0;
  int fixfrq=0;
  int cpid=0;
  int rxonly=0;
  int setintt=0;  /* flag to override auto-calc of integration time */

  int n;
  int status=0;

  int beams=0;
  int total_scan_usecs=0;
  int total_integration_usecs=0;

  int bufsc=0;    /* a buffer at the end of scan; historically this has   */
  int bufus=0;    /*   been set to 3.0s to account for what???            */
  unsigned char hlp=0;
  unsigned char option=0;
  unsigned char version=0;

  /* Flag and variables for beam synchronizing */
  int bm_sync = 0;
  int bmsc    = 6;
  int bmus    = 0;

  /* Soundingscan specific parameters  */
  /* bistatic frequencies ?
  int sndfreqs[] = {10250, 10725, 11500, 11925, 12375,
                    12800, 13550, 14450, 15200, 15800};
  int nfreqs  = 10;
  */

  /* Iceland test frequencies */
  int sndfreqs[] = {10200, 10400, 10600, 10800, 11000};
  int nfreqs  = 5;

  /*
  int sndfreqs[] = {10250, 10725, 11800, 12800, 13800, 14500, 15200, 15800};
  int nfreqs  = 8;
  */
  int freqcnt = 0;
  int offset  = 0;  /* freq offset kHz for second radar */

  struct sequence *seq;

  seq=OpsSequenceMake();
  OpsBuild8pulse(seq);

  cp     = 1200;        /* CPID */
  intsc  = 6;           /* integration period; recomputed below ... */
  intus  = 0;
  mppul  = seq->mppul;  /* number of pulses */
  mplgs  = seq->mplgs;  /* number of lags */
  mpinc  = seq->mpinc;  /* multi-pulse increment [us] */
  rsep   = 45;          /* same for the range separation */
  txpl   = 300;         /* pulse length [us]; gets redefined below... */

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */

  OptionAdd(&opt, "di",     'x', &discretion);
  OptionAdd(&opt, "frang",  'i', &frang);
  OptionAdd(&opt, "rsep",   'i', &rsep);
  OptionAdd(&opt, "nrang",  'i', &nrang);
  OptionAdd(&opt, "dt",     'i', &day);
  OptionAdd(&opt, "nt",     'i', &night);
  OptionAdd(&opt, "df",     'i', &dfrq);
  OptionAdd(&opt, "nf",     'i', &nfrq);
  OptionAdd(&opt, "xcf",    'x', &xcnt);
  OptionAdd(&opt, "ep",     'i', &errlog.port);
  OptionAdd(&opt, "sp",     'i', &shell.port);
  OptionAdd(&opt, "bp",     'i', &baseport);
  OptionAdd(&opt, "stid",   't', &ststr);
  OptionAdd(&opt, "fast",   'x', &fast);
  OptionAdd(&opt, "nowait", 'x', &scannowait);
  OptionAdd(&opt, "sb",     'i', &sbm);
  OptionAdd(&opt, "eb",     'i', &ebm);
  OptionAdd(&opt, "fixfrq", 'x', &fixfrq);   /* fix the transmit frequency  */
  OptionAdd(&opt, "frqrng", 'i', &frqrng);   /* fix the FCLR window [kHz]   */
  OptionAdd(&opt, "cpid",   'i', &cpid);     /* allow user to specify CPID, *
                                                e.g., RX-only               */
  OptionAdd(&opt, "rxonly", 'x', &rxonly);   /* RX-only mode                */
  OptionAdd(&opt, "offset", 'i', &offset);   /* offset freq for dual in kHz */
  OptionAdd(&opt, "bm_sync",'x', &bm_sync);  /* flag to enable beam sync    */
  OptionAdd(&opt, "bmsc",   'i', &bmsc);     /* beam sync period, sec       */
  OptionAdd(&opt, "bmus",   'i', &bmus);     /* beam sync period, microsec  */
  OptionAdd(&opt, "intsc",  'i', &intsc);
  OptionAdd(&opt, "intus",  'i', &intus);
  OptionAdd(&opt, "setintt",'x', &setintt);

  OptionAdd(&opt, "c",      'i', &cnum);
  OptionAdd(&opt, "ros",    't', &roshost);  /* Set the roshost IP address  */
  OptionAdd(&opt, "debug"  ,'x', &debug);
  OptionAdd(&opt, "-help",  'x', &hlp);      /* just dump some parameters   */
  OptionAdd(&opt, "-option",'x', &option);
  OptionAdd(&opt,"-version",'x', &version);

  /* process the commandline; need this for setting errlog port */
  arg = OptionProcess(1,argc,argv,&opt,rst_opterr);

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

  backward = (sbm > ebm) ? 1 : 0;   /* allow for non-standard scan dir. */

  if (hlp) {
    usage();

    printf("  start beam: %2d\n", sbm);
    printf("  end   beam: %2d\n", ebm);
    if (backward) printf("  backward\n");
    else printf("  forward\n");

    return (-1);
  }

  if (ststr==NULL) ststr=dfststr;

  channel = cnum;

  OpsStart(ststr);

  status = SiteBuild(libstr);
  if (status==-1) {
    fprintf(stderr,"Could not load site library.\n");
    exit(1);
  }

  status = SiteStart(roshost,ststr);
  if (status==-1) {
    fprintf(stderr,"Error reading site configuration file.\n");
    exit(1);
  }

  /* reprocess the commandline since some things are reset by SiteStart */
  arg = OptionProcess(1,argc,argv,&opt,NULL);
  backward = (sbm > ebm) ? 1 : 0;   /* this almost certainly got reset */

  if (rxonly) {
    strcpy(progid, "rxonlybistaticsound; BISTATIC");
    strcpy(progname, "rxonlybistaticsound; BISTATIC");
    txpow = 1;
  } else {
    if (fast) sprintf(progname,"soundingscan (fast)");
    else sprintf(progname,"soundingscan");
  }

  printf("Station ID: %s  %d\n",ststr,stid);
  strncpy(combf,progid,80);

  if ((errlog.sock=TCPIPMsgOpen(errlog.host,errlog.port))==-1)
    fprintf(stderr,"Error connecting to error log.\n");

  if ((shell.sock=TCPIPMsgOpen(shell.host,shell.port))==-1)
    fprintf(stderr,"Error connecting to shell.\n");

  for (n=0; n<tnum; n++) task[n].port += baseport;

  OpsSetupCommand(argc,argv);
  OpsSetupShell();

  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l"
                  " frqrng l xcnt l", &sbm,&ebm, &dfrq,&nfrq,
                  &frqrng,&xcnt);

  beams = abs(ebm-sbm)+1;
  if (fast) {
    cp   += 1;
    scnsc = 60;
    scnus = 0;
  } else {
    scnsc = 120;
    scnus = 0;
  }

  if ((scannowait==0) && (setintt==0)) {
    total_scan_usecs = scnsc*1e6 + scnus - (bufsc*1e6 + bufus);
    total_integration_usecs = total_scan_usecs/beams;
    intsc = total_integration_usecs/1e6;
    intus = total_integration_usecs - (intsc*1e6);
  }

  OpsSetupIQBuf(intsc,intus,mppul,mpinc,nbaud);

  status = SiteSetupRadar();
  if (status !=0) {
    ErrLog(errlog.sock,progname,"Error locating hardware.");
    exit(1);
  }

  printf("Initial Setup Complete: Station ID: %s  %d\n",ststr,stid);

  if (cpid) cp = cpid;  /* user is setting the CPID;
                           discretionary flips sign below */

  if (discretion) cp = -cp;

  txpl = (rsep*20)/3;

  OpsLogStart(errlog.sock,progname,argc,argv);
  OpsSetupTask(tnum,task,errlog.sock,progname);

  for (n=0; n<tnum; n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen((char *)command),command);
  }

  printf("Preparing OpsFitACFStart Station ID: %s  %d\n",ststr,stid);
  OpsFitACFStart();

  printf("Preparing SiteTimeSeq Station ID: %s  %d\n",ststr,stid);
  tsgid = SiteTimeSeq(seq->ptab);

  if (bm_sync) skip = OpsFindSkip(scnsc,scnus, bmsc,bmus, 0);
  else         skip = OpsFindSkip(scnsc,scnus, intsc,intus, 0);
  if (backward) {
    bmnum = sbm-skip;
    if (bmnum < ebm) bmnum = sbm;
  } else {
    bmnum = sbm+skip;
    if (bmnum > ebm) bmnum = sbm;
  }

  printf("Entering Scan loop Station ID: %s  %d\n",ststr,stid);
  do {

    printf("Entering Site Start Scan Station ID: %s  %d\n",ststr,stid);
    if (SiteStartScan() !=0) continue;

    TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
    if (OpsReOpen(2,0,0) !=0) {
      ErrLog(errlog.sock,progname,"Opening new files.");
      for (n=0; n<tnum; n++) {
        RMsgSndClose(task[n].sock);
        RMsgSndOpen(task[n].sock,strlen((char *)command),command);
      }
    }

    scan = 1; /* scan flag */

    ErrLog(errlog.sock,progname,"Starting scan.");
    if (xcnt > 0) {
      cnt++;
      if (cnt == xcnt) {
        xcf = 1;
        cnt = 0;
      } else xcf = 0;
    } else xcf = 0;

    /* Logic to change frequency band on every scan, but also sync to minute */
    TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
/*    freqcnt = mt % (scnsc/60 * nfreqs); WTF is this?! */
    freqcnt = mt*60/scnsc % nfreqs;
    stfrq = sndfreqs[freqcnt] + offset;

    /* Logic to change frequency band on every scan */
/*
    stfrq = sndfreqs[freqcnt] + offset;
    freqcnt = (freqcnt == nfreqs-1) ? 0 : freqcnt+1;
*/

    do {  /* start of scan */

      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

/*
      if (OpsDayNight()==1) {
        stfrq = dfrq;
      } else {
        stfrq = nfrq;
      }
*/

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%02d:%02d:%02d:%06d)",
                      bmnum,intsc,intus,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);

      ErrLog(errlog.sock,progname,"Starting Integration.");
      printf("Entering Site Start Intt Station ID: %s  %d\n",ststr,stid);
      SiteStartIntt(intsc,intus);

      /* clear frequency search business */
      ErrLog(errlog.sock,progname,"Doing clear frequency search.");
      sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
      ErrLog(errlog.sock,progname, logtxt);
      tfreq = SiteFCLR(stfrq,stfrq+frqrng);

      if (fixfrq) tfreq = stfrq;
      /*if ( (fixfrq > 8000) && (fixfrq < 25000) ) tfreq = fixfrq;*/

      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog.sock,progname,logtxt);

      nave = SiteIntegrate(seq->lags);
      if (nave < 0) {
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

      msg.num   = 0;
      msg.tsize = 0;

      tmpbuf = RadarParmFlatten(prm,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf, PRM_TYPE,0);

      tmpbuf = IQFlatten(iq,prm->nave,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,IQ_TYPE,0);

      RMsgSndAdd(&msg,sizeof(unsigned int)*2*iq->tbadtr,
                 (unsigned char *)badtr,BADTR_TYPE,0);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,(unsigned char *)sharedmemory,
                  IQS_TYPE,0);

      tmpbuf = RawFlatten(raw,prm->nrang,prm->mplgs,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,RAW_TYPE,0);

      tmpbuf=FitFlatten(fit,prm->nrang,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,FIT_TYPE,0);

      for (n=0; n<tnum; n++) RMsgSndSend(task[n].sock,&msg);

      for (n=0; n<msg.num; n++) {
        if (msg.data[n].type==PRM_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==IQ_TYPE)  free(msg.ptr[n]);
        if (msg.data[n].type==RAW_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==FIT_TYPE) free(msg.ptr[n]);
      }

      RadarShell(shell.sock,&rstable);

      scan = 0;
      if (bmnum == ebm) break;
      if (backward) bmnum--;
      else bmnum++;

      if (bm_sync) {  /* synchronize beam */
        ErrLog(errlog.sock,progname,"Synchronizing beam.");
        SiteEndScan(bmsc,bmus,5000);
      }

    } while (1);

    bmnum = sbm;

    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");
    if (scannowait==0) SiteEndScan(scnsc,scnus,5000);
  } while (1);

  for (n=0; n<tnum; n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  return 0;
}

void usage(void)
{
  printf("\nsoundingscan [command-line options]\n\n");
  printf("command-line options:\n");
  printf("  -stid char: radar string (required)\n");
  printf("    -di     : indicates running during discretionary time\n");
  printf("  -fast     : 1-min scan (2-min default)\n");
  printf(" -frang int : delay to first range (km) [180]\n");
  printf("  -rsep int : range separation (km) [45]\n");
  printf(" -nrang int : number of range gates\n");
  printf("    -dt int : hour when day freq. is used\n");
  printf("    -nt int : hour when night freq. is used\n");
  printf("    -df int : daytime frequency (kHz)\n");
  printf("    -nf int : nighttime frequency (kHz)\n");
  printf("   -xcf     : set for computing XCFs\n");
  printf("    -sb int : starting beam\n");
  printf("    -eb int : ending beam\n");
  printf("    -ep int : error log port\n");
  printf("    -sp int : shell port\n");
  printf("    -bp int : base port\n");
  printf("  -cpid int : set to override control program id\n");
  printf("-fixfrq int : transmit on fixed frequency (kHz)\n");
  printf("-frqrng int : set the clear frequency search window (kHz)\n");
  printf("-nowait     : do not wait at end of scan boundary.\n");
  printf("-rxonly     : bistatic RX only mode.\n");
  printf("-bm_sync    : set to enable beam syncing.\n");
  printf("  -bmsc int : beam syncing interval seconds.\n");
  printf("  -bmus int : beam syncing interval microseconds.\n");
  printf("-setintt    : set to enable integration period override.\n");
  printf(" -intsc int : integration period seconds.\n");
  printf(" -intus int : integration period microseconds.\n");
  printf("     -c int : channel number for multi-channel radars.\n");
  printf("   -ros char: change the roshost IP address\n");
  printf(" --help     : print this message and quit.\n");
  printf("\n");
}

