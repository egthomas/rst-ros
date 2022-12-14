/* themisscan.c
 ============
 Author: J.Spaleta
 With Modifications by M. McClorey and SGS

 Basic Parameters:
   Scan from start_beam to end_beam in sequence with a camping beam sampled
   on alternate integrations, i.e., every other beam.

 Details:
   - integration period is fixed to 3s to coincide with THEMIS ground-based
     observations. Can be changed, but currently is an integer number of sec.

   - do NOT repeat camping beam multiple times.
     E.g.,  ... 3 6 4 6 5 6 7 6 8 6 9 ...

   - can change to 1-min scan but then limited to 11 beam extent for scan.

   - any extra beams at end of scan are set to camping beam.
     E.g. for -sb 0 -eb 3:  0 6 1 6 2 6 3 6 6 6 6 ...

   - can add buffer at end of scan (see bufsc and bufus) which reduces number
     of beams sampled (with no buffer you have 40 beams in a 2-min scan)

   - MUST use -stid xxx with this control program or seg faults

 v1.6 - SGS IMPORTANT change: start beam is the actual starting beam of the
            radar. For a westward scanning radar the start beam is greater
            than the end beam reflecting the fact that these radars typically
            scan in a counter-clockwise direction.
 */

/*
 (c) 2010 JHU/APL & Others - Please Consult LICENSE.superdarn-rst.3.1-beta-18-gf704e97.txt for more information.
 */

#include <math.h>
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
#include "setup.h"
#include "sync.h"
#include "site.h"
#include "sitebuild.h"
#include "siteglobal.h"

char *ststr=NULL;
char *dfststr="tst";
char *libstr="ros";
void *tmpbuf;
size_t tmpsze;
char progid[80]={"themisscan"};
char progname[256];
int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: themisscan --help\n");
  return(-1);
}

int main(int argc,char *argv[]) {

  /*
   * commentary here: SGS
   * It seems that the mode should be decoupled from the pulse sequence.
   * The pulse table and lag table should be externally defined with some
   * way of determining the time it takes for a given sequence.
   */
  int ptab[8] = {0,14,22,24,27,31,42,43};

  int lags[LAG_SIZE][2] = {
    { 0, 0},    /*  0 */
    {42,43},    /*  1 */
    {22,24},    /*  2 */
    {24,27},    /*  3 */
    {27,31},    /*  4 */
    {22,27},    /*  5 */

    {24,31},    /*  7 */
    {14,22},    /*  8 */
    {22,31},    /*  9 */
    {14,24},    /* 10 */
    {31,42},    /* 11 */
    {31,43},    /* 12 */
    {14,27},    /* 13 */
    { 0,14},    /* 14 */
    {27,42},    /* 15 */
    {27,43},    /* 16 */
    {14,31},    /* 17 */
    {24,42},    /* 18 */
    {24,43},    /* 19 */
    {22,42},    /* 20 */
    {22,43},    /* 21 */
    { 0,22},    /* 22 */

    { 0,24},    /* 24 */

    {43,43}};   /* alternate lag-0  */

  char logtxt[1024]="";
  char tempLog[40];

  int exitpoll=0;
  int scannowait=0;
  int scnsc=120;
  int scnus=0;
  int skip;
/*  int skipsc= 3;      serves same purpose as globals intsc and intus */
/*  int skipus= 0;*/
  int cnt=0;
  int i,n;
  unsigned char discretion=0;
  int status=0;
  int fixfrq=0;
  int cbm=7;        /* default camping beam */

  /* new variables for dynamically creating beam sequences */
  int *intgt;
  int *bms;       /* scanning beams */
  int nintgs;     /* number of integration periods per scan */
  int tbm;
  int odd;
  int bufsc=0;    /* a buffer at the end of scan; historically this has   */
  int bufus=0;    /*   been set to 3.0s to account for what???            */
  unsigned char hlp=0;
  unsigned char option=0;

  /* standard radar defaults */
  cp     = 3300;    /* themisscan cpid */
  intsc  = 3;       /* integration period; fundamental to THEMIS */
  intus  = 0;       /*  but can be changed here */
  mppul  = 8;
  mplgs  = 23;
  mpinc  = 1500;
  dmpinc = 1500;
  nrang  = 100;
  rsep   = 45;
  txpl   = 300;     /* note: recomputed below */

/* ========= PROCESS COMMAND LINE ARGUMENTS ============= */

  OptionAdd(&opt,"di",    'x',&discretion);
  OptionAdd(&opt,"frang", 'i',&frang);
  OptionAdd(&opt,"rsep",  'i',&rsep);
  OptionAdd(&opt,"dt",    'i',&day);
  OptionAdd(&opt,"nt",    'i',&night);
  OptionAdd(&opt,"df",    'i',&dfrq);
  OptionAdd(&opt,"nf",    'i',&nfrq);
  OptionAdd(&opt,"xcf",   'x',&xcnt);
  OptionAdd(&opt,"nrang", 'i',&nrang);
  OptionAdd(&opt,"sb",    'i',&sbm);
  OptionAdd(&opt,"eb",    'i',&ebm);
  OptionAdd(&opt,"ep",    'i',&errlog.port);
  OptionAdd(&opt,"sp",    'i',&shell.port);
  OptionAdd(&opt,"bp",    'i',&baseport);
  OptionAdd(&opt,"stid",  't',&ststr);
  OptionAdd(&opt,"fixfrq",'i',&fixfrq);     /* fix the transmit frequency */
  OptionAdd(&opt,"camp",  'i',&cbm);        /* camping beam number */
  OptionAdd(&opt,"ros",   't',&roshost);    /* Set the roshost IP address */
  OptionAdd(&opt,"debug", 'x',&debug);
  OptionAdd(&opt,"-help", 'x',&hlp);        /* just dump some parameters */
  OptionAdd(&opt,"-option",'x',&option);

  /* Process all of the command line options
      Important: need to do this here because we need stid and ststr */
  arg=OptionProcess(1,argc,argv,&opt,rst_opterr);

  if (arg==-1) {
    exit(-1);
  }

  if (option==1) {
    OptionDump(stdout,&opt);
    exit(0);
  }

  /* number of integration periods possible in scan time */
  /* modified to allow no buffer at end of scan. change bufsc to 3s for 
     standard 3s delay at end of scan */
  nintgs = (int)floor( (scnsc+scnus*1e-6 - (bufsc+bufus*1e-6))/
                        (intsc + intus*1e-6) );

  /* arrays for integration start times and beam sequences */
  intgt = (int *)malloc(nintgs*sizeof(int));
  bms   = (int *)malloc(nintgs*sizeof(int));

  /* start time of each integration period */
  for (i=0; i<nintgs; i++)
    intgt[i] = i*(intsc + intus*1e-6);

  /* Create a list of the beams that will be sequenced through, alternating
   * between the camping beam and each other beam in turn */

  odd = (sbm == cbm);       /* odd/even camp beams */
  backward = (sbm > ebm) ? 1 : 0;   /* allow for non-standard scan dir. */

  if (backward) {
    tbm = sbm;
    if (sbm == cbm) tbm--;

    for (i=0; i<nintgs; i++){
      if ((i%2 == odd) && (tbm >= ebm)) {
        /* exclude camp bm from regular beams */
        if ((tbm == cbm) && (sbm != cbm)) tbm--;
        if (tbm >= ebm) bms[i] = tbm;
        else bms[i] = cbm;
        tbm--;
      } else {
        bms[i] = cbm;
      }
    }
  } else {  /* forward */
    tbm = sbm;
    if (sbm == cbm) tbm++;

    for (i=0; i<nintgs; i++){
      if ((i%2 == odd) && (tbm <= ebm)) {
        /* exclude camp bm from regular beams */
        if ((tbm == cbm) && (sbm != cbm)) tbm++;
        if (tbm <= ebm) bms[i] = tbm;
        else bms[i] = cbm;
        tbm++;
      } else {
        bms[i] = cbm;
      }
    }
  }

  if (hlp) {
    usage();

    printf("  start beam: %2d\n", sbm);
    printf("  end   beam: %2d\n", ebm);
    printf("  camp  beam: %2d\n", cbm);
    printf("\n");
    printf("sqnc  stme  bmno\n");
    for (i=0; i<nintgs; i++) {
      printf(" %2d   %3d    %2d", i, intgt[i], bms[i]);
      if (bms[i] == cbm) printf("   CAMP");
      printf("\n");
    }

    free(intgt);
    free(bms);

    return (-1);
  }

  /* end of main Dartmouth mods */
  /* not sure if -nrang commandline option works */

  if (ststr==NULL) ststr=dfststr;

  /* rst/usr/codebase/superdarn/src.lib/os/ops.1.10/src/setup.c */
  OpsStart(ststr);

  status=SiteBuild(libstr);

  if (status==-1) {
    fprintf(stderr,"Could not load site library.\n");
    exit(1);
  }

  /* IMPORTANT: sbm and ebm are reset by this function */
  status = SiteStart(roshost,ststr);
  if (status==-1) {
    fprintf(stderr,"Error reading site configuration file.\n");
    exit(1);
  }

  /* Reprocess the command line to restore desired parameters */
  arg=OptionProcess(1,argc,argv,&opt,NULL);
  backward = (sbm > ebm) ? 1 : 0;   /* this almost certainly got reset */

  strncpy(combf,progid,80);

  if ((errlog.sock=TCPIPMsgOpen(errlog.host,errlog.port))==-1) {
    fprintf(stderr,"Error connecting to error log.\n");
  }
  if ((shell.sock=TCPIPMsgOpen(shell.host,shell.port))==-1) {
    fprintf(stderr,"Error connecting to shell.\n");
  }

  for (n=0;n<tnum;n++) task[n].port+=baseport;

  /* dump beams to log file */
  sprintf(progname,"themisscan");
  for (i=0; i<nintgs; i++){
    sprintf(tempLog, "%3d", bms[i]);
    strcat(logtxt, tempLog);
  }
  ErrLog(errlog.sock,progname,logtxt);

  /* rst/usr/codebase/superdarn/src.lib/os/ops.1.10/src */
  OpsSetupCommand(argc,argv);
  OpsSetupShell();

  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l dfrang l nfrang l"
          " dmpinc l nmpinc l frqrng l xcnt l", &sbm,&ebm, &dfrq,&nfrq,
          &dfrang,&nfrang, &dmpinc,&nmpinc, &frqrng,&xcnt);

  status=SiteSetupRadar();

  fprintf(stderr,"Status:%d\n",status);

  if (status !=0) {
    ErrLog(errlog.sock,progname,"Error locating hardware.");
    exit(1);
  }

  if (discretion) cp = -cp;

  txpl=(rsep*20)/3;     /* computing TX pulse length */

  OpsLogStart(errlog.sock,progname,argc,argv);
  OpsSetupTask(tnum,task,errlog.sock,progname);

  for (n=0;n<tnum;n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen( (char *) command),command);
  }

  OpsFitACFStart();
  
  tsgid=SiteTimeSeq(ptab);      /* get the timing sequence */

  do {

    if (SiteStartScan() !=0) continue;

    if (OpsReOpen(2,0,0) !=0) {
      ErrLog(errlog.sock,progname,"Opening new files.");
      for (n=0;n<tnum;n++) {
        RMsgSndClose(task[n].sock);
        RMsgSndOpen(task[n].sock,strlen( (char *) command),command);
      }
    }

    scan=1;

    ErrLog(errlog.sock,progname,"Starting scan.");

    if (xcnt>0) {
      cnt++;
      if (cnt==xcnt) {
        xcf=1;
        cnt=0;
      } else xcf=0;
    } else xcf=0;

    skip=OpsFindSkip(scnsc,scnus,intsc,intus,0);

    /* is this C99? why is this a sequential control structure with variables
     * being declared and not attached to anything? */
    {
      int tv;
      int bv;
      int iv;
      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
      iv = intsc*1000000 + intus;   /* integration period in microseconds */
      bv = scnsc*1000000 + scnus;   /* scan period in microseconds */
      /* select the beam based on the current time.
         centered on integration peroid (+iv/2)
         minus a delay (-.1us) not sure how solid this number is */
      tv = (mt* 60 + sc)* 1000000 + us + iv/2 - 100000;
      skip = (tv % bv)/iv;
      if (skip > nintgs-1) skip = 0;
      else if (skip < 0)   skip = 0;
    }

    bmnum = bms[skip];      /* no longer need forward and backward arrays... */

    do {

      /* Synchronize to the desired start time */

      /* This will only work, if the total time through the do loop is < 3s */
      /* If this is not the case, decrease the Integration time */
      /* MAX < or <=  3s ? */
      /* once again, don't like this... */

      {
        int t_now;
        int t_dly;
        TimeReadClock( &yr, &mo, &dy, &hr, &mt, &sc, &us);
        t_now = ( (mt*60 + sc)*1000 + us/1000 ) % (scnsc*1000 + scnus/1000);
        t_dly = intgt[skip]*1000 - t_now;
        if (t_dly > 0) usleep(t_dly);
      }

      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

      if (OpsDayNight()==1) {
        stfrq=dfrq;
        mpinc=dmpinc;
        frang=dfrang;
      } else {
        stfrq=nfrq;
        mpinc=nmpinc;
        frang=nfrang;
      }

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%d:%d:%d:%d)",bmnum,
              intsc,intus,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);

      ErrLog(errlog.sock,progname,"Starting Integration.");
      SiteStartIntt(intsc,intus);

      ErrLog(errlog.sock,progname,"Doing clear frequency search.");
      sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
      ErrLog(errlog.sock,progname, logtxt);
      tfreq=SiteFCLR(stfrq,stfrq+frqrng);

      if ( (fixfrq > 8000) && (fixfrq < 25000) ) tfreq = fixfrq;

      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog.sock,progname,logtxt);
      nave=SiteIntegrate(lags);
      if (nave<0) {
        sprintf(logtxt,"Integration error:%d",nave);
        ErrLog(errlog.sock,progname,logtxt);
        continue;
      }
      sprintf(logtxt,"Number of sequences: %d",nave);
      ErrLog(errlog.sock,progname,logtxt);

      OpsBuildPrm(prm,ptab,lags);
      OpsBuildIQ(iq,&badtr);
      OpsBuildRaw(raw);

      FitACF(prm,raw,fblk,fit,site,tdiff,-999);
      FitSetAlgorithm(fit,"fitacf2");

      msg.num=0;
      msg.tsize=0;

      tmpbuf=RadarParmFlatten(prm,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,PRM_TYPE,0);

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

      RMsgSndAdd(&msg,strlen(progname)+1,(unsigned char *) progname,
                 NME_TYPE,0);

      for (n=0;n<tnum;n++) RMsgSndSend(task[n].sock,&msg);

      for (n=0;n<msg.num;n++) {
        if (msg.data[n].type==PRM_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==IQ_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==RAW_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==FIT_TYPE) free(msg.ptr[n]);
      }

      RadarShell(shell.sock,&rstable);

      if (exitpoll !=0) break;
      scan=0;
      if (skip == (nintgs-1)) break;
      skip++;
      bmnum = bms[skip];

    } while (1);

    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");
    if ((exitpoll==0) && (scannowait==0)) SiteEndScan(scnsc,scnus,5000);
  } while (exitpoll==0);

  for (n=0;n<tnum;n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  free(bms);
  free(intgt);

  return 0;
}


void usage(void)
{
  printf("\nthemisscan [command-line options]\n\n");
  printf("command-line options:\n");
  printf("    -di     : indicates running during discretionary time\n");
  printf(" -frang int : delay to first range (km) [180]\n");
  printf("  -rsep int : range separation (km) [45]\n");
  printf("    -dt int : hour when day freq. is used [site.c]\n");
  printf("    -nt int : hour when night freq. is used [site.c]\n");
  printf("    -df int : daytime frequency (kHz) [site.c]\n");
  printf("    -nf int : nighttime frequency (kHz) [site.c]\n");
  printf("   -xcf     : set for computing XCFs [global.c]\n");
  printf(" -nrang int : number or range gates [limit.h]\n");
  printf("    -sb int : starting beam [site.c]\n");
  printf("    -eb int : ending beam [site.c]\n");
  printf("  -camp int : camping beam [7]\n");
  printf("    -ep int : error log port (must be set here for dual radars)\n");
  printf("    -sp int : shell port (must be set here for dual radars)\n");
  printf("    -bp int : base port (must be set here for dual radars)\n");
  printf("  -stid char: radar string (must be set here for dual radars)\n");
  printf("-fixfrq int : transmit on fixed frequency (kHz)\n");
  printf("  -ros char : change the roshost IP address\n");
  printf(" --help     : print this message and quit.\n");
  printf("\n");
}

