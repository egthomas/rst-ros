/* interleavescan.c
 ============
 Author: S.G.Shepherd

 A mode which interleaves miniscans (jumps by 4 beams) in order to provide
  larger spatial coverage in a faster time but still do all beams in 1 minute.

 In support of the ERG mission.

 The mode here is specific to 20 beams for cve (0-19) and cvw (23-4).

 Eliminating the 'fast' option since we want this to be a 1-minute scan.

 Updates:
   20160531 - first version

 */

/*
 license stuff should go here...
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
#include "sequence.h"
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
char progid[80]={"interleavescan"};
char progname[256];
int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: interleavescan --help\n");
  return(-1);
}

int main(int argc,char *argv[]) {

  char logtxt[1024]="";
  char tempLog[40];

  int scannowait=0;
  /* these are set for a standard 1-min scan, any changes here will affect
     the number of beams sampled, etc.
   */
  int scnsc=60;
  int scnus=0;

  int skip;
/*  int skipsc= 3;  serves same purpose as globals intsc and intus */
/*  int skipus= 0;*/
  int cnt=0;
  int i,n;
  unsigned char discretion=0;
  int status=0;
  int fixfrq=0;

  /* new variables for dynamically creating beam sequences */
  int *bms;           /* scanning beams                                     */
  int intgt[20];      /* start times of each integration period             */
  int nintgs=20;      /* number of integration periods per scan; SGS 1-min  */
  unsigned char hlp=0;
  unsigned char option=0;
  unsigned char version=0;

  /*
    beam sequences for 24-beam MSI radars but only using 20 most meridional
      beams;
   */
  /* count     1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 */
  /*          21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 */
  int bmse[20] =
             { 0, 4, 8,12,16, 2, 6,10,14,18, 1, 5, 9,13,17, 3, 7,11,15,19};
  int bmsw[20] =
             {23,19,15,11, 7,21,17,13, 9, 5,22,18,14,10, 6,20,16,12, 8, 4};

  /*
    beam sequences for 16-beam radars (forward and backward)
   */
  /* count     1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 */
  int bmsf[16] =
             { 0, 4, 8,12, 2, 6,10,14, 1, 5, 9,13, 3, 7,11,15};
  int bmsb[16] =
             {15,11, 7, 3,13, 9, 5, 1,14,10, 6, 2,12, 8, 4, 0};

  struct sequence *seq;

  seq=OpsSequenceMake();
  OpsBuild8pulse(seq);

  /* standard radar defaults */
  cp     = 191;         /* using 191 per memorandum */
  intsc  = 3;           /* integration period; not sure how critical this is */
  intus  = 0;           /*  but can be changed here */
  mppul  = seq->mppul;
  mplgs  = seq->mplgs;
  mpinc  = seq->mpinc;
  dmpinc = seq->mpinc;
  rsep   = 45;
  txpl   = 300;         /* note: recomputed below */

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
  OptionAdd(&opt,"ep",    'i',&errlog.port);
  OptionAdd(&opt,"sp",    'i',&shell.port);
  OptionAdd(&opt,"bp",    'i',&baseport);
  OptionAdd(&opt,"stid",  't',&ststr);
  OptionAdd(&opt,"fixfrq",'i',&fixfrq);     /* fix the transmit frequency */
  OptionAdd(&opt,"c"  ,   'i',&cnum);
  OptionAdd(&opt,"ros",   't',&roshost);    /* Set the roshost IP address */
  OptionAdd(&opt,"debug", 'x',&debug);
  OptionAdd(&opt,"-help", 'x',&hlp);        /* just dump some parameters */
  OptionAdd(&opt,"-option",'x',&option);
  OptionAdd(&opt,"-version",'x',&version);

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

  if (version==1) {
    OptionVersion(stdout);
    exit(0);
  }

  if (ststr==NULL) ststr=dfststr;

  /* Point to the beams here */
  if ((strcmp(ststr,"cve") == 0) || (strcmp(ststr,"ice") == 0) || (strcmp(ststr,"fhe") == 0)) {
    bms = bmse;     /* 1-min sequence */
  } else if ((strcmp(ststr,"cvw") == 0) || (strcmp(ststr,"icw") == 0) || (strcmp(ststr,"bks") == 0)) {
    bms = bmsw;     /* 1-min sequence */
  } else if (strcmp(ststr,"fhw") == 0) {
    bms = bmsw;     /* 1-min sequence */
    for (i=0; i<nintgs; i++)
      bms[i] -= 2;
  } else {
    if ((strcmp(ststr,"kap") == 0) || (strcmp(ststr,"ksr") == 0)) {
      bms = bmsb;
    } else {
      bms = bmsf;
    }
    nintgs = 16;
  }

  /* start time of each integration period */
  for (i=0; i<nintgs; i++)
    intgt[i] = i*(intsc + intus*1e-6);

  if (hlp) {
    usage();

/*  printf("  start beam: %2d\n", sbm); */
/*  printf("  end   beam: %2d\n", ebm); */
    printf("\n");
    printf("sqnc  stme  bmno\n");
    for (i=0; i<nintgs; i++) {
      printf(" %2d   %3d    %2d", i, intgt[i], bms[i]);
      printf("\n");
    }

    return (-1);
  }

  /* end of main Dartmouth mods */
  /* not sure if -nrang commandline option works */

  if (ststr==NULL) ststr=dfststr;

  channel = cnum;

  /* rst/usr/codebase/superdarn/src.lib/os/ops.1.10/src/setup.c */
  OpsStart(ststr);

  /* rst/usr/codebase/superdarn/src.lib/os/site.1.5/src/build.c */
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
  sprintf(progname,"interleavescan");
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

  tsgid=SiteTimeSeq(seq->ptab);  /* get the timing sequence */

  if (FreqTest(ftable,fixfrq) == 1) fixfrq = 0;

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

    bmnum = bms[skip];  /* no longer need forward and backward arrays... */

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

      if (fixfrq > 0) tfreq = fixfrq;

      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog.sock,progname,logtxt);
      nave=SiteIntegrate(seq->lags);
      if (nave<0) {
        sprintf(logtxt,"Integration error:%d",nave);
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

      scan=0;
      if (skip == (nintgs-1)) break;
      skip++;
      bmnum = bms[skip];

    } while (1);

    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");
    if (scannowait==0) SiteEndScan(scnsc,scnus,5000);
  } while (1);

  for (n=0;n<tnum;n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  return 0;
}


void usage(void)
{
  printf("\ninterleavescan [command-line options]\n\n");
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
/*  printf("    -sb int : starting beam [site.c]\n");
    printf("    -eb int : ending beam [site.c]\n");
    printf("  -camp int : camping beam [7]\n");*/
  printf("    -ep int : error log port (must be set here for dual radars)\n");
  printf("    -sp int : shell port (must be set here for dual radars)\n");
  printf("    -bp int : base port (must be set here for dual radars)\n");
  printf("  -stid char: radar string (must be set here for dual radars)\n");
  printf("-fixfrq int : transmit on fixed frequency (kHz)\n");
  printf("     -c int : channel number for multi-channel radars.\n");
  printf("   -ros char: change the roshost IP address.\n");
  printf(" --help     : print this message and quit.\n");
  printf("\n");
}

