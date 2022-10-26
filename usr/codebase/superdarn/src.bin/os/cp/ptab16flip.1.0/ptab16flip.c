/* ptab16flip.c
   ============
   Author: J.Spaleta
   Modified: E.G.Thomas

*/

/*
 license stuff should go here...
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
#include "setup.h"
#include "sync.h"
#include "site.h"
#include "sitebuild.h"
#include "siteglobal.h"
#include "rosmsg.h"
#include "tsg.h"

char *ststr=NULL;
char *dfststr="tst";

void *tmpbuf;
size_t tmpsze;

char progid[80]={"ptab16flip 2022/09/06"};
char progname[256];

int arg=0;
struct OptionData opt;

int tnum=2;
struct TCPIPMsgHost task[4]={
  {"127.0.0.1",1,-1}, /* iqwrite */
  {"127.0.0.1",2,-1}, /* rawacfwrite */
  {"127.0.0.1",3,-1}, /* fitacfwrite */
  {"127.0.0.1",4,-1}  /* rtserver */
};

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: ptab16flip --help\n");
  return(-1);
}

int main(int argc,char *argv[])
{

  int ptab_short[8] = {0,14,22,24,27,31,42,43};
  int ptab_long[16] = {0,4,19,42,78,127,191,270,364,474,600,745,905,1083,1280,1495};

  int lags_short[LAG_SIZE][2] = {
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


  int lags_long[LAG_SIZE][2] = {
    {1495,1495},   /*  0 */
    {0,4},         /*  1 */
    {4,19},        /*  2 */
    {0,19},        /*  3 */
    {19,42},       /*  4 */
    {42,78},       /*  5 */
    {4,42},        /*  6 */
    {0,42},        /*  7 */
    {78,127},      /*  8 */
    {19,78},       /*  9 */
    {127,191},     /*  10 */
    {4,78},        /*  11 */
    {0,78},        /*  12 */
    {191,270},     /*  13 */
    {42,127},      /*  14 */
    {270,364},     /*  15 */
    {19,127},      /*  16 */
    {364,474},     /*  17 */
    {78,191},      /*  18 */
    {4,127},       /*  19 */
    {474,600},     /*  20 */
    {0,127},       /*  21 */
    {127,270},     /*  22 */
    {600,745},     /*  23 */
    {42,191},      /*  24 */
    {745,905},     /*  25 */
    {191,364},     /*  26 */
    {19,191},      /*  27 */
    {905,1083},    /*  28 */
    {4,191},       /*  29 */
    {0,191},       /*  30 */
    {78,270},      /*  31 */
    {1083,1280},   /*  32 */
    {270,474},     /*  33 */
    {1280,1495},   /*  34 */
    {42,270},      /*  35 */
    {127,364},     /*  36 */
    {364,600},     /*  37 */
    {19,270},      /*  38 */
    {4,270},       /*  39 */
    {0,270},       /*  40 */
    {474,745},     /*  41 */
    {191,474},     /*  42 */
    {78,364},      /*  43 */
    {600,905},     /*  44 */
    {42,364},      /*  45 */
    {270,600},     /*  46 */
    {745,1083},    /*  47 */
    {19,364},      /*  48 */
    {127,474},     /*  49 */
    {4,364},       /*  50 */
    {0,364},       /*  51 */
    {905,1280},    /*  52 */
    {364,745},     /*  53 */
    {78,474},      /*  54 */
    {191,600},     /*  55 */
    {1083,1495},   /*  56 */
    {474,905},     /*  57 */
    {42,474},      /*  58 */
    {19,474},      /*  59 */
    {4,474},       /*  60 */
    {127,600},     /*  61 */
    {0,474},       /*  62 */
    {270,745},     /*  63 */
    {600,1083},    /*  64 */
    {78,600},      /*  65 */
    {745,1280},    /*  66 */
    {364,905},     /*  67 */
    {191,745},     /*  68 */
    {42,600},      /*  69 */
    {19,600},      /*  70 */
    {905,1495},    /*  71 */
    {4,600},       /*  72 */
    {0,600},       /*  73 */
    {474,1083},    /*  74 */
    {127,745},     /*  75 */
    {270,905},     /*  76 */
    {78,745},      /*  77 */
    {600,1280},    /*  78 */
    {42,745},      /*  79 */
    {191,905},     /*  80 */
    {364,1083},    /*  81 */
    {19,745},      /*  82 */
    {4,745},       /*  83 */
    {0,745},       /*  84 */
    {745,1495},    /*  85 */
    {127,905},     /*  86 */
    {474,1280},    /*  87 */
    {270,1083},    /*  88 */
    {78,905},      /*  89 */
    {42,905},      /*  90 */
    {19,905},      /*  91 */
    {191,1083},    /*  92 */
    {600,1495},    /*  93 */
    {4,905},       /*  94 */
    {0,905},       /*  95 */
    {364,1280},    /*  96 */
    {127,1083},    /*  97 */
    {78,1083},     /*  98 */
    {270,1280},    /*  99 */
    {474,1495},    /*  100 */
    {42,1083},     /*  101 */
    {19,1083},     /*  102 */
    {4,1083},      /*  103 */
    {0,1083},      /*  104 */
    {191,1280},    /*  105 */
    {364,1495},    /*  106 */
    {127,1280},    /*  107 */
    {78,1280},     /*  108 */
    {270,1495},    /*  109 */
    {42,1280},     /*  110 */
    {19,1280},     /*  111 */
    {4,1280},      /*  112 */
    {0,1280},      /*  113 */
    {191,1495},    /*  114 */
    {127,1495},    /*  115 */
    {78,1495},     /*  116 */
    {42,1495},     /*  117 */
    {19,1495},     /*  118 */
    {4,1495},      /*  119 */
    {0,1495},      /*  120 */
    {1495,1495}};  /*  121 */

  char logtxt[1024];

  int scnsc=120;    /* total scan period in seconds */
  int scnus=0;
  int skip;
  int cnt=0;

  unsigned char discretion=0;
  int fixfrq=0;

  int n;
  int status=0;

  int beams=0;
  int total_scan_usecs=0;
  int total_integration_usecs=0;
  int debug=0;

  int bufsc=4;    /* a buffer at the end of scan; historically this has   */
  int bufus=0;    /*   been set to 3.0s to account for what???            */

  int mpinc_short=1500;
  int mplgs_short=23;
  int mppul_short=8;

  int mpinc_long=100;
  int mplgs_long=121;
  int mppul_long=16;
  int flipflop=0;

  unsigned char hlp=0;

  if (debug) {
    printf("Size of int %u\n",sizeof(int));
    printf("Size of long %u\n",sizeof(long));
    printf("Size of long long %u\n",sizeof(long long));
    printf("Size of struct TRTimes %u\n",sizeof(struct TRTimes));
    printf("Size of struct SeqPRM %u\n",sizeof(struct SeqPRM));
    printf("Size of struct RosData %u\n",sizeof(struct RosData));
    printf("Size of struct DataPRM %u\n",sizeof(struct DataPRM));
    printf("Size of Struct ControlPRM  %u\n",sizeof(struct ControlPRM));
    printf("Size of Struct RadarPRM  %u\n",sizeof(struct RadarPRM));
    printf("Size of Struct ROSMsg  %u\n",sizeof(struct ROSMsg));
    printf("Size of Struct CLRFreq  %u\n",sizeof(struct CLRFreqPRM));
    printf("Size of Struct TSGprm  %u\n",sizeof(struct TSGprm));
    printf("Size of Struct SiteSettings  %u\n",sizeof(struct SiteSettings));
  }

  cp     = 9916;    /* CPID */
  intsc  = 3;       /* integration period; recomputed below ... */
  intus  = 0;
  nrang  = 300;     /* the number of ranges gets set in SiteXXXStart() */
  rsep   = 15;      /* same for the range separation */
  txpl   = 100;     /* pulse length [us]; gets redefined below... */

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */

  OptionAdd(&opt, "di",     'x', &discretion);
  OptionAdd(&opt, "dt",     'i', &day);
  OptionAdd(&opt, "nt",     'i', &night);
  OptionAdd(&opt, "df",     'i', &dfrq);
  OptionAdd(&opt, "nf",     'i', &nfrq);
  OptionAdd(&opt, "xcf",    'x', &xcnt);
  OptionAdd(&opt, "ep",     'i', &errlog.port);
  OptionAdd(&opt, "sp",     'i', &shell.port);
  OptionAdd(&opt, "bp",     'i', &baseport);
  OptionAdd(&opt, "stid",   't', &ststr);
  OptionAdd(&opt, "sb",     'i', &sbm);
  OptionAdd(&opt, "eb",     'i', &ebm);
  OptionAdd(&opt, "fixfrq", 'i', &fixfrq);     /* fix the transmit frequency */
  OptionAdd(&opt, "frqrng", 'i', &frqrng);     /* fix the FCLR window [kHz] */
  OptionAdd(&opt, "-help",  'x', &hlp);        /* just dump some parameters */

  /* process the commandline; need this for setting errlog port */
  arg=OptionProcess(1,argc,argv,&opt,rst_opterr);

  if (arg==-1) {
    exit(-1);
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

  OpsStart(ststr);

  status=SiteBuild(ststr);
  if (status==-1) {
    fprintf(stderr,"Could not identify station.\n");
    exit(1);
  }

  SiteStart();

  /* reprocess the commandline since some things are reset by SiteStart */
  arg=OptionProcess(1,argc,argv,&opt,NULL);
  backward = (sbm > ebm) ? 1 : 0;   /* this almost certainly got reset */

  printf("Station ID: %s  %d\n",ststr,stid);
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

  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l dfrang l nfrang l"
                  " dmpinc l nmpinc l frqrng l xcnt l", &sbm,&ebm, &dfrq,&nfrq,
                  &dfrang,&nfrang, &dmpinc,&nmpinc, &frqrng,&xcnt);

  status=SiteSetupRadar();
  printf("Initial Setup Complete: Station ID: %s  %d\n",ststr,stid);

  if (status !=0) {
    ErrLog(errlog.sock,progname,"Error locating hardware.");
    exit (1);
  }

  beams=2*(abs(ebm-sbm)+1);

  /* Automatically calculate the integration times */
  /* Note that I have added a buffer here to account for things at the end
     of the scan. Traditionally this has been set to 3s, but I cannot find
     any justification of the need for it. -SGS */
  total_scan_usecs = scnsc*1e6 + scnus - (bufsc*1e6 + bufus);
  total_integration_usecs = total_scan_usecs/beams;
  intsc = total_integration_usecs/1e6;
  intus = total_integration_usecs - (intsc*1e6);

  if (discretion) cp = -cp;

  txpl=(rsep*20)/3;

  sprintf(progname,"ptab16flip");

  OpsLogStart(errlog.sock,progname,argc,argv);
  OpsSetupTask(tnum,task,errlog.sock,progname);

  for (n=0;n<tnum;n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen((char *)command),command);
  }

  printf("Preparing OpsFitACFStart Station ID: %s  %d\n",ststr,stid);
  OpsFitACFStart();

  printf("Entering Scan loop Station ID: %s  %d\n",ststr,stid);
  do {

    printf("Entering Site Start Scan Station ID: %s  %d\n",ststr,stid);
    if (SiteStartScan() !=0) continue;

    if (OpsReOpen(2,0,0) !=0) {
      ErrLog(errlog.sock,progname,"Opening new files.");
      for (n=0;n<tnum;n++) {
        RMsgSndClose(task[n].sock);
        RMsgSndOpen(task[n].sock,strlen( (char *) command),command);
      }
    }

    scan = 1;   /* scan flag */

    ErrLog(errlog.sock,progname,"Starting scan.");
    if (xcnt>0) {
      cnt++;
      if (cnt==xcnt) {
        xcf=1;
        cnt=0;
      } else xcf=0;
    } else xcf=0;

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

      if (flipflop==0) {
        mpinc=mpinc_short;
        mplgs=mplgs_short;
        mppul=mppul_short;
        tsgid=SiteTimeSeq(ptab_short);
      } else {
        mpinc=mpinc_long;
        mplgs=mplgs_long;
        mppul=mppul_long;
        tsgid=SiteTimeSeq(ptab_long);
      }

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus mppul:%d (%d:%d:%d:%d)",
                     bmnum,intsc,intus,mppul,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);

      ErrLog(errlog.sock,progname,"Starting Integration.");
      printf("Entering Site Start Intt Station ID: %s  %d\n",ststr,stid);
      SiteStartIntt(intsc,intus);

      /* clear frequency search business */
      ErrLog(errlog.sock,progname,"Doing clear frequency search.");
      sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
      ErrLog(errlog.sock,progname, logtxt);
      tfreq=SiteFCLR(stfrq,stfrq+frqrng);

      if ( (fixfrq > 8000) && (fixfrq < 25000) ) tfreq = fixfrq;

      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog.sock,progname,logtxt);

      if (flipflop==0) {
        nave=SiteIntegrate(lags_short);
      } else {
        nave=SiteIntegrate(lags_long);
      }
      if (nave < 0) {
        sprintf(logtxt,"Integration error:%d",nave);
        ErrLog(errlog.sock,progname,logtxt);
        continue;
      }
      sprintf(logtxt,"Number of sequences: %d",nave);
      ErrLog(errlog.sock,progname,logtxt);

      if (flipflop==0) {
        OpsBuildPrm(prm,ptab_short,lags_short);
      } else {
        OpsBuildPrm(prm,ptab_long,lags_long);
      }
      OpsBuildIQ(iq,&badtr);
      OpsBuildRaw(raw);

      FitACF(prm,raw,fblk,fit,site,tdiff,-999);
      FitSetAlgorithm(fit,"fitacf2");

      msg.num=0;
      msg.tsize=0;

      tmpbuf=RadarParmFlatten(prm,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf, PRM_TYPE,0);

      tmpbuf=IQFlatten(iq,prm->nave,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,IQ_TYPE,0);

      RMsgSndAdd(&msg,sizeof(unsigned int)*2*iq->tbadtr,
                 (unsigned char *) badtr,BADTR_TYPE,0);

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,(unsigned char *)sharedmemory,
                 IQS_TYPE,0);

      tmpbuf=RawFlatten(raw,prm->nrang,prm->mplgs,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,RAW_TYPE,0);

      tmpbuf=FitFlatten(fit,prm->nrang,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,FIT_TYPE,0);

      RMsgSndAdd(&msg,strlen(progname)+1,(unsigned char *)progname,
                 NME_TYPE,0);

      for (n=0;n<tnum;n++) RMsgSndSend(task[n].sock,&msg);

      for (n=0; n<msg.num; n++) {
        if ( (msg.data[n].type == PRM_TYPE) ||
             (msg.data[n].type == IQ_TYPE)  ||
             (msg.data[n].type == RAW_TYPE) ||
             (msg.data[n].type == FIT_TYPE) )  free(msg.ptr[n]);
      }

      RadarShell(shell.sock,&rstable);

      scan = 0;
      if (flipflop==0) {
        flipflop++;
      } else {
        flipflop=0;
        if (bmnum == ebm) break;

        if (backward) bmnum--;
        else bmnum++;
      }

    } while (1);

    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");
    SiteEndScan(scnsc,scnus,5000);

  } while (1);

  for (n=0; n<tnum; n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  return 0;
}


void usage(void)
{
    printf("\nptab16flip [command-line options]\n\n");
    printf("command-line options:\n");
    printf("    -di     : indicates running during discretionary time\n");
    printf("    -dt int : hour when day freq. is used [site.c]\n");
    printf("    -nt int : hour when night freq. is used [site.c]\n");
    printf("    -df int : daytime frequency (kHz) [site.c]\n");
    printf("    -nf int : nighttime frequency (kHz) [site.c]\n");
    printf("   -xcf     : set for computing XCFs [global.c]\n");
    printf("    -sb int : starting beam [site.c]\n");
    printf("    -eb int : ending beam [site.c]\n");
    printf("    -ep int : error log port (must be set here for dual radars)\n");
    printf("    -sp int : shell port (must be set here for dual radars)\n");
    printf("    -bp int : base port (must be set here for dual radars)\n");
    printf(" -stid char : radar string (must be set here for dual radars)\n");
    printf("-fixfrq int : transmit on fixed frequency (kHz)\n");
    printf("-frqrng int : set the clear frequency search window (kHz)\n");
    printf(" --help     : print this message and quit.\n");
    printf("\n");
}

