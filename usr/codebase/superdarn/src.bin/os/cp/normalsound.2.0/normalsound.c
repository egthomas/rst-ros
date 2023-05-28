/* normalsound.c
   =============
   Author: E.G.Thomas

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
#include "snddata.h"
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
#include "snd.h"
#include "sync.h"
#include "site.h"
#include "sitebuild.h"
#include "siteglobal.h"
#include "rosmsg.h"
#include "tsg.h"

#define RT_TASK 3


char *ststr=NULL;
char *dfststr="tst";
char *libstr="ros";

void *tmpbuf;
size_t tmpsze;

char progid[80]={"normalsound 2023/04/05"};
char progname[256];

int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: normalsound --help\n");
  return(-1);
}

int main(int argc,char *argv[])
{

  char logtxt[1024];

  int scnsc=120;    /* total scan period in seconds */
  int scnus=0;
  int skip;
  int cnt=0;

  unsigned char fast=1;
  unsigned char slow=0;
  unsigned char discretion=0;
  int fixfrq=0;

  int i,n;
  int status=0;

  int beams=0;
  int total_scan_usecs=0;
  int total_integration_usecs=0;
  int def_intt_sc=0;
  int def_intt_us=0;
  int def_nrang=0;

  unsigned char hlp=0;
  unsigned char option=0;
  unsigned char version=0;


  /* ---------------- Variables for sounding --------------- */
  int *snd_bms;
  int snd_bmse[]={0,2,4,6,8,10,12,14,16,18};   /* beam sequences for 24-beam MSI radars using only */
  int snd_bmsw[]={22,20,18,16,14,12,10,8,6,4}; /*  the 20 most meridional beams */
  int snd_freq_cnt=0, snd_bm_cnt=0;
  int snd_bms_tot=10, odd_beams=0;
  int snd_freq;
  int snd_frqrng=100;
  int snd_nrang=75;
  int snd_sc=12;
  int snd_intt_sc=1;
  int snd_intt_us=500000;
  float snd_time, snd_intt, time_needed=1.25;

  snd_intt = snd_intt_sc + snd_intt_us*1e-6;
  /* ------------------------------------------------------- */

  struct sequence *seq;

  seq=OpsSequenceMake();
  OpsBuild8pulse(seq);

  cp     = 155;         /* CPID */
  intsc  = 7;           /* integration period; recomputed below ... */
  intus  = 0;
  mppul  = seq->mppul;  /* number of pulses */
  mplgs  = seq->mplgs;  /* number of lags */
  mpinc  = seq->mpinc;  /* multi-pulse increment [us] */
  rsep   = 45;          /* same for the range separation */
  txpl   = 300;         /* pulse length [us]; gets redefined below... */

  dmpinc = nmpinc = mpinc;  /* set day and night to the same,
                               but could change */

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
  OptionAdd(&opt, "slow",   'x', &slow);
  OptionAdd(&opt, "sb",     'i', &sbm);
  OptionAdd(&opt, "eb",     'i', &ebm);
  OptionAdd(&opt, "fixfrq", 'i', &fixfrq);     /* fix the transmit frequency */
  OptionAdd(&opt, "frqrng", 'i', &frqrng);     /* fix the FCLR window [kHz] */
  OptionAdd(&opt, "sfrqrng",'i', &snd_frqrng); /* sounding FCLR window [kHz] */
  OptionAdd(&opt, "sndsc",  'i', &snd_sc);     /* sounding duration per scan [sec] */
  OptionAdd(&opt, "c"  ,    'i', &cnum);
  OptionAdd(&opt, "ros",    't', &roshost);    /* Set the roshost IP address */
  OptionAdd(&opt, "debug",  'x', &debug);
  OptionAdd(&opt, "-help",  'x', &hlp);        /* just dump some parameters */
  OptionAdd(&opt, "-option",'x', &option);
  OptionAdd(&opt,"-version",'x',&version);

  /* process the commandline; need this for setting errlog port */
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

  /* Point to the beams here */
  if ((strcmp(ststr,"cve") == 0) || (strcmp(ststr,"ice") == 0) || (strcmp(ststr,"fhe") == 0)) {
    snd_bms = snd_bmse;
  } else if ((strcmp(ststr,"cvw") == 0) || (strcmp(ststr,"icw") == 0)) {
    snd_bms = snd_bmsw;
  } else if (strcmp(ststr,"fhw") == 0) {
    snd_bms = snd_bmsw;
    for (i=0; i<snd_bms_tot; i++)
      snd_bms[i] -= 2;
  } else {
    printf("Error: Not intended for station %s\n", ststr);
    return (-1);
  }

  /* Load the sounding frequencies */
  OpsLoadSndFreqs(ststr);

  OpsStart(ststr);

  status=SiteBuild(libstr);
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
  arg=OptionProcess(1,argc,argv,&opt,NULL);
  backward = (sbm > ebm) ? 1 : 0;   /* this almost certainly got reset */

  if (fast) sprintf(progname,"normalsound (fast)");
  else sprintf(progname,"normalsound");

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
  if (status !=0) {
    ErrLog(errlog.sock,progname,"Error locating hardware.");
    exit(1);
  }

  printf("Initial Setup Complete: Station ID: %s  %d\n",ststr,stid);

  if (slow) fast = 0;

  beams=abs(ebm-sbm)+1;
  if (fast) {
    cp    = 157;
    scnsc = 60;
    scnus = 0;
  } else {
    scnsc = 120;
    scnus = 0;
  }

  /* Automatically calculate the integration times */
  total_scan_usecs = (scnsc-snd_sc)*1E6 + scnus;
  total_integration_usecs = total_scan_usecs/beams;
  def_intt_sc = total_integration_usecs/1E6;
  def_intt_us = total_integration_usecs - (def_intt_sc*1e6);

  def_nrang = nrang;

  intsc = def_intt_sc;
  intus = def_intt_us;

  if (discretion) cp = -cp;

  txpl=(rsep*20)/3;

  OpsLogStart(errlog.sock,progname,argc,argv);
  OpsSetupTask(tnum,task,errlog.sock,progname);

  for (n=0;n<tnum;n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen((char *)command),command);
  }

  printf("Preparing OpsFitACFStart Station ID: %s  %d\n",ststr,stid);
  OpsFitACFStart();

  printf("Preparing OpsSndStart Station ID: %s %d\n",ststr,stid);
  OpsSndStart();

  OpsFindSndSkip(ststr,snd_bms,snd_bms_tot,&snd_bm_cnt,&odd_beams);

  printf("Entering Scan loop Station ID: %s  %d\n",ststr,stid);
  do {

    if (def_nrang != snd_nrang) {
      printf("Preparing SiteTimeSeq Station ID: %s  %d\n",ststr,stid);
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
        continue;
      }
    }

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
        mpinc=dmpinc;
        frang=dfrang;
      } else {
        stfrq=nfrq;
        mpinc=nmpinc;
        frang=nfrang;
      }

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%d:%d:%d:%d)",
                     bmnum,intsc,intus,hr,mt,sc,us);
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

      nave=SiteIntegrate(seq->lags);
      if (nave < 0) {
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
      if (bmnum == ebm) break;

      if (backward) bmnum--;
      else bmnum++;

    } while (1);


    /* In here comes the sounder code */
    /* set the "sounder mode" scan variable */
    scan = -2;

    /* set the xcf variable to do cross-correlations (AOA) */
    if (xcnt > 1) xcf = 1;

    /* set the sounding mode integration time and number of ranges */
    intsc = snd_intt_sc;
    intus = snd_intt_us;
    nrang = snd_nrang;

    /* make a new timing sequence for the sounding */
    if (def_nrang != snd_nrang) {
      tsgid = SiteTimeSeq(seq->ptab);
      if (tsgid !=0) {
        if (tsgid==-2) {
          ErrLog(errlog.sock,progname,"Error registering SND timing sequence.");
        } else if (tsgid==-1) {
          ErrLog(errlog.sock,progname,"SND TSGMake error code: 0 (tsgbuff==NULL)");
        } else {
          sprintf(logtxt,"SND TSGMake error code: %d",tsgid);
          ErrLog(errlog.sock,progname,logtxt);
        }
        continue;
      }
    }

    /* we have time until the end of the minute to do sounding */
    /* minus a safety factor given in time_needed */
    TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);
    snd_time = 60.0 - (sc + us*1e-6);

    while (snd_time-snd_intt > time_needed) {

      /* set the beam */
      bmnum = snd_bms[snd_bm_cnt] + odd_beams;

      /* snd_freq will be an array of frequencies to step through */
      snd_freq = snd_freqs[snd_freq_cnt];

      /* the scanning code is here */
      sprintf(logtxt,"Integrating SND beam:%d intt:%ds.%dus (%d:%d:%d:%d)",bmnum,intsc,intus,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);
      ErrLog(errlog.sock,progname,"Setting SND beam.");
      SiteStartIntt(intsc,intus);

      ErrLog(errlog.sock, progname, "Doing SND clear frequency search.");
      sprintf(logtxt, "FRQ: %d %d", snd_freq, snd_frqrng);
      ErrLog(errlog.sock,progname, logtxt);
      tfreq = SiteFCLR(snd_freq, snd_freq + snd_frqrng);

      sprintf(logtxt,"Transmitting SND on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog.sock, progname, logtxt);

      nave = SiteIntegrate(seq->lags);
      if (nave < 0) {
        sprintf(logtxt, "SND integration error: %d", nave);
        ErrLog(errlog.sock,progname, logtxt);
        continue;
      }
      sprintf(logtxt,"Number of SND sequences: %d",nave);
      ErrLog(errlog.sock,progname,logtxt);

      OpsBuildPrm(prm,seq->ptab,seq->lags);
      OpsBuildIQ(iq,&badtr);
      OpsBuildRaw(raw);
      FitACF(prm,raw,fblk,fit,site,tdiff,-999);

      ErrLog(errlog.sock, progname, "Sending SND messages.");
      msg.num = 0;
      msg.tsize = 0;

      tmpbuf=RadarParmFlatten(prm,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,PRM_TYPE,0);

      tmpbuf=FitFlatten(fit,prm->nrang,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,FIT_TYPE,0);

      RMsgSndSend(task[RT_TASK].sock,&msg);
      for (n=0;n<msg.num;n++) {
        if (msg.data[n].type==PRM_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==IQ_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==RAW_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==FIT_TYPE) free(msg.ptr[n]);
      }

      sprintf(logtxt, "SBC: %d  SFC: %d", snd_bm_cnt, snd_freq_cnt);
      ErrLog(errlog.sock, progname, logtxt);

      /* set the scan variable for the sounding mode data file only */
      if ((bmnum == snd_bms[0]) && (snd_freq == snd_freqs[0])) {
        prm->scan = 1;
      } else {
        prm->scan = 0;
      }

      OpsBuildSnd(prm,fit);

      /* save the sounding mode data */
      OpsWriteSnd(errlog.sock, progname, snd, ststr);

      ErrLog(errlog.sock, progname, "Polling SND for exit.\n");

      /* check for the end of a beam loop */
      snd_freq_cnt++;
      if (snd_freq_cnt >= snd_freqs_tot) {
        /* reset the freq counter and increment the beam counter */
        snd_freq_cnt = 0;
        snd_bm_cnt++;
        if (snd_bm_cnt >= snd_bms_tot) {
          snd_bm_cnt = 0;
          odd_beams = !odd_beams;
        }
      }

      /* see if we have enough time for another go round */
      TimeReadClock(&yr, &mo, &dy, &hr, &mt, &sc, &us);
      snd_time = 60.0 - (sc + us*1e-6);
    }

    /* now wait for the next normalscan */
    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");

    intsc = def_intt_sc;
    intus = def_intt_us;
    nrang = def_nrang;

    SiteEndScan(scnsc,scnus,5000);

  } while (1);

  for (n=0; n<tnum; n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  return 0;
}


void usage(void)
{
    printf("\nnormalsound [command-line options]\n\n");
    printf("command-line options:\n");
    printf("    -di     : indicates running during discretionary time\n");
    printf(" -frang int : delay to first range (km) [180]\n");
    printf("  -rsep int : range separation (km) [45]\n");
    printf(" -nrang int : number of range gates [100]\n");
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
    printf("-sfrqrng int: set the sounding FCLR search window (kHz)\n");
    printf(" -sndsc int : set the sounding duration per scan (sec)\n");
    printf("     -c int : channel number for multi-channel radars.\n");
    printf("   -ros char: change the roshost IP address\n");
    printf(" --help     : print this message and quit.\n");
    printf("\n");
}

