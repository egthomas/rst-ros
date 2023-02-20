/* testsound.c
   ===========
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
#include "rawwrite.h"
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

#include "sndwrite.h"

#define MAX_SND_FREQS 12

void write_snd_record(char *progname, struct RadarParm *prm,
                      struct FitData *fit);

void write_raw_snd_record(char *progname, struct RadarParm *prm,
                          struct RawData *raw);

#define RT_TASK 3


char *ststr=NULL;
char *dfststr="tst";
char *libstr="ros";

void *tmpbuf;
size_t tmpsze;

char progid[80]={"testsound 2022/02/18"};
char progname[256];

int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: testsound --help\n");
  return(-1);
}

int main(int argc,char *argv[])
{

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

  if (debug) {
    printf("Size of int %lu\n",sizeof(int));
    printf("Size of long %lu\n",sizeof(long));
    printf("Size of long long %lu\n",sizeof(long long));
    printf("Size of struct TRTimes %lu\n",sizeof(struct TRTimes));
    printf("Size of struct SeqPRM %lu\n",sizeof(struct SeqPRM));
    printf("Size of struct RosData %lu\n",sizeof(struct RosData));
    printf("Size of struct DataPRM %lu\n",sizeof(struct DataPRM));
    printf("Size of Struct ControlPRM  %lu\n",sizeof(struct ControlPRM));
    printf("Size of Struct RadarPRM  %lu\n",sizeof(struct RadarPRM));
    printf("Size of Struct ROSMsg  %lu\n",sizeof(struct ROSMsg));
    printf("Size of Struct CLRFreq  %lu\n",sizeof(struct CLRFreqPRM));
    printf("Size of Struct TSGprm  %lu\n",sizeof(struct TSGprm));
    printf("Size of Struct SiteSettings  %lu\n",sizeof(struct SiteSettings));
  }


  /* ---------------- Variables for sounding --------------- */
  char snd_filename[100];
  FILE *snd_dat;
  /* If the file $SD_SND_PATH/sounder_[rad].dat exists, the next two parameters are read from it */
  /* the file contains one integer value per line */
  int snd_freqs_tot=8;
  int snd_freqs[MAX_SND_FREQS]= {11000, 12000, 13000, 14000, 15000, 16000, 17000, 18000, 0, 0, 0, 0 };
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

  char *path;

  snd_intt = snd_intt_sc + snd_intt_us*1e-6;
  /* ------------------------------------------------------- */


  cp     = 1100;    /* CPID */
  intsc  = 7;       /* integration period; recomputed below ... */
  intus  = 0;
  mppul  = 8;       /* number of pulses; tied to array above ... */
  mplgs  = 23;      /* same here for the number of lags */
  mpinc  = 1500;    /* multi-pulse increment [us] */
  nrang  = 100;     /* the number of ranges gets set in SiteXXXStart() */
  rsep   = 45;      /* same for the range separation */
  txpl   = 300;     /* pulse length [us]; gets redefined below... */

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
  OptionAdd(&opt,"-version",'x', &version);

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

  /* load the sounder frequencies from file in site directory if present */
  path = getenv("SD_SITE_PATH");
  if (path == NULL) {
    fprintf(stderr,"Environment variable 'SD_SITE_PATH' not defined.\n");
  }

  sprintf(snd_filename,"%s/site.%s/sounder_%s.dat", path, ststr, ststr);
  fprintf(stderr,"Checking Sounder File: %s\n",snd_filename);
  snd_dat = fopen(snd_filename, "r");
  if (snd_dat != NULL) {
    fscanf(snd_dat, "%d", &snd_freqs_tot);
    if (snd_freqs_tot > 12) snd_freqs_tot = 12;
    for (snd_freq_cnt=0; snd_freq_cnt < snd_freqs_tot; snd_freq_cnt++)
      fscanf(snd_dat, "%d", &snd_freqs[snd_freq_cnt]);
    snd_freq_cnt = 0;
    fclose(snd_dat);
    fprintf(stderr,"Sounder File: %s read\n",snd_filename);
  } else {
    fprintf(stderr,"Sounder File: %s not found\n",snd_filename);
  }

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
    cp    = 1101;
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

  if (fast) sprintf(progname,"testsound (fast)");
  else sprintf(progname,"testsound");

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

    printf("Preparing SiteTimeSeq Station ID: %s  %d\n",ststr,stid);
    tsgid=SiteTimeSeq(ptab);

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

      nave=SiteIntegrate(lags);
      if (nave < 0) {
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
    xcf = 1;

    /* set the sounding mode integration time and number of ranges */
    intsc = snd_intt_sc;
    intus = snd_intt_us;
    nrang = snd_nrang;

    /* make a new timing sequence for the sounding */
    tsgid = SiteTimeSeq(ptab);

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

      nave = SiteIntegrate(lags);
      if (nave < 0) {
        sprintf(logtxt, "SND integration error: %d", nave);
        ErrLog(errlog.sock,progname, logtxt);
        continue;
      }
      sprintf(logtxt,"Number of SND sequences: %d",nave);
      ErrLog(errlog.sock,progname,logtxt);

      OpsBuildPrm(prm,ptab,lags);
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

      /* save the sounding mode data */
      write_snd_record(progname, prm, fit);
      write_raw_snd_record(progname, prm, raw);

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
    printf("\ntestsound [command-line options]\n\n");
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


/********************** function write_snd_record() ************************/
/* changed the output to dmap format */

void write_snd_record(char *progname, struct RadarParm *prm, struct FitData *fit) {

  char data_path[100], data_filename[50], filename[80];

  char *snd_dir;
  FILE *out;

  char logtxt[1024]="";
  int status;

  /* set up the data directory */
  /* get the snd data dir */
  snd_dir = getenv("SD_SND_PATH");
  if (snd_dir == NULL)
    sprintf(data_path,"/data/ros/snd/");
  else {
    memcpy(data_path,snd_dir,strlen(snd_dir));
    data_path[strlen(snd_dir)] = 0;
  }

  /* make up the filename */
  /* YYYYMMDD.HH.rad.snd */
  sprintf(data_filename, "%04d%02d%02d.%02d.%s", prm->time.yr, prm->time.mo, prm->time.dy, (prm->time.hr/ 2)* 2, ststr);

  /* finally make the filename */
  sprintf(filename, "%s%s.snd", data_path, data_filename);

  /* open the output file */
  fprintf(stderr,"Sounding Data File: %s\n",filename);
  out = fopen(filename,"a");
  if (out == NULL) {
    /* crap. might as well go home */
    sprintf(logtxt,"Unable to open sounding file:%s",filename);
    ErrLog(errlog.sock,progname,logtxt);
    return;
  }

  /* write the sounding record */
  status = SndFwrite(out, prm, fit);
  if (status == -1) {
    ErrLog(errlog.sock,progname,"Error writing sounding record.");
  } else {
    ErrLog(errlog.sock,progname,"Sounding record successfully written.");
  }

  fclose(out);
}


/********************** function write_raw_snd_record() ************************/
/* changed the output to rawacf format */

void write_raw_snd_record(char *progname, struct RadarParm *prm, struct RawData *raw) {

  char data_path[100], data_filename[50], filename[80];

  char *snd_dir;
  FILE *out;

  char logtxt[1024]="";
  int status;

  /* set up the data directory */
  /* get the snd data dir */
  snd_dir = getenv("SD_SND_PATH");
  if (snd_dir == NULL)
    sprintf(data_path,"/data/ros/snd/");
  else {
    memcpy(data_path,snd_dir,strlen(snd_dir));
    data_path[strlen(snd_dir)] = '/';
    data_path[strlen(snd_dir)+1] = 0;
  }

  /* make up the filename */
  /* YYYYMMDD.HH.rad.snd */
  sprintf(data_filename, "%04d%02d%02d.%02d.%s", prm->time.yr, prm->time.mo, prm->time.dy, (prm->time.hr/ 2)* 2, ststr);

  /* finally make the filename */
  sprintf(filename, "%s%s.snd.rawacf", data_path, data_filename);

  /* open the output file */
  fprintf(stderr,"Sounding Data File: %s\n",filename);
  out = fopen(filename,"a");
  if (out == NULL) {
    /* crap. might as well go home */
    sprintf(logtxt,"Unable to open sounding file:%s",filename);
    ErrLog(errlog.sock,progname,logtxt);
    return;
  }

  /* write the sounding record */
  status = RawFwrite(out, prm, raw);
  if (status == -1) {
    ErrLog(errlog.sock,progname,"Error writing rawacf sounding record.");
  } else {
    ErrLog(errlog.sock,progname,"Rawacf sounding record successfully written.");
  }

  fclose(out);
}
