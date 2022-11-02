/* interleavesound_cv.c
 ======================
 Author: E.G.Thomas

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
#include "setup.h"
#include "sync.h"
#include "site.h"
#include "sitebuild.h"
#include "siteglobal.h"

#include "sndwrite.h"

#define MAX_SND_FREQS 12

void write_snd_record(char *progname, struct RadarParm *prm,
                      struct FitData *fit);

#define RT_TASK 3

char *ststr=NULL;
char *dfststr="tst";
char *libstr="ros";
void *tmpbuf;
size_t tmpsze;
char progid[80]={"interleavesound 2022/10/25"};
char progname[256];
int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: interleavesound --help\n");
  return(-1);
}

int main(int argc,char *argv[]) {

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

  int scnsc=60;
  int scnus=0;

  int skip;
  int cnt=0;
  int i,n;
  unsigned char discretion=0;
  int status=0;
  int fixfrq=0;

  int def_nrang=0;

  /* new variables for dynamically creating beam sequences */
  int *bms;           /* scanning beams                                     */
  int intgt[20];      /* start times of each integration period             */
  int nintgs=20;      /* number of integration periods per scan; SGS 1-min  */
  unsigned char hlp=0;
  unsigned char option=0;

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
  int fast_intt_sc=2;
  int fast_intt_us=400000;
  int snd_intt_sc=1;
  int snd_intt_us=500000;
  float snd_time, snd_intt, time_needed=1.25;

  char *path;

  snd_intt = snd_intt_sc + snd_intt_us*1e-6;
  /* ------------------------------------------------------- */


  /* standard radar defaults */
  cp     = 197;
  intsc  = fast_intt_sc;
  intus  = fast_intt_us;
  mppul  = 8;
  mplgs  = 23;
  mpinc  = 1500;
  dmpinc = 1500;
  nrang  = 100;
  rsep   = 45;
  txpl   = 300;     /* note: recomputed below */
  dfrq   = 10200;

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
  OptionAdd(&opt,"frqrng",'i',&frqrng);     /* fix the FCLR window [kHz] */
  OptionAdd(&opt,"sfrqrng",'i',&snd_frqrng); /* sounding FCLR window [kHz] */
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

  if (hlp) {
    usage();
    return (-1);
  }

  /* start time of each integration period */
  for (i=0; i<nintgs; i++)
    intgt[i] = i*(intsc + intus*1e-6);

  if (ststr==NULL) ststr=dfststr;

  /* Point to the beams here */
  if (strcmp(ststr,"cve") == 0) {
    bms = bmse;
    snd_bms = snd_bmse;
  } else if (strcmp(ststr,"cvw") == 0) {
    bms = bmsw;
    snd_bms = snd_bmsw;
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
  sprintf(progname,"interleavesound (fast)");
  for (i=0; i<nintgs; i++){
    sprintf(tempLog, "%3d", bms[i]);
    strcat(logtxt, tempLog);	
  }
  ErrLog(errlog.sock,progname,logtxt);

  OpsSetupCommand(argc,argv);
  OpsSetupShell();

  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l dfrang l nfrang l"
                  " dmpinc l nmpinc l frqrng l xcnt l", &sbm,&ebm, &dfrq,&nfrq,
                  &dfrang,&nfrang, &dmpinc,&nmpinc, &frqrng,&xcnt);

  status=SiteSetupRadar();

  fprintf(stderr,"Status:%d\n",status);

  if (status !=0) {
    ErrLog(errlog.sock,progname,"Error locating hardware.");
    exit (1);
  }

  def_nrang = nrang;

  if (discretion) cp = -cp;

  txpl=(rsep*20)/3;     /* computing TX pulse length */

  OpsLogStart(errlog.sock,progname,argc,argv);
  OpsSetupTask(tnum,task,errlog.sock,progname);

  for (n=0;n<tnum;n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock, strlen((char *)command), command);
  }

  OpsFitACFStart();

  do {

    tsgid=SiteTimeSeq(ptab);  /* get the timing sequence */

    if (SiteStartScan() !=0) continue;

    if (OpsReOpen(2,0,0) !=0) {
      ErrLog(errlog.sock,progname,"Opening new files.");
      for (n=0;n<tnum;n++) {
        RMsgSndClose(task[n].sock);
        RMsgSndOpen(task[n].sock,strlen( (char *) command),command);
      }
    }

    scan = 1;

    ErrLog(errlog.sock,progname,"Starting scan.");

    if (xcnt>0) {
      cnt++;
      if (cnt==xcnt) {
        xcf=1;
        cnt=0;
      } else xcf=0;
    } else xcf=0;

    skip = OpsFindSkip(scnsc,scnus,intsc,intus,0);

    bmnum = bms[skip];      /* no longer need forward and backward arrays... */

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

      scan = 0;
      if (skip == (nintgs-1)) break;
      skip++;
      bmnum = bms[skip];

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

    /* now wait for the next interleavescan */
    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");

    intsc = fast_intt_sc;
    intus = fast_intt_us;
    nrang = def_nrang;

    SiteEndScan(scnsc,scnus,5000);

  } while (1);

  for (n=0;n<tnum;n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  return 0;
}


void usage(void)
{
    printf("\ninterleavesound [command-line options]\n\n");
    printf("command-line options:\n");
    printf("     -di     : indicates running during discretionary time\n");
    printf("  -frang int : delay to first range (km) [180]\n");
    printf("   -rsep int : range separation (km) [45]\n");
    printf("     -dt int : hour when day freq. is used [site.c]\n");
    printf("     -nt int : hour when night freq. is used [site.c]\n");
    printf("     -df int : daytime frequency (kHz) [site.c]\n");
    printf("     -nf int : nighttime frequency (kHz) [site.c]\n");
    printf("    -xcf     : set for computing XCFs [global.c]\n");
    printf("  -nrang int : number or range gates [limit.h]\n");
    printf("     -ep int : error log port (must be set here for dual radars)\n");
    printf("     -sp int : shell port (must be set here for dual radars)\n");
    printf("     -bp int : base port (must be set here for dual radars)\n");
    printf("   -stid char: radar string (must be set here for dual radars)\n");
    printf(" -fixfrq int : transmit on fixed frequency (kHz)\n");
    printf(" -frqrng int : set the clear frequency search window (kHz)\n");
    printf("-sfrqrng int : set the sounding FCLR search window (kHz)\n");
    printf("   -ros char : change the roshost IP address\n");
    printf("  --help     : print this message and quit.\n");
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
    data_path[strlen(snd_dir)] = '/';
    data_path[strlen(snd_dir)+1] = 0;
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
