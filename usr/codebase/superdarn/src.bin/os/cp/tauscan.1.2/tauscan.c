/* tauscan.c
  ============
  Author: R.J.Barnes
   Based on algorithms developed by R.A. Greenwald and K.Oksavik
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
#include "rosmsg.h"
#include "tsg.h"

char *ststr=NULL;
char *dfststr="tst";
char *libstr="ros";

void *tmpbuf;
size_t tmpsze;

char progid[80]={"tauscan"};
char progname[256];

int arg=0;
struct OptionData opt;

char *roshost=NULL;
int tnum=4;

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: tauscan --help\n");
  return(-1);
}

int main(int argc,char *argv[]) {

  char logtxt[1024];

  int scannowait=0;

  int scnsc=120;
  int scnus=0;
  int skip;
  int cnt=0;

  unsigned char fast=0;
  unsigned char discretion=0;

  unsigned char option=0;
  unsigned char version=0;

  int status=0,n;

  int beams=0;
  int total_scan_usecs=0;
  int total_integration_usecs=0;

  struct sequence *seq;

  seq=OpsSequenceMake();
  OpsBuildTauscan(seq);

  /* standard radar parameters */
  cp=502;
  intsc=5;
  intus=500000;
  mppul=seq->mppul;
  mplgs=seq->mplgs;
  mplgexs=seq->mplgexs;
  mpinc=seq->mpinc;
  rsep=45;
  txpl=300;

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */

  OptionAdd(&opt, "di",     'x', &discretion);
  OptionAdd(&opt, "frang",  'i', &frang);
  OptionAdd(&opt, "rsep",   'i', &rsep);
  OptionAdd(&opt, "dt",     'i', &day);
  OptionAdd(&opt, "nt",     'i', &night);
  OptionAdd(&opt, "df",     'i', &dfrq);
  OptionAdd(&opt, "nf",     'i', &nfrq);
  OptionAdd(&opt, "xcf",    'i', &xcnt);
  OptionAdd(&opt, "ep",     'i', &errlog.port);
  OptionAdd(&opt, "sp",     'i', &shell.port);
  OptionAdd(&opt, "bp",     'i', &baseport);
  OptionAdd(&opt, "stid",   't', &ststr);
  OptionAdd(&opt, "fast",   'x', &fast);
  OptionAdd(&opt, "nowait", 'x', &scannowait);
  OptionAdd(&opt, "sb",     'i', &sbm);
  OptionAdd(&opt, "eb",     'i', &ebm);
  OptionAdd(&opt, "c",      'i', &cnum);
  OptionAdd(&opt, "ros",    't', &roshost);  /* Set the roshost IP address */
  OptionAdd(&opt, "debug",  'x', &debug);
  OptionAdd(&opt, "-option",'x', &option);
  OptionAdd(&opt,"-version",'x', &version);

  /* Process all of the command line options
   * Important: need to do this here because we need stid and ststr for
   *            calls to site specific programs */
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

  /* rst/usr/codebase/superdarn/src.lib/os/ops.1.10/src/setup.c */
  OpsStart(ststr);

  /* rst/usr/codebase/superdarn/src.lib/os/site.1.5/src/build.c */
  /* NOTE: the function just assigns the remaing functions starting with
   *       'Site' to the appropriate site specific functions, i.e.,
   *       SiteStart() is SiteRosStart(), etc. */
  status=SiteBuild(libstr);

  if (status==-1) {
    fprintf(stderr,"Could not load site library.\n");
    exit(1);
  }

  /* IMPORTANT: sbm and ebm are reset by this function */
  /* rst/usr/codebase/superdarn/src.lib/os/site.xxx.1.0/src/site.c */
  status = SiteStart(roshost,ststr);
  if (status==-1) {
    fprintf(stderr,"Error reading site configuration file.\n");
    exit(1);
  }

  /* Reprocess the command line to restore desired parameters */
  /* Important: need to do this for command line arguments to work */
  arg=OptionProcess(1,argc,argv,&opt,NULL);

  if (fast) sprintf(progname,"tauscan (fast)");
  else sprintf(progname,"tauscan");

  printf("Station ID: %s %d\n" ,ststr, stid);
  strncpy(combf,progid,80);

  if ((errlog.sock=TCPIPMsgOpen(errlog.host,errlog.port))==-1) {
    fprintf(stderr,"Error connecting to error log.\n");
  }

  if ((shell.sock=TCPIPMsgOpen(shell.host,shell.port))==-1) {
    fprintf(stderr,"Error connecting to shell.\n");
  }

  for (n=0;n<tnum;n++) task[n].port+=baseport;

  /* rst/usr/codebase/superdarn/src.lib/os/ops.1.10/src */
  OpsSetupCommand(argc,argv);
  OpsSetupShell();

  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l "
                           "frqrng l xcnt l",
                           &sbm,&ebm, &dfrq,&nfrq,
                           &frqrng,&xcnt);

  beams=abs(ebm-sbm)+1;
  if (fast) {
    cp=503;
    scnsc=60;
    scnus=0;
  } else {
    scnsc=120;
    scnus=0;
  }

  /* some trickery to get integration time from beams and total scan time */
  total_scan_usecs=(scnsc-3)*1E6+scnus;
  total_integration_usecs=total_scan_usecs/beams;
  intsc=total_integration_usecs/1E6;
  intus=total_integration_usecs-(intsc*1E6);

  OpsSetupIQBuf(intsc,intus,mppul,mpinc,nbaud);

  /* rst/usr/codebase/superdarn/src.lib/os/site.xxx.1.0/src/site.c */
  status=SiteSetupRadar();
  if (status !=0) {
    ErrLog(errlog.sock,progname,"Error locating hardware.");
    exit(1);
  }

  printf("Initial Setup Complete: Station ID: %s %d\n",ststr,stid);

  if (discretion) cp= -cp;

  txpl=(rsep*20)/3;

  OpsLogStart(errlog.sock,progname,argc,argv);
  OpsSetupTask(tnum,task,errlog.sock,progname);

  for (n=0;n<tnum;n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen( (char *) command),command);
  }

  printf("Preparing OpsFitACFStart Station ID: %s %d\n", ststr, stid);
  /* rst/usr/codebase/superdarn/src.lib/os/ops.1.10/src/setup.c */
  OpsFitACFStart();

  printf("Preparing SiteTimeSeq Station ID: %s %d\n",ststr,stid);
  /* rst/usr/codebase/superdarn/src.lib/os/site.xxx.1.0/src/site.c */
  tsgid=SiteTimeSeq(seq->ptab);

  /* rst/usr/codebase/superdarn/src.lib/os/ops.1.10/src/sync.c */
  skip=OpsFindSkip(scnsc,scnus,intsc,intus,0);

  if (backward) {
    bmnum=sbm-skip;
    if (bmnum<ebm) bmnum=sbm;
  } else {
    bmnum=sbm+skip;
    if (bmnum>ebm) bmnum=sbm;
  }

  printf("entering Scan Loop Station ID: %s %d\n",ststr, stid);
  do {

    printf("Entering Site Start Scan Station ID: %s %d\n",ststr,stid);
    /* rst/usr/codebase/superdarn/src.lib/os/site.xxx.1.0/src/site.c */
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

    if (xcnt>0) {
      cnt++;
      if (cnt==xcnt) {
        xcf=1;
        cnt=0;
      } else xcf=0;
    } else xcf=0;

    do {

      TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

      if (OpsDayNight()==1) {
        stfrq=dfrq;
      } else {
        stfrq=nfrq;
      }

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%02d:%02d:%02d:%06d)"
              " mpinc:%d", bmnum,intsc,intus,hr,mt,sc,us,mplgexs);
      ErrLog(errlog.sock,progname,logtxt);

      ErrLog(errlog.sock,progname,"Starting Integration.");
      printf("Entering Site Start Intt Station ID: %s %d\n",ststr,stid);
      /* rst/usr/codebase/superdarn/src.lib/os/site.xxx.1.0/src/site.c */
      SiteStartIntt(intsc,intus);

      printf("Entering Site FCLR Station ID: %s %d\n", ststr, stid);
      ErrLog(errlog.sock,progname,"Doing clear frequency search.");

      sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
      ErrLog(errlog.sock,progname, logtxt);

      /* rst/usr/codebase/superdarn/src.lib/os/site.xxx.1.0/src/site.c */
      tfreq=SiteFCLR(stfrq,stfrq+frqrng);

      sprintf(logtxt,"Transmitting on: %d (Noise=%g)",tfreq,noise);
      ErrLog(errlog.sock,progname,logtxt);

      printf("Entering Site Integrate Station ID: %s %d \n", ststr, stid);
      /* rst/usr/codebase/superdarn/src.lib/os/site.xxx.1.0/src/site.c */
      nave=SiteIntegrate(seq->lags);
      if (nave<0) {
        sprintf(logtxt,"Integration error:%d",nave);
        ErrLog(errlog.sock,progname,logtxt);
        continue;
      }
      sprintf(logtxt,"Number of sequences: %d",nave);
      ErrLog(errlog.sock,progname,logtxt);

      /* rst/usr/codebase/superdarn/src.lib/os/ops.1.10/src/build.c */
      OpsBuildPrm(prm,seq->ptab,seq->lags);
      OpsBuildIQ(iq,&badtr);
      OpsBuildRaw(raw);

      /* rst/codebase/superdarn/src.lib/tk/fitacf.2.5/src/fitacf.c */
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

      RMsgSndAdd(&msg,strlen(sharedmemory)+1,(unsigned char *)sharedmemory,
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
      if (bmnum==ebm) break;
      if (backward) bmnum--;
      else bmnum++;

    } while (1);

    bmnum = sbm;
    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");
    /* rst/usr/codebase/superdarn/src.lib/os/site.xxx.1.0/src/site.c */
    if (scannowait==0) SiteEndScan(scnsc,scnus,5000);
  } while (1);

  for (n=0;n<tnum;n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  /* rst/usr/codebase/superdarn/src.lib/os/site.xxx.1.0/src/site.c */
  SiteExit(0);

  return 0;
}

