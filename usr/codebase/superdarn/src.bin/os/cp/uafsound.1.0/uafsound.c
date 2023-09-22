/* uafsound.c
   ============
   Author: R.J.Barnes & J.Spaleta & J.Klein & E.G.Thomas
*/

/*
 LICENSE AND DISCLAIMER
 
 Copyright (c) 2012 The Johns Hopkins University/Applied Physics Laboratory
 
 This file is part of the Radar Software Toolkit (RST).
 
 RST is free software: you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version.

 RST is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public License
 along with RST.  If not, see <http://www.gnu.org/licenses/>.
 
 
  
*/
/* Includes provided by the OS environment */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <zlib.h>
#include <math.h>

/* Includes provided by the RST */ 
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
#include "tcpipmsg.h"
#include "rmsg.h"
#include "rmsgsnd.h"
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

/* sorry, included for checking sanity checking pcode sequences with --test (JTK)*/
#include "tsg.h" 
#include "maketsg.h"

#define MAX_INTEGRATIONS_PER_SCAN 100

#define RT_TASK 3

int arg=0;
struct OptionData opt;

void usage(void);

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: uafsound --help\n");
  return(-1);
}


int main(int argc,char *argv[]) {
  char progid[80]={"uafsound 2023/09/18"};
  char progname[256]="uafsound";
  char modestr[32];

  char *roshost=NULL;

  char *ststr=NULL;
  char *dfststr="tst";

  char *libstr="ros";

  unsigned char help=0;
  unsigned char option=0;
  unsigned char version=0;
  unsigned char test=0;

  unsigned char fast=0;
  unsigned char discretion=0;

  int status=0,n,i;
  int clrscan=0;
  int cpid=0;

  /* Variables need for interprocess communications */
  char logtxt[1024];
  void *tmpbuf;
  size_t tmpsze;
/*
  struct TCPIPMsgHost shell={"127.0.0.1",44101,-1};
*/
  int tnum=4;      

  /* lists for parameters across a scan, need to send to usrp_server for swings to work.. */
  int32_t scan_clrfreq_bandwidth_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t scan_clrfreq_fstart_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t scan_beam_number_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t nBeams_per_scan = 0;
  int current_beam, iBeam;

  /* lists for parameters across a sounding scan, need to send to usrp_server for swings to work.. */
  int32_t snd_clrfreq_bandwidth_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_clrfreq_fstart_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_beam_number_list[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_bc[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_fc[MAX_INTEGRATIONS_PER_SCAN];
  int32_t snd_nBeams_per_scan = 0;
  int snd_iBeam;

  /* time sync of integration periods/ beams */
  int sync_scan;
  int time_now,  time_to_wait; /* times in ms for period synchronization */
  int *scan_times;  /* scan times in ms */

/* Integration period variables */
  int scnsc=120;
  int scnus=0;
  int total_scan_usecs=0;
  int total_integration_usecs=0;

  /* Variables for possibly different normal vs snd nrang/rsep */
  int def_intt_sc=0;
  int def_intt_us=0;
  int def_nrang=0;
  int def_rsep=0;
  int def_txpl=0;

  /* Variables for controlling clear frequency search */
  struct timeval t0,t1;
  int elapsed_secs=0;
  int clrskip=-1;
  int default_clrskip_secs=30;
  int startup=1;
  int fixfrq=-1;

  /* XCF processing variables */
  int cnt=0;

  /* ---------------- Variables for sounding --------------- */
  int *snd_bms;
  int snd_bmse[]={0,2,4,6,8,10,12,14,16,18};   /* beam sequences for 24-beam MSI radars using only */
  int snd_bmsw[]={22,20,18,16,14,12,10,8,6,4}; /*  the 20 most meridional beams */
  int snd_freq_cnt=0, snd_bm_cnt=0;
  int snd_bms_tot=10, odd_beams=0;
  int snd_freq;
  int snd_frqrng=100;
  int snd_nrang=75;
  int snd_rsep=45;
  int snd_txpl=300;
  int snd_sc=-1;
  int snd_intt_sc=1;
  int snd_intt_us=500000;
  float snd_time, snd_intt, time_needed=0.1;
  /* ------------------------------------------------------- */

  struct sequence *seq;

  seq=OpsSequenceMake();
  OpsBuild8pulse(seq);

/* Set default values of globally defined variables here*/
  cp     = 9000;    /*unused Alaska cpid, will be reset below  */
  intsc  = 7;
  intus  = 0;
  mppul  = seq->mppul;
  mplgs  = seq->mplgs;
  mpinc  = seq->mpinc;
  rsep   = 45;
  txpl   = 300;
  nbaud  = 1;

  /* ========= PROCESS COMMAND LINE ARGUMENTS ============= */

  OptionAdd(&opt, "-help", 'x', &help);
  OptionAdd(&opt, "-option", 'x', &option);
  OptionAdd(&opt, "-version", 'x', &version);

  OptionAdd(&opt, "debug", 'x', &debug);
  OptionAdd(&opt, "test", 'x', &test);
  OptionAdd(&opt, "di", 'x', &discretion);
  OptionAdd(&opt, "fast", 'x', &fast);
  OptionAdd(&opt, "clrscan", 'x', &clrscan);

  OptionAdd(&opt, "tau", 'i',  &mpinc);
  OptionAdd(&opt, "nrang", 'i', &nrang);
  OptionAdd(&opt, "frang", 'i', &frang);
  OptionAdd(&opt, "rsep", 'i', &rsep);
  OptionAdd(&opt, "dt", 'i', &day);
  OptionAdd(&opt, "nt", 'i', &night);
  OptionAdd(&opt, "df", 'i', &dfrq);
  OptionAdd(&opt, "nf", 'i', &nfrq);
  OptionAdd(&opt, "fixfrq", 'i', &fixfrq);
  OptionAdd(&opt, "xcf", 'i', &xcnt);
  OptionAdd(&opt, "ep", 'i', &errlog.port);
  OptionAdd(&opt, "sp", 'i', &shell.port);
  OptionAdd(&opt, "bp", 'i', &baseport);
  OptionAdd(&opt, "sb", 'i', &sbm);
  OptionAdd(&opt, "eb", 'i', &ebm);
  OptionAdd(&opt, "c",'i',&cnum);
  OptionAdd(&opt, "clrskip", 'i', &clrskip);
  OptionAdd(&opt, "cpid", 'i', &cpid);

  OptionAdd(&opt, "ros", 't', &roshost);
  OptionAdd(&opt, "stid", 't', &ststr);
  OptionAdd(&opt, "lib", 't', &libstr);

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

  /* print help */
  if(help) {
    printf("UAFSOUND: : An attempt to get frequency sounding modes working for UAF USRP-style radars \n\n");
    usage();
    return 0;
  }

  /* Load site library argument here */
  if (ststr==NULL) ststr=dfststr;

  channel = cnum;

  /* Point to the beams here */
  if ((strcmp(ststr,"cve") == 0) || (strcmp(ststr,"ice") == 0) || (strcmp(ststr,"fhe") == 0)) {
    snd_bms = snd_bmse;
  } else if ((strcmp(ststr,"cvw") == 0) || (strcmp(ststr,"icw") == 0) || (strcmp(ststr,"bks") == 0)) {
    snd_bms = snd_bmsw;
  } else if (strcmp(ststr,"fhw") == 0) {
    snd_bms = snd_bmsw;
    for (i=0; i<snd_bms_tot; i++)
      snd_bms[i] -= 2;
  } else {
    snd_bms = snd_bmse;
    snd_bms_tot = 8;
    snd_intt_sc = 2;
    snd_intt_us = 0;
  }

  snd_intt = snd_intt_sc + snd_intt_us*1e-6;

  printf("Requested :: ststr: %s libstr: %s\n",ststr,libstr);
/* This loads Radar Site information from hdw.dat files */
  OpsStart(ststr);

  /* Load the sounding frequencies */
  OpsLoadSndFreqs(ststr);

/* This loads Site library via dlopen and maps: site library specific functions into Site name space */
  status=SiteBuild(libstr);
  if (status==-1) {
    fprintf(stderr,"Could not load requested site library\n");
    exit(1);
  }


/* Run SiteStart library function to load Site specific default values for global variables
 * This should be run before all options are parsed and before any task sockets are opened
 * arguments: host ip address, 3-letter station string
*/
  
  status=SiteStart(roshost,ststr);
  if (status==-1) {
    fprintf(stderr,"SiteStart failure\n");
    exit(1);
  }

/* load any provided argument values overriding default values provided by SiteStart */ 
  arg = OptionProcess(1,argc,argv,&opt,NULL);

 /* ========= SET PARAMETER TO EMULATE OTHER CONTROL PROGRAMS ============= */

  /* NORMAL beam order */
  fprintf(stderr, "Initializing normal beam pattern...\n");
  nBeams_per_scan = abs(ebm-sbm)+1;
  current_beam = sbm;

  /* defaults for normalsound, adjusted   */  
  cp=155;
  scnsc = 120;
  scnus = 0;
  snd_sc = 20;

  sync_scan = 0; 

  /* FAST option */  
  if (fast) {     /* If fast option selected use 1 minute scan boundaries */
    cp    = 157;
    scnsc = 60;
    scnus = 0;
    snd_sc = 12;
    sprintf(modestr," (fast)");
    strncat(progname,modestr,sizeof(progname)-strlen(progname)-1);
  } 

  for (iBeam =0; iBeam < nBeams_per_scan; iBeam++){
     scan_beam_number_list[iBeam] = current_beam;
     current_beam += backward ? -1:1;
  }

  snd_nBeams_per_scan = (snd_sc-time_needed)/(snd_intt_sc + snd_intt_us*1e-6);

  /* Print out details of beams */ 
  fprintf(stderr, "Sequence details: \n");
  for (iBeam =0; iBeam < nBeams_per_scan; iBeam++){
    fprintf(stderr, "  sequence %2d: beam: %2d, \n",iBeam, scan_beam_number_list[iBeam] );
  }



/* Open Connection to errorlog */  
  if ((errlog.sock=TCPIPMsgOpen(errlog.host,errlog.port))==-1) {    
    fprintf(stderr,"Error connecting to error log.\n Host: %s  Port: %d\n",errlog.host,errlog.port);
  }
/* Open Connection to radar shell */  
  if ((shell.sock=TCPIPMsgOpen(shell.host,shell.port))==-1) {    
    fprintf(stderr,"Error connecting to shell.\n");
  }

/* Open Connection to helper utilities like fitacfwrite*/  
  for (n=0;n<tnum;n++) task[n].port+=baseport;

/* Prep command string for tasks */ 
  strncpy(combf,progid,80);   
  OpsSetupCommand(argc,argv);
  OpsLogStart(errlog.sock,progname,argc,argv);  
  OpsSetupTask(tnum,task,errlog.sock,progname);
  for (n=0;n<tnum;n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen( (char *) command),command);     
  }


  /* Initialize timing variables */
  elapsed_secs=0;
  gettimeofday(&t1,NULL);
  gettimeofday(&t0,NULL);
  
  /* Automatically calculate the integration times */
  total_scan_usecs = (scnsc-snd_sc)*1E6 + scnus;
  total_integration_usecs = total_scan_usecs/nBeams_per_scan;
  def_intt_sc = total_integration_usecs/1E6;
  def_intt_us = total_integration_usecs - (def_intt_sc*1E6);

  def_nrang = nrang;

  intsc = def_intt_sc;
  intus = def_intt_us;

  /* Configure phasecoded operation if nbaud > 1 */ 
  pcode=(int *)malloc((size_t)sizeof(int)*seq->mppul*nbaud);
  OpsBuildPcode(nbaud,seq->mppul,pcode);

  /* Set special cpid if provided on commandline */
  if(cpid > 0)
     cp=cpid;

  /* Set cp to negative value indication discretionary period */
  if (discretion)
     cp= -cp;


  /* Calculate tx pulse length setting from range separation */
  txpl = (nbaud*rsep*20)/3;

  def_rsep = rsep;
  def_txpl = txpl;

  /* Attempt to adjust mpinc to be a multiple of 10 and a muliple of txpl */
  if ((mpinc % txpl) || (mpinc % 10))  {
    ErrLog(errlog.sock,progname,"Error: mpinc not multiple of txpl... checking to see if it can be adjusted");
    sprintf(logtxt,"Initial: mpinc: %d txpl: %d  nbaud: %d  rsep: %d", mpinc , txpl, nbaud, rsep);
    ErrLog(errlog.sock,progname,logtxt);
    if((txpl % 10)==0) {

      ErrLog(errlog.sock,progname, "Attempting to adjust mpinc to correct");
      if (mpinc < txpl) mpinc=txpl;
      int minus_remain=mpinc % txpl;
      int plus_remain=txpl -(mpinc % txpl);
      if (plus_remain > minus_remain)
        mpinc = mpinc - minus_remain;
      else
         mpinc = mpinc + plus_remain;
      if (mpinc==0) mpinc = mpinc + plus_remain;

    }
  }
  /* Check mpinc and if still invalid, exit with error */
  if ((mpinc % txpl) || (mpinc % 10) || (mpinc==0))  {
     sprintf(logtxt,"Error: mpinc: %d txpl: %d  nbaud: %d  rsep: %d", mpinc , txpl, nbaud, rsep);
     ErrLog(errlog.sock,progname,logtxt);
     SiteExit(0);
  }

  if(test) {
        
    fprintf(stdout,"Control Program Argument Parameters::\n");
    fprintf(stdout,"  xcf arg:: xcnt: %d\n",xcnt);
    fprintf(stdout,"  clrskip arg:: value: %d\n",clrskip);
    fprintf(stdout,"  cpid: %d progname: \'%s\'\n",cp,progname);
    fprintf(stdout,"Scan Sequence Parameters::\n");
    fprintf(stdout,"  txpl: %d mpinc: %d nbaud: %d rsep: %d\n",txpl,mpinc,nbaud,rsep);
    fprintf(stdout,"  intsc: %d intus: %d scnsc: %d scnus: %d\n",intsc,intus,scnsc,scnus);
    fprintf(stdout,"  sbm: %d ebm: %d  nBeams_per_scan: %d\n",sbm,ebm,nBeams_per_scan);
    
    /* TODO: ADD PARAMETER CHECKING, SEE IF PCODE IS SANE AND WHATNOT */
   if(nbaud >= 1) {
        /* create tsgprm struct and pass to TSGMake, check if TSGMake makes something valid */
        /* checking with SiteTimeSeq(ptab); would be easier, but that talks to hardware..*/
        /* the job of aggregating a tsgprm from global variables should probably be a function in maketsg.c */
        int flag = 0;

        if (tsgprm.pat !=NULL) free(tsgprm.pat);
        if (tsgbuf !=NULL) TSGFree(tsgbuf);

        memset(&tsgprm,0,sizeof(struct TSGprm));   
        tsgprm.nrang   = nrang;
        tsgprm.frang   = frang;
        tsgprm.rsep    = rsep; 
        tsgprm.smsep   = smsep;
        tsgprm.txpl    = txpl;
        tsgprm.mppul   = mppul;
        tsgprm.mpinc   = mpinc;
        tsgprm.mlag    = 0;
        tsgprm.nbaud   = nbaud;
        tsgprm.stdelay = 18 + 2;
        tsgprm.gort    = 1;
        tsgprm.rtoxmin = 0;

        tsgprm.pat  = malloc(sizeof(int)*mppul);
        tsgprm.code = seq->ptab;

        for (i=0;i<tsgprm.mppul;i++) 
           tsgprm.pat[i]=seq->ptab[i];

        tsgbuf=TSGMake(&tsgprm,&flag);
        fprintf(stdout,"Sequence Parameters::\n");
        fprintf(stdout,"  lagfr: %d smsep: %d  txpl: %d\n",tsgprm.lagfr,tsgprm.smsep,tsgprm.txpl);
    
        if(tsgprm.smsep == 0 || tsgprm.lagfr == 0) {
            fprintf(stdout,"Sequence Parameters::\n");
            fprintf(stdout,"  lagfr: %d smsep: %d  txpl: %d\n",tsgprm.lagfr,tsgprm.smsep,tsgprm.txpl);
            fprintf(stdout,"WARNING: lagfr or smsep is zero, invalid timing sequence genrated from given baud/rsep/nrang/mpinc will confuse TSGMake and FitACF into segfaulting");
        }

        else {
            fprintf(stdout,"The phase coded timing sequence looks good\n");
        }
    } else {
        fprintf(stdout,"WARNING: nbaud needs to be  > 0\n");
    }

    OpsFitACFStart();
 
    fprintf(stdout,"Test option enabled, exiting\n");
    return 0;
  }

  OpsSetupIQBuf(intsc,intus,mppul,mpinc,nbaud);

  /* SiteSetupRadar, establish connection to usrp_server and do initial setup of memory buffers for raw samples */
  printf("Running SiteSetupRadar Station ID: %s  %d\n",ststr,stid);
  status=SiteSetupRadar();
  if (status !=0) {
    ErrLog(errlog.sock,progname,"Error connection to usrp_server.");
    exit (1);
  }

  printf("Preparing OpsFitACFStart Station ID: %s  %d\n",ststr,stid);
  OpsFitACFStart();

  printf("Preparing OpsSndStart Station ID: %s %d\n",ststr,stid);
  OpsSndStart();

  OpsFindSndSkip(ststr,snd_bms,snd_bms_tot,&snd_bm_cnt,&odd_beams);

  if ((def_nrang == snd_nrang) && (def_rsep == snd_rsep)) {
    printf("Preparing SiteTimeSeq Station ID: %s  %d\n",ststr,stid);
    tsgid=SiteTimeSeq(seq->ptab);
  }


  printf("Entering Scan loop Station ID: %s  %d\n",ststr,stid);
  do {
    if ((def_nrang != snd_nrang) || (def_rsep != snd_rsep)) {
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

    /* reset clearfreq paramaters, in case daytime changed */
    for (iBeam =0; iBeam < nBeams_per_scan; iBeam++){
      scan_clrfreq_fstart_list[iBeam] = (int32_t) (OpsDayNight() == 1 ? dfrq : nfrq); 
      scan_clrfreq_bandwidth_list[iBeam] = frqrng;
      current_beam += backward ? -1:1;
    }

    /* Set iBeam for scan loop  */ 
    iBeam = OpsFindSkip(scnsc,scnus,intsc,intus,nBeams_per_scan);

    /* send scan data to usrp_sever */
    //if (SiteStartScan(nBeams_per_scan, scan_beam_number_list, scan_clrfreq_fstart_list, scan_clrfreq_bandwidth_list, fixfrq, sync_scan, scan_times, scnsc, scnus, intsc, intus, iBeam) !=0){
    if (SiteStartScan() !=0){
         ErrLog(errlog.sock,progname,"Received error from usrp_server in ROS:SiteStartScan. Probably channel frequency issue in SetActiveHandler.");  
         sleep(1);
         continue;
    }

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
    if(clrscan) startup=1;
    if (xcnt>0) {
      cnt++;
      if (cnt==xcnt) {
        xcf=1;
        cnt=0;
      } else xcf=0;
    } else xcf=0;




    /* Scan loop for sequences/beams  */
    do {  
      bmnum = scan_beam_number_list[iBeam];

      TimeReadClock( &yr, &mo, &dy, &hr, &mt, &sc, &us);


      /* THIS IS NOW IS USRP_SERVER */
      /* SYNC periods/beams */ /*
      if (sync_scan) {
          time_now     = ( (mt*60 + sc)*1000 + us/1000 ) % (scnsc*1000 + scnus/1000);
          time_to_wait = scan_times[iBeam] - time_now;
          if (time_to_wait > 0){
             printf("Sync periods: Waiting for %d ms ...", time_to_wait);
             usleep(time_to_wait);
             printf("done.\n");
          } else {
             printf("Sync periods: Not waiting, sinc periods is %d ms too late.", time_to_wait);
          }
      }  */

      /* TODO: JDS: You can not make any day night changes that impact TR gate timing at dual site locations. Care must be taken with day night operation*/ 
      stfrq = scan_clrfreq_fstart_list[iBeam];
      if(fixfrq>0) {
        stfrq=fixfrq;
        tfreq=fixfrq;
        noise=0; 
      }

      ErrLog(errlog.sock,progname,"Starting Integration.");
      sprintf(logtxt,"Int parameters:: rsep: %d mpinc: %d sbm: %d ebm: %d nrang: %d nbaud: %d clrskip_secs: %d clrscan: %d cpid: %d",
              rsep,mpinc,sbm,ebm,nrang,nbaud,clrskip,clrscan,cp);
      ErrLog(errlog.sock,progname,logtxt);

      sprintf(logtxt,"Integrating beam:%d intt:%ds.%dus (%02d:%02d:%02d:%06d)",bmnum, intsc,intus,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);
            
      printf("Entering Site Start Intt Station ID: %s  %d\n",ststr,stid);
      SiteStartIntt(intsc,intus);
      gettimeofday(&t1,NULL);
      elapsed_secs=t1.tv_sec-t0.tv_sec;
      if (elapsed_secs<0) elapsed_secs=0;
      if ((elapsed_secs >= clrskip) || (startup==1)) {
          startup = 0;
          ErrLog(errlog.sock,progname,"Doing clear frequency search.");  
          sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
          ErrLog(errlog.sock,progname, logtxt);
  
          if (fixfrq<=0) {
              tfreq=SiteFCLR(stfrq,stfrq+frqrng);
          }
          t0.tv_sec  = t1.tv_sec;
          t0.tv_usec = t1.tv_usec;
      }
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

      /* Processing and sending data */ 
      OpsBuildPrm(prm,seq->ptab,seq->lags);    
      OpsBuildIQ(iq,&badtr);
      OpsBuildRaw(raw);
      FitACF(prm,raw,fblk,fit,site,tdiff,-999);
      FitSetAlgorithm(fit,"fitacf2");
      
      msg.num   = 0;
      msg.tsize = 0;

      tmpbuf = RadarParmFlatten(prm,&tmpsze);
      RMsgSndAdd(&msg, tmpsze, tmpbuf, PRM_TYPE, 0); 

      tmpbuf=IQFlatten(iq, prm->nave, &tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,IQ_TYPE,0);

      RMsgSndAdd(&msg, sizeof(unsigned int)*2*iq->tbadtr, (unsigned char *) badtr, BADTR_TYPE, 0);
      RMsgSndAdd(&msg, strlen(sharedmemory)+1, (unsigned char *) sharedmemory, IQS_TYPE, 0);

      tmpbuf=RawFlatten(raw,prm->nrang,prm->mplgs,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,RAW_TYPE,0); 
 
      tmpbuf=FitFlatten(fit,prm->nrang,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,FIT_TYPE,0); 
     
     
      for (n=0;n<tnum;n++) RMsgSndSend(task[n].sock,&msg); 

      for (n=0;n<msg.num;n++) {
        if (msg.data[n].type==PRM_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==IQ_TYPE)  free(msg.ptr[n]);
        if (msg.data[n].type==RAW_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==FIT_TYPE) free(msg.ptr[n]); 
      }          

      scan = 0;

      iBeam++;
      if (iBeam >= nBeams_per_scan) break;

    } while (1);


    /* In here comes the sounder code */
    /* set the "sounder mode" scan variable */
    scan = -2;

    /* set the xcf variable to do cross-correlations (AOA) */
    if (xcnt > 1) xcf = 1;

    /* set the sounding mode integration time, number of ranges,
     * and range separation */
    intsc = snd_intt_sc;
    intus = snd_intt_us;
    nrang = snd_nrang;
    rsep = snd_rsep;
    txpl = snd_txpl;

    for (snd_iBeam=0; snd_iBeam < snd_nBeams_per_scan; snd_iBeam++) {
      snd_beam_number_list[snd_iBeam] = snd_bms[snd_bm_cnt] + odd_beams;
      snd_clrfreq_fstart_list[snd_iBeam] = snd_freqs[snd_freq_cnt];
      snd_clrfreq_bandwidth_list[snd_iBeam] = snd_frqrng;
      snd_bc[snd_iBeam] = snd_bm_cnt;
      snd_fc[snd_iBeam] = snd_freq_cnt;

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
    }

    /* Print out details of sounding beams */ 
    fprintf(stderr, "Sounding sequence details: \n");
    for (snd_iBeam =0; snd_iBeam < snd_nBeams_per_scan; snd_iBeam++){
      fprintf(stderr, "  sequence %2d: beam: %2d freq: %5d, \n",snd_iBeam,
              snd_beam_number_list[snd_iBeam], snd_clrfreq_fstart_list[snd_iBeam] );
    }

    snd_iBeam = 0;

    /* send sounding scan data to usrp_sever */
    //if (SiteStartScan(snd_nBeams_per_scan, snd_beam_number_list, snd_clrfreq_fstart_list, snd_clrfreq_bandwidth_list, 0, sync_scan, scan_times, snd_sc, 0, snd_intt_sc, snd_intt_us, snd_iBeam) !=0){
    if (SiteStartScan() !=0){
         ErrLog(errlog.sock,progname,"Received error from usrp_server in ROS:SiteStartScan. Probably channel frequency issue in SetActiveHandler.");  
         sleep(1);
         continue;
    }

    /* make a new timing sequence for the sounding */
    if ((def_nrang != snd_nrang) || (def_rsep != snd_rsep)) {
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
      bmnum = snd_beam_number_list[snd_iBeam];

      /* snd_freq will be an array of frequencies to step through */
      snd_freq = snd_clrfreq_fstart_list[snd_iBeam];

      /* the scanning code is here */
      sprintf(logtxt,"Integrating SND beam:%d intt:%ds.%dus (%02d:%02d:%02d:%06d)",bmnum,intsc,intus,hr,mt,sc,us);
      ErrLog(errlog.sock,progname,logtxt);
      ErrLog(errlog.sock,progname,"Starting SND Integration.");
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
      OpsBuildRaw(raw);
      FitACF(prm,raw,fblk,fit,site,tdiff,-999);

      msg.num = 0;
      msg.tsize = 0;

      tmpbuf=RadarParmFlatten(prm,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,PRM_TYPE,0);

      tmpbuf=FitFlatten(fit,prm->nrang,&tmpsze);
      RMsgSndAdd(&msg,tmpsze,tmpbuf,FIT_TYPE,0);

      RMsgSndSend(task[RT_TASK].sock,&msg);
      for (n=0;n<msg.num;n++) {
        if ( (msg.data[n].type == PRM_TYPE) ||
             (msg.data[n].type == FIT_TYPE) )  free(msg.ptr[n]);
      }

      sprintf(logtxt, "SBC: %d  SFC: %d", snd_bc[snd_iBeam], snd_fc[snd_iBeam]);
      ErrLog(errlog.sock, progname, logtxt);

      /* set the scan variable for the sounding mode data file only */
      if ((bmnum == snd_bms[0]) && (snd_freq == snd_freqs[0])) {
        prm->scan = 1;
      } else {
        prm->scan = 0;
      }

      OpsBuildSnd(snd,prm,fit);

      /* save the sounding mode data */
      OpsWriteSnd(errlog.sock, progname, snd, ststr);

      ErrLog(errlog.sock, progname, "Polling SND for exit.");

      snd_iBeam++;
      if (snd_iBeam >= snd_nBeams_per_scan) break;

      /* see if we have enough time for another go round */
      TimeReadClock(&yr, &mo, &dy, &hr, &mt, &sc, &us);
      snd_time = 60.0 - (sc + us*1e-6);
    }

    ErrLog(errlog.sock,progname,"Waiting for scan boundary.");

    /* make sure we didn't miss any beams/frequencies */
    if (snd_iBeam < snd_nBeams_per_scan-1) {
      snd_bm_cnt = snd_bc[snd_iBeam-1];
      snd_freq_cnt = snd_fc[snd_iBeam-1];
    }

    intsc = def_intt_sc;
    intus = def_intt_us;
    nrang = def_nrang;
    rsep = def_rsep;
    txpl = def_txpl;

    SiteEndScan(scnsc,scnus,5000);

  } while (1);

  for (n=0;n<tnum;n++) RMsgSndClose(task[n].sock);
  
  /* free space allocated for arguements */
  free(ststr);
  free(roshost);
  
  ErrLog(errlog.sock,progname,"Ending program.");


  SiteExit(0);

  return 0;   
}


void usage(void)
{
  printf(" --help     : Prints help information and then exits\n");
  printf(" -debug     : Enable debugging messages\n");
  printf("  -test     : Test-only, report parameter settings and exit without connecting to ros server\n");
  printf("    -di     : Flag this is discretionary time operation\n");
  printf("  -fast     : Flag this as fast 1-minute scan duration\n");
  printf("-clrscan    : Force clear frequency search at start of scan\n");
  printf("   -tau int : Lag spacing in usecs\n");
  printf(" -nrang int : Number of range cells\n");
  printf(" -frang int : Distance to first range cell in km\n");
  printf("  -rsep int : Range cell extent in km\n");
  printf("    -dt int : UTC Hour indicating start of day time operation\n");
  printf("    -nt int : UTC Hour indicating start of night time operation\n");
  printf("    -df int : Day time transmit frequency in kHz\n");
  printf("    -nf int : Night time transmit frequency in kHz\n");
  printf("-fixfrq int : Fixes the transmit frequency of the radar to one frequency, in kHz\n");
  printf("   -xcf int : Enable xcf, -xcf 1: for all sequences -xcf 2: for every other sequence, etc...\n");
  printf("    -ep int : Local TCP port for errlog process\n");
  printf("    -sp int : Local TCP port for radarshell process\n");
  printf("    -bp int : Local TCP port for start of support task processes\n");
  printf("    -sb int : Limits the minimum beam to the given value\n");
  printf("    -eb int : Limits the maximum beam number to the given value\n");
  printf("     -c int : Radar Channel number, minimum value 1\n");
  printf("-clrskip int : Minimum number of seconds to skip between clear frequency search\n");
  printf("  -cpid int : Select control program ID number\n");
  printf("  -ros char : IP address of ROS server process\n");
  printf(" -stid char : The station ID string. For example, use aze for azores east.\n");
  printf("-libstr char : The site library string. For example, use ros for common libsite.ros\n");
  printf("\n");
}

