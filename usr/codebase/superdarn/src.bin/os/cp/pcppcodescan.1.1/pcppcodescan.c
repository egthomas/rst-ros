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
#include "setup.h"
#include "sync.h"

#include "site.h"
#include "sitebuild.h"
#include "siteglobal.h"

/* #define PCPBEAM 9 
Not possible to set two different beams for dual sites.
*/

#define PCPFNUM 8
#define PCPCPID 9213		/* never used. SGS */
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

  int ptab[8] = {0,14,22,24,27,31,42,43};
  int *bcode;
  int bcode2[2]={1,-1};
  int bcode3[3]={1,1,-1};
  int bcode4[4]={1,1,-1,1};
  int bcode5[5]={1,1,1,-1,1};
  int bcode7[7]={1,1,1,-1,-1,1,-1};
  int bcode11[11]={1,1,1,-1,-1,-1,1,-1,-1,1,-1};
  int bcode13[13]={1,1,1,1,1,-1,-1,1,1,-1,1,-1,1};

  int lags[LAG_SIZE][2] = {	/* we really should decouple this from CPs. SGS */
    { 0, 0},		/*  0 */
    {42,43},		/*  1 */
    {22,24},		/*  2 */
    {24,27},		/*  3 */
    {27,31},		/*  4 */
    {22,27},		/*  5 */

    {24,31},		/*  7 */
    {14,22},		/*  8 */
    {22,31},		/*  9 */
    {14,24},		/* 10 */
    {31,42},		/* 11 */
    {31,43},		/* 12 */
    {14,27},		/* 13 */
    { 0,14},		/* 14 */
    {27,42},		/* 15 */
    {27,43},		/* 16 */
    {14,31},		/* 17 */
    {24,42},		/* 18 */
    {24,43},		/* 19 */
    {22,42},		/* 20 */
    {22,43},		/* 21 */
    { 0,22},		/* 22 */

    { 0,24},		/* 24 */

    {43,43}};		/* alternate lag-0  */

    char logtxt[1024];

  int exitpoll=0;
 
  int scnsc=60;
  int scnus=0;
  int skip;
/*  int cnt=0; */

  unsigned char fast=0;
  unsigned char discretion=0;

  unsigned char option=0;

  int status=0,n,i;

  int PCPBEAM=9;    /* Set PCPBEAM to default to beam 9 -KTS */

  cp=PCPCPID;
  intsc=2;					/* Why is this set to 2 seconds and changed later? SGS */
  intus=0;
  mppul=8;
  mplgs=23;
  mpinc=1560;
  nrang=565;			/* 3390 km range at 6 km range separation */
  nrang=750;			/* 4500 km range at 6 km range separation */
  rsep=6;
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

  OptionAdd(&opt, "sbm", 'i', &sbm);
  OptionAdd(&opt, "ebm", 'i', &ebm);
  OptionAdd(&opt, "ep",  'i', &errlog.port);
  OptionAdd(&opt, "sp",  'i', &shell.port); 
  OptionAdd(&opt, "bp",  'i', &baseport); 
  OptionAdd(&opt, "stid",'t', &ststr); 
  OptionAdd(&opt, "fast",'x', &fast);
  OptionAdd(&opt, "ros", 't', &roshost);  /* Set the roshost IP address */
  OptionAdd(&opt, "debug",'x', &debug);
  OptionAdd(&opt, "-option",'x', &option);
   
  arg=OptionProcess(1,argc,argv,&opt,rst_opterr);

  if (arg==-1) {
    exit(-1);
  }

  if (option==1) {
    OptionDump(stdout,&opt);
    exit(0);
  }
 
  if (ststr==NULL) ststr=dfststr;

  if(nbaud==2) bcode=bcode2;
  if(nbaud==3) bcode=bcode3;
  if(nbaud==4) bcode=bcode4;
  if(nbaud==5) bcode=bcode5;
  if(nbaud==7) bcode=bcode7;
  if(nbaud==11) bcode=bcode11;
  if(nbaud==13) bcode=bcode13;
  pcode=(int *)malloc((size_t)sizeof(int)*mppul*nbaud);
  for(i=0;i<mppul;i++){
    for(n=0;n<nbaud;n++){
      pcode[i*nbaud+n]=bcode[n];
    }
  }
  
  printf("Station String: %s\n",ststr);
  OpsStart(ststr);

  status=SiteBuild(libstr);

  if (status==-1) {
    fprintf(stderr,"Could not load site library.\n");
    exit(1);
  }

  status = SiteStart(roshost,ststr);	/* sbm and ebm are reset by this function. SGS */
  if (status==-1) {
    fprintf(stderr,"Error reading site configuration file.\n");
    exit(1);
  }

  arg=OptionProcess(1,argc,argv,&opt,NULL);  /* this fixes it... SGS */

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
   
  RadarShellParse(&rstable,"sbm l ebm l dfrq l nfrq l dfrang l nfrang l dmpinc l nmpinc l frqrng l xcnt l",                        
                  &sbm,&ebm,                              
                  &dfrq,&nfrq,                  
                  &dfrang,&nfrang,                            
                  &dmpinc,&nmpinc,                            
                  &frqrng,&xcnt);      
  
  status=SiteSetupRadar();
  
  if (status !=0) {
    ErrLog(errlog.sock,progname,"Error locating hardware.");
    exit (1);
  }


     cp=PCPCPID;			/* why do we need this block? SGS */
     scnsc=60;
     scnus=0;
     intsc=2;
     intus=0;

  if (discretion) cp= -cp;

  txpl=(nbaud*rsep*20)/3;

  sprintf(progname,"pcppcodescan");

  OpsLogStart(errlog.sock,progname,argc,argv);  

  OpsSetupTask(tnum,task,errlog.sock,progname);
  for (n=0;n<tnum;n++) {
    RMsgSndReset(task[n].sock);
    RMsgSndOpen(task[n].sock,strlen( (char *) command),command);     
  }
  
  OpsFitACFStart();

  tsgid=SiteTimeSeq(ptab);

  do {

     cp=PCPCPID;		/* does the cp ever change? SGS */
     scnsc=60;	/* this never changes and is defined 3 times... SGS */
     scnus=0;		/* this one too... SGS */
     intsc=3;
     intus=0;
  
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
   
    xcf=1;                              /*  FHR unable to do xcf at this time  21Dec2011  */
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
        frang=dfrang;
      } else {
        stfrq=nfrq;
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
            
      printf("FRQ: %d %d", stfrq, frqrng);
      tfreq=SiteFCLR(stfrq-frqrng/2,stfrq+frqrng/2);
      
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
      RMsgSndAdd(&msg,strlen(progname)+1,(unsigned char *)progname,NME_TYPE,0);
      for (n=0;n<tnum;n++) RMsgSndSend(task[n].sock,&msg);
      for (n=0;n<msg.num;n++) {
        if (msg.data[n].type==PRM_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==IQ_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==RAW_TYPE) free(msg.ptr[n]);
        if (msg.data[n].type==FIT_TYPE) free(msg.ptr[n]);
      }

      RadarShell(shell.sock,&rstable);

      if (exitpoll !=0) break;		/* where does exitpoll ever get set? SGS */
      scan=0;
      if (bmnum==ebm) break;			/* it seems that it is critical that the
																		 number of beams must fit within the
																		 integration time, no? SGS */
      if (backward) bmnum--;
      else bmnum++;

    } while (1);
    /* ** Single beam sounding to fill remaining time ******************* */
    if (exitpoll==0){
      	scan=-2;
       	bmnum=PCPBEAM;
				intsc=1;
        intus=0;
       	for (pcpcnt=0;pcpcnt<PCPFNUM;pcpcnt++) {
       		stfrq=pcpfreqs[pcpcnt];
         	TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

      		sprintf(logtxt,"Sounding beam:%d intt:%ds.%dus (%d:%d:%d:%d)",bmnum,
                      intsc,intus,hr,mt,sc,us);
	      	ErrLog(errlog.sock,progname,logtxt);
      		ErrLog(errlog.sock,progname,"Starting Integration.");
            
      		SiteStartIntt(intsc,intus);

      		ErrLog(errlog.sock,progname,"Doing clear frequency search."); 
      		sprintf(logtxt, "FRQ: %d %d", stfrq, frqrng);
      		ErrLog(errlog.sock,progname, logtxt);
            
      		printf("FRQ: %d %d", stfrq, frqrng);
      		tfreq=SiteFCLR(stfrq-frqrng/2,stfrq+frqrng/2);
      
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
      		RMsgSndAdd(&msg,strlen(progname)+1,(unsigned char *) progname,
		 	NME_TYPE,0);
      		for (n=0;n<tnum;n++) RMsgSndSend(task[n].sock,&msg);
      		for (n=0;n<msg.num;n++) {
        		if (msg.data[n].type==PRM_TYPE) free(msg.ptr[n]);
        		if (msg.data[n].type==IQ_TYPE) free(msg.ptr[n]);
        		if (msg.data[n].type==RAW_TYPE) free(msg.ptr[n]);
        		if (msg.data[n].type==FIT_TYPE) free(msg.ptr[n]);
      		}
       }
    }
    /* ************************** END Single beam sounding to fill remaining time ******************* */
    ErrLog(errlog.sock,progname,"Waiting for scan boundary."); 
    SiteEndScan(scnsc,scnus,5000);

  } while (exitpoll==0);
  
  for (n=0;n<tnum;n++) RMsgSndClose(task[n].sock);

  ErrLog(errlog.sock,progname,"Ending program.");

  SiteExit(0);

  return 0;   
} 
 
