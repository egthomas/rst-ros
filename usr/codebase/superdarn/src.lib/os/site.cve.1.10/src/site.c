/* site.c
   ====== 
   Author R.J.Barnes
*/

/*
 $License$
*/

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>
#include <libconfig.h>
#include "rtypes.h"
#include "limit.h"
#include "tsg.h"
#include "maketsg.h"
#include "acf.h"
#include "acfex.h"
#include "tcpipmsg.h"
#include "rosmsg.h"
#include "shmem.h"
#include "global.h"
#include "site.h"
#include "siteglobal.h"

#define REAL_BUF_OFFSET 0
#define IMAG_BUF_OFFSET 1
#define USEC 1000000.0
int CVE_exit_flag=0;

struct timeval tock;

config_t cfg;
char station[10];


void SiteCveExit(int signum)
{
  struct ROSMsg msg;

  switch(signum) {
    case 2:
      cancel_count++;
      CVE_exit_flag = signum;
      if (debug) printf("SiteCveExit: Sig 2: %d\n", CVE_exit_flag);
      if (cancel_count < 3) break;
    case 0:
      if (debug) printf("SiteCveExit: Sig 0: %d\n",CVE_exit_flag);
      if (CVE_exit_flag != 0) {
        msg.type = QUIT;
        TCPIPMsgSend(ros.sock, &msg, sizeof(struct ROSMsg));
        TCPIPMsgRecv(ros.sock, &msg, sizeof(struct ROSMsg));
        if (debug) {
          fprintf(stderr,"QUIT:type=%c\n",msg.type);
          fprintf(stderr,"QUIT:status=%d\n",msg.status);
        }
        close(ros.sock);
        config_destroy(&cfg);
        if (samples != NULL)
          ShMemFree((unsigned char *)samples,sharedmemory,IQBUFSIZE,1,shmemfd);
        exit(errno);
      }
      break;
    default:
      printf("SiteCveExit: Sig %d: %d\n",signum,CVE_exit_flag);
      if (CVE_exit_flag == 0) CVE_exit_flag = signum;
      if (CVE_exit_flag != 0) {
        msg.type = QUIT;
        TCPIPMsgSend(ros.sock, &msg, sizeof(struct ROSMsg));
        TCPIPMsgRecv(ros.sock, &msg, sizeof(struct ROSMsg));
        if (debug) {
          fprintf(stderr,"QUIT:type=%c\n",msg.type);
          fprintf(stderr,"QUIT:status=%d\n",msg.status);
        }
        close(ros.sock);
        config_destroy(&cfg);
        if (samples != NULL)
          ShMemFree((unsigned char *)samples,sharedmemory,IQBUFSIZE,1,shmemfd);
        exit(errno);
      }
      break;
  }
}


int SiteCveStart(char *host,char *ststr)
{
  int n,ltemp,retval;
  const char *str;
  char *dfststr="cve";

  char *config_dir=NULL;
  char config_filepath[256];

  signal(SIGPIPE, SiteCveExit);
  signal(SIGINT,  SiteCveExit);
  signal(SIGUSR1, SiteCveExit);
  CVE_exit_flag = 0;
  cancel_count  = 0;

  ros.sock = 0;

  config_dir = getenv("SD_SITE_PATH");
  if (config_dir == NULL) {
    fprintf(stderr,"Environment varaible 'SD_SITE_PATH' not defined\nSiteCveStart aborting\n");
    return -1;
  }

  if (ststr==NULL) ststr=dfststr;

  sprintf(config_filepath,"%s/site.%s/%s.cfg",config_dir,ststr,ststr);

  config_init(&cfg);
  retval = config_read_file(&cfg,config_filepath);
  if (retval == CONFIG_FALSE) {
    fprintf(stderr,"Site Cfg Read Error:: %d - %s\n",
            config_error_line(&cfg), config_error_text(&cfg));
  }

  /* Get the station name */
  if (config_lookup_string(&cfg, "station", &str)) {
    strcpy(station,str);
  } else {
    fprintf(stderr,"Site Cfg Error:: No station setting in configuration file\n");
    return -1;
  }

  /* Get the radar beam scan direction */
  if (config_lookup_int(&cfg, "backward", &ltemp)) {
    backward = ltemp;
  } else {
    backward = 0;
    fprintf(stderr,"Site Cfg Warning:: 'backward' setting undefined in site cfg file, using: %d\n",backward);
  }

  /* Get the XCF (ie elevation angle) collection option */
  if (config_lookup_int(&cfg, "xcf", &ltemp)) {
    xcf = ltemp;
  } else {
    xcf = 1;
    fprintf(stderr,"Site Cfg Warning:: 'xcf' setting undefined in site cfg file, using: %d\n",xcf);
  }

  /* Get the starting beam number */
  if (config_lookup_int(&cfg, "sbm", &ltemp)) {
    sbm = ltemp;
  } else {
    sbm = 0;
    fprintf(stderr,"Site Cfg Warning:: 'sbm' setting undefined in site cfg file, using: %d\n",sbm);
  }

  /* Get the ending beam number */
  if (config_lookup_int(&cfg, "ebm", &ltemp)) {
    ebm = ltemp;
  } else {
    ebm = 23;
    fprintf(stderr,"Site Cfg Warning:: 'ebm' setting undefined in site cfg file, using: %d\n",ebm);
  }

  /* Get the radar number to register with the ROS server */
  if (config_lookup_int(&cfg, "rnum", &ltemp)) {
    rnum = ltemp;
  } else {
    rnum = 1;
    fprintf(stderr,"Site Cfg Warning:: 'rnum' setting undefined in site cfg file, using: %d\n",rnum);
  }

  /* Get the channel number to register with the ROS server */
  if (config_lookup_int(&cfg, "cnum", &ltemp)) {
    cnum = ltemp;
  } else {
    cnum = 1;
    fprintf(stderr,"Site Cfg Warning:: 'cnum' setting undefined in site cfg file, using: %d\n",cnum);
  }

  /* Get the Universal time to start using the day frequency (dfrq) */
  if (config_lookup_int(&cfg, "day", &ltemp)) {
    day = ltemp;
  } else {
    day = 15;
    fprintf(stderr,"Site Cfg Warning:: 'day' setting undefined in site cfg file, using: %d\n",day);
  }

  /* Get the Universal time to start using the night frequency (nfrq) */
  if (config_lookup_int(&cfg, "night", &ltemp)) {
    night = ltemp;
  } else {
    night = 3;
    fprintf(stderr,"Site Cfg Warning:: 'night' setting undefined in site cfg file, using: %d\n",night);
  }

  /* Get the day frequency */
  if (config_lookup_int(&cfg, "dfrq", &ltemp)) {
    dfrq = ltemp;
  } else {
    dfrq = 14700;
    fprintf(stderr,"Site Cfg Warning:: 'dfrq' setting undefined in site cfg file, using: %d\n",dfrq);
  }

  /* Get the night frequency */
  if (config_lookup_int(&cfg, "nfrq", &ltemp)) {
    nfrq = ltemp;
  } else {
    nfrq = 10400;
    fprintf(stderr,"Site Cfg Warning:: 'nfrq' setting undefined in site cfg file, using: %d\n",nfrq);
  }

  /* Get whether you need to correct for inverted phase between main and interf rf signal
   *   invert=0         No inversion necessary
   *   invert=non-zero  Inversion necessary */
  if (config_lookup_int(&cfg, "invert", &ltemp)) {
    invert = ltemp;
  } else {
    invert = 1;
    fprintf(stderr,"Site Cfg Warning:: 'invert' setting undefined in site cfg file, using: %d\n",invert);
  }

  /* Get the number of rx channels (typically 1) */
  if (config_lookup_int(&cfg, "rxchn", &ltemp)) {
    rxchn = ltemp;
  } else {
    rxchn = 1;
    fprintf(stderr,"Site Cfg Warning:: 'rxchn' setting undefined in site cfg file, using: %d\n",rxchn);
  }

  /* Get the ROS server address */
  if (host !=NULL) {
    strcpy(ros.host,host);
  } else if (config_lookup_string(&cfg, "ros.host", &str)) {
    strcpy(ros.host,str);
  } else {
    fprintf(stderr,"Site Cfg Warning:: 'ros.host' setting undefined in site cfg file, using: %s\n",ros.host);
  }

  /* Get the ROS server port */
  if (config_lookup_int(&cfg, "ros.port", &ltemp)) {
    ros.port = ltemp;
  } else {
    fprintf(stderr,"Site Cfg Warning:: 'ros.port' setting undefined in site cfg file, using: %d\n",ros.port);
  }

  /* Get the errlog server address */
  if (config_lookup_string(&cfg, "errlog.host", &str)) {
    strcpy(errlog.host,str);
  } else {
    fprintf(stderr,"Site Cfg Warning:: 'errlog.host' setting undefined in site cfg file, using: %s\n",errlog.host);
  }

  /* Get the errlog server port */
  if (config_lookup_int(&cfg, "errlog.port", &ltemp)) {
    errlog.port = ltemp;
  } else {
    fprintf(stderr,"Site Cfg Warning:: 'errlog.port' setting undefined in site cfg file, using: %d\n",errlog.port);
  }

  /* Get the shell server address */
  if (config_lookup_string(&cfg, "shell.host", &str)) {
    strcpy(shell.host,str);
  } else {
    fprintf(stderr,"Site Cfg Warning:: 'shell.host' setting undefined in site cfg file, using: %s\n",shell.host);
  }

  /* Get the shell server port */
  if (config_lookup_int(&cfg, "shell.port", &ltemp)) {
    shell.port = ltemp;
  } else {
    fprintf(stderr,"Site Cfg Warning:: 'shell.port' setting undefined in site cfg file, using: %d\n",shell.port);
  }

  /* Get the task server address */
  if (config_lookup_string(&cfg, "task.host", &str)) {
    for (n=0;n<4;n++) strcpy(task[n].host,str);
  } else {
    fprintf(stderr,"Site Cfg Warning:: 'task.host' setting undefined in site cfg file, using: %s\n",task[0].host);
  }

  /* Get the task server baseport */
  if (config_lookup_int(&cfg, "task.baseport", &ltemp)) {
    baseport = ltemp;
  } else {
    fprintf(stderr,"Site Cfg Warning:: 'task.baseport' setting undefined in site cfg file, using: %d\n",baseport);
  }

  return 0;
}


int SiteCveSetupRadar()
{
  int32 temp32,data_length;;
  char ini_entry_name[80];
  char requested_entry_type,returned_entry_type;
  struct ROSMsg smsg,rmsg;

  if ((ros.sock=TCPIPMsgOpen(ros.host,ros.port)) == -1) return -1;
  smsg.type = SET_RADAR_CHAN;
  TCPIPMsgSend(ros.sock, &smsg,sizeof(struct ROSMsg));
  temp32 = rnum;
  TCPIPMsgSend(ros.sock, &temp32, sizeof(int32));
  temp32 = cnum;
  TCPIPMsgSend(ros.sock, &temp32, sizeof(int32));
  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
  if (rmsg.status < 0) {
    fprintf(stderr, "Requested radar channel unavailable\n"
                    "Sleeping 1 second and exiting\n");
    sleep(1);
    SiteCveExit(-1);
  }
  if (debug) {
    fprintf(stderr,"SET_RADAR_CHAN:type=%c\n",rmsg.type);
    fprintf(stderr,"SET_RADAR_CHAN:status=%d\n",rmsg.status);
  }

  smsg.type = QUERY_INI_SETTINGS;
  TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
  sprintf(ini_entry_name,"site_settings:ifmode");
  requested_entry_type = 'b';
  returned_entry_type  = ' ';
  temp32 = -1;
  ifmode = -1;
  data_length = strlen(ini_entry_name);
  TCPIPMsgSend(ros.sock, &data_length, sizeof(int32));
  TCPIPMsgSend(ros.sock, &ini_entry_name, data_length*sizeof(char));
  TCPIPMsgSend(ros.sock, &requested_entry_type, sizeof(char));
  TCPIPMsgRecv(ros.sock, &returned_entry_type, sizeof(char));
  TCPIPMsgRecv(ros.sock, &data_length, sizeof(int32));
  if (returned_entry_type == requested_entry_type)
    TCPIPMsgRecv(ros.sock, &temp32, sizeof(int32));

  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
  if (debug) {
    fprintf(stderr,"QUERY_INI_SETTINGS:type=%c\n",rmsg.type);
    fprintf(stderr,"QUERY_INI_SETTINGS:status=%d\n",rmsg.status);
    fprintf(stderr,"QUERY_INI_SETTINGS:entry_name=%s\n",ini_entry_name);
    fprintf(stderr,"QUERY_INI_SETTINGS:entry_type=%c\n",returned_entry_type);
    fprintf(stderr,"QUERY_INI_SETTINGS:entry_value=%d\n",temp32);
  }
  if ((rmsg.status) && (temp32 >= 0)) ifmode = temp32;
  if ((ifmode != 0) && (ifmode != 1)) {
    fprintf(stderr,"QUERY_INI_SETTINGS: Bad IFMODE)\n");
    exit(0);
  }

  smsg.type = GET_PARAMETERS;
  TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
  TCPIPMsgRecv(ros.sock, &rprm, sizeof(struct ControlPRM));
  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
  if (debug) {
    fprintf(stderr,"GET_PARAMETERS:type=%c\n",rmsg.type);
    fprintf(stderr,"GET_PARAMETERS:status=%d\n",rmsg.status);
  }

  sprintf(sharedmemory,"IQBuff_Cve_%d_%d",rnum,cnum);

  samples=(int16 *)
    ShMemAlloc(sharedmemory,IQBUFSIZE,O_RDWR | O_CREAT,1,&shmemfd);

  return 0;
}


int SiteCveStartScan()
{
  struct ROSMsg smsg,rmsg;

  smsg.type = SET_ACTIVE;
  TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));

  return 0;
}


int SiteCveStartIntt(int sec,int usec)
{
  struct ROSMsg smsg,rmsg;
  int total_samples=0;
  double secs;

  SiteCveExit(0);
  if (debug) fprintf(stderr,"SiteCveStartInt: start\n");

  total_samples = tsgprm.samples + tsgprm.smdelay;
  smsg.type = PING;
  TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
  if (debug) {
    fprintf(stderr,"PING:type=%c\n",rmsg.type);
    fprintf(stderr,"PING:status=%d\n",rmsg.status);
  }

  smsg.type = GET_PARAMETERS;
  TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
  TCPIPMsgRecv(ros.sock, &rprm, sizeof(struct ControlPRM));
  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
  if (debug) {
    fprintf(stderr,"GET_PARAMETERS:type=%c\n",rmsg.type);
    fprintf(stderr,"GET_PARAMETERS:status=%d\n",rmsg.status);
  }

  rprm.tbeam = bmnum;
  rprm.tfreq = 12000;
  rprm.trise = 5000;
  rprm.baseband_samplerate = ((double)nbaud/(double)txpl)*1E6;
  rprm.filter_bandwidth    = rprm.baseband_samplerate;
  rprm.match_filter        = 1;
  rprm.number_of_samples   = total_samples + nbaud + 10;
  rprm.priority            = cnum;
  rprm.buffer_index        = 0;

  smsg.type = SET_PARAMETERS;
  TCPIPMsgSend(ros.sock,&smsg,sizeof(struct ROSMsg));
  TCPIPMsgSend(ros.sock,&rprm,sizeof(struct ControlPRM));
  TCPIPMsgRecv(ros.sock,&rmsg,sizeof(struct ROSMsg));
  if (debug) {
    fprintf(stderr,"SET_PARAMETERS:type=%c\n",rmsg.type);
    fprintf(stderr,"SET_PARAMETERS:status=%d\n",rmsg.status);
  }

  gettimeofday(&tock,NULL);
  secs = sec + (double)usec/1E6;
  tock.tv_sec  += floor(secs);
  tock.tv_usec += (secs-floor(secs))*1E6;

  if (debug) fprintf(stderr,"SiteCveStartInt: end\n");

  return 0;
}


int SiteCveFCLR(int stfreq,int edfreq)
{
  int32 tfreq;
  struct ROSMsg smsg,rmsg;
  struct CLRFreqPRM fprm;
  int total_samples=0;

  SiteCveExit(0);

  total_samples = tsgprm.samples + tsgprm.smdelay;
  rprm.tbeam = bmnum;
  rprm.tfreq = tfreq;
  rprm.rfreq = tfreq;
  rprm.trise = 5000;
  rprm.baseband_samplerate = ((double)nbaud/(double)txpl)*1E6;
  rprm.filter_bandwidth    = rprm.baseband_samplerate;
  rprm.match_filter        = 1;
  rprm.number_of_samples   = total_samples + nbaud + 10;
  rprm.priority            = cnum;
  rprm.buffer_index        = 0;

  smsg.type = SET_PARAMETERS;
  TCPIPMsgSend(ros.sock,&smsg,sizeof(struct ROSMsg));
  TCPIPMsgSend(ros.sock,&rprm,sizeof(struct ControlPRM));
  TCPIPMsgRecv(ros.sock,&rmsg,sizeof(struct ROSMsg));
  if (debug) {
    fprintf(stderr,"SET_PARAMETERS:type=%c\n",rmsg.type);
    fprintf(stderr,"SET_PARAMETERS:status=%d\n",rmsg.status);
  }

  fprm.start = stfreq;
  fprm.end   = edfreq;
  fprm.nave  = 20;
  fprm.filter_bandwidth = 250;

  smsg.type = REQUEST_CLEAR_FREQ_SEARCH;
  TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
  TCPIPMsgSend(ros.sock, &fprm, sizeof(struct CLRFreqPRM));
  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
  if (debug) {
    fprintf(stderr,"REQUEST_CLEAR_FREQ_SEARCH:type=%c\n",rmsg.type);
    fprintf(stderr,"REQUEST_CLEAR_FREQ_SEARCH:status=%d\n",rmsg.status);
  }

  smsg.type = REQUEST_ASSIGNED_FREQ;
  TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
  TCPIPMsgRecv(ros.sock,&tfreq, sizeof(int32));
  TCPIPMsgRecv(ros.sock,&noise, sizeof(float));
  TCPIPMsgRecv(ros.sock,&rmsg, sizeof(struct ROSMsg));
  if (debug) {
    fprintf(stderr,"REQUEST_ASSIGNED_FREQ:type=%c\n",rmsg.type);
    fprintf(stderr,"REQUEST_ASSIGNED_FREQ:status=%d\n",rmsg.status);
  }

  return tfreq;
}


int SiteCveTimeSeq(int *ptab)
{
  int i;
  int flag,index=0;
  struct ROSMsg smsg,rmsg;
  struct SeqPRM tprm;

  SiteCveExit(0);

  if (tsgbuf != NULL) TSGFree(tsgbuf);
  if (tsgprm.pat != NULL) free(tsgprm.pat);
  memset(&tsgprm,0,sizeof(struct TSGprm));

  tsgprm.nrang   = nrang;
  tsgprm.frang   = frang;
  tsgprm.rtoxmin = 0;
  tsgprm.stdelay = 18+2;
  tsgprm.gort    = 1;
  tsgprm.rsep    = rsep;
  tsgprm.smsep   = smsep;
  tsgprm.txpl    = txpl;
  tsgprm.mpinc   = mpinc;
  tsgprm.mppul   = mppul;
  tsgprm.mlag    = 0;
  tsgprm.nbaud   = nbaud;
  tsgprm.code    = pcode;
  tsgprm.pat = malloc(sizeof(int)*tsgprm.mppul);
  for (i=0; i<tsgprm.mppul; i++) tsgprm.pat[i] = ptab[i];

  tsgbuf = TSGMake(&tsgprm,&flag);

  if (tsgbuf==NULL) {
    if (flag != 0) return flag;
    else return -1;
  }

  tprm.index   = index;
/*  memcpy(&tprm.buf,tsgbuf,sizeof(struct TSGbuf));*/
  tprm.len     = tsgbuf->len;
  tprm.step    = CLOCK_PERIOD;
  tprm.samples = tsgprm.samples;
  tprm.smdelay = tsgprm.smdelay;

  smsg.type = REGISTER_SEQ;
  TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
  TCPIPMsgSend(ros.sock, &tprm, sizeof(struct SeqPRM));
  TCPIPMsgSend(ros.sock, tsgbuf->rep, sizeof(unsigned char)*tprm.len);
  TCPIPMsgSend(ros.sock, tsgbuf->code, sizeof(unsigned char)*tprm.len);
  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
  if (debug) {
    fprintf(stderr,"REGISTER_SEQ:type=%c\n",rmsg.type);
    fprintf(stderr,"REGISTER_SEQ:status=%d\n",rmsg.status);
  }

  if (rmsg.status != 1) return -2;

  lagfr = tsgprm.lagfr;
  smsep = tsgprm.smsep;
  txpl  = tsgprm.txpl;

  return index;
}


int SiteCveIntegrate(int (*lags)[2])
{
  int *lagtable[2]={NULL,NULL};
  int lagsum[LAG_SIZE];

  int badrng=0;
  int i,j;
  int roff=REAL_BUF_OFFSET;
  int ioff=IMAG_BUF_OFFSET;
  int rngoff=2;

  struct timeval tick;
  struct timeval tack;
  double tval=0,tavg=0;
  double time_diff=0;
  struct ROSMsg smsg,rmsg;

  int iqoff=0; /* Sequence offset in bytes for current sequence relative to *
                           start of samples buffer */
  int iqsze=0; /* Total number of bytes so far recorded into samples buffer*/

  int nave=0;

  int atstp=0;
  int thr=0,lmt=0;
  int aflg=0,abflg=0;
  void *dest; /*AJ*/
  int total_samples=0; /*AJ*/
  int usecs;
  short I,Q;

  /* phase code declarations */
  int n,nsamp, *code, Iout, Qout;
  uint32 uQ32,uI32;

  if (debug) fprintf(stderr,"CVE SiteIntegrate: start\n");

  SiteCveExit(0);

  if (nrang >= MAX_RANGE) return -1;
  for (j=0; j<LAG_SIZE; j++) lagsum[j] = 0;

  if (mplgexs == 0) {
    lagtable[0] = malloc(sizeof(int)*(mplgs+1));
    lagtable[1] = malloc(sizeof(int)*(mplgs+1));

    for (i=0; i<=mplgs; i++) {
      lagtable[0][i] = lags[i][0];
      lagtable[1][i] = lags[i][1];
    }
  } else {
    lagtable[0] = malloc(sizeof(int)*(mplgexs+1));
    lagtable[1] = malloc(sizeof(int)*(mplgexs+1));

    for (i=0; i<=mplgexs; i++) {
      lagtable[0][i] = lags[i][0];
      lagtable[1][i] = lags[i][1];
      j = abs(lags[i][0]-lags[i][1]);
      lagsum[j]++;
    }
  }

  total_samples = tsgprm.samples + tsgprm.smdelay;
  smpnum = total_samples;
  skpnum = tsgprm.smdelay;  /* skpnum != 0  returns 1, which is used as the  *
                                            dflg argument in ACFCalculate to *
                                            enable smdelay usage in offset   *
                                            calculations */

  badrng = ACFBadLagZero(&tsgprm,mplgs,lagtable);

  gettimeofday(&tick,NULL);
  gettimeofday(&tack,NULL);

  for (i=0; i<MAX_RANGE; i++) {
    pwr0[i] = 0;
    for (j=0; j<LAG_SIZE*2; j++) {
      acfd[i*LAG_SIZE*2+j] = 0;
      xcfd[i*LAG_SIZE*2+j] = 0;
    }
  }

  while (1) {
    SiteCveExit(0);

    seqtval[nave].tv_sec  = tick.tv_sec;
    seqtval[nave].tv_nsec = tick.tv_usec*1000;
    seqatten[nave] = 0;
    seqnoise[nave] = 0;

    seqbadtr[nave].num = 0;
    if (seqbadtr[nave].start != NULL)  free(seqbadtr[nave].start);
    if (seqbadtr[nave].length != NULL) free(seqbadtr[nave].length);
    seqbadtr[nave].start  = NULL;
    seqbadtr[nave].length = NULL;

    tval = (tick.tv_sec+tick.tv_usec/1.0e6) - (tack.tv_sec+tack.tv_usec/1.0e6);

    if (nave > 0) tavg = tval/nave;
     
    tick.tv_sec  += floor(tavg);
    tick.tv_usec += 1.0e6*(tavg-floor(tavg));

    time_diff  = (tick.tv_sec-tock.tv_sec);
    time_diff += (tick.tv_usec-tock.tv_usec)/1E6;
    if (time_diff > 0.) break;

    rprm.tbeam = bmnum;
    rprm.tfreq = tfreq;
    rprm.rfreq = tfreq;
    rprm.trise = 5000;
    rprm.baseband_samplerate = ((double)nbaud/(double)txpl)*1E6;
    rprm.filter_bandwidth    = rprm.baseband_samplerate;
    rprm.match_filter        = 1;
    rprm.number_of_samples   = total_samples + nbaud + 10;
    rprm.priority            = cnum;
    rprm.buffer_index        = 0;

    usecs = (int)(rprm.number_of_samples/rprm.baseband_samplerate*1E6);

    smsg.type = SET_PARAMETERS;
    TCPIPMsgSend(ros.sock,&smsg,sizeof(struct ROSMsg));
    TCPIPMsgSend(ros.sock,&rprm,sizeof(struct ControlPRM));
    TCPIPMsgRecv(ros.sock,&rmsg,sizeof(struct ROSMsg));
    if (debug) {
      fprintf(stderr,"SET_PARAMETERS:type=%c\n",rmsg.type);
      fprintf(stderr,"SET_PARAMETERS:status=%d\n",rmsg.status);
    }

    smsg.type = SET_READY_FLAG;
    TCPIPMsgSend(ros.sock,&smsg,sizeof(struct ROSMsg));
    TCPIPMsgRecv(ros.sock,&rmsg,sizeof(struct ROSMsg));
    if (debug) {
      fprintf(stderr,"SET_READY_FLAG:type=%c\n",rmsg.type);
      fprintf(stderr,"SET_READY_FLAG:status=%d\n",rmsg.status);
    }

    usleep(usecs);

    smsg.type = GET_DATA;
    if (rdata.main != NULL) free(rdata.main);
    if (rdata.back != NULL) free(rdata.back);
    rdata.main = NULL;
    rdata.back = NULL;
    TCPIPMsgSend(ros.sock,&smsg,sizeof(struct ROSMsg));
    if (debug) fprintf(stderr,"CVE GET_DATA: recv dprm\n");

    TCPIPMsgRecv(ros.sock,&dprm,sizeof(struct DataPRM));
    if (rdata.main) free(rdata.main);
    if (rdata.back) free(rdata.back);
      if (debug) fprintf(stderr,"CVE GET_DATA: samples %d status %d\n",
                         dprm.samples,dprm.status);
    if (dprm.status == 0) {
      if (debug)
        fprintf(stderr,"CVE GET_DATA: rdata.main: uint32: %d array: %d\n",
                (int)sizeof(uint32),(int)sizeof(uint32)*dprm.samples);

      rdata.main = malloc(sizeof(uint32)*dprm.samples);
      rdata.back = malloc(sizeof(uint32)*dprm.samples);
      if (debug) fprintf(stderr,"CVE GET_DATA: recv main\n");

      TCPIPMsgRecv(ros.sock, rdata.main, sizeof(uint32)*dprm.samples);
      if (debug) fprintf(stderr,"CVE GET_DATA: recv back\n");

      TCPIPMsgRecv(ros.sock, rdata.back, sizeof(uint32)*dprm.samples);
      if (badtrdat.start_usec    != NULL) free(badtrdat.start_usec);
      if (badtrdat.duration_usec != NULL) free(badtrdat.duration_usec);
      if (debug)
        fprintf(stderr,"CVE GET_DATA: trtimes length %d\n",badtrdat.length);

      TCPIPMsgRecv(ros.sock, &badtrdat.length, sizeof(badtrdat.length));
      if (debug) 
        fprintf(stderr,"CVE GET_DATA: badtrdat.start_usec: uint32: %d array: "
                "%d\n",(int)sizeof(uint32),(int)sizeof(uint32)*badtrdat.length);
      badtrdat.start_usec    = malloc(sizeof(uint32)*badtrdat.length);
      badtrdat.duration_usec = malloc(sizeof(uint32)*badtrdat.length);
      if (debug) fprintf(stderr,"CVE GET_DATA: start_usec\n");

      TCPIPMsgRecv(ros.sock, badtrdat.start_usec, sizeof(uint32)*badtrdat.length);
      if (debug) fprintf(stderr,"CVE GET_DATA: duration_usec\n");

      TCPIPMsgRecv(ros.sock, badtrdat.duration_usec,sizeof(uint32)*badtrdat.length);
      TCPIPMsgRecv(ros.sock, &num_transmitters, sizeof(int));
      TCPIPMsgRecv(ros.sock, &txstatus.AGC, sizeof(int)*num_transmitters);
      TCPIPMsgRecv(ros.sock, &txstatus.LOWPWR, sizeof(int)*num_transmitters);
    }
    TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
    if (debug) {
      fprintf(stderr,"CVE GET_DATA:type=%c\n",rmsg.type);
      fprintf(stderr,"CVE GET_DATA:status=%d\n",rmsg.status);
    }

    smsg.type = GET_PARAMETERS;
    TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
    TCPIPMsgRecv(ros.sock, &rprm, sizeof(struct ControlPRM));
    TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));
    if (debug) {
      fprintf(stderr,"CVE GET_PARAMETERS:type=%c\n",rmsg.type);
      fprintf(stderr,"CVE GET_PARAMETERS:status=%d\n",rmsg.status);
      fprintf(stderr,"CVE Number of samples: dprm.samples:%d tsprm.samples:%d "
                     "total_samples:%d\n",dprm.samples,tsgprm.samples,
                     total_samples);
      fprintf(stderr,"CVE nave=%d\n",nave);
    }

    if (dprm.status == 0) {
      nsamp = (int)dprm.samples;

      /* invert interf phase here if necessary */
      if (invert !=0) {
        for(n=0; n<(nsamp); n++) {
          Q = ((rdata.main)[n] & 0xffff0000) >> 16;
          I = (rdata.main)[n] & 0x0000ffff;
          Q = -Q;
          I = -I;
          uQ32 = ((uint32) Q) << 16;
          uI32 = ((uint32) I) & 0xFFFF;
          (rdata.main)[n] = uQ32|uI32;
        }
      }

    /* decode phase coding here */
      if (nbaud > 1) {
        code = pcode;
        for (n=0; n<(nsamp-nbaud); n++) {
          /* Process the main array sample: n */
          Iout = 0;
          Qout = 0;
          for (i=0; i<nbaud; i++) {
            Q = ((rdata.main)[n+i] & 0xffff0000) >> 16;
            I = (rdata.main)[n+i] & 0x0000ffff;
            Iout += (int)I*(int)code[i];
            Qout += (int)Q*(int)code[i];
          }
          Iout /= nbaud;
          Qout /= nbaud;
          I = (short)Iout;
          Q = (short)Qout;
          uQ32 = ((uint32) Q) << 16;
          uI32 = ((uint32) I) & 0xFFFF;
          (rdata.main)[n] = uQ32|uI32;

          /* Process the interf array sample: n */
          Iout = 0;
          Qout = 0;
          for (i=0; i<nbaud; i++) {
            Q = ((rdata.back)[n+i] & 0xffff0000) >> 16;
            I = (rdata.back)[n+i] & 0x0000ffff;
            Iout += (int)I*(int)code[i];
            Qout += (int)Q*(int)code[i];
          }
          Iout /= nbaud;
          Qout /= nbaud;
          I = (short)Iout;
          Q = (short)Qout;
          uQ32 = ((uint32) Q) << 16;
          uI32 = ((uint32) I) & 0xFFFF;
          (rdata.back)[n] = uQ32|uI32;
        }
      }
      if (dprm.samples < total_samples) {
        fprintf(stderr,"Not enough  samples from the ROS in SiteIntegrate\n");
        fflush(stderr);
      }

    /* copy samples here */
      seqoff[nave] = iqsze/2;/*Sequence offset in 16bit units */
      seqsze[nave] = total_samples*2*2; /* Sequence length in 16bit units */

      seqbadtr[nave].num    = badtrdat.length;
      seqbadtr[nave].start  = malloc(sizeof(uint32)*badtrdat.length);
      seqbadtr[nave].length = malloc(sizeof(uint32)*badtrdat.length);

      memcpy(seqbadtr[nave].start,badtrdat.start_usec,
             sizeof(uint32)*badtrdat.length);
      memcpy(seqbadtr[nave].length,badtrdat.duration_usec,
             sizeof(uint32)*badtrdat.length);

/* AJ new way, does work for some reason */
/* samples is natively an int16 pointer */
/* rdata.main is natively an uint32 pointer */
/* rdata.back is natively an uint32 pointer */
/* total_samples*8 represents number of bytes for main and back samples */

      dest = (void *)(samples);  /* look iqoff bytes into samples area */
      dest += iqoff;
      if ((iqoff+total_samples*2*sizeof(uint32)) < IQBUFSIZE) {
        memmove(dest,rdata.main,total_samples*sizeof(uint32));
        /* skip ahead number of samples * 32 bit per sample to account for
           rdata.main */
        dest += total_samples*sizeof(uint32);
        memmove(dest,rdata.back,total_samples*sizeof(uint32));
      } else {
        fprintf(stderr,"IQ Buffer overrun in SiteIntegrate\n");
        fflush(stderr);
      }
      /* Total of number bytes so far copied into samples array */
      iqsze += total_samples*sizeof(uint32)*2;
      if (debug) {
        fprintf(stderr,"CVE seq %d :: ioff: %8d\n",nave,iqoff);
        fprintf(stderr,"CVE seq %d :: rdata.main 16bit 8-11: "
                "%8d %8d %8d %8d\n",nave,
                ((int16 *)rdata.main)[8],((int16 *)rdata.main)[9],
                ((int16 *)rdata.main)[10],((int16 *)rdata.main)[11]);
        dest = (void *)(samples);
        dest += iqoff;
        fprintf(stderr,"CVE seq %d :: samples    16bit 8-11: "
                "%8d %8d %8d %8d\n",nave,
                ((int16 *)dest)[8],((int16 *)dest)[9],
                ((int16 *)dest)[10],((int16 *)dest)[11]);
        fprintf(stderr,"CVE seq %d :: rdata.back 16bit 8-11: "
                "%8d %8d %8d %8d\n",nave,
                ((int16 *)rdata.back)[8],((int16 *)rdata.back)[9],
                ((int16 *)rdata.back)[10],((int16 *)rdata.back)[11]);
        dest += total_samples*4;
        fprintf(stderr,"CVE seq %d :: samples    16bit 8-11: "
                "%8d %8d %8d %8d\n",nave,
                ((int16 *)dest)[8],((int16 *)dest)[9],
                ((int16 *)dest)[10],((int16 *)dest)[11]);
        fprintf(stderr,"CVE seq %d :: iqsze: %8d\n",nave,iqsze);
      }

    /* calculate ACF */   
      if (mplgexs == 0) {
        dest = (void *)(samples);
        dest += iqoff;
        rngoff = 2*rxchn;
        if (debug) fprintf(stderr,"CVE seq %d :: rngoff %d rxchn %d\n"
                                  "CVE seq %d :: ACFSumPower\n",
                           nave,rngoff,rxchn, nave);
        aflg = ACFSumPower(&tsgprm,mplgs,lagtable,pwr0, (int16 *)dest,
                           rngoff,skpnum!=0, roff,ioff,badrng, noise,mxpwr,
                           seqatten[nave]*atstp, thr,lmt,&abflg);
        if (debug) fprintf(stderr,"CVE seq %d :: rngoff %d rxchn %d\n"
                                  "CVE seq %d :: ACFCalculate acf\n",
                           nave,rngoff,rxchn, nave);
        ACFCalculate(&tsgprm, (int16 *)dest,rngoff,skpnum!=0, roff,ioff,mplgs,
                     lagtable,acfd,ACF_PART,2*total_samples,badrng,
                     seqatten[nave]*atstp,NULL);
        if (xcf == 1 ) {
          if (debug) fprintf(stderr,"CVE seq %d :: rngoff %d rxchn %d\n"
                                    "CVE seq %d :: ACFCalculate xcf\n",
                             nave,rngoff,rxchn, nave);
          ACFCalculate(&tsgprm, (int16 *)dest,rngoff,skpnum!=0, roff,ioff,
                       mplgs,lagtable,xcfd,XCF_PART,2*total_samples,badrng,
                       seqatten[nave]*atstp,NULL);
        }
        if ((nave > 0) && (seqatten[nave] != seqatten[nave-1])) {
          if (debug) fprintf(stderr,"CVE seq %d :: rngoff %d rxchn %d\n"
                                    "CVE seq %d :: ACFNormalize\n",
                             nave,rngoff,rxchn, nave);
          ACFNormalize(pwr0,acfd,xcfd,tsgprm.nrang,mplgs,atstp);
        }
        if (debug) fprintf(stderr,"CVE seq %d :: rngoff %d rxchn %d\n",
                                   nave,rngoff,rxchn);
      }
      nave++;
      iqoff = iqsze;  /* set the offset bytes for the next sequence */
    } else {
      fprintf(stderr,"SiteCveIntegrate:: Bad pulse sequence status: %d\n",
                      dprm.status);
      fflush(stderr);
    }
    gettimeofday(&tick,NULL);
  }

  /* Now divide by nave to get the average pwr0 and acfd values for the 
     integration period */ 

   if (mplgexs == 0) {
     if (nave > 0) {
       ACFAverage(pwr0,acfd,xcfd,nave,tsgprm.nrang,mplgs);
/*
       for(range=0; range < nrang;range++) {
         pwr0[range]=(double)pwr0[range]/(double)nave;

         for(lag=0;lag < mplgs; lag++) {
           acfd[range*(2*mplgs)+2*lag]= (double) acfd[range*(2*mplgs)+2*lag]/
                                       (double) nave;
           acfd[range*(2*mplgs)+2*lag+1]= (double) acfd[range*(2*mplgs)+2*lag+1]/
                                       (double) nave;
         }
       }
*/
     }
   } else if (nave > 0) {
     /* ACFEX calculation */
     ACFexCalculate(&tsgprm, (int16 *)samples,nave*smpnum,nave,smpnum,
                    roff,ioff, mplgs,mplgexs,lagtable,lagsum, pwr0,acfd,&noise);
   }
   free(lagtable[0]);
   free(lagtable[1]);
   if (debug) {
     fprintf(stderr,"CVE SiteIntegrate: iqsize in bytes: %d in 16bit samples:"
                    "  %d in 32bit samples: %d\n",iqsze,iqsze/2,iqsze/4);
     fprintf(stderr,"CVE SiteIntegrate: end\n");
   }

   SiteCveExit(0);

   return nave;
}


int SiteCveEndScan(int bsc,int bus, unsigned sleepus)
{
  struct ROSMsg smsg,rmsg;
  struct timeval tock;
  struct timeval tick;
  double bnd;
  double tme;
  int count=0;

  SiteCveExit(0);

  bnd = bsc + bus/USEC;

  if (gettimeofday(&tock,NULL)==-1) return -1;

  tme = tock.tv_sec + tock.tv_usec/USEC;
  tme = bnd*floor(1.0+tme/bnd);
  tock.tv_sec  = tme;
  tock.tv_usec = (tme-floor(tme))*USEC;

  smsg.type = SET_INACTIVE;
  TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
  TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));

  gettimeofday(&tick,NULL);
  while (1) {
    if (tick.tv_sec > tock.tv_sec) break;
    if ((tick.tv_sec == tock.tv_sec) && (tick.tv_usec > tock.tv_usec)) break;
    smsg.type = PING;
    TCPIPMsgSend(ros.sock, &smsg, sizeof(struct ROSMsg));
    TCPIPMsgRecv(ros.sock, &rmsg, sizeof(struct ROSMsg));

    if (debug) {
      fprintf(stderr,"PING:type=%c\n",rmsg.type);
      fprintf(stderr,"PING:status=%d\n",rmsg.status);
      fprintf(stderr,"PING:count=%d\n",count);
      fflush(stderr);
    }
    count++;
    usleep(sleepus);
    gettimeofday(&tick,NULL);
  }

  return 0;
}

