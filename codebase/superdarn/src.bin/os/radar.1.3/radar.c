/* radar.c
   =======
   Author: R.J.Barnes
*/

/*
 (c) 2010 JHU/APL & Others - Please Consult LICENSE.superdarn-rst.3.1-beta-18-gf704e97.txt for more information.
 
 
 
*/


#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include "option.h"
#include "tsg.h"
#include "maketsg.h"

#include "tcpipmsg.h"
#include "rosmsg.h"

#include "errstr.h"
#include "hlpstr.h"

#define DEF_PORT 45000

struct OptionData opt;
unsigned char vb;
int tfreq=11800;


int operate(pid_t parent,int sock) {

  struct ROSMsg smsg,rmsg;
  struct ControlPRM rprm;
  struct CLRFreqPRM fprm;
  struct SeqPRM tprm;
  struct TSGprm tsgprm;
  struct TSGbuf *tsgbuf=NULL;
  struct RosData rdata;
  struct DataPRM dprm;
  struct TRTimes badtrdat;

  int rnum,cnum,s;
  float noise=0.5;

  int32 temp_int32,data_length;
  char entry_name[80];
  char entry_type,return_type;

  int num_transmitters=16;

  int AGC[16],LOPWR[16];

  memset(&rprm,0,sizeof(struct ControlPRM));
  memset(&fprm,0,sizeof(struct CLRFreqPRM));
  memset(&tprm,0,sizeof(struct SeqPRM));
  memset(&tsgprm,0,sizeof(struct TSGprm));
  memset(&dprm,0,sizeof(struct DataPRM));
  memset(&rdata,0,sizeof(struct RosData));
  memset(&badtrdat,0,sizeof(struct TRTimes));

  while(1) {
    s=TCPIPMsgRecv(sock,&smsg,sizeof(struct ROSMsg));
    if (s !=sizeof(struct ROSMsg)) break;

    rmsg.type=smsg.type;

    switch (smsg.type) {
      case SET_RADAR_CHAN:
        if (vb) fprintf(stderr,"SET_RADAR_CHAN\n");
        TCPIPMsgRecv(sock, &rnum, sizeof(int));
        TCPIPMsgRecv(sock, &cnum, sizeof(int));
        break;

      case QUERY_INI_SETTINGS:
        if (vb) fprintf(stderr,"QUERY_INI_SETTINGS\n");
        TCPIPMsgRecv(sock, &data_length, sizeof(int32));
        TCPIPMsgRecv(sock, &entry_name, data_length*sizeof(char));
        TCPIPMsgRecv(sock, &entry_type, sizeof(char));
        return_type='b';
        TCPIPMsgSend(sock, &return_type, sizeof(char));
        data_length=1;
        TCPIPMsgSend(sock, &data_length, sizeof(int32));
        temp_int32=0;
        TCPIPMsgSend(sock, &temp_int32, data_length*sizeof(int32));
        break;

      case GET_PARAMETERS:
        if (vb) fprintf(stderr,"GET_PARAMETERS\n");
        TCPIPMsgSend(sock, &rprm, sizeof(struct ControlPRM));
        break;

      case SET_PARAMETERS:
        if (vb) fprintf(stderr,"SET_PARAMETERS\n");
        TCPIPMsgRecv(sock, &rprm, sizeof(struct ControlPRM));
        break;

      case REQUEST_CLEAR_FREQ_SEARCH:
        if (vb) fprintf(stderr,"REQUEST_CLEAR_FREQ_SEARCH\n");
        TCPIPMsgRecv(sock, &fprm, sizeof(struct CLRFreqPRM));
        tfreq=fprm.start;
        break;

      case REQUEST_ASSIGNED_FREQ:
        if (vb) fprintf(stderr,"REQUEST_ASSIGNED_FREQ\n");
        TCPIPMsgSend(sock, &tfreq, sizeof(int));
        TCPIPMsgSend(sock, &noise, sizeof(float));
        break;

      case REGISTER_SEQ:
        if (vb) fprintf(stderr,"REGISTER_SEQ\n");
        TCPIPMsgRecv(sock, &tprm, sizeof(struct SeqPRM));
        if (tsgbuf !=NULL) TSGFree(tsgbuf);
        tsgbuf=malloc(sizeof(struct TSGbuf));
        tsgbuf->len=tprm.len;
        tsgbuf->rep=malloc(sizeof(unsigned char)*tsgbuf->len);
        tsgbuf->code=malloc(sizeof(unsigned char)*tsgbuf->len);
        if (vb) fprintf(stderr,"%d\n",tprm.len);
        TCPIPMsgRecv(sock, tsgbuf->rep, sizeof(unsigned char)*tsgbuf->len);
        TCPIPMsgRecv(sock, tsgbuf->code, sizeof(unsigned char)*tsgbuf->len);
        break;

      case PING:
        if (vb) fprintf(stderr,"PING\n");
        break;

      case SET_READY_FLAG:
        if (vb) fprintf(stderr,"SET_READY_FLAG\n");
        break;

      case GET_DATA:
        if (vb) fprintf(stderr,"GET_DATA\n");

        if (rdata.main !=NULL) free(rdata.main);
        if (rdata.back !=NULL) free(rdata.back);

        if (vb) fprintf(stderr,"Number of samples %d\n",dprm.samples);
        dprm.samples=tprm.samples+tprm.smdelay+30;
        dprm.status=0;

        rdata.main=malloc(4*dprm.samples);
        rdata.back=malloc(4*dprm.samples);

        if (badtrdat.start_usec !=NULL) free(badtrdat.start_usec);
        if (badtrdat.duration_usec !=NULL) free(badtrdat.duration_usec);
        badtrdat.length=7;
        badtrdat.start_usec= malloc(sizeof(unsigned int)*badtrdat.length);
        badtrdat.duration_usec= malloc(sizeof(unsigned int)*badtrdat.length);

        TCPIPMsgSend(sock,&dprm,sizeof(struct DataPRM));
        if(dprm.status==0) {
          TCPIPMsgSend(sock, rdata.main, 4*dprm.samples);
          TCPIPMsgSend(sock, rdata.back, 4*dprm.samples);

          TCPIPMsgSend(sock, &badtrdat.length, sizeof(badtrdat.length));

          TCPIPMsgSend(sock, badtrdat.start_usec,
                       sizeof(unsigned int)*badtrdat.length);
          TCPIPMsgSend(sock, badtrdat.duration_usec,
                       sizeof(unsigned int)*badtrdat.length);

          TCPIPMsgSend(sock,&num_transmitters,sizeof(int));

          TCPIPMsgSend(sock,AGC,sizeof(int)*num_transmitters);
          TCPIPMsgSend(sock,LOPWR,sizeof(int)*num_transmitters);
        }
        break;

      case QUIT:
        if (vb) fprintf(stderr,"QUIT\n");
        break;

      default:
        break;
    }

    if (vb) fprintf(stderr,"-return-\n");

    rmsg.status=1;
    s=TCPIPMsgSend(sock,&rmsg,sizeof(struct ROSMsg));

    if (s !=sizeof(struct ROSMsg)) break;
  }

  if (vb) fprintf(stderr,"EXIT\n");
  return 0;
}


int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: radar --help\n");
  return(-1);
}


int main(int argc,char *argv[]) {

  int port=DEF_PORT,arg=0;
  int sock;
  int sc_reuseaddr=1,temp;

  unsigned char help=0;
  unsigned char option=0;
  unsigned char version=0;

  socklen_t length;
  socklen_t clength;

  struct sockaddr_in server;
  struct sockaddr_in client;

  fd_set ready;

  struct hostent *gethostbyname();
  pid_t root;

  int msgsock=0;

  OptionAdd(&opt,"-help",'x',&help);
  OptionAdd(&opt,"-option",'x',&option);
  OptionAdd(&opt,"-version",'x',&version);
  OptionAdd(&opt,"vb",'x',&vb);
  OptionAdd(&opt,"lp",'i',&port);

  arg=OptionProcess(1,argc,argv,&opt,rst_opterr);

  if (arg==-1) {
    exit(-1);
  }

  if (help==1) {
    OptionPrintInfo(stdout,hlpstr);
    exit(0);
  }

  if (option==1) {
    OptionDump(stdout,&opt);
    exit(0);
  }

  if (version==1) {
    OptionVersion(stdout);
    exit(0);
  }

  signal(SIGCHLD,SIG_IGN);
  signal(SIGPIPE,SIG_IGN);

  root=getpid();

  sock=socket(AF_INET,SOCK_STREAM,0); /* create our listening socket */
  if (sock<0) {
    perror("opening stream socket");
    exit(1);
  }

  /* set socket options */
  temp=setsockopt(sock,SOL_SOCKET,SO_REUSEADDR,&sc_reuseaddr,
                  sizeof(sc_reuseaddr));

  /* name and bind socket to an address and port number */

  server.sin_family=AF_INET;
  server.sin_addr.s_addr=INADDR_ANY;
  if (port !=0) server.sin_port=htons(port);
  else server.sin_port=0;

  if (bind(sock,(struct sockaddr *) &server,sizeof(server))) {
    perror("binding stream socket");
    exit(1);
  }

  /* Find out assigned port number and print it out */

  length=sizeof(server);
  if (getsockname(sock,(struct sockaddr *) &server,&length)) {
    perror("getting socket name");
    exit(1);
  }

  listen(sock,5); /* mark our socket willing to accept connections */

  do {

    /* block until someone wants to attach to us */

    FD_ZERO(&ready);
    FD_SET(sock,&ready);
    if (select(sock+1,&ready,0,0,NULL) < 0) {
      perror("while testing for connections");
      continue;
    }

    /* Accept the connection from the client */

    fprintf(stdout,"Accepting a new connection...\n");
    clength=sizeof(client);
    msgsock=accept(sock,(struct sockaddr *) &client,&clength);

    if (msgsock==-1) {
      perror("accept");
      continue;
    }

    if (fork() == 0) {
      close(sock);
      operate(root,msgsock);
      exit(0);
    }

    close(msgsock);
  } while(1);

  return 0;

}

