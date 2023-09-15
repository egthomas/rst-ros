/* radarshell.c
   ============
   Author: R.J.Barnes
*/

/*
 LICENSE AND DISCLAIMER
 
 Copyright (c) 2012 The Johns Hopkins University/Applied Physics Laboratory
 
 This file is part of the RST Radar Operating System (RST-ROS).
 
 RST-ROS is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <https://www.gnu.org/licenses/>. 
 
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include "option.h"
#include "tcpipmsg.h"
#include "hlpstr.h"
#include "shell.h"

#define SHELL_SEND 's'
#define SHELL_OK '0'
#define SHELL_ERROR 'e'
#define SHELL_REPLY 'r'


char *dhost="127.0.0.1";
int dport=45001;

struct TCPIPMsgHost server;

int num=0;
size_t *offset=NULL;
size_t size=0;
char *buffer=NULL;

int arg=0;
struct OptionData opt;

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: radarshell --help\n");
  return(-1);
}

int main(int argc,char *argv[]) {

  unsigned char help=0;
  unsigned char option=0;
  unsigned char version=0;
  int smsg,rmsg;
  int s;

  OptionAdd(&opt,"-help",'x',&help);
  OptionAdd(&opt,"-option",'x',&option);
  OptionAdd(&opt,"-version",'x',&version);

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

  if (arg<argc) {
    strcpy(server.host,argv[1]);
    server.port=atoi(argv[2]);
  } else {
    strcpy(server.host,dhost);
    server.port=dport;
  }

  fprintf(stdout,"radar_shell\n");
  fprintf(stdout,"===========\n\n");
  fprintf(stdout,"Connecting to: %s %d \n\n",server.host,server.port);

  if ((server.sock=TCPIPMsgOpen(server.host,server.port))==-1) {
    fprintf(stderr,"Error connecting to shell.\n");
    exit(-1);
  }

  fprintf(stdout,"Downloading parameters..\n");

  smsg=SHELL_SEND;
  s=TCPIPMsgSend(server.sock,&smsg,sizeof(int));
  if (s !=sizeof(int)) {
    fprintf(stderr,"Communication error.\n");
    exit(1);
  }

  s=TCPIPMsgRecv(server.sock,&rmsg,sizeof(int));
  if (s !=sizeof(int)) {
    fprintf(stderr,"Communication error.\n");
    exit(1);
  }

  if (rmsg !=SHELL_OK) {
    fprintf(stderr,"No available parameters.\n");
    exit(1);
  }

  s=TCPIPMsgRecv(server.sock,&num,sizeof(int));
  if (s !=sizeof(int)) {
    fprintf(stderr,"Communication error.\n");
    exit(1);
  }

  s=TCPIPMsgRecv(server.sock,&size,sizeof(size_t));
  if (s !=sizeof(size_t)) {
    fprintf(stderr,"Communication error.\n");
    exit(1);
  }

  offset=malloc(sizeof(size_t)*num);
  if (offset==NULL) {
    fprintf(stderr,"Failed to allocate memory.\n");
    exit(1);
  }

  s=TCPIPMsgRecv(server.sock,offset,num*sizeof(size_t));
  if (s !=sizeof(size_t)*num) {
    fprintf(stderr,"Communication error.\n");
    exit(1);
  }

  buffer=malloc(size);
  if (buffer==NULL) {
     fprintf(stderr,"Failed to allocate memory.\n");
     exit(1);
  }

  s=TCPIPMsgRecv(server.sock,buffer,size);
  if (s !=size) {
    fprintf(stderr,"Communication error.\n");
    exit(1);
  }


  /* enter the shell */

  s=shell(num,size,offset,buffer);

  if (s !=0) {
    fprintf(stderr,"Closing shell without uploading parameters.\n");
    exit(0);
  }

  smsg=SHELL_REPLY;
  s=TCPIPMsgSend(server.sock,&smsg,sizeof(int));
  if (s !=sizeof(int)) {
    fprintf(stderr,"Communication error.\n");
    exit(1);
  }

  s=TCPIPMsgRecv(server.sock,&rmsg,sizeof(int));
  if (s !=sizeof(int)) {
    fprintf(stderr,"Communication error.\n");
    exit(1);
  }

  if (rmsg !=SHELL_OK) {
    fprintf(stderr,"Cannot upload parameters.\n");
    exit(1);
  }

  s=TCPIPMsgSend(server.sock,&num,sizeof(int));
  if (s !=sizeof(int)) {
    fprintf(stderr,"Communication error.\n");
    exit(1);
  }

  s=TCPIPMsgSend(server.sock,&size,sizeof(size_t));
  if (s !=sizeof(size_t)) {
    fprintf(stderr,"Communication error.\n");
    exit(1);
  }

  s=TCPIPMsgSend(server.sock,offset,sizeof(size_t)*num);
  if (s !=sizeof(size_t)*num) {
    fprintf(stderr,"Communication error.\n");
    exit(1);
  }

  s=TCPIPMsgSend(server.sock,buffer,size);
  if (s !=size) {
    fprintf(stderr,"Communication error.\n");
    exit(1);
  }

  fprintf(stderr,"Parameters succesfully uploaded.\n");
  return 0;
}
