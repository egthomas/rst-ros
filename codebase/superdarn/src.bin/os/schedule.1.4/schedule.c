/* schedule.c
   ==========
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


#include <time.h>
#include <sys/types.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include "option.h"

#include "tcpipmsg.h"
#include "errlog.h"

#include "log_info.h"
#include "schedule.h"
#include "refresh.h"
#include "check_program.h"

#include "errstr.h"
#include "hlpstr.h"

struct OptionData opt;

struct scd_blk schedule;
struct scd_blk save_sched;

char *logname=NULL;
char *logpath=NULL;


void child_handler(int signum) {
  pid_t pid;
  if (schedule.pid !=-1) pid=wait(NULL);
  schedule.pid=-1;
}


int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: schedule --help\n");
  return(-1);
}


int main(int argc,char *argv[]) {

  unsigned char dyflg=0;
  unsigned char hrflg=0;
  unsigned char tmtflg=0;
  unsigned char mtflg=0;

  int arg=0;

  unsigned char help=0;
  unsigned char option=0;
  unsigned char version=0;

  char logtxt[512];
  char name[256];
  char tmp[256];
  char *radar=NULL;
  char *path;

  FILE *fp;

  sigset_t set;
  struct sigaction act;
  struct sigaction oldact;

  schedule.refresh=24*3600;

  OptionAdd(&opt,"-help",'x',&help);
  OptionAdd(&opt,"-option",'x',&option);
  OptionAdd(&opt,"-version",'x',&version);

  OptionAdd(&opt,"d",'x',&dyflg);
  OptionAdd(&opt,"h",'x',&hrflg);
  OptionAdd(&opt,"t",'x',&tmtflg);
  OptionAdd(&opt,"m",'x',&mtflg);

  OptionAdd(&opt,"name",'t',&radar);

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

  if (argc==arg) {
    OptionPrintInfo(stdout,errstr);
    exit(0);
  }

  sigemptyset(&set);

  act.sa_flags=0;
  act.sa_mask=set;
  act.sa_handler=child_handler;
  sigaction(SIGCHLD,&act,&oldact);

  path = getenv("SD_SCDLOG_PATH");
  if (path == NULL) {
    fprintf(stderr,"SD_SCDLOG_PATH not found\n");
    strcpy(tmp,"/data/ros/scdlog/");
  } else {
    strcpy(tmp,path);
    strcat(tmp,"/");
  }
  logpath=tmp;

  if (radar !=NULL) {
    strcpy(name,radar);
    strcat(name,".scdlog");
  } else {
    strcpy(name,"scdlog");
  }
  logname=name;

  if (dyflg) schedule.refresh=24*3600;
  if (hrflg) schedule.refresh=3600;
  if (tmtflg) schedule.refresh=10*60;
  if (mtflg) schedule.refresh=60;
  strcpy(schedule.name,argv[arg]);

  schedule.num=0;
  schedule.pid=-1;
  schedule.cnt=0;
  strcpy(schedule.command,"Null");

  path = getenv("USR_BINPATH");
  if (path == NULL) {
    log_info(0,"USR_BINPATH not found");
    strcpy(schedule.path,"/");
  } else {
    strcpy(schedule.path,path);
  }

  fp=fopen(schedule.name,"r");
  if (fp==NULL) {
    log_info(0,"Schedule file not found");
    exit(-1);
  }

  if (load_schedule(fp,&schedule) !=0) {
    log_info(0,"Error reading schedule file");
    exit(-1);
  }

  fclose(fp);
  schedule.cnt=set_schedule(&schedule);
  sleep(1);
  check_program(&schedule,schedule.cnt-1);
  print_schedule(&schedule);

  do {
    if (schedule.pid ==-1) {
      log_info(0,"Control program not running - Failed or Died\n");
      log_info(0,"Waiting 5 seconds and trying to restart\n");
      sleep(5);
      check_program(&schedule,schedule.cnt-1);
    }

    if (test_refresh(&schedule) != 0) {
      struct stat nstat1, nstat2;
      log_info(0, "Waiting for schedule file.");
      stat(schedule.name, &nstat1);
      /* sleep(10); */
      stat(schedule.name, &nstat2);
      while (nstat1.st_mtime != nstat2.st_mtime) {
        log_info(0, "File is being modified.");
        stat(schedule.name, &nstat1);
        sleep(10);
        stat(schedule.name, &nstat2);
      }
      log_info(0, "Refreshing schedule");
      fp = fopen(schedule.name, "r");
      if (fp == NULL) {
        sprintf(logtxt, "Schedule file %s not found", schedule.name);
        log_info(0, logtxt);
      } else {
        memcpy(&save_sched, &schedule, sizeof(schedule));
        if (load_schedule(fp, &schedule) != 0) {
          log_info(0, "Error reading updated schedule file");
          memcpy(&schedule, &save_sched, sizeof(schedule));
        }
        fclose(fp);
      }
      schedule.cnt = set_schedule(&schedule);

      check_program(&schedule, schedule.cnt - 1);
      print_schedule(&schedule);
    }

    if (test_schedule(&schedule) !=0) {
      check_program(&schedule,schedule.cnt);
      schedule.cnt=set_schedule(&schedule);
    }
    sleep(1);

  } while(1);

  return 0;
}

