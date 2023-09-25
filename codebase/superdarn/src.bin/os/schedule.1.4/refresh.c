/* refresh.c
   =========
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
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "rtime.h"
#include "log_info.h"
#include "schedule.h"

int then=-1;
int tdy=-1;
time_t tval=-1;


int test_refresh(struct scd_blk *ptr) {

  struct stat buf;
  int yr,mo,dy,hr,mt,sc,us;
  int nowsec,now,rc;
  TimeReadClock(&yr,&mo,&dy,&hr,&mt,&sc,&us);

  nowsec=hr*3600+mt*60+sc;
  now=nowsec/ptr->refresh;

  /* test to see if the schedule has been altered */

  if ((rc=stat(ptr->name,&buf)) !=0) return -1;
  if (tval==-1) tval=buf.st_mtime;
  if (buf.st_mtime !=tval) {
    tval=buf.st_mtime;
    then=now;
    tdy=dy;
    return 1;
  }

  /* okay check whether we're on a refresh boundary */

  if (then==-1) then=now;
  if (tdy==-1) tdy=dy;
  if ((then != now) || (tdy != dy)) {
    tval=buf.st_mtime;
    then=now;
    tdy=dy;
    return 1;
  }

  return 0;
}
