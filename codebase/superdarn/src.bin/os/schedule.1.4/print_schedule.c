/* print_schedule.c
   ================
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
#include "rtime.h"
#include "log_info.h"
#include "schedule.h"


void print_schedule(struct scd_blk *ptr) {/* prints out the schedule */
  int c;
  char txt[1050];
  if (ptr->num==0) {
    log_info(1,"No schedule file loaded");
    return;
  }

  sprintf(txt,"Schedule file %s loaded",ptr->name);
  log_info(1,txt);

  sprintf(txt,"Command path -> %s",ptr->path);
  log_info(1,txt);

  if (ptr->entry[0].stime==-1) {
    sprintf(txt,"Default Program -> %s\n",ptr->entry[0].command);
    log_info(1,txt);
  }

  if ((ptr->cnt >0) && (ptr->cnt<ptr->num)) {
    int yr,mo,dy,hr,mt;
    double sc;
    log_info(1,"Pending programs :\n");
    for (c=ptr->cnt;c<ptr->num;c++) {
      if (ptr->entry[c].stime==-1) continue;
      TimeEpochToYMDHMS(ptr->entry[c].stime,&yr,&mo,&dy,&hr,&mt,&sc);
      sprintf(txt,"%d : %d %02d %02d : %02d %02d -> %s",c,yr,mo,dy,hr,mt,
              ptr->entry[c].command);
      log_info(1,txt);
    }
  } else log_info(1,"There are no pending programs");

  sprintf(txt,"\nCurrent program -> %s",ptr->command);
  log_info(1,txt);

  sprintf(txt,"Schedule reloaded every %d seconds",ptr->refresh);
  log_info(1,txt);
}
