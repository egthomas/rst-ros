/* log_info.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

extern char *logname;
extern char *logpath;

void log_info(int flg,char *str) {
  FILE *fp;
  char *date;
  char lpath[256];
  time_t ltime;
  struct tm *time_of_day;

  time(&ltime);
  time_of_day=gmtime(&ltime);

  date=asctime(time_of_day);
  date[strlen(date)-1]=0;
  if (flg==0) fprintf(stderr,"%s : %d : %s\n",date,getpid(),str);
  else fprintf(stderr,"%s\n",str);

  sprintf(lpath,"%s%.4d%.2d%.2d.%s",logpath,1900+
          time_of_day->tm_year,time_of_day->tm_mon+1,
          time_of_day->tm_mday,logname);
  fp=fopen(lpath,"a");
  if (fp !=NULL) {
    if (flg==0) fprintf(fp,"%s : %d : %s\n",date,getpid(),str);
    else fprintf(fp,"%s\n",str);
    fclose(fp);
  }
}

