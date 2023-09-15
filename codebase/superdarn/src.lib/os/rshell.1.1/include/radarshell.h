/* radarshell.h
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

#ifndef _RADARSHELL_H
#define _RADARSHELL_H

#define STR_MAX 128

enum {  /* variable types */
  var_SHORT,
  var_INT,
  var_LONG,
  var_FLOAT,
  var_DOUBLE,
  var_STRING
};

struct RShellEntry {
  char *name;
  int type;
  void *data;
};

struct RShellTable {
  int num;
  struct RShellEntry *ptr;
};

struct RShellBuffer {
  int num;
  size_t len;
  size_t *off;
  char *buf;
};

struct RShellBuffer *RShellBufferMake();
int RShellBufferAlloc(struct RShellBuffer *ptr,void *buf,int sze);
void *RShellBufferRead(struct RShellBuffer *ptr,int num);
void RShellBufferFree(struct RShellBuffer *ptr);

int RadarShell(int sock,struct RShellTable *rptr);
int RadarShellAdd(struct RShellTable *rptr,
                  char *name,int type,void *data);
void RadarShellFree(struct RShellTable *rptr);
struct RShellEntry *RadarShellRead(struct RShellTable *rptr,int num);
struct RShellEntry *RadarShellFind(struct RShellTable *rptr,char *name);

int RadarShellParse(struct RShellTable *rptr,char *name,...);

#endif
