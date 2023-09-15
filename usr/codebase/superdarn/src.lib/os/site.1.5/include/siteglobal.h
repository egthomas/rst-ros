/* siteglobal.h
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

#ifndef _SITEGLOBAL_H
#define _SITEGLOBAL_H

extern int num_transmitters;
/*extern struct timeval tock;*/
extern struct ControlPRM rprm;
extern struct RosData rdata;
extern struct DataPRM dprm;
extern struct TRTimes badtrdat;
extern struct TXStatus txstatus;
extern struct SiteLibrary sitelib;
extern int cancel_count;
extern int dmatch;

extern struct TCPIPMsgHost ros;
extern struct TCPIPMsgHost errlog;
extern struct TCPIPMsgHost shell;
extern struct TCPIPMsgHost task[4];
extern int baseport;

#endif
