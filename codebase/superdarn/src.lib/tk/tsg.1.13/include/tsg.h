/* tsg.h
   =====
   Author: R.J.Barnes
*/

/*
Copyright (c) 2012 The Johns Hopkins University/Applied Physics Laboratory
 
This file is part of the Radar Software Toolkit (RST).

RST is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

Modifications:
*/ 



#ifndef _TSG_H
#define _TSG_H

#define CLOCK_PERIOD	10		/* clock period in microseconds */

struct TSGbuf {
  unsigned char *code;
  unsigned char *rep;
  int len;
};

struct TSGprm {
  int nrang,          /* number of ranges */
      frang,          /* distance to first range */
      rsep,           /* range gate separation */
      smsep,          /* sample separation */
      lagfr,
      txpl,           /* length of pulse */
      mppul,          /* number of pulses in the sequence */
      mpinc,	      /* multi-pulse increment */
      mlag,           /* maximum lag in the sequence */
      nbaud,	      /* number of baud in the phase code */
      samples,	      /* number of samples generated by the sequence */
      smdelay,        /* sample delay */
      stdelay,        /* delay at front of sequence */
      gort;           /* gate or trigger with scope sync*/
  int rtoxmin;        /* delay between receiver off and pulse */
      int *pat,	      /* pointer to the pulse pattern */
	  *code;      /* pointer to the phase code table */
};

struct TSGtable {
  int num;
  int max;
  int *active;
  struct TSGprm *buf;
};

#define TSG_OK 0
#define TSG_INV_RSEP 1
#define TSG_NO_SMSEP 2
#define TSG_INV_MPPUL_SMSEP 3
#define TSG_INV_PAT 4
#define TSG_INV_MPINC_SMSEP 5
#define TSG_INV_LAGFR_SMSEP 6
#define TSG_INV_DUTY_CYCLE 7
#define TSG_INV_ODD_SMSEP 8
#define TSG_INV_TXPL_BAUD 9
#define TSG_INV_MEMORY 10
#define TSG_INV_PHASE_DELAY 11

#endif
