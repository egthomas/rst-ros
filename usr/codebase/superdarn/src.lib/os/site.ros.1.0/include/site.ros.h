/* site.ros.h
   ==========
   Author: R.J.Barnes
*/


#ifndef _SITEROS_H
#define _SITEROS_H

int SiteRosStart(char *host,char *ststr);
int SiteRosSetupRadar();
int SiteRosStartScan();
int SiteRosStartIntt(int intsc,int intus);
int SiteRosFCLR(int stfreq,int edfreq);
int SiteRosTimeSeq(int *ptab);
int SiteRosIntegrate(int (*lags)[2]);
int SiteRosEndScan(int bsc,int bus, unsigned sleepus);
void SiteRosExit(int signum);

#endif

