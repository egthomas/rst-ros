/* site.h
   ======
   Author: R.J.Barnes
*/


#ifndef _SITE_H
#define _SITE_H

struct SiteLibrary {
  int (*start)(char *);
  int (*cfgstart)(char *);
  int (*setupradar)();
  int (*startscan)();
  int (*startintt)(int,int);
  int (*fclr)(int,int);
  int (*tmseq)(int *);
  int (*integrate)(int (*lags)[2]);
  int (*endscan)(int,int, unsigned);
  void (*exit)(int);
};

int SiteStart(char *host);
int SiteCfgStart(char *host);
int SiteSetupRadar();
int SiteStartScan();
int SiteStartIntt(int intsc,int intus);
int SiteFCLR(int stfreq,int edfreq);
int SiteTimeSeq(int *ptab);
int SiteIntegrate(int (*lags)[2]);
int SiteEndScan(int bndsc,int bndus, unsigned sleepus);
void SiteExit(int signo);


#endif

