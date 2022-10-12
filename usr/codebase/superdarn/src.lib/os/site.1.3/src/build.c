/* site.c
   ====== 
   Author R.J.Barnes
*/

/*
 (c) 2010 JHU/APL & Others - Please Consult LICENSE.superdarn-rst.3.1-beta-18-gf704e97.txt for more information.
 
 
 
*/


#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>
#include "rtypes.h"
#include "limit.h"
#include "tsg.h"
#include "maketsg.h"
#include "acf.h"
#include "tcpipmsg.h"
#include "rosmsg.h"
#include "shmem.h"
#include "global.h"
#include "site.h"
#include "siteglobal.h"

#include "site.cve.h"
#include "site.cvw.h"

int SiteBuild(int stid) {
  switch (stid) {
     
  case 207:
    sitelib.start=SiteCveStart;
    sitelib.setupradar=SiteCveSetupRadar;
    sitelib.startscan=SiteCveStartScan;
    sitelib.startintt=SiteCveStartIntt;
    sitelib.fclr=SiteCveFCLR;
    sitelib.tmseq=SiteCveTimeSeq;
    sitelib.integrate=SiteCveIntegrate;
    sitelib.endscan=SiteCveEndScan;
    sitelib.exit=SiteCveExit;
    break;
  case 206:
    sitelib.start=SiteCvwStart;
    sitelib.setupradar=SiteCvwSetupRadar;
    sitelib.startscan=SiteCvwStartScan;
    sitelib.startintt=SiteCvwStartIntt;
    sitelib.fclr=SiteCvwFCLR;
    sitelib.tmseq=SiteCvwTimeSeq;
    sitelib.integrate=SiteCvwIntegrate;
    sitelib.endscan=SiteCvwEndScan;
    sitelib.exit=SiteCvwExit;
    break;
  default:
    return -1;
    break;
  }
  return 0;
}





