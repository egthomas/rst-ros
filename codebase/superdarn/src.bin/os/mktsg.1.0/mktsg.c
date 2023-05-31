/* mktsg.c
   =======
   Author: R.J.Barnes
*/

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include "rtypes.h"
#include "option.h"
#include "tsg.h"
#include "maketsg.h"

#include "errstr.h"
#include "hlpstr.h"

struct OptionData opt;

int rst_opterr(char *txt) {
  fprintf(stderr,"Option not recognized: %s\n",txt);
  fprintf(stderr,"Please try: mktsg --help\n");
  return(-1);
}

int main(int argc,char *argv[]) {

  int arg;
  unsigned char help=0;
  unsigned char option=0;
  unsigned char version=0;

  int i=0,n=0;
  int nrang=75;
  int frang=180;
  int rsep=45;
  int smsep=300;
  int mpinc=1500;
  int mppul=8;

  int flag;

  int *pcode;
  int nbaud=1;
  int txpl;
  int old=0;
  int longpulse=0;
  int tauscan11=0;
  int tauscan13=0;

  int *bcode;
  int bcode1[1]={1};
  int bcode2[2]={1,-1};
  int bcode3[3]={1,1,-1};
  int bcode4[4]={1,1,-1,1};
  int bcode5[5]={1,1,1,-1,1};
  int bcode7[7]={1,1,1,-1,-1,1,-1};
  int bcode11[11]={1,1,1,-1,-1,-1,1,-1,-1,1,-1};
  int bcode13[13]={1,1,1,1,1,-1,-1,1,1,-1,1,-1,1};

  int patn_old[7]   = {0,9,12,20,22,26,27};
  int patn_short[8] = {0,14,22,24,27,31,42,43};
  int patn_long[16] = {0,4,19,42,78,127,191,270,364,474,600,745,905,1083,1280,1495};
  int patn_tau11[11]  = {0,10,13,14,19,21,31,33,38,39,42};
  int patn_tau13[13]  = {0,15,16,23,27,29,32,47,50,52,56,63,64};

  struct TSGprm prm;
  struct TSGbuf *buf=NULL;

  OptionAdd(&opt, "-help",     'x',&help);
  OptionAdd(&opt, "-option",   'x',&option);
  OptionAdd(&opt, "-version",  'x',&version);

  OptionAdd(&opt, "tauscan11",   'x', &tauscan11);
  OptionAdd(&opt, "tauscan13",   'x', &tauscan13);
  OptionAdd(&opt, "longpulse", 'x', &longpulse);
  OptionAdd(&opt, "old",       'x', &old);

  OptionAdd(&opt, "baud",      'i', &nbaud);
  OptionAdd(&opt, "rsep",      'i', &rsep);
  OptionAdd(&opt, "nrang",     'i', &nrang);
  OptionAdd(&opt, "frang",     'i', &frang);
  OptionAdd(&opt, "mpinc",     'i', &mpinc);

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

  if (tauscan11) {
    mpinc = 3000;
    mppul = 11;
  } else if (tauscan13) {
    mpinc = 2400;
    mppul = 13;
  } else if (longpulse) {
    nrang = 300;
    rsep  = 15;
    smsep = 100;
    mpinc = 100;
    mppul = 16;
  } else if (old) {
    mpinc = 2400;
    mppul = 7;
  }

  /* Re-process the command line options for custom nrang, rsep, etc */
  arg=OptionProcess(1,argc,argv,&opt,rst_opterr);

  switch (nbaud) {
    case  1: bcode = bcode1;  break;
    case  2: bcode = bcode2;  break;
    case  3: bcode = bcode3;  break;
    case  4: bcode = bcode4;  break;
    case  5: bcode = bcode5;  break;
    case  7: bcode = bcode7;  break;
    case 11: bcode = bcode11; break;
    case 13: bcode = bcode13; break;
    default:
      fprintf(stderr,"Invalid nbaud: %d  (1,2,3,4,5,7,11,13)\n",nbaud);
      exit(-1);
  }

  pcode = (int *)malloc((size_t)sizeof(int)*mppul*nbaud);
  for (i=0; i<mppul; i++) {
    for (n=0; n<nbaud; n++) {
      pcode[i*nbaud+n] = bcode[n];
    }
  }

  txpl = (nbaud*rsep*20)/3;

  prm.nrang   = nrang;
  prm.frang   = frang;
  prm.rtoxmin = 0;
  prm.stdelay = 18+2;
  prm.gort    = 1;
  prm.rsep    = rsep;
  prm.smsep   = smsep;
  prm.txpl    = txpl;
  prm.mpinc   = mpinc;
  prm.mppul   = mppul;
  prm.mlag    = 0;
  prm.smdelay = 0;
  prm.nbaud   = nbaud;
  prm.code    = pcode;
  prm.pat     = malloc(sizeof(int)*prm.mppul);

  if (tauscan11) {
    for (i=0;i<prm.mppul;i++) prm.pat[i] = patn_tau11[i];
  } else if (tauscan13) {
    for (i=0;i<prm.mppul;i++) prm.pat[i] = patn_tau13[i];
  } else if (longpulse) {
    for (i=0;i<prm.mppul;i++) prm.pat[i] = patn_long[i];
  } else if (old) {
    for (i=0;i<prm.mppul;i++) prm.pat[i] = patn_old[i];
  } else {
    for (i=0;i<prm.mppul;i++) prm.pat[i] = patn_short[i];
  }

  buf=TSGMake(&prm,&flag);

  if (buf==NULL) {
    fprintf(stderr,"TSGMake Error: %d",flag);
    switch (flag) {
      case  1: fprintf(stderr," (TSG_INV_RSEP)\n"); break;
      case  2: fprintf(stderr," (TSG_NO_SMSEP)\n"); break;
      case  3: fprintf(stderr," (TSG_INV_MPPUL_SMSEP)\n"); break;
      case  4: fprintf(stderr," (TSG_INV_PAT)\n"); break;
      case  5: fprintf(stderr," (TSG_INV_MPINC_SMSEP)\n"); break;
      case  6: fprintf(stderr," (TSG_INV_LAGFR_SMSEP)\n"); break;
      case  7: fprintf(stderr," (TSG_INV_DUTY_CYCLE)\n"); break;
      case  8: fprintf(stderr," (TSG_INV_ODD_SMSEP)\n"); break;
      case  9: fprintf(stderr," (TSG_INV_TXPL_BAUD)\n"); break;
      case 10: fprintf(stderr," (TSG_INV_MEMORY)\n"); break;
      case 11: fprintf(stderr," (TSG_INV_PHASE_DELAY)\n"); break;
      default: fprintf(stderr,"\n"); break;
    }
    exit(-1);
  }

  for (i=0;i<buf->len;i++) {
    if (buf->code[i] & 0x80) fprintf(stdout,"*");
    else fprintf(stdout," ");
    if (buf->code[i] & 0x01) fprintf(stdout,"s");
    if (buf->code[i] & 0x02) fprintf(stdout,"r");
    if (buf->code[i] & 0x04) fprintf(stdout,"x");
    if (buf->code[i] & 0x08) fprintf(stdout,"a");
    if (buf->code[i] & 0x10) fprintf(stdout,"p");
    if ((buf->code[i] & 0x7f)==0) fprintf(stdout,"z");
    fprintf(stdout," %d\n",buf->rep[i]);
  }

  return 0;
}
