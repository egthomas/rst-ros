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
  int mpinc=2400;
  int mppul=8;

  int flag;

  int *pcode;
  int nbaud=1;
  int txpl;
  int longpulse=0;
  int tauscan=0;

  int *bcode;
  int bcode1[1]={1};
  int bcode2[2]={1,-1};
  int bcode3[3]={1,1,-1};
  int bcode4[4]={1,1,-1,1};
  int bcode5[5]={1,1,1,-1,1};
  int bcode7[7]={1,1,1,-1,-1,1,-1};
  int bcode11[11]={1,1,1,-1,-1,-1,1,-1,-1,1,-1};
  int bcode13[13]={1,1,1,1,1,-1,-1,1,1,-1,1,-1,1};

  int patn_short[8] = {0,14,22,24,27,31,42,43};
  int patn_long[16] = {0,4,19,42,78,127,191,270,364,474,600,745,905,1083,1280,1495};
  int patn_tau[11]  = {0,10,13,14,19,21,31,33,38,39,42};

  struct TSGprm prm;
  struct TSGbuf *buf=NULL;

  OptionAdd(&opt, "-help",     'x',&help);
  OptionAdd(&opt, "-option",   'x',&option);
  OptionAdd(&opt, "-version",  'x',&version);

  OptionAdd(&opt, "tauscan",   'x', &tauscan);
  OptionAdd(&opt, "longpulse", 'x', &longpulse);

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

  if (tauscan) {
    mpinc = 3000;
    mppul = 11;
  } else if (longpulse) {
    nrang = 300;
    rsep  = 15;
    smsep = 100;
    mpinc = 100;
    mppul = 16;
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
      fprintf(stderr,"Invalid nbaud: %d\n",nbaud);
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

  if (tauscan) {
    for (i=0;i<prm.mppul;i++) prm.pat[i] = patn_tau[i];
  } else if (longpulse) {
    for (i=0;i<prm.mppul;i++) prm.pat[i] = patn_long[i];
  } else {
    for (i=0;i<prm.mppul;i++) prm.pat[i] = patn_short[i];
  }

  buf=TSGMake(&prm,&flag);
  if (buf==NULL) {
    fprintf(stderr,"TSGMake Error: %d\n",flag);
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
