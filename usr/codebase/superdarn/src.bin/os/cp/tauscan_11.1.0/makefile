# Makefile for tauscan_11
# =======================
# by R.J.Barnes
#
#

include $(MAKECFG).$(SYSTEM)



INCLUDE=-I$(IPATH)/base -I$(IPATH)/general -I$(IPATH)/superdarn \
        -I$(USR_IPATH)/superdarn
OBJS = tauscan_11.o
SRC=tauscan_11.c
DSTPATH = $(USR_BINPATH)
OUTPUT = tauscan_11
LIBS= -lsite.1 -lradarshell.1 -lshmem.1 -lops.1 -lrmsgsnd.1 \
      -ltcpipmsg.1 -lerrlog.1 -lfreq.1 -lacfex.1 -lacf.1 -lfit.1 \
      -lraw.1 -lcfit.1 -lrscan.1 -lfitacf.1 -lfitacfex.1 -liqdata.1 \
      -ltsg.1 -lradar.1 -ldmap.1 -lopt.1 -lrtime.1 -lrcnv.1 -lelevation.1

LFLAGS=-rdynamic

ifeq ($(SYSTEM),linux)
  SLIB=-lm -lrt -lz -ldl
else
  SLIB=-lm -lz -ldl
endif

include $(MAKEBIN).$(SYSTEM)
