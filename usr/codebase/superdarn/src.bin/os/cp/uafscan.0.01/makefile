# Makefile for uafscan
# =======================
# 
#
#

include $(MAKECFG).$(SYSTEM)



INCLUDE=-I$(IPATH)/base -I$(IPATH)/general -I$(IPATH)/superdarn \
        -I$(USR_IPATH)/superdarn
OBJS = uafscan.o
SRC=uafscan.c
DSTPATH = $(USR_BINPATH)
OUTPUT = uafscan
LIBS=-lops.1 -lsite.1 -lradarshell.1 -ltcpipmsg.1 -lrmsgsnd.1 \
     -lerrlog.1 -lfreq.1 -lfit.1 -lraw.1 -lcfit.1 -lrscan.1 \
     -lfitacf.1 -liqdata.1 -lradar.1 -ldmap.1 -lopt.1 -lrtime.1 \
     -lrcnv.1 -ltsg.1 -lelevation.1

LFLAGS=-rdynamic


ifeq ($(SYSTEM),linux)
  SLIB=-lm -lrt -lz -ldl
else
  SLIB=-lm -lz -ldl
endif

include $(MAKEBIN).$(SYSTEM)
