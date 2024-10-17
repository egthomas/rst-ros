# Makefile for iq_fix
# ===================
# Author: E.G.Thomas
#
include $(MAKECFG).$(SYSTEM)

INCLUDE=-I$(IPATH)/base -I$(IPATH)/general -I$(IPATH)/superdarn
OBJS = iq_fix.o
SRC=hlpstr.h errstr.h iq_fix.c
DSTPATH = $(BINPATH)
OUTPUT = iq_fix
LIBS=-liqdata.1 -lopt.1 -ldmap.1 -lradar.1 -lrtime.1 -lrcnv.1 
SLIB=-lm -lz
 

include $(MAKEBIN).$(SYSTEM)
