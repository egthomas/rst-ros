# Makefile for mktsg
# ==================
# Author: R.J.Barnes

include $(MAKECFG).$(SYSTEM)

INCLUDE=-I$(IPATH)/base -I$(IPATH)/general -I$(IPATH)/superdarn
OBJS = mktsg.o 
SRC=hlpstr.h errstr.h mktsg.c

DSTPATH = $(BINPATH)
OUTPUT = mktsg
LIBS=-ltsg.1 -lopt.1
SLIB=-lm

include $(MAKEBIN).$(SYSTEM)

