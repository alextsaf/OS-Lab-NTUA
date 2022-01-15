###################################################
#
# Makefile
# Simple TCP/IP communication using cryptos
#
# Vangelis Koukis <vkoukis@cslab.ece.ntua.gr>
#
###################################################

CC = gcc

CFLAGS = -Wall
CFLAGS += -g
# CFLAGS += -O2 -fomit-frame-pointer -finline-functions

LIBS = 

BINS = crypto-server crypto-client

all: $(BINS)

crypto-server: crypto-server.c crypto-common.h
	$(CC) $(CFLAGS) -o $@ $< $(LIBS)

crypto-client: crypto-client.c crypto-common.h
	$(CC) $(CFLAGS) -o $@ $< $(LIBS)

clean:
	rm -f *.o *~ $(BINS)
