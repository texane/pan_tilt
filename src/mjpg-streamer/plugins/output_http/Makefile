###############################################################
#
# Purpose: Makefile for "M-JPEG Streamer"
# Author.: Tom Stoeveken (TST)
# Version: 0.3
# License: GPL
#
###############################################################

CC ?= gcc

PT_DIR := ../../../pantilt
CFLAGS += -I$(PT_DIR)

BCM2835_DIR := ../../../bcm2835-1.17
CFLAGS += -I$(BCM2835_DIR)/src

OTHER_HEADERS = ../../mjpg_streamer.h ../../utils.h ../output.h ../input.h

CFLAGS += -O1 -DLINUX -D_GNU_SOURCE -Wall -shared -fPIC
#CFLAGS +=  -g
#CFLAGS += -DDEBUG
LFLAGS += -lpthread -ldl

all: output_http.so

clean:
	rm -f *.a *.o core *~ *.so *.lo

output_http.so: $(OTHER_HEADERS) output_http.c httpd.lo pt.lo bcm2835.lo
	$(CC) $(CFLAGS) -o $@ output_http.c httpd.lo pt.lo bcm2835.lo

httpd.lo: $(OTHER_HEADERS) httpd.h httpd.c
	$(CC) -c $(CFLAGS) -o $@ httpd.c

pt.lo: $(PT_DIR)/pt.c
	$(CC) -c $(CFLAGS) -o $@ $<

bcm2835.lo: $(BCM2835_DIR)/src/bcm2835.c
	$(CC) -c $(CFLAGS) -o $@ $<
