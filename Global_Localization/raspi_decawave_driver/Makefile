CC=g++
DECAINC=decadriver
PLATFORMINC=platform
PROTOINC=../proto
NANOPBINC=../../Networking/proto/nanopb
CFLAGS=-I$(DECAINC) -I$(PLATFORMINC) -I$(PROTOINC) -I$(NANOPBINC) -lpigpiod_if2 -pthread -std=c++11

DECA=decadriver/deca_device.c decadriver/deca_params_init.c
PLATFORM=platform/raspi_init.c platform/deca_spi.c platform/deca_mutex.c platform/deca_sleep.c
PROTO=../proto/dwdistance.pb.c
NANOPB=../../Networking/proto/nanopb/pb_common.c ../../Networking/proto/nanopb/pb_encode.c

#SRCS=$(wildcard *.c)
SRCS=beacon.c
PROGS=$(patsubst %.c,%,$(SRCS))

all: $(PROGS)

%: %.c
	$(CC) $< $(DECA) $(PLATFORM) $(PROTO) $(NANOPB) $(CFLAGS) -o bin/$@

clean:
	rm bin/*
