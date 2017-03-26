CC=g++
DECAINC=decadriver
PLATFORMINC=platform
CFLAGS=-I$(DECAINC) -I$(PLATFORMINC) -lwiringPi -lpigpio -lpigpiod_if2 -lpthread -std=c++11

DECA=decadriver/deca_device.c decadriver/deca_params_init.c
PLATFORM=platform/raspi_init.c platform/deca_spi.c platform/deca_mutex.c platform/deca_sleep.c

SRCS=$(wildcard *.c)
PROGS=$(patsubst %.c,%,$(SRCS))

all: $(PROGS)

%: %.c
	$(CC) $< $(DECA) $(PLATFORM) $(CFLAGS) -o bin/$@
