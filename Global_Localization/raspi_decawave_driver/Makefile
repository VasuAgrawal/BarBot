CC=g++
INC=decadriver
CFLAGS=-I$(INC) -lwiringPi
DECA=decadriver/deca_device.c decadriver/deca_params_init.c decadriver/deca_spi.c decadriver/deca_sleep.c decadriver/deca_mutex.c

SRCS=$(wildcard *.c)
PROGS=$(patsubst %.c,%,$(SRCS))

all: $(PROGS)

%: %.c
	$(CC) $< $(DECA) $(CFLAGS) -o $@
