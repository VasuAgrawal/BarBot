#ifndef _SPI_H_
#define _SPI_H_

#include <stdint.h>
#include <wiringPiSPI.h>

#include "constants.h"

#define MAX_PACKET_SIZE 128

typedef struct spi_packet {
	uint8_t header;
	uint8_t message[MAX_PACKET_SIZE];
} spi_packet_t;

void readRegister(uint8_t channel, uint8_t reg, uint8_t bytes, spi_packet_t* packet);
void writeRegister(uint8_t channel, uint8_t reg, uint8_t bytes, spi_packet_t* packet);
void transmitMessage(uint8_t bytes, uint8_t* buf);
void receiveMessage(uint8_t bytes, uint8_t* buf);

#endif /* _SPI_H_ */