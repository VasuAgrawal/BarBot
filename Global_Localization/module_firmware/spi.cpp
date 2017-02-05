#include "spi.h"
#include "constants.h"

/** 
 * @function readRegister
 * @brief Reads bytes from a register
 * @param channel  The SPI channel to read from
 * @param reg      The register address to read from
 * @param bytes    Number of bytes to read
 * @param packet   SPI packet to transmit
 *
 * Does not require more than 1 octet header because this application will
 * not use sub-indexing
 */
void readRegister(uint8_t channel, uint8_t reg, uint8_t bytes, spi_packet_t* packet) {
	packet->header = 0;
	packet->header |= reg; // Register to read from

    // SPI transaction
    wiringPiSPIDataRW(channel, packet->header, bytes + 1);
}

/**
 * @function writeRegister
 * @brief Writes bytes to a register
 * @param channel	The SPI channel to write to
 * @param reg 		The register address to write to
 * @param bytes		Number of bytes to write
 * @param packet 	SPI packet to transmit
 *
 * Does not require more than 1 octet header because this application will
 * not use sub-indexing
 */
void writeRegister(uint8_t channel, uint8_t reg, uint8_t bytes, spi_packet_t* packet) {
	packet->header = 0;
	packet->header |= reg; // Register to write to
	packet->header |= DWM_HEADER_WRITE; // Write flag

	// SPI transaction
	wiringPiSPIDataRW(channel, packet->header, bytes + 1);
}

/**
 * @function transmitMessage
 * @brief Transmits a message from the Decawave chip
 * @param channel	The SPI channel the chip is on
 * @param bytes		Number of bytes in the message
 * @param buf 		The message to transmit
 */
void transmitMessage(uint8_t channel, uint8_t bytes, uint8_t* buf) {
	spi_packet_t tx_config;
	spi_packet_t tx_start;
	spi_packet_t tx_message;

	// Configuration information for transmission
	readRegister(channel, DWM_REG_TX_FCTRL, 5, tx_config);
	tx_config.message[0] = bytes;

	// Configuration to start transmission
	readRegister(channel, DWM_REG_SYS_CTRL, 4, tx_start);
	tx_start.message[0] |= DWM_CTRL_TXSTRT;

	// Load the TX buffer
	for (int i = 0; i < bytes; i++) {
		tx_message.message[i] = buf[i];
	}
	writeRegister(channel, DWM_REG_TX_BUFFER, bytes, tx_message);
	// Write the TX configuration
	writeRegister(channel, DWM_REG_TX_FCTRL, 5, tx_config);
	// Start the transmission
	writeRegister(channel, DWM_REG_SYS_CTRL, 4, tx_start);
}

/**
 * @function receiveMessage
 * @brief Transmits a message from the Decawave chip
 * @param channel	The SPI channel the chip is on
 * @param bytes		Number of bytes in the message
 * @param buf 		The message to transmit
 */
void receiveMessage(uint8_t channel, uint8_t bytes, uint8_t* buf) {
	spi_packet_t rx_message;

	readRegister(channel, DWM_REG_RX_BUFFER, bytes, rx_message);

	for (int i = 0; i < bytes; i++) {
		buf[i] = rx_message.message[i];
	}
}