/**
 * @file	deca_spi.c
 * @brief	SPI access functions
 *
 * Hardware-specific SPI access functions for the Raspberry Pi platform
 *
 * @author  Vivek Sridhar <vivek4830@gmail.com>
 */

#include <stdint.h>
#include <pigpiod_if2.h>
#include <deca_device_api.h>
#include <stdio.h>

#include "deca_spi.h"

#define DWM_SPI_CS          0
#define DWM_SPI_FREQUENCY   500000

int spiHandle;

extern int piHandle;

/**
 * @function openspi()
 * @brief Initializes the Decawave SPI channel, default channel 0
 *
 * @return 0 on success, -1 on error
 */
int openspi()
{
    int spiconfig = 0;
    spiconfig |= (1 << 8); // Configure it to use spi1

    spiHandle = spi_open(piHandle, DWM_SPI_CS, DWM_SPI_FREQUENCY, spiconfig);

    if (spiHandle < 0) {
        return -1;
    }
    return 0;
}

/**
 * @function closespi()
 * @brief Closes the Decawave SPI channel
 *
 * @return 0 on success, -1 on error
 */
int closespi(void)
{
	return 0;
}

/**
 * @function int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodyLength, const uint8 *bodyBuffer))
 * @brief Abstraction for Decawave SPI write
 * @param headerLength  Number of bytes in header
 * @param headerBuffer  Buffer that stores the SPI transaction header
 * @param bodyLength    Number of bytes in message body
 * @param bodyBuffer    Buffer that stores the SPI message body
 *
 * Hardware-specific write function for SPI on the Raspberry Pi platform. Takes
 * separate byte buffers for header and data.
 *
 * @return 0 on success, -1 on error
 */
#pragma GCC optimize ("O3")
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodyLength, const uint8 *bodyBuffer)
{
    int i;
    
    decaIrqStatus_t s = decamutexon();

    // Construct SPI buffer
    char spi_buf[headerLength + bodyLength];
    for (i = 0; i < headerLength; i++) {
        spi_buf[i] = headerBuffer[i];
    }
    for (i = 0; i < bodyLength; i++) {
        spi_buf[i + headerLength] = bodyBuffer[i];
    }

    // Perform SPI transaction
    spi_write(piHandle, spiHandle, spi_buf, headerLength + bodyLength);

    decamutexoff(s);

    return 0;
}

/**
 * @function int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readLength, uint8 *readBuffer)
 * @brief Abstraction for Decawave SPI read
 * @param headerLength  Number of bytes in header
 * @param headerBuffer  Buffer that stores the SPI transaction header
 * @param readLength    Number of bytes to read
 * @param readBuffer    Buffer to store the bytes that are read
 *
 * Hardware-specific write function for SPI on the Raspberry Pi platform. Takes
 * separate byte buffers for header and for data to be written to.
 *
 * @return Offset into read buffer where first byte may be found, or -1 on error
 */
#pragma GCC optimize ("O3")
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readLength, uint8 *readBuffer)
{
    int i;
    
    decaIrqStatus_t s = decamutexon();

    // Construct SPI buffer
    char tx_buf[headerLength + readLength];
    for (i = 0; i < headerLength; i++) {
        tx_buf[i] = headerBuffer[i];
    }
    char rx_buf[headerLength + readLength];

    // Perform SPI transaction
    spi_xfer(piHandle, spiHandle, tx_buf, rx_buf, headerLength + readLength);

    // Copy read data back into readBuffer
    for (i = 0; i < readLength; i++) {
        readBuffer[i] = rx_buf[i + headerLength];
    }

    decamutexoff(s);

    return 0;
}
