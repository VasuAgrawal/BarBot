/**
 * @file	deca_spi.c
 * @brief	SPI access functions
 *
 * Hardware-specific SPI access functions for the Raspberry Pi platform
 *
 * @author  Vivek Sridhar <vivek4830@gmail.com>
 */

#include <stdint.h>
#include <SPI.h>
#include "deca_device_api.h"

#include "deca_spi.h"

#define DWM_CHIP_SELECT     7

/**
 * @function openspi()
 * @brief Initializes the Decawave SPI channel, default channel 0
 *
 * @return 0 on success, -1 on error
 */
int openspi()
{
    SPI.begin();
    pinMode(DWM_CHIP_SELECT, OUTPUT);
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
    
    //decaIrqStatus_t s = decamutexon();

    // Construct SPI buffer
    uint8_t spi_buf[headerLength + bodyLength];
    for (i = 0; i < headerLength; i++) {
        spi_buf[i] = headerBuffer[i];
    }
    for (i = 0; i < bodyLength; i++) {
        spi_buf[i + headerLength] = bodyBuffer[i];
    }

    // Perform SPI transaction
    digitalWrite(DWM_CHIP_SELECT, LOW);
	  SPI.transfer(spi_buf, headerLength + bodyLength);
    digitalWrite(DWM_CHIP_SELECT, HIGH);

    //decamutexoff(s);

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
    
    //decaIrqStatus_t s = decamutexon();

    // Construct SPI buffer
    uint8_t spi_buf[headerLength + readLength];
    for (i = 0; i < headerLength; i++) {
        spi_buf[i] = headerBuffer[i];
    }

    // Perform SPI transaction
    digitalWrite(DWM_CHIP_SELECT, LOW);
	  SPI.transfer(spi_buf, headerLength + readLength);
    digitalWrite(DWM_CHIP_SELECT, HIGH);
    
    // Copy read data back into readBuffer
    for (i = 0; i < readLength; i++) {
        readBuffer[i] = spi_buf[i + headerLength];
    }

    //decamutexoff(s);

    return 0;
}
