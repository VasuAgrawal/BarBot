/**
 * @file    raspi_init.c
 * @brief   Initialization of platform-specific Decawave functionality
 *
 * This file contains the platform-specific initialization and management of
 * interrupt functionality for the Decawave chip on the Raspberry Pi.
 *
 * @author Vivek Sridhar <vivek4830@gmail.com>
 */

#include <stdlib.h>
#include <stdio.h>
#include <pigpiod_if2.h>
#include <pigpio.h>
#include <deca_device_api.h>

#include "deca_spi.h"
#include "raspi_init.h"

/** @brief Flag for masking interrupts - used in decamutex */
int DECA_MUTEX_FLAG;

/**
 * @function void DECAWAVE_RPI_ISR()
 * @brief Interrupt handler for the Decawave chip
 */
void DECAWAVE_RPI_ISR(int pi, unsigned user_gpio, unsigned level, uint32_t tick) {
    printf("isr\n");
    if (DECA_MUTEX_FLAG == 1) {
        dwt_isr();
    }
}

/**
 * @function void raspiDecawaveInit()
 * @brief Initializes platform-specific hardware for the Decawave
 */
void raspiDecawaveInit() {
    // Set up the pigpio library
    if (gpioInitialise() < 0) {
        printf("Failed to initialize pigpio\n");
        exit(EXIT_FAILURE);
    }

    // Initialize SPI
    if (openspi() != 0) {
        printf("Failed to initialize SPI!\n");
        exit(EXIT_FAILURE);
    }

    // Set up the Decawave interrupt handler
    // Defaults to interrupt on a rising edge (Decawave chip IRQ polarity is
    // default active high)
    DECA_MUTEX_FLAG = 1; // Interrupts "enabled"
    if (callback(pigpio_start(NULL, NULL), DWM_INTERRUPT_PIN, RISING_EDGE, DECAWAVE_RPI_ISR) < 0) {
        printf("Failed to initialize callback!\n");
        exit(EXIT_FAILURE);
    }
    //wiringPiISR(DWM_INTERRUPT_PIN, INT_EDGE_RISING, DECAWAVE_RPI_ISR);
}
