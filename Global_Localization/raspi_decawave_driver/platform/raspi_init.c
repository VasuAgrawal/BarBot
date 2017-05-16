/*******************************************************************************
 * @file    raspi_init.c                                                       *
 * @brief   Initialization of platform-specific Decawave functionality         *
 *                                                                             *
 * This file contains the platform-specific initialization and management of   *
 * interrupt functionality for the Decawave chip on the Raspberry Pi.          *
 *                                                                             *
 * @author Vivek Sridhar <vivek4830@gmail.com>                                 *
 ******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <pigpiod_if2.h>
#include <deca_device_api.h>

#include "deca_spi.h"
#include "raspi_init.h"

// Flag for masking interrupts - used in decamutex
volatile int DECA_MUTEX_FLAG;

// Handle for the Raspberry Pi
int piHandle;

/**
 * @function void DECAWAVE_RPI_ISR()
 * @brief Interrupt handler for the Decawave chip
 */
void DECAWAVE_RPI_ISR(int pi, unsigned user_gpio, unsigned level, 
                      uint32_t tick) 
{
    if (DECA_MUTEX_FLAG == 1) { dwt_isr(); }
}

/**
 * @function reset_decawave
 * @brief Performs a reset of the Decawave DWM1000 chip
 */
void reset_decawave()
{
    set_mode(piHandle, DWM_RESET_PIN, PI_OUTPUT);
    gpio_write(piHandle, DWM_RESET_PIN, 0);
    deca_sleep(100);
    set_pull_up_down(piHandle, DWM_RESET_PIN, PI_PUD_OFF);
    set_mode(piHandle, DWM_RESET_PIN, PI_INPUT);
}

/**
 * @function void raspiDecawaveInit()
 * @brief Initializes platform-specific hardware for the Decawave
 */
void raspi_decawave_init() 
{
    // Initialize the GPIO
    piHandle = pigpio_start(NULL, NULL);

    reset_decawave();

    // Disable reset pin
    set_mode(piHandle, DWM_RESET_PIN, PI_OUTPUT);
    gpio_write(piHandle, DWM_RESET_PIN, 0);
    deca_sleep(100);
    set_pull_up_down(piHandle, DWM_RESET_PIN, PI_PUD_OFF);
    set_mode(piHandle, DWM_RESET_PIN, PI_INPUT);

    // Initialize SPI
    if (openspi() != 0) {
        printf("Failed to initialize SPI!\n");
        exit(EXIT_FAILURE);
    }

    // Set up the Decawave interrupt handler
    // Defaults to interrupt on a rising edge (Decawave chip IRQ polarity is
    // default active high)
    DECA_MUTEX_FLAG = 1; // Interrupts "enabled"
    if (callback(piHandle, DWM_INTERRUPT_PIN, RISING_EDGE, 
                 DECAWAVE_RPI_ISR) < 0) {
        printf("Failed to initialize IRQ callback!\n");
        exit(EXIT_FAILURE);
    }
}
