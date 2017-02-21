/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Simple RX example code
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <stdio.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <raspi_init.h>

/* Example application name and version to display on LCD screen. */
#define APP_NAME "SIMPLE RX v1.2"

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Buffer to store received frame. See NOTE 1 below. */
#define FRAME_LEN_MAX 127
static uint8 rx_buffer[FRAME_LEN_MAX];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16 frame_len = 0;

void rxErrorISR(const dwt_cb_data_t *cbData) {
    printf("Error in receiving frame\n");
    /* Clear RX error events in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
}

void rxGoodISR(const dwt_cb_data_t *cbData) {
    printf("Frame received\n");
    /* A frame has been received, copy it to our local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
    if (frame_len <= FRAME_LEN_MAX)
    {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    /* Clear good RX frame event in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);


    /* Clear local RX buffer to avoid having leftovers from previous receptions  This is not necessary but is included here to aid reading
     * the RX buffer.
     * This is a good place to put a breakpoint. Here (after first time through the loop) the local status register will be set for last event
     * and if a good receive has happened the data buffer will have the data in it, and frame_len will be set to the length of the RX frame. */
    for (i = 0 ; i < FRAME_LEN_MAX; i++ )
    {
        rx_buffer[i] = 0;
    }

    /* Activate reception immediately. See NOTE 3 below. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/**
 * Application entry point.
 */
int main(void)
{
    // Initialize SPI
    raspiDecawaveInit();

    /* Initialize */
    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
    {
        printf("DWM1000: Initialization Failed!\n");
        while (1)
        { };
    }
    printf("DWM1000: Initialization Complete\n");
    
    /* Configure DW1000. See NOTE 3 below. */
    dwt_configure(&config);
    printf("DWM1000: Configuration Complete\n");

    int deviceId = dwt_readdevid();
    printf("DWM1000: Device ID %x\n", deviceId);

    /* Set up interrupt handlers */
    dwt_setcallbacks(NULL, rxGoodIST, NULL, rxErrorISR);

    /* Activate reception immediately. See NOTE 3 below. */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* Loop forever receiving frames. */
    while (1);
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. In this example, maximum frame length is set to 127 bytes which is 802.15.4 UWB standard maximum frame length. DW1000 supports an extended
 *    frame length (up to 1023 bytes long) mode which is not used in this example.
 * 2. In this example, LDE microcode is not loaded upon calling dwt_initialise(). This will prevent the IC from generating an RX timestamp. If
 *    time-stamping is required, DWT_LOADUCODE parameter should be used. See two-way ranging examples (e.g. examples 5a/5b).
 * 3. Manual reception activation is performed here but DW1000 offers several features that can be used to handle more complex scenarios or to
 *    optimise system's overall performance (e.g. timeout after a given time, automatic re-enabling of reception in case of errors, etc.).
 * 4. We use polled mode of operation here to keep the example as simple as possible but RXFCG and error/timeout status events can be used to generate
 *    interrupts. Please refer to DW1000 User Manual for more details on "interrupts".
 * 5. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *    DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/
