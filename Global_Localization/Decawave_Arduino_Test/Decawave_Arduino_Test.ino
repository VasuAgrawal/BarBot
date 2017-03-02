#include <SPI.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi.h"

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

/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 *     - byte 10/11: frame check-sum, automatically set by DW1000.  */
static uint8 tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0, 0};
/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 1000

typedef uint64_t uint64;

static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);

void setup() {
  // Pause to allow time for opening serial console
  delay(5000);
  
  // Set up Serial for debugging
  Serial.begin(115200);

  // Set up SPI
  openspi();

  //@TODO: Set up ISR

  // Initialize Decawave Chip
  if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
    Serial.println("DWM1000: Initialization failed!");
    while(1);
  }
  Serial.println("DWM1000: Initialization Complete");

  dwt_configure(&config);
  Serial.println("DWM1000: Configuration Complete");

  int deviceId = dwt_readdevid();
  Serial.print("DWM1000: Device ID ");
  Serial.println(deviceId);
}

void loop() {
  /* Write frame data to DW1000 and prepare transmission. See NOTE 4 below.*/
  dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

  /* Start transmission. */
  dwt_starttx(DWT_START_TX_IMMEDIATE);

  /* Poll DW1000 until TX frame sent event set. See NOTE 5 below.
   * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
   * function to access it.*/
  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
  { };

  uint64 tx_ts = get_tx_timestamp_u64();

  Serial.println("Sent message");
  Serial.print("Timestamp: ");
  Serial.print((unsigned int)tx_ts);

  /* Clear TX frame sent event. */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

  /* Execute a delay between transmissions. */
  deca_sleep(TX_DELAY_MS);

  /* Increment the blink frame sequence number (modulo 256). */
  tx_msg[BLINK_FRAME_SN_IDX]++;
  
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    printf("tx buf: ");
    for (i = 4; i >= 0; i--)
    {
        printf("|%d", ts_tab[i]);
        ts <<= 8;
        ts |= ts_tab[i];
    }
    printf("|\n");
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    printf("rx buf: ");
    for (i = 4; i >= 0; i--)
    {
        printf("|%d", ts_tab[i]);
        ts <<= 8;
        ts |= ts_tab[i];
    }
    printf("|\n");
    return ts;
}
