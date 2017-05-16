/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Double-sided two-way ranging (DS TWR) initiator example code
 *
 *           This is a simple code example which acts as the initiator in a DS TWR distance measurement exchange. This application sends a "poll"
 *           frame (recording the TX time-stamp of the poll), and then waits for a "response" message expected from the "DS TWR responder" example
 *           code (companion to this application). When the response is received its RX time-stamp is recorded and we send a "final" message to
 *           complete the exchange. The final message contains all the time-stamps recorded by this application, including the calculated/predicted TX
 *           time-stamp for the final message itself. The companion "DS TWR responder" example application works out the time-of-flight over-the-air
 *           and, thus, the estimated distance between the two devices.
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

#include <deca_device_api.h>
#include <deca_regs.h>
#include <raspi_init.h>
#include <dwdistance.pb.h>
#include <pb_encode.h>

#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

// Macros for output logging level
//#define DEBUG 1
#define INFO 1

// Debug print macro
#ifdef DEBUG
#define DEBUG_PRINT(x) printf x
#else
#define DEBUG_PRINT(x) do {} while(0)
#endif

// Info print macro
#ifdef INFO
#define INFO_PRINT(x) printf x
#else
#define INFO_PRINT(x)
#endif

// Inter-ranging delay period, in milliseconds.
#define RNG_DELAY_MS 20

// Default antenna delay values for 64 MHz PRF.
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

// Longest length frame that this code is supposed to handle
#define RX_BUF_LEN 23

// Message types
#define MSG_TYPE_POLL 0x21
#define MSG_TYPE_RESP 0x10
#define MSG_TYPE_FINAL 0x23
#define MSG_TYPE_MASTER_SWITCH 0x32
#define MSG_TYPE_MODE_SWITCH 0x33

// Indexes to access some of the fields in the frames.
#define FINAL_MSG_POLL_TX_TS_IDX 9
#define FINAL_MSG_RESP_RX_TS_IDX 13
#define FINAL_MSG_FINAL_TX_TS_IDX 17
#define FINAL_MSG_TS_LEN 4
#define MSG_DEST_ADDR_IDX 4
#define MSG_SRC_ADDR_IDX 6
#define MSG_TYPE_IDX 8
#define SWITCH_MSG_TIME_SEC_IDX 9
#define SWITCH_MSG_TIME_USEC_IDX 13

// UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor. 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu.
#define UUS_TO_DWT_TIME 65536

// Speed of light in air, in metres per second.
#define SPEED_OF_LIGHT 299702547

// Frame delays
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
#define RESP_RX_TO_FINAL_TX_DLY_UUS 8000
#define RESP_RX_TIMEOUT_UUS 10000

// Distance regression information
#define X_COEFF 0.962467985748344
#define Y_INTERCEPT 0.0252350208688039

// Number of beacons in the system
#define NUM_BEACONS 7
// Number of tags being localized in the system
#define NUM_TAGS 2

// Default communication configuration. We use here EVK1000's default mode (mode 3).
static dwt_config_t config = {
    5,               // Channel number.
    DWT_PRF_64M,     // Pulse repetition frequency.
    DWT_PLEN_1024,   // Preamble length. Used in TX only.
    DWT_PAC32,       // Preamble acquisition chunk size. Used in RX only.
    9,               // TX preamble code. Used in TX only.
    9,               // RX preamble code. Used in RX only.
    1,               // 0 to use standard SFD, 1 to use non-standard SFD.
    DWT_BR_850K,     // Data rate.
    DWT_PHRMODE_STD, // PHY header mode.
    (1025 + 64 - 32) // SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
};

// Frames used in the ranging process.
static uint8 tx_poll_msg[] = {0x41, 0x88, 0xCA, 0xDE, 'W', 'A', 'V', 'E', MSG_TYPE_POLL, 0, 0};
static uint8 rx_poll_msg[] = {0x41, 0x88, 0xCA, 0xDE, 'W', 'A', 'V', 'E', MSG_TYPE_POLL, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0xCA, 0xDE, 'V', 'E', 'W', 'A', MSG_TYPE_RESP, 0x02, 0, 0, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0xCA, 0xDE, 'V', 'E', 'W', 'A', MSG_TYPE_RESP, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0xCA, 0xDE, 'W', 'A', 'V', 'E', MSG_TYPE_FINAL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 rx_final_msg[] = {0x41, 0x88, 0xCA, 0xDE, 'W', 'A', 'V', 'E', MSG_TYPE_FINAL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 master_switch_msg[] = {0x41, 0x88, 0xCA, 0xDE, 'W', 'A', 'V', 'E', MSG_TYPE_MASTER_SWITCH, 0, 0};
static uint8 mode_switch_msg[] = {0x41, 0x88, 0xCA, 0xDE, 'W', 'A', 'V', 'E', MSG_TYPE_MODE_SWITCH, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Buffer to store received response message.
static uint8 rx_buffer[RX_BUF_LEN];

// Hold copy of status register state here for reference so that it can be examined at a debug breakpoint.
static uint32 status_reg = 0;

// Time-stamps of frames transmission/reception, expressed in device time units.
// As they are 40-bit wide, we need to define a 64-bit int type to handle them.
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;
static uint64 final_rx_ts;

// Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint.
static double tof;
static double distance;

// ID of the current device
static uint8 device_addr;

// Current mode of the system
typedef enum localization_mode {INITIALIZATION, TRACKING} localization_mode;
static localization_mode mode;

// Server information
static int serverfd;
static std::string serverip = "192.168.1.108";
static std::string serverport = "8888";

// Function Declarations
static int compute_distance_init();
static int compute_distance_resp();
void write_to_server(double distance, int send_id, int recv_id, bool beacon);
static int connect_to_server(std::string addr, std::string port);
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
static void switch_msg_set_sys_time();
static void switch_msg_get_sys_time(const uint8 *rx_buf, struct timeval *tv);
static void set_msg_addresses(uint8 master_addr, uint8 slave_addr);
static int validate_frame(uint8* frame, uint8 expected_type);

/**
 * Application entry point.
 */
int main(int argc, char *argv[]) {
	// Read command line arguments
	if (argc != 2) {
		printf("Usage: [ID: 0, 1, 2, etc.]\n");
		exit(EXIT_FAILURE);
	}
	else {
        // Set device address
        device_addr = atoi(argv[1]);
	}

    // Mode starts in initialization
    mode = INITIALIZATION;

    // Start with board specific hardware init.
    raspi_decawave_init();

    // Initialize
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        INFO_PRINT(("DWM1000: Initialization Failed!\n"));
        exit(EXIT_FAILURE);
    }
    INFO_PRINT(("DWM1000: Initialization Complete\n"));
    
    // Configure DW1000.
    dwt_configure(&config);
    INFO_PRINT(("DWM1000: Configuration Complete\n"));

    int deviceId = dwt_readdevid();
    INFO_PRINT(("DWM1000: Device ID %x\n", deviceId));

    // Apply default antenna delay value.
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    // Set up messages with appropriate IDs
    set_msg_addresses(0, device_addr);

    // Use time information to compute how long each message ping took. This
    // allows us to properly calculate how much to delay the next transmission
    // by, so variance in transmission time does not cause messages to overlap
    // as time goes on.
    struct timespec spec;
    long start_ms;
    long end_ms;

    // Loop forever acting as a beacon
    while (1) {
        if (mode == INITIALIZATION) {
        	// Wait for beacons to finish initializing
            compute_distance_resp();
        }
        else if (mode == TRACKING) {
            INFO_PRINT(("Entered tracking mode\n"));
            deca_sleep(100 * (device_addr - NUM_BEACONS));
            while(1) {
                clock_gettime(CLOCK_REALTIME, &spec);
                start_ms = round(spec.tv_nsec / 1.0e6);
                // Regularly ping the beacon network
                for (uint8 target_addr = 0; target_addr < NUM_BEACONS; target_addr++) {
                    set_msg_addresses(device_addr, target_addr);
                    compute_distance_init();
                    deca_sleep(RNG_DELAY_MS);
                }
                clock_gettime(CLOCK_REALTIME, &spec);
                end_ms = round(spec.tv_nsec / 1.0e6);

                // Each round will take RNG_DELAY_MS * NUM_BEACONS
                // We want to give each tag that much time to finish
                deca_sleep(((RNG_DELAY_MS * NUM_BEACONS) * NUM_TAGS) - (end_ms - start_ms));
            }
        }
    }
}

/**
 * @function compute_distance_init
 *
 * Performs a ranging computation of the distance, from the initiating side.
 * Waits for acknowledgment that the exchange has completed before exiting this function.
 *
 * Returns 0 on success, -1 on failure.
 */
static int compute_distance_init() 
{
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    
    // Write frame data to DW1000 and prepare transmission.
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); // Zero offset in TX buffer.
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); // Zero offset in TX buffer, ranging.

    // Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
    // set by dwt_setrxaftertxdelay() has elapsed.
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    // We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout.
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));

    if (status_reg & SYS_STATUS_RXFCG) {
        uint32 frame_len;

        // Clear good RX frame event and TX frame sent in the DW1000 status register.
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

        // A frame has been received, read it into the local buffer.
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
        if (frame_len <= RX_BUF_LEN) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        // Validate the requested type of frame to process appropriately.
        if (validate_frame(rx_buffer, MSG_TYPE_RESP) == 0) {
            uint32 final_tx_time;
            int ret;

            // Retrieve poll transmission and response reception timestamp.
            poll_tx_ts = get_tx_timestamp_u64();
            resp_rx_ts = get_rx_timestamp_u64();

            // Compute final message transmission time.
            final_tx_time = (resp_rx_ts + (2*RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(final_tx_time);

            // Final TX timestamp is the transmission time we programmed plus the TX antenna delay.
            final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

            // Write all timestamps in the final message.
            final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
            final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
            final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

            // Write and send final message.
            dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); // Zero offset in TX buffer.
            dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1); // Zero offset in TX buffer, ranging.
            ret = dwt_starttx(DWT_START_TX_DELAYED);

            // If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one.
            if (ret == DWT_SUCCESS) {
                // Poll DW1000 until TX frame sent event set.
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS));

                // Clear TXFRS event.
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                return 0;
            }
        }
    }
    else {
        // Clear RX error/timeout events in the DW1000 status register.
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        // Reset RX to properly reinitialise LDE operation.
        dwt_rxreset();
    }

    return -1;
}

/**
 * @function compute_distance_resp
 *
 * Performs a ranging computation of the distance, from the receiving side.
 * Sends acknowledgment that the exchange has completed before exiting this function.
 *
 * Returns 0 on success, -1 on failure.
 */
static int compute_distance_resp() 
{
    // Clear reception timeout to start next ranging process.
    dwt_setrxtimeout(5*RESP_RX_TIMEOUT_UUS);

    // Activate reception immediately.
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    // Poll for reception of a frame or error/timeout.
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)));

    if (status_reg & SYS_STATUS_RXFCG) {
        uint32 frame_len;

        // Clear good RX frame event in the DW1000 status register.
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

        // A frame has been received, read it into the local buffer.
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
        if (frame_len <= RX_BUFFER_LEN) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        // Validate the requested type of frame to process appropriately.
        if (validate_frame(rx_buffer, MSG_TYPE_POLL) == 0) {
            int ret;

            // Retrieve poll reception timestamp.
            poll_rx_ts = get_rx_timestamp_u64();

            // Write and send the response message.
            tx_resp_msg[MSG_DEST_ADDR_IDX] = rx_buffer[MSG_SRC_ADDR_IDX];
            tx_resp_msg[MSG_DEST_ADDR_IDX+1] = rx_buffer[MSG_SRC_ADDR_IDX];
            dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); // Zero offset in TX buffer.
            dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); // Zero offset in TX buffer, ranging.

            //@TODO TDMA on beacon side, to respond in parallel to pings from tags
            ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

            // If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one.
            if (ret == DWT_ERROR) {
                INFO_PRINT(("Error transmitting response frame\n"));
                return -1;
            }

            return 0;
        }
        // Check that the frame is a final message sent by "DS TWR initiator" example.
        else if (validate_frame(rx_buffer, MSG_TYPE_FINAL) == 0) {
            uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
            uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
            double Ra, Rb, Da, Db;
            int64 tof_dtu;

            // Retrieve response transmission and final reception timestamps.
            resp_tx_ts = get_tx_timestamp_u64();
            final_rx_ts = get_rx_timestamp_u64();

            // Get timestamps embedded in the final message.
            final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
            final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
            final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);
            
            // Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped.
            poll_rx_ts_32 = (uint32)poll_rx_ts;
            resp_tx_ts_32 = (uint32)resp_tx_ts;
            final_rx_ts_32 = (uint32)final_rx_ts;
            Ra = (double)(resp_rx_ts - poll_tx_ts);
            Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
            Da = (double)(final_tx_ts - resp_rx_ts);
            Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
            tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

            tof = tof_dtu * DWT_TIME_UNITS;
            distance = tof * SPEED_OF_LIGHT;

            // Apply regression
            distance = X_COEFF*distance + Y_INTERCEPT;

#ifdef USING_SERVER
            // Transmit distance to server
            write_to_server(distance, device_addr,
                            rx_buffer[MSG_SRC_ADDR_IDX], true);
            // We must construct two symmetric packets if we are in tracking
            // mode, as the server expects two packets, one marked as a beacon
            // and the other not marked as a beacon.
            // If we are in tracking mode, send the symmetric message.
            if (mode == TRACKING) {
                write_to_server(distance, device_addr,
                                rx_buffer[MSG_SRC_ADDR_IDX], false);
            }
#endif // USING_SERVER

            INFO_PRINT(("%3.2f\n", distance));
            return 0;
        }
        else if (validate_frame(rx_buffer, MSG_TYPE_MODE_SWITCH) == 0) {
            // Switch to tracking mode
            mode = TRACKING; 
            return 0;
        }
    }
    else {
        // Clear RX error/timeout events in the DW1000 status register.
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        // Set RX to properly reinitialise LDE operation.
        dwt_rxreset();
    }

    return -1;
}

void write_to_server(double distance, int send_id, int recv_id, bool beacon) 
{
    // Construct nanopb message
    DwDistance distProto = DwDistance_init_default;
    uint8_t buffer[DwDistance_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    // Fill out fields in proto
    distProto.dist = distance;
    distProto.send_id = device_addr;
    distProto.recv_id = rx_buffer[MSG_SRC_ADDR_IDX];
    distProto.beacon = true;

    // Serialize proto and encode size into header for sending to server
    pb_encode(&stream, DwDistance_fields, &distProto);
    uint32_t byte_size = stream.bytes_written;
    uint8_t *bytes = new uint8_t[4 + byte_size];

    // Little endian encoding of packet size
    bytes[0] = (byte_size >> 0) & 0xFF;
    bytes[1] = (byte_size >> 8) & 0xFF;
    bytes[2] = (byte_size >> 16) & 0xFF;
    bytes[3] = (byte_size >> 24) & 0xFF;

    for (int i = 0; i < byte_size; i++) {
        bytes[i+4] = buffer[i];
    }

    // Attempt to send the array over the socket
    if (write(serverfd, bytes, 4 + byte_size) < 0) {
        INFO_PRINT(("Error in writing to socket!\n"));
    }

    // Clean up
    delete[] bytes;
}

/**
 * @function connect_to_server
 * 
 * Makes one attempt at connecting to the server at the given destination. A
 * failure is indicated with a negative number. Implement some try-again logic
 * above this if you want.
 */
int connect_to_server(std::string addr, std::string port) 
{
  struct addrinfo hints, *listp, *p;
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET; // IPv4
  hints.ai_socktype = SOCK_STREAM; // TCP
  hints.ai_flags = AI_NUMERICSERV;
  hints.ai_flags |= AI_ADDRCONFIG;

  int rv;
  if ((rv = getaddrinfo(addr.c_str(), port.c_str(), &hints, &listp)) != 0) {
    std::cerr << "Error in getaddrinfo: " << gai_strerror(rv) << std::endl;
    freeaddrinfo(listp);
    return -1;
  }

  int fd;
  for (p = listp; ; p = p->ai_next) {
    // Create a socket descriptor
    if ((fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) < 0) {
      std::cerr << "Error in creating socket." << std::endl;
      continue;
    }

    // Attempt to connect to that socket descriptor.
    if (connect(fd, p->ai_addr, p->ai_addrlen) < 0) {
      std::cerr << "Error in connecting to socket." << std::endl;
      close(fd);
      continue;
    }
    
    // If we get here, we know that we've successfully connected probably.
    break;
  }

  // Check to make sure that we don't have a null thing.
  if (p == nullptr) {
    std::cerr << "Failed to connect" << std::endl;
    freeaddrinfo(listp);
    return -2;
  }

  freeaddrinfo(listp);
  return fd;
}

/**
 * @function get_tx_timestamp_u64
 *
 * Get the TX timestamp in a 64-bit variable.
 * This function assumes that the length of timestamps is 40 bits, for both TX and RX
 */
static uint64 get_tx_timestamp_u64(void) 
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/**
 * @function get_rx_timestamp_u64
 *
 * Get the RX timestamp in a 64-bit variable.
 * This function assumes that length of timestamps is 40 bits, for both TX and RX
 */
static uint64 get_rx_timestamp_u64(void) 
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/**
 * @function final_msg_set_tx
 *
 * Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final message,
 * the least significant byte is at the lower address.
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts) 
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

/**
 * @function final_msg_get_ts
 *
 * Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final message,
 * the least significant byte is at the lower address.
 */
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts) 
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
        *ts += ts_field[i] << (i * 8);
    }
}

/**
 * @function switch_msg_set_sys_time
 *
 * Fill the time fields in the switch message with the current system time.
 *
 * @TODO Implement a use for this to synchronize time between beacons and tags
 */
static void switch_msg_set_sys_time() 
{
    int i;
    struct timeval tv;
    gettimeofday(&tv, NULL);

    INFO_PRINT(("Sending time of: %d sec, %d us\n", 
                (int)tv.tv_sec, (int)tv.tv_usec));

    for (i = 0; i < 4; i++) {
        mode_switch_msg[SWITCH_MSG_TIME_SEC_IDX + i] = (uint8)tv.tv_sec;
        mode_switch_msg[SWITCH_MSG_TIME_USEC_IDX + i] = (uint8)tv.tv_usec;

        tv.tv_sec >>= 8;
        tv.tv_usec >>= 8;
    }
}

/**
 * @function switch_msg_get_sys_time
 *
 * Fill a given system time struct with the values from the message.
 * 
 * @TODO Implement a use for this to synchronize time between beacons and tags
 */
static void switch_msg_get_sys_time(const uint8 *rx_buf, struct timeval *tv) 
{
    int i;

    tv->tv_sec = 0;
    tv->tv_usec = 0;

    for (i = 0; i < 4; i++) {
        tv->tv_sec += rx_buf[SWITCH_MSG_TIME_SEC_IDX + i] << (i * 8);
        tv->tv_usec += rx_buf[SWITCH_MSG_TIME_USEC_IDX + i] << (i * 8);
    }
}

/**
 * @function set_msg_addresses
 *
 * Update the addreses of each message when we change targets
 */
static void set_msg_addresses(uint8 master_addr, uint8 slave_addr) 
{
	// Poll message addresses
	tx_poll_msg[MSG_SRC_ADDR_IDX] = master_addr;
	tx_poll_msg[MSG_SRC_ADDR_IDX+1] = master_addr;
	tx_poll_msg[MSG_DEST_ADDR_IDX] = slave_addr;
	tx_poll_msg[MSG_DEST_ADDR_IDX+1] = slave_addr;

	rx_poll_msg[MSG_SRC_ADDR_IDX] = master_addr;
	rx_poll_msg[MSG_SRC_ADDR_IDX+1] = master_addr;
	rx_poll_msg[MSG_DEST_ADDR_IDX] = slave_addr;
	rx_poll_msg[MSG_DEST_ADDR_IDX+1] = slave_addr;

	// Response message addresses
	tx_resp_msg[MSG_SRC_ADDR_IDX] = slave_addr;
	tx_resp_msg[MSG_SRC_ADDR_IDX+1] = slave_addr;
	tx_resp_msg[MSG_DEST_ADDR_IDX] = master_addr;
	tx_resp_msg[MSG_DEST_ADDR_IDX+1] = master_addr;

	rx_resp_msg[MSG_SRC_ADDR_IDX] = slave_addr;
	rx_resp_msg[MSG_SRC_ADDR_IDX+1] = slave_addr;
	rx_resp_msg[MSG_DEST_ADDR_IDX] = master_addr;
	rx_resp_msg[MSG_DEST_ADDR_IDX+1] = master_addr;

	// Final message addresses
	tx_final_msg[MSG_SRC_ADDR_IDX] = master_addr;
	tx_final_msg[MSG_SRC_ADDR_IDX+1] = master_addr;
	tx_final_msg[MSG_DEST_ADDR_IDX] = slave_addr;
	tx_final_msg[MSG_DEST_ADDR_IDX+1] = slave_addr;

	rx_final_msg[MSG_SRC_ADDR_IDX] = master_addr;
	rx_final_msg[MSG_SRC_ADDR_IDX+1] = master_addr;
	rx_final_msg[MSG_DEST_ADDR_IDX] = slave_addr;
	rx_final_msg[MSG_DEST_ADDR_IDX+1] = slave_addr;

    // Switch message addresses
    master_switch_msg[MSG_SRC_ADDR_IDX] = master_addr;
    master_switch_msg[MSG_SRC_ADDR_IDX+1] = master_addr;
    master_switch_msg[MSG_DEST_ADDR_IDX] = slave_addr;
    master_switch_msg[MSG_DEST_ADDR_IDX+1] = slave_addr;
}

/**
 * @function validate_frame
 * Validates the frame against the fixed parameters, as well as the expected 
 * type of frame
 */
static int validate_frame(uint8* frame, uint8 expected_type) 
{
	// Validate frame control bytes
	if ((frame[0] != 0x41) || (frame[1] != 0x88)) {
		return -1;
	}
	/// Validate PAN ID
	if ((frame[2] != 0xCA) || (frame[3] != 0xDE)) {
		return -1;
	}
    // Validate that the message is of the expected type
    if (frame[MSG_TYPE_IDX] != expected_type) {
        return -1;
    }
    // If the type is mode switch, we don't care which device it was intended for, since it was intended for all of them
    if (expected_type == MSG_TYPE_MODE_SWITCH) {
        return 0;
    }
	// Validate that message is intended for this device
	if ((frame[MSG_DEST_ADDR_IDX] != device_addr) || (frame[MSG_DEST_ADDR_IDX+1] != device_addr)) {
		return -1;
	}
	return 0;
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 9 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2/3: PAN ID (0xDECA).
 *     - byte 4/5: destination address, see NOTE 3 below.
 *     - byte 6/7: source address, see NOTE 3 below.
 *     - byte 8: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used here for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 * 4. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 *    data rate used.
 * 5. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 6. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 7. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 8. As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW1000
 *    register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
 *    response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
 *    lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower
 *    8 bits.
 * 9. In this operation, the high order byte of each 40-bit timestamps is discarded. This is acceptable as those time-stamps are not separated by
 *    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays (needed in the
 *    time-of-flight computation) can be handled by a 32-bit subtraction.
 ****************************************************************************************************************************************************/
