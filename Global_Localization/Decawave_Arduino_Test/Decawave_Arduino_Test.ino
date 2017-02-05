#include <SPI.h>

#define MSG_MAX_LEN 32
#define chipSelectPin 10
#define interruptPin 2

#define DWM_REG_DEV_ID      0x00
#define DWM_REG_EUI         0x01
#define DWM_REG_PANADR      0x03
#define DWM_REG_SYS_CFG     0x04
#define DWM_REG_SYS_TIME    0x06
#define DWM_REG_TX_FCTRL    0x08
#define DWM_REG_TX_BUFFER   0x09
#define DWM_REG_DX_TIME     0x0A
#define DWM_REG_RX_FWTO     0x0C
#define DWM_REG_SYS_CTRL    0x0D
#define DWM_REG_SYS_MASK    0x0E
#define DWM_REG_SYS_STATUS  0x0F
#define DWM_REG_RX_FINFO    0x10
#define DWM_REG_RX_BUFFER   0x11
#define DWM_REG_RX_FQUAL    0x12
#define DWM_REG_RX_TTCKI    0x13
#define DWM_REG_RX_TTCKO    0x14
#define DWM_REG_RX_TIME     0x15
#define DWM_REG_TX_TIME     0x17
#define DWM_REG_TX_ANTD     0x18
#define DWM_REG_SYS_STATE   0x19
#define DWM_REG_ACK_RESP_T  0x1A
#define DWM_REG_RX_SNIFF    0x1D
#define DWM_REG_TX_POWER    0x1E
#define DWM_REG_CHAN_CTRL   0x1F
#define DWM_REG_USR_SFD     0x21
#define DWM_REG_AGC_CTRL    0x23
#define DWM_REG_EXT_SYNC    0x24
#define DWM_REG_ACC_MEM     0x25
#define DWM_REG_GPIO_CTRL   0x26
#define DWM_REG_DRX_CONF    0x27
#define DWM_REG_RF_CONF     0x28
#define DWM_REG_TX_CAL      0x2A
#define DWM_REG_FS_CTRL     0x2B
#define DWM_REG_AON         0x2C
#define DWM_REG_OTP_IF      0x2D
#define DWM_REG_LDE_CTRL    0x2E
#define DWM_REG_DIG_DIAG    0x2F
#define DWM_REG_PMSC        0x36

#define DWM_HEADER_WRITE    (1 << 7)

#define DWM_CTRL_TXSTRT     (1 << 1)
#define DWM_CTRL_WAIT4RESP  (1 << 7)
#define DWM_CTRL_RXENAB     (1 << 8)

#define DWM_INT_RXDFR       (1 << 13)

byte msg_buf[MSG_MAX_LEN];

void ISR_receive() {
  Serial.println("message received");
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  // Start the SPI library
  SPI.begin();

  // Initialize chip select pin
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);
  delay(100);

  // Configure chip to generate interrupts on receiving a message
  /*
  byte sys_mask[4];
  ((uint32_t *)sys_mask)[0] |= DWM_INT_RXDFR;
  writeRegister(DWM_REG_SYS_MASK, 4, sys_mask);
  */
  
  // Attach interrupt service routine
  //attachInterrupt(digitalPinToInterrupt(interruptPin), ISR_receive, RISING);

  // Set up test message
  msg_buf[0] = 0xDE;
  msg_buf[1] = 0xAD;
  msg_buf[2] = 0xBE;
  msg_buf[3] = 0xEF;
}

void loop() {
  // Read the device ID from the chip
  readRegister(1, 8, msg_buf);
  for (int i = 0; i < 8; i++) {
    Serial.print(msg_buf[0], HEX);
  }
  Serial.println(' ');
  // Transmit a message from the chip periodically
  transmitMessage(4, msg_buf);
  Serial.println("sent message");
  delay(1000);
}

//@TODO: Currently assuming single octet headers. Expand this.

// Transmit a message
void transmitMessage(byte numBytes, byte buf[]) {
  // Configuration information for transmission
  byte tx_config[5];
  // Assume standard frame length - 127 bytes max
  // Length of message - bits 6-0 of first config byte
  tx_config[0] = numBytes;
  // Everything else can be left as default

  byte sys_ctrl[4];
  readRegister(DWM_REG_SYS_CTRL, 4, sys_ctrl);
  sys_ctrl[0] |= DWM_CTRL_TXSTRT;

  // Load the TX buffer
  writeRegister(DWM_REG_TX_BUFFER, numBytes, buf);
  // Write the TX configuration
  writeRegister(DWM_REG_TX_FCTRL, 5, tx_config);
  // Start the transmission
  writeRegister(DWM_REG_SYS_CTRL, 4, sys_ctrl);
}

// Read a message (assuming a message is ready to be read - this function does not perform that check
void receiveMessage(byte numBytes, byte buf[]) {
  readRegister(DWM_REG_RX_BUFFER, numBytes, buf);
}

// Read bytes from a register into a buffer
void readRegister(byte reg, int numBytes, byte buf[]) {
  byte header = 0;
  header |= reg; // Register to read from

  // SPI transaction
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(header);
  for (int i = 0; i < numBytes; i++) {
    buf[i] = SPI.transfer(0x00);
  }
  digitalWrite(chipSelectPin, HIGH);
}

// Write bytes from a buffer into a register
void writeRegister(byte reg, int numBytes, byte buf[]) {
  byte header = 0;
  header |= reg;
  header |= DWM_HEADER_WRITE;

  // SPI transaction
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(header);
  for (int i = 0; i < numBytes; i++) {
    SPI.transfer(buf[i]);
  }
  digitalWrite(chipSelectPin, HIGH);
}

