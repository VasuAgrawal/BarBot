#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

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

#define DWM_CFG_RXAUTR      (1 << 6)    // Byte 4

#define DWM_CTRL_TXSTRT     (1 << 1)    // Byte 1
#define DWM_CTRL_WAIT4RESP  (1 << 7)    // Byte 1
#define DWM_CTRL_RXENAB     (1 << 1)    // Byte 2

#define DWM_MASK_TXFRB      (1 << 4)    // Byte 1
#define DWM_MASK_RXDFR      (1 << 6)    // Byte 2

#define DWM_STATUS_TXFRB    (1 << 4)    // Byte 1
#define DWM_STATUS_TXFRS    (1 << 7)    // Byte 1
#define DWM_STATUS_RXPRD    (1 << 1)    // Byte 2
#define DWM_STATUS_RXPHD    (1 << 4)    // Byte 2
#define DWM_STATUS_RXPHE    (1 << 5)    // Byte 2
#define DWM_STATUS_RXDFR    (1 << 6)    // Byte 2

#endif /* _CONSTANTS_H_ */
