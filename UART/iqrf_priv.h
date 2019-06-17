/*
 * Name: iqrf_priv.h
 * Author: Jakub Parez
 *
 */

#ifndef _IQRF_PRIV_H_
#define _IQRF_PRIV_H_

#include "MKL28Z7.h"

#include <stdint.h>

#include "iqrf.h"

#include "soc_def.h"
#include "res_alloc.h"

/*-----------------------------------------------------------------------------------------------------------+
 +                                             general settings                                              +
 +-----------------------------------------------------------------------------------------------------------*/

// version
#define IQRF_API_MIN_OS_VERSION_SUPPORETED   0x40  // this iqrf api is compatible with iqrf os 4.0 and higher

// general
#define VALID_FLAGS_BIT_SZ      64
#define URGENT_FLAGS_BIT_SZ     64
#define BATTERY_FLAGS_BIT_SZ    64

// RTOS tools - timer
#define IQRF_SPI_TIMER_NAME         "iqrf_spi_timer1"
#define IQRF_SPI_TIMER_PERIOD       ((TickType_t) 200)  // 0.005ms * 200 = 1000
#define IQRF_SPI_TIMER_BLOCK_TIME   ((TickType_t)   0)
#define IQRF_SPI_TIMER_ID           (void*)0

// RTOS evt
//#define IQRF_EVTG_BIT_SUCCESS   	0x1u
//#define IQRF_EVTG_BIT_TIMEOUT   	0x2u

// comm safety measures
#define IQRF_DPA_MESSAGE_RETRIES    1
#define IQRF_SPI_MAX_TRIES_CHECK    10
#define IQRF_SPI_MAX_TRIES_CMD      1

// IQRF DPA - timing
#define IQRF_DPA_POLLCHECK_INTERVAL    10     // 10ms recommended
#define IQRF_DPA_RESPONSE_TIMEOUT      500    // 30ms avg
#define IQRF_DPA_CONFIRMATION_TIMEOUT  800    // 200ms 1st, 30ms next
#define IQRF_DPA_FRC_TIMEOUT           500    // FRC read safety timeout
#define IQRF_DPA_FRC_TIMEOUT2          100    // FRC broadcast safety timeout
#define IQRF_DPA_BOND_TIMEOUT          10000

// IQRF meas time
#define IQRF_MEAS_TIMESPAN_TEMP    1000   // 500ms + delta
#define IQRF_MEAS_TIMESPAN_CO2     8000   // 5000ms + delta
#define IQRF_MEAS_TIMESPAN_DUST    12000  // 10000ms + delta
#define IQRF_MEAS_TIMESPAN_MAX     IQRF_MEAS_TIMESPAN_TEMP    // IQRF_MEAS_TIMESPAN_DUST

#define IQRF_LITTLE_ENDIAN

/*-----------------------------------------------------------------------------------------------------------+
 +                                                   utils                                                   +
 +-----------------------------------------------------------------------------------------------------------*/

#define NUMARGS8(...)                  (sizeof((uint8_t[]){__VA_ARGS__})/sizeof(uint8_t))
#define BITMAP_GET_BIT(m,b,s)         (*(m + (b / s)) &  (uint8_t)(1 << (b % s)))
#define BITMAP_SET_BIT(m,b,s)         (*(m + (b / s)) |= (uint8_t)(1 << (b % s)))
#define LO_BYTE16(x)                  ((uint8_t) ((x) & 0xFF))
#define HI_BYTE16(x)                  ((uint8_t) ((x) >> 8u ))
#define IS_EQUAL_3_AND(x,a,b,c)       ((a == x) && (b == x) && (c == x))
#define IS_EQUAL_4_AND(x,a,b,c,d)     ((a == x) && (b == x) && (c == x) && (d == x))
#define NOT_EQUAL_3_AND(x,a,b,c)      ((a != x) && (b != x) && (c != x))
#define NOT_EQUAL_4_AND(x,a,b,c,d)    ((a != x) && (b != x) && (c != x) && (d != x))
#define __CONVERT_U8_TO_U16(h,l)      ((uint16_t)(h << 8u) | l)
#define __CONVERT_U8_TO_U32(h,a,b,l)  ((uint32_t)(h << 24u) | (uint32_t)(a << 16u) | (uint32_t)(b << 8u) | l)

#ifdef IQRF_LITTLE_ENDIAN
#define CONVERT_U8_TO_U16(h,l)        __CONVERT_U8_TO_U16(h,l)
#define CONVERT_U8_TO_U32(h,a,b,l)    __CONVERT_U8_TO_U32(h,a,b,l)
#else
#define CONVERT_U8_TO_U16(h,l)        __CONVERT_U8_TO_U16(l,h)
#define CONVERT_U8_TO_U32(h,a,b,l)    __CONVERT_U8_TO_U32(l,b,a,h)
#endif

/*-----------------------------------------------------------------------------------------------------------+
 +                                             IQRF UART protocol                                            +
 +-----------------------------------------------------------------------------------------------------------*/
#define IQRF_UART_FLAG      0x7E
#define IQRF_UART_ESCAPE    0x7D
#define IQRF_UART_MAX_LEN   64

/*-----------------------------------------------------------------------------------------------------------+
 +                                             IQRF SPI protocol                                             +
 +-----------------------------------------------------------------------------------------------------------*/
// timing
#define IQRF_SPI_DELAY_BEFORE_MSG   20
#define IQRF_SPI_DELAY_AFTER_MSG    20
#define IQRF_SPI_DELAY_AFTER_CHECK  0
#define IQRF_SPI_DELAY_AFTER_CMD    0
#define IQRF_SPI_DELAY_RETRY_CHECK  0
#define IQRF_SPI_DELAY_RETRY_CMD    10

// DPA utils
#define IQRF_DPA_MSG_EX(a,b,c,d,...)   (uint8_t[]){ a, b, c, d ,__VA_ARGS__ }, NUMARGS8(a, b, c, d, __VA_ARGS__)
#define IQRF_DPA_MSG(a,b,c,d,...)      (uint8_t[]){ a, b, c, d ,__VA_ARGS__ }
// IQRF_DPA_MSG_EX is rather slow, use only for debug and quick tests, IQRF_DPA_MSG is the fnc you want to call

#define IQRF_SPI_HW_ERROR(a)   (a == IQRF_SPI_STATUS_NOT_ACTIVE1 || \
                                a == IQRF_SPI_STATUS_NOT_ACTIVE2 || \
                                a == IQRF_SPI_STATUS_SUSPENDED)

#define IQRF_SPI_WRONG_MODE(a) (a == IQRF_SPI_STATUS_READY_PROG || \
                                a == IQRF_SPI_STATUS_READY_DBG)

#define IQRF_DPA_FOURSOME_EMPTY       (IQRF_DPA_FOURSOME) { NADR : 0, PNUM: 0, PCMD: 0, HWPID: 0 }

#define IQRF_DPA_STATUS_WRONG_RX_LEN  (IQRF_DPA_MESSAGE_STATUS) { uart_result   : IQRF_UART_SUCCESS, \
                                                                  dpa_resp_code : IQRF_DPA_ERROR_FAIL };



/*-----------------------------------------------------------------------------------------------------------+
 +                                           IQRF custom settings                                            +
 +-----------------------------------------------------------------------------------------------------------*/
// IQRF misc
#define IQRF_POLL_DATA_LEN         4
#define IQRF_SLEEP_N_SCALER        2    // 2.097s
#define IQRF_SLEEP_C_SCALER        1    // 1.024s
#define IQRF_PDATA_FRC_LEN         3
#define IQRF_PDATA_FRC_SLEEP_LEN   8

// IQRF custom commands
#define IQRF_CUSTOM_FRC_URGENT   0xFF  // todo
#define IQRF_CUSTOM_FRC_CH_B_L   0xC0  // ch1
#define IQRF_CUSTOM_FRC_CH_B_H   0xC1  // ch1
#define IQRF_CUSTOM_FRC_CH_A_L   0xC2  // ch0
#define IQRF_CUSTOM_FRC_CH_A_H   0xC3  // ch0
#define IQRF_CUSTOM_NODE_READ    0x00
#define IQRF_CUSTOM_COORD_SLEEP  0x00

// IQRF custom PDATA
#define IQRF_PDATA_URGENT       IQRF_CUSTOM_FRC_URGENT, 0x00, 0x00
#define IQRF_PDATA_READ_CH_B_L  IQRF_CUSTOM_FRC_CH_B_L, 0x00, 0x00
#define IQRF_PDATA_READ_CH_B_H  IQRF_CUSTOM_FRC_CH_B_H, 0x00, 0x00
#define IQRF_PDATA_READ_CH_A_L  IQRF_CUSTOM_FRC_CH_A_L, 0x00, 0x00
#define IQRF_PDATA_READ_CH_A_H  IQRF_CUSTOM_FRC_CH_A_H, 0x00, 0x00

#define IQRF_PDATA_READ_CH_A_H_AND_SLEEP(s)  0x08, PNUM_OS, CMD_OS_SLEEP, HWPID_DONT_CHECK_LH, \
                                             LO_BYTE16(s), HI_BYTE16(s), 0x02 // sleep[seconds] = s / SCALER

// IQRF read fail return data
#define IQRF_POLL_DATA_ZEROS (IQRF_POLL_DATA) { chan_a: 0, chan_b: 0, naddr: 0, pnum:    0, \
                                                pcmd:   0, hwpid:  0, errn:  0, dpa_val: 0 }


/*-----------------------------------------------------------------------------------------------------------+
 +                                                 IQRF DPA                                                  +
 +-----------------------------------------------------------------------------------------------------------*/
// IQRF DPA - data len
#define IQRF_DPA_FOURSOME_LEN        6
#define IQRF_DPA_DEFAULT_LEN         8
#define IQRF_DPA_MAX_DAT_BLOCK_LEN   56
#define IQRF_DPA_NODES_BITMAP_SIZE   32
#define IQRF_DPA_NODE_INFO_LEN       12
#define IQRF_DPA_OS_INFO_LEN         12
#define IQRF_DPA_PER_ENUM_MIN_LEN    12
#define IQRF_DPA_EMBEDDED_PERS_LEN   4
#define IQRF_DPA_USER_PER_LEN        12
#define IQRF_DPA_CONFIRMATION_LEN    11
#define IQRF_DPA_NOTIFICATION_LEN    IQRF_DPA_FOURSOME_LEN
#define IQRF_DPA_DISCOVERY_INFO_LEN  11
#define IQRF_DPA_ADDR_INFO_LEN       2
#define IQRF_DPA_BOND_NODE_LEN       2
#define IQRF_DPA_REMOVE_BOND_LEN     1
#define IQRF_DPA_BACKUP_NODE_LEN     49
#define IQRF_DPA_DISCOVERY_DATA_LEN  240
#define IQRF_DPA_DISCOVERY_DATA_NOD  48
#define IQRF_DPA_SELECTED_NODES_LEN  31

// IQRF DPA - FRC lens
#define IQRF_DPA_FRC_MAX_LEN         55
#define IQRF_DPA_FRC_EXTRA_LEN       9
#define IQRF_DPA_FRC_MAX_ADDR_2BIT   184
#define IQRF_DPA_FRC_EXTRA_LEN_2BIT  47
#define IQRF_DPA_FRC_2BIT_1ST_START  0
#define IQRF_DPA_FRC_2BIT_1ST_END    29
#define IQRF_DPA_FRC_2BIT_2ND_START  32
#define IQRF_DPA_FRC_2BIT_2ND_END    61

// IQRF DPA - misc
#define IQRF_DPA_TIME_CALC(q,r,t,x)  (((q + 1) * t) + x + ((r + 1) * t))  // x = safety timeout
#define IQRF_DPA_TIME_CALC_EX(a,x)   IQRF_DPA_TIME_CALC(a.hops_out,a.hops_back,a.timeslot,x)
#define IQRF_FRC_OFFLINE_VALUE       0xFF00
#define IQRF_FRC_TIME_CALC(b,d)      (b * 30) + ((d + 2) * 100) + 40 + 1000   // IQRF OS guide (+210)
#define IQRF_FRC_DECODE(h,l)         ((h * 255) + l - 256)

// IQRD DPA - EEPROM addresses
#define IQRF_EEEPROM_ADDR_VRNS       0x5000
#define IQRF_EEEPROM_ADDR_ZONES      0x5200
#define IQRF_EEEPROM_ADDR_PARENTS    0x5300

// IQRF DPA - NADR
#define NADR_DEFAULT_H      0x00
#define NADR_COORDINATOR_L  0x00
#define NADR_NODE_FIRST_L   0x01
#define NADR_NODE_LAST_L    0xEF
#define NADR_TEMPORARY_L    0xFE
#define NADR_BROADCAST_L    0xFF
#define NADR_COORDINATOR_LH NADR_COORDINATOR_L, NADR_DEFAULT_H
#define NADR_NODE_FIRST_LH  NADR_NODE_FIRST_L, NADR_DEFAULT_H
#define NADR_NODE_LAST_LH   NADR_NODE_LAST_L, NADR_DEFAULT_H
#define NADR_TEMPORARY_LH   NADR_TEMPORARY_L, NADR_DEFAULT_H
#define NADR_BROADCAST_LH   NADR_BROADCAST_L, NADR_DEFAULT_H
#define IQRF_MAX_NODES      NADR_NODE_LAST_L

// IQRF DPA - PNUM
#define PNUM_COORDINATOR    0x00
#define PNUM_NODE           0x01
#define PNUM_OS             0x02
#define PNUM_EEPROM         0x03
#define PNUM_EEEPROM        0x04
#define PNUM_RAM            0x05
#define PNUM_LEDR           0x06
#define PNUM_LEDG           0x07
#define PNUM_SPI            0x08
#define PNUM_IO             0x09
#define PNUM_THERMOMETER    0x0A
#define PNUM_PWM            0x0B
#define PNUM_UART           0x0C
#define PNUM_FRC            0x0D
#define PNUM_USER           0x20   // Number of the 1st user peripheral
#define PNUM_USER_MAX       0x3E   // Number of the last user peripheral
#define PNUM_MAX            0x7F   // Maximum peripheral number
#define PNUM_ERROR_FLAG     0xFE
#define PNUM_ENUM           0xFF

// IQRF DPA - command : coordinator
#define CMD_COORDINATOR_ADDR_INFO                  0
#define CMD_COORDINATOR_DISCOVERED_DEVICES         1
#define CMD_COORDINATOR_BONDED_DEVICES             2
#define CMD_COORDINATOR_CLEAR_ALL_BONDS            3
#define CMD_COORDINATOR_BOND_NODE                  4
#define CMD_COORDINATOR_REMOVE_BOND                5
#define CMD_COORDINATOR_REBOND_NODE                6
#define CMD_COORDINATOR_DISCOVERY                  7
#define CMD_COORDINATOR_SET_DPAPARAMS              8
#define CMD_COORDINATOR_SET_HOPS                   9
#define CMD_COORDINATOR_DISCOVERY_DATA             10
#define CMD_COORDINATOR_BACKUP                     11
#define CMD_COORDINATOR_RESTORE                    12
#define CMD_COORDINATOR_READ_REMOTELY_BONDED_MID   15
#define CMD_COORDINATOR_CLEAR_REMOTELY_BONDED_MID  16
#define CMD_COORDINATOR_ENABLE_REMOTE_BONDING      17

// IQRF DPA - command : node
#define CMD_NODE_READ                         0
#define CMD_NODE_REMOVE_BOND                  1
#define CMD_NODE_READ_REMOTELY_BONDED_MID     2
#define CMD_NODE_CLEAR_REMOTELY_BONDED_MID    3
#define CMD_NODE_ENABLE_REMOTE_BONDING        4
#define CMD_NODE_REMOVE_BOND_ADDRESS          5
#define CMD_NODE_BACKUP                       6
#define CMD_NODE_RESTORE                      7

// IQRF DPA - command : OS
#define CMD_OS_READ               0
#define CMD_OS_RESET              1
#define CMD_OS_READ_CFG           2
#define CMD_OS_RFPGM              3
#define CMD_OS_SLEEP              4
#define CMD_OS_BATCH              5
#define CMD_OS_SET_SECURITY       6
#define CMD_OS_RESTART            8
#define CMD_OS_WRITE_CFG_BYTE     9
#define CMD_OS_LOAD_CODE          10
#define CMD_OS_SELECTIVE_BATCH    11
#define CMD_OS_WRITE_CFG          15

// IQRF DPA - command : RAM
#define CMD_RAM_READ         0
#define CMD_RAM_WRITE        1

// IQRF DPA - command : EEPROM
#define CMD_EEPROM_READ      CMD_RAM_READ
#define CMD_EEPROM_WRITE     CMD_RAM_WRITE
#define CMD_EEEPROM_XREAD    ( CMD_RAM_READ  + 2 )
#define CMD_EEEPROM_XWRITE   ( CMD_RAM_WRITE + 2 )

// IQRF DPA - command : LED
#define CMD_LED_SET_OFF      0
#define CMD_LED_SET_ON       1
#define CMD_LED_GET          2
#define CMD_LED_PULSE        3

// IQRF DPA - command : SPI
#define CMD_SPI_WRITE_READ   0

// IQRF DPA - command : IO
#define CMD_IO_DIRECTION     0
#define CMD_IO_SET           1
#define CMD_IO_GET           2

// IQRF DPA - command : misc
#define CMD_THERMOMETER_READ       0
#define CMD_PWM_SET                0
#define CMD_UART_OPEN              0
#define CMD_UART_CLOSE             1
#define CMD_UART_WRITE_READ        2
#define CMD_UART_CLEAR_WRITE_READ  3

// IQRF DPA - command : FRC
#define CMD_FRC_SEND               0
#define CMD_FRC_EXTRARESULT        1
#define CMD_FRC_SEND_SELECTIVE     2
#define CMD_FRC_SET_PARAMS         3
#define CMD_FRC_ACK_BROADCAST      0x81

// IQRF DPA - device exploration
#define CMD_GET_PER_INFO    0x3F

// IQRF DPA - HWPID
#define HWPID_DEFAULT_L         0x00
#define HWPID_DEFAULT_H         0x00
#define HWPID_DEFAULT           0x0000  // No HW Profile specified
#define HWPID_DEFAULT_LH        HWPID_DEFAULT_H, HWPID_DEFAULT_L
#define HWPID_DONT_CHECK_L      0xFF
#define HWPID_DONT_CHECK_H      0xFF
#define HWPID_DONT_CHECK        0xFFFF  // Use this type to override HW Profile ID check
#define HWPID_DONT_CHECK_LH     HWPID_DONT_CHECK_H, HWPID_DONT_CHECK_L

// IQRF DPA - custom HWPID
#define HWPID_CUSTOM_COORDINATOR_L    0x12
#define HWPID_CUSTOM_COORDINATOR_H    0x0F
#define HWPID_CUSTOM_COORDINATOR      0x0F12
#define HWPID_CUSTOM_COORDINATOR_LH   HWPID_CUSTOM_COORDINATOR_L, HWPID_CUSTOM_COORDINATOR_H
#define HWPID_CUSTOM_NODE_FIRST_L     0x12
#define HWPID_CUSTOM_NODE_FIRST_H     0x05
#define HWPID_CUSTOM_NODE_FIRST       0x0512
#define HWPID_CUSTOM_NODE_FIRST_LH    HWPID_CUSTOM_NODE_FIRST_L, HWPID_CUSTOM_NODE_FIRST_H
#define HWPID_CUSTOM_NODE_LAST_L      0x12
#define HWPID_CUSTOM_NODE_LAST_H      0x0C
#define HWPID_CUSTOM_NODE_LAST        0x0C12
#define HWPID_CUSTOM_NODE_LAST_LH     HWPID_CUSTOM_NODE_LAST_L, HWPID_CUSTOM_NODE_LAST_H

// IQRF DPA - peripheral types
#define PERIPHERAL_TYPE_DUMMY         0x00
#define PERIPHERAL_TYPE_COORDINATOR   0x01
#define PERIPHERAL_TYPE_NODE          0x02
#define PERIPHERAL_TYPE_OS            0x03
#define PERIPHERAL_TYPE_EEPROM        0x04
#define PERIPHERAL_TYPE_BLOCK_EEPROM  0x05
#define PERIPHERAL_TYPE_RAM           0x06
#define PERIPHERAL_TYPE_LED           0x07
#define PERIPHERAL_TYPE_SPI           0x08
#define PERIPHERAL_TYPE_IO            0x09
#define PERIPHERAL_TYPE_UART          0x0a
#define PERIPHERAL_TYPE_THERMOMETER   0x0b
#define PERIPHERAL_TYPE_ADC           0x0c
#define PERIPHERAL_TYPE_PWM           0x0d
#define PERIPHERAL_TYPE_FRC           0x0e
#define PERIPHERAL_TYPE_USER_AREA     0x80

// IQRF DPA - custom DPA handler events
#define DPA_EVENT_DPAREQUEST               0
#define DPA_EVENT_INTERRUPT                1
#define DPA_EVENT_IDLE                     2
#define DPA_EVENT_INIT                     3
#define DPA_EVENT_NOTIFICATION             4
#define DPA_EVENT_AFTERROUTING             5
#define DPA_EVENT_BEFORESLEEP              6
#define DPA_EVENT_AFTERSLEEP               7
#define DPA_EVENT_RESET                    8
#define DPA_EVENT_DISABLEINTERRUPTS        9
#define DPA_EVENT_FRCVALUE                 10
#define DPA_EVENT_RECEIVEDPARESPONSE       11
#define DPA_EVENT_IFACERECEIVE             12
#define DPA_EVENT_RECEIVEDPAREQUEST        13
#define DPA_EVENT_BEFORESENDINGDPARESPONSE 14
#define DPA_EVENT_PEERTOPEER               15
#define DPA_EVENT_AUTHORIZEPREBONDING      16
#define DPA_EVENT_USERDPAVALUE             17
#define DPA_EVENT_FRCRESPONSETIME          18
#define DPA_EVENT_BONDINGBUTTON            19

// IQRF DPA - extended peripherals
#define PERIPHERAL_TYPE_EXTENDED_DEFAULT     0b00
#define PERIPHERAL_TYPE_EXTENDED_READ        0b01
#define PERIPHERAL_TYPE_EXTENDED_WRITE       0b10
#define PERIPHERAL_TYPE_EXTENDED_READ_WRITE  ( PERIPHERAL_TYPE_EXTENDED_READ | PERIPHERAL_TYPE_EXTENDED_WRITE )

// IQRF DPA - baud rates
#define DPA_BAUD_1200       0x00
#define DPA_BAUD_2400       0x01
#define DPA_BAUD_4800       0x02
#define DPA_BAUD_9600       0x03
#define DPA_BAUD_19200      0x04
#define DPA_BAUD_38400      0x05
#define DPA_BAUD_57600      0x06
#define DPA_BAUD_115200     0x07
#define DPA_BAUD_230400     0x08

// IQRF DPA - user FRC codes
#define FRC_USER_BIT_FROM   0x40
#define FRC_USER_BIT_TO     0x7F
#define FRC_USER_BYTE_FROM  0xC0
#define FRC_USER_BYTE_TO    0xDF
#define FRC_USER_2BYTE_FROM 0xF0
#define FRC_USER_2BYTE_TO   0xFF


/*-----------------------------------------------------------------------------------------------------------+
 +                                                   enums                                                   +
 +-----------------------------------------------------------------------------------------------------------*/

typedef enum
{
    IQRF_STATE_DIS,
    IQRF_STATE_EN
} IQRF_STATE;

typedef enum
{
    IQRF_DPA_MSG_RESPONSE_ONLY =     0,  // REUEST -> RESPONSE
    IQRF_DPA_MSG_CONFIRM_ONLY =      1,  // REUEST -> CONFIRMATION
    IQRF_DPA_MSG_CONF_AND_RESP =     2   // REUEST -> CONFIRMATION -> RESPONSE
} IQRF_DPA_MSG_MODE;

typedef enum
{
    IQRF_UART_SUCCESS =      0,
    IQRF_UART_BAD_CRC =      1,
    IQRF_UART_TIMEOUT =      2,
	IQRF_UART_GENERAL_ERR =  3
} IQRF_UART_RESULT;

typedef enum
{
    IQRF_DPA_STATUS_NO_ERROR =                   0,      // No error
    IQRF_DPA_ERROR_FAIL =                        1,      // General fail
    IQRF_DPA_ERROR_PCMD =                        2,      // Incorrect PCMD
    IQRF_DPA_ERROR_PNUM =                        3,      // Incorrect PNUM or PCMD
    IQRF_DPA_ERROR_ADDR =                        4,      // Incorrect Address
    IQRF_DPA_ERROR_DATA_LEN =                    5,      // Incorrect Data length
    IQRF_DPA_ERROR_DATA =                        6,      // Incorrect Data
    IQRF_DPA_ERROR_HWPID =                       7,      // Incorrect HW Profile ID used
    IQRF_DPA_ERROR_NADR =                        8,      // Incorrect NADR
    IQRF_DPA_ERROR_IFACE_CUSTOM_HANDLER =        9,      // Data from interface consumed by Custom DPA Handler
    IQRF_DPA_ERROR_MISSING_CUSTOM_DPA_HANDLER =  10,     // Custom DPA Handler is missing
    IQRF_DPA_ERROR_USER_FROM =                   0x20,   // Beginning of the user code error interval
    IQRF_DPA_ERROR_USER_TO =                     0x3F,   // End of the user error code interval
    IQRF_DPA_STATUS_RESERVED_FLAG =              0x40,   // Bit/flag reserved for a future use
    IQRF_DPA_STATUS_ASYNC_RESPONSE =             0x80,   // Bit to flag asynchronous response from [N]
    IQRF_DPA_STATUS_CONFIRMATION =               0xFF,   // Error code used to mark confirmation
    IQRF_DPA_STATUS_NO_RESPONSE =                0xEE    // Node is offline, no response after confirmation
} IQRF_DPA_RESP_CODE;

typedef enum
{
    IQRF_LED_RED =   PNUM_LEDR,
    IQRF_LED_GREEN = PNUM_LEDG
} IQRF_LED;

typedef enum
{
    IQRF_LED_OFF =   CMD_LED_SET_OFF,
    IQRF_LED_ON =    CMD_LED_SET_ON,
    IQRF_LED_PULSE = CMD_LED_PULSE
} IQRF_LED_ACTION;


/*-----------------------------------------------------------------------------------------------------------+
 +                                                  structs                                                  +
 +-----------------------------------------------------------------------------------------------------------*/
// ____ IQRF DPA
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

typedef uint16_t IQRF_NADR;
typedef uint8_t IQRF_NADR_L;  // faster to use this
typedef uint8_t IQRF_PNUM;
typedef uint8_t IQRF_PCMD;
typedef uint16_t IQRF_HWPID;

typedef struct __attribute__ ((packed))
{
    IQRF_UART_RESULT result;
    uint8_t* data;
    uint8_t data_len_in_buff;
} IQRF_CMD_YIELD;

typedef struct __attribute__ ((packed))
{
    IQRF_NADR NADR;
    IQRF_PNUM PNUM;
    IQRF_PCMD PCMD;
    IQRF_HWPID HWPID;
} IQRF_DPA_FOURSOME;

typedef struct __attribute__ ((packed))
{
    //IQRF_SPI_RESULT result;
    IQRF_DPA_MSG_MODE mode;
    uint32_t timeout;
    IQRF_DPA_FOURSOME foursome;
    uint8_t PData_len;
    uint8_t PData[];
} IQRF_DPA_MESSAGE_IN;

//typedef IQRF_DPA_MESSAGE_IN IQRF_DPA_REQUEST;

typedef struct __attribute__ ((packed))
{
	IQRF_UART_RESULT result;
    IQRF_DPA_FOURSOME foursome;
    IQRF_DPA_RESP_CODE resp_code;
    uint8_t DPA_value;  // rssi
    uint8_t hops_out;
    uint8_t timeslot;
    uint8_t hops_back;
} IQRF_DPA_CONFIRMATION;

typedef struct __attribute__ ((packed))
{
    IQRF_UART_RESULT result;
    IQRF_DPA_FOURSOME foursome;
} IQRF_DPA_NOTIFICATION;

typedef struct __attribute__ ((packed))
{
	IQRF_UART_RESULT result;
    IQRF_DPA_FOURSOME foursome;
    IQRF_DPA_RESP_CODE resp_code;
    uint8_t DPA_value;  // rssi
    uint8_t Data_len;
    uint8_t Data[IQRF_DPA_MAX_DAT_BLOCK_LEN];
} IQRF_DPA_RESPONSE;

typedef struct __attribute__ ((packed))
{
    IQRF_DPA_RESP_CODE dpa_resp_code;
    IQRF_UART_RESULT uart_result;
} IQRF_DPA_MESSAGE_STATUS;

typedef struct __attribute__ ((packed))
{
    IQRF_DPA_MSG_MODE mode;
    IQRF_DPA_MESSAGE_STATUS status;
    IQRF_DPA_CONFIRMATION confirmation;
    IQRF_DPA_RESPONSE response;
} IQRF_DPA_MESSAGE_OUT;


// ____ IQRF data
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

typedef struct __attribute__ ((packed))
{
    IQRF_DPA_MESSAGE_STATUS status;
    uint8_t disc_nr;
} IQRF_DPA_DISCOVERY_INFO;

typedef struct __attribute__ ((packed))
{
    IQRF_DPA_MESSAGE_STATUS status;
    // arrays are indexed by node logical address
    uint8_t VRNs[IQRF_DPA_DISCOVERY_DATA_LEN];
    uint8_t Zones[IQRF_DPA_DISCOVERY_DATA_LEN];
    uint8_t Parents[IQRF_DPA_DISCOVERY_DATA_LEN];
} IQRF_DPA_DISCOVERY_DATA;

typedef struct __attribute__ ((packed))
{
    IQRF_DPA_MESSAGE_STATUS status;
    uint8_t ntw_addr;
    uint8_t ntw_vrn;
    uint8_t ntw_zin;
    uint8_t ntw_did;
    uint8_t ntw_pvrn;
    uint8_t ntw_useraddress;
    uint8_t ntw_id;
    uint8_t ntw_vrnfnz;
    uint8_t ntw_cfg;
    uint8_t flags;
} IQRF_DPA_NODE_INFO;

typedef struct __attribute__ ((packed))
{
    IQRF_DPA_MESSAGE_STATUS status;
    uint32_t module_id;
    uint8_t os_version;
    uint8_t mcu_type;
    uint16_t os_build;
    uint8_t rssi;
    uint8_t supply_voltage;
    uint8_t flags;
    uint8_t slot_limits;
} IQRF_DPA_OS_INFO;


/*-----------------------------------------------------------------------------------------------------------+
 +                                              fnc prototypes                                               +
 +-----------------------------------------------------------------------------------------------------------*/
// IQRF API :: main
//static IQRF_DPA_MESSAGE_OUT iqrf_dpa_message_ex(IQRF_DPA_MESSAGE_IN msg);
static IQRF_DPA_MESSAGE_OUT iqrf_dpa_message(IQRF_DPA_MSG_MODE mode,
                                             uint32_t timeout,
                                             const uint8_t* msg,
                                             uint8_t len);

// IQRF API :: wrappers
static IQRF_DPA_MESSAGE_STATUS  iqrf_dpa_get_addressing_information(uint8_t* out_DevNr, uint8_t* out_DID);
static IQRF_DPA_MESSAGE_STATUS  iqrf_dpa_get_discovered_nodes(uint8_t* out_bitmap_32B);
static IQRF_DPA_MESSAGE_STATUS  iqrf_dpa_get_bonded_nodes(uint8_t* out_bitmap_32B);
static IQRF_DPA_MESSAGE_STATUS  iqrf_dpa_led_control(IQRF_NADR_L nadr, IQRF_LED led, IQRF_LED_ACTION action);

static IQRF_DPA_DISCOVERY_INFO  iqrf_dpa_discovery(uint8_t tx_power, uint8_t max_addr, uint32_t timeout);
static IQRF_DPA_DISCOVERY_DATA  iqrf_dpa_discovery_data();  // for network visualization
static IQRF_DPA_NODE_INFO       iqrf_dpa_node_read(IQRF_NADR_L nadr);
static IQRF_DPA_OS_INFO         iqrf_dpa_os_read(IQRF_NADR_L nadr);
static IQRF_DPA_MESSAGE_STATUS  iqrf_dpa_os_reset(IQRF_NADR_L nadr);
static IQRF_DPA_PERIPHERAL_ENUM iqrf_dpa_peripheral_enumeration(IQRF_NADR_L nadr);
static IQRF_DPA_MESSAGE_STATUS  iqrf_dpa_backup(IQRF_NADR_L nadr, uint8_t** out_data_49B, uint8_t* out_len);
static IQRF_DPA_MESSAGE_STATUS  iqrf_dpa_restore(IQRF_NADR_L nadr, uint8_t** data_49B, uint8_t len);
static IQRF_DPA_MESSAGE_STATUS  iqrf_dpa_clear_all_bonds();
static IQRF_DPA_MESSAGE_STATUS  iqrf_dpa_remove_bonded_node(uint8_t BondAddr, uint8_t* out_DevNr);
static IQRF_DPA_MESSAGE_STATUS  iqrf_dpa_bond_node(uint8_t* out_BondAddr, uint8_t* out_DevNr);
static IQRF_DPA_MESSAGE_STATUS  iqrf_dpa_bond_node_ex(uint8_t ReqAddr,
                                                      uint8_t mask,
                                                      uint8_t* out_BondAddr,
                                                      uint8_t* out_DevNr);

// IQRF API :: core utils
inline static IQRF_DPA_CONFIRMATION iqrf_dpa_msg_to_confirmation(IQRF_CMD_YIELD yield);
inline static IQRF_DPA_NOTIFICATION iqrf_dpa_msg_to_notification(IQRF_CMD_YIELD yield);
inline static IQRF_DPA_RESPONSE iqrf_dpa_msg_to_response(IQRF_CMD_YIELD yield);
inline static IQRF_DPA_FOURSOME iqrf_dpa_msg_to_foursome(IQRF_CMD_YIELD yield);
inline static uint8_t iqrf_1wire_crc(const uint8_t* data, uint8_t len, uint8_t initial);
inline static int8_t iqrf_get_available_nodes(uint8_t* available_nodes,
                                              uint8_t* bonded_nodes_bitmap,
                                              uint8_t* discovered_nodes_bitmap,
                                              uint8_t bonded_nodes_count,
                                              IQRF_NADR_L* node_highest_addr);

// IQRF API :: wrappers utils
inline static void iqrf_dpa_msg_struct_to_array(IQRF_DPA_MESSAGE_IN msg, uint8_t* array);
inline static IQRF_DPA_NODE_INFO iqrf_dpa_data_to_node_info(uint8_t* data);
inline static IQRF_DPA_OS_INFO iqrf_dpa_data_to_os_info(uint8_t* data);

// custom tools
/*
static IQRF_DPA_MESSAGE_STATUS iqrf_coordinator_put_sleep(uint16_t seconds);
static IQRF_DPA_MESSAGE_STATUS iqrf_node_put_sleep(IQRF_NADR_L nadr, uint16_t seconds);
static IQRF_DPA_MESSAGE_STATUS iqrf_all_nodes_put_sleep(uint16_t seconds);
*/

#endif /* _IQRF_PRIV_H_ */
