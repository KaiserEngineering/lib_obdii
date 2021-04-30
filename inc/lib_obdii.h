/*-------------------------------------------------------------------*
 *
 * @File: lib_obdii.h
 *
 * @Description:
 *
 * @Author: Matt Kaiser (matt@kaiserengineering.io)
 *
 *-------------------------------------------------------------------*/

#ifndef LIB_OBDII_H_
#define LIB_OBDII_H_

#include "stdio.h"
#include "string.h"
#include "lib_pid.h"
#include "lib_unit_conversion.h"
#include "lib_pid.h"

#define OBDII_MAX_FRAMES 0x04
#define OBDII_DLC        0x08
#define OBDII_MAX_PIDS   25
#define OBDII_MAX_MSGS   8
#define OBDII_RX_BUF_SIZE OBDII_MAX_FRAMES * OBDII_DLC

#define ISO_15765_2_FRAME_TYPE_MASK            0xF0

/**************************************************************
 * The single frame transferred contains the complete payload
 * of up to 7 bytes (normal addressing) or 6 bytes (extended
 * addressing).
 **************************************************************/
#define ISO_15765_2_SINGLE_FRAME               0x00

/**************************************************************
 * The first frame of a longer multi-frame message packet,
 * used when more than 6/7 bytes of data segmented must be
 * communicated. The first frame contains the length of the
 * full packet, along with the initial data.
 **************************************************************/
#define ISO_15765_2_FIRST_FRAME_FRAME          0x10

/**************************************************************
 * A frame containing subsequent data for a multi-frame packet.
 **************************************************************/
#define ISO_15765_2_CONNSECUTIVE_FRAME         0x20

/**************************************************************
 * The response from the receiver, acknowledging a First-frame
 * segment. It lays down the parameters for the transmission
 * of further consecutive frames.
 **************************************************************/
#define ISO_15765_2_FLOW_CONTROL_FRAME         0x30

#define ISO_15765_2_BYTE0_SIZE_MASK            0x0F
#define ISO_15765_2_BYTE1_SIZE_MASK            0xFF

/* A single frame will have the data length as the first byte */
#define CAN_SF_DATA_LEN_POS            0x00

/* A single frame will have the data length as the second byte */
#define CAN_FF_DATA_LEN_POS            0x01

/* Supporting bytes before data: length and mode */
#define CAN_SINGLE_FRAME_SUPPORTING_BYTES          0x02
/* Supporting bytes before data: frame, length and mode */
#define CAN_FIRST_FRAME_SUPPORTING_BYTES           0x03
/* Supporting bytes before data: frame */
#define CAN_CONNSECUTIVE_FRAME_SUPPORTING_BYTES    0x01

#define CAN_FLOW_CONTROL_FRAME_SUPPORTING_BYTES    0x01

#define CAN_SINGLE_FRAME_MODE_POS                  0x01

#define MULTI_FRAME_MODE_POS                       0x02

typedef uint8_t (*TRANSMIT_DATA)(uint8_t *data, uint8_t len);

typedef enum _obdii_packet_manager_status {
    OBDII_PM_CRITICAL_ERROR,
    OBDII_PM_NORMAL,
    OBDII_PM_TIMEOUT,
    OBDII_PM_TX_FAILURE,
    OBDII_PM_NEW_DATA,
    OBDII_PM_IDLE,
    OBDII_PM_ERROR
}OBDII_PACKET_MANAGER_STATUS, *POBDII_PACKET_MANAGER_STATUS;

typedef enum _obdii_process_status {
    OBDII_PID_NOT_SUPPORTED,
    OBDII_CAN_PCKT_MISALIGNED,    /* The received packet does not meet the OBDII standard */
    OBDII_PACKET_PROCESS_SUCCESS
} OBDII_PROCESS_STATUS, *POBDII_PROCESS_STATUS ;

typedef enum _obdii_service {
    OBDII_RESERVED            = 0x00, /* Reserved */
    OBDII_CURRENT_DATA        = 0x01, /* Show current data */
    OBDII_FREEZE_FRAME        = 0x02, /* Show freeze frame data */
    OBDII_SHOW_DTC            = 0x03, /* Show stored Diagnostic Trouble Codes */
    OBDII_CLEAR_DTC           = 0x04, /* Clear Diagnostic Trouble Codes and stored values */
    OBDII_TEST_RESULTS_OXYGEN = 0x05, /* Test results, oxygen sensor monitoring (non CAN only) */
    OBDII_TEST_RESULTS_OTHER  = 0x06, /* Test results, other component/system monitoring */
    OBDII_PENDING_DIAGNOSTIC  = 0x07, /* Show pending Diagnostic Trouble Codes */
    OBDII_CTRL_OPERATION      = 0x08, /* Control operation of on-board component/system */
    OBDII_REQ_VEHICLE_INFO    = 0x09, /* Request vehicle information */
    OBDII_PERMANENT_DTC       = 0x0A  /* Permanent Diagnostic Trouble Codes */
} OBDII_SERVICE, *POBDII_SERVICE ;

typedef enum _obdii_status {
    OBDII_ERROR                  = 0x0,  /* OBDII Error */
    OBDII_OK                     = 0x1,  /* OBDII Okay  */
    OBDII_UNSUPPORTED_CAN_PACKET = 0x2,  /* OBDII CAN packet cannot be interpreted */
    OBDII_PID_REQ_EMPTY          = 0x3,  /* No PIDs were requested */
    OBDII_IDLE                   = 0x4,  /* No PIDs have been requested yet */
    OBDII_MAX_PIDS_REACHED       = 0x5,  /* The max number of PID requests have been reached */
    OBDII_PACKET_GEN_ERROR       = 0x6,
    OBDII_UNSUPPORTED_PID_REQ    = 0x7
} OBDII_STATUS, *POBDII_STATUS;



typedef struct _obdii_diagnostics {

    uint32_t tx_abort_count;
    uint32_t rx_abort_count;
    uint32_t rx_count;
    uint32_t tx_failure;
    uint8_t error;

} OBDII_DIAGNOSTICS, *POBDII_DIAGNOSTICS;

typedef struct _obdii_frame {

    uint8_t buf[OBDII_DLC];

} OBDII_FRAME, *POBDII_FRAME;

typedef struct _obdii_msg {

	uint8_t mode;

	OBDII_FRAME frame[OBDII_MAX_FRAMES];

    uint8_t num_frames;

    uint8_t current_frame;

} OBDII_MSG, *POBDII_MSG;

typedef struct _obdii_init {

    uint32_t timeout;

    TRANSMIT_DATA transmit;

    uint32_t arbitration_ID;       /* Arbitration identifier */

    uint8_t IDE;                    /* Identifier extension bit (IDE) */
        #define OBDII_STD_IDE 0x01  /* Standard 11-bit identifier */
        #define OBDII_EXT_IDE 0x02  /* Extended 29-bit identifier */

} OBDII_INIT, *POBDII_INIT;

typedef struct _obdii_packet_manager {

    uint8_t num_msgs;

    uint8_t current_msg;

	OBDII_MSG msg[OBDII_MAX_MSGS];

    uint32_t obdii_time;

    uint16_t status_flags;
        #define OBDII_PACKET_GENERATED  0x1  /* The OBD-II packet is up to date with the latest PIDs */
        #define OBDII_PENDING_RESPONSE  0x2  /* A CAN packet has been sent and is waiting for a response */
        #define OBDII_RESPONSE_RECEIVED 0x4  /* A sent message received a response */
        #define OBDII_COMM_PAUSE        0x8  /* Stop all communication */
    OBDII_INIT init;

    uint8_t num_pids;

    PTR_PID_DATA stream[OBDII_MAX_PIDS];

    uint8_t rx_buf[OBDII_RX_BUF_SIZE];

    int16_t rx_remaining_bytes;

    uint8_t rx_byte_count;

    OBDII_DIAGNOSTICS diagnostic;

} OBDII_PACKET_MANAGER, *POBDII_PACKET_MANAGER;

/**************************************************************
 * Increment the OBD-II tick timer, this should be called every
 * 1ms.
 *
 * @arguments: void
 *
 * @returns: void
 **************************************************************/
void OBDII_tick( void );


/**************************************************************
 * OBD-II initialize shall be called on boot or reset. No
 * parameter needs to be set for initialization. This function
 * acts as a way to explicitly declare the initial variable
 * conditions.
 *
 * @arguments: &OBDII_PACKET_MANAGER
 *
 * @returns: void
 **************************************************************/
void OBDII_Initialize( POBDII_PACKET_MANAGER dev );


OBDII_STATUS OBDII_add_PID_request( POBDII_PACKET_MANAGER dev, PTR_PID_DATA pid );

OBDII_STATUS OBDII_remove_PID_request( POBDII_PACKET_MANAGER dev, PTR_PID_DATA pid );


/**************************************************************
 * OBD-II service must be called routinely, ideally in the main
 * "while loop". Refer to @OBDII_PACKET_MANAGER_STATUS for
 * possible statuses that can be returned.
 *
 * @arguments: &OBDII_PACKET_MANAGER
 *
 * @returns: OBDII_PACKET_MANAGER_STATUS
 **************************************************************/
OBDII_PACKET_MANAGER_STATUS OBDII_Service( POBDII_PACKET_MANAGER dev );

/**************************************************************
 * Re-enable communication, this only needs to be called if the
 * communication has been paused.
 *
 * @arguments: &OBDII_PACKET_MANAGER
 *
 * @returns: void
 **************************************************************/
void OBDII_Continue( POBDII_PACKET_MANAGER dev );

/**************************************************************
 * Pause the library from requesting data. This is important
 * for when another device is present. This allows the system
 * to save the current PID stream but stop communication.
 *
 * @arguments: &OBDII_PACKET_MANAGER
 *
 * @returns: void
 **************************************************************/
void OBDII_Pause( POBDII_PACKET_MANAGER dev );

OBDII_STATUS OBDII_Add_Packet( POBDII_PACKET_MANAGER dev, uint16_t arbitration_id, uint8_t* packet_data );

float OBDII_Get_Value_Byte_PID( POBDII_PACKET_MANAGER dev, uint16_t pid );

#endif /* LIB_OBDII_H_ */
