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

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "isotp.h"
#include "lib_pid.h"

#define OBDII_MAX_FRAMES ISOTP_MAX_FRAMES
#define OBDII_DLC        ISOTP_DLC
#define OBDII_MAX_PIDS   25
#define OBDII_MAX_MSGS   8

#ifndef OBDII_MODE1_PIDS_PER_REQUEST
#define OBDII_MODE1_PIDS_PER_REQUEST  6
#endif

#ifndef OBDII_MODE22_PIDS_PER_REQUEST
#define OBDII_MODE22_PIDS_PER_REQUEST 2
#endif

#ifndef OBDII_DEFAULT_PIDS_PER_REQUEST
#define OBDII_DEFAULT_PIDS_PER_REQUEST OBDII_MAX_PIDS
#endif

typedef uint8_t (*TRANSMIT_OBDII_DATA)(uint8_t *data, uint8_t len);

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

    PTR_PID_DATA pid_filter[OBDII_MAX_PIDS];

    uint8_t num_pid_filters;

	OBDII_FRAME frame[OBDII_MAX_FRAMES];

    uint8_t num_frames;

    uint8_t current_frame;

} OBDII_MSG, *POBDII_MSG;

typedef struct _obdii_init {

    uint32_t timeout;

    TRANSMIT_OBDII_DATA transmit;

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

    volatile uint16_t status_flags;
        #define OBDII_PACKET_GENERATED  0x1  /* The OBD-II packet is up to date with the latest PIDs */
        #define OBDII_PENDING_RESPONSE  0x2  /* A CAN packet has been sent and is waiting for a response */
        #define OBDII_RESPONSE_RECEIVED 0x4  /* A sent message received a response */
        #define OBDII_COMM_PAUSE        0x8  /* Stop all communication */
    OBDII_INIT init;

    uint8_t num_pids;

    PTR_PID_DATA stream[OBDII_MAX_PIDS];

    ISOTP_LINK isotp;

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

OBDII_STATUS OBDII_resync(POBDII_PACKET_MANAGER dev);

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

/*----------------------------------------------------------------------------------*
 * This function performs a strict byte-for-byte comparison of @p packet_data
 * against the library’s FC-CTS template:
 *   { 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
 *
 * Where:
 *   - 0x30 = PCI: Flow Control (0x3 << 4) with FlowStatus = CTS (0x0)
 *   - Byte1 (BS)  = 0x00 → block size (no limit)
 *   - Byte2 (STmin) = 0x00 → minimum separation time
 *   - Remaining bytes = 0x00 (reserved)
 *
 * @param[in]  packet_data  Pointer to an 8-byte CAN payload (DLC == 8). Must not be NULL.
 *
 * @return  1 if the payload matches exactly; 0 otherwise (including NULL pointer).
 *
 * @see ISO 15765-2 (ISO-TP), Flow Control (FC) frame format.
 *----------------------------------------------------------------------------------*/
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

#ifdef __cplusplus
}
#endif

#endif /* LIB_OBDII_H_ */
