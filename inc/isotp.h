/*
 * @File: isotp.h
 *
 * @Description: Barebones ISO 15765-2 receive reassembly helpers.
 */

#ifndef ISOTP_H_
#define ISOTP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ISOTP_DLC        0x08
#define ISOTP_MAX_FRAMES 0x04
#define ISOTP_RX_BUF_SIZE (ISOTP_MAX_FRAMES * ISOTP_DLC)

#define ISOTP_FRAME_TYPE_MASK       0xF0
#define ISOTP_SINGLE_FRAME          0x00
#define ISOTP_FIRST_FRAME           0x10
#define ISOTP_CONSECUTIVE_FRAME     0x20
#define ISOTP_FLOW_CONTROL_FRAME    0x30

#define ISOTP_BYTE0_SIZE_MASK       0x0F
#define ISOTP_BYTE1_SIZE_MASK       0xFF

#define ISOTP_SF_DATA_LEN_POS       0x00
#define ISOTP_FF_DATA_LEN_POS       0x01

#define ISOTP_SINGLE_FRAME_DATA_POS      0x01
#define ISOTP_FIRST_FRAME_DATA_POS       0x02
#define ISOTP_CONSECUTIVE_FRAME_DATA_POS 0x01

typedef enum _isotp_status {
    ISOTP_ERROR,
    ISOTP_OK,
    ISOTP_IN_PROGRESS,
    ISOTP_COMPLETE,
    ISOTP_FLOW_CONTROL_REQUIRED,
    ISOTP_UNSUPPORTED_FRAME,
    ISOTP_OVERFLOW,
    ISOTP_SEQUENCE_ERROR
} ISOTP_STATUS;

typedef uint8_t (*ISOTP_TRANSMIT)(uint8_t *data, uint8_t len);

typedef struct _isotp_link {
    uint8_t rx_buf[ISOTP_RX_BUF_SIZE];
    uint16_t rx_len;
    uint16_t rx_expected_len;
    uint8_t next_sequence;
} ISOTP_LINK, *PISOTP_LINK;

void ISOTP_Initialize( PISOTP_LINK link );
void ISOTP_Reset( PISOTP_LINK link );
ISOTP_STATUS ISOTP_Add_Frame( PISOTP_LINK link, const uint8_t *frame, uint8_t len );
uint8_t *ISOTP_Get_Payload( PISOTP_LINK link );
uint16_t ISOTP_Get_Payload_Length( PISOTP_LINK link );
uint8_t ISOTP_Send_Flow_Control( ISOTP_TRANSMIT transmit );
uint8_t is_flow_control_frame( uint8_t* packet_data );

#ifdef __cplusplus
}
#endif

#endif /* ISOTP_H_ */
