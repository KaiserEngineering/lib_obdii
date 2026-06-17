/*
 * @File: isotp.c
 *
 * @Description: Barebones ISO 15765-2 receive reassembly helpers.
 */

#include <string.h>
#include "isotp.h"

static uint8_t flow_control_frame[ISOTP_DLC] = {
    0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static ISOTP_STATUS append_payload( PISOTP_LINK link, const uint8_t *data, uint8_t len )
{
    if( (link == 0) || (data == 0) )
        return ISOTP_ERROR;

    uint16_t remaining = link->rx_expected_len - link->rx_len;
    uint8_t copy_len = (len > remaining) ? (uint8_t)remaining : len;

    if( (link->rx_len + copy_len) > ISOTP_RX_BUF_SIZE )
        return ISOTP_OVERFLOW;

    memcpy(&link->rx_buf[link->rx_len], data, copy_len);
    link->rx_len += copy_len;

    if( link->rx_len >= link->rx_expected_len )
        return ISOTP_COMPLETE;

    return ISOTP_IN_PROGRESS;
}

void ISOTP_Initialize( PISOTP_LINK link )
{
    ISOTP_Reset(link);
}

void ISOTP_Reset( PISOTP_LINK link )
{
    if( link == 0 )
        return;

    memset(link->rx_buf, 0, ISOTP_RX_BUF_SIZE);
    link->rx_len = 0;
    link->rx_expected_len = 0;
    link->next_sequence = 1;
}

ISOTP_STATUS ISOTP_Add_Frame( PISOTP_LINK link, const uint8_t *frame, uint8_t len )
{
    if( (link == 0) || (frame == 0) || (len < ISOTP_DLC) )
        return ISOTP_ERROR;

    switch( frame[0] & ISOTP_FRAME_TYPE_MASK )
    {
        case ISOTP_SINGLE_FRAME:
        {
            uint8_t payload_len = frame[0] & ISOTP_BYTE0_SIZE_MASK;

            if( payload_len > (ISOTP_DLC - ISOTP_SINGLE_FRAME_DATA_POS) )
                return ISOTP_OVERFLOW;

            ISOTP_Reset(link);
            link->rx_expected_len = payload_len;
            return append_payload(link, &frame[ISOTP_SINGLE_FRAME_DATA_POS], payload_len);
        }

        case ISOTP_FIRST_FRAME:
        {
            uint16_t payload_len = ((uint16_t)(frame[0] & ISOTP_BYTE0_SIZE_MASK) << 8) |
                                   (uint16_t)(frame[ISOTP_FF_DATA_LEN_POS] & ISOTP_BYTE1_SIZE_MASK);

            if( payload_len > ISOTP_RX_BUF_SIZE )
                return ISOTP_OVERFLOW;

            ISOTP_Reset(link);
            link->rx_expected_len = payload_len;

            ISOTP_STATUS status = append_payload(
                link,
                &frame[ISOTP_FIRST_FRAME_DATA_POS],
                ISOTP_DLC - ISOTP_FIRST_FRAME_DATA_POS
            );

            if( status == ISOTP_COMPLETE )
                return ISOTP_COMPLETE;

            return ISOTP_FLOW_CONTROL_REQUIRED;
        }

        case ISOTP_CONSECUTIVE_FRAME:
        {
            uint8_t sequence = frame[0] & ISOTP_BYTE0_SIZE_MASK;

            if( sequence != link->next_sequence )
            {
                ISOTP_Reset(link);
                return ISOTP_SEQUENCE_ERROR;
            }

            link->next_sequence = (link->next_sequence + 1) & ISOTP_BYTE0_SIZE_MASK;

            return append_payload(
                link,
                &frame[ISOTP_CONSECUTIVE_FRAME_DATA_POS],
                ISOTP_DLC - ISOTP_CONSECUTIVE_FRAME_DATA_POS
            );
        }

        case ISOTP_FLOW_CONTROL_FRAME:
            return ISOTP_OK;

        default:
            return ISOTP_UNSUPPORTED_FRAME;
    }
}

uint8_t *ISOTP_Get_Payload( PISOTP_LINK link )
{
    if( link == 0 )
        return 0;

    return link->rx_buf;
}

uint16_t ISOTP_Get_Payload_Length( PISOTP_LINK link )
{
    if( link == 0 )
        return 0;

    return link->rx_len;
}

uint8_t ISOTP_Send_Flow_Control( ISOTP_TRANSMIT transmit )
{
    if( transmit == 0 )
        return 0;

    return transmit(flow_control_frame, ISOTP_DLC);
}

uint8_t is_flow_control_frame( uint8_t* packet_data )
{
    if( packet_data == 0 )
        return 0;

    return (memcmp(packet_data, flow_control_frame, ISOTP_DLC) == 0) ? 1u : 0u;
}
