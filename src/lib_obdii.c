/*-------------------------------------------------------------------*
 *
 * @File: lib_obdii.c
 *
 * @Description:
 *
 * @Author: Matt Kaiser (matt@kaiserengineering.io)
 *
 * ISO 15765-2
 *
 *-------------------------------------------------------------------*/

#include "lib_obdii.h"

#define MODE_NOT_CONFIGURED 0xAA

static void next_byte( uint8_t *frame, uint8_t *cur_byte );
static OBDII_STATUS obdii_generate_PID_Request( POBDII_PACKET_MANAGER dev );
static void clear_obdii_packets( POBDII_PACKET_MANAGER dev );
static void clear_diagnostics( POBDII_PACKET_MANAGER dev );
static void refresh_timeout( POBDII_PACKET_MANAGER dev );
static OBDII_PROCESS_STATUS OBDII_Process_Packet( POBDII_PACKET_MANAGER dev );
static void clear_pid_entries( POBDII_PACKET_MANAGER dev );
static PTR_PID_DATA find_response_pid( POBDII_PACKET_MANAGER dev, uint8_t mode, uint8_t *rx_buf, uint16_t rx_len, uint8_t cur_byte, uint8_t *pid_len );
static uint8_t obdii_pid_pack_limit( uint8_t mode );
static OBDII_STATUS obdii_add_pid_to_request_msg( POBDII_PACKET_MANAGER dev, PTR_PID_DATA pid );

#define OBDII_PID_NOT_FOUND   ((OBDII_STATUS)0x85)

uint32_t obdii_tick = 0;

/* Returns the number of bytes a PID request has, if the first byte is 0 *
 * then the pid is a single byte, otherwise it is 2 bytes.               */
static uint8_t get_num_bytes( uint16_t pid ) { return ( 1U + ( (pid >> 8) || 0) ); }

/* Re-enable communication, this only needs to be called if the          *
 * communication has been paused.                                        */
void OBDII_Continue( POBDII_PACKET_MANAGER dev )
{
    /* Clear the pause flag */
    dev->status_flags &= ~OBDII_COMM_PAUSE;

    /* Refresh the timeout */
    refresh_timeout(dev);
}

/* Pause the library from requesting data. This is important for when    *
 * another device is present. This allows the system to save the current *
 * PID stream but stop communication.                                    */
void OBDII_Pause( POBDII_PACKET_MANAGER dev )
{
    /* Set the pause flag */
    dev->status_flags |= OBDII_COMM_PAUSE;

    /* Clear the pending message flag since communication has paused and *
     * the packet will be lost.                                          */
    dev->status_flags &= ~OBDII_PENDING_RESPONSE;
    ISOTP_Reset(&dev->isotp);
}

void OBDII_Initialize( POBDII_PACKET_MANAGER dev )
{
    dev->status_flags = 0;
    clear_obdii_packets(dev);
    clear_diagnostics(dev);
    clear_pid_entries(dev);
    ISOTP_Initialize(&dev->isotp);
    dev->obdii_time = obdii_tick;
}

static int8_t obdll_find_pid_index(POBDII_PACKET_MANAGER dev, PTR_PID_DATA pid) {
    if (!dev || !pid) return -1;
    for (uint8_t i = 0; i < dev->num_pids; i++) {
        if (dev->stream[i] == pid) return (int8_t)i;
    }
    return -1;
}

OBDII_STATUS OBDII_add_PID_request(POBDII_PACKET_MANAGER dev, PTR_PID_DATA pid)
{
    if (!dev || !pid) return OBDII_ERROR;

    /* Check if the PID is supported based on a length greater than 0 */
    if (lookup_payload_length(pid->pid_uuid) == 0)
        return OBDII_UNSUPPORTED_PID_REQ;

    /* Prevent duplicates */
    if (obdll_find_pid_index(dev, pid) >= 0)
        return OBDII_OK; /* already present; no regeneration needed */

    /* Verify another PID can be added */
    if (dev->num_pids >= OBDII_MAX_PIDS)
        return OBDII_MAX_PIDS_REACHED;

    /* Add the PID request, then enable its state */
    dev->stream[dev->num_pids] = pid;
    dev->num_pids++;

    /* Force packet regeneration */
    dev->status_flags &= ~OBDII_PACKET_GENERATED;

    return OBDII_OK;
}

OBDII_STATUS OBDII_remove_PID_request(POBDII_PACKET_MANAGER dev, PTR_PID_DATA pid)
{
    if (!dev || !pid) return OBDII_ERROR;

    int8_t idx = obdll_find_pid_index(dev, pid);
    if (idx < 0) return OBDII_PID_NOT_FOUND;

    /* Shift down everything after idx */
    for (uint8_t i = (uint8_t)idx; i + 1 < dev->num_pids; i++) {
        dev->stream[i] = dev->stream[i + 1];
    }
    /* Clear last slot and decrement count */
    dev->num_pids--;
    dev->stream[dev->num_pids] = NULL;

    /* Force packet regeneration */
    dev->status_flags &= ~OBDII_PACKET_GENERATED;

    return OBDII_OK;
}

OBDII_STATUS OBDII_resync(POBDII_PACKET_MANAGER dev)
{
    /* Force packet regeneration */
    dev->status_flags &= ~OBDII_PACKET_GENERATED;

    return OBDII_OK;
}

OBDII_PACKET_MANAGER_STATUS OBDII_Service( POBDII_PACKET_MANAGER dev )
{
    /*************************************************************************
     * Nothing shall happen until PID[s] are requested.
     ************************************************************************/
    if( (dev->num_pids == 0) || (dev->status_flags & OBDII_COMM_PAUSE) )
        return OBDII_PM_IDLE;

    /*************************************************************************
     * If the PID request is up to date, then continue normal operation.
     ************************************************************************/
    if( dev->status_flags & OBDII_PACKET_GENERATED )
    {
        /*************************************************************************
         * If a message has been sent and the library is waiting for a response
         * then continually check the message does not timeout .
         **************************************************************************/
        if( dev->status_flags & OBDII_PENDING_RESPONSE )
        {
            /* Verify the message hasn't timed out */
           if( obdii_tick >= (dev->obdii_time + dev->init.timeout) )
           {
               /* If so, clear the pending message flag in order to re-send the data */
               dev->status_flags &= ~OBDII_PENDING_RESPONSE;
               ISOTP_Reset(&dev->isotp);

               /* Abort the tx message and increment the transmission abort counter */
               dev->diagnostic.tx_abort_count++;

               /* Refresh the timer, and attempt to re-transmit */
               refresh_timeout(dev);

               /* Indicate that a timeout occurred */
               return OBDII_PM_TIMEOUT;
           }

           /* Timeout has not occurred, this is normal operation */
           return OBDII_PM_NORMAL;
        }

        else
        {
            /* The last message was received, send the next packet */
            if( dev->init.transmit( dev->msg[dev->current_msg].frame[dev->msg[dev->current_msg].current_frame].buf , OBDII_DLC ) == 0 )
                dev->diagnostic.tx_failure++;

            /* Indicate successful transmission */
            dev->status_flags |= OBDII_PENDING_RESPONSE;

            /* Increment the frame */
            dev->msg[dev->current_msg].current_frame = (dev->msg[dev->current_msg].current_frame + 1) % dev->msg[dev->current_msg].num_frames;

            /* Refresh the timeout */
            refresh_timeout(dev);

            if( dev->status_flags & OBDII_RESPONSE_RECEIVED )
            {
                dev->status_flags &= ~OBDII_RESPONSE_RECEIVED;

                OBDII_PROCESS_STATUS status = OBDII_Process_Packet(dev);

                /* If we are on frame 0, we know that a full message has been sent. On to the next message */
                if( dev->msg[dev->current_msg].current_frame == 0 )
                    dev->current_msg = ( dev->current_msg + 1 ) % dev->num_msgs;

                if( status == OBDII_PACKET_PROCESS_SUCCESS )
                {
                    return OBDII_PM_NEW_DATA;
                }
                else if( status == OBDII_CAN_PCKT_MISALIGNED || status == OBDII_PID_NOT_SUPPORTED )
                {
                    dev->diagnostic.error = (uint8_t)status;
                    return OBDII_PM_ERROR;
                }
            }

            return OBDII_PM_NORMAL;
        }
    }
    /*************************************************************************
     * There is a new PID request or a change to the current request.
     * Re-generate the PID request.
     ************************************************************************/
    else {
        /* Generate the new PID request */
        if( obdii_generate_PID_Request(dev) == OBDII_OK )
        {
            /* Indicate the OBDII packet is up to date */
            dev->status_flags |= OBDII_PACKET_GENERATED;

            /* Nothing has changed in the perspective of the application layer */
            return OBDII_PM_NORMAL;
        }
        else
            return OBDII_PACKET_GEN_ERROR;
    }

    /* HOW DID I GET HERE!? */
    return OBDII_PM_CRITICAL_ERROR;
}

OBDII_STATUS OBDII_Add_Packet( POBDII_PACKET_MANAGER dev, uint16_t arbitration_id, uint8_t* packet_data )
{
    /* Verify the CAN packet is intended for the Digital Dash */
    if( arbitration_id == 0x7E8 ) //TODO
    {
        ISOTP_STATUS status = ISOTP_Add_Frame(&dev->isotp, packet_data, OBDII_DLC);

        if( status == ISOTP_FLOW_CONTROL_REQUIRED )
        {
            ISOTP_Send_Flow_Control(dev->init.transmit);
            refresh_timeout( dev );
        }
        else if( status == ISOTP_COMPLETE )
        {
            /* Indicate a full CAN Packet has been received */
            dev->status_flags &= ~OBDII_PENDING_RESPONSE;

            /* Indicate that the full CAN packet should be processed */
            dev->status_flags |= OBDII_RESPONSE_RECEIVED;
        }
        else if( (status == ISOTP_ERROR) || (status == ISOTP_UNSUPPORTED_FRAME) || (status == ISOTP_OVERFLOW) || (status == ISOTP_SEQUENCE_ERROR) )
        {
            dev->diagnostic.rx_abort_count++;
            dev->diagnostic.error = (uint8_t)status;
            return OBDII_UNSUPPORTED_CAN_PACKET;
        }

    } else if ( arbitration_id == 0x7E0 ) {
        refresh_timeout( dev );
    }
    return OBDII_OK;
}

static inline uint8_t pid_is_enabled(PTR_PID_DATA p)
{
    return (p != NULL) && (p->num_activated > 0);
}

static uint8_t obdii_pid_pack_limit( uint8_t mode )
{
    uint8_t limit = OBDII_DEFAULT_PIDS_PER_REQUEST;

    if( mode == MODE1 )
        limit = OBDII_MODE1_PIDS_PER_REQUEST;
    else if( mode == MODE22 )
        limit = OBDII_MODE22_PIDS_PER_REQUEST;

    return (limit == 0U) ? 1U : limit;
}

static OBDII_STATUS obdii_add_pid_to_request_msg( POBDII_PACKET_MANAGER dev, PTR_PID_DATA pid )
{
    uint8_t mode = get_mode_by_uuid(pid->pid_uuid);
    uint8_t limit = obdii_pid_pack_limit(mode);

    for( uint8_t msg = 0; msg < dev->num_msgs; msg++ )
    {
        if( dev->msg[msg].mode != mode )
            continue;

        if( dev->msg[msg].num_pid_filters >= limit )
            continue;

        dev->msg[msg].pid_filter[dev->msg[msg].num_pid_filters] = pid;
        dev->msg[msg].num_pid_filters++;
        return OBDII_OK;
    }

    if( dev->num_msgs >= OBDII_MAX_MSGS )
        return OBDII_MAX_PIDS_REACHED;

    dev->msg[dev->num_msgs].mode = mode;
    dev->msg[dev->num_msgs].pid_filter[0] = pid;
    dev->msg[dev->num_msgs].num_pid_filters = 1;
    dev->num_msgs++;

    return OBDII_OK;
}

static OBDII_PROCESS_STATUS OBDII_Process_Packet( POBDII_PACKET_MANAGER dev )
{
    uint8_t curByte = 0;
    uint8_t *rx_buf = ISOTP_Get_Payload(&dev->isotp);
    uint16_t rx_len = ISOTP_Get_Payload_Length(&dev->isotp);

    refresh_timeout( dev );

    if( (rx_buf == 0) || (rx_len == 0) )
        return OBDII_CAN_PCKT_MISALIGNED;

    if( rx_buf[curByte] == 0x7F )
        return OBDII_PID_NOT_SUPPORTED;

    uint8_t mode = rx_buf[curByte++] - 0x40;

    while( curByte < rx_len )
    {
        uint8_t pid_len = 0;
        PTR_PID_DATA pid = find_response_pid(dev, mode, rx_buf, rx_len, curByte, &pid_len);

        if( pid == NULL )
            return OBDII_CAN_PCKT_MISALIGNED;

        curByte += pid_len;

        uint8_t payload_len = lookup_payload_length( pid->pid_uuid );

        if( payload_len == 0 )
            return OBDII_PID_NOT_SUPPORTED;

        if( (curByte + payload_len) > rx_len )
            return OBDII_CAN_PCKT_MISALIGNED;

        uint8_t tmpDataBuf[4] = {0, 0, 0, 0};

        for ( uint8_t data = 0; data < payload_len ; data++ )
            tmpDataBuf[data] = rx_buf[curByte++];

        float pid_value = get_pid_value( pid->pid_uuid, tmpDataBuf );
        update_pid_data(pid, pid_value, obdii_tick);
    }

    refresh_timeout(dev);

    return OBDII_PACKET_PROCESS_SUCCESS;
}

static PTR_PID_DATA find_response_pid( POBDII_PACKET_MANAGER dev, uint8_t mode, uint8_t *rx_buf, uint16_t rx_len, uint8_t cur_byte, uint8_t *pid_len )
{
    for( uint8_t pid_num = 0; pid_num < dev->num_pids; pid_num++ )
    {
        PTR_PID_DATA pid = dev->stream[pid_num];

        if( !pid_is_enabled(pid) )
            continue;

        if( get_mode_by_uuid(pid->pid_uuid) != mode )
            continue;

        uint16_t pid_word = get_pid_by_uuid(pid->pid_uuid);
        uint8_t len = get_num_bytes(pid_word);

        if( (cur_byte + len) > rx_len )
            continue;

        if( len == 1 )
        {
            if( rx_buf[cur_byte] == (pid_word & 0xFF) )
            {
                *pid_len = len;
                return pid;
            }
        }
        else if( len == 2 )
        {
            uint16_t response_pid = ((uint16_t)rx_buf[cur_byte] << 8) | (uint16_t)rx_buf[cur_byte + 1];

            if( response_pid == pid_word )
            {
                *pid_len = len;
                return pid;
            }
        }
    }

    return NULL;
}

static OBDII_STATUS obdii_generate_PID_Request(POBDII_PACKET_MANAGER dev)
{
    /*************************************************************************
     * Verify there are enabled PID requests.
     *************************************************************************/
    if (!dev) return OBDII_ERROR;

    uint8_t enabled_count = 0;
    for (uint8_t i = 0; i < dev->num_pids; i++)
        if (pid_is_enabled(dev->stream[i])) enabled_count++;

    if (enabled_count == 0)
        return OBDII_PID_REQ_EMPTY;

    /*************************************************************************
     * Reset message index and clear packet buffers.
     *************************************************************************/
    dev->current_msg = 0;
    clear_obdii_packets(dev);  // assumed to reset dev->num_msgs, frames, etc.

    /*************************************************************************
     * Build request list using the per-mode packing limits.
     *************************************************************************/
    for (uint8_t i = 0; i < dev->num_pids; i++)
    {
        PTR_PID_DATA p = dev->stream[i];
        if (!pid_is_enabled(p)) continue;

        OBDII_STATUS status = obdii_add_pid_to_request_msg(dev, p);
        if( status != OBDII_OK )
            return status;
    }

    /*************************************************************************
     * For each mode, size and emit frames using only ENABLED PIDs.
     *************************************************************************/
    for (uint8_t msg = 0; msg < dev->num_msgs; msg++)
    {
        uint8_t num_bytes = 1; // +1 for service
        uint8_t cur_byte  = 0;
        uint8_t frame     = 0;
        uint8_t payload_bytes_written = 1;

        /* Compute total payload bytes (service + PIDs) */
        for (uint8_t i = 0; i < dev->msg[msg].num_pid_filters; i++) {
            PTR_PID_DATA p = dev->msg[msg].pid_filter[i];
            uint16_t pid_word = get_pid_by_uuid(p->pid_uuid);
            if ((pid_word >> 8) != 0) num_bytes += 1;   // high byte present
            if ((pid_word & 0xFF) != 0) num_bytes += 1; // low byte present
        }

        /* Single vs. multi-frame header */
        if (num_bytes <= (OBDII_DLC - 1U)) { // -1 for length byte
            dev->msg[msg].frame[0].buf[cur_byte++] = num_bytes;           // LEN
            dev->msg[msg].frame[0].buf[cur_byte++] = dev->msg[msg].mode;  // MODE
        } else {
            dev->msg[msg].frame[0].buf[cur_byte++] = (frame | ISOTP_FIRST_FRAME);      // FF
            dev->msg[msg].frame[0].buf[cur_byte++] = num_bytes;           // LEN
            dev->msg[msg].frame[0].buf[cur_byte++] = dev->msg[msg].mode;  // MODE
        }

        /* Emit PIDs */
        for (uint8_t i = 0; i < dev->msg[msg].num_pid_filters; i++)
        {
            PTR_PID_DATA p = dev->msg[msg].pid_filter[i];
            uint16_t pid_word = get_pid_by_uuid(p->pid_uuid);

            if ((pid_word >> 8) != 0) {
                dev->msg[msg].frame[frame].buf[cur_byte] = (pid_word >> 8) & 0xFF;
                payload_bytes_written++;

                if( payload_bytes_written < num_bytes )
                    next_byte(&frame, &cur_byte);
            }

            if ((pid_word & 0xFF) != 0) {
                dev->msg[msg].frame[frame].buf[cur_byte] = pid_word & 0xFF;
                payload_bytes_written++;

                if( payload_bytes_written < num_bytes )
                    next_byte(&frame, &cur_byte);
            }

            /* If we advanced to a new frame, patch its PCI byte to CF */
            if ((frame > 0) && (cur_byte <= 2)) {
                dev->msg[msg].frame[frame].buf[0] = frame | ISOTP_CONSECUTIVE_FRAME;
            }
        }

        dev->msg[msg].num_frames = frame + 1;
    }

    return OBDII_OK;
}

static void refresh_timeout( POBDII_PACKET_MANAGER dev )
{
    dev->obdii_time = obdii_tick;
}

void OBDII_tick( void )
{
    obdii_tick++;
}


static void clear_obdii_packets( POBDII_PACKET_MANAGER dev )
{
    for( uint8_t i = 0; i < OBDII_MAX_MSGS; i++ )
    {
        dev->msg[i].mode = MODE_NOT_CONFIGURED;

        dev->msg[i].num_pid_filters = 0;

        for( uint8_t pid = 0; pid < OBDII_MAX_PIDS; pid++ )
            dev->msg[i].pid_filter[pid] = NULL;

        dev->msg[i].current_frame = 0;

        dev->msg[i].num_frames = 0;

        for( uint8_t index = 0; index < OBDII_MAX_FRAMES; index++ )
            memset(dev->msg[i].frame[index].buf, 0x55, OBDII_DLC);
    }

    dev->num_msgs = 0;
}

static void next_byte( uint8_t *frame, uint8_t *cur_byte )
{
    if( ((*cur_byte) + 0x01U) < OBDII_DLC )
    {
        (*cur_byte)++;
    } else
    {
        (*frame)++;
        (*cur_byte) = 0x01U;
    }
}

static void clear_diagnostics( POBDII_PACKET_MANAGER dev )
{
    dev->diagnostic.tx_abort_count = 0;
    dev->diagnostic.rx_abort_count = 0;
    dev->diagnostic.rx_count = 0;
    dev->diagnostic.tx_failure = 0;
}

static void clear_pid_entries( POBDII_PACKET_MANAGER dev )
{
    for( uint8_t i = 0; i < OBDII_MAX_PIDS; i++)
    {
        //dev->stream.pid[i] = 0;
        //dev->pid_results[i] = 0;
    }

    /* Reset the byte count */
    dev->num_pids = 0;
}

float OBDII_Get_Value_Byte_PID( POBDII_PACKET_MANAGER dev, uint16_t pid )
{
    for( uint8_t i = 0; i < dev->num_pids; i++ )
    {
        //if( pid == dev->stream.pid[i] )
            //return dev->pid_results[i];
        return 0;
    }
    return 0;
}
