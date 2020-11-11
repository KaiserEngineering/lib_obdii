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

#include <lib_obdii.h>

#define MODE_NOT_CONFIGURED 0xAA

static void next_byte( uint8_t *frame, uint8_t *cur_byte );
static OBDII_STATUS obdii_generate_PID_Request( POBDII_PACKET_MANAGER dev );
static void clear_obdii_packets( POBDII_PACKET_MANAGER dev );
static void clear_diagnostics( POBDII_PACKET_MANAGER dev );
static void flush_obdii_rx_buf( POBDII_PACKET_MANAGER dev );
static void refresh_timeout( POBDII_PACKET_MANAGER dev );
static uint8_t lookup_payload_length( uint8_t mode, uint16_t PID );
static float get_pid_value( uint8_t mode, uint16_t pid, uint8_t data[] );
static OBDII_PROCESS_STATUS OBDII_Process_Packet( POBDII_PACKET_MANAGER dev );
static void clear_pid_entries( POBDII_PACKET_MANAGER dev );

static uint8_t flow_control_frame[OBDII_DLC] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint32_t obdii_tick = 0;

/* Returns the number of bytes a PID request has, if the first byte is 0 *
 * then the pid is a single byte, otherwise it is 2 bytes.               */
static uint8_t get_num_bytes( uint16_t pid ) { return ( 1U + ( (pid >> 8) || 0) ); }

void OBDII_Initialize( POBDII_PACKET_MANAGER dev )
{
    dev->status_flags = 0;
    clear_obdii_packets(dev);
    clear_diagnostics(dev);
    clear_pid_entries(dev);
    dev->obdii_time = obdii_tick;
}

OBDII_STATUS OBDII_add_PID_request( POBDII_PACKET_MANAGER dev, PTR_PID_DATA pid )
{
    /* Clear the packet generated flag to start packet regeneration */
    dev->status_flags &= ~OBDII_PACKET_GENERATED;

    /* Verify another PID can be added */
    if( dev->num_pids + 1 > OBDII_MAX_PIDS )
        return OBDII_MAX_PIDS_REACHED;

    /* Add the PID request */
    dev->stream[dev->num_pids] = pid;

    dev->num_pids++;

    /* Return a success */
    return OBDII_OK;
}

OBDII_PACKET_MANAGER_STATUS OBDII_Service( POBDII_PACKET_MANAGER dev )
{
    /*************************************************************************
     * Nothing shall happen until PID[s] are requested.
     ************************************************************************/
    if( dev->num_pids == 0 )
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
    if( arbitration_id >= 0x7E0 ) //TODO
    {
        /* Number of bytes in the CAN packet that is not data */
        uint8_t num_supporting_bytes = OBDII_DLC;

        /* Check if packet is single frame */
        if( ( packet_data[0] & ISO_15765_2_FRAME_TYPE_MASK ) == ISO_15765_2_SINGLE_FRAME )
        {
            /* New message, reset buffer */
            flush_obdii_rx_buf(dev);

            /* Begin counting how many more bytes are expected */
            dev->rx_remaining_bytes = ( packet_data[0] & ISO_15765_2_BYTE0_SIZE_MASK );

            /* First byte has been received */
            dev->rx_remaining_bytes--;

            /* Number of bytes in the CAN packet that is not data */
            num_supporting_bytes = CAN_SINGLE_FRAME_SUPPORTING_BYTES;

            /* Save the mode byte */
            dev->rx_buf[ dev->rx_byte_count++ ] = packet_data[CAN_SINGLE_FRAME_MODE_POS];
        }

        /* Check if packet is first frame */
        else if ( ( packet_data[0] & ISO_15765_2_FRAME_TYPE_MASK ) == ISO_15765_2_FIRST_FRAME_FRAME )
        {
            /* New message, reset buffer */
            flush_obdii_rx_buf(dev);

            /* Begin counting how many more bytes are expected */
            dev->rx_remaining_bytes = ( ( packet_data[0] & ISO_15765_2_BYTE0_SIZE_MASK) << 8 ) | ( packet_data[1] & ISO_15765_2_BYTE1_SIZE_MASK );

            /* First byte has been received */
            dev->rx_remaining_bytes--;

            /* Number of bytes in the CAN packet that is not data */
            num_supporting_bytes = CAN_FIRST_FRAME_SUPPORTING_BYTES;

            /* Save the mode byte */
            dev->rx_buf[ dev->rx_byte_count++ ] = packet_data[MULTI_FRAME_MODE_POS];
        }

        /* Check if packet is consecutive frame */
        else if ( ( packet_data[0] & ISO_15765_2_FRAME_TYPE_MASK ) == ISO_15765_2_CONNSECUTIVE_FRAME )
        {
            /* Number of bytes in the CAN packet that is not data */
            num_supporting_bytes = CAN_CONNSECUTIVE_FRAME_SUPPORTING_BYTES;
        }

        /* Check if packet is flow control frame */
        else if ( ( packet_data[0] & ISO_15765_2_FRAME_TYPE_MASK ) == ISO_15765_2_FLOW_CONTROL_FRAME )
        {
            /* Number of bytes in the CAN packet that is not data */
            num_supporting_bytes = CAN_FLOW_CONTROL_FRAME_SUPPORTING_BYTES;
        }

        /* Oh no! What happened!? */
        else {
            return OBDII_UNSUPPORTED_CAN_PACKET;
        }

        /* Copy data to the RX buffer */
        for( uint8_t i = num_supporting_bytes; i < OBDII_DLC; i++ )
        {
            /* More bytes expected */
            if( dev->rx_remaining_bytes > 0 )
            {
                /* Copy bytes to RX buffer */
                dev->rx_buf[ dev->rx_byte_count++ ] = packet_data[i];

                /* Decrement the remaining bytes */
                dev->rx_remaining_bytes--;
            }
        }

        /* Determine if a flow control message is necessary */
        if( dev->rx_remaining_bytes > 0 )
        {
            /* Generate flow control packet */
            dev->init.transmit( flow_control_frame , OBDII_DLC );

            refresh_timeout( dev );
        }

        /* See if all of the bytes have been received */
        else if ( dev->rx_remaining_bytes <= 0 )
        {
            /* Indicate a full CAN Packet has been received */
            dev->status_flags &= ~OBDII_PENDING_RESPONSE;

            /* Indicate that the full CAN packet should be processed */
            dev->status_flags |= OBDII_RESPONSE_RECEIVED;
        }

    }
    return OBDII_OK;
}

static OBDII_PROCESS_STATUS OBDII_Process_Packet( POBDII_PACKET_MANAGER dev )
{
    uint8_t curByte = 0;

    refresh_timeout( dev );

    uint8_t mode = dev->rx_buf[curByte++] - 0x40;

    for( uint8_t pid_num = 0; pid_num < dev->num_pids; pid_num++ )
    {
        if( dev->stream[pid_num]->mode == mode )
        {
            if( lookup_payload_length( dev->stream[pid_num]->mode, dev->stream[pid_num]->pid ) > 0 )
            {
                uint16_t pid = 0;
                uint8_t pid_len = 0;

                pid_len = get_num_bytes( dev->stream[pid_num]->pid );

                if( pid_len == 1 ) {
                    pid = dev->rx_buf[curByte++];
                }
                else if ( pid_len == 2 ) {
                    pid = dev->rx_buf[curByte++];
                    pid = ( pid << 8 ) | ( dev->rx_buf[curByte++] & 0xFF );
                }

                if( pid == dev->stream[pid_num]->pid )
                {
                    uint8_t tmpDataBuf[4] = {0, 0, 0, 0};

                    /* Save the PID's payload ( 1 to 4 bytes ) */
                    for ( uint8_t data = 0; data < lookup_payload_length( dev->stream[pid_num]->mode, dev->stream[pid_num]->pid ) ; data++ )
                    {
                        tmpDataBuf[data] = dev->rx_buf[curByte];

                        curByte++;
                    }

                    dev->stream[pid_num]->pid_value = get_pid_value( dev->stream[pid_num]->mode, dev->stream[pid_num]->pid, tmpDataBuf );

                    if( dev->stream[pid_num]->pid_unit != get_pid_base_unit( dev->stream[pid_num]->mode, dev->stream[pid_num]->pid ) )
                        dev->stream[pid_num]->pid_value = convert_units( get_pid_base_unit( dev->stream[pid_num]->mode, dev->stream[pid_num]->pid ), dev->stream[pid_num]->pid_unit, dev->stream[pid_num]->pid_value );

                } else {
                    return OBDII_CAN_PCKT_MISALIGNED;
                }
            } else {
                return OBDII_PID_NOT_SUPPORTED;
            }
        }
    }

    refresh_timeout(dev);

    return OBDII_PACKET_PROCESS_SUCCESS;
}

static uint8_t lookup_payload_length( uint8_t mode, uint16_t PID )
{
    switch ( mode )
    {
        case MODE1:
            switch ( PID )
            {
                case MODE1_CALCULATED_ENGINE_LOAD_VALUE:
                    return MODE1_CALCULATED_ENGINE_LOAD_VALUE_LEN;

                case MODE1_ENGINE_COOLANT_TEMPERATURE:
                    return MODE1_ENGINE_COOLANT_TEMPERATURE_LEN;

                case MODE1_ENGINE_RPM:
                    return MODE1_ENGINE_RPM_LEN;

                case MODE1_INTAKE_MANIFOLD_ABSOLUTE_PRESSURE:
                    return MODE1_INTAKE_MANIFOLD_ABSOLUTE_PRESSURE_LEN;

                case MODE1_VEHICLE_SPEED:
                    return MODE1_VEHICLE_SPEED_LEN;

                case MODE1_INTAKE_AIR_TEMPERATURE:
                    return MODE1_INTAKE_AIR_TEMPERATURE_LEN;

                case MODE1_MAF_AIR_FLOW_RATE:
                    return MODE1_MAF_AIR_FLOW_RATE_LEN;

                case MODE1_THROTTLE_POSITION:
                    return MODE1_THROTTLE_POSITION_LEN;

                case MODE1_BAROMETRIC_PRESSURE:
                    return MODE1_BAROMETRIC_PRESSURE_LEN;

                case MODE1_ABSOLUTE_LOAD_VALUE:
                    return MODE1_ABSOLUTE_LOAD_VALUE_LEN;

                case MODE1_AMBIENT_AIR_TEMPERATURE:
                    return MODE1_AMBIENT_AIR_TEMPERATURE_LEN;

                default:
                    return 0x00;
            }
            break;

        case MODE22:
            switch ( PID )
            {
                case MODE22_INTAKE_AIR_TEMPERATURE:
                    return MODE22_INTAKE_AIR_TEMPERATURE_LEN;

                case MODE22_CHARGE_AIR_TEMPERATURE:
                    return MODE22_CHARGE_AIR_TEMPERATURE_LEN;

                default:
                    return 0;
            }
            break;

        default:
            return 0;
        }
}

static OBDII_STATUS obdii_generate_PID_Request( POBDII_PACKET_MANAGER dev )
{
    /*************************************************************************
     * Verify there are PID request.
     ************************************************************************/
    if( dev->num_pids <= 0 )
    {
        return OBDII_PID_REQ_EMPTY;
    }

    /*************************************************************************
     * Parse through each packet and initialize it to the determined header,
     * CAN bus mode and fill the packet buffer with 0x55.
     *************************************************************************/
    clear_obdii_packets(dev);

    for( uint8_t i = 0; i < dev->num_pids; i++ )
    {
        for( uint8_t j = 0; j < OBDII_MAX_MSGS; j++ )
        {
            /* If a match is found there is no need to increment */
            if( dev->stream[i]->mode == dev->msg[j].mode )
                break;

            /* No match found, add the mode to the next message struct */
            if( j == OBDII_MAX_MSGS - 1 )
                dev->msg[dev->num_msgs++].mode = dev->stream[i]->mode;
        }
    }

    for(uint8_t msg = 0; msg < dev->num_msgs; msg++ )
    {
        uint8_t num_bytes = 1; //+1 for service
        uint8_t pid_count = 0;
        uint8_t cur_byte  = 0;
        uint8_t frame     = 0;

        /*************************************************************************
         * Parse through the PID array and determine the length of each PID.
         * Populate the PID length in each typedef.
         *************************************************************************/
        for( uint8_t i = 0; i < dev->num_pids; i++ )
        {
            if( dev->msg[msg].mode == dev->stream[i]->mode )
                num_bytes += (1U + ((dev->stream[i]->pid >> 8) || 0));
        }

        /*************************************************************************
         * Determine if this will be a single frame request or multiframe
         * request. This is directly related to the number of bytes in the packet
         * and the size of the buffer.
         *************************************************************************/
        if( num_bytes < (OBDII_DLC - 1U) ) // -1 for length byte
        {
            /*************** Length ***************/
            dev->msg[msg].frame[0].buf[cur_byte++] = num_bytes;

            /**************** Mode ****************/
            dev->msg[msg].frame[0].buf[cur_byte++] = dev->msg[msg].mode;
        }
        else  // Multi frame
        {
            /**************** Frame ****************/
            dev->msg[msg].frame[0].buf[cur_byte++] = (frame | 0x10);

            /*************** Length ***************/
            dev->msg[msg].frame[0].buf[cur_byte++] = num_bytes;

            /**************** Mode ****************/
            dev->msg[msg].frame[0].buf[cur_byte++] = dev->msg[msg].mode;
        }

        /*************************************************************************
         * Iterate through every single PID and fill the buffer.
         *************************************************************************/
        for( pid_count = 0; pid_count < dev->num_pids; pid_count++ )
        {
            if( dev->msg[msg].mode == dev->stream[pid_count]->mode )
            {

                if( (dev->stream[pid_count]->pid >> 8) || 0 )
                {
                    /**************** PID byte 2 ****************/
                    dev->msg[msg].frame[frame].buf[cur_byte] = (dev->stream[pid_count]->pid >> 8) & 0xFF;

                    /************* Increment Buffer *************/
                    next_byte( &frame, &cur_byte );
                }

                if( dev->stream[pid_count]->pid & 0xFF )
                {
                    /**************** PID byte 1 ****************/
                    dev->msg[msg].frame[frame].buf[cur_byte] = dev->stream[pid_count]->pid & 0xFF;

                    /************* Increment Buffer *************/
                    next_byte( &frame, &cur_byte );
                }

                if ( (frame > 0) & (cur_byte == 1) )
                {
                    dev->msg[msg].frame[frame].buf[0] = frame | 0x20;
                }
            }

            dev->msg[msg].num_frames = frame + 1;
        }
    }

    return OBDII_OK;
}

static float get_pid_value( uint8_t mode, uint16_t pid, uint8_t data[] )
{
    switch( mode )
    {
        case MODE1:
            switch( pid )
            {
                /*    Equation: (A * 100) / 255    */
                case MODE1_CALCULATED_ENGINE_LOAD_VALUE:
                    return (((float)data[A]) * (float)100) / (float)255;
                    break;

                case MODE1_INTAKE_AIR_TEMPERATURE:
                case MODE1_ENGINE_COOLANT_TEMPERATURE:
                case MODE1_AMBIENT_AIR_TEMPERATURE:
                    return ((float)data[A] - (float)40);
                    break;

                case MODE1_ENGINE_RPM:
                    return (((float)256 * (float)data[A] ) + (float)data[B] ) / (float)4;
                    break;

                case MODE1_INTAKE_MANIFOLD_ABSOLUTE_PRESSURE:
                case MODE1_VEHICLE_SPEED:
                case MODE1_BAROMETRIC_PRESSURE:
                    return (float)data[A];
                    break;

                case MODE1_MAF_AIR_FLOW_RATE:
                    return (((float)256 * (float)data[A] ) + (float)data[B] ) / (float)100;
                    break;

                default:
                    return -1;
                    break;
            }

        case MODE22:
            switch( pid )
            {
                case MODE22_CHARGE_AIR_TEMPERATURE:
                    return (((float)256 * (float)data[A] ) + (float)data[B] ) / (float)64;

                case MODE22_INTAKE_AIR_TEMPERATURE:
                    return ((float)data[A] - (float)40);
                    break;

                default:
                    return -1;
                    break;
            }

        default:
            return -1;
    }
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
    for( uint8_t i = 0; i < OBDII_MAX_MSGS; i++)
    {
        dev->msg[i].mode = MODE_NOT_CONFIGURED;

        dev->msg[i].current_frame = 0;

        dev->msg[i].num_frames = 0;

        for( uint8_t index = 0; index < OBDII_MAX_FRAMES; index++)
            memset(dev->msg[i].frame[index].buf, 0x55, OBDII_DLC);
    }
}

static void flush_obdii_rx_buf( POBDII_PACKET_MANAGER dev )
{
    dev->rx_byte_count = 0;
    memset(dev->rx_buf, 0, OBDII_RX_BUF_SIZE);
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
