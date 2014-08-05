/*
 * CAN and CANopen code for the dsPIC33E bootloader.
 * The functions in this file send and receive CAN and CANopen messages without
 * using interrupts.
 * DMA0 and DMA1 are used to transfer data to and from the CAN module.
 * Ken Caluwaerts 2014 <ken@caluwaerts.eu>
 *
 * Copyright © 2014, Ken Caluwaerts
 * All rights reserved.
 *
 * The CAN(CANopen) and UART bootloader for the dsPIC33E bootloader is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 */
#include <stdint.h>
#include "p33Exxxx.h"
#include "bootloader.h"
#include "canbus.h"
#ifndef BL_UART
/*CAN message buffers*/
/*TODO: use only the minimum amount of memory */
static unsigned int ecan1RXMsgBuf[32][8] __attribute__((aligned(32 * 16)));
static unsigned int ecan1TXMsgBuf[32][8] __attribute__((aligned(32 * 16)));

void can_init()
{
    unsigned int config;
    unsigned int irq;
    unsigned long int stb_address;
    unsigned int pad_address;
    unsigned int count;
    // 0 normal, 1 disable, 2 loopback, 3 listen-only, 4 configuration, 7 listen all messages
    uint8_t desired_mode = 0;

    /* peripheral pin select for CAN */
    TRISBbits.TRISB10 = 0; //pin 21 (TDO/RP42/RB10) is CAN TX output
    _RP42R = 0b001110;
    //pin 22 (TDI/RP43/RB11) is CAN RX input
    RPINR26bits.C1RXR = 43;

    C1CTRL1bits.WIN = 0; // Allow configuration of masks and filters

    //CONFIG CAN
    // Make sure the ECAN module is in configuration mode.
    // It should be this way after a hardware reset, but
    // we make sure anyways.
    C1CTRL1bits.REQOP = 4;
    while (C1CTRL1bits.OPMODE != 4);

    //1Mbaud TODO: remove these hardcoded values
    // Setup our frequencies for time quanta calculations.
    // FCAN is selected to be FCY*2 = FP*2 = 140Mhz
    C1CTRL1bits.CANCKS = 1; // 1 => FP*2; 0 => FP
    C1CFG1bits.BRP = 4; //9 = (140MHz/(2*(14*1Mbaud)))-1 [14 TQ/bit = Bit Time]
    // Based on Bit Time
    C1CFG1bits.SJW = 0;//(parameters[3] & 0x0600) >> 9;
    C1CFG2bits.PRSEG = 0;//5*TQ//b; // Set propagation segment time
    C1CFG2bits.SEG1PH = 5;//4*TQ//a; // Set segment 1 time
    C1CFG2bits.SEG2PHTS = 0x1; // Keep segment 2 time programmable
    C1CFG2bits.SEG2PH = 5;//4*TQ //c; // Set phase segment 2 time
    C1CFG2bits.SAM = 1; // Triple-sample for majority rules at bit sample point

    // No FIFO, 32 Buffers
    C1FCTRL = 0xC01F;
//    C1FCTRL = 0;
//    C1FCTRLbits.DMABS = 0;
//    C1FCTRLbits.FSA = 0;

    // Clear all interrupt bits
    C1RXFUL1 = C1RXFUL2 = C1RXOVF1 = C1RXOVF2 = 0x0000;
    C1CTRL1bits.WIN = 1; // Allow configuration of masks and filters
    // Configure buffer settings.
    C1TR01CON = 0;
    //buffer 0 is transmit
    C1TR01CONbits.TX0PRI = 3;
    C1TR01CONbits.TXEN0 = 1;

    //CONFIG DMA
    //TX
    DMA1CONbits.SIZE = 0; //word transfer mode
    DMA1CONbits.DIR = 0x1; //RAM to peripheral
    DMA1CONbits.AMODE = 0x2; //peripheral indirect addressing mode
    DMA1CONbits.MODE = 0x0; //continuous, no ping-pong
    DMA1REQ = 70; // CAN1 TX
    DMA1CNT = 7; // 8 words per transfer
    DMA1PAD = (volatile unsigned int)&C1TXD;
    DMA1STAL = (unsigned int)ecan1TXMsgBuf;
    DMA1STAH = 0x0;
    config = DMA1CON|0b1000000000000000;
    irq = 70; //CAN TX
    count = 7;   //8 words per transfer
    pad_address = (volatile unsigned int)&C1TXD;
    //sta_address = ecan1MsgBuf;
    stb_address = 0x0;
    OpenDMA1( config, irq, (long unsigned int)ecan1TXMsgBuf,
        stb_address,pad_address, count );

    //RX
    DMA0CONbits.SIZE = 0;
    DMA0CONbits.DIR = 0; //Read to RAM from peripheral
    DMA0CONbits.AMODE = 2; // Continuous mode, single buffer
    DMA0CONbits.MODE = 0; // Peripheral Indirect Addressing
    DMA0REQ = 34; // CAN1 RX
    DMA0CNT = 7; // 8 words per transfer
    DMA0PAD = (volatile unsigned int)&C1RXD;
    DMA0STAL = (unsigned int)ecan1RXMsgBuf;
    DMA0STAH = 0x0;
    config = DMA0CON|0b1000000000000000;
    irq = 0x22;// Select ECAN1 RX as DMA Request source
    count = 7;   //8 words per transfer
    //DMA0CONbits.CHEN = 1; // Enable DMA Channel 0
    pad_address = (volatile unsigned int)&C1RXD;
    stb_address = 0x0;
    OpenDMA0( config, irq, (long unsigned int)ecan1RXMsgBuf,
        stb_address,pad_address, count );

    // Enable DMA
    IFS0bits.DMA0IF = 0;
    DMA0CONbits.CHEN = 1;
    DMA1CONbits.CHEN = 1;
    IEC0bits.DMA0IE = 0; // Disable DMA Channel 0 interrupt (everything is handled in the CAN interrupt)

    // Setup message filters and masks.
    C1FMSKSEL1bits.F0MSK=0x0; //Filter 0 will use mask 0
    C1RXM0SIDbits.SID = 0x0;//0x000; // accept all
    //C1FEN1bits.FLTEN0 = 1;
    C1RXF0SIDbits.SID = 0x0;//0x7EE;// set filter
    C1RXM0SIDbits.MIDE = 0x1; //only receive standard frames
    C1RXF0SIDbits.EXIDE= 0x0;
    C1BUFPNT1bits.F0BP = 0x0;//0x1; //use message buffer 1 to receive data
    C1FEN1bits.FLTEN0=0x1; //Filter 0 enabled for Identifier match with incoming message

    C1CTRL1bits.WIN = 0;

    // Place ECAN1 into normal mode
    desired_mode = 0;
    C1CTRL1bits.REQOP = desired_mode;
    while (C1CTRL1bits.OPMODE != desired_mode);

    // Disable interrupts for ECAN1 (interrupts don't work in the bootloader)
    C1INTFbits.RBIF = 0;
    C1INTFbits.RBOVIF = 0;
    IFS2bits.C1IF = 0;

    IEC2bits.C1IE = 0; // Disable interrupts for ECAN1 peripheral
    C1INTEbits.TBIE = 0; // Disable TX buffer interrupt
    C1INTEbits.RBIE = 0; // Disable RX buffer interrupt
    C1INTEbits.ERRIE = 0;
    C1INTEbits.IVRIE = 0;
    C1INTEbits.RBOVIE = 0;
}

/**
 * Receive a CAN message (if any)
 * @param m Message pointer to fill with the received data
 * @return 0 if no message was received, 1 if a CAN message was received (m valid)
 */
uint8_t can_receive_msg(Message *m) {
    uint8_t ide = 0;
    uint8_t srr = 0;
    uint32_t id = 0;
    uint8_t index_byte;
    uint16_t buffer;
    uint16_t *ecan_msg_buf_ptr;
    static uint8_t packet_idx;
    unsigned i;
    uint16_t j;
    if (C1INTFbits.RBIF) {
        m->cob_id=0xFFFF;
        m->rtr=1;
        m->len=255;

        // Obtain the buffer the message was stored into, checking that the value is valid to refer to a buffer
        //if (C1VECbits.ICODE < 32) { /*Only works with interrupts enabled*/
        for(j=0;j<16;++j){
            if(C1RXFUL1&(1<<j)){
                buffer = j; /*TODO: is there a more efficient way to do this?*/
                break;
            }
            //buffer = C1VECbits.ICODE;
        }

        ecan_msg_buf_ptr = ecan1RXMsgBuf[buffer];

        // Clear the buffer full status bit so more messages can be received.
        if (C1RXFUL1 & (1 << buffer)) {
            C1RXFUL1 &= ~(1 << buffer);
        }

        // Read the first word to see the message type
        ide = ecan_msg_buf_ptr[0] & 0x0001;
        srr = ecan_msg_buf_ptr[0] & 0x0002;

        /* Format the message properly according to whether it
         * uses an extended identifier or not.
         */
        if (ide == 0) {
            m->cob_id = (uint32_t) ((ecan_msg_buf_ptr[0] & 0x1FFC) >> 2);
        } else {
            //ehm, error. only std messages supported for now
        }

        /* If message is a remote transmit request, mark it as such.
         * Otherwise it will be a regular transmission so fill its
         * payload with the relevant data.
         */
        if (srr == 1) {
            m->rtr = 1; /*not supported*/
        } else {
            m->rtr = 0;
            m->len = (uint8_t) (ecan_msg_buf_ptr[2] & 0x000F);
            m->data[0] = (uint8_t) (ecan_msg_buf_ptr[3]&0xFF);
            m->data[1] = (uint8_t) ((ecan_msg_buf_ptr[3] & 0xFF00) >> 8);
            m->data[2] = (uint8_t) (ecan_msg_buf_ptr[4]&0xFF);
            m->data[3] = (uint8_t) ((ecan_msg_buf_ptr[4] & 0xFF00) >> 8);
            m->data[4] = (uint8_t) (ecan_msg_buf_ptr[5]&0xFF);
            m->data[5] = (uint8_t) ((ecan_msg_buf_ptr[5] & 0xFF00) >> 8);
            m->data[6] = (uint8_t) (ecan_msg_buf_ptr[6]&0xFF);
            m->data[7] = (uint8_t) ((ecan_msg_buf_ptr[6] & 0xFF00) >> 8);
        }

        // Make sure to clear the interrupt flag.
        C1RXFUL1 = 0;
        C1INTFbits.RBIF = 0;
        return 1;
    }
    return 0;
}

void can_send_msg(Message *m)
{
    uint32_t word0 = 0, word1 = 0, word2 = 0;
    uint32_t sid10_0 = 0, eid5_0 = 0, eid17_6 = 0;

    // Divide the identifier into bit-chunks for storage
    // into the registers.
        sid10_0 = (m->cob_id & 0x7FF);

    word0 |= (sid10_0 << 2);
    word2 |= (eid5_0 << 10);

    /*TODO: use multiple transmit buffers*/
    while(C1TR01CONbits.TXREQ1 == 1);

    ecan1TXMsgBuf[1][0] = word0;
    ecan1TXMsgBuf[1][1] = word1;
    ecan1TXMsgBuf[1][2] = ((word2 & 0xFFF0) + m->len);
    ecan1TXMsgBuf[1][3] = (((uint16_t) m->data[1]) << 8) |(m->data[0]&0xFF);
    ecan1TXMsgBuf[1][4] = (((uint16_t) m->data[3]) << 8) |(m->data[2]&0xFF);
    ecan1TXMsgBuf[1][5] = (((uint16_t) m->data[5]) << 8) |(m->data[4]&0xFF);
    ecan1TXMsgBuf[1][6] = (((uint16_t) m->data[7]) << 8) |(m->data[6]&0xFF);

    // Set the correct transfer intialization bit (TXREQ) based on message buffer.
    C1TR01CONbits.TXEN1 = 1;
    C1TR01CONbits.TX0PRI = 0;
    C1TR01CONbits.TXREQ1 = 1;
}

/* CANopen specific code below */

/**
 * Send an SDO abort message
 * @param sdo_OD_idx
 * @param sdo_OD_sidx
 * @param error
 */
void can_send_SDO_abort(uint16_t sdo_OD_idx, uint8_t sdo_OD_sidx, uint32_t error) {
    Message can_msg_transmit;
    can_msg_transmit.cob_id = 0x580 + CAN_NODE_ID;
    can_msg_transmit.len = 8;
    can_msg_transmit.rtr = 0;
    can_msg_transmit.data[0] = 0x4 << 5;
    can_msg_transmit.data[1] = sdo_OD_idx & 0xFF;
    can_msg_transmit.data[2] = (sdo_OD_idx >> 8)&0xFF;
    can_msg_transmit.data[3] = sdo_OD_sidx & 0xFF;
    can_msg_transmit.data[7] = error>>24; /*See page 519 of the CANOpen book*/
    can_msg_transmit.data[6] = (error>>16)&0xFF;
    can_msg_transmit.data[5] = (error>>8)&0xFF;
    can_msg_transmit.data[4] = (error)&0xFF;
    can_send_msg(&can_msg_transmit);
}

/**
 * Send an SDO abort message when trying to access a non-existent OD entry.
 * @param sdo_OD_idx
 * @param sdo_OD_sidx
 */
void can_send_unsupported_OD(uint16_t sdo_OD_idx, uint8_t sdo_OD_sidx) {
    can_send_SDO_abort(sdo_OD_idx,sdo_OD_sidx,0x06020000);
}

/**
 * Send an SDO download response message
 * @param sdo_OD_idx
 * @param sdo_OD_sidx
 */
void can_send_sdo_download_response(uint16_t sdo_OD_idx, uint8_t sdo_OD_sidx) {
    Message can_msg_transmit;
    can_msg_transmit.cob_id = 0x580 + CAN_NODE_ID;
    can_msg_transmit.len = 8;
    can_msg_transmit.rtr = 0;
    can_msg_transmit.data[0] = 0x3 << 5;
    can_msg_transmit.data[1] = sdo_OD_idx & 0xFF;
    can_msg_transmit.data[2] = (sdo_OD_idx >> 8)&0xFF;
    can_msg_transmit.data[3] = sdo_OD_sidx & 0xFF;
    can_send_msg(&can_msg_transmit);
}

/**
 * Send an SDO download segment response message
 * @param toggle the toggle bit value of the corresponding SDO download segment
 */
void can_send_sdo_download_segment_response(uint8_t toggle){
    Message can_msg_transmit;
    can_msg_transmit.cob_id = 0x580 + CAN_NODE_ID;
    can_msg_transmit.len = 8;
    can_msg_transmit.rtr = 0;
    can_msg_transmit.data[0] = (0x1 << 5) | ((toggle&0b1)<<4);
    can_msg_transmit.data[1] = 0x00;
    can_msg_transmit.data[2] = 0x00;
    can_msg_transmit.data[3] = 0x00;
    can_msg_transmit.data[7] = 0x00;
    can_msg_transmit.data[6] = 0x00;
    can_msg_transmit.data[5] = 0x00;
    can_msg_transmit.data[4] = 0x00;
    can_send_msg(&can_msg_transmit);
}

/**
 * Send an SDO upload response for an expedited SDO.
 * Little endian conversion should be handled in the calling function.
 * @param sdo_OD_idx
 * @param sdo_OD_sidx
 * @param num_bytes_unused number of unused bytes in can_exp_buffer (0-3)
 * @param can_exp_buffer
 */
void can_send_sdo_upload_response_expedited(uint16_t sdo_OD_idx, uint8_t sdo_OD_sidx, unsigned num_bytes_unused, uint8_t* can_exp_buffer){
    Message can_msg_transmit;
    can_msg_transmit.cob_id = 0x580 + CAN_NODE_ID;
    can_msg_transmit.len = 8;
    can_msg_transmit.rtr = 0;
    can_msg_transmit.data[0] = (0x2 << 5) | ((num_bytes_unused & 0b11)<<2) | 0b11;
    can_msg_transmit.data[1] = sdo_OD_idx & 0xFF;
    can_msg_transmit.data[2] = (sdo_OD_idx >> 8)&0xFF;
    can_msg_transmit.data[3] = sdo_OD_sidx & 0xFF;
    /*Handle little endian in caller function!*/
    switch (num_bytes_unused & 0b11){
        case 0:
            can_msg_transmit.data[7] = can_exp_buffer[3];
        case 1:
            can_msg_transmit.data[6] = can_exp_buffer[2];
        case 2:
            can_msg_transmit.data[5] = can_exp_buffer[1];
        case 3:
            can_msg_transmit.data[4] = can_exp_buffer[0];
        default:
            break;
    };

    can_send_msg(&can_msg_transmit);
}

/**
 * Send a CANopen heartbeat message
 * @param state node state
 */
void can_send_heartbeat(uint8_t state) {
    Message can_msg_transmit;
    can_msg_transmit.cob_id = 0x700 + CAN_NODE_ID;
    can_msg_transmit.len = 1;
    can_msg_transmit.rtr = 0;
    can_msg_transmit.data[0] = state & 127;
    can_send_msg(&can_msg_transmit);
}
#endif
