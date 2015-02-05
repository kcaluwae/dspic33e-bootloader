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
#include <pps.h>
#include <dma.h>
#include <ecan.h>
#include <xc.h>
#include "bootloader.h"
#include "canbus.h"
#ifndef BL_UART
/*CAN message buffers*/
//static unsigned int ecan1RXMsgBuf[8][8] __attribute__((aligned(8 * 16)));
//static unsigned int ecan1TXMsgBuf[8][8] __attribute__((aligned(8 * 16)));
__eds__ static unsigned int ecan1RXMsgBuf[8][8] __attribute__((eds,space(dma),aligned(8 * 16)));
__eds__ static unsigned int ecan1TXMsgBuf[8][8] __attribute__((eds,space(dma),aligned(8 * 16)));


uint8_t txreq_bitarray = 0;

///////////////////////////////////////////////////////////////////////////////
////////////////////// USER FUNCTIONS Replace when this is a library //////////

/******************************************************************************
 * Function:     void Ecan1WriteRxAcptFilter(int16_t n, int32_t identifier,
 *               uint16_t exide,uint16_t bufPnt,uint16_t maskSel)
 *
 * PreCondition:  None
 *
 * Input:         n-> Filter number [0-15]
 *                identifier-> Bit ordering is given below
 *                Filter Identifier (29-bits) :
 *                0b000f ffff ffff ffff ffff ffff ffff ffff
 *                     |____________|_____________________|
 *                        SID10:0          EID17:0
 *
 *               Filter Identifier (11-bits) :
 *               0b0000 0000 0000 0000 0000 0fff ffff ffff
 *                                           |___________|
 *                                             SID10:
 *               exide -> "0" for standard identifier
 *                        "1" for Extended identifier
 *               bufPnt -> Message buffer to store filtered message [0-15]
 *               maskSel -> Optinal Masking of identifier bits [0-3]
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      Configures Acceptance filter "n" for ECAN1.
 *****************************************************************************/
void Ecan1WriteRxAcptFilter(int16_t n, int32_t identifier, uint16_t exide, uint16_t bufPnt, uint16_t maskSel)
{
	uint32_t sid10_0 = 0;

	uint32_t eid15_0 = 0;

	uint32_t eid17_16 = 0;
	uint16_t *sidRegAddr;
	uint16_t *bufPntRegAddr;
	uint16_t *maskSelRegAddr;
	uint16_t *fltEnRegAddr;

	C1CTRL1bits.WIN = 1;

	// Obtain the Address of CiRXFnSID, CiBUFPNTn, CiFMSKSELn and CiFEN register for a given filter number "n"
	sidRegAddr = (uint16_t *) (&C1RXF0SID + (n << 1));
	bufPntRegAddr = (uint16_t *) (&C1BUFPNT1 + (n >> 2));
	maskSelRegAddr = (uint16_t *) (&C1FMSKSEL1 + (n >> 3));
	fltEnRegAddr = (uint16_t *) (&C1FEN1);

	// Bit-filed manupulation to write to Filter identifier register
	if (exide == 1) { // Filter Extended Identifier
		eid15_0 = (identifier & 0xFFFF);
		eid17_16 = (identifier >> 16) & 0x3;
		sid10_0 = (identifier >> 18) & 0x7FF;

		*sidRegAddr = (((sid10_0) << 5) + 0x8) + eid17_16; // Write to CiRXFnSID Register
		*(sidRegAddr + 1) = eid15_0; // Write to CiRXFnEID Register
	} else { // Filter Standard Identifier
		sid10_0 = (identifier & 0x7FF);
		*sidRegAddr = (sid10_0) << 5; // Write to CiRXFnSID Register
		*(sidRegAddr + 1) = 0; // Write to CiRXFnEID Register
	}

	*bufPntRegAddr = (*bufPntRegAddr) & (0xFFFF - (0xF << (4 * (n & 3)))); // clear nibble
	*bufPntRegAddr = ((bufPnt << (4 * (n & 3))) | (*bufPntRegAddr)); // Write to C1BUFPNTn Register
	*maskSelRegAddr = (*maskSelRegAddr) & (0xFFFF - (0x3 << ((n & 7) * 2))); // clear 2 bits
	*maskSelRegAddr = ((maskSel << (2 * (n & 7))) | (*maskSelRegAddr)); // Write to C1FMSKSELn Register
	*fltEnRegAddr = ((0x1 << n) | (*fltEnRegAddr)); // Write to C1FEN1 Register
	C1CTRL1bits.WIN = 0;
}

/******************************************************************************
 * Function:     void Ecan1WriteRxAcptMask(int16_t m, int32_t identifier,
 *               uint16_t mide, uint16_t exide)
 *
 * PreCondition:  None
 *
 * Input:        m-> Mask number [0-2]
		 identifier-> Bit ordering is given below n-> Filter number [0-15]
 *                identifier-> Bit ordering is given below
 *                Filter mask Identifier (29-bits) :
 *                0b000f ffff ffff ffff ffff ffff ffff ffff
 *                     |____________|_____________________|
 *                        SID10:0          EID17:0
 *
 *               Filter mask Identifier (11-bits) :
 *               0b0000 0000 0000 0000 0000 0fff ffff ffff
 *                                           |___________|
 *                                             SID10:
 *               mide ->  "0"  Match either standard or extended address message
 *                             if filters match
 *                        "1"  Match only message types that correpond to
 *                             'exide' bit in filter
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      Configures Acceptance filter "n" for ECAN1.
 *****************************************************************************/
void Ecan1WriteRxAcptMask(int16_t m, int32_t identifier, uint16_t mide, uint16_t exide)
{
	uint32_t sid10_0 = 0;

	uint32_t eid15_0 = 0;

	uint32_t eid17_16 = 0;
	uint16_t *maskRegAddr;

	C1CTRL1bits.WIN = 1;

	// Obtain the Address of CiRXMmSID register for given Mask number "m"
	maskRegAddr = (uint16_t *) (&C1RXM0SID + (m << 1));

	// Bit-filed manupulation to write to Filter Mask register
	if (exide == 1) { // Filter Extended Identifier
		eid15_0 = (identifier & 0xFFFF);
		eid17_16 = (identifier >> 16) & 0x3;
		sid10_0 = (identifier >> 18) & 0x7FF;

		if (mide == 1) {
			*maskRegAddr = ((sid10_0) << 5) + 0x0008 + eid17_16; // Write to CiRXMnSID Register
		} else {
			*maskRegAddr = ((sid10_0) << 5) + eid17_16; // Write to CiRXMnSID Register
		}

		*(maskRegAddr + 1) = eid15_0; // Write to CiRXMnEID Register
	} else { // Filter Standard Identifier
		sid10_0 = (identifier & 0x7FF);
		if (mide == 1) {
			*maskRegAddr = ((sid10_0) << 5) + 0x0008; // Write to CiRXMnSID Register
		} else {
			*maskRegAddr = (sid10_0) << 5; // Write to CiRXMnSID Register
		}

		*(maskRegAddr + 1) = 0; // Write to CiRXMnEID Register
	}

	C1CTRL1bits.WIN = 0;
}
///////////////////////////////////////////////////////////////////////////////
////////////////////// USER FUNCTIONS Replace when this is a library //////////

void can_init()
{
	// Sets up the correct pins for the Motor board
        //__builtin_write_OSCCONL(OSCCON & ~(1<<6));
        __builtin_write_OSCCONL(OSCCON & ~(1 << 6));
        TRISFbits.TRISF4 = 1;//1
        TRISFbits.TRISF5 = 0;//0
        IN_FN_PPS_C1RX = IN_PIN_PPS_RP100; //C1Rx
	OUT_PIN_PPS_RP101 = OUT_FN_PPS_C1TX; //C1Tx
        //RPINR26bits.C1RXR = 100;
        //RPOR9bits.RP100R = 0b001110;
        //IN_FN_PPS_C1RX = IN_PIN_PPS_RP100; //C1Rx
        //OUT_PIN_PPS_RP101 = OUT_FN_PPS_C1TX; //C1Tx

        //Lock PPS Registers
        //__builtin_write_OSCCONL(OSCCON | (1 << 6));
        __builtin_write_OSCCONL(OSCCON | (1 << 6));
	
	unsigned int config;
	unsigned int irq;
	unsigned long int stb_address;
	unsigned int pad_address;
	unsigned int count;
	// 0 normal, 1 disable, 2 loopback, 3 listen-only, 4 configuration, 7 listen all messages
	uint8_t desired_mode = 0; //(parameters[0] & 0x001C) >> 2;

	C1CTRL1bits.WIN = 0;

	//CONFIG CAN
	// Make sure the ECAN module is in configuration mode.
	// It should be this way after a hardware reset, but
	// we make sure anyways.
	C1CTRL1bits.REQOP = 4;
	while (C1CTRL1bits.OPMODE != 4);



	//1Mbaud
	// Setup our frequencies for time quanta calculations.
	// FCAN is selected to be FCY*2 = FP*2 = 140Mhz
	C1CTRL1bits.CANCKS = 0;  // 0 => FP*2; 1 => FP (MU806 only)
	C1CFG1bits.BRP = 6; //6 = (140MHz/(2*(10*1Mbaud)))-1 [10 TQ/bit = Bit Time]
	// Based on Bit Time
	// 10 = 1(SJW) + 4(Propagation Seg.) + 3(Phase Seg. 1) + 2(Phase Seg. 2)
	// (Progagation Seg. + Phase Seg. 1) >= Phase Seg. 2
	// Phase Seg. 2 > SJW

	C1CFG1bits.SJW = 0; // (Value-1)
	C1CFG2bits.PRSEG = 3; // Set propagation segment time (Value-1)
	C1CFG2bits.SEG1PH = 2; // Set segment 1 time (Value-1)
	C1CFG2bits.SEG2PHTS = 0x1; // Keep segment 2 time programmable
	C1CFG2bits.SEG2PH = 1; // Set phase segment 2 time (Value-1)
	C1CFG2bits.SAM = 1; // Triple-sample for majority rules at bit sample point
	C1FCTRLbits.DMABS = 0x00;

	// Clear all interrupt bits
	C1RXFUL1 = C1RXFUL2 = C1RXOVF1 = C1RXOVF2 = 0x0000;

	// Configure buffer settings.
	C1TR01CON = 0;
	//buffer 0 is transmit
	C1TR01CONbits.TXEN0 = 1;
	C1TR01CONbits.TXEN1 = 1;
	C1TR23CONbits.TXEN2 = 1;
	C1TR23CONbits.TXEN3 = 1;
	C1TR45CONbits.TXEN4 = 1;
	C1TR45CONbits.TXEN5 = 1;
	C1TR67CONbits.TXEN6 = 1;
	C1TR67CONbits.TXEN7 = 0;


//	//CONFIG DMA
//	//TX
//	DMA1CONbits.SIZE = 0; //word transfer mode
//	DMA1CONbits.DIR = 0x1; //RAM to peripheral
//	DMA1CONbits.AMODE = 0x2; //peripheral indirect addressing mode
//	DMA1CONbits.MODE = 0x0; //continuous, no ping-pong
//	DMA1REQ = 70; // CAN1 TX
//	DMA1CNT = 7; // 8 words per transfer
//	DMA1PAD = (volatile unsigned int) &C1TXD;
//	DMA1STAL = (unsigned int) ecan1TXMsgBuf;
//	DMA1STAH = 0x0;
//
//	//RX
//	DMA0CONbits.SIZE = 0;
//	DMA0CONbits.DIR = 0; //Read to RAM from peripheral
//	DMA0CONbits.AMODE = 2; // Continuous mode, single buffer
//	DMA0CONbits.MODE = 0; // Peripheral Indirect Addressing
//	DMA0REQ = 34; // CAN1 RX
//	DMA0CNT = 7; // 8 words per transfer
//	DMA0PAD = (volatile unsigned int) &C1RXD;
//	DMA0STAL = (unsigned int) ecan1RXMsgBuf;
//	DMA0STAH = 0x0;
//
//
//	// Enable DMA
//	IFS0bits.DMA0IF = 0;
//	IFS0bits.DMA1IF = 0;
//	DMA0CONbits.CHEN = 1;
//	DMA1CONbits.CHEN = 1;
//	IEC0bits.DMA0IE = 0; // Disable DMA Channel 0 interrupt (everything is handled in the CAN interrupt)
//	IEC0bits.DMA1IE = 0;
//
//	Ecan1WriteRxAcptFilter(7, 0x000, 0, 7, 0);
//	Ecan1WriteRxAcptMask(0, 0x000, 0, 0);

        //TX
	DMA5CONbits.SIZE = 0; //word transfer mode
	DMA5CONbits.DIR = 0x1; //RAM to peripheral
	DMA5CONbits.AMODE = 0x2; //peripheral indirect addressing mode
	DMA5CONbits.MODE = 0x0; //continuous, no ping-pong
	DMA5REQ = 70; // CAN1 TX
	DMA5CNT = 7; // 8 words per transfer
	DMA5PAD = (volatile unsigned int) &C1TXD;
	DMA5STAL = (unsigned int) ecan1TXMsgBuf;
	DMA5STAH = 0x0;

	// RX
	DMA4CONbits.SIZE = 0;
	DMA4CONbits.DIR = 0; //Read to RAM from peripheral
	DMA4CONbits.AMODE = 2; // Continuous mode, single buffer
	DMA4CONbits.MODE = 0; // Peripheral Indirect Addressing
	DMA4REQ = 34; // CAN1 RX
	DMA4CNT = 7; // 8 words per transfer
	DMA4PAD = (volatile unsigned int) &C1RXD;
	DMA4STAL = (unsigned int) ecan1RXMsgBuf;
	DMA4STAH = 0x0;

	// Enable DMA
	IFS3bits.DMA5IF = 0;
	IFS2bits.DMA4IF =	 0;
	DMA5CONbits.CHEN = 1;
	DMA4CONbits.CHEN = 1;
	DisableIntDMA5; // Disable DMA interrupt (everything is handled in the CAN interrupt)
	DisableIntDMA4;
        IEC0bits.DMA0IE = 0; // Disable DMA Channel 0 interrupt (everything is handled in the CAN interrupt)
//	IEC0bits.DMA1IE = 0;

//	Ecan1WriteRxAcptFilter(7,0x000,0,7,0);
//	Ecan1WriteRxAcptMask(0,0x000,0,0);
	C1CTRL1bits.WIN=0x1;
	C1FMSKSEL1bits.F7MSK = 0x0;
	C1RXM0SIDbits.SID = 0x00;
	C1RXF0SIDbits.SID = 0x00;
	C1RXM0SIDbits.MIDE = 1;
	C1RXF0SIDbits.EXIDE= 0x0;
	C1BUFPNT2bits.F7BP = 7;
	C1FEN1bits.FLTEN7 = 1;
	C1CTRL1bits.WIN=0x0;

	// Place ECAN1 into normal mode
	desired_mode = 0b000;
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
uint8_t can_receive_msg(Message *m)
{
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
		m->cob_id = 0xFFFF;
		m->rtr = 1;
		m->len = 255;

		// Obtain the buffer the message was stored into, checking that the value is valid to refer to a buffer
		//if (C1VECbits.ICODE < 32) { /*Only works with interrupts enabled*/
		for (j = 0; j < 8; ++j) {
			if (C1RXFUL1 & (1 << j)) {
				buffer = j; /*TODO: is there a more efficient way to do this?*/
				break;
			}
			//buffer = C1VECbits.ICODE;
		}

		ecan_msg_buf_ptr = (__eds__ uint16_t) ecan1RXMsgBuf[buffer];

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
//                IFS3bits.DMA5IF = 0;
//                IFS2bits.DMA4IF = 0;
//                CAN1ClearRXFUL1();
//                CAN1ClearRXFUL2();
//                CAN1ClearRXOVF1();
//                CAN1ClearRXOVF2();
//                IFS2bits.C1IF = 0;

		C1INTFbits.RBIF = 0;
		return 1;
	}
        else {
//            IFS3bits.DMA5IF = 0;
//            IFS2bits.DMA4IF = 0;
//            CAN1ClearRXFUL1();
//            CAN1ClearRXFUL2();
//            CAN1ClearRXOVF1();
//            CAN1ClearRXOVF2();
//            IFS2bits.C1IF = 0;
            return 0;
        }
}

void can_send_msg(Message *m)
{
	uint32_t word0 = 0, word1 = 0, word2 = 0;
	uint32_t sid10_0 = 0, eid5_0 = 0, eid17_6 = 0;

	// Variables for setting correct TXREQ bit
	static uint8_t bufferSwitch = 0;
	static char firstTime = 1;

	sid10_0 = (m->cob_id & 0x7FF);

	word0 |= (sid10_0 << 2);
	word2 |= (eid5_0 << 10);

	// Set remote transmit bits
	if (m->rtr) {
		word0 |= 0x2;
		word2 |= 0x0200;
	}

	switch (bufferSwitch) {
	case 0:
		ecan1TXMsgBuf[0][0] = word0;
		ecan1TXMsgBuf[0][1] = word1;
		ecan1TXMsgBuf[0][2] = ((word2 & 0xFFF0) + m->len);
		ecan1TXMsgBuf[0][3] = (((uint16_t) m->data[1]) << 8) | (m->data[0]&0xFF);
		ecan1TXMsgBuf[0][4] = (((uint16_t) m->data[3]) << 8) | (m->data[2]&0xFF);
		ecan1TXMsgBuf[0][5] = (((uint16_t) m->data[5]) << 8) | (m->data[4]&0xFF);
		ecan1TXMsgBuf[0][6] = (((uint16_t) m->data[7]) << 8) | (m->data[6]&0xFF);
		txreq_bitarray = txreq_bitarray | 0b00000001;
		bufferSwitch++;
		break;
	case 1:
		ecan1TXMsgBuf[1][0] = word0;
		ecan1TXMsgBuf[1][1] = word1;
		ecan1TXMsgBuf[1][2] = ((word2 & 0xFFF0) + m->len);
		ecan1TXMsgBuf[1][3] = (((uint16_t) m->data[1]) << 8) | (m->data[0]&0xFF);
		ecan1TXMsgBuf[1][4] = (((uint16_t) m->data[3]) << 8) | (m->data[2]&0xFF);
		ecan1TXMsgBuf[1][5] = (((uint16_t) m->data[5]) << 8) | (m->data[4]&0xFF);
		ecan1TXMsgBuf[1][6] = (((uint16_t) m->data[7]) << 8) | (m->data[6]&0xFF);
		txreq_bitarray = txreq_bitarray | 0b00000010;
		bufferSwitch++;
		break;
	case 2:
		ecan1TXMsgBuf[2][0] = word0;
		ecan1TXMsgBuf[2][1] = word1;
		ecan1TXMsgBuf[2][2] = ((word2 & 0xFFF0) + m->len);
		ecan1TXMsgBuf[2][3] = (((uint16_t) m->data[1]) << 8) | (m->data[0]&0xFF);
		ecan1TXMsgBuf[2][4] = (((uint16_t) m->data[3]) << 8) | (m->data[2]&0xFF);
		ecan1TXMsgBuf[2][5] = (((uint16_t) m->data[5]) << 8) | (m->data[4]&0xFF);
		ecan1TXMsgBuf[2][6] = (((uint16_t) m->data[7]) << 8) | (m->data[6]&0xFF);
		txreq_bitarray = txreq_bitarray | 0b00000100;
		bufferSwitch++;
		break;

	case 3:
		ecan1TXMsgBuf[3][0] = word0;
		ecan1TXMsgBuf[3][1] = word1;
		ecan1TXMsgBuf[3][2] = ((word2 & 0xFFF0) + m->len);
		ecan1TXMsgBuf[3][3] = (((uint16_t) m->data[1]) << 8) | (m->data[0]&0xFF);
		ecan1TXMsgBuf[3][4] = (((uint16_t) m->data[3]) << 8) | (m->data[2]&0xFF);
		ecan1TXMsgBuf[3][5] = (((uint16_t) m->data[5]) << 8) | (m->data[4]&0xFF);
		ecan1TXMsgBuf[3][6] = (((uint16_t) m->data[7]) << 8) | (m->data[6]&0xFF);
		txreq_bitarray = txreq_bitarray | 0b00001000;
		bufferSwitch++;
		break;
	case 4:
		ecan1TXMsgBuf[4][0] = word0;
		ecan1TXMsgBuf[4][1] = word1;
		ecan1TXMsgBuf[4][2] = ((word2 & 0xFFF0) + m->len);
		ecan1TXMsgBuf[4][3] = (((uint16_t) m->data[1]) << 8) | (m->data[0]&0xFF);
		ecan1TXMsgBuf[4][4] = (((uint16_t) m->data[3]) << 8) | (m->data[2]&0xFF);
		ecan1TXMsgBuf[4][5] = (((uint16_t) m->data[5]) << 8) | (m->data[4]&0xFF);
		ecan1TXMsgBuf[4][6] = (((uint16_t) m->data[7]) << 8) | (m->data[6]&0xFF);
		txreq_bitarray = txreq_bitarray | 0b00010000;
		bufferSwitch++;
		break;
	case 5:
		ecan1TXMsgBuf[5][0] = word0;
		ecan1TXMsgBuf[5][1] = word1;
		ecan1TXMsgBuf[5][2] = ((word2 & 0xFFF0) + m->len);
		ecan1TXMsgBuf[5][3] = (((uint16_t) m->data[1]) << 8) | (m->data[0]&0xFF);
		ecan1TXMsgBuf[5][4] = (((uint16_t) m->data[3]) << 8) | (m->data[2]&0xFF);
		ecan1TXMsgBuf[5][5] = (((uint16_t) m->data[5]) << 8) | (m->data[4]&0xFF);
		ecan1TXMsgBuf[5][6] = (((uint16_t) m->data[7]) << 8) | (m->data[6]&0xFF);
		txreq_bitarray = txreq_bitarray | 0b00100000;
		bufferSwitch++;
		break;
	case 6:
		ecan1TXMsgBuf[6][0] = word0;
		ecan1TXMsgBuf[6][1] = word1;
		ecan1TXMsgBuf[6][2] = ((word2 & 0xFFF0) + m->len);
		ecan1TXMsgBuf[6][3] = (((uint16_t) m->data[1]) << 8) | (m->data[0]&0xFF);
		ecan1TXMsgBuf[6][4] = (((uint16_t) m->data[3]) << 8) | (m->data[2]&0xFF);
		ecan1TXMsgBuf[6][5] = (((uint16_t) m->data[5]) << 8) | (m->data[4]&0xFF);
		ecan1TXMsgBuf[6][6] = (((uint16_t) m->data[7]) << 8) | (m->data[6]&0xFF);
		txreq_bitarray = txreq_bitarray | 0b01000000;
		bufferSwitch=0;
		break;
	default:
		bufferSwitch = 0;
		break;
	}
}

/* CANopen specific code below */

/**
 * Send an SDO abort message
 * @param sdo_OD_idx
 * @param sdo_OD_sidx
 * @param error
 */
void can_send_SDO_abort(uint16_t sdo_OD_idx, uint8_t sdo_OD_sidx, uint32_t error)
{
	Message can_msg_transmit;
	can_msg_transmit.cob_id = 0x580 + CAN_NODE_ID;
	can_msg_transmit.len = 8;
	can_msg_transmit.rtr = 0;
	can_msg_transmit.data[0] = 0x4 << 5;
	can_msg_transmit.data[1] = sdo_OD_idx & 0xFF;
	can_msg_transmit.data[2] = (sdo_OD_idx >> 8)&0xFF;
	can_msg_transmit.data[3] = sdo_OD_sidx & 0xFF;
	can_msg_transmit.data[7] = error >> 24; /*See page 519 of the CANOpen book*/
	can_msg_transmit.data[6] = (error >> 16)&0xFF;
	can_msg_transmit.data[5] = (error >> 8)&0xFF;
	can_msg_transmit.data[4] = (error)&0xFF;
	can_send_msg(&can_msg_transmit);
}

/**
 * Send an SDO abort message when trying to access a non-existent OD entry.
 * @param sdo_OD_idx
 * @param sdo_OD_sidx
 */
void can_send_unsupported_OD(uint16_t sdo_OD_idx, uint8_t sdo_OD_sidx)
{
	can_send_SDO_abort(sdo_OD_idx, sdo_OD_sidx, 0x06020000);
}

/**
 * Send an SDO download response message
 * @param sdo_OD_idx
 * @param sdo_OD_sidx
 */
void can_send_sdo_download_response(uint16_t sdo_OD_idx, uint8_t sdo_OD_sidx)
{
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
void can_send_sdo_download_segment_response(uint8_t toggle)
{
	Message can_msg_transmit;
	can_msg_transmit.cob_id = 0x580 + CAN_NODE_ID;
	can_msg_transmit.len = 8;
	can_msg_transmit.rtr = 0;
	can_msg_transmit.data[0] = (0x1 << 5) | ((toggle & 0b1) << 4);
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
void can_send_sdo_upload_response_expedited(uint16_t sdo_OD_idx, uint8_t sdo_OD_sidx, unsigned num_bytes_unused, uint8_t* can_exp_buffer)
{
	Message can_msg_transmit;
	can_msg_transmit.cob_id = 0x580 + CAN_NODE_ID;
	can_msg_transmit.len = 8;
	can_msg_transmit.rtr = 0;
	can_msg_transmit.data[0] = (0x2 << 5) | ((num_bytes_unused & 0b11) << 2) | 0b11;
	can_msg_transmit.data[1] = sdo_OD_idx & 0xFF;
	can_msg_transmit.data[2] = (sdo_OD_idx >> 8)&0xFF;
	can_msg_transmit.data[3] = sdo_OD_sidx & 0xFF;
	/*Handle little endian in caller function!*/
	switch (num_bytes_unused & 0b11) {
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
void can_send_heartbeat(uint8_t state)
{
	Message can_msg_transmit;
	can_msg_transmit.cob_id = 0x700 + CAN_NODE_ID;
	can_msg_transmit.len = 1;
	can_msg_transmit.rtr = 0;
	can_msg_transmit.data[0] = state & 127;
	can_send_msg(&can_msg_transmit);
}
#endif
