/*
 * CAN(CANopen) and UART bootloader for the dsPIC33E bootloader.
 *
 * Example project (main program).
 *
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
 *
 */
#include <stdint.h>
#include "p33Exxxx.h"

#define FCY   70000000 //70Mhz
#define BAUDRATE        115200
#define BRGVAL          37 //((FCY/BAUDRATE)/16)-1
#define CAN_NODE_ID 0x3

typedef struct {
  uint16_t cob_id;	/**< message's ID */
  uint8_t rtr;		/**< remote transmission request. (0 if not rtr message, 1 if rtr message) */
  uint8_t len;		/**< message's length (0 to 8) */
  uint8_t data[8];      /**< message's data */
} Message;


/*DO NOT USE CONFIGURATION MEMORY (it triggers code protection in the bootloader) */
//_FOSCSEL(FNOSC_FRCPLL & IESO_OFF & PWMLOCK_OFF); //PWMLOCK_OFF needed to configure PWM
//_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE);
//_FWDT(FWDTEN_OFF);
///* Disable JTAG */
////_FICD(JTAGEN_OFF); // & ICS_PGD2);
//
//_FGS(GCP_OFF); // Disable Code Protection

void test_uart() {
    U2MODEbits.UEN = 0b00; //disable UART2

    RPINR18bits.U1RXR = 47; //pin 26==RPI47 is RX input
    _RP36R = 0b01; //U2TX to RP36 (pin 11)

    /*TX*/
    U1MODEbits.UEN = 0b00;
    U1STAbits.UTXEN = 1;
    U1STAbits.UTXISEL0 = 0;
    U1STAbits.UTXISEL1 = 0;

    U1MODEbits.BRGH = 1;
    U1BRG = 34; //500000 baud/s
    IEC0bits.U1TXIE = 1;
    IFS0bits.U1TXIF = 0;
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN = 1; //DO THIS AFTER UARTEN OR IT WON'T WORK!!!

    /*RX*/
    IEC0bits.U1RXIE = 1; //enable receive interrupt
    U1STAbits.URXISEL = 00; //Interrupt flag bit is set when a character is received

    _U1TXInterrupt();
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void) {
    IFS0bits.U1TXIF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U1ErrInterrupt(void) {
    IFS0bits.U1TXIF = 0;
    U1STAbits.OERR = 0;
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
    /*UART echo*/
    unsigned tmp = U1RXREG & 0xFF; //read byte
    U1TXREG = tmp;
    IFS0bits.U1RXIF = 0;
}

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

    IEC2bits.C1IE = 1; // Enable interrupts for ECAN1 peripheral
    C1INTEbits.TBIE = 0; // Disable TX buffer interrupt
    C1INTEbits.RBIE = 1; // Enable RX buffer interrupt
    C1INTEbits.ERRIE = 0;
    C1INTEbits.IVRIE = 0;
    C1INTEbits.RBOVIE = 0;
}


void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void) {
    uint8_t ide = 0;
    uint8_t srr = 0;
    uint32_t id = 0;
    uint8_t index_byte;
    uint16_t buffer;
    Message m;
    uint16_t *ecan_msg_buf_ptr;
    static uint8_t packet_idx;
    unsigned i;
    uint16_t j;
    if (C1INTFbits.RBIF) {
        m.cob_id=0xFFFF;
        m.rtr=1;
        m.len=255;

        // Obtain the buffer the message was stored into, checking that the value is valid to refer to a buffer
        if (C1VECbits.ICODE < 32) {
            buffer = C1VECbits.ICODE;
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
            m.cob_id = (uint32_t) ((ecan_msg_buf_ptr[0] & 0x1FFC) >> 2);
        } else {
            //ehm, error. only std messages supported for now
        }

        /* If message is a remote transmit request, mark it as such.
         * Otherwise it will be a regular transmission so fill its
         * payload with the relevant data.
         */
        if (srr == 1) {
            m.rtr = 1; /*not supported*/
        } else {
            m.rtr = 0;
            m.len = (uint8_t) (ecan_msg_buf_ptr[2] & 0x000F);
            m.data[0] = (uint8_t) (ecan_msg_buf_ptr[3]&0xFF);
            m.data[1] = (uint8_t) ((ecan_msg_buf_ptr[3] & 0xFF00) >> 8);
            m.data[2] = (uint8_t) (ecan_msg_buf_ptr[4]&0xFF);
            m.data[3] = (uint8_t) ((ecan_msg_buf_ptr[4] & 0xFF00) >> 8);
            m.data[4] = (uint8_t) (ecan_msg_buf_ptr[5]&0xFF);
            m.data[5] = (uint8_t) ((ecan_msg_buf_ptr[5] & 0xFF00) >> 8);
            m.data[6] = (uint8_t) (ecan_msg_buf_ptr[6]&0xFF);
            m.data[7] = (uint8_t) ((ecan_msg_buf_ptr[6] & 0xFF00) >> 8);
        }

        if(m.cob_id==0x0){
            if(m.data[1] == 0x0 || m.data[1] == CAN_NODE_ID){
                if(m.data[0]==129){
                    //reset
                    asm("RESET");
                    Nop();
                    Nop();
                    Nop();
                }
            }
        }
        // Make sure to clear the interrupt flag.
        C1RXFUL1 = 0;
        C1INTFbits.RBIF = 0;
        return 1;
    }
    return 0;
}

int main(void) {
    /*
     * 70 MIPS:
     * FOSC = Fin*(PLLDIV+2)/((PLLPRE+2)*2(PLLPOST+1))
     * FOSC = 7.37MHz(74+2)/((0+2)*2(0+1)) = 140.03Mhz
     * Fcy = FOSC/2
     */
    PLLFBD = 74;
    CLKDIVbits.PLLPRE = 0;
    CLKDIVbits.PLLPOST = 0;
    while (OSCCONbits.LOCK != 1);

    RCONbits.SWDTEN = 0; /* Disable Watch Dog Timer*/

    /*Configure pins*/
    //Set all pins to input
    TRISA = 0xFFFF;
    TRISB = 0xFFFF;

    TRISBbits.TRISB4 = 0; //RP36 is output (UART2TX)
    TRISBbits.TRISB12 = 0; //RB12 is output (test pin)
    //Configure all analog ports as digital I/O
    ANSELA = 0x0;
    ANSELB = 0x0;

    test_uart(); /*UART echo */

    can_init();

    TRISBbits.TRISB12 = 0; //LED

    unsigned long a;
    while (1) {
        //for (a = 0; a < 1500000; ++a);
        //for (a = 0; a < 1500000; ++a);
        for (a = 0; a < 500000; ++a);
        LATBbits.LATB12 = 1;
        for (a = 0; a < 500000; ++a);
        LATBbits.LATB12 = 0;
    }



}
