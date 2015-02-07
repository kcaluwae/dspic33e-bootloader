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

    // Initiate Clock Switch to FRC oscillator with PLL (NOSC=0b001)
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(OSCCON | 0x01);

    // Wait for Clock switch to occur
    while (OSCCONbits.COSC != 0b001);

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

    TRISBbits.TRISB12 = 0; //LED

    unsigned long a;
    while (1) {
        for (a = 0; a < 1500000; ++a);
        for (a = 0; a < 1500000; ++a);
        for (a = 0; a < 500000; ++a);
        LATBbits.LATB12 = !LATBbits.LATB12;
        for (a = 0; a < 500000; ++a);
        LATBbits.LATB12 = !LATBbits.LATB12;
    }



}
