/*
 * CAN(CANopen) and UART bootloader for the dsPIC33E.
 *
 * The UART section of this file is loosely based on the AN1094 application
 * note from Microchip, which is available at:
 * http://www.microchip.com/stellent/idcplg?IdcService=SS_GET_PAGE&nodeId=1824&appnote=en530200
 *
 * The CAN implementation is not fully CANopen compliant, but is expected to
 * be compatible with most CANopen implementations (tested using CANfestival).
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
#include <pps.h>
#include "p33Exxxx.h"
#include "bootloader.h"
#include "canbus.h"

//_FOSCSEL(FNOSC_FRCPLL & IESO_OFF & PWMLOCK_OFF);
///*PWMLOCK_OFF needed to configure PWM*/
//_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE);
//_FWDT(FWDTEN_OFF);
///* Disable JTAG */
//_FICD(JTAGEN_OFF); /* & ICS_PGD2);*/
//_FGS(GCP_OFF); /* Disable Code Protection */

/*DO NOT USE CONFIGURATION MEMORY (it triggers code protection in the bootloader) */
_FGS(GWRP_OFF & GSS_OFF & GSSK_OFF); // Disable Code Protection
_FOSCSEL(FNOSC_FRC & IESO_OFF);
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE);
_FWDT(FWDTEN_OFF);
_FICD(ICS_PGD1 & JTAGEN_OFF);// & RSTPRI_AF);


/*defined in memory.s*/
extern uint32_t read_latch(uint16_t, uint16_t);
extern void reset_device();
//extern void write_row_pm_33E(uint16_t,uint16_t,ureg32_t*);

/*This buffer is used to write/read program memory internally*/
char buffer[PM_ROW_SIZE * 3 + 1];

#ifndef BL_UART
/*This buffer is used by the CAN code to temporary store program memory*/
uint8_t can_prog_data[PM_ROW_SIZE*3+4];
#endif

int main(void)
 {
    ureg32_t source_addr;
    ureg32_t delay;
    /*
     * 70 MIPS:
     * FOSC = Fin*(PLLDIV+2)/((PLLPRE+2)*2(PLLPOST+1))
     * FOSC = 7.37MHz(74+2)/((0+2)*2(0+1)) = 140.03Mhz
     * Fcy = FOSC/2
     */
//    PLLFBD = 74;
//    CLKDIVbits.PLLPRE = 0;
//    CLKDIVbits.PLLPOST = 0;
//    while (OSCCONbits.LOCK != 1);
//    RCONbits.SWDTEN = 0; /* Disable Watch Dog Timer*/

    PLLFBD = 74;
    CLKDIVbits.PLLPRE = 0;
    CLKDIVbits.PLLPOST = 0;

    // Initiate Clock Switch to FRC oscillator with PLL (NOSC=0b001)
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(OSCCON | 0x01);

    // Wait for Clock switch to occur
    while (OSCCONbits.COSC != 0b001);

    while (OSCCONbits.LOCK!= 1);

    source_addr.val32 = 0x1000;

    delay.val32 = BOOT_DELAY; //ReadLatch(SourceAddr.Word.HW, SourceAddr.Word.LW);

    if (delay.val[0] == 0) {
        reset_device();
    }

    /*Disable all interrupts*/
    INTCON2bits.GIE = 0;

    /*Timer setup */
    T2CONbits.T32 = 1; /* increments every instruction cycle */
    IFS0bits.T3IF = 0; /* Clear the Timer3 Interrupt Flag */
    IEC0bits.T3IE = 0; /* Disable Timer3 Interrup Service Routine */

    if ((delay.val32 & 0x000000FF) != 0xFF) {
        /* Convert seconds into timer count value */
        delay.val32 = ((uint32_t) (FCY)) * ((uint32_t) (delay.val[0]));
        PR3 = delay.word.HW;
        PR2 = delay.word.LW;
        TMR3HLD = 0x0000;
        TMR2 = 0x0000;
        T2CONbits.TON = 1;
    }

    /*Configure pins*/
    /*Set all pins to input*/    
    TRISB = 0xFFFF;
    TRISC = 0xFFFF;
    TRISD = 0xFFFF;
    TRISE = 0xFFFF;
    TRISF = 0xFFFF;
    TRISG = 0xFFFF;

    /*Configure all analog ports as digital I/O*/
    ANSELB = 0;
    ANSELC = 0;
    ANSELD = 0;
    ANSELE = 0;
    ANSELG = 0;

    /*LEDs*/
    #define TRIS_LED1     TRISBbits.TRISB8
    #define TRIS_LED2     TRISBbits.TRISB9
    #define TRIS_LED3     TRISBbits.TRISB10
    #define TRIS_LED4     TRISBbits.TRISB11

    #define LED1     LATBbits.LATB8
    #define LED2     LATBbits.LATB9
    #define LED3     LATBbits.LATB10
    #define LED4     LATBbits.LATB11
    TRIS_LED1 = 0;
    TRIS_LED2 = 0;
    TRIS_LED3 = 0;
    TRIS_LED4 = 0;

    LED1 = LED2 = LED3 = LED4 = 0;

    /*Disable modules that are not needed at this point*/
    SPI1STATbits.SPIEN = 0;
    SPI2STATbits.SPIEN = 0;
    SPI3STATbits.SPIEN = 0;
    SPI4STATbits.SPIEN = 0;

    PTCONbits.PTEN = 0;

    I2C1CONbits.I2CEN = 0;
    I2C2CONbits.I2CEN = 0;

    U1STAbits.UTXEN = 0;
    U2STAbits.UTXEN = 0;
    U3STAbits.UTXEN = 0;
    U4STAbits.UTXEN = 0;

    U1CONbits.USBEN = 0;

    PMCONbits.PMPEN = 0;

    LED1 = 1;
    
#ifdef BL_UART
    /* Serial version of the bootloader */
    char command;

    OUT_PIN_PPS_RP68 = OUT_FN_PPS_U2TX; //U2Tx
    IN_FN_PPS_U2RX = IN_PIN_PPS_RP67; //U2Rx

    U2BRG = BRGVAL; /*  BAUD Rate Setting of Uart2  */
    U2MODE = 0x8000; /* Reset UART to 8-n-1, alt pins, and enable */
    U2MODEbits.BRGH = 0;
    U2STA = 0x0400; /* Reset status register and enable TX */

    while (1) {
        get_char(&command);
        switch (command) {
            case COMMAND_WRITE_PM: /* tested */
            {
                int size;
                /*Get the page address*/
                get_char(&(source_addr.val[0]));
                get_char(&(source_addr.val[1]));
                get_char(&(source_addr.val[2]));
                source_addr.val[3] = 0;

                /*Get the page contents*/
                for (size = 0; size < PM_ROW_SIZE * 3; size++) {
                    get_char(&(buffer[size]));
                }
                LED4 = !LED4;
                /*Erase the page*/
                erase_33E(source_addr.word.HW, source_addr.word.LW);
                write_PM(buffer, source_addr); /*program page */

                put_char(COMMAND_ACK); /*Send Acknowledgement */
                break;
            }
            case COMMAND_READ_ID:
            {
                ureg32_t temp;

                source_addr.val32 = 0xFF0000;
                temp.val32 = read_latch(source_addr.word.HW, source_addr.word.LW);
                write_buffer(&(temp.val[0]), 4);
                source_addr.val32 = 0xFF0002;
                temp.val32 = read_latch(source_addr.word.HW, source_addr.word.LW);
                write_buffer(&(temp.val[0]), 4);

                break;
            }	
            case COMMAND_RESET:
            case COMMAND_NACK:
            {
                reset_device();
                break;
            }
            default:
                put_char(COMMAND_NACK);
                break;
        }

    }
#endif
/*Don't replace this with #else (ifndefs can be toggled in MPLAB X) */
#ifndef BL_UART
    /* CAN version of the bootloader*/
    can_init();
    LED2 = 1;
    can_states state = CAN_STATE_BOOTUP;
    uint8_t can_msg_received = 0;
    uint8_t can_msg_transmitted = 0;
    Message can_msg_rec;
    uint8_t can_exp_buffer[4]; /*Expedited data buffer*/
    ureg32_t temp;
    uint8_t sdo_ccs, sdo_n, sdo_e, sdo_s;
    uint16_t sdo_OD_idx;
    uint8_t sdo_OD_sidx;
    uint16_t can_heartbeat_time = 100; /*Heartbeat producer period in ms*/
    unsigned can_disable_boot_timeout = 1;
    ureg32_t delay_hb;
    unsigned can_receiving_prog_data = 0; /*is program data upload in progress?*/
    uint16_t can_prog_data_ctr = 0; /*number of data bytes received for data upload*/
    uint8_t can_toggle = 0;
    uint8_t can_toggle_received;
    uint8_t can_last_segment;
    uint8_t can_segment_n;
    uint8_t can_segment_num_bytes;
    uint16_t can_num_segments_ctr;
    uint16_t i;
    extern uint8_t txreq_bitarray;

    can_msg_rec.cob_id = 0;
    can_msg_rec.len = 0;
    can_msg_rec.rtr = 0;
    
    /*Set up a timer with a 1ms period*/
    T4CONbits.T32 = 1; /* to increment every instruction cycle */
    IFS1bits.T5IF = 0; /* Clear the Timer5 Interrupt Flag */
    IEC1bits.T5IE = 0; /* Disable Timer5 Interrup Service Routine */

    /* Convert seconds into timer count value */
    delay_hb.val32 = (((uint32_t) (FCY)) / ((uint32_t) 1000)) * ((uint32_t) (can_heartbeat_time));
    PR5 = delay_hb.word.HW;
    PR4 = delay_hb.word.LW;
    TMR5HLD = 0x0000;
    TMR4 = 0x0000;
    /* Enable Timer */
    if(can_heartbeat_time>0){
        T4CONbits.TON = 1;
    } else{
        T4CONbits.TON = 0;
    }

    //TEST
//    source_addr.val32 = 0x2000;//8372224L;
//    erase_33E(source_addr.word.HW, source_addr.word.LW);
//    for(i=0;i<1024*3;++i){
//        buffer[i] = 'A';
//    }
//    buffer[0] = 'K';
//    buffer[1] = 'e';
//    buffer[2] = 'n';
//    buffer[3] = '!';
//    buffer[4] = '!';
//    buffer[5] = '!';
//    write_PM(buffer, source_addr);

    while(1){
        if(!can_disable_boot_timeout && IFS0bits.T3IF){
            /*Bootloader timeout*/
            can_send_heartbeat(4);
            reset_device();
        }

	// Handle the non blocking CAN transmission sequence 
		if (txreq_bitarray & 0b00000001 && !C1TR01CONbits.TXREQ0) {
			C1TR01CONbits.TXREQ0 = 1;
			txreq_bitarray = txreq_bitarray & 0b11111110;
		}
		if (txreq_bitarray & 0b00000010 && !C1TR01CONbits.TXREQ1) {
			C1TR01CONbits.TXREQ1 = 1;
			txreq_bitarray = txreq_bitarray & 0b11111101;
		}
		if (txreq_bitarray & 0b00000100 && !C1TR23CONbits.TXREQ2) {
			C1TR23CONbits.TXREQ2 = 1;
			txreq_bitarray = txreq_bitarray & 0b11111011;
		}
		if (txreq_bitarray & 0b00001000 && !C1TR23CONbits.TXREQ3) {
			C1TR23CONbits.TXREQ3 = 1;
			txreq_bitarray = txreq_bitarray & 0b11110111;
		}
		if (txreq_bitarray & 0b00010000 && !C1TR45CONbits.TXREQ4) {
			C1TR45CONbits.TXREQ4 = 1;
			txreq_bitarray = txreq_bitarray & 0b11101111;
		}
		if (txreq_bitarray & 0b00100000 && !C1TR45CONbits.TXREQ5) {
			C1TR45CONbits.TXREQ5 = 1;
			txreq_bitarray = txreq_bitarray & 0b11011111;
		}
		if (txreq_bitarray & 0b01000000 && !C1TR67CONbits.TXREQ6) {
			C1TR67CONbits.TXREQ6 = 1;
			txreq_bitarray = txreq_bitarray & 0b10111111;
		}

        /*Receive a CAN message*/
        can_msg_received = can_receive_msg(&can_msg_rec);
        if(can_msg_received){
            LED3 = !LED3;
        }
        /* Clear transmitted interrupt flag */
        if (C1INTFbits.TBIF) {
            C1INTFbits.TBIF = 0; //message was transmitted, nothing to do
            can_msg_transmitted = 1; //can be used by the state machine
        }
        IFS2bits.C1IF = 0; //clear CAN interrupt flag
        
        if(IFS1bits.T5IF){
            LED2 = !LED2;
        }

        /*State machine*/
        switch(state){
            case CAN_STATE_BOOTUP:
                /* CAN Initialization state */
                /* Send bootup message*/
                can_send_heartbeat(0);
                state = CAN_STATE_BOOTUP_WAIT;
                break;
            case CAN_STATE_BOOTUP_WAIT:
                if(can_msg_transmitted){
                    state = CAN_STATE_BOOTUP_PREOP;
                }
                /*incoming messages are ignored in this state*/
                break;
            case CAN_STATE_BOOTUP_PREOP:
                /*This is the default state*/
                /*Check whether a heartbeat message needs to be sent*/
                if(IFS1bits.T5IF == 1){
                    can_send_heartbeat(127);
                    IFS1bits.T5IF = 0;
                }
                
                if(can_msg_received && (can_msg_rec.cob_id==0x0)){
                   /*Check whether we received an NMT master message*/
                    if(can_msg_rec.data[1]==0x00 || can_msg_rec.data[1]==CAN_NODE_ID){
                        /*Message addressed to this node or all nodes*/
                        switch(can_msg_rec.data[0]){ /*requested mode*/
                            case 1: /*Operational*/
                                /*Switch to main program*/
                                reset_device();
                                break;
                            case 2: /*Stopped*/
                                /*TODO: add support for this?*/
                                /*Do nothing*/
                                break;
                            case 128: /*Pre-operational*/
                                /*Do nothing*/
                                break;
                            case 129: /*Reset node*/
                                /*Reset bootloader*/
                                asm("goto 0x800");
                                Nop();
                                Nop();
                                Nop();
                                //asm("RESET");
                                break;
                            case 130: /*Reset communication*/
                                /*TODO: do we need to do anything here?*/
                                /*Do nothing*/
                                break;
                            default:
                                /*Do nothing*/
                                break;
                        };
                    }
                } /*Check whether we received an SDO read/write*/
                else if(can_msg_received && (can_msg_rec.cob_id==0x600+CAN_NODE_ID)){ /*There's only a single SDO channel*/
                    sdo_ccs = (can_msg_rec.data[0]>>5)&0b111;
                    sdo_s = (can_msg_rec.data[0])&0b1;
                    sdo_e = (can_msg_rec.data[0]>>1)&0b1;
                    sdo_n = (can_msg_rec.data[0]>>2)&0b11;
                    sdo_OD_idx = (((uint16_t)can_msg_rec.data[2])<<8)|(can_msg_rec.data[1]&0xFF); //note the endianness
                    sdo_OD_sidx = can_msg_rec.data[3];
                    switch(sdo_ccs){
                        case 0:
                            /*SDO download request (i.e. a write in progress)*/
                            if(!can_receiving_prog_data){
                                can_send_SDO_abort(0x1F50,0x01,0x08000000);
                            } else {
                                can_toggle_received = (can_msg_rec.data[0]&0b10000)>>4;
                                if(can_toggle_received == can_toggle){
                                    //OK, correct toggle received
                                    can_toggle ^= 1;
                                } else {
                                    //toggle error
                                    can_receiving_prog_data = 0;
                                    can_send_SDO_abort(0x1F50,0x01,0x05030000);
                                    break;
                                }
                                can_num_segments_ctr++;
                                can_last_segment = can_msg_rec.data[0]&0b1;
                                can_segment_n = (can_msg_rec.data[0]>>1)&0b111;
                                can_segment_num_bytes = 7-can_segment_n;

                                if(can_prog_data_ctr+can_segment_num_bytes<=(PM_ROW_SIZE*3+3)){
                                    /*copy data to buffer*/
                                    for(i=0;i<can_segment_num_bytes;++i){
                                        can_prog_data[can_prog_data_ctr++]=can_msg_rec.data[i+1];
                                    }
                                } else {
                                    /*Too much data received, abort*/
                                    can_receiving_prog_data = 0;
                                    can_send_SDO_abort(0x1F50,0x01,0x06070012);
                                    break;
                                }

                                if(can_last_segment){
                                    /*Check if all data was received*/
                                    if(can_prog_data_ctr==(PM_ROW_SIZE*3+3)){
                                        /*Everything seems OK, program the memory page*/
                                        can_receiving_prog_data = 0;
                                        source_addr.val[0] = can_prog_data[0];
                                        source_addr.val[1] = can_prog_data[1];
                                        source_addr.val[2] = can_prog_data[2];
                                        source_addr.val[3] = 0;
                                        if(source_addr.val32>0){
                                            can_prog_data_ctr--;
                                        }

                                        /*Get the page contents*/
                                        for (i = 0; i < PM_ROW_SIZE * 3; i++) {
                                            buffer[i] = can_prog_data[i+3];
                                        }
                                        LED4 = !LED4;
                                        erase_33E(source_addr.word.HW, source_addr.word.LW);
                                        write_PM(buffer, source_addr);
                                        
                                        can_send_sdo_download_segment_response(can_toggle_received);
                                    } else {
                                        /*wrong data size, abort*/
                                        can_receiving_prog_data = 0;
                                        can_send_SDO_abort(0x1F50,0x01,0x06070010);
                                        break;
                                    }
                                } else{
                                    /*Continue receiving data*/
                                    can_send_sdo_download_segment_response(can_toggle_received);
                                }
                            }
                            break;
                        case 1:
                            /*Initiate SDO download request (i.e. a write request)*/
                            can_receiving_prog_data = 0; /*If we receive an SDO download request during a program upload: ABORT */
                            switch(sdo_OD_idx){
                                case 0x1017:
                                    /*Heartbeat producer*/
                                    if(sdo_OD_sidx==0x00){
                                        /*Producer Heartbeat Time*/
                                        can_heartbeat_time = (can_msg_rec.data[4]&0xFF) | (((uint16_t) can_msg_rec.data[5]) << 8);
                                        /* Convert seconds into timer count value */
                                        delay_hb.val32 = (((uint32_t) (FCY)) / ((uint32_t) 1000)) * ((uint32_t) (can_heartbeat_time));
                                        T4CONbits.TON = 0;
                                        PR5 = delay_hb.word.HW;
                                        PR4 = delay_hb.word.LW;
                                        TMR5HLD = 0x0000;
                                        TMR4 = 0x0000;
                                        /* Enable Timer if HB time>0 */
                                        if(can_heartbeat_time>0){
                                            T4CONbits.TON = 1;
                                        }
                                        can_send_sdo_download_response(sdo_OD_idx,sdo_OD_sidx);
                                    } else {
                                        can_send_unsupported_OD(sdo_OD_idx,sdo_OD_sidx);
                                    }
                                    break;
                                case 0x1f50:
                                    /*Write program memory*/
                                    switch(sdo_OD_sidx){
                                        case 0x01:
                                            can_disable_boot_timeout = 1; /*Disable timeout once we start receiving program data*/
                                            /*Set up data structures for program memory upload*/
                                            can_receiving_prog_data = 1;

                                            can_prog_data_ctr = 0; //number of data bytes received
                                            //TODO: in the Canfestival implementation, the first 4 bytes contain the number of bytes to send
                                            //use this to check if the correct number of bytes has been received.
//                                            for(i=0;i<can_prog_data_ctr;++i){
//                                                can_prog_data[i] = can_msg_rec.data[4+i];
//                                            }
                                            //can_num_segments_ctr = 1;
                                            can_toggle = 0;
                                            can_send_sdo_download_response(sdo_OD_idx,sdo_OD_sidx);
                                            break;
                                        default:
                                           can_send_unsupported_OD(sdo_OD_idx,sdo_OD_sidx);
                                    };
                                case 0x1f51:
                                    /*Program Control*/
                                    switch(sdo_OD_sidx){
                                        case 0x01:
                                            /*Program control state: */   
                                            if(can_msg_rec.data[4] == 0x01){
                                                /*Writing a 1 to this object starts the main program*/
                                                can_send_sdo_download_response(sdo_OD_idx,sdo_OD_sidx);
                                                /*Wait until SDO response is transmitted*/
                                                while(C1TR01CONbits.TXREQ1 == 1);
                                                reset_device();
                                            } else if(can_msg_rec.data[4]==0x00){
                                                /*Writing 0 to this object disables the boot timeout timer*/
                                                can_disable_boot_timeout = 1;
                                                can_send_sdo_download_response(sdo_OD_idx,sdo_OD_sidx);
                                            }
                                            break;
                                        default:
                                            can_send_unsupported_OD(sdo_OD_idx,sdo_OD_sidx);
                                    };
                                    break;
                                default:
                                    can_send_unsupported_OD(sdo_OD_idx,sdo_OD_sidx);
                                    break;
                            };
                            break;
                        case 2:
                            /*Initiate SDO upload request (i.e. a read request)*/
                            can_receiving_prog_data = 0; /*If we receive an SDO upload request during a program upload: ABORT */
                            switch(sdo_OD_idx){
                                case 0x1000: /*Device type*/
                                    if(sdo_OD_sidx==0x00){
                                        /*Read device type information*/
                                        can_exp_buffer[0] = CAN_DEVICE_TYPE[3];//'t';
                                        can_exp_buffer[1] = CAN_DEVICE_TYPE[2];//'o';
                                        can_exp_buffer[2] = CAN_DEVICE_TYPE[1];//'o';
                                        can_exp_buffer[3] = CAN_DEVICE_TYPE[0];//'b';
                                        can_send_sdo_upload_response_expedited(sdo_OD_idx,sdo_OD_sidx, 0, can_exp_buffer);
                                    } else {
                                        can_send_unsupported_OD(sdo_OD_idx,sdo_OD_sidx);
                                    }
                                    break;
                                case 0x1001: /*Error registry*/
                                    if(sdo_OD_sidx==0x00){
                                        /*Read error registry*/
                                        can_exp_buffer[0] = 0; /*TODO: implement error registry values*/
                                        can_send_sdo_upload_response_expedited(sdo_OD_idx,sdo_OD_sidx, 3, can_exp_buffer);
                                    } else {
                                        can_send_unsupported_OD(sdo_OD_idx,sdo_OD_sidx);
                                    }
                                    break;
                                case 0x1017:
                                    /*Heartbeat producer*/
                                    if(sdo_OD_sidx==0x00){
                                        /*Producer Heartbeat Time*/
                                        can_exp_buffer[0] = can_heartbeat_time&0xFF;
                                        can_exp_buffer[1] = can_heartbeat_time>>8;
                                        can_send_sdo_upload_response_expedited(sdo_OD_idx,sdo_OD_sidx, 2, can_exp_buffer);
                                    } else {
                                        can_send_unsupported_OD(sdo_OD_idx,sdo_OD_sidx);
                                    }
                                    break;
                                case 0x1018: /*Identity*/
                                    switch(sdo_OD_sidx){
                                        case 0x00:
                                            /*Number of entries*/
                                            can_exp_buffer[0] = 4;
                                            can_send_sdo_upload_response_expedited(sdo_OD_idx,sdo_OD_sidx, 3, can_exp_buffer);
                                            break;
                                        case 0x01:
                                            /*Vendor ID*/
                                            can_exp_buffer[3] = CAN_VENDOR_ID[0];
                                            can_exp_buffer[2] = CAN_VENDOR_ID[1];
                                            can_exp_buffer[1] = CAN_VENDOR_ID[2];
                                            can_exp_buffer[0] = CAN_VENDOR_ID[3];
                                            can_send_sdo_upload_response_expedited(sdo_OD_idx,sdo_OD_sidx, 0, can_exp_buffer);
                                            break;
                                        case 0x02:
                                            /*Product code: read device id*/
                                            source_addr.val32 = 0xFF0000;
                                            temp.val32 = read_latch(source_addr.word.HW, source_addr.word.LW);
                                            can_exp_buffer[3] = temp.val[0];
                                            can_exp_buffer[2] = temp.val[1];
                                            can_exp_buffer[1] = temp.val[2];
                                            can_exp_buffer[0] = temp.val[3];
                                            can_send_sdo_upload_response_expedited(sdo_OD_idx,sdo_OD_sidx, 0, can_exp_buffer);
                                            break;
                                        case 0x03:
                                            /*Revision number*/
                                            can_exp_buffer[3] = CAN_REVISION_NUM[0];
                                            can_exp_buffer[2] = CAN_REVISION_NUM[1];
                                            can_exp_buffer[1] = CAN_REVISION_NUM[2];
                                            can_exp_buffer[0] = CAN_REVISION_NUM[3];
                                            can_send_sdo_upload_response_expedited(sdo_OD_idx,sdo_OD_sidx, 0, can_exp_buffer);
                                            break;
                                        case 0x04:
                                            /*Serial number*/
                                            can_exp_buffer[3] = CAN_SERIAL_NUMBER[0];
                                            can_exp_buffer[2] = CAN_SERIAL_NUMBER[1];
                                            can_exp_buffer[1] = CAN_SERIAL_NUMBER[2];
                                            can_exp_buffer[0] = CAN_SERIAL_NUMBER[3];
                                            can_send_sdo_upload_response_expedited(sdo_OD_idx,sdo_OD_sidx, 0, can_exp_buffer);
                                            break;
                                        default:
                                            can_send_unsupported_OD(sdo_OD_idx,sdo_OD_sidx);
                                    };
                                    break;
                                case 0x1f50:
                                    /*No support for reading the firmware, yet*/
                                    switch(sdo_OD_sidx){
                                        case 0x00:
                                            /*Number of entries*/
                                            can_exp_buffer[0] = 1;
                                            can_send_sdo_upload_response_expedited(sdo_OD_idx,sdo_OD_sidx, 3, can_exp_buffer);
                                            break;
                                        default:
                                            can_send_unsupported_OD(sdo_OD_idx,sdo_OD_sidx);
                                    };
                                    break;
                                case 0x1f51:
                                    /*Program Control*/
                                    switch(sdo_OD_sidx){
                                        case 0x00:
                                            /*Number of entries*/
                                            can_exp_buffer[0] = 1;
                                            can_send_sdo_upload_response_expedited(sdo_OD_idx,sdo_OD_sidx, 3, can_exp_buffer);
                                            break;
                                        case 0x01:
                                            /*Program control state: always 0 (we're in bootloader mode).*/
                                            /*TODO: This entry should read 1 in normal execution mode (main program).*/                                            can_exp_buffer[0] = 0x00;
                                            can_send_sdo_upload_response_expedited(sdo_OD_idx,sdo_OD_sidx, 3, can_exp_buffer);
                                            break;
                                        default:
                                            can_send_unsupported_OD(sdo_OD_idx,sdo_OD_sidx);
                                    };
                                    break;
                                default:
                                    /*unsupported OD*/
                                    can_send_unsupported_OD(sdo_OD_idx,sdo_OD_sidx);
                            };
                            break;
                        case 3:
                            /*SDO upload request (i.e. a continued read request)*/
                            can_receiving_prog_data = 0; /*If we receive an SDO download request during a program upload: ABORT */
                            can_send_unsupported_OD(sdo_OD_idx,sdo_OD_sidx);
                            break;
                    };
                }
                break;
            default:
                /* Error: restart the bootloader */
                asm("goto 0x800");
                Nop();
                Nop();
                Nop();
                break;
        };
        can_msg_transmitted = 0;
        can_msg_received = 0;
    }
#endif

}

#ifdef BL_UART
/******************************************************************************/
void get_char(char * ptr_char) {
    while (1) {
        /* check timer expired*/
        if (IFS0bits.T3IF == 1) {
            *ptr_char = COMMAND_NACK;
            break;
        }
        /* check for UART receive errors */
        if (U2STAbits.FERR == 1) {
            continue;
        }

        /* must clear the overrun error to keep uart receiving */
        if (U2STAbits.OERR == 1) {
            U2STAbits.OERR = 0;
            continue;
        }

        /* get the data */
        if (U2STAbits.URXDA == 1) {
            T2CONbits.TON = 0; /* Disable timer countdown */
            *ptr_char = U2RXREG;
            break;
        }
    }
}

/******************************************************************************/
void put_char(char data) {
    while (!U2STAbits.TRMT);
    U2TXREG = data;
}



/******************************************************************************/

void write_buffer(char * ptr_data, int size) {
    unsigned i;
    for (i = 0; i < size; i++) {
        put_char(ptr_data[i]);
    }
}
#endif

/******************************************************************************/

void write_PM(char * ptrData, ureg32_t source_addr) {
#ifdef DOUBLE_WORD_WRITE
    ureg32_t temp[2];
    ureg32_t temp_addr;
    ureg32_t temp_data;
    unsigned i;

    for (i = 0; i < PM_ROW_SIZE; i += 2) {
        temp_addr.val32 = source_addr.val32 + (i<<1);
        temp[0].val[3] = 0; //phantom byte
        temp[0].val[2] = ptrData[i*3+2];
        temp[0].val[1] = ptrData[i*3+1];
        temp[0].val[0] = ptrData[i*3];
        temp[1].val[3] = 0; //phantom byte
        temp[1].val[2] = ptrData[i*3+5];
        temp[1].val[1] = ptrData[i*3+4];
        temp[1].val[0] = ptrData[i*3+3];
        write_pm_33E(temp_addr.word.HW, temp_addr.word.LW, temp);
    }
#else
    ureg32_t temp[128];
    ureg32_t temp_addr;
    ureg32_t temp_data;
    unsigned i,j;
    uint16_t row_offset;
    //write a single program word
    for(j=0;j<8;++j){
        row_offset = j<<7;
        for (i = 0; i < 128; i++) {
            temp[i].val[3] = 0; //phantom byte
            temp[i].val[2] = ptrData[i*3+2+row_offset];
            temp[i].val[1] = ptrData[i*3+1+row_offset];
            temp[i].val[0] = ptrData[i*3+row_offset];
        }
        temp_addr.val32 = source_addr.val32 + (row_offset<<1);
        write_row_pm_33E(temp_addr.word.HW, temp_addr.word.LW, temp);
    }
#endif
}

/******************************************************************************/



