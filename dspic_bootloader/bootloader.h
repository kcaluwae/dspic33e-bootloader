/*
 * CAN(CANopen) and UART bootloader for the dsPIC33E.
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
 */
#ifndef BOOTLOADER_H
#define	BOOTLOADER_H

#include <stdint.h>
#ifdef	__cplusplus
extern "C" {
#endif

/*Enable this to use the serial version of the bootloader*/
//#define BL_UART
/*Enable this if your device does not support row writing.
  If you are not sure, then enable it (all devices support
  double word writes). The 806/810 devices appear to support
  row writes (e.g. dsPIC33EPXXXMU8YY), while the 502/506/510
  devices are limited to double word writes (e.g. dsPIC33EP256GP506).*/
//#define DOUBLE_WORD_WRITE
    

/*
* Program memory page size (1024 instructions for the dsPIC33E
*   See e.g. TABLE 1/2 on pages 2-... (page erase size column) for this value
*   http://ww1.microchip.com/downloads/en/DeviceDoc/70000657H.pdf
*/
#define PM_ROW_SIZE     1024
#define FCY             70000000 //70Mhz
#define BOOT_DELAY      10  /*Bootloader timeout in seconds*/
#define CAN_NODE_ID 0x02
#define MOTOR_BOARD
//#ifndef MOTOR_BOARD
#define DOUBLE_WORD_WRITE //for some reason the ROW programming mode only works for a single page
//#endif
static const char CAN_DEVICE_TYPE[] = "boot";       /*4 bytes*/
static const char CAN_VENDOR_ID[] = "NASA";         /*4 bytes*/
static const uint8_t CAN_REVISION_NUM[] = {0,1,0,1};/*4 bytes*/
static const char CAN_SERIAL_NUMBER[] = "Ken.";     /*4 bytes*/

//#define BL_UART
#ifdef BL_UART
/*UART specific code*/
#define COMMAND_NACK     0x00
#define COMMAND_ACK      0x01
/*#define COMMAND_READ_PM  0x02*/
#define COMMAND_WRITE_PM 0x03
#define COMMAND_RESET    0x08
#define COMMAND_READ_ID  0x09
#define BAUDRATE        115200
#define BRGVAL          37 //((FCY/BAUDRATE)/16)-1

void put_char(char);
void get_char(char*);
void write_buffer(char*, int);
#endif

typedef union ureg32 {
    uint32_t val32;
    struct {
        uint16_t LW;
        uint16_t HW;
    } word;
    char val[4];
} ureg32_t;

void write_PM(char *, ureg32_t);

#ifdef	__cplusplus
}
#endif

#endif	/* BOOTLOADER_H */

