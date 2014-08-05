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

#ifndef CANBUS_H
#define	CANBUS_H

#include <stdint.h>
#include "bootloader.h"

#ifdef	__cplusplus
extern "C" {
#endif


#ifndef BL_UART

typedef struct {
  uint16_t cob_id;	/**< message's ID */
  uint8_t rtr;		/**< remote transmission request. (0 if not rtr message, 1 if rtr message) */
  uint8_t len;		/**< message's length (0 to 8) */
  uint8_t data[8];      /**< message's data */
} Message;

void can_init();
uint8_t can_receive_msg(Message *m);
void can_send_msg(Message *m);
void can_send_SDO_abort(uint16_t sdo_OD_idx, uint8_t sdo_OD_sidx, uint32_t error);
void can_send_unsupported_OD(uint16_t sdo_OD_idx, uint8_t sdo_OD_sidx);
void can_send_sdo_download_response(uint16_t sdo_OD_idx, uint8_t sdo_OD_sidx);
void can_send_sdo_download_segment_response(uint8_t toggle);
void can_send_sdo_upload_response_expedited(uint16_t sdo_OD_idx, uint8_t sdo_OD_sidx, unsigned num_bytes_unused, uint8_t* can_exp_buffer);
void can_send_heartbeat(uint8_t state);

typedef enum {CAN_STATE_BOOTUP, CAN_STATE_BOOTUP_WAIT, CAN_STATE_BOOTUP_PREOP} can_states;

#endif

#ifdef	__cplusplus
}
#endif

#endif	/* CANBUS_H */

