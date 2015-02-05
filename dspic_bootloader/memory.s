;********************************************************************
;
; Assembly code to read and write program memory on the dsPIC33E and
; PIC24E.
;
; Reading program memory is straightforward:
; Load the page address into TBLPAG and use tblrdl and tblrdh to read
; the instruction (24 bits).
;
; Writing to program memory is a two-step process: 
; Erase a full page of program memory and then write the program memory
; of the erased page using the write latches.
; The write latches (typically at address 0xFA0000) temporary store instructions.
; These instructions are then written to the program memory space by 
; writing a key sequence and setting NVMCON to the correct value.
; The addresses to write are defined by TBLWTH/TBLWTL instructions.
;
; NOTE: do not write to the last page of the Flash Configuration.
; This triggers the code protection feature and will prevent subsequent
; writes to program memory.
; If you don't set configuration bits in your main program, 
; this shouldn't happen.
;
; Ken Caluwaerts 2014 <ken@caluwaerts.eu>
;
; Copyright © 2014, Ken Caluwaerts
; All rights reserved.
;
; The CAN(CANopen) and UART bootloader for the dsPIC33E bootloader is licensed
; under the Apache License, Version 2.0 (the "License");
; you may not use this file except in compliance with the License.
; You may obtain a copy of the License at
; http://www.apache.org/licenses/LICENSE-2.0.
;
; Unless required by applicable law or agreed to in writing,
;  software distributed under the License is distributed on an
;  "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
;  either express or implied. See the License for the specific language
; governing permissions and limitations under the License.
;
;********************************************************************/

.include "p33Exxxx.inc"
.global _write_row_pm_33E, _read_latch, _reset_device, _erase_33E, _write_pm_33E ;C called

_read_latch: 
	mov	W0,TBLPAG
    NOP
	tblrdl [W1],W0
	tblrdh [W1],W1
	return

_reset_device:
    goto 0x2000 ; Start address of the main code
    NOP
    NOP
    NOP

_erase_33E:
    ; Define the start address of the page to erase
    ;.equ PROG_ADDR, 0x1000
    ; Set up the NVMADR registers to the starting address of the page
    ;MOV #tblpage(PROG_ADDR),W0
    ;;MOV W0,NVMADRU
    ;MOV #tbloffset(PROG_ADDR),W0
    ;;MOV W1,NVMADR
    ; Set up NVMCON to erase one page of Program Memory
    ;;MOV #0x4003,W0
    ;;MOV W0,NVMCON
    ; Disable interrupts < priority 7 for next 5 instructions
    ; Assumes no level 7 peripheral interrupts
    ;.equ PROG_ADDR, 0x0000
    ;MOV #tblpage(PROG_ADDR),W0
    MOV W0,NVMADRU
    ;MOV #tbloffset(PROG_ADDR),W0
    MOV W1,NVMADR
    MOV #0x4003,W0
    MOV W0,NVMCON
    DISI #06
    ; Write the KEY Sequence
    MOV #0x55,W0
    MOV W0,NVMKEY
    MOV #0xAA,W0
    MOV W0,NVMKEY
    ; Start the erase operation
    BSET NVMCON,#15
    ; Insert two NOPs after the erase cycle (required)
    NOP
    NOP
    return

_write_pm_33E:
    ; Define the address from where the programming has to start
    ;.equ PROG_ADDR, 0x01800;
    ; Load the destination address to be written
    ;MOV #tblpage(PROG_ADDR),W9
    ;MOV #tbloffset(PROG_ADDR),W8
    MOV W0,NVMADRU
    MOV W1,NVMADR
    ;MOV W9,NVMADRU
    ;MOV W8,NVMADR;
    ; Load the two words into the latches
    ; W2 points to the address of the data to write to the latches
    ; Set up a pointer to the first latch location to be written
    MOV #0xFA,W0
    MOV W0,TBLPAG
    MOV #0,W1
    ; Perform the TBLWT instructions to write the latches
    TBLWTL [W2++],[W1]
    TBLWTH [W2++],[W1++]
    TBLWTL [W2++],[W1]
    TBLWTH [W2++],[W1++]
    ; Setup NVMCON for word programming
    MOV #0x4001,W0
    MOV W0,NVMCON
    ; Disable interrupts < priority 7 for next 5 instructions
    ; Assumes no level 7 peripheral interrupts
    DISI #06
    ; Write the key sequence
    MOV #0x55,W0
    MOV W0,NVMKEY
    MOV #0xAA,W0
    MOV W0,NVMKEY
    ; Start the write cycle
    BSET NVMCON,#15
    NOP
    NOP
    return




_write_row_pm_33E:
; Define the address from where the programming has to start
;.equ PROG_ADDR, 0x01800;
; Load the destination address to be written
; Writes a full row (128 instructions) to program memory.
; Make sure that your device supports this!
;push W9
;push W8
;push W3
; Define the address from where the programming has to start
;.equ PROG_ADDR, 0x02000
; Load the NVMADR register with the starting programming address
;;MOV #tblpage(PROG_ADDR),W9
;MOV #tbloffset(PROG_ADDR),W8
;MOV W9,NVMADRU
;MOV W8,NVMADR
MOV W0,NVMADRU
MOV W1,NVMADR

; Setup NVMCON to write 1 row of program memory
MOV #0x4002,W0
MOV W0,NVMCON
; Load the program memory write latches
; This example loads 128 write latches
; W2 points to the address of the data to write to the latches
; Set up a pointer to the first latch location to be written
MOV #0xFA,W0
MOV W0,TBLPAG
MOV #0,W1
; Perform the TBLWT instructions to write the latches
; W2 is incremented in the TBLWTH instruction to point to the
; next instruction location
MOV #128,W0
loop:
;TBLWTL.b [W2++], [W1++]
;TBLWTL.b [W2++], [W1--]
;TBLWTH.b [W2++], [W1]
TBLWTL [W2++],[W1]
TBLWTH [W2++],[W1++]
TBLWTL [W2++],[W1]
TBLWTH [W2++],[W1++]
;INC2 W1, W1
DEC W0, W0
BRA NZ, loop
; Disable interrupts < priority 7 for next 5 instructions
; Assumes no level 7 peripheral interrupts
DISI #06
; Write the KEY sequence
MOV #0x55,W0
MOV W0,NVMKEY
MOV #0xAA,W0
MOV W0,NVMKEY
; Start the programming sequence
BSET NVMCON,#15
; Insert two NOPs after programming
NOP
NOP
;pop W3
;pop W8
;pop W9
return

.end
