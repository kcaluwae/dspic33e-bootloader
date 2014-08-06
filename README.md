dsPIC33E-bootloader 
===================

CAN (CANopen) and UART bootloader for Microchip dsPIC33E and related microcontrollers.

Features
------------------
- dsPIC33E/PIC24E programming using a serial or CAN connection
- Client (host) written in Python with CAN uploader available in C and Python
- CANopen compatible
- UART uses 1 memory page, CAN uses 2 or 3 (depending on optimization level) pages
- Tested on Linux (Ubuntu) and the BeagleBone Black (Angstrom)

Design
------------------
####Target (microcontroller)
The bootloader uses the first memory page(s) after the interrupt vector table (typically at address 0x800 for devices with 1024 instructions/page). The main application should start on the next free page.

The bootloader receives a full program memory page through CAN or UART and then proceeds to write the page to flash memory. This process is repeated for each non-empty page.

The CAN version of the bootloader is activated by disabling the BL_UART define. It provides a basic CANopen implementation (heartbeat, NMT, identity and bootloader objects). The program data is transferred through object 0x1F50, subindex 0x01 (SDO).

Interrupts cannot/should not be used in the bootloader (overwritten by the target application).

####Host
The main application (/host/python) is written in python and can be used to read Hex files and write them to a device. A basic CAN uploader is provided in Python and a CANopen compatible uploader is integrated into the CanFestival framework. 
####Directory Structure
- dspic_bootloader: MPLAB X project for the bootloader (program this to the device once).
- dspic_main_example: MPLAB X project containing a correctly configured sample application that can be programmed to a device using the bootloader.
- host: Python and CanFestival tools to upload the code from a host computer to a microcontroller.
- media: Some videos demonstrating how to use the Python host script.

dsPIC33E vs dsPIC33F
------------------
The memory layout and programming interfaces of the dsPIC33E and dsPIC33F (and PIC24E/PIC24F) series differ significantly. The most important change is probably the page size. The dsPIC33E has 1024 or 512 instructions per page (we only tested the 1024 instructions version). You can find more information about the programming options for your device in table 1 of http://ww1.microchip.com/downloads/en/DeviceDoc/70000657H.pdf (or the datasheet of your device).

Reading program memory is straightforward:
Load the page address into TBLPAG and use tblrdl and tblrdh to read
the instruction (24 bits).

Writing to program memory is a two-step process: 
Erase a full page of program memory and then write the program memory
of the erased page using the write latches.
The write latches (typically at address 0xFA0000) temporary store instructions.
These instructions are then written to the program memory space by 
writing a key sequence and setting NVMCON to the correct value.
The addresses to write are defined by TBLWTH/TBLWTL instructions.

All dsPIC33Es have at least two write latches (i.e. instructions are programmed in pairs) and some devices have 128 (one row) write latches. The bootloader currently only supports two write latches. This does not affect the performance in practice. See http://ww1.microchip.com/downloads/en/DeviceDoc/70609D.pdf for more information.

Usage
------------------
####Bootloader project configuration
The bootloader program should start on the second page of program memory (address 0x800 on devices with 1024 instruction pages). If not, programming the interrupt vector table would overwrite the bootloader. 
This is easily accomplished by modifying the linker script (see linker.gld) provided by Microchip (typically located at /opt/microchip/xc16/v1.11/support/dsPIC33E/gld/). 

Change these three lines: 

program (xr)   : ORIGIN = 0x800,         LENGTH = 0x2A7EA (decrease LENGTH accordingly)

__CODE_BASE = 0x800; 

__CODE_LENGTH = 0x2A7EC; (decrease LENGTH accordingly)

####Target Project Configuration
The target applications should start on a new page after the bootloader. This is typically third page (UART) or fifth page (CAN, non space-optimized) (page 3 = 0x1000, page 4 = 0x800, page 5 = 0x2000 on devices with 1024 instruction pages). 
This is easily accomplished by modifying the linker script (see linker.gld). 

Change these three lines (decrease LENGTH!): 

program (xr)   : ORIGIN = 0x2000,        LENGTH = 0x28FFE 

__CODE_BASE = 0x2000; 

__CODE_LENGTH = 0x29000;

The target application should not set flash configuration bytes (e.g. _FOSCSEL(FNOSC_FRCPLL & IESO_OFF & PWMLOCK_OFF); ). These values are stored in the last page of program memory and erasing them in the bootloader triggers code protection (see Section 27.1 in http://ww1.microchip.com/downloads/en/DeviceDoc/70000657H.pdf).

####Uploading an Application to the Device
The dspic33E_bootloader.py script in host/python loads a Hex file and writes it to a file or device using UART or CAN ('python dspic33E_bootloader.py -h' to see all options, http://youtu.be/kFbsQ2aVa68 or http://youtu.be/svuVLHAyaHY for usage examples). 

The general flow of this tool is: load and parse a hex file, detect memory pages to program, write raw memory pages to file and or device.

#####UART
The UART uses the same protocol as the AN1094 bootloader by Microchip. This protocol is very simple (a single command byte followed by a datastream, if any) and not robust (no error checking!). Therefore, additional checks should be added when the UART version is to be used in production.

The UART uploader is activated using the '-i UART' argument:
python dspic33E_bootloader.py -i UART -o '/dev/ttyUSB0' -r 115200 dspic_main_example.production.hex

#####CAN
The data transfer is based on the CANopen standard. More precisely the object dictionary entry at index 0x1F50, subindex 0x01 is used to program memory pages. Data is transferred one memory page at a time using SDO (addres 0x600+node id) and block transfers are not supported. The first 3 bytes contain the page address, followed by the 24 bit instructions of the current page. The main program can be started by writing 0x01 to object 0x1F51, subindex 0x01 or sending an NMT message to put the device in operational mode. The bootloader is not fully CANopen compliant (deemed unnecessary), but should be compatible with most CANopen devices.

The python script provides two methods to program the device. 

First, we provide a pure python version (based on python-can). This implementation simply sends the raw CAN messages generated by the CANopen client, assuming there are no errors. The CANopen protocol is not implemented and this tool should thus only be used for debugging purposes. The advantage is that a CANopen stack is not required on the host.

To use this implementation, use the '-m python' flag:
python dspic33E_bootloader.py -u -n 3 -m python dspic_main_example.production.hex

Second, a data uploader has been integrated into CanFestival (/host/canfestival). This implementation is CANopen compatible and will detect errors etc. Note that the default CanFestival maximum SDO transfer size is only 32 bytes. Increase this in the 'configure' file of CanFestival and recompile it. The python tool can (edit the source code to modify the CanFestival and socket-can paths) run this implementation directly:
python dspic33E_bootloader.py -u -n 3 dspic_main_example.production.hex

####Caveats
- No data verification is currently used on the microcontroller. CAN/CANopen provides CRC and other mechanisms, so this is mostly an issue for the UART version.


License
------------------
Apache 2

Ken Caluwaerts <ken@caluwaerts.eu> 2014
