'''
	dsPIC33E bootloader with support for CAN (CANopen) and UART
	Ken Caluwaerts <ken@caluwaerts.eu> 2014
	
	Copyright (C) 2014, Ken Caluwaerts
	All rights reserved.
	
	The CAN(CANopen) and UART bootloader for the dsPIC33E bootloader is licensed
	under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at
	http://www.apache.org/licenses/LICENSE-2.0.
	
	Unless required by applicable law or agreed to in writing,
	software distributed under the License is distributed on an
	"AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
	either express or implied. See the License for the specific language
	governing permissions and limitations under the License.
'''
import numpy as np
import time
import io
import argparse
import sys
import os
try:
	import serial
except ImportError:
	print "Could not import python serial"
try:
	import can
except ImportError:
	print "Could not import python CAN"

canopenshell_loc = '/home/super/canfestival/CanFestival-3-a82d867e7850/examples/CANOpenShell/CANOpenShell'
cansocket_loc = '/home/super/canfestival/CanFestival-3-a82d867e7850/drivers/can_socket/libcanfestival_can_socket.so'

bootloader_cmd = {"READ_ID":0x09,"WRITE_PM":0x03,"ACK":0x01,"NACK":0x00,"RESET":0x08}

def open_port(device = '/dev/ttyUSB0', baudrate=115200):
	'''
		Opens a serial port and returns its handle.
	'''
	return serial.Serial(device,baudrate)

def read_id(port):
	'''
		Reads the device id and revision of the uC.
	'''
	port.write(bytearray([bootloader_cmd["READ_ID"]]))
	tmp = port.read(8)
	dev_id = tmp[1::-1].encode('hex') #endianness
	dev_revision = tmp[5:3:-1].encode('hex')
	return dev_id, dev_revision
	
def parse_hex(hex_file,memory):
	'''
		Parses a hex file (provided as a list of strings) and copies its contents to a memory object.
	'''
	ext_address = 0
	for line in hex_file:
		#parse format
		byte_count = int(line[1:3],base=16)
		address = int(line[3:7],base=16)
		record_type = int(line[7:9],base=16)
		if(record_type==1):
			print "EOF record"
		elif(record_type==4):
			print "Extended address"
			ext_address = int(line[9:13],base=16)<<16
		elif(record_type==0):
			address = (ext_address+address)/2
			#for i in xrange(bytecount)
			print "data: %d bytes at address %d \t %.5x"%(byte_count,address,address)
			#instruction "==4bytes 00xxxxxx" per iteration 
			for i in xrange(byte_count/4):
				cur_address = address+i*2#addresses increase in steps of two
				opcode_little_endian = line[9+i*8:9+(i+1)*8]
				opcode = opcode_little_endian[6:8]+opcode_little_endian[4:6]+opcode_little_endian[2:4]+opcode_little_endian[0:2]
				opcode_num = int(opcode,base=16)
				print "address: %.6x opcode: %.6x"%(cur_address,opcode_num)
				memory.write(cur_address,(0,(opcode_num>>16)&0xFF,(opcode_num>>8)&0xFF,opcode_num&0xFF))
	
def load_hex_file(file_name):
	'''
		Opens a hex file and loads its contents into a list.
	'''
	f = open(file_name,'rb')
	hex_file = [l for l in f]
	f.close()
	return hex_file
	
def program_uc(memory,dev_id,port):
	#read device id
	dev_id_r, dev_rev_r = read_id(port)
	if(int(dev_id_r,base=16)==dev_id):
		print "device IDs match %s"%dev_id_r
	else:
		raise "device IDs do not match %x - %s"%(dev_id,dev_id_r)
	#get pages to program
	pic_mem, pic_mem_addr = memory.data_to_transmit()
	for i, idx in enumerate(pic_mem_addr):
		print "programming page %d/%d: \t %.6x"%(i,pic_mem_addr.shape[0],idx)
		#send program page command
		port.write(bytearray([bootloader_cmd["WRITE_PM"]]))
		time.sleep(0.01)
		#send address
		port.write(bytearray([idx&0xFF,(idx>>8)&0xFF,(idx>>16)&0xFF])) #little endian
		#send page data
		for j in xrange(pic_mem.shape[1]):
			port.write(bytearray([pic_mem[i,j,2]])) #little endian
			port.write(bytearray([pic_mem[i,j,1]]))
			port.write(bytearray([pic_mem[i,j,0]]))
		#read acknowledgment
		reply = ord(port.read(1))		
		if(reply==bootloader_cmd["ACK"]):
			print "success"
		else:
			print "failed: %x"%reply
			break
		
	print "programming complete, resetting microcontroller"
	#send reset command
	time.sleep(0.01)
	port.write(bytearray([bootloader_cmd["RESET"]]))
	
def write_uC_code_memory(memory,dev_id,fname):
	'''
		Writes the microcontroller program memory (non-zero) to a file.
		The resulting file can be programmed using the CANOpen bootloader.
	'''
	#write header
	#byte 0		uint8		magic byte (0x00 = program memory)
	#byte 1-2	uint16		number of lines in file
	#byte 3-6	uint32		device id
	#byte 7-N			page to program (lines)
	#				byte 0		uint8	magic byte (0x00 = write page to uC memory)
	#				byte 1-2	uint16	number of instructions on page 
	#				byte 3-5	uint24	page address	
	#				byte 6-8,9-11...uint24	instructions to program		
	#read device id
	with io.FileIO(fname,'w') as stream:
		stream.write(bytearray([0x00])) #magic byte
	
		pic_mem, pic_mem_addr = memory.data_to_transmit()
		stream.write(bytearray([pic_mem.shape[0]>>8,pic_mem.shape[0]&0xFF])) #number of lines
	
		stream.write(bytearray([dev_id>>24,(dev_id>>16)&0xFF,(dev_id>>8)&0xFF,(dev_id)&0xFF])) #dev id
	
		#program memory lines
		for i, idx in enumerate(pic_mem_addr):
			print "writing program page %d/%d: \t %.6x"%(i,pic_mem_addr.shape[0],idx)
			stream.write(bytearray([0x00]))
			stream.write(bytearray([0x04,0x00]))#1024 instructions per page
			stream.write(bytearray([idx&0xFF,(idx>>8)&0xFF,(idx>>16)&0xFF])) #page address little endian
			#send page data
			for j in xrange(pic_mem.shape[1]):
				stream.write(bytearray([pic_mem[i,j,2]])) #little endian
				stream.write(bytearray([pic_mem[i,j,1]]))
				stream.write(bytearray([pic_mem[i,j,0]]))	
	
class pic_memory(object):
	def __init__(self,num_pages=171):
		self.data = np.zeros((num_pages*1024,4),dtype=np.uint8) #just one big continuous chunk of memory. Note that addresses increase in steps of two
		self.tags = np.zeros(num_pages,dtype=np.uint8) #0 = empty, 1 = dirty program memory
		
	def write(self, address, data):#data is assumed to be in the format phantombyte(0) 23..16 15..8 7..0
		'''
			Stores an instruction in the memory object.
			Data is supposed to be a list/array of 4 uint8s (bytes). The first is a phantom byte (0).
		'''
		address=int(address) #just to make sure
		mem_address = address>>1 #addresses increase in steps of two
		page_address = mem_address>>10
		self.tags[page_address] = 1#mark as dirty
		self.data[mem_address] = data
		
	def data_to_transmit(self):
		'''
			Creates a list of dirty pages to transmit to the microcontroller.
			Returns a numpy uint8 array (N by 1024 by 3, no phantom byte uint8) and a numpy array of page addresses (uint).
		'''
		N = np.sum(self.tags==1)
		pic_mem = np.zeros((N,1024,3),dtype=np.uint8)
		pic_mem_addr = np.where(self.tags==1)[0]<<11 #multiply addresses by 2048 (1024 instructions in steps of two)
		for i, idx in enumerate(pic_mem_addr):
			pic_mem[i] = self.data[idx>>1:(idx>>1)+1024,1:]
		return pic_mem, pic_mem_addr
	
	def set_boot_address(self,address=0x800):
		'''
			Changes the goto instruction that is executed when the uC boots up.
			Address should be an unsigned int.
		'''
		self.write(0x0,(0x00,0x04,(address>>8)&0xFF,address&0xFE)) #0x0004 is a GOTO instruction (http://ww1.microchip.com/downloads/en/DeviceDoc/70157C.pdf page 196)
		self.write(0x2,(0x00,0x00,0x00,(address>>16)&0x7F))
		
def open_can_bus(busname='can0'):
	return can.interface.Bus(busname)

def upload_code_canopen_raw(memory, node_id, bus):
	'''
		Upload the program memory over CAN to a microcontroller by using raw CANopen messages.
		This code does NOT check for errors and should only be used for testing purposes.
	'''
	pic_mem, pic_mem_addr = memory.data_to_transmit()
	#reset uC
	print "Python hack resetting the microcontroller"
	msg = can.Message(arbitration_id=0x0,data=[129,node_id],extended_id=False)
	bus.send(msg)
	time.sleep(1) #wait a bit for the uC to reset
	#send data
	for i, idx in enumerate(pic_mem_addr):
		print "Writing program page using python hack %d/%d: \t %.6x"%(i,pic_mem_addr.shape[0],idx)
		#SDO download initiate
		msg = can.Message(arbitration_id=0x600+node_id,data=[0b00100001,0x50,0x1F,0x01,0,0,0,0],extended_id=False)
		bus.send(msg)
		time.sleep(0.05)
		data = []
		
		data.extend([idx&0xFF,(idx>>8)&0xFF,(idx>>16)&0xFF]) #page address little endian
		for j in xrange(pic_mem.shape[1]):
			data.extend([pic_mem[i,j,2]]) #little endian
			data.extend([pic_mem[i,j,1]])
			data.extend([pic_mem[i,j,0]])
		
		toggle = 0
		j = 0
		last_msg = 0
		num_bytes = len(data)
		while(not last_msg):
			#There are way more efficient ways of doing this, but it's just a hack
			data_msg = np.zeros(7,dtype=np.uint8)
			n = 0
			for k in xrange(7):
				if(j+k>=num_bytes):
					last_msg = 1
					n = 7-k
					break
				else:
					data_msg[k] = data[j+k] 
			msg = can.Message(arbitration_id=0x600+node_id,data=[toggle<<4|n<<1|last_msg,data_msg[0],data_msg[1],data_msg[2],data_msg[3],data_msg[4],data_msg[5],data_msg[6]],extended_id=False)
			bus.send(msg)
			toggle = not toggle
			j+=7
		
		time.sleep(0.1)
	#start uC
	msg = can.Message(arbitration_id=0x0,data=[1,node_id],extended_id=False)
	bus.send(msg)


if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='dsPIC33E bootloader with support for CAN (CANopen) and UART',epilog="Ken Caluwaerts <ken@caluwaerts.eu> 2014")
	parser.add_argument("--interface","-i",choices=["UART","CAN"],default="CAN",help="Hardware interface: CAN or UART. Default CAN.")
	parser.add_argument("--output","-o",type=str,help="Output: the filename of the generated binary file in case CAN is used (default == input filename.bin), the hardware interface in case of UART (default /dev/ttyUSB0)",default=None)
	parser.add_argument("--devid","-d",default=0x1f65,type=int,help="Device id (write as decimal number, e.g. 0x1f65 should be written as 8037). Default 0x1F65.")
	parser.add_argument("--bootaddress","-b",type=int,default=0x800,help="Bootloader address (write as decimal number, e.g. 0x800 should be written as 2048). Default 0x800.")
	parser.add_argument("--donotmodifybootaddress","-a", action="store_true", help="Do not modify boot address of the HEX file. (Default: modify).")
	parser.add_argument("--baudrate","-r",type=int,default=115200,help="UART baudrate (Default: 115200)")
	parser.add_argument("--uploadcan","-u",action="store_true",help="Uploads the firmware over CAN (see --canuploadmethod to define the implementation)")
	parser.add_argument("--canuploadmethod","-m",choices=["CANfestival","python"],default="CANfestival",help="How to upload firmware over CAN (if -u enabled). Using CANopen and CANfestival ('CANfestival') or raw python CAN messages ('python'). (Default: 'CANfestival')")
	parser.add_argument("--nodeid","-n",type=int, default=3,help="CANopen node ID (see --uploadcan). (Default: 3)")
	parser.add_argument("--canbus","-c",type=str,default="can0", help="Which can bus to use (only relevant when -u and -m python are active) (Default: 'can0')")
	parser.add_argument("hexfile",type=str,help="Input HEX file (typically generated by MPLAB X)")
	try:
		args = parser.parse_args()
	except:
		print "Unexpected error:", sys.exc_info()[0]
		parser.print_help() 
		sys.exit(-1)
		
	boot_address = args.bootaddress
	dev_id = args.devid
	fname = args.hexfile
	iface = args.interface
	output = args.output
	modify_boot_address = not args.donotmodifybootaddress
	baudrate = args.baudrate
	uploadcan = args.uploadcan
	node_id = args.nodeid
	canuploadmethod = args.canuploadmethod
	canbus = args.canbus
	if(iface=="UART"):
		#UART
		hex_file = load_hex_file(fname)
		memory = pic_memory()
		parse_hex(hex_file,memory)
		if(modify_boot_address):
			print "Modifying boot address"
			memory.set_boot_address(boot_address)	
		if(output is None):
			output="/dev/ttyUSB0"
		print "Opening serial port"
		port = open_port(output,baudrate)
		print "Programming microcontroller"
		program_uc(memory,dev_id,port)
	else:
		#CAN
		hex_file = load_hex_file(fname)
		memory = pic_memory()
		parse_hex(hex_file,memory)
		if(modify_boot_address):
			print "Modifying boot address"
			memory.set_boot_address(boot_address)	
		if(output is None):
			output=os.path.splitext(fname)[0]+".bin"
		print "Writing program memory to file (%s)"%output
		write_uC_code_memory(memory,dev_id,output)
		if(uploadcan):
			if(canuploadmethod=="CANfestival"):
				import subprocess
				subprocess.call([canopenshell_loc, 'load#%s,can0,1000,2,0'%cansocket_loc, 'srst#3', 'wait#1', 'bldr#%d,%s'%(node_id,output), 'ssta#3', 'wait#5', 'quit'])
			else:
				bus = open_can_bus(canbus)
				upload_code_canopen_raw(memory, node_id, bus)
