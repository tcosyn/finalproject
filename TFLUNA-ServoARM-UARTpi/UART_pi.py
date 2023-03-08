#!?usr/bin/env python
# -*- coding: utf-8 -*-
# lsusb to check device name
#dmesg | grep "tty" to find port name

import serial
import time

#PCB_UART = serial.Serial('/dev/serial0', 9600, timeout=1)#connecting PCB UART Baud rate at 9600
#print("connected to: " + PCB_UART.portstr)

PCB_UART = serial.Serial('/dev/serial0', 9600, timeout=0)#connecting PCB UART Baud rate at 115200
print("connected to: " + PCB_UART.portstr)

while True:
	command = input("write your char\n") + "\n"

	print(command)
	#command = "T\n"
	if command == 's':
		PCB_UART.close()
		break
	PCB_UART.write(command.encode())
	#PCB_UART.write(command.encode('UTF-8'))
	time.sleep(0.06)
	print("String sent\n")
	
	print(PCB_UART.readlines())
	#data_from_uart = PCB_UART.read(1)
	#data_from_uart = PCB_UART.read()
	#print(data_from_uart) #read a byte
	#PCB_UART.write(command)
	#s = ''
	#while PCB_UART.inWaiting():
		#s += PCB_UART.read()
	#print(s)
	time.sleep(2)
