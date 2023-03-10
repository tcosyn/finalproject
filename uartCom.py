import serial as serial
import time 

PCB_UART = serial.Serial('/dev/serial0', 9600, timeout=0)#connecting PCB UART Baud rate at 115200
print("connected to: " + PCB_UART.portstr)

def extractBits():
    bytes_recieved = serial.read(2)
    bytes_direction = bytes_recieved[0]
    bytes_speed = bytes_recieved[1]
    motor_dir_byte = bytes_direction
    motor_dir_byte = motor_dir_byte & 0b00000011
    motor_speed_byte = bytes_speed
    # Shift the motor direction byte to the left by 2 bits
    # and OR it with the motor speed byte to get a 10-bit sequence
    result = (motor_speed_byte << 2) | motor_dir_byte
    # Format the result as a binary string with 10 digits
    binary_string = "{0:010b}".format(result)
    # Print the binary string
    print(binary_string)
    to_send = result
    serial.write(to_send.encode())
    serial.flushOutput()

while True:
    extractBits()
    time.sleep(2)
