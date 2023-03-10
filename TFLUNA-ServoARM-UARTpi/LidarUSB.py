#author:Milind Devnani
import serial
import time

bytesAllowed = 8
checkBytes = 89

print("Now connecting to LiDAR")
Lidar_UART = serial.Serial("/dev/ttyUSB0", 115200,timeout=0) # Connecting Lidar UART at baud rate 115200

def getDistance(LidarBytes):
    if LidarBytes[0] == checkBytes:
                if LidarBytes[1] == checkBytes: # check first two bytes
                    dist = LidarBytes[2] + LidarBytes[3]*256 # distance in next two bytes
                    dist = dist/100.0
                    return dist

#reading bytes from TF UART and then calling seprate function to calculate distance 
def readLidarBytes():
    while not False:
        numBytes = Lidar_UART.in_waiting # count the number of bytes of the serial port
        if bytesAllowed < numBytes:
            LidarBytes = Lidar_UART.read(9) # From TF_LUNA take the 9 bytes of data coming in, we only care about the distance bytes
            Lidar_UART.reset_input_buffer() # clearing the UART Communication to get updated data
            dist = getDistance(LidarBytes)
            return dist

while True:
    if Lidar_UART.isOpen() == False:#checking if lidar UART is open
        Lidar_UART.open() # opening Lidar UART commnunication if not already open
    
    try:
        dist = readLidarBytes() # calling function to take in Lidar data
        #print("Actual Distance Read: ",dist)
        newDistValue = int(dist*100) #scaling up my Luna distance values from meter to cm
        print("Distance is cm ",newDistValue)
            
    except:
        dist = 2
        motors_speed = 127 #distance to keep
        
Lidar_UART.close() # close serial port
