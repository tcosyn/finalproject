#author: Milind Devnani
#there is no work around to not using serial and time
import serial
import time

bytesAllowed = 8
checkBytes = 89

Threshold_distance = 84.4
Threshold_distance_min = Threshold_distance -2
Threshold_distance_max = Threshold_distance +2


Lidar_minimum_distance = 84 #cm
Lidar_maximum_distance = 210 #cm

#motor speed can goto 0-255 as motor driver accepts analogue value
motor_min_speed = 100
motor_max_speed = 255

motors_speed = 127 #motor driver accpets 0-255 value, at start we are sending normal speed value

#starting arduino communication
PCB_UART = ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)#connecting PCB UART Baud rate at 9600
print("connected to: " + PCB_UART.portstr)

print("Now connecting to LiDAR")
Lidar_UART = serial.Serial("/dev/serial0", 115200,timeout=0) # Connecting Lidar UART at baud rate 115200

def MapDistaceToMotorSpeed(newDistValue):
    return int((newDistValue-Lidar_minimum_distance)/(Lidar_maximum_distance - Lidar_minimum_distance)*(motor_max_speed-motor_min_speed)+motor_min_speed)
    
    
    
#calculating distance value from TF bytes recieved
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
        print("Actual Distance Read: ",dist)
        newDistValue = dist*100 #scaling up my Luna distance values from meter to cm
        
        
        motors_speed = MapDistaceToMotorSpeed(newDistValue);
        msg = "Distance Reading for PCB %s cm"%newDistValue
        print(msg)
        
        if newDistValue > Threshold_distance:
            #move fast
            msg = "motor speeding up" 
            print(msg)
            
        if (newDistValue < Threshold_distance) and (newDistValue < 30):
            #very very slow or stop
            msg = "motor almost stopped" 
            print(msg)
        
        elif newDistValue == Threshold_distance:
            #maintain speed
            msg = "motors at normal speed" 
            print(msg)
        
        elif newDistValue < Threshold_distance:
            #slow_down
            msg = "motor slowing down" 
            print(msg)
            
    except:
        dist = 2
        motors_speed = 127 #distance to keep
        
    command = str(motors_speed)+'?'
    print("this is my command", command)
    PCB_UART.write(command.encode())
    #PCB_UART.write(command)
    time.sleep(0.2)
    print("cycle completed.<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
    #data = PCB_UART.readlines()
    #print(data)

Lidar_UART.close() # close serial port
