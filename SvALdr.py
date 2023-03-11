#author: Milind Devnani
from time import sleep
import RPi.GPIO as GPIO
import math
import serial
import time

distance = 0

def setServoAngleUpDown(angle):
    servo = 17
    
    #print("Gpio setup")
    GPIO.setup(servo, GPIO.OUT)
    
    #print("setup gpio")
    pwm = GPIO.PWM(servo, 50)
    
    #print("pwm start")
    pwm.start(8)
    dutyCycle = angle / 18. + 3.
    
    #print(f"setting duty cycle: {dutyCycle}")
    pwm.ChangeDutyCycle(dutyCycle)
    #print("------\n")
    #sleep(0.01)


def setServoAngleLeftRight(angle):
    servo = 27
    GPIO.setup(servo, GPIO.OUT)
    pwm = GPIO.PWM(servo, 50)
    pwm.start(8)
    dutyCycle = angle / 18. + 3.
    pwm.ChangeDutyCycle(dutyCycle)
    #sleep(0.01)

def init():
    global bytesAllowed, checkBytes, Lidar_UART, currAngle

    bytesAllowed = 11
    checkBytes = 89
    currAngle = 78
    distance = 0

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    print("Now connecting to LiDAR")
    Lidar_UART = serial.Serial("/dev/ttyUSB0", 115200, timeout=0)  # Connecting Lidar UART at baud rate 115200


    # IMPORTANT, WE HAVE TO MAKE SURE THE ARM STARTS AT 90 DEGREES AT STARTUP
    for i in range(4):
        setServoAngleUpDown(78)
        #sleep(0.05)



def adjustAngle(x):
    a = .00093137255
    b = .8818627451
    c = -8.911
    newAng = (a * (x ** 2)) + (b * x) + c

    if newAng < 0:
        newAng = 0
    elif newAng > 180:
        newAng = 180

    return newAng

def getDistance(LidarBytes):
    for i in range(9):
        if LidarBytes[i] == checkBytes:
            if LidarBytes[i+1] == checkBytes: # check first two byte
                print("distance retrieved")
                dist = LidarBytes[i+2] + LidarBytes[i+3]*256 # distance in next two bytes
                dist = dist/100.0
                print(f"distance (cm): {dist*100}")
                return dist
        
    return -1



# reading bytes from TF UART and then calling seprate function to calculate distance
def readLidarBytes():
    #while not False:
    tsRLB = time.time()

    while not False:
        print("count numbytes")
        numBytes = Lidar_UART.in_waiting  # count the number of bytes of the serial port
        print(f"numbytes = {numBytes}")
        if bytesAllowed < numBytes:
            # From TF_LUNA take the 11 bytes of data coming in, we only care about the distance bytes
            print("reading lidar bits")
            LidarBytes = Lidar_UART.read(11)

            # clearing the UART Communication to get updated data
            print("resetting input buffer")
            Lidar_UART.reset_input_buffer()
            
            print("getting dist")
            tsGD = time.time()
            dist = getDistance(LidarBytes)
            teGD = time.time()
            print(f"runtime: {(teGD-tsGD)}")

            return dist


def calculateAngle(zoffset, yoffset):
    global currAngle
    flag = 1
    angle = math.atan(yoffset / zoffset) * 180 / math.pi
    currAngle += 90
    #currAngle += flag * angle
    #currAngle = max(0, min(180, currAngle))
    return currAngle




def LidarLatestDistance():
    tsLLD = time.time()
    print("checking if lidar is open")
    if Lidar_UART.isOpen() == False:  # checking if lidar UART is open
        print("lidar not open, opening lidar")
        Lidar_UART.open()  # opening Lidar UART commnunication if not already open

    try:
        print("calling lidarfunc")
        tsRLB = time.time()
        dist = readLidarBytes()  # calling function to take in Lidar data
        teRLB = time.time()
        print(f"runtime: {(teRLB-tsRLB)}")
        
        # print("Actual Distance Read: ",dist)
        newDistValue = int(dist * 100)  # scaling up my Luna distance values from meter to cm
        print("Distance is cm ", newDistValue)
        if newDistValue < 2:
            newDistValue = 2
            return newDistValue
        else:
            return newDistValue
    except:
        ("Lidar broken")



    #except Exception as e:
        #print(e)
      #  return 1  # if Lidar is not working, sending 1-cm


def moveArm(Y_offset_value):

    new_yoffset = Y_offset_value
    
    tsLLD = time.time()
    new_LidrDist = LidarLatestDistance()
    teLLD = time.time()
    print(f"runtime: {(teLLD-tsLLD)}")
    
    print(f"new_yoffset: {type(new_yoffset)}")

    new_angle = calculateAngle(new_LidrDist, new_yoffset)
    
    new_angle = adjustAngle(new_yoffset)

    for i in range(1, 5):
	    
        setServoAngleUpDown(Y_offset_value)
        #sleep(0.01)
        #print(new_angle)
    print("Arm cycle completed as per offset_value")
    return new_angle



	

#step1 >> import this complete file in main_opencv_code >> import SvALdr as s,a,b,etc...

#step2 >> whenever you want to move arm  by passing y_offset call this function like this >> s(or whatever it has been imported as).move_Arm_by_Yoffset(Y_offset_value)

#put this at end of opencv main file
# GPIO.cleanup()
# print("Program completed")
