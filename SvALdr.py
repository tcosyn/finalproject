#author: Milind Devnani
from time import sleep
import RPi.GPIO as GPIO
import math
import serial
import time

def setServoAngleUpDown(angle):
    servo = 17
    GPIO.setup(servo, GPIO.OUT)
    pwm = GPIO.PWM(servo, 50)
    pwm.start(8)
    dutyCycle = angle / 18. + 3.
    pwm.ChangeDutyCycle(dutyCycle)
    sleep(0.01)


def setServoAngleLeftRight(angle):
    servo = 27
    GPIO.setup(servo, GPIO.OUT)
    pwm = GPIO.PWM(servo, 50)
    pwm.start(8)
    dutyCycle = angle / 18. + 3.
    pwm.ChangeDutyCycle(dutyCycle)
    sleep(0.01)

def init():
    global bytesAllowed, checkBytes, Lidar_UART, currAngle

    bytesAllowed = 8
    checkBytes = 89
    currAngle = 78

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    print("Now connecting to LiDAR")
    Lidar_UART = serial.Serial("/dev/ttyUSB0", 115200, timeout=0)  # Connecting Lidar UART at baud rate 115200



    # IMPORTANT, WE HAVE TO MAKE SURE THE ARM STARTS AT 90 DEGREES AT STARTUP
    for i in range(4):
        setServoAngleUpDown(78)
        sleep(0.05)



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
    if LidarBytes[0] == checkBytes:
        if LidarBytes[1] == checkBytes:  # check first two bytes
            dist = LidarBytes[2] + LidarBytes[3] * 256  # distance in next two bytes
            dist = dist / 100.0
            return dist


# reading bytes from TF UART and then calling seprate function to calculate distance
def readLidarBytes():
    while not False:
        numBytes = Lidar_UART.in_waiting  # count the number of bytes of the serial port
        if bytesAllowed < numBytes:
            # From TF_LUNA take the 9 bytes of data coming in, we only care about the distance bytes
            LidarBytes = Lidar_UART.read(9)

            # clearing the UART Communication to get updated data
            Lidar_UART.reset_input_buffer()

            dist = getDistance(LidarBytes)
            return dist


def calculateAngle(xoffset, yoffset):
    global currAngle
    flag = 1
    if yoffset < 0:
        yoffset *= -1
        flag = -1
    angle = math.atan(yoffset / xoffset) * 180 / math.pi
    currAngle += flag * angle
    currAngle = max(0, min(180, currAngle))
    return currAngle




def LidarLatestDistance():
    if Lidar_UART.isOpen() == False:  # checking if lidar UART is open
        Lidar_UART.open()  # opening Lidar UART commnunication if not already open

    try:
        dist = readLidarBytes()  # calling function to take in Lidar data
        # print("Actual Distance Read: ",dist)
        newDistValue = int(dist * 100)  # scaling up my Luna distance values from meter to cm
        print("Distance is cm ", newDistValue)
        if newDistValue < 2:
            newDistValue = 2
        return newDistValue


    except Exception as e:
        print(e)
        return 1  # if Lidar is not working, sending 1-cm


def move_Arm_by_Yoffset(Y_offset_value):

	# new_yoffset = int(input("Type yoffset: "))
    new_yoffset = Y_offset_value
        

	# new_LidrDist = int(input("Type xoffset: "))
    new_LidrDist = LidarLatestDistance()

	#new_angle = calculateAngle(new_LidrDist, new_yoffset)
    
    new_angle = adjustAngle(new_yoffset)

    for i in range(1, 5):
        setServoAngleUpDown(new_angle)
        sleep(0.01)
        print(new_angle)
    print("Arm cycle completed as per offset_value")

init()

#step1 >> import this complete file in main_opencv_code >> import SvALdr as s,a,b,etc...

#step2 >> whenever you want to move arm  by passing y_offset call this function like this >> s(or whatever it has been imported as).move_Arm_by_Yoffset(Y_offset_value)

#put this at end of opencv main file
# GPIO.cleanup()
# print("Program completed")
