#cited: https://github.com/makerportal/tfluna-python. I used this as reference to code the Lidar.
#author: Milind Devnani
from time import sleep
import RPi.GPIO as GPIO
import math
import serial
import time
import pigpio
import numpy as np
#from gpiozero import AngularServo

# Initializing global variables
distance = 0
oldDuty = 7.333333
num_steps = 3
#/
sticker_not_in_centre = True

last_servo_postiion = 45
# Function to move the servo motor up and down based on the given angle
def setServoAngleUpDown(angle):
    global oldDuty
    # Setting up the GPIO pins for the servo motor
    #print("Gpio setup")
    GPIO.setup(servo, GPIO.OUT)
    
    #print("setup gpio")
    pwm = GPIO.PWM(servo, 50)# PWM frequency of 50 Hz
    
    #print("pwm start")
    pwm.start(8)# Starting the PWM signal with duty cycle of 8%
    # Calculating the duty cycle based on the angle of the servo motor
    dutyCycle = angle / 18. + 3.
    # Calculating the step size based on the number of steps
    step_size = (dutyCycle - oldDuty)/num_steps
    # Incrementing the duty cycle in steps to smoothly move the servo motor
    for i in range(num_steps):
        pwm.ChangeDutyCycle(oldDuty + step_size)
        oldDuty = oldDuty + step_size
        sleep(.02)
    
    #print(f"setting duty cycle: {dutyCycle}")
    #print("------\n")
    pwm.ChangeDutyCycle(dutyCycle)
    # Setting the final duty cycle for the servo motor
    sleep(0.02) #sleep time will not change with num_steps FOR NOW
    # Stopping the PWM signal
    pwm.ChangeDutyCycle(0)
    
    #Perhaps do all this in the init function then have setservoangle only be to changedutycycle?
    #constantly starting the pwm to be at 8 seems like it may cause jittering
    
#def setServoAngleUpDown(angle):
    #servo = 12



    #pwm.set_PWM_frequency(servo,50)

    #PW = angle*(2000.0/180.0) + 500.
    #pwm.set_servo_pulsewidth(servo,PW)

    #sleep(0.02)

    
    # #pigpio implementation

def setServoAngleLeftRight(angle):
    servo = 27# Setting the GPIO pin for the servo motor
    GPIO.setup(servo, GPIO.OUT)# Setting up the GPIO pin for the servo motor
    pwm = GPIO.PWM(servo, 300)# PWM frequency of 300 Hz
    pwm.start(8)# Starting the PWM signal with duty cycle of 8%
    # Calculating the duty cycle based on the angle of the servo motor
    dutyCycle = angle / 18. + 3. # Setting the duty cycle for the servo motor
    pwm.ChangeDutyCycle(dutyCycle)
    sleep(0.02)
    # Stopping the PWM signal
    pwm.ChangeDutyCycle(0)
    
#def setServoAngleUpDown(angle):
    #servo = Angular Servo(12, min_angle=10, max_angle=180)
    #servo.angle = angle
    #sleep(0.2)

# Function to initialize the necessary variables and settings
def init():
    global bytesAllowed, checkBytes, Lidar_UART, currAngle, servo, pwm

    bytesAllowed = 12# Number of bytes taken at a time for UART communication
    checkBytes = 89# Constant address for TF-Luna UART communication
    currAngle = 78# Current angle of the servo motor
    distance = 0# Initial distance measurement
    servo = 12# GPIO pin for the servo motor
    # servo = 18

    GPIO.setmode(GPIO.BOARD)# Setting the GPIO pin numbering mode
    GPIO.setwarnings(False)# Disabling warnings for the GPIO pins
    #pwm = pigpio.pi()
    #pwm.set_mode(servo,pigpio.OUTPUT)
    # Connecting to the TF-Luna LiDAR using UART communication
    print("Now connecting to LiDAR")
    # Connecting to the TF-Luna LiDAR using UART communication
    Lidar_UART = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)  # Connecting Lidar UART at baud rate 115200

    # IMPORTANT, WE HAVE TO MAKE SURE THE ARM STARTS AT 90 DEGREES AT STARTUP
    # Resetting the servo motor position to 90 degrees
    for i in range(5):
        print("resetting servo position")
        setServoAngleUpDown(78)
        sleep(0.02)

def adjustAngle(x):
    a = .00093137255
    b = .8818627451
    c = -8.911
    # calculate the new angle using the quadratic equation with coefficients a, b, and c
    newAng = (a * (x ** 2)) + (b * x) + c
    # print the quadratic equation for debugging purposes
    print(f"{a * (x ** 2)} + {b*x} + {c}")
    
    # if the new angle is less than or equal to 89, set it to 89
    if newAng <= 89:
        newAng = 89
    # if the new angle is greater than 91, set it to 91
    elif newAng >91:
        newAng = 91
    # return the adjusted angle
    return newAng

def getDistance(LidarBytes):
    print("getting dist")
    print(f"LidarBytes length: {len(LidarBytes)}")
    print(f"LidarBytes: {(LidarBytes[0])} {(LidarBytes[1])} {(LidarBytes[2])} {(LidarBytes[3])} {(LidarBytes[4])} {(LidarBytes[5])} {(LidarBytes[6])} {(LidarBytes[7])} {(LidarBytes[8])} {(LidarBytes[9])} {(LidarBytes[10])} ")

    for i in range(9):
        print(i)
        if LidarBytes[i] == checkBytes: # check if first byte matches checkBytes value
            if LidarBytes[i+1] == checkBytes: # check first two byte
                print(f"distance retrieved: {LidarBytes[i+2]}, {LidarBytes[i+3]}")
                dist = LidarBytes[i+2] # distance in next byte
                dist = dist/100.0# convert distance from centimeters to meters
                print(f"distance (cm): {dist*100}")
                return dist
    dist = -1# if no distance is found, return -1
    print(f"distance (cm): {dist}")

    return dist



# reading bytes from TF UART and then calling seprate function to calculate distance
def readLidarBytes():
    #while not False:
    tsRLB = time.time()

    while not False:
        print("count numbytes")
        numBytes = Lidar_UART.in_waiting  # count the number of bytes of the serial port
        print(f"numbytes = {numBytes}")
        if bytesAllowed < numBytes:
            # From TF_LUNA take the 12 bytes of data coming in, we only care about the distance bytes
            print("reading lidar bits")
            Lidar_UART.flushInput()
            LidarBytes = Lidar_UART.read(12)

            # clearing the UART Communication to get updated data
            print("resetting input buffer")
            Lidar_UART.reset_input_buffer()
            
            tsGD = time.time()
            dist = getDistance(LidarBytes)
            teGD = time.time()
            print(f"runtime: {(teGD-tsGD)}")
            print(dist)
            return dist

'''
def calculateAngle(zoffset, yoffset):
    global currAngle
    #10,-150 >> atan >> -86,-15513, -4938
    angle = math.atan(yoffset / zoffset) * 180 / math.pi
    currAngle = angle+90
    #currAngle += flag * angle
    #currAngle = max(0, min(180, currAngle))
    return currAngle
'''

def calculateAngle(zoffset, yoffset):
    global currAngle
    global last_servo_postiion
    flag = 1 # initialize flag variable as 1
    
    if yoffset <=60 and yoffset >= -20: # if yoffset is between -20 and 60, return the last servo position
        
        return last_servo_postiion
        
    if yoffset < 0: # if yoffset is negative, multiply it by -1 and change the flag to -1
        yoffset*=-1
        flag = -1
    angle = math.atan(yoffset/xoffset) * 180/math.pi # calculate the angle using the trigonometric function, arctan and convert it from radians to degrees
    currAngle += flag * angle # add the calculated angle to the current angle with the appropriate flag value
    currAngle = max(0, min(90, currAngle))
    currAngle = 90 #78means 90degree

    last_servo_postiion = currAngle # update the last servo position with the current angle
    return currAngle# return the current angle


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
    # Assign Y_offset_value to a new variable
    new_yoffset = Y_offset_value
    print("-----START-----")
    
    # Declare global variables
    global last_servo_postiion
    global sticker_not_in_centre
    
    # Check if sticker is already in the centre, if so return
    if new_yoffset <=60 and new_yoffset >-20:
        print("No need to move arm. Sticker is at centre ")
        return

    else:
       # Move arm up if sticker is above the centre
        if new_yoffset>60:
            last_servo_postiion = last_servo_postiion + 1
            setServoAngleUpDown(last_servo_postiion)
            #sleep(0.02)
            print("stcker is up from centre. moving arm up")
        
        # Move arm down if sticker is below the centre    
        if new_yoffset<-20:
            last_servo_postiion = last_servo_postiion - 1
            setServoAngleUpDown(last_servo_postiion)
            #sleep(0.02)
            print("stcker is down from centre. moving arm down")
        
    print("------END------\n")
    #return new_angle

#step1 >> import this complete file in main_opencv_code >> import SvALdr as s,a,b,etc...

#step2 >> whenever you want to move arm  by passing y_offset call this function like this >> s(or whatever it has been imported as).move_Arm_by_Yoffset(Y_offset_value)

#put this at end of opencv main file
# GPIO.cleanup()
# print("Program completed")
