#author:Milind Devnani
from time import sleep
import RPi.GPIO as GPIO
import math
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

currAngle = 90

def setServoAngleUpDown(angle):
	servo = 17
	GPIO.setup(servo, GPIO.OUT)
	pwm = GPIO.PWM(servo, 50)
	pwm.start(8)
	dutyCycle = angle / 18. + 3.
	pwm.ChangeDutyCycle(dutyCycle)
	sleep(0.01)
	#pwm.stop()


def setServoAngleLeftRight(angle):
	servo = 27
	GPIO.setup(servo, GPIO.OUT)
	pwm = GPIO.PWM(servo, 50)
	pwm.start(8)
	dutyCycle = angle / 18. + 3.
	pwm.ChangeDutyCycle(dutyCycle)
	sleep(0.01)
	#pwm.stop()


#for i in range(0,90,5):
	#setServoAngle(servo,i)
	#print(i)
	
def calculateAngle(xoffset, yoffset):
	global currAngle
	flag = 1
	if yoffset < 0:
		yoffset*=-1
		flag = -1
	angle = math.atan(yoffset/xoffset) * 180/math.pi
	currAngle += flag * angle
	currAngle = max(0, min(90, currAngle))
	
#IMPORTANT, WE HAVE TO MAKE SURE THE ARM STARTS AT 90 DEGREES AT STARTUP
for i in range(4):
	setServoAngleUpDown(90)
	sleep(0.05)
	#setServoAngleLeftRight(78)
	#sleep(0.05)
	
while True:
	#presetangle = int(75)
	#for i in range(1,5):
		#setServoAngleLeftRight(presetangle)
		#sleep(0.01)
	mycommand1 = int(input("Type yoffset: "))
	mycommand2 = int(input("Type xoffset: "))
	#mycommand3 = int(input("Type left and right arm angle: "))

	#calculateAngle(mycommand1, mycommand2)
	for i in range(1,5):
		setServoAngleUpDown(mycommand1)
		sleep(0.01)
		print(currAngle)
GPIO.cleanup()
print("Program completed")
