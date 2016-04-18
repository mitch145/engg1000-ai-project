# -----------------------------------------------------------------------------
# Copyright (c) 2015 Denis Demidov <dennis.demidov@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# -----------------------------------------------------------------------------

# In this demo an Explor3r robot with touch sensor attachement drives
# autonomously. It drives forward until an obstacle is bumped (determined by
# the touch sensor), then turns in a random direction and continues. The robot
# slows down when it senses obstacle ahead (with the infrared sensor).
#
# The program may be stopped by pressing any button on the brick.
#
# This demonstrates usage of motors, sound, sensors, buttons, and leds.

from time   import sleep
import sys
import os
import subprocess
import threading

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from ev3dev.auto import *

#Connect motors
rightMotor = LargeMotor(OUTPUT_A)
leftMotor  = LargeMotor(OUTPUT_D)

us = UltrasonicSensor(); assert us.connected
gs = GyroSensor(); assert gs.connected

gs.mode = 'GYRO-ANG'

# We will need to check EV3 buttons state.
btn = Button()

# defining forwards and backwards
forwards = 1
backwards = -1

def start():
	# wait for button press to start
	while not btn.any():
		sleep(0.1)

def turn(dir):

	# We want to turn the robot wheels in opposite directions
	rightMotor.run_timed(duty_cycle_sp=dir*backwards*100, time_sp=100)
	leftMotor.run_timed(duty_cycle_sp=dir*forwards*100, time_sp=100)

def speak(string):
	os.system("espeak -a 1300 -s 130 -v la --stdout '"+string+"' | aplay")

def main():
	while not btn.any():
	
		direction = gs.value()
		distance = us.value()
		print "distance: " + str(distance)
		while distance >= 300:
          		distance = us.value()
			sleep(0.1)
			turn(-1)
			#os.system("turn")

		#t3=threading.Thread(target=speak, args=('target acquired',))
		t3.start()
		rightMotor.stop(stop_command='brake')
		leftMotor.stop(stop_command='brake')
		print "straight"
		
		while not btn.any():
		
			distance = us.value()
                        print "distance: "+str(distance)
			dc = 100
			#os.system("straight")
			sleep(0.1)
			rightMotor.run_timed(duty_cycle_sp=forwards*100, time_sp=1500)
        		leftMotor.run_timed(duty_cycle_sp=forwards*100, time_sp=1500)

		print rightMotor.position, leftMotor.position

t1=threading.Thread(target=speak, args=('Kill All Humans',))
t2=threading.Thread(target=main)
t3=threading.Thread(target=speak, args=('Target acquired',))
t4=threading.Thread(target=speak, args=('i will be back',))

start()

t1.start()

sleep(3)

t2.start()

t1.join()
t2.join()

rightMotor.stop()
leftMotor.stop()

t4.start()

t4.join()