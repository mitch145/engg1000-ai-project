from time   import sleep
import sys
import os

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

def start():
	# wait for button press to start
	sleep(3)
	while not btn.any():
		sleep(0.1)

def main():
        input = gs.value()
        gain = 1
        forwardOut = 50

        while not btn.any():
                #print str(gs.value()) + " + " + str(us.value())
                error = input - gs.value()
                turnOut = gain * error
                rightOut = forwardOut + turnOut
                leftOut = forwardOut - turnOut

                if rightOut > 100
                  rightOut = 100
                if rightOut < 0
                  rightOut = 0

                if leftOut > 100
                  leftOut = 100
                if leftOut < 0
                  leftOut = 0

                rightMotor.run_timed(duty_cycle_sp=rightOut, time_sp=100)
                leftMotor.run_timed(duty_cycle_sp=leftOut, time_sp=100)


start()
print "start"
main()
print "main"
rightMotor.stop()
leftMotor.stop()
