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
ts1 = TouchSensor(INPUT_4); assert ts1.connected

# We will need to check EV3 buttons state.
btn = Button()

input = 0

def start():
    global input
    # wait for button press to start
    sleep(1)
    input = gs.value()
    while not btn.any():
        sleep(0.1)

def backup():
    rightMotor.stop(stop_command='brake')
    rightMotor.stop(stop_command='brake')
    leftMotor.run_timed(duty_cycle_sp=-70, time_sp=1000)
    rightMotor.run_timed(duty_cycle_sp=-70, time_sp=1000)
        
    while any(m.state for m in (leftMotor, rightMotor)):
        sleep(0.1)

    rightMotor.stop(stop_command='brake')
    rightMotor.stop(stop_command='brake')

def turn(turnAmount):
    global input
    input = input + turnAmount


def main():
    global input
    turnGain = 1
    wallGain = 0.5
    desDistToWall = 100 #mm
    forwardOut = 80

    while not btn.any():
            # wall following
            wallError = -wallGain*(us.value() - desDistToWall)
            if wallError > 100:
                wallError = 100
            if wallError < -100:
                wallError = -100
            
 
            # forward wall collision
            if ts1.value():
              backup()
              turn(90)

            # update the motors
            error = wallError + input - gs.value()
            turnOut = turnGain * error
            rightOut = forwardOut + turnOut
            leftOut = forwardOut - turnOut

            if rightOut > 100:
              rightOut = 100
            if rightOut < 0:
              rightOut = 0

            if leftOut > 100:
              leftOut = 100
            if leftOut < 0:
              leftOut = 0

            rightMotor.run_timed(duty_cycle_sp=rightOut, time_sp=100)
            leftMotor.run_timed(duty_cycle_sp=leftOut, time_sp=100)


start()
print "start"
sleep(2)
main()
print "main"
rightMotor.stop()
leftMotor.stop()
