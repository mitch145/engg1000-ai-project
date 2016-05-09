from time   import sleep
import sys
import os
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from ev3dev.auto import *

#Connect motors
rightMotor = LargeMotor(OUTPUT_A)
leftMotor  = LargeMotor(OUTPUT_D)

us = UltrasonicSensor(); assert us.connected
gs = GyroSensor(); assert gs.connected
cs = ColorSensor(); assert cs.connected
ts1 = TouchSensor(INPUT_4); assert ts1.connected

gs.mode = 'GYRO-G&A'
cs.mode = 'RGB-RAW'

# We will need to check EV3 buttons state.
btn = Button()

input = 0
wallDerivator = 0.0
wallIntegrator = 0.0
cyclesWithoutTurn = 0
startHeading = 0
wallOut = 0
forwardOut = 0
smoothedGyro = gs.value(0)
filterVal = 0.1

def start():
    global input
    global startHeading
    # wait for button press to start
    sleep(1)
    Leds.set_color(Leds.LEFT, Leds.RED)
    Leds.set_color(Leds.RIGHT, Leds.RED)
    #input = gs.value(0)
    while not btn.any():
        sleep(0.1)

    # reset gryo
    print "cal gyro"
    #sleep()
    gs.mode = 'GYRO-RATE'
    sleep(1)
    gs.mode = 'GYRO-G&A'
    input = gs.value(0)
    startHeading = gs.value(0)
    print "start heading: %d" % startHeading

def backup():
    leftMotor.stop(stop_command='brake')
    rightMotor.stop(stop_command='brake')
    leftMotor.run_timed(duty_cycle_sp=-70, time_sp=800)
    rightMotor.run_timed(duty_cycle_sp=-70, time_sp=800)
        
    while any(m.state for m in (leftMotor, rightMotor)):
        sleep(0.1)

    leftMotor.stop(stop_command='brake')
    rightMotor.stop(stop_command='brake')

def turn(turnAmount):
    global input
    global cyclesWithoutTurn
    target = input + turnAmount
    error = target - gs.value(0)
    cyclesWithoutTurn = 0

    if turnAmount < 0:
        print "turn left"
        while error < 0 and not btn.any():
            rightMotor.run_timed(duty_cycle_sp=0, time_sp=50)
            leftMotor.run_timed(duty_cycle_sp=80, time_sp=50)
            error = target - gs.value(0)
            sleep(0.02)
            #print error

    if turnAmount > 0:
        print "turn right"
        while error > 0 and not btn.any():
            rightMotor.run_timed(duty_cycle_sp=80, time_sp=50)
            leftMotor.run_timed(duty_cycle_sp=0, time_sp=50)
            error = target - gs.value(0)
            sleep(0.02)
#print error

    input = gs.value(0)

    print "stop turning"

    leftMotor.stop(stop_command='brake')
    rightMotor.stop(stop_command='brake')

def gyroDrift():
    global smoothedGyro
    global filterVal

    print "gyro angle: %d, gyro rate: %d" % (gs.value(0), gs.value(1))
    cyclesWithoutTurn += 1
    smoothedGyro = (gs.value(0) * (1 - filterVal)) + (smoothedGyro  *  filterVal);
    if cyclesWithoutTurn > 30:
        print "updating heading..."
        leftMotor.stop(stop_command='brake')
        rightMotor.stop(stop_command='brake')
        sleep(0.5)
        if(gs.value(1) > 0:
            print "gyro drift detected, resetting gyro..."
            gs.mode = 'GYRO-RATE'
            sleep(1)
            gs.mode = 'GYRO-G&A'
            sleep(1)
            print "gyro reset complete..."
        input = smoothedGyro
        smoothedGyro = gs.value(0)
        cyclesWithoutTurn = 0

def main():
    global input
    global wallDerivator
    global wallIntegrator
    global startHeading
    global forwardOut
    global wallOut
    wallPGain = 0.3
    wallIGain = 0.025
    wallDGain = 0.2
    wallDValue = 0.0
    wallPValue = 0.0
    wallIValue = 0.0
    wallError = 0
    desDistToWall = 100.0 #mm
    forwardOut = 80
    turnError = 0
    global cyclesWithoutTurn

    Leds.set_color(Leds.RIGHT, Leds.GREEN)
    Leds.set_color(Leds.LEFT, Leds.GREEN)

    while not btn.any():
        #gyro drift correction
        gyroDrift()

        # loop handling
        tempHeading = startHeading - gs.value(0)
        if tempHeading > 540 or tempHeading < -540:
            # turn 180 and continue search
            print "tempHeading = %d startHeading: %d gs.value(0): %d" % (tempHeading, startHeading, gs.value(0))
            Sound.tone([(750, 2000, 50)])
            print "MAZE LOOP DETECTED!!!!!!"
            turn(170)

        # colour detection
        print "R: %d G: %d B: %d" % (cs.value(0), cs.value(1), cs.value(2))

        if cs.value(0) > 15 and cs.value(0) > (cs.value(1) + cs.value(2)):#cs.value(0) == 5:
            Sound.tone([(1500, 200, 50)] * 10)
            print "OBJECTIVE DETECTED!!!!!"
            backup()
            turn(170)

        # wall following
        wallError = -(us.value() - desDistToWall)
        if wallIntegrator > 50:
            wallIntegrator = 50
        if wallIntegrator < -50:
            wallIntegrator = -50

        wallIntegrator += wallError 

        wallPValue = wallPGain * wallError
        wallIValue = wallIGain * wallIntegrator
        wallDValue = wallDGain * (wallError - wallDerivator)

        #print "P: %d I: %d D: %d" % (wallPValue, wallIValue, wallDValue)

        wallDerivator = wallError

        wallOut = wallPValue + wallDValue + wallIValue
        #print "wall error %d" % wallError
        
        # left corner handling
        if us.value() > 400:
            print "left corner detected"
            # turn off wall following
            wallOut = 0

            #leftMotor.run_timed(duty_cycle_sp = 80, time_sp = 250)
            #rightMotor.run_timed(duty_cycle_sp = 80, time_sp = 250)
            #while any(m.state for m in (leftMotor, rightMotor)):
            #    sleep(0.1)
            loopCount = 0
            while loopCount < 10 and not btn.any():
                forward()
                loopCount += 1
            

            leftMotor.stop(stop_command='brake')
            rightMotor.stop(stop_command='brake')

            #sleep(0.2)
            
            turn(-90)

            #sleep(0.2)

            #leftMotor.run_timed(duty_cycle_sp = 80, time_sp = 1300)
            #rightMotor.run_timed(duty_cycle_sp = 80, time_sp = 1300)
            #while any(m.state for m in (leftMotor, rightMotor)):
            #    sleep(0.1)

            loopCount = 0
            while loopCount < 20 and not btn.any():
                forward()
                loopCount += 1

            if us.value() > 400:
                print "double turn detected"
                #sleep(0.2)
                turn(-90)
                #sleep(0.2)
                #leftMotor.run_timed(duty_cycle_sp = 80, time_sp = 1300)
                #rightMotor.run_timed(duty_cycle_sp = 80, time_sp = 1300)
                #while any(m.state for m in (leftMotor, rightMotor)):
                #    sleep(0.1)
                loopCount = 0
                while loopCount < 20 and not btn.any():
                    forward()
                    loopCount += 1

            print "turn complete"

        # forward wall collision
        frontCollision()

        # continue onwards
        forward()

def frontCollision():
    if ts1.value():
        print "front collision, turning right"
        backup()
        turn(+77)

def forward():
    global input
    global wallOut
    global forwardOut
    turnGain = 1.5

    # update the motors
    turnError = input - gs.value(0)
    turnOut = turnGain * (turnError + wallOut)
    rightOut = forwardOut + turnOut
    leftOut = forwardOut - turnOut
    #print "turn error %d gyro val %d" % (turnError, gs.value(0))

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

print "ready to start"
start()
print "starting"
main()
print "main"
rightMotor.stop()
leftMotor.stop()

