from time   import sleep
import sys
import os
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from ev3dev.auto import *

#Connect motors
rightMotor = LargeMotor(OUTPUT_A)
leftMotor  = LargeMotor(OUTPUT_D)

#connect gripper
gripper = LargeMotor(OUTPUT_B)

# sensors
us = UltrasonicSensor(); assert us.connected
gs = GyroSensor(); assert gs.connected
cs = ColorSensor(); assert cs.connected
ts1 = TouchSensor(INPUT_4); assert ts1.connected

gs.mode = 'GYRO-G&A'
cs.mode = 'RGB-RAW'
#us.mode = 'US-SI-CM'

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
wallFollowEnable = True
turnError = 0
sonarDist = 0
turnCounter = 0

def main():
    global input
    global wallDerivator
    global wallIntegrator
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
    global turnError
    global cyclesWithoutTurn
    global wallFollowEnable
    global sonarDist
    global turnCounter

    Leds.set_color(Leds.RIGHT, Leds.GREEN)
    Leds.set_color(Leds.LEFT, Leds.GREEN)


    while not btn.any():
        sonarDist = us.value(0)
        turnCounter = turnCounter + 1
        print "turnCounter: %d" % turnCounter
        #gyro drift correction
        #gyroDrift()

        #grip()
        

        # loop handling
        mazeLoop()

        # colour detection
        detectRed()

        # wall following
        if wallFollowEnable == False and turnCounter > 25:
            wallFollowEnable = True
            print "wall following enabled"
            turnCounter = 0

        if wallFollowEnable == True:
            wallError = -(sonarDist - desDistToWall)

            wallIntegrator += wallError 

            if wallIntegrator > 50:
                wallIntegrator = 50
            if wallIntegrator < -50:
                wallIntegrator = -50

            wallPValue = wallPGain * wallError
            wallIValue = wallIGain * wallIntegrator
            wallDValue = wallDGain * (wallError - wallDerivator)

            #print "P: %d I: %d D: %d" % (wallPValue, wallIValue, wallDValue)

            wallDerivator = wallError

            wallOut = wallPValue + wallDValue + wallIValue
            #print "wall error %d" % wallError

        if wallFollowEnable == False:
            wallOut = 0
        
        # left corner handling
        leftCorner()

        # forward wall collision
        frontCollision()

        # continue onwards
        motion()


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
    sleep(1)
    input = gs.value(0)
    startHeading = gs.value(0)
    print "start heading: %d" % startHeading
    gripper.position = 0

def backup():
    leftMotor.stop(stop_command='brake')
    rightMotor.stop(stop_command='brake')
    leftMotor.run_timed(duty_cycle_sp=-70, time_sp=800)
    rightMotor.run_timed(duty_cycle_sp=-70, time_sp=800)
        
    while any(m.state for m in (leftMotor, rightMotor)):
        sleep(0.1)

    leftMotor.stop(stop_command='brake')
    rightMotor.stop(stop_command='brake')

def gyroDrift():
    global smoothedGyro
    global filterVal
    global cyclesWithoutTurn

    print "gyro angle: %d, gyro rate: %d, smoothed gyro: %d" % (gs.value(0), gs.value(1), smoothedGyro)
    cyclesWithoutTurn += 1
    smoothedGyro = (gs.value(0) * (1 - filterVal)) + (smoothedGyro  *  filterVal);
    if cyclesWithoutTurn > 30:
        print "updating heading..."
        leftMotor.stop(stop_command='brake')
        rightMotor.stop(stop_command='brake')
        sleep(0.5)
        if gs.value(1) > 1 or gs.value(1) < -1:
            print "gyro drift detected, resetting gyro..."
            Sound.tone([(2500, 200, 50)] * 2)
            gs.mode = 'GYRO-RATE'
            sleep(1)
            gs.mode = 'GYRO-G&A'
            sleep(1)
            print "gyro reset complete..."
        input = smoothedGyro
        smoothedGyro = gs.value(0)
        cyclesWithoutTurn = 0

def mazeLoop():
    global startHeading
    global input

    tempHeading = startHeading - gs.value(0)
    if tempHeading > 540 or tempHeading < -540:
        # turn 180 and continue search
        print "tempHeading = %d startHeading: %d gs.value(0): %d" % (tempHeading, startHeading, gs.value(0))
        Sound.tone([(750, 2000, 50)])
        print "MAZE LOOP DETECTED!!!!!!"
        input += 180

def detectRed():
    global input
    #print "R: %d G: %d B: %d" % (cs.value(0), cs.value(1), cs.value(2))

    if cs.value(0) > 15 and cs.value(0) > (cs.value(1) + cs.value(2)):
        Sound.tone([(1500, 200, 50)] * 10)
        print "OBJECTIVE DETECTED!!!!!"
        grip()
        backup()
        input += 180

def leftCorner():
    global wallOut
    global wallFollowEnable
    global input
    global sonarDist
    global turnCounter

    if sonarDist > 400:
        wallout = 0
        if wallFollowEnable == True:
            if turnCounter < 10:
                turnCounter = -20
            else:
                turnCounter = 0 # the counter is advanced further for  left turn
            print "left corner detected"
            input -= 90
            wallFollowEnable = False
            print "wall following disabled"
            print "target heading: %d" % input

        

def frontCollision():
    global input
    global turnCounter
    global wallFollowEnable

    if ts1.value():
        print "front collision, turning right"
        turnCounter = 0
        wallFollowEnable = False
        backup()
        input += 90

def motion():
    global input
    global wallOut
    global forwardOut
    turnGain = 1.5
    turnSpeed = 50

    # update the motors
    turnError = input + wallOut - gs.value(0) + 0.0

    print "turn error: %d" % turnError

    if turnError > turnSpeed:
        turnError = turnSpeed
    if turnError < -turnSpeed:
        turnError = -turnSpeed

    turnOut = turnGain * turnError
    rightOut = forwardOut + turnOut
    leftOut = forwardOut - turnOut
    print "turn error %.2f heading %d target %d wallOut %d" % (turnError, gs.value(0), input ,wallOut)

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

def grip():
    while gripper.position > -70:
    	gripper.run_to_abs_pos(speed_regulation_enabled='on', speed_sp=100, position_sp=-70)

def release():
    while gripper.position < 0:
        gripper.run_to_abs_pos(speed_regulation_enabled='on', speed_sp=100, position=0)

print "ready to start"
start()
print "starting"
main()
print "main"
rightMotor.stop()
leftMotor.stop()
