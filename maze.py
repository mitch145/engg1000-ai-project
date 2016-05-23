

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
sonarLeft = UltrasonicSensor(INPUT_1); assert sonarLeft.connected
sonarFront = UltrasonicSensor(INPUT_4); assert sonarFront.connected
gyro = GyroSensor(); assert gyro.connected
colour = ColorSensor(); assert colour.connected
#bumper = TouchSensor(INPUT_4); assert bumper.connected

gyro.mode = 'GYRO-G&A'
colour.mode = 'RGB-RAW'
#sonarLeft.mode = 'US-SI-CM'
#sonarFront.mode = 'US-SI-CM'

# We will need to check EV3 buttons state.
btn = Button()

input = 0
wallDerivator       = 0.0
wallIntegrator      = 0.0
cyclesWithoutTurn   = 0
startHeading        = 0
wallOut             = 0
forwardOut          = 0
smoothedGyro        = gyro.value(0)
FILTER_FREQ         = 0.1
wallFollowEnable    = True
turnError           = 0
sonarDist           = 0
turnCounter         = 0

def main():
    global input
    global wallDerivator
    global wallIntegrator
    global forwardOut
    global wallOut
    WALL_P_GAIN     = 0.3
    WALL_I_GAIN     = 0.025
    WALL_D_GAIN     = 0.2
    wallDValue      = 0.0
    wallPValue      = 0.0
    wallIValue      = 0.0
    wallError       = 0
    WALL_DES_DIST   = 100.0 #mm
    forwardOut      = 80
    global turnError
    global cyclesWithoutTurn
    global wallFollowEnable
    global sonarDist
    global turnCounter

    Leds.set_color(Leds.RIGHT, Leds.GREEN)
    Leds.set_color(Leds.LEFT, Leds.GREEN)


    while not btn.any():
        sonarDist = sonarLeft.value(0)
        turnCounter = turnCounter + 1
        print "turnCounter: %d" % turnCounter
        #gyro drift correction
        #gyroDrift()
        
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
            wallError = -(sonarDist - WALL_DES_DIST)

            wallIntegrator += wallError 

            if wallIntegrator > 50:
                wallIntegrator = 50
            if wallIntegrator < -50:
                wallIntegrator = -50

            wallPValue = WALL_P_GAIN * wallError
            wallIValue = WALL_I_GAIN * wallIntegrator
            wallDValue = WALL_D_GAIN * (wallError - wallDerivator)

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

    while not btn.any():
        sleep(0.1)

    # reset gryo
    print "cal gyro"
    gyro.mode = 'GYRO-RATE'
    sleep(1)
    gyro.mode = 'GYRO-G&A'
    sleep(1)
    input = gyro.value(0)
    startHeading = gyro.value(0)
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
    global FILTER_FREQ
    global cyclesWithoutTurn

    print "gyro angle: %d, gyro rate: %d, smoothed gyro: %d" % (gyro.value(0), gyro.value(1), smoothedGyro)
    cyclesWithoutTurn += 1
    smoothedGyro = (gyro.value(0) * (1 - FILTER_FREQ)) + (smoothedGyro  *  FILTER_FREQ);
    if cyclesWithoutTurn > 30:
        print "updating heading..."
        leftMotor.stop(stop_command='brake')
        rightMotor.stop(stop_command='brake')
        sleep(0.5)
        if gyro.value(1) > 1 or gyro.value(1) < -1:
            print "gyro drift detected, resetting gyro..."
            Sound.tone([(2500, 200, 50)] * 2)
            gyro.mode = 'GYRO-RATE'
            sleep(1)
            gyro.mode = 'GYRO-G&A'
            sleep(1)
            print "gyro reset complete..."
        input = smoothedGyro
        smoothedGyro = gyro.value(0)
        cyclesWithoutTurn = 0

def mazeLoop():
    global startHeading
    global input

    tempHeading = startHeading - gyro.value(0)
    if tempHeading > 540 or tempHeading < -540:
        # turn 180 and continue search
        print "tempHeading = %d startHeading: %d gyro.value(0): %d" % (tempHeading, startHeading, gyro.value(0))
        Sound.tone([(750, 2000, 50)])
        print "MAZE LOOP DETECTED!!!!!!"
        input += 180

def detectRed():
    global input
    GRIPPER_OPEN    = -70
    GRIPPER_CLOSED  = 0
    #print "R: %d G: %d B: %d" % (colour.value(0), colour.value(1), colour.value(2))

    if colour.value(0) > 15 and colour.value(0) > (colour.value(1) + colour.value(2)):
        Sound.tone([(1500, 200, 50)] * 10)
        print "OBJECTIVE DETECTED!!!!!"
        grip(GRIPPER_CLOSED)
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

    print "sonarFront %d" % sonarFront.value(0)

    if sonarFront.value(0) < 15:
        print "front collision, turning right"
        turnCounter = 0
        wallFollowEnable = False
        backup()
        input += 90

def motion():
    global input
    global wallOut
    global forwardOut
    TURN_P_GAIN = 1.5
    TURN_MAX_DEG_PER_CYCLE = 50

    turnError = input + wallOut - gyro.value(0) + 0.0

    #print "turn error: %d" % turnError

    if turnError > TURN_MAX_DEG_PER_CYCLE:
        turnError = TURN_MAX_DEG_PER_CYCLE
    if turnError < -TURN_MAX_DEG_PER_CYCLE:
        turnError = -TURN_MAX_DEG_PER_CYCLE

    turnOut = TURN_P_GAIN * turnError
    rightOut = forwardOut + turnOut
    leftOut = forwardOut - turnOut
    #print "turn error %.2f heading %d target %d wallOut %d" % (turnError, gyro.value(0), input ,wallOut)

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

def grip(gripAmount):
    while gripper.position != gripAmount:
    	gripper.run_to_abs_pos(speed_regulation_enabled='on', speed_sp=100, position_sp = gripAmount)

print "ready to start"
start()
print "starting"
main()
print "main"
rightMotor.stop()
leftMotor.stop()
