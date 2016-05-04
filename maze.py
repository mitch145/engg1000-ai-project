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
cs = ColorSensor(); assert cs.connected
ts1 = TouchSensor(INPUT_4); assert ts1.connected

gs.mode = 'GYRO-ANG'
cs.mode = 'COL-COLOR'

# We will need to check EV3 buttons state.
btn = Button()

input = 0
wallDerivator = 0.0
wallIntegrator = 0.0
cyclesWithoutTurn = 0

def start():
    global input
    # wait for button press to start
    sleep(1)
    Leds.set_color(Leds.LEFT, Leds.RED)
    Leds.set_color(Leds.RIGHT, Leds.RED)
    #input = gs.value()
    while not btn.any():
        sleep(0.1)

    # reset gryo
    print "cal gyro"
    sleep(1)
    gs.mode = 'GYRO-RATE'
    gs.mode = 'GYRO-ANG'
    input = gs.value()

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
    error = target - gs.value()
    cyclesWithoutTurn = 0

    if turnAmount < 0:
        print "turn left"
        while error < 0 and not btn.any():
            rightMotor.run_timed(duty_cycle_sp=0, time_sp=50)
            leftMotor.run_timed(duty_cycle_sp=80, time_sp=50)
            error = target - gs.value()
            #print error

    if turnAmount > 0:
        print "turn right"
        while error > 0 and not btn.any():
            rightMotor.run_timed(duty_cycle_sp=80, time_sp=50)
            leftMotor.run_timed(duty_cycle_sp=0, time_sp=50)
            error = target - gs.value()
            #print error

    input = gs.value()

    print "stop turning"

    leftMotor.stop(stop_command='brake')
    rightMotor.stop(stop_command='brake')

def main():
    global input
    global wallDerivator
    global wallIntegrator
    turnGain = 1.5
    wallPGain = 0.3
    wallIGain = 0.025
    wallDGain = 0.2
    wallDValue = 0.0
    wallPValue = 0.0
    wallIValue = 0.0
    wallOut = 0
    wallError = 0
    desDistToWall = 100.0 #mm
    forwardOut = 80
    turnError = 0
    global cyclesWithoutTurn

    Leds.set_color(Leds.RIGHT, Leds.GREEN)
    Leds.set_color(Leds.LEFT, Leds.GREEN)

    while not btn.any():
            #reset input if the robot has followed a wall for a while without a turn
            cyclesWithoutTurn += 1
            #print "cycles without a turn: %d" % cyclesWithoutTurn

            if cyclesWithoutTurn > 30 and wallError*wallError <= 25:
                print "updating heading"
                input = gs.value()
                cyclesWithoutTurn = 0

            # colour detection
            # cs.value(0) will be 5 when red is detected
            #print "Colour: %d" % cs.value(0)

            if cs.value(0) == 5:
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
            if us.value() > 300:
                print "left corner detected"
                # turn off wall following
                wallOut = 0

                leftMotor.run_timed(duty_cycle_sp = 80, time_sp = 200)
                rightMotor.run_timed(duty_cycle_sp = 80, time_sp = 200)
 
                while any(m.state for m in (leftMotor, rightMotor)):
                    sleep(0.1)

                leftMotor.stop(stop_command='brake')
                rightMotor.stop(stop_command='brake')

                sleep(0.2)
                
                turn(-70)

                sleep(0.2)

                leftMotor.run_timed(duty_cycle_sp = 80, time_sp = 1300)
                rightMotor.run_timed(duty_cycle_sp = 80, time_sp = 1300)

                while any(m.state for m in (leftMotor, rightMotor)):
                    sleep(0.1)

                if us.value() > 300:
                    print "double turn detected"
                    sleep(0.2)
                    turn(-70)
                    sleep(0.2)
                    leftMotor.run_timed(duty_cycle_sp = 80, time_sp = 1300)
                    rightMotor.run_timed(duty_cycle_sp = 80, time_sp = 1300)
                    while any(m.state for m in (leftMotor, rightMotor)):
                        sleep(0.1)

                print "turn complete"

            # forward wall collision
            if ts1.value():
                print "front collision, turning right"
                backup()
                turn(+70)

            # update the motors
            turnError = input - gs.value()
            turnOut = turnGain * (turnError + wallOut)
            rightOut = forwardOut + turnOut
            leftOut = forwardOut - turnOut
            #print "turn error %d gyro val %d" % (turnError, gs.value())

            #print error

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

