ENGG1000 RoboRescue Project

<<<<<<< HEAD
ssh robot@192.168.43.251
maker

cd ..; cd ..; cd boot; cd flash; cd lib; cd demo;

espeak -a 1300 -s 130 -v la --stdout "$@" | aplay
=======
ssh robot@192.168.43.49
maker

Python multithreading
import threading
thread1 = threading.Thread(target=function name)
thread1.start() Starts the function in the thread
thread1.join() Waits for the thread to finish
>>>>>>> 473ec7be588ed6571e063e12ad013bc2531314ad


#Clear the bullshit
sudo truncate -s 0 maze.py

/////////////////////////////////////////////////
USER MANUAL:
- start ev3 without sensors in
- connect via USB to avoid freezing
- align robot to maze with alignment arms
- no movement from before starting program
- when leds go red press any button

