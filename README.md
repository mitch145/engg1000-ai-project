ENGG1000 RoboRescue Project

ssh robot@192.168.43.251
maker

cd ..; cd ..; cd boot; cd flash; cd lib; cd demo;

espeak -a 1300 -s 130 -v la --stdout "$@" | aplay
