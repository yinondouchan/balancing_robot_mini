move_motor: move_motor.cpp move_motor.h hall_effect_sensor
	g++ hall_effect_sensor.o move_motor.cpp -o move_motor -I. -lwiringPi
	
hall_effect_sensor: hall_effect_sensor.cpp hall_effect_sensor.h
	g++ -c hall_effect_sensor.cpp -I.
	
clean:
	rm -rf *.o
