default: robot

clean:
	rm -f struct_shm.h
	rm -f BasicRobot
	rm -f Robot
	rm -f robot_debug.txt log.csv

quit:
	python -c "import robot; robot.init(); robot.wshm('quit',1)"

kill:
	sudo pkill Robot

struct_shm.h: create_c_header.py shared_memory_specification.yaml
	python create_c_header.py     # creates struct_shm.h

robot : Robot.cpp Robot.h BasicRobot.cpp struct_shm.h
	g++ Robot.cpp BasicRobot.cpp -o Robot -Llib -ldhd -lrt -lpthread -lusb-1.0 -Wall

run: robot
	python benchmark.py

