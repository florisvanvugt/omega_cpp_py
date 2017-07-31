default: robot

clean:
	rm -f struct_shm.h
	rm -f BasicRobot
	rm -f Robot

kill:
	python -c "import robot; robot.init(); robot.wshm('quit',1)"

struct_shm.h: create_c_header.py shared_memory_specification.txt
	python create_c_header.py

robot : Robot.cpp Robot.h BasicRobot.cpp struct_shm.h
	g++ Robot.cpp BasicRobot.cpp -o Robot -Llib -ldhd -lrt -lpthread -lusb-1.0

run: robot
	python benchmark.py

