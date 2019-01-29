
# use this for the Omega robot at the lab
ROBOT_TYPE=lab

# use this for the Sigma robot at the hospital
#ROBOT_TYPE=hosp

default: robot

clean:
	rm -f struct_shm.h
	rm -f BasicRobot
	rm -f Robot
	rm -f robot_debug.txt log.csv
	rm -f instruction.cpp
	rm -f include/dhdc.h
	rm -f *.pyc
	rm -f readme.html


doc: readme.md schema.png
	pandoc -o readme.html readme.md
	xdg-open readme.html &

quit:
	python -c "import robot; robot.init(); robot.wshm('quit',1)"

kill:
	sudo pkill Robot

struct_shm.h: create_c_header.py shared_memory_specification.yaml
	python create_c_header.py     # creates struct_shm.h

instruction.cpp: create_c_header.py shared_memory_specification.yaml variable_groups.yaml
		python create_c_header.py

include/dhdc.h:
	ln -sf dhdc_$(ROBOT_TYPE).h include/dhdc.h

schema.png: schema.svg
	inkscape --export-png=schema.png schema.svg


robot : Robot.cpp Robot.h BasicRobot.cpp BasicRobot.h Controllers.cpp  instruction.cpp instruction.h struct_shm.h lib include/dhdc.h
	g++ Robot.cpp BasicRobot.cpp Controllers.cpp instruction.cpp  -o Robot -Llib -ldhd$(ROBOT_TYPE) -lrt -lpthread -lusb-1.0 -lcomedi -lm -Wall

run: robot
	python example_play_traj.py
