
# Omega C++ / Python code

This code is based heavily on code developed by Mathilde Chaplain as part of her internship at the Motor Control Lab. The code is built on the Omega robot drivers which you can download from the manufacturer (`dhd-3.2.2.1466-linux-x86_64.tar.gz`).

This is a simple interface for the Omega Robot. It is implemented in Python 2 but the part which controls the robot is still in C++.

## Requirements

Python 2 with the following modules:
* `subprocess` which is used to launch the C++ part of the code.
* `numpy` which is used to create list in python
* `pygame` for playing a sound to give a report to the subject
* `mmap` for creating the shared memory and accessing it

These last can be install easily using the command `sudo apt-get install python-<module's name>`

## Usage

The robot C++ scripts (`BasicRobot.cpp` and `Robot.cpp`) are in the principal folder (`omega_cpp_py/`) and need to be compiled. For this, you need to be in this directory and write this command from the prompt:

```
make
```



You can create your own python script to control the robot using the simple functions in `robot.py`. For example :

```python
import robot
import time
for sharedmem import *

robot.launch()
robot.init()
robot.load()

robot.move_to(0.01,0.02,0,2) # move to point (x=0.01,y=0.02,z=0.) in t=2 seconds.
while (robot.move_is_done() == False):
  pass  #wait until the end of the movement
robot.hold_at() #Keep the robot at the position
time.sleep(2) # during 2 seconds

robot.unload() #Stop the C++ script
```

If you want to execute your own python scripts, you can easily do it writing the name of your python scripts instead of the old one in the `makefile` script. Compilation and executing will be automatically done for C++ scripts and Python scripts.

## Files

All the files C++ and python need to be in the same folder to be executed.

* `BasicRobot.cpp` and `Robot.cpp` -- contains all the C++ code which controls the robot at a lower level than Python code.
* `create_c_header.py` -- create the C++ header file which contains the structure of the shared memory for the C++ code
* `sharedmem.py` -- infrastructure for accessing the shared memory
* `robot.py` -- contains simple functions to control the robot




## Shared memory

In reality, the robot can only be controlled by a C++ program. But we want to run the experiment with python, that is why we need to use a shared memory. In the `shared_memory_specification.txt` document, we write the parameter that we want to add in the shared memory. `create_c_header.py` creates the C++ header file which contains the structure used by the C++ script to write into the shared memory. The C++ program create a shared memory after having computed the size of this one thanks to the `shared_memory_specification.txt`. From within Python, it is quite easy to access to the shared memory. Actually, to read a parameter, you can use this function `robot.rshm(variable's name)`, and to write in this one, you need to use the function `robot.wshm(variable'name, value)`.

IMPORTANT: In the `shared_memory_specification.txt`, you can only put long int or double because C++ allocates only trunks of 8 bytes. if you put a int which takes 4 bytes and then a double, the double's address is going to be 4 bytes higher. As it is not the case in python, this is a problem because parameters in the shared memory did not have the same address' offset.  

## Programmer's notes

Sometimes, when the program is not well-closed, the Robot C++ process can stay opened. In this case, if you execute again the program, it will not be able to open the device.

In this case you can try and run `make kill` which will send the robot the instruction to quit through the shared memory.

If for some reason the robot is no longer listening to the shared memory, you need to kill the Robot process manually from the prompt.
For that, you have to invoke the command `ps -Al`, looking for the robot process' ID and then, you need to do `sudo kill <ID>`.  
