
# Omega C++ / Python code

This code is based heavily on code developed by Mathilde Chaplain as part of her internship at the Motor Control Lab. The code is built on the Omega robot drivers which you can download from the manufacturer (`dhd-3.2.2.1466-linux-x86_64.tar.gz` [which you can download here]((http://www.forcedimension.com/download/sdk)).

This is a simple interface for the Omega Robot. It is implemented in Python 2 but the part which controls the robot is still in C++.

## Requirements

Python 2 with the following modules:
* `subprocess` which is used to launch the C++ part of the code.
* `numpy` for numeric computation.
* `mmap` for creating the shared memory and accessing it.

These last can be install easily using the command `sudo apt-get install python-<module's name>` or using `pip`.

We have been running this on a 64-bit version of Xunbuntu 16.04.02, running kernel `4.4.0-87-generic`.

To run the robot you also need to install libusb1, under Ubuntu this can be achieved through `sudo apt install libusb-1.0-0-de`.



## Usage

Turn on the Omega robot using the switch in the back. One of the two front LEDs will blink. Press the `RESET` button on the robot controller box. Now you should be good to go.

The robot C++ scripts (`BasicRobot.cpp` and `Robot.cpp`) are in the principal folder (`omega_cpp_py/`) and need to be compiled. For this, you need to be in this directory and write this command from the prompt:

```
make
```



You can create your own python script to control the robot using the simple functions in `robot.py`. For example :

```python
import robot
import time

robot.launch() # launches the robot C++ script
robot.init()   # initialises shared memory

robot.move_to(0.01,0.02,0,2) # move to point (x=0.01,y=0.02,z=0.) in t=2 seconds.
while (robot.move_is_done() == False):
  pass  #wait until the end of the movement
robot.hold_at() #Keep the robot at the position
time.sleep(2) # during 2 seconds

robot.unload() #Stop the C++ script
```



## Files

All the files C++ and python need to be in the same folder to be executed.

Core files:

* `BasicRobot.cpp` and `Robot.cpp` -- contains all the C++ code which controls the robot at a lower level than Python code.
* `shared_memory_specification.txt` -- specifies the structure of the shared memory. This is read both by Python and by C++. You can freely add your own variables but be careful when you remove existing variables since some of the code may depend on them.
* `create_c_header.py` -- create the C++ header file which contains the structure of the shared memory for the C++ code
* `sharedmem.py` -- infrastructure for accessing the shared memory
* `robot.py` -- contains simple functions to control the robot

* `lib/libdhd.a` -- contains the proprietary robot code (pre-compiled)
* `include/libdhd.h` -- idem, header file

Further, various examples are provided in the form of scripts we call `example_<something>.py`. Take a look at these for inspiration. You can always remove these files since they are not part of the core.




## Under the hood

To control the robot main loop iteration time (the number of cycles per second) you can adjust `MAIN_LOOP_TIME_S = .0025;` which is defined in `Robot.h`.


### Recording a trajectory

Quite simple, call `robot.start_capture()` and some time later, `robot.stop_capture()`, the latter function will return a list of captured positions, coded as a tuple `(x,y,z)`.


### Writing your own controller

Quite easy: you need to write a custom controller function that sets the forces `fx, fy, fz` and call this function from the `Robot::mainLoop()` function defined in `Robot.cpp`. To activate the controller from within Python, call `robot.controller(N)` where `N` is the number you assigned to your controller.



## Shared memory

In reality, the robot can only be controlled by a C++ program. But we want to run the experiment with python, that is why we need to use a shared memory. In the `shared_memory_specification.txt` document, we write the parameter that we want to add in the shared memory. `create_c_header.py` creates the C++ header file which contains the structure used by the C++ script to write into the shared memory. The C++ program create a shared memory after having computed the size of this one thanks to the `shared_memory_specification.txt`. From within Python, it is quite easy to access to the shared memory. Actually, to read a parameter, you can use this function `robot.rshm(variable's name)`, and to write in this one, you need to use the function `robot.wshm(variable'name, value)`.

Each line in the `shared_memory_specification.txt` creates one variable in the shared memory. Each variable has a name and a type. The types conform to the `struct` specification in Python ([see here for a specification](https://docs.python.org/2/library/struct.html#format-characters)).

For example, the line
```
x d
```

creates a variable named `x` which has a double floating point precision value.

Lists of values can be created using:
```
record_x 4000d
```

which creates a list of 4000 doubles, for example to hold a trajectory in shared memory.

IMPORTANT: In the `shared_memory_specification.txt`, you can only put long int or double because C++ allocates only trunks of 8 bytes. if you put a int which takes 4 bytes and then a double, the double's address is going to be 4 bytes higher. As it is not the case in python, this is a problem because parameters in the shared memory did not have the same address' offset.  

For more information about memory alignment, see e.g. [some Stack overflow discussion](https://stackoverflow.com/questions/5435841/memory-alignment-in-c-structs) and many other pages online.




## Issues / TODO
- [ ] **Logging to a specified file** Currently we log to `log.csv` always. However, I am starting to think we should do logging only through Python pulling from shared memory, because in this way you have access to data immediately and don't need to search through the robot log after the fact.
- [ ] **Allow comments in shared memory specification**
- [ ] **Benchmark testing** of desired trajectory and actually produced trajectory.
- [ ] **Mock Inmotion** with some adjustments, we could pretend this is a planar robot and use it to test Inmotion2 code.
- [ ] **Super-user** do we have to be that to launch the robot code?
- [ ] **Instability** of the servo control - I wonder what this is due to?



## Programmer's notes

Sometimes, when the program is not well-closed, the Robot C++ process can stay opened. In this case, if you execute again the program, it will not be able to open the device.

In this case you can try and run `make quit` which will send the robot the instruction to quit through the shared memory. If this does not work, you can run `make kill` which will attempt to kill the robot process (the hard way).

If for some reason the robot is no longer listening to the shared memory, you need to kill the Robot process manually from the prompt.
For that, you have to invoke the command `ps -Al`, looking for the robot process' ID and then, you need to do `sudo kill <ID>`.  
