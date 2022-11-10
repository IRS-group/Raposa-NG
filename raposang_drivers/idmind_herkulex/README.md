# IDMindHerkuleX

This package contains ROS (Kinetic) drivers for a set of HerkuleX servos.

It contains a ros launch file to run the node that will communicate with the herkulex, as well as an optional test launch file, which can be used to test the system.

The main launch file depends on the library pySerial, and the teleop launchfile, configure to run with a Logitech F710, depends on the ros package joy_node.

## Getting Started

Copy the package to an existing catkin workspace and run:
```
catkin_make
```

After sourcing the environment, edit the port name in the configuration file:
```
roscd idmind_herkulex/cfg
nano idmind_herkulex.yaml
```

Replace /dev/ttyUSB0 by your device name.

Aditionally, you may configure the servo ID's, if you wish to use more or less, and set their default position.

Launch the program:
```
roslaunch idmind_herkulex herkulex.launch
```

You can inspect the system status via ROS Topics:
```
rostopic echo /idmind_herkulex/motor_status
```

By default, the torque of each motor is enabled, and each led will blink green 3 times.

Use the ros api to toggle torque, send angles and change led colors:
```
idmind_herkulex/set_torque
idmind_herkulex/send_motor_command
idmind_herkulex/set_led
```

The parameter motor expects the id of the motor, as defined in the configuration file. By default, varies between 0 and 3.

The parameter color is a string. Possible values: "black", "green", "red", "blue".

The parameter playtime defines how fast the motor should move, in centiseconds, and must be between 100 and 250.

The parameter angle defines the goal of the motor, in degrees, and must be between -150 and 150.

## Running the test program

After installing all dependencies, connect a Logitech F710 controller to the PC and run the teleop program:
```
roslaunch idmind_herkulex teleop.launch
```

Use the left knob to control the servos 0 and 1.

Use the right knob to control the servos 2 and 3.

## Project Overview

Under the idmind_herkulex folder you will find all configuration, launch and source files.

The source files include node.py, which defines the ROS API, herkulex.py, a third party library which defines the functions used to communicate with the herkulex controllers, and teleop.py, a test program.

The node itself, publishes board status at a rate of 20hz.