# Servo control with Raspberry Pi Pico using RT kernel
This serves as a simple example and a template to create real-time control using FreeRTOS on Raspberry Pi Pico. This particular package serves as a Hardware Interface to interact with ros2_control and moveit2 packages in ROS2.
## Prerequisites
- This example works with PCA9685 servo controller using the [Adafruit-Servo-Driver-Library-Pi-Pico](https://github.com/grzesiek2201/Adafruit-Servo-Driver-Library-Pi-Pico) library to interact with it, but feel free to use other hardware and libraries.
- The communication to the Pico is through a Serial connection (e.g. USB port on your PC).
- Make sure you have downloaded the Pico SDK and exported its path as the PICO_SDK_PATH environmental variable.
- Download the FreeRTOS repo into this repo's main directory.
## Installation
Create a build directory and build the project:
```
mkdir build
cd build
cmake ..
make
```
After building, upload the <i>serial_servo.uf2</i> file to the Pico.

## Interaction
Receive the servo position:
```
send: e <servo_number>\n
```
i.e.
```
e 0\n
```
received:
```
0 0.00000
```
meaning the first servo is at position 0.

Command the servo position:
```
send: p <servo_number> <angle>
```
i.e.
```
p 2 1.57
```
commands the servo at pin 2 to move to 90 degrees.

## Parameters
Currently only one parameters has been specified for quick modifying:
```
RECEIVE_DELAY
```
defined at the top of the main.cpp file, which defaults to 3ms. If you wish to increase the delay, which causes slower interaction with the Pico, increase this value. From my testing, 3ms works well enough with 100Hz refresh rate on the ros2_control node.

## Issues and limitations
There are no known issues or limitations. In case you find any, please create a thread in this repo.

