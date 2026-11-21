
# ESP32 Line Follower Robot 🤖

## Overview

This project is an autonomous line-following robot
built using an ESP32 microcontroller.

The robot uses an infrared sensor array to detect
a path and adjusts its motor speeds to follow it.

## Components Used

- ESP32 NodeMCU
- TB6612FNG Motor Driver
- 8-Channel IR Sensor Array
- N20 DC Gear Motors
- Li-ion Batteries
- Robot Chassis

## Working Principle

The IR sensor array detects the contrast between
the track and the surrounding surface.

The ESP32 processes the sensor readings and
controls the motors through the TB6612FNG driver.

A PID-based control approach is used to improve
the robot's tracking and turning performance.

## Key Features

- Real-time line detection
- Differential motor control
- PID-based correction
- Embedded C/C++ programming

## Technologies Used

- ESP32
- Arduino IDE
- Embedded C/C++
- IR Sensors
- Motor Control

## Learning Outcomes

- Interfacing sensors with ESP32
- Understanding motor driver operation
- Implementing PID control
- Debugging hardware and software together

