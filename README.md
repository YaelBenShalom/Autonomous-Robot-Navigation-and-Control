# Autonomous-Robot-Navigation-and-Control


## Table of Contents

- [Description](#description)
- [Overview](#overview)
    - [Tasks](#tasks)
    - [System Description](#system-description)


## Description
In this project, I build a wheeled robot and programmed it to navigate autonomously through a series of tasks, using Arduino, motors, encoders and distance sensors.<br>
Take a look at my [portfolio post](https://yaelbenshalom.github.io/mechatronics/index.html) for more information about the project.


## Overview

### Tasks
1. Navigating through a maze
2. Driving through changing-slope way, keeping constant roll
3. Following curved wall, keeping constant distance
4. Driving on slope with changing gradient, keeping constant speed
5. Locating and reaching the finish line flag

### System Description
I used several components to control the robot motion:
1. Arduino
2. Motors & encoders:
    - PMW motor and encoder - One on each rear wheel to activate and control the motion
    - Servo motor - Located on the front of the vehicle and used to rotate the TOF sensor to desired scanning angle
    - Motor driver (SparkFun Ardumoto 50829)
3. Sensors
    - IR sensor - One on each side of the vehicle to measure side distance
    - Time of Flight sensor (Adafruit VL53L0X) - Assembled on the servo motor to scan the surrounding space
    - US (ultrasonik) sensor - Located on the top of the vehicle to measure distance from objects above the vehicle
    - IMU (MPU-9250) - 9-axis (gyro, accelerometer, compass) MEMS motion tracking device
4. Led matrix - Indicates mission number and mission success/fail

The system components:
    <p align="center">
        <img align="center" src="https://github.com/YaelBenShalom/Autonomous-Robot-Navigation-and-Control/blob/main/images/project4_system_explanation.png" width="75%">
    </p>

Arduino pin usage:
    <p align="center">
        <img align="center" src="https://github.com/YaelBenShalom/Autonomous-Robot-Navigation-and-Control/blob/main/images/project4_pin_use.png" width="60%">
    </p>

The autonomous vehicle:
    <p align="center">
        <img align="center" src="https://github.com/YaelBenShalom/Autonomous-Robot-Navigation-and-Control/blob/main/images/vehicle.jpg" width="60%">
    </p>
