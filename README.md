# Autonomous-Robot-Navigation-and-Control
GitHub repository - https://github.com/YaelBenShalom/Autonomous-Robot-Navigation-and-Control

## Overview
In this project, I build a wheeled robot and programmed it to navigate autonomously through a series of tasks, Using Arduino, motors, encoders and distance sensors.

### The Tasks
1. Navigating through a maze
2. Driving through changing-slope way, keeping constant roll
3. following curved wall, keeping constant distance
4. Driving on slope with changing gradient, keeping constant speed
5. Locating and reaching the finish line flag

### System Description
I used several components to control the robot motion:
1. Arduino
2. Motors & encoders:
    - PMW motor and encoder - One on each rear wheel to activate and control the motion
    - Servo motor - Located on the front of the vehicle and used to rotate the TOF sensor to desired scanning angle
3. Sensors
    - IR sensor - One on each side of the vehicle to measure side distance
    - TOF (Time of Flight) sensor - Assembled on the servo motor to scan the surrounding space
    - US (ultrasonik) sensor - Located on the top of the vehicle to measure distance from objects above the vehicle
    - IMU 9250 (9 DOF) - Measures orientation (Gyroscope) and acceleration (Accelerometer). Compass is not in use
4. Led matrix - Indicates mission number and mission success/fail

The system components:
![system_explanation](https://github.com/YaelBenShalom/Autonomous-Robot-Navigation-and-Control/blob/main/images/project4_system_explanation.png)

Arduino pin usage:
![system_explanation](https://github.com/YaelBenShalom/Autonomous-Robot-Navigation-and-Control/blob/main/images/project4_pin_use.png)

The autonomous vehicle:
![system_explanation](https://github.com/YaelBenShalom/Autonomous-Robot-Navigation-and-Control/blob/main/images/vehicle.png)