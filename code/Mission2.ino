#include <LEDMatrixDriver.hpp>
#include <SPI.h>
#include <Servo.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <math.h>

void Mission_2() {
  /// Code for mission 2

  Start_Mission((byte *)&LedMatrix_Mission_2, 0, 0, 8, 8); // Turn on Led matrix
  Serial.println("Start Mission 2");
  Move_Forwards(65); // start driving
  delay(1000);
  Turn(40, 100);
  delay(800);
  Turn(100, 40);
  delay(900);
  MPU9250Calculate(deltat);
  delay(70);
  float Roll0 = MPU9250_ReturnRoll(); // Reference values
  float Pitch0 = MPU9250_ReturnPitch();
  Servo_Motor.write(90); // move Servo to front
  float Front_Dist =
      TOF_Sensor.readRangeContinuousMillimeters(); // read from the TOF sensor
  while ((Front_Dist > 200) &&
         (Mission_Number == 2)) { // while mission 2 and dist from wall > 20 cm
    Move_Forwards(65);            // start driving
    noInterrupts();
    Motor_Flag = Motor_Flag_Counter;
    Motor_Flag_Counter = false;
    interrupts();
    if (Motor_Flag) {
      MPU9250Calculate(deltat);
      while (((Pitch0 - MPU9250_ReturnPitch()) > 7) && (Mission_Number == 2)) {
        Turn(40, 100);
        noInterrupts();
        Motor_Flag = Motor_Flag_Counter;
        Motor_Flag_Counter = false;
        interrupts();
        if (Motor_Flag) {
          MPU9250Calculate(deltat);
        }
        Front_Dist = TOF_Sensor.readRangeContinuousMillimeters();
      }
      Roll = MPU9250_ReturnRoll(); // read Roll
      Front_Dist =
          TOF_Sensor.readRangeContinuousMillimeters(); // read distance from
                                                       // front wall
      while (((Roll - Roll0) > 5) && (Mission_Number == 2)) { // If tilting
                                                              // right
        Turn(20, 100);                                        // Turn right
        noInterrupts();
        Motor_Flag = Motor_Flag_Counter;
        Motor_Flag_Counter = false;
        interrupts();
        if (Motor_Flag) {
          MPU9250Calculate(deltat);
          Roll = MPU9250_ReturnRoll(); // update Roll
        }
        Front_Dist =
            TOF_Sensor.readRangeContinuousMillimeters(); // update Front_Dist
      }
      while (((Roll - Roll0) < -5) && (Mission_Number == 2)) { // If tilting
                                                               // left
        Turn(100, 20);                                         // Turn left
        noInterrupts();
        Motor_Flag = Motor_Flag_Counter;
        Motor_Flag_Counter = false;
        interrupts();
        if (Motor_Flag) {
          MPU9250Calculate(deltat);
          Roll = MPU9250_ReturnRoll(); // update Roll
        }
        Front_Dist =
            TOF_Sensor.readRangeContinuousMillimeters(); // update Front_Dist
      }
    }
  }
  Stop();
  delay(3000);
  if (Mission_Number == 2) {
    Turn_Right_In_Angle(70); // Turn right before next mission
  }
  noInterrupts();
  Mission_Number = 3; // Next Mission
  interrupts();
}
