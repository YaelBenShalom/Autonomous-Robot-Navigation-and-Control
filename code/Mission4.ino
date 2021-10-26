void Mission_4() {
  float distance_left;  // Value is distance to the left, read from left sensor
                        // [cm]
  float distance_front; // Value is distance infront, read from forward sensor
                        // [cm]
  float Speed = 60;

  Start_Mission((byte *)&LedMatrix_Mission_4, 0, 0, 8,
                8); // Function of the led show for mission end
  Serial.println("Start Mission 4");
  MPU9250Calculate(deltat);
  float AccX0 = MPU9250_ReturnAccX();
  float Yaw0 = MPU9250_ReturnYaw();
  float Pitch0 = MPU9250_ReturnPitch();
  float Pitch = MPU9250_ReturnPitch() - Pitch0;
  Move_Forwards(Speed); // drive forwards with 60% speed
  delay(200);
  Servo_Motor.write(80);
  distance_front =
      TOF_Sensor
          .readRangeContinuousMillimeters(); // is that the correct function?
  while ((distance_front >= 100) && (Mission_Number == 4)) {
    noInterrupts();
    Motor_Flag = Motor_Flag_Counter;
    Motor_Flag_Counter = false;
    interrupts();
    if (Motor_Flag) {
      distance_front =
          TOF_Sensor.readRangeContinuousMillimeters(); // is that the correct
                                                       // function?
      MPU9250Calculate(deltat);
      Pitch = MPU9250_ReturnPitch();
    }
    Serial.println(Pitch);
    float Acc = MPU9250_ReturnAccX(); // return the acceleration
    Serial.print("Acc is \t");
    Serial.println(Acc);
    AccX =
        Acc * cos(Pitch * PI / 180); // calculate the acceleration on the x axis
    Serial.print("AccX is \t");
    Serial.println(AccX);
    while ((AccX < 0) &&
           (Mission_Number ==
            4)) { // while slowing down, increase the power of the wheels
      Speed += 1;
      Serial.println(Speed);
      Serial.println("Speed up dude");
      Move_Forwards(Speed);
      AccX = MPU9250_ReturnAccX(); // keep measuring the acceleration
    }
    while ((AccX > 0) &&
           (Mission_Number ==
            4)) { // while speeding up, decrease the power of the wheels
      Speed -= 1;
      Serial.println(Speed);
      Serial.println("Slow down");
      Move_Forwards(Speed);
      AccX = MPU9250_ReturnAccX(); // keep measuring the acceleration
    }
    Move_Forwards(Speed); // apply new power of the wheels to maintain constant
                          // velocity on x axis
    delay(100);
  }
  Stop();
  delay(500);
  if (Mission_Number == 4) {
    Turn_Right_In_Angle(120);
  }
  noInterrupts();
  Mission_Number = 5; // Next Mission
  interrupts();
}
