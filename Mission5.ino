void Mission_5() {
  MPU9250Calculate(deltat);
  const int nominal_speed = 60;
  const float Pitch0 = MPU9250_ReturnPitch();
  const int Dist_DownConst = 475;
  const int Dist_WallConst = 500;
  const int approachConst = 250;
  const int final_distConst = 100;
  float Pitch;
  boolean Found1 = false;
  boolean Found2 = false;
  int Dist_Down = Dist_DownConst; // distance down the ramp [mm] || NEED TO MEASURE MAZE TO CHANGE VALUE!
  int Dist_Wall = Dist_WallConst; // distance from ramp end to the wall [mm] || NEED TO MEASURE MAZE TO CHANGE VALUE!
  int approach = approachConst; // distance to coarse approach the pole [mm] || NEED TO MEASUER MAZE TO CHANGE VALUE!
  int final_dist = final_distConst; // [mm]

  Start_Mission((byte*) & LedMatrix_Mission_5, 0, 0, 8, 8);
  Serial.println("Start Mission 5");

  while (Mission_Number == 5 ) {
    delay(1000);
    Pitch = Pitch0;
    Move_Forwards(nominal_speed); //Go down the ramp
    while (Dist_Down > 0) { // || (Pitch > Pitch0 + 3)) { //while still on ramp & no Pitch
      noInterrupts();
      Motor_Flag = Motor_Flag_Counter;
      Motor_Flag_Counter = false;
      interrupts();
      if (Motor_Flag) {
        MPU9250Calculate(deltat);
        Pitch = MPU9250_ReturnPitch();
      }
      Dist_Down = Dist_DownConst - dist_cntr * ENCODER_TO_ANGLE * WHEEL_RADIUS; //ANGLE IN RAD*RADIUS = DISTANCE CROSSED
      Serial.print("Dist down: \t");
      Serial.println(Dist_Down);
      Serial.print("Pitch: \t");
      Serial.println(Pitch);
    }
    Stop();
    delay(1000);
    //************************************
    /*
      Turn_Right(90);//_In_Angle(90); // Rotate the robot in-place in 90 degrees to the Right (pole side)
      delay(5000);
      noInterrupts();
      dist_cntr = 0;
      interrupts();
      //************************************
      Serial.println("approaching");
      Move_Forwards(nominal_speed); //Go towards the pole
      while (approach > 0) { // Approach the pole
      approach = approachConst - dist_cntr * ENCODER_TO_ANGLE * WHEEL_RADIUS; //ANGLE IN RAD*RADIUS = DISTANCE CROSSED
      Serial.print("Approach Dist: \t");
      Serial.println(approach);
      }
      Stop();
      delay(500);
      //***********************************
      Serial.println("Turning Left");
      Turn_Left_In_Angle(90); // Rotate the robot in-place in 90 degrees to the Left (parallel to pole)
      Servo_Motor.write(0); //"Right"
      delay(5000);
    */
    //************************************
    noInterrupts();
    dist_cntr = 0;
    interrupts();
    Servo_Motor.write(20); //"Right"
    delay(1000);
    Move_Forwards(nominal_speed); //Go towards the wall
    while ((Dist_Wall > 0) && (!Found1)) {
      Dist_Wall = Dist_WallConst - dist_cntr * ENCODER_TO_ANGLE * WHEEL_RADIUS; //ANGLE IN RAD*RADIUS = DISTANCE CROSSED
      Serial.println(TOF_Sensor.readRangeContinuousMillimeters());
      if (TOF_Sensor.readRangeContinuousMillimeters() <= 750) { // [mm] assuming pole distance 250mm || NEED TO MEASUER MAZE TO CHANGE VALUE!
        Found1 = true;
      }
    }
    delay(300);
    Stop();
    noInterrupts();
    dist_cntr = 0;
    interrupts();
    delay(500);
    //************************************
    Turn_Right_In_Angle(90); // Rotate the robot in-place in 90 degrees to the right
    Servo_Motor.write(80); //"Forward"
    delay(1000);
    Move_Forwards(nominal_speed); //Go towards the pole
    //************************************
    noInterrupts();
    dist_cntr = 0;
    interrupts();
    delay(500);
    while (approach > 0) {
      approach = approachConst - dist_cntr * ENCODER_TO_ANGLE * WHEEL_RADIUS; //ANGLE IN RAD*RADIUS = DISTANCE CROSSED
      Serial.println(approach);
    }
    Stop();
    noInterrupts();
    dist_cntr = 0;
    interrupts();
    int dist = 0;
    delay(500);
    //************************************
    Servo_Motor.write(45);
    while (!Found2) {
      for (int i = 45; i < 135; i++) { // i is the servo angle, for loop scans with ToF
        Servo_Motor.write(i);
        Serial.println(TOF_Sensor.readRangeContinuousMillimeters());
        if (TOF_Sensor.readRangeContinuousMillimeters() <= 500) {
          Found2 = true;
          if ( i > 80) {
            int iangle = 90 - map(i, 80, 160, 0, 90);
            Turn_Left_In_Angle(iangle);
            delay(1000);
          }
          else {
            int iangle = 90 - map(i, 0, 80, 0, 90);
            Turn_Right_In_Angle(iangle);
            delay(1000);
          }
          break;
        }
      }
      // in case didn't find pole, need to move left/right and search??
    }
    noInterrupts();
    dist_cntr = 0;
    interrupts();
    delay(500);
    //************************************
    Found2 = false;
    Servo_Motor.write(45);
    while (!Found2) {
      for (int i = 45; i < 135; i++) { // i is the servo angle, for loop scans with ToF
        Servo_Motor.write(i);
        Serial.println(TOF_Sensor.readRangeContinuousMillimeters());
        if (TOF_Sensor.readRangeContinuousMillimeters() <= 400) {
          Found2 = true;
          dist = TOF_Sensor.readRangeContinuousMillimeters();
          /*
            if ( i > 80) {
            int iangle = 90 - map(i, 80, 160, 0, 90);
            Turn_Left_In_Angle(iangle);
            delay(5000);
            }
            else {
            int iangle = 90 - map(i, 0, 80, 0, 90);
            Turn_Right(iangle);//_In_Angle(iangle);
            delay(5000);
            }
          */
          break;
        }
      }
      // in case didn't find pole, need to move left/right and search??
    }
    //************************************
    Move_Forwards(nominal_speed); //Go towards the pole
    while ((TOF_Sensor.readRangeContinuousMillimeters() <= dist + 50) && (TOF_Sensor.readRangeContinuousMillimeters() >= 100)) {
      Serial.print(TOF_Sensor.readRangeContinuousMillimeters());
      Serial.println("\t CHARGE!!");
      //final_dist = 100 - dist_cntr * ENCODER_TO_ANGLE * WHEEL_RADIUS; //ANGLE IN RAD*RADIUS = DISTANCE CROSSED
    }
    Stop();
    Serial.println("done :)");
    Start_Mission((byte*) & LedMatrix_Mission_Smile, 0, 0, 8, 8);
    delay(10000);
  }
}

/*

  ENVELOPE NEEDED
  1. interrupt that counts encoder click
  2. interrupt that measures tof/ir distances
   XX a. ToF_dist = distance measured by the ToF sensor
  XX 3.Rotate_ToF(angle) - global function that rotates the servo with ToF to the absolute degree in the parenthesis

  UNRESOLVED ISSUES
  1. need to zeorize dist_cntr in order to calculate the change in distance
*/
