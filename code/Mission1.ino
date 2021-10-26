void Mission_1() {
  Start_Mission((byte *)&LedMatrix_Mission_1, 0, 0, 8, 8);
  ReadSensors();
  cur_dist = IR_Left_Distance;
  last_dist = cur_dist;
  while (Mission_Number == 1) {
    // Serial.println("in while");
    Serial.print("TOF ");
    Serial.println(TOF_Distance_Front);
    Serial.print("LEFT ");
    Serial.println(TOF_Distance_Left);
    Serial.println(US_Distance_Read);
    Stop(); // Function that stops the robot
    delay(500);
    if (US_Distance_Read < 20) { // Checks ,mission end term
      // Start_Mission((byte*) & LedMatrix_Mission_2, 0, 0, 8, 8);
      // Serial.println("ceiling");
      Mission_Number += 1; // Mission counter up
    } else {               // Continue Mission
      Serial.println("start map");
      map_environment(); // Using sensor read to analize the surrounding
      Serial.print("close front ");
      Serial.println(close_front);
      Serial.print("close left ");
      Serial.println(close_left);
      Serial.print("wall end ");
      Serial.println(wall_end);
      Serial.print("wall sweetspot ");
      Serial.println(wall_follow_pos);
      Serial.print("corner ");
      Serial.println(corner);

      maze_step(); // Considering the map from last function do your step
      delay(150);
      ReadSensors();
    }
  }
  Stop();
  noInterrupts();
  Mission_Number = 2;
  interrupts();
}

//////////////////// Functions ////////////////////
void map_environment() { // Function takes the sensors values and maps the
                         // robots surrounding
  wall_end = false;
  wall_follow_pos = false;
  corner = false;
  close_front = false;
  close_left = false;
  // Checking left side and front
  if (TOF_Distance_Left < 15.0) {
    close_left = true;
  }
  if (TOF_Distance_Front < 9.0) {
    Serial.println(TOF_Distance_Front);
    close_front = true;
  }

  // Checking robot position relative to the wall
  if (close_front == true && close_left == true) {
    corner = true;
  } else if ((TOF_Distance_Left < 40 && TOF_Distance_Left > 15.0 &&
              close_front == false)) {
    wall_follow_pos = true;
  } else if (IR_Left_Distance - last_dist > 50.0) {
    // Checking if we found the pass through the wall
    wall_end = true;
  } else {
    // Otherwise return all flags to FALSE
    wall_end = false;
    wall_follow_pos = false;
  }
}

void maze_step() { // Function decides how to move the robot considering the
                   // flags raised by last function
  if (corner) {    // If in a corner turn 90 deg right
    Stop();
    last_dist = cur_dist;
    delay(100);
    Turn_Right_In_Angle(90);
    Serial.println("Turning right 90");
  } else if (close_front) { // If not in corner but close to wall infront, turn
                            // 90 deg right
    Serial.println("wall infront");
    Stop();
    if (TOF_Distance_Left > TOF_Distance_Right) {
      Turn_Left_In_Angle(90);
    } else {
      Turn_Right_In_Angle(90);
    }
    last_dist = cur_dist;
    delay(100);
    Serial.println("Turning 90");

  } else if (close_left) { // If not in corner but close to left, turn slightly
                           // right
    Stop();
    delay(100);
    Turn_Right_In_Angle(2);
    Serial.println("Slight right");

  } else if (wall_follow_pos) { // If Position OK to follow wall continue
                                // forward
    Move_Forwards(60);
    delay(1000);
    Stop();
    Serial.println("Forward wall");

  } else if (wall_end) { // If wall ends turn in big arc to the left hoping to
                         // find another wall
    Stop();
    last_dist = cur_dist;
    Move_Forwards(60);
    delay(200);
    Turn(100, 0);
    delay(1000);
    Turn_Left_In_Angle(70);
    Serial.println("big left");
  } else {
    Stop();
    Move_Forwards(60);
    delay(700);
    Stop();
    Serial.println("forward");
  }
}

void forward_wall() {

  cur_dist = IR_Left_Distance;
  err_dist = cur_dist - last_dist;
  // If error is 0 then just drive forwards
  if (err_dist == 0) {
    Move_Forwards(50);
    delay(1000);
    Stop();
  }
  // If error (+) then we need to get closer to wall
  else if (err_dist > 0) {
    Turn_Left_In_Angle(2);
    Serial.print("err>0");
    Move_Forwards(60);
    delay(1000);
    // last_dist = cur_dist;
  }
  // If error (-) then we need to get farther from wall
  else if (err_dist < 0) {
    Turn_Right_In_Angle(2);
    Serial.print("err<0");
    Move_Forwards(60);
    delay(1000);
    // last_dist = cur_dist;
  }
  last_dist = cur_dist;
}
