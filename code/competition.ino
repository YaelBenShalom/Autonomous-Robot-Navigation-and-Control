#include <LEDMatrixDriver.hpp>
#include <SPI.h>
#include <Servo.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <math.h>

#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69 // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68 // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C  // Address of magnetometer
#endif

VL53L0X TOF_Sensor;
Servo Servo_Motor;

volatile int Button = 18; // Define button pin that can get interrupts

const uint8_t PWM_Right = 3; // Angular velocity of right motor
const uint8_t PWM_Left =
    2; // Angular velocity of left motor, connected to pin 11
const uint8_t DIR_Right = 12; // Direction of right motor
const uint8_t DIR_Left = 13;  // Direction of left motor
const uint8_t Encoder = 19;   // Wheel encoder pin

const int IR_Right = A7;       // Define IR right sensor pin
const int IR_Left = A0;        // Define IR left sensor pin
const uint8_t US_Distance = 8; // Define US sensor pin

const uint8_t LedMatrix_Pin = 49; // Define LedMatrix CS pin
const int LedMatrix_Width = 7;    // Define LedMatrix width
const int LedMatrix_Hight = 7;    // Define LedMatrix hight
const int LedMatrix_Segments = 1; // Define number of matrix in use
LEDMatrixDriver lmd(LedMatrix_Segments, LedMatrix_Pin); // Define the LedMatrix

int AccX, AccY, AccZ, GyrX, GyrY, GyrZ, MagX, MagY, MagZ;
float Yaw, Pitch, Roll;

float IR_Right_Distance;  // Holds current distance of Right IR Sensor
float IR_Left_Distance;   // Holds current distance of Left IR Sensor
float US_Distance_Read;   // Holds current distance of US Sensor
float TOF_Distance_Left;  // Holds current distance of TOF Sensor
float TOF_Distance_Right; // Holds current distance of TOF Sensor
float TOF_Distance_Front; // Holds current distance of TOF Sensor

float cur_dist;
float last_dist;
float err_dist;

bool close_left;  // If robot is too close to the wall to the left
bool close_front; // If robot is too close to the wall in front
bool corner; // If robot is in a corner (wall to the left and front) value is
             // TRUE
bool wall_follow_pos; // If robot is located good to follow wall value is TRUE
bool wall_end;        // If no wall to the left and in front

///////////////////  Initialize variables  ///////////////////

const float Pi = 3.14;
int Mission_Number = 4; // Initialize mission counter

float Servo_Position =
    90; // [degree] Initialize variable to store the servo position
float Speed_Right = 0; // [%] Percentage of the velocity for the right motor,
                       // when 100 is the highest and 0 is the lowest
float Speed_Left = 0; // [%] Percentage of the velocity for the left motor, when
                      // 100 is the highest and 0 is the lowest
const float Rotation_Speed = 75.0; // [%] Percentage of the velocity, when 100
                                   // is the highest and 0 is the lowest
// Represents the Duty Cycle of velocity

float Turning_Angle = 90.0;
float Current_Angle = 0;
float Desired_Angle = 0;
float deltat = 0.02; // 50Hz sample time

const int WHEEL_RADIUS = 32;             //[mm]
const float ENCODER_TO_ANGLE = 0.2 * Pi; // NEED TO CALCULATE TO CHANGE VALUE!

int DIV;
uint16_t Timer1_Counter;          // Setup Timer1 counter
volatile uint16_t dist_cntr;      // Setup encoder counter
volatile bool Motor_Flag_Counter; // Setup counter flag variables
bool Motor_Flag;                  // Setup counter flag variables

// Visual indication for start of mission 1
byte LedMatrix_Mission_1[8] = {B00000000, B00000100, B00000100, B00000100,
                               B00000100, B00000100, B00000000, B00000000};

// Visual indication for start of mission 2
byte LedMatrix_Mission_2[8] = {B00000000, B00111100, B00000100, B00111100,
                               B00100000, B00111100, B00000000, B00000000};

// Visual indication for start of mission 3
byte LedMatrix_Mission_3[8] = {B00000000, B00111100, B00000100, B00111100,
                               B00000100, B00111100, B00000000, B00000000};

// Visual indication for start of mission 4
byte LedMatrix_Mission_4[8] = {B00000000, B00100100, B00100100, B00111100,
                               B00000100, B00000100, B00000000, B00000000};

// Visual indication for start of mission 5
byte LedMatrix_Mission_5[8] = {B00000000, B00111100, B00100000, B00111100,
                               B00000100, B00111100, B00000000, B00000000};

// Visual indication for end of mission 1
byte LedMatrix_Mission_Smile[8] = {B00000000, B01000010, B00000000, B00000000,
                                   B10000001, B01000010, B00111100, B00000000};

void Mission_1();
void Mission_2();
void Mission_3();
void Mission_4();
void Mission_5();
typedef void (*Mission_Function)();
// Mission array for main loop
Mission_Function Mission_Array[5] = {
    &Mission_1, &Mission_2, &Mission_3, &Mission_4, &Mission_5,
};

void setup() {
  DIV = 20;
  Timer1_Counter = 0;         // Initialize Timer1 counter
  Motor_Flag_Counter = false; // Initialize counter flag variables
  Motor_Flag = false;         // Initialize counter flag variables

  Timer1.initialize(1000); // 1kHz timer rollover
  Timer1.attachInterrupt(Timer1_isr);
  Timer3.initialize(40); // 40 us = 25 kHz - TODO
  Serial.begin(9600);
  Wire.begin();

  ///////////////////  Pin configuration  ///////////////////
  pinMode(Button, INPUT); // Define interrupt pin as INPUT
  lmd.setEnabled(true);
  lmd.setIntensity(10);
  pinMode(PWM_Right, OUTPUT);
  pinMode(PWM_Left, OUTPUT);
  pinMode(DIR_Right, OUTPUT);
  pinMode(DIR_Left, OUTPUT);
  pinMode(IR_Right, INPUT);    // Returns distance from right (IR sensor)
  pinMode(IR_Left, INPUT);     // Returns distance from left (IR sensor)
  pinMode(US_Distance, INPUT); // Returns distance from top (US sensor)

  Servo_Motor.attach(9);
  // Define interrupt to measure distance advancement
  attachInterrupt(digitalPinToInterrupt(Encoder), dist_counter, RISING);
  // Define interrupt on button pin
  attachInterrupt(digitalPinToInterrupt(Button), Skip_Mission, FALLING);
  // Initialize and calibrate sensor
  MPU9250ReadAngles_Setup();

  TOF_Sensor.init();
  TOF_Sensor.setTimeout(500);
  TOF_Sensor.startContinuous();
}

void loop() {
  Serial.println("Start Mission");
  delay(3000);
  // Use mission array to activate the current mission
  Mission_Array[Mission_Number - 1]();
  Serial.println("Mission done");
}

///////////////// Motion functions /////////////////

// Function for movind forwards
void Move_Forwards(float Speed) {
  convertion_const = 1023 / 100.0;
  digitalWrite(DIR_Right, HIGH);
  digitalWrite(DIR_Left, HIGH);
  Timer3.pwm(PWM_Right, Speed * convertion_const);
  Timer3.pwm(PWM_Left, Speed * convertion_const);
}

// Function for moving backwards
void Move_Backwards(float Speed) {
  convertion_const = 1023 / 100.0;
  digitalWrite(DIR_Right, LOW);
  digitalWrite(DIR_Left, LOW);
  Timer3.pwm(PWM_Right, (Speed * convertion_const);
  Timer3.pwm(PWM_Left, (Speed * convertion_const);
}

// Function for turning around to the right in specify angle
void Turn_Right_In_Angle(float Angle) {
  Stop();
  int mission = Mission_Number;
  MPU9250Calculate(deltat);
  Yaw = MPU9250_ReturnYaw();
  Current_Angle = Yaw;
  Desired_Angle = Current_Angle - Angle;
  if (Desired_Angle >= 360) {
    Desired_Angle -= 360;
  } else if (Desired_Angle < 0) {
    Desired_Angle += 360;
  }
  convertion_const = 1023 / 100.0;
  digitalWrite(DIR_Right, LOW);
  digitalWrite(DIR_Left, HIGH);
  Timer3.pwm(PWM_Right, Rotation_Speed * convertion_const);
  Timer3.pwm(PWM_Left, Rotation_Speed * convertion_const);
  threshold = 2.0;
  while (((Yaw <= Desired_Angle - threshold) ||
          (Yaw >= Desired_Angle + threshold)) &&
         (mission == Mission_Number)) {
    noInterrupts();
    Motor_Flag = Motor_Flag_Counter;
    Motor_Flag_Counter = false;
    interrupts();
    if (Motor_Flag) {
      MPU9250Calculate(deltat);
      Yaw = MPU9250_ReturnYaw();
      Serial.println(Yaw);
    }
  }
  Stop();
}

// Function for turning around to the left in specify angle
void Turn_Left_In_Angle(float Angle) {
  Stop();
  int mission = Mission_Number;
  MPU9250Calculate(deltat);
  Yaw = MPU9250_ReturnYaw();
  Current_Angle = Yaw;
  Desired_Angle = Current_Angle + Angle;
  if (Desired_Angle >= 360) {
    Desired_Angle -= 360;
  } else if (Desired_Angle < 0) {
    Desired_Angle += 360;
  }
  convertion_const = 1023 / 100.0;
  digitalWrite(DIR_Right, HIGH);
  digitalWrite(DIR_Left, LOW);
  Timer3.pwm(PWM_Right, Rotation_Speed * convertion_const);
  Timer3.pwm(PWM_Left, Rotation_Speed * convertion_const);
  threshold = 2.0;
  while (((Yaw <= Desired_Angle - threshold) ||
          (Yaw >= Desired_Angle + threshold)) &&
         (mission == Mission_Number)) {
    noInterrupts();
    Motor_Flag = Motor_Flag_Counter;
    Motor_Flag_Counter = false;
    interrupts();
    if (Motor_Flag) {
      MPU9250Calculate(deltat);
      Yaw = MPU9250_ReturnYaw();
      Serial.println(Yaw);
    }
  }
  Stop();
}

// Function for turning (right or left) while driving
void Turn(float Speed_Right, float Speed_Left) {
  convertion_const = 1023 / 100.0;
  digitalWrite(DIR_Right, HIGH);
  digitalWrite(DIR_Left, HIGH);
  Timer3.pwm(PWM_Right, Speed_Right * convertion_const);
  Timer3.pwm(PWM_Left, Speed_Left * convertion_const);
}

// Function for stopping both motors
void Stop() {
  Timer3.pwm(PWM_Right, 0);
  Timer3.pwm(PWM_Left, 0);
}

///////////////////  End mission functions  ///////////////////

// Function for skipping a mission
void Skip_Mission() {
  noInterrupts();
  Stop();
  Mission_Number += 1;
  delay(500);
  interrupts();
}

// Visual indication for start mission
void Start_Mission(byte *matrix, int x, int y, int width, int height) {
  byte mask = B10000000;
  for (int iy = 0; iy < height; iy++) {
    for (int ix = 0; ix < width; ix++) {
      lmd.setPixel(x + ix, y + iy, (bool)(matrix[iy] & mask));
      mask = mask >> 1;
    }
    mask = B10000000;
    lmd.display();
  }
}

///////////////////  Timers Functions  ///////////////////

// Function to initialize timer1
void Timer1_isr(void) {
  Timer1_Counter++;
  if (Timer1_Counter >= DIV) {
    Timer1_Counter = 0;
    Motor_Flag_Counter = true;
  }
}

// Function to raise counter
void dist_counter(void) { dist_cntr++; }

/////////////////// Sensor functions  ///////////////////

// Function to read sensors
void ReadSensors() {
  // Initial Linearization constants
  a = 0.0002391473;
  b = 0.0100251467 TOF_Distance_Front = 0;
  TOF_Distance_Left = 0;
  TOF_Distance_Right = 0;
  US_Distance_Read = 0;
  for (int i = 1; i < 4; i++) {
    IR_Right_Distance += (1 / (a * analogRead(IR_Right) - b));
    IR_Left_Distance += (1 / (a * analogRead(IR_Left) - b));
    US_Distance_Read += analogRead(US_Distance);
  }
  // Output
  distance_scale = 3.0;
  IR_Right_Distance = IR_Right_Distance / distance_scale;
  IR_Left_Distance = IR_Left_Distance / distance_scale;
  Read_TOF();
  US_Distance_Read = US_Distance_Read / distance_scale;
}

// Function to read time of flight sensor
void Read_TOF() {
  Servo_Motor.write(10);
  delay(1000);
  distance_scale = 10.0;
  TOF_Distance_Right =
      TOF_Sensor.readRangeContinuousMillimeters() / distance_scale;
  if (TOF_Sensor.timeoutOccurred())
    Serial.print("TIMEOUT");

  Servo_Motor.write(85);
  delay(1000);
  TOF_Distance_Front =
      TOF_Sensor.readRangeContinuousMillimeters() / distance_scale;
  if (TOF_Sensor.timeoutOccurred())
    Serial.print("TIMEOUT");

  Servo_Motor.write(160);
  delay(1000);
  TOF_Distance_Left =
      TOF_Sensor.readRangeContinuousMillimeters() / distance_scale;
  if (TOF_Sensor.timeoutOccurred())
    Serial.print("TIMEOUT");
}
