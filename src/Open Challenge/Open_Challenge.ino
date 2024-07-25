#include <NewPing.h> // Library for ultrasonic sensor
#include <Servo.h> // Library for controlling servo motors
#include "Arduino.h" // Standard Arduino library
#include "I2Cdev.h" // Library for I2C communication
#include "MPU6050_6Axis_MotionApps20.h" // Library for MPU6050 sensor with DMP
#include "math.h" // Standard math library
#include "mpu_fusion.h" // Custom library for MPU fusion

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Variable declarations
int counter = 0; // Counter for turns
float distance = 0, real_distance = 0; // Distance measurements
float angle = 0, Angle = 0; // Angle measurements
float highest_angle = 0.27; // Highest allowable angle
float jbes = 0; // Servo position adjustment
float currentTimePID = 0, lastTimePID = 0, errorPID = 0, referencePID = 0, outputPID = 0; // PID control variables
float theta = 0, theta_mpu = 0, ref_mpu = 0; // Angle variables for MPU
float angle_mean = 0; // Mean angle
int reference_counter = 5; // Counter for reference calculation
float turn_right_ang = 45; // Turn right angle
float Set_Servo = 1; // Servo set flag
float last_errorPID = 0, deltaTimePID = 0; // Last angle error and delta time
int sign = 1; // Sign for direction
float first_ref = 0; // First reference angle
int cnt = 0; // General counter
int arr_right[3] = {3, 4, 5}; // Array for right sensors
int arr_left[3] = {0, 1, 2}; // Array for left sensors
int arrl[2] = {0, 1}; // Another array for left sensors
int pid_spd = 140; // PID speed
int pid_spd_slow = 120; // Slow PID speed
int turn_spd = 110; // Turning speed
int drib_spd = 105; // Dribbling speed
int dribend_spd = 110; // Dribbling end speed
int spd_after_turn = 150; // Speed after turn
int spd_back = 105; // Backward speed
int pid_red_spd = 85; // Reduced PID speed
int pid_green_spd = 85; // Green PID speed

int ptl = 72; // Position left
int ptr = 75; // Position right

float kpl = 1.0; // PID proportional constant left
float kpr = 1.0; // PID proportional constant right
float kp = kpl; // PID proportional constant

double kpc = 3, kdc = 2.3; // PID constants for control
double kpb = 4.6, kdb = 3.8; // PID constants for back control
float max_ang = 120, min_ang = 13; // Max and min angles for servo
int pos_R = 78 , pos_L = 60; // Servo positions
int pos = pos_L; // Initial servo position

///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Servo declarations
Servo myservo;
Servo myservo2;
Servo myservo3;

// Pin definitions
#define flt 24
#define fle 25

#define blt 52
#define ble 53

#define frt 26
#define fre 27

#define brt 42
#define bre 43

#define ft 22
#define fe 23

#define bt 30
#define be 31

#define mlt 44
#define mle 45

#define mrt 36
#define mre 37

#define MAX_DISTANCE 310 // Maximum distance for ultrasonic sensors
#define MAX_DISTANCE2 500 // Secondary maximum distance for ultrasonic sensors

// Arrays for sensor pins
int echo[8] = {ble, mle, fle, bre, mre, fre, fe, be};
int trig[8] = {blt, mlt, flt, brt, mrt, frt, ft, bt};

// Sensor indices
int bl = 0, ml = 1, fl = 2;
int br = 3, mr = 4, fr = 5;
int f = 6, b = 7;
int arr1[] = {0, 1, 2}; // Left sensors
int arr2[] = {3, 4, 5}; // Right sensors

float d[8]; // Distance array for sensors
float m = 0.0185; // Distance conversion factor

// Array of NewPing objects for ultrasonic sensors
NewPing ultra[8] = {
  NewPing(trig[0], echo[0], MAX_DISTANCE),
  NewPing(trig[1], echo[1], MAX_DISTANCE),
  NewPing(trig[2], echo[2], MAX_DISTANCE),
  NewPing(trig[3], echo[3], MAX_DISTANCE),
  NewPing(trig[4], echo[4], MAX_DISTANCE),
  NewPing(trig[5], echo[5], MAX_DISTANCE),
  NewPing(trig[6], echo[6], MAX_DISTANCE),
  NewPing(trig[7], echo[7], MAX_DISTANCE),
};

// Function to get distance from ultrasonic sensor
float dist(int id, int it) {
  d[id] = m * ultra[id].ping_median(it); // Convert ping time to distance
  if (d[id] == 0) d[id] = MAX_DISTANCE; // If no ping, set distance to max
  return d[id];
}

// Motor control pins
#define R_EN 11
#define L_EN 33
#define R_PWM 12
#define L_PWM 13

// Function to move forward with specified speed
void forward(int pwm) {
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, pwm);
}

// Function to stop the motors
void stopp() {
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
}

// Function to move backward with specified speed
void backward(int pwm = 0) {
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  analogWrite(R_PWM, pwm);
  analogWrite(L_PWM, 0);
}

// MPU fusion object
mpu_fusion imu(mx, my);

// Function to get Z-axis angle (yaw) from MPU
void get_AngleZ() {
  imu.get_Angle_Z(yaw); // Get yaw value
  if (Wire.getWireTimeoutFlag()) { // Check for I2C timeout
    Wire.end();
    delay(2000);
    Wire.begin();
    delay(5000);
    Wire.clearWireTimeoutFlag();
  }
  delay(5); // Delay to avoid too frequent DMP requests
  delta_yaw = yaw - last_yaw; // Calculate delta yaw
  if (delta_yaw < 150 && delta_yaw > -150) { // Handle DMP value wrap-around
    real_value += delta_yaw;
    imu_overall_val = real_value;
  }
  last_yaw = yaw;
}

// Function to calculate angle based on direction
float Angle_cal(int dir) {
  if (dir == 1) {
    Angle = atan((dist(2, 3) - dist(0, 3)) / 15.9);
  } else {
    Angle = atan((dist(5, 3) - dist(3, 3)) / 15.9);
  }

  if (Angle > highest_angle) {
    Angle = highest_angle;
  }
  if (Angle < (-1) * highest_angle) {
    Angle = (-1) * highest_angle;
  }
  Angle = Angle * 180 / M_PI;
  return Angle;
}

// Function to calculate distance based on direction
float Distance(int dir) {
  float distance;
  if (dir == 1) {
    distance = (0.65 * dist(2, 3) + 0.35 * dist(0, 3));
  } else {
    distance = (0.65 * dist(5, 3) + 0.35 * dist(3, 3));
  }
  angle = Angle_cal(dir) * M_PI / 180;
  real_distance = distance * cos(angle);
  return real_distance;
}

// Proportional control function for left side
void P_L(float kp) {
  jbes = pos - sign * (kp * (Angle_cal(sign)));
  if (jbes < min_ang) jbes = min_ang;
  if (jbes > max_ang) jbes = max_ang;
  myservo3.write(jbes);
}

// PID control function for angle adjustment
void controlPIDAngle(double current_angle) {
  currentTimePID = (micros()) * (1e-3);
  deltaTimePID = currentTimePID - lastTimePID;
  lastTimePID = currentTimePID;
  errorPID = current_angle - referencePID;
  outputPID = pos - (kpc * errorPID + ((errorPID - last_errorPID) / deltaTimePID) * kdc);
  if (outputPID > max_ang) outputPID = max_ang;
  if (outputPID < min_ang) outputPID = min_ang;
  myservo3.write(outputPID);
  last_errorPID = errorPID;
}

// Backward PID control function for angle adjustment
void controlPIDBackAngle(double current_angle) {
  currentTimePID = (micros()) * (1e-3);
  deltaTimePID = currentTimePID - lastTimePID;
  lastTimePID = currentTimePID;
  errorPID = current_angle - referencePID;
  outputPID = pos + (kpb * errorPID + ((errorPID - last_errorPID) / deltaTimePID) * kdb);
  if (outputPID > max_ang) outputPID = max_ang;
  if (outputPID < min_ang) outputPID = min_ang;
  myservo3.write(outputPID);
  last_errorPID = errorPID;
}

// Function to take reference for angle calculation
void take_ref(int dir) {
  angle_mean = 0;
  while (reference_counter--) {
    angle_mean += Angle_cal(dir);
  }
  reference_counter = 5;
  get_AngleZ();
  referencePID = imu_overall_val;
  referencePID -= (dir * (angle_mean / reference_counter));
}

// Function to take reference from MPU
void take_ref_mpu() {
  float mpu_mean = 0;
  for (int i = 0; i < reference_counter; i++) {
    get_AngleZ();
    mpu_mean += imu_overall_val;
  }
  referencePID = mpu_mean / reference_counter;
}

// Function to adjust reference based on given angle
void take_ref_mpu_ang(int angg) {
  referencePID = referencePID + sign * angg;
}

// Function to parse sensor values and find minimum
double Parse_Val(int *y, int l, int iter = 3) {
  double minn = 1.0 * MAX_DISTANCE;
  int ii;
  for (int j = 0; j < l; j++) {
    int i = y[j];
    d[i] = dist(i, iter);
    if (d[i] < minn) {
      minn = d[i];
      ii = i;
    }
    delay(1);
  }
  return ii;
}

// Start function to initialize servo and take reference
void start() {
  if (Set_Servo) {
    myservo3.write(pos_L);
    Set_Servo = 0;
    take_ref(1);
  }
}

// Function to reverse sensor arrays if conditions met
void revultra() {
  if (dist(0, 3) > 100 || dist(2, 3) > 100) {
    for (int i = 0; i < 3; i++) {
      arr1[i] = i + 3;
      arr2[i] = i;
    }
    pos = pos_R;
    sign = -1;
    ptl = ptr;
    kp = kpl;
  }
}

// Turn function for robot
void turn() {
  while (true) {
    if (dist(6, 3) > 35) break;
    backward(spd_back);
    get_AngleZ();
    controlPIDBackAngle(imu_overall_val);
  }
  myservo3.write(pos);
  theta = ptl - Angle_cal(sign);
  get_AngleZ();
  theta_mpu = imu_overall_val;
  ref_mpu = theta_mpu;
  theta = ref_mpu + sign * theta;

  while (true) {
    if ((sign * (theta_mpu - theta)) >= 0) break;
    get_AngleZ();
    theta_mpu = imu_overall_val;
    forward(turn_spd);
    myservo3.write(pos + sign * turn_right_ang);
  }
  myservo3.write(pos);
  take_ref(sign);
  counter += 1;
  while (true) {
    if (dist(7, 3) > 100) break;
    forward(pid_spd);
    get_AngleZ();
    controlPIDAngle(imu_overall_val);
  }
}

// Arduino setup function
void setup() {
  imu.init();
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  Serial.begin(9600);
  myservo2.attach(8);
  myservo2.write(70);
  myservo3.attach(7);
  myservo.attach(9);
  myservo.write(110);
}

// Arduino loop function
void loop() {
  if (!counter) {
    start();
    while (true) {
      if (dist(2, 3) > 100 || dist(0, 3) > 100 || dist(3, 3) > 100 || dist(5, 3) > 100) {
        stopp();
        break;
      }
      if (dist(6, 3) <= 100 && dist(6, 3) > 0) {
        forward(pid_spd_slow);
        get_AngleZ();
        controlPIDAngle(imu_overall_val);
      }
      if (dist(6, 3) > 100) {
        forward(pid_spd);
        get_AngleZ();
        controlPIDAngle(imu_overall_val);
      }
    }
    revultra();
  }
  if (dist(6, 3) <= 100 && dist(6, 3) > 0) {
    forward(pid_spd_slow);
    get_AngleZ();
    controlPIDAngle(imu_overall_val);
    if (dist(arr2[2], 3) > 90 || dist(arr2[0], 3) > 90) {
      stopp();
      turn();
    }
  }
  if (dist(6, 3) > 100) {
    forward(pid_spd);
    get_AngleZ();
    controlPIDAngle(imu_overall_val);
  }

  if (counter == 12) {
    stopp();
    delay(1000000);
  }
}
