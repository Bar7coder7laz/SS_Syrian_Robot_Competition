#include "Arduino.h" // Standard Arduino library for basic functionalities
#include "I2Cdev.h" // Library for I2C communication
#include "MPU6050_6Axis_MotionApps20.h" // Library for MPU6050 sensor with DMP (Digital Motion Processor) functionalities
#include "math.h" // Standard math library

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h" // Include Wire library if using Arduino I2C implementation
#endif

// Initialize yaw-related variables
float yaw = 0, delta_yaw = 0, last_yaw = 0, real_value = 0, imu_overall_val = 0, number_of_turns = 0, imu_current_val;

// Create an instance of MPU6050 sensor class
MPU6050 mpu;

// Set calibration offsets for the MPU6050
int mx = 60, my = -55;

// Define a class for IMU (Inertial Measurement Unit) functionalities
class mpu_fusion {
  private:
    // Define a macro for readable Yaw, Pitch, and Roll output
    #define OUTPUT_READABLE_YAWPITCHROLL

    // Private member variables
    bool blinkState = false;
    bool dmpReady = false; // Indicates if DMP is ready
    uint8_t mpuIntStatus; // MPU interrupt status
    uint8_t devStatus; // Device status
    uint16_t packetSize; // Expected DMP packet size
    uint16_t fifoCount; // Count of bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // Variables to hold sensor data
    Quaternion q; // Quaternion for sensor orientation
    VectorInt16 aa; // Accelerometer data
    VectorInt16 aaReal; // Gravity-free accelerometer data
    VectorInt16 aaWorld; // World-frame accelerometer data
    VectorFloat gravity; // Gravity vector

    float euler[3]; // Euler angles (not used in this code)
    float ypr[3]; // Yaw, Pitch, Roll angles
    uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' }; // Teapot demo packet

    volatile bool mpuInterrupt = false; // Indicates if MPU interrupt has occurred
    int offx = 0, offy = 0; // Offsets for gyro calibration

  public:
    mpu_fusion(int& setx, int& sety); // Constructor
    ~mpu_fusion(); // Destructor
    void init(); // Initialization function
    void get_Angle_Z(float& yaw); // Function to get yaw angle
};

// Constructor definition
inline mpu_fusion::mpu_fusion(int& setx, int& sety) {
  offx = setx; // Set x-axis offset
  offy = sety; // Set y-axis offset
}

// Destructor definition
mpu_fusion::~mpu_fusion() { }

// Initialization function definition
inline void mpu_fusion::init() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(); // Start I2C communication
  Wire.setClock(400000); // Set I2C clock speed to 400kHz
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true); // Alternative I2C setup (if applicable)
#endif
  mpu.initialize(); // Initialize MPU6050
  devStatus = mpu.dmpInitialize(); // Initialize DMP
  // Set gyro and accelerometer offsets
  mpu.setXGyroOffset(offx);
  mpu.setYGyroOffset(offy);
  mpu.setZGyroOffset(1);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(-2);

  if (devStatus == 0) { // Check if DMP initialization was successful
    mpu.CalibrateAccel(6); // Calibrate accelerometer
    mpu.CalibrateGyro(6); // Calibrate gyro
    mpu.PrintActiveOffsets(); // Print active offsets
    mpu.setDMPEnabled(true); // Enable DMP
    dmpReady = true; // Set DMP ready flag to true

    packetSize = mpu.dmpGetFIFOPacketSize(); // Get expected DMP packet size
  }
}

// Function to get yaw angle
inline void mpu_fusion::get_Angle_Z(float& yaw) {
  if (!dmpReady) return; // Return if DMP is not ready
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Check for new DMP packet
    mpu.dmpGetQuaternion(&q, fifoBuffer); // Get quaternion
    mpu.dmpGetGravity(&gravity, &q); // Get gravity vector
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // Get yaw, pitch, and roll values
    yaw = ypr[0] * 180 / M_PI; // Convert yaw from radians to degrees
  }
}
