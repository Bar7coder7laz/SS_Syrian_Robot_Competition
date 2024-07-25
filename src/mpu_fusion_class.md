
# `mpu_fusion` Class

## Overview

The `mpu_fusion` class interfaces with the MPU6050 sensor, utilizing its Digital Motion Processor (DMP) to provide accurate orientation data, specifically the yaw angle. The class handles the initialization, calibration, and reading of sensor data to give reliable yaw values.

## Algorithm Explanation

### Initialization

1. **I2C Setup:**
   - The class starts I2C communication and sets the clock speed to ensure fast data transfer. This setup is crucial for timely communication between the MPU6050 sensor and the microcontroller.

2. **MPU6050 Initialization:**
   - The MPU6050 sensor is initialized, and the DMP is set up. This involves configuring the sensor's internal registers to prepare it for data collection and processing.

3. **Set Offsets:**
   - Gyro and accelerometer offsets are configured. These offsets are determined during the calibration process to account for any sensor biases. Proper offsets ensure that the sensor provides accurate measurements.

4. **Calibration:**
   - The accelerometer and gyroscope are calibrated to eliminate any bias and ensure accurate readings. Calibration involves adjusting the sensor readings to match known reference values, which helps in reducing systematic errors.

5. **Enable DMP:**
   - If the DMP initialization is successful, it is enabled to start processing the sensor data. The DMP processes raw data from the accelerometer and gyroscope to provide refined orientation data.

6. **Retrieve Packet Size:**
   - The expected packet size for the DMP is retrieved for future data reads. This packet size indicates how much data will be sent in each DMP update, allowing for efficient data handling.

### Data Processing

1. **Check DMP Readiness:**
   - Before processing any data, the class checks if the DMP is ready. This ensures that the data being read is valid and reliable.

2. **Read DMP Packet:**
   - The class checks for a new DMP packet, which contains the processed sensor data. DMP packets encapsulate orientation data calculated from raw sensor inputs.

3. **Extract Quaternion:**
   - The quaternion representing the sensor’s orientation is extracted from the DMP packet. Quaternions are a mathematical representation that avoids the gimbal lock problem and provides smooth, continuous orientation tracking.

4. **Calculate Gravity Vector:**
   - Using the quaternion, the gravity vector is calculated. This step is crucial for distinguishing between linear acceleration (caused by movement) and gravitational acceleration (caused by tilt). The gravity vector helps in isolating the tilt component of the sensor data.

5. **Compute Yaw, Pitch, and Roll:**
   - The yaw, pitch, and roll angles are computed from the quaternion and gravity vector. These angles describe the orientation of the sensor in three-dimensional space. 
   - **Yaw:** Rotation around the vertical axis.
   - **Pitch:** Rotation around the lateral axis.
   - **Roll:** Rotation around the longitudinal axis.

6. **Store Yaw Angle:**
   - The yaw angle, representing the rotation around the vertical axis, is converted from radians to degrees for easier interpretation and stored in the provided variable. Converting to degrees makes the data more intuitive and useful for applications.

### Sensor Fusion

The MPU6050 uses sensor fusion algorithms within its DMP to combine data from the accelerometer and gyroscope. Here's how it works:

1. **Accelerometer Data:**
   - Measures linear acceleration in three axes (X, Y, Z). This data is used to determine the tilt of the sensor. However, accelerometer data can be noisy and is susceptible to linear motion.

2. **Gyroscope Data:**
   - Measures angular velocity in three axes. Gyroscopes provide good short-term accuracy but tend to drift over time.

3. **Complementary Filter:**
   - The DMP likely uses a complementary filter or a similar algorithm to combine accelerometer and gyroscope data. 
   - The accelerometer provides long-term stability, correcting the drift of the gyroscope.
   - The gyroscope provides short-term stability, smoothing out the accelerometer’s noise.

4. **Quaternion Calculation:**
   - The sensor fusion algorithm calculates quaternions from the combined data. Quaternions provide a robust way to represent rotations, avoiding issues like gimbal lock.

5. **Gravity Vector Calculation:**
   - The quaternion is used to compute the gravity vector. This helps in isolating the gravity component from the total acceleration, allowing for accurate orientation estimation.

6. **Angle Computation:**
   - Yaw, pitch, and roll angles are computed using the quaternion and gravity vector. These angles provide a comprehensive description of the sensor's orientation.

## Usage

The `mpu_fusion` class can be used in projects requiring accurate orientation data from the MPU6050 sensor, such as:

- Robotics
- Drones
- Motion tracking applications
- Virtual reality systems

## Example Usage

To use the `mpu_fusion` class, initialize it with the appropriate gyro offsets, call the `init()` method to set up the sensor, and use `get_Angle_Z()` to retrieve the yaw angle.

## Notes

- Ensure the MPU6050 sensor is properly connected to your Arduino board.
- The `mpu_fusion` class relies on the DMP firmware provided by InvenSense, which processes raw sensor data for more accurate orientation estimates.

## Conclusion

The `mpu_fusion` class provides a comprehensive solution for integrating the MPU6050 sensor into your projects, leveraging its DMP for accurate and reliable orientation data.
