# Content

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.
* 
Engineering Materials
====

# Robot Mechanism

In self-driving cars, the most important aspect is their mechanical mechanisms, which ensure smooth and safe operation. Our design incorporates differential gears for the Rear-Wheel Drive (RWD) system, providing the necessary balance and control during turns. For steering, we utilize a Direct-Acting Steering mechanism, which offers precise control and reliability.

## Differential Gears

Differential gears play a crucial role in automotive engineering. They allow the wheels on a vehicle to rotate at different speeds, particularly when the vehicle is turning. This functionality is vital because the inner wheels travel a shorter distance compared to the outer wheels during a turn.

### Types of Differentials

1. **Open Differential**: The most common type, found in many non-performance vehicles. It is simple and cost-effective but can transfer torque to a wheel without traction, causing it to spin uselessly.

   <p align="center">
     <img src="https://github.com/Bar7coder7laz/SS_Syrian_Robot_Competition/blob/main/other/images/Differential-Image-1.webp" width="600" height="400">
   </p>

2. **Locking Differential**: Often used in off-road vehicles, this type can lock the wheels to spin at the same speed, providing better traction on uneven surfaces.

   <p align="center">
     <img src="https://github.com/Bar7coder7laz/SS_Syrian_Robot_Competition/blob/main/other/images/locking%20diff.jpg" width="600" height="400">
   </p>

3. **Limited-Slip Differential (LSD)**: Combines features of both open and locking differentials, allowing wheels to spin at different speeds but can transfer more torque to the wheel with better traction.

   <p align="center">
     <img src="https://github.com/Bar7coder7laz/SS_Syrian_Robot_Competition/blob/main/other/images/kmp-lsd-with-names-01.png" width="600" height="400">
   </p>

### Benefits of Differential Gears

- **Improved Handling**: Differentials enhance vehicle handling, especially around corners, by allowing wheels to rotate at appropriate speeds.
- **Increased Traction**: Locking and limited-slip differentials provide better traction on slippery or uneven surfaces.
- **Enhanced Stability**: They improve stability by distributing power evenly to the wheels, preventing loss of control.

## Direct-Acting Steering

Direct-Acting Steering mechanisms provide precise and responsive control, essential for the safety and efficiency of self-driving cars. This system translates the driver's or automated system's input directly into wheel movement without the delay and complexity of traditional steering systems.

### Advantages of Direct-Acting Steering

- **Enhanced Precision**: Direct control over the wheel movement allows for more accurate and reliable steering.
- **Increased Safety**: By reducing the potential for mechanical failure, direct-acting systems improve the overall safety of the vehicle.
- **Better Responsiveness**: The immediate translation of steering input to wheel action ensures a quicker response to driving conditions.

## Fusion Between Differential Gears and Direct-acting Steering Mechanisms in Our Robot

### Differential Gears Using Lego Parts
In our robot we used differential gears as shown in the following photo:

<p align="center">
  <img src="https://github.com/Bar7coder7laz/SS_Syrian_Robot_Competition/blob/main/other/images/diff%20lego%20pars.webp" width="600" height="400">
</p>

### Direct-Acting Steering depending on Picar-X Kit
The steps of assembling the steering mechanism are shown below step-by-step:

<p align="center">
  <img src="https://github.com/Bar7coder7laz/SS_Syrian_Robot_Competition/blob/main/other/images/1.png" width="600" height="400">
</p>

<p align="center">
  <img src="https://github.com/Bar7coder7laz/SS_Syrian_Robot_Competition/blob/main/other/images/2.png" width="600" height="400">
</p>

<p align="center">
  <img src="https://github.com/Bar7coder7laz/SS_Syrian_Robot_Competition/blob/main/other/images/3.png" width="600" height="400">
</p>

<p align="center">
  <img src="https://github.com/Bar7coder7laz/SS_Syrian_Robot_Competition/blob/main/other/images/4.png" width="600" height="400">
</p>

Incorporating differential gears and direct-acting steering mechanisms in self-driving cars is essential for ensuring smooth turns, improved traction, precise control, and overall vehicle stability. Understanding these components helps appreciate the technology that contributes to safer and more efficient driving experiences.

## Vehicle Main Body

The vehicle main body is based on Picar-x kit and the building instructions are available via the [link](https://github.com/sunfounder/sf-pdf/raw/master/assembly_file/a0000710-picar-x.pdf) with some vital modifications and crucial restructuring in order to be able to meet the competition requirements:

The robot Rear Wheel Drive (RWD) is the most important factor in any car whether it is a real car or a self-driving car. The Picar-x kit had two DC motors each one is responsible for one wheel. However, we are required to use one DC so we crafted a new design for the RWD. In this design, the transmission of motion is achieved through the implementation of differential gears using LEGO parts. The Importance of Differential Gears lies in that it enables the rear wheels to spin simultaneously at various speeds while preserving power distribution. When the robot is turning or there is a discrepancy in traction between the two wheels, the differential gear's main job is to allow the wheels on the same axle to rotate at different rates. It permits the wheels to spin at different speeds while distributing torque from the DC Motor to them. In order for the rear wheels to be compatible with the Lego differential gears, they were also replaced with Lego wheels. It's crucial to note that we utilized the identical Direct Acting Steering mechanism as the kit described above; but, because of their low friction characteristic, the front wheels had to be replaced due to the fact that the robot needs more force to steer at the right value.

Due to space constraints, we also added two extra plexi plates to the primary chassis of the robot( the primary chassis can only hold a place for the LiPo battery that we have). The first plexi was required for eight ultrasonic sensors (three on each side, one in front, and one in the back, and one in the middle with a drift angle of 35 degrees),  and a place for our main microcontroller is Atmega which is mounted on Arduino Mega 2560 board.Moreover, this plexi also contains a small fan for the RaspberryPi 4B, and a BTS7960 to control the DC motor. The RaspberryPi 4B, our robot's primary microprocessor, is located in the second plexi. On the other hand, the RaspberryPi is linked to a camera module that is located on two SG90 Servos, one of which moves it horizontally and the other vertically. Lastly, MPU_6050 is situated both above and in the middle of the robot with regard to the x and y axes. For more accurate measurements, we positioned the mpu precisely in the center of the robot using Lego parts.

### Picar-X Robot _VS_ Scud Storm Robot _(Our Robot)_ 
The following photos show a comparasion between our robot and the Picar-X Robot. This comparasion is to show how much effort did we put to the design to satisfy our algorithm requirments and the competition requirments:


## Mobility Management

### DC Motor

In general, the motors used in small self-driving cars are expected to possess several key attributes including light weight, compactness, and high efficiency. These characteristics are necessary to ensure that the motor can operate for extended periods of time while utilizing a limited power source. A motor that is relatively low-speed could be suitable for a small self-driving car as it can provide adequate torque to drive the vehicle while also being lightweight and compact. However, the selection of a motor for a self-driving car is influenced by a variety of factors such as the vehicle's size and weight, the desired speed and torque output, and the power requirements of the electronic components. Other considerations include the availability and cost of replacement parts as well as the level of expertise needed for installation and maintenance. After taking into account all of these factors the DC JGA25- 245 RPM motor was chosen because it met the desired specifications. It provides the following features: 

| Speed (r/min) | Current (Amp) | Torque (Kg.cm) | Power (Watt) |
|---------------|----------------|----------------|--------------|
| 133           | 0.5            | 0.75           | 1.1          |
| 245           | 0.65           | 1.4            | 2.4          |

### Motor Control Setup

To control the DC motor we used BTS 7960 driver circuit and the Arduino Mega board. Then we followed these steps:

1. We connected the BTS 7960 driver to the Arduino Mega. The BTS 7960 driver has six input pins which are used to control the motor. However, we only needed 4 pins to control our motor reliably. These pins are connected to the appropriate pins on the Arduino Mega. The connections are:
   - L_EN and R_EN pins are connected to digital output pins on the Arduino Mega.
   - L_PWM and R_PWM are connected to PWM output pin on the Arduino Mega.
2. Then we connected the DC motor to the BTS 7960 driver: The DC motor is connected to the output terminals of the BTS 7960 driver. The polarity of these connections determines the direction of the motor rotation.
3. Lastly, we connected the driver to an external 12V power supply.

### Servo Motor

Our choice of a SG90 Micro Digital 9G Servo (SunFounder Servo) motor for a small self-driving car depended on several factors which are presented below:

- **Compact size:** The SunFounder Servo motor is a small and lightweight motor (10± 0.5g) which makes it ideal for use in small self-driving cars where space is limited.
- **High torque:** Despite its small size, the SunFounder Servo motor can provide a relatively Stall Torque (1.4 kg/cm (4.8V) 1.6 kg/cm (6V)) which is essential for steering the car accurately and precisely.
- **Wide operating range:** The SunFounder Servo motor has a wide operating range of 0-180 degrees with a mechanical limit angle is 360 degrees which makes it versatile and suitable for a variety of steering applications.
- **Low power consumption:** The motor has a relatively low power consumption as follows:
  - Standby Current: ≤4 mA
  - Consumption Current (at 4.8V No Load): ≦50mA
  - Consumption Current (at 6 V no load): ≦60mA
  - Stall Current (at locked 4.8V): ≦550mA
  - Stall Current (at locked 6V): ≦650mA
- **High speed:** The motor has a suitable speed for the steering task (No Load Speed: 4.8V ≦0.14sec/60°; 6V ≦0.12sec/60°).

## Robot Design

Designing a robot for the Future Engineers WRO competition requires careful planning and consideration of several factors. For that we followed the following steps:

### Understanding the Competition Requirements

The first step is to thoroughly read and understand the competition rules and requirements. This helped us determine the specific tasks that the robot needs to perform and the constraints that need to be considered such as the weight and the dimensions of the robot.

### Defining the Robot's Objectives

Based on the competition requirements we determined the objectives of the robot such as the tasks it needs to perform the environment it will operate in and the constraints it needs to adhere to.

### Designing the Robot's Mechanical System
#### RWD Mechannism

It was previousely explained in details in this repository.

#### Steering Mechanism

Once the objectives are defined design the mechanical system of the robot including the wheels motors and any other mechanical components needed to achieve the objectives. As a steering mechanism, we chose the Direct-Acting steering mechanism which is a popular choice for small self-driving cars because it provides accurate and precise control over steering while also being relatively simple to implement. This mechanism uses a linkage system to allow the front wheels to turn at the same angles which is better when dealing with small self-driving cars than Ackermann steering for example. This design allows the car to have a small turning radius which is important for navigating tight spaces and making sharp turns. Additionally, it provides a stable and predictable steering response which is crucial for self-driving cars that need to maintain precise control over their movements.

### Choosing the Electronic Components

The selection process for the electronic components that will govern the robot's movements and behaviors has been conducted meticulously. These components include the microcontroller- an Arduino Mega board, and the microprocessor- a RaspberryPi 4B. Sensory devices have also been chosen encompassing Ultrasonic sensors, mpu6050 (gyroscope and acceleration sensor), and a RaspberryPi Camera 1.3 module (5 MegaPixels) which will provide the robot with the ability to perceive its environment. Actuators including a DC Motor and a Servo Motor have been chosen to facilitate the physical actions of the robot. The power supply will be ensured by one Li-poly RC 12 V and two 18650 li-ion (3.7 V) batteries. Additional electrical components such as the driver circuit and two step down voltage regulators (Lm2596) have also been selected to augment the robot's functionality. These components collectively will provide the necessary control over the robot's movements and actions.

## Power and Sense Management

### Power Supply

In our design, we utilized two 18650 li-ion (3.7 V) batteries to provide power for the RaspberryPi (since we have serial communication, RaspberryPi supplies Arduino). Additionally, the Li-poly RC 12 V is responsible to power all the other electronic components.

### Sensors

In addition to using a gyroscope, acceleration sensors (mpu 6050), and a camera to detect pillars, we integrated eight ultrasonic sensors (HC-SR04), three of which were placed on each side of the robot, one on the front side, and one on the left back side. Two of the sensors on each side have a unique feature in that two of them are parallel to the walls, but the middle one has a drifting angle of 35 degrees, a number that was carefully studied because it adds a great deal of flexibility to the obstacles management algorithm.

## Open Challenge Strategy

### Ultrasonic Sensors Reading

We created a newping array that contains all the sensors. It contains each sensor's trigger pin echo pin and max distance to ping. We also used a function that calculates the distance measured by an ultrasonic sensor with a specific id. The function uses a median filter to obtain a more accurate and stable distance measurement.

### Keeping the Robot Straight in the First Straightforward Section

We used a PD controlling algorithm that depends on the value of the tilt angle measured by the gyroscope sensor to calculate the corrected angle. This algorithm controls the robot in a way that makes the tilt angle measured by the gyroscope equal to a reference angle. And any deviating from this reference will be considered an error and will be corrected by the algorithm. We used this algorithm to control the robot movement in the first straightforward section before identifying the movement direction. The algorithm works as follows:

#### Identifying the Reference Angle

The reference angle is the angle measured by the gyroscope at the beginning of the round. It is important to consider that the robot may not be completely straight at the beginning of the round so the reference angle is corrected by subtracting the tilt angle of the robot that is calculated depending on ultrasonic sensors’ readings.

#### Calculating Angle Error

The angle error is the difference between the angle measured by the gyroscope during the movement and the reference angle.

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?\color{White}\text{angError}_t%20=%20\text{Ang}_t%20-%20\text{Ang}_{t-1}%20\quad%20(1)" style="background-color: black; padding: 10px; border-radius: 5px; width: 30%;">
</p>

#### Calculating the PD Output

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?\color{White}\text{out}%20=%20k_p%20\times%20\text{angError}_t%20+%20k_d%20\times%20\frac{\text{angError}_t%20-%20\text{angError}_{t-1}}{\text{timeDifference}}" style="background-color: black; padding: 10px; border-radius: 5px; width: 50%;">
</p>

Where:
- out is the output correction of the algorithm.
- angError_t is the angle error calculated at the time t.
- angError_{t-1} is the angle error calculated at the time t-1.
- timeDifference is the time difference between the two readings of the angle.
- k_p is the proportional constant.
- k_d is the differential constant.

#### Calculating the Corrected Angle

The corrected angle is the angle that would be set to the servo motor to correct the robot movement. It is calculated with the following formula:

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?\color{White}\text{correctedAng}%20=%20\text{servoAng}%20-%20\text{out}" style="background-color: black; padding: 10px; border-radius: 5px; width: 30%;">
</p>

Where:
- servoAng is the angle that would be set to the servo motor to keep the robot straight.

### Detecting the First Turn and the Direction of Movement

If one of the left ultrasonics detects a distance greater than a specific threshold, then the direction of movement is counterclockwise. On the other hand, if one of the right ultrasonics detects a distance greater than the threshold, the direction of movement is clockwise. To make the code more dynamic we created two arrays called arr1 and arr2. Arr1 stores the IDs of the ultrasonic sensors that would be used to detect the wall, whereas arr2 stores the IDs of the ultrasonic sensors that would be used to detect a turn. The values of these two arrays are identified in the first turn.

### Keeping the Robot Straight After Identifying the Direction of Movement

After identifying the direction of movement the robot will be kept straight depending on wall detection algorithm that detects the outer wall. This algorithm controls the robot in a way that minimizes the tilt angle of the robot that is calculated using ultrasonic sensors’ readings. It calculates the tilt angle of the robot and multiplies it by a constant to obtain the corrected angle that would be set to servo motor. The value of this constant is obtained from experiment and calibration. The algorithm works as follows:

#### Calculating the Tilt Angle of the Robot

This angle is calculated depending on the readings of the two left or two right ultrasonic sensors as follows:

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?\color{White}\text{tiltAng}%20=%20\arctan\left(\frac{\text{dis1}%20-%20\text{dis2}}{\text{UltraDis}}\right)" style="background-color: black; padding: 10px; border-radius: 5px; width: 30%;">
</p>

Where:
- dis1 is the distance measured by the front left or front right ultrasonic sensor.
- dis2 is the distance measured by the back left or back right ultrasonic sensor.
- UltraDis is the distance between the front left and the back left sensors which is the same as the distance between the front right and the back right sensors.

#### Calculating the Corrected Angle

The corrected angle is the angle that would be set to the servo motor to correct the robot movement and is calculated as follows:

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?\color{White}\text{correctedAng}%20=%20\text{StraightAng}%20-%20\text{sign}%20\times%20k_p%20\times%20\text{tiltAng}" style="background-color: black; padding: 10px; border-radius: 5px; width: 50%;">
</p>

Where:
- correctedAng is the angle that would be set to servo motor to correct the robot movement.
- StraightAng is the servo angle that keeps the robot straight.
- Sign is a variable that indicates the direction of movement. It is equal to 1 if the direction is CW, whereas it is equal to -1 if the direction is CCW.
- k_p is a constant identified by experiment and calibration.
- tiltAng is the tilt angle of the robot.

### Detecting Turns

A turn is detected if one of the ultrasonic sensors whose IDs are stored in arr2 array measures a distance greater than a specific threshold.

### Turning the Robot

Once a turn is detected the robot would turn left or right depending on the identified movement direction. The turn angle is calculated depending on the tilt angle of the robot before the turn. After the turn angle is calculated the robot would turn until the difference between the current gyro angle and the reference gyro angle would be equal to turn angle. The turn angle is calculated as following:

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?\color{White}\text{turnLeftAng}%20=%20\text{requiredLeftAng}%20-%20\text{tiltAng}" style="background-color: black; padding: 10px; border-radius: 5px; width: 40%;">
</p>

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?\color{White}\text{turnRightAng}%20=%20\text{requiredRightAng}%20-%20\text{tiltAng}" style="background-color: black; padding: 10px; border-radius: 5px; width: 40%;">
</p>

Where: 
- turnLeftAng is the required change in gyro angle to complete the left turn.
- turnRightAng is the required change in gyro angle to complete the right turn.
- tiltAng is the tilt angle of the robot from the wall which is calculated with formula (4).
- requiredLeftAng is the required change in gyro angle to complete the left turn in case the tilt angle of the robot before the turn is zero.
- requiredRightAng is the required change in gyro angle to complete the right turn in case the tilt angle of the robot before the turn is zero.

### Stopping the Robot in the Finish Section

After completing three laps the robot must stop in the finish section. To do this we used a counter that counts the number of turn times. The value of this counter is increased after completing each turn. When its value is equal to 12 the three laps will be completed. The finish section is recognized depending on the value of the front ultrasonic sensor. The robot would stop when the value of the front distance is less than a specific threshold.

## Obstacle Managment Strategy
