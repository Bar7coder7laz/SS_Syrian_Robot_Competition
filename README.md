Engineering materials
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

## Content

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

## Introduction

_This part must be filled by participants with the technical clarifications about the code: which modules the code consists of, how they are related to the electromechanical components of the vehicle, and what is the process to build/compile/upload the code to the vehicle’s controllers._
