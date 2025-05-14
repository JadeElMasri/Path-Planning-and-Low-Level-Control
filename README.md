This repository contains my personal contributions to our Final Year Project (FYP), titled "Developing Robot SHARE-C's Full Autonomy."

The system was primarily developed on an NVIDIA Jetson TX2 and an Arduino Mega, communicating using rosserial on ROS Melodic.

My work involved:

- Integrated RTAB-MAP for perception, including mapping and localization both experimentally and simulation (Gazebo)

- Configured and calibrated the move_base navigation stack for autonomous path planning.

- Developed low-level PID motor controllers on the Arduino.

- Implemented keyboard teleoperation for manual robot control.

- Created a custom battery monitoring node to estimate and publish battery percentage from raw voltage readings.

Note: This repository includes only the components I developed. Other aspects of the project (e.g., SLAM tuning, GUI, mechanical design) were handled by my teammates.

## 📌 Poster Preview

<img src="./FYP_Poster.png" alt="Poster Preview" width="500"/>

## 🤖 Robot – SHARE-C

<img src="./sharec_robot.PNG" alt="SHARE-C Robot" width="300"/>

## 📹 Video Demonstration

The following link : https://youtu.be/2a8PZjOhHwQ Video showcasing the motivation and demonstration of this project 
