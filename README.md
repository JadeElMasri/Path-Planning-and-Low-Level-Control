This repository contains my personal contributions to our Final Year Project (FYP), titled "Developing Robot SHARE-C's Full Autonomy."

The system was primarily developed on an NVIDIA Jetson TX2 and an Arduino Mega, communicating using rosserial on ROS Melodic.

My work involved:

- Integrated **RTAB-MAP** for perception, including mapping and localization both experimentally and in simulation (Gazebo)

- Set up, developed and configured **move_base** for path planning.

- Developed low-level **PID controllers** on the Arduino.

- Implemented **keyboard teleoperation** for manual robot control.

- Created a custom **battery monitoring** node to estimate and publish battery percentage from voltage sensor readings.

Note: This repository includes only the components I developed. Other aspects of the project (e.g., Self-disinfection system, GUI, Mechanical Design) were handled by my teammates.

## 📌 Poster Preview

<img src="./FYP_Poster.png" alt="Poster Preview" width="500"/>

## 🤖 Robot – SHARE-C

<img src="./sharec_robot.PNG" alt="SHARE-C Robot" width="300"/>

## 📹 Video Demonstration

This [video](https://youtu.be/2a8PZjOhHwQ) presents the project motivation and a comprehensive demonstration of SHARE-C’s main functionalities.
