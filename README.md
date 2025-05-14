This repository contains my personal contributions to our Final Year Project (FYP), titled "Developing Robot SHARE-C's Full Autonomy."

The system was primarily developed on an NVIDIA Jetson TX2 and an Arduino Mega, communicating using rosserial on ROS Melodic.

My work involved:

- Integrated **RTAB-MAP** for perception, including mapping and localization both experimentally and in simulation (Gazebo)

- Set up, developed and configured **move_base** path planning.

- Developed low-level **PID controllers** on the Arduino.

- Implemented **keyboard teleoperation** for manual robot control.

- Created a custom **battery monitoring** node to estimate and publish battery percentage from voltage sensor's readings.

Note: This repository includes only the components I developed. Other aspects of the project (e.g., SLAM tuning, GUI, mechanical design) were handled by my teammates.

## 📌 Poster Preview

<img src="./FYP_Poster.png" alt="Poster Preview" width="500"/>

## 🤖 Robot – SHARE-C

<img src="./sharec_robot.PNG" alt="SHARE-C Robot" width="300"/>

## 📹 Video Demonstration

 [Click here to watch the project demonstration on YouTube](https://youtu.be/2a8PZjOhHwQ)
