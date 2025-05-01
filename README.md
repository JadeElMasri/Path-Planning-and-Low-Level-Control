# Path-Planning-and-Low-Level-Control

This repository contains my individual contributions to our Final Year Project (FYP), titled **"Developing Robot SHARE-C's Full Autonomy"**, specifically focusing on **path planning** and **low-level control**.

The system was primarily developed on an **NVIDIA Jetson TX2** and an **Arduino Mega**, communicating via **rosserial** under **ROS Melodic**.

My responsibilities included:
- Setting up and tuning the `move_base` navigation stack, used in conjunction with `rtabmap` SLAM for perception and localization.
- Developing low-level **PID motor controllers** on the Arduino.
- Integrating **teleoperation via keyboard** for manual control.
- Creating a custom **battery monitoring node** to estimate and publish battery percentage from raw sensor voltage data.

 Note: This repository showcases only the components I personally developed. Other parts of the project (e.g., SLAM implementation, GUI, or mechanical design) were handled by my teammates.

[Uploading FYP Final Poster.pdf…](FYP Poster)
