# Automated Banana Cutting System

![ROS](https://img.shields.io/badge/ROS-Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3-3776AB?style=for-the-badge&logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-Computer%20Vision-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)
![DOBOT](https://img.shields.io/badge/Hardware-DOBOT%20Magician-ff69b4?style=for-the-badge&logo=robot&logoColor=white)

##  Project Description

This repository implements a robotic system for precision processing of banana bunches. At the moment the project tests feasibility using a DOBOT Magician robotic arm. The final system will autonomously identify, cut, and separate banana hands.

The main objective is to separate banana hands from bunches while maximizing efficiency and minimizing product damage, using an end-effector designed specifically for this task.

## Main Characteristics

* **Computer Vision:** Implementation of image processing algorithms to spatially locate each banana hand and determine the optimal cutting point on the stem.
* **End-Effector:** Development of a customized end-effector tool that combines simultaneous cutting and gripping mechanisms. This allows securing the fruit before cutting to prevent drops.
* **Automatic Classification:** Classify the bunch's hands.
* **Trajectory Planning**: Trajectory planning to carefully place each separated hand in a collection area.

## System Operation

1.  **Detection:** The camera captures the bunch and the algorithm computes cutting coordinates.
2.  **Approach:** The DOBOT moves to the target position.
3.  **Execution:** End-effector grips and cuts the hand.
4.  **Collection:** The robot moves the separated hand to the output tray.

Are you insterested in this project or want to contribute? Feel free to reach out! If you find this project helpful, please give us a star ⭐! 