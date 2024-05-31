# Montecarlo Visual Loc

<div align="center">
<img width=600px src="https://github.com/GuilleAQ/Mobile-Robotics_IRS-23/blob/main/P5/resources/figures/1.png" alt="explode"></a> 
</div>

<h3 align="center"> Montecarlo Visual Loc </h3>

<div align="center">
<img width=100px src="https://img.shields.io/badge/status-finished-brightgreen" alt="explode"></a>
<img width=100px src="https://img.shields.io/badge/license-Apache-orange" alt="explode"></a>
</div>

## Table of Contents
- [Table of Contents](#table-of-contents)
- [Introduction](#introduction)
- [Task Description](#task-description)
- [First Approach](#first-approach)
- [Algorithm](#algorithm)
- [Setup and Installation](#setup-and-installation)
- [Usage](#usage)
- [Video Demo](#video-demo)
- [References](#references)
- [Contributors](#contributors)
- [License](#license)

## Introduction
This project aims to develop a visual localization algorithm based on the particle filter, commonly known as Monte Carlo Localization (MCL). This method is used to estimate the position of a robot within a known map using sensor data.

## Task Description
The aim of this practice is to develop a visual localization algorithm based on the particle filter. The algorithm will handle noisy sensor data to provide an accurate position estimate of the robot within the environment.

## First Approach
In the initial phase, I focused on understanding the basics of Monte Carlo Localization and setting up a simulation environment. Key tasks included:
- Familiarizing with the particle filter algorithm.
- Setting up the simulation environment.
- Developing the initial version of the localization algorithm.

## Algorithm
The Monte Carlo Localization algorithm consists of the following steps:
1. **Initialization**: Generate a set of particles with random positions and orientations.
2. **Prediction**: Update particle positions based on the robot's motion model.
3. **Measurement Update**: Weight each particle based on how well the predicted measurements match the actual sensor data.
4. **Resampling**: Select particles with high weights to form a new set of particles.

<div align="center">
<img width=600px src="https://github.com/GuilleAQ/Mobile-Robotics_IRS-23/blob/main/P5/resources/figures/2.png" alt="explode"></a> 
</div>

### Key Components
- **Particle Filter**: The core of the MCL algorithm, which uses particles to represent possible robot positions.
- **Motion Model**: Predicts the next state of each particle based on the robot's movements.
- **Sensor Model**: Updates the weights of particles based on the similarity between predicted and actual sensor readings.

## Setup and Installation
To set up the project, follow these steps:
1. Clone the repository:
   ```sh
   git clone https://github.com/GuilleAQ/Mobile-Robotics_IRS-23.git
   cd Mobile-Robotics_IRS-23/P5
   ```

## Usage
To run the localization algorithm, execute the following command:

```sh
python3 MonteCarloLaserLocalization.py
```
This script initializes the particle filter and runs the localization algorithm using simulated sensor data.

## Video Demo
A video demonstration of the Monte Carlo Localization algorithm in action can be found [here](https://urjc-my.sharepoint.com/personal/g_alcocer_2020_alumnos_urjc_es/_layouts/15/stream.aspx?id=%2Fpersonal%2Fg%5Falcocer%5F2020%5Falumnos%5Furjc%5Fes%2FDocuments%2FDocumentos%2Fvideo%5Fsim%2Emp4&nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJTdHJlYW1XZWJBcHAiLCJyZWZlcnJhbFZpZXciOiJTaGFyZURpYWxvZy1MaW5rIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXcifX0%3D&referrer=StreamWebApp%2EWeb&referrerScenario=AddressBarCopied%2Eview%2E18368be3%2Daf66%2D4cab%2D8b3d%2Da8559f1b7d71)

## References
For more detailed information on Monte Carlo Localization and its implementation, refer to the following resources:

[JdeRobot Robotics Academy - Monte Carlo Visual Localization](https://jderobot.github.io/RoboticsAcademy/exercises/ComputerVision/montecarlo_visual_loc)

## Contributors
[GuilleAQ](https://github.com/GuilleAQ)

## License
This project is licensed under the Apache License. See the LICENSE file for details.


