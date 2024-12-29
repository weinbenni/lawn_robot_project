# Lawn Robot Project

Welcome to the Lawn Robot Project! This repository serves as a foundation for developing a robotic lawn mower using ROS 2 (Robot Operating System).

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [License](#license)


## Overview
The Lawn Robot Project is a ROS2 based project that simulates a Yardforce Compact 300RBS using Gazebo Harmonic and ROS2 Jazzy. The main goal of the project is to create a digital version 
of the real robot with all sensors. This is to enable programming of intelligent functions using onboard and additional sensors.


## Features
- **ROS2 Integration**: All functions are implemented using ROS2 Jazzy.
- **Gazebo Simulation**: The robot is fully modelled using Gazebo Harmonic with ROS2 Jazzy. 
- **Full Sensor Integration**: All of the onboard sensors are modelled in Gazebo.


## Installation
To set up the project on your local machine, follow these steps:  

1. **Clone the Repository**:  
   ```bash
   git clone https://github.com/weinbenni/lawn_robot_project.git
   ```

2. **Navigate to the Project Directory**:  
   ```bash
   cd lawn_robot_project
   ```

3. **Install Dependencies**:  
   Ensure you have ROS 2 installed on your system. Then, install the necessary packages:  
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```
   Make sure you are in ```~/dev_ws/```

   Additionally you need Gazebo Harmonic installed. For testing reasons you might also want to install 
   ```bash
   sudo apt-get update
   sudo apt-get install ros-${ROS_DISTRO}-ros-gz
   sudo apt-get install ros-jazzy-teleop-twist-keyboard 
   ```

   

5. **Build the Project**:  
   ```bash
   colcon build
   ```

   Alternatively, to activate automatic builds on changes:
   ```bash
   colcon build --symlink-install
   ```

6. **Source the Setup Script**:  
   ```bash
   source install/setup.bash
   ```


## Usage
After installation, you can launch the simulation environment:  
```bash
ros2 launch lawn_robot_project gazebo_model.launch.py
```
This command starts the simulated environment gazebo, as well as the joint state publisher, differential controller and the robot topic and spawns a robot object into gazebo.


## Project Structure
The repository is organized as follows:  
```
lawn_robot_project/
│
├── launch/         # Launch files to start the robot and simulation environments
├── meshes/         # 3D models and meshes used for visualization
├── worlds/         # Simulation world files for testing the robot
├── parameters/     # Parameters for simulation
├── description/    # Robot description and Gazebo Configuration
│
├── CMakeLists.txt  # Build configuration file
└── package.xml     # Package metadata file
```



## License
This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE.md) file for details.  
