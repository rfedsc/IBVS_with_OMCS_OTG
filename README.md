# IBVS_with_OMCS_OTG

A ROS (Robot Operating System) package for **Image-Based Visual Servoing (IBVS)** integrated with **Ruckig Online Trajectory Generation (OTG)** and **Model Predictive Control (MPC)** for the JR603 robotic arm. This package implements real-time visual servoing control, smooth trajectory planning, EtherCAT communication, PID control, and MPC-based IBVS control, with support for AprilTag detection and multi-version control algorithm iterations.

## Table of Contents

- [Overview](#overview)
- [Dependencies](#dependencies)
- [Package Structure](#package-structure)
- [Getting Started](#getting-started)
- [Key Features](#key-features)
- [Experimental Tests](#experimental-tests)
- [Launch Files](#launch-files)
- [Custom Messages/Services](#custom-messages-services)
- [License](#license)

## Overview

This package is specifically designed for the JR603 6-degree-of-freedom (DoF) robotic arm, focusing on advanced Image-Based Visual Servoing (IBVS) control. It integrates Ruckig Online Trajectory Generation (OTG) for smooth motion planning and Model Predictive Control (MPC) for optimized IBVS performance. The package includes comprehensive modules for kinematics (forward/inverse), real-time AprilTag detection, EtherCAT-based low-latency motion control, PID tuning, and multiple versions of IBVS control algorithms (including feedforward and MPC variants), enabling flexible, precise, and robust robotic arm operation.

## Dependencies

- ROS Noetic (Ubuntu 20.04) / Melodic (Ubuntu 18.04)
- [Ruckig](https://github.com/pantor/ruckig) (OTG library for smooth trajectory generation)
- [ViSP](https://visp.inria.fr/) (Visual Servoing Platform)
- OpenCV (for image processing and AprilTag detection)
- Python 3 (for kinematics calculations, hardware communication, and control scripting)
- EtherCAT communication libraries (for JR603 hardware interface)
- MPC solver dependencies (for model predictive control implementation)
- `geometry_msgs`, `sensor_msgs`, `std_msgs` (standard ROS message packages)
- YAML-cpp (for MPC configuration parsing)

## Package Structure

```plaintext
IBVS_with_OMCS_OTG/
├── CMakeLists.txt                  # ROS build configuration
├── LICENSE                         # License information
├── mpc_config.yaml                 # MPC controller configuration file
├── package.xml                     # ROS package metadata and dependencies
├── README.md                       # Project documentation
├── docs/                           # Documentation resources
│   └── images/                     # Experimental test images
│       ├── camera_velocity_and_error_curves.png  # Velocity/error experiment results
│       ├── image_trajectory.png    # IBVS visual trajectory results
│       └── test_platform.jpg       # Experimental platform setup
├── include/lys_visp_demo/          # C++ header files (empty in current structure)
├── launch/                         # ROS launch files for quick execution
│   ├── jr603_fd_ibvs_ctrlV0_pos_socket.launch    # Feedforward IBVS V0
│   ├── jr603_fd_ibvs_ctrlV1_pos_socket.launch    # Feedforward IBVS V1
│   ├── jr603_ibvs_pos_socket.launch              # Basic IBVS position control
│   ├── jr603_ibvs_pos_with_ruckig_socket.launch  # IBVS with Ruckig OTG
│   ├── jr603_ibvs_test.launch                    # IBVS functionality test
│   ├── jr603_mpc_ibvs_ctrlV4_pos_socket.launch   # MPC-based IBVS V4
│   ├── jr603_mpc_ibvs_ctrlV5_pos_socket.launch   # MPC-based IBVS V5
│   ├── jr603_mpc_ibvs_ctrlV6_pos_socket.launch   # MPC-based IBVS V6
│   ├── jr603_mpc_ibvs_pos_socket.launch          # Basic MPC IBVS
│   ├── jr603_pid_test.launch                     # PID controller tuning/test
│   ├── jr603_velocity_ethercatV0.launch          # EtherCAT velocity control V0
│   ├── jr603_velocity_test.launch                # Velocity control test
│   ├── jr603_velocity_test_V2.launch             # Velocity control test V2
│   ├── jr603_velocity_with_ruckig_etharcatV0.launch  # Velocity control + Ruckig V0
│   ├── jr603_velocity_with_ruckig_etharcatV1.launch  # Velocity control + Ruckig V1
│   ├── jr603_velocity_with_ruckig_etharcatV2.launch  # Velocity control + Ruckig V2
│   ├── jr603_velocity_with_ruckig_etharcatV3.launch  # Velocity control + Ruckig V3
│   ├── jr603_velocity_with_ruckig_etharcatV4.launch  # Velocity control + Ruckig V4
│   └── mpc_config.yaml             # MPC configuration for launch
├── msg/                            # Custom ROS message definitions
│   ├── AprilTagCorners.msg         # AprilTag corner coordinates
│   ├── HomogeneousTransform.msg    # 4x4 homogeneous transformation matrix
│   └── PixelCoordinates.msg        # 2D pixel coordinates of visual features
├── scripts/                        # Python control and utility scripts
│   ├── apriltag_detect.py          # AprilTag detection node
│   ├── dh.py                       # DH parameter kinematics
│   ├── fkine_6dof.py               # 6DoF forward kinematics
│   ├── ikine_jr603.py              # JR603 inverse kinematics
│   ├── jr603_pid_ctrl.py           # Python PID controller
│   ├── ethercat_socket_driver.py   # EtherCAT socket communication driver
│   ├── jr603_joints_pos_with_ruckig.py  # Ruckig OTG joint position control
│   ├── velocity_camera2end.py      # Camera velocity to end-effector conversion
│   ├── lys_python_sdk/             # Custom Python SDK for JR603 control
│   │   ├── fkine_6dof.py           # SDK forward kinematics
│   │   ├── ikine_jr603.py          # SDK inverse kinematics
│   │   ├── elephant_robot.py       # Robot hardware interface
│   │   └── __pycache__/            # Python compiled cache
│   └── __pycache__/                # Python compiled cache
├── src/                            # C++ source files
│   ├── jr603_IBVS.cpp              # Core IBVS implementation
│   ├── jr603_mpc_ibvs_ctrlV4.cpp   # MPC IBVS controller V4
│   ├── jr603_mpc_ibvs_ctrlV5.cpp   # MPC IBVS controller V5
│   ├── jr603_mpc_ibvs_ctrlV6.cpp   # MPC IBVS controller V6
│   ├── jr603_pos_with_otg.cpp      # Position control with Ruckig OTG
│   ├── jr603_velocity_with_otg.cpp # Velocity control with Ruckig OTG
│   ├── mpc_ibvs_controller.hpp     # MPC controller header
│   ├── pid.cpp                     # PID controller implementation
│   ├── otg.cpp                     # Ruckig OTG wrapper
│   ├── visp_ctrl.cpp               # ViSP visual servoing control
│   └── libruckig.so                # Ruckig shared library
└── srv/                            # Custom ROS service definitions
    └── SendCommand.srv             # Service for sending JR603 motion commands
```

- ## Getting Started

  ### 1. Clone the Repository

  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/your-username/IBVS_with_OMCS_OTG.git  # Replace with your repo URL
  cd .. && catkin_make
  source devel/setup.bash
  ```

  ### 2. Install Dependencies

  ```bash
  # Install ROS dependencies
  rosdep install --from-paths src --ignore-src -r -y
  
  # Install additional dependencies
  sudo apt-get install ros-noetic-visp ros-noetic-opencv-apps
  sudo apt-get install libyaml-cpp-dev
  
  # Install Ruckig library (follow official instructions)
  # https://github.com/pantor/ruckig
  
  # Install MPC solver (e.g., ipopt or qpOASES)
  sudo apt-get install coinor-libipopt-dev
  ```

  ### 3. Configure MPC Parameters

  Edit the MPC configuration file to adjust control parameters:

  ```bash
  nano ~/catkin_ws/src/IBVS_with_OMCS_OTG/mpc_config.yaml
  ```

  ### 4. Run a Launch File

  #### Basic IBVS with Ruckig OTG

  ```bash
  roslaunch IBVS_with_OMCS_OTG jr603_ibvs_pos_with_ruckig_socket.launch
  ```

  #### MPC-based IBVS (Version 5)

  ```bash
  roslaunch IBVS_with_OMCS_OTG jr603_mpc_ibvs_ctrlV5_pos_socket.launch
  ```

  #### Velocity Control with EtherCAT and Ruckig

  ```bash
  roslaunch IBVS_with_OMCS_OTG jr603_velocity_with_ruckig_etharcatV2.launch
  ```

  ## Key Features

  - **Multi-version IBVS Control**: Implements basic IBVS, feedforward-enhanced IBVS (V0/V1), and MPC-based IBVS (V4/V5/V6) using ViSP and custom control logic.
  - **Ruckig OTG Integration**: Generates jerk-limited, smooth trajectories for position/velocity control across multiple versions (V0-V4) to ensure safe and stable motion.
  - **Model Predictive Control (MPC)**: Advanced MPC-based IBVS control with configurable parameters via YAML for optimized visual servoing performance.
  - **Complete Kinematics Suite**: Forward (FK) and inverse (IK) kinematics for JR603 6DoF arm implemented in both Python and C++.
  - **EtherCAT Communication**: Low-latency hardware interface with multiple version iterations for reliable JR603 arm control.
  - **AprilTag Detection**: Real-time detection and feature extraction of AprilTag markers for visual servoing targets.
  - **PID Control**: Configurable PID controllers for joint velocity/position regulation with dedicated test launch files.
  - **Multi-sensor Support**: Includes scripts for Intel RealSense D435 camera (color/depth) for visual feedback.
  - **Modular Architecture**: Separated control algorithms, trajectory generation, and hardware interface for easy extension and testing.

## Experimental Tests

This section presents the experimental platform and test results of the IBVS visual servoing for the JR603 robotic arm, verifying the performance of Ruckig OTG trajectory optimization, PID control, and MPC-based IBVS closed-loop control.

### 1. Experimental Platform

The test platform consists of the JR603 6DoF robotic arm, an industrial camera, an AprilTag calibration board, and a host computer running ROS. The hardware connection and deployment are shown below:

<img src="docs/images/test_platform.jpg" width="600" alt="JR603 Experimental Plaform">

### 2. Visual Servoing Trajectory

The following figure shows the visual trajectory tracking effect of the robotic arm end-effector during IBVS control. The trajectory is calculated based on the feature points of the detected AprilTag markers, reflecting the real-time response of the closed-loop control system:

<img src="C:\Users\lys\AppData\Roaming\Typora\typora-user-images\image-20260302121958314.png" width="50%">

<img src="C:\Users\lys\AppData\Roaming\Typora\typora-user-images\image-20260302122027121.png" width="50%">

### 3. Velocity and Error Curves

The curves below display the camera velocity response and visual error convergence during the experiment. It can be observed that the Ruckig OTG library effectively optimizes the motion process, reducing the system's jerk and enabling the visual error to converge stably to the target range:

![image-20260302120041341](C:\Users\lys\AppData\Roaming\Typora\typora-user-images\image-20260302120041341.png)

![](C:\Users\lys\AppData\Roaming\Typora\typora-user-images\image-20260302115923229.png)

## Launch Files

The `launch/` directory contains pre-configured launch files organized by control type:

### IBVS Control

|                Launch File                 |                         Description                         |
| :----------------------------------------: | :---------------------------------------------------------: |
|       `jr603_ibvs_pos_socket.launch`       |    Basic IBVS position control via socket communication     |
| `jr603_ibvs_pos_with_ruckig_socket.launch` | IBVS position control with Ruckig OTG trajectory generation |
|  `jr603_fd_ibvs_ctrlV0_pos_socket.launch`  |             Feedforward-enhanced IBVS version 0             |
|  `jr603_fd_ibvs_ctrlV1_pos_socket.launch`  |             Feedforward-enhanced IBVS version 1             |
|          `jr603_ibvs_test.launch`          |                Basic IBVS functionality test                |

### MPC-based IBVS

|                Launch File                |               Description                |
| :---------------------------------------: | :--------------------------------------: |
|    `jr603_mpc_ibvs_pos_socket.launch`     |       Basic MPC-based IBVS control       |
| `jr603_mpc_ibvs_ctrlV4_pos_socket.launch` |        MPC IBVS control version 4        |
| `jr603_mpc_ibvs_ctrlV5_pos_socket.launch` | MPC IBVS control version 5 (recommended) |
| `jr603_mpc_ibvs_ctrlV6_pos_socket.launch` |        MPC IBVS control version 6        |

### Velocity Control

|                  Launch File                   |                Description                |
| :--------------------------------------------: | :---------------------------------------: |
|       `jr603_velocity_ethercatV0.launch`       | Basic EtherCAT velocity control version 0 |
|          `jr603_velocity_test.launch`          |   Velocity control test (base version)    |
|        `jr603_velocity_test_V2.launch`         |      Velocity control test version 2      |
| `jr603_velocity_with_ruckig_etharcatV0.launch` |  Velocity control + Ruckig OTG version 0  |
| `jr603_velocity_with_ruckig_etharcatV1.launch` |  Velocity control + Ruckig OTG version 1  |
| `jr603_velocity_with_ruckig_etharcatV2.launch` |  Velocity control + Ruckig OTG version 2  |
| `jr603_velocity_with_ruckig_etharcatV3.launch` |  Velocity control + Ruckig OTG version 3  |
| `jr603_velocity_with_ruckig_etharcatV4.launch` |  Velocity control + Ruckig OTG version 4  |

### PID Testing

|       Launch File       |             Description              |
| :---------------------: | :----------------------------------: |
| `jr603_pid_test.launch` | PID controller tuning and validation |

## Custom Messages/Services

### Messages (`msg/`)

- `AprilTagCorners.msg`: Stores 2D pixel coordinates of the four corners of detected AprilTag markers
- `HomogeneousTransform.msg`: Represents a 4x4 homogeneous transformation matrix for robot pose representation
- `PixelCoordinates.msg`: Contains 2D pixel coordinates of visual features (e.g., AprilTag center points)

### Services (`srv/`)

- `SendCommand.srv`: Custom service for sending motion commands to the JR603 arm, supporting joint angle commands, Cartesian pose commands, and velocity commands

## License

This project is licensed under the [MIT License](LICENSE) - see the LICENSE file for details.

------

*For hardware-specific setup (e.g., JR603 EtherCAT configuration, camera calibration) or troubleshooting, refer to the JR603 robotic arm's official documentation and Intel RealSense SDK documentation.*

------

*For hardware-specific setup (e.g., JR603 EtherCAT configuration) or troubleshooting, refer to the JR603 robotic arm's official documentation.*
