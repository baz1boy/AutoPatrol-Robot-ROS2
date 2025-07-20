# Pabot - An autonomous patrol robot based on ROS 2 and Navigation 2 (EN)

This is an autonomous patrol robot project based on ROS 2 Humble and Navigation 2. After a one-click launch of the simulation environment, navigation system, and application, the robot can autonomously patrol between predefined waypoints on a mapped environment. Upon reaching each target point, the robot first plays a voice message announcing the location, then captures a real-time image using its camera and saves it locally with voice announcement.

This project aims to provide a complete, modular, and easily extensible ROS 2 application example, covering core concepts such as simulation, navigation, node communication, parameter configuration, and multi-node integrated launch.

**Demo Video**:  [![YouTube](https://img.shields.io/badge/YouTube-Watch-red?logo=youtube&logoColor=white)](https://youtu.be/tghYcGstC8o)

## 1. Project Introduction

This project implements a simulation function for an autonomous patrol robot based on ROS 2 and Navigation 2.

The functions of each package are as follows:
<pre>autopatrol_robot        # Functional package for autonomous patrol implementation
autopatrol_interfaces   # Interfaces related to the autonomous patrol system
pabot_description       # Robot description package, including simulation configurations
pabot_navigation2       # Navigation configuration package for the robot
pabot_application       # Python application example code for robot navigation
nav2_custom_planner     # Custom path planning algorithm
nav2_custom_controller  # Custom control algorithm</pre>

This project includes the following core features:
*   **Simulation Environment**: A Gazebo world with a custom room layout.

*   **Robot Model**: A two-wheeled differential drive robot model (URDF/XACRO), integrated with `ros2_control`.

*   **Autonomous Navigation**: Implements localization (AMCL), path planning (NavFn), and local path tracking (DWB) using the Nav2 navigation stack.

*   **Main Application - Autonomous Patrol**:
    *   `patrol_node`: Loads waypoints from a config file and controls the robot to patrol them in order.
    *   `speaker`: A voice playback service node that receives text requests and uses `gTTS` and `mpg123` for real-time speech synthesis and playback.

*   **Modular Interface**:
    *   `autopatrol_interfaces`: Defines the `SpeechText.srv` service to decouple voice logic from application code.

*   **One-Click Launch**: A top-level `pabot_all.launch.py` launch file is provided to launch the entire system—including Gazebo simulation, Nav2 navigation, and all custom application nodes—with a single command.

## 2. Usage Instruction

The development environment for this project is as follows:

*   **Operating System**： Ubuntu 22.04
*   **ROS Version**：ROS 2 Humble
*   **Core Languages**：Python

### 2.1 Installation

This project uses **slam-toolbox** for mapping, **navigation2** for navigation, **Gazebo** for simulation, and **ros2-control** for motion control. Please install the required dependencies before building the project using the following commands:

1. **Install SLAM and Navigation 2**
```bash
sudo apt install ros-$ROS_DISTRO-nav2-bringup 
sudo apt install ros-$ROS_DISTRO-slam-toolbox
```

2. **Install simulation-related packages**
```bash
sudo apt install gazebo
sudo apt install ros-$ROS_DISTRO-robot-state-publisher
sudo apt install ros-$ROS_DISTRO-joint-state-publisher 
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs 
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-humble-ros2-control 
sudo apt install ros-$ROS_DISTRO-xacro
```

3. **Install speech synthesis and image-related packages**
```bash
sudo apt install python3-pip  -y
sudo apt install ros-$ROS_DISTRO-tf-transformations
sudo pip3 install transforms3d
pip install gTTS langdetect
sudo apt install mpg123
# sudo apt install espeak-ng -y
# sudo pip3 install espeakng
```

### 2.2 Run

After installing the dependencies, you can build the project in the workspace directory using the colcon tool:

**Build the packages**:
```bash
colcon build
source install/setup.bash
```

**Observe the robot**:
After launch, the robot will automatically initialize and then begin navigating sequentially to the waypoints defined in the `patrol_robot/config/patrol_config.yaml` file. Before and after reaching each point, you will hear a voice prompt. Upon arrival at each target point, the robot will take a photo and announce the event via speech. The newly captured images can be found in the `autopatrol_robot/images/ directory`.

#### 2.2.1 One-Click Run

**One-click Launch**:
```bash
ros2 launch autopatrol_robot pabot_all.launch.py
```
This command will:
- Launch the Gazebo simulation environment and load the robot model.
- Launch the full Nav2 navigation stack, including AMCL localization, planner, controller, etc.
- Launch `patrol_node` to begin executing the patrol task.
- Launch `speaker` to wait for speech playback requests.

#### 2.2.2 Run Step by Step

Alternatively, you can launch each component in separate terminals:

1.  **Launch Simulation**
```bash
source install/setup.bash
ros2 launch pabot_description gazebo_sim.launch.py
```

2.  **Launch Navigation**
```bash
source install/setup.bash
ros2 launch pabot_navigation2 navigation2.launch.py
```

3.  **Launch Autopatrol Robot**
```bash
source install/setup.bash
ros2 launch autopatrol_robot autopatrol.launch.py
```

### 2.3 Customize Patrol Path and Speed

*   **Modify Patrol Points**:
    Edit the `autopatrol_robot/config/patrol_config.yaml` file and modify the `target_points` list to customize the patrol path.

*   **Adjust Robot Speed**:
    Edit the `patrol_robot_navigation2/config/nav2_params.yaml` file. You can adjust parameters under `controller_server` → `FollowPath` and `velocity_smoother` to modify the robot’s maximum and minimum driving speeds.

## 3. Author

- [bazi](https://youtu.be/tghYcGstC8o)
