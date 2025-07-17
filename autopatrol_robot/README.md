# An autonomous patrol robot based on ROS 2 and Navigation 2 (ZH-CN)

This is an autonomous patrol robot project based on ROS 2 Humble and Navigation 2. After a one-click launch of the simulation environment, navigation system, and application, the robot can autonomously patrol between predefined waypoints on a mapped environment. Upon reaching each target point, the robot first plays a voice message announcing the location, then captures a real-time image using its camera and saves it locally with voice announcement.

This project aims to provide a complete, modular, and easily extensible ROS 2 application example, covering core concepts such as simulation, navigation, node communication, parameter configuration, and multi-node integrated launch.

## 1.Introduction

This project implements a simulation function for an autonomous patrol robot based on ROS 2 and Navigation 2.

各功能包功能如下：
<pre>autopatrol_robot        #自动巡检实现功能包
autopatrol_interfaces   #自动巡检相关接口
pabot_description       #机器人描述文件，包含仿真相关配置
pabot_navigation2       #机器人导航配置文件
pabot_application       #机器人导航应用 Python 代码
nav2_custom_planner     #自定义规划算法
nav2_custom_controller  #自定义控制算法</pre>

本项目包含以下核心功能：
*   **仿真环境**: 使用 Gazebo 搭建了一个包含自定义房间布局的仿真世界。

*   **机器人模型**: 包含一个两轮差速驱动的机器人模型（URDF/XACRO），并集成了 `ros2_control` 进行控制。

*   **自主导航**: 利用 Nav2 导航栈实现机器人的定位（AMCL）、路径规划（NavFn）和局部路径跟踪（DWB）。

*   **核心应用 - 自主巡逻**:
    *   `patrol_node`: 核心巡逻逻辑节点，负责从配置文件加载路径点，并控制机器人依次导航。
    *   `speaker`: 语音播放服务节点，接收文本请求，通过 `gTTS` 和 `mpg123` 实现语音的实时合成与播放。

*   **模块化接口**:
    *   `autopatrol_interfaces`: 一个独立的接口包，定义了语音播放服务的 `SpeechText.srv` 接口，实现了应用与服务的解耦。

*   **一键启动**: 提供了一个顶层的 `pabot_all.launch.py` 文件，能够一键启动包括 Gazebo 仿真、Nav2 导航以及所有自定义应用节点在内的完整系统. 

## 2.使用方法

本项目开发平台信息如下：

*   **系统版本**： Ubuntu 22.04
*   **ROS 版本**：ROS 2 Humble
*   **核心语言**：Python, C++

### 2.1安装

本项目建图采用 slam-toolbox，导航采用 Navigation 2, 仿真采用 Gazebo，运动控制采用 ros2-control 实现，构建之前请先安装依赖，指令如下：

1. **安装 SLAM 和 Navigation 2**
```bash
sudo apt install ros-$ROS_DISTRO-nav2-bringup 
sudo apt install ros-$ROS_DISTRO-slam-toolbox
```

2. **安装仿真相关功能包**
```bash
sudo apt install ros-$ROS_DISTRO-robot-state-publisher
sudo apt install ros-$ROS_DISTRO-joint-state-publisher 
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs 
sudo apt install ros-$ROS_DISTRO-ros2-controllers 
sudo apt install ros-$ROS_DISTRO-xacro
```

3. **安装语音合成和图像相关功能包**
```bash
sudo apt install python3-pip  -y
sudo apt install ros-$ROS_DISTRO-tf-transformations
sudo pip3 install transforms3d
pip install gTTS langdetect
sudo apt install mpg123
# sudo apt install espeak-ng -y
# sudo pip3 install espeakng
```

### 2.2运行

安装完成依赖后，可以使用 colcon 工具在workspace目录中进行构建:

**构建功能包**
```bash
colcon build
source install/setup.bash
```

**观察机器人**:
启动后，机器人会自动进行初始化，然后开始依次导航到 `patrol_robot/config/patrol_config.yaml` 文件中定义的路径点。每个点出发前后，你会听到语音提示，并在到达目标点后进行拍照并且语音播报，你可以在 `autopatrol_robot/images/` 目录下看到新记录的图像。

#### 2.2.1 运行

**一键启动**:
```bash
ros2 launch autopatrol_robot pabot_all.launch.py
```
这条命令将会:
- 启动 Gazebo 仿真环境并加载机器人模型。
- 启动完整的 Nav2 导航栈，包括 AMCL 定位、路径规划器、控制器等。
- 启动 `patrol_node` 开始执行巡逻任务。
- 启动 `speaker` 等待语音播报请求。

#### 2.2.2 运行

或者通过打开多个新终端依次启动：

1.  **运行仿真**
```bash
source install/setup.bash
ros2 launch pabot_description gazebo_sim.launch.py
```

2.  **运行导航**
```bash
source install/setup.bash
ros2 launch pabot_navigation2 navigation2.launch.py
```

3.  **运行自动巡检**
```bash
source install/setup.bash
ros2 launch autopatrol_robot autopatrol.launch.py
```

### 2.3 自定义巡逻路径和速度

*   **修改巡逻点**:
    编辑 `autopatrol_robot/config/patrol_config.yaml` 文件，修改 `target_points` 列表即可自定义巡逻路径。

*   **调整机器人速度**:
    编辑 `patrol_robot_navigation2/config/nav2_params.yaml` 文件。你可以修改 `controller_server`--`FollowPath` 和 `velocity_smoother` 中的参数来调整机器人的最大最小行驶速度。

## 3.作者

- [bazi](http://)
