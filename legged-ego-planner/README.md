# Overview

This repository implements local obstacle avoidance and navigation tasks for quadruped robots based on traditional VMC controllers and Ego-planner.

This branch have been tested in **ROS Noetic** under **Ubuntu 20.04**. Some users report it can run in **ROS Melodic** under **Ubuntu 18.04**. To ensure optimal performance, we recommend testing this branch in **ROS Noetic** under **Ubuntu 20.04**.

![1690627750459](image/README/overview.png)

# Install

```bash
sudo apt install libyaml-cpp-dev
sudo apt install libeigen3-dev
sudo apt install liblcm-dev
sudo apt install libglm-dev
sudo apt-get install libarmadillo-dev

sudo apt-get install ros-noetic-controller-interface  ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-joint-state-publisher ros-noetic-robot-state-publisher ros-noetic-controller-manager ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller
```

# Build

```bash
git clone  https://gitee.com/HUAWEI-ASCEND/quadruped-robot.git
cd legged-ego-planner
catkin_make -DCMAKE_BUILD_TYPE=Release
```

# Usage

Terminal 1: Start gazebo, load simulation environment and robot. In order to make the system run more smoothly, it's better to modify the argument `wname` in `legged-ego-planner/src/robots/src/simulation/unitree_gazebo/launch/normal.launch` from `forest` to `earth`.

```bash
source devel/setup.bash
taskset -c 0,1,2 roslaunch unitree_gazebo normal.launch
```

Terminal 2: Start robot control

```bash
source devel/setup.bash
cd  src/robots/src/ascend-quadruped-cpp
taskset -c 3,4,5 rosrun ascend_quadruped_cpp a1_sim  2>/dev/null
```

Terminal 3: Start ego-planner

```bash
source devel/setup.bash
taskset -c 6,7 roslaunch ego_planner run_in_sim.launch 2>/dev/null
```

Terminal 4: Start Rviz

```bash
source devel/setup.bash
taskset -c 6,7 roslaunch ego_planner  rviz.launch 2>/dev/null
```

Click '2D Nav Goal' in rviz, give the target point, and then the robot will move along the planned local trajectory to the target point

# Acknowledgements

- This work extends [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) to legged robot navigation.

# Communication

If you have any question, please join our discussion group by scanning the following wechat QR code.

<img src="image/QR-code.jpg" alt="QR" title="" width="200" align=center />
