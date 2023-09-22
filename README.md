# multi_tb3_sim
Multiple turtlebot3 burger simulation setup test package using ROS2 Humble and Gazebo

## Requirements
- ROS2 Humble
- Gazebo
- Python3.10

## Installation
```sh
cd ~/ros2_ws/src/multi_tb3_sim
python3 -m pip install -r requirements.txt
rosdep install -i -y --from-paths .
```

## Usage
### single tb3
```py
ros2 launch tb3_gazebo test_single_tb3.launch.py
```

### multi tb3
```py
ros2 launch tb3_gazebo test_multi_tb3.launch.py
```

## Directory structure
### tb3_gazebo
- launch
- rviz
- urdf\
Modified from [turtlebot3 github](https://github.com/ROBOTIS-GIT/turtlebot3) based on Apache License2.0
- worlds
