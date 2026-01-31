# ROS2 Autonomous Navigation

Autonomous robot navigation system implemented in ROS2 Humble with Gazebo simulation. Features wall following using linear regression for angle estimation and obstacle avoidance using LiDAR segmentation.

## Project Overview

This project was developed for the Robotics and Advanced Control (RKA) course at EHU. It contains two ROS2 packages:

1. **Wall Following**: Maintains parallel orientation to walls using linear regression on LiDAR points
2. **Obstacle Avoidance (rwander)**: Navigates freely while avoiding obstacles by identifying the widest free segment

## Wall Following

### Algorithm

The wall following system uses scikit-learn's LinearRegression to estimate the wall angle:

1. **LiDAR Point Filtering**: Select points from indices 400-800 (right side, 90 degrees) within 2.0m range
2. **Cartesian Conversion**: Transform polar coordinates to (x, y) using:
```
   x = distance * cos(bearing)
   y = distance * sin(bearing)
```
3. **Linear Regression**: Fit a line y = c0 + c1*x to the filtered points
4. **Angle Calculation**: Compute robot-wall angle as theta = atan2(c1, 1)
5. **Proportional Control**: Apply angular velocity w = Kp * theta

### Kp Parameter Analysis

Extensive testing was performed to determine optimal Kp values:

| Kp Value | Behavior |
|----------|----------|
| < 1.0 | Too slow reaction, robot collides with wall |
| 1.0 - 2.0 | Optimal range, stable wall following |
| > 2.0 | Over-correction, robot loses wall tracking |

The system logs data to CSV files for analysis: `wf_ls_{kp}.csv`

### Results

With Kp = 1.5 and linear velocity v = 0.2 m/s:
- Robot completes full circuit around rectangular room
- Maintains stable distance from wall
- Smooth corner transitions

## Obstacle Avoidance (rwander)

### Algorithm

The obstacle avoidance system identifies free segments in the LiDAR scan:

1. **Scan Processing**: Use front 180 degrees (indices 400-1200)
2. **Free Segment Detection**: Identify consecutive readings > 2.0m as free space
3. **Segment Selection**: Choose segment with maximum length, using average distance as tiebreaker
4. **Direction Calculation**:
```
   target_direction = (segment_start + segment_end) / 2
   error = 800 - target_direction
   turn = radians(-error * 0.3)
```
5. **Speed Control**: Linear velocity based on turn angle: `speed = abs(cos(turn)) * 0.5`
6. **Recovery**: If no free segment found, stop and rotate 70 degrees

### Results

| Metric | Value |
|--------|-------|
| Collision-free operation | > 10 minutes |
| Coverage | Full environment explored |
| Position independence | Consistent behavior from any starting point |

## Installation

### Prerequisites

- ROS2 Humble
- Gazebo
- Python 3.10+
- ROSbot simulation packages

### Build
```bash
cd ~/ros2_ws/src
git clone https://github.com/gorkadabo1/ros2-autonomous-navigation.git
cd ..
colcon build --packages-select wall_following rwander
source install/setup.bash
```

## Usage

### Wall Following
```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch rosbot_gazebo simple.launch.py

# Terminal 2: Run wall following node
ros2 launch wall_following wf_ls.launch.py kp:=1.5
```

### Obstacle Avoidance
```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch rosbot_gazebo simple.launch.py

# Terminal 2: Run obstacle avoidance node
ros2 launch rwander rwander.launch.py
```

## Project Structure
```
ros2-autonomous-navigation/
├── wall_following/          # Wall following package
│   ├── package.xml
│   ├── setup.py
│   ├── wall_following/
│   │   └── wall_follow_ls.py
│   └── launch/
│       └── wf_ls.launch.py
├── rwander/                 # Obstacle avoidance package
│   ├── package.xml
│   ├── setup.py
│   ├── rwander/
│   │   └── rwander.py
│   └── launch/
│       └── rwander.launch.py
└── docs/
    ├── wall_following_report.pdf
    └── obstacle_avoidance_report.pdf
```

## Technical Details

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | sensor_msgs/LaserScan | LiDAR input |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/rosbot_pose` | geometry_msgs/Pose2D | Robot odometry |

### Dependencies

- rclpy
- sensor_msgs
- geometry_msgs
- numpy
- scikit-learn (wall_following only)

## Course

Robotics and Advanced Control (Robotika eta Kontrol Adimenduak) EHU