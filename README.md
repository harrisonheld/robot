# robot
A ROS2 self-driving RC car written in Python, with Gazebo simulation.

## Prerequisites
- Ubuntu 24.04 (Noble)
- ROS2 (Jazzy)
- Gazebo (Harmonic)
- Python 3.12+
- `ros-jazzy-ackermann-msgs`
- `ros-jazzy-gazebo-ros-pkgs`
- `ros-jazzy-xacro`

## Building
```bash
# From the workspace root
colcon build --symlink-install
```

## Running the Simulation
```bash
source install/setup.bash
source /opt/ros/jazzy/setup.bash
ros2 launch robot_bringup robot.launch.py
```

Optional launch arguments:

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use Gazebo simulation clock. |
| `use_rviz` | `true` | Launch RViz2 for visualisation. |

Example – headless (no RViz2):

```bash
ros2 launch robot_bringup robot.launch.py use_rviz:=false
```

## Injecting Topics
A `geometry_msgs/msg/Twist` is a command which has a translational velocity and rotational velocity.
For a ground robot, we typically only use linear.x - how fast to move forward at, and angular.z, how much to rotate.
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.5}, angular: {z: 0.2}}"
```

## Unit Tests
```bash
python -m pytest src/driver/test/ \
                 src/state_estimation/test/ \
                 src/perception/test/ \
                 src/planning/test/ -v
```