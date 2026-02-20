# robot

A ROS2 self-driving RC car written in Python, with Gazebo simulation.

## Architecture

The autonomy stack is split into three layers that communicate over ROS2 topics:

```
                    Sensors (LiDAR, IMU from Gazebo)
                         │ /scan, /imu
┌────────────────────────▼────────────────────────────────────┐
│              Filtering and Perception                       │
│  Processes raw LiDAR + IMU data.                            │
│  Subscribes: /scan, /imu                                    │
│  Publishes:  /odometry/filtered, /obstacles                 │
└────────────────────────┬────────────────────────────────────┘
                         │ /obstacles
┌────────────────────────▼────────────────────────────────────┐
│                        Planning                             │
│  Reactive obstacle avoidance planner.                       │
│  Subscribes: /obstacles                                     │
│  Publishes:  /ackermann_cmd                                 │
└────────────────────────┬────────────────────────────────────┘
                         │ /ackermann_cmd
┌────────────────────────▼────────────────────────────────────┐
│                        Driver                               │
│  Translates Ackermann commands to actuator / Twist commands.│
│  Subscribes: /ackermann_cmd                                 │
│  Publishes:  /cmd_vel                                       │
└─────────────────────────────────────────────────────────────┘
                         │ /cmd_vel
                    Actuators (Gazebo)
```

### ROS2 Packages

| Package | Description |
|---|---|
| `driver` | Low-level actuator interface. Converts `AckermannDriveStamped` commands to `Twist` messages consumed by the Gazebo drive plugin. |
| `filtering_and_perception` | Processes raw LiDAR scans and IMU data into a filtered odometry estimate and obstacle detections. |
| `planning` | Reactive obstacle-avoidance planner that generates `AckermannDriveStamped` commands. |
| `robot_bringup` | Launch files, URDF robot description (xacro), Gazebo world, and RViz2 configuration. |

## Prerequisites

- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) (or later)
- [Gazebo](https://gazebosim.org/) (classic, version ≥ 11)
- Python 3.10+
- `ros-humble-ackermann-msgs`
- `ros-humble-gazebo-ros-pkgs`
- `ros-humble-xacro`

## Building

```bash
# From the workspace root
cd /path/to/robot
colcon build --symlink-install
source install/setup.bash
```

## Running the Simulation

```bash
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

## Testing

Unit tests cover the core logic of each layer and can be run without a full
ROS2 installation:

```bash
python -m pytest src/driver/test/ \
                 src/filtering_and_perception/test/ \
                 src/planning/test/ -v
```

## Topic Map

```
Gazebo ──/scan──────────────► filtering_and_perception_node
Gazebo ──/imu───────────────► filtering_and_perception_node
                               │
                               ├──/odometry/filtered──► (RViz2 / consumers)
                               └──/obstacles──────────► planning_node
                                                         │
                                                         └──/ackermann_cmd──► driver_node
                                                                               │
                                                                               └──/cmd_vel──► Gazebo
```
