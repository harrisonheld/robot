# robot

A ROS2 self-driving RC car written in Python, with Gazebo simulation.

## Architecture

The autonomy stack is split into four layers that communicate over ROS2 topics:

```
         /imu (Gazebo)                    /scan (Gazebo)
              │                                │
┌─────────────▼─────────────┐  ┌──────────────▼──────────────┐
│      State Estimation     │  │         Perception           │
│  Integrates IMU → heading │  │  LiDAR scan → obstacles      │
│  Subscribes: /imu         │  │  Subscribes: /scan           │
│  Publishes:               │  │  Publishes:                  │
│    /odometry/filtered     │  │    /obstacles                │
└───────────────────────────┘  └──────────────┬──────────────┘
                                               │ /obstacles
                               ┌──────────────▼──────────────┐
                               │           Planning           │
                               │  Reactive obstacle avoidance │
                               │  Subscribes: /obstacles      │
                               │  Publishes:  /ackermann_cmd  │
                               └──────────────┬──────────────┘
                                               │ /ackermann_cmd
                               ┌──────────────▼──────────────┐
                               │            Driver            │
                               │  Ackermann → Twist           │
                               │  Subscribes: /ackermann_cmd  │
                               │  Publishes:  /cmd_vel        │
                               └──────────────┬──────────────┘
                                               │ /cmd_vel
                                        Actuators (Gazebo)
```

### ROS2 Packages

| Package | Description |
|---|---|
| `driver` | Low-level actuator interface. Converts `AckermannDriveStamped` commands to `Twist` messages consumed by the Gazebo drive plugin. |
| `state_estimation` | Integrates IMU gyroscope data to produce a filtered heading odometry estimate on `/odometry/filtered`. |
| `perception` | Processes raw LiDAR scans to detect the closest obstacle and publishes its distance and bearing on `/obstacles`. |
| `planning` | Reactive obstacle-avoidance planner that generates `AckermannDriveStamped` commands. |
| `robot_bringup` | Launch files, URDF robot description (xacro), Gazebo world, and RViz2 configuration. |

## Prerequisites

- Ubuntu 24.04 (Noble Numbat)
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- [Gazebo](https://gazebosim.org/) (classic, version ≥ 11)
- Python 3.12+
- `ros-jazzy-ackermann-msgs`
- `ros-jazzy-gazebo-ros-pkgs`
- `ros-jazzy-xacro`

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
                 src/state_estimation/test/ \
                 src/perception/test/ \
                 src/planning/test/ -v
```

## Topic Map

```
Gazebo ──/imu───────────────► state_estimation_node ──/odometry/filtered──► (RViz2 / consumers)
Gazebo ──/scan──────────────► perception_node ──/obstacles──► planning_node
                                                                │
                                                                └──/ackermann_cmd──► driver_node
                                                                                      │
                                                                                      └──/cmd_vel──► Gazebo
```

