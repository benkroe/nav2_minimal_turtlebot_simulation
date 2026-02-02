# nav2_minimal_turtlebot_simulation

This repository is a fork of the [nav2_minimal_turtlebot_simulation](https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation) package, adapted specifically for TurtleBot4 with enhancements for multi-robot simulations.

The original package was designed for single-robot use with the Nav2 stack. To enable multi-robot use with the TurtleBot 4, the model was changed as following.

1. **Namespacing on the Simulation (Gazebo) Level**: Added namespacing on the simulation level to allow multiple robots in the same simulation without overlapping sensors
2. **Addition of Proximity Sensors**: Equips the robot with sensors to detect nearby objects or other robots.
3. **Simplified Visual Meshes**: Replaces complex meshes with cylinders and cubes to accelerate simulation performance.
4. **Gazebo to ROS2 Bridges**: Brings the positions of the all robots to ROS 2, to allow for neighbour sensing and loggin of positions direct in ROS 2. 
5. **Launch Files for Multiple Robots**: Supports launching multiple robot.
6. **Variable to switch of lidar**: Allows for easy switch between robot with and without lidar

While TurtleBot3 support is included for compatibility, the primary focus of this fork is TurtleBot4.

## Installation and Dependencies

### Dependencies
- ROS 2 (any distribution)
- Gazebo
- ROS packages: `ros_gz_bridge`, `ros_gz_interfaces`, `ros_gz_image`, `ros_gz_sim`, `robot_state_publisher`, `xacro`, `joint_state_publisher`, `urdf`

### Installation Steps
1. Clone this repository into the `src` directory of your ROS 2 workspace:
   ```
   git clone git@github.com:benkroe/nav2_minimal_turtlebot_simulation.git
   ```
2. Build the workspace:
   ```
   colcon build --symlink-install
   ```
3. Source the setup script:
   ```
   source install/setup.bash
   ```
4. Launch the simulation using the provided launch files (see below).

## Launch Files

This package provides several launch files for different simulation scenarios, all located in `nav2_minimal_tb4_sim/launch/`.

### simulation.launch.py
Launches a single TurtleBot4 robot in a Gazebo simulation environment. Includes RViz for visualization, robot state publisher, and all necessary bridges.

**Usage:**
```
ros2 launch nav2_minimal_tb4_sim simulation.launch.py
```

**Key Parameters:**
- `namespace`: Top-level namespace (default: empty)
- `use_rviz`: Whether to start RViz (default: True)
- `spawn_robot`: Whether to spawn the robot (default: True)
- Other parameters for pose, world, etc.

### multi_robot_simulation.launch.py
Launches multiple TurtleBot4 robots in the same Gazebo world, each with its own namespace. Useful for testing multi-robot navigation and coordination.

**Usage:**
```
ros2 launch nav2_minimal_tb4_sim multi_robot_simulation.launch.py
```

**Key Parameters:**
- `rviz_for_all_robots`: Launch RViz for each robot (default: True)
- `robot_sdf`: Path to the robot URDF/xacro file (default: TurtleBot4)
- Other parameters for world, simulator settings, etc.

By default, spawns 4 robots with namespaces `tb1`, `tb2`, `tb3`, `tb4`.

### spawn_tb4.launch.py
Spawns a single TurtleBot4 robot in an existing Gazebo simulation. Used internally by the other launch files but can be used standalone for custom setups.

**Usage:**
```
ros2 launch nav2_minimal_tb4_sim spawn_tb4.launch.py namespace:=my_robot
```

**Key Parameters:**
- `namespace`: Namespace for the robot
- `robot_name`: Name of the robot in Gazebo
- `robot_sdf`: Path to the robot URDF/xacro file
- Pose parameters (`x_pose`, `y_pose`, etc.)

## Using the enable_rplidar Variable

The TurtleBot4 URDF xacro file includes an `enable_rplidar` property that controls whether the RPLIDAR sensor is included in the robot model. This is set to `0` (disabled) by default to improve simulation performance, especially in multi-robot scenarios.

To enable the RPLIDAR sensor:
- When processing the xacro file manually: `xacro turtlebot4.urdf.xacro enable_rplidar:=1`
- In custom launch files, modify the `robot_sdf` parameter or the xacro command to include `enable_rplidar:=1`


