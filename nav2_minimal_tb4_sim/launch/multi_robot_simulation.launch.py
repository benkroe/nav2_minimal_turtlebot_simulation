
import os
from typing import List, Dict

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    sim_dir = get_package_share_directory('nav2_minimal_tb4_sim')
    desc_dir = get_package_share_directory('nav2_minimal_tb4_description')
    launch_dir = os.path.join(sim_dir, 'launch')

    # Global args
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    robot_sdf = LaunchConfiguration('robot_sdf')
    rviz_for_all_robots = LaunchConfiguration('rviz_for_all_robots')

    # Default robot configurations. Extend or make this YAML-driven later.
    # Each entry: namespace, name, x, y, z, R, P, Y
    default_robot_configs: List[Dict[str, str]] = [
        {'namespace': 'tb1', 'name': 'tb4_1', 'x': '0.0',  'y': '0.0',  'z': '0.05', 'R': '0.0', 'P': '0.0', 'Y': '0.0'},
        {'namespace': 'tb2', 'name': 'tb4_2', 'x': '1.5',  'y': '0.0',  'z': '0.05', 'R': '0.0', 'P': '0.0', 'Y': '0.0'},
        {'namespace': 'tb3', 'name': 'tb4_3', 'x': '2.0',  'y': '0.0',  'z': '0.05', 'R': '0.0', 'P': '0.0', 'Y': '0.0'},
        # {'namespace': 'tb4', 'name': 'tb4_4', 'x': '2.5',  'y': '0.0',  'z': '0.05', 'R': '0.0', 'P': '0.0', 'Y': '0.0'},
        # Add more robots as needed
    ]

    # Declarations
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use Gazebo clock if true'
    )
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator', default_value='True', description='Whether to start Gazebo'
    )
    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='False', description='Run Gazebo client'
    )
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(sim_dir, 'worlds', 'mission.sdf'),
        description='World SDF to load',
    )
    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(desc_dir, 'urdf', 'standard', 'turtlebot4.urdf.xacro'),
        description='Path to TurtleBot4 URDF/xacro',
    )
    declare_rviz_first_cmd = DeclareLaunchArgument(
        'rviz_for_all_robots', default_value='False', description='Launch RViz for each robot'
    )

    # Start the simulation world once (no robot, no rviz at sim level)
    # We reuse simulation.launch.py to bring up Gazebo server/client only.
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'simulation.launch.py')),
        launch_arguments={
            'namespace': '',
            'use_rviz': 'False',            # no RViz at sim level
            'use_simulator': use_simulator,
            'use_sim_time': use_sim_time,
            'use_robot_state_pub': 'False', # no global robot_state_publisher
            'spawn_robot': 'False',          # do not spawn a robot here
            'headless': headless,
            'world': world,
        }.items(),
    )

    # For each robot config, include spawn_tb4.launch.py
    robot_includes = []
    for idx, cfg in enumerate(default_robot_configs):
        # Apply the RViz toggle to every robot
        use_rviz_this = rviz_for_all_robots

        robot_includes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'spawn_tb4.launch.py')),
                launch_arguments={
                    'namespace': cfg['namespace'],
                    'use_rviz': use_rviz_this,
                    'use_simulator': use_simulator,
                    'use_sim_time': use_sim_time,
                    'robot_name': cfg['name'],
                    'robot_sdf': robot_sdf,
                    'x_pose': cfg['x'],
                    'y_pose': cfg['y'],
                    'z_pose': cfg['z'],
                    'roll': cfg['R'],
                    'pitch': cfg['P'],
                    'yaw': cfg['Y'],
                }.items(),
            )
        )

    ld = LaunchDescription()

    # Args
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_rviz_first_cmd)

    # World once
    ld.add_action(start_world)

    # Robots
    for inc in robot_includes:
        ld.add_action(inc)

    return ld
