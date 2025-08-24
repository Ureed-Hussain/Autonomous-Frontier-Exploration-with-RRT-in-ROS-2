from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    SetEnvironmentVariable,
    DeclareLaunchArgument
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    nav2_bringup = get_package_share_directory('nav2_bringup')
    slam_toolbox = get_package_share_directory('slam_toolbox')
    autonomous_pkg = get_package_share_directory('autonomous_explorer')

    # Nav2 and SLAM params
    nav2_params_path = os.path.join(nav2_bringup, 'params', 'nav2_params.yaml')
    slam_params_path = os.path.join(slam_toolbox, 'config', 'mapper_params_online_async.yaml')

    # RViz config
    rviz_config_path = os.path.join(nav2_bringup, 'rviz', 'nav2_default_view.rviz')

    # Launch arguments for robot spawn pose
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')

    declare_x_pose = DeclareLaunchArgument(
        'x_pose', default_value='3.33',
        description='Initial X position of TurtleBot3'
    )
    declare_y_pose = DeclareLaunchArgument(
        'y_pose', default_value='1.045',
        description='Initial Y position of TurtleBot3'
    )
    declare_yaw_pose = DeclareLaunchArgument(
        'yaw_pose', default_value='0.0',
        description='Initial Yaw orientation of TurtleBot3'
    )

    # Launch the custom world with TurtleBot3
    custom_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(autonomous_pkg, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw_pose': yaw_pose,
            'use_sim_time': 'true'
        }.items()
    )

    # Navigation2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_params_path
        }.items()
    )

    # SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': slam_params_path
        }.items()
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Explorer node
    explorer_node = Node(
        package='autonomous_explorer',
        executable='explorer_node',
        output='screen'
    )

    return LaunchDescription([
        # Declare robot spawn args
        declare_x_pose,
        declare_y_pose,
        declare_yaw_pose,

        # Set TurtleBot3 model
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),

        # Launch Gazebo + robot
        LogInfo(msg='[LAUNCH] Starting Gazebo with custom world and TurtleBot3...'),
        custom_world_launch,

        # Launch Nav2 after 10 sec
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg='[LAUNCH] Gazebo loaded. Starting Navigation2...'),
                nav2_launch
            ]
        ),

        # Launch SLAM after 15 sec
        TimerAction(
            period=15.0,
            actions=[
                LogInfo(msg='[LAUNCH] Nav2 started. Launching SLAM Toolbox...'),
                slam_launch
            ]
        ),

        # Launch RViz after 20 sec
        TimerAction(
            period=20.0,
            actions=[
                LogInfo(msg='[LAUNCH] SLAM started. Launching RViz2...'),
                rviz_node
            ]
        ),

        # Launch Explorer node after 25 sec
        TimerAction(
            period=25.0,
            actions=[
                LogInfo(msg='[LAUNCH] RViz2 started. Launching Explorer Node...'),
                explorer_node
            ]
        )
    ])
