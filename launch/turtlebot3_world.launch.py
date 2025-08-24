import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Launch arguments (configurable from command line)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.01')
    yaw_pose = LaunchConfiguration('yaw_pose', default='0.0')

    # Declare arguments so user can override them in CLI or parent launch
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true'
    )
    declare_x_pose = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Initial X position of TurtleBot3'
    )
    declare_y_pose = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Initial Y position of TurtleBot3'
    )
    declare_z_pose = DeclareLaunchArgument(
        'z_pose', default_value='0.01',
        description='Initial Z position of TurtleBot3'
    )
    declare_yaw_pose = DeclareLaunchArgument(
        'yaw_pose', default_value='0.0',
        description='Initial Yaw orientation of TurtleBot3'
    )

    # Custom world
    world = os.path.join(
        os.getenv("HOME"), "autonomous_explorer", "src",
        "autonomous_explorer", "world", "box.world"
    )

    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot state publisher (provides TF from URDF)
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Spawn TurtleBot3 in Gazebo at given pose
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'yaw_pose': yaw_pose
        }.items()
    )

    # Build launch description
    ld = LaunchDescription()

    # Declare args
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_z_pose)
    ld.add_action(declare_yaw_pose)

    # Actions
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld
