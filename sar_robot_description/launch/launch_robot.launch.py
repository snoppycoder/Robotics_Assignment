import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # -------------------------------
    # Robot Description (URDF/Xacro)
    # -------------------------------
    pkg_path = get_package_share_directory('sar_robot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'sar_robot.urdf.xacro')

    robot_description_config = xacro.process_file(xacro_file)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_config.toxml()},
            {'use_sim_time': True}
        ]
    )

    # -------------------------------
    # Ignition Gazebo
    # -------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # -------------------------------
    # Spawn Robot (IMPORTANT FIX)
    # -------------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'sar_robot',
            '-z', '0.3'   # <<< prevents ground collision on spawn
        ],
    )

    # -------------------------------
    # ROS ↔ Gazebo Bridge
    # -------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            # ROS -> Gazebo (teleop)
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',

            # Gazebo -> ROS (odometry)
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',

            # Clock
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'
        ],
    )

    # -------------------------------
    # Launch Everything
    # -------------------------------
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge
    ])
