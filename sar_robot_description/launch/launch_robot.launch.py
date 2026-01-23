import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Get the package directory
    pkg_path = get_package_share_directory('sar_robot_description')
    
    # Process Xacro file
    xacro_file = os.path.join(pkg_path, 'urdf', 'sar_robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_config.toxml()},
            {'use_sim_time': True}
        ]
    )

    # Gazebo Sim (Using empty.sdf for maximum stability)
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

    # Entity Creation Node
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'sar_robot',
            '-z', '0.3'
        ],
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/model/sar_robot/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.GpuLaserScan',
            '/model/sar_robot/scan@sensor_msgs/msg/LaserScan@gz.msgs.GpuLaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        remappings=[
            ('/model/sar_robot/scan', '/scan'),
        ],
        parameters=[{'use_sim_time': True}]
    )

    # Delay bridge start to allow Gazebo to create topics/entities
    bridge = TimerAction(
        period=3.0,
        actions=[bridge_node]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge
    ])