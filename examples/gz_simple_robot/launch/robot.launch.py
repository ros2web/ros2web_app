import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchService
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_gz_simple_robot = get_package_share_directory('gz_simple_robot')

    gz_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_simple_robot, 'launch', 'world.launch.py'))
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',

        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/camera2@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/simple_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/simple_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ],
        parameters=[{'qos_overrides./model/simple_robot.subscriber.reliability': 'reliable'}],
        output='screen'
    )

    return LaunchDescription([
        gz_world,
        bridge,
    ])


def main():
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()


if __name__ == '__main__':
    main()
