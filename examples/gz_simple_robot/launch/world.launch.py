import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchService
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_gz_simple_robot = get_package_share_directory('gz_simple_robot')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'))
    )
    #
    declare_gz_args = DeclareLaunchArgument(
        'gz_args', default_value=['-r ', os.path.join(
            pkg_gz_simple_robot, 'worlds', 'simple-world.sdf')],
        description='Gazebo arguments')

    spawn_sdf = Node(package='ros_gz_sim', executable='create',
                     arguments=['-name', 'simple_robot',
                                '-x', '0.0',
                                '-y', '0.0',
                                '-z', '1.0',
                                '-file', os.path.join(pkg_gz_simple_robot, 'models', 'simple_robot.sdf')],
                     output='screen')

    return LaunchDescription([
        declare_use_sim,
        declare_gz_args,
        gz_sim,
        spawn_sdf
    ])


def main():
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()


if __name__ == '__main__':
    main()
