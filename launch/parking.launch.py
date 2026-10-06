"""Point-to-point road drive with a legal one-way parking maneuver."""
from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    for key, value in [
        ('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11'),
        ('GAZEBO_MODEL_PATH', '/usr/share/gazebo-11/models'),
        ('GAZEBO_PLUGIN_PATH', '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'),
    ]:
        current = os.environ.get(key, '')
        if value not in current.split(os.pathsep):
            os.environ[key] = value + os.pathsep + current
    os.environ.setdefault('OGRE_RESOURCE_PATH',
                          '/usr/lib/x86_64-linux-gnu/OGRE-1.9.0')
    share = Path(get_package_share_directory('sandbox_autodrive'))
    gazebo = Path(get_package_share_directory('gazebo_ros')) / 'launch'
    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo / 'gzserver.launch.py')),
        launch_arguments={'world': str(share / 'worlds/sandbox.world'),
                          'verbose': 'true'}.items())
    client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo / 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')))
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', str(share / 'config/sandbox.rviz')],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('rviz')))
    driver = Node(package='sandbox_autodrive', executable='parking_driver',
                  output='screen', parameters=[{
                      'use_sim_time': True,
                      'bay_index': ParameterValue(LaunchConfiguration('bay_index'),
                                                  value_type=int),
                      'park_seconds': ParameterValue(LaunchConfiguration('park_seconds'),
                                                     value_type=float),
                  }])
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('stabilize_pose', default_value='true'),
        DeclareLaunchArgument('bay_index', default_value='5',
                              description='Supported parking bay index, 0 through 5'),
        DeclareLaunchArgument('park_seconds', default_value='-1.0',
                              description='Negative waits parked until /parking_depart'),
        server,
        client,
        Node(package='sandbox_autodrive', executable='traffic_signal',
             output='screen', parameters=[{'use_sim_time': True}]),
        driver,
        Node(package='sandbox_autodrive', executable='pose_stabilizer',
             output='screen', parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('stabilize_pose'))),
        rviz,
    ])
