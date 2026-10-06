from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    for key, value in [('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11'), ('GAZEBO_MODEL_PATH', '/usr/share/gazebo-11/models'), ('GAZEBO_PLUGIN_PATH', '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins')]:
        current=os.environ.get(key, '')
        if value not in current.split(os.pathsep):
            os.environ[key]=value+os.pathsep+current
    os.environ.setdefault('OGRE_RESOURCE_PATH', '/usr/lib/x86_64-linux-gnu/OGRE-1.9.0')
    share=Path(get_package_share_directory('sandbox_autodrive'))
    gazebo=Path(get_package_share_directory('gazebo_ros'))/'launch'
    server=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo/'gzserver.launch.py')),launch_arguments={'world':str(share/'worlds/sandbox.world'),'verbose':'true'}.items())
    client=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo/'gzclient.launch.py')),condition=IfCondition(LaunchConfiguration('gui')))
    rviz=Node(package='rviz2',executable='rviz2',name='rviz2',arguments=['-d',str(share/'config/sandbox.rviz')],parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('rviz')))
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true',description='Show Gazebo GUI'),
        DeclareLaunchArgument('rviz',default_value='true',description='Show RViz route and vehicle'),
        DeclareLaunchArgument('stabilize_pose',default_value='true',description='Keep the small car stable on the elevated road'),
        DeclareLaunchArgument('slam',default_value='false',description='Map the board from the simulated laser scan'),
        server,client,
        Node(package='sandbox_autodrive',executable='traffic_signal',output='screen',parameters=[{'use_sim_time':True}]),
        Node(package='sandbox_autodrive',executable='route_follower',output='screen',parameters=[{'use_sim_time':True}]),
        Node(package='sandbox_autodrive',executable='pose_stabilizer',output='screen',parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('stabilize_pose'))),
        Node(package='tf2_ros',executable='static_transform_publisher',name='laser_tf',
             arguments=['0','0','0.10','0','0','0','chassis','laser_frame'],
             parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('slam'))),
        Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',
             parameters=[str(share/'config/slam.yaml'),{'use_sim_time':True}],
             output='screen',condition=IfCondition(LaunchConfiguration('slam'))),
        rviz])
