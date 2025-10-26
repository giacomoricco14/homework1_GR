# armando_display.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    # percorso urdf
    pkg_path = get_package_share_directory('armando_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'arm.urdf')

    # Leggi il file URDF
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Percorso del pacchetto
    rviz_config_path = os.path.join(pkg_path, 'config_armando', 'standing.rviz')

    ## Robot params
    params = {'robot_description': robot_desc}

    # xacro_armando = os.path.join(pkg_path, "urdf", "arm.urdf.xacro")
    # params = {"robot_description": 
    #           Command(['xacro ', xacro_armando, ' j0:=2.0', ' j1:=0.2', ' j2:=0.0', ' j3:=0.0'])}

    # Arguments
    jsp_gui = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('jsp_gui'))
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp_gui'))
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    # Lista dei nodi da lanciare
    return LaunchDescription([
        jsp_gui,
        joint_state_publisher,
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz2,
    ])