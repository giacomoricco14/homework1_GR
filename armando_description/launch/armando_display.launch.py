from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    # Percorso del pacchetto
    pkg_path = get_package_share_directory('armando_description')

    # Percorsi dei file
    # urdf_path = os.path.join(pkg_path, 'urdf', 'arm.urdf')
    rviz_config_path = os.path.join(pkg_path, 'config_armando','rviz_armando.rviz')

    # Leggi il file URDF
    #with open(urdf_path, 'r') as infp:
    #   robot_desc = infp.read()

    
    xacro_armando = os.path.join(pkg_path, "urdf", "arm.urdf.xacro")

    #params = {"robot_description": Command(['xacro', xacro_armando, 'j0_pos:=2.0', ' j1_pos:=0.2', 'j2_pos:=0.0', 'j3_pos:=0.0'])}
    
    params = {
        "robot_description": Command([
            "xacro ", xacro_armando,
            " j0_pos:=2.0 j1_pos:=0.2 j2_pos:=0.0 j3_pos:=0.0"
        ])
    }


    # Parametri del robot
    # params = {'robot_description': robot_desc}

    # Argomento per scegliere se usare la GUI o no
    jsp_gui = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )

    # Nodi ROS2
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

