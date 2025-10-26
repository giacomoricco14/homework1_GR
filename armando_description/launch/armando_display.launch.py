# armando display launcher

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package path
    pkg_path = get_package_share_directory('armando_description')
    # Urdf path
    urdf_path = os.path.join(pkg_path, 'urdf', 'arm.urdf')
    # read the urdf and store the robot description
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Rviz config path
    rviz_config_path = os.path.join(pkg_path, 'config_armando', 'standing.rviz')

    # Params: containing the robot description
    params = {'robot_description': robot_desc}


    #### Arguments ####

    jsp_gui = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )


    #### Nodes ####

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


    # List of Arguments and Nodes
    return LaunchDescription([
        jsp_gui,
        joint_state_publisher,
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz2,
    ])