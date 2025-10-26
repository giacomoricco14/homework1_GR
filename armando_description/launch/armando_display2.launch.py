# TO DO

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
     # Arguments
    declared_arguments = []
    
    # declared_arguments.append(DeclareLaunchArgument(
    #         'use_hardware', #name of the argument
    #         default_value='0',
    #         description="Select '0' to test the code in simulation; select '1' to test the code on the hardware.",
    #     )
    # )

    # use_hardware = LaunchConfiguration("use_hardware")

    declared_arguments.append(DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
        )
    )
    
  

    arm_description_path = os.path.join(
        get_package_share_directory('armando_description'))
    urdf_path = os.path.join(arm_description_path, "urdf", "arm.urdf")
    with open(urdf_path,'r') as infp:
        robot_desc = infp.read()
    robot_description_arm = {"robot_description": robot_desc}  

    rviz_config = os.path.join(arm_description_path, "config", "rviz", "standing.rviz")
    
    # Nodes
    nodes_to_start = []
  
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description_arm,
                    {"use_sim_time": True},
            ],
        remappings=[
            ('/joint_states', '/joint_command'),
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[robot_description_arm,
                    {"use_sim_time": True},
            ],
        # condition=UnlessCondition(PythonExpression([use_hardware, ' == 0']))
        condition=UnlessCondition(LaunchConfiguration('jsp_gui'))
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[robot_description_arm,
                    {"use_sim_time": True},
            ],
        # condition=IfCondition(PythonExpression([use_hardware, ' == 0']))
        condition=IfCondition(LaunchConfiguration('jsp_gui'))
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )
    

    nodes_to_start = [
        robot_state_publisher_node,  
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ]
        
    return LaunchDescription(declared_arguments + nodes_to_start) 