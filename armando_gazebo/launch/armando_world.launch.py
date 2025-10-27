# Software License Agreement (BSD License 2.0)
#
# Copyright (c) 2023, Metro Robots
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Metro Robots nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
import os
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():

 # Percorso del pacchetto
    pkg_path = get_package_share_directory('armando_gazebo')

    # Percorsi dei file
    # urdf_path = os.path.join(pkg_path, 'urdf', 'arm.urdf')
    
    #  # Leggi il file URDF
    # with open(urdf_path, 'r') as infp:
    #     robot_desc = infp.read()
        
    xacro_armando = PathJoinSubstitution([pkg_path, "urdf", "arm.urdf.xacro"])
    # Parametri del robot
    # params = {'robot_description': robot_desc}
    params = {'robot_description': Command(['xacro ', xacro_armando, ' j0:=2.0', ' j1:=0.2', ' j2:=0.0', ' j3:=0.0'])}
    
    # these are the arguments you can pass this launch file, for example paused:=true
    controller_arg = DeclareLaunchArgument(
        name='ctrl',
        description = '0 = pos controller, 1 = trajectory controller',
        default_value='0',
    )
    user = LaunchConfiguration("ctrl")
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
    )
    package_arg = DeclareLaunchArgument('urdf_package',
                                        description='The package where the robot description is located',
                                        default_value='armando_gazebo')
    model_arg = DeclareLaunchArgument('urdf_package_path',
                                      description='The path to the robot description relative to the package root',
                                      default_value='urdf/arm.urdf.xacro') # punto 2c


    empty_world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'pause': 'true',
            'gz_args': ['-r ', 'empty.sdf'],
        }.items(),
    )

    description_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_package_path': LaunchConfiguration('urdf_package_path')}.items()
    )

    # push robot_description to factory and spawn robot in gazebo
    urdf_spawner_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='urdf_spawner',
        arguments=['-topic', '/robot_description', '-entity', 'armando', '-z', '0.5', '-unpause'],
        output='screen',
    )

    # Punto 2c:
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )  

    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],  
        condition=IfCondition(PythonExpression([user, ' == 0']))
    ) 

    trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],  
        condition=IfCondition(PythonExpression([user, ' == 1']))
    ) 

    #Launch the ros2 controllers after the model spawns in Gazebo 
    delay_joint_traj_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_spawner_node, # come si accende questo nodo aspetta il timer e ...
            on_exit=[position_controller, trajectory_controller], #... in uscita apri questo nodo
        )
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=urdf_spawner_node,
                on_exit=[joint_state_broadcaster],
            )
    )

    bridge_camera = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "--ros-args",
            "-r", "/camera:=/videocamera"],
        output="screen"
    )

    return LaunchDescription([
        controller_arg,
        gui_arg,
        package_arg,
        model_arg,
        empty_world_launch,
        description_launch_py,
        urdf_spawner_node,
        delay_joint_traj_controller,
        delay_joint_state_broadcaster,
        bridge_camera
    ])
