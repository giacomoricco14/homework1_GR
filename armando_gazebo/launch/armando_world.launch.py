# armando world launcher

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package path
    pkg_path = get_package_share_directory('armando_description')
        
    # Armando xacro path
    xacro_armando = PathJoinSubstitution([pkg_path, "urdf", "armando.urdf.xacro"])
    # Robot description given by the xacro command
    params = {'robot_description': Command(['xacro ', xacro_armando, ' j0:=2.0', ' j1:=0.2', ' j2:=0.0', ' j3:=0.0'])}


    #### Arguments ####

    controller_arg = DeclareLaunchArgument(
        name='ctrl',
        description = 'Select: 0 for pos controller; 1 for trajectory controller',
        default_value='0',
    )
    user = LaunchConfiguration("ctrl")

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
    )

    package_arg = DeclareLaunchArgument('urdf_package',
                                        description='The package where the robot description is located',
                                        default_value='armando_description')
    
    model_arg = DeclareLaunchArgument('urdf_package_path',
                                        description='The path to the robot description relative to the package root',
                                        default_value='urdf/armando.urdf.xacro') # introduced from point 2c


    #### Nodes ####

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

    # Nodes introduced starting from point 2c:
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

    # Launch the ros2 controllers after the Robot construction in Gazebo 
    delay_joint_traj_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_spawner_node, # Once this node is active, wait
            on_exit=[position_controller, trajectory_controller], # On exit, start this nodes
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
            "-r", "/camera:=/videocamera"], # Remapping
        output="screen"
    )


    # List of Arguments and Nodes
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
