import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():


    suffix_arg = DeclareLaunchArgument(
        'suffix',
        default_value='v1',
        description='Suffix pour les articulations et liens'
    )
    suffix = LaunchConfiguration('suffix')

    # 2. Construction du robot_description AVEC le suffixe
    pkg_share = FindPackageShare(package='robot_arm_pkg').find('robot_arm_pkg')
    xacro_file_path = os.path.join(pkg_share, 'urdf', 'arm.urdf.xacro')

    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file_path, ' suffix:=', suffix]), 
        value_type=str
    )


    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content},
                    {'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_robot_arm',
                   '-z', '0.0'],
        output='screen'
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    load_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller"],
        output="screen",
    )

    custom_controller = Node(
        package="robot_arm_pkg",
        executable="controller_node",
        name='controller_node',
        output="screen",
        parameters=[{'use_sim_time':True}]
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        suffix_arg,
        node_robot_state_publisher,
        clock_bridge,
        gazebo,
        spawn_entity,
        load_joint_state_broadcaster,
        load_effort_controller,
        custom_controller

    ])
