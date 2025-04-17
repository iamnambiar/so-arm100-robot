import os
from pathlib import Path
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    description_dir = get_package_share_directory('soarm100_description')

    model_name = LaunchConfiguration("model_name")
    declare_model_arg = DeclareLaunchArgument(
        name="model_name",
        default_value="soarm100_5dof.urdf.xacro",
        description="Name of the model in urdf.xacro format",
        choices=["soarm100_5dof.urdf.xacro"]
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(description_dir).parent.resolve())
        ]
    )

    robot_description = ParameterValue(
        Command(
            ['xacro ', 
             PathJoinSubstitution([description_dir, 
                                   "urdf", 
                                   model_name])]), 
             value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     "use_sim_time": True}]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                "launch"
            ), "/gz_sim.launch.py"]
        ),
        launch_arguments=[
            ('gz_args', [' -v 4 -r empty.sdf'])
        ]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'soarm100']
    )

    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )

    return LaunchDescription([
        declare_model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gz_sim,
        gz_spawn_entity,
        gz_ros2_bridge
    ])