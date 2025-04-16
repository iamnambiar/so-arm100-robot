import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model_name = LaunchConfiguration("model_name")
    declare_model_arg = DeclareLaunchArgument(
        name="model_name",
        default_value="soarm100_5dof.urdf.xacro",
        description="Name of the model in urdf.xacro format",
        choices=["soarm100_5dof.urdf.xacro"]
    )

    # robot_description = ParameterValue(Command(['xacro ', os.path.join(get_package_share_directory('soarm100_description'), "urdf", )]))
    robot_description = ParameterValue(Command(['xacro ', 
                                                PathJoinSubstitution([get_package_share_directory('soarm100_description'), "urdf", model_name])]))

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", os.path.join(get_package_share_directory('soarm100_description'), 'rviz', 'display.rviz')]
    )

    return LaunchDescription([
        declare_model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])