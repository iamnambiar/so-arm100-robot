import os
from launch import LaunchDescription
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    description_dir = get_package_share_directory('soarm100_description')

    model_name = LaunchConfiguration("model_name")
    is_sim = LaunchConfiguration("is_sim")

    declare_model_arg = DeclareLaunchArgument(
        name="model_name",
        default_value="soarm100_5dof.urdf.xacro",
        description="Name of the model in urdf.xacro format",
        choices=["soarm100_5dof.urdf.xacro"]
    )
    declare_is_sim_arg = DeclareLaunchArgument(
        name="is_sim",
        default_value="True",
        description=""
    )

    robot_description = ParameterValue(
        Command(
            ['xacro ', 
             PathJoinSubstitution([description_dir, 
                                   "urdf", 
                                   model_name])]), 
             value_type=str)
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        condition=UnlessCondition(is_sim),
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": is_sim}]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": is_sim},
            os.path.join(
                get_package_share_directory("soarm100_control"),
                "config",
                "soarm100_controllers.yaml"
            )
        ],
        condition=UnlessCondition(is_sim)
    )
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    return LaunchDescription([
        declare_model_arg,
        declare_is_sim_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
    ])