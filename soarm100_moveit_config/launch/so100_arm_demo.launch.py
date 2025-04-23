import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils.moveit_configs_builder import MoveItConfigsBuilder

def generate_launch_description():
    is_sim = LaunchConfiguration("is_sim")
    declare_is_sim_arg = DeclareLaunchArgument(name="is_sim", default_value="True")

    rviz_config = LaunchConfiguration("rviz_config")
    declare_rviz_config_arg = DeclareLaunchArgument(name="rviz_config", 
                                                    default_value=os.path.join(
                                                        get_package_share_directory("soarm100_moveit_config"),
                                                        "config", "moveit.rviz"))

    moveit_config = (
        MoveItConfigsBuilder(robot_name="SO_5DOF_ARM100",
                             package_name="soarm100_moveit_config")
                             .robot_description(file_path=os.path.join(get_package_share_directory("soarm100_description"), "urdf", "soarm100_5dof.urdf.xacro"))
                             .robot_description_semantic(file_path="config/soarm100_5dof.srdf")
                             .trajectory_execution(file_path="config/moveit_controllers.yaml")
                             .robot_description_kinematics(file_path="config/kinematics.yaml")
                             .joint_limits(file_path="config/joint_limits.yaml")
                             .to_moveit_configs())
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": is_sim, 
                                              "publish_robot_description_semantic": True}],
        arguments=["--ros-args", "--log-level", "info"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits]
    )

    return LaunchDescription([
        declare_is_sim_arg,
        declare_rviz_config_arg,
        move_group_node,
        rviz_node,
    ])