import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg = get_package_share_directory('jaka_dual_arm')

    moveit_config = (
        MoveItConfigsBuilder("jaka_dual_arm", package_name="jaka_dual_arm")
        .robot_description(file_path=os.path.join(pkg, 'urdf', 'dual_arm.urdf.xacro'))
        .robot_description_semantic(file_path=os.path.join(pkg, 'config', 'dual_arm.srdf'))
        .robot_description_kinematics(file_path=os.path.join(pkg, 'config', 'kinematics.yaml'))
        .trajectory_execution(file_path=os.path.join(pkg, 'config', 'moveit_controllers.yaml'))
        .joint_limits(file_path=os.path.join(pkg, 'config', 'joint_limits.yaml'))
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package='jaka_dual_arm',
            executable='coordinator',
            output='screen',
            parameters=[moveit_config.to_dict()],
        ),
    ])
