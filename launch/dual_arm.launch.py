import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('jaka_dual_arm')

    # ------------------------------------------------------------------
    # Robot Description (URDF)
    # ------------------------------------------------------------------
    xacro_file = os.path.join(pkg, 'urdf', 'dual_arm.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # ------------------------------------------------------------------
    # SRDF
    # ------------------------------------------------------------------
    srdf_file = os.path.join(pkg, 'config', 'dual_arm.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------
    kinematics = {'robot_description_kinematics': {
        'left_arm': {
            'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
            'kinematics_solver_search_resolution': 0.005,
            'kinematics_solver_timeout': 0.005,
        },
        'right_arm': {
            'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
            'kinematics_solver_search_resolution': 0.005,
            'kinematics_solver_timeout': 0.005,
        },
    }}

    # ------------------------------------------------------------------
    # Trajectory Execution
    # ------------------------------------------------------------------
    trajectory_execution = {
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': {
            'controller_names': [
                'left_arm_controller',
                'right_arm_controller',
            ],
            'left_arm_controller': {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': [
                    'left_joint_1', 'left_joint_2', 'left_joint_3',
                    'left_joint_4', 'left_joint_5', 'left_joint_6',
                ],
            },
            'right_arm_controller': {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': [
                    'right_joint_1', 'right_joint_2', 'right_joint_3',
                    'right_joint_4', 'right_joint_5', 'right_joint_6',
                ],
            },
        },
    }

    # ------------------------------------------------------------------
    # Joint Limits
    # ------------------------------------------------------------------
    joint_limits_file = os.path.join(pkg, 'config', 'joint_limits.yaml')
    with open(joint_limits_file, 'r') as f:
        joint_limits_yaml = yaml.safe_load(f)
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}

    # ------------------------------------------------------------------
    # OMPL Planning
    # ------------------------------------------------------------------
    ompl_yaml_file = os.path.join(pkg, 'config', 'ompl_planning.yaml')
    with open(ompl_yaml_file, 'r') as f:
        ompl_yaml = yaml.safe_load(f)

    ompl_config = {
        'planning_plugins': ['ompl_interface/OMPLPlanner'],
        'request_adapters': [
            'default_planning_request_adapters/ResolveConstraintFrames',
            'default_planning_request_adapters/ValidateWorkspaceBounds',
            'default_planning_request_adapters/CheckStartStateBounds',
            'default_planning_request_adapters/CheckStartStateCollision',
        ],
        'response_adapters': [
            'default_planning_response_adapters/AddTimeOptimalParameterization',
            'default_planning_response_adapters/ValidateSolution',
            'default_planning_response_adapters/DisplayMotionPath',
        ],
    }
    for key, value in ompl_yaml.items():
        if key not in ('planning_plugin', 'planning_plugins',
                       'request_adapters', 'response_adapters'):
            ompl_config[key] = value

    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_config,
    }

    # ------------------------------------------------------------------
    # MoveIt move_group parameters
    # ------------------------------------------------------------------
    move_group_params = [
        robot_description,
        robot_description_semantic,
        kinematics,
        robot_description_planning,
        trajectory_execution,
        planning_pipelines,
        {'use_sim_time': False},
    ]

    # ------------------------------------------------------------------
    # Nodes
    # ------------------------------------------------------------------
    return LaunchDescription([

        # ── Joint State Bridge  (jaka topics → /joint_states) ──────
        # เป็น publisher เดียวของ /joint_states
        # ไม่มี ros2_control_node และ joint_state_broadcaster แล้ว
        # เพราะมันทำให้ค่า race กัน (0,0,0,0,0,0 สลับกับค่าจริง)
        Node(
            package='jaka_dual_arm',
            executable='joint_state_bridge',
            name='jaka_joint_state_bridge',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),

        # ── Jaka Action Server (MoveIt → jaka_driver service) ──────
        Node(
            package='jaka_dual_arm',
            executable='jaka_action_server',
            name='jaka_dual_arm_action_server',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),

        # ── MoveIt move_group ───────────────────────────────────────
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=move_group_params,
        ),

        # ── Robot State Publisher ───────────────────────────────────
        # อ่าน /joint_states จาก bridge แล้วคำนวณ TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),

        # ── RViz2 ───────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg, 'config', 'dual_arm.rviz')],
            parameters=move_group_params,
        ),
    ])
