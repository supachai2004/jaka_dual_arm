import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from geometry_msgs.msg import Pose
import math


class DualArmCoordinator(Node):
    def __init__(self):
        super().__init__('dual_arm_coordinator')
        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for move_group...')
        self.client.wait_for_server()

        # Grasp offset จาก object center (เมตร)
        # ปรับค่านี้ตามตำแหน่งจริงของ gripper
        self.left_offset_y  =  0.15
        self.right_offset_y = -0.15

        # Subscribe รับ object pose command
        self.sub = self.create_subscription(
            Pose,
            '/object_command',
            self.on_object_command,
            10
        )

        self.get_logger().info('Dual Arm Coordinator ready!')
        self.get_logger().info('Waiting for /object_command...')

    def on_object_command(self, msg):
        self.get_logger().info(
            f'Received command: x={msg.position.x:.3f} '
            f'y={msg.position.y:.3f} z={msg.position.z:.3f}'
        )
        self.move_to_home()

    def move_to_home(self):
        self.get_logger().info('Moving both arms to home...')

        goal = MoveGroup.Goal()
        goal.request.group_name = 'both_arms'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1

        joints = [
            'left_joint_1', 'left_joint_2', 'left_joint_3',
            'left_joint_4', 'left_joint_5', 'left_joint_6',
            'right_joint_1', 'right_joint_2', 'right_joint_3',
            'right_joint_4', 'right_joint_5', 'right_joint_6',
        ]

        constraints = Constraints()
        for joint in joints:
            jc = JointConstraint()
            jc.joint_name = joint
            jc.position = 0.0
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        code = result_future.result().result.error_code.val
        if code == 1:
            self.get_logger().info('SUCCESS!')
        else:
            self.get_logger().error(f'FAILED: error code {code}')


def main():
    rclpy.init()
    node = DualArmCoordinator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
