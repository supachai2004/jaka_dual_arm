#!/usr/bin/env python3
"""
joint_state_bridge.py

Subscribe:
  /jaka_driver_1/joint_position  (sensor_msgs/JointState)  → left arm
  /jaka_driver_2/joint_position  (sensor_msgs/JointState)  → right arm

Publish:
  /joint_states  (sensor_msgs/JointState)
  joint names จะถูก remap:
    joint_1..6  →  left_joint_1..6   (จาก driver_1)
    joint_1..6  →  right_joint_1..6  (จาก driver_2)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ชื่อ joint ที่ driver publish มา
DRIVER_JOINT_NAMES = [
    'joint_1', 'joint_2', 'joint_3',
    'joint_4', 'joint_5', 'joint_6',
]

# ชื่อที่ MoveIt / URDF ใช้ (ต้องตรงกับ URDF และ SRDF)
LEFT_JOINT_NAMES  = [f'left_{j}'  for j in DRIVER_JOINT_NAMES]
RIGHT_JOINT_NAMES = [f'right_{j}' for j in DRIVER_JOINT_NAMES]
ALL_JOINT_NAMES   = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES


class JointStateBridge(Node):
    def __init__(self):
        super().__init__('jaka_joint_state_bridge')

        self._left_msg  = None
        self._right_msg = None

        # Publisher → /joint_states
        self._pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscribe driver_1 → left arm
        self.create_subscription(
            JointState,
            '/jaka_driver_1/joint_position',
            self._cb_left,
            10,
        )

        # Subscribe driver_2 → right arm
        self.create_subscription(
            JointState,
            '/jaka_driver_2/joint_position',
            self._cb_right,
            10,
        )

        self.get_logger().info(
            'JointStateBridge started\n'
            f'  left  joints: {LEFT_JOINT_NAMES}\n'
            f'  right joints: {RIGHT_JOINT_NAMES}'
        )

    # ------------------------------------------------------------------
    def _cb_left(self, msg: JointState):
        self._left_msg = msg
        self._try_publish()

    def _cb_right(self, msg: JointState):
        self._right_msg = msg
        self._try_publish()

    # ------------------------------------------------------------------
    def _try_publish(self):
        if self._left_msg is None or self._right_msg is None:
            return

        left_pos  = list(self._left_msg.position)
        right_pos = list(self._right_msg.position)

        if len(left_pos) != 6 or len(right_pos) != 6:
            self.get_logger().warn(
                f'Unexpected joint count — left:{len(left_pos)} right:{len(right_pos)}'
            )
            return

        left_vel  = list(self._left_msg.velocity)  if self._left_msg.velocity  else [0.0]*6
        right_vel = list(self._right_msg.velocity) if self._right_msg.velocity else [0.0]*6
        left_eff  = list(self._left_msg.effort)    if self._left_msg.effort    else [0.0]*6
        right_eff = list(self._right_msg.effort)   if self._right_msg.effort   else [0.0]*6

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = ALL_JOINT_NAMES
        js.position = left_pos  + right_pos
        js.velocity = left_vel  + right_vel
        js.effort   = left_eff  + right_eff

        self._pub.publish(js)

        self.get_logger().debug(
            f'Published /joint_states — '
            f'left: {[f"{v:.3f}" for v in left_pos]} | '
            f'right: {[f"{v:.3f}" for v in right_pos]}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
