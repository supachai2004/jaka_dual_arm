#!/usr/bin/env python3
"""
jaka_action_server.py
รับ FollowJointTrajectory จาก MoveIt → เรียก /jaka_driver_N/joint_move service
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from jaka_msgs.srv import Move

LEFT_JOINTS = [
    'left_joint_1', 'left_joint_2', 'left_joint_3',
    'left_joint_4', 'left_joint_5', 'left_joint_6',
]
RIGHT_JOINTS = [
    'right_joint_1', 'right_joint_2', 'right_joint_3',
    'right_joint_4', 'right_joint_5', 'right_joint_6',
]

DEFAULT_VELO = 0.5
DEFAULT_ACC  = 0.5


class DualArmActionServer(Node):
    def __init__(self):
        super().__init__('jaka_dual_arm_action_server')

        # ใช้ callback group เดียวกันทั้ง node
        self._cb_group = ReentrantCallbackGroup()

        # Service clients
        self._left_client = self.create_client(
            Move, '/jaka_driver_1/joint_move',
            callback_group=self._cb_group,
        )
        self._right_client = self.create_client(
            Move, '/jaka_driver_2/joint_move',
            callback_group=self._cb_group,
        )

        # Action servers — ลงทะเบียนบน node เดียวกัน
        self._left_action = ActionServer(
            self,
            FollowJointTrajectory,
            '/left_arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_left,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self._right_action = ActionServer(
            self,
            FollowJointTrajectory,
            '/right_arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_right,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        self.get_logger().info('DualArmActionServer ready')
        self.get_logger().info('  LEFT  → /jaka_driver_1/joint_move')
        self.get_logger().info('  RIGHT → /jaka_driver_2/joint_move')

    # ------------------------------------------------------------------
    def _goal_cb(self, goal_request):
        self.get_logger().info('Goal received')
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------
    async def _execute_left(self, goal_handle):
        return await self._execute(goal_handle, LEFT_JOINTS, self._left_client, 'left')

    async def _execute_right(self, goal_handle):
        return await self._execute(goal_handle, RIGHT_JOINTS, self._right_client, 'right')

    # ------------------------------------------------------------------
    async def _execute(self, goal_handle, joint_names, client, arm_name):
        trajectory  = goal_handle.request.trajectory
        points      = trajectory.points
        traj_joints = trajectory.joint_names

        self.get_logger().info(f'[{arm_name}] Executing {len(points)} waypoints')

        # map joint order
        try:
            idx_map = [traj_joints.index(j) for j in joint_names]
        except ValueError as e:
            self.get_logger().error(f'[{arm_name}] Joint name mismatch: {e}')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # รอ service พร้อมครั้งเดียวก่อน execute
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'[{arm_name}] joint_move service not available')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        for i, point in enumerate(points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            pose = [point.positions[j] for j in idx_map]

            if (len(point.velocities) >= 6 and any(v != 0.0 for v in point.velocities)):
                velo = max(max(abs(v) for v in point.velocities[:6]), 0.05)
            else:
                velo = DEFAULT_VELO

            req            = Move.Request()
            req.pose       = pose
            req.has_ref    = False
            req.ref_joint  = [0.0] * 6
            req.mvvelo     = velo
            req.mvacc      = DEFAULT_ACC
            req.mvtime     = 0.0
            req.mvradii    = 0.0
            req.coord_mode = 0
            req.index      = 0

            future = client.call_async(req)
            await future

            if future.result() is None:
                self.get_logger().error(f'[{arm_name}] call failed at waypoint {i}')
                goal_handle.abort()
                return FollowJointTrajectory.Result()

            # feedback
            fb = FollowJointTrajectory.Feedback()
            fb.header.stamp      = self.get_clock().now().to_msg()
            fb.joint_names       = joint_names
            fb.desired.positions = pose
            fb.actual.positions  = pose
            fb.error.positions   = [0.0] * 6
            goal_handle.publish_feedback(fb)

            self.get_logger().info(f'[{arm_name}] waypoint {i+1}/{len(points)} done')

        goal_handle.succeed()
        self.get_logger().info(f'[{arm_name}] Trajectory complete')
        return FollowJointTrajectory.Result()


def main(args=None):
    rclpy.init(args=args)
    node = DualArmActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
