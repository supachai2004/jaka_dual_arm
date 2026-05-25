"""
Cooperative dual-arm manipulation coordinator.

Both flanges grip a rigid virtual object at fixed offsets. Given a target
object pose (xyz + rpy via PoseStamped), the node computes IK for each arm
and executes synchronized motion so both arms move together.

Architecture
------------
The ROS2 spin loop runs in a background thread (processes service/action
callbacks and resolves futures).  The main thread does all blocking work —
init, IK, planning — using threading.Event to wait on each future.

Usage (with dual_arm_moveit.launch.py already running):
  ros2 run jaka_dual_arm coordinator

Then send object targets in world frame (absolute pose of object centre):

  # Translate +5 cm in X from current object position
  ros2 topic pub --once /object_target geometry_msgs/msg/PoseStamped \\
    "{header: {frame_id: world}, pose: {position: {x: 0.05, y: 0.0, z: 0.0}, \\
     orientation: {w: 1.0}}}"

  # Translate +5 cm in Z, tilt 30° around X (in current object position)
  ros2 topic pub --once /object_target geometry_msgs/msg/PoseStamped \\
    "{header: {frame_id: world}, pose: {position: {x: 0.0, y: 0.0, z: 0.05}, \\
     orientation: {x: 0.259, y: 0.0, z: 0.0, w: 0.966}}}"
"""

import queue
import socket
import threading

import numpy as np
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    PlanningScene,
)
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from visualization_msgs.msg import Marker


# ── transform helpers (scipy quaternions are [x, y, z, w] — same as ROS) ────

def compose(p1, q1, p2, q2):
    """T1 × T2  →  (pos, quat)"""
    r1 = Rot.from_quat(q1)
    return p1 + r1.apply(p2), (r1 * Rot.from_quat(q2)).as_quat()


def invert_tf(p, q):
    """Invert rigid-body transform (p, q)."""
    ri = Rot.from_quat(q).inv()
    return ri.apply(-p), ri.as_quat()


def pose_to_pq(pose: Pose):
    p = np.array([pose.position.x, pose.position.y, pose.position.z])
    q = np.array([pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w])
    return p, q


def pq_to_pose(p, q) -> Pose:
    pose = Pose()
    pose.position.x  = float(p[0])
    pose.position.y  = float(p[1])
    pose.position.z  = float(p[2])
    pose.orientation.x = float(q[0])
    pose.orientation.y = float(q[1])
    pose.orientation.z = float(q[2])
    pose.orientation.w = float(q[3])
    return pose


# ── TCP Interface ─────────────────────────────────────────────────────────────

class TCPInterface:
    """
    TCP server (port 9090) that pushes EE poses to robot clients and
    synchronises motion via the ready/go/done handshake.

    Connection types (determined by first line received):
      robot:left   or   robot:right   — robot client
      lift:x,y,z:rx,ry,rz            — command client (degrees)

    Heartbeat: server sends "ping\\n" every 100 ms; robot must reply
    "pong\\n" within 500 ms or emergency-stop is triggered.
    """

    HEARTBEAT_INTERVAL = 0.10   # s between pings
    HEARTBEAT_TIMEOUT  = 0.50   # s to wait for pong
    READY_TIMEOUT      = 10.0   # s
    DONE_TIMEOUT       = 30.0   # s

    def __init__(self, node, port: int = 9090):
        self._node = node
        self._port = port
        self._robots: dict = {}   # 'left'/'right' → entry dict
        self._lock = threading.Lock()

    def start(self):
        threading.Thread(target=self._server_loop, daemon=True).start()
        self._node.get_logger().info(f'TCP interface starting on :{self._port}')

    # ── server ────────────────────────────────────────────────────────────────

    def _server_loop(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(('', self._port))
            srv.listen(10)
            srv.settimeout(1.0)
            self._node.get_logger().info(f'TCP server ready on :{self._port}')
            while True:
                try:
                    sock, addr = srv.accept()
                except socket.timeout:
                    continue
                threading.Thread(
                    target=self._dispatch, args=(sock, addr), daemon=True
                ).start()

    # ── dispatch ──────────────────────────────────────────────────────────────

    def _dispatch(self, sock: socket.socket, addr):
        line = self._readline(sock, timeout=5.0)
        if not line:
            sock.close()
            return
        if line.startswith('robot:'):
            name = line[6:]
            if name in ('left', 'right'):
                self._handle_robot(name, sock, addr)
            else:
                self._node.get_logger().warn(f'TCP: unknown robot name {name!r}')
                sock.close()
        elif line.startswith('lift:'):
            self._handle_command(sock, line)
        else:
            self._node.get_logger().warn(f'TCP: unknown first message {line!r}')
            sock.close()

    # ── robot connection ──────────────────────────────────────────────────────

    def _handle_robot(self, name: str, sock: socket.socket, addr):
        entry = {
            'sock':       sock,
            'pong_event': threading.Event(),
            'msg_queue':  queue.Queue(),
            'alive':      True,
        }
        with self._lock:
            old = self._robots.get(name)
            if old:
                old['alive'] = False
                try:
                    old['sock'].close()
                except Exception:
                    pass
            self._robots[name] = entry
        self._node.get_logger().info(f'Robot [{name}] connected from {addr}')

        recv_t = threading.Thread(
            target=self._robot_recv, args=(name, entry), daemon=True)
        hb_t = threading.Thread(
            target=self._robot_heartbeat, args=(name, entry), daemon=True)
        recv_t.start()
        hb_t.start()
        recv_t.join()   # block until socket closed

        entry['alive'] = False
        with self._lock:
            if self._robots.get(name) is entry:
                del self._robots[name]
        try:
            sock.close()
        except Exception:
            pass
        self._node.get_logger().info(f'Robot [{name}] disconnected')

    def _robot_recv(self, name: str, entry: dict):
        sock = entry['sock']
        while entry['alive']:
            line = self._readline(sock, timeout=1.0)
            if line is None:
                continue
            if line == 'pong':
                entry['pong_event'].set()
            else:
                entry['msg_queue'].put(line)

    def _robot_heartbeat(self, name: str, entry: dict):
        while entry['alive']:
            entry['pong_event'].clear()
            if not self._send(entry['sock'], 'ping'):
                break
            if not entry['pong_event'].wait(timeout=self.HEARTBEAT_TIMEOUT):
                self._node.get_logger().error(
                    f'Heartbeat timeout — robot [{name}]')
                self._emergency_stop(f'heartbeat timeout ({name})')
                entry['alive'] = False
                break
            threading.Event().wait(self.HEARTBEAT_INTERVAL)

    # ── command handler ───────────────────────────────────────────────────────

    def _handle_command(self, sock: socket.socket, command: str):
        # Parse "lift:x,y,z:rx_deg,ry_deg,rz_deg"
        try:
            _, pos_str, rpy_str = command.split(':')
            x, y, z   = [float(v) for v in pos_str.split(',')]
            rx, ry, rz = [float(v) for v in rpy_str.split(',')]
        except (ValueError, IndexError) as exc:
            self._node.get_logger().error(f'TCP: bad command {command!r}: {exc}')
            self._send(sock, 'error')
            sock.close()
            return

        if self._node._T_obj_L is None:
            self._node.get_logger().error('TCP: coordinator not initialised yet')
            self._send(sock, 'error')
            sock.close()
            return

        new_p = np.array([x, y, z])
        new_q = Rot.from_euler('xyz', [rx, ry, rz], degrees=True).as_quat()
        new_q = Rot.from_quat(new_q).inv().as_quat()   # sign-convention fix

        (lp, lq), (rp, rq) = self._node._compute_ee_poses(new_p, new_q)

        with self._lock:
            robots = dict(self._robots)

        if 'left' not in robots or 'right' not in robots:
            self._node.get_logger().error('TCP: both robots must be connected')
            self._send(sock, 'error')
            sock.close()
            return

        left_e  = robots['left']
        right_e = robots['right']

        def fmt_pose(p, q):
            return (f'pose:{p[0]:.6f},{p[1]:.6f},{p[2]:.6f}'
                    f':{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f}')

        if not self._send(left_e['sock'],  fmt_pose(lp, lq)):
            self._send(sock, 'error'); sock.close(); return
        if not self._send(right_e['sock'], fmt_pose(rp, rq)):
            self._send(sock, 'error'); sock.close(); return

        self._node.get_logger().info('TCP: poses sent — waiting for ready')

        # Wait for ready from both (concurrent)
        l_res, r_res = [None], [None]

        def _wait(entry, result, timeout):
            try:
                result[0] = entry['msg_queue'].get(timeout=timeout)
            except queue.Empty:
                result[0] = None

        lt = threading.Thread(target=_wait, args=(left_e,  l_res, self.READY_TIMEOUT))
        rt = threading.Thread(target=_wait, args=(right_e, r_res, self.READY_TIMEOUT))
        lt.start(); rt.start()
        lt.join();  rt.join()

        if l_res[0] != 'ready' or r_res[0] != 'ready':
            msg = f'ready timeout — left={l_res[0]!r} right={r_res[0]!r}'
            self._node.get_logger().error(f'TCP: {msg}')
            self._emergency_stop(msg)
            self._send(sock, 'error')
            sock.close()
            return

        self._node.get_logger().info('TCP: both ready — sending GO')

        # GO — send simultaneously
        self._send(left_e['sock'],  'go')
        self._send(right_e['sock'], 'go')

        # Wait for done from both (concurrent)
        l_res2, r_res2 = [None], [None]
        lt2 = threading.Thread(target=_wait, args=(left_e,  l_res2, self.DONE_TIMEOUT))
        rt2 = threading.Thread(target=_wait, args=(right_e, r_res2, self.DONE_TIMEOUT))
        lt2.start(); rt2.start()
        lt2.join();  rt2.join()

        if l_res2[0] == 'done' and r_res2[0] == 'done':
            self._node.get_logger().info('TCP: motion done')
            self._send(sock, 'done')
        else:
            msg = f'done timeout — left={l_res2[0]!r} right={r_res2[0]!r}'
            self._node.get_logger().error(f'TCP: {msg}')
            self._emergency_stop(msg)
            self._send(sock, 'error')
        sock.close()

    # ── emergency stop ────────────────────────────────────────────────────────

    def _emergency_stop(self, reason: str):
        self._node.get_logger().error(f'EMERGENCY STOP: {reason}')
        with self._lock:
            robots = dict(self._robots)
        for name, entry in robots.items():
            self._send(entry['sock'], 'stop')

    # ── socket helpers ────────────────────────────────────────────────────────

    @staticmethod
    def _readline(sock: socket.socket, timeout=None) -> str:
        sock.settimeout(timeout)
        buf = b''
        try:
            while True:
                b = sock.recv(1)
                if not b:
                    return None
                buf += b
                if buf.endswith(b'\n'):
                    return buf.decode().strip()
        except (socket.timeout, OSError):
            return None

    @staticmethod
    def _send(sock: socket.socket, msg: str) -> bool:
        try:
            sock.sendall((msg + '\n').encode())
            return True
        except OSError:
            return False


# ── node ─────────────────────────────────────────────────────────────────────

class CoopCoordinator(Node):

    LEFT_JOINTS = [
        'left_joint_1', 'left_joint_2', 'left_joint_3',
        'left_joint_4', 'left_joint_5', 'left_joint_6',
    ]
    RIGHT_JOINTS = [
        'right_joint_1', 'right_joint_2', 'right_joint_3',
        'right_joint_4', 'right_joint_5', 'right_joint_6',
    ]

    # Initial joint configuration (radians). Arms move here on startup so FK
    # can determine the object frame.  Both flanges end up at (0.40, ±0.30, 1.00)
    # in world frame — 0.60 m grip span; verified collision-free and full 6-DOF
    # workspace available from this pose.

    INIT_LEFT  = [-0.349, -2.618, -0.873,  1.571,  3.142,  1.571]
    INIT_RIGHT = [ 0.349, -0.524,  0.873, -1.571,  0.0,   -1.571]

    VEL = 0.3
    ACC = 0.3

    def __init__(self):
        super().__init__('coop_coordinator')

        self._mc     = ActionClient(self, MoveGroup, '/move_action')
        self._fk     = self.create_client(GetPositionFK, '/compute_fk')
        self._ik     = self.create_client(GetPositionIK, '/compute_ik')
        self._pub    = self.create_publisher(PlanningScene, '/planning_scene', 1)
        self._marker = self.create_publisher(Marker, '/payload_marker', 1)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._T_obj_L = None   # (pos, quat) of left_flange in object frame
        self._T_obj_R = None
        self._obj_pose = None  # current object pose in world (pos, quat)

        self._cur_L = list(self.INIT_LEFT)
        self._cur_R = list(self.INIT_RIGHT)

        # Lightweight callback — just queues the message for the main thread
        self._target_queue: list[PoseStamped] = []
        self._target_event = threading.Event()
        self.create_subscription(PoseStamped, '/object_target', self._queue_target, 10)

    # ── future helpers ───────────────────────────────────────────────────────

    def _wait(self, future, timeout=30.0):
        """Block until *future* resolves (spin loop is in a background thread)."""
        ev = threading.Event()
        future.add_done_callback(lambda _: ev.set())
        if not ev.wait(timeout=timeout):
            return None   # caller logs the error
        return future.result()

    # ── main control loop (called from main thread) ──────────────────────────

    def run(self):
        self.get_logger().info('Waiting for MoveIt services...')
        self._mc.wait_for_server()
        self._fk.wait_for_service()
        self._ik.wait_for_service()
        self.get_logger().info('Services ready — initialising grasp configuration.')

        self._setup_grasp()

        TCPInterface(self).start()

        self.get_logger().info(
            'Ready!  Publish PoseStamped (frame_id=world) to /object_target to move the object.')

        while rclpy.ok():
            if self._target_event.wait(timeout=1.0):
                self._target_event.clear()
                while self._target_queue:
                    msg = self._target_queue.pop(0)
                    self._handle_target(msg)

    def _queue_target(self, msg: PoseStamped):
        """Subscription callback — runs in spin thread, just enqueues."""
        self._target_queue.append(msg)
        self._target_event.set()

    def _compute_ee_poses(self, new_p, new_q):
        """Return ((lp, lq), (rp, rq)) — EE target poses without IK."""
        tL_p, _ = compose(new_p, new_q, *self._T_obj_L)
        tR_p, _ = compose(new_p, new_q, *self._T_obj_R)
        obj_rot  = Rot.from_quat(new_q)
        tL_q = (obj_rot * Rot.from_quat(self._lq_init)).as_quat()
        tR_q = (obj_rot * Rot.from_quat(self._rq_init)).as_quat()
        return (tL_p, tL_q), (tR_p, tR_q)

    # ── grasp initialisation ──────────────────────────────────────────────────

    def _setup_grasp(self):
        # Remove any stale payload box left by a previous coordinator run
        self._remove_scene_box()
        import time; time.sleep(0.3)  # give planning scene time to update

        self.get_logger().info('Moving to init joint configuration...')
        if not self._exec({'left': self.INIT_LEFT, 'right': self.INIT_RIGHT}):
            self.get_logger().error(
                'Cannot reach init config — check INIT_LEFT / INIT_RIGHT.')
            return

        lp, lq = self._call_fk('left_arm',  'left_flange',
                                self.LEFT_JOINTS,  self.INIT_LEFT)
        rp, rq = self._call_fk('right_arm', 'right_flange',
                                self.RIGHT_JOINTS, self.INIT_RIGHT)
        if lp is None or rp is None:
            self.get_logger().error('FK failed — cannot compute object frame.')
            return

        self.get_logger().info(
            f'Left  flange @ world: ({lp[0]:.4f}, {lp[1]:.4f}, {lp[2]:.4f})'
            f'  quat=({lq[0]:.3f},{lq[1]:.3f},{lq[2]:.3f},{lq[3]:.3f})')
        self.get_logger().info(
            f'Right flange @ world: ({rp[0]:.4f}, {rp[1]:.4f}, {rp[2]:.4f})'
            f'  quat=({rq[0]:.3f},{rq[1]:.3f},{rq[2]:.3f},{rq[3]:.3f})')

        obj_p = (lp + rp) / 2.0
        obj_q = np.array([0.0, 0.0, 0.0, 1.0])
        self._obj_pose = (obj_p, obj_q)

        inv_p, inv_q = invert_tf(obj_p, obj_q)
        identity_q = np.array([0.0, 0.0, 0.0, 1.0])
        self._T_obj_L = compose(inv_p, inv_q, lp, identity_q)
        self._T_obj_R = compose(inv_p, inv_q, rp, identity_q)
        self._lq_init = lq.copy()
        self._rq_init = rq.copy()

        grip_dist = float(np.linalg.norm(lp - rp))
        self.get_logger().info(
            f'Object centre @ world: ({obj_p[0]:.4f}, {obj_p[1]:.4f}, {obj_p[2]:.4f})')
        self.get_logger().info(f'Grip span: {grip_dist:.4f} m')
        self.get_logger().info(
            f'Left  offset in obj frame: {np.round(self._T_obj_L[0], 4)}')
        self.get_logger().info(
            f'Right offset in obj frame: {np.round(self._T_obj_R[0], 4)}')

        self._add_collision_box((obj_p, obj_q))
        self._update_scene_box()

    # ── object target handler ─────────────────────────────────────────────────

    def _handle_target(self, msg: PoseStamped):
        if self._T_obj_L is None:
            self.get_logger().warn('Not initialised yet — ignoring target.')
            return

        new_p, new_q = pose_to_pq(msg.pose)
        new_q = Rot.from_quat(new_q).inv().as_quat()
        rpy = Rot.from_quat(new_q).as_euler('xyz', degrees=True)
        self.get_logger().info(
            f'>>> Target object: pos=({new_p[0]:.4f},{new_p[1]:.4f},{new_p[2]:.4f})'
            f'  rpy_deg=({rpy[0]:.1f},{rpy[1]:.1f},{rpy[2]:.1f})')

        tL_p, _ = compose(new_p, new_q, *self._T_obj_L)
        tR_p, _ = compose(new_p, new_q, *self._T_obj_R)
        obj_rot = Rot.from_quat(new_q)
        tL_q = (obj_rot * Rot.from_quat(self._lq_init)).as_quat()
        tR_q = (obj_rot * Rot.from_quat(self._rq_init)).as_quat()

        self.get_logger().info(
            f'    Left  flange target: ({tL_p[0]:.4f},{tL_p[1]:.4f},{tL_p[2]:.4f})')
        self.get_logger().info(
            f'    Right flange target: ({tR_p[0]:.4f},{tR_p[1]:.4f},{tR_p[2]:.4f})')

        _zeros = [0.0] * 6
        lj = self._call_ik('left_arm',  'left_flange',  tL_p, tL_q, self._cur_L)
        if lj is None:
            lj = self._call_ik('left_arm', 'left_flange', tL_p, tL_q, self.INIT_LEFT)
        if lj is None:
            lj = self._call_ik('left_arm', 'left_flange', tL_p, tL_q, _zeros)
        rj = self._call_ik('right_arm', 'right_flange', tR_p, tR_q, self._cur_R)
        if rj is None:
            rj = self._call_ik('right_arm', 'right_flange', tR_p, tR_q, self.INIT_RIGHT)
        if rj is None:
            rj = self._call_ik('right_arm', 'right_flange', tR_p, tR_q, _zeros)

        if lj is None:
            self.get_logger().error('IK failed for left arm — pose may be unreachable.')
            return
        if rj is None:
            self.get_logger().error('IK failed for right arm — pose may be unreachable.')
            return

        self.get_logger().info(f'    Left  IK: {np.round(lj, 4)}')
        self.get_logger().info(f'    Right IK: {np.round(rj, 4)}')

        if self._exec({'left': lj, 'right': rj}):
            self._cur_L = list(lj)
            self._cur_R = list(rj)
            self._obj_pose = (new_p, new_q)
            self._update_collision_box((new_p, new_q))
            self._update_scene_box()
            self.get_logger().info('Motion SUCCEEDED')
        else:
            self.get_logger().error('Motion FAILED (planning or execution error)')

    # ── MoveIt helpers ────────────────────────────────────────────────────────

    def _exec(self, joints: dict) -> bool:
        goal = MoveGroup.Goal()
        goal.request.group_name = 'both_arms'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = self.VEL
        goal.request.max_acceleration_scaling_factor = self.ACC

        c = Constraints()
        for name, pos in zip(self.LEFT_JOINTS + self.RIGHT_JOINTS,
                              list(joints['left']) + list(joints['right'])):
            jc = JointConstraint()
            jc.joint_name      = name
            jc.position        = float(pos)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight          = 1.0
            c.joint_constraints.append(jc)
        goal.request.goal_constraints.append(c)

        f = self._mc.send_goal_async(goal)
        self.get_logger().info('  [exec] waiting for goal handle...')
        gh = self._wait(f, timeout=15.0)
        if not gh:
            self.get_logger().error('  [exec] goal handle future never resolved')
            return False
        if not gh.accepted:
            self.get_logger().error('  [exec] goal rejected by move_group')
            return False
        self.get_logger().info('  [exec] goal accepted, waiting for result...')
        rf = gh.get_result_async()
        result = self._wait(rf, timeout=30.0)
        if not result:
            self.get_logger().error('  [exec] result future never resolved')
            return False
        code = result.result.error_code.val
        if code != 1:
            self.get_logger().error(f'  [exec] motion error_code={code}')
            return False
        return True

    def _call_fk(self, group: str, link: str, joint_names: list, positions: list):
        req = GetPositionFK.Request()
        req.header.frame_id = 'world'
        req.fk_link_names   = [link]
        js = JointState()
        js.name     = joint_names
        js.position = [float(p) for p in positions]
        req.robot_state.joint_state = js

        resp = self._wait(self._fk.call_async(req))
        if not resp or not resp.pose_stamped:
            return None, None
        return pose_to_pq(resp.pose_stamped[0].pose)

    def _call_ik(self, group: str, link: str, pos, quat, seed=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name       = group
        req.ik_request.ik_link_name     = link
        req.ik_request.avoid_collisions = False  # MoveGroup planner handles collision for combined 12-joint state
        req.ik_request.timeout.sec      = 5

        ps = PoseStamped()
        ps.header.frame_id = 'world'
        ps.pose = pq_to_pose(pos, quat)
        req.ik_request.pose_stamped = ps

        if seed is not None:
            js = JointState()
            js.name     = self.LEFT_JOINTS if group == 'left_arm' else self.RIGHT_JOINTS
            js.position = [float(p) for p in seed]
            req.ik_request.robot_state.joint_state = js

        resp = self._wait(self._ik.call_async(req))
        if not resp or resp.error_code.val != 1:
            return None

        js    = resp.solution.joint_state
        names = self.LEFT_JOINTS if group == 'left_arm' else self.RIGHT_JOINTS
        try:
            return np.array([js.position[list(js.name).index(n)] for n in names])
        except (ValueError, IndexError):
            return None

    # ── visualisation ─────────────────────────────────────────────────────────

    def _add_collision_box(self, pose, size=(0.54, 0.10, 0.10)):
        """Add a box collision object to the MoveIt planning scene."""
        p, q = pose
        co = CollisionObject()
        co.id = 'payload'
        co.header.frame_id = 'world'
        co.header.stamp = self.get_clock().now().to_msg()
        co.operation = CollisionObject.ADD
        co.pose = pq_to_pose(p, q)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size)
        co.primitives.append(box)
        identity = Pose()
        identity.orientation.w = 1.0
        co.primitive_poses.append(identity)
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        self._pub.publish(scene)
        self.get_logger().info(
            f'Collision box added: size={size}  '
            f'pos=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f})')

    def _update_collision_box(self, new_pose):
        """Move the existing payload collision box to a new pose."""
        p, q = new_pose
        co = CollisionObject()
        co.id = 'payload'
        co.header.frame_id = 'world'
        co.header.stamp = self.get_clock().now().to_msg()
        co.operation = CollisionObject.MOVE
        co.pose = pq_to_pose(p, q)
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        self._pub.publish(scene)

    def _remove_collision_box(self):
        """Remove the payload collision object from the planning scene."""
        co = CollisionObject()
        co.id = 'payload'
        co.header.frame_id = 'world'
        co.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        self._pub.publish(scene)

    def _remove_scene_box(self):
        """Remove any stale collision payload left by a previous session."""
        self._remove_collision_box()

    def _update_scene_box(self):
        """Publish a visual-only Marker (no collision) showing the held object."""
        if self._obj_pose is None or self._T_obj_L is None:
            return
        obj_p, obj_q = self._obj_pose
        grip_span = float(np.linalg.norm(
            self._T_obj_L[0] - self._T_obj_R[0]))

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'object_center'
        t.transform.translation.x = float(obj_p[0])
        t.transform.translation.y = float(obj_p[1])
        t.transform.translation.z = float(obj_p[2])
        t.transform.rotation.x = float(obj_q[0])
        t.transform.rotation.y = float(obj_q[1])
        t.transform.rotation.z = float(obj_q[2])
        t.transform.rotation.w = float(obj_q[3])
        self._tf_broadcaster.sendTransform(t)

        m = Marker()
        m.header.frame_id = 'world'
        m.ns   = 'payload'
        m.id   = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose = pq_to_pose(obj_p, obj_q)
        m.scale.x = 0.08
        m.scale.y = max(grip_span, 0.10)
        m.scale.z = 0.08
        m.color.r = 0.2
        m.color.g = 0.6
        m.color.b = 1.0
        m.color.a = 0.6
        self._marker.publish(m)


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = CoopCoordinator()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
