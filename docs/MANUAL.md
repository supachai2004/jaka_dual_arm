# Operation Manual — JAKA Dual-Arm Cooperative Manipulation

**EN** | [TH ภาษาไทย](#คู่มือการใช้งาน--jaka-dual-arm-cooperative-manipulation)

---

## 1. System Prerequisites

Before starting, confirm:

- [ ] ROS2 Jazzy is sourced: `source /opt/ros/jazzy/setup.bash`
- [ ] Workspace is built: `colcon build` completed without errors
- [ ] Workspace is sourced: `source ~/ros2_ws/install/setup.bash`
- [ ] (Real robot) Both JAKA controllers are powered on and reachable by ping
- [ ] (Simulation) No real robot required — the mock controllers handle trajectory execution

---

## 2. Starting the System (Step by Step)

### Terminal 1 — MoveIt2 Stack

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch jaka_dual_arm dual_arm_moveit.launch.py
```

**What launches:**
- `ros2_control_node` — joint trajectory controller manager (100 Hz)
- `joint_state_broadcaster` — publishes `/joint_states`
- `left_arm_controller` — JointTrajectoryController for 6 left joints
- `right_arm_controller` — JointTrajectoryController for 6 right joints
- `move_group` — MoveIt2 OMPL planner + collision checking
- `robot_state_publisher` — publishes TF tree from URDF
- `rviz2` — visualisation with pre-configured display

**Expected output (no errors):**
```
[move_group]: move_group ready
[rviz2]: Stereo is NOT supported
```

Wait for RViz to open and show the robot model before proceeding.

### Terminals 2 & 3 — JAKA Hardware Drivers (real robots only)

> Skip this step for simulation. The coordinator auto-detects whether drivers are present.

Each JAKA robot runs an independent `jaka_driver` node. They share the same executable, so they **must be started under different namespaces** (`/left` and `/right`). This is what makes their service names unique.

```bash
# Terminal 2 — Left arm driver  (IP 192.168.0.2)
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run jaka_driver jaka_driver \
  --ros-args -r __ns:=/left -p ip:=192.168.0.2
```

```bash
# Terminal 3 — Right arm driver  (IP 192.168.0.1)
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run jaka_driver jaka_driver \
  --ros-args -r __ns:=/right -p ip:=192.168.0.1
```

After both drivers start, verify their services are visible:

```bash
ros2 service list | grep jaka
```

Expected output (at minimum):
```
/left/jaka_driver/joint_move
/right/jaka_driver/joint_move
```

If you do not see these two lines, do not start the coordinator. See [Troubleshooting 5.7](#57-robot-not-moving--jaka-driver-service-not-found).

### Terminal 4 (or 2 in simulation) — Coordinator Node

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run jaka_dual_arm coordinator
```

**Startup sequence — simulation mode (no drivers):**
```
[coordinator]: Waiting for MoveIt services...
[coordinator]: JAKA left driver not available — running simulation only
[coordinator]: JAKA right driver not available — running simulation only
[coordinator]: Services ready — initialising grasp configuration.
[coordinator]: Moving to init joint configuration...
[coordinator]:   [exec] waiting for goal handle...
[coordinator]:   [exec] goal accepted, waiting for result...
[coordinator]: Left  flange @ world: (0.4000,  0.3000, 1.0000)
[coordinator]: Right flange @ world: (0.4000, -0.3000, 1.0000)
[coordinator]: Object centre @ world: (0.4000, 0.0000, 1.0000)
[coordinator]: Grip span: 0.6000 m
[coordinator]: TCP server ready on :9090
[coordinator]: Ready!  Publish PoseStamped (frame_id=world) to /object_target
```

**Startup sequence — real robot mode (both drivers running):**
```
[coordinator]: Waiting for MoveIt services...
[coordinator]: JAKA left driver ready
[coordinator]: JAKA right driver ready
[coordinator]: Services ready — initialising grasp configuration.
[coordinator]: Moving to init joint configuration...
[coordinator]:   [exec] waiting for goal handle...
[coordinator]:   [exec] goal accepted, waiting for result...
[coordinator]: [jaka] both arms moved successfully
[coordinator]: Left  flange @ world: (0.4000,  0.3000, 1.0000)
[coordinator]: Right flange @ world: (0.4000, -0.3000, 1.0000)
[coordinator]: Object centre @ world: (0.4000, 0.0000, 1.0000)
[coordinator]: Grip span: 0.6000 m
[coordinator]: TCP server ready on :9090
[coordinator]: Ready!  Publish PoseStamped (frame_id=world) to /object_target
```

The system is now ready to receive commands.

---

## 3. Sending Commands via ROS2 Topic

The `/object_target` topic accepts `geometry_msgs/msg/PoseStamped`.

**Frame convention:**
- `frame_id` must be `world`
- Position = **absolute** target position of the object centre in world frame
- Orientation = target orientation as a quaternion

**Note on the sign convention:** The coordinator internally inverts the incoming quaternion (`.inv()`). This means you publish the orientation you want, and the code corrects for the JAKA flange frame convention automatically.

### 3.1 Translate only (keep current orientation)

```bash
# Move +5 cm in X
ros2 topic pub --once /object_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.45, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"

# Move +3 cm in Z (upward)
ros2 topic pub --once /object_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.40, y: 0.0, z: 1.03}, orientation: {w: 1.0}}}"
```

### 3.2 Translate and rotate

The easiest way to compute a quaternion from roll-pitch-yaw is to use Python:

```python
from scipy.spatial.transform import Rotation as Rot
import numpy as np

# 30° tilt around X axis
q = Rot.from_euler('xyz', [30, 0, 0], degrees=True).as_quat()
# q = [x, y, z, w]
print(q)
```

Then publish:
```bash
# Tilt 30° around X, position at (0.45, 0.0, 1.0)
ros2 topic pub --once /object_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.45, y: 0.0, z: 1.0}, \
   orientation: {x: 0.259, y: 0.0, z: 0.0, w: 0.966}}}"
```

### 3.3 Using a Python script

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as Rot

rclpy.init()
node = Node('cmd_sender')
pub = node.create_publisher(PoseStamped, '/object_target', 1)

msg = PoseStamped()
msg.header.frame_id = 'world'
msg.header.stamp = node.get_clock().now().to_msg()

# Target: x=0.45 m, z=1.05 m, rotated 15° around Z
msg.pose.position.x = 0.45
msg.pose.position.y = 0.0
msg.pose.position.z = 1.05
q = Rot.from_euler('xyz', [0, 0, 15], degrees=True).as_quat()
msg.pose.orientation.x = q[0]
msg.pose.orientation.y = q[1]
msg.pose.orientation.z = q[2]
msg.pose.orientation.w = q[3]

pub.publish(msg)
rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
```

---

## 4. TCP Command Format and Examples

Connect to the coordinator host on **TCP port 9090**.

### 4.1 Command format

```
lift:X,Y,Z:RX,RY,RZ\n
```

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| X | float | metres | Absolute target X in world frame |
| Y | float | metres | Absolute target Y in world frame |
| Z | float | metres | Absolute target Z in world frame |
| RX | float | degrees | Roll  (rotation around X) |
| RY | float | degrees | Pitch (rotation around Y) |
| RZ | float | degrees | Yaw   (rotation around Z) |

All values separated by commas. No spaces. Line must end with `\n`.

### 4.2 Response codes

| Response | Meaning |
|----------|---------|
| `done\n` | Motion completed successfully |
| `error\n` | Motion failed (IK failure, timeout, collision) |

### 4.3 Examples

```
# Move to (0.40, 0.00, 1.00), no rotation
lift:0.40,0.00,1.00:0.0,0.0,0.0

# Move to (0.45, 0.00, 1.05), rotate 15° around Z
lift:0.45,0.00,1.05:0.0,0.0,15.0

# Tilt 20° around X (tipping forward)
lift:0.40,0.00,1.00:20.0,0.0,0.0

# Rotate 45° around Z while lifting
lift:0.40,0.00,1.10:0.0,0.0,45.0
```

### 4.4 Python client example

```python
import socket
import time

def send_lift(host, x, y, z, rx=0.0, ry=0.0, rz=0.0, port=9090, timeout=45):
    cmd = f'lift:{x},{y},{z}:{rx},{ry},{rz}\n'
    with socket.create_connection((host, port), timeout=timeout) as s:
        s.sendall(cmd.encode())
        resp = b''
        while not resp.endswith(b'\n'):
            chunk = s.recv(16)
            if not chunk:
                break
            resp += chunk
    return resp.decode().strip()

# Usage
result = send_lift('127.0.0.1', 0.45, 0.0, 1.05, rz=15.0)
print(result)  # 'done' or 'error'
```

### 4.5 C++ client example (minimal)

```cpp
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>

int main() {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(9090);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    connect(fd, (sockaddr*)&addr, sizeof(addr));

    const char* cmd = "lift:0.45,0.0,1.05:0.0,0.0,15.0\n";
    send(fd, cmd, strlen(cmd), 0);

    char buf[32] = {};
    recv(fd, buf, sizeof(buf)-1, 0);
    printf("Response: %s\n", buf);
    close(fd);
}
```

---

## 5. Troubleshooting Guide

### 5.1 coordinator exits immediately

**Symptom:** Node starts and exits without "Ready!" message.

**Causes and fixes:**

| Symptom detail | Fix |
|---------------|-----|
| `Waiting for MoveIt services...` hangs | Launch `dual_arm_moveit.launch.py` first |
| `Cannot reach init config` | Check for joint limit violations in INIT_LEFT/INIT_RIGHT; verify URDF |
| `FK failed` | `compute_fk` service not available; check move_group is running |

### 5.2 IK failed

**Symptom:** `IK failed for left/right arm — pose may be unreachable.`

**Causes:**
- Target position is outside the arm's reachable workspace (~0.3 m – 1.2 m radius for A12)
- Target orientation is kinematically singular
- Seed joint state is far from the solution

**Fixes:**
1. Check the target position is within workspace
2. Try a simpler target (small delta from current position)
3. The coordinator already tries 3 seeds (current, INIT, zeros); if all fail the pose is unreachable

### 5.3 Motion failed (planning error)

**Symptom:** `Motion FAILED (planning or execution error)`, `error_code=-1` or similar.

**Causes:**
- Collision detected in the planned path
- Planning timeout exceeded (default 10 s)
- Start state is in collision

**Fixes:**
1. Check RViz for collision highlights (red links)
2. Increase planning time in `coordinator.py`:
   ```python
   goal.request.allowed_planning_time = 20.0
   ```
3. Increase planning attempts:
   ```python
   goal.request.num_planning_attempts = 20
   ```

### 5.4 TCP: both robots must be connected

**Symptom:** `TCP: both robots must be connected`

**Fix:** Ensure both `robot:left` and `robot:right` clients are connected before sending a `lift:` command.

### 5.5 Heartbeat timeout

**Symptom:** `Heartbeat timeout — robot [left/right]` followed by emergency stop.

**Causes:**
- Network packet loss between coordinator host and robot client
- Robot client process crashed or hung

**Fixes:**
1. Check network connectivity: `ping <robot_host>`
2. Check robot client logs for errors
3. Reconnect the robot client (it will re-register automatically)

### 5.6 RViz shows robot in wrong position

**Symptom:** Robot model in RViz does not match expected init pose.

**Fix:**
1. Check joint_state_broadcaster is running: `ros2 topic echo /joint_states`
2. Verify INIT_LEFT / INIT_RIGHT in `coordinator.py` match your physical setup

### 5.7 Robot not moving — JAKA driver service not found

**Symptom:** Coordinator logs `JAKA left/right driver not available — running simulation only`. Physical robots do not move.

**Diagnosis:**
```bash
ros2 service list | grep jaka
```

| Result | Cause |
|--------|-------|
| No output | `jaka_driver` nodes are not running |
| `/jaka_driver/joint_move` (no prefix) | Driver started without namespace |
| `/left/jaka_driver/joint_move` only | Right driver not started or crashed |

**Fixes:**

1. Driver not started — launch both drivers with the correct namespace flags:
   ```bash
   ros2 run jaka_driver jaka_driver \
     --ros-args -r __ns:=/left -p ip:=192.168.0.2

   ros2 run jaka_driver jaka_driver \
     --ros-args -r __ns:=/right -p ip:=192.168.0.1
   ```

2. Driver started without namespace — stop the driver and restart with `-r __ns:=/left` or `-r __ns:=/right`. The coordinator expects exactly `/left/jaka_driver/joint_move` and `/right/jaka_driver/joint_move`.

3. Driver running but service missing — the driver may have failed to connect to the robot IP. Check driver terminal for connection errors, then verify the robot controller is powered on and pingable:
   ```bash
   ping 192.168.0.2   # left arm
   ping 192.168.0.1   # right arm
   ```

4. Coordinator already started before drivers — restart the coordinator after the drivers are confirmed ready.

### 5.8 Robot not moving — joint_move service call fails

**Symptom:** Coordinator logs `[jaka_left] joint_move failed: ret=X msg=...` or `[jaka_right] joint_move failed`.

**Causes and fixes:**

| ret value | Meaning | Fix |
|-----------|---------|-----|
| `-1` | Robot controller error / fault | Check fault LEDs on JAKA controller box; clear fault via JAKA teach pendant |
| `0` returned but no motion | `ret=0` is success — robot may have already been at the target | Verify joint angles differ from current position |
| Timeout (result = None) | Service call took >30 s | Check network; restart driver |

To inspect the raw service response manually:
```bash
ros2 service call /left/jaka_driver/joint_move jaka_msgs/srv/Move \
  "{pose: [0.524, 2.443, 1.745, -1.047, 3.403, 0.0], has_ref: false,
    ref_joint: [], mvvelo: 0.1, mvacc: 0.1, mvtime: 0.0, mvradii: 0.0,
    coord_mode: 0, index: 0}"
```

---

## 6. Tuning INIT_LEFT and INIT_RIGHT

`INIT_LEFT` and `INIT_RIGHT` are the joint angles (radians) both arms move to on startup. These determine:
- The **object centre** (midpoint of the two flanges)
- The **grip span** (distance between flanges)
- The **reference orientation** for both end-effectors

### 6.1 Finding a good init pose

1. Use RViz's `MotionPlanning` panel to manually jog the arms to a desired pose
2. Read the resulting joint values from `/joint_states`:
   ```bash
   ros2 topic echo /joint_states --once
   ```
3. Copy the `position` array for `left_joint_1..6` → `INIT_LEFT` and `right_joint_1..6` → `INIT_RIGHT`

### 6.2 Constraints to satisfy

- Both flanges must face each other (grip axis aligned along Y in world frame for the default URDF)
- The midpoint of the two flanges becomes the object origin
- The grip span should match the actual object size (default config: ~0.60 m)
- The pose must be collision-free and have sufficient workspace margin for typical motions

### 6.3 Editing the values

```python
# coordinator.py  — lines ~367-368
INIT_LEFT  = [-0.349, -2.618, -0.873,  1.571,  3.142,  1.571]
INIT_RIGHT = [ 0.349, -0.524,  0.873, -1.571,  0.0,   -1.571]
```

After editing, rebuild and restart the coordinator:
```bash
cd ~/ros2_ws
colcon build --packages-select jaka_dual_arm
source install/setup.bash
ros2 run jaka_dual_arm coordinator
```

---

## 7. Adding a Gripper

The current setup uses flanges as the grip point. To add physical grippers:

### Step 1 — Add gripper geometry to URDF

Edit `urdf/dual_arm.urdf.xacro`. Add the gripper as a child of `left_flange` and `right_flange`:

```xml
<!-- Left gripper -->
<joint name="left_gripper_joint" type="fixed">
  <parent link="left_flange"/>
  <child link="left_gripper_base"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>
<link name="left_gripper_base">
  <visual>
    <geometry><box size="0.04 0.10 0.04"/></geometry>
  </visual>
  <collision>
    <geometry><box size="0.04 0.10 0.04"/></geometry>
  </collision>
</link>
```

Repeat for the right arm using `right_flange`.

### Step 2 — Update the IK link name

In `coordinator.py`, change the IK target link from `left_flange` / `right_flange` to the TCP (tool centre point) of the gripper:

```python
# Before:
lj = self._call_ik('left_arm', 'left_flange', tL_p, tL_q, ...)

# After (if gripper TCP link is named left_tcp):
lj = self._call_ik('left_arm', 'left_tcp', tL_p, tL_q, ...)
```

### Step 3 — Update the FK link name

```python
lp, lq = self._call_fk('left_arm', 'left_tcp', self.LEFT_JOINTS, self.INIT_LEFT)
rp, rq = self._call_fk('right_arm', 'right_tcp', self.RIGHT_JOINTS, self.INIT_RIGHT)
```

### Step 4 — Update the SRDF end-effector

In `config/dual_arm.srdf`:
```xml
<end_effector name="left_ee" parent_link="left_tcp" group="left_arm"/>
<end_effector name="right_ee" parent_link="right_tcp" group="right_arm"/>
```

### Step 5 — Add gripper ROS2 controller (if actuated)

In `config/ros2_controllers.yaml`, add a gripper controller. In `config/moveit_controllers.yaml`, declare it. Then add a gripper action client in `coordinator.py` to open/close before and after grasping.

### Step 6 — Rebuild and retest

```bash
cd ~/ros2_ws
colcon build --packages-select jaka_dual_arm
source install/setup.bash
ros2 launch jaka_dual_arm dual_arm_moveit.launch.py
# Verify in RViz that gripper geometry appears and no new collisions exist
```

---

## 8. Connecting Real JAKA Robots

The coordinator sends motion commands to physical JAKA robots via the `jaka_msgs/srv/Move` service — **independently of ros2_control**. The mock controllers still run (keeping RViz synchronised), while the real robots receive final joint positions directly over the network.

### 8.1 How it works

After MoveIt plans and executes on the mock controllers, the coordinator:
1. Extracts the **final joint positions** from the planned trajectory.
2. Calls `/left/jaka_driver/joint_move` and `/right/jaka_driver/joint_move` **simultaneously** in separate threads.
3. Waits for both calls to return before reporting success.

The `Move` service request uses joint-space mode (`coord_mode=0`) with `mvvelo=0.3` and `mvacc=0.3`. To change speed, edit `_send_to_jaka()` in `coordinator.py`.

### 8.2 Step-by-step startup for real robots

#### Step 1 — Build the JAKA driver package

```bash
cd ~/ros2_ws
colcon build --packages-select jaka_msgs jaka_driver
source install/setup.bash
```

#### Step 2 — Power on both robot controllers

- Left arm:  IP `192.168.0.2`
- Right arm: IP `192.168.0.1`

Verify network connectivity:
```bash
ping 192.168.0.2   # left arm — expect <1 ms on local subnet
ping 192.168.0.1   # right arm
```

#### Step 3 — Launch both JAKA drivers with namespaces

Both robots run the same `jaka_driver` executable. The `-r __ns:=/left` and `-r __ns:=/right` flags put each node into its own namespace, making their service names unique.

```bash
# Terminal 2 — Left arm
ros2 run jaka_driver jaka_driver \
  --ros-args -r __ns:=/left -p ip:=192.168.0.2

# Terminal 3 — Right arm
ros2 run jaka_driver jaka_driver \
  --ros-args -r __ns:=/right -p ip:=192.168.0.1
```

#### Step 4 — Verify services

```bash
ros2 service list | grep jaka
```

You must see both of these before proceeding:
```
/left/jaka_driver/joint_move
/right/jaka_driver/joint_move
```

If either is missing, see [Troubleshooting 5.7](#57-robot-not-moving--jaka-driver-service-not-found).

#### Step 5 — Launch MoveIt2 stack (Terminal 1)

```bash
ros2 launch jaka_dual_arm dual_arm_moveit.launch.py
```

Wait for RViz to open.

#### Step 6 — Launch coordinator (Terminal 4)

```bash
ros2 run jaka_dual_arm coordinator
```

Confirm these lines appear at startup:
```
[coordinator]: JAKA left driver ready
[coordinator]: JAKA right driver ready
```

If you see `not available` instead, the coordinator will operate in simulation-only mode and the physical robots will not move.

#### Step 7 — Lower VEL/ACC for initial tests

Edit `coordinator.py` before the first real run:
```python
VEL = 0.05   # 5 % — use for first run on real hardware
ACC = 0.05
```

Rebuild and restart:
```bash
colcon build --packages-select jaka_dual_arm
source install/setup.bash
ros2 run jaka_dual_arm coordinator
```

#### Step 8 — Test with small motions

Send a 2–3 cm delta and observe both physical robots before attempting larger moves:
```bash
ros2 topic pub --once /object_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.02, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

The coordinator log should show:
```
[coordinator]: [jaka] both arms moved successfully
[coordinator]: Motion SUCCEEDED
```

### 8.3 Pre-motion checklist for real robots

- [ ] E-stop button accessible and tested
- [ ] Workspace cleared of all obstacles and personnel
- [ ] Joint limits verified in `joint_limits.yaml`
- [ ] VEL/ACC set to ≤10% for initial tests
- [ ] Both robot controllers powered on, no fault LEDs
- [ ] `ros2 service list | grep jaka` shows both `/left` and `/right` services
- [ ] Coordinator logs `JAKA left driver ready` and `JAKA right driver ready`
- [ ] Motion verified in RViz simulation before executing on real hardware

---

---

# คู่มือการใช้งาน — JAKA Dual-Arm Cooperative Manipulation

## 1. ข้อกำหนดเบื้องต้น

ก่อนเริ่ม ตรวจสอบ:

- [ ] ROS2 Jazzy ถูก source แล้ว: `source /opt/ros/jazzy/setup.bash`
- [ ] Workspace ถูก build แล้ว: `colcon build` เสร็จโดยไม่มีข้อผิดพลาด
- [ ] Workspace ถูก source แล้ว: `source ~/ros2_ws/install/setup.bash`
- [ ] (หุ่นยนต์จริง) คอนโทรลเลอร์ JAKA ทั้งสองตัวเปิดแล้วและ ping ได้
- [ ] (จำลอง) ไม่ต้องการหุ่นยนต์จริง — mock controllers จัดการการทำงานเอง

---

## 2. การเริ่มต้นระบบ (ทีละขั้นตอน)

### Terminal 1 — MoveIt2 Stack

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch jaka_dual_arm dual_arm_moveit.launch.py
```

**สิ่งที่เปิดขึ้น:**
- `ros2_control_node` — ตัวจัดการ joint trajectory controller (100 Hz)
- `left_arm_controller` และ `right_arm_controller` — ควบคุมแต่ละแขน
- `move_group` — MoveIt2 OMPL planner + ตรวจสอบการชน
- `rviz2` — วิชวลไลเซชัน

รอจนกว่า RViz จะเปิดและแสดงโมเดลหุ่นยนต์ก่อนดำเนินการต่อ

### Terminal 2 & 3 — JAKA Hardware Drivers (หุ่นยนต์จริงเท่านั้น)

> ข้ามขั้นตอนนี้สำหรับโหมดจำลอง coordinator จะตรวจสอบอัตโนมัติ

JAKA driver ทั้งสองตัวใช้ executable เดียวกัน ต้องใช้ **namespace** เพื่อแยกชื่อ service

```bash
# Terminal 2 — แขนซ้าย (192.168.0.2)
ros2 run jaka_driver jaka_driver \
  --ros-args -r __ns:=/left -p ip:=192.168.0.2

# Terminal 3 — แขนขวา (192.168.0.1)
ros2 run jaka_driver jaka_driver \
  --ros-args -r __ns:=/right -p ip:=192.168.0.1
```

ตรวจสอบ services ก่อนเปิด coordinator:
```bash
ros2 service list | grep jaka
# ต้องเห็น:
# /left/jaka_driver/joint_move
# /right/jaka_driver/joint_move
```

### Terminal 4 (หรือ 2 ในโหมดจำลอง) — Coordinator Node

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run jaka_dual_arm coordinator
```

**โหมดจำลอง (ไม่มี driver):**
```
[coordinator]: JAKA left driver not available — running simulation only
[coordinator]: JAKA right driver not available — running simulation only
[coordinator]: Services ready — initialising grasp configuration.
...
[coordinator]: Ready!  Publish PoseStamped (frame_id=world) to /object_target
```

**โหมดหุ่นยนต์จริง (มี driver ทั้งสองตัว):**
```
[coordinator]: JAKA left driver ready
[coordinator]: JAKA right driver ready
[coordinator]: Services ready — initialising grasp configuration.
...
[coordinator]: [jaka] both arms moved successfully
[coordinator]: Ready!  Publish PoseStamped (frame_id=world) to /object_target
```

ระบบพร้อมรับคำสั่งแล้ว

---

## 3. การส่งคำสั่งผ่าน ROS2 Topic

### 3.1 เลื่อนตำแหน่งเท่านั้น (ไม่หมุน)

```bash
# เลื่อน +5 ซม. ในแกน X ไปที่ (0.45, 0.0, 1.0)
ros2 topic pub --once /object_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.45, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"
```

### 3.2 เลื่อนพร้อมหมุน

คำนวณ quaternion จาก RPY ด้วย Python:

```python
from scipy.spatial.transform import Rotation as Rot
# หมุน 30° รอบแกน X
q = Rot.from_euler('xyz', [30, 0, 0], degrees=True).as_quat()
# q = [x, y, z, w]
print(q)
```

แล้วส่ง:
```bash
ros2 topic pub --once /object_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.45, y: 0.0, z: 1.0}, \
   orientation: {x: 0.259, y: 0.0, z: 0.0, w: 0.966}}}"
```

---

## 4. รูปแบบคำสั่ง TCP และตัวอย่าง

เชื่อมต่อไปที่ **TCP พอร์ต 9090**

### รูปแบบคำสั่ง
```
lift:X,Y,Z:RX,RY,RZ\n
```

| พารามิเตอร์ | หน่วย | คำอธิบาย |
|------------|-------|---------|
| X, Y, Z | เมตร | ตำแหน่งเป้าหมายสัมบูรณ์ใน world frame |
| RX, RY, RZ | องศา | การหมุนแบบ RPY (XYZ Euler) |

### ตัวอย่าง

```
# เลื่อนไปที่ (0.40, 0.00, 1.00) ไม่หมุน
lift:0.40,0.00,1.00:0.0,0.0,0.0

# เลื่อนไปที่ (0.45, 0.00, 1.05) หมุน 15° รอบแกน Z
lift:0.45,0.00,1.05:0.0,0.0,15.0
```

### Python client

```python
import socket

def send_lift(host, x, y, z, rx=0.0, ry=0.0, rz=0.0, port=9090):
    cmd = f'lift:{x},{y},{z}:{rx},{ry},{rz}\n'
    with socket.create_connection((host, port), timeout=45) as s:
        s.sendall(cmd.encode())
        return s.recv(16).decode().strip()

result = send_lift('127.0.0.1', 0.45, 0.0, 1.05, rz=15.0)
print(result)  # 'done' หรือ 'error'
```

---

## 5. คู่มือแก้ไขปัญหา

### coordinator ออกทันทีโดยไม่มี "Ready!"

| อาการโดยละเอียด | วิธีแก้ |
|---------------|--------|
| `Waiting for MoveIt services...` ค้าง | เปิด `dual_arm_moveit.launch.py` ก่อน |
| `Cannot reach init config` | ตรวจสอบค่า INIT_LEFT/INIT_RIGHT ใน coordinator.py |
| `FK failed` | ตรวจสอบว่า move_group ทำงานอยู่ |

### IK failed

**สาเหตุ:**
- ตำแหน่งเป้าหมายอยู่นอกพื้นที่ทำงานของแขน (~0.3–1.2 ม.)
- มุมที่ต้องการเป็น singularity

**วิธีแก้:**
1. ตรวจสอบว่าตำแหน่งเป้าหมายอยู่ในพื้นที่ทำงาน
2. ลองเป้าหมายที่ง่ายกว่า (เลื่อนเล็กน้อยจากตำแหน่งปัจจุบัน)

### Motion FAILED

**วิธีแก้:**
1. ตรวจสอบ RViz หาการชน (link สีแดง)
2. เพิ่มเวลาวางแผนใน coordinator.py:
   ```python
   goal.request.allowed_planning_time = 20.0
   ```

---

## 6. การปรับ INIT_LEFT และ INIT_RIGHT

`INIT_LEFT` และ `INIT_RIGHT` คือมุมข้อต่อ (เรเดียน) ที่แขนทั้งสองเคลื่อนไปเมื่อเริ่มต้น

### วิธีหาค่าที่เหมาะสม

1. ใช้ RViz จอกแขนไปที่ท่าที่ต้องการ
2. อ่านค่าจาก `/joint_states`:
   ```bash
   ros2 topic echo /joint_states --once
   ```
3. คัดลอกค่า `position` สำหรับ `left_joint_1..6` → `INIT_LEFT` และ `right_joint_1..6` → `INIT_RIGHT`

### แก้ไขค่า

```python
# coordinator.py
INIT_LEFT  = [-0.349, -2.618, -0.873,  1.571,  3.142,  1.571]
INIT_RIGHT = [ 0.349, -0.524,  0.873, -1.571,  0.0,   -1.571]
```

หลังแก้ไข build และรีสตาร์ท coordinator:
```bash
cd ~/ros2_ws
colcon build --packages-select jaka_dual_arm
source install/setup.bash
ros2 run jaka_dual_arm coordinator
```

---

## 7. การเพิ่ม Gripper

### ขั้นตอนที่ 1 — เพิ่ม geometry ใน URDF

แก้ไข `urdf/dual_arm.urdf.xacro` เพิ่ม gripper เป็น child ของ `left_flange` และ `right_flange`

### ขั้นตอนที่ 2 — อัปเดต IK link name

```python
# coordinator.py
lj = self._call_ik('left_arm', 'left_tcp', tL_p, tL_q, ...)
rj = self._call_ik('right_arm', 'right_tcp', tR_p, tR_q, ...)
```

### ขั้นตอนที่ 3 — อัปเดต FK link name

```python
lp, lq = self._call_fk('left_arm', 'left_tcp', self.LEFT_JOINTS, self.INIT_LEFT)
rp, rq = self._call_fk('right_arm', 'right_tcp', self.RIGHT_JOINTS, self.INIT_RIGHT)
```

### ขั้นตอนที่ 4 — อัปเดต SRDF

```xml
<end_effector name="left_ee" parent_link="left_tcp" group="left_arm"/>
<end_effector name="right_ee" parent_link="right_tcp" group="right_arm"/>
```

### ขั้นตอนที่ 5 — Build และทดสอบ

```bash
cd ~/ros2_ws
colcon build --packages-select jaka_dual_arm
source install/setup.bash
ros2 launch jaka_dual_arm dual_arm_moveit.launch.py
```

---

## 8. การเชื่อมต่อหุ่นยนต์ JAKA จริง

coordinator ส่งคำสั่งไปยังหุ่นยนต์ JAKA จริงผ่าน service `jaka_msgs/srv/Move` **โดยตรง** โดยไม่ผ่าน ros2_control mock controllers ยังทำงานอยู่เพื่อให้ RViz อัปเดต ส่วนหุ่นยนต์จริงรับตำแหน่งข้อต่อสุดท้ายผ่านเครือข่าย

### 8.1 วิธีการทำงาน

หลัง MoveIt วางแผนและดำเนินการบน mock controllers coordinator จะ:
1. ดึง **ตำแหน่งข้อต่อสุดท้าย** จาก trajectory ที่วางแผนไว้
2. เรียก `/left/jaka_driver/joint_move` และ `/right/jaka_driver/joint_move` **พร้อมกัน** ใน thread แยกกัน
3. รอให้ทั้งสอง call กลับมาก่อนรายงานผล

### 8.2 ขั้นตอนการเริ่มต้นสำหรับหุ่นยนต์จริง

#### ขั้นตอนที่ 1 — Build JAKA driver

```bash
cd ~/ros2_ws
colcon build --packages-select jaka_msgs jaka_driver
source install/setup.bash
```

#### ขั้นตอนที่ 2 — เปิดคอนโทรลเลอร์หุ่นยนต์และตรวจสอบเครือข่าย

```bash
ping 192.168.0.2   # แขนซ้าย
ping 192.168.0.1   # แขนขวา
```

#### ขั้นตอนที่ 3 — เปิด JAKA drivers พร้อม namespace

```bash
# Terminal 2 — แขนซ้าย
ros2 run jaka_driver jaka_driver \
  --ros-args -r __ns:=/left -p ip:=192.168.0.2

# Terminal 3 — แขนขวา
ros2 run jaka_driver jaka_driver \
  --ros-args -r __ns:=/right -p ip:=192.168.0.1
```

#### ขั้นตอนที่ 4 — ตรวจสอบ services

```bash
ros2 service list | grep jaka
# ต้องเห็น:
# /left/jaka_driver/joint_move
# /right/jaka_driver/joint_move
```

หากไม่เห็น ดูหัวข้อแก้ปัญหา 5.7

#### ขั้นตอนที่ 5 — เปิด MoveIt2 และ coordinator

```bash
# Terminal 1
ros2 launch jaka_dual_arm dual_arm_moveit.launch.py

# Terminal 4
ros2 run jaka_dual_arm coordinator
```

ยืนยันว่าเห็น:
```
[coordinator]: JAKA left driver ready
[coordinator]: JAKA right driver ready
```

#### ขั้นตอนที่ 6 — ลด VEL/ACC สำหรับการทดสอบครั้งแรก

```python
# coordinator.py
VEL = 0.05   # 5% สำหรับการทดสอบครั้งแรก
ACC = 0.05
```

```bash
colcon build --packages-select jaka_dual_arm
source install/setup.bash
ros2 run jaka_dual_arm coordinator
```

#### ขั้นตอนที่ 7 — ทดสอบด้วยการเคลื่อนที่เล็กน้อย

```bash
ros2 topic pub --once /object_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.02, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

ดู log:
```
[coordinator]: [jaka] both arms moved successfully
[coordinator]: Motion SUCCEEDED
```

### Checklist ก่อนใช้กับหุ่นยนต์จริง

- [ ] ปุ่ม E-stop เข้าถึงและทดสอบแล้ว
- [ ] พื้นที่ทำงานปลอดคนและสิ่งกีดขวาง
- [ ] ตรวจสอบ joint limits ใน `joint_limits.yaml`
- [ ] VEL/ACC ≤ 10% สำหรับการทดสอบเริ่มต้น
- [ ] คอนโทรลเลอร์หุ่นยนต์ทั้งสองไม่มีไฟ fault
- [ ] `ros2 service list | grep jaka` แสดง services ทั้ง `/left` และ `/right`
- [ ] coordinator แสดง `JAKA left driver ready` และ `JAKA right driver ready`
- [ ] ตรวจสอบการเคลื่อนที่ใน RViz ก่อนดำเนินการกับหุ่นยนต์จริง
