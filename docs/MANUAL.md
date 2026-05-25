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

### Terminal 2 — Coordinator Node

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run jaka_dual_arm coordinator
```

**Startup sequence:**
```
[coordinator]: Waiting for MoveIt services...
[coordinator]: Services ready — initialising grasp configuration.
[coordinator]: Moving to init joint configuration...
[coordinator]:   [exec] waiting for goal handle...
[coordinator]:   [exec] goal accepted, waiting for result...
[coordinator]: Left  flange @ world: (0.4000,  0.3000, 1.0000)
[coordinator]: Right flange @ world: (0.4000, -0.3000, 1.0000)
[coordinator]: Object centre @ world: (0.4000, 0.0000, 1.0000)
[coordinator]: Grip span: 0.6000 m
[coordinator]: Collision box added: size=(0.54, 0.1, 0.1)  pos=(0.4000,0.0000,1.0000)
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

The current `ros2_controllers.yaml` uses the **mock hardware interface** (simulation). To control real JAKA A12 robots:

### Step 1 — Install JAKA ROS2 driver

```bash
# Follow JAKA's official ROS2 driver installation guide
# Typically:
cd ~/ros2_ws/src
git clone https://github.com/JAKACobot/jaka_ros2_driver.git
cd ~/ros2_ws
colcon build --packages-select jaka_ros2_driver
source install/setup.bash
```

### Step 2 — Configure robot IP addresses

In the JAKA driver config, set the controller IP for each arm:
```yaml
# e.g., jaka_driver_left.yaml
robot_ip: "192.168.1.10"   # Left arm controller IP

# jaka_driver_right.yaml
robot_ip: "192.168.1.11"   # Right arm controller IP
```

### Step 3 — Replace mock hardware with real hardware interface

In `urdf/dual_arm.urdf.xacro`, change the `hardware` plugin from mock to real:
```xml
<!-- Before (simulation) -->
<plugin>mock_components/GenericSystem</plugin>

<!-- After (real JAKA) -->
<plugin>jaka_hardware_interface/JAKAHardwareInterface</plugin>
```

### Step 4 — Verify joint state feedback

```bash
ros2 topic echo /joint_states
```

Confirm all 12 joints report the correct current position matching the physical robot.

### Step 5 — Lower VEL/ACC for first run

```python
# coordinator.py
VEL = 0.05   # 5% velocity for first run on real hardware
ACC = 0.05
```

### Step 6 — Test with small motions

Send a very small target delta (2–3 cm) and observe the physical motion before testing larger moves.

### Safety checklist before real-robot operation

- [ ] E-stop button accessible at all times
- [ ] Workspace cleared of all obstacles and personnel
- [ ] Joint limits verified in `joint_limits.yaml`
- [ ] VEL/ACC set to ≤10% for initial tests
- [ ] Both robot controllers show no fault LEDs
- [ ] TCP heartbeat test passed with robot clients connected

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

### Terminal 2 — Coordinator Node

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run jaka_dual_arm coordinator
```

รอจนเห็น:
```
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

### ขั้นตอนที่ 1 — ติดตั้ง JAKA ROS2 driver

```bash
cd ~/ros2_ws/src
git clone https://github.com/JAKACobot/jaka_ros2_driver.git
cd ~/ros2_ws
colcon build --packages-select jaka_ros2_driver
source install/setup.bash
```

### ขั้นตอนที่ 2 — กำหนด IP ของหุ่นยนต์

```yaml
robot_ip: "192.168.1.10"   # IP คอนโทรลเลอร์แขนซ้าย
robot_ip: "192.168.1.11"   # IP คอนโทรลเลอร์แขนขวา
```

### ขั้นตอนที่ 3 — เปลี่ยน hardware interface

ใน `urdf/dual_arm.urdf.xacro` เปลี่ยน plugin จาก mock เป็น real

### ขั้นตอนที่ 4 — ลด VEL/ACC สำหรับการทดสอบครั้งแรก

```python
VEL = 0.05   # 5% สำหรับการทดสอบครั้งแรก
ACC = 0.05
```

### Checklist ก่อนใช้กับหุ่นยนต์จริง

- [ ] ปุ่ม E-stop เข้าถึงได้ตลอดเวลา
- [ ] พื้นที่ทำงานปลอดคนและสิ่งกีดขวาง
- [ ] ตรวจสอบ joint limits ใน `joint_limits.yaml`
- [ ] VEL/ACC ≤ 10% สำหรับการทดสอบเริ่มต้น
- [ ] คอนโทรลเลอร์หุ่นยนต์ทั้งสองไม่มีไฟ fault
