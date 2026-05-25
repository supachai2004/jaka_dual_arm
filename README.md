# JAKA Dual-Arm Cooperative Manipulation

**EN** | [TH ภาษาไทย](#ภาษาไทย)

---

## Overview

This ROS2 package implements **cooperative dual-arm manipulation** using two JAKA A12 6-DOF robot arms. Both arms grip a single rigid object simultaneously and move it as a coordinated unit. A supervisor node (`coordinator`) computes IK for both flanges from a single object-pose command, plans collision-free trajectories via MoveIt2, and drives both arms in lock-step.

An optional **TCP server (port 9090)** lets external programs (e.g., a vision system or PLC) connect as robot clients and issue lift commands without needing ROS2.

---

## System Architecture

The coordinator runs **two execution paths in sequence** for each motion command:

- **① MoveIt path** — plans the trajectory, executes it on mock controllers so RViz stays in sync.
- **② JAKA direct path** — after planning, sends the final joint positions directly to each physical robot via `jaka_msgs/srv/Move`, bypassing ros2_control entirely. Both arms are commanded simultaneously in separate threads.

```
┌─────────────────────────────────────────────────────────────────────┐
│                          ROS2 Environment                            │
│                                                                      │
│  ┌──────────────┐    /object_target     ┌─────────────────────────┐ │
│  │ Vision / UI  │──PoseStamped────────►│      coordinator        │ │
│  │  or ros2     │                       │      (main node)        │ │
│  │  topic pub   │                       │                         │ │
│  └──────────────┘                       │  ┌───────────────────┐  │ │
│                                         │  │   TCPInterface    │  │ │
│  ┌──────────────┐   TCP port 9090       │  │   port 9090       │  │ │
│  │ External     │◄─────────────────────►│  └───────────────────┘  │ │
│  │ client       │  lift:x,y,z:rx,ry,rz  │                         │ │
│  └──────────────┘                       │      FK / IK / Plan     │ │
│                                         └────┬──────────────┬──────┘ │
│                              ① MoveIt path │              │ ② JAKA path
│              ┌──────────────────────────────▼──────────┐  │           │
│              │          MoveIt2  move_group             │  │           │
│              │   OMPL planner · KDL kinematics          │  │           │
│              │   both_arms (12 joints)                  │  │           │
│              └──────────────────┬───────────────────────┘  │           │
│                                 │ (RViz visualization)      │           │
│              ┌──────────────────▼───────────────────────┐  │           │
│              │     ros2_control mock controllers         │  │           │
│              │  left_arm_controller  right_arm_ctrl      │  │           │
│              └──────────────────────────────────────────┘  │           │
│                                                             │ /joint_move │
│              ┌──────────────────────────────────────────────▼───────┐   │
│              │  /left/jaka_driver          /right/jaka_driver        │   │
│              │  (192.168.0.2)              (192.168.0.1)             │   │
│              └──────────────┬──────────────────────┬────────────────┘   │
└─────────────────────────────┼──────────────────────┼────────────────────┘
                              │                      │
                      ┌───────▼──────┐      ┌────────▼─────┐
                      │  JAKA A12    │      │  JAKA A12    │
                      │  LEFT ARM    │      │  RIGHT ARM   │
                      │  (6 joints)  │      │  (6 joints)  │
                      └──────────────┘      └──────────────┘
                            │                      │
                            └──────────┬───────────┘
                                       │
                                ┌──────▼──────┐
                                │   Object    │
                                │  (gripped)  │
                                └─────────────┘
```

### Key Design Decisions

| Concern | Choice | Reason |
|---------|--------|--------|
| Kinematics solver | KDL | Reliable for 6-DOF serial chains |
| Motion planner | OMPL (RRTConnect) | Handles 12-DOF joint space efficiently |
| Arm synchronisation | Joint-space goal, `both_arms` group | One plan covers both arms atomically |
| Object frame | Midpoint of two flanges at init pose | No external sensor required |
| Collision object | MoveIt `CollisionObject` (payload) | MoveIt avoids self-collision with the gripped box |
| TCP protocol | Custom line-based text | Easy to implement in any language/PLC |
| JAKA direct execution | `jaka_msgs/srv/Move` per arm | Bypasses ros2_control; no hardware interface required |
| JAKA namespace | `/left/jaka_driver`, `/right/jaka_driver` | Two nodes share the same executable; namespaces differentiate them |

---

## Hardware Requirements

| Component | Specification |
|-----------|--------------|
| Robot arms | 2× JAKA A12 (6-DOF, 12 kg payload each) |
| Controller PC | Ubuntu 24.04 LTS, x86-64, ≥8 GB RAM |
| Network | Gigabit Ethernet (robot controllers on same subnet) |
| Mounting | Left arm at `(0, +0.30, 0)`, Right arm at `(0, -0.30, 0)` relative to world origin (adjustable in URDF) |

---

## Software Dependencies

| Package | Version |
|---------|---------|
| Ubuntu | 24.04 LTS |
| ROS2 | Jazzy Jalisco |
| MoveIt2 | Jazzy release |
| moveit_msgs | Jazzy |
| geometry_msgs | Jazzy |
| sensor_msgs | Jazzy |
| shape_msgs | Jazzy |
| visualization_msgs | Jazzy |
| tf2_ros | Jazzy |
| jaka_msgs | from `jaka_ros2` repo |
| jaka_driver | from `jaka_ros2` repo |
| Python | 3.12 |
| scipy | ≥1.11 |
| numpy | ≥1.26 |

---

## Installation

### 1. Install ROS2 Jazzy

Follow the official guide: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

```bash
sudo apt install ros-jazzy-desktop
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install MoveIt2 and dependencies

```bash
sudo apt install \
  ros-jazzy-moveit \
  ros-jazzy-moveit-ros-move-group \
  ros-jazzy-moveit-planners-ompl \
  ros-jazzy-moveit-simple-controller-manager \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-robot-state-publisher
```

### 3. Install Python dependencies

```bash
pip3 install scipy numpy
```

### 4. Clone and build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/supachai2004/jaka_dual_arm.git
cd ~/ros2_ws
colcon build --packages-select jaka_dual_arm
source install/setup.bash
```

---

## Quick Start Guide

### Simulation only (no physical robots)

```bash
# Terminal 1 — MoveIt2 stack
source ~/ros2_ws/install/setup.bash
ros2 launch jaka_dual_arm dual_arm_moveit.launch.py

# Terminal 2 — Coordinator (drivers absent; RViz-only mode)
source ~/ros2_ws/install/setup.bash
ros2 run jaka_dual_arm coordinator
```

The coordinator logs `JAKA left/right driver not available — running simulation only` and continues normally. All motion is visualised in RViz.

### With real JAKA robots

```bash
# Terminal 1 — MoveIt2 stack
source ~/ros2_ws/install/setup.bash
ros2 launch jaka_dual_arm dual_arm_moveit.launch.py

# Terminal 2 — Left JAKA driver  (IP 192.168.0.2)
source ~/ros2_ws/install/setup.bash
ros2 run jaka_driver jaka_driver \
  --ros-args -r __ns:=/left -p ip:=192.168.0.2

# Terminal 3 — Right JAKA driver  (IP 192.168.0.1)
source ~/ros2_ws/install/setup.bash
ros2 run jaka_driver jaka_driver \
  --ros-args -r __ns:=/right -p ip:=192.168.0.1

# Terminal 4 — Coordinator
source ~/ros2_ws/install/setup.bash
ros2 run jaka_dual_arm coordinator
```

Verify driver services are visible before starting the coordinator:
```bash
ros2 service list | grep jaka
# Expected:
# /left/jaka_driver/joint_move
# /right/jaka_driver/joint_move
# (plus other jaka_driver services)
```

The coordinator startup log confirms real-robot mode:
```
[coordinator]: JAKA left driver ready
[coordinator]: JAKA right driver ready
[coordinator]: Services ready — initialising grasp configuration.
...
[coordinator]: Ready!  Publish PoseStamped (frame_id=world) to /object_target to move the object.
```

### Send a move command

```bash
# Move object +5 cm in X
ros2 topic pub --once /object_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.05, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

---

## TCP Interface Protocol

The coordinator starts a TCP server on **port 9090** for external clients.

### Connection types

#### Robot client
A robot controller connects and identifies itself:
```
robot:left\n
robot:right\n
```
After connecting, the server sends heartbeat pings every 100 ms. The robot must reply `pong\n` within 500 ms or an emergency stop is triggered.

#### Command client
An external program (vision system, PLC, GUI) issues a single lift command then disconnects:
```
lift:x,y,z:rx_deg,ry_deg,rz_deg\n
```

**Fields:**

| Field | Unit | Description |
|-------|------|-------------|
| x, y, z | metres | Target object position in world frame (absolute) |
| rx, ry, rz | degrees | Target object orientation as RPY (XYZ Euler) |

**Response sequence:**

```
Client sends:  lift:0.45,0.0,1.05:0.0,0.0,0.0
Server→robots: pose:0.45,0.30,1.05:qx,qy,qz,qw   (left)
Server→robots: pose:0.45,-0.30,1.05:qx,qy,qz,qw  (right)
Robots reply:  ready
Server→robots: go
Robots reply:  done
Server→client: done
```

If any step fails or times out: server replies `error\n` and triggers emergency stop.

**Example — Python client:**
```python
import socket

with socket.socket() as s:
    s.connect(('192.168.1.100', 9090))
    s.sendall(b'lift:0.45,0.0,1.05:0.0,0.0,15.0\n')
    resp = s.recv(16).decode().strip()
    print(resp)  # 'done' or 'error'
```

### Timeouts

| Phase | Timeout |
|-------|---------|
| Robot ready | 10 s |
| Motion done | 30 s |
| Heartbeat pong | 500 ms |

---

## Safety Guidelines

1. **Always keep the workspace clear** before sending any move command.
2. **Verify the init pose** in simulation (RViz) before running on a real robot.
3. **Never exceed joint limits** — the URDF and `joint_limits.yaml` enforce hardware limits; do not loosen them.
4. **TCP heartbeat** — if a robot client disconnects mid-motion the server triggers an emergency stop via `stop\n` to all connected robots. Implement `stop` handling in your robot client.
5. **Collision object** — the gripped payload is registered in MoveIt's planning scene as a `payload` box (0.54 × 0.10 × 0.10 m). MoveIt's planner respects it. Do not remove it while the arms are moving.
6. **Reduce VEL/ACC** for the first run with a real load. Edit `coordinator.py`:
   ```python
   VEL = 0.1   # start at 10 %
   ACC = 0.1
   ```
7. **Emergency stop** — publish nothing and kill the coordinator process (`Ctrl-C`). MoveIt will hold the last joint positions. Note: when JAKA drivers are active, killing the coordinator does **not** stop the robot mid-move via `joint_move` — use the physical E-stop on the JAKA controller box for immediate hardware halt.
8. **Simulation vs real robot** — the coordinator logs whether each JAKA driver is reachable at startup. If you see `running simulation only`, no commands reach the physical robots regardless of what is published to `/object_target`.

See [docs/SAFETY.md](docs/SAFETY.md) for full safety procedures.

---

---

# ภาษาไทย

## ภาพรวมโครงการ

แพ็กเกจ ROS2 นี้ใช้สำหรับ **การควบคุมแขนหุ่นยนต์สองแขนแบบร่วมมือ** โดยใช้แขน JAKA A12 6 แกน จำนวน 2 ตัว แขนทั้งสองจับวัตถุแข็งชิ้นเดียวพร้อมกัน และเคลื่อนที่เป็นหน่วยเดียวกัน โหนดหลัก (`coordinator`) คำนวณ IK สำหรับหน้าแปลนทั้งสองจากคำสั่งท่าทางวัตถุเพียงคำสั่งเดียว วางแผนเส้นทางหลบหลีกการชนผ่าน MoveIt2 และขับเคลื่อนแขนทั้งสองพร้อมกัน

**TCP server (พอร์ต 9090)** ให้โปรแกรมภายนอก (เช่น ระบบ vision หรือ PLC) เชื่อมต่อและส่งคำสั่งโดยไม่ต้องใช้ ROS2

---

## สถาปัตยกรรมระบบ

coordinator รัน **สองเส้นทางการดำเนินการ** ต่อคำสั่งการเคลื่อนที่:
- **① MoveIt path** — วางแผน trajectory และดำเนินการบน mock controllers เพื่อให้ RViz อัปเดต
- **② JAKA direct path** — หลังวางแผน ส่งตำแหน่งข้อต่อสุดท้ายโดยตรงไปยังหุ่นยนต์แต่ละตัวผ่าน `jaka_msgs/srv/Move` โดยส่งทั้งสองแขนพร้อมกัน

```
┌──────────────────────────────────────────────────────────────────────┐
│                         ROS2 Environment                              │
│                                                                       │
│  ┌──────────────┐    /object_target     ┌──────────────────────────┐ │
│  │ Vision / UI  │──PoseStamped────────►│      coordinator         │ │
│  └──────────────┘                       │      (โหนดหลัก)          │ │
│                                         │                          │ │
│  ┌──────────────┐   TCP พอร์ต 9090      │  TCPInterface พอร์ต 9090 │ │
│  │ ไคลเอนต์     │◄─────────────────────►│                          │ │
│  │ ภายนอก       │  lift:x,y,z:rx,ry,rz  │  FK / IK / Plan          │ │
│  └──────────────┘                       └────┬──────────────┬───────┘ │
│                              ① MoveIt path │              │ ② JAKA    │
│              ┌───────────────────────────────▼──────────┐  │           │
│              │       MoveIt2  move_group                 │  │           │
│              │  OMPL · KDL · both_arms (12 แกน)         │  │           │
│              └──────────────────┬────────────────────────┘  │           │
│              ┌──────────────────▼────────────────────────┐  │           │
│              │   ros2_control mock controllers (100 Hz)   │  │           │
│              │  left_arm_controller  right_arm_controller │  │           │
│              └────────────────────────────────────────────┘  │           │
│                                                               │ /joint_move
│              ┌──────────────────────────────────────────────▼────┐      │
│              │  /left/jaka_driver (192.168.0.2)                   │      │
│              │  /right/jaka_driver (192.168.0.1)                  │      │
│              └─────────────┬─────────────────────────┬────────────┘      │
└─────────────────────────────┼─────────────────────────┼───────────────────┘
                              │                         │
                      ┌───────▼───────┐       ┌─────────▼──────┐
                      │ JAKA A12 ซ้าย  │       │ JAKA A12 ขวา   │
                      │  (6 แกน)       │       │  (6 แกน)       │
                      └───────┬───────┘       └───────┬────────┘
                              └──────────┬─────────────┘
                                    ┌────▼────┐
                                    │  วัตถุ  │
                                    └─────────┘
```

---

## ความต้องการด้านฮาร์ดแวร์

| ส่วนประกอบ | รายละเอียด |
|-----------|-----------|
| แขนหุ่นยนต์ | JAKA A12 จำนวน 2 ตัว (6 แกน, รับน้ำหนักได้ 12 กก. ต่อตัว) |
| คอมพิวเตอร์ควบคุม | Ubuntu 24.04 LTS, x86-64, RAM ≥ 8 GB |
| เครือข่าย | Gigabit Ethernet (คอนโทรลเลอร์หุ่นยนต์ในซับเน็ตเดียวกัน) |
| การติดตั้ง | แขนซ้ายที่ `(0, +0.30, 0)`, แขนขวาที่ `(0, -0.30, 0)` เทียบกับ world frame |

---

## การพึ่งพาซอฟต์แวร์

| แพ็กเกจ | เวอร์ชัน |
|---------|---------|
| Ubuntu | 24.04 LTS |
| ROS2 | Jazzy Jalisco |
| MoveIt2 | Jazzy release |
| Python | 3.12 |
| scipy | ≥1.11 |
| numpy | ≥1.26 |

---

## ขั้นตอนการติดตั้ง

### 1. ติดตั้ง ROS2 Jazzy

```bash
sudo apt install ros-jazzy-desktop
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. ติดตั้ง MoveIt2 และ dependencies

```bash
sudo apt install \
  ros-jazzy-moveit \
  ros-jazzy-moveit-ros-move-group \
  ros-jazzy-moveit-planners-ompl \
  ros-jazzy-moveit-simple-controller-manager \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-robot-state-publisher
```

### 3. ติดตั้ง Python dependencies

```bash
pip3 install scipy numpy
```

### 4. Clone และ build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/supachai2004/jaka_dual_arm.git
cd ~/ros2_ws
colcon build --packages-select jaka_dual_arm
source install/setup.bash
```

---

## คู่มือเริ่มต้นใช้งานอย่างรวดเร็ว

### โหมดจำลอง (ไม่มีหุ่นยนต์จริง)

```bash
# Terminal 1 — MoveIt2
source ~/ros2_ws/install/setup.bash
ros2 launch jaka_dual_arm dual_arm_moveit.launch.py

# Terminal 2 — Coordinator
source ~/ros2_ws/install/setup.bash
ros2 run jaka_dual_arm coordinator
```

coordinator แสดง `JAKA left/right driver not available — running simulation only` และทำงานต่อตามปกติ

### ใช้งานกับหุ่นยนต์ JAKA จริง

```bash
# Terminal 1 — MoveIt2
source ~/ros2_ws/install/setup.bash
ros2 launch jaka_dual_arm dual_arm_moveit.launch.py

# Terminal 2 — JAKA driver แขนซ้าย (192.168.0.2)
source ~/ros2_ws/install/setup.bash
ros2 run jaka_driver jaka_driver \
  --ros-args -r __ns:=/left -p ip:=192.168.0.2

# Terminal 3 — JAKA driver แขนขวา (192.168.0.1)
source ~/ros2_ws/install/setup.bash
ros2 run jaka_driver jaka_driver \
  --ros-args -r __ns:=/right -p ip:=192.168.0.1

# Terminal 4 — Coordinator
source ~/ros2_ws/install/setup.bash
ros2 run jaka_dual_arm coordinator
```

ตรวจสอบ services ก่อนเปิด coordinator:
```bash
ros2 service list | grep jaka
# คาดหวัง:
# /left/jaka_driver/joint_move
# /right/jaka_driver/joint_move
```

### ส่งคำสั่งเคลื่อนที่

```bash
# เลื่อนวัตถุไป +5 ซม. ในแกน X
ros2 topic pub --once /object_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.05, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

---

## โปรโตคอล TCP Interface

coordinator เปิด TCP server บน **พอร์ต 9090** สำหรับไคลเอนต์ภายนอก

### ประเภทการเชื่อมต่อ

#### ไคลเอนต์หุ่นยนต์
คอนโทรลเลอร์หุ่นยนต์เชื่อมต่อและระบุตัวตน:
```
robot:left\n
robot:right\n
```

#### ไคลเอนต์คำสั่ง
โปรแกรมภายนอกส่งคำสั่ง lift แล้วตัดการเชื่อมต่อ:
```
lift:x,y,z:rx_deg,ry_deg,rz_deg\n
```

**ตัวอย่าง — Python client:**
```python
import socket

with socket.socket() as s:
    s.connect(('192.168.1.100', 9090))
    s.sendall(b'lift:0.45,0.0,1.05:0.0,0.0,15.0\n')
    resp = s.recv(16).decode().strip()
    print(resp)  # 'done' หรือ 'error'
```

---

## แนวทางความปลอดภัย

1. **เคลียร์พื้นที่ทำงาน** ก่อนส่งคำสั่งเคลื่อนที่ทุกครั้ง
2. **ตรวจสอบ init pose** ใน RViz ก่อนรันกับหุ่นยนต์จริง
3. **TCP heartbeat** — หากหุ่นยนต์ตัดการเชื่อมต่อระหว่างการเคลื่อนที่ จะเกิด emergency stop อัตโนมัติ
4. **ลด VEL/ACC** สำหรับการรันครั้งแรกกับน้ำหนักจริง เริ่มที่ 10%
5. **Emergency stop** — กด `Ctrl-C` ที่ coordinator process MoveIt จะค้างไว้ที่ตำแหน่งล่าสุด หมายเหตุ: เมื่อ JAKA drivers ทำงานอยู่ การกด Ctrl-C ไม่หยุดหุ่นยนต์ที่กำลังเคลื่อนที่ ต้องกดปุ่ม E-stop บนกล่องคอนโทรลเลอร์ JAKA
6. **โหมดจำลอง vs หุ่นยนต์จริง** — coordinator แสดง log ว่า JAKA driver พร้อมหรือไม่เมื่อเริ่มต้น หากเห็น `running simulation only` คำสั่งจะไม่ถึงหุ่นยนต์จริง

ดู [docs/SAFETY.md](docs/SAFETY.md) สำหรับขั้นตอนความปลอดภัยเต็มรูปแบบ
