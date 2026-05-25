# Safety Manual — JAKA Dual-Arm Cooperative Manipulation

**EN** | [TH ภาษาไทย](#คู่มือความปลอดภัย--jaka-dual-arm-cooperative-manipulation)

> **WARNING:** Dual-arm cooperative manipulation involves two industrial robots moving in close proximity. Incorrect operation can cause severe equipment damage and serious personal injury. Read this manual completely before operating the system.

---

## 1. General Safety Rules

### 1.1 Personnel

| Rule | Detail |
|------|--------|
| Trained operators only | Only personnel who have read this manual and completed hands-on training may operate the system |
| No lone operation | At least two people present during initial commissioning and any test with a new configuration |
| Bystander exclusion | No unauthorised personnel within 2 m of the robot workspace during operation |
| Protective equipment | Safety glasses and closed-toe shoes required within 1 m of the robots |

### 1.2 Workspace

| Rule | Detail |
|------|--------|
| Clear workspace | Remove all tools, cables, and loose objects from the robot reach envelope before each run |
| Defined safety zone | Mark a 2 m perimeter around the dual-arm cell with floor tape or a light curtain |
| No reaching in | Never reach into the workspace while the coordinator is running, even if the robots appear stationary |
| Secure mounting | Verify both robot bases are bolted to the mounting frame with correct torque before each session |

### 1.3 Software guards

| Rule | Detail |
|------|--------|
| Simulation first | Always verify a new motion in RViz simulation before executing on real hardware |
| Confirm real-robot mode | Check coordinator startup log: `JAKA left driver ready` / `JAKA right driver ready`. If you see `running simulation only`, the physical robots are NOT receiving commands |
| Low speed first | For any new target configuration use VEL ≤ 0.1, ACC ≤ 0.1 (10%) |
| Incremental testing | Test in small position deltas (≤ 5 cm per command) before attempting large motions |
| Collision object active | Never remove the `payload` collision object from the planning scene while arms are moving |
| Namespace verification | Before starting the coordinator with real robots, confirm `ros2 service list \| grep jaka` shows both `/left/jaka_driver/joint_move` and `/right/jaka_driver/joint_move`. A missing namespace means that arm will not move |

---

## 2. Emergency Stop Procedures

### 2.1 Software emergency stop (coordinator kill)

This is the first action to take for any unexpected behaviour.

```
Step 1 — Go to Terminal 2/4 (coordinator)
Step 2 — Press Ctrl-C
Step 3 — Confirm the node exits (no more log output)
Step 4 — Verify robots are stationary
```

MoveIt's controller will hold the last joint position after the coordinator exits. The robots will **not** fall or drift because ros2_control maintains position control.

> **Important when JAKA drivers are active:** Killing the coordinator does **not** abort a `joint_move` call that is already in-flight. The JAKA driver will continue moving the robot to the commanded position even after the coordinator exits. Use the physical E-stop (Section 2.2) to stop the robot immediately if needed.

### 2.2 Hardware E-stop

If software stop is insufficient or the coordinator cannot be killed:

```
Step 1 — Press the physical E-stop button on the JAKA controller box
Step 2 — Both robot controllers will cut motor power
Step 3 — Support any gripped object manually to prevent it falling
Step 4 — Do NOT release the E-stop until the workspace is clear and the 
          fault has been identified and resolved
```

### 2.3 Network disconnect E-stop (TCP clients)

If a robot TCP client loses its connection to the coordinator:

```
The coordinator detects the disconnect within one heartbeat cycle (≤ 600 ms).
It sends 'stop\n' to all other connected robot clients automatically.
Robot clients must implement the 'stop' command to halt motion immediately.
```

**Robot client must-implement checklist:**
- [ ] Handle `stop\n` message at any point in the motion sequence
- [ ] On `stop`: abort current trajectory, hold current joint position
- [ ] Re-register with `robot:left\n` / `robot:right\n` after the fault is cleared

### 2.4 Power failure

```
Step 1 — Both robots drop to gravity-compensated mode (JAKA default)
Step 2 — Support the gripped object manually
Step 3 — After power is restored, do NOT move the arms until the coordinator
          has re-run _setup_grasp() and re-established the object frame
Step 4 — Restart full system from Terminal 1 (MoveIt) then Terminal 2 (coordinator)
```

---

## 3. Heartbeat System Explanation

The TCP heartbeat prevents the system from continuing motion when a robot client silently dies (e.g., network cable pulled, process crash).

### 3.1 How it works

```
Coordinator (server)                Robot client
      │                                   │
      │── ping\n ─────────────────────► │  (every 100 ms)
      │                                   │
      │◄─ pong\n ────────────────────── │  (must reply within 500 ms)
      │                                   │
      │── ping\n ─────────────────────► │
      │                                   │
      │       [no pong within 500 ms]     │
      │                                   │
      ├── HEARTBEAT TIMEOUT ──────────────┤
      │                                   │
      │── stop\n ─────────────────────► │  (all connected robots)
      │                                   │
      └── EMERGENCY STOP triggered ───────┘
```

### 3.2 Parameters (coordinator.py)

```python
HEARTBEAT_INTERVAL = 0.10   # seconds between pings (100 ms)
HEARTBEAT_TIMEOUT  = 0.50   # seconds to wait for pong (500 ms)
```

### 3.3 What triggers heartbeat timeout

| Cause | Result |
|-------|--------|
| Network cable disconnected | EMERGENCY STOP within ≤ 600 ms |
| Robot client process crashes | EMERGENCY STOP within ≤ 600 ms |
| Robot client OS freezes | EMERGENCY STOP within ≤ 600 ms |
| Network congestion (>500 ms latency) | EMERGENCY STOP — reduce network load |

### 3.4 Implementing pong in your robot client

```python
# Python robot client skeleton
import socket, threading

def robot_client(host, port, side):  # side = 'left' or 'right'
    with socket.create_connection((host, port)) as s:
        s.sendall(f'robot:{side}\n'.encode())

        for line in iter(lambda: readline(s), None):
            if line == 'ping':
                s.sendall(b'pong\n')
            elif line == 'pose':
                # parse and queue trajectory
                pass
            elif line == 'go':
                # execute queued trajectory; send 'done' when finished
                s.sendall(b'done\n')
            elif line == 'stop':
                # ABORT immediately — highest priority
                abort_current_motion()
                break

def readline(sock):
    buf = b''
    while True:
        b = sock.recv(1)
        if not b:
            return None
        buf += b
        if buf.endswith(b'\n'):
            return buf.decode().strip()
```

---

## 4. What To Do When a Robot Stops Unexpectedly

### 4.1 Identify the type of stop

Check the coordinator log output (Terminal 2):

| Log message | Meaning |
|-------------|---------|
| `Heartbeat timeout — robot [left/right]` | Robot client disconnected or network failed |
| `EMERGENCY STOP: heartbeat timeout (left)` | See Section 4.2 |
| `Motion FAILED (planning or execution error)` | MoveIt planning/execution failure, NOT an emergency |
| `IK failed for left/right arm` | Target pose is unreachable, NOT an emergency |
| `TCP: ready timeout` | Robot did not reach ready state in time |
| `TCP: done timeout` | Robot did not complete motion in time |

### 4.2 Heartbeat-triggered emergency stop recovery

```
1. Do NOT approach the robot immediately
2. Verify all motion has fully stopped (watch for residual movement)
3. Identify the disconnected robot (log says left or right)
4. Check the robot client on that machine for errors
5. Fix the root cause (network, process crash, etc.)
6. Press Ctrl-C on the coordinator (Terminal 2) to fully reset
7. Restart: ros2 run jaka_dual_arm coordinator
8. Reconnect robot clients after coordinator prints "TCP server ready on :9090"
```

### 4.3 Planning failure recovery

A planning failure (IK failed, Motion FAILED) is **not** an emergency stop. The robots remain stationary at their last position.

```
1. Check coordinator log for the specific error
2. Verify the target pose is within the workspace
3. If collision detected: check RViz for red links, adjust the target
4. Send a corrected target via /object_target
5. The coordinator will retry automatically
```

### 4.4 After any unexpected stop — pre-restart checklist

- [ ] Establish why the stop occurred — do not restart without understanding the cause
- [ ] Verify both robots are physically stationary
- [ ] Verify the gripped object is secure (or supported manually)
- [ ] Clear any fault LEDs on the JAKA controller boxes
- [ ] Verify the workspace is clear of personnel
- [ ] Restart coordinator: `ros2 run jaka_dual_arm coordinator`
- [ ] Confirm "Ready!" message before sending any new commands
- [ ] First command after restart: move back to the init pose (small delta)

---

## 5. Workspace Limits and Joint Limits

### 5.1 Joint limits (from joint_limits.yaml)

The OMPL planner and ros2_control both enforce these. Do not exceed them in INIT_LEFT / INIT_RIGHT or target poses.

```
All joints: ±360° (±6.283 rad) unless further restricted in joint_limits.yaml
Velocity:   Max defined per joint — coordinator uses VEL=0.3 scaling (30%)
```

### 5.2 Cartesian workspace guideline for JAKA A12

| Axis | Safe working range from base |
|------|------------------------------|
| Reach | 0.30 m – 1.15 m radial |
| Height | -0.50 m – +1.30 m (Z) |
| Note | Avoid fully-extended (singularity) configurations |

### 5.3 Grip span constraint

The two flanges are connected to the same rigid object. Moving the object must not require either arm to exceed its workspace. Keep target positions within:
- ±0.20 m from the init object position in any single move
- Rotations ≤ 90° from the init orientation per command

---

## 6. Pre-Operation Safety Checklist

Complete before every session:

**Hardware:**
- [ ] Both robot bases bolted securely
- [ ] All cables routed clear of the motion envelope
- [ ] No personnel or tools inside the 2 m safety zone
- [ ] E-stop button tested and functional — physically press and release before each session
- [ ] Both JAKA controllers powered on, no fault LEDs
- [ ] Both robot controllers reachable: `ping 192.168.0.2` and `ping 192.168.0.1` reply

**Software — simulation mode:**
- [ ] ROS2 environment sourced correctly
- [ ] `dual_arm_moveit.launch.py` running, RViz shows robot model
- [ ] `coordinator` running, "Ready!" message confirmed
- [ ] (TCP) Robot clients connected and heartbeat confirmed in log

**Software — real robot mode (additional checks):**
- [ ] Both `jaka_driver` nodes started with correct namespaces (`/left`, `/right`) and IP addresses
- [ ] `ros2 service list | grep jaka` shows `/left/jaka_driver/joint_move` **and** `/right/jaka_driver/joint_move`
- [ ] Coordinator startup log shows `JAKA left driver ready` **and** `JAKA right driver ready` — NOT `running simulation only`
- [ ] First test motion verified in RViz simulation before executing on real hardware
- [ ] VEL and ACC set to ≤ 10% for initial session
- [ ] A second person present to monitor robot motion and operate E-stop if needed

**Payload:**
- [ ] Object mass ≤ 12 kg (within per-arm limit)
- [ ] Object securely gripped by both flanges (or grippers)
- [ ] Collision object `payload` visible in RViz planning scene

---

---

# คู่มือความปลอดภัย — JAKA Dual-Arm Cooperative Manipulation

> **คำเตือน:** การควบคุมแขนหุ่นยนต์สองแขนแบบร่วมมือเกี่ยวข้องกับหุ่นยนต์อุตสาหกรรมสองตัวที่เคลื่อนที่ใกล้กัน การใช้งานที่ไม่ถูกต้องอาจทำให้อุปกรณ์เสียหายอย่างรุนแรงและเกิดอันตรายต่อร่างกาย อ่านคู่มือนี้ทั้งหมดก่อนใช้งานระบบ

---

## 1. กฎความปลอดภัยทั่วไป

### 1.1 บุคลากร

| กฎ | รายละเอียด |
|----|-----------|
| เฉพาะผู้ปฏิบัติงานที่ผ่านการฝึกอบรม | เฉพาะผู้ที่อ่านคู่มือนี้และผ่านการฝึกอบรมเท่านั้น |
| ไม่ปฏิบัติงานคนเดียว | ต้องมีอย่างน้อย 2 คนในช่วง commissioning และการทดสอบ configuration ใหม่ |
| กันคนออกจากพื้นที่ | ไม่อนุญาตให้ผู้ไม่เกี่ยวข้องอยู่ภายใน 2 เมตรจากพื้นที่ทำงานขณะปฏิบัติงาน |
| อุปกรณ์ป้องกัน | สวมแว่นตานิรภัยและรองเท้าหัวปิดภายใน 1 เมตรจากหุ่นยนต์ |

### 1.2 พื้นที่ทำงาน

| กฎ | รายละเอียด |
|----|-----------|
| พื้นที่ว่าง | ขจัดเครื่องมือ สาย และสิ่งของหลวมออกจากพื้นที่เอื้อมถึงของหุ่นยนต์ก่อนทุกครั้ง |
| เขตปลอดภัยที่กำหนด | ทำเครื่องหมายรอบเซลล์แขนคู่ด้วยเทปพื้นหรือ light curtain |
| ห้ามยื่นมือเข้า | อย่ายื่นมือเข้าไปในพื้นที่ทำงานขณะที่ coordinator ทำงานอยู่ แม้หุ่นยนต์จะดูนิ่งอยู่ |
| ยึดฐานให้มั่น | ตรวจสอบว่าฐานหุ่นยนต์ทั้งสองยึดด้วยโบลต์ที่แรงบิดที่ถูกต้องก่อนทุก session |

### 1.3 การป้องกันด้านซอฟต์แวร์

| กฎ | รายละเอียด |
|----|-----------|
| จำลองก่อนเสมอ | ตรวจสอบการเคลื่อนที่ใหม่ใน RViz ก่อนรันบนฮาร์ดแวร์จริงเสมอ |
| ยืนยันโหมดหุ่นยนต์จริง | ตรวจสอบ log เริ่มต้น: `JAKA left driver ready` / `JAKA right driver ready` หากเห็น `running simulation only` หุ่นยนต์จริงไม่รับคำสั่ง |
| ความเร็วต่ำก่อน | ใช้ VEL ≤ 0.1, ACC ≤ 0.1 (10%) สำหรับ configuration ใหม่ |
| ทดสอบแบบค่อยเป็นค่อยไป | ทดสอบด้วย delta ตำแหน่งเล็กน้อย (≤ 5 ซม. ต่อคำสั่ง) |
| Collision object ต้องใช้งานอยู่ | อย่าลบ collision object `payload` ออกจาก planning scene ขณะแขนเคลื่อนที่ |
| ตรวจสอบ namespace | ก่อนเปิด coordinator กับหุ่นยนต์จริง ยืนยันว่า `ros2 service list \| grep jaka` แสดง `/left/jaka_driver/joint_move` และ `/right/jaka_driver/joint_move` |

---

## 2. ขั้นตอนการหยุดฉุกเฉิน

### 2.1 หยุดฉุกเฉินด้วยซอฟต์แวร์ (kill coordinator)

นี่คือการดำเนินการแรกสำหรับพฤติกรรมที่ไม่คาดคิด

```
ขั้นตอนที่ 1 — ไปที่ Terminal 2/4 (coordinator)
ขั้นตอนที่ 2 — กด Ctrl-C
ขั้นตอนที่ 3 — ยืนยันว่าโหนดออก (ไม่มี log output อีกต่อไป)
ขั้นตอนที่ 4 — ตรวจสอบว่าหุ่นยนต์นิ่ง
```

MoveIt's controller จะค้างไว้ที่ตำแหน่งข้อต่อสุดท้ายหลัง coordinator ออก หุ่นยนต์จะ **ไม่** ตก ros2_control ยังคง position control ไว้

> **สำคัญเมื่อ JAKA drivers ทำงานอยู่:** การกด Ctrl-C จะ **ไม่** หยุด `joint_move` call ที่กำลังดำเนินอยู่ JAKA driver จะเคลื่อนหุ่นยนต์ต่อไปจนถึงตำแหน่งที่สั่งไว้แม้ coordinator จะปิดแล้ว ใช้ E-stop ฮาร์ดแวร์ (ส่วน 2.2) เพื่อหยุดทันที

### 2.2 E-stop ฮาร์ดแวร์

หากการหยุดด้วยซอฟต์แวร์ไม่เพียงพอ:

```
ขั้นตอนที่ 1 — กดปุ่ม E-stop ที่กล่องคอนโทรลเลอร์ JAKA
ขั้นตอนที่ 2 — คอนโทรลเลอร์ทั้งสองจะตัดกำลังมอเตอร์
ขั้นตอนที่ 3 — รองรับวัตถุที่จับไว้ด้วยมือเพื่อป้องกันการตก
ขั้นตอนที่ 4 — อย่าปล่อย E-stop จนกว่าพื้นที่จะปลอดภัย
               และระบุสาเหตุของปัญหาได้แล้ว
```

### 2.3 การตัดการเชื่อมต่อเครือข่าย E-stop (TCP clients)

หากไคลเอนต์หุ่นยนต์ TCP ขาดการเชื่อมต่อ:

```
coordinator ตรวจพบการตัดการเชื่อมต่อภายในหนึ่ง heartbeat cycle (≤ 600 มิลลิวินาที)
ส่ง 'stop\n' ไปยังไคลเอนต์หุ่นยนต์ที่เชื่อมต่ออยู่ทั้งหมดโดยอัตโนมัติ
ไคลเอนต์หุ่นยนต์ต้องใช้คำสั่ง 'stop' เพื่อหยุดการเคลื่อนที่ทันที
```

### 2.4 ไฟดับ

```
ขั้นตอนที่ 1 — หุ่นยนต์ทั้งสองเข้าสู่โหมด gravity-compensated (ค่าเริ่มต้น JAKA)
ขั้นตอนที่ 2 — รองรับวัตถุที่จับไว้ด้วยมือ
ขั้นตอนที่ 3 — หลังไฟกลับมา อย่าขยับแขนจนกว่า coordinator
               จะรัน _setup_grasp() ใหม่และสร้าง object frame ใหม่
ขั้นตอนที่ 4 — รีสตาร์ทระบบทั้งหมด: Terminal 1 (MoveIt) แล้ว Terminal 2 (coordinator)
```

---

## 3. ระบบ Heartbeat อธิบาย

TCP heartbeat ป้องกันไม่ให้ระบบดำเนินการเคลื่อนที่ต่อเมื่อไคลเอนต์หุ่นยนต์หายไปโดยไม่มีสัญญาณ

### 3.1 วิธีการทำงาน

```
Coordinator (server)                ไคลเอนต์หุ่นยนต์
      │                                    │
      │── ping\n ──────────────────────► │  (ทุก 100 มิลลิวินาที)
      │                                    │
      │◄─ pong\n ─────────────────────── │  (ต้องตอบภายใน 500 มิลลิวินาที)
      │                                    │
      │       [ไม่มี pong ภายใน 500 มิลลิวินาที]
      │                                    │
      ├── HEARTBEAT TIMEOUT ───────────────┤
      │                                    │
      │── stop\n ──────────────────────► │  (หุ่นยนต์ทั้งหมดที่เชื่อมต่ออยู่)
      │                                    │
      └── EMERGENCY STOP เปิดใช้งาน ───────┘
```

### 3.2 พารามิเตอร์ (coordinator.py)

```python
HEARTBEAT_INTERVAL = 0.10   # วินาทีระหว่าง ping (100 มิลลิวินาที)
HEARTBEAT_TIMEOUT  = 0.50   # วินาทีรอ pong (500 มิลลิวินาที)
```

### 3.3 สิ่งที่ทำให้เกิด heartbeat timeout

| สาเหตุ | ผลลัพธ์ |
|--------|--------|
| ถอดสายเครือข่าย | EMERGENCY STOP ภายใน ≤ 600 มิลลิวินาที |
| process ไคลเอนต์หยุดทำงาน | EMERGENCY STOP ภายใน ≤ 600 มิลลิวินาที |
| OS ไคลเอนต์ค้าง | EMERGENCY STOP ภายใน ≤ 600 มิลลิวินาที |
| network congestion (> 500 มิลลิวินาที latency) | EMERGENCY STOP — ลดภาระเครือข่าย |

### 3.4 การใช้งาน pong ในไคลเอนต์หุ่นยนต์

```python
# Python robot client skeleton
import socket

def robot_client(host, port, side):  # side = 'left' หรือ 'right'
    with socket.create_connection((host, port)) as s:
        s.sendall(f'robot:{side}\n'.encode())

        for line in iter(lambda: readline(s), None):
            if line == 'ping':
                s.sendall(b'pong\n')
            elif line == 'go':
                # ดำเนิน trajectory; ส่ง 'done' เมื่อเสร็จ
                s.sendall(b'done\n')
            elif line == 'stop':
                # ยกเลิกทันที — ความสำคัญสูงสุด
                abort_current_motion()
                break
```

---

## 4. สิ่งที่ต้องทำเมื่อหุ่นยนต์หยุดโดยไม่คาดคิด

### 4.1 ระบุประเภทการหยุด

ตรวจสอบ log output ของ coordinator (Terminal 2):

| ข้อความ log | ความหมาย |
|------------|---------|
| `Heartbeat timeout — robot [left/right]` | ไคลเอนต์หุ่นยนต์ตัดการเชื่อมต่อหรือเครือข่ายล้มเหลว |
| `EMERGENCY STOP: heartbeat timeout` | ดูหัวข้อ 4.2 |
| `Motion FAILED (planning or execution error)` | ความล้มเหลวของการวางแผน/ดำเนินการ MoveIt ไม่ใช่ฉุกเฉิน |
| `IK failed for left/right arm` | ท่าเป้าหมายไม่สามารถเข้าถึงได้ ไม่ใช่ฉุกเฉิน |

### 4.2 การฟื้นตัวจาก heartbeat emergency stop

```
1. อย่าเข้าใกล้หุ่นยนต์ทันที
2. ตรวจสอบว่าการเคลื่อนที่หยุดสมบูรณ์แล้ว
3. ระบุหุ่นยนต์ที่ตัดการเชื่อมต่อ (log บอกว่าซ้ายหรือขวา)
4. ตรวจสอบ log ไคลเอนต์หุ่นยนต์ในเครื่องนั้น
5. แก้ไขสาเหตุหลัก (เครือข่าย, process crash, ฯลฯ)
6. กด Ctrl-C ที่ coordinator (Terminal 2) เพื่อ reset อย่างสมบูรณ์
7. รีสตาร์ท: ros2 run jaka_dual_arm coordinator
8. เชื่อมต่อไคลเอนต์หุ่นยนต์ใหม่หลัง coordinator พิมพ์ "TCP server ready on :9090"
```

### 4.3 การฟื้นตัวจากความล้มเหลวในการวางแผน

ความล้มเหลวในการวางแผน (IK failed, Motion FAILED) **ไม่ใช่** emergency stop หุ่นยนต์จะอยู่นิ่งที่ตำแหน่งล่าสุด

```
1. ตรวจสอบ log ของ coordinator สำหรับข้อผิดพลาดเฉพาะ
2. ตรวจสอบว่าตำแหน่งเป้าหมายอยู่ในพื้นที่ทำงาน
3. หากตรวจพบการชน: ตรวจสอบ RViz หา link สีแดง ปรับเป้าหมาย
4. ส่งเป้าหมายที่แก้ไขแล้วผ่าน /object_target
```

### 4.4 Checklist ก่อนรีสตาร์ทหลังหยุดโดยไม่คาดคิด

- [ ] ระบุสาเหตุของการหยุด — อย่ารีสตาร์ทโดยไม่เข้าใจสาเหตุ
- [ ] ตรวจสอบว่าหุ่นยนต์ทั้งสองนิ่งสนิท
- [ ] ตรวจสอบว่าวัตถุที่จับอยู่ปลอดภัย
- [ ] ล้าง fault LEDs บนกล่องคอนโทรลเลอร์ JAKA
- [ ] ตรวจสอบว่าพื้นที่ปลอดบุคลากร
- [ ] รีสตาร์ท coordinator: `ros2 run jaka_dual_arm coordinator`
- [ ] ยืนยัน "Ready!" ก่อนส่งคำสั่งใหม่
- [ ] คำสั่งแรกหลังรีสตาร์ท: กลับไปที่ init pose (delta เล็กน้อย)

---

## 5. ขีดจำกัดพื้นที่ทำงานและข้อต่อ

### 5.1 ขีดจำกัดข้อต่อ (จาก joint_limits.yaml)

ทั้ง OMPL planner และ ros2_control บังคับใช้ขีดจำกัดเหล่านี้

### 5.2 แนวทางพื้นที่ทำงาน Cartesian สำหรับ JAKA A12

| แกน | ช่วงการทำงานที่ปลอดภัยจากฐาน |
|-----|--------------------------|
| รัศมีเอื้อม | 0.30 ม. – 1.15 ม. |
| ความสูง | -0.50 ม. – +1.30 ม. (Z) |
| หมายเหตุ | หลีกเลี่ยง configuration ที่ยืดออกเต็มที่ (singularity) |

### 5.3 ข้อจำกัด grip span

หน้าแปลนทั้งสองเชื่อมต่อกับวัตถุแข็งชิ้นเดียวกัน รักษาตำแหน่งเป้าหมายไว้ที่:
- ±0.20 ม. จากตำแหน่งวัตถุ init ในการเคลื่อนที่ครั้งเดียว
- การหมุน ≤ 90° จาก init orientation ต่อคำสั่ง

---

## 6. Checklist ความปลอดภัยก่อนปฏิบัติงาน

ทำก่อนทุก session:

**ฮาร์ดแวร์:**
- [ ] ฐานหุ่นยนต์ทั้งสองยึดอย่างแน่นหนา
- [ ] สายทุกเส้นวางอยู่นอกพื้นที่การเคลื่อนที่
- [ ] ไม่มีบุคลากรหรือเครื่องมือในเขตปลอดภัย 2 เมตร
- [ ] ปุ่ม E-stop ทดสอบแล้วและทำงานได้ — กดและปล่อยก่อนทุก session
- [ ] คอนโทรลเลอร์ JAKA ทั้งสองเปิดแล้ว ไม่มีไฟ fault
- [ ] `ping 192.168.0.2` และ `ping 192.168.0.1` ตอบสนอง

**ซอฟต์แวร์ — โหมดจำลอง:**
- [ ] source ROS2 environment ถูกต้อง
- [ ] `dual_arm_moveit.launch.py` ทำงาน RViz แสดงโมเดลหุ่นยนต์
- [ ] `coordinator` ทำงาน ยืนยันข้อความ "Ready!"
- [ ] (TCP) ไคลเอนต์หุ่นยนต์เชื่อมต่อและ heartbeat ยืนยันใน log

**ซอฟต์แวร์ — โหมดหุ่นยนต์จริง (เพิ่มเติม):**
- [ ] `jaka_driver` ทั้งสองตัวเปิดด้วย namespace ที่ถูกต้อง (`/left`, `/right`) และ IP ที่ถูกต้อง
- [ ] `ros2 service list | grep jaka` แสดง `/left/jaka_driver/joint_move` **และ** `/right/jaka_driver/joint_move`
- [ ] coordinator แสดง `JAKA left driver ready` **และ** `JAKA right driver ready` — ไม่ใช่ `running simulation only`
- [ ] ตรวจสอบการเคลื่อนที่ครั้งแรกใน RViz ก่อนดำเนินการกับหุ่นยนต์จริง
- [ ] VEL และ ACC ตั้งไว้ที่ ≤ 10% สำหรับ session แรก
- [ ] มีบุคคลที่สองคอยดูหุ่นยนต์และพร้อมกด E-stop

**Payload:**
- [ ] น้ำหนักวัตถุ ≤ 12 กก. (ภายในขีดจำกัดต่อแขน)
- [ ] วัตถุจับอย่างแน่นหนาโดยหน้าแปลนทั้งสอง
- [ ] Collision object `payload` มองเห็นได้ใน RViz planning scene
