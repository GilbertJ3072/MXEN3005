# MXEN3005_FinalProject

A ROS2 package (`ourbeloved`) for teleoperation of a **Interbotix wx250s 6-DOF robotic arm** using a PS4 DualShock controller. Supports real-time **joint-space** and **Cartesian-space** control, autonomous **precise joint positioning** via a PI controller, and a **fire trigger** output for an attached end-effector.

---

## Table of Contents

- [Dependencies](#dependencies)
- [Package Structure](#package-structure)
- [Launch Files](#launch-files)
- [Nodes](#nodes)
- [Control Scheme](#control-scheme)
  - [Joint Control Mode](#joint-control-mode)
  - [Cartesian Control Mode](#cartesian-control-mode)
  - [Precise Joint Positioning](#precise-joint-positioning)
  - [Fire Node](#fire-node)
- [ROS2 Topics](#ros2-topics)
- [Known Issues / Notes](#known-issues--notes)

---

## Dependencies

| Dependency | Purpose |
|---|---|
| `rclpy` | ROS2 Python client library |
| `xarmclient` | Hardware interface for the wx250s arm |
| `wx250s_kinematics` | Forward (`fk`) and inverse (`ik`) kinematics for the wx250s |
| `sensor_msgs` | `Joy`, `JointState` message types |
| `std_msgs` | `Bool`, `String` message types |
| `numpy` | Numerical operations |
| `joy` (ROS2 pkg) | PS4 controller driver — publishes to `/joy` |
| `robot_state_publisher` | Publishes URDF TF tree for visualisation |
| `joint_state_publisher` | Aggregates joint states for RViz |
| `rviz2` | 3D visualisation |

---

## Package Structure

```
ourbeloved/
├── ourbeloved/
│   ├── __init__.py
│   ├── controller_subscriber.py   # Main teleoperation node (joint & Cartesian control)
│   ├── fire_node.py               # End-effector fire trigger node
│   ├── joint_state_node.py        # Publishes joint states in radians
│   └── joint_state_node_deg.py    # Publishes joint states in degrees
├── launch/
│   ├── ourbeloved_cannon.xml      # Main arm launch file
│   └── ourbeloved_controller.xml  # Controller + RViz visualisation launch file
└── urdf/
    └── wx250s_viz.urdf.xacro      # Robot URDF (required for visualisation)
```

---

## Launch Files

### `ourbeloved_cannon.xml` — Main Arm Launch

Starts the arm server and all control nodes. Run this on the machine connected to the arm.

```bash
ros2 launch ourbeloved ourbeloved_cannon.xml
```

Launches:
- `xarmserver` — hardware server for the wx250s
- `controller_subscriber` — main teleoperation node
- `fire_node` — end-effector trigger node
- `joint_state_node` — joint state publisher (radians)
- `joint_state_node_deg` — joint state publisher (degrees)

---

### `ourbeloved_controller.xml` — Controller + Visualisation Launch

Starts the PS4 controller driver and RViz visualisation. Can be run on a separate machine.

```bash
ros2 launch ourbeloved ourbeloved_controller.xml
# or with a custom URDF filename:
ros2 launch ourbeloved ourbeloved_controller.xml filename:=wx250s_viz
```

Launches:
- `joy_node` — PS4 controller driver
- `robot_state_publisher` — broadcasts URDF transforms
- `joint_state_publisher` — sources from `/joint_state` topic
- `rviz2` — 3D visualisation

> ⚠️ **Note:** The `joy_node` launch configuration may require tweaking depending on your system's joystick device path. Check `joy_node` parameters if the controller is not detected.

---

## Nodes

### `controller_subscriber`

The primary teleoperation node. Subscribes to the `/joy` topic and translates PS4 inputs into arm commands. Supports two real-time control modes (joint and Cartesian) and a separate autonomous precise-positioning mode driven by an external topic.

**Subscriptions:**
- `/joy` (`sensor_msgs/Joy`) — PS4 controller input
- `/precise_joints` (`sensor_msgs/JointState`) — commanded goal joint positions for autonomous PI-controlled movement

**Timer:** Runs at 20 Hz (every 50 ms), continuously sending joint commands to the arm when sticks are active.

---

### `fire_node`

Monitors the R1 button and publishes a rising/falling edge `Bool` to the `/fire` topic. Designed for an attached end-effector (e.g. a cannon or pneumatic actuator).

**Subscriptions:** `/joy`  
**Publishes:** `/fire` (`std_msgs/Bool`)

---

### `joint_state_node`

Reads joint positions from the arm at ~30 Hz and publishes them in **radians** for use by `robot_state_publisher` and RViz.

**Publishes:** `/joint_state` (`sensor_msgs/JointState`)

---

### `joint_state_node_deg`

Same as above but publishes in **degrees** — useful for debugging and logging.

**Publishes:** `/joint_state_deg` (`sensor_msgs/JointState`)

---

## Control Scheme

### PS4 Controller Map

| Input | Assignment |
|---|---|
| Left Stick X (`axes[0]`) | Joint 0 (base rotation) |
| Left Stick Y (`axes[1]`) | Joint 1 / Cartesian Z |
| Right Stick X (`axes[3]`) | Joint 3 / Cartesian X |
| Right Stick Y (`axes[4]`) | Joint 2 / Cartesian Y |
| D-Pad X (`axes[6]`) | Joint 5 (wrist roll) |
| D-Pad Y (`axes[7]`) | Joint 4 (wrist pitch) |
| **Share** (`buttons[8]`) | Switch to **Cartesian Mode** |
| **Options** (`buttons[9]`) | Switch to **Joint Mode** |
| **L3 / Home** (`buttons[10]`) | Home arm |
| **Square** (`buttons[3]`) | Rest arm |
| **R1** (`buttons[5]`) | Fire end-effector |

---

### Joint Control Mode

> Activated by pressing **Options** (`buttons[9]`). Default mode on startup.

Each analog stick axis directly offsets one joint angle. The commanded joint position is continuously accumulated — holding a stick drives the joint at a steady rate, while releasing it stops motion.

**Joint assignments:**

| Stick / D-Pad | Joint |
|---|---|
| Left Stick X | Joint 0 — base yaw |
| Left Stick Y | Joint 1 — shoulder |
| Right Stick Y | Joint 2 — elbow |
| Right Stick X | Joint 3 — forearm roll |
| D-Pad Y | Joint 4 — wrist pitch |
| D-Pad X | Joint 5 — wrist roll |

Commands are sent with `motion_mode='high_acc'` at **40 deg/s** per joint. Goal validity is checked before applying any new joint target — invalid goals (out-of-range or self-collision) are silently rejected.

When no stick input is detected, the node reads back the arm's actual position and uses that as the new reference, preventing drift accumulation.

---

### Cartesian Control Mode

> Activated by pressing **Share** (`buttons[8]`).

In Cartesian mode, stick axes are interpreted as **end-effector velocity** in 3D space. The target Cartesian displacement is computed relative to the current end-effector pose using **forward kinematics** (`fk`), then converted to joint angles using **numerical inverse kinematics** (`ik`).

**Stick assignments:**

| Stick | Cartesian Axis |
|---|---|
| Right Stick Y (`axes[4]`) | X (forward/back) |
| Right Stick X (`axes[3]`) | Y (left/right) |
| Left Stick Y (`axes[1]`) | Z (up/down) |

The path from the current pose to the goal is broken into intermediate steps of ≤2mm in each axis before calling IK — this improves numerical IK convergence for larger motions. If any intermediate IK step fails (singularity or unreachable pose), the command is dropped and the arm holds its current position.

Commands run at **10 deg/s** per joint to allow smoother Cartesian paths.

---

### Precise Joint Positioning

An autonomous positioning mode triggered by publishing to `/precise_joints`. Intended for programmatic control from other nodes (e.g. a planning or sequencing node).

**Phases:**

1. **Initial** — sends a single `high_acc` command to the goal joint positions.
2. **Moving** — monitors actual joint movement. Once all joints settle (movement < 0.2° over 5 consecutive cycles), transitions to the PI phase.
3. **PI** — applies a **proportional-integral controller** to correct residual error for each joint independently:
   - `Kp = 0.5`, `Ki = 0.2`
   - Integral is clamped to ±50° to prevent windup
   - Corrections are applied as an offset from the goal position
4. **Done** — once all joints are within **±0.1°** of goal for 5 consecutive cycles, positioning completes and normal controller input resumes.

If the user moves a stick during precise positioning, the autonomous phase is immediately aborted.

---

### Fire Node

Pressing **R1** (`buttons[5]`) publishes `True` on `/fire`. Releasing it publishes `False`. Edge detection prevents repeated triggers from a held button.

---

## ROS2 Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/joy` | `sensor_msgs/Joy` | Subscribed | PS4 controller input |
| `/precise_joints` | `sensor_msgs/JointState` | Subscribed | Autonomous goal joints |
| `/joint_state` | `sensor_msgs/JointState` | Published | Joint positions (radians) |
| `/joint_state_deg` | `sensor_msgs/JointState` | Published | Joint positions (degrees) |
| `/fire` | `std_msgs/Bool` | Published | End-effector fire trigger |

---

## Known Issues / Notes

- **`joy_node` device path** — depending on your Linux system, the PS4 controller may not be detected automatically. You may need to pass `device:=/dev/input/jsX` to `joy_node` in `ourbeloved_controller.xml`.
- **Cartesian mode IK** — the IK solver operates on a zeroed reference frame (`[0,0,0,0,0,0]`) as the base for FK, not the arm's actual current joint state. This is intentional for incremental Cartesian deltas but means orientation is not preserved during Cartesian motion.
- **`xarmserver`** — the arm hardware server must be running before any nodes attempt to connect via `XArm()`. The launch file starts it first, but a short startup delay may occasionally cause connection errors on slower systems.
- **Gripper control** — gripper toggle code exists in `controller_subscriber.py` but is currently commented out. Uncomment and assign to a button as needed.
