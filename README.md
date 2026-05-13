# MXEN3005_FinalProject

A ROS2 package (`ourbeloved`) for operation of a **wx250s 6-DOF arm** using a PS4 controller. Supports real-time **joint** and **cartesian** control, autonomous **precise joint positioning** via a PI controller, and a **fire trigger** node that publishes to a fire topic based on controller input.

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
| `std_msgs` | `Bool` message type |
| `numpy` | Numerical operations |
| `joy` (ROS2 pkg) | PS4 controller driver — publishes to `/joy` |
| `robot_state_publisher` | Publishes URDF TF tree for visualisation |
| `joint_state_publisher` | Aggregates joint states for RViz |
| `rviz2` | 3D visualisation |

---

## Condensed Package Structure

```
.
└── mxen_ws
    └── src
        └── ourbeloved
            ├── config
            │   ├── wx250s.rviz
            │   └── wx250s_viz.rviz
            ├── launch
            │   ├── ourbeloved_cannon.xml
            │   └── ourbeloved_controller.xml
            ├── ourbeloved
            │   ├── controller_subscriber.py
            │   ├── fire_node.py
            │   ├── __init__.py
            │   ├── joint_state_node_deg.py
            │   ├── joint_state_node.py
            │   └── py.typed
            ├── package.xml
            ├── setup.cfg
            ├── setup.py
            └── urdf
                └── wx250s_viz.urdf.xacro

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
- `controller_subscriber` — teleoperation node
- `fire_node` — trigger node
- `joint_state_node` — joint state publisher (radians)
- `joint_state_node_deg` — joint state publisher (degrees)

---

### `ourbeloved_controller.xml` — Controller + Visualisation Launch

Starts the PS4 controller driver and RViz visualisation. Run on a separate machine.

```bash
ros2 launch ourbeloved ourbeloved_controller.xml
```

Launches:
- `joy_node` — PS4 controller driver
- `robot_state_publisher` — broadcasts URDF transforms
- `joint_state_publisher` — sources from `/joint_state` topic
- `rviz2` — 3D visualisation

---

## Nodes

### `controller_subscriber`

The primary teleoperation node. Subscribes to the `/joy` topic and translates PS4 inputs into arm commands. Supports two real-time control modes (joint and Cartesian) and a separate autonomous precise-positioning mode driven by an external topic.

**Subscriptions:**
- `/joy` (`sensor_msgs/Joy`) — PS4 controller input
- `/precise_joints` (`sensor_msgs/JointState`) — commanded goal joint positions for autonomous PI-controlled movement

**Timer:** Runs at 20 Hz, continuously sending joint commands to the arm when sticks are active.

---

### `fire_node`

Monitors the R1 button and publishes a rising/falling edge `Bool` to the `/fire` topic. This is used for firing in the minigame.

**Subscriptions:** `/joy`  
**Publishes:** `/fire` (`std_msgs/Bool`)

---

### `joint_state_node`

Reads joint positions from the arm at 30 Hz and publishes them in **radians** for use by `robot_state_publisher` and RViz.

**Publishes:** `/joint_state` (`sensor_msgs/JointState`)

---

### `joint_state_node_deg`

Same as above but publishes in **degrees** — useful for debugging and logging.

**Publishes:** `/joint_state_deg` (`sensor_msgs/JointState`)

---

## Control Scheme

### Joint Control Scheme

> Activated by pressing **Options**. Default mode on startup.

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

> Activated by pressing **Share**.

In Cartesian mode, stick axes are interpreted as **end-effector velocity** in 3D space. The target Cartesian displacement is computed relative to the current end-effector pose using **forward kinematics** (`fk`), then converted to joint angles using **numerical inverse kinematics** (`ik`).

**Stick assignments:**

| Stick | Cartesian Axis |
|---|---|
| Right Stick Y | X (forward/back) |
| Right Stick X | Y (left/right) |
| Left Stick Y | Z (up/down) |

The path from the current pose to the goal is broken into intermediate steps of ≤2mm in each axis before calling IK — this improves numerical IK convergence for larger motions. If any intermediate IK step fails (singularity or unreachable pose), the command is dropped and the arm holds its current position.

Commands run at **10 deg/s** per joint to allow smoother Cartesian paths.

---

### Precise Joint Positioning

An autonomous positioning mode triggered by publishing to `/precise_joints`. Intended for programmatic control from other nodes (e.g. a planning or sequencing node).

```bash
ros2 topic pub --once /precise_joints sensor_msgs/msg/JointState "{position: [j0, j1, j2, j3, j4, j5]}"
```

**Phases:**

1. **Initial** — sends a single `high_acc` command to the goal joint positions.
2. **Moving** — monitors actual joint movement. Once all joints settle (movement < 0.2° over 5 consecutive cycles), transitions to the PI phase.
3. **PI** — applies a **proportional-integral controller** to correct residual error for each joint independently:
   - `Kp = 0.5`, `Ki = 0.2`
4. **Done** — once all joints are within **±0.1°** of goal for 5 consecutive cycles, positioning completes and normal controller input resumes.

If the user moves a stick during precise positioning, the autonomous phase is immediately aborted.

---

### Fire Node

Pressing **R1** publishes `True` on `/fire`. Releasing it publishes `False`. Edge detection prevents repeated triggers from a held button.

---

## ROS2 Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/joy` | `sensor_msgs/Joy` | Subscribed | PS4 controller input |
| `/precise_joints` | `sensor_msgs/JointState` | Subscribed | Autonomous goal joints |
| `/joint_state` | `sensor_msgs/JointState` | Published | Joint positions (radians) |
| `/joint_state_deg` | `sensor_msgs/JointState` | Published | Joint positions (degrees) |
| `/fire` | `std_msgs/Bool` | Published | End-effector fire trigger |
