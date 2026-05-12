# Franka Panda Operational Space Controller (MuJoCo)

A C++ simulation of a **Franka Emika Panda** 7-DOF robot arm performing a pick-and-place task, built on MuJoCo 3.0.1 with OpenGL/GLFW rendering and Eigen for linear algebra.

## Overview

The project implements **operational space control** — torques are computed in Cartesian space and mapped back to joint torques using the robot's dynamics. A state machine drives the robot through a full pick-and-place cycle: approach, grasp, lift, move, and release.

## Dependencies

- [MuJoCo 3.0.1](https://mujoco.org/) — physics engine
- [GLFW](https://www.glfw.org/) — window/OpenGL context (`libglfw3-dev`)
- [Eigen3](https://eigen.tuxfamily.org/) — matrix/linear algebra
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) — Franka Panda MJCF model (`franka_emika_panda/scene2.xml`)

## Build

```bash
make
```

This produces the `controller` binary. The `basic` binary (a simpler joint-lock demo) can be built by uncommenting its line in the Makefile.

## Running

```bash
./controller
```

The simulation loads `scene2.xml` from the MuJoCo Menagerie Panda model. Press **Backspace** to reset the simulation to its initial state.

### Camera Controls

| Input | Action |
|-------|--------|
| Left drag | Rotate view |
| Right drag | Pan view |
| Scroll | Zoom |
| Shift + drag | Horizontal move/rotate |

## Project Structure

```
andres/
├── controller.cpp   # Main simulation: state machine + control loop
├── robot.h          # Model class interface
├── robot.cpp        # Model class implementation
├── basic.cc         # Minimal demo (joint-lock controller)
└── Makefile
```

## Architecture

### `Model` class ([robot.h](robot.h) / [robot.cpp](robot.cpp))

Wraps all robot kinematics and dynamics state. Updated each control cycle:

| Matrix/Vector | Dimensions | Description |
|---------------|-----------|-------------|
| `M` | 7×7 | Joint-space mass matrix |
| `J` | 6×7 | Body Jacobian (linear + angular rows) |
| `L` | 6×6 | End-effector mass matrix in Cartesian space |
| `Jbar` | 7×6 | Dynamically consistent pseudoinverse of J |
| `Nbar` | 7×7 | Nullspace projection matrix |
| `g` | 7×1 | Gravity/Coriolis compensation vector |
| `q` / `dq` | 7×1 | Joint positions / velocities |
| `x` | 3×1 | End-effector Cartesian position |
| `dx` | 6×1 | End-effector Cartesian velocity (linear + angular) |
| `quat` | — | End-effector orientation (quaternion) |

Key derived quantities:
```
L    = (J * M⁻¹ * Jᵀ)⁻¹
Jbar = M⁻¹ * Jᵀ * L
Nbar = I - Jbar * J
```

### Controller ([controller.cpp](controller.cpp))

Runs at **1 kHz** inside MuJoCo's `mjcb_control` callback. Two control modes:

**Type 1 — Operational Space Control (default)**

Computes a desired Cartesian velocity from position and orientation error, applies a velocity saturation limit `nu`, then maps to joint torques:

```
xdot_d = -(kp/kd) * [x - xd ; -log(qd * q⁻¹)]
nu     = min(1, Vmax / ||xdot_d||)
Fx     = kd * (nu * xdot_d - dx)
tau    = Jᵀ * L * Fx + g + Nbarᵀ * M * (-kd * dq)
```

The nullspace term keeps unactuated joint-space motion damped without affecting the end-effector task.

**Type 0 — Joint Space Control**

PD control directly on joint angles:
```
Fq  = kp * (qd - q) - kd * dq
tau = M * Fq + g + Nbarᵀ * M * (-kd * dq)
```

### Pick-and-Place State Machine

| State | Action |
|-------|--------|
| 1 | Move end-effector above cube at (0.5, 0.0, 0.21), gripper open |
| 2 | Close gripper, hold position for ~0.3 s |
| 3 | Lift to (0.5, 0.0, 0.51) with gripper closed |
| 4 | Move to drop location (0.5, 0.3, 0.21), open gripper |
| 5 (default) | Return to raised position, reset cube pose, loop back to state 1 |

## Control Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `kpx` | 200 | Proportional gain |
| `kdx` | 200 | Derivative gain |
| `Vmax` | 0.3 m/s | End-effector velocity limit |
| `control_frequency` | 1000 Hz | Controller update rate |
| `offset` | 0.0584 m | Gripper-to-EE z-offset |
