# Quadruped — Controller

PS2 controller reference for the ESP-NOW quadruped controller (FW 1.3.x). General controller for a 12dof quadruped robot. Arduino based with the use of an ESP32 and a PS2 controller.

---

## Startup procedure

1. Power on the robot and position robot's legs to the neutral 0 degree position.
2. Press **Cross** to set the current position as home (zeroes all 12 actuators
   and configures voltage limits and gains).
3. Pick a mode (see below) and go, defaults to **Kinematics Demo** mode.

---

## Mode switching

Mode switches have a ~1 second cooldown between presses.

| Buttons        | Mode                                  |
|----------------|---------------------------------------|
| **L2**         | Kinematics Demo (body pose control)   |
| **R2**         | Crawl (original)                      |
| **R2 + R1**    | Trot (original)                       |
| **R2 + L1**    | Improved Trot                         |
| **L2 + R1**    | Improved Crawl                        |

The current mode is printed over USB serial on every switch (`STATE MACHINE: ...`).

---

## Global buttons (any mode)

| Button       | Action |
|--------------|--------|
| **Cross**    | Set home. Current physical pose becomes zero for all actuators, then sends motor config (`MLV3`, `MLU10`, `MQP1`). 1s cooldown. |
| **Triangle** | Flip. Lowers the body to the ground, inverts leg orientation, and stands back up facing the other way. Blocking sequence — controller input is ignored while it runs. 7s cooldown. |
| **Square**   | Unused. |

---

## Kinematics Demo (body pose control)

**Nothing moves unless L1 or R1 is held.** With both released the robot
holds its last pose.

While holding **L1 or R1**:

| Input                  | Controls                              |
|------------------------|---------------------------------------|
| Left stick up/down     | Body height (Z)                       |
| Left stick left/right  | Yaw (twist body)                      |
| Right stick up/down    | Pitch (nose up/down)                  |
| Right stick left/right | Roll (lean side to side)              |
| D-pad up/down          | Translate body forward/backward (X)   |
| D-pad left/right       | Translate body left/right (Y)         |

While holding **L1 + R1 together**, the D-pad changes function:

| Input          | Controls                                            |
|----------------|-----------------------------------------------------|
| D-pad up/down  | Standing height setpoint changes (`r_state.height`). This is the baseline height used by all modes and will persist, in other words, it changes the working height of the robot. |

Pose limits (from `IK_parameters`): ±50 mm X/Y translation,
±22.5° yaw, ±20° pitch, ±51° roll.

---

## Walking modes — Crawl, Trot, Claude Trot, Claude Crawl

All four walking modes share the same controls. No enable button — the gait
runs whenever stick input exceeds the deadzone and parks itself when the
sticks are centered (legs mid-step finish their step before stopping).

| Input                  | Controls                                        |
|------------------------|-------------------------------------------------|
| Left stick up/down     | Walk forward/backward                           |
| Left stick left/right  | Strafe left/right                               |
| Right stick left/right | Turn (yaw)                                      |
| Right stick up/down    | Body pitch while walking                        |
| D-pad up/down          | Body height, 1.5 mm steps (hold for continuous) |
| D-pad left/right       | Changes the stance width (distance between left and right feet), change is persistent across modes.   |

Notes:

- Changing height or stance width while stopped counts as motion input —
  the robot takes a few steps to resettle its feet. This is by design.
- Max speeds per mode come from the call sites in `d_LOOP.ino` and the
  `Gait_parameters` struct:

| Mode           | Step X | Step Y | Yaw   | Pitch |
|----------------|--------|--------|-------|-------|
| Crawl          | 50 mm  | 30 mm  | 12°   | 12°   |
| Trot           | 50 mm  | 25 mm  | 8°    | 12°   |
| Improved Trot  | 50 mm  | 25 mm  | 8°    | 12°   |
| Improved Crawl | 50 mm  | 30 mm  | 12°   | 12°   |

### What the Improved variants change

Compared to the originals, the improved gaits add:

- **Speed-scaled swing height** — feet lift 25 mm at low speed, up to
  `gait.step_length_z` (70 mm) at full stick, instead of always lifting
  the full height.
- **Touchdown retraction** (`retract = 0.85`) — feet land slightly short
  of full reach for softer, more self-stabilizing touchdowns.
- **Improved Trot only:** swing is 80% of stance duration, creating a brief
  four-foot support window at each diagonal-pair handover.

---

## Serial commands (USB debug, 115200 baud)

Handled by `serialEvent()` in `d_LOOP.ino`. Commands end with newline.

Forwarded verbatim to all three motors of every enabled leg. Examples: `home` (home everything), `MAP0.5` (set P-gain), `MLV3` (set velocity limit).

---

## Quick troubleshooting

- **Robot won't move in Kinematics Demo:** you're not holding L1/R1.
