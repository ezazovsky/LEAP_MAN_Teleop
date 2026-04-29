# Coordinate Frames in RM65 Teleoperation (Actual Implementation)

## Overview

This document describes what the code currently does in [src/realman_utils.py](src/realman_utils.py) and [src/teleop_recorder.py](src/teleop_recorder.py).

Key point: the teleoperation controller is **delta-from-home**, not absolute world-position control.

## Delta Terminology (Important)

- Home-referenced delta: transform from tracker home pose to current tracker pose.
  - This is the primary control signal used to build target_pose.
- Per-cycle step delta: difference between this cycle's target_pose and last_filtered_pose.
  - This is only used inside safety filtering for jump/rate limiting.

When this document says delta, it means the home-referenced delta unless explicitly labeled per-cycle step delta.

## Frames That Matter in Runtime

### 1. SteamVR world frame

- Produced by OpenVR as a 4x4 pose from tracker.get_T().
- Defined by the Lighthouse setup (tracking space).
- Used as an intermediate source frame.

### 2. Tracker home frame anchor

- Captured once at teleop calibration/start:
  - tracker_home_T = get_current_tracker_matrix().copy()
- This is the reference for all later tracker motion.

### 3. Robot base frame

- Robot Cartesian commands are interpreted in robot base coordinates.
- Robot home pose is captured once:
  - robot_home_pose = get_current_robot_pose()

### 4. End-effector frame

- Commanded as Cartesian pose [x, y, z, rx, ry, rz].
- Sent through rm_movep_canfd after safety + smoothing.

## What the Controller Actually Computes

Each cycle, the mapper does:

1. Current tracker pose:
   - current_T = get_current_tracker_matrix()

2. Relative motion from home:
   - T_delta = inv(tracker_home_T) @ current_T

3. Extract home-referenced deltas:
   - pos_delta = T_delta[:3, 3]
   - rotvec_delta from T_delta[:3, :3]

4. Axis remap:
   - position: [-y, -x, -z]
   - rotation vector: [-ry, -rx, -rz]

5. Add to robot home pose:
   - target_pose = robot_home_pose + remapped_delta

6. Apply safety bounds, then smoothing, then send to robot.

Note on safety: the safety layer additionally compares against last_filtered_pose each cycle to limit step size and angular rate.


## Why the Offset Cancels (Important)

Let:

- O = origin_inv (constant)
- H = raw tracker pose at home (from tracker.get_T())
- C = raw tracker pose at current cycle

Then the code uses:

- home = O @ H
- current = O @ C
- T_delta = inv(home) @ current

So:

T_delta = inv(O @ H) @ (O @ C)
        = (inv(H) @ inv(O)) @ (O @ C)
        = inv(H) @ C

The constant O cancels exactly.

Result:

- In this implementation, a fixed base_station_origin does not change commanded motion.
- You do not need to manually calibrate that constant for relative teleop behavior.

## What Actually Needs Calibration

The meaningful runtime anchors are:

1. tracker_home_T (captured at startup)
2. robot_home_pose (captured at startup)
3. Axis remap convention (how home-referenced tracker deltas map to robot target deltas)

If these are consistent, teleop works independent of the absolute SteamVR origin.

## When base_station_origin Would Matter

It matters if you switch to absolute mapping or mix absolute tracker poses into control logic, for example:

- Commanding robot from absolute world coordinates directly.
- Logging/using absolute tracker coordinates for external fusion.
- Any code path that does not subtract home using the same transformed frame.

It can also matter if the SteamVR reference frame changes during operation (recenter/reset), since that breaks the "constant O" assumption.

## Safety and Workspace Notes

Safety limits still apply in robot coordinates after delta mapping:

- X/Y bounds: [-0.37, 0.37]
- Z bounds: [0.05, 0.40]
- Reach bounds: min 0.15, soft 0.54, hard 0.58

These are independent of absolute world origin and are enforced after target pose construction.

## Practical Takeaway

Your understanding is correct:

- The system controls the arm from home-referenced tracker deltas.
- Absolute VR world origin is not the controlling reference in the current code path.
- The hardcoded base_station_origin is currently redundant for motion behavior (as long as it stays constant).
