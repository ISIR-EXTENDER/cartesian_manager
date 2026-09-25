# cartesian_manager

`cartesian_manager` is a ROS 2 package that sits between user-level Cartesian command sources and the downstream `qontrol_controller`.

It has two jobs:

1. Collect Cartesian velocity inputs, such as joystick, tablet, and visual-servoing commands.
2. Apply the selected command mode, then publish either:
   - a shaped Cartesian velocity command for QP Cartesian control, or
   - a named joint-position target command for QP joint-target control.

The package deliberately keeps robot control authority in `qontrol_controller`. In particular, joint targets are not executed directly here. `cartesian_manager` only selects and publishes the target. `qontrol_controller` converts the joint error to a bounded joint velocity task inside its QP.

## Package Layout

```text
cartesian_manager/
  CMakeLists.txt
  package.xml
  README.md
  bringup/
    config/
      explorer_params.yaml
    launch/
      explorer.launch.py
  docs/
    technical_guide.md
  include/cartesian_manager/
    core/
      input_manager.hpp
      manager.hpp
      types.hpp
      shapers/
        shaper.hpp
        geometric/
          jaco.hpp
          snake.hpp
        behaviour/
          intent_scaling.hpp
          joint_target.hpp
    ros/
      cartesian_manager.hpp
      parameter_parsing.hpp
      topic_manager.hpp
  src/
    cartesian_manager.yaml
    core/
      input_manager.cpp
      manager.cpp
      shapers/geometric/
        jaco.cpp
        snake.cpp
    ros/
      cartesian_manager.cpp
      main.cpp
      parameter_parsing.cpp
      topic_manager.cpp
```

## Runtime Flow

Normal Cartesian command flow:

```text
joystick / tablet / visual-servoing TwistStamped
        |
        v
CartesianManagerROS subscribers
        |
        v
manager_core::InputManager
        |
        v
manager_core::Manager
        |
        v
geometric shaper: both, jaco, or snake
        |
        v
output normalization (||v|| <= 1, ||ω|| <= 1)
        |
        v
rate limiter (bounds dv/dt and dω/dt)
        |
        v
/cartesian_command TwistStamped
        |
        v
qontrol_controller CartesianVelocity QP task
```

Joint target flow:

```text
/mode_request: "behaviour/joint_target/home"
        |
        v
manager_core::Manager validates "home"
        |
        v
CartesianManagerROS publishes /joint_target_command sensor_msgs/JointState once
        |
        v
qontrol_controller JointVelocity QP task moves toward the target
        |
        v
cartesian_manager immediately returns to passthrough
```

This one-shot dispatch is important. `cartesian_manager` must not stay in `joint_target` mode forever, otherwise it would keep publishing zero Cartesian velocity and joystick control would never resume.

## Build

From the workspace root:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select cartesian_manager
```

For local CMake-only validation from this package directory:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cmake -S . -B /tmp/cartesian_manager_build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/cartesian_manager_build --target cartesian_manager_node
```

`/usr/bin/python3` is recommended when generating parameters if another Python environment does not provide ROS Python packages such as `catkin_pkg`.

## Launch

### Robot commands

Explorer:

```bash
source install/setup.bash
ros2 launch cartesian_manager explorer.launch.py use_simulation:=true
```

```bash
source install/setup.bash
ros2 launch cartesian_manager explorer.launch.py use_simulation:=false
```

Kinova:

```bash
source install/setup.bash
ros2 launch cartesian_manager kinova.launch.py use_simulation:=false robot_ip:=192.168.1.10
```

The Explorer-style bringup is:

```bash
source install/setup.bash
ros2 launch cartesian_manager explorer.launch.py
```

Simulation:

```bash
ros2 launch cartesian_manager explorer.launch.py use_simulation:=true
```

Hardware:

```bash
ros2 launch cartesian_manager explorer.launch.py use_simulation:=false
```

The launch file starts:

- the Explorer simulation or hardware base launch,
- `qontrol_explorer`,
- the gripper controller,
- `cartesian_manager_node`,
- `joy_node`,
- `joystick_mapper`.

Pass `extra_params:=<file>` to load a parameter file after the robot's params file, for example to add `visual_servoing` to `inputs.sources` without editing the example config. Both launch files accept it.

The launch/config files are installed from:

- `bringup/launch/explorer.launch.py`
- `bringup/config/explorer_params.yaml`

## Important Topics

Default topics from `bringup/config/explorer_params.yaml`:

| Topic | Type | Direction | Meaning |
| --- | --- | --- | --- |
| `/joystick_cartesian_command` | `geometry_msgs/msg/TwistStamped` | input | Joystick Cartesian velocity command. |
| `/tablet_cartesian_command` | `geometry_msgs/msg/TwistStamped` | input | Tablet Cartesian velocity command. |
| `/visual_servoing_cartesian_command` | `geometry_msgs/msg/TwistStamped` | input | Visual-servoing Cartesian velocity command. |
| `/mode_request` | `std_msgs/msg/String` | input | Mode selection request. |
| `/pose_target` | `geometry_msgs/msg/PoseStamped` | input | Dynamic Cartesian pose target executed immediately. |
| `/ee_pose` | `geometry_msgs/msg/PoseStamped` | input | Current end-effector pose from `qontrol_controller`. |
| `/ee_velocity` | `geometry_msgs/msg/TwistStamped` | input | Current end-effector velocity from `qontrol_controller`. |
| `/ee_jac` | `std_msgs/msg/Float64MultiArray` | input | Current end-effector Jacobian. |
| `/joint_states` | `sensor_msgs/msg/JointState` | input | Current joint state. |
| `/cartesian_command` | `geometry_msgs/msg/TwistStamped` | output | Cartesian velocity sent to `qontrol_controller`. |
| `/joint_target_command` | `sensor_msgs/msg/JointState` | output | Named joint-position target sent to `qontrol_controller`. |
| `/cartesian_manager/status` | `diagnostic_msgs/msg/DiagnosticStatus` | output | Latched manager state: `geometric`, `behaviour`, `target` and enabled `inputs`, published on change. |

## Mode Requests

Publish mode requests as `std_msgs/msg/String` on `/mode_request`.

Examples:

```bash
ros2 topic pub --once /mode_request std_msgs/msg/String "{data: 'geometric/both'}"
ros2 topic pub --once /mode_request std_msgs/msg/String "{data: 'geometric/jaco'}"
ros2 topic pub --once /mode_request std_msgs/msg/String "{data: 'geometric/snake'}"
ros2 topic pub --once /mode_request std_msgs/msg/String "{data: 'behaviour/passthrough'}"
ros2 topic pub --once /mode_request std_msgs/msg/String "{data: 'behaviour/joint_target/home'}"
ros2 topic pub --once /mode_request std_msgs/msg/String "{data: 'behaviour/pose_target/ready'}"
ros2 topic pub --once /mode_request std_msgs/msg/String "{data: 'behaviour/intent_scaling'}"
```

Mode strings are normalized before parsing:

- uppercase becomes lowercase,
- `-` becomes `_`,
- the package uses the British spelling `behaviour`.

## Parameters

Generated parameter definitions live in:

- `src/cartesian_manager.yaml`

Runtime Explorer parameters live in:

- `bringup/config/explorer_params.yaml`

The main groups are:

- `update_rate_hz`
- `frames`
- `topics`
- `inputs`
- `shapers`
- `rate_limiter`
- `behaviours`

Topic names, frames, the update rate, input source declarations, and target arrays are startup-only parameters. Change them in the launch configuration and restart the node. Input enable flags, timeouts, shaper gains and limits, rate limits, and pose-target gains and tolerances can change at runtime. The node validates those updates before applying them.

## Output Processing

After the geometric shaper runs, `Manager::update` applies two sequential post-processing steps to ensure the output sent to `qontrol_controller` is always bounded and continuous.

### Output Normalisation

Each output component is clamped so that its Euclidean norm never exceeds 1.0 (the unit scale expected by `qontrol_controller`):

```
||v_out||  > 1  →  v_out  = v_out  / ||v_out||
||ω_out||  > 1  →  ω_out  = ω_out  / ||ω_out||
```

This matters most in **snake mode**: the coupling term `gain × (z_tool × v_linear)` can easily produce `||ω|| > 1` at large linear velocities, which would otherwise exceed the `command_max_angular_velocity` scaling applied downstream.

### Rate Limiter

A vectorial slew-rate limiter bounds the step-to-step change in the normalised command. This prevents velocity discontinuities when the snake mode is toggled while the end-effector is already moving.

The per-cycle maximum delta is:

```
Δv_max  = max_linear_acceleration  × dt
Δω_max  = max_angular_acceleration × dt
```

If `||command - previous|| > Δ_max`, the output is clipped in the direction of the desired change, preserving the rotation/translation axis:

```
command = previous + (delta / ||delta||) × Δ_max
```

Setting either acceleration to `<= 0.0` disables limiting for that component.

The rate limiter state is **reset to zero** whenever:

- no valid command is available (input timeout), or
- the `joint_target` behaviour is active.

This avoids a stale `previous` value causing a phantom ramp when the joystick resumes.

Configure both accelerations in `bringup/config/explorer_params.yaml`:

```yaml
cartesian_manager:
  ros__parameters:
    rate_limiter:
      max_linear_acceleration: 2.0   # normalised units/s  (0.5 s to reach full speed)
      max_angular_acceleration: 2.0  # normalised units/s  (0.5 s to reach full angular speed)
```

## Frames

Frame names are configured under `frames`:

- `base_frame`: commands already in base are summed directly.
- `ee_frame`: commands in this frame are rotated into base with the latest `ee_pose`.
- `hybrid_frame`: commands in this frame are rotated into base with the manager-computed hybrid pose.
- `default_input_frame_id`: used when an incoming `TwistStamped` has an empty `header.frame_id`; it must match one of the three command frames above.
- `output_frame_id`: fallback frame for published zero commands.

`cartesian_manager` does not use TF lookup. `ee_pose` must be stamped in `frames.base_frame`; the manager derives the hybrid pose from it.

## Pose Targets

Configure named Cartesian poses under `behaviours.pose_targets`. The arrays follow
`target_names` order. Each target has one `frame_ids` entry, three XYZ
`positions` values, and four XYZW `orientations` values:

```yaml
cartesian_manager:
  ros__parameters:
    behaviours:
      pose_targets:
        target_names: [ready]
        frame_ids: [base_link]
        positions: [0.4, 0.0, 0.3]
        orientations: [0.0, 0.0, 0.0, 1.0]
        linear_kp: 1.0
        angular_kp: 1.0
        max_linear_velocity: 0.1
        max_angular_velocity: 0.2
        position_tolerance: 0.01
        orientation_tolerance: 0.05
```

Send `behaviour/pose_target/ready` on `/mode_request` to start following the
pose. The manager publishes Cartesian velocity toward the target without
requiring a joystick command. Every target frame must equal `frames.base_frame`,
and the incoming `ee_pose` frame must match it. Velocity
becomes zero when both position and orientation are within tolerance, then
the manager returns to input control on the next update. Send
`behaviour/passthrough` to return to input control earlier. The controller uses
proportional gains only: `linear_kp` for position error and `angular_kp` for
orientation error. Each command is capped by its configured maximum velocity.

To execute a pose that was not configured at startup, publish it directly on
`/pose_target`:

```bash
ros2 topic pub --once /pose_target geometry_msgs/msg/PoseStamped   "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.6, y: 0.270, z: 0.32}, orientation: {x: -0.22, y: 0.85, z: 0.41, w: 0.45}}}"
```

The pose is executed immediately and replaces any pose target already in progress. Its
`header.frame_id` must be the configured `frames.base_frame`; empty or different frames are
rejected because the manager does not perform TF lookups. Non-finite poses and zero quaternions
are also rejected, while valid quaternions are normalized. Named YAML targets and
`behaviour/pose_target/<name>` remain available.

## Intent Scaling

`behaviour/intent_scaling` speeds the arm up while the operator keeps pushing the same way. A push starts at
`min_scale` of the linear command and rises to full scale; releasing the input, or turning back against the push,
starts slow again. The output stays within unit scale, so `command_max_linear_velocity` in `qontrol_controller`
remains the top speed. Angular commands are not scaled. `behaviour/passthrough` turns it off.

```yaml
    behaviours:
      intent_scaling:
        min_scale: 0.4              # scale at the start of a push
        window_sec: 0.5             # how far back the push is measured
        consistency_threshold: 0.5  # mean push along the current direction above which the scale rises
        gain: 1.0                   # scale change per second per unit off the threshold
```

With these values a full push reaches full scale in about 1.6 s; a push under half the stick never does. All four
can be changed at runtime with `ros2 param set`.

## Joint Targets

Named joint targets are configured under:

```yaml
cartesian_manager:
  ros__parameters:
    behaviours:
      joint_targets:
        joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
        target_names: [home]
        positions: [2.5, 0.3, -2.4, 2.97, 1.2, -0.5]
```

`positions` is flattened in `target_names` order. If there are 6 joints and 2 targets, the array must contain 12 values.

When `behaviour/joint_target/home` is received, the node publishes:

- `msg.name = joint_names`
- `msg.position = positions for home`

on `/joint_target_command`.

## Relation To qontrol_controller

`qontrol_controller` is expected to subscribe to `/cartesian_command` and `/joint_target_command`.

The joint-target execution belongs there:

```text
q_error = q_target - q_current
qdot_target = joint_target_gain * q_error
qdot_target is clamped by joint_target_max_velocity
qdot_target is sent to a Qontrol Task::JointVelocity
```

This keeps joint limits and QP constraints active while moving to a target.

## Development Notes

For extension details, see:

- `docs/technical_guide.md`
