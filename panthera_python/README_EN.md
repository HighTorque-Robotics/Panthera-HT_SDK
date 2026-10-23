# Panthera Robot Control Python SDK

[简体中文](README.md)

## Features

- **Multiple control modes**
  - Position and velocity control with maximum torque limits
  - Five-parameter MIT control: position, velocity, torque, Kp, and Kd
  - Independent gripper control
- **Kinematics**
  - Forward kinematics (FK)
  - Inverse kinematics (IK)
- **Dynamics**
  - Gravity compensation
  - Coriolis compensation
  - Mass matrix
  - Friction compensation
  - Full rigid-body dynamics
- **Trajectory planning**
  - Seventh-order polynomial interpolation with continuous velocity, acceleration, and jerk
  - Seventh-order polynomial interpolation with endpoint velocity constraints
- **Safety features**
  - Joint position limits
  - Torque limits
  - Timeout detection
  - Target position detection
- **Leader-follower control**, including dual-arm teleoperation and trajectory recording/playback
- **YAML-based configuration**

## Base Coordinate Frame

![Base coordinate frame](images/base_joint.png)

## Installation

### Recommended: create a dedicated Conda environment

Use a separate environment to avoid conflicts with system packages such as ROS:

```bash
conda create -n panthera python=3.10
conda activate panthera
```

Python 3.9 through 3.12 are supported.

### Option 1: install a prebuilt wheel (recommended for Ubuntu 22)

> The wheels were built on Ubuntu 22. If they are incompatible with Ubuntu 20 or another system, build the SDK from source as described in Option 2.

Run the following commands from `panthera_python/` and select the wheel matching your Python version:

```bash
# Python 3.9
pip install motor_whl/hightorque_robot-1.2.0-cp39-cp39-linux_x86_64.whl

# Python 3.10
pip install motor_whl/hightorque_robot-1.2.0-cp310-cp310-linux_x86_64.whl

# Python 3.11
pip install motor_whl/hightorque_robot-1.2.0-cp311-cp311-linux_x86_64.whl

# Python 3.12
pip install motor_whl/hightorque_robot-1.2.0-cp312-cp312-linux_x86_64.whl
```

Version 1.0.0 wheels for aarch64 and Python 3.9 through 3.12 are also provided in `motor_whl/`. Check the architecture with `uname -m`, then install the matching wheel. For example:

```bash
pip install motor_whl/hightorque_robot-1.0.0-cp310-cp310-linux_aarch64.whl
```

### Option 2: build from source (Ubuntu 20 or other systems)

Run all commands from the `Panthera-HT_SDK/` repository root after activating the target Conda environment.

1. Install system dependencies:

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    libglib2.0-dev \
    libyaml-cpp-dev \
    libserialport-dev
```

The source tree includes LCM and serial-library sources, so a separate `liblcm-dev` package is unnecessary. The Python extension uses the active Conda environment and does not require the system `python3-dev` or `python3-pip` packages.

2. Install Python build dependencies:

```bash
python -m pip install -U pip setuptools wheel pybind11
```

3. Build and install the motor SDK into the active Conda environment:

```bash
cmake -S panthera_cpp/motor_cpp -B panthera_cpp/motor_cpp/build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX"
cmake --build panthera_cpp/motor_cpp/build -j"$(nproc)"
cmake --install panthera_cpp/motor_cpp/build
```

4. Build and install the Python bindings into the active Conda environment:

```bash
CMAKE_PREFIX_PATH="$CONDA_PREFIX" \
python -m pip install -e ./panthera_python --no-build-isolation --no-deps
```

This invokes CMake to build `_hightorque_robot` and installs `hightorque_robot` as an editable package. It does not modify `/usr/local` or affect wheel installations in other virtual environments.

### Install high-level Python dependencies

This step is required when using `Panthera_lib`. Run it after either installation option:

```bash
# Recommended: run from the repository root
cd panthera_python
python -m pip install -r requirements.txt
cd ..
```

Alternatively, install the packages individually:

```bash
python -m pip install "numpy>=1.19.0,<2.0"
python -m pip install "pyyaml>=6.0"
python -m pip install "pin>=2.6.0"
python -m pip install "scipy>=1.9.0"
python -m pip install "osqp>=1.1.0,<2.0"
```

- Install the `pin` package, which provides the Pinocchio robot dynamics library. Do not install the unrelated `pinocchio` testing package.
- PyYAML 6.0 or later is required for Python 3.12 and later.
- In a Conda environment, use `pip` or `python -m pip` rather than `pip3`.

Optional dependencies are not currently included in `requirements.txt`:

| Feature | Installation command |
| --- | --- |
| Trajectory GUI and GUI offline tests | `python -m pip install PyQt5` |
| Plotting examples | `python -m pip install matplotlib` |
| Keyboard teleoperation | `python -m pip install pynput` |
| BETOP/XInput gamepad | `python -m pip install evdev` |
| GELLO and Feetech servo input | `python -m pip install feetech-servo-sdk` |

### Verify the installation

```bash
python -c "import hightorque_robot; print('Motor SDK installed')"
python -c "import pinocchio as pin; print('pin installed')"
python -c "import yaml; print('PyYAML installed')"
```

`Panthera_lib` is provided as source and is imported from the `scripts` directory:

```bash
cd panthera_python/scripts
python -c "from Panthera_lib import Panthera"
```

## Quick Start

### Communication board wiring

![Communication board wiring1](images/Board1.png)
![Communication board wiring2](images/Board2.png)

### Grant serial-port access

After connecting the communication boards, list the serial devices. Their number depends on the number of connected boards, not the number of motors:

```bash
ls /dev/ttyACM*
sudo chmod -R 777 /dev/ttyACM*
```

### Check robot state

For the first run, enter `panthera_python/scripts/` and inspect all joint states:

```bash
python 0_robot_get_state.py
```

The program prints port, motor ID, initialization information, joint position, velocity, mode, and fault status.

### Run position-velocity control

```bash
python 1_Joint_PosVel_control.py
```

The arm moves through several target positions at specified speeds. Completing the sequence indicates that basic communication and control work correctly.

### Record and replay a trajectory

`5_record_trajectory.py` records arm and gripper motion while the arm is under gravity compensation. It currently uses `Follower.yaml`. Press `Ctrl+C` to stop; the recording is saved under `scripts/trajectory/` with a name such as `trajectory_20251211_160537.jsonl`.

```bash
python 5_record_trajectory.py
```

Set `TRAJECTORY_FILE` in `5_replay_trajectory.py` to the saved filename, then run:

```bash
python 5_replay_trajectory.py
```

To use the leader arm, change the script's `config_path` from `Follower.yaml` to `Leader.yaml`. Use the corresponding configuration during recording as well.

### Trajectory GUI: record, replay, and compose

Run from `panthera_python/scripts/`:

```bash
python 5_trajectory_gui.py
```

Opening the GUI alone does not connect to the robot. The default configuration is `Follower.yaml`; change `CONFIG_PATH` near the top of the script when needed.

1. Click **Start Gravity Compensation** to connect and enter hand-guiding mode.
2. Click **Start Recording (Auto Name)**, move the arm and gripper, then click **Stop Recording and Save**. Gravity compensation remains active.
3. Select a trajectory, set the replay count or infinite loop and interval, then start playback. The robot first transitions from its actual pose to the trajectory start.
4. **Stop Playback (Keep Gravity Compensation)** returns to hand-guiding mode. **Stop All (Enter Damping)** ends control. Closing the window also applies damping and stops worker threads.

The GUI previews six joint curves and supports renaming and deleting trajectories. The trajectory composer can combine recordings, pauses, transitions, and return-to-zero actions. If a requested transition is too short, the GUI extends it according to the configured velocity and acceleration limits.

Recordings are stored as `scripts/trajectory/*.jsonl`. Saving a composed sequence creates both a replayable `.jsonl` and an editable `.sequence.json` recipe. Keep all source trajectories referenced by the recipe.

GUI playback uses MIT control throughout. Command-line playback uses position-velocity control to reach the starting point, then MIT control for the recorded trajectory.

### Leader-follower teleoperation

Connect the follower arm to CAN port 1 and the leader arm to CAN port 2, then run:

```bash
python 5_teleop_control.py
```

### GELLO teleoperation

From the repository root, enter the scripts directory and specify the GELLO serial port:

```bash
cd panthera_python/scripts
python 7_gello_teleop.py --gello-port /dev/ttyACM1
```

To verify GELLO joint input and mapping without connecting a Panthera follower arm:

```bash
python 7_gello_teleop.py --gello-port /dev/ttyACM1 --dry-run
```

### Keyboard and gamepad control

Run one entry point from `panthera_python/scripts/`:

```bash
python 7_keyboard_cartesian_pos_control.py
python 7_keyboard_cartesian_vel_control.py
python 7_gamepad_cartesian_vel_control.py
```

The keyboard scripts control Cartesian pose and Cartesian velocity respectively. Their key mappings are documented at the top of each script. The gamepad program uses Linux `evdev` and defaults to a BETOP A2P3A XInput controller. Set its device path and speed parameters near the top of the script and ensure that the user can read the corresponding `/dev/input/` device.

## Basic Example

```python
from Panthera_lib import Panthera

# Uses the Follower configuration by default.
robot = Panthera()

# Or specify a configuration file.
# robot = Panthera("../robot_param/Leader.yaml")

joint_pos = robot.get_current_pos()
print(f"Current joint positions: {joint_pos}")

target_pos = [0.0, 0.5, 0.5, 0.0, 0.5, 0.0]
target_vel = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
max_torque = [10.0, 10.0, 10.0, 5.0, 5.0, 5.0]

robot.Joint_Pos_Vel(target_pos, target_vel, max_torque)
robot.Joint_Pos_Vel(
    target_pos,
    target_vel,
    max_torque,
    iswait=True,
    tolerance=0.01,
    timeout=15.0,
)

robot.gripper_open()
# robot.gripper_close(pos=0.0)
```

## API Reference

### `Panthera`

`Panthera` inherits from `htr.Robot` and provides robot-level control methods.

#### Initialization

- `Panthera(config_path=None)`: creates a robot instance. If omitted, `config_path` defaults to `Follower.yaml`.

The `robot.gripper_installed` field is required. For a physical robot without a gripper, configure `Follower.yaml` or `Leader.yaml` as follows:

```yaml
robot:
  gripper_installed: false
  gripper_name: "gripper"
```

When set to `false`, the low-level SDK initializes only the six arm joints and does not probe for a seventh gripper motor. The dynamics model removes `urdf.end_effector_link` and all its descendants, then uses `urdf.end_effector_link_without_gripper` as the control endpoint. Parallel branches, such as a camera attached to the parent of `end_effector_link`, remain in the model. Gravity compensation, FK, IK, MoveL, and VR Cartesian control therefore use the gripper-free model without requiring changes to the original URDF or `motor_param.motor_num`. Set the field to `true` when the physical gripper is installed.

If a custom URDF has already removed `end_effector_link`, gripper-free mode can still load it. The program also removes residual joints and descendants that reference that name. The link named by `end_effector_link_without_gripper` must remain present.

```yaml
urdf:
  end_effector_link: "gripper_center_link"
  end_effector_link_without_gripper: "link6"
```

Gripper configuration is read only from YAML, so restart the program after changing it. At runtime, check `robot.has_gripper`. When no gripper is available, gripper control methods return `False` and gripper state methods return a zero-valued placeholder with `available=False`.

#### State methods

- `get_current_state()`: all joint states
- `get_current_pos()`: joint positions as `np.ndarray [6,]`
- `get_current_vel()`: joint velocities as `np.ndarray [6,]`
- `get_current_torque()`: joint torques as `np.ndarray [6,]`
- `get_current_state_gripper()`: gripper state
- `get_current_pos_gripper()`: gripper position
- `get_current_vel_gripper()`: gripper velocity
- `get_current_torque_gripper()`: gripper torque

#### Control methods

- `Joint_Pos_Vel(pos, vel, max_tqu=None, iswait=False, tolerance=0.1, timeout=15.0)`
  - Independent joint position-velocity control with per-joint maximum torque
  - `pos`: target positions `[6,]` in rad
  - `vel`: target velocities `[6,]` in rad/s
  - `max_tqu`: maximum torques `[6,]` in Nm; configuration defaults are used when omitted
  - `iswait`: wait for the target
  - `tolerance`: position tolerance in rad
  - `timeout`: timeout in seconds
- `moveJ(pos, duration, max_tqu=None, iswait=False, tolerance=0.1, timeout=15.0)`
  - Synchronized joint-space motion
  - `duration`: motion duration in seconds
- `pos_vel_tqe_kp_kd(pos, vel, tqe, kp, kd)`
  - Five-parameter MIT control
  - `tqe`: feedforward torque `[6,]` in Nm
- `gripper_control(pos, vel, max_tqu)`: gripper position-velocity control
- `gripper_control_MIT(pos, vel, tqe, kp, kd, send=True)`: gripper MIT control; `send=False` caches the command for a later synchronized arm send
- `gripper_open(pos=1.6, vel=0.5, max_tqu=0.5)`: open the gripper
- `gripper_close(pos=0.0, vel=0.5, max_tqu=0.5)`: close the gripper
- `Joint_Vel(vel)`: six-joint velocity control
- `moveL(target_position, target_rotation=None, duration=None, use_spline=True, max_tqu=None)`: Cartesian linear motion
- `set_brake()`: put all motors, including the gripper, into firmware damping mode

#### Position checks

- `check_position_reached(target_positions, tolerance=0.1)`
- `wait_for_position(target_positions, tolerance=0.01, timeout=15.0)`

#### Kinematics

- `forward_kinematics(joint_angles=None)` returns `{'position': [x,y,z], 'rotation': R, 'transform': T, 'joint_angles': q}`.
- `inverse_kinematics(target_position, target_rotation=None, init_q=None, max_iter=1000, eps=1e-3, damping=1e-2, adaptive_damping=True, multi_init=True, num_attempts=8)` returns a joint solution `[6,]` or `None`.

#### Dynamics

- `get_Gravity(q=None)`: gravity torque `G(q)`
- `get_Coriolis(q=None, v=None)`: Coriolis matrix `C(q,v)`
- `get_Coriolis_vector(q=None, v=None)`: Coriolis vector `C(q,v)v`
- `get_Mass_Matrix(q=None)`: mass matrix `M(q)`
- `get_Inertia_Terms(q=None, a=None)`: inertia torque `M(q)a`
- `get_Dynamics(q=None, v=None, a=None)`: full dynamics `τ = M(q)a + C(q,v)v + G(q)`
- `get_friction_compensation(vel=None, Fc=None, Fv=None, vel_threshold=0.01)`: friction torque using `τ_friction = Fc * sign(vel) + Fv * vel`

#### Trajectory planning

- `septic_interpolation(start_pos, end_pos, duration, current_time)` returns position, velocity, and acceleration.
- `septic_interpolation_with_velocity(start_pos, end_pos, start_vel, end_vel, duration, current_time)` adds endpoint velocity constraints and returns position, velocity, and acceleration.

#### Inherited low-level methods

- `motor_send_cmd()`: send motor commands
- `send_get_motor_state_cmd()`: request motor state
- `set_stop()`: stop all motors
- `set_reset()`: reset all motors
- `set_timeout(timeout_ms)`: set the communication timeout

Low-level motor IDs, CAN buses, and motor models are configured in `robot_param/motor_param/*.yaml`.

## Project Layout

```text
panthera_python/
├── motor_whl/                         # Prebuilt Python wheels
├── Panthera-HT_description_V1.1.0/   # Robot model and simulation assets
│   ├── urdf/                          # Leader/follower URDFs and instructions
│   ├── meshes/                        # GLB visual and STL collision meshes
│   ├── mujoco/                        # MuJoCo scenes, meshes, and viewer scripts
│   ├── image/                         # URDF documentation images
│   ├── config/                        # Joint-name configuration
│   └── launch/                        # ROS launch files
├── robot_param/
│   ├── Leader.yaml
│   ├── Follower.yaml
│   └── motor_param/                   # Motor and robot configuration
├── scripts/
│   ├── Panthera_lib/
│   │   ├── Panthera.py                # High-level Panthera class
│   │   ├── qp_ik.py                   # Pinocchio + OSQP velocity IK
│   │   └── recorder.py                # Trajectory recorder
│   ├── 0_robot_get_state.py
│   ├── 0_robot_set_zero.py
│   ├── 1_Joint_PosVel_control.py
│   ├── 2_gravity_compensation_control.py
│   ├── 5_record_trajectory.py
│   ├── 5_replay_trajectory.py
│   ├── 5_trajectory_gui.py
│   ├── 5_teleop_control.py
│   ├── 6_moveL_pos_control.py
│   ├── 6_moveL_rotate_control.py
│   ├── 7_gello_teleop.py
│   ├── 7_keyboard_cartesian_pos_control.py
│   ├── 7_keyboard_cartesian_vel_control.py
│   ├── 7_gamepad_cartesian_vel_control.py
│   ├── 7_vr_cartesian_control.py
│   ├── 7_vr_dual_cartesian_control.py
│   ├── 7_vr_usb_start_quickly.py      # USB VR quick launcher
│   ├── Quest_controller_stream/       # Quest data relay and documentation
│   ├── trajectory/                    # Runtime recordings and sequences
│   ├── tests/                         # Hardware-free automated tests
│   └── motor_example/                 # Low-level motor examples
├── hightorque_robot/                  # Python entry package for bindings
├── src/bindings.cpp                   # pybind11 bindings
├── CMakeLists.txt
├── images/
├── requirements.txt
├── setup.py
├── pyproject.toml
├── README.md                          # Chinese documentation
└── README_EN.md                       # English documentation
```

## Quest VR Teleoperation

### Components and data flow

```text
Quest Browser / controllers
        │ WebSocket
        ▼
Quest_controller_stream          Relays data only; it does not control a robot
        │ UDP 127.0.0.1:5005
        ▼
Single- or dual-arm receiver     Solves QP and sends MIT commands
        │
        ▼
Panthera robot
```

| Program | Purpose | Controls a robot? |
| --- | --- | --- |
| `Quest_controller_stream/start_stream_server.py` | Serves the Quest web page and relays WebSocket controller data to UDP port 5005 | No |
| `Quest_controller_stream/quest_stream_ui.py` | Desktop debugging UI for viewing and verifying controller data | No |
| `7_vr_cartesian_control.py` | Receives UDP data; the right controller operates one `Follower` arm | Yes, single arm |
| `7_vr_dual_cartesian_control.py` | Receives UDP data; the left controller operates `Leader` and the right controller operates `Follower` | Yes, dual arm |
| `7_vr_usb_start_quickly.py` | Starts the USB relay and one selected receiver | Indirectly; recommended entry point |

For USB teleoperation, run only `7_vr_usb_start_quickly.py`. Do not separately start `Quest_controller_stream` or either receiver at the same time, because doing so causes port conflicts or duplicate robot commands.

### Prerequisites

```bash
python -m pip install -r panthera_python/requirements.txt
python -m pip install websockets
sudo apt install adb openssl
```

The quick launcher supports USB only and requires `adb`. `openssl` is needed when manually configuring Wi-Fi relay certificates.

Connect the Quest by USB, accept the USB debugging prompt in the headset, and verify:

```bash
adb devices
```

The device state must be `device`, not `unauthorized`.

### USB quick start (recommended)

```bash
cd panthera_python/scripts

# USB + single arm
python 7_vr_usb_start_quickly.py

# USB + dual arm
python 7_vr_usb_start_quickly.py --controller dual
```

Single-arm mode uses the right controller for the arm configured by `Follower.yaml`. Dual-arm mode uses the left controller for `Leader.yaml` and the right controller for `Follower.yaml`.

1. Connect the Quest over USB and verify it with `adb devices`.
2. Run one of the commands above.
3. The launcher configures `adb reverse`, starts the relay, and opens `http://localhost:8080` in Quest Browser.
4. Click **Start XR Stream** in the headset.
5. Wait until the terminal confirms Quest data reception and robot initialization, then hold `Squeeze` to control the arm.

At startup, the launcher closes stale Quest Browser WebVR sessions and tabs so the page uses the USB `localhost` connection through `adb reverse`.

```bash
# Do not open Quest Browser automatically
python 7_vr_usb_start_quickly.py --no-open-quest

# Print commands without connecting devices
python 7_vr_usb_start_quickly.py --dry-run
```

Press `Ctrl+C` once to stop. The launcher stops the relay, closes the Quest Browser page and WebXR session through ADB, and waits for the robot to return to zero. Keep the terminal open and robot power connected until homing completes. Repeated `Ctrl+C` presses during homing do not bypass the safe shutdown sequence.

The Quest Browser page does not occupy ports 8080 or 8765 on the computer; the relay process owns those ports. The launcher terminates that process during shutdown. With `--no-open-quest`, it does not automatically open or close Quest Browser.

### Manual split startup (debugging only)

Use two terminals and do not run the quick launcher at the same time.

Terminal 1, data relay only:

```bash
cd panthera_python/scripts/Quest_controller_stream

# USB relay
python start_stream_server.py --adb always --udp-ip 127.0.0.1 --udp-port 5005

# Wi-Fi relay; choose this or USB, not both
python start_stream_server.py --adb never --udp-ip 127.0.0.1 --udp-port 5005
```

You may instead use `python quest_stream_ui.py`. The desktop UI and `start_stream_server.py` are mutually exclusive. See the [Quest Controller Stream guide](scripts/Quest_controller_stream/README.md) for details.

Open the address printed by the relay or UI in Quest Browser and click **Start XR Stream**. If you used the UI's **Start Verify** function or `udp_receiver_test.py`, stop it before starting robot control so that UDP port 5005 is released.

Terminal 2, one receiver only:

```bash
cd panthera_python/scripts

# Single arm: right controller operates Follower
python 7_vr_cartesian_control.py

# Dual arm: left controls Leader and right controls Follower; choose one receiver
python 7_vr_dual_cartesian_control.py
```

### Controller mapping and control method

- Hold `Squeeze` on the corresponding controller to activate control. Single-arm mode uses the right controller; dual-arm mode uses both controllers.
- Move the controller to command end-effector position.
- Rotate the controller to command end-effector orientation.
- Hold `Squeeze` and press `Trigger` to control the corresponding gripper.
- Release `Squeeze` to stop motion and hold the current position.

Both receivers listen on local UDP port 5005. They use the velocity-level QP in `Panthera_lib/qp_ik.py` with OSQP to generate continuous joint-position targets, then command the robot in MIT mode. They do not save trajectory CSV files or create plots. The robot moves to a preset pose at startup and attempts to return to zero during a normal shutdown.

The `VR_MAX_*` settings near the top of both receiver scripts configure VR limits. Joint velocity limits default to `[3, 3, 3, 3, 4, 4] rad/s`, while joint position limits remain active. Offline tests verify software logic but cannot validate physical motion direction or tracking performance.

## Offline Tests

Run from the repository root:

```bash
cd panthera_python

# Trajectory GUI: requires NumPy, PyYAML, and PyQt5
QT_QPA_PLATFORM=offscreen python -m unittest discover -s scripts/tests -p 'test_trajectory_gui.py' -v

# VR/QP: requires the high-level packages in requirements.txt
python -m unittest discover -s scripts/tests -p 'test_vr_qp.py' -v

# Optional gripper: uses a simulated low-level SDK
python -m unittest discover -s scripts/tests -p 'test_optional_gripper.py' -v
```

- `test_trajectory_gui.py` uses a mock robot and temporary trajectory directory to test interpolation, playback timing, loops and cancellation, sequence recipes, joint limits, complete seven-motor commands, and shutdown behavior.
- `test_vr_qp.py` replaces the native motor SDK and checks QP solving and VR integration logic.
- `test_optional_gripper.py` replaces the low-level SDK and checks six- and seven-motor configurations, missing-gripper handling, and dynamics-model pruning.

These tests do not connect to hardware. GUI tests run offscreen. Regular examples such as `scripts/1_forward_kinematics_test.py` create a robot instance and are not offline tests.

## Troubleshooting

### URDF loading fails

Check the URDF path in the YAML configuration. Relative paths are resolved from the directory containing the configuration file.

### Inverse kinematics does not converge

Confirm that the target lies inside the robot workspace. You may also tune `max_iter` and `eps`.

### Motors do not connect

Check the communication-board switches and motor power. A green motor power button indicates that power is on. If connection still fails, inspect the cables between motors.

### The VR receiver does not receive controller data

Confirm that `scripts/Quest_controller_stream` is running and verify all of the following:

- Relay UDP IP is `127.0.0.1`.
- Relay UDP port is `5005`.
- The selected VR receiver is running on the same computer.
- **Start XR Stream** has been clicked in Quest Browser.
- In USB mode, `adb devices` lists the Quest as `device`.
- In Wi-Fi mode, the Quest and computer are on the same LAN and the browser has accepted the HTTPS certificate warning.

### Automatically grant serial-port permissions

Create or edit a udev rule:

```bash
sudo gedit /etc/udev/rules.d/99-tegra-devices.rules
```

Add:

```text
KERNEL=="ttyACM*", MODE="0777"
```

Reload the rules and reconnect the computer and communication board:

```bash
sudo udevadm control --reload-rules
```

### `libserialport.so.0` is missing

```bash
sudo apt-get update
sudo apt-get install libserialport-dev
dpkg -L libserialport-dev | grep "\.so"
```

### Importing `hightorque_robot` fails because `libyaml-cpp.so.0.6` is missing

First confirm that the appropriate `hightorque_robot` wheel is installed. If the system provides yaml-cpp 0.7 while an older wheel requires 0.6, the dynamic-library versions are incompatible. Prefer the source-build instructions above so the SDK links against the installed dependencies, or choose a wheel matching the current Python version and architecture. Do not delete or overwrite the system yaml-cpp library to accommodate an old wheel.

After installing the appropriate library or rebuilding, update the linker cache if required:

```bash
sudo ldconfig
```

## License

MIT License

## Contributing

Issues and pull requests are welcome.

## Contact

For questions, open an issue or contact the maintainers.
