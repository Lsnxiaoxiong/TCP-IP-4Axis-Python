# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

TCP-IP-4Axis-Python is a Python-based SDK for controlling Dobot robotic arms (MG400/M1Pro series) via TCP/IP protocol. The project provides APIs for robot control, motion planning, and integrates with Intel RealSense cameras and YOLOv8 for vision-guided robotic operations (e.g., keyboard pressing).

**Compatible controller versions:** V1.5.6.0 and above

## Quick Start

### Prerequisites

1. Connect to robot controller via Ethernet or wireless:
   - LAN1 (wired): `192.168.1.6`
   - LAN2 (wired): `192.168.2.6`
   - Wireless: `192.168.9.1`

2. Set your computer's IP to the same subnet as the controller

3. Install dependencies:
   ```bash
   pip install numpy pyrealsense2 opencv-python onnxruntime
   ```

### Running the Application

```bash
# Keyboard pressor with vision (LabVIEW integration) - main application
python src/keyboard_pressor.py

# Camera calibration utility
python src/realsense_435i.py  # Then call cap.cali()

# Test robot connection
python src/dobot_mg400.py
```

**Note:** Legacy entry points (`main.py`, `mainUI.py`, `PythonExample.py`) have been deleted. The current main application is `src/keyboard_pressor.py`.

## Architecture

### System Architecture

The system follows a layered architecture:

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                        │
│         src/keyboard_pressor.py (main orchestrator)         │
├─────────────────────────────────────────────────────────────┤
│                    Vision Module Layer                      │
│    src/realsense_435i.py │ src/yolov8_onnx.py              │
├─────────────────────────────────────────────────────────────┤
│                    Robot Module Layer                       │
│              src/dobot_mg400.py (DobotMG400 wrapper)        │
├─────────────────────────────────────────────────────────────┤
│                    Core API Layer                           │
│  dobot_api.py (DobotApi, DobotApiDashboard, DobotApiMove)   │
├─────────────────────────────────────────────────────────────┤
│                    Network Layer                            │
│         TCP/IP (Ports: 29999, 30003, 30004, 45678)          │
├─────────────────────────────────────────────────────────────┤
│                    Hardware Layer                           │
│              Dobot Controller (MG400/M1Pro)                 │
│              Intel RealSense D435i Camera                   │
└─────────────────────────────────────────────────────────────┘
```

### Core Modules

```
├── dobot_api.py          # Core API classes (DobotApi, DobotApiDashboard, DobotApiMove, MyType)
├── src/                  # Main application modules
│   ├── dobot_mg400.py    # Dobot MG400 robot wrapper (handles motion, feedback, error threads)
│   ├── realsense_435i.py # Intel RealSense D435i camera wrapper
│   ├── keyboard_pressor.py # Main application orchestrator (vision-guided keyboard pressing)
│   ├── tcp_server.py     # TCP server for LabVIEW integration (port 45678)
│   ├── config.py         # Configuration loader (cali.json)
│   └── cali.json         # Camera calibration parameters
├── yolo/                 # YOLO detection module
│   └── yolov8_onnx.py    # YOLOv8 ONNX inference engine
├── camera/               # Camera utilities (legacy location)
└── files/                # Alarm configuration files
    ├── alarm_controller.json  # Controller error codes
    └── alarm_servo.json       # Servo error codes
```

**Note:** Legacy files (`main.py`, `mainUI.py`, `PythonExample.py`, `ui.py`, `test/`) have been deleted in the current refactored structure. The main application now runs from `src/keyboard_pressor.py`.

### Key Ports

| Port | Purpose | Class |
|------|---------|-------|
| 29999 | Dashboard (control commands) | `DobotApiDashboard` |
| 30003 | Move (motion commands) | `DobotApiMove` |
| 30004 | Feedback (robot state) | `DobotApi` |
| 45678 | LabVIEW integration | `TCPServer` |

### Common Usage Pattern

```python
from dobot_api import DobotApiDashboard, DobotApiMove, DobotApi, MyType

# Connect to all three ports
dashboard = DobotApiDashboard("192.168.1.6", 29999)
move = DobotApiMove("192.168.1.6", 30003)
feed = DobotApi("192.168.1.6", 30004)

# Enable robot
dashboard.EnableRobot()

# Move to position
move.MovL(200, 300, -50, 180)

# Wait for completion
move.Sync()

# Read feedback data
data = feed.socket_dobot.recv(1440)
feedInfo = np.frombuffer(data, dtype=MyType)
print(feedInfo['tool_vector_actual'][0])
```

## Vision-Guided Operation Flow

The `KeyboardPressor` class orchestrates vision-guided robot operations:

1. **Initialize**: Load camera, robot, and YOLO model
2. **Inference Thread**: Continuously capture frames and run object detection
3. **Command Processing**: Receive commands via TCP (from LabVIEW)
   - `{"cmd": 1}` - Press detected key (0, 1, or 2)
   - `{"point": [x, y]}` - Press at specific pixel coordinates
4. **Pixel-to-Pose Conversion**: Convert pixel coordinates to robot delta poses using calibration parameters
5. **Motion Execution**: Move robot to target position and press

### YOLO Detection Classes

The system detects keyboard number keys using YOLOv8:
- Class 0: Number key 0
- Class 1: Number key 1
- Class 2: Number key 2

### TCP Server Commands (LabVIEW Integration)

The `TCPServer` (port 45678) accepts JSON commands:
```json
{"cmd": 1}           // Press detected key (value: 0, 1, or 2)
{"point": [x, y]}    // Press at specific pixel coordinates
```

### Calibration (cali.json)

```json
{
  "left_top": {"x": 0, "y": 0},           // Robot pose at camera view top-left
  "right_bottom": {"x": 0, "y": 0},       // Robot pose at camera view bottom-right
  "pix_left_top": {"x": 0, "y": 0},       // Pixel coordinates of top-left corner
  "pix_right_bottom": {"x": 0, "y": 0},   // Pixel coordinates of bottom-right corner
  "press_deep": 0,                         // Key press depth (mm)
  "cap_to_keyborad": 0,                    // Camera to keyboard distance (mm)
  "robot_on_keyboard": 0,                  // Robot Z when on keyboard
  "max_deep": 0,                           // Maximum press depth Z coordinate
  "init_pos": {"pix_x": 0, "pix_y": 0, "pos_x": 0, "pos_y": 0}  // Initial position
}
```

Run `doc/doc.md` for detailed calibration instructions.

## Error Handling

### Robot Modes
- `1`: INIT, `4`: DISABLED, `5`: ENABLE, `7`: RUNNING
- `9`: ERROR, `10`: PAUSE

### Clearing Errors
```python
dashboard.ClearError()
dashboard.Continue()  # Resume motion queue
```

Error codes are stored in `files/alarm_controller.json` and `files/alarm_servo.json`.

## Threading Model

### main.py (4 threads)
1. **Main thread** - Connects and sends commands
2. **Feedback thread** - Continuously reads 1440-byte state packets
3. **Error handling thread** - Monitors and clears errors
4. **Motion thread** - Executes movement commands

### keyboard_pressor.py (multiple threads)
1. **Main thread** - TCP server command processing
2. **Inference thread** - Camera capture and YOLO detection
3. **Feedback thread** - Robot state monitoring (in DobotMG400)
4. **Error handling thread** - Robot error monitoring (in DobotMG400)
5. **Camera thread** - RealSense frame capture (in RealSense435i)

Use `globalLockValue` (threading.Lock) to protect shared state:
- `current_actual` - Current TCP coordinates [x, y, z, r]
- `algorithm_queue` - Queue status flag
- `enableStatus_robot` - Robot enable status
- `robotErrorState` - Error state flag

## Important Notes

- Always ensure robot is in a safe position before running motion commands
- Use `Sync()` to block until motion queue completes
- If motion queue is blocked, call `ClearError()` then `Continue()`
- Feedback data validation: check `test_value == 0x123456789abcdef`
- Pixel-to-pose conversion uses `scale_x` and `scale_y` from calibration (note: X/Y axes are swapped in conversion)
- **MyType dtype structure** (1440 bytes feedback packet): Key fields include:
  - `tool_vector_actual`: Actual TCP position [x, y, z, rx, ry, rz]
  - `EnableStatus`: Robot enable status (1=enabled)
  - `ErrorStatus`: Error flag (1=error present)
  - `isRunQueuedCmd`: Motion queue execution status
  - `robot_mode`: Current robot mode

## Development Guidelines

### Adding New Motion Commands

1. Add method to `DobotApiMove` class in `dobot_api.py`
2. Use `sendRecvMsg()` for synchronous command/response
3. Test using Python interactive shell or add test script in `src/test/`

### Calibration Workflow

For detailed calibration instructions, see `doc/doc.md`:
1. **Camera setup**: Use Intel RealSense Viewer to adjust camera angle
2. **Initial position**: Mark the initial robot position in camera view
3. **Corner calibration**: Record pixel and robot coordinates for view corners
4. **Height calibration**: Measure camera-to-keyboard depth and robot Z positions

### Debugging Tips

- Check robot mode via feedback data before sending motion commands
- Use `dashboard.RobotMode()` to query current mode
- Monitor `ErrorStatus` in feedback thread for real-time error detection
- Use `doc/doc.md` for camera and robot calibration procedures
- Error codes are defined in `files/alarm_controller.json` and `files/alarm_servo.json`
