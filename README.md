# ST5 VAC-EI - Line-Following Robot

A comprehensive line-following robot system developed for CentraleSupélec's ST5 VAC Integration teaching module. This project combines computer vision, path planning, and embedded systems to create an autonomous robot capable of navigating complex paths using line detection and intersection recognition.

## Overview

This system integrates:
- **Computer Vision**: Real-time line detection and intersection recognition using OpenCV
- **Path Planning**: BFS-based pathfinding with dynamic replanning capabilities
- **Motor Control**: Arduino-based motor control with encoder feedback
- **Obstacle Detection**: Infrared sensor integration for obstacle avoidance
- **GUI Interface**: Tkinter-based control panel with live visualization
- **Autonomous Navigation**: PD controller for smooth line following

## Hardware Requirements

### Robot Components
- **Arduino board** (with motor control shield)
  - 2x DC motors with encoders
  - Motor driver (PWM control on pins 5, 6)
  - Directional control (pins 4, 7)
- **Raspberry Pi or PC** (for vision processing)
- **Camera module** (Raspberry Pi Camera or USB webcam)
- **Infrared distance sensor** (connected to A0)
- **Servo motor** (optional, pin 9)
- **Ultrasonic sensor** (optional, TRIG: pin 10, ECHO: pin 11)

### Track Requirements
- High-contrast line (black on white or white on black)
- Intersection markers for navigation (cross, T-junction, or border patterns)
- Recommended track: figure-eight pattern with intersections

## Software Requirements

### Python Dependencies
```
Python 3.7+
opencv-python (cv2)
numpy
pyserial
tkinter (usually included with Python)
```

### Installation
```bash
# Install Python dependencies
pip install opencv-python numpy pyserial

# For Raspberry Pi Camera support (optional)
pip install picamera

# Clone the repository
git clone https://github.com/rvfamaestre/st5-vac-ei.git
cd st5-vac-ei
```

### Arduino Setup
1. Open `serial_link.ino` in Arduino IDE
2. Verify pin configuration in `parameters.h` matches your hardware
3. Upload to Arduino board
4. Note the serial port (e.g., `/dev/ttyACM0` on Linux, `COM3` on Windows)

## Project Structure

```
st5-vac-ei/
├── dialogue.py              # Main control logic and CLI interface
├── menu_ui.py              # Tkinter GUI for robot control
├── intersections.py        # Intersection detection algorithms
├── infrared_utils.py       # Infrared sensor communication
├── serial_link.ino         # Arduino motor control firmware
├── parameters.h            # Arduino hardware pin configuration
├── huit.jpg                # Reference track image (figure-eight)
├── huit_obstacle.jpg       # Track with obstacle markers
└── suivi_ligne_huit_eleve_2022b.slx  # Simulink model (optional)
```

## Usage

### GUI Mode (Recommended)
Launch the graphical interface:
```bash
python dialogue.py
```

The GUI provides:
- **Connection panel**: Configure Arduino serial port and baudrate
- **Camera preview**: Live feed with line detection overlay
- **Control modes**:
  - Manual Arduino commands
  - Autonomous line following
  - Path following (predefined sequence)
  - A-to-B navigation with pathfinding
- **Grid visualization**: Track map with robot position

### CLI Mode
For command-line control:
```python
from dialogue import run_cli_menu
run_cli_menu()
```

Menu options:
1. **Direct Arduino dialogue**: Send raw commands to Arduino
2. **Autonomous line following**: Robot follows line with PD control
3. **Predefined path**: Follow sequence (L=left, R=right, S=straight, B=U-turn)
4. **Navigate A to B**: Automated pathfinding between grid positions

## Features

### Vision Processing
- **Adaptive thresholding**: Handles varying lighting conditions
- **HSV-based line detection**: Robust color segmentation
- **Morphological operations**: Noise reduction and line enhancement
- **Intersection detection**: Recognizes crosses, T-junctions, and borders
- **Region of Interest (ROI)**: Optimized processing area

### Control System
- **PD Controller**: Proportional-Derivative steering control
- **Centroid smoothing**: Exponential moving average (α=0.6)
- **Motor synchronization**: Differential drive for turns
- **Speed ramping**: Smooth acceleration/deceleration

### Path Planning
- **BFS Pathfinding**: Shortest path on grid graph
- **Dynamic replanning**: Adapts to blocked edges or obstacles
- **Intersection actions**: Automated turn execution at waypoints
- **Heading tracking**: Maintains orientation during navigation

### Safety Features
- **Infrared obstacle detection**: Automatic stop on collision risk
- **Connection monitoring**: Arduino heartbeat checks
- **Emergency stop**: Immediate motor cutoff (Q command)
- **Timeout handling**: Recovers from communication failures

## Configuration

### Camera Settings (in `dialogue.py`)
```python
CAMERA_RESOLUTION = (640, 480)  # Image resolution
CAMERA_FRAMERATE = 32           # Frames per second
```

### Line Detection Parameters
```python
# HSV color thresholds (dialogue.py)
LOW_HSV = np.array([0, 0, 0])      # Black line
HIGH_HSV = np.array([180, 255, 50])

# PD Controller gains
Kp = 0.8   # Proportional gain
Kd = 0.3   # Derivative gain
```

### Intersection Detection (in `intersections.py`)
```python
V_MIN = 0.20          # Vertical density threshold
H_MIN = 0.15          # Horizontal density threshold
MIN_HORIZ_WIDTH = 0.05  # Minimum width ratio
```

### Arduino Configuration (in `parameters.h`)
```c
#define SERIAL_BAUD 115200
#define IR_pin A0
#define motor1PWM 5
#define motor2PWM 6
```

## Communication Protocol

### Arduino Commands
- **A22**: Initialize connection (binary mode)
- **R**: Read infrared sensor
- **Motor commands**: 8-byte binary packets (left speed, right speed)
- **a**: Disconnect

### Binary Protocol
- Motor speeds: int16 (-255 to 255)
- Timestamps: int32 (milliseconds)
- Sensor readings: int16 (raw ADC values)

## Troubleshooting

### Camera Issues
- **No camera detected**: Check `/dev/video0` or camera module connection
- **Poor line detection**: Adjust HSV thresholds for lighting conditions
- **Slow framerate**: Reduce resolution or disable feedback overlay

### Arduino Connection
- **Port not found**: Check `ls /dev/ttyACM*` or `ls /dev/ttyUSB*`
- **Permission denied**: Add user to dialout group: `sudo usermod -a -G dialout $USER`
- **No response**: Verify baud rate matches (115200)

### Robot Behavior
- **Oscillation**: Reduce Kp or increase Kd
- **Slow response**: Increase Kp
- **Misses intersections**: Lower detection thresholds in `intersections.py`
- **False intersections**: Raise WHITE_RATIO_MIN or density thresholds

## Development

### Adding New Features
- **Custom intersection patterns**: Modify `detect_intersection_with_metrics()` in `intersections.py`
- **Alternative controllers**: Replace PD logic in `compute_steering_command()`
- **Sensor fusion**: Extend `infrared_utils.py` for additional sensors
- **Path strategies**: Implement new planning algorithms in `plan_path_bfs()`

### Testing
Test individual modules:
```python
# Test line detection
from dialogue import detect_line, init_camera
camera = init_camera()
image = capture_image(camera)
cx, cy, contour = detect_line(image, feedback=True)

# Test intersection detection
from intersections import detect_intersection_with_metrics
result = detect_intersection_with_metrics(image)

# Test infrared sensor
from infrared_utils import get_infrared_distance_cm
distance = get_infrared_distance_cm(arduino)
```

## Academic Context

This project is part of CentraleSupélec's **ST5 VAC-EI (Electronique et Informatique)** teaching module, focusing on:
- Embedded systems integration
- Real-time control systems
- Computer vision applications
- Hardware-software co-design

## License

Copyright (c) 2025 CentraleSupélec. Educational use permitted with attribution.
