# Arduino AI RobotArm Studio

## AI Face Tracking Robot Arm with Arduino App Lab, Qualcomm AI, and ROS 2

Arduino AI RobotArm Studio demonstrates how an **Arduino VENTUNO Q** can run an end-to-end edge AI robotics application.

The project combines:

- Arduino App Lab
- Qualcomm-accelerated AI inference
- Arduino WebUI
- ROS 2 Jazzy
- Hiwonder JetArm
- USB camera input

The system detects a person or face in a live camera stream, calculates the target position, and commands the robotic arm to follow the target in real time.

All AI processing, user-interface hosting, ROS 2 communication, and robot control run locally on the VENTUNO Q.

---

## Project Overview

The application uses Arduino App Lab as the AI and user-interface layer, while ROS 2 handles deterministic robot control.

```text
USB Camera
    │
    ▼
Arduino App Lab
Video Object Detection Brick
Qualcomm AI Runner
    │
    ├── Live WebUI camera stream
    │
    └── Detection bounding boxes
              │
              ▼
        Python main.py
              │
              ▼
       UDP JSON messages
              │
              ▼
   app_lab_ros_bridge.py
              │
              ▼
          ROS 2 Jazzy
              │
              ▼
 ros_robot_controller
              │
              ▼
      Hiwonder JetArm
```

---

## Main Features

- Live USB-camera stream in the App Lab WebUI
- Qualcomm-accelerated object or face detection
- Real-time bounding-box visualization
- Target confidence and position display
- Enable and disable tracking from the browser
- Center Robot command
- Emergency Stop command
- ROS 2 servo control
- Persistent robot calibration through a YAML file
- Automatic ROS services at system startup
- Optional App Lab **Run at startup** support
- Safe startup with tracking disabled

---

## Architecture

The application is divided into two primary layers.

### Arduino App Lab Layer

Arduino App Lab manages:

- USB-camera acquisition
- AI inference
- Live video streaming
- WebUI controls
- Detection events
- UDP communication with the ROS 2 bridge

### ROS 2 Layer

ROS 2 manages:

- UDP detection reception
- Tracking enable and disable state
- Detection smoothing
- Deadband filtering
- Servo motion limits
- Pan and tilt calibration
- Servo command publication
- Robot safety behavior

This separation keeps AI application development simple while preserving reliable robot control.

---

## System Components

| Component | Responsibility |
|---|---|
| USB Camera | Captures the live video stream |
| VENTUNO Q | Runs App Lab, AI inference, WebUI, ROS 2, and robot control |
| Video Object Detection Brick | Performs accelerated AI inference |
| WebUI Brick | Hosts the browser-based dashboard |
| `python/main.py` | Processes detections and sends UDP messages |
| `app_lab_ros_bridge.py` | Converts detections and UI commands into ROS 2 actions |
| `ros_robot_controller` | Publishes commands to the Hiwonder servo bus |
| Hiwonder JetArm | Executes pan and tilt movement |

---

## Hardware Requirements

- Arduino VENTUNO Q
- Hiwonder JetArm
- USB camera
- Powered USB-C hub
- External 5 V power supply for the hub
- Network connection
- Computer or mobile device with a web browser

---

## Software Requirements

- Arduino App Lab
- Ubuntu on VENTUNO Q
- ROS 2 Jazzy
- Python 3
- Arduino App Bricks
- `video_objectdetection` Brick
- `web_ui` Brick
- Hiwonder ROS 2 robot controller packages

---

## Project Structure

```text
ArduinoApps/
└── arduino-ai-robotarm-studio/
    ├── app.yaml
    ├── README.md
    ├── python/
    │   └── main.py
    └── assets/
        ├── index.html
        ├── app.js
        ├── style.css
        ├── img/
        └── docs_assets/

jetarm_ros2_ws/
├── app_lab_ros_bridge.py
├── config/
│   └── face_tracker.yaml
├── src/
├── build/
├── install/
└── log/
```

---

## Data Flow

### Detection Data

The App Lab AI runner returns detections containing a class label, confidence score, and bounding box.

Example:

```python
{
    "person": [
        {
            "confidence": 0.91,
            "bounding_box_xyxy": (120, 85, 390, 470)
        }
    ]
}
```

The application calculates the center of the bounding box:

```python
center_x = (x1 + x2) / 2
center_y = (y1 + y2) / 2
```

The coordinates are normalized:

```python
normalized_x = center_x / frame_width
normalized_y = center_y / frame_height
```

A UDP packet is then sent to the ROS 2 bridge.

```json
{
  "message_type": "detection",
  "label": "person",
  "confidence": 0.91,
  "normalized_x": 0.52,
  "normalized_y": 0.44,
  "frame_width": 640,
  "frame_height": 480
}
```

---

## WebUI Control Messages

The WebUI communicates with `python/main.py` using App Lab WebUI messages.

### Enable Tracking

```javascript
ui.send_message('set_tracking', {
  enabled: true
});
```

The backend sends this UDP message:

```json
{
  "message_type": "control",
  "command": "set_tracking",
  "enabled": true
}
```

### Disable Tracking

```json
{
  "message_type": "control",
  "command": "set_tracking",
  "enabled": false
}
```

### Center Robot

```json
{
  "message_type": "control",
  "command": "center"
}
```

### Emergency Stop

```json
{
  "message_type": "control",
  "command": "emergency_stop"
}
```

---

## ROS 2 Bridge

The ROS 2 bridge listens for UDP messages on port `5600`.

```text
0.0.0.0:5600
```

It processes two message categories:

- `detection`
- `control`

The bridge publishes servo commands to:

```text
/ros_robot_controller/bus_servo/set_position
```

Message type:

```text
ros_robot_controller_msgs/msg/ServosPosition
```

Servo mapping:

| Function | Servo ID |
|---|---:|
| Horizontal pan | 1 |
| Vertical tilt | 4 |

---

## Robot Calibration

The robot calibration is stored in:

```text
/home/hiwonder/jetarm_ros2_ws/config/face_tracker.yaml
```

Example:

```yaml
/app_lab_face_tracking_bridge:
  ros__parameters:
    listen_host: "0.0.0.0"
    listen_port: 5600

    pan_servo_id: 1
    tilt_servo_id: 4

    pan_center: 500
    tilt_center: 200

    pan_min: 425
    pan_max: 575
    tilt_min: 150
    tilt_max: 300

    invert_pan: true
    invert_tilt: true

    pan_gain: 55.0
    tilt_gain: 12.0

    deadband_x: 0.06
    deadband_y: 0.08

    smoothing_alpha_x: 0.22
    smoothing_alpha_y: 0.15

    pan_max_step: 10
    tilt_max_step: 5

    minimum_servo_change: 2

    control_interval: 0.03
    movement_duration: 0.06

    face_timeout: 0.5
    start_armed: false
```

> Keep `start_armed: false` so the robot never begins moving automatically after boot.

---

## Running the Project Manually

### 1. Start the ROS 2 Robot Controller

```bash
cd ~/jetarm_ros2_ws

source /opt/ros/jazzy/setup.bash
source ~/jetarm_ros2_ws/install/setup.bash

ros2 launch ros_robot_controller \
  ros_robot_controller.launch.py
```

### 2. Start the App Lab ROS Bridge

```bash
cd ~/jetarm_ros2_ws

source /opt/ros/jazzy/setup.bash
source ~/jetarm_ros2_ws/install/setup.bash

python3 app_lab_ros_bridge.py \
  --ros-args \
  --params-file ~/jetarm_ros2_ws/config/face_tracker.yaml
```

### 3. Start the App Lab Project

Open Arduino App Lab and run:

```text
Arduino AI RobotArm Studio
```

### 4. Open the WebUI

```text
http://<VENTUNO-IP>:7000
```

Example:

```text
http://192.168.1.68:7000
```

### 5. Enable Tracking

Use the **Enable Tracking** switch in the WebUI.

---

## Automatic Startup

Arduino App Lab includes a **Run at startup** switch. Enable it for this project.

The ROS 2 controller and bridge can run as systemd services.

### ROS 2 Controller Service

```text
jetarm-ros-controller.service
```

### App Lab ROS Bridge Service

```text
app-lab-ros-bridge.service
```

Check whether they are enabled:

```bash
systemctl is-enabled jetarm-ros-controller.service
systemctl is-enabled app-lab-ros-bridge.service
```

Expected result:

```text
enabled
enabled
```

The boot sequence is:

```text
VENTUNO Q boots
    │
    ├── ROS 2 robot controller starts
    │
    ├── App Lab ROS bridge starts
    │
    ├── Arduino App Lab starts the project
    │
    ├── Camera and Qualcomm AI runner start
    │
    └── WebUI becomes available on port 7000
```

Tracking remains disabled until the user enables it from the WebUI.

---

## WebUI

The WebUI provides:

- Live camera feed
- Detection bounding boxes
- Confidence threshold slider
- Recent detection list
- Enable Tracking switch
- Center Robot button
- Emergency Stop button
- Target class
- Target confidence
- Target coordinates
- Robot connection messages

The frontend uses:

```javascript
const ui = new WebUI();
```

Browser-to-backend communication:

```javascript
ui.send_message('set_tracking', {
  enabled: true
});
```

Backend-to-browser communication:

```python
ui.send_message(
    "tracking_state",
    message={
        "enabled": True
    }
)
```

---

## Motion Control

The ROS 2 bridge converts target displacement into incremental servo movement.

```text
Detection center
    │
    ▼
Normalized X and Y
    │
    ▼
Low-pass filtering
    │
    ▼
Deadband
    │
    ▼
Pan and tilt gain
    │
    ▼
Maximum step limits
    │
    ▼
Servo range limits
    │
    ▼
ROS 2 servo command
```

The pan and tilt axes use independent parameters because the vertical axis normally requires stronger smoothing and lower speed.

---

## Safety

The project includes several safety mechanisms:

- Tracking starts disabled
- WebUI enable and disable control
- Center Robot command
- Emergency Stop command
- Detection timeout
- Servo position limits
- Pan and tilt movement limits
- Stale-target rejection
- Automatic disarm on App Lab restart

Keep the robot clear of people and objects during calibration.

Never use unverified full servo ranges such as `0` to `1000` without confirming the robot's mechanical limits.

---

## Troubleshooting

### WebUI Does Not Open

Use the VENTUNO Q network address rather than `127.0.0.1`.

```text
http://<VENTUNO-IP>:7000
```

Check the listener:

```bash
ss -ltnp | grep 7000
```

### Robot Does Not Move

Confirm both ROS nodes are active:

```bash
ros2 node list
```

Expected nodes include:

```text
/app_lab_face_tracking_bridge
/ros_robot_controller
```

Check servo commands:

```bash
ros2 topic echo \
  /ros_robot_controller/bus_servo/set_position
```

### WebUI Toggle Changes but Robot Does Not Move

Monitor bridge logs:

```bash
sudo journalctl \
  -u app-lab-ros-bridge.service \
  -f
```

Expected message:

```text
WEBUI SET TRACKING: ENABLED
```

### UDP Packets Do Not Reach the Bridge

Confirm UDP port `5600` is listening:

```bash
ss -lunp | grep 5600
```

Test packets directly:

```bash
nc -ul 5600
```

The App Lab project must use the Docker host gateway as `ROS_BRIDGE_HOST`.

### Robot Moves in the Opposite Direction

Reverse the affected axis:

```yaml
invert_pan: true
invert_tilt: true
```

### Robot Shakes or Oscillates

Adjust:

```yaml
deadband_x
deadband_y
smoothing_alpha_x
smoothing_alpha_y
pan_max_step
tilt_max_step
minimum_servo_change
```

Increase the deadband or reduce the smoothing alpha to filter more detection noise.

---

## Performance Design

The video stream is not sent through ROS 2.

Instead:

- App Lab owns the camera stream.
- Qualcomm AI performs inference.
- The WebUI receives the live video.
- ROS 2 receives only normalized target coordinates.

This significantly reduces CPU use, memory copies, and latency.

```text
Camera and WebUI: high-frame-rate path
Detection coordinates: low-bandwidth UDP path
Servo controller: independent ROS 2 control path
```

---

## Extending the Project

The same architecture can support additional robot skills:

- Face tracking
- Person following
- Gesture control
- Hand tracking
- Object following
- QR-code actions
- AprilTag tracking
- Voice commands
- Edge Impulse custom models
- Multi-robot ROS 2 control

The AI model can change without redesigning the ROS 2 robot-control layer.