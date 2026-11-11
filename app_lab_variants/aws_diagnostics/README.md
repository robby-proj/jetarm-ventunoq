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

---

## Deployment Variants

This project exists as four Arduino App Lab apps on this board, all sharing the same face-tracking/ROS 2 core above but differing in what sits on top of it. Only one app runs at a time; clone further variants from V1.2 if you need to branch again.

| App | Adds |
|---|---|
| **Arduino AI RoboArm Studio V1.2** | Base app only, no cloud telemetry |
| **Arduino AI RoboArm Studio - Arduino Cloud** | Arduino Cloud Brick |
| **Arduino AI RoboArm Studio - AWS** | AWS IoT Core via a custom Python module |
| **Arduino AI RoboArm Studio - AWS Diagnostics** (this app) | Everything in the AWS app, plus a local LLM diagnostics assistant reachable from AWS IoT Core and from a chat box in the local WebUI |

---

## Arduino Cloud Integration (Arduino Cloud app)

Uses the catalog `arduino:arduino_cloud` Brick (`ArduinoCloud` class). Registers 7 cloud variables mirroring the robot's live state:

- `tracking_enabled` (bool, two-way — writable from the Cloud dashboard, calls back into `set_tracking()`)
- `robot_status` (string: idle / tracking / centered / emergency_stop)
- `target_label`, `target_confidence`, `normalized_x`, `normalized_y` — current detection target, throttled to 2 Hz
- `last_voice_command` — every transcribed voice phrase, not just matched robot commands

Setup: connect the board via App Lab **Settings → Arduino Cloud → Connect device**, then create a Thing with matching Cloud Variables (same names/types) — data won't sync until those exist. The Brick talks to a local `arduino-cloud-connector` daemon running on the board OS; the app never touches credentials directly.

Startup safety: `set_tracking(False, source="system")` runs explicitly on boot so a stale `tracking_enabled: true` left in the Cloud from a previous session can't leave the dashboard out of sync with the robot's real (disabled) state.

---

## AWS IoT Core Integration (AWS app)

No catalog Brick exists for AWS, and a real one can't be published through App Lab's app-level tools (see below). Instead, `python/aws_iot_bridge.py` is a small, self-contained `paho-mqtt` wrapper (`AWSIoTBridge` class) deliberately shaped like `ArduinoCloud` — `publish()` / `publish_state()` for device→cloud, `on_write()` for cloud→device. It's a plain file: copy it, plus `paho-mqtt==1.6.1` in `python/requirements.txt`, into any other App Lab app to reuse it.

Publishes the same 7 variables as the Cloud app, as one JSON document to `arduino/roboarm/state` (2 Hz throttle), and subscribes to `arduino/roboarm/tracking_enabled/set` for two-way control.

Setup required in the AWS console (no tool access from App Lab for this part):

1. **IoT Core → Settings** — copy the account's device data endpoint and provide it at runtime as `AWS_IOT_ENDPOINT`. The public source intentionally contains no account-specific endpoint.
2. **Create a Thing**, auto-generate a certificate, **activate** it, and attach a policy granting at minimum `iot:Connect` / `iot:Publish` / `iot:Subscribe` / `iot:Receive`.
3. Download the device certificate, private key, and public `AmazonRootCA1.pem` root CA and install them **only in the local App Lab project** under `python/certs/` as `device-certificate.pem.crt`, `private.pem.key`, and `AmazonRootCA1.pem`. These files are intentionally excluded from the public repository; see `python/certs/README.md`.

**Known gotcha**: TLS/mutual-auth can succeed (certificate Active, policy correct) while MQTT CONNECT is dropped (`rc=7`, `MQTT_ERR_CONN_LOST`) if the region in the configured endpoint does not match the IoT Core region. Always copy the exact endpoint from **IoT Core → Settings** and provide it as `AWS_IOT_ENDPOINT`.

---

## AI Diagnostics Integration (this app)

Adds a local LLM diagnostics assistant on top of everything in the AWS app, using the catalog `arduino:llm` Brick (`LargeLanguageModel` class) running `Qwen3.5-0.8B-Q4_0` on the CPU-based `llamacpp` runtime — deliberately a small model on CPU rather than the larger Qwen 3-4B on the Qualcomm NPU, because the NPU is already shared by the face-detection and ASR models and loading a third large model onto it caused memory-cleanup failures (`status_code=500`) on first use. For an occasional diagnostics query, a small CPU model is plenty and avoids that contention entirely.

**Two independent entry points, one shared pipeline.** Both call the same `ask_diagnostics_llm()` → `gather_diagnostics()` → `llm.chat()` → `sanitize_llm_response()` path in `python/main.py`, so answers are consistent regardless of which one is used:

- **AWS IoT Core**: publish `{"prompt": "..."}` to `arduino/roboarm/diagnostics_request/set`, read the answer from `arduino/roboarm/diagnostics_response` (wrapped as `{"value": {"prompt": ..., "response": ..., "timestamp": ...}}`).
- **Local WebUI**: a chat box ("Diagnostics Assistant" card) at the bottom of the dashboard. Backed by `ui.on_message("diagnostics_chat", ...)` / `ui.send_message("diagnostics_chat_response", ...)`.

Both handlers run the LLM call on a background thread, never on the MQTT network thread or the WebUI's own message-handling thread — local inference can take a few seconds and must not block tracking toggles or other messages arriving in the meantime.

**Grounded in live counters, not hallucinated diagnoses.** `gather_diagnostics()` returns a JSON snapshot covering UDP bridge health (success/failure/consecutive-failure counts), tracking/robot status, voice/ASR state, the last detection (label, confidence, low-confidence count), AWS connection state, and system CPU/mem/disk (via `psutil`, added to `python/requirements.txt`). The system prompt explicitly tells the model to answer only from this data, to suggest checking the robot or recalibrating only when the counters actually support it (repeated emergency stops, UDP failures, or a run of low-confidence detections), and to reply in plain continuous prose — no markdown, bullets, or headers. A `sanitize_llm_response()` regex pass strips any markdown the model adds anyway, as a guaranteed fallback.

**What this can't do**: it can't detect actual servo wear or calibration drift directly — that calibration lives in the ROS 2 YAML config this app doesn't touch. "Service/calibration" suggestions are the LLM interpreting the tracked symptom counters, not a hardware fault diagnosis.

---

## Why a Python Module Instead of a "Brick"

Three shapes were considered for both cloud integrations:

1. **Catalog Brick** (`from arduino.app_bricks.X import ...`, listed in `bricks_list`) — not creatable through App Lab's tools; requires Arduino's own brick-publishing pipeline.
2. **Local custom brick** (`bricks/<id>/brick_compose.yaml`, a sidecar Docker container scoped to one app — the pattern the built-in MCP Gateway example uses) — technically possible, but adds an on-device Docker build and an internal API for no functional gain: still no secrets-UI, and certs still live in the app folder either way.
3. **Plain reusable Python module** — chosen for AWS. `arduino_cloud` itself is only as simple as it looks because of a privileged, OS-level `arduino-cloud-connector` daemon built into the board image by Arduino, plus a dedicated App Lab Settings provisioning flow — a mechanism outside what app-level tooling can replicate.