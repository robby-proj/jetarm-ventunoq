[JETARM_VENTUNOQ_APPLICATION_GUIDE_UBUNTU.md](https://github.com/user-attachments/files/31443235/JETARM_VENTUNOQ_APPLICATION_GUIDE_UBUNTU.md)

# Ventuno Q + JetArm AI RoboArm Studio — Application & Integration Guide

**Version:** 1.1 (Ubuntu Edition)  
**Documented baseline:** Git commit `5a44208`  
**Repository:** `git@github.com:robby-proj/jetarm-ventunoq.git`  
**Target:** Ventuno Q + Hiwonder JetArm  
**Runtime:** Ubuntu / ROS 2 Jazzy / Arduino App Lab  
**Application:** `arduino-ai-roboarm-studio-v1-2`

> **Source of truth:** The Git repository and baseline commit are authoritative. Deploy source files from Git; do not recreate `main.py` or the bridge manually from documentation snippets.

## 1. Purpose

This Ubuntu edition allows another engineer to reproduce, commission, operate, tune, troubleshoot, and recover the complete JetArm solution on Ventuno Q. The pipeline is camera-based face/person detection in Arduino App Lab → UDP JSON → ROS 2 bridge → JetArm bus servo controller. WebUI and persistent voice controls share the same control path.

### 1.1 Supported platform and base prerequisites

This guide targets **Ventuno Q running Ubuntu**. Before deploying the application layer, confirm the base platform provides:

- Ubuntu on Ventuno Q with `systemd`.
- ROS 2 Jazzy installed under `/opt/ros/jazzy`.
- Hiwonder JetArm ROS 2 controller installed and able to expose `/ros_robot_controller`.
- Arduino App Lab capable of running `arduino-ai-roboarm-studio-v1-2`.
- Python 3, Git, and SSH access for repository deployment.
- The `arduino` and `hiwonder` users/paths used by this validated integration, or equivalent paths updated consistently in the service and deployment commands.

> **Important:** This guide deploys the integrated JetArm application on top of an Ubuntu Ventuno Q that already has the base JetArm ROS controller available. If a fresh Ubuntu image does not yet provide that controller, install and validate the JetArm ROS 2 base stack before enabling AI tracking.

## 2. Architecture

```text
CAMERA / VIDEO
      |
      v
Arduino App Lab
VideoObjectDetection + WebUI + Persistent ASR
      |
      | UDP JSON detection/control, port 5600
      v
/app_lab_face_tracking_bridge
app_lab_ros_bridge.py
      |
      | ros_robot_controller_msgs/ServosPosition
      v
/ros_robot_controller/bus_servo/set_position
      |
      v
JetArm ROS Controller -> Pan servo ID 1 / Tilt servo ID 4
```

The stable bridge starts disarmed and uses bounded normal tracking. Prediction, distance boost, and continuous stale-error integration are not part of the production design.

## 3. Key paths

```text
App Lab project:
/home/arduino/ArduinoApps/arduino-ai-roboarm-studio-v1-2

App Lab Python:
/home/arduino/ArduinoApps/arduino-ai-roboarm-studio-v1-2/python/main.py

ROS workspace:
/home/hiwonder/jetarm_ros2_ws

Bridge:
/home/hiwonder/jetarm_ros2_ws/app_lab_ros_bridge.py

Active tracker:
/home/hiwonder/jetarm_ros2_ws/config/face_tracker.yaml

Service:
/etc/systemd/system/app-lab-ros-bridge.service
```

## 4. Repository layout

```text
jetarm-ventunoq/
├── app_lab/
│   ├── app.yaml
│   ├── python/main.py
│   ├── assets/
│   └── README.md
├── ros2/
│   ├── app_lab_ros_bridge.py
│   ├── config/
│   │   ├── face_tracker.yaml
│   │   ├── face_tracker.PRODUCTION.yaml
│   │   └── face_tracker.COMMISSIONING.yaml
│   ├── src/JetArm/
│   └── tools/
├── systemd/app-lab-ros-bridge.service
└── docs/
    ├── VERSIONS.txt
    ├── ros-jazzy-packages.txt
    └── python-packages.txt
```

## 5. App Lab behavior

- Tracking labels: `face`, then `person` fallback.
- Detector baseline: confidence `0.50`, `debounce_sec=0.0`.
- Largest target for the first available tracking label is selected.
- Bounding-box center is normalized and sent over UDP.
- Tracking starts **OFF**.
- Persistent ASR is bound to `CARD=XFMDPV0018,DEV=0`.
- Voice control uses one persistent transcription stream.

### Voice phrases

| Intent | Examples |
|---|---|
| Emergency | emergency stop, stop robot, robot stop |
| Center | center robot, go to center, return to center |
| Tracking OFF | disable tracking, stop tracking, stop following me |
| Tracking ON | enable tracking, start tracking, follow me, track me |

## 6. Stable ROS bridge

- Node: `/app_lab_face_tracking_bridge`
- UDP: `0.0.0.0:5600`
- Servo topic: `/ros_robot_controller/bus_servo/set_position`
- Pan: ID 1; center 500
- Tilt: ID 4; center 200
- Starts `DISARMED`
- Stable file: 510 lines
- Stable SHA-256: `0a4f9bfeea0d31cb46b1691ff15febf673e26666bd96b511a0440f24c3a48cb6`

## 7. Production tracking profile

```yaml
/app_lab_face_tracking_bridge:
  ros__parameters:
    listen_host: "0.0.0.0"
    listen_port: 5600
    pan_servo_id: 1
    tilt_servo_id: 4
    pan_center: 500
    tilt_center: 200
    pan_min: 225
    pan_max: 775
    tilt_min: 120
    tilt_max: 400
    invert_pan: true
    invert_tilt: true
    pan_gain: 130.0
    tilt_gain: 50.0
    deadband_x: 0.025
    deadband_y: 0.030
    smoothing_alpha_x: 0.60
    smoothing_alpha_y: 0.45
    pan_max_step: 32
    tilt_max_step: 19
    minimum_servo_change: 1
    control_interval: 0.010
    movement_duration: 0.090
    face_timeout: 0.30
    start_armed: false
```

`movement_duration=0.090` is the validated smoothness/responsiveness point. `0.100` tested worse. The wide production envelope must not be used for first movement on an uncommissioned robot.

## 8. Commissioning profile

Use `face_tracker.COMMISSIONING.yaml` for every new arm:

- Pan: 425–575
- Tilt: 150–300
- Pan/Tilt gain: 55 / 12
- Deadband: 0.06 / 0.08
- Smoothing alpha: 0.22 / 0.15
- Max step: 10 / 5
- Minimum change: 2
- Control interval: 0.03 s
- Movement duration: 0.06 s
- Timeout: 0.5 s
- Start armed: false

## 9. New Ventuno Q deployment (Ubuntu)

Before cloning the application repository, verify that Ubuntu is running, ROS 2 Jazzy is available at `/opt/ros/jazzy`, and the base JetArm ROS controller has been installed. The application bridge depends on that base ROS layer.


### Clone

```bash
cd /home/arduino
git clone git@github.com:robby-proj/jetarm-ventunoq.git
cd jetarm-ventunoq
git checkout 5a44208
```

### Verify JetArm controller

```bash
sudo systemctl status jetarm-ros-controller.service --no-pager -l

sudo -u hiwonder bash -lc '
source /opt/ros/jazzy/setup.bash
[ -f /home/hiwonder/jetarm_ros2_ws/install/setup.bash ] && source /home/hiwonder/jetarm_ros2_ws/install/setup.bash
ros2 node list
'
```

Expected: `/ros_robot_controller`.

### Deploy ROS workspace

```bash
sudo systemctl stop app-lab-ros-bridge.service 2>/dev/null || true
sudo mkdir -p /home/hiwonder/jetarm_ros2_ws/{src,config}
sudo cp -a /home/arduino/jetarm-ventunoq/ros2/src/. /home/hiwonder/jetarm_ros2_ws/src/
sudo cp /home/arduino/jetarm-ventunoq/ros2/app_lab_ros_bridge.py /home/hiwonder/jetarm_ros2_ws/app_lab_ros_bridge.py
sudo cp /home/arduino/jetarm-ventunoq/ros2/config/face_tracker.COMMISSIONING.yaml /home/hiwonder/jetarm_ros2_ws/config/face_tracker.yaml
sudo chown -R hiwonder:hiwonder /home/hiwonder/jetarm_ros2_ws

sudo -u hiwonder bash -lc '
source /opt/ros/jazzy/setup.bash
cd /home/hiwonder/jetarm_ros2_ws
colcon build --symlink-install
'
```

### Install systemd bridge

```bash
sudo cp /home/arduino/jetarm-ventunoq/systemd/app-lab-ros-bridge.service /etc/systemd/system/app-lab-ros-bridge.service
sudo systemctl daemon-reload
sudo systemctl enable app-lab-ros-bridge.service
sudo systemctl start app-lab-ros-bridge.service
sleep 2
sudo journalctl -u app-lab-ros-bridge.service --since "1 minute ago" --no-pager
```

Expected: `DISARMED`, `NORMAL conservative face tracking ENABLED`, and UDP `0.0.0.0:5600`.

### Deploy App Lab

```bash
mkdir -p /home/arduino/ArduinoApps/arduino-ai-roboarm-studio-v1-2
cp -a /home/arduino/jetarm-ventunoq/app_lab/. /home/arduino/ArduinoApps/arduino-ai-roboarm-studio-v1-2/
```

Open App Lab and launch the project. Verify video/detection and WebUI before tracking.

## 10. Commissioning sequence

1. Start with `COMMISSIONING.yaml`.
2. Verify bridge starts disarmed.
3. Center robot.
4. Test horizontal direction slowly.
5. Test vertical direction slowly.
6. Verify no binding/cable strain at conservative limits.
7. Stop tracking.
8. Install production profile.
9. Restart bridge and center.
10. Expand tests gradually into the production envelope.

## 11. Switch to production profile

```bash
sudo systemctl stop app-lab-ros-bridge.service
sudo cp /home/arduino/jetarm-ventunoq/ros2/config/face_tracker.PRODUCTION.yaml /home/hiwonder/jetarm_ros2_ws/config/face_tracker.yaml
sudo chown hiwonder:hiwonder /home/hiwonder/jetarm_ros2_ws/config/face_tracker.yaml
sudo systemctl start app-lab-ros-bridge.service
```

## 12. Diagnostics

```bash
sudo systemctl status app-lab-ros-bridge.service --no-pager -l
sudo journalctl -u app-lab-ros-bridge.service -f
sudo ss -lunp | grep ':5600'

sudo -u hiwonder bash -lc '
source /opt/ros/jazzy/setup.bash
source /home/hiwonder/jetarm_ros2_ws/install/setup.bash
ros2 node list
ros2 topic info /ros_robot_controller/bus_servo/set_position
'
```

Verify bridge identity:

```bash
wc -l /home/hiwonder/jetarm_ros2_ws/app_lab_ros_bridge.py
sha256sum /home/hiwonder/jetarm_ros2_ws/app_lab_ros_bridge.py
grep -n "NORMAL conservative face tracking ENABLED" /home/hiwonder/jetarm_ros2_ws/app_lab_ros_bridge.py
```

## 13. Troubleshooting highlights

| Symptom | Action |
|---|---|
| No movement | Check bridge service, UDP 5600, ROS controller node, detector target, tracking enabled. |
| Wrong direction | Disable tracking; adjust `invert_pan` or `invert_tilt`. |
| Runaway/extreme motion | Emergency stop; restore stable bridge/profile; do not run old predictive/vector variants. |
| Throttle/brief pause | Detector cadence is the likely limit; keep `movement_duration=0.090` baseline. |
| Voice fails | Check XFM microphone binding and persistent ASR session. |
| New arm binds | Return to commissioning profile and reduce robot-specific travel limits. |

## 14. Git workflow

```bash
cd /home/arduino/jetarm-ventunoq
git status
git log --oneline --decorate -5
git pull --ff-only
```

Before any tuning, commit or branch from the known-good state. For the documented stable baseline:

```bash
git checkout 5a44208
```

## 15. Safety rules

- Keep `start_armed: false`.
- New robots always start with `COMMISSIONING.yaml`.
- Never change range + gain + max step in the same experiment.
- Always keep an emergency-stop path available while tuning.
- Do not deploy old prediction/vector-control experiments.
- Record robot-specific mechanical calibration in Git.

## 16. Release checklist

- [ ] Correct Git commit/tag deployed
- [ ] JetArm controller active
- [ ] `/ros_robot_controller` visible
- [ ] Bridge active and disarmed
- [ ] UDP 5600 listening
- [ ] App Lab camera/detection works
- [ ] WebUI tracking on/off works
- [ ] Center works
- [ ] Emergency stop works
- [ ] Voice commands work (if fitted)
- [ ] Pan direction verified
- [ ] Tilt direction verified
- [ ] Conservative envelope mechanically safe
- [ ] Production profile installed only after commissioning
- [ ] Wide range tested gradually
- [ ] Robot-specific differences committed/documented
