# SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l.
# SPDX-License-Identifier: MPL-2.0

import json
import os
import socket
import threading
import time
from datetime import UTC, datetime

from arduino.app_bricks.asr import AutomaticSpeechRecognition
from arduino.app_bricks.video_objectdetection import VideoObjectDetection
from arduino.app_bricks.web_ui import WebUI
from arduino.app_peripherals.microphone import Microphone
from arduino.app_utils import App


# ===========================================================================
# Configuration
# ===========================================================================

ROS_BRIDGE_HOST = os.getenv("ROS_BRIDGE_HOST", "172.21.0.1")
ROS_BRIDGE_PORT = int(os.getenv("ROS_BRIDGE_PORT", "5600"))
FRAME_WIDTH = int(os.getenv("FRAME_WIDTH", "640"))
FRAME_HEIGHT = int(os.getenv("FRAME_HEIGHT", "480"))

TRACK_LABELS = ("face", "person")
DETECTION_SEND_INTERVAL = 0.025
ASR_STARTUP_DELAY = 10.0
VOICE_COMMAND_COOLDOWN = 1.5

# Explicit XFM-DP binding. This is the stable ALSA card name we validated.
XFM_MIC_DEVICE = "CARD=XFMDPV0018,DEV=0"
XFM_SAMPLE_RATE = 16000
XFM_CHANNELS = 1
XFM_BUFFER_SIZE = 2000


# ===========================================================================
# App Lab bricks
# ===========================================================================

ui = WebUI()

detection_stream = VideoObjectDetection(
    confidence=0.50,
    debounce_sec=0.0,
)

# Bind App Lab to the XFM-DP microphone instead of the first USB mic.
xfm_mic = Microphone(
    device=XFM_MIC_DEVICE,
    sample_rate=XFM_SAMPLE_RATE,
    channels=XFM_CHANNELS,
    buffer_size=XFM_BUFFER_SIZE,
    shared=True,
    auto_reconnect=True,
)

# Since we pass mic=xfm_mic, this application owns the microphone lifecycle.
asr = AutomaticSpeechRecognition(
    mic=xfm_mic,
    language="en",
)

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


# ===========================================================================
# Runtime state
# ===========================================================================

last_detection_send_time = 0.0
tracking_requested = False

# ASR runs once, persistently. The UI toggle only enables/disables commands.
voice_commands_enabled = True
asr_stream_running = False
asr_thread = None
asr_start_lock = threading.Lock()

last_voice_command = ""
last_voice_command_time = 0.0


# ===========================================================================
# UDP / ROS helpers
# ===========================================================================

def send_udp(payload: dict) -> bool:
    try:
        encoded_payload = json.dumps(payload).encode("utf-8")
        udp_socket.sendto(
            encoded_payload,
            (ROS_BRIDGE_HOST, ROS_BRIDGE_PORT),
        )
        return True
    except Exception as exc:
        print(f"[WARN] UDP send failed: {exc}", flush=True)
        return False


def send_control(command: str, **values) -> bool:
    payload = {
        "message_type": "control",
        "command": command,
        "timestamp": time.time(),
        **values,
    }

    success = send_udp(payload)

    if success:
        print(
            f"[CONTROL] command={command} values={values}",
            flush=True,
        )

    return success


# ===========================================================================
# Robot control
# ===========================================================================

def set_tracking(enabled: bool, source: str = "system") -> None:
    global tracking_requested

    enabled = bool(enabled)
    tracking_requested = enabled

    success = send_control("set_tracking", enabled=enabled)

    print(
        f"[TRACKING] source={source} enabled={enabled} success={success}",
        flush=True,
    )

    try:
        ui.send_message("tracking_state", message={"enabled": enabled})
        ui.send_message(
            "robot_notice",
            message={
                "text": "Tracking enabled" if enabled else "Tracking disabled",
                "level": "success" if enabled else "info",
            },
        )
    except Exception as exc:
        print(f"[WARN] Tracking UI update failed: {exc}", flush=True)


def center_robot(source: str = "system") -> None:
    global tracking_requested

    tracking_requested = False
    send_control("center")

    try:
        ui.send_message("tracking_state", message={"enabled": False})
        ui.send_message(
            "robot_notice",
            message={"text": "Center command sent", "level": "info"},
        )
    except Exception:
        pass

    print(f"[ROBOT] source={source} CENTER", flush=True)


def emergency_stop_robot(source: str = "system") -> None:
    global tracking_requested

    tracking_requested = False
    send_control("emergency_stop")

    try:
        ui.send_message("tracking_state", message={"enabled": False})
        ui.send_message(
            "robot_notice",
            message={"text": "Emergency stop activated", "level": "error"},
        )
    except Exception:
        pass

    print(f"[ROBOT] source={source} EMERGENCY STOP", flush=True)


# ===========================================================================
# Voice helpers
# ===========================================================================

def send_voice_state() -> None:
    try:
        ui.send_message(
            "voice_state",
            message={
                # Keep compatibility with the current app.js toggle.
                "listening": voice_commands_enabled,
                "stream_active": asr_stream_running,
                "microphone": "XFMDPV0018",
            },
        )
    except Exception as exc:
        print(f"[VOICE WARN] Could not update voice UI: {exc}", flush=True)


def normalize_voice_text(text) -> str:
    if text is None:
        return ""

    text = str(text).strip().lower()

    for character in (".", ",", "!", "?", ":", ";"):
        text = text.replace(character, "")

    return " ".join(text.split())


def voice_command_allowed(command_name: str) -> bool:
    global last_voice_command
    global last_voice_command_time

    now = time.monotonic()

    if (
        command_name == last_voice_command
        and now - last_voice_command_time < VOICE_COMMAND_COOLDOWN
    ):
        print(
            f"[VOICE] Duplicate command suppressed: {command_name}",
            flush=True,
        )
        return False

    last_voice_command = command_name
    last_voice_command_time = now
    return True


# ===========================================================================
# Voice command parser
# ===========================================================================

def process_voice_command(raw_text) -> None:
    text = normalize_voice_text(raw_text)

    if not text:
        return

    print(f"[VOICE TEXT] {text}", flush=True)

    try:
        ui.send_message(
            "voice_transcription",
            message={"text": text},
        )
    except Exception:
        pass

    # ASR continues even when robot voice commands are muted.
    if not voice_commands_enabled:
        print(f"[VOICE IGNORED] Commands disabled: {text!r}", flush=True)
        return

    emergency_phrases = (
        "emergency stop",
        "stop robot",
        "robot stop",
    )

    if any(phrase in text for phrase in emergency_phrases):
        if voice_command_allowed("emergency_stop"):
            print("[VOICE COMMAND] EMERGENCY STOP", flush=True)
            emergency_stop_robot(source="voice")
        return

    center_phrases = (
        "center robot",
        "centre robot",
        "center the robot",
        "centre the robot",
        "robot center",
        "go to center",
        "return to center",
    )

    if any(phrase in text for phrase in center_phrases):
        if voice_command_allowed("center"):
            print("[VOICE COMMAND] CENTER", flush=True)
            center_robot(source="voice")
        return

    tracking_off_phrases = (
        "disable tracking",
        "stop tracking",
        "tracking off",
        "turn tracking off",
        "stop face tracking",
        "stop following",
        "stop following me",
    )

    if any(phrase in text for phrase in tracking_off_phrases):
        if voice_command_allowed("tracking_off"):
            print("[VOICE COMMAND] TRACKING OFF", flush=True)
            set_tracking(False, source="voice")
        return

    tracking_on_phrases = (
        "enable tracking",
        "start tracking",
        "tracking on",
        "turn tracking on",
        "start face tracking",
        "follow me",
        "track me",
    )

    if any(phrase in text for phrase in tracking_on_phrases):
        if voice_command_allowed("tracking_on"):
            print("[VOICE COMMAND] TRACKING ON", flush=True)
            set_tracking(True, source="voice")
        return

    print(f"[VOICE] No robot command matched: {text!r}", flush=True)


# ===========================================================================
# Persistent ASR worker
# ===========================================================================

def persistent_asr_worker() -> None:
    """
    Open exactly one XFM microphone stream and one ASR transcription session.

    This addresses the two failures we isolated:
      - wrong USB microphone selection
      - ASRBusyError from starting a second transcription session
    """

    global asr_stream_running

    print("[VOICE] Persistent ASR worker starting", flush=True)

    try:
        print(
            f"[VOICE] Starting XFM microphone: {XFM_MIC_DEVICE}",
            flush=True,
        )

        # We supplied mic=xfm_mic, so we own start()/stop().
        xfm_mic.start()

        print("[VOICE] XFM microphone started", flush=True)

        # Exactly ONE ASR transcription session.
        with asr.transcribe_stream() as stream:
            asr_stream_running = True

            print("[VOICE] ==========================================", flush=True)
            print("[VOICE] Persistent ASR stream ACTIVE", flush=True)
            print("[VOICE] Microphone: XFMDPV0018", flush=True)
            print("[VOICE] Stable ref: CARD=XFMDPV0018,DEV=0", flush=True)
            print(
                "[VOICE] Robot voice commands: "
                + ("ENABLED" if voice_commands_enabled else "DISABLED"),
                flush=True,
            )
            print("[VOICE] ==========================================", flush=True)

            send_voice_state()

            for chunk in stream:
                chunk_type = getattr(chunk, "type", "unknown")
                chunk_text = getattr(chunk, "data", "")

                print(
                    f"[ASR] type={chunk_type} text={chunk_text!r}",
                    flush=True,
                )

                try:
                    ui.send_message(
                        "transcription",
                        message={
                            "type": chunk_type,
                            "text": chunk_text,
                        },
                    )
                except Exception as exc:
                    print(
                        f"[VOICE WARN] Transcription UI send failed: {exc}",
                        flush=True,
                    )

                if chunk_text:
                    process_voice_command(chunk_text)

    except Exception as exc:
        print(
            f"[VOICE ERROR] Persistent ASR failed: "
            f"{type(exc).__name__}: {exc}",
            flush=True,
        )

        try:
            ui.send_message(
                "robot_notice",
                message={
                    "text": "ASR stream stopped: " + str(exc),
                    "level": "error",
                },
            )
        except Exception:
            pass

    finally:
        asr_stream_running = False

        try:
            xfm_mic.stop()
            print("[VOICE] XFM microphone stopped", flush=True)
        except Exception as exc:
            print(
                f"[VOICE WARN] Could not stop XFM microphone: {exc}",
                flush=True,
            )

        send_voice_state()
        print("[VOICE] Persistent ASR worker STOPPED", flush=True)


def start_persistent_asr_once() -> None:
    global asr_thread

    with asr_start_lock:
        if asr_thread is not None and asr_thread.is_alive():
            print("[VOICE] Persistent ASR already running", flush=True)
            return

        asr_thread = threading.Thread(
            target=persistent_asr_worker,
            daemon=True,
            name="ventuno-xfm-asr",
        )
        asr_thread.start()

    print("[VOICE] Persistent ASR startup requested", flush=True)


def delayed_asr_startup() -> None:
    print(
        f"[VOICE] ASR will start in {ASR_STARTUP_DELAY:.1f} seconds",
        flush=True,
    )

    time.sleep(ASR_STARTUP_DELAY)
    start_persistent_asr_once()


# ===========================================================================
# WebUI voice controls
# ===========================================================================

def start_dictation(session_id, data):
    """Enable robot voice commands. Do not start another ASR session."""

    global voice_commands_enabled

    voice_commands_enabled = True

    print("[VOICE] Robot voice commands ENABLED", flush=True)
    send_voice_state()

    try:
        ui.send_message(
            "robot_notice",
            message={"text": "Voice commands enabled", "level": "success"},
        )
    except Exception:
        pass


def stop_dictation(session_id, data):
    """Mute robot voice commands while keeping the persistent ASR alive."""

    global voice_commands_enabled

    voice_commands_enabled = False

    print("[VOICE] Robot voice commands DISABLED", flush=True)
    print("[VOICE] ASR stream remains active", flush=True)

    send_voice_state()

    try:
        ui.send_message(
            "robot_notice",
            message={"text": "Voice commands disabled", "level": "info"},
        )
    except Exception:
        pass


def new_recording(session_id, data):
    # Kept only for compatibility with the Edge Dictation frontend.
    # Never cancel/restart the persistent ASR session here.
    print(
        "[VOICE] new_recording ignored (persistent ASR mode)",
        flush=True,
    )


def set_language(session_id, data):
    try:
        language = data.get("language")

        if not language:
            return

        asr.language = language
        print(f"[VOICE] ASR language={language}", flush=True)

    except Exception as exc:
        print(f"[VOICE ERROR] Language change failed: {exc}", flush=True)


# ===========================================================================
# WebUI robot callbacks
# ===========================================================================

def parse_enabled(value) -> bool:
    if isinstance(value, dict):
        value = value.get("enabled", value.get("value", False))

    if isinstance(value, str):
        return value.strip().lower() in {
            "true",
            "1",
            "yes",
            "on",
            "enabled",
        }

    return bool(value)


def set_tracking_callback(sid, value):
    print(
        f"[WEBUI RX] set_tracking sid={sid} value={value!r}",
        flush=True,
    )

    enabled = parse_enabled(value)
    set_tracking(enabled, source="webui")


def center_robot_callback(sid, value):
    center_robot(source="webui")


def emergency_stop_callback(sid, value):
    emergency_stop_robot(source="webui")


def request_status_callback(sid, value):
    send_control("request_status")


def override_threshold(sid, threshold):
    try:
        detection_stream.override_threshold(float(threshold))
        print(
            f"[WEBUI] Detection threshold={float(threshold):.2f}",
            flush=True,
        )
    except Exception as exc:
        print(f"[WARN] Threshold update failed: {exc}", flush=True)


# ===========================================================================
# WebUI registration
# ===========================================================================

ui.on_message("set_tracking", set_tracking_callback)
ui.on_message("center_robot", center_robot_callback)
ui.on_message("emergency_stop", emergency_stop_callback)
ui.on_message("request_robot_status", request_status_callback)
ui.on_message("override_th", override_threshold)

ui.on_message("start_dictation", start_dictation)
ui.on_message("stop_dictation", stop_dictation)
ui.on_message("new_recording", new_recording)
ui.on_message("set_language", set_language)


# ===========================================================================
# Browser connection
# ===========================================================================

def webui_connected(sid):
    try:
        ui.send_message(
            "tracking_state",
            message={"enabled": tracking_requested},
        )
        send_voice_state()
    except Exception as exc:
        print(f"[WARN] WebUI sync failed: {exc}", flush=True)

    send_control("request_status")


ui.on_connect(webui_connected)


# ===========================================================================
# Detection processing
# ===========================================================================

def largest_detection(items):
    if not items:
        return None

    def bounding_box_area(item):
        box = item.get("bounding_box_xyxy")

        if not box or len(box) != 4:
            return 0

        x1, y1, x2, y2 = box
        width = max(0, x2 - x1)
        height = max(0, y2 - y1)
        return width * height

    return max(items, key=bounding_box_area)


def choose_tracking_target(detections: dict):
    for label in TRACK_LABELS:
        candidate = largest_detection(
            detections.get(label, [])
        )

        if candidate is not None:
            return label, candidate

    return None, None


def send_detections_to_ui_and_ros(detections: dict):
    global last_detection_send_time

    for label, values in detections.items():
        for value in values:
            try:
                ui.send_message(
                    "detection",
                    message={
                        "content": label,
                        "confidence": value.get("confidence"),
                        "timestamp": datetime.now(UTC).isoformat(),
                    },
                )
            except Exception:
                pass

    label, target = choose_tracking_target(detections)

    if target is None:
        try:
            ui.send_message(
                "target_state",
                message={"detected": False},
            )
        except Exception:
            pass
        return

    box = target.get("bounding_box_xyxy")

    if not box or len(box) != 4:
        return

    x1, y1, x2, y2 = [float(value) for value in box]
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    normalized_x = center_x / FRAME_WIDTH
    normalized_y = center_y / FRAME_HEIGHT
    confidence = float(target.get("confidence", 0.0))

    target_message = {
        "detected": True,
        "label": label,
        "confidence": confidence,
        "bbox": [x1, y1, x2, y2],
        "center_x": center_x,
        "center_y": center_y,
        "normalized_x": normalized_x,
        "normalized_y": normalized_y,
    }

    try:
        ui.send_message(
            "target_state",
            message=target_message,
        )
    except Exception:
        pass

    now = time.monotonic()

    if now - last_detection_send_time < DETECTION_SEND_INTERVAL:
        return

    last_detection_send_time = now

    detection_payload = {
        "message_type": "detection",
        **target_message,
        "frame_width": FRAME_WIDTH,
        "frame_height": FRAME_HEIGHT,
        "timestamp": time.time(),
    }

    if send_udp(detection_payload):
        print(
            f"[UDP] {label} "
            f"x={normalized_x:.3f} "
            f"y={normalized_y:.3f} "
            f"confidence={confidence:.3f}",
            flush=True,
        )


detection_stream.on_detect_all(send_detections_to_ui_and_ros)


# ===========================================================================
# Safe startup
# ===========================================================================

# Robot tracking starts OFF.
tracking_requested = False
send_control("set_tracking", enabled=False)

print(
    f"[INFO] ROS bridge destination: "
    f"{ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}",
    flush=True,
)

print("[INFO] Robot tracking starts DISABLED", flush=True)
print("[INFO] XFM microphone explicitly selected:", flush=True)
print(f"       {XFM_MIC_DEVICE}", flush=True)
print("[INFO] Persistent App Lab ASR configured", flush=True)
print("[INFO] Voice robot commands start ENABLED", flush=True)
print("[INFO] One persistent transcription session will be used", flush=True)
print("[INFO] Voice commands:", flush=True)
print("       Enable tracking", flush=True)
print("       Disable tracking", flush=True)
print("       Follow me", flush=True)
print("       Center robot", flush=True)
print("       Emergency stop", flush=True)


# ===========================================================================
# Delayed persistent ASR startup
# ===========================================================================

asr_boot_thread = threading.Thread(
    target=delayed_asr_startup,
    daemon=True,
    name="asr-delayed-start",
)
asr_boot_thread.start()


# ===========================================================================
# Start App Lab
# ===========================================================================

App.run()