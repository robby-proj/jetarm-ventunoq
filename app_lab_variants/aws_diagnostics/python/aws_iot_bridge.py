# SPDX-FileCopyrightText: Copyright (C) Arduino s.r.l.
# SPDX-License-Identifier: MPL-2.0

"""Minimal AWS IoT Core MQTT bridge for Arduino App Lab.

Public-repository safety:
- No AWS IoT endpoint, certificate, private key, access key, or account-specific
  identifier is embedded in this file.
- AWS_IOT_ENDPOINT must be supplied at runtime.
- Device certificate/private key files are local runtime files and must never be
  committed to Git.

The class mirrors the ergonomics of the Arduino Cloud brick: publish(),
publish_state(), and on_write() are intentionally small and reusable.
"""

import json
import os
import socket
from pathlib import Path

import paho.mqtt.client as mqtt


class AWSIoTBridge:
    def __init__(
        self,
        endpoint=None,
        client_id=None,
        cert_path=None,
        key_path=None,
        ca_path=None,
        topic_prefix=None,
        port=8883,
    ):
        # Endpoint is intentionally REQUIRED at runtime. Never hard-code an
        # account-specific endpoint in a public repository.
        self.endpoint = (endpoint or os.getenv("AWS_IOT_ENDPOINT", "")).strip()

        # MQTT client IDs are not credentials, but avoiding a hard-coded device
        # identity makes the public example reusable across boards/accounts.
        default_client_id = f"ventunoq-{socket.gethostname()}"
        self.client_id = (
            client_id
            or os.getenv("AWS_IOT_CLIENT_ID", "").strip()
            or default_client_id
        )

        self.cert_path = cert_path or os.getenv(
            "AWS_IOT_CERT_PATH", "python/certs/device-certificate.pem.crt"
        )
        self.key_path = key_path or os.getenv(
            "AWS_IOT_KEY_PATH", "python/certs/private.pem.key"
        )
        self.ca_path = ca_path or os.getenv(
            "AWS_IOT_CA_PATH", "python/certs/AmazonRootCA1.pem"
        )
        self.topic_prefix = (
            topic_prefix
            or os.getenv("AWS_IOT_TOPIC_PREFIX", "arduino/roboarm")
        ).strip("/")
        self.port = int(os.getenv("AWS_IOT_PORT", str(port)))

        self._connected = False
        self._write_callbacks = {}

        self._client = mqtt.Client(client_id=self.client_id)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_log = self._on_log
        self._client.on_message = self._on_message

    @property
    def connected(self) -> bool:
        return self._connected

    def _missing_tls_files(self):
        required = [self.ca_path, self.cert_path, self.key_path]
        return [str(path) for path in required if not Path(path).is_file()]

    def connect(self):
        if not self.endpoint:
            print(
                "[AWS IOT] AWS_IOT_ENDPOINT is not configured; "
                "AWS connection disabled",
                flush=True,
            )
            return

        missing = self._missing_tls_files()
        if missing:
            print(
                "[AWS IOT] TLS files are not installed locally; "
                f"AWS connection disabled. Missing: {', '.join(missing)}",
                flush=True,
            )
            return

        try:
            self._client.tls_set(
                ca_certs=self.ca_path,
                certfile=self.cert_path,
                keyfile=self.key_path,
            )
            self._client.connect(self.endpoint, self.port, keepalive=60)
            self._client.loop_start()
        except Exception as exc:
            print(f"[AWS IOT WARN] Connect failed: {exc}", flush=True)

    def _on_connect(self, client, userdata, flags, rc):
        self._connected = rc == 0
        print(f"[AWS IOT] Connected (rc={rc})", flush=True)

        for name in self._write_callbacks:
            topic = f"{self.topic_prefix}/{name}/set"
            client.subscribe(topic)
            print(f"[AWS IOT] Subscribed to {topic}", flush=True)

    def _on_disconnect(self, client, userdata, rc):
        self._connected = False
        print(f"[AWS IOT] Disconnected (rc={rc})", flush=True)

    def _on_log(self, client, userdata, level, buf):
        print(f"[AWS IOT LOG] {buf}", flush=True)

    def _on_message(self, client, userdata, msg):
        parts = msg.topic.split("/")
        name = parts[-2] if parts[-1] == "set" else parts[-1]

        try:
            payload = json.loads(msg.payload.decode())
            value = payload.get("value", payload)
        except Exception as exc:
            print(f"[AWS IOT WARN] Bad message on {msg.topic}: {exc}", flush=True)
            return

        callback = self._write_callbacks.get(name)
        if callback:
            callback(value)

    def on_write(self, name, callback):
        """Invoke callback(value) for `<topic_prefix>/<name>/set`."""
        self._write_callbacks[name] = callback
        if self._connected:
            self._client.subscribe(f"{self.topic_prefix}/{name}/set")

    def publish(self, name, value) -> bool:
        """Publish a device value to `<topic_prefix>/<name>`."""
        if not self._connected:
            return False
        topic = f"{self.topic_prefix}/{name}"
        result = self._client.publish(topic, json.dumps({"value": value}), qos=1)
        return result.rc == mqtt.MQTT_ERR_SUCCESS

    def publish_state(self, state: dict) -> bool:
        """Publish a state document to `<topic_prefix>/state`."""
        if not self._connected:
            return False
        topic = f"{self.topic_prefix}/state"
        result = self._client.publish(topic, json.dumps(state), qos=1)
        return result.rc == mqtt.MQTT_ERR_SUCCESS
