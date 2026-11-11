# AWS Diagnostics — public-repository-safe setup

The AWS Diagnostics variant adds:

- AWS IoT Core MQTT telemetry and cloud-to-device control
- Local LLM diagnostics (`llama.cpp` / Qwen model via the App Lab LLM brick)
- Diagnostics requests from both AWS IoT Core and the local WebUI
- Live system/robot counters used to ground diagnostics answers

## Security model

The public repository contains **code and configuration templates only**.
Never commit:

- AWS IoT private keys
- AWS IoT device certificates
- `.env` files containing real deployment values
- AWS access-key / secret-key pairs
- session tokens or passwords

The following local files are expected by default and are deliberately ignored:

```text
python/certs/device-certificate.pem.crt
python/certs/private.pem.key
python/certs/AmazonRootCA1.pem
```

`AWS_IOT_ENDPOINT` must be supplied at runtime; there is no real endpoint in
public source. `AWS_IOT_CLIENT_ID` is optional; if omitted, the bridge derives
a non-secret client ID from the board hostname.

## Local configuration

Use `.env.example` only as a reference. Keep the real `.env` local.

Required runtime value:

```text
AWS_IOT_ENDPOINT=YOUR_ENDPOINT-ats.iot.YOUR_REGION.amazonaws.com
```

Optional values:

```text
AWS_IOT_CLIENT_ID=ventunoq-demo
AWS_IOT_TOPIC_PREFIX=arduino/roboarm
AWS_IOT_CERT_PATH=python/certs/device-certificate.pem.crt
AWS_IOT_KEY_PATH=python/certs/private.pem.key
AWS_IOT_CA_PATH=python/certs/AmazonRootCA1.pem
```

## AWS IoT policy

Use least privilege. Limit the policy to the Thing/client and only the topics
required by this application. The application uses the `arduino/roboarm` topic
prefix by default, including state publication and `/set` subscriptions.

## Before every public push

```bash
git status --short
./scripts/check_no_secrets.sh
git diff --cached
```

If a private key was ever committed or pushed, assume it is compromised:
revoke/rotate the AWS IoT certificate and remove the material from Git history.
