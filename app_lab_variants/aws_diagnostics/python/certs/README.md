# Local AWS IoT certificates — not committed

This directory is intentionally empty in the public repository.

For a local deployment, copy the AWS IoT Core files here using these names:

- `device-certificate.pem.crt` — Thing/device certificate
- `private.pem.key` — **private key; secret; never commit**
- `AmazonRootCA1.pem` — Amazon public root CA

The repository-level `.gitignore` and this directory-level `.gitignore` are
intended to block these files from Git.

Before committing, always run:

```bash
git status --short
./scripts/check_no_secrets.sh
```

If a private key or device certificate has ever been committed to a public
repository, removing the file in a later commit is not sufficient. Revoke /
rotate the AWS IoT certificate and remove the secret from Git history.
