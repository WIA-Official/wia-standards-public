# 🪪 WIA Digital ID CLI

Decentralized identifier (DID), Verifiable Credentials, and revocation tooling
conforming to the WIA Digital ID standard.

## Installation

```bash
chmod +x digital-id.sh
sudo ln -s "$(pwd)/digital-id.sh" /usr/local/bin/digital-id
```

Requires `curl` and `jq` (https://jqlang.org).

## Authentication

```bash
export WIA_DIGITAL_ID_API_KEY="your-api-key"
export WIA_DIGITAL_ID_URL="https://api.wia.live/digital-id/v1"
```

## Resources

| Resource | Description |
|----------|-------------|
| `identities` | DIDs and identity profiles |
| `credentials` | Verifiable Credentials (W3C VC Data Model 2.0) |
| `revocations` | Revocation list entries |
| `resolvers` | DID method resolver registrations |

## Examples

### Verify a Verifiable Credential JWT

```bash
digital-id verify eyJhbGciOiJFZERTQSIsImtpZCI6...
```

### Revoke a credential

```bash
digital-id revoke vc-2026-04-25-001
```

### List credentials

```bash
digital-id list credentials --limit 100
```

## Reference

- W3C Decentralized Identifiers (DIDs) v1.0
- W3C Verifiable Credentials Data Model 2.0
- IETF JOSE (RFC 7515 / 7517 / 7518 / 7519)

License: MIT — 弘益人間 (홍익인간) Benefit All Humanity
