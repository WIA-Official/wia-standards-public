# WIA-IOT_SECURITY: Phase 3 - Protocol Specification
**Version:** 1.0
**Status:** Official
**Philosophy:** 弘益人間 (Benefit All Humanity)

## 1. Overview

This document specifies the security protocols, encryption standards, and communication protocols for IoT devices. All implementations MUST follow these specifications to ensure secure communication.

## 2. Transport Layer Security

### 2.1 TLS Requirements

**Minimum Requirements:**
- TLS version: 1.3 (REQUIRED), 1.2 (DEPRECATED)
- Certificate validation: REQUIRED
- Mutual TLS (mTLS): REQUIRED for device authentication
- Session resumption: OPTIONAL
- 0-RTT: DISABLED (security risk)

**Cipher Suites (TLS 1.3):**
```
Priority 1: TLS_AES_256_GCM_SHA384
Priority 2: TLS_CHACHA20_POLY1305_SHA256
Priority 3: TLS_AES_128_GCM_SHA256
```

### 2.2 Certificate Requirements

**Device Certificates:**
- Format: X.509 v3
- Key algorithm: ECDSA P-256 or RSA 2048+
- Signature algorithm: SHA-256 or better
- Validity period: Maximum 825 days
- Certificate chain: Complete chain to trusted root
- Extensions REQUIRED:
  - Subject Alternative Name (SAN)
  - Key Usage
  - Extended Key Usage
  - CRL Distribution Points or OCSP

**Example Certificate Structure:**
```
Certificate:
    Data:
        Version: 3 (0x2)
        Serial Number: 550e8400-e29b-41d4-a716-446655440000
        Signature Algorithm: ecdsa-with-SHA256
        Issuer: CN=IoT Device CA, O=SecureIoT Corp
        Validity
            Not Before: Jan 12 00:00:00 2025 GMT
            Not After : Jan 12 23:59:59 2026 GMT
        Subject: CN=SI-2000, SN=SI2000-2025-001234
        Subject Public Key Info:
            Public Key Algorithm: id-ecPublicKey
            Public-Key: (256 bit)
        X509v3 extensions:
            X509v3 Key Usage: critical
                Digital Signature, Key Agreement
            X509v3 Extended Key Usage:
                TLS Web Client Authentication
            X509v3 Subject Alternative Name:
                URI:urn:dev:mac:001A2B3C4D5E
```

### 2.3 Certificate Pinning

Devices SHOULD implement certificate pinning for critical connections:

```json
{
  "pinnedCertificates": [
    {
      "subject": "CN=IoT Gateway",
      "pinType": "SPKI",
      "hash": "sha256:ABC123...",
      "validFrom": "2025-01-01T00:00:00Z",
      "validUntil": "2026-01-01T00:00:00Z"
    }
  ],
  "pinningPolicy": "enforce",
  "backupPins": 2
}
```

## 3. Wireless Security Protocols

### 3.1 Wi-Fi Security

**Required Standards:**
- WPA3-Personal (SAE) - REQUIRED for new deployments
- WPA2-PSK (AES) - MINIMUM acceptable for legacy devices
- WEP - PROHIBITED
- Open networks - PROHIBITED for production

**Wi-Fi Configuration:**
```json
{
  "ssid": "IoT-Secure-Network",
  "security": "WPA3-SAE",
  "psk": "strong-passphrase-min-20-chars",
  "eap": {
    "method": "TLS",
    "identity": "device-550e8400",
    "certificate": "device-cert.pem",
    "privateKey": "device-key.pem"
  },
  "hiddenSSID": false,
  "macFiltering": true
}
```

### 3.2 Bluetooth LE Security

**Pairing Requirements:**
- Pairing method: Secure Connections Only (LE Secure Connections)
- Authentication: ECDH key exchange
- Encryption: AES-128-CCM
- Bonding: REQUIRED with secure storage
- Just Works pairing: PROHIBITED

**BLE Security Configuration:**
```json
{
  "securityLevel": "high",
  "pairingMethod": "numeric-comparison",
  "requireMITM": true,
  "requireBonding": true,
  "ioCapability": "display-yes-no",
  "maxBondedDevices": 10
}
```

### 3.3 LoRaWAN Security

**Network Security:**
- Activation: OTAA (Over-The-Air Activation) REQUIRED
- ABP (Activation By Personalization) - DEPRECATED
- LoRaWAN version: 1.1 or higher
- Encryption: AES-128

**LoRaWAN Keys:**
```json
{
  "appEUI": "0011223344556677",
  "devEUI": "0011223344556677",
  "appKey": "00112233445566778899AABBCCDDEEFF",
  "nwkKey": "FFEEDDCCBBAA99887766554433221100",
  "joinServer": "https://join.example.com"
}
```

## 4. Application Layer Protocols

### 4.1 MQTT Security

**MQTT over TLS (MQTTS):**
- Port: 8883 (REQUIRED)
- Port 1883 (unencrypted): PROHIBITED
- MQTT version: 5.0 or 3.1.1
- Authentication: Username/password + TLS client certificate
- Authorization: Topic-based ACLs

**MQTT Security Configuration:**
```json
{
  "broker": "mqtts://mqtt.example.com:8883",
  "clientId": "device-550e8400",
  "username": "device-550e8400",
  "password": "secure-token",
  "tls": {
    "caCert": "ca-cert.pem",
    "clientCert": "device-cert.pem",
    "clientKey": "device-key.pem",
    "verifyServerCert": true
  },
  "keepAlive": 60,
  "cleanSession": false,
  "qos": 1,
  "retained": false
}
```

**Topic Authorization:**
```json
{
  "deviceId": "550e8400-e29b-41d4-a716-446655440000",
  "allowedTopics": {
    "publish": [
      "devices/550e8400/telemetry",
      "devices/550e8400/events",
      "devices/550e8400/status"
    ],
    "subscribe": [
      "devices/550e8400/commands",
      "devices/550e8400/config",
      "broadcast/firmware-updates"
    ]
  }
}
```

### 4.2 CoAP Security (CoAPS)

**DTLS Requirements:**
- DTLS version: 1.3 (REQUIRED), 1.2 (ACCEPTABLE)
- Cipher suites: TLS_ECDHE_ECDSA_WITH_AES_128_CCM_8
- Pre-shared key (PSK) mode: SUPPORTED
- Raw public key mode: SUPPORTED
- Certificate mode: REQUIRED for production

**CoAP Security Configuration:**
```json
{
  "endpoint": "coaps://coap.example.com:5684",
  "dtls": {
    "version": "1.3",
    "mode": "certificate",
    "certificate": "device-cert.der",
    "privateKey": "device-key.der",
    "peerVerification": true
  },
  "confirmable": true,
  "blockSize": 1024
}
```

### 4.3 HTTPS REST API

**Security Requirements:**
- HTTPS only (HTTP PROHIBITED)
- TLS 1.3 REQUIRED
- HSTS header REQUIRED
- API authentication: OAuth 2.0 or JWT
- Request signing: HMAC-SHA256 or Ed25519

**API Request Signing:**
```
Authorization: WIA-HMAC-SHA256 Credential=deviceId/date/region/service,
  SignedHeaders=host;x-wia-date;content-type,
  Signature=hexencode(hmac-sha256(signingKey, stringToSign))
```

## 5. Encryption Standards

### 5.1 Symmetric Encryption

**Approved Algorithms:**
- AES-256-GCM (REQUIRED)
- ChaCha20-Poly1305 (OPTIONAL)
- AES-128-GCM (ACCEPTABLE for constrained devices)

**Prohibited Algorithms:**
- AES-CBC without authentication
- AES-ECB
- DES, 3DES
- RC4

**Encryption Configuration:**
```json
{
  "algorithm": "AES-256-GCM",
  "keyLength": 256,
  "ivLength": 96,
  "tagLength": 128,
  "keyDerivation": "HKDF-SHA256",
  "saltLength": 128
}
```

### 5.2 Asymmetric Encryption

**Approved Algorithms:**
- RSA-OAEP-SHA256 with 2048+ bit keys
- ECDH with P-256, P-384, or Curve25519
- Ed25519 for signatures

**Key Generation:**
```json
{
  "algorithm": "ECDSA",
  "curve": "P-256",
  "encoding": "PEM",
  "storageType": "TPM",
  "backupEnabled": false
}
```

### 5.3 Hashing Algorithms

**Approved Hash Functions:**
- SHA-256 (REQUIRED)
- SHA-384, SHA-512 (OPTIONAL)
- BLAKE2b (OPTIONAL)

**Prohibited Hash Functions:**
- MD5
- SHA-1

## 6. Key Management Protocol

### 6.1 Key Generation

**Requirements:**
- Entropy source: Hardware RNG or TRNG
- Minimum entropy: 256 bits
- Key generation: On-device only
- Key export: PROHIBITED (except for backup with encryption)

### 6.2 Key Storage

**Storage Requirements:**
- Hardware-backed: TPM 2.0, Secure Element, or TEE
- Software fallback: Encrypted keystore with device-bound key
- Permission model: Key access restricted to authorized apps only

**Key Storage Hierarchy:**
```
Root Key (Hardware-backed, non-exportable)
  └─ Device Master Key (Derived)
      ├─ Encryption Keys (Per-purpose)
      ├─ Signing Keys (Per-purpose)
      └─ Authentication Keys (Per-protocol)
```

### 6.3 Key Rotation

**Rotation Policy:**
- Certificate keys: Every 365 days
- Session keys: Every session
- Pre-shared keys: Every 90 days
- Encryption keys: Every 180 days or after compromise

**Rotation Process:**
```json
{
  "rotationId": "rot-123",
  "keyType": "device-certificate",
  "oldKeyId": "key-old-abc",
  "newKeyId": "key-new-xyz",
  "rotationDate": "2025-01-12T00:00:00Z",
  "overlapPeriod": 86400,
  "autoRotate": true,
  "notifyBeforeDays": 30
}
```

## 7. VPN and Tunneling

### 7.1 VPN Support

**Supported VPN Protocols:**
- WireGuard (RECOMMENDED)
- IPsec/IKEv2 (ACCEPTABLE)
- OpenVPN (ACCEPTABLE)
- PPTP (PROHIBITED)
- L2TP without IPsec (PROHIBITED)

**WireGuard Configuration:**
```ini
[Interface]
PrivateKey = <device-private-key>
Address = 10.0.0.2/24
DNS = 10.0.0.1

[Peer]
PublicKey = <server-public-key>
Endpoint = vpn.example.com:51820
AllowedIPs = 0.0.0.0/0
PersistentKeepalive = 25
```

### 7.2 Secure Tunneling

**SSH Tunneling for Management:**
```json
{
  "tunnelType": "SSH",
  "remoteHost": "manage.example.com",
  "remotePort": 22,
  "localPort": 8080,
  "username": "device-550e8400",
  "authentication": "publickey",
  "privateKey": "device-ssh-key.pem",
  "knownHosts": "known_hosts",
  "compression": true,
  "keepAlive": 60
}
```

## 8. Security Protocol Compliance

### 8.1 OWASP IoT Top 10 Compliance

| Item | Requirement | Implementation |
|------|-------------|----------------|
| I1 | Weak passwords | Certificate-based auth |
| I2 | Insecure network | TLS 1.3, mTLS required |
| I3 | Insecure ecosystem | API security, rate limiting |
| I4 | Lack of secure update | Signed firmware, rollback |
| I5 | Insecure data storage | Encrypted storage, TPM |
| I6 | Data protection | Encryption in transit/rest |
| I7 | Insecure data transfer | TLS/DTLS required |
| I8 | Lack of device management | Remote management API |
| I9 | Insecure default settings | Secure by default config |
| I10 | Lack of physical hardening | Tamper detection support |

### 8.2 Industry Standards

- NIST Cybersecurity Framework
- ETSI EN 303 645 (IoT Security Standard)
- IEC 62443 (Industrial Security)
- ISO/IEC 27001 (Information Security)
- CSA IoT Security Controls Framework

---

**弘익인간 (Benefit All Humanity)**

*© 2025 WIA - World Certification Industry Association*
*MIT License*
