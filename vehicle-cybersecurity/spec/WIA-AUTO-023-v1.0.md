# WIA-AUTO-023: Vehicle Cybersecurity Specification v1.0

> **Standard ID:** WIA-AUTO-023
> **Version:** 1.0.0
> **Published:** 2025-12-26
> **Status:** Active
> **Authors:** WIA Automotive Security Group

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Threat Landscape](#2-threat-landscape)
3. [ECU Security](#3-ecu-security)
4. [In-Vehicle Network Security](#4-in-vehicle-network-security)
5. [External Interface Security](#5-external-interface-security)
6. [OTA Update Security](#6-ota-update-security)
7. [Intrusion Detection and Response](#7-intrusion-detection-and-response)
8. [ISO/SAE 21434 Compliance](#8-iso-sae-21434-compliance)
9. [Data Formats](#9-data-formats)
10. [API Interface](#10-api-interface)
11. [Security Protocols](#11-security-protocols)
12. [References](#12-references)

---

## 1. Introduction

### 1.1 Purpose

This specification defines comprehensive cybersecurity requirements for modern connected and autonomous vehicles, providing a framework to protect against cyber threats while enabling innovation in automotive technology.

### 1.2 Scope

The standard covers:
- Electronic Control Unit (ECU) security
- In-vehicle network protection (CAN, LIN, FlexRay, Ethernet)
- External interface security (V2X, telematics, OBD-II)
- Over-the-air (OTA) update security
- Intrusion detection and incident response
- Cryptographic key management
- ISO/SAE 21434 compliance requirements

### 1.3 Philosophy

**弘익人間 (Benefit All Humanity)** - This standard aims to protect vehicle occupants, pedestrians, and the broader transportation ecosystem by establishing robust cybersecurity frameworks that prevent malicious attacks while preserving privacy and enabling beneficial connected vehicle services.

### 1.4 Terminology

- **ECU (Electronic Control Unit)**: Embedded computer that controls vehicle subsystems
- **CAN (Controller Area Network)**: Broadcast-based in-vehicle network protocol
- **V2X (Vehicle-to-Everything)**: Communication between vehicle and external entities
- **OTA (Over-the-Air)**: Wireless software/firmware updates
- **HSM (Hardware Security Module)**: Dedicated cryptographic processor
- **IDS (Intrusion Detection System)**: Security monitoring and threat detection
- **TARA (Threat Analysis and Risk Assessment)**: ISO 21434 risk assessment method
- **CAL (Cybersecurity Assurance Level)**: Security rigor classification (1-4)

---

## 2. Threat Landscape

### 2.1 Attack Surface

Modern vehicles have expanded attack surfaces:

```
External Attack Vectors:
├── Remote Attacks
│   ├── Cellular/5G connectivity
│   ├── Wi-Fi networks
│   ├── V2X communication
│   └── Cloud APIs
├── Proximity Attacks
│   ├── Bluetooth
│   ├── NFC/RFID
│   └── Tire pressure sensors
└── Physical Attacks
    ├── OBD-II port
    ├── USB/SD interfaces
    ├── Infotainment system
    └── Diagnostic tools
```

### 2.2 Attack Vectors

#### 2.2.1 Remote Exploitation
- **Telematics Backend**: Cloud service vulnerabilities
- **Mobile Apps**: Companion app security flaws
- **V2X Messages**: Malicious message injection
- **Wi-Fi/Cellular**: Network-based attacks

#### 2.2.2 Physical Access
- **OBD-II Port**: Direct CAN bus access
- **USB/SD Card**: Malware introduction
- **ECU Replacement**: Hardware tampering
- **Debug Interfaces**: JTAG/SWD exploitation

#### 2.2.3 Supply Chain
- **Compromised Components**: Malicious hardware
- **Third-Party Software**: Vulnerable libraries
- **Service Centers**: Malicious updates
- **Aftermarket Devices**: Untrusted accessories

### 2.3 Threat Categories (STRIDE)

| Category | Vehicle Examples | Impact |
|----------|-----------------|--------|
| **S**poofing | Fake V2X messages, GPS spoofing | Misdirection, confusion |
| **T**ampering | CAN message injection, ECU flashing | Vehicle control, safety |
| **R**epudiation | False diagnostics, altered logs | Legal, warranty issues |
| **I**nformation Disclosure | Location tracking, privacy breach | Privacy violation |
| **D**enial of Service | CAN bus flooding, jamming | Loss of functionality |
| **E**levation of Privilege | Debug mode access, root exploit | Full vehicle control |

### 2.4 Impact Assessment

```
Impact Scale:
Critical (4): Life-threatening safety impact
High (3):    Safety-related functionality affected
Medium (2):  Comfort/convenience features compromised
Low (1):     Minor functionality affected

Examples:
- Steering/Braking compromise: Critical
- ADAS disablement: High
- Infotainment malfunction: Medium
- Climate control glitch: Low
```

### 2.5 Real-World Attack Examples

1. **Jeep Cherokee (2015)**: Remote exploitation via telematics
2. **Tesla Model S**: Key fob relay attacks
3. **CAN Injection**: Speed/RPM gauge manipulation
4. **GPS Spoofing**: Location falsification
5. **Charging Station**: Man-in-the-middle attacks

---

## 3. ECU Security

### 3.1 Secure Boot

#### 3.1.1 Boot Chain of Trust

```
Boot Sequence:
1. Hardware Root of Trust (Immutable ROM)
   ├── Verify Bootloader signature
   ├── Load Bootloader
   └── Transfer control

2. Bootloader (Signed by OEM)
   ├── Verify OS/Firmware signature
   ├── Load OS/Firmware
   └── Transfer control

3. Operating System (Signed)
   ├── Verify Application signatures
   ├── Load Applications
   └── Run vehicle functions

Each stage verifies next stage before execution
```

#### 3.1.2 Implementation Requirements

```c
// Secure Boot Verification
typedef struct {
    uint8_t signature[256];      // RSA-2048 or ECDSA-256
    uint8_t public_key[256];     // OEM public key
    uint32_t image_size;
    uint32_t image_crc;
    uint8_t version[16];
    uint8_t timestamp[8];
} SecureBootHeader;

bool verify_secure_boot(const void* image, size_t size) {
    SecureBootHeader* header = (SecureBootHeader*)image;

    // 1. Verify signature using HSM
    if (!hsm_verify_signature(header->signature,
                              image + sizeof(SecureBootHeader),
                              header->image_size,
                              header->public_key)) {
        return false;
    }

    // 2. Verify CRC
    uint32_t calculated_crc = crc32(image + sizeof(SecureBootHeader),
                                    header->image_size);
    if (calculated_crc != header->image_crc) {
        return false;
    }

    // 3. Check version rollback protection
    if (!check_version_valid(header->version)) {
        return false;
    }

    return true;
}
```

### 3.2 Hardware Security Module (HSM)

#### 3.2.1 HSM Functions

- **Key Storage**: Secure storage of cryptographic keys
- **Encryption/Decryption**: AES-128/256, RSA-2048/4096
- **Signing/Verification**: ECDSA, RSA signatures
- **Random Number Generation**: True RNG for cryptography
- **Secure Key Derivation**: Key generation and derivation

#### 3.2.2 HSM Architecture

```
┌─────────────────────────────────────┐
│         ECU Application             │
├─────────────────────────────────────┤
│      Crypto Service Layer (CSL)     │
├─────────────────────────────────────┤
│   Hardware Security Module (HSM)    │
│  ┌──────────┬──────────┬──────────┐ │
│  │ Key Store│Crypto Acc│  True RNG│ │
│  └──────────┴──────────┴──────────┘ │
│  ┌──────────────────────────────┐   │
│  │  Tamper Detection & Response │   │
│  └──────────────────────────────┘   │
└─────────────────────────────────────┘
```

#### 3.2.3 Key Lifecycle Management

```
Key Lifecycle Stages:
1. Generation: Created in HSM
2. Installation: Provisioned during manufacturing
3. Activation: Enabled for use
4. Usage: Normal cryptographic operations
5. Deactivation: Temporarily disabled
6. Destruction: Securely erased
7. Archive: Long-term secure storage (optional)

Key Types:
- Master Keys: Root cryptographic keys
- Device Keys: Unique per-ECU keys
- Session Keys: Temporary keys for communication
- Update Keys: OTA update verification keys
```

### 3.3 Code Signing

#### 3.3.1 Software Signing Requirements

All executable code must be digitally signed:

```
Signing Process:
1. Developer writes code
2. Code review and testing
3. Build process creates binary
4. OEM signs binary with private key
5. Binary + signature distributed
6. ECU verifies signature with public key
7. Only execute if signature valid
```

#### 3.3.2 Signature Format

```json
{
  "software": {
    "name": "Engine_Control_Module",
    "version": "2.4.1",
    "build": "20250115-1234",
    "hash": {
      "algorithm": "SHA-256",
      "value": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
    }
  },
  "signature": {
    "algorithm": "ECDSA-P256",
    "value": "304502210094...",
    "certificate": "MIIDXTCCAkWgAwIBAgIJ..."
  },
  "signer": {
    "organization": "OEM_Name",
    "certificate_chain": ["cert1", "cert2", "root"]
  },
  "timestamp": "2025-01-15T12:34:56Z",
  "metadata": {
    "target_ecus": ["ECM", "TCM"],
    "min_hw_version": "1.0",
    "dependencies": []
  }
}
```

### 3.4 Secure Storage

#### 3.4.1 Protected Data Types

- **Cryptographic Keys**: Never exposed in plaintext
- **Calibration Data**: Protected from tampering
- **VIN & Serial Numbers**: Immutable identifiers
- **Security Logs**: Tamper-evident audit trails
- **User Credentials**: Encrypted passwords/tokens

#### 3.4.2 Storage Security Mechanisms

```
Security Layer Stack:
┌────────────────────────────────┐
│   Application Data             │
├────────────────────────────────┤
│   Encryption (AES-256-GCM)     │
├────────────────────────────────┤
│   Authentication (HMAC-SHA256) │
├────────────────────────────────┤
│   Flash Memory (Protected)     │
└────────────────────────────────┘

Write Protection:
- Read-only regions for bootloader
- Write protection for critical data
- CRC/ECC for integrity verification
```

---

## 4. In-Vehicle Network Security

### 4.1 CAN Bus Security

#### 4.1.1 CAN Bus Vulnerabilities

Traditional CAN has no built-in security:
- **No Authentication**: Any ECU can send any message
- **No Encryption**: All messages in plaintext
- **Broadcast**: All nodes see all messages
- **No Access Control**: No permission management

#### 4.1.2 CAN Security Mechanisms

**Message Authentication Codes (MAC)**

```c
// CAN Message with MAC
typedef struct {
    uint32_t can_id;           // Standard CAN identifier
    uint8_t data[8];           // Payload data
    uint8_t mac[4];            // Truncated HMAC (CANauth)
    uint32_t counter;          // Freshness counter
} SecureCANMessage;

// Generate MAC
void generate_can_mac(SecureCANMessage* msg, const uint8_t* key) {
    uint8_t mac_input[16];

    memcpy(mac_input, &msg->can_id, 4);
    memcpy(mac_input + 4, msg->data, 8);
    memcpy(mac_input + 12, &msg->counter, 4);

    uint8_t full_mac[32];
    hmac_sha256(mac_input, 16, key, 32, full_mac);

    memcpy(msg->mac, full_mac, 4);  // Truncate to 32 bits
}

// Verify MAC
bool verify_can_mac(const SecureCANMessage* msg, const uint8_t* key) {
    SecureCANMessage verify_msg = *msg;
    generate_can_mac(&verify_msg, key);

    return memcmp(msg->mac, verify_msg.mac, 4) == 0;
}
```

**CAN Firewall Rules**

```yaml
can_firewall_rules:
  # Allow specific CAN IDs
  whitelist:
    - id: 0x100
      source: ECM
      rate_limit: 100  # messages per second

    - id: 0x200
      source: ABS
      rate_limit: 50

  # Block ranges
  blacklist:
    - id_range: [0x700, 0x7FF]
      reason: "Diagnostic range - restricted"

  # Anomaly detection
  anomaly_rules:
    - detect: sudden_rate_change
      threshold: 200%
      action: alert

    - detect: new_can_id
      action: block

    - detect: malformed_message
      action: drop
```

#### 4.1.3 CAN-FD Security

CAN-FD allows longer messages for authentication:

```
CAN Classic: 8 bytes payload
CAN-FD:      64 bytes payload

Additional space for:
- Full MAC (16-32 bytes)
- Counter/Timestamp (4-8 bytes)
- Additional security metadata
```

### 4.2 Automotive Ethernet Security

#### 4.2.1 IEEE 802.1AE MACsec

```
MACsec Frame Format:
┌──────────────────────────────────┐
│  Ethernet Header                 │
├──────────────────────────────────┤
│  MACsec Security Tag (8 bytes)   │
│  - Packet Number (PN)            │
│  - Security Channel Identifier   │
├──────────────────────────────────┤
│  Encrypted Payload               │
│  (Original Ethernet Payload)     │
├──────────────────────────────────┤
│  Integrity Check Value (ICV)     │
│  (16 bytes)                      │
└──────────────────────────────────┘

Encryption: AES-128-GCM or AES-256-GCM
```

#### 4.2.2 IPsec for Automotive Ethernet

```yaml
ipsec_configuration:
  mode: transport  # or tunnel
  protocol: ESP    # Encapsulating Security Payload

  encryption:
    algorithm: AES-256-CBC
    key_exchange: IKEv2

  authentication:
    algorithm: HMAC-SHA-256

  policies:
    - src: camera_ecu
      dst: adas_ecu
      action: encrypt
      spi: 0x12345678

    - src: lidar_ecu
      dst: adas_ecu
      action: encrypt
      spi: 0x23456789
```

### 4.3 FlexRay Security

FlexRay has built-in determinism but limited security:

```yaml
flexray_security:
  # Secure scheduling
  static_segment:
    - slot: 1
      ecu: safety_controller
      allowed_ids: [0x01, 0x02]

  # Message authentication
  authentication:
    method: end-to-end
    mac_length: 32  # bits
    freshness: counter

  # Monitoring
  bus_guardian:
    enabled: true
    detect_babbling_idiot: true
    action: isolate_node
```

### 4.4 Gateway Security

The central gateway is critical:

```
┌────────────────────────────────────────┐
│        Central Gateway ECU             │
│  ┌──────────────────────────────────┐  │
│  │     Firewall & Packet Filter     │  │
│  ├──────────────────────────────────┤  │
│  │  Network Segmentation (VLANs)    │  │
│  ├──────────────────────────────────┤  │
│  │  Protocol Translation            │  │
│  ├──────────────────────────────────┤  │
│  │  Intrusion Detection             │  │
│  └──────────────────────────────────┘  │
└────────────────────────────────────────┘
       │        │        │        │
    CAN-PT   CAN-CH  FlexRay  Ethernet
    (Infot)  (Chassis) (ADAS) (Cameras)
```

**Gateway Firewall Rules**

```c
typedef struct {
    network_id_t source_network;
    network_id_t dest_network;
    message_id_t message_id;
    action_t action;           // ALLOW, BLOCK, INSPECT
    uint32_t rate_limit;       // messages/second
    bool require_auth;
} GatewayRule;

// Example rules
GatewayRule rules[] = {
    // Block infotainment from chassis network
    { .source_network = NET_INFOTAINMENT,
      .dest_network = NET_CHASSIS,
      .message_id = ANY,
      .action = BLOCK },

    // Allow diagnostics only at low rate
    { .source_network = NET_OBD,
      .dest_network = NET_CHASSIS,
      .message_id = ANY,
      .action = ALLOW,
      .rate_limit = 10,
      .require_auth = true },
};
```

---

## 5. External Interface Security

### 5.1 V2X Communication Security

#### 5.1.1 PKI Infrastructure

```
V2X Security Certificate Hierarchy:
┌─────────────────────────────────┐
│   Root CA (Certificate Authority)│
└──────────────┬──────────────────┘
               │
    ┌──────────┴──────────┐
    │                     │
┌───▼─────┐       ┌──────▼────┐
│ Inter CA│       │  Inter CA │
│ (Region)│       │ (Region)  │
└───┬─────┘       └──────┬────┘
    │                    │
┌───▼──────────┐   ┌────▼──────────┐
│Enrollment CA │   │  Authorization │
│  (Long-term) │   │  CA (Short)    │
└───┬──────────┘   └────┬───────────┘
    │                   │
┌───▼──────┐      ┌─────▼──────┐
│ Vehicle  │      │Pseudonym   │
│   Cert   │      │Certificates│
│(Identity)│      │ (Privacy)  │
└──────────┘      └────────────┘
```

#### 5.1.2 V2X Message Security

```c
// IEEE 1609.2 Secured Message
typedef struct {
    uint8_t protocol_version;
    security_type_t type;  // SIGNED, ENCRYPTED, SIGNED_ENCRYPTED

    // Signer info
    signer_info_t signer;
    uint8_t certificate[500];  // Pseudonym certificate

    // Message content
    uint8_t* payload;
    size_t payload_len;

    // Security trailer
    signature_t signature;     // ECDSA-256
    uint32_t generation_time;
    location_t generation_location;
} V2XSecuredMessage;

// Sign V2X message
void sign_v2x_message(V2XSecuredMessage* msg,
                      const uint8_t* private_key,
                      const uint8_t* certificate) {
    // Include generation time and location
    msg->generation_time = get_gps_time();
    msg->generation_location = get_gps_location();

    // Create signature over payload + metadata
    uint8_t hash[32];
    sha256_hash(msg->payload, msg->payload_len, hash);

    ecdsa_sign_p256(hash, 32, private_key, &msg->signature);

    // Attach pseudonym certificate
    memcpy(msg->certificate, certificate, 500);
}
```

#### 5.1.3 Privacy Protection

```
Pseudonym Certificate Strategy:
- Vehicle receives pool of pseudonym certificates
- Each certificate used for limited time (5-10 minutes)
- Certificates rotated to prevent tracking
- No link between pseudonyms and vehicle identity

Pseudonym Pool Management:
┌────────────────────────────────┐
│  Cert Pool (20 certificates)   │
│  ┌──┬──┬──┬──┬──┬──┬──┬──┐    │
│  │ 1│ 2│ 3│ 4│ 5│ 6│ 7│ 8│... │
│  └──┴──┴──┴──┴──┴──┴──┴──┘    │
│  Active: Cert #3               │
│  Rotation: Every 5 minutes     │
│  Refill: When < 5 remaining    │
└────────────────────────────────┘
```

### 5.2 Telematics Security

#### 5.2.1 TLS Configuration

```yaml
telematics_tls:
  version: TLS_1.3
  cipher_suites:
    - TLS_AES_256_GCM_SHA384
    - TLS_CHACHA20_POLY1305_SHA256

  mutual_authentication: true

  client_certificate:
    type: X.509
    key_algorithm: ECDSA-P256
    storage: HSM

  server_certificate:
    validation: strict
    pinning: enabled
    pins:
      - "sha256/AAAAAAA..."
      - "sha256/BBBBBBB..."

  session:
    resumption: true
    timeout: 3600  # seconds
```

#### 5.2.2 Backend API Security

```typescript
// Authenticated API Request
interface TelematicsRequest {
  vin: string;
  timestamp: number;
  nonce: string;

  // Request data
  command: string;
  parameters: Record<string, any>;

  // Authentication
  signature: string;  // HMAC-SHA256
  device_token: string;
}

// Server-side verification
function verifyTelematicsRequest(req: TelematicsRequest): boolean {
  // 1. Check timestamp freshness (prevent replay)
  const now = Date.now() / 1000;
  if (Math.abs(now - req.timestamp) > 300) {  // 5 minutes
    return false;
  }

  // 2. Check nonce uniqueness (prevent replay)
  if (isNonceUsed(req.nonce)) {
    return false;
  }

  // 3. Verify signature
  const deviceKey = getDeviceKey(req.vin, req.device_token);
  const message = `${req.vin}${req.timestamp}${req.nonce}${req.command}`;
  const expectedSig = hmacSha256(message, deviceKey);

  if (req.signature !== expectedSig) {
    return false;
  }

  // 4. Check authorization
  if (!isAuthorized(req.vin, req.command)) {
    return false;
  }

  return true;
}
```

### 5.3 OBD-II Security

#### 5.3.1 Access Control

```c
// OBD-II Security Levels
typedef enum {
    OBD_LEVEL_PUBLIC = 0,      // Standard diagnostics
    OBD_LEVEL_ENHANCED = 1,    // Extended data
    OBD_LEVEL_PROTECTED = 2,   // Manufacturer specific
    OBD_LEVEL_SECURE = 3       // ECU programming
} OBDSecurityLevel;

// Security access challenge-response
typedef struct {
    uint32_t seed;             // Random challenge
    uint32_t key;              // Computed response
    uint8_t level;             // Security level
    uint32_t timestamp;        // Valid time
} OBDSecuritySession;

// Generate challenge
uint32_t obd_generate_seed(void) {
    return true_random_u32();
}

// Compute response (simplified)
uint32_t obd_compute_key(uint32_t seed, OBDSecurityLevel level) {
    // Manufacturer-specific algorithm
    uint8_t secret[16];
    get_secret_for_level(level, secret);

    uint8_t input[20];
    memcpy(input, &seed, 4);
    memcpy(input + 4, secret, 16);

    uint32_t key = crc32(input, 20);
    return key ^ 0x12345678;  // Obfuscation
}
```

#### 5.3.2 Rate Limiting

```c
// OBD-II rate limiting
#define MAX_OBD_REQUESTS_PER_SECOND 10
#define MAX_SECURITY_ATTEMPTS 3

typedef struct {
    uint32_t request_count;
    uint32_t last_reset_time;
    uint32_t failed_auth_count;
    bool locked;
} OBDRateLimiter;

bool obd_check_rate_limit(OBDRateLimiter* limiter) {
    uint32_t now = get_milliseconds();

    // Reset counter every second
    if (now - limiter->last_reset_time > 1000) {
        limiter->request_count = 0;
        limiter->last_reset_time = now;
    }

    // Check if locked due to failed attempts
    if (limiter->locked) {
        return false;
    }

    // Check rate limit
    if (limiter->request_count >= MAX_OBD_REQUESTS_PER_SECOND) {
        return false;
    }

    limiter->request_count++;
    return true;
}
```

---

## 6. OTA Update Security

### 6.1 Update Package Format

```json
{
  "package": {
    "id": "OTA-2025-001-ECM",
    "version": "3.2.1",
    "previous_version": "3.2.0",
    "release_date": "2025-01-15T10:00:00Z",
    "priority": "security-critical"
  },

  "targets": [
    {
      "ecu": "ECM",
      "hardware_version": "2.0",
      "software_version": "3.2.0",
      "vin_range": "WBA*"
    }
  ],

  "payload": {
    "type": "differential",
    "size_bytes": 2097152,
    "compressed": true,
    "compression": "lzma",
    "encryption": {
      "algorithm": "AES-256-GCM",
      "key_id": "update-key-2025-01"
    }
  },

  "integrity": {
    "algorithm": "SHA-256",
    "hash": "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
  },

  "signature": {
    "algorithm": "RSA-4096",
    "value": "MIIBIjANBgkqhkiG9w0BAQEF...",
    "certificate_chain": [
      "-----BEGIN CERTIFICATE-----\n...",
      "-----BEGIN CERTIFICATE-----\n..."
    ]
  },

  "metadata": {
    "description": "Security patch for CVE-2025-001",
    "rollback_allowed": false,
    "installation": {
      "requires_ignition_off": true,
      "requires_full_charge": true,
      "estimated_duration_seconds": 600,
      "requires_user_confirmation": true
    }
  }
}
```

### 6.2 Update Verification Process

```c
typedef enum {
    UPDATE_VERIFY_OK = 0,
    UPDATE_VERIFY_INVALID_SIGNATURE,
    UPDATE_VERIFY_HASH_MISMATCH,
    UPDATE_VERIFY_INCOMPATIBLE,
    UPDATE_VERIFY_ROLLBACK_DETECTED
} UpdateVerifyResult;

UpdateVerifyResult verify_ota_update(const OTAPackage* pkg) {
    // 1. Verify certificate chain
    if (!verify_certificate_chain(pkg->cert_chain,
                                   pkg->cert_chain_len)) {
        return UPDATE_VERIFY_INVALID_SIGNATURE;
    }

    // 2. Verify digital signature
    uint8_t package_hash[32];
    sha256_hash(pkg->payload, pkg->payload_len, package_hash);

    if (!rsa_verify(package_hash, 32,
                    pkg->signature, pkg->signature_len,
                    pkg->public_key)) {
        return UPDATE_VERIFY_INVALID_SIGNATURE;
    }

    // 3. Verify payload hash
    uint8_t calculated_hash[32];
    sha256_hash(pkg->payload, pkg->payload_len, calculated_hash);

    if (memcmp(calculated_hash, pkg->expected_hash, 32) != 0) {
        return UPDATE_VERIFY_HASH_MISMATCH;
    }

    // 4. Check version compatibility
    if (!is_version_compatible(pkg->target_version,
                               get_current_version())) {
        return UPDATE_VERIFY_INCOMPATIBLE;
    }

    // 5. Check rollback protection
    if (pkg->target_version < get_current_version() &&
        !pkg->rollback_allowed) {
        return UPDATE_VERIFY_ROLLBACK_DETECTED;
    }

    return UPDATE_VERIFY_OK;
}
```

### 6.3 Secure Update Installation

```
Update Installation Flow:
┌─────────────────────────────────┐
│ 1. Download package             │
│    - TLS encrypted connection   │
│    - Resume capability          │
└─────────────┬───────────────────┘
              │
┌─────────────▼───────────────────┐
│ 2. Verify package               │
│    - Signature check            │
│    - Hash verification          │
│    - Compatibility check        │
└─────────────┬───────────────────┘
              │
┌─────────────▼───────────────────┐
│ 3. Pre-installation checks      │
│    - Battery level > 50%        │
│    - Vehicle stationary         │
│    - Ignition off (if required) │
└─────────────┬───────────────────┘
              │
┌─────────────▼───────────────────┐
│ 4. Backup current version       │
│    - Create recovery image      │
│    - Store in separate partition│
└─────────────┬───────────────────┘
              │
┌─────────────▼───────────────────┐
│ 5. Install update               │
│    - Decrypt payload            │
│    - Write to flash             │
│    - Verify written data        │
└─────────────┬───────────────────┘
              │
┌─────────────▼───────────────────┐
│ 6. Post-installation validation │
│    - Verify integrity           │
│    - Run self-tests             │
│    - Check functionality        │
└─────────────┬───────────────────┘
              │
      ┌───────┴───────┐
      │               │
┌─────▼─────┐   ┌────▼─────┐
│  Success   │   │  Failure │
│  Commit    │   │  Rollback│
│  Update    │   │  to Backup│
└────────────┘   └──────────┘
```

### 6.4 Rollback Protection

```c
// Version management with rollback protection
typedef struct {
    uint32_t major;
    uint32_t minor;
    uint32_t patch;
    uint32_t security_version;  // Monotonic counter
} FirmwareVersion;

// Security version stored in OTP (One-Time Programmable) memory
bool check_rollback_protection(const FirmwareVersion* new_ver) {
    FirmwareVersion current;
    get_current_version(&current);

    // Security version must never decrease
    if (new_ver->security_version < current.security_version) {
        log_security_event(EVENT_ROLLBACK_ATTEMPT);
        return false;
    }

    return true;
}

// After successful update, increment security version
void commit_security_version(uint32_t new_sec_ver) {
    // Write to OTP - cannot be reversed
    otp_write_security_version(new_sec_ver);
}
```

---

## 7. Intrusion Detection and Response

### 7.1 Intrusion Detection System (IDS)

#### 7.1.1 Detection Methods

**Signature-Based Detection**

```yaml
ids_signatures:
  - name: "CAN Bus Flooding"
    type: dos
    condition: "can_frame_rate > 5000/sec"
    severity: high
    action: alert_and_throttle

  - name: "Unauthorized Diagnostic Access"
    type: privilege_escalation
    condition: "obd_security_level == 3 AND auth_failed"
    severity: critical
    action: block_and_alert

  - name: "Suspicious CAN ID"
    type: anomaly
    condition: "can_id NOT IN whitelist"
    severity: medium
    action: log_and_inspect
```

**Anomaly-Based Detection**

```python
# Machine learning-based anomaly detection
class CANAnomalyDetector:
    def __init__(self):
        self.baseline_profile = self.load_baseline()
        self.model = self.load_ml_model()

    def detect_anomaly(self, can_frame):
        # Extract features
        features = {
            'can_id': can_frame.id,
            'dlc': can_frame.dlc,
            'data_entropy': self.calculate_entropy(can_frame.data),
            'inter_arrival_time': can_frame.timestamp - self.last_timestamp,
            'sequence_pattern': self.check_sequence(can_frame)
        }

        # Compare to baseline
        anomaly_score = self.model.predict(features)

        if anomaly_score > ANOMALY_THRESHOLD:
            return {
                'is_anomaly': True,
                'score': anomaly_score,
                'features': features,
                'recommendation': self.suggest_action(anomaly_score)
            }

        return {'is_anomaly': False}
```

#### 7.1.2 IDS Architecture

```
┌──────────────────────────────────────────────┐
│           IDS Central Processing             │
│  ┌────────────────────────────────────────┐  │
│  │   Rule Engine & ML Models              │  │
│  └────────────────────────────────────────┘  │
│  ┌────────────────────────────────────────┐  │
│  │   Event Correlation & Analysis         │  │
│  └────────────────────────────────────────┘  │
│  ┌────────────────────────────────────────┐  │
│  │   Alert Management & Response          │  │
│  └────────────────────────────────────────┘  │
└──────────┬───────────────────────────────────┘
           │
    ┌──────┴──────┬──────────┬──────────┐
    │             │          │          │
┌───▼───┐   ┌────▼────┐ ┌───▼───┐  ┌──▼────┐
│CAN IDS│   │V2X IDS  │ │OBD IDS│  │TE IDS │
│Monitor│   │Monitor  │ │Monitor│  │Monitor│
└───────┘   └─────────┘ └───────┘  └───────┘
```

### 7.2 Incident Response

#### 7.2.1 Response Actions

```c
typedef enum {
    RESPONSE_LOG,           // Log event only
    RESPONSE_ALERT,         // Alert driver/backend
    RESPONSE_THROTTLE,      // Rate limit traffic
    RESPONSE_ISOLATE,       // Isolate affected network
    RESPONSE_BLOCK,         // Block malicious entity
    RESPONSE_SHUTDOWN,      // Emergency shutdown
    RESPONSE_SAFE_MODE      // Enter safe/limp mode
} ResponseAction;

// Incident response decision matrix
ResponseAction determine_response(const SecurityIncident* incident) {
    // Critical safety impact -> immediate action
    if (incident->affects_safety) {
        if (incident->severity == CRITICAL) {
            return RESPONSE_SAFE_MODE;
        }
        return RESPONSE_ISOLATE;
    }

    // High confidence attack -> block
    if (incident->confidence > 0.9 && incident->severity >= HIGH) {
        return RESPONSE_BLOCK;
    }

    // Medium severity -> throttle and alert
    if (incident->severity == MEDIUM) {
        return RESPONSE_THROTTLE | RESPONSE_ALERT;
    }

    // Default: log and monitor
    return RESPONSE_LOG;
}
```

#### 7.2.2 Incident Logging

```json
{
  "incident": {
    "id": "INC-2025-001-12345",
    "timestamp": "2025-01-15T14:30:45.123Z",
    "vin": "WBA12345678901234",
    "type": "intrusion_attempt",
    "severity": "high",
    "confidence": 0.95
  },

  "detection": {
    "method": "signature",
    "signature_id": "CAN-FLOOD-001",
    "detector": "can_ids_monitor",
    "evidence": {
      "can_id": "0x7DF",
      "frame_rate": 8500,
      "threshold": 5000,
      "duration_seconds": 12
    }
  },

  "context": {
    "vehicle_state": "ignition_on",
    "speed_kmh": 65,
    "location": {
      "latitude": 37.7749,
      "longitude": -122.4194
    },
    "active_ecus": ["ECM", "TCM", "IVI", "ADAS"]
  },

  "response": {
    "actions_taken": ["alert_driver", "throttle_can", "notify_backend"],
    "effectiveness": "successful",
    "impact": "minimal"
  },

  "forensics": {
    "packet_capture": "base64_encoded_data...",
    "logs": ["log1.txt", "log2.txt"],
    "hash": "sha256:abc123..."
  }
}
```

### 7.3 Security Monitoring

#### 7.3.1 Real-Time Monitoring

```typescript
interface SecurityMonitor {
  // Network monitoring
  monitorNetwork(network: NetworkType): Observable<NetworkEvent>;

  // ECU health monitoring
  monitorECU(ecuId: string): Observable<ECUHealth>;

  // Behavioral analysis
  detectAnomalies(stream: Observable<Event>): Observable<Anomaly>;

  // Threat intelligence
  updateThreatFeeds(): Promise<void>;
}

// Example usage
const monitor = new SecurityMonitor();

monitor.monitorNetwork('CAN').subscribe(event => {
  if (event.type === 'anomaly') {
    handleAnomaly(event);
  }
});

monitor.detectAnomalies(canStream).subscribe(anomaly => {
  if (anomaly.severity >= Severity.HIGH) {
    triggerIncidentResponse(anomaly);
  }
});
```

#### 7.3.2 Metrics and KPIs

```yaml
security_metrics:
  # Detection metrics
  detection_rate: 0.98       # True positive rate
  false_positive_rate: 0.02  # False alarm rate
  detection_latency_ms: 150  # Time to detect

  # Response metrics
  response_time_ms: 500      # Time to respond
  mitigation_success: 0.95   # Successful mitigations

  # System health
  ids_uptime: 0.9999        # IDS availability
  log_integrity: 1.0         # No log tampering

  # Coverage
  monitored_networks: ["CAN", "FlexRay", "Ethernet", "V2X"]
  monitored_interfaces: ["OBD", "Telematics", "Bluetooth"]
  signature_count: 1500
  ml_model_accuracy: 0.94
```

---

## 8. ISO/SAE 21434 Compliance

### 8.1 Standard Overview

ISO/SAE 21434 "Road vehicles — Cybersecurity engineering" provides:
- Risk-based approach to cybersecurity
- Lifecycle coverage from concept to decommissioning
- Integration with ISO 26262 (functional safety)

### 8.2 TARA (Threat Analysis and Risk Assessment)

#### 8.2.1 Asset Identification

```yaml
assets:
  - id: ASSET-001
    name: "Steering Control ECU"
    type: ECU
    criticality: safety_critical
    value: very_high

  - id: ASSET-002
    name: "Infotainment System"
    type: ECU
    criticality: non_safety
    value: medium

  - id: ASSET-003
    name: "Vehicle Location Data"
    type: data
    criticality: privacy_sensitive
    value: high
```

#### 8.2.2 Threat Scenarios

```yaml
threat_scenarios:
  - id: THREAT-001
    name: "Remote Steering Manipulation"
    asset: ASSET-001
    attack_path: "Telematics -> Gateway -> CAN -> Steering ECU"
    attack_feasibility: low
    impact: severe

    mitigation:
      - control: "Network segmentation"
        effectiveness: high
      - control: "Message authentication"
        effectiveness: high
      - control: "Intrusion detection"
        effectiveness: medium
```

#### 8.2.3 Risk Determination

```
Risk = Attack Feasibility × Impact

Attack Feasibility (0-10):
- Elapsed Time
- Specialist Expertise
- Knowledge of System
- Window of Opportunity
- Equipment Required

Impact (1-4):
- Safety Impact
- Financial Impact
- Operational Impact
- Privacy Impact

Risk Level:
- Critical: Risk >= 36
- High:     Risk >= 24
- Medium:   Risk >= 12
- Low:      Risk < 12
```

### 8.3 Cybersecurity Requirements

```yaml
cybersecurity_requirements:
  - id: REQ-SEC-001
    description: "All ECU firmware shall be digitally signed"
    rationale: "Prevent unauthorized code execution"
    cal: CAL-4
    verification: "Signature verification test"

  - id: REQ-SEC-002
    description: "CAN messages from safety ECUs shall include MAC"
    rationale: "Prevent message spoofing on critical networks"
    cal: CAL-3
    verification: "MAC validation test"

  - id: REQ-SEC-003
    description: "OBD-II access shall be rate limited to 10 req/sec"
    rationale: "Prevent diagnostic port DoS attacks"
    cal: CAL-2
    verification: "Rate limit functional test"
```

### 8.4 CAL (Cybersecurity Assurance Level)

```
CAL-1: Basic assurance
- Standard security practices
- Basic testing
- Example: Comfort features

CAL-2: Medium assurance
- Enhanced security controls
- Security testing
- Example: Infotainment

CAL-3: High assurance
- Strong security controls
- Penetration testing
- Example: ADAS, Telematics

CAL-4: Very high assurance
- State-of-the-art security
- Comprehensive testing
- Example: Autonomous driving
```

### 8.5 Lifecycle Integration

```
ISO 21434 Lifecycle Phases:
┌──────────────────────────────────┐
│ 1. Concept Phase                 │
│    - Define cybersecurity goals  │
│    - Initial TARA                │
└─────────────┬────────────────────┘
              │
┌─────────────▼────────────────────┐
│ 2. Development Phase             │
│    - Security requirements       │
│    - Secure design               │
│    - Security testing            │
└─────────────┬────────────────────┘
              │
┌─────────────▼────────────────────┐
│ 3. Production Phase              │
│    - Secure provisioning         │
│    - Manufacturing security      │
└─────────────┬────────────────────┘
              │
┌─────────────▼────────────────────┐
│ 4. Operations Phase              │
│    - Monitoring                  │
│    - Incident response           │
│    - Updates                     │
└─────────────┬────────────────────┘
              │
┌─────────────▼────────────────────┐
│ 5. Decommissioning               │
│    - Secure data deletion        │
│    - Key revocation              │
└──────────────────────────────────┘
```

---

## 9. Data Formats

### 9.1 Security Event Format

```json
{
  "event_type": "security",
  "schema_version": "1.0",
  "event": {
    "id": "EVT-2025-001-12345",
    "timestamp": "2025-01-15T14:30:45.123Z",
    "severity": "high",
    "category": "intrusion_attempt",
    "source": {
      "type": "can_bus",
      "identifier": "CAN-HS",
      "ecu": "unknown"
    },
    "description": "Anomalous CAN traffic pattern detected",
    "details": {
      "can_id": "0x7DF",
      "data": "0x02010D00000000",
      "rate": 8500,
      "threshold_exceeded": true
    },
    "response": {
      "action": "throttle_and_alert",
      "status": "completed",
      "timestamp": "2025-01-15T14:30:45.456Z"
    }
  },
  "vehicle": {
    "vin": "WBA12345678901234",
    "model": "Example Model",
    "year": 2025
  },
  "metadata": {
    "firmware_version": "3.2.1",
    "security_profile": "enhanced",
    "location": {
      "latitude": 37.7749,
      "longitude": -122.4194
    }
  }
}
```

### 9.2 Vulnerability Report Format

```json
{
  "vulnerability": {
    "id": "WIA-AUTO-2025-001",
    "cve_id": "CVE-2025-12345",
    "title": "CAN Bus Message Injection Vulnerability",
    "description": "Insufficient validation allows unauthorized CAN messages",
    "discovered": "2025-01-10",
    "disclosed": "2025-01-20",
    "severity": "high",
    "cvss": {
      "version": "3.1",
      "score": 8.6,
      "vector": "CVSS:3.1/AV:A/AC:L/PR:N/UI:N/S:C/C:H/I:H/A:H"
    }
  },

  "affected": {
    "components": ["CAN Gateway ECU"],
    "versions": ["1.0", "1.1", "1.2"],
    "vehicles": {
      "models": ["Model A", "Model B"],
      "years": [2023, 2024, 2025],
      "vin_ranges": ["WBA*", "WBX*"]
    }
  },

  "impact": {
    "safety": "high",
    "privacy": "medium",
    "availability": "high",
    "attack_scenario": "Attacker with OBD access can inject steering commands"
  },

  "mitigation": {
    "patch_available": true,
    "patch_id": "PATCH-2025-001",
    "workarounds": [
      "Disable OBD-II access when not needed",
      "Monitor for anomalous CAN traffic"
    ],
    "fix_description": "Add MAC validation to CAN gateway"
  },

  "timeline": {
    "discovered": "2025-01-10",
    "vendor_notified": "2025-01-11",
    "patch_developed": "2025-01-15",
    "patch_released": "2025-01-20",
    "public_disclosure": "2025-01-25"
  }
}
```

---

## 10. API Interface

### 10.1 Security Assessment API

```typescript
interface SecurityAssessmentRequest {
  vin: string;
  assessment_type: 'full' | 'quick' | 'targeted';
  targets?: string[];  // Specific ECUs or networks
  options?: {
    include_penetration_test: boolean;
    deep_scan: boolean;
    compliance_check: boolean;
  };
}

interface SecurityAssessmentResponse {
  assessment_id: string;
  timestamp: Date;
  vin: string;

  overall_score: number;  // 0-100
  risk_level: 'low' | 'medium' | 'high' | 'critical';

  findings: {
    vulnerabilities: Vulnerability[];
    weaknesses: SecurityWeakness[];
    recommendations: Recommendation[];
  };

  compliance: {
    iso21434: ComplianceStatus;
    unece_wp29: ComplianceStatus;
    nhtsa: ComplianceStatus;
  };

  network_security: {
    can_bus: NetworkSecurityStatus;
    flexray: NetworkSecurityStatus;
    ethernet: NetworkSecurityStatus;
    external: ExternalInterfaceStatus;
  };
}
```

### 10.2 OTA Update API

```typescript
interface OTAUpdateRequest {
  vin: string;
  update_package_url: string;
  target_ecus: string[];
  priority: 'critical' | 'high' | 'normal' | 'low';
  schedule?: {
    immediate: boolean;
    scheduled_time?: Date;
    require_user_approval: boolean;
  };
}

interface OTAUpdateResponse {
  update_id: string;
  status: 'pending' | 'downloading' | 'verifying' |
          'installing' | 'completed' | 'failed';

  progress: {
    stage: string;
    percent_complete: number;
    estimated_time_remaining_seconds: number;
  };

  verification: {
    signature_valid: boolean;
    hash_valid: boolean;
    compatibility_check: boolean;
    security_scan: boolean;
  };

  installation: {
    pre_checks_passed: boolean;
    backup_created: boolean;
    rollback_available: boolean;
  };

  result?: {
    success: boolean;
    error_message?: string;
    affected_ecus: string[];
    new_versions: Record<string, string>;
  };
}
```

### 10.3 Intrusion Detection API

```typescript
interface IDSConfiguration {
  enabled: boolean;

  monitored_networks: {
    can_high_speed: boolean;
    can_low_speed: boolean;
    flexray: boolean;
    ethernet: boolean;
  };

  detection_methods: {
    signature_based: boolean;
    anomaly_based: boolean;
    behavioral_analysis: boolean;
  };

  response_actions: {
    log: boolean;
    alert_driver: boolean;
    alert_backend: boolean;
    auto_mitigate: boolean;
  };

  sensitivity: 'low' | 'medium' | 'high' | 'paranoid';
}

interface IDSAlert {
  alert_id: string;
  timestamp: Date;
  severity: 'info' | 'low' | 'medium' | 'high' | 'critical';

  threat: {
    type: string;
    description: string;
    confidence: number;  // 0-1
    attack_stage: string;
  };

  source: {
    network: string;
    identifier: string;
    location: string;
  };

  evidence: {
    raw_data: string;
    decoded_data: any;
    context: Record<string, any>;
  };

  response: {
    actions_taken: string[];
    effectiveness: string;
    user_action_required: boolean;
  };
}
```

---

## 11. Security Protocols

### 11.1 Secure Communication Protocol

```
Secure Channel Establishment:
┌─────────────────────────────────┐
│ 1. Mutual Authentication        │
│    - ECU A presents certificate │
│    - ECU B verifies and presents│
│    - Both verify certificates   │
└─────────────┬───────────────────┘
              │
┌─────────────▼───────────────────┐
│ 2. Key Exchange                 │
│    - ECDH key agreement         │
│    - Derive session keys        │
│    - KDF: HKDF-SHA256           │
└─────────────┬───────────────────┘
              │
┌─────────────▼───────────────────┐
│ 3. Secure Communication         │
│    - Encryption: AES-256-GCM    │
│    - Authentication: HMAC       │
│    - Freshness: Counter/Time    │
└─────────────────────────────────┘
```

### 11.2 Key Management Protocol

```yaml
key_management:
  # Key hierarchy
  hierarchy:
    root_key:
      storage: HSM
      rotation: never
      backup: secure_facility

    device_key:
      derived_from: root_key
      storage: HSM
      rotation: annually
      unique_per: vehicle

    session_key:
      derived_from: device_key
      storage: volatile_memory
      rotation: per_session
      lifetime: 24_hours

  # Key provisioning
  provisioning:
    phase: manufacturing
    method: secure_facility
    verification: dual_approval
    logging: full_audit_trail

  # Key rotation
  rotation:
    schedule: automatic
    trigger:
      - time_based: true
      - usage_based: true
      - compromise_suspected: true
    process: seamless
    old_key_retention: 30_days
```

### 11.3 Certificate Management

```typescript
interface CertificatePolicy {
  // Certificate lifecycle
  validity_period: {
    enrollment_cert: '10 years',
    authorization_cert: '3 years',
    pseudonym_cert: '1 week'
  };

  // Issuance
  issuance: {
    approval_required: boolean;
    identity_verification: 'strong' | 'medium' | 'basic';
    rate_limit: number;  // certs per day
  };

  // Revocation
  revocation: {
    crl_update_frequency: '24 hours';
    ocsp_enabled: boolean;
    revocation_reasons: string[];
  };

  // Renewal
  renewal: {
    auto_renewal: boolean;
    renewal_window_days: 30;
    notification: boolean;
  };
}
```

---

## 12. References

### 12.1 Standards and Regulations

1. **ISO/SAE 21434:2021** - Road vehicles — Cybersecurity engineering
2. **UNECE WP.29** - UN Regulation on Cybersecurity and Software Updates
3. **SAE J3061** - Cybersecurity Guidebook for Cyber-Physical Vehicle Systems
4. **ISO 26262** - Road vehicles — Functional safety
5. **IEEE 1609.2** - Security Services for Applications and Management Messages
6. **NIST Cybersecurity Framework** - Framework for Improving Critical Infrastructure
7. **AUTOSAR** - Automotive Open System Architecture security specifications

### 12.2 Cryptographic Standards

| Algorithm | Standard | Key Size | Use Case |
|-----------|----------|----------|----------|
| AES-GCM | FIPS 197 | 128/256 bit | Symmetric encryption |
| ECDSA | FIPS 186-4 | P-256/P-384 | Digital signatures |
| RSA | PKCS#1 | 2048/4096 bit | Legacy signatures |
| HMAC-SHA256 | FIPS 198-1 | 256 bit | Message authentication |
| ECDH | SP 800-56A | P-256/P-384 | Key exchange |
| HKDF | RFC 5869 | Variable | Key derivation |

### 12.3 Security Guidelines

- **NHTSA Cybersecurity Best Practices** (2016)
- **ENISA Good Practices for Security of Smart Cars** (2019)
- **Auto-ISAC Automotive Cybersecurity Best Practices** (2022)
- **OWASP Embedded Application Security** (2023)

### 12.4 Research Papers

1. Miller, C., & Valasek, C. (2015). "Remote Exploitation of an Unaltered Passenger Vehicle"
2. Checkoway, S., et al. (2011). "Comprehensive Experimental Analyses of Automotive Attack Surfaces"
3. Koscher, K., et al. (2010). "Experimental Security Analysis of a Modern Automobile"
4. Petit, J., & Shladover, S. E. (2015). "Potential Cyberattacks on Automated Vehicles"

### 12.5 WIA Standards Integration

- **WIA-INTENT**: Intent-based security policy management
- **WIA-OMNI-API**: Unified API gateway with security
- **WIA-BLOCKCHAIN**: Immutable audit logging
- **WIA-IOT**: IoT device security for connected sensors
- **WIA-SOCIAL**: Secure V2V social networking
- **WIA-QUANTUM**: Post-quantum cryptography readiness

### 12.6 Industry Organizations

- **Auto-ISAC** - Automotive Information Sharing and Analysis Center
- **SAE International** - Society of Automotive Engineers
- **ISO** - International Organization for Standardization
- **UNECE** - United Nations Economic Commission for Europe
- **NHTSA** - National Highway Traffic Safety Administration

---

## Appendix A: Threat Examples

### A.1 CAN Bus Attack Example

```c
// Attacker code (for educational purposes only)
void inject_malicious_can_frame(void) {
    can_frame_t frame;

    // Target: Speed display (example CAN ID)
    frame.can_id = 0x123;
    frame.can_dlc = 8;

    // Inject false speed value (200 km/h)
    frame.data[0] = 0xC8;  // 200 km/h
    frame.data[1] = 0x00;
    // ... rest of data

    // Send frame to CAN bus
    can_send(&frame);
}

// Defense: MAC validation
bool validate_can_frame(const can_frame_t* frame) {
    // Verify MAC appended to frame
    uint8_t received_mac[4];
    memcpy(received_mac, &frame->data[4], 4);

    uint8_t calculated_mac[4];
    compute_can_mac(frame, calculated_mac);

    return memcmp(received_mac, calculated_mac, 4) == 0;
}
```

### A.2 Risk Calculation Example

```
Scenario: Remote ECU Compromise via Telematics

Attack Feasibility Analysis:
- Elapsed Time: 6 (< 1 day) = 1 point
- Specialist Expertise: Expert (6 points)
- Knowledge of System: Public (0 points)
- Window of Opportunity: Unlimited (0 points)
- Equipment: Standard (0 points)

Attack Feasibility = 1 + 6 + 0 + 0 + 0 = 7 (out of 24)

Impact Analysis:
- Safety: Severe (4 points)
- Financial: Major (3 points)
- Operational: Severe (4 points)
- Privacy: Major (3 points)

Impact = max(4, 3, 4, 3) = 4 (Severe)

Risk = Attack Feasibility × Impact = 7 × 4 = 28
Risk Level: HIGH (requires mitigation)
```

---

**弘益人間 (홍익인간) · Benefit All Humanity**

*WIA-AUTO-023 Specification v1.0*
*© 2025 SmileStory Inc. / WIA*
*MIT License*
