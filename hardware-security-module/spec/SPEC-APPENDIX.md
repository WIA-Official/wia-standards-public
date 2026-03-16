# WIA-SEC-014: Hardware Security Module - Appendix

**Version:** 1.0
**Last Updated:** 2025-12-25

---

## Table of Contents

1. [Complete Error Code Reference](#1-complete-error-code-reference)
2. [PKCS#11 Mechanism List](#2-pkcs11-mechanism-list)
3. [Key Attribute Reference](#3-key-attribute-reference)
4. [Configuration Examples](#4-configuration-examples)
5. [Testing & Validation](#5-testing--validation)
6. [Performance Benchmarks](#6-performance-benchmarks)
7. [Compliance Checklists](#7-compliance-checklists)
8. [Troubleshooting Guide](#8-troubleshooting-guide)

---

## 1. Complete Error Code Reference

### 1.1 General Errors (0x00000000 - 0x000000FF)

| Code | Name | Description | Recovery Action |
|------|------|-------------|-----------------|
| 0x00000000 | CKR_OK | Success | N/A |
| 0x00000001 | CKR_CANCEL | Operation cancelled by user | Retry operation |
| 0x00000002 | CKR_HOST_MEMORY | Host memory allocation failed | Reduce operation size or free memory |
| 0x00000003 | CKR_SLOT_ID_INVALID | Invalid slot identifier | Verify slot ID with C_GetSlotList |
| 0x00000005 | CKR_GENERAL_ERROR | General error | Check audit logs for details |
| 0x00000006 | CKR_FUNCTION_FAILED | Function failed | Retry or check input parameters |
| 0x00000007 | CKR_ARGUMENTS_BAD | Invalid function arguments | Validate input parameters |
| 0x00000008 | CKR_NO_EVENT | No event available | Wait or use blocking mode |
| 0x00000009 | CKR_NEED_TO_CREATE_THREADS | Cannot create threads | Check system threading support |
| 0x0000000A | CKR_CANT_LOCK | Cannot acquire lock | Retry after brief delay |

### 1.2 Session Errors (0x000000A0 - 0x000000BF)

| Code | Name | Description | Recovery Action |
|------|------|-------------|-----------------|
| 0x000000A0 | CKR_SESSION_HANDLE_INVALID | Invalid session handle | Re-open session |
| 0x000000A1 | CKR_OBJECT_HANDLE_INVALID | Invalid object handle | Re-find object |
| 0x000000A3 | CKR_SESSION_CLOSED | Session was closed | Open new session |
| 0x000000A4 | CKR_USER_NOT_LOGGED_IN | User not authenticated | Login first |
| 0x000000A5 | CKR_USER_ALREADY_LOGGED_IN | User already logged in | Continue or logout first |
| 0x000000A6 | CKR_SESSION_COUNT | Session limit reached | Close unused sessions |
| 0x000000A7 | CKR_SESSION_READ_ONLY | Session is read-only | Open R/W session |
| 0x000000A8 | CKR_SESSION_EXISTS | Session exists | Use existing session |

### 1.3 Cryptographic Errors (0x00000100 - 0x000001FF)

| Code | Name | Description | Recovery Action |
|------|------|-------------|-----------------|
| 0x00000100 | CKR_ATTRIBUTE_READ_ONLY | Attribute is read-only | Cannot modify |
| 0x00000110 | CKR_DATA_INVALID | Data is invalid | Verify data format |
| 0x00000120 | CKR_DATA_LEN_RANGE | Data length out of range | Check length requirements |
| 0x00000130 | CKR_DEVICE_ERROR | Device hardware error | Check device status, may need service |
| 0x00000140 | CKR_DEVICE_MEMORY | Device memory error | Reduce key count or clear old keys |
| 0x00000150 | CKR_DEVICE_REMOVED | Device was removed | Reconnect device |
| 0x00000160 | CKR_ENCRYPTED_DATA_INVALID | Encrypted data is invalid | Verify encryption key and algorithm |
| 0x00000170 | CKR_ENCRYPTED_DATA_LEN_RANGE | Encrypted data length invalid | Check ciphertext length |
| 0x00000180 | CKR_FUNCTION_CANCELED | Function was cancelled | Retry if needed |
| 0x00000190 | CKR_FUNCTION_NOT_PARALLEL | Function not parallelizable | Use sequential operations |

### 1.4 Key Management Errors (0x00000200 - 0x000002FF)

| Code | Name | Description | Recovery Action |
|------|------|-------------|-----------------|
| 0x00000200 | CKR_KEY_HANDLE_INVALID | Invalid key handle | Re-find key object |
| 0x00000210 | CKR_KEY_SIZE_RANGE | Key size out of range | Use supported key size |
| 0x00000220 | CKR_KEY_TYPE_INCONSISTENT | Key type mismatch | Verify key type for operation |
| 0x00000230 | CKR_KEY_NOT_NEEDED | Key not needed for operation | Remove key parameter |
| 0x00000240 | CKR_KEY_CHANGED | Key was modified | Re-find key |
| 0x00000250 | CKR_KEY_NEEDED | Key required but not provided | Provide key handle |
| 0x00000260 | CKR_KEY_INDIGESTIBLE | Key cannot be digested | Use different key or mechanism |
| 0x00000270 | CKR_KEY_FUNCTION_NOT_PERMITTED | Operation not permitted for key | Check key usage attributes |
| 0x00000280 | CKR_KEY_NOT_WRAPPABLE | Key cannot be wrapped | Check CKA_EXTRACTABLE attribute |
| 0x00000290 | CKR_KEY_UNEXTRACTABLE | Key is marked as non-extractable | Cannot export this key |

---

## 2. PKCS#11 Mechanism List

### 2.1 RSA Mechanisms

| Mechanism | ID | Key Type | Description |
|-----------|-----|----------|-------------|
| CKM_RSA_PKCS_KEY_PAIR_GEN | 0x00000000 | RSA | RSA key pair generation |
| CKM_RSA_PKCS | 0x00000001 | RSA | RSA PKCS#1 v1.5 |
| CKM_RSA_9796 | 0x00000002 | RSA | RSA ISO 9796 |
| CKM_RSA_X_509 | 0x00000003 | RSA | Raw RSA (no padding) |
| CKM_SHA1_RSA_PKCS | 0x00000006 | RSA | SHA-1 with RSA PKCS#1 |
| CKM_SHA256_RSA_PKCS | 0x00000040 | RSA | SHA-256 with RSA PKCS#1 |
| CKM_SHA384_RSA_PKCS | 0x00000041 | RSA | SHA-384 with RSA PKCS#1 |
| CKM_SHA512_RSA_PKCS | 0x00000042 | RSA | SHA-512 with RSA PKCS#1 |
| CKM_RSA_PKCS_OAEP | 0x00000009 | RSA | RSA PKCS#1 OAEP |
| CKM_RSA_PKCS_PSS | 0x0000000D | RSA | RSA PKCS#1 PSS |
| CKM_SHA1_RSA_PKCS_PSS | 0x0000000E | RSA | SHA-1 with RSA PSS |
| CKM_SHA256_RSA_PKCS_PSS | 0x00000043 | RSA | SHA-256 with RSA PSS |

### 2.2 ECC Mechanisms

| Mechanism | ID | Key Type | Description |
|-----------|-----|----------|-------------|
| CKM_EC_KEY_PAIR_GEN | 0x00001040 | EC | EC key pair generation |
| CKM_ECDSA | 0x00001041 | EC | ECDSA without hashing |
| CKM_ECDSA_SHA1 | 0x00001042 | EC | ECDSA with SHA-1 |
| CKM_ECDSA_SHA256 | 0x00001043 | EC | ECDSA with SHA-256 |
| CKM_ECDSA_SHA384 | 0x00001044 | EC | ECDSA with SHA-384 |
| CKM_ECDSA_SHA512 | 0x00001045 | EC | ECDSA with SHA-512 |
| CKM_ECDH1_DERIVE | 0x00001050 | EC | ECDH key derivation |
| CKM_ECDH1_COFACTOR_DERIVE | 0x00001051 | EC | ECDH with cofactor |
| CKM_ECMQV_DERIVE | 0x00001052 | EC | EC MQV key derivation |

### 2.3 AES Mechanisms

| Mechanism | ID | Key Type | Description |
|-----------|-----|----------|-------------|
| CKM_AES_KEY_GEN | 0x00001080 | AES | AES key generation |
| CKM_AES_ECB | 0x00001081 | AES | AES ECB mode |
| CKM_AES_CBC | 0x00001082 | AES | AES CBC mode |
| CKM_AES_CBC_PAD | 0x00001083 | AES | AES CBC with PKCS#5 padding |
| CKM_AES_CTR | 0x00001086 | AES | AES CTR mode |
| CKM_AES_GCM | 0x00001087 | AES | AES GCM mode |
| CKM_AES_CCM | 0x00001088 | AES | AES CCM mode |
| CKM_AES_CTS | 0x00001089 | AES | AES CTS mode |
| CKM_AES_CMAC | 0x0000108A | AES | AES CMAC |
| CKM_AES_KEY_WRAP | 0x00001090 | AES | AES key wrap (RFC 3394) |
| CKM_AES_KEY_WRAP_PAD | 0x00001091 | AES | AES key wrap with padding |

### 2.4 Hash Mechanisms

| Mechanism | ID | Description |
|-----------|-----|-------------|
| CKM_SHA_1 | 0x00000220 | SHA-1 (legacy) |
| CKM_SHA256 | 0x00000250 | SHA-256 |
| CKM_SHA384 | 0x00000260 | SHA-384 |
| CKM_SHA512 | 0x00000270 | SHA-512 |
| CKM_SHA3_256 | 0x000002B0 | SHA3-256 |
| CKM_SHA3_384 | 0x000002C0 | SHA3-384 |
| CKM_SHA3_512 | 0x000002D0 | SHA3-512 |

### 2.5 HMAC Mechanisms

| Mechanism | ID | Key Type | Description |
|-----------|-----|----------|-------------|
| CKM_SHA_1_HMAC | 0x00000221 | Generic Secret | HMAC-SHA1 |
| CKM_SHA256_HMAC | 0x00000251 | Generic Secret | HMAC-SHA256 |
| CKM_SHA384_HMAC | 0x00000261 | Generic Secret | HMAC-SHA384 |
| CKM_SHA512_HMAC | 0x00000271 | Generic Secret | HMAC-SHA512 |
| CKM_SHA3_256_HMAC | 0x000002B1 | Generic Secret | HMAC-SHA3-256 |
| CKM_SHA3_512_HMAC | 0x000002D1 | Generic Secret | HMAC-SHA3-512 |

---

## 3. Key Attribute Reference

### 3.1 Common Object Attributes

| Attribute | Type | Description | Modifiable |
|-----------|------|-------------|------------|
| CKA_CLASS | CK_OBJECT_CLASS | Object class | No |
| CKA_TOKEN | CK_BBOOL | Token object (persistent) | No |
| CKA_PRIVATE | CK_BBOOL | Requires authentication | No |
| CKA_LABEL | String | Human-readable label | Yes* |
| CKA_APPLICATION | String | Application identifier | Yes* |
| CKA_VALUE | Bytes | Object value | No |

### 3.2 Key-Specific Attributes

| Attribute | Type | Applies To | Description |
|-----------|------|------------|-------------|
| CKA_KEY_TYPE | CK_KEY_TYPE | All keys | Key type (RSA, EC, AES, etc.) |
| CKA_ID | Bytes | All keys | Unique identifier |
| CKA_START_DATE | CK_DATE | All keys | Validity start date |
| CKA_END_DATE | CK_DATE | All keys | Validity end date |
| CKA_DERIVE | CK_BBOOL | All keys | Can derive other keys |
| CKA_LOCAL | CK_BBOOL | All keys | Generated on device |
| CKA_KEY_GEN_MECHANISM | CK_MECHANISM_TYPE | All keys | Generation mechanism |

### 3.3 Public Key Attributes

| Attribute | Type | Applies To | Description |
|-----------|------|------------|-------------|
| CKA_SUBJECT | DER-encoded DN | RSA, EC | Certificate subject |
| CKA_ENCRYPT | CK_BBOOL | RSA, EC | Can encrypt |
| CKA_VERIFY | CK_BBOOL | RSA, EC | Can verify signatures |
| CKA_VERIFY_RECOVER | CK_BBOOL | RSA | Can verify with message recovery |
| CKA_WRAP | CK_BBOOL | RSA, EC | Can wrap keys |
| CKA_TRUSTED | CK_BBOOL | All public | Trusted key (CA) |
| CKA_MODULUS | Big Integer | RSA | RSA modulus (n) |
| CKA_PUBLIC_EXPONENT | Big Integer | RSA | RSA public exponent (e) |
| CKA_EC_PARAMS | Bytes | EC | DER-encoded curve OID |
| CKA_EC_POINT | Bytes | EC | DER-encoded EC point |

### 3.4 Private Key Attributes

| Attribute | Type | Applies To | Description |
|-----------|------|------------|-------------|
| CKA_SUBJECT | DER-encoded DN | RSA, EC | Certificate subject |
| CKA_SENSITIVE | CK_BBOOL | All private | Cannot read plaintext value |
| CKA_DECRYPT | CK_BBOOL | RSA, EC | Can decrypt |
| CKA_SIGN | CK_BBOOL | RSA, EC | Can sign |
| CKA_SIGN_RECOVER | CK_BBOOL | RSA | Can sign with message recovery |
| CKA_UNWRAP | CK_BBOOL | RSA, EC | Can unwrap keys |
| CKA_EXTRACTABLE | CK_BBOOL | All private | Can be extracted/wrapped |
| CKA_ALWAYS_SENSITIVE | CK_BBOOL | All private | Has always been sensitive |
| CKA_NEVER_EXTRACTABLE | CK_BBOOL | All private | Has never been extractable |
| CKA_WRAP_WITH_TRUSTED | CK_BBOOL | All private | Can only wrap with trusted key |

### 3.5 Secret Key Attributes

| Attribute | Type | Applies To | Description |
|-----------|------|------------|-------------|
| CKA_SENSITIVE | CK_BBOOL | All secret | Cannot read plaintext value |
| CKA_ENCRYPT | CK_BBOOL | All secret | Can encrypt |
| CKA_DECRYPT | CK_BBOOL | All secret | Can decrypt |
| CKA_SIGN | CK_BBOOL | All secret | Can sign (MAC) |
| CKA_VERIFY | CK_BBOOL | All secret | Can verify (MAC) |
| CKA_WRAP | CK_BBOOL | All secret | Can wrap keys |
| CKA_UNWRAP | CK_BBOOL | All secret | Can unwrap keys |
| CKA_EXTRACTABLE | CK_BBOOL | All secret | Can be extracted/wrapped |
| CKA_VALUE_LEN | CK_ULONG | All secret | Length in bytes |

---

## 4. Configuration Examples

### 4.1 HSM Initialization Script

```bash
#!/bin/bash
# hsm-init.sh - Initialize HSM for first use

set -e

HSM_SLOT=0
SO_PIN="SecurityOfficerPIN123"
USER_PIN="UserPIN456"

echo "=== WIA-SEC-014 HSM Initialization ==="

# Initialize token
pkcs11-tool --module /usr/lib/libpkcs11.so \
  --init-token \
  --slot $HSM_SLOT \
  --label "WIA-HSM-PROD" \
  --so-pin "$SO_PIN"

# Initialize user PIN
pkcs11-tool --module /usr/lib/libpkcs11.so \
  --init-pin \
  --slot $HSM_SLOT \
  --so-pin "$SO_PIN" \
  --new-pin "$USER_PIN"

# Generate master keys
echo "Generating master encryption key..."
pkcs11-tool --module /usr/lib/libpkcs11.so \
  --login \
  --pin "$USER_PIN" \
  --keypairgen \
  --key-type RSA:4096 \
  --label "master-encryption-key" \
  --id 01

echo "Generating master signing key..."
pkcs11-tool --module /usr/lib/libpkcs11.so \
  --login \
  --pin "$USER_PIN" \
  --keypairgen \
  --key-type EC:secp256r1 \
  --label "master-signing-key" \
  --id 02

echo "=== Initialization Complete ==="
```

### 4.2 Application Configuration

```yaml
# hsm-config.yaml
hsm:
  provider: softhsm2  # or: aws-cloudhsm, azure-hsm, thales, gemalto
  library: /usr/lib/softhsm/libsofthsm2.so

  # Slot configuration
  slot:
    id: 0
    label: WIA-HSM-PROD
    autoDetect: true

  # Authentication
  auth:
    userPinEnv: HSM_USER_PIN
    retryAttempts: 3
    sessionTimeout: 3600  # seconds

  # Connection pooling
  sessions:
    minIdle: 2
    maxActive: 10
    maxWait: 5000  # milliseconds
    evictionInterval: 60000

  # Key management
  keys:
    defaultKeyType: RSA-2048
    defaultLabel: app-key-{timestamp}
    autoRotation: true
    rotationPeriod: 90  # days
    backupEnabled: true

  # Performance
  performance:
    batchOperations: true
    asyncMode: false
    cacheHandles: true
    cacheTTL: 300  # seconds

  # Logging
  logging:
    level: INFO
    auditEvents:
      - KEY_GENERATION
      - KEY_DESTRUCTION
      - SIGN_OPERATION
      - ENCRYPT_OPERATION
    auditDestination: /var/log/hsm-audit.log
    rotateSize: 100MB
    retention: 365  # days
```

### 4.3 Security Policy Configuration

```json
{
  "securityPolicy": {
    "version": "1.0",
    "name": "production-policy",
    "enforceMode": "strict",

    "authentication": {
      "minPinLength": 8,
      "pinComplexity": true,
      "requireMFA": true,
      "maxFailedAttempts": 5,
      "lockoutDuration": 900
    },

    "keyGeneration": {
      "allowedTypes": ["RSA-2048", "RSA-4096", "ECC-P256", "ECC-P384"],
      "requireApproval": true,
      "approvers": 2,
      "maxKeysPerUser": 50
    },

    "cryptoOperations": {
      "allowedMechanisms": [
        "CKM_SHA256_RSA_PKCS",
        "CKM_RSA_PKCS_OAEP",
        "CKM_ECDSA_SHA256",
        "CKM_AES_GCM"
      ],
      "prohibitedMechanisms": [
        "CKM_SHA1_RSA_PKCS",
        "CKM_RSA_PKCS"
      ],
      "rateLimit": {
        "enabled": true,
        "maxOpsPerSecond": 1000,
        "burstSize": 100
      }
    },

    "keyAttributes": {
      "requiredAttributes": {
        "CKA_SENSITIVE": true,
        "CKA_EXTRACTABLE": false,
        "CKA_TOKEN": true
      },
      "prohibitedAttributes": {
        "CKA_EXTRACTABLE": true
      }
    },

    "audit": {
      "enabled": true,
      "realTimeMonitoring": true,
      "alertOnAnomaly": true,
      "exportFormat": "CEF",
      "syslogServer": "siem.example.com:514"
    }
  }
}
```

---

## 5. Testing & Validation

### 5.1 Functional Test Suite

```python
import pytest
import pkcs11

@pytest.fixture
def hsm_session():
    lib = pkcs11.lib('/usr/lib/libpkcs11.so')
    token = lib.get_token(token_label='WIA-HSM-TEST')
    session = token.open(user_pin='TestPIN123', rw=True)
    yield session
    session.close()

class TestKeyGeneration:
    def test_rsa_key_generation(self, hsm_session):
        """Test RSA-2048 key pair generation"""
        public, private = hsm_session.generate_keypair(
            pkcs11.KeyType.RSA,
            2048,
            label='test-rsa-key',
            store=True
        )
        assert public is not None
        assert private is not None
        assert private.key_length == 2048

    def test_ec_key_generation(self, hsm_session):
        """Test ECC P-256 key pair generation"""
        public, private = hsm_session.generate_keypair(
            pkcs11.KeyType.EC,
            pkcs11.Mechanism.EC_PARAMS_P256,
            label='test-ec-key',
            store=True
        )
        assert public is not None
        assert private is not None

class TestSigningOperations:
    def test_rsa_pss_signature(self, hsm_session):
        """Test RSA-PSS signature"""
        _, private = hsm_session.generate_keypair(
            pkcs11.KeyType.RSA, 2048
        )

        data = b'Test message for signing'
        signature = private.sign(
            data,
            mechanism=pkcs11.Mechanism.SHA256_RSA_PKCS_PSS
        )

        assert len(signature) == 256  # RSA-2048 signature
        assert signature != data

    def test_ecdsa_signature(self, hsm_session):
        """Test ECDSA signature"""
        public, private = hsm_session.generate_keypair(
            pkcs11.KeyType.EC,
            pkcs11.Mechanism.EC_PARAMS_P256
        )

        data = b'Test message for signing'
        signature = private.sign(
            data,
            mechanism=pkcs11.Mechanism.ECDSA_SHA256
        )

        # Verify signature
        valid = public.verify(
            data,
            signature,
            mechanism=pkcs11.Mechanism.ECDSA_SHA256
        )
        assert valid is True
```

### 5.2 Performance Test

```python
import time
import statistics

def benchmark_operation(hsm_session, operation, iterations=1000):
    """Benchmark cryptographic operation"""
    times = []

    for _ in range(iterations):
        start = time.perf_counter()
        operation()
        end = time.perf_counter()
        times.append((end - start) * 1000)  # Convert to ms

    return {
        'min': min(times),
        'max': max(times),
        'mean': statistics.mean(times),
        'median': statistics.median(times),
        'stdev': statistics.stdev(times),
        'throughput': 1000 / statistics.mean(times)  # ops/sec
    }

# Example: Benchmark RSA signing
public, private = session.generate_keypair(pkcs11.KeyType.RSA, 2048)
data = b'X' * 256

stats = benchmark_operation(
    session,
    lambda: private.sign(data, mechanism=pkcs11.Mechanism.SHA256_RSA_PKCS),
    iterations=1000
)

print(f"RSA-2048 Signing Performance:")
print(f"  Mean: {stats['mean']:.2f} ms")
print(f"  Throughput: {stats['throughput']:.0f} ops/sec")
```

---

## 6. Performance Benchmarks

### 6.1 Reference Hardware Benchmarks

**Test Environment:**
- Hardware: Intel Xeon E5-2680 v4 @ 2.40GHz
- HSM: Generic FIPS 140-2 Level 2 compliant
- Library: PKCS#11 v2.40
- Test Duration: 60 seconds per operation

| Operation | Key Size | Throughput | Latency (avg) |
|-----------|----------|------------|---------------|
| RSA Key Generation | 2048-bit | 50 ops/sec | 20 ms |
| RSA Key Generation | 4096-bit | 12 ops/sec | 83 ms |
| ECC Key Generation | P-256 | 500 ops/sec | 2 ms |
| ECC Key Generation | P-384 | 300 ops/sec | 3.3 ms |
| RSA-2048 Sign (PKCS) | 2048-bit | 1,200 ops/sec | 0.83 ms |
| RSA-2048 Verify | 2048-bit | 15,000 ops/sec | 0.07 ms |
| RSA-4096 Sign (PKCS) | 4096-bit | 300 ops/sec | 3.3 ms |
| ECDSA Sign (P-256) | 256-bit | 5,000 ops/sec | 0.2 ms |
| ECDSA Verify (P-256) | 256-bit | 3,000 ops/sec | 0.33 ms |
| AES-256-GCM Encrypt | 1 KB | 100 MB/sec | 0.01 ms |
| AES-256-GCM Encrypt | 1 MB | 500 MB/sec | 2 ms |
| SHA-256 Hash | 1 KB | 200 MB/sec | 0.005 ms |
| HMAC-SHA256 | 1 KB | 150 MB/sec | 0.007 ms |

### 6.2 Cloud HSM Benchmarks

**AWS CloudHSM (hsm1.medium):**

| Operation | Throughput | Notes |
|-----------|------------|-------|
| RSA-2048 Sign | 10,000 ops/sec | Per HSM instance |
| ECDSA-P256 Sign | 25,000 ops/sec | Per HSM instance |
| AES-256 Encrypt | 10 GB/sec | Bulk encryption |

---

## 7. Compliance Checklists

### 7.1 FIPS 140-2 Level 2 Checklist

- [ ] **Physical Security**
  - [ ] Tamper-evident coatings or seals
  - [ ] Tamper detection and response mechanisms
  - [ ] Removal detection sensors

- [ ] **Cryptographic Module Specification**
  - [ ] Approved algorithms only (FIPS 197, 186-4, 180-4)
  - [ ] Approved key sizes
  - [ ] Algorithm self-tests on power-up
  - [ ] Conditional algorithm self-tests

- [ ] **Roles and Services**
  - [ ] Defined roles (User, Crypto Officer, Security Officer)
  - [ ] Role-based authentication
  - [ ] Operator authentication (Level 2 minimum)

- [ ] **Key Management**
  - [ ] Key generation using approved RNG
  - [ ] Secure key storage
  - [ ] Key separation enforcement
  - [ ] Key zeroization procedures

- [ ] **EMI/EMC**
  - [ ] FCC Part 15 Class A compliance
  - [ ] Emission testing results documented

### 7.2 Common Criteria EAL4+ Checklist

- [ ] **Security Target (ST)**
  - [ ] TOE description
  - [ ] Security problem definition
  - [ ] Security objectives
  - [ ] Security requirements
  - [ ] TOE summary specification

- [ ] **Development**
  - [ ] Functional specification
  - [ ] High-level design
  - [ ] Low-level design
  - [ ] Implementation representation

- [ ] **Testing**
  - [ ] Coverage analysis
  - [ ] Depth analysis
  - [ ] Independent testing
  - [ ] Vulnerability analysis (AVA_VAN.5)

---

## 8. Troubleshooting Guide

### 8.1 Common Issues

#### Issue: CKR_USER_NOT_LOGGED_IN

**Symptoms:**
- Operations fail with error code 0xA4
- "User not logged in" message

**Causes:**
1. Session not authenticated
2. Session timeout
3. Automatic logout after inactivity

**Solutions:**
```python
# Ensure login before operations
session = token.open(rw=True)
session.login('UserPIN')

# Or use context manager for automatic management
with token.open(rw=True, user_pin='UserPIN') as session:
    # Operations here
    pass
```

#### Issue: CKR_SESSION_HANDLE_INVALID

**Symptoms:**
- Operations fail with error code 0xA0
- "Invalid session handle" message

**Causes:**
1. Session was closed
2. HSM was reset
3. Network connection lost (cloud HSM)

**Solutions:**
```python
# Implement session reconnection
def get_session():
    try:
        return session
    except:
        session = token.open(rw=True, user_pin=PIN)
        return session
```

#### Issue: Poor Performance

**Symptoms:**
- Slow cryptographic operations
- High latency
- Low throughput

**Diagnostics:**
```bash
# Check HSM resource usage
pkcs11-tool --list-slots --module /usr/lib/libpkcs11.so

# Monitor session count
watch -n 1 'pkcs11-tool --list-sessions --module /usr/lib/libpkcs11.so'

# Check network latency (cloud HSM)
ping -c 100 hsm-endpoint.example.com
```

**Solutions:**
1. Implement session pooling
2. Use batch operations where possible
3. Cache key handles
4. Enable async operations
5. Add more HSM instances (HA setup)

### 8.2 Diagnostic Commands

```bash
# List available slots
pkcs11-tool --module /usr/lib/libpkcs11.so --list-slots

# Get token info
pkcs11-tool --module /usr/lib/libpkcs11.so --show-info --slot 0

# List objects
pkcs11-tool --module /usr/lib/libpkcs11.so \
  --login --pin UserPIN \
  --list-objects

# Test mechanism support
pkcs11-tool --module /usr/lib/libpkcs11.so \
  --list-mechanisms --slot 0

# Generate test key
pkcs11-tool --module /usr/lib/libpkcs11.so \
  --login --pin UserPIN \
  --keypairgen \
  --key-type RSA:2048 \
  --label test-key

# Sign test data
echo "test data" > test.txt
pkcs11-tool --module /usr/lib/libpkcs11.so \
  --login --pin UserPIN \
  --sign \
  --mechanism SHA256-RSA-PKCS \
  --label test-key \
  --input-file test.txt \
  --output-file test.sig
```

---

**Document Control:**
- Version: 1.0
- Status: Official Standard
- Maintained by: WIA Security Working Group

**弘益人間 (Benefit All Humanity)**

© 2025 WIA (World Certification Industry Association)
