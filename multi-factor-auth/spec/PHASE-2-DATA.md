# WIA-SEC-008: Multi-Factor Authentication - PHASE 2 DATA FORMAT

**Version:** 1.0.0
**Status:** Draft
**Last Updated:** 2025-12-25
**Category:** Security (SEC)

---

## 1. Overview

This phase defines data formats, schemas, and structures for Multi-Factor Authentication, including OTP formats, credential storage, and authentication tokens.

### Philosophy: 弘益人間 (Benefit All Humanity)

Standardized data formats enable interoperability, security, and accessibility across different MFA implementations, benefiting all users and organizations.

---

## 2. OTP Data Formats

### 2.1 TOTP (Time-Based OTP) - RFC 6238

**Parameters:**
```json
{
  "type": "totp",
  "secret": "BASE32_ENCODED_SECRET",
  "algorithm": "SHA1|SHA256|SHA512",
  "digits": 6,
  "period": 30,
  "issuer": "Example Corp",
  "account": "user@example.com"
}
```

**OTP URL Format:**
```
otpauth://totp/Example:user@example.com?secret=JBSWY3DPEHPK3PXP&issuer=Example&algorithm=SHA1&digits=6&period=30
```

**Generation Algorithm:**
```
TOTP = HOTP(K, T)
where:
  K = shared secret
  T = floor((current_time - T0) / time_step)
  T0 = 0 (Unix epoch)
  time_step = 30 seconds (default)
```

### 2.2 HOTP (HMAC-Based OTP) - RFC 4226

**Parameters:**
```json
{
  "type": "hotp",
  "secret": "BASE32_ENCODED_SECRET",
  "algorithm": "SHA1",
  "digits": 6,
  "counter": 0,
  "issuer": "Example Corp",
  "account": "user@example.com"
}
```

**OTP URL Format:**
```
otpauth://hotp/Example:user@example.com?secret=JBSWY3DPEHPK3PXP&issuer=Example&counter=0&digits=6
```

**Generation Algorithm:**
```
HOTP(K,C) = Truncate(HMAC-SHA-1(K, C))
where:
  K = shared secret
  C = counter value (8-byte integer)
```

### 2.3 SMS OTP Format

**Message Template:**
```
Your [SERVICE] verification code is: [CODE]
This code expires in [MINUTES] minutes.
Do not share this code with anyone.
```

**Data Model:**
```json
{
  "type": "sms_otp",
  "phone": "+1234567890",
  "code": "123456",
  "expires_at": "2025-12-25T12:30:00Z",
  "max_attempts": 3,
  "cooldown": 60
}
```

---

## 3. Authentication Factor Data Models

### 3.1 Password Credential

```json
{
  "type": "password",
  "user_id": "user123",
  "password_hash": "bcrypt_hash",
  "salt": "random_salt",
  "algorithm": "bcrypt",
  "rounds": 12,
  "created_at": "2025-01-01T00:00:00Z",
  "last_changed": "2025-12-25T00:00:00Z",
  "expires_at": "2026-03-25T00:00:00Z",
  "history": ["previous_hash_1", "previous_hash_2"]
}
```

### 3.2 Hardware Token

```json
{
  "type": "hardware_token",
  "user_id": "user123",
  "token_id": "yubikey_serial_12345",
  "protocol": "FIDO2|U2F|OATH",
  "public_key": "BASE64_PUBLIC_KEY",
  "aaguid": "device_guid",
  "counter": 42,
  "enrolled_at": "2025-12-25T00:00:00Z",
  "last_used": "2025-12-25T12:00:00Z",
  "nickname": "My YubiKey"
}
```

### 3.3 Biometric Template

```json
{
  "type": "biometric",
  "user_id": "user123",
  "modality": "fingerprint|face|iris|voice",
  "template_hash": "SHA256_HASH",
  "template_protected": true,
  "encryption": "AES-256-GCM",
  "quality_score": 0.95,
  "enrolled_at": "2025-12-25T00:00:00Z",
  "device_id": "device_123",
  "metadata": {
    "sensor_type": "optical",
    "liveness_detected": true
  }
}
```

### 3.4 Device Fingerprint

```json
{
  "type": "device_fingerprint",
  "user_id": "user123",
  "fingerprint_id": "fp_abc123",
  "user_agent": "Mozilla/5.0...",
  "ip_address": "192.168.1.1",
  "geolocation": {
    "country": "US",
    "region": "California",
    "city": "San Francisco"
  },
  "device_info": {
    "os": "macOS",
    "browser": "Chrome",
    "screen_resolution": "1920x1080",
    "timezone": "America/Los_Angeles"
  },
  "created_at": "2025-12-25T00:00:00Z",
  "trusted": true
}
```

---

## 4. Authentication Session Format

### 4.1 Session Token

```json
{
  "session_id": "sess_abc123",
  "user_id": "user123",
  "created_at": "2025-12-25T12:00:00Z",
  "expires_at": "2025-12-25T14:00:00Z",
  "factors_used": [
    {
      "type": "password",
      "verified_at": "2025-12-25T12:00:00Z"
    },
    {
      "type": "totp",
      "verified_at": "2025-12-25T12:00:05Z"
    }
  ],
  "risk_score": 0.15,
  "device_fingerprint": "fp_abc123",
  "ip_address": "192.168.1.1",
  "metadata": {
    "user_agent": "Mozilla/5.0...",
    "mfa_satisfied": true
  }
}
```

### 4.2 JWT Token Format

```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT"
  },
  "payload": {
    "iss": "https://auth.example.com",
    "sub": "user123",
    "aud": "https://api.example.com",
    "exp": 1735142400,
    "iat": 1735135200,
    "nbf": 1735135200,
    "jti": "jwt_abc123",
    "amr": ["pwd", "otp"],
    "acr": "urn:mace:incommon:iap:silver",
    "azp": "client_id_123",
    "scope": "openid profile email"
  },
  "signature": "BASE64URL_SIGNATURE"
}
```

---

## 5. Enrollment Data Format

### 5.1 TOTP Enrollment Request

```json
{
  "user_id": "user123",
  "factor_type": "totp",
  "secret": "JBSWY3DPEHPK3PXP",
  "algorithm": "SHA256",
  "digits": 6,
  "period": 30,
  "qr_code": "data:image/png;base64,...",
  "backup_codes": [
    "ABC123-DEF456",
    "GHI789-JKL012"
  ]
}
```

### 5.2 Hardware Token Enrollment

```json
{
  "user_id": "user123",
  "factor_type": "hardware_token",
  "protocol": "FIDO2",
  "challenge": "BASE64_CHALLENGE",
  "rp": {
    "name": "Example Corp",
    "id": "example.com"
  },
  "user": {
    "id": "BASE64_USER_ID",
    "name": "user@example.com",
    "displayName": "John Doe"
  },
  "pubKeyCredParams": [
    {"type": "public-key", "alg": -7},
    {"type": "public-key", "alg": -257}
  ],
  "timeout": 60000,
  "attestation": "direct"
}
```

---

## 6. Recovery Data Format

### 6.1 Backup Codes

```json
{
  "user_id": "user123",
  "codes": [
    {
      "code": "ABC123-DEF456",
      "hash": "SHA256_HASH",
      "used": false,
      "used_at": null
    },
    {
      "code": "GHI789-JKL012",
      "hash": "SHA256_HASH",
      "used": true,
      "used_at": "2025-12-20T10:00:00Z"
    }
  ],
  "generated_at": "2025-12-25T00:00:00Z",
  "total_codes": 10,
  "remaining_codes": 9
}
```

### 6.2 Recovery Key

```json
{
  "user_id": "user123",
  "recovery_key": "RECOVERY-KEY-ABC123-DEF456-GHI789",
  "key_hash": "SHA256_HASH",
  "created_at": "2025-12-25T00:00:00Z",
  "expires_at": "2026-12-25T00:00:00Z",
  "max_uses": 1,
  "used_count": 0
}
```

---

## 7. Risk Assessment Data

### 7.1 Risk Score Calculation

```json
{
  "session_id": "sess_abc123",
  "risk_score": 0.75,
  "risk_level": "high",
  "factors": [
    {
      "factor": "unknown_device",
      "weight": 0.3,
      "score": 0.9
    },
    {
      "factor": "unusual_location",
      "weight": 0.25,
      "score": 0.8
    },
    {
      "factor": "unusual_time",
      "weight": 0.15,
      "score": 0.6
    },
    {
      "factor": "velocity_anomaly",
      "weight": 0.2,
      "score": 0.7
    },
    {
      "factor": "behavioral_pattern",
      "weight": 0.1,
      "score": 0.5
    }
  ],
  "recommendation": "require_step_up_auth",
  "calculated_at": "2025-12-25T12:00:00Z"
}
```

---

## 8. Push Notification Format

### 8.1 Push Auth Request

```json
{
  "notification_id": "notif_abc123",
  "user_id": "user123",
  "device_token": "fcm_token_xyz",
  "title": "Login Approval Required",
  "body": "Approve login from Chrome on macOS in San Francisco?",
  "data": {
    "type": "auth_request",
    "session_id": "sess_abc123",
    "ip_address": "192.168.1.1",
    "location": "San Francisco, CA",
    "device": "Chrome on macOS",
    "expires_at": "2025-12-25T12:05:00Z"
  },
  "actions": [
    {"action": "approve", "title": "Approve"},
    {"action": "deny", "title": "Deny"}
  ]
}
```

---

## 9. Data Storage Requirements

### 9.1 Encryption at Rest

- All secrets MUST be encrypted using AES-256-GCM
- Encryption keys MUST be stored in HSM or secure key vault
- Regular key rotation (recommended: every 90 days)

### 9.2 Data Retention

- Active credentials: No expiration
- Session data: 90 days after expiration
- Audit logs: Minimum 1 year
- Backup codes: Until used or regenerated

### 9.3 Privacy Considerations

- Biometric templates MUST NOT store raw biometric data
- PII MUST be encrypted
- User consent required for data collection
- Right to deletion compliance (GDPR Article 17)

---

## 10. Interoperability

### 10.1 Standard Export Format

```json
{
  "version": "1.0",
  "exported_at": "2025-12-25T00:00:00Z",
  "user_id": "user123",
  "factors": [
    {
      "type": "totp",
      "secret": "BASE32_SECRET",
      "algorithm": "SHA256",
      "digits": 6,
      "period": 30
    }
  ],
  "backup_codes": ["CODE1", "CODE2"]
}
```

### 10.2 Import Validation

- Verify data schema compliance
- Validate cryptographic parameters
- Check for duplicate entries
- Sanitize user inputs

---

## 11. Audit Log Schema

Every authentication decision MUST be recorded in a tamper-evident log:

```json
{
  "eventId": "01HG…",
  "ts": "2026-04-26T13:00:00Z",
  "subjectId": "u-12345",
  "tenantId": "acme",
  "factorTypes": ["webauthn-platform"],
  "outcome": "success",
  "ip": "203.0.113.42",
  "userAgent": "Mozilla/5.0 …",
  "geoCity": "Seoul",
  "geoCountry": "KR",
  "riskScore": 0.12,
  "stepUpReasons": [],
  "policyVersion": "2026-04-21",
  "previousEntryHash": "sha256:…"
}
```

Logs MUST be append-only; mutation requires creating a new event whose `previousEntryHash` chains to the entry being superseded. Storage MUST be object-locked for at least 13 months.

## 12. References

- [RFC 6238 - TOTP](https://datatracker.ietf.org/doc/html/rfc6238)
- [RFC 4226 - HOTP](https://datatracker.ietf.org/doc/html/rfc4226)
- [FIDO2 CTAP](https://fidoalliance.org/specs/fido-v2.0-ps-20190130/fido-client-to-authenticator-protocol-v2.0-ps-20190130.html)
- [JWT RFC 7519](https://datatracker.ietf.org/doc/html/rfc7519)
- [WebAuthn Level 3 (W3C)](https://www.w3.org/TR/webauthn-3/)
- [RFC 9457 — Problem Details for HTTP APIs](https://datatracker.ietf.org/doc/html/rfc9457)
- [NIST SP 800-63B](https://pages.nist.gov/800-63-3/sp800-63b.html)

---

**Copyright © 2025 SmileStory Inc. / WIA**
**弘益人間 (홍익인간) · Benefit All Humanity**
