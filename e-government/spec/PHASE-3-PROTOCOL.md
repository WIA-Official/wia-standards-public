# WIA-SOC-003 Phase 3: Security Protocol Specification

**Version:** 1.0.0
**Status:** Approved
**Last Updated:** 2025-12-26

弘益人間 (Hongik Ingan) - Benefit All Humanity

---

## 1. Overview

Phase 3 defines comprehensive security protocols for e-government systems, including authentication mechanisms, encryption standards, access control, audit logging, and compliance requirements.

## 2. Authentication Protocols

### 2.1 Multi-Factor Authentication (MFA)

#### 2.1.1 Authentication Levels

| Level | Factors Required | Use Cases |
|-------|-----------------|-----------|
| Low | 1 factor (password) | Public information access |
| Medium | 2 factors (password + OTP) | Service requests, profile updates |
| High | 3+ factors (password + biometric + token) | Financial transactions, identity changes |

#### 2.1.2 Biometric Authentication

**Supported Methods**:
- **Face Recognition**: Using FaceID, facial landmarks (FIDO2 compliant)
- **Fingerprint**: Touch ID, in-display sensors
- **Iris Scan**: High-security applications
- **Voice Recognition**: Phone-based authentication

**Security Requirements**:
```json
{
  "biometric": {
    "storage": "secure_enclave",
    "comparison": "on_device",
    "templates": "encrypted_hash_only",
    "liveness": "required",
    "falseAcceptRate": "< 1:100000",
    "falseRejectRate": "< 1:100"
  }
}
```

### 2.2 JWT Token Structure

```json
{
  "header": {
    "alg": "RS256",
    "typ": "JWT",
    "kid": "gov-key-2025-01"
  },
  "payload": {
    "iss": "https://auth.egov.kr",
    "sub": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
    "aud": "https://api.egov.kr",
    "exp": 1640558400,
    "iat": 1640554800,
    "nbf": 1640554800,
    "jti": "token-unique-id",
    "scope": ["service.read", "service.write", "document.read"],
    "verification_level": "high",
    "country": "KR"
  },
  "signature": "RS256_signature_data"
}
```

### 2.3 OAuth 2.0 / OpenID Connect

**Authorization Flow**:
```
1. Citizen → Authorization Request → Government Auth Server
2. Citizen authenticates (MFA)
3. Government Auth Server → Authorization Code → Citizen
4. Citizen → Authorization Code → Service Provider
5. Service Provider → Access Token Request → Government Auth Server
6. Government Auth Server → Access Token + ID Token → Service Provider
```

**Token Exchange**:
```http
POST /oauth/token
Content-Type: application/x-www-form-urlencoded

grant_type=authorization_code
&code=AUTH_CODE_HERE
&redirect_uri=https://service.example.com/callback
&client_id=CLIENT_ID
&client_secret=CLIENT_SECRET
&code_verifier=PKCE_VERIFIER
```

## 3. Encryption Standards

### 3.1 Data at Rest

**Algorithm**: AES-256-GCM

**Key Management**:
```json
{
  "keyManagement": {
    "storage": "HSM (Hardware Security Module)",
    "algorithm": "RSA-4096 (key encryption key)",
    "rotation": "90 days",
    "backup": "air-gapped secure facility",
    "recovery": "M-of-N secret sharing (3-of-5)"
  },
  "dataKeys": {
    "algorithm": "AES-256-GCM",
    "derivation": "HKDF-SHA256",
    "uniquePerRecord": true,
    "iv": "random 96-bit nonce"
  }
}
```

**Implementation Example**:
```python
from cryptography.hazmat.primitives.ciphers.aead import AESGCM
import os

def encrypt_citizen_data(data: bytes, master_key: bytes) -> dict:
    # Generate unique data key
    data_key = AESGCM.generate_key(bit_length=256)
    nonce = os.urandom(12)

    # Encrypt data
    aesgcm = AESGCM(data_key)
    ciphertext = aesgcm.encrypt(nonce, data, None)

    # Encrypt data key with master key
    master_aesgcm = AESGCM(master_key)
    master_nonce = os.urandom(12)
    encrypted_key = master_aesgcm.encrypt(master_nonce, data_key, None)

    return {
        "ciphertext": ciphertext,
        "nonce": nonce,
        "encrypted_key": encrypted_key,
        "master_nonce": master_nonce,
        "algorithm": "AES-256-GCM"
    }
```

### 3.2 Data in Transit

**Protocol**: TLS 1.3

**Configuration**:
```nginx
ssl_protocols TLSv1.3;
ssl_ciphers 'ECDHE-RSA-AES256-GCM-SHA384:ECDHE-RSA-AES128-GCM-SHA256';
ssl_prefer_server_ciphers on;
ssl_ecdh_curve secp384r1;
ssl_session_cache shared:SSL:10m;
ssl_session_timeout 10m;
ssl_stapling on;
ssl_stapling_verify on;
ssl_certificate /path/to/cert.pem;
ssl_certificate_key /path/to/key.pem;
ssl_trusted_certificate /path/to/chain.pem;
```

**Certificate Requirements**:
- **Type**: X.509 v3
- **Key Size**: RSA 4096-bit or ECC P-384
- **Signature**: SHA-384 or higher
- **Validity**: Maximum 398 days
- **Revocation**: OCSP Must-Staple
- **Transparency**: Certificate Transparency (CT) logs

### 3.3 End-to-End Encryption (E2EE)

For highly sensitive communications:

```json
{
  "e2ee": {
    "algorithm": "X25519 (key exchange) + ChaCha20-Poly1305 (encryption)",
    "perfectForwardSecrecy": true,
    "keyRotation": "per session",
    "metadata": "encrypted",
    "verifiability": "Signal Protocol double ratchet"
  }
}
```

## 4. Access Control

### 4.1 Role-Based Access Control (RBAC)

**Role Hierarchy**:
```json
{
  "roles": {
    "citizen": {
      "permissions": ["read:own_profile", "write:own_profile", "submit:service_request"]
    },
    "officer": {
      "inherits": ["citizen"],
      "permissions": ["read:requests", "update:request_status", "approve:documents"]
    },
    "supervisor": {
      "inherits": ["officer"],
      "permissions": ["approve:requests", "access:analytics", "manage:officers"]
    },
    "admin": {
      "inherits": ["supervisor"],
      "permissions": ["manage:system", "access:audit_logs", "configure:services"]
    },
    "data_protection_officer": {
      "permissions": ["access:all_audit_logs", "enforce:privacy", "delete:citizen_data"]
    }
  }
}
```

### 4.2 Attribute-Based Access Control (ABAC)

**Policy Example**:
```json
{
  "policy": {
    "id": "tax-filing-access",
    "effect": "allow",
    "principal": {
      "type": "citizen",
      "attributes": {
        "age": {"$gte": 19},
        "citizenship": "KR",
        "verification_level": {"$in": ["medium", "high"]}
      }
    },
    "action": "submit:tax_return",
    "resource": {
      "type": "service",
      "id": "SRV-TAX-001"
    },
    "conditions": {
      "time": {"$between": ["2025-01-01", "2025-12-31"]},
      "ip": {"$in": ["KR", "JP", "US"]},
      "mfa": {"$eq": true}
    }
  }
}
```

### 4.3 Zero Trust Architecture

**Principles**:
1. **Never Trust, Always Verify**: Every request authenticated and authorized
2. **Least Privilege**: Minimum permissions required
3. **Micro-Segmentation**: Network segmented by service
4. **Continuous Monitoring**: Real-time threat detection

**Implementation**:
```yaml
zero_trust:
  identity_verification:
    - device_posture_check
    - geo_location_validation
    - behavior_analytics
    - risk_scoring

  network_segmentation:
    - service_mesh (Istio)
    - mutual_TLS (mTLS)
    - network_policies (Kubernetes)

  continuous_monitoring:
    - SIEM integration
    - anomaly_detection (ML-based)
    - threat_intelligence_feeds
```

## 5. Audit Logging

### 5.1 Audit Event Structure

```json
{
  "@type": "AuditEvent",
  "eventId": "EVT-2025-123456",
  "timestamp": "2025-12-26T14:30:00.123Z",
  "severity": "INFO",
  "category": "data_access",
  "actor": {
    "type": "citizen",
    "id": "urn:uuid:550e8400-e29b-41d4-a716-446655440000",
    "ip": "203.0.113.42",
    "location": {"country": "KR", "region": "Seoul"},
    "device": {
      "type": "mobile",
      "os": "iOS 17.0",
      "app": "WIA-Gov-App/1.2.3"
    }
  },
  "action": "read",
  "resource": {
    "type": "document",
    "id": "DOC-2025-5678",
    "classification": "personal"
  },
  "result": {
    "status": "success",
    "code": 200
  },
  "context": {
    "requestId": "550e8400-e29b-41d4-a716-446655440000",
    "sessionId": "SESSION-2025-7890",
    "permissions": ["document.read"]
  },
  "compliance": {
    "gdpr": true,
    "retention": "P10Y",
    "legalBasis": "legitimate_interest"
  }
}
```

### 5.2 Immutable Audit Trail

**Blockchain Storage**:
```json
{
  "blockchain": {
    "network": "government-permissioned-blockchain",
    "consensus": "PBFT (Practical Byzantine Fault Tolerance)",
    "blockTime": "30 seconds",
    "storage": {
      "eventHash": "sha256 hash of audit event",
      "merkleRoot": "merkle root of event batch",
      "previousBlockHash": "hash chain for integrity"
    },
    "verification": {
      "publicVerification": true,
      "verificationUrl": "https://audit.egov.kr/verify/{eventId}"
    }
  }
}
```

### 5.3 Sensitive Action Logging

**High-Risk Actions** (require enhanced logging):
- Identity changes
- Permission modifications
- Data exports
- Cross-border data transfers
- Administrative actions

```json
{
  "sensitiveAction": {
    "type": "identity_change",
    "before": {"name": "Kim MinJun", "address": "Seoul, Gangnam-gu"},
    "after": {"name": "Kim MinJun", "address": "Seoul, Songpa-gu"},
    "approvals": [
      {
        "approver": "officer-001",
        "timestamp": "2025-12-26T14:00:00Z",
        "verification": "biometric"
      }
    ],
    "notification": {
      "channels": ["email", "sms"],
      "sentTo": ["citizen", "data_protection_officer"]
    }
  }
}
```

## 6. Security Monitoring

### 6.1 Threat Detection

**Monitored Indicators**:
```yaml
threat_detection:
  authentication_anomalies:
    - failed_login_attempts: "> 5 in 15 minutes"
    - unusual_location: "login from new country"
    - impossible_travel: "login from distant locations within short time"

  data_access_anomalies:
    - bulk_downloads: "> 100 documents in 1 hour"
    - after_hours_access: "access between 22:00 - 06:00"
    - unusual_permissions: "access to resources outside normal pattern"

  network_anomalies:
    - port_scanning: "systematic port probing"
    - ddos_indicators: "traffic spike > 1000% normal"
    - sql_injection: "malicious query patterns"
```

### 6.2 Incident Response

**Automated Response**:
```json
{
  "incident_response": {
    "level_1_minor": {
      "actions": ["log", "alert_soc"],
      "examples": ["single_failed_login", "rate_limit_warning"]
    },
    "level_2_moderate": {
      "actions": ["log", "alert_soc", "require_mfa"],
      "examples": ["multiple_failed_logins", "unusual_location"]
    },
    "level_3_major": {
      "actions": ["log", "alert_soc", "alert_dpo", "temporary_account_lock"],
      "examples": ["suspected_account_takeover", "bulk_data_access"]
    },
    "level_4_critical": {
      "actions": ["log", "alert_all", "account_lock", "session_termination", "initiate_investigation"],
      "examples": ["confirmed_breach", "privilege_escalation"]
    }
  }
}
```

## 7. Compliance and Privacy

### 7.1 GDPR Compliance

**Data Subject Rights**:
```json
{
  "gdpr_rights": {
    "right_to_access": {
      "endpoint": "GET /citizen/{id}/data",
      "format": "JSON, PDF, CSV",
      "response_time": "30 days"
    },
    "right_to_rectification": {
      "endpoint": "PUT /citizen/{id}",
      "verification": "high"
    },
    "right_to_erasure": {
      "endpoint": "DELETE /citizen/{id}",
      "verification": "high",
      "retention_override": ["legal_obligation", "public_interest"],
      "response_time": "30 days"
    },
    "right_to_portability": {
      "endpoint": "GET /citizen/{id}/export",
      "format": "JSON-LD, XML",
      "response_time": "30 days"
    },
    "right_to_object": {
      "endpoint": "POST /citizen/{id}/object",
      "processing_halt": "immediate"
    }
  }
}
```

### 7.2 Data Minimization

**Principles**:
```yaml
data_minimization:
  collection:
    - collect_only_necessary_fields
    - justify_each_data_point
    - regular_review_of_requirements

  storage:
    - pseudonymization_where_possible
    - anonymization_for_analytics
    - encryption_of_pii

  retention:
    - automatic_deletion_after_period
    - retention_policy_per_data_type
    - legal_hold_exceptions
```

### 7.3 Privacy Impact Assessment (PIA)

**Required for**:
- New services
- Major system changes
- Cross-border data transfers
- High-risk processing

**Assessment Framework**:
```json
{
  "pia": {
    "data_flows": "map all personal data flows",
    "risk_assessment": "identify privacy risks",
    "mitigation": "propose safeguards",
    "stakeholder_input": "consult citizens and DPO",
    "approval": "sign-off by DPO and legal",
    "review": "annual review required"
  }
}
```

## 8. Penetration Testing

### 8.1 Testing Schedule

| Scope | Frequency | Method |
|-------|-----------|--------|
| Full System | Annually | External vendor |
| Critical Services | Quarterly | Internal + External |
| New Features | Pre-release | Internal |
| Bug Bounty | Continuous | Public program |

### 8.2 Testing Requirements

```yaml
penetration_testing:
  methodology:
    - OWASP_Top_10
    - SANS_Top_25
    - CWE_Top_25

  scope:
    - api_endpoints
    - web_applications
    - mobile_apps
    - infrastructure
    - social_engineering (limited)

  reporting:
    - severity_classification (Critical/High/Medium/Low)
    - proof_of_concept
    - remediation_recommendations
    - retest_after_fixes
```

---

**弘益人間 · Benefit All Humanity**

© 2025 WIA / SmileStory Inc.
