# WIA Security Communication Protocol Standard

**Version**: 1.0.0
**Status**: Draft
**Phase**: 3 of 4

---

## 1. 개요 (Overview)

WIA Security Communication Protocol은 보안 시스템 간의 표준화된 통신 방법을 정의합니다. 본 프로토콜은 취약점 스캐너, SIEM, 위협 인텔리전스 플랫폼, Zero Trust 정책 엔진 등 다양한 보안 컴포넌트 간의 안전하고 효율적인 데이터 교환을 보장합니다.

### 1.1 설계 원칙

- **보안 우선**: TLS 1.3, mTLS, Post-Quantum Cryptography 지원
- **상호운용성**: TAXII 2.1, STIX 2.1 호환
- **확장성**: 플러그인 가능한 프로토콜 어댑터
- **실시간성**: WebSocket 기반 이벤트 스트리밍
- **신뢰성**: 메시지 서명 및 무결성 검증

### 1.2 범위

- 메시지 교환 프로토콜 (Request/Response, Event Stream)
- 암호화 및 인증 메커니즘
- Zero Trust 네트워크 프로토콜
- SIEM 연동 프로토콜
- 위협 인텔리전스 공유 (TAXII 2.1 호환)
- 실시간 알림 스트리밍

---

## 2. 프로토콜 계층 (Protocol Layers)

### 2.1 계층 구조

```
┌─────────────────────────────────────┐
│   Application Protocol              │
│   (WIA Security Message Exchange)   │
├─────────────────────────────────────┤
│   Message Format Layer              │
│   (JSON / Protocol Buffers)         │
├─────────────────────────────────────┤
│   Security Layer                    │
│   (TLS 1.3, mTLS, PQC Hybrid)       │
├─────────────────────────────────────┤
│   Transport Layer                   │
│   (HTTPS, WebSocket, gRPC)          │
└─────────────────────────────────────┘
```

### 2.2 Transport Layer

지원되는 전송 프로토콜:

| 프로토콜 | 용도 | 포트 |
|---------|------|------|
| HTTPS | REST API, Request/Response | 443 |
| WebSocket | 실시간 이벤트 스트리밍 | 443 |
| gRPC | 고성능 서비스 간 통신 | 50051 |

---

## 3. 메시지 형식 (Message Format)

### 3.1 기본 메시지 Envelope

모든 WIA Security 프로토콜 메시지는 다음 기본 구조를 따릅니다:

```json
{
  "$schema": "https://wia.live/schemas/security/message/v1.schema.json",
  "protocol_version": "1.0.0",
  "message_id": "uuid-v4",
  "timestamp": "ISO 8601 with timezone",
  "message_type": "request | response | event | notification",
  "sender": {
    "id": "sender-component-uuid",
    "type": "scanner | siem | pdp | pep | threat_intel | endpoint",
    "name": "Component Display Name",
    "certificate_fingerprint": "SHA-256 of TLS certificate"
  },
  "receiver": {
    "id": "receiver-component-uuid",
    "type": "component-type",
    "name": "Component Display Name"
  },
  "security": {
    "encryption": "TLS1.3 | PQC_HYBRID",
    "signature_algorithm": "Ed25519 | Dilithium3",
    "signature": "base64-encoded-signature",
    "nonce": "unique-nonce-for-replay-protection"
  },
  "payload": {
    "content_type": "application/json | application/stix+json",
    "encoding": "utf-8",
    "data": {}
  },
  "metadata": {
    "correlation_id": "for-request-response-pairing",
    "causation_id": "parent-message-id",
    "priority": "critical | high | medium | low",
    "ttl_seconds": 3600,
    "trace_id": "distributed-tracing-id"
  }
}
```

### 3.2 필드 정의

| 필드 | 타입 | 필수 | 설명 |
|-----|------|------|------|
| `$schema` | string | 권장 | 스키마 URL |
| `protocol_version` | string | 필수 | 프로토콜 버전 (SemVer) |
| `message_id` | UUID | 필수 | 메시지 고유 식별자 |
| `timestamp` | ISO 8601 | 필수 | 메시지 생성 시간 |
| `message_type` | enum | 필수 | 메시지 유형 |
| `sender` | object | 필수 | 송신자 정보 |
| `receiver` | object | 필수 | 수신자 정보 |
| `security` | object | 필수 | 보안 정보 |
| `payload` | object | 필수 | 메시지 페이로드 |
| `metadata` | object | 선택 | 메타데이터 |

### 3.3 메시지 유형

#### 3.3.1 Request (요청)

```json
{
  "message_type": "request",
  "payload": {
    "content_type": "application/json",
    "data": {
      "request_type": "scan_start | access_decision | threat_query | ...",
      "parameters": {}
    }
  }
}
```

#### 3.3.2 Response (응답)

```json
{
  "message_type": "response",
  "payload": {
    "data": {
      "status": "success | error | partial",
      "result": {},
      "error": {
        "code": "ERROR_CODE",
        "message": "Human readable message",
        "details": {}
      }
    }
  },
  "metadata": {
    "correlation_id": "original-request-message-id",
    "processing_time_ms": 45
  }
}
```

#### 3.3.3 Event (이벤트)

```json
{
  "message_type": "event",
  "payload": {
    "data": {
      "event_type": "vulnerability_found | threat_detected | policy_violation | ...",
      "event_data": {}
    }
  },
  "metadata": {
    "priority": "critical",
    "requires_acknowledgment": true
  }
}
```

#### 3.3.4 Notification (알림)

```json
{
  "message_type": "notification",
  "payload": {
    "data": {
      "notification_type": "system_status | heartbeat | config_change",
      "notification_data": {}
    }
  }
}
```

---

## 4. 보안 및 인증 (Security & Authentication)

### 4.1 TLS 1.3 구성

```yaml
tls_config:
  min_version: "1.3"
  max_version: "1.3"
  cipher_suites:
    - TLS_AES_256_GCM_SHA384
    - TLS_CHACHA20_POLY1305_SHA256
    - TLS_AES_128_GCM_SHA256

  # Deprecated ciphers - NEVER use
  disabled_ciphers:
    - TLS_RSA_*
    - TLS_DH_*
    - TLS_ECDH_*
```

### 4.2 상호 TLS 인증 (mTLS)

```yaml
mtls_config:
  mode: "strict"  # strict | optional | off
  client_auth_required: true

  ca_certificates:
    - path: "/etc/wia-security/certs/ca.pem"
      purpose: "client_ca"

  server_certificate:
    cert_path: "/etc/wia-security/certs/server.pem"
    key_path: "/etc/wia-security/certs/server-key.pem"

  client_certificate:
    cert_path: "/etc/wia-security/certs/client.pem"
    key_path: "/etc/wia-security/certs/client-key.pem"

  certificate_validation:
    verify_hostname: true
    verify_expiry: true
    check_revocation: true  # OCSP or CRL
    allowed_organizations:
      - "WIA Security Partners"
      - "Trusted SOC Providers"
```

### 4.3 Post-Quantum Hybrid 암호화

양자 내성 암호화를 위한 하이브리드 모드 지원:

```yaml
pqc_hybrid_config:
  enabled: true
  kem_algorithm: "Kyber768"  # NIST PQC standard
  classical_fallback: "X25519"
  signature_algorithm: "Dilithium3"  # For message signing
```

### 4.4 JWT 토큰 인증

서비스 간 인증을 위한 JWT 토큰:

```json
{
  "header": {
    "alg": "EdDSA",
    "typ": "JWT",
    "kid": "key-id-001"
  },
  "payload": {
    "iss": "https://auth.wia.live",
    "sub": "component-id",
    "aud": "wia-security-api",
    "exp": 1702656000,
    "iat": 1702569600,
    "jti": "unique-token-id",
    "scope": [
      "read:alerts",
      "write:events",
      "admin:policies"
    ],
    "component": {
      "id": "scanner-001",
      "type": "vulnerability_scanner",
      "organization": "SOC-Central"
    }
  }
}
```

### 4.5 메시지 서명

모든 메시지는 송신자의 개인키로 서명됩니다:

```python
# Signature creation
message_bytes = canonical_json(message_without_signature)
signature = sign_ed25519(private_key, message_bytes)
message["security"]["signature"] = base64_encode(signature)

# Signature verification
message_bytes = canonical_json(message_without_signature)
is_valid = verify_ed25519(public_key, message_bytes, signature)
```

---

## 5. Zero Trust 프로토콜 (Zero Trust Protocol)

### 5.1 개요

WIA Security Zero Trust 프로토콜은 NIST SP 800-207 가이드라인을 따르며, 지속적인 인증과 최소 권한 원칙을 구현합니다.

### 5.2 구성 요소

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Subject   │────▶│     PEP     │────▶│  Resource   │
│  (User/App) │     │  (Enforce)  │     │  (Data/Svc) │
└─────────────┘     └──────┬──────┘     └─────────────┘
                          │
                          ▼
                   ┌─────────────┐
                   │     PDP     │
                   │  (Decide)   │
                   └──────┬──────┘
                          │
          ┌───────────────┼───────────────┐
          ▼               ▼               ▼
    ┌──────────┐   ┌──────────┐   ┌──────────┐
    │ Identity │   │ Context  │   │ Policy   │
    │  Store   │   │  Engine  │   │  Store   │
    └──────────┘   └──────────┘   └──────────┘
```

### 5.3 접근 결정 요청 (Access Decision Request)

```json
{
  "message_type": "request",
  "sender": {
    "id": "pep-gateway-001",
    "type": "pep"
  },
  "receiver": {
    "id": "pdp-central",
    "type": "pdp"
  },
  "payload": {
    "data": {
      "request_type": "access_decision",
      "subject": {
        "user_id": "user@example.com",
        "device_id": "device-uuid",
        "device_posture": {
          "os_version": "macOS 14.1",
          "patch_level": "2024-12-01",
          "antivirus_status": "up_to_date",
          "disk_encryption": true,
          "firewall_enabled": true,
          "trust_score": 0.92
        },
        "location": {
          "ip": "203.0.113.45",
          "country": "KR",
          "city": "Seoul",
          "network_type": "corporate_vpn"
        },
        "authentication": {
          "method": "mfa",
          "factors": ["password", "totp", "biometric"],
          "auth_time": "2024-12-14T10:30:00Z",
          "session_id": "session-uuid"
        },
        "risk_signals": {
          "impossible_travel": false,
          "unusual_time": false,
          "unusual_device": false,
          "compromised_credential": false
        }
      },
      "resource": {
        "type": "database",
        "id": "customer-db-prod",
        "classification": "confidential",
        "environment": "production",
        "location": "aws-ap-northeast-2"
      },
      "action": "read",
      "context": {
        "request_time": "2024-12-14T10:35:00Z",
        "day_type": "business_day",
        "business_justification": "Customer support ticket #12345"
      }
    }
  }
}
```

### 5.4 접근 결정 응답 (Access Decision Response)

```json
{
  "message_type": "response",
  "payload": {
    "data": {
      "decision": "permit",
      "decision_id": "decision-uuid",
      "conditions": [
        {
          "type": "time_limit",
          "constraint": "session_expires_at",
          "value": "2024-12-14T18:35:00Z"
        },
        {
          "type": "continuous_auth",
          "constraint": "reauth_interval_minutes",
          "value": 30
        },
        {
          "type": "data_limit",
          "constraint": "max_records_per_query",
          "value": 1000
        }
      ],
      "obligations": [
        {
          "type": "logging",
          "action": "log_all_queries",
          "target": "siem-central",
          "parameters": {
            "detail_level": "verbose",
            "include_results": false
          }
        },
        {
          "type": "alerting",
          "action": "alert_on_anomaly",
          "condition": "query_count > 100 in 5 minutes",
          "target": "soc-team"
        },
        {
          "type": "masking",
          "action": "mask_pii",
          "fields": ["ssn", "credit_card", "phone"]
        }
      ],
      "valid_from": "2024-12-14T10:35:00Z",
      "valid_until": "2024-12-14T18:35:00Z",
      "session_token": "encrypted-access-token"
    }
  },
  "metadata": {
    "correlation_id": "original-request-id",
    "decision_time_ms": 12
  }
}
```

### 5.5 정책 정의 형식 (Policy Definition)

```json
{
  "policy_id": "policy-001",
  "name": "Production Database Access",
  "version": "1.0.0",
  "effect": "permit",
  "subjects": {
    "match": "any",
    "conditions": [
      {"attribute": "role", "operator": "in", "value": ["dba", "sre", "support"]},
      {"attribute": "mfa_verified", "operator": "eq", "value": true}
    ]
  },
  "resources": {
    "match": "all",
    "conditions": [
      {"attribute": "type", "operator": "eq", "value": "database"},
      {"attribute": "environment", "operator": "eq", "value": "production"}
    ]
  },
  "actions": ["read", "write"],
  "conditions": {
    "match": "all",
    "rules": [
      {"attribute": "time_of_day", "operator": "between", "value": ["09:00", "18:00"]},
      {"attribute": "day_type", "operator": "eq", "value": "business_day"},
      {"attribute": "device_trust_score", "operator": "gte", "value": 0.8},
      {"attribute": "network_type", "operator": "in", "value": ["corporate", "vpn"]}
    ]
  },
  "obligations": [
    {"type": "logging", "level": "verbose"},
    {"type": "masking", "fields": ["pii"]}
  ],
  "priority": 100,
  "enabled": true
}
```

---

## 6. SIEM 연동 프로토콜 (SIEM Integration)

### 6.1 이벤트 전송 흐름

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   Scanner   │────▶│ WIA Gateway │────▶│    SIEM     │
│   / EDR     │     │  (Adapter)  │     │  (Splunk,   │
│   / IDS     │     │             │     │   Elastic)  │
└─────────────┘     └─────────────┘     └─────────────┘
       │                   │                   │
       │    WIA Format     │   Native Format   │
       └───────────────────┴───────────────────┘
```

### 6.2 이벤트 수집 등록 (Event Subscription)

```json
{
  "message_type": "request",
  "payload": {
    "data": {
      "request_type": "subscribe",
      "subscription": {
        "id": "sub-001",
        "name": "Critical Alerts to SIEM",
        "source_types": ["scanner", "edr", "ids", "firewall"],
        "event_types": ["alert", "incident", "threat_intel"],
        "filters": {
          "severity": {"gte": 7},
          "priority": ["critical", "high"]
        },
        "delivery": {
          "method": "webhook",
          "endpoint": "https://siem.example.com/api/v1/events",
          "format": "wia_native",
          "batch_size": 100,
          "flush_interval_seconds": 5
        },
        "retry_policy": {
          "max_retries": 3,
          "backoff_type": "exponential",
          "initial_delay_ms": 1000
        }
      }
    }
  }
}
```

### 6.3 SIEM 형식 변환 (Format Conversion)

#### WIA → Splunk HEC

```json
{
  "time": 1702556100,
  "host": "scanner-001",
  "source": "wia-security",
  "sourcetype": "wia:alert",
  "index": "security",
  "event": {
    "wia_event": { /* Original WIA event */ }
  }
}
```

#### WIA → Elastic ECS

```json
{
  "@timestamp": "2024-12-14T10:35:00.000Z",
  "ecs": {"version": "8.0.0"},
  "event": {
    "kind": "alert",
    "category": ["intrusion_detection"],
    "type": ["info"],
    "severity": 7
  },
  "wia": {
    "alert_id": "ALERT-001",
    "original": { /* Original WIA event */ }
  }
}
```

---

## 7. TAXII 2.1 호환 프로토콜 (TAXII 2.1 Compatible)

### 7.1 Discovery

#### Request
```
GET /taxii2/
Accept: application/taxii+json;version=2.1
Authorization: Bearer <token>
```

#### Response
```json
{
  "title": "WIA Security Threat Intelligence Platform",
  "description": "TAXII 2.1 compatible threat intelligence sharing",
  "contact": "security@wia.live",
  "api_roots": [
    "https://threat-intel.wia.live/taxii2/api/v1/"
  ],
  "default": "https://threat-intel.wia.live/taxii2/api/v1/"
}
```

### 7.2 Collections

#### Request
```
GET /taxii2/api/v1/collections/
```

#### Response
```json
{
  "collections": [
    {
      "id": "collection-uuid",
      "title": "WIA Threat Indicators",
      "description": "Real-time threat indicators from WIA Security network",
      "alias": "wia-indicators",
      "can_read": true,
      "can_write": false,
      "media_types": [
        "application/stix+json;version=2.1"
      ]
    }
  ]
}
```

### 7.3 Objects (STIX Bundle)

#### Request
```
GET /taxii2/api/v1/collections/{collection-id}/objects/
Accept: application/stix+json;version=2.1
```

#### Response
```json
{
  "more": false,
  "objects": [
    {
      "type": "indicator",
      "spec_version": "2.1",
      "id": "indicator--uuid",
      "created": "2024-12-14T10:00:00.000Z",
      "modified": "2024-12-14T10:00:00.000Z",
      "name": "Malicious IP - APT29 C2",
      "description": "Command and control server for APT29 campaign",
      "pattern": "[ipv4-addr:value = '185.220.101.45']",
      "pattern_type": "stix",
      "valid_from": "2024-12-14T10:00:00.000Z",
      "valid_until": "2025-01-14T10:00:00.000Z",
      "kill_chain_phases": [
        {
          "kill_chain_name": "mitre-attack",
          "phase_name": "command-and-control"
        }
      ],
      "indicator_types": ["malicious-activity"],
      "confidence": 85
    }
  ]
}
```

---

## 8. 실시간 스트리밍 프로토콜 (Real-Time Streaming)

### 8.1 WebSocket 연결

#### Connection URL
```
wss://api.wia.live/v1/stream
```

#### Headers
```
Authorization: Bearer <token>
X-WIA-Client-ID: <client-uuid>
X-WIA-Protocol-Version: 1.0.0
```

### 8.2 구독 요청

```json
{
  "action": "subscribe",
  "subscriptions": [
    {
      "channel": "alerts",
      "filters": {
        "severity": ["critical", "high"],
        "categories": ["malware", "intrusion"]
      }
    },
    {
      "channel": "threat_intel",
      "filters": {
        "ioc_types": ["ip", "domain", "hash"]
      }
    },
    {
      "channel": "policy_decisions",
      "filters": {
        "decision": ["deny"]
      }
    }
  ]
}
```

### 8.3 구독 확인

```json
{
  "action": "subscribed",
  "subscription_id": "sub-uuid",
  "channels": ["alerts", "threat_intel", "policy_decisions"],
  "connected_at": "2024-12-14T10:00:00Z"
}
```

### 8.4 실시간 이벤트

```json
{
  "action": "event",
  "channel": "alerts",
  "sequence": 12345,
  "timestamp": "2024-12-14T10:35:00.000Z",
  "event": {
    "type": "alert",
    "severity": 9,
    "title": "Ransomware Activity Detected",
    "data": { /* WIA Security Alert format */ }
  }
}
```

### 8.5 Heartbeat

```json
{
  "action": "heartbeat",
  "timestamp": "2024-12-14T10:35:00.000Z",
  "server_time": "2024-12-14T10:35:00.123Z"
}
```

### 8.6 연결 종료

```json
{
  "action": "close",
  "reason": "client_disconnect | server_maintenance | auth_expired",
  "message": "Connection closed gracefully"
}
```

---

## 9. 에러 처리 (Error Handling)

### 9.1 에러 코드

| 코드 | 이름 | 설명 |
|------|------|------|
| E1001 | INVALID_MESSAGE | 메시지 형식 오류 |
| E1002 | INVALID_SIGNATURE | 서명 검증 실패 |
| E1003 | EXPIRED_MESSAGE | 메시지 TTL 초과 |
| E1004 | REPLAY_DETECTED | 재전송 공격 탐지 |
| E2001 | AUTH_REQUIRED | 인증 필요 |
| E2002 | AUTH_FAILED | 인증 실패 |
| E2003 | TOKEN_EXPIRED | 토큰 만료 |
| E2004 | INSUFFICIENT_SCOPE | 권한 부족 |
| E3001 | RESOURCE_NOT_FOUND | 리소스 없음 |
| E3002 | RATE_LIMITED | 요청 제한 초과 |
| E3003 | SERVICE_UNAVAILABLE | 서비스 불가 |
| E4001 | POLICY_DENIED | 정책에 의해 거부 |
| E5001 | INTERNAL_ERROR | 내부 오류 |

### 9.2 에러 응답 형식

```json
{
  "message_type": "response",
  "payload": {
    "data": {
      "status": "error",
      "error": {
        "code": "E2002",
        "name": "AUTH_FAILED",
        "message": "Certificate verification failed",
        "details": {
          "reason": "certificate_expired",
          "not_after": "2024-12-01T00:00:00Z"
        },
        "retry_after": null,
        "help_url": "https://docs.wia.live/security/errors/E2002"
      }
    }
  }
}
```

---

## 10. Rate Limiting

### 10.1 기본 제한

| 엔드포인트 | 제한 | 윈도우 |
|-----------|------|--------|
| REST API | 1000 req | 1분 |
| WebSocket 메시지 | 100 msg | 1초 |
| TAXII Objects | 10000 obj | 1시간 |
| Bulk Upload | 100 MB | 1시간 |

### 10.2 Rate Limit 헤더

```
X-RateLimit-Limit: 1000
X-RateLimit-Remaining: 950
X-RateLimit-Reset: 1702556160
X-RateLimit-Window: 60
```

---

## 11. 버전 관리 및 호환성 (Versioning)

### 11.1 프로토콜 버전

- **Major**: 호환성이 깨지는 변경
- **Minor**: 새로운 기능 추가 (하위 호환)
- **Patch**: 버그 수정

### 11.2 버전 협상

```
Accept: application/wia-security+json;version=1.0
Content-Type: application/wia-security+json;version=1.0
X-WIA-Protocol-Version: 1.0.0
```

---

## 12. 참고문헌 (References)

### 프로토콜 표준
- RFC 8446: TLS 1.3
- RFC 8705: mTLS
- RFC 7519: JWT
- RFC 6455: WebSocket

### 보안 표준
- NIST SP 800-207: Zero Trust Architecture
- OASIS TAXII 2.1
- OASIS STIX 2.1
- MITRE ATT&CK

### Post-Quantum Cryptography
- NIST FIPS 203: ML-KEM (Kyber)
- NIST FIPS 204: ML-DSA (Dilithium)

---

<div align="center">

**WIA Security Communication Protocol**

Phase 3: Secure Communication

**弘益人間** - Benefit All Humanity

</div>
