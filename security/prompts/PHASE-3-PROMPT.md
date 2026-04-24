# Phase 3: Communication Protocol
## Claude Code 작업 프롬프트

---

**Standard**: WIA Security (Cybersecurity Standards)
**Phase**: 3 of 4
**목표**: 보안 통신 프로토콜 및 메시징 표준 정의
**난이도**: ★★★★★
**예상 작업량**: 프로토콜 스펙 + 구현 + 테스트 + 암호화 통신

---

## 🎯 Phase 3 목표

### 핵심 질문
```
"Phase 1에서 데이터 형식을 정의하고,
 Phase 2에서 API를 구현했다.

 이제 보안 도구들이 어떻게 서로 통신할 것인가?

 - 취약점 스캐너 ↔ SIEM 연동?
 - SOC ↔ 위협 인텔리전스 플랫폼 통신?
 - 제로 트러스트 아키텍처에서 정책 엔진 ↔ 엔드포인트 통신?
 - 암호화된 메시지 교환?
 - 실시간 위협 정보 스트리밍?

 모든 통신을 표준화된 프로토콜로 정의할 수 있을까?"
```

### 목표
```
보안 시스템 간 통신 프로토콜 정의

- 메시지 교환 프로토콜 (Request/Response, Event Stream)
- 암호화 및 인증 메커니즘 (TLS 1.3, mTLS, PQC)
- Zero Trust 네트워크 프로토콜
- SIEM 연동 프로토콜
- 위협 인텔리전스 공유 (TAXII 2.1 호환)
- WebSocket/gRPC 기반 실시간 통신
```

---

## 📡 프로토콜 계층

### 1. Transport Layer (전송 계층)

```
┌─────────────────────────────────────┐
│   Application Protocol              │
│   (Security Data Exchange)          │
├─────────────────────────────────────┤
│   Message Format                    │
│   (JSON, Protocol Buffers)          │
├─────────────────────────────────────┤
│   Encryption Layer                  │
│   (TLS 1.3, Post-Quantum Hybrid)    │
├─────────────────────────────────────┤
│   Transport                         │
│   (HTTPS, WebSocket, gRPC)          │
└─────────────────────────────────────┘
```

### 2. Authentication & Authorization

```
Zero Trust Model:
- Continuous authentication (mTLS)
- JWT-based authorization
- Device attestation
- Context-based access control
- Policy Decision Point (PDP)
```

---

## 🔐 보안 통신 프로토콜 설계

### 기본 메시지 구조

```json
{
  "$schema": "https://wia.live/schemas/security/message/v1.schema.json",
  "protocol_version": "1.0.0",
  "message_id": "uuid-v4",
  "timestamp": "ISO 8601",
  "message_type": "request | response | event | notification",
  "sender": {
    "id": "sender-uuid",
    "type": "scanner | siem | policy_engine | endpoint",
    "certificate_fingerprint": "SHA-256 hash"
  },
  "receiver": {
    "id": "receiver-uuid",
    "type": "component-type"
  },
  "security": {
    "encryption": "TLS1.3 | PQC_HYBRID",
    "signature": "Ed25519 | Dilithium3",
    "signature_value": "base64-encoded"
  },
  "payload": {
    "content_type": "application/json",
    "data": {}
  },
  "metadata": {
    "correlation_id": "for request-response pairing",
    "priority": "critical | high | medium | low",
    "ttl_seconds": 3600
  }
}
```

---

## 📋 프로토콜 유형별 정의

### 1. 취약점 스캔 결과 전송 (Scanner → SIEM)

#### Request: 스캔 시작 알림
```json
{
  "message_type": "event",
  "sender": {
    "id": "scanner-001",
    "type": "vulnerability_scanner"
  },
  "receiver": {
    "id": "siem-central",
    "type": "siem"
  },
  "payload": {
    "content_type": "application/json",
    "data": {
      "event_type": "scan_started",
      "scan_id": "scan-2024-001",
      "target": {
        "ip_range": "192.168.1.0/24",
        "scan_type": "full"
      },
      "estimated_duration_minutes": 120
    }
  }
}
```

#### Response: SIEM 확인
```json
{
  "message_type": "response",
  "sender": {
    "id": "siem-central",
    "type": "siem"
  },
  "receiver": {
    "id": "scanner-001",
    "type": "vulnerability_scanner"
  },
  "payload": {
    "content_type": "application/json",
    "data": {
      "status": "acknowledged",
      "tracking_id": "siem-track-12345",
      "storage_location": "s3://siem-data/scans/scan-2024-001"
    }
  },
  "metadata": {
    "correlation_id": "original-message-id"
  }
}
```

#### Event: 취약점 발견 (실시간 스트리밍)
```json
{
  "message_type": "event",
  "payload": {
    "data": {
      "event_type": "vulnerability_found",
      "scan_id": "scan-2024-001",
      "vulnerability": {
        "id": "CVE-2024-12345",
        "severity": "critical",
        "cvss_score": 9.8,
        "affected_host": "192.168.1.100",
        "port": 443,
        "service": "OpenSSL 3.2.0"
      }
    }
  },
  "metadata": {
    "priority": "critical",
    "requires_immediate_action": true
  }
}
```

---

### 2. Zero Trust Policy Enforcement

#### Request: Access Decision Query
```json
{
  "message_type": "request",
  "sender": {
    "id": "policy-enforcement-point-01",
    "type": "pep"
  },
  "receiver": {
    "id": "policy-decision-point-central",
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
          "last_patch_date": "2024-11-15",
          "antivirus_status": "up_to_date",
          "encryption_enabled": true,
          "trust_score": 0.92
        },
        "location": {
          "ip": "203.0.113.45",
          "country": "US",
          "network_type": "corporate_vpn"
        },
        "authentication": {
          "method": "mfa",
          "factors": ["password", "totp"],
          "auth_time": "2024-12-14T10:30:00Z"
        }
      },
      "resource": {
        "type": "database",
        "id": "customer-db-prod",
        "classification": "confidential",
        "location": "aws-us-east-1"
      },
      "action": "read",
      "context": {
        "time_of_day": "business_hours",
        "day_of_week": "weekday",
        "risk_level": "medium"
      }
    }
  }
}
```

#### Response: Access Decision
```json
{
  "message_type": "response",
  "payload": {
    "data": {
      "decision": "permit",
      "conditions": [
        {
          "type": "time_limit",
          "value": "session expires in 8 hours"
        },
        {
          "type": "continuous_auth",
          "value": "re-auth required every 30 minutes"
        },
        {
          "type": "monitoring",
          "value": "all queries logged and audited"
        }
      ],
      "obligations": [
        {
          "type": "logging",
          "target": "siem-central",
          "detail_level": "verbose"
        },
        {
          "type": "alerting",
          "condition": "if query_count > 100 in 5 minutes"
        }
      ],
      "valid_until": "2024-12-14T18:30:00Z",
      "session_token": "encrypted-token-here"
    }
  },
  "metadata": {
    "correlation_id": "original-request-id",
    "decision_time_ms": 15
  }
}
```

---

### 3. Threat Intelligence Sharing (TAXII 2.1 Compatible)

#### Discovery Request
```json
{
  "message_type": "request",
  "payload": {
    "data": {
      "protocol": "TAXII-2.1-compatible",
      "request_type": "discovery",
      "api_root": "/taxii2/"
    }
  }
}
```

#### Discovery Response
```json
{
  "message_type": "response",
  "payload": {
    "data": {
      "title": "WIA Security Threat Intelligence Platform",
      "description": "TAXII 2.1 compatible threat intelligence sharing",
      "contact": "security@wia.live",
      "api_roots": [
        "https://threat-intel.wia.live/api/v1/",
        "https://threat-intel.wia.live/api/v2/"
      ],
      "default": "https://threat-intel.wia.live/api/v1/"
    }
  }
}
```

#### STIX Bundle Push
```json
{
  "message_type": "event",
  "payload": {
    "content_type": "application/stix+json",
    "data": {
      "type": "bundle",
      "id": "bundle--uuid",
      "objects": [
        {
          "type": "indicator",
          "id": "indicator--uuid",
          "created": "2024-12-14T10:00:00.000Z",
          "modified": "2024-12-14T10:00:00.000Z",
          "name": "Malicious IP",
          "pattern": "[ipv4-addr:value = '185.220.101.45']",
          "pattern_type": "stix",
          "valid_from": "2024-12-14T10:00:00.000Z",
          "labels": ["malicious-activity"],
          "indicator_types": ["malicious-activity"],
          "kill_chain_phases": [
            {
              "kill_chain_name": "mitre-attack",
              "phase_name": "command-and-control"
            }
          ]
        },
        {
          "type": "threat-actor",
          "id": "threat-actor--uuid",
          "created": "2024-12-14T10:00:00.000Z",
          "modified": "2024-12-14T10:00:00.000Z",
          "name": "APT29",
          "aliases": ["Cozy Bear", "The Dukes"],
          "sophistication": "advanced",
          "resource_level": "government",
          "primary_motivation": "espionage"
        }
      ]
    }
  }
}
```

---

### 4. Real-Time Alert Streaming (WebSocket)

#### WebSocket Connection Handshake
```json
{
  "message_type": "request",
  "payload": {
    "data": {
      "action": "subscribe",
      "channels": [
        "critical_alerts",
        "vulnerability_updates",
        "threat_intel_feed"
      ],
      "filters": {
        "severity": ["critical", "high"],
        "categories": ["malware", "intrusion", "data_breach"]
      }
    }
  }
}
```

#### Alert Event (Server → Client)
```json
{
  "message_type": "event",
  "payload": {
    "data": {
      "alert_type": "security_incident",
      "severity": "critical",
      "title": "Potential Data Exfiltration Detected",
      "description": "Unusual outbound traffic pattern detected",
      "source": {
        "host": "web-server-03",
        "ip": "10.0.1.50",
        "user": "www-data"
      },
      "destination": {
        "ip": "45.142.212.61",
        "country": "Unknown",
        "reputation": "malicious"
      },
      "indicators": {
        "data_volume_mb": 2500,
        "duration_seconds": 180,
        "connection_count": 1,
        "ports": [443]
      },
      "recommended_actions": [
        "Isolate affected host",
        "Block destination IP at firewall",
        "Initiate incident response procedure",
        "Preserve forensic evidence"
      ],
      "mitre_attack": {
        "tactic": "TA0010 - Exfiltration",
        "technique": "T1041 - Exfiltration Over C2 Channel"
      }
    }
  },
  "metadata": {
    "priority": "critical",
    "requires_acknowledgment": true,
    "escalation_timeout_seconds": 300
  }
}
```

---

### 5. AI Security Model Validation Protocol

#### Request: Model Inference Query
```json
{
  "message_type": "request",
  "payload": {
    "data": {
      "request_type": "inference",
      "model_id": "malware-classifier-v2",
      "input": {
        "file_hash": "a3f8b2c1e...",
        "file_size": 2048576,
        "file_type": "PE32 executable",
        "metadata": {
          "source": "email_attachment",
          "sender": "suspicious@example.com"
        }
      },
      "options": {
        "explain": true,
        "confidence_threshold": 0.85
      }
    }
  }
}
```

#### Response: Inference Result with Explanation
```json
{
  "message_type": "response",
  "payload": {
    "data": {
      "prediction": "malicious",
      "confidence": 0.94,
      "classification": {
        "family": "ransomware",
        "variant": "lockbit_3.0",
        "severity": "critical"
      },
      "explanation": {
        "method": "SHAP",
        "top_features": [
          {
            "feature": "imports_CryptEncrypt",
            "importance": 0.32,
            "value": true
          },
          {
            "feature": "suspicious_strings",
            "importance": 0.28,
            "value": ["DECRYPT_FILES", "BTC_ADDRESS"]
          },
          {
            "feature": "packer_detected",
            "importance": 0.15,
            "value": "UPX"
          }
        ]
      },
      "recommended_action": "quarantine_and_alert",
      "additional_scans": {
        "sandboxing": "recommended",
        "yara_rules": ["ransomware_lockbit"]
      }
    }
  }
}
```

---

## 🔒 암호화 및 인증

### TLS 1.3 Configuration

```yaml
tls_config:
  version: "1.3"
  cipher_suites:
    - TLS_AES_256_GCM_SHA384
    - TLS_CHACHA20_POLY1305_SHA256
    - TLS_AES_128_GCM_SHA256

  certificate_authentication:
    mode: "mutual"  # mTLS
    client_cert_required: true
    ca_certificates:
      - "/etc/ssl/certs/wia-ca.pem"

  post_quantum_hybrid:
    enabled: true
    kem_algorithm: "Kyber768"
    classical_fallback: "X25519"
```

### Message Signature (Ed25519 / Dilithium)

```rust
use ed25519_dalek::{Keypair, Signature, Signer};

fn sign_message(message: &[u8], keypair: &Keypair) -> Signature {
    keypair.sign(message)
}

fn verify_signature(
    message: &[u8],
    signature: &Signature,
    public_key: &PublicKey
) -> bool {
    public_key.verify(message, signature).is_ok()
}
```

---

## 📁 산출물 목록

```
/spec/PHASE-3-COMMUNICATION-PROTOCOL.md
/spec/protocols/
├── message-format.md
├── authentication.md
├── zero-trust-protocol.md
├── siem-integration.md
├── threat-intel-sharing.md
└── real-time-streaming.md

/spec/schemas/
├── message.schema.json
├── request.schema.json
├── response.schema.json
├── event.schema.json
└── notification.schema.json

/api/rust/src/protocol/
├── mod.rs
├── message.rs              # 메시지 구조
├── transport.rs            # HTTP/WebSocket/gRPC
├── encryption.rs           # TLS 1.3, PQC
├── authentication.rs       # mTLS, JWT
├── zero_trust.rs           # ZT 프로토콜
├── siem.rs                 # SIEM 연동
├── taxii.rs                # TAXII 2.1
└── streaming.rs            # WebSocket 스트리밍

/api/rust/src/server/
├── mod.rs
├── http_server.rs
├── websocket_server.rs
└── grpc_server.rs

/examples/
├── secure_client.rs
├── websocket_stream.rs
├── taxii_client.rs
└── zero_trust_pdp.rs
```

---

## ✅ 완료 체크리스트

```
□ 프로토콜 스펙 문서 작성
□ 메시지 형식 JSON Schema 정의
□ TLS 1.3 설정 구현
□ mTLS 인증 구현
□ JWT 토큰 발급/검증 구현
□ Post-Quantum Hybrid 암호화 지원
□ WebSocket 서버 구현
□ gRPC 서비스 정의 (optional)
□ Zero Trust 프로토콜 구현
□ SIEM 연동 프로토콜 구현
□ TAXII 2.1 호환 구현
□ 실시간 알림 스트리밍 구현
□ 메시지 서명/검증 구현
□ 통합 테스트 (클라이언트-서버)
□ 성능 테스트 (처리량, 지연시간)
□ 보안 테스트 (침투 테스트)
□ 예제 코드 작성
□ README 업데이트
```

---

## 🔄 작업 순서

```
1. 프로토콜 스펙 문서 작성
   - 메시지 형식 정의
   - 통신 패턴 정의
   ↓
2. JSON Schema 생성
   ↓
3. Rust 프로토콜 모듈 구현
   - message.rs
   - transport.rs
   ↓
4. TLS 1.3 / mTLS 구현
   ↓
5. 인증 모듈 구현 (JWT)
   ↓
6. WebSocket 서버 구현
   ↓
7. Zero Trust 프로토콜 구현
   ↓
8. SIEM 연동 프로토콜
   ↓
9. TAXII 2.1 구현
   ↓
10. 실시간 스트리밍 구현
   ↓
11. 테스트 작성 및 실행
   ↓
12. 예제 코드 작성
   ↓
13. 완료 체크리스트 확인
   ↓
14. Phase 4 시작 가능
```

---

## ⚠️ 주의사항

### DO (해야 할 것)

```
✅ TLS 1.3 이상만 사용
✅ mTLS로 양방향 인증 구현
✅ 메시지 서명으로 무결성 보장
✅ 타임스탬프로 재생 공격 방지
✅ Rate limiting 구현
✅ 메시지 크기 제한 (DoS 방지)
✅ 연결 타임아웃 설정
✅ 에러 처리 철저히
✅ 로깅 (단, 민감정보 제외)
```

### DON'T (하지 말 것)

```
❌ TLS 1.2 이하 사용
❌ 자체 암호화 프로토콜 구현
❌ 인증 없는 연결 허용
❌ 민감 정보 평문 전송
❌ 하드코딩된 인증 정보
❌ 무제한 메시지 크기
❌ 에러 메시지에 민감 정보 포함
```

---

## 🔗 참고 자료

### 프로토콜 표준
- **TLS 1.3**: RFC 8446
- **mTLS**: RFC 8705
- **JWT**: RFC 7519
- **WebSocket**: RFC 6455
- **gRPC**: https://grpc.io/

### 보안 표준
- **TAXII 2.1**: https://docs.oasis-open.org/cti/taxii/v2.1/
- **STIX 2.1**: https://docs.oasis-open.org/cti/stix/v2.1/
- **NIST SP 800-207**: Zero Trust Architecture

### Rust 라이브러리
- **tokio**: https://tokio.rs/
- **tokio-tungstenite**: WebSocket
- **tonic**: gRPC for Rust
- **rustls**: TLS implementation
- **jsonwebtoken**: JWT

---

## 🚀 작업 시작

이제 Phase 3 작업을 시작하세요.

첫 번째 단계: **프로토콜 스펙 문서 작성**

```markdown
/spec/PHASE-3-COMMUNICATION-PROTOCOL.md
```

안전한 통신의 구현을 위해! 🔐📡

---

<div align="center">

**Phase 3 of 4**

Communication Protocol

🔐 Secure, Authenticated, Encrypted 📡

🛡️ 弘益人間 - Benefit All Humanity 🛡️

</div>
