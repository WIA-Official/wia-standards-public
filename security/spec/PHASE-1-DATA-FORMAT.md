# WIA Security Data Format Standard

**Version**: 1.0.0
**Status**: Draft
**Phase**: 1 of 4

---

## 1. 개요 (Overview)

WIA Security Data Format Standard는 사이버보안 이벤트, 위협 인텔리전스, 취약점, 사고 정보를 표준화된 JSON 형식으로 정의합니다.

### 1.1 설계 원칙

- **호환성**: STIX 2.1, OCSF, ECS와 호환 가능한 구조
- **확장성**: 새로운 이벤트 유형 추가 용이
- **명확성**: 명확한 필드 정의와 타입
- **ATT&CK 통합**: MITRE ATT&CK 프레임워크 필수 지원

### 1.2 범위

- 보안 경고 (Security Alert)
- 위협 인텔리전스 (Threat Intelligence)
- 취약점 (Vulnerability)
- 보안 사고 (Incident)
- 네트워크 이벤트 (Network Event)
- 엔드포인트 이벤트 (Endpoint Event)
- 인증 이벤트 (Authentication Event)

---

## 2. 용어 정의 (Terminology)

| 용어 | 정의 |
|-----|------|
| **Event** | 보안 관련 발생 사건의 기록 |
| **Alert** | 탐지 규칙에 의해 생성된 경고 |
| **Indicator** | 위협을 식별하는 데 사용되는 데이터 (IOC) |
| **TTP** | Tactics, Techniques, and Procedures |
| **CVE** | Common Vulnerabilities and Exposures |
| **CVSS** | Common Vulnerability Scoring System |

---

## 3. 기본 구조 (Base Structure)

### 3.1 공통 Envelope

모든 WIA Security 이벤트는 다음 기본 구조를 따릅니다:

```json
{
  "$schema": "https://wia.live/security/v1/schema.json",
  "version": "1.0.0",
  "id": "uuid",
  "type": "event_type",
  "timestamp": "ISO 8601",
  "severity": 0-10,
  "source": { ... },
  "data": { ... },
  "context": { ... },
  "mitre": { ... },
  "meta": { ... }
}
```

### 3.2 필드 정의

| 필드 | 타입 | 필수 | 설명 |
|-----|------|------|------|
| `$schema` | string | 권장 | 스키마 URL |
| `version` | string | 필수 | 형식 버전 (SemVer) |
| `id` | string (UUID) | 필수 | 고유 식별자 |
| `type` | string | 필수 | 이벤트 유형 |
| `timestamp` | string (ISO 8601) | 필수 | 이벤트 발생 시간 |
| `severity` | number (0-10) | 필수 | 심각도 |
| `source` | object | 필수 | 데이터 소스 정보 |
| `data` | object | 필수 | 유형별 상세 데이터 |
| `context` | object | 선택 | 컨텍스트 정보 |
| `mitre` | object | 선택 | MITRE ATT&CK 매핑 |
| `meta` | object | 선택 | 메타데이터 |

### 3.3 Source 객체

```json
{
  "source": {
    "type": "siem|edr|ids|firewall|scanner|custom",
    "name": "데이터 소스 이름",
    "vendor": "제조사",
    "version": "버전"
  }
}
```

### 3.4 Context 객체

```json
{
  "context": {
    "host": {
      "hostname": "string",
      "ip": ["string"],
      "mac": ["string"],
      "os": {
        "name": "string",
        "version": "string",
        "build": "string"
      }
    },
    "network": {
      "source": {
        "ip": "string",
        "port": "number",
        "hostname": "string"
      },
      "destination": {
        "ip": "string",
        "port": "number",
        "hostname": "string"
      },
      "protocol": "string",
      "direction": "inbound|outbound|internal"
    },
    "user": {
      "name": "string",
      "domain": "string",
      "email": "string",
      "employee_id": "string"
    },
    "cloud": {
      "provider": "aws|azure|gcp",
      "account_id": "string",
      "region": "string"
    }
  }
}
```

### 3.5 MITRE ATT&CK 객체

```json
{
  "mitre": {
    "tactic": "TA0001",
    "tactic_name": "Initial Access",
    "technique": "T1566",
    "technique_name": "Phishing",
    "sub_technique": "T1566.001",
    "sub_technique_name": "Spearphishing Attachment"
  }
}
```

### 3.6 Meta 객체

```json
{
  "meta": {
    "confidence": 0.0-1.0,
    "tags": ["string"],
    "labels": { "key": "value" },
    "raw": "원본 로그 (선택)",
    "correlation_id": "관련 이벤트 ID"
  }
}
```

---

## 4. 이벤트 유형별 데이터 형식 (Event Type Definitions)

### 4.1 Security Alert (보안 경고)

**type**: `"alert"`

```json
{
  "data": {
    "alert_id": "ALERT-2025-001234",
    "title": "경고 제목",
    "description": "경고 설명",
    "category": "malware|intrusion|policy|reconnaissance|other",
    "status": "new|investigating|resolved|false_positive|closed",
    "priority": "critical|high|medium|low|info",
    "assignee": "담당자",
    "detection_rule": {
      "id": "RULE-001",
      "name": "규칙 이름",
      "version": "1.0"
    },
    "indicators": [
      {
        "type": "ip|domain|url|hash|email|file",
        "value": "값",
        "context": "설명"
      }
    ],
    "first_seen": "ISO 8601",
    "last_seen": "ISO 8601",
    "count": 1
  }
}
```

### 4.2 Threat Intelligence (위협 인텔리전스)

**type**: `"threat_intel"`

```json
{
  "data": {
    "threat_type": "malware|apt|campaign|botnet|ransomware|phishing",
    "threat_name": "위협 이름",
    "threat_family": "위협 패밀리",
    "aliases": ["별칭"],
    "first_seen": "ISO 8601",
    "last_seen": "ISO 8601",
    "status": "active|inactive|unknown",
    "indicators": [
      {
        "type": "ip|domain|url|file_hash_md5|file_hash_sha1|file_hash_sha256|email|mutex|registry",
        "value": "값",
        "confidence": 0.0-1.0,
        "first_seen": "ISO 8601",
        "last_seen": "ISO 8601",
        "context": {
          "description": "설명",
          "kill_chain_phase": "단계"
        }
      }
    ],
    "ttps": [
      {
        "tactic": "TA0001",
        "tactic_name": "Initial Access",
        "technique": "T1566.001",
        "technique_name": "Spearphishing Attachment"
      }
    ],
    "target_sectors": ["finance", "healthcare"],
    "target_countries": ["KR", "US"],
    "references": ["URL"],
    "report_id": "리포트 ID"
  }
}
```

### 4.3 Vulnerability (취약점)

**type**: `"vulnerability"`

```json
{
  "data": {
    "vuln_id": "CVE-2024-12345",
    "title": "취약점 제목",
    "description": "취약점 설명",
    "cvss": {
      "version": "3.1",
      "score": 9.8,
      "vector": "CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:H/I:H/A:H",
      "severity": "critical|high|medium|low|none",
      "base_score": 9.8,
      "temporal_score": 9.5,
      "environmental_score": 9.0
    },
    "cwe": ["CWE-78", "CWE-94"],
    "affected_products": [
      {
        "vendor": "제조사",
        "product": "제품명",
        "versions": ["< 2.0.0", "1.x"],
        "cpe": "cpe:2.3:..."
      }
    ],
    "exploit_available": true,
    "exploit_details": {
      "type": "poc|weaponized|in_the_wild",
      "url": "익스플로잇 URL",
      "maturity": "proof_of_concept|functional|high"
    },
    "patch_available": true,
    "patch_details": {
      "vendor_url": "패치 URL",
      "fixed_versions": ["2.0.0"],
      "workarounds": ["임시 조치"]
    },
    "references": ["URL"],
    "published": "ISO 8601",
    "modified": "ISO 8601"
  }
}
```

### 4.4 Incident (보안 사고)

**type**: `"incident"`

```json
{
  "data": {
    "incident_id": "INC-2025-0042",
    "title": "사고 제목",
    "description": "사고 설명",
    "category": "malware|phishing|ransomware|data_breach|ddos|unauthorized_access|insider_threat|apt|other",
    "status": "new|triaging|investigating|containing|eradicating|recovering|closed",
    "priority": "critical|high|medium|low",
    "impact": {
      "availability": "none|low|medium|high|critical",
      "confidentiality": "none|low|medium|high|critical",
      "integrity": "none|low|medium|high|critical",
      "financial": "unknown|minimal|moderate|significant|severe",
      "reputation": "unknown|minimal|moderate|significant|severe"
    },
    "timeline": [
      {
        "timestamp": "ISO 8601",
        "action": "수행 작업",
        "actor": "수행자",
        "details": "상세 내용"
      }
    ],
    "affected_assets": [
      {
        "type": "host|server|network|application|database|user",
        "id": "자산 ID",
        "name": "자산 이름",
        "ip": "IP 주소",
        "criticality": "critical|high|medium|low"
      }
    ],
    "iocs": [
      {
        "type": "ip|domain|url|hash|email",
        "value": "값",
        "description": "설명",
        "first_seen": "ISO 8601"
      }
    ],
    "response_actions": [
      {
        "action": "isolate_host|block_ip|disable_user|quarantine_file|collect_forensics",
        "target": "대상",
        "status": "pending|in_progress|completed|failed",
        "timestamp": "ISO 8601",
        "actor": "수행자",
        "notes": "비고"
      }
    ],
    "root_cause": "근본 원인",
    "lessons_learned": "교훈",
    "created_at": "ISO 8601",
    "updated_at": "ISO 8601",
    "closed_at": "ISO 8601",
    "lead_analyst": "담당 분석가",
    "team": ["팀원"]
  }
}
```

### 4.5 Network Event (네트워크 이벤트)

**type**: `"network_event"`

```json
{
  "data": {
    "event_type": "connection|dns|http|tls|smtp|ftp|ssh|rdp|custom",
    "protocol": "TCP|UDP|ICMP|GRE|ESP",
    "direction": "inbound|outbound|internal",
    "action": "allowed|blocked|dropped|reset",
    "source": {
      "ip": "IP 주소",
      "port": 12345,
      "hostname": "호스트명",
      "mac": "MAC 주소",
      "geo": {
        "country": "국가 코드",
        "country_name": "국가명",
        "city": "도시",
        "latitude": 37.5665,
        "longitude": 126.9780,
        "asn": 12345,
        "org": "조직명"
      }
    },
    "destination": {
      "ip": "IP 주소",
      "port": 443,
      "hostname": "호스트명",
      "geo": { ... }
    },
    "bytes_sent": 1024,
    "bytes_received": 4096,
    "packets_sent": 10,
    "packets_received": 15,
    "duration_ms": 5000,
    "rule_matched": "매칭된 규칙",
    "application": "애플리케이션",
    "url": "요청 URL",
    "http": {
      "method": "GET|POST|...",
      "status_code": 200,
      "user_agent": "User-Agent",
      "referer": "Referer",
      "content_type": "Content-Type"
    },
    "dns": {
      "query": "쿼리 도메인",
      "query_type": "A|AAAA|CNAME|MX|...",
      "response": ["응답"],
      "response_code": "NOERROR|NXDOMAIN|..."
    },
    "tls": {
      "version": "TLS 1.3",
      "cipher": "TLS_AES_256_GCM_SHA384",
      "sni": "SNI",
      "certificate": {
        "issuer": "발급자",
        "subject": "주체",
        "not_before": "ISO 8601",
        "not_after": "ISO 8601",
        "fingerprint_sha256": "지문"
      }
    }
  }
}
```

### 4.6 Endpoint Event (엔드포인트 이벤트)

**type**: `"endpoint_event"`

```json
{
  "data": {
    "event_type": "process_creation|process_termination|file_create|file_modify|file_delete|registry_create|registry_modify|registry_delete|network_connection|dll_load|driver_load|service_install",
    "host": {
      "hostname": "호스트명",
      "ip": ["IP 주소"],
      "mac": ["MAC 주소"],
      "os": {
        "name": "Windows|Linux|macOS",
        "version": "버전",
        "build": "빌드"
      },
      "domain": "도메인",
      "agent_id": "에이전트 ID"
    },
    "process": {
      "pid": 1234,
      "ppid": 5678,
      "name": "프로세스명",
      "path": "실행 경로",
      "command_line": "명령줄",
      "user": "실행 사용자",
      "integrity_level": "system|high|medium|low",
      "start_time": "ISO 8601",
      "hash": {
        "md5": "MD5 해시",
        "sha1": "SHA1 해시",
        "sha256": "SHA256 해시"
      },
      "signature": {
        "signed": true,
        "valid": true,
        "issuer": "서명자",
        "subject": "주체"
      },
      "parent": {
        "pid": 5678,
        "name": "부모 프로세스명",
        "path": "부모 경로",
        "command_line": "부모 명령줄"
      }
    },
    "file": {
      "path": "파일 경로",
      "name": "파일명",
      "extension": "확장자",
      "size": 1024,
      "hash": {
        "md5": "MD5",
        "sha1": "SHA1",
        "sha256": "SHA256"
      },
      "created": "ISO 8601",
      "modified": "ISO 8601",
      "accessed": "ISO 8601",
      "owner": "소유자",
      "permissions": "권한"
    },
    "registry": {
      "key": "레지스트리 키",
      "value_name": "값 이름",
      "value_data": "값 데이터",
      "value_type": "REG_SZ|REG_DWORD|...",
      "previous_data": "이전 값"
    },
    "network_connection": {
      "direction": "inbound|outbound",
      "protocol": "TCP|UDP",
      "local_ip": "로컬 IP",
      "local_port": 12345,
      "remote_ip": "원격 IP",
      "remote_port": 443,
      "state": "연결 상태"
    },
    "dll": {
      "path": "DLL 경로",
      "name": "DLL 이름",
      "hash": { ... },
      "signed": true
    }
  }
}
```

### 4.7 Authentication Event (인증 이벤트)

**type**: `"auth_event"`

```json
{
  "data": {
    "event_type": "login_success|login_failure|logout|password_change|account_locked|account_unlocked|mfa_success|mfa_failure|privilege_escalation",
    "result": "success|failure",
    "failure_reason": "invalid_credentials|account_disabled|account_locked|expired_password|mfa_failed|unknown",
    "user": {
      "name": "사용자명",
      "domain": "도메인",
      "email": "이메일",
      "employee_id": "사번",
      "groups": ["그룹"],
      "roles": ["역할"]
    },
    "source": {
      "ip": "소스 IP",
      "hostname": "소스 호스트",
      "geo": {
        "country": "국가",
        "city": "도시"
      },
      "user_agent": "User-Agent",
      "device_type": "desktop|mobile|tablet|unknown"
    },
    "target": {
      "type": "application|host|service|network",
      "name": "대상 이름",
      "ip": "대상 IP",
      "url": "대상 URL"
    },
    "auth_method": "password|sso|certificate|biometric|mfa|api_key|oauth",
    "mfa_used": true,
    "mfa_method": "totp|sms|push|hardware_token|biometric",
    "session_id": "세션 ID",
    "logon_type": "interactive|network|batch|service|remote_interactive",
    "attempt_count": 5,
    "previous_login": "ISO 8601",
    "risk_score": 0.75,
    "risk_factors": ["unusual_location", "unusual_time", "failed_attempts"]
  }
}
```

---

## 5. MITRE ATT&CK 매핑 (ATT&CK Mapping)

### 5.1 전술 (Tactics)

| ID | 이름 | 설명 |
|----|------|------|
| TA0001 | Initial Access | 초기 접근 |
| TA0002 | Execution | 실행 |
| TA0003 | Persistence | 지속성 |
| TA0004 | Privilege Escalation | 권한 상승 |
| TA0005 | Defense Evasion | 방어 회피 |
| TA0006 | Credential Access | 자격 증명 접근 |
| TA0007 | Discovery | 탐색 |
| TA0008 | Lateral Movement | 측면 이동 |
| TA0009 | Collection | 수집 |
| TA0010 | Exfiltration | 유출 |
| TA0011 | Command and Control | 명령 및 제어 |
| TA0040 | Impact | 영향 |
| TA0042 | Resource Development | 리소스 개발 |
| TA0043 | Reconnaissance | 정찰 |

### 5.2 매핑 예시

```json
{
  "mitre": {
    "tactic": "TA0002",
    "tactic_name": "Execution",
    "technique": "T1059",
    "technique_name": "Command and Scripting Interpreter",
    "sub_technique": "T1059.001",
    "sub_technique_name": "PowerShell"
  }
}
```

---

## 6. STIX 2.1 호환성 (STIX Compatibility)

### 6.1 변환 매핑

| WIA Security Type | STIX 2.1 Type |
|------------------|---------------|
| threat_intel | indicator, malware, threat-actor |
| vulnerability | vulnerability |
| incident | incident (extension) |
| alert | sighting |
| ioc (ip) | ipv4-addr, ipv6-addr |
| ioc (domain) | domain-name |
| ioc (url) | url |
| ioc (hash) | file |

### 6.2 STIX Bundle 생성

```json
{
  "type": "bundle",
  "id": "bundle--xxx",
  "objects": [
    {
      "type": "indicator",
      "spec_version": "2.1",
      "id": "indicator--xxx",
      "created": "2025-12-14T00:00:00.000Z",
      "modified": "2025-12-14T00:00:00.000Z",
      "name": "Malicious IP",
      "pattern": "[ipv4-addr:value = '192.168.1.100']",
      "pattern_type": "stix",
      "valid_from": "2025-12-14T00:00:00.000Z"
    }
  ]
}
```

---

## 7. 심각도 및 우선순위 체계 (Severity & Priority)

### 7.1 심각도 (Severity)

| 값 | 레벨 | 설명 |
|----|------|------|
| 0 | Informational | 정보성 |
| 1-2 | Low | 낮음 |
| 3-4 | Medium | 중간 |
| 5-6 | High | 높음 |
| 7-8 | Critical | 심각 |
| 9-10 | Emergency | 긴급 |

### 7.2 우선순위 (Priority)

| 값 | 레벨 | 설명 | 대응 시간 |
|----|------|------|----------|
| critical | 심각 | 즉각 대응 필요 | < 15분 |
| high | 높음 | 긴급 대응 필요 | < 1시간 |
| medium | 중간 | 조속한 대응 필요 | < 4시간 |
| low | 낮음 | 일반 대응 | < 24시간 |
| info | 정보 | 참고용 | N/A |

---

## 8. 확장성 (Extensibility)

### 8.1 커스텀 필드

`meta.custom` 객체를 통해 확장 필드를 추가할 수 있습니다:

```json
{
  "meta": {
    "custom": {
      "vendor_specific_field": "값",
      "internal_tracking_id": "ID"
    }
  }
}
```

### 8.2 커스텀 이벤트 타입

`type` 필드에 `custom_` 접두사를 사용하여 커스텀 이벤트를 정의할 수 있습니다:

```json
{
  "type": "custom_ot_security",
  "data": {
    "plc_id": "PLC-001",
    "register_change": { ... }
  }
}
```

---

## 9. 버전 관리 (Versioning)

- **Major**: 호환성이 깨지는 변경
- **Minor**: 새로운 기능 추가 (하위 호환)
- **Patch**: 버그 수정

현재 버전: `1.0.0`

---

## 10. 참고문헌 (References)

1. [OASIS STIX 2.1](https://docs.oasis-open.org/cti/stix/v2.1/stix-v2.1.html)
2. [OCSF Schema](https://schema.ocsf.io/)
3. [Elastic Common Schema](https://www.elastic.co/guide/en/ecs/current/index.html)
4. [MITRE ATT&CK](https://attack.mitre.org/)
5. [FIRST CVSS](https://www.first.org/cvss/)
6. [RFC 5424 - Syslog Protocol](https://tools.ietf.org/html/rfc5424)

---

<div align="center">

**WIA Security Standard**

Phase 1: Data Format

**弘益人間** - Benefit All Humanity

</div>
