# Phase 1 사전 조사 결과

## 개요

WIA Security Standard의 데이터 형식을 설계하기 위한 기존 표준 및 도구 조사 결과입니다.

---

## 1. 기존 표준 분석

### STIX 2.1 (Structured Threat Information Expression)

- **구조**: Bundle → Objects (SDO, SRO, SCO)
- **주요 객체 유형**:
  - SDO (Domain Objects): Attack Pattern, Campaign, Identity, Indicator, Malware, Threat Actor, Tool, Vulnerability
  - SRO (Relationship Objects): Relationship, Sighting
  - SCO (Cyber Observable): File, IPv4-Addr, Domain-Name, URL, etc.
- **장점**:
  - 산업 표준으로 널리 채택
  - TAXII 2.1과 함께 위협 정보 공유에 최적화
  - 풍부한 관계 모델링
- **단점**:
  - 복잡한 구조
  - 학습 곡선이 높음
  - 실시간 이벤트 처리에 최적화되지 않음
- **WIA Security 적용**: STIX 호환성 유지, 변환 레이어 제공

### OCSF (Open Cybersecurity Schema Framework)

- **구조**: Event Classes → Categories → Activities
- **이벤트 클래스**:
  - System Activity (1xxx): Process, File, Registry
  - Findings (2xxx): Detection, Vulnerability
  - IAM (3xxx): Authentication, Authorization
  - Network Activity (4xxx): DNS, HTTP, Network Connection
  - Application Activity (6xxx): Web, Email
- **장점**:
  - AWS, Splunk, IBM 등 주요 벤더 지원
  - 명확한 이벤트 분류 체계
  - 실시간 이벤트 처리에 적합
- **단점**:
  - 상대적으로 새로운 표준
  - 위협 인텔리전스보다 이벤트 중심
- **WIA Security 적용**: 이벤트 분류 체계 참조, 일부 필드 구조 채택

### Elastic Common Schema (ECS)

- **필드 구조**:
  - Base fields: @timestamp, message, tags
  - Event fields: event.kind, event.category, event.type
  - Host fields: host.name, host.ip, host.os
  - User fields: user.name, user.domain
  - Source/Destination: source.ip, destination.ip
- **장점**:
  - 단순하고 직관적
  - Elasticsearch 생태계와 완벽 호환
  - 풍부한 필드 정의
- **단점**:
  - Elastic 중심 설계
  - STIX와 직접 호환되지 않음
- **WIA Security 적용**: 필드 네이밍 컨벤션 참조

### MITRE ATT&CK

- **구조**:
  - Tactics (전술): 14개 (Initial Access, Execution, Persistence, ...)
  - Techniques (기법): 200+ 기법
  - Sub-techniques: 세부 기법
- **JSON 형식**:
  ```json
  {
    "tactic": "TA0001",
    "technique": "T1566",
    "sub_technique": "T1566.001"
  }
  ```
- **WIA Security 적용**: 모든 위협 이벤트에 ATT&CK 매핑 필수

### CVE/CVSS

- **CVE 형식**: CVE-YYYY-NNNNN
- **CVSS 3.1 벡터**:
  ```
  CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:H/I:H/A:H
  ```
- **심각도 레벨**: None (0.0), Low (0.1-3.9), Medium (4.0-6.9), High (7.0-8.9), Critical (9.0-10.0)
- **WIA Security 적용**: 취약점 데이터에 CVE/CVSS 필수 포함

---

## 2. SIEM 이벤트 형식 분석

### Splunk

- **이벤트 구조**:
  ```json
  {
    "time": 1702550400,
    "host": "server01",
    "source": "/var/log/auth.log",
    "sourcetype": "syslog",
    "_raw": "원본 로그"
  }
  ```
- **CIM (Common Information Model)**: 보안 이벤트 정규화
- **주요 필드**: action, app, dest, dvc, src, user

### IBM QRadar (LEEF)

- **형식**: LEEF:2.0|Vendor|Product|Version|EventID|...
- **필드**: devTime, usrName, src, dst, sev
- **특징**: 파이프(|)로 구분된 고정 형식

### Suricata EVE JSON

- **이벤트 유형**: alert, dns, http, tls, flow, fileinfo
- **구조**:
  ```json
  {
    "timestamp": "2025-12-14T10:00:00.000000+0000",
    "event_type": "alert",
    "src_ip": "10.0.0.1",
    "dest_ip": "192.168.1.1",
    "alert": {
      "signature": "ET MALWARE ...",
      "severity": 1
    }
  }
  ```

---

## 3. EDR 이벤트 형식 분석

### CrowdStrike Falcon

- **Detection 이벤트**:
  ```json
  {
    "detection_id": "ldt:...",
    "severity": 4,
    "tactic": "Execution",
    "technique": "PowerShell",
    "hostname": "WORKSTATION",
    "command_line": "..."
  }
  ```
- **특징**: MITRE ATT&CK 기본 매핑

### Microsoft Defender for Endpoint

- **Alert 스키마**:
  ```json
  {
    "alertId": "...",
    "severity": "High",
    "category": "Malware",
    "mitreTechniques": ["T1059.001"]
  }
  ```

---

## 4. 공통점 분석

### 모든 이벤트에 공통으로 필요한 필드

| 필드 | 설명 | 필수 여부 |
|-----|------|----------|
| id | 고유 식별자 | 필수 |
| timestamp | 이벤트 발생 시간 | 필수 |
| type | 이벤트 유형 | 필수 |
| severity | 심각도 | 필수 |
| source | 데이터 소스 | 필수 |
| host | 호스트 정보 | 권장 |
| user | 사용자 정보 | 선택 |
| network | 네트워크 정보 | 선택 |

### 카테고리별 고유 필드

| 카테고리 | 고유 필드 |
|---------|----------|
| Alert | title, description, status, assignee |
| Threat Intel | indicators, ttps, threat_type |
| Vulnerability | cve_id, cvss, affected_products |
| Incident | timeline, affected_assets, response_actions |
| Network | src_ip, dst_ip, protocol, bytes |
| Endpoint | process, file, registry |
| Auth | user, result, auth_method |

---

## 5. 결론

### 표준 형식 설계 방향

1. **OCSF 참조**: 이벤트 분류 체계 및 기본 구조
2. **STIX 호환**: 위협 인텔리전스 데이터 변환 지원
3. **ECS 필드**: 명확한 필드 네이밍 컨벤션
4. **ATT&CK 필수**: 모든 위협 이벤트에 ATT&CK 매핑

### STIX 호환성 고려

- WIA Security → STIX 2.1 변환 함수 제공
- STIX Bundle 생성 지원
- TAXII 2.1 서버/클라이언트 호환

### OCSF 참조

- 이벤트 클래스 분류 체계 적용
- severity, status 등 공통 필드 참조
- 확장 가능한 구조 유지

---

## 참고문헌

- [STIX 2.1 Specification](https://docs.oasis-open.org/cti/stix/v2.1/stix-v2.1.html)
- [OCSF Schema](https://schema.ocsf.io/)
- [Elastic Common Schema](https://www.elastic.co/guide/en/ecs/current/index.html)
- [MITRE ATT&CK](https://attack.mitre.org/)
- [CVE/CVSS](https://www.first.org/cvss/)
- [Suricata EVE JSON](https://suricata.readthedocs.io/en/latest/output/eve/eve-json-output.html)
