# Phase 1: Data Format Standard
## Claude Code 작업 프롬프트

---

**Standard**: WIA Security (Cybersecurity Standards)
**Phase**: 1 of 4
**목표**: 사이버보안 표준 데이터의 통일된 형식 정의
**난이도**: ★★★★☆
**예상 작업량**: 스펙 문서 1개 + JSON Schema + 예제 파일

---

## 🎯 Phase 1 목표

### 핵심 질문
```
"취약점 평가, 침투 테스트, 위협 인텔리전스, 보안 감사...
 각각 다른 도구와 형식으로 데이터를 생성한다.

 양자 내성 암호, AI 보안, 제로 트러스트 아키텍처까지
 모두 다른 방식으로 데이터를 관리한다.

 이걸 하나의 표준 형식으로 통일할 수 있을까?"
```

### 목표
```
사이버보안 영역에 관계없이
모든 보안 평가, 취약점 스캔, 위협 분석 데이터가
동일한 JSON 형식으로 표현되도록
Security Data Format Standard를 정의한다.
```

---

## 📋 사전 조사 (웹서치 필수)

### 1단계: 보안 기술 영역별 조사

아래 보안 기술 유형별로 웹서치하여 실제 데이터 형식을 조사하세요:

| 보안 영역 | 조사 대상 | 웹서치 키워드 |
|----------|----------|--------------|
| **Post-Quantum Crypto** | NIST PQC, Lattice-based crypto | "NIST post quantum cryptography standard 2024" |
| **Penetration Testing** | PTES, OWASP, Metasploit | "penetration testing report format JSON" |
| **Zero Trust Security** | NIST SP 800-207, BeyondCorp | "zero trust architecture data model" |
| **AI Security** | MITRE ATLAS, Adversarial ML | "AI security threat model data format" |
| **Threat Intelligence** | STIX/TAXII, MISP | "STIX 2.1 threat intelligence format" |
| **Vulnerability Management** | CVE, CVSS, NVD | "CVE JSON schema CVSS v3.1" |

### 2단계: 기존 표준/프레임워크 조사

| 표준/프레임워크 | 조사 내용 | 웹서치 키워드 |
|----------------|----------|--------------|
| **NIST Cybersecurity Framework** | 보안 프레임워크 구조 | "NIST CSF 2.0 implementation" |
| **MITRE ATT&CK** | 위협 행동 분류 | "MITRE ATT&CK framework data model" |
| **OWASP** | 웹 보안 취약점 | "OWASP Top 10 2024 data format" |
| **CIS Controls** | 보안 통제 | "CIS Controls v8 JSON format" |
| **STIX/TAXII** | 위협 인텔리전스 공유 | "STIX 2.1 specification" |
| **SARIF** | 정적 분석 결과 | "SARIF static analysis results format" |

### 3단계: 보안 도구 데이터 형식 조사

| 도구 카테고리 | 대표 도구 | 웹서치 키워드 |
|-------------|---------|--------------|
| **SIEM** | Splunk, ELK, QRadar | "SIEM log format JSON CEF" |
| **Vulnerability Scanner** | Nessus, OpenVAS, Qualys | "Nessus XML export format" |
| **Penetration Testing** | Metasploit, Burp Suite | "Metasploit report format XML" |
| **IDS/IPS** | Snort, Suricata, Zeek | "Suricata EVE JSON output format" |
| **SOAR** | Splunk SOAR, Cortex XSOAR | "SOAR playbook data format" |

### 4단계: 조사 결과 정리

조사 후 `/spec/RESEARCH-PHASE-1.md`에 다음을 정리:

```markdown
# Phase 1 사전 조사 결과

## 1. Post-Quantum Cryptography

### NIST PQC Standards
- 선정 알고리즘: CRYSTALS-Kyber (KEM), CRYSTALS-Dilithium (서명)
- 키 교환 데이터 구조: [조사 내용]
- 암호화 파라미터: [조사 내용]
- 필요 데이터 필드: [분석]

### 양자 내성 프로토콜
- TLS 1.3 PQC 확장: [조사 내용]
- 하이브리드 암호화: [조사 내용]
...

## 2. Penetration Testing

### PTES (Penetration Testing Execution Standard)
- 보고서 구조: [조사 내용]
- 취약점 분류: [조사 내용]
- 위험도 평가 방식: [조사 내용]

### Metasploit Framework
- 스캔 결과 형식: [조사 내용]
- 익스플로잇 데이터: [조사 내용]
...

## 3. Zero Trust Security

### NIST SP 800-207
- Zero Trust 아키텍처 구성요소: [조사 내용]
- 정책 엔진 데이터 모델: [조사 내용]
- 신뢰 평가 메트릭: [조사 내용]

### 실제 구현 (BeyondCorp, etc)
- 컨텍스트 기반 접근제어 데이터: [조사 내용]
...

## 4. AI Security

### MITRE ATLAS
- AI/ML 위협 분류: [조사 내용]
- 적대적 공격 유형: [조사 내용]

### Adversarial ML Threats
- Evasion, Poisoning, Model Extraction: [조사 내용]
- 방어 메커니즘 데이터: [조사 내용]
...

## 5. Threat Intelligence

### STIX 2.1
- 관찰 가능 객체 (Observable): [조사 내용]
- 위협 행위자 (Threat Actor): [조사 내용]
- 공격 패턴 (Attack Pattern): [조사 내용]

### MISP (Malware Information Sharing Platform)
- 이벤트 데이터 구조: [조사 내용]
...

## 6. Vulnerability Management

### CVE/CVSS
- CVE 레코드 구조: [조사 내용]
- CVSS v3.1 메트릭: [조사 내용]
- NVD 데이터 피드: [조사 내용]

## 7. 공통점 분석
- 모든 보안 영역에 공통으로 필요한 필드: [분석]
  - 식별자 (ID)
  - 타임스탬프
  - 심각도/위험도
  - 설명/컨텍스트
  - 참조 정보

- 영역별 고유 필드: [분석]

## 8. 결론
- 표준 형식 설계 방향: [제안]
- 확장성 고려사항: [제안]
- STIX/SARIF 등 기존 표준과의 호환성: [제안]
```

---

## 🏗️ 표준 설계

### 기본 구조 (제안)

```json
{
  "$schema": "https://wia.live/schemas/security/assessment.schema.json",
  "version": "1.0.0",
  "assessment": {
    "id": "고유 식별자 (UUID)",
    "type": "평가 유형",
    "name": "평가 프로젝트명",
    "status": "상태",
    "severity": "전체 심각도"
  },
  "target": {
    "organization": "대상 조직",
    "systems": ["대상 시스템 목록"],
    "scope": "평가 범위",
    "environment": "환경 정보"
  },
  "findings": [
    {
      "영역별 발견사항"
    }
  ],
  "timeline": {
    "started_at": "시작 시간",
    "completed_at": "완료 시간",
    "duration_hours": "소요 시간"
  },
  "compliance": {
    "frameworks": ["NIST CSF", "ISO 27001"],
    "controls": ["적용된 통제 목록"]
  },
  "meta": {
    "created_at": "생성일",
    "updated_at": "수정일",
    "assessor": "평가자",
    "tool_used": "사용 도구",
    "methodology": "평가 방법론"
  }
}
```

### 평가 유형별 `findings` 구조

#### Post-Quantum Cryptography Assessment
```json
{
  "findings": [
    {
      "id": "PQC-001",
      "category": "post_quantum_crypto",
      "title": "Legacy RSA-2048 Key Exchange Detected",
      "description": "System uses RSA-2048 for key exchange, vulnerable to quantum attacks",
      "severity": "high",
      "cvss_score": 7.5,
      "quantum_threat": {
        "vulnerable_to": "Shor's algorithm",
        "estimated_break_time": "minutes (with sufficient qubits)",
        "quantum_resistance": "none"
      },
      "current_crypto": {
        "algorithm": "RSA",
        "key_size": 2048,
        "usage": "key_exchange"
      },
      "recommended_pqc": {
        "algorithm": "CRYSTALS-Kyber",
        "security_level": "NIST Level 3",
        "key_size": 1568,
        "implementation": "liboqs"
      },
      "migration_plan": {
        "approach": "hybrid_mode",
        "steps": [
          "Deploy Kyber alongside RSA",
          "Monitor compatibility",
          "Gradual transition"
        ],
        "estimated_effort": "medium"
      }
    }
  ]
}
```

#### Penetration Testing Report
```json
{
  "findings": [
    {
      "id": "PENTEST-042",
      "category": "penetration_testing",
      "title": "SQL Injection in User Authentication",
      "description": "Username parameter vulnerable to SQL injection",
      "severity": "critical",
      "cvss_score": 9.8,
      "cvss_vector": "CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:H/I:H/A:H",
      "owasp_category": "A03:2021 – Injection",
      "vulnerability": {
        "type": "SQL Injection",
        "location": "/api/v1/auth/login",
        "parameter": "username",
        "method": "POST",
        "payload": "admin' OR '1'='1",
        "affected_component": "authentication_service"
      },
      "exploitation": {
        "difficulty": "easy",
        "prerequisites": "none",
        "impact": "Full database access, authentication bypass",
        "proof_of_concept": "curl -X POST ... [PoC command]"
      },
      "remediation": {
        "priority": "immediate",
        "recommendation": "Use parameterized queries/prepared statements",
        "code_example": "SELECT * FROM users WHERE username = ?",
        "verification": "Retest with same payload"
      },
      "mitre_attack": {
        "tactics": ["TA0001 - Initial Access"],
        "techniques": ["T1190 - Exploit Public-Facing Application"]
      }
    }
  ]
}
```

#### Zero Trust Architecture Assessment
```json
{
  "findings": [
    {
      "id": "ZTA-015",
      "category": "zero_trust",
      "title": "Insufficient Device Trust Validation",
      "description": "Policy engine lacks continuous device posture assessment",
      "severity": "medium",
      "nist_zt_pillar": "device_security",
      "current_state": {
        "device_inventory": "complete",
        "device_authentication": "present",
        "continuous_monitoring": "absent",
        "posture_assessment": "initial_only"
      },
      "gap_analysis": {
        "missing_controls": [
          "Real-time device health monitoring",
          "Dynamic trust score calculation",
          "Automatic quarantine for compromised devices"
        ],
        "risk_level": "medium"
      },
      "zero_trust_maturity": {
        "current_level": "Initial (Level 1)",
        "target_level": "Advanced (Level 3)",
        "maturity_score": 2.3
      },
      "recommended_implementation": {
        "solution": "Deploy EDR with ZT integration",
        "components": [
          "Continuous device monitoring agent",
          "Policy engine with device signals",
          "Automated remediation workflow"
        ],
        "vendors": ["CrowdStrike", "SentinelOne", "Microsoft Defender"]
      }
    }
  ]
}
```

#### AI Security Assessment
```json
{
  "findings": [
    {
      "id": "AISEC-008",
      "category": "ai_security",
      "title": "ML Model Vulnerable to Adversarial Examples",
      "description": "Image classifier susceptible to FGSM evasion attacks",
      "severity": "high",
      "mitre_atlas": {
        "tactic": "AML.T0015 - Evade ML Model",
        "technique": "AML.T0043.000 - Craft Adversarial Data"
      },
      "model_info": {
        "type": "image_classifier",
        "architecture": "ResNet-50",
        "framework": "TensorFlow 2.13",
        "task": "malware_detection_via_images"
      },
      "attack_details": {
        "attack_type": "FGSM (Fast Gradient Sign Method)",
        "epsilon": 0.03,
        "success_rate": 0.87,
        "misclassification_rate": "87% of test samples",
        "targeted": false
      },
      "robustness_metrics": {
        "clean_accuracy": 0.94,
        "adversarial_accuracy": 0.13,
        "robustness_score": 0.14
      },
      "defense_recommendations": {
        "primary": "Adversarial Training",
        "techniques": [
          "Train on adversarial examples (PGD-10)",
          "Input transformation (JPEG compression, bit depth reduction)",
          "Ensemble defenses",
          "Certified defenses (randomized smoothing)"
        ],
        "expected_improvement": "Adversarial accuracy: 0.13 → 0.65"
      }
    }
  ]
}
```

#### Threat Intelligence Report
```json
{
  "findings": [
    {
      "id": "TI-2024-0156",
      "category": "threat_intelligence",
      "title": "APT29 Targeting Critical Infrastructure",
      "description": "Observed campaign using novel phishing techniques",
      "severity": "critical",
      "confidence": "high",
      "threat_actor": {
        "name": "APT29 (Cozy Bear)",
        "aliases": ["The Dukes", "Nobelium"],
        "origin": "Russia",
        "motivation": "espionage",
        "sophistication": "advanced"
      },
      "campaign": {
        "name": "Operation WinterStorm",
        "active_since": "2024-01-15",
        "target_sectors": ["energy", "government", "telecommunications"],
        "target_regions": ["North America", "Europe"]
      },
      "ttps": {
        "initial_access": "T1566.001 - Spearphishing Attachment",
        "execution": "T1059.001 - PowerShell",
        "persistence": "T1547.001 - Registry Run Keys",
        "command_and_control": "T1071.001 - Web Protocols (HTTPS)"
      },
      "iocs": {
        "domains": ["update-security[.]com", "ms-cdn[.]net"],
        "ips": ["185.220.101.45", "45.142.212.61"],
        "file_hashes": [
          {
            "algorithm": "SHA-256",
            "value": "a3f8b2c1e..."
          }
        ],
        "email_subjects": ["Urgent: Security Update Required"]
      },
      "recommendations": {
        "immediate": [
          "Block IOCs in firewall/IDS",
          "Search logs for compromise indicators",
          "Review email gateway rules"
        ],
        "strategic": [
          "Implement email authentication (DMARC/DKIM)",
          "Deploy EDR on critical systems",
          "Conduct security awareness training"
        ]
      }
    }
  ]
}
```

#### Vulnerability Assessment (CVE/CVSS)
```json
{
  "findings": [
    {
      "id": "CVE-2024-12345",
      "category": "vulnerability",
      "title": "Buffer Overflow in OpenSSL 3.2.0",
      "description": "Heap buffer overflow in SSL handshake processing",
      "severity": "critical",
      "cvss": {
        "version": "3.1",
        "vector_string": "CVSS:3.1/AV:N/AC:L/PR:N/UI:N/S:U/C:H/I:H/A:H",
        "base_score": 9.8,
        "base_severity": "CRITICAL",
        "exploitability_score": 3.9,
        "impact_score": 5.9
      },
      "cwe": {
        "id": "CWE-787",
        "name": "Out-of-bounds Write"
      },
      "affected_systems": {
        "vendor": "OpenSSL",
        "product": "OpenSSL",
        "versions": ["3.2.0"],
        "platforms": ["linux", "windows", "macos"]
      },
      "vulnerability_details": {
        "discovered_by": "Security Researcher",
        "published_date": "2024-03-15",
        "exploit_available": true,
        "exploit_maturity": "functional",
        "ransomware_use": false
      },
      "impact_assessment": {
        "confidentiality": "high",
        "integrity": "high",
        "availability": "high",
        "scope": "remote_code_execution",
        "attack_vector": "network"
      },
      "remediation": {
        "patch_available": true,
        "fixed_versions": ["3.2.1", "3.3.0"],
        "workaround": "Disable vulnerable TLS extensions",
        "patch_priority": "critical",
        "sla": "24 hours"
      },
      "references": [
        "https://nvd.nist.gov/vuln/detail/CVE-2024-12345",
        "https://www.openssl.org/news/secadv/...",
        "https://www.exploit-db.com/exploits/..."
      ]
    }
  ]
}
```

---

## 📁 산출물 목록

Phase 1 완료 시 다음 파일을 생성해야 합니다:

### 1. 조사 문서
```
/spec/RESEARCH-PHASE-1.md
```

### 2. 표준 스펙 문서
```
/spec/PHASE-1-DATA-FORMAT.md

내용:
1. 개요 (Overview)
2. 용어 정의 (Terminology)
   - CVE, CVSS, APT, IOC, TTP, MITRE ATT&CK 등
3. 기본 구조 (Base Structure)
4. 보안 평가 유형별 데이터 형식
   - Post-Quantum Cryptography
   - Penetration Testing
   - Zero Trust Architecture
   - AI Security
   - Threat Intelligence
   - Vulnerability Management
5. 심각도 분류 체계 (Severity Classification)
6. 통합 및 상호운용성 (Integration & Interoperability)
   - STIX/TAXII 매핑
   - SARIF 호환성
   - SIEM 연동
7. 확장성 (Extensibility)
8. 버전 관리 (Versioning)
9. 예제 (Examples)
10. 참고문헌 (References)
```

### 3. JSON Schema 파일
```
/spec/schemas/
├── security-assessment.schema.json       (기본 스키마)
├── pqc-assessment.schema.json            (양자내성암호)
├── pentest-report.schema.json            (침투테스트)
├── zero-trust-assessment.schema.json     (제로트러스트)
├── ai-security-assessment.schema.json    (AI보안)
├── threat-intelligence.schema.json       (위협인텔리전스)
└── vulnerability-assessment.schema.json  (취약점관리)
```

### 4. 예제 데이터 파일
```
/examples/sample-data/
├── pqc-crypto-audit-example.json
├── pentest-web-app-example.json
├── zero-trust-maturity-example.json
├── ai-model-security-example.json
├── apt-campaign-example.json
└── cve-vulnerability-example.json
```

### 5. 매핑 문서
```
/spec/mappings/
├── STIX-MAPPING.md          (STIX 2.1 매핑)
├── MITRE-ATTCK-MAPPING.md   (MITRE ATT&CK 매핑)
├── NIST-CSF-MAPPING.md      (NIST CSF 매핑)
└── SARIF-MAPPING.md         (SARIF 매핑)
```

---

## ✅ 완료 체크리스트

Phase 1 완료 전 확인:

```
□ 웹서치로 6개 보안 영역 데이터 형식 조사 완료
□ 기존 표준 (STIX, SARIF, CVE 등) 조사 완료
□ /spec/RESEARCH-PHASE-1.md 작성 완료
□ /spec/PHASE-1-DATA-FORMAT.md 작성 완료 (50+ 페이지)
□ JSON Schema 파일 생성 완료 (기본 + 영역별 6개)
□ 예제 데이터 파일 생성 완료 (6개)
□ 매핑 문서 작성 완료 (STIX, MITRE, NIST, SARIF)
□ JSON Schema로 예제 데이터 검증 통과
□ README 업데이트 (Phase 1 완료 표시)
```

---

## 🔄 작업 순서

```
1. 웹서치로 보안 기술 및 기존 표준 조사
   - PQC, Pentest, Zero Trust, AI Security
   - STIX, MITRE ATT&CK, CVE/CVSS
   ↓
2. /spec/RESEARCH-PHASE-1.md 작성
   - 각 영역별 조사 내용 정리
   - 공통점/차이점 분석
   ↓
3. 조사 결과 바탕으로 표준 설계
   - 기본 구조 설계
   - 영역별 확장 구조 설계
   ↓
4. /spec/PHASE-1-DATA-FORMAT.md 작성
   - 상세 스펙 문서화
   ↓
5. JSON Schema 파일 생성
   - 기본 스키마 + 6개 영역별 스키마
   ↓
6. 예제 데이터 파일 생성
   - 실제 시나리오 기반 예제
   ↓
7. 기존 표준 매핑 문서 작성
   - STIX, MITRE, NIST, SARIF
   ↓
8. 스키마 검증 테스트
   - 모든 예제 데이터 검증
   ↓
9. 완료 체크리스트 확인
   ↓
10. Phase 2 시작 가능
```

---

## ⚠️ 주의사항

### DO (해야 할 것)

```
✅ NIST, MITRE, OWASP 등 공신력 있는 표준 참조
✅ CVE, CVSS 등 기존 표준과의 호환성 보장
✅ 실제 보안 도구의 출력 형식 조사 (Nessus, Metasploit 등)
✅ STIX/TAXII와의 상호운용성 고려
✅ 심각도 분류 명확히 정의 (Critical, High, Medium, Low)
✅ 타임스탬프는 ISO 8601 형식 사용
✅ 모든 ID는 UUID v4 사용
✅ 과학적/기술적으로 검증된 보안 지식 기반
```

### DON'T (하지 말 것)

```
❌ 추측으로 보안 데이터 형식 정의
❌ 기존 표준 무시하고 새로 만들기
❌ 민감 정보 (실제 취약점 상세) 예제에 포함
❌ 단일 벤더 종속적인 설계
❌ 보안 전문 용어 없이 작성
❌ 과장되거나 근거 없는 위협 시나리오
```

---

## 🔗 참고 자료

### 표준 문서
- **NIST PQC**: https://csrc.nist.gov/projects/post-quantum-cryptography
- **MITRE ATT&CK**: https://attack.mitre.org/
- **STIX 2.1**: https://docs.oasis-open.org/cti/stix/v2.1/
- **CVE/CVSS**: https://www.cve.org/, https://www.first.org/cvss/
- **SARIF**: https://sarifweb.azurewebsites.net/
- **NIST CSF**: https://www.nist.gov/cyberframework
- **OWASP Top 10**: https://owasp.org/www-project-top-ten/

### 도구 문서
- **Metasploit**: https://docs.metasploit.com/
- **Nessus**: https://docs.tenable.com/
- **OpenVAS**: https://www.openvas.org/
- **Suricata**: https://suricata.io/

---

## 🚀 작업 시작

이제 Phase 1 작업을 시작하세요.

첫 번째 단계: **웹서치로 NIST Post-Quantum Cryptography 표준 조사**

```
검색 키워드: "NIST post quantum cryptography selected algorithms 2024"
```

사이버보안의 미래를 위해! 🔐🛡️

---

<div align="center">

**Phase 1 of 4**

Security Data Format Standard

🔐 弘益人間 - Benefit All Humanity 🛡️

</div>
