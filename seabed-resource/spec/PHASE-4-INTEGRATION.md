# WIA-ENE-037 — Phase 4: INTEGRATION

> Seabed Resource canonical Phase 4 specification per the WIA Standards four-Phase architecture.

> Domain: 해저 자원 — 해저 광물 자원 · ISA · 망간단괴 · 환경 보호.

## A.1 Scope

This Phase covers the canonical integration layer of the WIA-ENE-037 standard. It composes with the Phase 1 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

## A.2 Normative references

- International Seabed Authority (ISA) Mining Code (draft)
- UNCLOS 1982 Part XI
- 1994 Implementation Agreement Annex
- ISA Regulations on Prospecting and Exploration for Polymetallic Nodules
- ISA Regulations on Polymetallic Sulphides
- ISA Regulations on Cobalt-rich Ferromanganese Crusts
- Convention on Biological Diversity (CBD)
- MARPOL 73/78 Annex V (Garbage)
- London Protocol 1996 (Dumping)

## 3. 탐사 및 평가 프로세스

### 3.1 탐사 단계

#### 3.1.1 원격 탐사 (Remote Sensing)

```
위성 및 선박 탐사:
1. 해저 지형 매핑
   - 다중빔 음향측심기 (Multibeam Echosounder)
   - 측면주사 음파탐지기 (Side-scan Sonar)
   - 해저 지진파 탐사

2. 지구화학 분석
   - 수중 화학 센서
   - 해수 샘플링
   - 플룸 추적 (Plume Tracking)

3. 자력/중력 탐사
   - 자기장 이상 탐지
   - 중력 이상 매핑
```

#### 3.1.2 ROV/AUV 조사

```json
{
  "rovSurvey": {
    "vehicleType": "ROV-6000",
    "maxDepth": {
      "value": 6000,
      "unit": "meters"
    },
    "equipments": [
      "HD cameras",
      "Manipulator arms",
      "Sample collectors",
      "CTD sensors",
      "Magnetometer"
    ],
    "surveyArea": {
      "value": 25,
      "unit": "km²"
    },
    "sampleCollection": {
      "nodules": 150,
      "sediment": 30,
      "waterSamples": 50
    }
  }
}
```

### 3.2 자원 평가

#### 3.2.1 매장량 추정

```
Grade × Tonnage 모델:
- 자원량 = 면적 × 밀도 × 두께
- 품위 분석 (Assay)
- 회수율 추정
- 경제성 평가
```

**예시: 다금속 단괴**
```json
{
  "resourceEstimate": {
    "surveyArea": {
      "value": 5000,
      "unit": "km²"
    },
    "noduleDensity": {
      "value": 15,
      "unit": "kg/m²"
    },
    "totalResource": {
      "value": 75000000,
      "unit": "tonnes"
    },
    "metalGrades": {
      "nickel": { "value": 1.3, "unit": "percent" },
      "copper": { "value": 1.1, "unit": "percent" },
      "cobalt": { "value": 0.24, "unit": "percent" },
      "manganese": { "value": 29.0, "unit": "percent" }
    },
    "containedMetal": {
      "nickel": { "value": 975000, "unit": "tonnes" },
      "copper": { "value": 825000, "unit": "tonnes" },
      "cobalt": { "value": 180000, "unit": "tonnes" }
    }
  }
}
```

---


## 7. 국제 규제 및 라이선스

### 7.1 ISA (International Seabed Authority) 규정

#### 7.1.1 라이선스 유형

| 라이선스 유형 | 목적 | 기간 | 면적 제한 | 요구사항 |
|------------|------|------|----------|----------|
| **탐사 라이선스** | 자원 조사 및 평가 | 15년 (연장 가능) | 75,000 km² → 25,000 km² | EIA, 연례 보고서 |
| **개발 라이선스** | 상업적 채굴 준비 | 협상 기반 | TBD | 환경 관리 계획 |
| **상업 라이선스** | 광물 채굴 | 30년 (최대) | 협상 기반 | 로열티 지불, 모니터링 |

#### 7.1.2 환경 규정

```
ISA 환경 기준:

1. 환경 영향 평가 (EIA)
   ✓ 기준선 조사 (최소 2년)
   ✓ 영향 예측 모델링
   ✓ 완화 조치 계획
   ✓ 모니터링 프로그램

2. 지역 환경 관리 계획 (REMP)
   ✓ 보존 구역 설정
   ✓ 환경 임계값
   ✓ 긴급 대응 계획

3. 보고 의무
   ✓ 연례 보고서
   ✓ 환경 모니터링 데이터
   ✓ 사고 즉시 보고
```

### 7.2 국가별 규제

| 국가/지역 | 관할 구역 | 규제 기관 | 주요 요구사항 |
|----------|----------|----------|--------------|
| 🇯🇵 일본 | EEZ (배타적 경제수역) | METI, MOE | 환경 영향 평가, 지역 협의 |
| 🇰🇷 한국 | EEZ | 해양수산부 | 해양환경영향평가, 허가 |
| 🇳🇿 뉴질랜드 | EEZ | EPA | 해양 동의, 환경 모니터링 |
| 🇵🇬 파푸아뉴기니 | 영해 | MRA | 채굴 계약, 로열티 |

---


## 11. API 엔드포인트

### 11.1 주요 API

```
기본 URL: https://api.wia.org/ene-037/v1

인증:
  Header: X-API-Key: {your_api_key}
  Header: X-Operator-ID: {operator_id}
```

#### 라이선스 관리
```
POST   /licenses              # 라이선스 등록
GET    /licenses/{id}         # 라이선스 조회
GET    /licenses              # 라이선스 목록
PUT    /licenses/{id}         # 라이선스 업데이트
```

#### 탐사 데이터
```
POST   /exploration/surveys   # 탐사 데이터 제출
GET    /exploration/surveys   # 탐사 기록 조회
POST   /exploration/samples   # 샘플 데이터 등록
GET    /exploration/resources # 자원 평가 조회
```

#### 채굴 운영
```
POST   /mining/daily-production    # 일일 생산 보고
GET    /mining/production-history  # 생산 기록 조회
POST   /mining/operations         # 운영 데이터 제출
GET    /mining/statistics         # 통계 조회
```

#### 환경 모니터링
```
POST   /environmental/baseline     # 기준선 데이터
POST   /environmental/monitoring   # 모니터링 데이터
GET    /environmental/reports      # 환경 보고서
POST   /environmental/incidents    # 환경 사고 보고
GET    /environmental/compliance   # 준수 현황
```

#### ROV/AUV 운영
```
POST   /vehicles/missions          # 미션 등록
GET    /vehicles/missions/{id}     # 미션 조회
POST   /vehicles/telemetry         # 텔레메트리 데이터
GET    /vehicles/status            # 차량 상태
```

---

---

## Z.1 Audit transport and observability hooks (Phase 4)

Every Phase 4 envelope SHOULD emit a structured log line at the
host's audit transport: timestamp per RFC 3339, host identifier,
tenant identifier, envelope class, envelope identifier, operation
outcome, and a W3C Trace Context `traceparent` propagated end-to-end
so a single operation can be reconstructed across hosts. Phase 2
surfaces this trace identifier as the `X-WIA-Trace-Id` response
header. Phase 3 protocol exchanges propagate the trace identifier
inside the exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (Splunk, Elastic, Sumo Logic,
Wazuh, Microsoft Sentinel) per OpenTelemetry semantic conventions,
with `wia.standard.slug` = `seabed-resource` and `wia.standard.phase` =
`4` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 4)

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 Sec 5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations. The composition also lets the operator's SIEM
correlate per-tenant audit records across multiple standards
without per-standard schema-mapping work.

## Z.3 Capabilities discovery and SemVer (Phase 4)

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-seabed-resource-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 4)

Phase 4 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California
CPRA per Cal. Civ. Code Sec 1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Subject-rights endpoints (access, rectification,
erasure, portability, restriction, objection) compose with
WIA-OMNI-API Sec 5 subject-rights surface and need not be
re-implemented per-standard.

## Z.5 DR / continuity envelope per ISO 22301 (Phase 4)

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-exit
envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity.

## Z.6 Supply-chain envelope per SLSA (Phase 4)

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per SPDX 2.3 / 3.0 (per ISO/IEC 5962 + Linux Foundation SPDX)
or CycloneDX 1.6 (per OWASP Foundation). The SBOM enumerates every
direct + transitive dependency with the per-component name +
version + licence + supplier + per-component hash + per-component
PURL (Package URL per package-url spec) + per-component CPE
(Common Platform Enumeration per NIST). Supply-chain attestation
follows in-toto per CNCF in-toto + SLSA (Supply-chain Levels for
Software Artifacts) per OpenSSF SLSA Framework — typically targeting
SLSA Level 3 for hosted production deployments.

弘益人間 — Benefit All Humanity.

---

## Z.1 Audit transport and observability hooks (Phase 4 (variant 1))

Every Phase 4 envelope SHOULD emit a structured log line at the
host's audit transport: timestamp per RFC 3339, host identifier,
tenant identifier, envelope class, envelope identifier, operation
outcome, and a W3C Trace Context `traceparent` propagated end-to-end
so a single operation can be reconstructed across hosts. Phase 2
surfaces this trace identifier as the `X-WIA-Trace-Id` response
header. Phase 3 protocol exchanges propagate the trace identifier
inside the exchange envelope so that a federation crossing remains
correlatable end-to-end. Phase 4 integrators consume the audit
stream into the operator's SIEM (Splunk, Elastic, Sumo Logic,
Wazuh, Microsoft Sentinel) per OpenTelemetry semantic conventions,
with `wia.standard.slug` = `seabed-resource` and `wia.standard.phase` =
`4` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 4 (variant 1))

This Phase composes with WIA-OMNI-API for credential storage,
WIA-AIR-SHIELD for runtime trust list, WIA-SOCIAL Phase 3 Sec 5 for
federation handshake, WIA-INTENT for workload intent declaration,
and (where personal data is processed) WIA Secure Enclave for
sealed-data envelopes. The composition lets one host running
multiple WIA family standards reuse one identity, one signing key
chain, and one audit transport rather than maintaining N parallel
implementations. The composition also lets the operator's SIEM
correlate per-tenant audit records across multiple standards
without per-standard schema-mapping work.

## Z.3 Capabilities discovery and SemVer (Phase 4 (variant 1))

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-seabed-resource-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 4 (variant 1))

Phase 4 envelopes that carry personal data MUST honour the
operator's per-jurisdiction privacy law (EU GDPR per Regulation
2016/679; UK GDPR per UK Data Protection Act 2018; California
CPRA per Cal. Civ. Code Sec 1798.100; Brazil LGPD per Lei 13.709/2018;
Canada PIPEDA per S.C. 2000 c.5; Korea PIPA per 개인정보 보호법;
Japan APPI per 個人情報の保護に関する法律; Australia Privacy Act
1988 per Cth) including data-minimisation, purpose-limitation,
storage-limitation, integrity + confidentiality, and accountability
principles. Subject-rights endpoints (access, rectification,
erasure, portability, restriction, objection) compose with
WIA-OMNI-API Sec 5 subject-rights surface and need not be
re-implemented per-standard.

## Z.5 DR / continuity envelope per ISO 22301 (Phase 4 (variant 1))

Hosts running this Phase MUST publish a continuity-of-operations
envelope per ISO 22301:2019 + ISO/IEC 27031 + NIST SP 800-34 Rev 1
covering: per-host RTO (Recovery Time Objective) per the operator's
business-impact analysis; per-host RPO (Recovery Point Objective)
tied to the host's audit-stream replication policy; per-host
backup envelope (per-region cross-replicated immutable backup with
the per-tier retention envelope per the per-jurisdiction record-
retention policy); per-host failover-rehearsal envelope (typically
quarterly per the operator's BC/DR program); per-host vendor-exit
envelope so the operator can migrate the host to an alternate
implementation without losing audit-trail continuity.

## Z.6 Supply-chain envelope per SLSA (Phase 4 (variant 1))

Every host implementation MUST publish a software-bill-of-materials
(SBOM) per SPDX 2.3 / 3.0 (per ISO/IEC 5962 + Linux Foundation SPDX)
or CycloneDX 1.6 (per OWASP Foundation). The SBOM enumerates every
direct + transitive dependency with the per-component name +
version + licence + supplier + per-component hash + per-component
PURL (Package URL per package-url spec) + per-component CPE
(Common Platform Enumeration per NIST). Supply-chain attestation
follows in-toto per CNCF in-toto + SLSA (Supply-chain Levels for
Software Artifacts) per OpenSSF SLSA Framework — typically targeting
SLSA Level 3 for hosted production deployments.

弘益人間 — Benefit All Humanity.
