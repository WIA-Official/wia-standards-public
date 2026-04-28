# WIA-ENE-037 — Phase 3: PROTOCOL

> Seabed Resource canonical Phase 3 specification per the WIA Standards four-Phase architecture.

> Domain: 해저 자원 — 해저 광물 자원 · ISA · 망간단괴 · 환경 보호.

## A.1 Scope

This Phase covers the canonical protocol layer of the WIA-ENE-037 standard. It composes with the Phase 4 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

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

## 2. 해저 자원 분류 체계

### 2.1 자원 유형별 분류

#### 2.1.1 다금속 단괴 (Polymetallic Nodules)

| 특성 | 설명 | 매장 위치 | 주요 금속 | 상업 가치 |
|------|------|----------|----------|----------|
| 형태 | 감자 모양의 광물 덩어리 | 심해 평원 (4,000-6,000m) | Mn, Ni, Cu, Co | 높음 |
| 크기 | 2-15 cm 직경 | 태평양, 인도양 | REE (희토류) | 매우 높음 |
| 밀도 | 10-60 kg/m² | 클라리온-클리퍼턴 구역 | Fe, Zn | 중간 |
| 성장률 | 1-10 mm/백만년 | 페루 분지 | Mo, Li | 높음 |

**주요 채굴 구역**:
- **CCZ**: Clarion-Clipperton Zone (태평양)
- **Indian Ocean**: 중앙 인도양 분지
- **Peru Basin**: 페루 남동부 해역

**채굴 방법**:
```
해저 집광 시스템:
- 집광 로봇 (Collector Robot)
- 수직 이송 파이프 (Riser Pipe)
- 모선 처리 시스템
- 퇴적물 재배치 시스템
```

#### 2.1.2 해저 열수광상 (Seafloor Massive Sulfides, SMS)

| 특성 | 설명 | 매장 위치 | 주요 금속 | 품위 |
|------|------|----------|----------|------|
| 형태 | 화산 활동으로 형성된 광상 | 중앙 해령 (1,500-4,000m) | Cu, Zn, Pb | 5-10% Cu |
| 크기 | 수만-수백만 톤 | 열수 분출공 주변 | Au, Ag | 1-20 g/t Au |
| 온도 | 200-400°C (활성 분출공) | 태평양, 대서양 | Fe, S | 10-30% Zn |
| 수명 | 수천-수만 년 | 아시아-태평양 해역 | Co, Se | 0.1-1% Co |

**주요 탐사 구역**:
- **Mid-Atlantic Ridge**: 대서양 중앙 해령
- **East Pacific Rise**: 동태평양 해융기
- **Manus Basin**: 파푸아뉴기니 해역
- **Okinawa Trough**: 오키나와 트로프

**채굴 기술**:
```
절삭 및 이송:
- 해저 절삭기 (Seafloor Cutter)
- 크러셔 시스템
- 슬러리 펌프
- 수중 저장 탱크
```

#### 2.1.3 코발트 각 (Cobalt-Rich Crusts)

| 특성 | 설명 | 매장 위치 | 주요 금속 | 두께 |
|------|------|----------|----------|------|
| 형태 | 해저 암반 표면의 피복층 | 해산 정상부 (800-2,500m) | Co, Mn | 1-25 cm |
| 성장률 | 1-6 mm/백만년 | 태평양 해산군 | Ni, Cu | 평균 4 cm |
| 밀도 | 경질 피각 | 하와이 제도 | REE, Pt | 최대 260 mm |
| 품위 | Co 0.8-1.5% | 마샬 제도 | Te, Bi | 고밀도 |

**채굴 방법**:
```
표면 박리 시스템:
- 로봇 암 절삭기
- 고압 워터젯
- 진공 흡입 시스템
- 정밀 제어 시스템
```

#### 2.1.4 메탄 하이드레이트 (Methane Hydrates)

| 특성 | 설명 | 매장 위치 | 에너지 잠재력 | 위험도 |
|------|------|----------|-------------|--------|
| 형태 | 얼음 형태의 가스 하이드레이트 | 대륙붕 (200-2,000m) | 1 m³ → 164 m³ CH₄ | 높음 |
| 온도 | < 10°C | 영구동토층 | 천연가스의 2배 | 지질학적 불안정 |
| 압력 | > 50 bar | 일본 해구 | 전세계 매장량: 10,000+ Gt C | 메탄 방출 위험 |
| 안정성 | 온도/압력 의존적 | 울릉분지 | 청정 에너지원 | 환경 영향 |

**개발 기술**:
```
생산 방법:
- 감압법 (Depressurization)
- 열자극법 (Thermal Stimulation)
- 억제제 주입법 (Inhibitor Injection)
- CO₂-CH₄ 교환법
```

---


## 6. ROV/AUV 운영

### 6.1 ROV (Remotely Operated Vehicle)

```json
{
  "rovOperation": {
    "rovId": "ROV-DS-6000-01",
    "rovType": "work_class_rov",
    "specifications": {
      "maxDepth": { "value": 6000, "unit": "meters" },
      "payload": { "value": 350, "unit": "kg" },
      "thrusters": 8,
      "power": { "value": 150, "unit": "kW" },
      "umbilicalLength": { "value": 7000, "unit": "meters" }
    },
    "mission": {
      "missionId": "MIS-2025-12-25-001",
      "missionType": "sample_collection",
      "startTime": "2025-12-25T06:00:00Z",
      "endTime": "2025-12-25T14:30:00Z",
      "duration": { "value": 8.5, "unit": "hours" },
      "maxDepth": { "value": 4650, "unit": "meters" }
    },
    "tasks": [
      {
        "taskType": "nodule_sampling",
        "samplesCollected": 35,
        "successRate": { "value": 100, "unit": "percent" }
      },
      {
        "taskType": "sediment_coring",
        "coresCollected": 12,
        "coreDepth": { "value": 50, "unit": "cm" }
      },
      {
        "taskType": "video_survey",
        "trackLength": { "value": 8.5, "unit": "km" },
        "videoQuality": "4K_UHD"
      },
      {
        "taskType": "environmental_monitoring",
        "measurements": [
          "temperature",
          "turbidity",
          "dissolved_oxygen",
          "pH"
        ]
      }
    ]
  }
}
```

### 6.2 AUV (Autonomous Underwater Vehicle)

```json
{
  "auvOperation": {
    "auvId": "AUV-MAP-01",
    "auvType": "survey_auv",
    "specifications": {
      "maxDepth": { "value": 6000, "unit": "meters" },
      "speed": { "value": 1.5, "unit": "m/s" },
      "endurance": { "value": 24, "unit": "hours" },
      "range": { "value": 130, "unit": "km" },
      "sensors": [
        "multibeam_sonar",
        "side_scan_sonar",
        "sub_bottom_profiler",
        "magnetometer",
        "CTD",
        "cameras"
      ]
    },
    "mission": {
      "missionId": "MIS-AUV-2025-12-24-001",
      "missionType": "bathymetric_mapping",
      "surveyPattern": "lawnmower",
      "lineSpacing": { "value": 200, "unit": "meters" },
      "altitude": { "value": 50, "unit": "meters" },
      "coverageArea": { "value": 25, "unit": "km²" },
      "dataQuality": "high_resolution"
    }
  }
}
```

---


## 10. 안전 프로토콜

### 10.1 심해 작업 안전

```
안전 체크리스트:

□ 날씨 조건 확인 (최대 풍속: 25 knots)
□ 해상 상태 (최대 유의파고: 3.5m)
□ 동적 위치 유지 시스템 (DP) 테스트
□ 비상 절단 시스템 (EDS) 확인
□ 통신 시스템 이중화
□ 비상 대응 팀 대기
□ 해저 장비 상태 점검
□ 전원 백업 시스템 확인
```

### 10.2 환경 비상 대응

```json
{
  "emergencyResponse": {
    "incidentType": "sediment_plume_exceedance",
    "triggerLevel": {
      "parameter": "turbidity",
      "threshold": { "value": 10, "unit": "FTU" },
      "exceedance": { "value": 15.5, "unit": "FTU" }
    },
    "responseActions": [
      {
        "action": "immediate_operation_suspension",
        "timestamp": "2025-12-25T10:15:00Z",
        "duration": { "value": 4, "unit": "hours" }
      },
      {
        "action": "enhanced_monitoring",
        "frequency": { "value": 15, "unit": "minutes" }
      },
      {
        "action": "regulator_notification",
        "notifiedAuthority": "ISA",
        "notificationTime": "2025-12-25T10:30:00Z"
      },
      {
        "action": "investigation",
        "rootCause": "collector_track_speed_exceeded",
        "correctiveAction": "speed_reduction_to_0.3m/s"
      }
    ],
    "resumptionCriteria": {
      "turbidityBelow": { "value": 5, "unit": "FTU" },
      "monitoringPeriod": { "value": 2, "unit": "hours" },
      "regulatorApproval": true
    }
  }
}
```

---

---

## Z.1 Audit transport and observability hooks (Phase 3)

Every Phase 3 envelope SHOULD emit a structured log line at the
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
`3` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 3)

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

## Z.3 Capabilities discovery and SemVer (Phase 3)

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 3)

Phase 3 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 3)

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

## Z.6 Supply-chain envelope per SLSA (Phase 3)

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
