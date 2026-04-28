# WIA-ENE-037 — Phase 2: API

> Seabed Resource canonical Phase 2 specification per the WIA Standards four-Phase architecture.

> Domain: 해저 자원 — 해저 광물 자원 · ISA · 망간단괴 · 환경 보호.

## A.1 Scope

This Phase covers the canonical api layer of the WIA-ENE-037 standard. It composes with the Phase 3 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

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

## 1. 개요

### 1.1 목적

WIA-ENE-037 해저 자원 개발 표준은 전 세계 해저 광물 자원의 탐사 및 개발을 위한 포괄적인 프레임워크를 제공합니다. 본 표준은 다금속 단괴, 해저 열수광상, 코발트 각, 메탄 하이드레이트 등의 해저 자원 개발에 대한 데이터 형식, 환경 보호 프로토콜, 안전 지침, 국제 규제 준수 요구사항을 정의합니다.

### 1.2 적용 범위

본 표준은 다음 영역에 적용됩니다:

- **탐사 활동**: 해저 지질 조사, 자원 매장량 평가
- **채굴 운영**: 다금속 단괴, 열수광상, 코발트 각 채취
- **메탄 하이드레이트**: 가스 하이드레이트 개발
- **환경 영향 평가**: 해양 생태계 보호 및 모니터링
- **ROV/AUV 운영**: 원격/자율 잠수정 작업
- **국제 규제**: 국제해저기구(ISA) 규정 준수
- **심해 작업**: 200m 이상 심도 작업
- **복원 및 모니터링**: 채굴 후 해양 환경 복원

### 1.3 핵심 원칙

1. **환경 최우선**: 해양 생태계 보호 및 생물다양성 보존
2. **예방적 접근**: 환경 영향 사전 평가 및 최소화
3. **투명성**: 운영 데이터 공개 및 국제 협력
4. **지속 가능성**: 장기적 해양 건강 유지
5. **기술 혁신**: 친환경 채굴 기술 개발

---


## 5. 환경 영향 평가 및 모니터링

### 5.1 환경 기준선 조사

```json
{
  "baselineStudy": {
    "studyId": "EIA-CCZ-2024-001",
    "studyPeriod": {
      "startDate": "2023-01-15",
      "endDate": "2025-01-14",
      "duration": { "value": 2, "unit": "years" }
    },
    "surveyArea": {
      "value": 75000,
      "unit": "km²"
    },
    "parameters": {
      "physicalOceanography": {
        "temperature": true,
        "salinity": true,
        "currents": true,
        "turbidity": true
      },
      "geochemistry": {
        "sedimentComposition": true,
        "porewaterChemistry": true,
        "metalConcentrations": true,
        "organicCarbon": true
      },
      "biology": {
        "megafauna": {
          "surveyed": true,
          "speciesIdentified": 245,
          "density": { "value": 0.8, "unit": "individuals/m²" },
          "biomass": { "value": 2.3, "unit": "g/m²" }
        },
        "macrofauna": {
          "surveyed": true,
          "speciesIdentified": 412,
          "density": { "value": 156, "unit": "individuals/m²" }
        },
        "meiofauna": {
          "surveyed": true,
          "density": { "value": 1250, "unit": "individuals/10cm²" }
        },
        "microbes": {
          "surveyed": true,
          "diversity": "high"
        }
      },
      "ecosystemFunctions": {
        "primaryProductivity": true,
        "bioturbation": true,
        "nutrientCycling": true,
        "carbonSequestration": true
      }
    }
  }
}
```

### 5.2 환경 영향 모니터링

#### 5.2.1 퇴적물 플룸 (Sediment Plume)

```json
{
  "sedimentPlume": {
    "monitoringId": "PLU-2025-12-25-001",
    "timestamp": "2025-12-25T14:30:00Z",
    "plumeType": "collector_plume",
    "measurements": {
      "turbidity": {
        "background": { "value": 0.02, "unit": "FTU" },
        "nearField": { "value": 8.5, "unit": "FTU" },
        "farField": { "value": 0.15, "unit": "FTU" }
      },
      "suspendedSediment": {
        "concentration": { "value": 250, "unit": "mg/L" },
        "settlingRate": { "value": 1.2, "unit": "mm/s" }
      },
      "plumeExtent": {
        "length": { "value": 5.2, "unit": "km" },
        "width": { "value": 1.8, "unit": "km" },
        "height": { "value": 12, "unit": "meters" }
      },
      "dispersalPattern": {
        "direction": 245,
        "currentSpeed": { "value": 0.05, "unit": "m/s" }
      }
    },
    "exceedanceEvents": {
      "turbidityThreshold": { "value": 10, "unit": "FTU" },
      "exceeded": false,
      "duration": { "value": 0, "unit": "hours" }
    }
  }
}
```

#### 5.2.2 해양 생물 모니터링

```json
{
  "biologicalMonitoring": {
    "monitoringId": "BIO-2025-12-001",
    "reportingPeriod": {
      "startDate": "2025-12-01",
      "endDate": "2025-12-31"
    },
    "impactAssessment": {
      "directImpact": {
        "areaCovered": { "value": 10.5, "unit": "km²" },
        "habitatLoss": { "value": 100, "unit": "percent" },
        "recoveryTime": { "value": 50, "unit": "years" }
      },
      "indirectImpact": {
        "sedimentationArea": { "value": 125, "unit": "km²" },
        "burialDepth": { "value": 2.5, "unit": "mm" },
        "mortalityRate": { "value": 15, "unit": "percent" }
      }
    },
    "biodiversity": {
      "megafaunaAbundance": {
        "miningArea": { "value": 0.1, "unit": "individuals/m²" },
        "referenceArea": { "value": 0.8, "unit": "individuals/m²" },
        "reduction": { "value": 87.5, "unit": "percent" }
      },
      "endemicSpecies": {
        "identified": 23,
        "threatened": 5,
        "protectionMeasures": "avoidance_zones"
      }
    },
    "noiseImpact": {
      "sourceLevel": { "value": 185, "unit": "dB re 1 µPa @ 1m" },
      "frequency": { "value": 500, "unit": "Hz" },
      "impactRadius": { "value": 2.5, "unit": "km" }
    }
  }
}
```

### 5.3 환경 관리 계획 (EMP)

```
핵심 완화 조치:

1. 공간 관리
   ✓ 보존 참조 구역 (30% of license area)
   ✓ 영향 참조 구역
   ✓ 보존 구역 (APEIs)

2. 시간 관리
   ✓ 계절적 제한 (산란기 회피)
   ✓ 연속 채굴 시간 제한
   ✓ 회복 기간 설정

3. 기술적 완화
   ✓ 퇴적물 재배치 시스템
   ✓ 집광기 최적화 (선택적 수집)
   ✓ 실시간 모니터링 시스템

4. 적응적 관리
   ✓ 모니터링 결과 기반 조정
   ✓ 환경 임계값 설정
   ✓ 긴급 중단 프로토콜
```

---


## 9. 데이터 보고 및 투명성

### 9.1 연례 보고서

```json
{
  "annualReport": {
    "reportId": "AR-2025-WIA-SB-CCZ-001",
    "reportingYear": 2025,
    "operatorId": "OP-DEEPSEA-01",
    "licenseNumber": "ISA-EXP-2025-001",
    "submissionDate": "2026-03-31",
    "summary": {
      "explorationActivities": {
        "rovDives": 48,
        "auvMissions": 24,
        "areaSurveyed": { "value": 1250, "unit": "km²" },
        "samplesCollected": 2450
      },
      "resourceAssessment": {
        "estimatedResource": {
          "value": 125000000,
          "unit": "tonnes_nodules"
        },
        "averageGrade": {
          "nickel": { "value": 1.28, "unit": "percent" },
          "copper": { "value": 1.15, "unit": "percent" },
          "cobalt": { "value": 0.22, "unit": "percent" }
        }
      },
      "environmentalMonitoring": {
        "baselineStations": 35,
        "biodiversitySurveys": 12,
        "waterQualityMeasurements": 2850,
        "newSpeciesIdentified": 8
      },
      "expenditure": {
        "exploration": { "value": 25000000, "currency": "USD" },
        "environmental": { "value": 8000000, "currency": "USD" },
        "research": { "value": 5000000, "currency": "USD" }
      }
    }
  }
}
```

### 9.2 실시간 데이터 공유

```
공개 데이터:
✓ 운영 위치 (1시간 지연)
✓ 환경 모니터링 데이터
✓ 생산 통계 (월간)
✓ 환경 사고 보고

기밀 데이터:
✗ 상세 자원 평가
✗ 채굴 기술 상세
✗ 경제성 분석
```

---


## 부록

### A. 용어집

| 용어 | 영문 | 정의 |
|------|------|------|
| 다금속 단괴 | Polymetallic Nodules | 해저 표면의 망간 함유 광물 덩어리 |
| 해저 열수광상 | Seafloor Massive Sulfides | 해저 열수 분출공 주변의 황화물 광상 |
| 코발트 각 | Cobalt-Rich Crusts | 해산 표면의 코발트 함유 피각 |
| 메탄 하이드레이트 | Methane Hydrate | 저온 고압의 얼음 형태 가스 |
| 국제해저기구 | ISA | International Seabed Authority |
| 원격조종잠수정 | ROV | Remotely Operated Vehicle |
| 자율잠수정 | AUV | Autonomous Underwater Vehicle |
| 배타적 경제수역 | EEZ | Exclusive Economic Zone |

### B. 메탄 하이드레이트 안정성 곡선

```
온도-압력 안정 영역:

Depth (m)  |  Temp (°C)  |  Pressure (bar)  |  State
-----------|-------------|------------------|--------
   500     |     4       |      50          |  Stable
  1000     |     3       |     100          |  Stable
  2000     |     2       |     200          |  Stable
  3000     |     2       |     300          |  Stable

불안정 조건:
- 온도 상승 → 해리
- 압력 감소 → 해리
- 메탄 방출 위험
```

### C. 환경 임계값 매트릭스

| 파라미터 | 기준선 | 경고 | 위험 | 긴급 중단 |
|---------|--------|------|------|----------|
| 탁도 (FTU) | < 0.05 | 5-10 | 10-20 | > 20 |
| 퇴적률 (mm/day) | < 0.1 | 1-5 | 5-10 | > 10 |
| 용존산소 (mg/L) | > 6 | 4-6 | 2-4 | < 2 |
| pH | 7.8-8.2 | 7.5-7.8 | 7.0-7.5 | < 7.0 |
| 소음 (dB) | < 120 | 120-150 | 150-180 | > 180 |

---

**버전 이력**

| 버전 | 날짜 | 변경 내용 |
|------|------|-----------|
| 1.0.0 | 2025-12-25 | 초기 버전 발행 |

---

**弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

*지속 가능한 해저 자원 개발을 통해 인류의 번영과 해양 생태계 보호를 동시에 실현합니다* 🌊


---

---

## Z.1 Audit transport and observability hooks (Phase 2)

Every Phase 2 envelope SHOULD emit a structured log line at the
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
`2` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 2)

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

## Z.3 Capabilities discovery and SemVer (Phase 2)

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 2)

Phase 2 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 2)

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

## Z.6 Supply-chain envelope per SLSA (Phase 2)

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
