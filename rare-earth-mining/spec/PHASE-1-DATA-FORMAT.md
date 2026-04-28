# WIA-ENE-041 — Phase 1: DATA-FORMAT

> Rare Earth Mining canonical Phase 1 specification per the WIA Standards four-Phase architecture.

> Domain: 희토류 채굴 — 희토류 자원 · 광산 안전 · 환경 영향 평가 · 분리 정제.

## A.1 Scope

This Phase covers the canonical data-format layer of the WIA-ENE-041 standard. It composes with the Phase 2 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

## A.2 Normative references

- ISO 14001:2015 EMS
- ISO 14040:2006 / 14044:2006 LCA
- ICMM Mining Principles 2024
- IRMA Standard for Responsible Mining v2.0 2024
- Towards Sustainable Mining (TSM) Protocol
- Equator Principles 4 (2020)
- OECD Due Diligence for Conflict-Free Mineral Supply Chains
- EU Critical Raw Materials Act (Reg 2024/1252)
- USGS Mineral Commodity Summaries 2024

## # WIA-ENE-041: 희토류 채굴 표준 💎

> **Version:** 1.0.0
> **발행일:** 2025-12-25
> **상태:** Active
> **弘益人間 (홍익인간) - 널리 인간을 이롭게 하라**

---


## 3. 적용 범위

### 3.1 대상 원소

이 표준은 17개 희토류 원소에 적용됩니다:

#### 경희토류 (Light REE, LREE)
| 원소 | 기호 | 원자번호 | 주요 용도 |
|------|------|----------|-----------|
| 란타넘 | La | 57 | 촉매, 배터리, 광학 유리 |
| 세륨 | Ce | 58 | 촉매, 연마재, 합금 |
| 프라세오디뮴 | Pr | 59 | 자석, 합금, 착색제 |
| 네오디뮴 | Nd | 60 | 영구자석 (NdFeB), 레이저 |
| 프로메튬 | Pm | 61 | 방사성 동위원소 (인공) |
| 사마륨 | Sm | 62 | 자석, 촉매, 핵반응 제어 |
| 유로퓸 | Eu | 63 | 형광체 (LED, TV), 위조 방지 |

#### 중희토류 (Heavy REE, HREE)
| 원소 | 기호 | 원자번호 | 주요 용도 |
|------|------|----------|-----------|
| 가돌리늄 | Gd | 64 | MRI 조영제, 자석 |
| 테르븀 | Tb | 65 | 형광체, 자석 (고온용) |
| 디스프로슘 | Dy | 66 | 영구자석 (고온용), 레이저 |
| 홀뮴 | Ho | 67 | 레이저, 자석 |
| 어븀 | Er | 68 | 광섬유, 레이저, 착색제 |
| 툴륨 | Tm | 69 | 의료용 X선, 레이저 |
| 이터븀 | Yb | 70 | 레이저, 합금, 스테인리스강 |
| 루테튬 | Lu | 71 | 촉매, PET 스캔, 합금 |

#### 준희토류 (Associated)
| 원소 | 기호 | 원자번호 | 주요 용도 |
|------|------|----------|-----------|
| 스칸듐 | Sc | 21 | 항공 합금, LED, 연료전지 |
| 이트륨 | Y | 39 | LED, 레이저, 초전도체 |

### 3.2 대상 광상

- **바스트네사이트(Bastnäsite)**: 탄산염 광물 (세계 주요 광상)
- **모나자이트(Monazite)**: 인산염 광물 (토륨 함유 주의)
- **제노타임(Xenotime)**: 중희토류 주 공급원
- **이온흡착형 점토(Ion-Adsorption Clay)**: 중국 남부 특수 광상

### 3.3 제외 대상

- 군사용 희토류 채굴 (별도 보안 표준 적용)
- 해저 채굴 (기술 개발 중, 향후 표준 예정)

---


## 7. 채굴 방법

### 7.1 노천 채굴 (Open-Pit Mining)

#### 적용: 바스트네사이트, 제노타임 광상

```
Process:
1. 표토 제거 (Overburden Removal)
   - 깊이: 10-100m
   - 폐석량: 광석의 5-20배

2. 채광 (Ore Extraction)
   - 발파 → 굴삭 → 운반
   - 광석 품위: 5-10% TREO

3. 일차 선광 (Primary Beneficiation)
   - 파쇄 → 분쇄 → 자력/부유 선광
   - 정광 품위: 50-70% TREO
```

#### 환경 영향
```
- 토지 훼손: 1 톤 REO 생산 → 2,000 m² 토지 파괴
- 폐석 발생: 1 톤 REO → 200-400 톤 폐석
- 복원 필수: 채굴 종료 후 지형 복원, 식생 회복
```

### 7.2 원위치 침출 (In-Situ Leaching, ISL)

#### 적용: 이온흡착형 점토 광상

```
Process:
1. 탐사 시추 (Exploration Drilling)
   - 광체 확인 및 품위 측정
   - 침출 구역 설계

2. 침출액 주입 (Lixiviant Injection)
   - 침출제: 황산암모늄 (NH₄)₂SO₄ 3-5%
   - 주입 속도: 10-20 L/m²/day
   - 침출 시간: 30-90일

3. 모액 회수 (Pregnant Leach Solution, PLS)
   - REE 농도: 200-500 ppm
   - 침전제: 옥살산 (C₂H₂O₄)

4. 희토류 침전
   - 희토류 옥살산염 → 소성 → REO
```

#### 환경 위험
```
⚠️ 심각한 환경 문제:
- 암모늄 오염: 지하수 질산염 농도 급증
- 토양 산성화: pH 4-5로 저하
- 식생 고사: 침출 지역 식물 대량 사멸
- 중금속 유출: Al, Mn, Fe 동반 용출
```

### 7.3 사광 채굴 (Placer Mining)

#### 적용: 모나자이트 해변 사광상

```
Process:
1. 준설 (Dredging)
   - 해변 모래 수집
   - 처리량: 1,000-5,000 톤/일

2. 중력 선광 (Gravity Separation)
   - 스파이럴 농축기
   - 자력 선별
   - 정전 선별

3. 정광 회수
   - 모나자이트 정광: 50-60% TREO
   - 부산물: 일메나이트, 지르콘, 루타일
```

---


## 11. 데이터 형식 표준

### 11.1 광산 등록

```json
{
  "mineId": "REE-MINE-2025-USA-001",
  "registrationDate": "2025-12-25T00:00:00Z",
  "mine": {
    "name": "Mountain Pass Rare Earth Mine",
    "operator": "MP Materials",
    "country": "USA",
    "state": "California",
    "location": {
      "lat": 35.4887,
      "lon": -115.5470,
      "elevation_m": 1340
    },
    "mineType": "OPEN_PIT",
    "oreType": "BASTNASITE",
    "capacity_tons_per_year": 40000,
    "treoGrade_percent": 7.8
  },
  "composition": {
    "La2O3_percent": 33.2,
    "Ce2O3_percent": 49.1,
    "Pr6O11_percent": 4.3,
    "Nd2O3_percent": 12.0,
    "Sm2O3_percent": 0.8,
    "Eu2O3_percent": 0.1,
    "Gd2O3_percent": 0.2,
    "other_percent": 0.3
  },
  "radioactive": {
    "thorium_ppm": 150,
    "uranium_ppm": 20,
    "classification": "LOW_LEVEL"
  },
  "environmental": {
    "waterUsage_m3_per_ton": 8.5,
    "landDisturbance_hectares": 485,
    "restorationPlan": true,
    "restorationFund_USD": 25000000
  },
  "certifications": ["ISO14001", "ISO9001", "OHSAS18001"]
}
```

### 11.2 생산 배치 기록

```json
{
  "batchId": "REE-BATCH-2025-MP-0012345",
  "productionDate": "2025-12-20",
  "mineId": "REE-MINE-2025-USA-001",
  "processing": {
    "facilityId": "REE-PROC-USA-MP-001",
    "facilityName": "MP Materials Processing Facility",
    "location": {
      "city": "Mountain Pass",
      "state": "CA",
      "country": "USA"
    }
  },
  "stages": [
    {
      "stage": "EXTRACTION",
      "method": "OPEN_PIT_MINING",
      "date": "2025-12-10",
      "rawOre_tons": 1000,
      "treoGrade_percent": 7.8
    },
    {
      "stage": "BENEFICIATION",
      "method": "FLOTATION",
      "date": "2025-12-12",
      "concentrate_tons": 150,
      "treoGrade_percent": 52.0,
      "recovery_percent": 85
    },
    {
      "stage": "ROASTING",
      "temperature_C": 750,
      "date": "2025-12-14",
      "reo_tons": 120
    },
    {
      "stage": "ACID_LEACHING",
      "acid": "H2SO4",
      "concentration_percent": 85,
      "date": "2025-12-15",
      "leach_efficiency_percent": 92
    },
    {
      "stage": "SOLVENT_EXTRACTION",
      "extractant": "P507",
      "stages": 28,
      "date": "2025-12-18",
      "separation": {
        "La2O3_kg": 3500,
        "Ce2O3_kg": 5200,
        "Pr6O11_kg": 450,
        "Nd2O3_kg": 1280,
        "others_kg": 70
      }
    }
  ],
  "products": [
    {
      "element": "Nd",
      "compound": "Nd2O3",
      "quantity_kg": 1280,
      "purity_percent": 99.5,
      "pricePerKg_USD": 95,
      "totalValue_USD": 121600
    },
    {
      "element": "Pr",
      "compound": "Pr6O11",
      "quantity_kg": 450,
      "purity_percent": 99.0,
      "pricePerKg_USD": 85,
      "totalValue_USD": 38250
    }
  ],
  "waste": {
    "tailings_tons": 850,
    "radioactiveWaste_kg": 125,
    "radioactivity_Bq": 2500,
    "wasteWater_m3": 8500,
    "treatment": "NEUTRALIZATION_AND_SETTLING"
  },
  "environmentalImpact": {
    "co2Emissions_tons": 12.5,
    "waterUsage_m3": 8500,
    "energyConsumption_MWh": 85
  }
}
```

---

---

## Z.1 Audit transport and observability hooks (Phase 1)

Every Phase 1 envelope SHOULD emit a structured log line at the
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
with `wia.standard.slug` = `rare-earth-mining` and `wia.standard.phase` =
`1` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 1)

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

## Z.3 Capabilities discovery and SemVer (Phase 1)

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-rare-earth-mining-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 1)

Phase 1 envelopes that carry personal data MUST honour the
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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 1)

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

## Z.6 Supply-chain envelope per SLSA (Phase 1)

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
