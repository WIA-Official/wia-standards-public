# WIA-ENE-041 — Phase 2: API

> Rare Earth Mining canonical Phase 2 specification per the WIA Standards four-Phase architecture.

> Domain: 희토류 채굴 — 희토류 자원 · 광산 안전 · 환경 영향 평가 · 분리 정제.

## A.1 Scope

This Phase covers the canonical api layer of the WIA-ENE-041 standard. It composes with the Phase 3 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

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

## 목차

1. [개요](#1-개요)
2. [표준의 목적](#2-표준의-목적)
3. [적용 범위](#3-적용-범위)
4. [용어 정의](#4-용어-정의)
5. [희토류 원소 분류](#5-희토류-원소-분류)
6. [광상 유형](#6-광상-유형)
7. [채굴 방법](#7-채굴-방법)
8. [분리 및 정제 공정](#8-분리-및-정제-공정)
9. [방사성 폐기물 관리](#9-방사성-폐기물-관리)
10. [공급망 보안](#10-공급망-보안)
11. [데이터 형식 표준](#11-데이터-형식-표준)
12. [API 인터페이스](#12-api-인터페이스)
13. [추적 및 보고](#13-추적-및-보고)
14. [인증 및 준수](#14-인증-및-준수)
15. [부록](#15-부록)

---


## 4. 용어 정의

### 4.1 핵심 용어

| 용어 | 정의 |
|------|------|
| **희토류 원소 (REE)** | 란타넘계 원소 15개 + 스칸듐, 이트륨 |
| **경희토류 (LREE)** | La, Ce, Pr, Nd, Pm, Sm, Eu (원자번호 57-63) |
| **중희토류 (HREE)** | Gd, Tb, Dy, Ho, Er, Tm, Yb, Lu (원자번호 64-71) |
| **REO** | Rare Earth Oxide (희토류 산화물) |
| **TREO** | Total Rare Earth Oxide (총 희토류 산화물) |
| **바스트네사이트** | (Ce,La)CO₃F - 경희토류 주 광상 |
| **모나자이트** | (Ce,La,Nd,Th)PO₄ - 토륨 함유 광물 |
| **이온흡착형** | 점토에 이온 상태로 흡착된 희토류 광상 |

### 4.2 광업 용어

| 용어 | 정의 |
|------|------|
| **품위 (Grade)** | 광석 내 희토류 함량 (% TREO) |
| **정광 (Concentrate)** | 선광 후 농축된 광물 |
| **침출 (Leaching)** | 화학 용액으로 금속 추출 |
| **용매 추출 (SX)** | Solvent Extraction - 희토류 분리 |
| **소성 (Calcination)** | 고온 처리로 산화물 생성 |
| **환원 (Reduction)** | 산화물 → 금속 전환 |

---


## 8. 분리 및 정제 공정

### 8.1 희토류 추출 (Extraction)

#### 산 침출 (Acid Leaching)

```
바스트네사이트 처리:
1. 소성 (Roasting)
   - 온도: 600-900°C
   - (Ce,La)CO₃F → (Ce,La)₂O₃ + CO₂ + HF

2. 황산 침출
   - 농도: H₂SO₄ 70-90%
   - 온도: 80-100°C
   - 시간: 2-4시간
   - REO → RE₂(SO₄)₃ (가용성 황산염)

3. 불순물 제거
   - Fe, Al, Ca, Th 침전 제거
   - pH 조정: 1.5-3.0
```

#### 알칼리 침출 (Alkali Leaching)

```
모나자이트 처리:
1. 수산화나트륨 분해
   - NaOH 70% @ 150°C, 3시간
   - (Ce,La,Th)PO₄ + 3NaOH → (Ce,La,Th)(OH)₃ + Na₃PO₄

2. 염산 용해
   - HCl 6M
   - RE(OH)₃ → RECl₃

3. 토륨 분리 ⚠️
   - pH 5.8: ThO₂ 침전
   - 방사성 폐기물 처리 필수
```

### 8.2 용매 추출 (Solvent Extraction, SX)

희토류 원소들은 화학적 성질이 매우 유사하여 **수십 단계의 용매 추출**이 필요합니다.

#### 추출제 (Extractant)

```yaml
P507 (D2EHPA):
  - 가장 일반적인 추출제
  - 유기상: P507 in kerosene (0.5-1M)
  - pH 의존적: 원소별 최적 pH 차이 이용

P204 (HDEHP):
  - 중희토류 분리에 효과적

Cyanex 272:
  - HREE/LREE 분리
```

#### 분리 순서

```
혼합 희토류 용액 (Mixed RE Solution)
  ↓ SX Stage 1: Ce⁴⁺ 분리 (산화 후 추출)
세륨 제거 용액
  ↓ SX Stage 2-10: La, Pr, Nd 순차 분리
  ↓ 각 원소별 5-15 단계 추출
  ↓ SX Stage 11-20: Sm, Eu, Gd 분리
  ↓ SX Stage 21-35: HREE 분리 (Tb, Dy, Ho, Er, Tm, Yb, Lu)
고순도 개별 희토류 용액 (99.9%+)
```

#### 회수 (Stripping)

```
1. 역추출 (Back Extraction)
   - 강산 사용: HCl 4-6M 또는 H₂SO₄ 1-2M
   - 고농도 희토류 용액 획득

2. 침전 (Precipitation)
   - 옥살산: (NH₄)₂C₂O₄ → RE₂(C₂O₄)₃
   - 또는 탄산염: (NH₄)₂CO₃ → RE₂(CO₃)₃

3. 소성 (Calcination)
   - 900-1,100°C
   - RE₂(C₂O₄)₃ → RE₂O₃ + CO + CO₂
```

### 8.3 금속 환원 (Metal Reduction)

#### 전기분해 (Electrolysis)

```
용융염 전기분해:
- 온도: 800-1,000°C
- 전해질: RECl₃ + NaCl/KCl
- 음극: 희토류 금속 석출
- 양극: 염소 가스 발생
- 순도: 99-99.9%
```

#### 금속 열환원 (Metallothermic Reduction)

```
칼슘 환원:
- RE₂O₃ + 3Ca → 2RE + 3CaO
- 온도: 1,100-1,300°C (진공/아르곤)
- 용도: Nd, Dy 금속 생산

플루오라이드 환원:
- REF₃ + 3Ca → RE + 3CaF₂
- 더 높은 순도 (99.9%+)
```

---


## 12. API 인터페이스

### 12.1 RESTful API 엔드포인트

#### 광산 등록

```http
POST /api/v1/mines/register
Content-Type: application/json
Authorization: Bearer {api_key}

Request Body:
{
  "mine": {...},
  "composition": {...},
  "radioactive": {...},
  "environmental": {...}
}

Response (201 Created):
{
  "mineId": "REE-MINE-2025-USA-001",
  "registrationNumber": "US-DOI-REE-2025-1234",
  "certificationUrl": "https://cert.wia.org/ree-041/mines/REE-MINE-2025-USA-001.pdf",
  "trackingUrl": "https://track.wia.org/ree-041/REE-MINE-2025-USA-001"
}
```

#### 배치 생산 기록

```http
POST /api/v1/batches/submit
Content-Type: application/json
Authorization: Bearer {api_key}

Request Body:
{
  "batchId": "REE-BATCH-2025-MP-0012345",
  "mineId": "REE-MINE-2025-USA-001",
  "processing": {...},
  "stages": [...],
  "products": [...]
}

Response (200 OK):
{
  "batchId": "REE-BATCH-2025-MP-0012345",
  "blockchainTx": "0xabc123...",
  "certificateUrl": "https://cert.wia.org/ree-041/batches/REE-BATCH-2025-MP-0012345.pdf",
  "qrCode": "https://api.wia.org/ree-041/qr/REE-BATCH-2025-MP-0012345.png"
}
```

#### 공급망 추적

```http
GET /api/v1/batches/{batchId}/supply-chain
Authorization: Bearer {api_key}

Response (200 OK):
{
  "batchId": "REE-BATCH-2025-MP-0012345",
  "origin": {
    "mine": "Mountain Pass",
    "country": "USA",
    "extractionDate": "2025-12-10"
  },
  "supplyChain": [
    {
      "stage": "MINING",
      "facility": "Mountain Pass Mine",
      "date": "2025-12-10",
      "location": "California, USA"
    },
    {
      "stage": "PROCESSING",
      "facility": "MP Materials Processing",
      "date": "2025-12-15",
      "location": "California, USA"
    },
    {
      "stage": "SEPARATION",
      "facility": "MP Materials SX Plant",
      "date": "2025-12-18",
      "location": "California, USA"
    },
    {
      "stage": "SHIPMENT",
      "destination": "Neo Performance Materials",
      "date": "2025-12-20",
      "location": "Colorado, USA"
    }
  ],
  "currentLocation": {
    "facility": "Neo Performance Materials",
    "address": "13777 E. Arapahoe Rd, Centennial, CO",
    "timestamp": "2025-12-22T10:00:00Z"
  },
  "products": [
    {
      "element": "Nd",
      "form": "Nd2O3",
      "quantity_kg": 1280,
      "purity_percent": 99.5
    }
  ],
  "certifications": ["ISO9001", "ISO14001", "IATF16949"],
  "radioactiveSafety": {
    "level": "LOW",
    "radiation_Bq_g": 18,
    "certified": true
  }
}
```

#### 가격 조회

```http
GET /api/v1/prices/current
Authorization: Bearer {api_key}

Response (200 OK):
{
  "timestamp": "2025-12-25T08:00:00Z",
  "currency": "USD",
  "unit": "kg",
  "prices": [
    {"element": "La", "compound": "La2O3", "price": 3.50, "change_7d": "+2.3%"},
    {"element": "Ce", "compound": "CeO2", "price": 2.80, "change_7d": "-1.5%"},
    {"element": "Pr", "compound": "Pr6O11", "price": 85.00, "change_7d": "+5.2%"},
    {"element": "Nd", "compound": "Nd2O3", "price": 95.00, "change_7d": "+8.7%"},
    {"element": "Dy", "compound": "Dy2O3", "price": 420.00, "change_7d": "+12.3%"},
    {"element": "Tb", "compound": "Tb4O7", "price": 1500.00, "change_7d": "+15.8%"},
    {"element": "Y", "compound": "Y2O3", "price": 12.50, "change_7d": "+3.1%"}
  ],
  "indices": {
    "WIA_REE_Index": 1285.4,
    "change_7d": "+7.2%",
    "change_30d": "+18.5%"
  }
}
```

### 12.2 WebSocket 실시간 가격 피드

```javascript
// 실시간 희토류 가격 모니터링
const ws = new WebSocket('wss://api.wia.org/v1/ree-041/prices/stream');

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  console.log('실시간 가격:', data);
};

// 실시간 데이터 예시
{
  "timestamp": "2025-12-25T08:15:32Z",
  "element": "Nd",
  "compound": "Nd2O3",
  "price_USD_kg": 96.20,
  "change_percent": "+1.3%",
  "volume_kg": 12500,
  "exchange": "Shanghai Rare Earth Exchange"
}
```

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
with `wia.standard.slug` = `rare-earth-mining` and `wia.standard.phase` =
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
