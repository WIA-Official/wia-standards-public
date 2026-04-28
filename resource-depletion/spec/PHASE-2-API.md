# WIA-ENE-039 — Phase 2: API

> Resource Depletion canonical Phase 2 specification per the WIA Standards four-Phase architecture.

> Domain: 자원 고갈 — 자원 한계 · 순환경제 · 디커플링 · 자원 발자국.

## A.1 Scope

This Phase covers the canonical api layer of the WIA-ENE-039 standard. It composes with the Phase 3 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

## A.2 Normative references

- ISO 14040:2006 / 14044:2006 LCA
- ISO 14045:2012 (Eco-efficiency)
- ISO 14046:2014 (Water footprint)
- ISO 14067:2018 (Carbon footprint of products)
- Global Footprint Network National Accounts
- UNEP IRP Global Resources Outlook 2024
- EU Eurostat Material Flow Accounts methodology
- OECD Material Flows and Resource Productivity
- World Resources Institute Aqueduct Water Risk Atlas

## 목차

1. [개요](#1-개요)
2. [적용 범위](#2-적용-범위)
3. [용어 정의](#3-용어-정의)
4. [자원 분류 체계](#4-자원-분류-체계)
5. [데이터 모델](#5-데이터-모델)
6. [고갈 평가 방법론](#6-고갈-평가-방법론)
7. [조기 경보 시스템](#7-조기-경보-시스템)
8. [대응 전략](#8-대응-전략)
9. [순환 경제 통합](#9-순환-경제-통합)
10. [모니터링 및 보고](#10-모니터링-및-보고)
11. [성과 지표 (KPI)](#11-성과-지표-kpi)
12. [통합 및 상호운용성](#12-통합-및-상호운용성)
13. [보안 및 데이터 거버넌스](#13-보안-및-데이터-거버넌스)

---


## 4. 자원 분류 체계

### 4.1 자원 카테고리 (8가지)

| 분류 코드 | 자원 유형 | 주요 자원 | 재생 가능성 |
|----------|----------|----------|------------|
| RD-01 | 화석 연료 | 석유, 가스, 석탄 | 비재생 |
| RD-02 | 금속 | 구리, 철, 알루미늄, 희토류 | 재활용 가능 |
| RD-03 | 비금속 광물 | 인광석, 칼륨, 석회석 | 비재생 |
| RD-04 | 수자원 | 담수, 지하수 | 재생 가능 |
| RD-05 | 토양 | 경작지 표토 | 재생 속도 느림 |
| RD-06 | 생물 자원 | 목재, 수산 자원 | 재생 가능 |
| RD-07 | 전략 자원 | 리튬, 코발트, 희토류 | 제한적 재활용 |
| RD-08 | 기타 자원 | 모래, 자갈 | 재생 불가 |

### 4.2 고갈 위험도 등급

- **Level 1 - 안전**: R/P > 100년
- **Level 2 - 주의**: 50년 < R/P < 100년
- **Level 3 - 경고**: 25년 < R/P < 50년
- **Level 4 - 위험**: 10년 < R/P < 25년
- **Level 5 - 심각**: R/P < 10년

### 4.3 전략적 중요도

```
임계 자원 (Critical Resources):
├── 에너지 전환: 리튬, 코발트, 희토류
├── 식량 안보: 인광석, 칼륨
├── 산업 기반: 구리, 철, 알루미늄
└── 생존 필수: 담수, 경작지
```

---


## 8. 대응 전략

### 8.1 효율성 개선 (Efficiency)

#### 8.1.1 생산 효율

- **목표**: 채굴 회수율 향상
- **방법**:
  - 첨단 채굴 기술 도입
  - 정밀 탐사 및 매핑
  - 잔류 자원 회수
- **목표치**: 회수율 10% 향상

#### 8.1.2 사용 효율

- **목표**: 제품당 자원 투입 감소
- **방법**:
  - 경량화 설계
  - 공정 최적화
  - 폐기물 최소화
- **목표치**: 물질 강도 20% 감소

### 8.2 재활용 및 순환 (Recycling)

#### 8.2.1 재활용률 목표

| 자원 | 현재 | 2030 목표 | 2050 목표 |
|------|------|----------|----------|
| 철 | 70% | 85% | 95% |
| 알루미늄 | 60% | 80% | 90% |
| 구리 | 50% | 70% | 85% |
| 희토류 | 10% | 40% | 70% |
| 플라스틱 | 9% | 50% | 80% |

#### 8.2.2 도시 광산 (Urban Mining)

- 전자폐기물 회수
- 건축 폐기물 재활용
- 산업 폐기물 재처리

### 8.3 대체 개발 (Substitution)

#### 8.3.1 소재 대체

- **고갈 자원**: 희토류 영구자석
- **대체재**: 페라이트 자석, 전환 유도 모터
- **적용 분야**: 전기차, 풍력 터빈

#### 8.3.2 에너지 전환

- **화석 연료** → **재생 에너지**
- **리튬 이온 배터리** → **나트륨 이온, 고체 전해질**

### 8.4 비축 관리 (Strategic Reserves)

#### 8.4.1 비축 대상

- 에너지: 원유, 천연가스
- 전략 광물: 희토류, 텅스텐, 코발트
- 식량: 곡물, 비료 (인광석)

#### 8.4.2 비축 규모

- **기준**: 연간 소비량의 90일분
- **비상 방출 기준**: 가격 50% 급등 또는 공급 중단

### 8.5 탐사 및 개발 (Exploration)

#### 8.5.1 신규 매장지 탐사

- 심해 광물 자원
- 극지 자원
- 소행성 광업 (장기)

#### 8.5.2 기술 혁신

- AI 기반 탐사
- 저품위 광석 처리
- 바이오 채굴 (Biomining)

---


## 12. 통합 및 상호운용성

### 12.1 WIA 생태계 통합

#### 12.1.1 연계 표준

- **WIA-ENE-001 (Climate)**: 자원 채굴 탄소 배출 연계
- **WIA-ENE-022 (Waste Management)**: 폐기물 재활용 연계
- **WIA-ENE-023 (Recycling)**: 순환 경제 데이터 통합
- **WIA-BLOCKCHAIN**: 자원 공급망 추적
- **WIA-AI**: 고갈 예측 AI 모델

#### 12.1.2 API 엔드포인트

```
GET    /api/v1/resources/{id}/status        # 자원 현황 조회
GET    /api/v1/resources/{id}/forecast      # 고갈 예측 조회
POST   /api/v1/resources/{id}/production    # 생산 데이터 등록
GET    /api/v1/alerts/active                # 활성 경보 조회
POST   /api/v1/strategies                   # 대응 전략 등록
GET    /api/v1/analytics/dashboard          # KPI 대시보드
```

### 12.2 데이터 표준

- **포맷**: JSON, CSV, Parquet
- **단위**: SI 단위계 (톤, m³, kWh)
- **좌표계**: WGS84
- **시간**: ISO 8601 (UTC)

### 12.3 국제 표준 준수

- **ISO 14046**: 수자원 발자국
- **ISO 14044**: 전과정 평가 (LCA)
- **ISO 20400**: 지속가능한 조달
- **CRIRSCO**: 광물 매장량 보고 국제 기준

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
with `wia.standard.slug` = `resource-depletion` and `wia.standard.phase` =
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
`/.well-known/wia-resource-depletion-capabilities` enumerating per-endpoint
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

---

## Z.1 Audit transport and observability hooks (Phase 2 (variant 1))

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
with `wia.standard.slug` = `resource-depletion` and `wia.standard.phase` =
`2` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 2 (variant 1))

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

## Z.3 Capabilities discovery and SemVer (Phase 2 (variant 1))

Hosts SHOULD publish a capabilities document at
`/.well-known/wia-resource-depletion-capabilities` enumerating per-endpoint
optionality. Clients MUST treat unsupported capabilities as absent
rather than as an error condition; a client that needs a capability
the host does not advertise MUST surface a clear configuration
error rather than silently degrade. Hosts moving from one minor
version to the next MUST publish the change in the host's release
notes with the per-capability migration window per IETF RFC 8594
(Sunset header) + RFC 9745 (Deprecation header) + RFC 9651
(Structured Field Values) so machine consumers can plan migration
without waiting for human-channel notification.

## Z.4 Privacy envelope per per-jurisdiction law (Phase 2 (variant 1))

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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 2 (variant 1))

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

## Z.6 Supply-chain envelope per SLSA (Phase 2 (variant 1))

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
