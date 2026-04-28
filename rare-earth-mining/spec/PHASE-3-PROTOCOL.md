# WIA-ENE-041 — Phase 3: PROTOCOL

> Rare Earth Mining canonical Phase 3 specification per the WIA Standards four-Phase architecture.

> Domain: 희토류 채굴 — 희토류 자원 · 광산 안전 · 환경 영향 평가 · 분리 정제.

## A.1 Scope

This Phase covers the canonical protocol layer of the WIA-ENE-041 standard. It composes with the Phase 4 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

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

## 1. 개요

### 1.1 배경

희토류 원소(Rare Earth Elements, REE)는 현대 첨단 기술의 핵심 재료로, 전 세계적으로 **연간 약 30만 톤**이 채굴되고 있으며, 이 중 **중국이 60%**를 생산합니다.

희토류는 다음 분야에 필수적입니다:
- **영구자석**: 네오디뮴(Nd), 디스프로슘(Dy) - 전기차, 풍력 터빈
- **촉매**: 란타넘(La), 세륨(Ce) - 석유 정제, 배기가스 처리
- **형광체**: 유로퓸(Eu), 테르븀(Tb) - LED, 디스플레이
- **배터리**: 란타넘(La) - 니켈-수소 배터리
- **광학**: 이트륨(Y) - 레이저, 광섬유

### 1.2 문제점

현재 희토류 채굴의 주요 문제점:

1. **환경 오염**: 채굴 및 정제 과정에서 심각한 토양, 수질 오염
2. **방사성 폐기물**: 토륨(Th), 우라늄(U) 함유로 인한 방사성 폐기물 발생
3. **공급망 집중**: 중국 의존도 과도 (60% 생산, 85% 정제)
4. **전략적 중요성**: 군사, 첨단 기술 분야의 필수 자원
5. **가격 변동성**: 지정학적 요인으로 인한 극심한 가격 변동
6. **추적 부재**: 원산지 및 공급망 투명성 부족

### 1.3 WIA-ENE-041의 가치

이 표준은 다음을 제공합니다:

- ✅ **지속가능한 채굴**: 환경 영향 최소화 및 복원
- ✅ **안전한 폐기물 관리**: 방사성 폐기물의 체계적 관리
- ✅ **공급망 투명성**: 블록체인 기반 원산지 추적
- ✅ **재활용 촉진**: 도시광산(e-waste)에서 희토류 회수
- ✅ **글로벌 협력**: 국제 표준과의 호환

---


## 5. 희토류 원소 분류

### 5.1 전략적 중요도 등급

```
CRITICAL (매우 중요):
  - 네오디뮴 (Nd): 영구자석 - 전기차, 풍력
  - 디스프로슘 (Dy): 고온 자석 - 전기차 모터
  - 테르븀 (Tb): 형광체, 자석
  - 유로퓸 (Eu): LED 형광체
  - 이트륨 (Y): LED, 레이저

HIGH (중요):
  - 프라세오디뮴 (Pr): 자석 합금
  - 사마륨 (Sm): SmCo 자석
  - 가돌리늄 (Gd): MRI, 자석

MODERATE (보통):
  - 란타넘 (La): 촉매, 배터리
  - 세륨 (Ce): 촉매, 연마재
  - 어븀 (Er): 광섬유 증폭기
```

### 5.2 가격 변동성 (2024 기준)

| 원소 | 가격 (USD/kg) | 최근 5년 변동폭 | 주요 공급국 |
|------|---------------|----------------|-------------|
| **네오디뮴 (Nd)** | $80-150 | ±300% | 중국 (85%), 호주, 미얀마 |
| **디스프로슘 (Dy)** | $350-500 | ±400% | 중국 (95%), 베트남 |
| **테르븀 (Tb)** | $1,200-1,800 | ±350% | 중국 (95%) |
| **유로퓸 (Eu)** | $800-1,200 | ±250% | 중국 (90%) |
| **이트륨 (Y)** | $10-20 | ±150% | 중국 (70%), 베트남 |
| **세륨 (Ce)** | $5-10 | ±100% | 중국 (60%), 베트남 |

---


## 9. 방사성 폐기물 관리

### 9.1 방사성 물질 현황

#### 토륨 (Thorium, Th-232)

```yaml
주요 광상: 모나자이트 (3-12% ThO₂)
반감기: 14.05억 년 (⍺ 붕괴)
방사선: 알파선 (낮은 투과력, 높은 이온화)
건강 위험: 흡입 시 폐암, 골암 위험
규제: IAEA Transport Category I-III
```

#### 우라늄 (Uranium, U-238/U-235)

```yaml
함량: 모나자이트 0.1-0.5%
반감기: U-238 44.7억 년, U-235 7.04억 년
방사선: 알파선, 감마선
규제: 핵물질 관리 (IAEA)
```

### 9.2 방사성 폐기물 분류

#### Level 1: 저준위 (Low-Level)

```
특성:
- 방사능: <100 Bq/g
- 반감기: <30년
- 대상: 바스트네사이트 부산물, 제노타임

처리:
- 격리 매립 (Isolated Disposal)
- 차폐: 콘크리트 또는 납 차폐
- 모니터링: 연 2회 이상
```

#### Level 2: 중준위 (Intermediate-Level)

```
특성:
- 방사능: 100-10,000 Bq/g
- 토륨 함량: 0.5-3%
- 대상: 모나자이트 정광, 침전 슬러지

처리:
- 시멘트 고화 (Cementation)
- 스테인리스 드럼 밀봉
- 중간 저장 시설 (30-50년)
- 최종 처분장 격리
```

#### Level 3: 고준위 (High-Level)

```
특성:
- 방사능: >10,000 Bq/g
- 토륨 농축물 (>10% ThO₂)
- 우라늄 농축물

처리:
- 유리화 (Vitrification) 또는 시너지 고화
- 다중 차폐 (스테인리스 + 납 + 콘크리트)
- 깊은 지층 처분 (Deep Geological Repository)
- 1,000년 이상 격리
```

### 9.3 작업자 안전

#### 방사선 안전 기준

```yaml
선량 한도 (IAEA):
  작업자: 20 mSv/년 (평균 5년)
  일반인: 1 mSv/년

개인 보호구 (PPE):
  - 전신 보호복 (Tyvek suit)
  - 호흡 보호구 (PAPR, P100 필터)
  - 이중 장갑
  - 개인선량계 (TLD 또는 OSL)

모니터링:
  - 공기 중 라돈: <400 Bq/m³
  - 표면 오염: <4 Bq/cm² (알파)
  - 개인 선량: 월 단위 측정
```

---


## 13. 추적 및 보고

### 13.1 MRV (Monitoring, Reporting, Verification)

#### Tier 1: 실시간 모니터링

```yaml
광산 모니터링:
  - 일일 생산량 (톤/일)
  - TREO 품위 (%)
  - 개별 원소 함량
  - 방사성 폐기물 발생량
  - 용수 사용량
  - 에너지 소비량

업데이트 주기: 실시간 (1분)
접근: https://dashboard.wia.org/ree-041
```

#### Tier 2: 월간 보고서

```yaml
규제 기관 제출용:
  - 생산량 통계
  - 방사성 폐기물 관리 내역
  - 환경 영향 평가
  - 복원 진척도
  - 안전 사고 기록

형식: PDF + JSON + XML
제출 기한: 매월 10일까지
제출처: 광업국, 환경부, 원자력안전위원회
```

#### Tier 3: 연간 감사

```yaml
제3자 검증:
  - 현장 실사 (광산, 정제 시설)
  - 방사성 안전 감사
  - 환경 복원 검증
  - 공급망 투명성 감사
  - ISO 인증 갱신

감사 기관: SGS, Bureau Veritas, TÜV
주기: 연 1회
```

### 13.2 성과 지표 (KPI)

```yaml
생산 효율:
  채광 효율: >85%
  정광 회수율: >80%
  분리 순도: >99.5%
  금속 수율: >95%

환경 성과:
  용수 재활용률: >70%
  폐기물 재활용: >60%
  토지 복원율: >90% (채굴 종료 5년 내)
  CO₂ 배출 감축: >20% (vs. 2020 기준)

안전 지표:
  방사선 피폭: <10 mSv/년 (작업자)
  환경 방사능: <100 Bq/m² (주변 지역)
  안전 사고율: <1.0 per 200,000 work hours
  환경 사고: 0건
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
with `wia.standard.slug` = `rare-earth-mining` and `wia.standard.phase` =
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

---

## Z.1 Audit transport and observability hooks (Phase 3 (variant 1))

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
with `wia.standard.slug` = `rare-earth-mining` and `wia.standard.phase` =
`3` as required attributes. The audit envelope follows the
canonical W3C Trace Context binary format on the wire when the
host operates over a binary protocol (e.g., gRPC over HTTP/2 or
MQTT 5) and the canonical W3C Trace Context text format when the
host operates over a text protocol (e.g., HTTP/1.1 or REST/JSON).

## Z.2 Cross-standard composition (Phase 3 (variant 1))

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

## Z.3 Capabilities discovery and SemVer (Phase 3 (variant 1))

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

## Z.4 Privacy envelope per per-jurisdiction law (Phase 3 (variant 1))

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

## Z.5 DR / continuity envelope per ISO 22301 (Phase 3 (variant 1))

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

## Z.6 Supply-chain envelope per SLSA (Phase 3 (variant 1))

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
