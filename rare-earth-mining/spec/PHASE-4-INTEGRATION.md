# WIA-ENE-041 — Phase 4: INTEGRATION

> Rare Earth Mining canonical Phase 4 specification per the WIA Standards four-Phase architecture.

> Domain: 희토류 채굴 — 희토류 자원 · 광산 안전 · 환경 영향 평가 · 분리 정제.

## A.1 Scope

This Phase covers the canonical integration layer of the WIA-ENE-041 standard. It composes with the Phase 1 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

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

## 2. 표준의 목적

### 2.1 핵심 목표

1. **환경 보호**: 채굴로 인한 환경 파괴 최소화 및 복원
2. **안전 관리**: 방사성 물질의 안전한 처리 및 보관
3. **공급망 보안**: 전략 자원의 안정적 공급 확보
4. **투명성 확보**: 채굴부터 최종 제품까지 완전한 추적성
5. **순환경제**: 재활용을 통한 2차 자원 확보

### 2.2 기대 효과

- **환경**: 채굴 지역 환경 복원율 90% 이상 달성
- **경제**: 글로벌 희토류 시장 $150억 (2030년 예상)
- **자원**: 재활용을 통한 공급량 30% 증대
- **안보**: 공급망 다변화로 전략 자원 안정성 확보

---


## 6. 광상 유형

### 6.1 바스트네사이트 (Bastnäsite)

#### 특성
```yaml
화학식: (Ce,La)CO₃F
품위: 5-10% TREO
희토류 구성: LREE 주도 (Ce 50%, La 30%, Nd 12%)
토륨 함량: 낮음 (<0.1%)
주요 광산: Mountain Pass (미국), Bayan Obo (중국)
```

#### 장점
- 토륨 함량 낮아 방사성 폐기물 최소
- 경제성 우수
- 대규모 매장량

#### 단점
- 중희토류 함량 낮음
- 대형 노천 채굴 필요 (환경 영향)

### 6.2 모나자이트 (Monazite)

#### 특성
```yaml
화학식: (Ce,La,Nd,Th)PO₄
품위: 3-12% TREO
희토류 구성: LREE (Ce, La, Nd 주)
토륨 함량: 높음 (3-12%)
주요 산지: 인도, 브라질, 호주 (해변 사광상)
```

#### 장점
- 네오디뮴 함량 높음
- 해변 사광상으로 채굴 용이
- 부산물로 토륨 회수 가능

#### 단점
- **방사성 폐기물 대량 발생** (토륨, 우라늄)
- 환경 규제 엄격
- 분리 공정 복잡

### 6.3 이온흡착형 점토 (Ion-Adsorption Clay)

#### 특성
```yaml
형태: 점토 광물에 이온 형태로 흡착
품위: 0.05-0.2% TREO (매우 낮음)
희토류 구성: HREE 주도 (Dy, Tb, Y 풍부)
방사성: 거의 없음
주요 산지: 중국 남부 (광둥, 장시, 푸젠)
```

#### 장점
- **중희토류 주 공급원** (Dy, Tb, Y)
- 원위치 침출(In-Situ Leaching) 가능
- 방사성 폐기물 없음

#### 단점
- 품위 극히 낮음 (대량 처리 필요)
- 암모늄 침출로 심각한 수질 오염
- 산림 파괴 (표토 제거)

---


## 10. 공급망 보안

### 10.1 전략적 중요성

#### 글로벌 공급망 현황 (2024)

```
생산 (광산):
  중국:     60% (170,000 톤/년)
  미국:     15% (43,000 톤/년)
  미얀마:    8% (23,000 톤/년)
  호주:     7% (20,000 톤/년)
  베트남:   5% (14,000 톤/년)
  기타:     5%

분리/정제:
  중국:     85% (절대 우위)
  에스토니아: 5%
  말레이시아: 4%
  미국:     3%
  기타:     3%

소비 (최종 제품):
  중국:     40%
  일본:     15%
  미국:     12%
  EU:       10%
  한국:     8%
  기타:     15%
```

#### 공급망 위험

```
⚠️ 주요 위험 요소:
1. 지정학적 긴장 (중국-미국 무역 분쟁)
2. 수출 쿼터 및 제한 (중국 2010-2015 사례)
3. 환경 규제 강화 (채굴 중단)
4. 가격 조작 (시장 지배력 남용)
5. 단일 공급원 의존 (SPOF)
```

### 10.2 공급망 다변화 전략

#### Tier 1: 1차 공급원

```
목표: 자체 광산 및 정제 능력 확보

주요 프로젝트:
- 미국: Mountain Pass (MP Materials)
- 호주: Mt Weld (Lynas Rare Earths)
- 캐나다: Nechalacho (Vital Metals)
- 브라질: Araxá (CBMM)
```

#### Tier 2: 재활용 (Urban Mining)

```
e-waste에서 희토류 회수:
- 하드디스크: Nd 자석 (1-2g/HDD)
- 형광등: Y, Eu 형광체
- 촉매: La, Ce 회수
- 배터리: La-Ni 배터리

경제성:
- 1차 채굴 대비 30-50% 저렴
- CO₂ 배출 70% 감소
- 방사성 폐기물 없음
```

#### Tier 3: 대체 재료 연구

```
자석:
- Ferrite (페라이트) 자석 개발
- Mn-Al 자석 (Dy-free)

형광체:
- 무희토류 LED (InGaN 기반)

촉매:
- Pt, Pd 기반 촉매 대체
```

### 10.3 블록체인 추적

```json
{
  "batchId": "REE-2025-MP-0012345",
  "origin": {
    "mine": "Mountain Pass",
    "country": "USA",
    "gps": {"lat": 35.4887, "lon": -115.5470},
    "extractionDate": "2025-11-15"
  },
  "composition": {
    "Nd2O3": "12.8%",
    "Pr6O11": "3.5%",
    "Dy2O3": "0.12%",
    "TREO": "65.3%"
  },
  "processing": [
    {
      "facility": "MP Materials Processing",
      "step": "SEPARATION",
      "date": "2025-11-20",
      "certification": "ISO14001"
    },
    {
      "facility": "Neo Performance Materials",
      "step": "METAL_REDUCTION",
      "date": "2025-11-25",
      "certification": "ISO9001"
    }
  ],
  "radioactive": {
    "thorium_ppm": 85,
    "uranium_ppm": 12,
    "radiation_Bq_g": 18,
    "safetyLevel": "LOW"
  },
  "blockchain": {
    "txHash": "0xabc123...",
    "network": "WIA-BLOCKCHAIN",
    "timestamp": "2025-11-26T08:00:00Z"
  }
}
```

---


## 14. 인증 및 준수

### 14.1 국제 표준

#### ISO 14001 (환경경영시스템)
```
범위: 채굴 및 정제 환경 관리
요구사항:
  ✓ 환경 정책 수립
  ✓ 환경 영향 평가
  ✓ 법규 준수
  ✓ 지속적 개선
```

#### ISO 9001 (품질경영시스템)
```
범위: 희토류 제품 품질 관리
요구사항:
  ✓ 품질 방침
  ✓ 공정 관리
  ✓ 검사 및 시험
  ✓ 추적성
```

#### IATF 16949 (자동차 산업)
```
범위: 자동차용 희토류 자석 공급
요구사항:
  ✓ 제품 승인 절차 (PPAP)
  ✓ 공정 능력 (Cpk ≥ 1.67)
  ✓ 추적성 및 리콜 시스템
```

### 14.2 방사성 안전 규제

#### IAEA (국제원자력기구)

```
BSS (Basic Safety Standards):
  - 작업자 선량 한도: 20 mSv/년 (평균 5년)
  - 일반인 선량 한도: 1 mSv/년
  - 방사성 폐기물 분류 및 처분

Transport Regulations:
  - UN2912: Radioactive material, low specific activity
  - 포장 요구사항 (Type A, Type B)
  - 라벨 및 표시
```

#### 국가별 규제

```
미국 (NRC):
  - 10 CFR Part 20: 방사선 안전 기준
  - 40 CFR Part 190: 환경 방사선 기준
  - 광미 저장 시설 허가

중국 (MEE):
  - GB 18871-2002: 전리방사선 방호 기준
  - 희토류 산업 오염물 배출 기준

호주 (ARPANSA):
  - Radiation Protection and Nuclear Safety Act
  - 광산 방사선 안전 규정
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
with `wia.standard.slug` = `rare-earth-mining` and `wia.standard.phase` =
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
with `wia.standard.slug` = `rare-earth-mining` and `wia.standard.phase` =
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
