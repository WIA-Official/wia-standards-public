# WIA-ENE-039 — Phase 3: PROTOCOL

> Resource Depletion canonical Phase 3 specification per the WIA Standards four-Phase architecture.

> Domain: 자원 고갈 — 자원 한계 · 순환경제 · 디커플링 · 자원 발자국.

## A.1 Scope

This Phase covers the canonical protocol layer of the WIA-ENE-039 standard. It composes with the Phase 4 document for cross-Phase integration and inherits cross-standard behaviour from the wider WIA Standards family per the README.md scope statement.

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

## 1. 개요

### 1.1 목적

WIA-ENE-039 자원 고갈 대응 표준은 지구 자원의 고갈을 모니터링하고, 예측하며, 효과적인 대응 전략을 수립하기 위한 국제 표준입니다. 본 표준은 자원 보존, 순환 경제 구축, 지속가능한 소비 패턴 확립을 목표로 합니다.

### 1.2 핵심 원칙

- **예방 우선 (Prevention First)**: 고갈 이전 사전 대응
- **효율적 사용 (Efficient Use)**: 자원 생산성 극대화
- **순환 이용 (Circular Use)**: 재활용 및 재사용 촉진
- **대체 개발 (Substitution)**: 신소재 및 대체재 개발
- **공정한 분배 (Fair Distribution)**: 세대 간, 국가 간 형평성
- **투명성 (Transparency)**: 데이터 공개 및 추적
- **협력 (Collaboration)**: 국제적 공조 체계

### 1.3 적용 대상

- 정부 및 자원 관리 기관
- 광업 및 자원 개발 기업
- 제조업 및 산업체
- 환경 연구 기관
- 국제 기구 및 NGO
- 순환 경제 플랫폼
- 자원 거래소 및 시장

---


## 5. 데이터 모델

### 5.1 자원 현황 정보

```typescript
interface ResourceStatus {
  // 기본 정보
  resourceId: string;
  resourceCode: string;              // RD-01 ~ RD-08
  name: string;
  scientificName?: string;

  // 매장량 정보
  reserves: {
    proven: number;                  // 확인 매장량 (톤)
    probable: number;                // 가능 매장량 (톤)
    potential: number;               // 잠재 매장량 (톤)
    lastUpdated: string;             // 갱신일
    confidence: number;              // 신뢰도 (0-100%)
  };

  // 생산 및 소비
  production: {
    annual: number;                  // 연간 생산량 (톤/년)
    trend: number;                   // 증감률 (%)
    topProducers: {
      country: string;
      quantity: number;
    }[];
  };

  consumption: {
    annual: number;                  // 연간 소비량 (톤/년)
    perCapita: number;               // 1인당 소비량 (kg/년)
    topConsumers: {
      country: string;
      quantity: number;
    }[];
  };

  // 고갈 지표
  depletion: {
    rpRatio: number;                 // R/P 비율 (년)
    depletionRate: number;           // 고갈 속도 (%/년)
    riskLevel: 1 | 2 | 3 | 4 | 5;   // 위험도
    estimatedExhaustion: string;     // 예상 고갈 시점
  };

  // 순환 경제
  circularity: {
    recyclingRate: number;           // 재활용률 (%)
    recoveryRate: number;            // 회수율 (%)
    circularityIndex: number;        // 순환 지수 (0-100)
  };

  // 메타데이터
  metadata: {
    dataSource: string;
    quality: number;                 // 데이터 품질 (0-100)
    verified: boolean;
    lastVerified: string;
  };
}
```

### 5.2 고갈 예측 모델

```typescript
interface DepletionForecast {
  forecastId: string;
  resourceId: string;
  modelType: 'linear' | 'exponential' | 'logistic' | 'monte_carlo';

  // 예측 시나리오
  scenarios: {
    name: string;                    // BAU, Low, Medium, High
    assumptions: {
      productionGrowth: number;      // 생산 증가율 (%/년)
      consumptionGrowth: number;     // 소비 증가율 (%/년)
      recyclingTarget: number;       // 재활용 목표 (%)
      substitutionRate: number;      // 대체율 (%/년)
    };

    results: {
      year: number;
      reserves: number;
      production: number;
      consumption: number;
      rpRatio: number;
    }[];

    exhaustionYear: number;          // 예상 고갈 연도
    confidence: number;              // 신뢰 구간 (%)
  }[];

  // 민감도 분석
  sensitivity: {
    parameter: string;
    impact: number;                  // 영향도
  }[];

  generatedAt: string;
  validUntil: string;
}
```

### 5.3 대응 전략 정보

```typescript
interface MitigationStrategy {
  strategyId: string;
  resourceId: string;
  strategyType: 'efficiency' | 'recycling' | 'substitution' | 'conservation' | 'exploration';

  // 전략 상세
  details: {
    name: string;
    description: string;
    targetYear: number;
    targetMetrics: {
      metric: string;
      baseline: number;
      target: number;
      unit: string;
    }[];
  };

  // 실행 계획
  implementation: {
    phase: string;
    startDate: string;
    endDate: string;
    milestones: {
      date: string;
      description: string;
      status: 'planned' | 'in_progress' | 'completed';
    }[];
  };

  // 효과 예측
  expectedImpact: {
    resourceSaved: number;           // 절감 자원량 (톤/년)
    lifetimeExtension: number;       // 수명 연장 (년)
    costReduction: number;           // 비용 절감 (원/년)
    emissionsReduced: number;        // 배출 감축 (톤 CO2eq)
  };

  // 투자 및 비용
  economics: {
    investmentRequired: number;      // 필요 투자액 (원)
    operatingCost: number;           // 운영 비용 (원/년)
    paybackPeriod: number;           // 회수 기간 (년)
    roi: number;                     // 투자 수익률 (%)
  };

  status: 'proposed' | 'approved' | 'active' | 'completed' | 'cancelled';
}
```

### 5.4 조기 경보 지표

```typescript
interface EarlyWarningIndicator {
  indicatorId: string;
  resourceId: string;

  // 지표 정보
  indicator: {
    name: string;
    type: 'depletion_rate' | 'price_volatility' | 'supply_risk' | 'geopolitical';
    value: number;
    unit: string;
    threshold: {
      warning: number;
      critical: number;
    };
  };

  // 경보 상태
  alert: {
    level: 'normal' | 'caution' | 'warning' | 'critical';
    triggered: boolean;
    triggerDate?: string;
    message: string;
  };

  // 추세 분석
  trend: {
    direction: 'improving' | 'stable' | 'deteriorating';
    velocity: number;                // 변화 속도
    forecast: {
      thirtyDays: number;
      ninetyDays: number;
      oneYear: number;
    };
  };

  timestamp: string;
}
```

---


## 9. 순환 경제 통합

### 9.1 순환 경제 원칙

```
원재료 채취
    ↓
제품 설계 ← 재활용 원료
    ↓           ↑
제조 생산       │
    ↓           │
사용 단계       │
    ↓           │
수거 및 분류    │
    ↓           │
재활용 ─────────┘
```

### 9.2 제품 수명 연장

- **설계 단계**: 모듈화, 수리 가능성
- **사용 단계**: 유지보수, 업그레이드
- **회수 단계**: 리퍼비시, 리매뉴팩처링

### 9.3 산업 공생 (Industrial Symbiosis)

- 한 산업의 폐기물 → 다른 산업의 원료
- 에너지 공유
- 물 재이용

### 9.4 순환율 측정

```
순환율 = (재활용 투입량 + 재사용량) / (총 자원 소비량) × 100
```

**목표**: 2030년 30%, 2050년 50%

---


## 13. 보안 및 데이터 거버넌스

### 13.1 데이터 분류

| 데이터 유형 | 보안 등급 | 암호화 | 공개 범위 |
|------------|----------|--------|----------|
| 공개 통계 | Public | 불필요 | 전체 공개 |
| 기업 생산 데이터 | Confidential | AES-256 | 인가자 |
| 국가 비축량 | Highly Confidential | E2E 암호화 | 정부 기관 |
| 미탐사 지역 정보 | Top Secret | 양자 암호 | 최소 권한 |

### 13.2 접근 통제

- **역할 기반 접근 제어 (RBAC)**
- **다중 인증 (MFA)** 필수
- **접근 로그** 5년 보관
- **정기 권한 검토** (6개월)

### 13.3 데이터 거버넌스

- **데이터 소유권**: 원생성자
- **데이터 품질**: ISO 8000 준수
- **개인정보보호**: GDPR, 개인정보보호법 준수
- **국가 간 데이터 이전**: 적법 절차

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
with `wia.standard.slug` = `resource-depletion` and `wia.standard.phase` =
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
