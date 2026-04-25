# WIA Education - Phase 2: API Interface

## Version
- **Version**: 1.0.0
- **Date**: 2025-12-16
- **Status**: Final

## Overview

본 문서는 WIA Education 접근성 표준의 API 인터페이스를 정의합니다.

## Rust SDK

완전한 Rust 구현이 제공됩니다:

**위치**: `api/rust/`

### 주요 컴포넌트

#### 1. 학습 콘텐츠 접근성
- **멀티미디어 자막**: 실시간 자막 생성
- **오디오 설명**: 시각 자료 음성 설명
- **점자 변환**: 텍스트 → 점자
- **수어 영상**: 수어 통역 삽입

#### 2. 평가 접근성
- **시간 조정**: 추가 시험 시간
- **형식 변경**: 대체 평가 형식
- **보조 도구**: 화면 읽기, 음성 입력
- **유연한 제출**: 다양한 제출 방식

#### 3. 학습 관리 시스템 통합
- **LTI 1.3**: Learning Tools Interoperability
- **xAPI**: Experience API
- **SCORM**: Sharable Content Object Reference Model
- **Canvas/Moodle/Blackboard**: LMS 통합

#### 4. WIA 생태계 통합
- **BCI 학습**: 뇌파로 학습 제어
- **AAC 참여**: AAC 디바이스로 수업 참여
- **Voice-Sign**: 음성/수어 강의
- **Smart Wheelchair**: 접근 가능한 교실

## API 구조

```rust
// 교육 접근성 인터페이스
pub trait EducationAccessibility {
    fn apply_accommodations(&mut self, profile: AccessibilityProfile) -> Result<()>;
    fn generate_captions(&self, audio: &[u8]) -> Result<String>;
    fn convert_to_braille(&self, text: &str) -> Result<String>;
}

// 학습 콘텐츠
pub trait LearningContent {
    fn add_audio_description(&mut self, description: &str) -> Result<()>;
    fn insert_sign_language(&mut self, video: &[u8]) -> Result<()>;
    fn adjust_complexity(&mut self, level: ComplexityLevel) -> Result<()>;
}

// 평가 시스템
pub trait Assessment {
    fn extend_time(&mut self, minutes: u32) -> Result<()>;
    fn enable_assistive_tech(&mut self, tools: Vec<AssistiveTool>) -> Result<()>;
    fn alternative_format(&mut self, format: AssessmentFormat) -> Result<()>;
}
```

## 통신 프로토콜

### LTI 1.3
```
POST /lti/launch
- LMS 통합
- 싱글 사인온
- 성적 동기화
```

### xAPI
```
POST /xapi/statements
- 학습 활동 기록
- 진도 추적
- 접근성 사용 로그
```

### REST API
```
GET    /courses/{id}/accessibility
POST   /assessments/{id}/accommodations
PUT    /content/{id}/captions
GET    /students/{id}/profile
```

### WebSocket
```
ws://edu.server/realtime
- 실시간 자막
- 라이브 수어 통역
- 동시 참여 지원
```

## 접근성 기능

### 콘텐츠 변환
- **Captions**: 자동 자막 생성
- **Transcripts**: 전체 스크립트
- **Audio Description**: 시각 자료 설명
- **Sign Language**: 수어 영상 삽입
- **Braille**: 점자 변환

### 평가 조정
- **Extra Time**: 1.5x ~ 2x 시간
- **Breaks**: 휴식 시간 허용
- **Assistive Tech**: 화면 읽기, 음성 입력
- **Alternative Format**: 구술, 프로젝트 등

### UI/UX 조정
- **Font Size**: 글자 크기 조절
- **High Contrast**: 고대비 모드
- **Screen Reader**: 화면 읽기 지원
- **Keyboard Navigation**: 키보드 전용 네비게이션

## 에러 처리

```rust
pub enum EducationError {
    ContentNotAccessible,
    AccommodationFailed,
    LMSIntegrationError,
    CaptionGenerationFailed,
}
```

## 예제

```rust
use wia_education::*;

// 접근성 프로필 적용
let mut course = Course::load("course-123")?;
let profile = AccessibilityProfile::for_visual_impairment();
course.apply_accommodations(profile)?;

// 자막 생성
let captions = course.generate_captions(audio_data)?;

// 평가 조정
let mut assessment = Assessment::load("test-456")?;
assessment.extend_time(30)?; // 30분 추가
assessment.enable_assistive_tech(vec![AssistiveTool::ScreenReader])?;
```

---

**Author**: Yeon Sam-Heum, Ph.D.  
**License**: MIT  
**弘益人間** - Benefit All Humanity

---

## Annex A — Conformance Tier Matrix

WIA conformance for edu is evaluated across three tiers:

| Tier | Scope | Mandatory artifacts | Audit cadence |
|------|-------|--------------------|----------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | OpenAPI 3.0 contract, JSON Schema validation report, security threat model | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Tier 1 artifacts + signed third-party assessor report against this PHASE | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Tier 2 artifacts + WIA accreditation, ISO/IEC 17065:2012 conformity assessment, evidence retention ≥ 7 years | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days.

---

## Annex B — Cross-Walk to International Standards

The PHASE specification reuses or normatively references published standards alongside this document. Implementers SHOULD review the relevant standards alongside this PHASE document; where a conflict exists, the more specific WIA requirement governs unless explicitly superseded by a binding national regulation.

- ISO/IEC 17065:2012 — Conformity assessment — Requirements for bodies certifying products, processes and services
- ISO/IEC 27001:2022 — Information security management systems
- IETF RFC 9457 — Problem Details for HTTP APIs
- IETF RFC 7519 — JSON Web Token (JWT)
- IETF RFC 6749 — The OAuth 2.0 Authorization Framework
- W3C PROV-DM — provenance data model

This cross-walk is informative only. WIA does not republish the referenced documents; readers MUST obtain authoritative copies from the issuing body. Cross-walk entries are reviewed at every minor version of this PHASE.

---

## Annex C — Reference Implementations and Test Vectors

### C.1 Reference Implementations

WIA does not require implementers to use a particular library, but maintains pointers to the canonical reference implementation directories under the WIA-Official GitHub organization:

- `wia-standards/standards/edu/api/` — TypeScript SDK skeleton
- `wia-standards/standards/edu/cli/` — POSIX shell client demonstrating the request/response contract
- `wia-standards/standards/edu/simulator/` — interactive browser-based simulator for the PHASE protocol

Each reference artifact ships with an MIT license and is intended as a starting point, not as a production-grade implementation.

### C.2 Test Vectors

A normative set of request/response pairs covering the schemas defined in this PHASE is published alongside this document. Implementations claiming Tier 2 or Tier 3 conformance MUST pass every published test vector in both serialization (request) and deserialization (response) directions, and MUST publish a signed report identifying which vectors were exercised.

Test vectors are versioned independently of the PHASE document; refer to the `test-vectors/` directory for the active version.

---

## Annex D — Open Questions and Future Work

This PHASE document captures the consensus position at v1.0. The following items are tracked for future minor or major revisions:

1. **Schema versioning policy** — formal MUST/SHOULD rules for backward-compatible vs. breaking changes between minor releases.
2. **Privacy-preserving aggregation** — guidance on differential privacy and secure aggregation patterns for telemetry channels, without prescribing a specific algorithm.
3. **Multilingual error catalogs** — localization strategy for the standard error codes defined in this PHASE.
4. **Long-term retention** — alignment with sectoral retention regulations across jurisdictions.
5. **Sustainability disclosure** — optional fields for energy and emissions reporting tied to operations covered by this PHASE.

Items in this annex are non-normative. Comments and proposals are accepted via the GitHub issues tracker on the WIA-Official organization.

---


## Annex E — Implementation Notes for PHASE-2-API-INTERFACE

The following implementation notes document field experience from pilot
deployments and are non-normative. They are republished here so that early
adopters can read them in context with the rest of PHASE-2-API-INTERFACE.

- **Operational scope** — implementations SHOULD declare their operational
  scope (single-tenant, multi-tenant, federated) in the OpenAPI document so
  that downstream auditors can score the deployment against the correct
  conformance tier in Annex A.
- **Schema evolution** — additive changes (new optional fields, new error
  codes) are non-breaking; renaming or removing fields, even in error
  payloads, MUST trigger a minor version bump.
- **Audit retention** — a 7-year retention window is sufficient to satisfy
  ISO/IEC 17065:2012 audit expectations in most jurisdictions; some
  regulators require longer retention, in which case the deployment policy
  MUST extend the retention window rather than relying on this PHASE's
  defaults.
- **Time synchronization** — sub-second deadlines depend on synchronized
  clocks. NTPv4 with stratum-2 servers is sufficient for most deadlines
  expressed in this PHASE; PTP is recommended for sites that require
  deterministic interlocks.
- **Error budget reporting** — implementations SHOULD publish a monthly
  error-budget summary (latency p95, error rate, violation hours) in the
  format defined by the WIA reporting profile to facilitate cross-vendor
  comparison without exposing tenant-specific data.

These notes are not requirements; they are a reference for field teams
mapping their existing operations onto WIA conformance.



## Annex F — Adoption Roadmap

The adoption roadmap for this PHASE document is non-normative and is intended to set expectations for early implementers about the relative stability of each section.

- **Stable** (sections marked normative with `MUST` / `MUST NOT`) — semantic versioning applies; breaking changes require a major version bump and at minimum 90 days of overlap with the prior major version on all WIA-published reference implementations.
- **Provisional** (sections in this Annex and Annex D) — items are tracked openly and may be promoted to normative status without a major version bump if community feedback supports promotion.
- **Reference** (test vectors, simulator behaviour, the reference TypeScript SDK) — versioned independently of this document so that mistakes in reference material can be corrected without amending the published PHASE document.

Implementers SHOULD subscribe to the WIA Standards GitHub release notifications to track promotions between these tiers. Comments on the roadmap are accepted via the GitHub issues tracker on the WIA-Official organization.
