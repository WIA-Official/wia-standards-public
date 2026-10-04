# WIA-AI-028 — AGI Safety

> AGI 안전 표준 — 폭넓은 추론·계획·도구 사용 능력을 가진 AI 시스템의 안전성 사례·운영 범위·인간 인가·실행 게이트웨이·정지와 봉쇄·적합성 평가

## Scope

WIA-AI-028 v1.0 defines an assurance and control framework for deploying
systems with broad reasoning, planning, learning, tool use, or delegated
execution capabilities. It covers:

- Configuration identification for models, instructions, memory, tools,
  evaluators, and control components.
- Operating envelopes and aggregate resource limits.
- Hazard arguments and evaluation evidence (safety cases).
- Deployment authorization and separation of duties.
- Mediation of tool use, external communication, persistent changes, and
  delegated work through action gateways.
- Suspension, revocation, containment, and reconciliation of uncertain effects.
- Evidence integrity, audit records, privacy, and access control.
- Registration APIs (`/v1/...`), lifecycle events, conformance testing, and
  certification records.

Out of scope for v1.0: a scientific test for the existence of AGI, a universal
acceptable-risk threshold, a training algorithm that guarantees alignment, a
method for proving that internal representations or objectives are benign,
permission to perform otherwise unauthorized activities, and operational
instructions for hazardous restricted activities. Physical deployments
additionally require an appropriate domain safety assessment.

## Conformance levels

| Level | Name | Adds |
|---|---|---|
| L1 | Basic | Explicit boundaries, evaluated controls, independent human authorization, complete action mediation, attributable records |
| L2 | Standard | Separate safety and operations approval; evaluation across declared operating variations |
| L3 | Advanced | Assessment independence, independent adversarial exercises, evidence custody outside the development team |

Conformance applies to an identified deployment configuration and its control
system. It does not establish that a system possesses artificial general
intelligence, has aligned objectives in every circumstance, or is safe outside
its assessed operating envelope.

## Normative references

- RFC 2119, RFC 8174 (requirement keywords)
- RFC 8259 (JSON), RFC 3339 (timestamps)
- JSON Schema Draft 2020-12

## Files

- `spec/WIA-AI-028-v1.0.md` — the full specification (v1.0)
- `ebook/ko/`, `ebook/en/` — the 8-chapter companion guide in Korean and English

## License and publisher

© SmileStory Inc. · WIA Standards
