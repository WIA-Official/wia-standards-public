# WIA-DIGITAL-EXECUTOR

디지털 유언집행인 표준 (Digital Executor Standard)

## 개요

WIA-DIGITAL-EXECUTOR는 디지털 유언장의 지시사항을 실행하는 유언집행인의 역할, 권한, 책임을 정의하는 표준입니다.

## 주요 기능

- **임명과 권한**: 디지털 유언집행인 지정 및 권한 부여
- **자격증명 접근**: 안전한 계정 접근 방법
- **지시 실행**: 디지털 유언장 지시사항 수행
- **플랫폼 통신**: 각 플랫폼과의 공식 절차
- **보고와 책임**: 진행 상황 보고 및 기록
- **인계와 완료**: 임무 완료 및 권한 종료

## 구조

```
digital-executor/
├── README.md
├── spec/
│   └── DIGITAL-EXECUTOR-v1.0.md
├── simulator/
│   └── index.html
└── ebook/
    ├── en/
    │   ├── index.html
    │   └── chapter-01.html ~ chapter-08.html
    └── ko/
        ├── index.html
        └── chapter-01.html ~ chapter-08.html
```

## 관련 표준

- WIA-DIGITAL-WILL: 디지털 유언장 표준
- WIA-DIGITAL-FUNERAL: 디지털 장례 표준
- WIA-DIGITAL-MEMORIAL: 디지털 추모 표준
- WIA-DIGITAL-ERASURE: 디지털 소멸권 표준

## 라이선스

WIA Standard License v1.0

---

**홍익인간 (弘益人間)** - Benefit All Humanity 🌍

## Conformance Tiers

| Tier      | Scope                                                      | Audit cadence |
|-----------|------------------------------------------------------------|---------------|
| Surface   | data formats accepted; no formal audit                     | self-attested |
| Verified  | annual third-party audit against PHASE documents           | annual        |
| Anchored  | continuous evidence package per Annex G; SBOM signed       | continuous    |

Implementations declare their tier in the OpenAPI document via the
`x-wia-conformance-tier` extension field.

## Reference Implementations

- TypeScript SDK: `api/typescript/`
- Schemas: JSON Schema 2020-12 in `spec/PHASE-1-DATA-FORMAT.md`
- Conformance vectors: `tests/phase-vectors/`

## Open Governance

Issues and proposals are tracked at github.com/WIA-Official/wia-standards
issues with the `digital-executor` label. The WIA Standards working group reviews open
issues at the start of every minor release cycle and publishes the
resulting decision log alongside the release notes.

弘益人間 (Hongik Ingan) — Benefit All Humanity
