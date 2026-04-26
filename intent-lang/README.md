# WIA-INTENT

> AI 시대의 프로그래밍 언어
>
> 홍익인간 (弘益人間) - Benefit All Humanity

## 개요

```
현재 프로그래밍:  "어떻게" 할지를 코드로 작성
WIA-INTENT:      "무엇을" 원하는지를 표현
```

## 왜 필요한가?

```
2025: 인간이 코드 작성, AI가 도움
2030: AI가 코드 작성, 인간이 검토
2040: AI가 AI를 위한 코드 작성
2050: 코드라는 개념 자체가 변함

→ 인간이 AI에게 의도를 전달하는 표준 언어가 필요
→ WIA-INTENT
```

## 특징

| 기존 언어 | WIA-INTENT |
|-----------|------------|
| 명령형 (어떻게) | 의도형 (무엇을) |
| 확정적 | 확률적 |
| 고정 코드 | 자기 진화 |
| 인간 가독성 | AI 최적화 |
| 윤리 없음 | 윤리 내장 |

## 예시

```wia-intent
// 기존 Python
// for i in range(10): do_something(i)

// WIA-INTENT
intent ProcessItems {
  want: each item processed
  constraints { order: any }  // AI가 알아서 병렬화
}
```

## 구조

```
intent-lang/
├── spec/
│   └── WIA-INTENT-v1.0.md     # 언어 명세
└── api/
    └── typescript/            # 레퍼런스 구현
        ├── src/
        │   ├── types.ts       # 타입 정의
        │   ├── parser/        # 파서
        │   └── index.ts       # 메인 API
        └── examples/          # 예제
```

## 빈 땅

| 기관 | AI 시대 언어 표준 |
|------|-------------------|
| IEEE | 없음 |
| ISO | 없음 |
| NIST | 없음 |
| **WIA** | **WIA-INTENT v1.0** |

---

**WIA - World Certification Industry Association**

---

## Conformance tiers

WIA conformance for **intent-lang** is evaluated across three tiers, mirrored in every PHASE document under `spec/`:

| Tier | Scope | Audit cadence |
|------|-------|---------------|
| Tier 1 — Self-declared | Internal use, pilot deployments | Annual self-review |
| Tier 2 — Third-party assessed | External partners, B2B integrations | Every 24 months |
| Tier 3 — Accredited | Public-facing or regulated deployments | Every 12 months |

Implementations MUST disclose their conformance tier in the OpenAPI `info.x-wia-tier` extension and on any public certification page. Tier downgrade events MUST be reported to the WIA registry within 30 days of detection. The conformity assessment process for Tier 3 is aligned with ISO/IEC 17065:2012 and depends on the documentary evidence retention policy described in the per-PHASE Annex A under `spec/`.

## Layout

```
standards/intent-lang/
├── README.md            # this document
├── index.html           # human-readable landing page
├── simulator/           # interactive browser-based simulator
├── spec/                # PHASE-1..N normative specifications
├── api/                 # reference TypeScript SDK skeleton
├── cli/                 # POSIX shell client
├── press/               # press kit (article + DALL·E prompts)
└── ebook/{en,ko}/      # eight-chapter ebook editions
```

The PHASE documents under `spec/` are the normative source. Code under `api/`, `cli/`, and `simulator/` is informative reference material that demonstrates the contract; production implementations may diverge as long as they preserve the PHASE contract.

## Open governance

Comments, proposals, and conformance reports are accepted via the GitHub issues tracker on the WIA-Official organization. Major version bumps follow the WIA Standards governance process documented at <https://wiastandards.com/governance>.

---

**弘益人間 · Benefit All Humanity** — © 2026 WIA. Licensed under MIT.
