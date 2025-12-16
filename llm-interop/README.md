# WIA-LLM-INTEROP

> AI들의 연동 표준
>
> 홍익인간 (弘益人間) - Benefit All Humanity

## 개요

WIA-LLM-INTEROP은 여러 AI/LLM들이 서로 **호환**되고 **협업**할 수 있게 하는 표준입니다.

```
수많은 LLM들 (GPT, Claude, Gemini, Llama, 특화 AI들...)
                    ↓
            서로 말이 안 통함
                    ↓
               표준이 필요
                    ↓
            WIA-LLM-INTEROP
```

## 표준 구성

| 표준 | 설명 | 상태 |
|------|------|------|
| **WIA-LLM-CAPABILITY** | AI 능력 선언 표준 | 프롬프트 준비됨 |
| **WIA-LLM-MESSAGE** | AI간 메시지 형식 | 프롬프트 준비됨 |
| **WIA-LLM-FEDERATION** | AI 연합 프로토콜 | 프롬프트 준비됨 |

## 구현 방법

각 표준의 프롬프트를 새로운 Claude Code 세션에 복사/붙여넣기:

```bash
llm-interop/
└── prompts/
    ├── WIA-LLM-CAPABILITY-PROMPT.md   # 복사 → 새 세션
    ├── WIA-LLM-MESSAGE-PROMPT.md      # 복사 → 새 세션
    └── WIA-LLM-FEDERATION-PROMPT.md   # 복사 → 새 세션
```

## 왜 지금인가?

| 영역 | 현재 상황 | WIA |
|------|-----------|-----|
| 양자 컴퓨팅 | NIST 진행중 | **스펙 + 구현 완료** |
| LLM 연합 | **아무도 안함** | **표준 준비중** |

이 영역은 **빈 땅**입니다.
- NIST: 아직 안 건드림
- IEEE: 아직 안 건드림
- ISO: 아직 안 건드림

WIA가 먼저 표준을 만들면 → **de facto 표준**

## 비전

```
[2024]
각 회사가 각자의 API로...

[2025-2026]
"의료 AI가 법률 AI한테 물어봐야 하는데 형식이 안 맞음"
"표준 있어요?"

[미래]
WIA-LLM-INTEROP v1.0
모든 AI가 이 표준으로 대화
```

## 참고

- WIA-QUANTUM 구현체: `/quantum/api/typescript/` (모범 사례)
- 철학: 홍익인간 (弘益人間)

---

**WIA - World Certification Industry Association**
