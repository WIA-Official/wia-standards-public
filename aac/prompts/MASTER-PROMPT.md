# WIA AAC Standard - Master Prompt for Claude Code

---

## 🎯 프로젝트 개요

### 프로젝트명
**WIA AAC Standard** - Open Standard for Augmentative and Alternative Communication

### 목표
전 세계 AAC(보완대체의사소통) 센서의 인터페이스를 표준화하여, 어떤 센서를 사용하든 어떤 소프트웨어와도 호환되는 오픈 표준 구축

### 철학
```
弘益人間 (홍익인간) - 널리 인간을 이롭게

- 오픈소스, MIT License
- 특허 없음, 영원히 무료
- 기존 WIA 생태계 (ISP, WIA Braille, WIA Talk)와 연동
```

---

## 📚 필수 참조 문서

Claude Code는 작업 전 반드시 아래 문서를 읽어야 합니다:

| 문서 | 경로 | 내용 |
|------|------|------|
| AAC 개요 | `/docs/WIA-AAC-OVERVIEW.md` | AAC 표준 전체 구조 |
| ISP 기반 | `/docs/ISP-SCIENTIFIC-FOUNDATION.md` | 수어 코드 체계 |
| WIA Braille 기반 | `/docs/WIA-BRAILLE-SCIENTIFIC-FOUNDATION.md` | 점자 체계 |
| WIA Talk 기반 | `/docs/WIA-TALK-SCIENTIFIC-FOUNDATION.md` | 수어 소통 체계 |

---

## 🏗️ 프로젝트 구조

```
wia-aac-standard/
├── docs/                     ← 기반 문서 (읽기 전용)
├── spec/                     ← 표준 스펙 (Phase별 생성)
├── prompts/                  ← Claude Code 프롬프트
├── examples/                 ← 레퍼런스 구현
│   ├── eye-tracker/
│   ├── switch/
│   ├── muscle-sensor/
│   └── brain-interface/
└── api/                      ← API 구현
    ├── core/
    ├── adapters/
    └── converters/
```

---

## 🎯 개발 Phase

| Phase | 내용 | 산출물 |
|:-----:|------|--------|
| **1** | Signal Format Standard | `/spec/PHASE-1-SIGNAL-FORMAT.md` + JSON Schema |
| **2** | API Interface Standard | `/spec/PHASE-2-API-INTERFACE.md` + TypeScript/Python API |
| **3** | Communication Protocol | `/spec/PHASE-3-PROTOCOL.md` + Protocol 구현 |
| **4** | WIA Ecosystem Integration | `/spec/PHASE-4-INTEGRATION.md` + 연동 코드 |

---

## 📋 작업 규칙

### 1. 문서 작성 규칙

```markdown
- 모든 스펙 문서는 한글 + 영문 병행
- 코드 예제는 TypeScript, Python 모두 제공
- JSON Schema는 draft-07 표준 사용
- 다이어그램은 Mermaid 또는 ASCII Art 사용
```

### 2. 코드 작성 규칙

```
- 변수명: camelCase (영어)
- 파일명: kebab-case
- 주석: 한글 또는 영어로 명확하게
- 타입: TypeScript strict mode
- 테스트: Jest 또는 pytest
```

### 3. 커밋 메시지 규칙

```
feat: 새 기능 추가
fix: 버그 수정
docs: 문서 수정
refactor: 리팩토링
test: 테스트 추가
```

---

## 🔗 외부 참조

### 기존 AAC 관련 표준/자료

| 자료 | URL | 참고 내용 |
|------|-----|----------|
| Intel ACAT | https://github.com/intel/acat | 호킹 박사 사용 시스템 |
| Tobii SDK | https://developer.tobii.com | 시선 추적 API 참고 |
| OpenBCI | https://openbci.com | 뇌파 인터페이스 참고 |
| ISAAC | https://www.isaac-online.org | AAC 국제 학회 |

### WIA 생태계

| 서비스 | URL |
|--------|-----|
| WIA Live | https://wia.live |
| ISP | https://wia.live/isp |
| WIA Braille | https://wia.live/wia-braille |
| WIA Talk | https://wia.live/wia-talk |

---

## ⚠️ 주의사항

### DO (해야 할 것)

```
✅ 각 Phase 프롬프트를 먼저 읽고 작업
✅ 기존 문서 참조하여 일관성 유지
✅ 실제 AAC 센서 스펙 웹서치로 확인
✅ 코드는 실행 가능하도록 완성
✅ 에러 처리 포함
```

### DON'T (하지 말 것)

```
❌ 추측으로 스펙 작성 (웹서치로 확인)
❌ 기존 문서와 충돌하는 내용
❌ 실행 불가능한 pseudo code만 작성
❌ Phase 순서 건너뛰기
```

---

## 🚀 작업 시작 방법

```bash
# 1. 현재 Phase 확인
cat prompts/CURRENT-PHASE.md

# 2. 해당 Phase 프롬프트 읽기
cat prompts/PHASE-{N}-PROMPT.md

# 3. 필요 문서 참조
cat docs/WIA-AAC-OVERVIEW.md

# 4. 작업 시작
# (프롬프트 지시에 따라)
```

---

## 📞 문의

```
프로젝트 관리자: 연삼흠
조직: SmileStory Inc. / WIA
연락처: contact@wia.family
```

---

**현재 Phase: 1**

`/prompts/PHASE-1-PROMPT.md`를 읽고 작업을 시작하세요.

---

<div align="center">

**弘益人間** - 널리 인간을 이롭게

🤟

</div>
