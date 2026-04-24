# WIA CI Octave Enhancement
## Master Prompt - 전체 개요

---

## 🎯 프로젝트 목표

**인공와우(Cochlear Implant) 사용자가 옥타브/피치를 인식할 수 있도록 하는 신호 처리 표준**

### 문제 정의

```
현재 인공와우의 한계:
├── 22개 전극으로 주파수 대역만 구분
├── Temporal Fine Structure(TFS) 손실
├── 옥타브/피치 정보 손실
├── "로봇 목소리"처럼 들림
├── 음악 감상 어려움
└── 성조 언어(중국어 등) 인식 어려움

목표:
├── 옥타브 정보를 정확하게 추출
├── CI 신호에 옥타브 정보 인코딩
├── CI 사용자의 피치 인식 향상
└── 표준으로 공개하여 전 세계 활용
```

---

## 📋 4개 Phase 개요

| Phase | 제목 | 목표 | 산출물 |
|-------|------|------|--------|
| **1** | CI Signal Analysis | CI 신호 형식 분석, 손실 정보 정의 | 분석 스펙, JSON Schema |
| **2** | Octave Detection | 정확한 옥타브 검출 알고리즘 | OctaveYIN 알고리즘 |
| **3** | Enhancement Protocol | 옥타브 정보 인코딩 방법 | 인핸스먼트 프로토콜 |
| **4** | System Integration | API 및 시스템 연동 | API 서버, SDK |

---

## 🔄 작업 흐름

```
Phase 1: CI Signal Analysis
══════════════════════════════════
목표: "현재 CI 신호에서 무엇이 손실되는지 파악"

작업:
├── CI 신호 처리 원리 조사 (웹서치)
├── 주파수-전극 매핑 분석
├── 옥타브/피치 손실 정량화
├── 표준 신호 형식 정의 (JSON Schema)
└── 분석 도구 구현

산출물:
├── /spec/RESEARCH-PHASE-1.md
├── /spec/PHASE-1-CI-SIGNAL-ANALYSIS.md
├── /spec/schemas/*.schema.json
└── /api/*/signal-analyzer.*
         │
         ▼
Phase 2: Octave Detection
══════════════════════════════════
목표: "원본 신호에서 옥타브 정보 정확하게 추출"

작업:
├── 피치 검출 알고리즘 조사 (웹서치)
├── 옥타브 결정 연구 조사
├── OctaveYIN 알고리즘 설계
├── 하모닉 분석 기반 옥타브 결정
└── 실시간 처리 구현

산출물:
├── /spec/RESEARCH-PHASE-2.md
├── /spec/PHASE-2-OCTAVE-DETECTION.md
└── /api/*/octave/*
         │
         ▼
Phase 3: Enhancement Protocol
══════════════════════════════════
목표: "옥타브 정보를 CI 신호에 인코딩"

작업:
├── CI 자극 전략 조사 (웹서치)
├── 청각 심리학 조사
├── Temporal Modulation 인코딩
├── Electrode Pattern 인코딩
├── CI Simulator (vocoder) 구현
└── 전후 비교 검증

산출물:
├── /spec/RESEARCH-PHASE-3.md
├── /spec/PHASE-3-ENHANCEMENT-PROTOCOL.md
├── /api/*/enhancement/*
└── /examples/simulation/*
         │
         ▼
Phase 4: System Integration
══════════════════════════════════
목표: "실제 시스템과 연동 및 API 공개"

작업:
├── CI 연동 가능성 조사 (웹서치)
├── REST/WebSocket API 설계
├── 클라이언트 SDK 구현
├── 웹 데모 앱 구현
└── WIA 생태계 통합

산출물:
├── /spec/RESEARCH-PHASE-4.md
├── /spec/PHASE-4-INTEGRATION.md
├── /api/*/server/*
├── /sdk/*
└── /examples/demo-app/*
```

---

## 📁 디렉토리 구조

```
/ci/
├── prompts/
│   ├── MASTER-PROMPT.md         ← 현재 문서
│   ├── CURRENT-PHASE.md         # 현재 진행 중인 Phase
│   ├── PHASE-1-PROMPT.md
│   ├── PHASE-2-PROMPT.md
│   ├── PHASE-3-PROMPT.md
│   └── PHASE-4-PROMPT.md
│
├── spec/
│   ├── RESEARCH-PHASE-1.md
│   ├── RESEARCH-PHASE-2.md
│   ├── RESEARCH-PHASE-3.md
│   ├── RESEARCH-PHASE-4.md
│   ├── PHASE-1-CI-SIGNAL-ANALYSIS.md
│   ├── PHASE-2-OCTAVE-DETECTION.md
│   ├── PHASE-3-ENHANCEMENT-PROTOCOL.md
│   ├── PHASE-4-INTEGRATION.md
│   └── schemas/
│       ├── wia-ci-signal-v1.schema.json
│       ├── octave-result.schema.json
│       └── enhancement.schema.json
│
├── api/
│   ├── typescript/
│   │   └── src/
│   │       ├── signal/          # Phase 1
│   │       ├── octave/          # Phase 2
│   │       ├── enhancement/     # Phase 3
│   │       └── server/          # Phase 4
│   └── python/
│       └── wia_ci/
│           ├── signal/
│           ├── octave/
│           ├── enhancement/
│           └── server/
│
├── sdk/
│   ├── typescript/
│   ├── python/
│   ├── swift/
│   └── kotlin/
│
├── examples/
│   ├── sample-data/
│   ├── simulation/
│   └── demo-app/
│
└── docs/
    ├── README.md
    └── WIA-CI-OVERVIEW.md
```

---

## 🎯 성공 기준

### 기술적 목표

| 메트릭 | 목표값 |
|-------|-------|
| 옥타브 검출 정확도 (순음) | > 99% |
| 옥타브 검출 정확도 (음악) | > 90% |
| 실시간 처리 지연 | < 50ms |
| 피치 명확도 향상 (시뮬레이션) | > 20% |

### 비기술적 목표

```
□ GitHub 공개 완료
□ npm/PyPI 패키지 공개
□ 웹 데모 배포
□ 학술 논문 작성 (선택)
□ CI 제조사 컨택 (선택)
```

---

## 🚀 시작하기

### 현재 Phase 확인

```bash
cat /ci/prompts/CURRENT-PHASE.md
```

### Phase 1 시작

```bash
# CURRENT-PHASE.md를 1로 설정하고
# PHASE-1-PROMPT.md 지시에 따라 작업 시작
```

---

## 🧠 핵심 개념 정리

### 왜 CI에서 옥타브가 손실되는가?

```
정상 청력:
├── 달팽이관 내 ~3,500개 유모세포
├── 각 유모세포 = 특정 주파수에 반응
├── 연속적인 주파수 해상도
├── Temporal Fine Structure 보존
└── 정확한 옥타브/피치 인식

인공와우:
├── 22개 전극
├── 각 전극 = 넓은 주파수 대역 담당
├── 이산적 주파수 해상도
├── Envelope만 전달 (TFS 손실)
└── 옥타브 모호성 (440Hz vs 220Hz vs 880Hz 구분 어려움)
```

### 우리의 해결책

```
1. 원본 신호에서 옥타브 정보 추출 (Phase 2)
   └── 하모닉 분석 기반 정확한 옥타브 결정

2. CI 신호에 옥타브 정보 인코딩 (Phase 3)
   ├── Temporal Modulation: 옥타브별 고유 변조 주파수
   ├── Electrode Pattern: 옥타브별 특정 전극 활성화
   └── Harmonic Enhancement: 하모닉 관계 강조

3. CI 사용자 경험
   ├── Before: "뭔가 음이 나는데, 무슨 음인지 모름"
   └── After: "A4 (440Hz) 음이구나!"
```

---

## 📜 라이선스

MIT License

Copyright (c) 2025 SmileStory Inc. / WIA

---

## 🙏 철학

### 홍익인간 (弘益人間)

```
"널리 인간을 이롭게 하라"

- 특허 없음 (의도적 미출원)
- 영원히 무료
- 오픈소스 (MIT License)
- 강요 없음
```

### 역사적 대칭

```
1888: IPA 탄생 → 모든 소리를 표기
2025: ISP 탄생 → 모든 수어를 표기
2025: WIA CI  → 모든 CI 사용자에게 옥타브를
```

---

<div align="center">

**WIA CI Octave Enhancement Standard**

*"로봇 목소리에서 벗어나, 음악을 듣다"*

**홍익인간 🤟**

</div>
