# WIA CI Octave Enhancement Standard

## 인공와우 옥타브 인핸스먼트 표준

**Version**: 0.1.0
**Date**: 2025-12-14
**Author**: SmileStory Inc. / WIA
**License**: MIT

---

## 🎯 개요

인공와우(Cochlear Implant) 사용자가 **옥타브/피치**를 인식할 수 있도록 하는 신호 처리 표준입니다.

### 문제

```
현재 인공와우 한계:
├── 22개 전극으로 주파수 대역만 구분
├── 옥타브/피치 정보 손실
├── "로봇 목소리"처럼 들림
├── 음악 감상 어려움
└── 성조 언어(중국어 등) 인식 어려움
```

### 해결책

```
WIA CI Octave Enhancement:
├── Phase 1: CI 신호 분석 표준
├── Phase 2: 옥타브 검출 알고리즘
├── Phase 3: 신호 인핸스먼트 프로토콜
└── Phase 4: 시스템 연동 API
```

---

## 📋 4 Phase 구조

| Phase | 제목 | 설명 | 상태 |
|-------|------|------|------|
| 1 | CI Signal Analysis | CI 신호 형식 분석 | 🔲 대기 |
| 2 | Octave Detection | 옥타브 검출 알고리즘 | 🔲 대기 |
| 3 | Enhancement Protocol | 인핸스먼트 프로토콜 | 🔲 대기 |
| 4 | System Integration | API 및 연동 | 🔲 대기 |

---

## 🚀 시작하기

### 1. 프롬프트 확인

```bash
cd prompts/
cat MASTER-PROMPT.md      # 전체 개요
cat CURRENT-PHASE.md      # 현재 Phase
cat PHASE-1-PROMPT.md     # Phase 1 상세
```

### 2. Phase 1 작업 시작

Phase 1 프롬프트의 지시에 따라 작업을 시작합니다.

---

## 📁 디렉토리 구조

```
/ci/
├── README.md               ← 현재 문서
├── prompts/                # Claude 작업 프롬프트
├── spec/                   # 표준 스펙 문서
├── api/                    # API 구현
├── sdk/                    # 클라이언트 SDK
├── examples/               # 예제
└── docs/                   # 문서
```

---

## 🧠 핵심 개념

### Temporal Fine Structure (TFS)

```
정상 청력: 유모세포가 음파의 미세 구조(TFS)를 감지
           → 정확한 피치/옥타브 인식

인공와우:  TFS 손실, Envelope만 전달
           → 옥타브 모호성 발생
```

### 우리의 접근법

```
1. 원본 신호에서 옥타브 정보 추출
2. CI 신호에 옥타브 정보 인코딩
3. 기존 어음 명료도 유지하면서 피치 인식 향상
```

---

## 📜 라이선스

MIT License - 영원히 무료, 오픈소스

---

## 🙏 철학

**홍익인간 (弘益人間)** - 널리 인간을 이롭게 하라

```
1888: IPA  → 모든 소리를 표기
2025: ISP  → 모든 수어를 표기
2025: WIA CI → 모든 CI 사용자에게 옥타브를
```

---

<div align="center">

**WIA CI Octave Enhancement Standard**

*"로봇 목소리에서 벗어나, 음악을 듣다"*

**홍익인간 🤟**

</div>
