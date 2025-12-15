# Phase 4 사전 조사 결과: WIA 생태계 연동

**조사일**: 2024년
**목적**: AI 시스템과 WIA 생태계 연동을 위한 기술 조사

---

## 1. AI 에이전트 프레임워크 비교

### 1.1 LangChain

**개요**: LLM 기반 애플리케이션 구축을 위한 오픈소스 프레임워크

**주요 특징**:
- 모듈식 아키텍처: 각 모듈이 특정 작업을 캡슐화
- 메모리 지원: 상호작용 간 컨텍스트 유지
- 다양한 도구 통합: 벡터 DB, API, 클라우드 서비스
- 체이닝 기능: 모듈을 연결하여 복잡한 파이프라인 구성

**WIA 연동 적합성**: ★★★★★
- 모듈식 설계로 WIA 커넥터 쉽게 추가 가능
- 메모리 기능으로 AAC 사용자 컨텍스트 유지
- 도구 통합으로 TTS, 점자 출력 연동 용이

### 1.2 AutoGPT

**개요**: 자율 AI 에이전트의 선구자적 프로젝트

**주요 특징**:
- 목표 지향 자율성: 고수준 목표를 하위 작업으로 분해
- 실시간 인터넷 접근
- 코드 생성 및 실행 능력
- 자기 수정 가능

**WIA 연동 적합성**: ★★★☆☆
- 자율성이 높아 접근성 보조에는 과도할 수 있음
- 프로토타이핑에 유용

### 1.3 CrewAI

**개요**: 멀티 에이전트 협업 프레임워크

**주요 특징**:
- 팀 기반 작업 할당
- 역할 기반 에이전트 정의
- 에이전트 간 통신

**WIA 연동 적합성**: ★★★★☆
- 다중 WIA 표준 연동 시 에이전트 역할 분담 가능
- AAC 에이전트, TTS 에이전트, 점자 에이전트 등 분리 가능

### 1.4 AutoGen (Microsoft)

**개요**: 대화형 멀티턴 추론 에이전트

**주요 특징**:
- 풍부한 멀티턴 추론
- 대화형 에이전트 설계
- Microsoft 생태계 통합

**WIA 연동 적합성**: ★★★★☆
- Azure 접근성 서비스와 연동 용이
- 대화형 AAC에 적합

### 1.5 시장 전망

> Gartner 예측: 2028년까지 기업 소프트웨어의 33%가 에이전틱 AI를 통합할 것
> (2024년 1% 미만에서 급증)

---

## 2. AI 텍스트-음성 변환 (TTS) API

### 2.1 Google Cloud Text-to-Speech

**특징**:
- DeepMind 기반 음성 합성
- 380+ 음성, 50+ 언어
- WaveNet 기술로 자연스러운 음성

**가격**:
- Standard 음성: 월 400만 자 무료
- WaveNet 음성: 월 100만 자 무료
- 신규 고객 $300 크레딧

**접근성 적용**: ★★★★★
- 다양한 언어/음성으로 글로벌 접근성 지원
- 고품질 음성으로 청각 이해도 향상

### 2.2 Amazon Polly

**특징**:
- Neural TTS (NTTS) 기술
- 자연스러운 억양, 강세, 휴지
- IVR, e-러닝, 접근성 도구에 활용

**접근성 적용**: ★★★★★
- 표현력 있는 음성으로 감정 전달
- 다양한 사용 사례 지원

### 2.3 ElevenLabs

**특징**:
- 최첨단 신경망 모델
- 고품질 음성 합성
- 커스터마이징 가능
- 다국어 지원

**접근성 적용**: ★★★★☆
- 고품질이지만 비용 고려 필요

### 2.4 Microsoft Azure Speech

**특징**:
- Neural TTS
- 다양한 음성 및 언어
- Azure 생태계 통합

**접근성 적용**: ★★★★★
- Microsoft 접근성 도구와 통합
- 엔터프라이즈급 안정성

### 2.5 오픈소스 옵션

**MARYTTS**:
- 다국어 TTS 플랫폼
- 영어, 프랑스어, 독일어, 이탈리아어, 러시아어 등
- 무료 사용

**CMU Flite (Festival Lite)**:
- 경량 런타임 TTS 엔진
- 빠른 속도와 효율성
- 오픈소스, 무료

---

## 3. 멀티모달 AI 모델 비교

### 3.1 GPT-4o (OpenAI)

**출시**: 2024년 5월

**특징**:
- "o" = omni (모든 모달리티)
- 텍스트, 오디오, 이미지, 비디오 입력
- 110 토큰/초 (GPT-4 Turbo 대비 3배 빠름)
- GPT-4 Turbo 대비 50% 저렴, 2배 빠름

**WIA 연동 적합성**: ★★★★★
- 이미지 설명으로 시각장애인 지원
- 오디오 처리로 음성 입력 지원

### 3.2 Gemini 1.5 Pro (Google)

**특징**:
- 텍스트, 이미지, 음악, 코드, 비디오로 처음부터 학습
- 100만 토큰 컨텍스트 윈도우 (200만 대기자 명단)
- 텍스트, 이미지, 오디오, 비디오 인터리빙 입력

**WIA 연동 적합성**: ★★★★★
- 가장 긴 컨텍스트로 복잡한 AAC 대화 지원
- 네이티브 멀티모달

### 3.3 Claude 3 Opus (Anthropic)

**특징**:
- Claude 3 패밀리 중 최고 성능
- 텍스트, 이미지, 차트, 다이어그램 처리
- 인지 작업 벤치마크 최고 기록

**WIA 연동 적합성**: ★★★★☆
- 안전성 중심 설계로 접근성 애플리케이션에 적합
- 높은 정확도

### 3.4 2024 멀티모달 트렌드

- 컨텍스트 길이 증가: 4K/8K → 100K+ → 2M 토큰
- 네이티브 멀티모달 학습
- 비용 절감 및 속도 향상

---

## 4. AI 접근성 표준 및 가이드라인

### 4.1 적용 표준

**WCAG 2.1**:
- Level A 및 AA 준수 필수
- 키보드, 스크린 리더, 음성 인식 접근성

**POUR 원칙**:
- Perceivable (인지 가능)
- Operable (조작 가능)
- Understandable (이해 가능)
- Robust (견고함)

**UDL (Universal Design for Learning)**:
- 다양한 학습 방식 지원
- 유연한 인터페이스

### 4.2 AI 기반 접근성 기술

**시각 보조**:
- Seeing AI (Microsoft)
- Be My Eyes + GPT-4
- 이미지 설명 생성

**청각 보조**:
- Live Captions
- Otter.ai
- 자동 자막 생성

**이동성 보조**:
- 음성 명령 제어
- 텍스트 받아쓰기
- 인터페이스 네비게이션

### 4.3 글로벌 수요

> WHO 통계: 전 세계 25억 명이 보조 기술 필요
> 고령화로 2050년까지 35억 명으로 증가 예상

---

## 5. WIA 생태계 연동 설계 권장사항

### 5.1 아키텍처 권장

```
┌─────────────────────────────────────────────────┐
│              WIA-AI Integration Hub              │
├─────────────────────────────────────────────────┤
│                                                  │
│  ┌─────────────┐  ┌─────────────┐              │
│  │   입력 계층  │  │   출력 계층  │              │
│  │  - AAC      │  │  - TTS      │              │
│  │  - BCI      │  │  - ISP      │              │
│  │  - Voice    │  │  - Braille  │              │
│  └──────┬──────┘  └──────┬──────┘              │
│         │                │                      │
│         ▼                ▼                      │
│  ┌─────────────────────────────────────────┐   │
│  │         AI Processing Layer              │   │
│  │  - LangChain 기반 에이전트 프레임워크    │   │
│  │  - 멀티모달 모델 (GPT-4o/Gemini/Claude)  │   │
│  │  - 컨텍스트 관리 및 메모리               │   │
│  └─────────────────────────────────────────┘   │
│                                                  │
└─────────────────────────────────────────────────┘
```

### 5.2 기술 스택 권장

| 구성 요소 | 권장 기술 | 대안 |
|----------|----------|------|
| AI 프레임워크 | LangChain | CrewAI, AutoGen |
| LLM | GPT-4o, Claude 3 | Gemini 1.5 Pro |
| TTS | Google Cloud TTS | Amazon Polly, Azure |
| 오픈소스 TTS | MARYTTS | CMU Flite |

### 5.3 구현 우선순위

1. **1순위**: WiaConnector 추상화 인터페이스
2. **2순위**: AAC 입력 어댑터 (가장 중요한 접근성 요구)
3. **3순위**: TTS 출력 어댑터 (가장 보편적인 출력)
4. **4순위**: BCI/Voice 입력 어댑터
5. **5순위**: ISP/Braille 출력 어댑터

---

## 6. 결론

### 6.1 핵심 발견

1. **AI 에이전트 프레임워크**: LangChain이 WIA 연동에 가장 적합
2. **TTS**: Google Cloud TTS가 품질/비용 균형 최적
3. **멀티모달**: GPT-4o가 속도/비용/성능 균형 최적
4. **접근성**: WCAG 2.1 준수 필수

### 6.2 다음 단계

1. PHASE-4-INTEGRATION.md 상세 스펙 작성
2. Rust 연동 모듈 구현
3. Mock 어댑터로 테스트 가능하게 구현
4. 예제 코드 작성

---

## 참고 자료

- [Top AI Agent Frameworks 2025](https://www.analyticsvidhya.com/blog/2024/07/ai-agent-frameworks/)
- [Best Text-to-Speech APIs 2025](https://www.edenai.co/post/best-text-to-speech-apis)
- [GPT-4o vs Gemini vs Claude Comparison](https://encord.com/blog/gpt-4o-vs-gemini-vs-claude-3-opus/)
- [AI and Accessibility](https://www.levelaccess.com/blog/ai-and-assistive-tech-key-advancements-in-accessibility/)
- [Digital Accessibility in AI Era](https://www.frontiersin.org/journals/artificial-intelligence/articles/10.3389/frai.2024.1349668/full)
