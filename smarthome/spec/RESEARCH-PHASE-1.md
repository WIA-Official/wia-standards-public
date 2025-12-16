# WIA Smart Home - Phase 1 Research

## 1. Executive Summary

이 문서는 WIA Smart Home Accessibility Standard를 위한 기술 조사 결과를 정리합니다.
스마트홈 접근성 관련 산업 현황, 기존 표준, 주요 기업/제품, 기술 트렌드를 분석합니다.

---

## 2. Industry Overview (산업 현황)

### 2.1 Smart Home Market

스마트홈 시장은 빠르게 성장하고 있으며, 2025년 기준 Matter 프로토콜이 산업 표준으로 자리잡고 있습니다.

**주요 특징:**
- IP 기반 통신 프로토콜 통합
- 음성 제어 보편화 (Alexa, Google Assistant, Siri)
- 접근성 기능이 핵심 차별화 요소로 부상

### 2.2 Accessibility Gap

장애인 및 고령자를 위한 스마트홈 접근성에는 여전히 격차가 존재합니다:

| 장애 유형 | 현재 지원 | 개선 필요 |
|----------|----------|----------|
| 시각 장애 | 음성 제어, TTS | 스크린 리더 호환성, 촉각 피드백 |
| 청각 장애 | 시각 알림 | 진동 알림, 수어 인식 |
| 이동 장애 | 음성 제어 | 스위치 제어, 시선 추적 |
| 인지 장애 | 자동화 | 단순화 인터페이스, 예측 지원 |

---

## 3. Existing Standards (기존 표준)

### 3.1 Matter Protocol (CSA-IoT)

**Connectivity Standards Alliance (CSA)**에서 개발한 Matter는 스마트홈 기기의 상호운용성을 위한 핵심 표준입니다.

**버전 히스토리:**
- Matter 1.0 (2022): 기본 기기 타입 (조명, 잠금장치, 스위치, 센서, 브릿지)
- Matter 1.1: Sleepy 기기 개선
- Matter 1.2: 추가 기기 타입
- Matter 1.4 (2024): 태양광 패널, 배터리, 에너지 자동화
- Matter 1.5 (예정 2025): 카메라, 스피커, 정원 관수

**데이터 모델:**
- TLV (Tag-Length-Value) 스키마
- 클러스터 기반 기능 정의
- Vendor ID (VID) / Product ID (PID) 시스템

**Sources:**
- [CSA-IoT Matter Solution](https://csa-iot.org/all-solutions/matter/)
- [Matter Core Specification 1.4](https://csa-iot.org/wp-content/uploads/2024/11/24-27349-006_Matter-1.4-Core-Specification.pdf)
- [Silicon Labs Matter Data Model](https://docs.silabs.com/matter/latest/matter-fundamentals-data-model/)

### 3.2 WCAG (Web Content Accessibility Guidelines)

W3C의 WCAG는 웹 접근성의 국제 표준으로, IoT 인터페이스에도 적용됩니다.

**버전:**
- WCAG 2.0 (2008): 기본 지침
- WCAG 2.1 (2018): 모바일 및 인지 장애 추가
- WCAG 2.2 (2023): 학습 장애, 집중력 장애 추가

**4대 원칙 (POUR):**
1. **Perceivable** (인지 가능): 대체 텍스트, 자막, 색상 대비
2. **Operable** (조작 가능): 키보드 접근, 충분한 시간
3. **Understandable** (이해 가능): 예측 가능, 입력 지원
4. **Robust** (견고함): 보조 기술 호환성

**Sources:**
- [W3C WCAG](https://www.w3.org/WAI/standards-guidelines/wcag/)
- [W3C Web of Things Accessibility](https://www.w3.org/WAI/APA/wiki/Web_of_Things)

### 3.3 Web of Things (WoT)

W3C의 Web of Things는 IoT 접근성을 위한 프레임워크를 제공합니다.

**핵심 개념:**
- Thing Description (TD): 기기 메타데이터
- Interaction Affordances: 속성, 액션, 이벤트
- Protocol Bindings: HTTP, MQTT, CoAP

---

## 4. Major Players (주요 기업/제품)

### 4.1 Platform Providers

| 플랫폼 | 회사 | 접근성 특징 |
|--------|------|------------|
| Amazon Alexa | Amazon | VoiceView 스크린 리더, 탭 투 알렉사 |
| Google Home | Google | TalkBack, 스위치 접근, 음성 매칭 |
| Apple Home | Apple | VoiceOver, 스위치 제어, AssistiveTouch |
| SmartThings | Samsung | 접근성 앱, 자동화 루틴 |
| Home Assistant | Open Source | 완전 커스터마이징 가능 |

### 4.2 Device Manufacturers

**조명:**
- Philips Hue: 음성 제어, 장면 설정
- LIFX: WiFi 직접 연결, 색상 피드백

**보안:**
- August/Yale: 스마트 잠금장치, 음성 제어
- Ring: 비디오 도어벨, 알림 커스터마이징

**환경 제어:**
- Ecobee/Nest: 스마트 온도조절기, 음성/원격 제어
- Lutron Caseta: 스마트 블라인드, 물리 버튼 유지

**센서:**
- Aqara: 다양한 센서, Matter 지원
- Eve: HomeKit/Matter, 프라이버시 중심

---

## 5. Technology Trends (기술 트렌드)

### 5.1 AI-Powered Accessibility

- **예측 자동화**: 사용 패턴 학습으로 선제적 제어
- **음성 인식 개선**: 비전형적 발화 패턴 지원
- **컴퓨터 비전**: 수어 인식, 제스처 인식

### 5.2 Multi-Modal Interaction

```
┌─────────────────────────────────────────────────────────┐
│                  Multi-Modal Input                       │
├───────────┬───────────┬───────────┬──────────┬─────────┤
│   음성    │   터치    │   스위치   │  시선   │  BCI    │
│  Voice    │   Touch   │  Switch   │  Gaze   │  Brain  │
└─────┬─────┴─────┬─────┴─────┬─────┴────┬────┴────┬────┘
      │           │           │          │         │
      └───────────┴───────────┼──────────┴─────────┘
                              │
                    ┌─────────▼─────────┐
                    │  Command Arbiter  │
                    │   (Priority)      │
                    └─────────┬─────────┘
                              │
                    ┌─────────▼─────────┐
                    │  Smart Home Hub   │
                    │  (Matter/WiFi)    │
                    └─────────┬─────────┘
                              │
       ┌──────────┬───────────┼───────────┬──────────┐
       │          │           │           │          │
   ┌───▼───┐  ┌───▼───┐  ┌────▼────┐  ┌───▼───┐  ┌──▼──┐
   │ Light │  │ Lock  │  │ Climate │  │ Blind │  │ ... │
   └───────┘  └───────┘  └─────────┘  └───────┘  └─────┘
```

### 5.3 Contextual Awareness

- **위치 기반**: 방별 자동 설정
- **시간 기반**: 일과 자동화
- **상황 인식**: 활동 감지 및 적응

### 5.4 Privacy-First Design

- **로컬 처리**: 클라우드 의존도 감소
- **데이터 최소화**: 필요한 데이터만 수집
- **투명한 동의**: 명확한 접근성 데이터 사용

---

## 6. Accessibility-Specific Requirements

### 6.1 Visual Impairment Support

| 기능 | 설명 | 우선순위 |
|------|------|----------|
| TTS | 모든 상태 음성 안내 | 필수 |
| Screen Reader | VoiceOver/TalkBack 호환 | 필수 |
| High Contrast | 고대비 UI | 필수 |
| Audio Cues | 비프음, 멜로디 피드백 | 권장 |
| Braille Display | 점자 디스플레이 지원 | 선택 |

### 6.2 Hearing Impairment Support

| 기능 | 설명 | 우선순위 |
|------|------|----------|
| Visual Alerts | 조명 깜빡임, 화면 알림 | 필수 |
| Vibration | 진동 피드백 | 필수 |
| Captions | 음성 명령 자막 표시 | 권장 |
| Sign Language | 수어 인식 (미래) | 선택 |

### 6.3 Motor Impairment Support

| 기능 | 설명 | 우선순위 |
|------|------|----------|
| Voice Control | 음성 명령 | 필수 |
| Switch Access | 외부 스위치 입력 | 필수 |
| Dwell Selection | 응시 선택 | 권장 |
| Large Targets | 큰 터치 영역 | 권장 |
| Sip-and-Puff | 불기/빨기 제어 | 선택 |

### 6.4 Cognitive Impairment Support

| 기능 | 설명 | 우선순위 |
|------|------|----------|
| Simplified UI | 단순화된 인터페이스 | 필수 |
| Predictive | 예측 기반 제안 | 권장 |
| Routine Support | 일과 자동화 | 권장 |
| Reminders | 알림 및 리마인더 | 권장 |
| Caregiver Mode | 보호자 모니터링 | 선택 |

---

## 7. Data Format Considerations

### 7.1 Required Data Entities

1. **User Profile**: 접근성 요구사항, 선호도
2. **Device Capability**: 기기 접근성 기능
3. **Room/Zone**: 공간 기반 구성
4. **Automation Rule**: 자동화 규칙
5. **Accessibility Event**: 접근성 관련 이벤트
6. **Notification**: 다중 모달 알림

### 7.2 Interoperability Requirements

- Matter 클러스터와 매핑 가능
- WCAG 원칙 반영
- 기존 스마트홈 플랫폼과 호환

---

## 8. Conclusion & Recommendations

### 8.1 Key Findings

1. **Matter**가 산업 표준으로 확립됨 - WIA 표준은 Matter와 호환되어야 함
2. **다중 모달 입력**이 핵심 - 음성, 터치, 스위치, 시선 등 모두 지원
3. **개인화**가 중요 - 장애 유형별 맞춤 설정 필요
4. **프라이버시** 고려 필수 - 접근성 데이터의 민감성

### 8.2 Recommendations for WIA Standard

1. **Matter 호환성**: 클러스터 기반 데이터 모델 채택
2. **WCAG 준수**: 4대 원칙을 데이터 구조에 반영
3. **확장성**: 새로운 접근성 기능 추가 가능한 구조
4. **국제화**: 다국어 지원 내장

---

## 9. Sources

### Standards & Specifications
- [CSA-IoT Matter](https://csa-iot.org/all-solutions/matter/)
- [Matter Core Specification](https://csa-iot.org/wp-content/uploads/2024/11/24-27349-006_Matter-1.4-Core-Specification.pdf)
- [W3C WCAG](https://www.w3.org/WAI/standards-guidelines/wcag/)
- [W3C Web of Things](https://www.w3.org/WAI/APA/wiki/Web_of_Things)

### Industry Resources
- [Guide Dogs UK - Matter Accessibility](https://www.guidedogs.org.uk/blog/matter-smart-home-accessibility)
- [Accessibility Checker - Smart Home Guide](https://www.accessibilitychecker.org/blog/smart-home-accessibility/)
- [Accessibility.works - AI Smart Home](https://www.accessibility.works/blog/ai-smart-home-accessibility/)
- [InclusionHub - IoT Experiences](https://www.inclusionhub.com/articles/designing-accessible-iot-experiences)

### Technical Documentation
- [Silicon Labs Matter Data Model](https://docs.silabs.com/matter/latest/matter-fundamentals-data-model/)
- [Google Home Matter Overview](https://developers.home.google.com/matter/overview)

---

*弘益人間 - 널리 인간을 이롭게 하다*
