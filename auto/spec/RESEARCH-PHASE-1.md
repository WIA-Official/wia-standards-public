# WIA Autonomous Vehicle Accessibility - Phase 1 Research

## 1. Executive Summary

자율주행차(AV)는 장애인에게 전례 없는 이동 자유를 약속합니다. 미국에서만 약 2,500만 명이 이동 제한 장애를 가지고 있으며, 완전 자율주행차는 시각장애인, 휠체어 사용자, 청각장애인 등에게 독립적인 이동 수단을 제공할 수 있습니다.

## 2. Industry Landscape

### 2.1 Major Players

| Company | Vehicle | Accessibility Features |
|---------|---------|----------------------|
| Waymo | Chrysler Pacifica | Braille labels, 점자 표시, 오디오 큐, 햅틱 피드백 |
| Cruise | Origin | 휠체어 접근성 연구 중 |
| BraunAbility | Concept AV | DOT Inclusive Design Challenge 우승 |
| Zoox | Custom Vehicle | 양방향 대칭 설계 |

### 2.2 Pilot Programs

- **goMARTI (Grand Rapids)**: 35마일 도로, 71개 정류장, 약 10% 휠체어 이용자
- **Waymo One**: Phoenix, San Francisco에서 상용 서비스
- **May Mobility**: 휠체어 접근 가능 셔틀 시범 운영

## 3. Existing Standards & Regulations

### 3.1 SAE J3016 - Levels of Driving Automation

| Level | Name | Description |
|-------|------|-------------|
| 0 | No Automation | 운전자가 모든 것을 제어 |
| 1 | Driver Assistance | 조향 또는 가속/감속 보조 |
| 2 | Partial Automation | 조향 및 가속/감속 보조 |
| 3 | Conditional Automation | 시스템이 전체 동적 운전 수행, 운전자 대기 |
| 4 | High Automation | 특정 조건에서 완전 자율 |
| 5 | Full Automation | 모든 조건에서 완전 자율 |

**Note**: J3016은 용어 표준이며, 안전 표준이 아님

### 3.2 ADA Accessibility Requirements (49 CFR Part 38)

- **휠체어 고정 공간**: 30인치 x 48인치 최소
- **고정 시스템 강도**:
  - 30,000 lb+ 차량: 4,000 lbs/휠체어
  - 30,000 lb 미만: 5,000 lbs/휠체어
- **진입/퇴장**: 핸드레일이 휠체어 이동을 방해하지 않아야 함

### 3.3 SAE Wheelchair Standards

| Standard | Description |
|----------|-------------|
| SAE J2092 | 휠체어 리프트 요구사항 |
| SAE J2093 | 휠체어 리프트 테스트 절차 |
| SAE J2094 | 장애인 운전자 차량 개조 |

### 3.4 Regulatory Gap

현재 연방 정부나 어떤 주에서도 **AV의 장애인 접근성을 의무화하지 않음**.
Urban Institute는 FTA가 새로운 AV 설계 시 휠체어 접근과 시청각 장애인 요구를 고려하도록 요구할 것을 권고.

## 4. Accessibility Features Analysis

### 4.1 Visual Impairment Support

| Feature | Implementation | Source |
|---------|---------------|--------|
| Braille Labels | 차량 내 물리적 버튼 | Waymo |
| Audio Cues | 회전, 이벤트 음성 안내 | Waymo |
| TalkBack/VoiceOver | 앱 스크린 리더 지원 | Waymo |
| Find My Car | 음향/햅틱 경로 안내 | Waymo |
| Horn Signal | 차량 위치 확인용 경적 | Waymo |

### 4.2 Hearing Impairment Support

| Feature | Implementation |
|---------|---------------|
| Visual Information | 화면으로 주변 상황 표시 |
| Light Signals | 청각 대신 시각 신호 |
| Vibration Alerts | 햅틱 피드백 알림 |

### 4.3 Mobility Impairment Support

| Feature | Challenge |
|---------|-----------|
| Ramp/Lift | 자동 램프 전개 |
| Flat Floor | 스티어링 휠 없는 설계로 가능 |
| Auto Securement | 자동 휠체어 고정 시스템 연구 중 |
| Wide Doors | 휠체어 진입 용이 |

### 4.4 Cognitive Impairment Support

| Feature | Implementation |
|---------|---------------|
| Simple UI | 최소화된 인터페이스 |
| Voice Commands | 음성 명령 지원 |
| Panic Button | 비상 정지 버튼 |
| Live Support | 원격 오퍼레이터 연결 |

## 5. HMI (Human-Machine Interface) Requirements

### 5.1 Multi-Modal Interaction

```
┌─────────────────────────────────────────┐
│           Accessible HMI               │
├─────────────┬─────────────┬────────────┤
│   Visual    │   Audio     │  Haptic    │
├─────────────┼─────────────┼────────────┤
│ Screen      │ TTS         │ Vibration  │
│ LED         │ Chimes      │ Force      │
│ Icons       │ Speech Rec  │ Texture    │
└─────────────┴─────────────┴────────────┘
```

### 5.2 Critical Functions

1. **Start Ride**: 탑승 시작
2. **Pull Over**: 즉시 정차
3. **Emergency Stop**: 비상 정지
4. **Contact Support**: 지원 연결
5. **Navigate**: 목적지 설정

## 6. Technical Trends

### 6.1 Automatic Wheelchair Securement

- 현재 연구 단계
- BraunAbility + Purdue University 협력
- DOT Inclusive Design Challenge 수상

### 6.2 Vehicle-to-Infrastructure (V2I)

- 자동 문 열기 신호
- 엘리베이터 호출
- 주차 공간 예약

### 6.3 Companion Apps

- 사전 접근성 설정
- 원격 차량 제어
- 실시간 상태 모니터링

## 7. Challenges & Gaps

### 7.1 Technical Challenges

| Challenge | Status |
|-----------|--------|
| 자동 휠체어 고정 | 연구 중 |
| 독립적 진입/퇴장 | 부분 해결 |
| 다양한 휠체어 호환 | 미해결 |
| 인지 장애 지원 | 초기 단계 |

### 7.2 Regulatory Gaps

- AV 접근성 의무화 없음
- 통합 표준 부재
- 인증 프로세스 미정립

### 7.3 Industry Gaps

- 휠체어 접근 가능 AV 상용화 부재
- 농촌 지역 서비스 부족
- 높은 비용

## 8. Recommendations for WIA Standard

### 8.1 Data Format Requirements

1. **Passenger Profile**: 접근성 요구사항 프로파일
2. **Vehicle Capabilities**: 차량 접근성 기능
3. **Trip Request**: 접근성 포함 경로 요청
4. **HMI Configuration**: 다중 모달 인터페이스 설정
5. **Securement Status**: 휠체어 고정 상태

### 8.2 Interoperability

- SAE J3016 레벨과 연동
- ADA 요구사항 준수
- Matter/Thread 스마트홈 통합

### 8.3 Safety Requirements

- 비상 정지 우선순위
- 실시간 모니터링
- 원격 지원 연결

## 9. Sources

- [U.S. Access Board - Inclusive Design of Autonomous Vehicles](https://www.access-board.gov/av/)
- [Urban Institute - Shared Autonomous Vehicles & Disabilities](https://www.urban.org/urban-wire/shared-autonomous-vehicles-could-improve-transit-access-people-disabilities-if-regulated)
- [Waymo Accessibility Features](https://support.google.com/waymo/answer/9566824?hl=en)
- [SAE J3016 Levels of Driving Automation](https://www.sae.org/blog/sae-j3016-update)
- [49 CFR Part 38 - ADA Accessibility Specifications](https://www.ecfr.gov/current/title-49/subtitle-A/part-38)
- [BraunAbility - Autonomous Vehicles for Wheelchair Users](https://www.braunability.com/us/en/blog/mobility-solutions/what-are-autonomous-vehicles.html)
- [Waymo Accessibility Network](https://waymo.com/waymo-accessibility-network/)
- [Stanton-Mast Bipartisan AV Accessibility Bill (2024)](https://stanton.house.gov/2024/1/stanton-mast-introduce-bipartisan-bill-to-make-autonomous-vehicles-more-accessible-for-people-with-disabilities)

---

*Research completed: December 2024*
*WIA Autonomous Vehicle Accessibility Standard - Phase 1*
