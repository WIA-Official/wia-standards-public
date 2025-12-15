# WIA Eye Gaze Standard - Phase 1 Research Report

## 연구 목적 (Research Objectives)

Phase 1에서는 기존 시선 추적 프로토콜과 표준을 분석하여 WIA Eye Gaze Interoperability Protocol의 기반을 마련합니다.

---

## 1. 기존 프로토콜 분석 (Existing Protocol Analysis)

### 1.1 Tobii Pro SDK

**출처**: [Tobii Pro SDK](https://developer.tobiipro.com/)

#### 데이터 스트림
- **Gaze Data**: 좌/우 눈 별도로 raw 데이터 제공
- **Gaze Origin**: 3D 공간 좌표 (눈 위치)
- **Gaze Point**: 화면상 시선 위치
- **Pupil Diameter**: 동공 직경 (mm)
- **Eye Openness**: 눈 열림 정도 (mm) - Spectrum, Fusion에서 지원
- **Eye Images**: 눈 이미지 스트림 (Spectrum, Fusion)

#### 타임스탬프
- Eye tracker 클럭 + 컴퓨터 클럭 이중 타임스탬프
- 자동 시간 보정으로 밀리초 단위 동기화 정확도
- Time Reference 스트림으로 추가 정밀 동기화 가능

#### API 특징
- 멀티플랫폼 (Windows, Linux, Mac)
- 다국어 바인딩 (.NET, Python, Matlab/Octave, C, Unity)
- 구독(Subscribe) 기반 데이터 스트리밍

#### Tobii Pro Glasses 3
- HTTP REST API + WebSocket
- WebRTC/RTSP로 라이브 스트리밍
- 씬 카메라 + 시선 데이터 동기화

---

### 1.2 Gazepoint GP3 Open Gaze API

**출처**: [Gazepoint API v2.0](https://www.gazept.com/dl/Gazepoint_API_v2.0.pdf)

#### 통신 프로토콜
- **TCP/IP** 기반 (포트 4242)
- **XML 형식** 명령/응답

#### 데이터 파라미터
| 파라미터 | 설명 |
|---------|------|
| TIME | 타임스탬프 |
| FPOGX/FPOGY | Fixation Point of Gaze (0.0-1.0) |
| LPOGX/LPOGY | Left eye POG |
| RPOGX/RPOGY | Right eye POG |
| LPCX/LPCY/LPD | Left pupil center & diameter |
| RPCX/RPCY/RPD | Right pupil center & diameter |
| LEYEX/LEYEY/LEYEZ | Left eye 3D position |
| REYEX/REYEY/REYEZ | Right eye 3D position |

#### API 명령 구조
```xml
<!-- Request -->
<GET ID="PRODUCT_ID" />

<!-- Response -->
<ACK ID="PRODUCT_ID" VALUE="GP3" />

<!-- Enable data stream -->
<SET ID="ENABLE_SEND_EYE_LEFT" STATE="1" />
<SET ID="ENABLE_SEND_POG_FIX" STATE="1" />
```

#### 특징
- 오픈 API 프로토콜 (개발자 라이선스 포함)
- 최대 150Hz 데이터 스트리밍
- 모든 프로그래밍 언어에서 사용 가능

---

### 1.3 Pupil Labs (Open Source)

**출처**: [Pupil Labs GitHub](https://github.com/pupil-labs/pupil)

#### 플랫폼 구성
1. 경량 헤드셋 (고해상도 카메라)
2. 오픈소스 소프트웨어 프레임워크
3. Pupil Player GUI (비디오 + 시선 시각화)

#### 데이터 형식
- **오픈 데이터 포맷** - 문서화된 데이터 구조
- Raw 데이터 + 분석 데이터 내보내기
- Pupil Cloud와 연동

#### 기술 사양
- Eye camera: 800x600 @ 30Hz
- UVC 호환 카메라 지원
- 양안(binocular) 설정 가능

#### 오픈소스 특징
- Python + C++ (성능 중요 부분)
- 플러그인 기반 확장
- MIT 라이선스 (일부 구성요소)

---

## 2. 표준화 동향 (Standardization Efforts)

### 2.1 ATIA Eye Gaze Standards Working Group

**출처**: [ATIA Eye Gaze Standards](https://www.atia.org/eyegazestandards/)

#### 배경
- 2024년 1월 ATIA 컨퍼런스에서 첫 회의 개최
- 시선 추적 기술의 **상호운용성(Interoperability)** 부재 문제 해결 목표
- Tolt Technologies의 Jay Beavers 주도

#### 핵심 목표
```
"Windows 환경에서 시선 인식 애플리케이션들이
서로 인식하고 협력하며 상호작용할 수 있는 프로세스"
```

#### 해결해야 할 문제
1. 신기술로서 확립된 상호운용성 표준 부재
2. 다른 제조사/개발자의 제품이 동일 디바이스에서 충돌
3. 사용자 경험 통합 필요

#### 참여 자격
- ATIA 회원 및 비회원 가능
- 시선 추적 기술에 관심 있는 개발 커뮤니티

---

### 2.2 ISO 표준

#### ISO 9241-971:2020
**Ergonomics of human-system interaction — Part 971: Accessibility of tactile/haptic interactive systems**

- 촉각/햅틱 시스템의 접근성 표준
- **시선 추적**을 대체 입력 방식 중 하나로 언급
- 다양한 입력 양식 지원: 마우스, 터치, 비접촉 제스처, **eye-tracking**, 스위치, 전신 움직임

#### 관련 표준
| 표준 | 내용 |
|-----|------|
| ISO 9241-9 | 포인팅 디바이스 평가 기준 |
| ISO 9241-910 | 촉각/햅틱 상호작용 프레임워크 |
| ISO 9241-920 | 촉각/햅틱 입력 디바이스 |
| ISO 9241-171 | 접근성 지침 |

---

### 2.3 W3C/WCAG

#### WCAG 2.1/2.2 관련 지침

**출처**: [W3C WAI WCAG](https://www.w3.org/WAI/standards-guidelines/wcag/)

- **Operable** 원칙: 키보드 이외의 입력 방식 지원
- 대체 입력 방식 (음성, 시선 등) 고려
- 명시적인 시선 추적 전용 지침은 없음

#### WCAG 3.0 (개발 중)
- 2024년 9월 Working Draft 업데이트
- 더 넓은 범위의 접근성 고려 중

---

### 2.4 EN 301 549 (유럽 접근성 표준)

- ICT 제품 및 서비스 접근성 요구사항
- WCAG 2.1 기반
- 보조기기 호환성 요구

---

## 3. AAC에서의 시선 추적 활용

### 3.1 Direct Selection 방법론

**출처**: [Communication Community - AAC Direct Selection](https://www.communicationcommunity.com/aac-direct-selection-access/)

#### 접근 방법 유형
1. **Touch** - 직접 터치
2. **Laser** - 레이저 포인터
3. **Head Tracking** - 머리 움직임 추적
4. **Eye Gaze** - 시선 추적

#### Eye Gaze 특징
- 적외선 기술 사용
- 동공 움직임 연속 추적
- **Dwell Time** 조절 가능 (선택까지 응시 시간)
- 손 사용이 불가능한 사용자에게 이상적

---

## 4. 데이터 품질 표준화

### 4.1 Eye Tracking Data Quality Reporting

**출처**: [ETRA 2024](https://etra.acm.org/2024/)

#### 주요 메트릭
| 메트릭 | 설명 |
|-------|------|
| Accuracy | 실제 시선 위치와 측정값 차이 (degrees) |
| Precision | 측정값의 일관성 (degrees) |
| Sampling Rate | 데이터 수집 주파수 (Hz) |
| Data Loss | 유효하지 않은 데이터 비율 |
| Latency | 처리 지연 시간 (ms) |

---

## 5. 분석 결론 (Analysis Conclusions)

### 5.1 공통점

| 항목 | 공통 패턴 |
|-----|----------|
| 좌표계 | 정규화 좌표 (0.0 - 1.0) 사용 |
| 양안 데이터 | 좌/우 눈 개별 데이터 + 결합 데이터 |
| 3D 데이터 | Gaze Origin (눈 위치), Gaze Direction (시선 방향) |
| 동공 | 동공 직경, 동공 중심 위치 |
| 타임스탬프 | 고정밀 타임스탬프 필수 |
| 유효성 플래그 | 데이터 유효성 표시 |

### 5.2 차이점

| 항목 | Tobii | Gazepoint | Pupil Labs |
|-----|-------|-----------|------------|
| 프로토콜 | 네이티브 SDK | TCP/IP + XML | Python API |
| 좌표 단위 | 화면 비율 | 화면 비율 | 픽셀/비율 혼용 |
| 이벤트 | 콜백 기반 | 폴링 기반 | 프레임 기반 |
| 라이선스 | 상용 | 개발자 무료 | 오픈소스 |

### 5.3 WIA 표준 설계 방향

1. **좌표 정규화**: 0.0 - 1.0 범위의 화면 비율 좌표 채택
2. **양안 데이터**: 좌/우 눈 개별 + 결합 데이터 모두 지원
3. **3D 지원**: 3D gaze origin/direction 옵션 포함
4. **이벤트 기반**: Fixation, Saccade, Blink 등 표준 이벤트 정의
5. **품질 메트릭**: Confidence, Validity 필드 필수
6. **프로토콜 독립**: JSON 기반으로 어떤 전송 계층에서도 사용 가능
7. **접근성 우선**: AAC/AT 사용 사례 최우선 고려

---

## 6. 참고 자료 (References)

### 제조사 SDK
- [Tobii Pro SDK Documentation](https://developer.tobiipro.com/)
- [Gazepoint API v2.0](https://www.gazept.com/dl/Gazepoint_API_v2.0.pdf)
- [Pupil Labs GitHub](https://github.com/pupil-labs/pupil)

### 표준화 기구
- [ATIA Eye Gaze Standards Working Group](https://www.atia.org/eyegazestandards/)
- [W3C WAI WCAG](https://www.w3.org/WAI/standards-guidelines/wcag/)
- [ISO 9241-971:2020](https://www.iso.org/standard/74511.html)

### 학회
- [ETRA 2024 - Eye Tracking Research & Applications](https://etra.acm.org/2024/)

---

<div align="center">

**WIA Eye Gaze Standard - Research Phase 1 Complete**

**홍익인간** - 널리 인간을 이롭게

</div>
