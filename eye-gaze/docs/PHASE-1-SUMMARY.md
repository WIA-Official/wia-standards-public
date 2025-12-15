# WIA Eye Gaze Standard - Phase 1 Summary

**Phase 1: Interoperability Protocol Definition**
**Status**: Complete
**Date**: 2025-01

---

## 1. Phase 1 개요 (Overview)

Phase 1에서는 WIA Eye Gaze Interoperability Protocol의 기반이 되는 데이터 포맷, 이벤트 시스템, 디바이스 역량 표준을 정의했습니다.

### 1.1 목표

- 모든 시선 추적 디바이스 간 **데이터 호환성** 확보
- 시선 기반 **이벤트의 표준화**
- 디바이스 **역량 자동 검색** 메커니즘 정의
- **AAC(보완대체의사소통)** 사용 사례 최우선 고려

### 1.2 철학

**弘益人間 (홍익인간)** - 널리 인간을 이롭게

- 오픈소스 (MIT License)
- 특허 없음, 영원히 무료
- 모든 보조기기 사용자를 위한 표준

---

## 2. 연구 결과 (Research Findings)

### 2.1 분석한 기존 프로토콜

| 제조사/프로젝트 | 프로토콜 | 특징 |
|---------------|---------|------|
| **Tobii Pro** | 네이티브 SDK | 양안/3D/이미지 스트림, 멀티플랫폼 |
| **Gazepoint** | TCP/IP + XML | 오픈 API, 간단한 구조, 150Hz |
| **Pupil Labs** | Python API | 완전 오픈소스, 착용형 전문 |

### 2.2 주요 표준화 동향

| 기구 | 활동 |
|-----|------|
| **ATIA** | Eye Gaze Standards Working Group (2024~) |
| **ISO** | 9241-971 - 접근성 표준에서 시선 추적 언급 |
| **W3C** | WCAG - 대체 입력 방식 지원 지침 |

### 2.3 핵심 발견

1. **좌표 정규화**: 대부분 0.0-1.0 정규화 좌표 사용
2. **양안 데이터**: 좌/우 눈 개별 데이터 제공이 표준
3. **상호운용성 부재**: 업계 전반적으로 통합 표준 없음
4. **AAC 특화 기능**: 대부분 디바이스에서 미지원

---

## 3. Phase 1 산출물 (Deliverables)

### 3.1 명세 문서 (Specifications)

| 문서 | 경로 | 내용 |
|-----|------|------|
| 연구 보고서 | `spec/RESEARCH-PHASE-1.md` | 기존 프로토콜 분석 결과 |
| 데이터 포맷 | `spec/DATA-FORMAT-SPEC.md` | GazePoint, EyeData 구조 |
| 이벤트 명세 | `spec/EVENT-SPEC.md` | 27종 표준 이벤트 정의 |
| 디바이스 역량 | `spec/DEVICE-CAPABILITY-SPEC.md` | 디바이스 기능 표현 방법 |

### 3.2 JSON 스키마 (JSON Schemas)

| 스키마 | 경로 | 설명 |
|-------|------|------|
| GazePoint | `schemas/gaze-point.schema.json` | 시선 데이터 포인트 검증 |
| GazeEvent | `schemas/gaze-event.schema.json` | 시선 이벤트 검증 |
| DeviceCapability | `schemas/device-capability.schema.json` | 디바이스 역량 검증 |

---

## 4. 핵심 설계 결정 (Key Design Decisions)

### 4.1 좌표계

```
정규화 좌표 (0.0 - 1.0)
├── 원점: 화면 좌상단 (0, 0)
├── X축: 오른쪽으로 증가
├── Y축: 아래쪽으로 증가
└── 화면 밖도 표현 가능 (음수 또는 >1.0)
```

**이유**: 해상도 독립성, 모든 제조사 호환

### 4.2 데이터 구조

```typescript
GazePoint (필수 필드)
├── timestamp: number    // Unix ms
├── x, y: number         // 정규화 좌표
├── confidence: number   // 0.0-1.0
└── valid: boolean       // 유효성

GazePoint (선택 필드)
├── leftEye, rightEye    // 개별 눈 데이터
├── fixation, saccade    // 상태 플래그
└── metadata             // 확장 데이터
```

**이유**: 필수 필드 최소화로 호환성 극대화

### 4.3 이벤트 시스템

```
27종 표준 이벤트
├── Eye Movement (6): fixation_*, saccade_*, smooth_pursuit_*
├── Eye State (6): blink_*, wink_*, double_blink
├── Interaction (6): dwell_*, gaze_enter, gaze_leave
└── System (9): calibration_*, tracking_*, device_*
```

**이유**: AAC 사용 사례 완벽 지원

### 4.4 AAC 특화 설계

```
접근성 우선 설계
├── Dwell Selection: 시선 응시로 선택
├── Blink/Wink Input: 깜빡임/윙크 입력
├── Tremor Compensation: 떨림 보정
├── Adaptive Dwell Time: 적응형 응시 시간
└── Fatigue Detection: 피로도 감지
```

---

## 5. 다음 단계 (Next Steps)

### Phase 2: API Interface Standard

```
목표: 표준 API 인터페이스 정의

작업:
├── TypeScript/Python API 설계
├── 디바이스 어댑터 인터페이스
├── 이벤트 리스너 패턴
└── 에러 처리 표준
```

### Phase 3: Communication Protocol

```
목표: 통신 프로토콜 표준화

작업:
├── WebSocket 메시지 포맷
├── REST API 엔드포인트
├── 실시간 스트리밍 프로토콜
└── 동기화 메커니즘
```

### Phase 4: Integration

```
목표: WIA 생태계 통합

작업:
├── WIA AAC Standard 연동
├── SDK 구현 (TypeScript, Python)
├── 데모 애플리케이션
└── 문서 및 예제
```

---

## 6. 파일 구조 (File Structure)

```
eye-gaze/
├── prompts/
│   ├── PHASE-1-PROMPT.md
│   ├── PHASE-2-PROMPT.md
│   ├── PHASE-3-PROMPT.md
│   └── PHASE-4-PROMPT.md
├── spec/
│   ├── RESEARCH-PHASE-1.md          ✅ Complete
│   ├── DATA-FORMAT-SPEC.md          ✅ Complete
│   ├── EVENT-SPEC.md                ✅ Complete
│   └── DEVICE-CAPABILITY-SPEC.md    ✅ Complete
├── schemas/
│   ├── gaze-point.schema.json       ✅ Complete
│   ├── gaze-event.schema.json       ✅ Complete
│   └── device-capability.schema.json ✅ Complete
├── api/
│   ├── typescript/src/              📋 Phase 2
│   └── python/wia_eye_gaze/         📋 Phase 2
├── examples/                        📋 Phase 4
└── docs/
    └── PHASE-1-SUMMARY.md           ✅ Complete (현재 문서)
```

---

## 7. 참고 자료 (References)

### 연구 소스

- [Tobii Pro SDK](https://developer.tobiipro.com/)
- [Gazepoint API v2.0](https://www.gazept.com/dl/Gazepoint_API_v2.0.pdf)
- [Pupil Labs GitHub](https://github.com/pupil-labs/pupil)
- [ATIA Eye Gaze Standards Working Group](https://www.atia.org/eyegazestandards/)
- [W3C WAI WCAG](https://www.w3.org/WAI/standards-guidelines/wcag/)
- [ISO 9241-971:2020](https://www.iso.org/standard/74511.html)

### WIA 생태계

- [WIA Live](https://wia.live)
- [ISP (International Sign Phonetic)](https://wia.live/isp)
- [WIA Braille](https://wia.live/wia-braille)
- [WIA Talk](https://wia.live/wia-talk)

---

## 8. 결론 (Conclusion)

Phase 1에서 WIA Eye Gaze Interoperability Protocol의 핵심 데이터 구조와 이벤트 시스템을 성공적으로 정의했습니다.

### 주요 성과

1. **3개 JSON 스키마** - 검증 가능한 데이터 포맷
2. **27종 표준 이벤트** - AAC 완벽 지원
3. **디바이스 역량 표준** - 자동 검색/협상 가능
4. **TypeScript + Python 타입** - 즉시 구현 가능

### 기대 효과

- 시선 추적 앱들 간 **상호운용성** 확보
- AAC 사용자의 **디바이스 선택권** 확대
- 개발자의 **진입 장벽** 감소
- 시선 추적 **시장 성장** 촉진

---

<div align="center">

**WIA Eye Gaze Standard - Phase 1 Complete**

**弘益人間** - 널리 인간을 이롭게

🤟

**Next: Phase 2 - API Interface Standard**

</div>
