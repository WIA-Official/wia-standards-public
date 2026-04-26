# WIA Eye Gaze Standard - Master Prompt

## 프로젝트 개요

**WIA Eye Gaze Protocol**은 시선 추적 디바이스와 앱 간의 상호운용성을 위한 오픈 표준입니다.

### 핵심 문제
- 전 세계 시선 추적 보조기기 사용자: 수십만 명 (ALS, 뇌성마비, 척수손상 등)
- 제조사마다 독자 프로토콜 → 앱 간 호환성 0
- ATIA 표준화 노력 시작됐지만 아직 초기 단계

### WIA 솔루션
| Phase | 내용 | 산출물 |
|-------|------|--------|
| 1 | 데이터 포맷 표준 | JSON Schema, 스펙 문서 |
| 2 | API Interface | TypeScript/Python/Rust SDK |
| 3 | 실시간 통신 | WebSocket, IPC 프로토콜 |
| 4 | 에코시스템 통합 | OS 접근성 API, 브라우저 확장 |

---

## 빠른 시작

```bash
# Phase 1 시작
cat prompts/PHASE-1-PROMPT.md
# 읽고 spec/ 폴더에 문서 작성

# Phase 2 시작
cat prompts/PHASE-2-PROMPT.md
# 읽고 api/ 폴더에 SDK 구현

# Phase 3 시작
cat prompts/PHASE-3-PROMPT.md
# 읽고 실시간 프로토콜 구현

# Phase 4 시작
cat prompts/PHASE-4-PROMPT.md
# 읽고 통합 모듈 구현
```

---

## 핵심 기술 컨셉

### Gaze Point (시선 포인트)
```typescript
interface GazePoint {
  timestamp: number;      // Unix ms
  x: number;              // 0.0 - 1.0
  y: number;              // 0.0 - 1.0
  confidence: number;     // 0.0 - 1.0
  fixation: boolean;
}
```

### Dwell Selection (응시 선택)
- 시선을 특정 지점에 일정 시간 유지하면 선택
- 기본 800ms, 사용자 조정 가능
- 시각적 피드백 필수

### App Communication
- 시선 추적 앱들이 서로 인식/협력
- 제어권 요청/허용/반환 프로토콜
- 타겟 동기화

---

## 참조 리소스

- [ATIA Eye Gaze Standards Working Group](https://www.atia.org/eyegazestandards/)
- [ISO 9241-971 Eye tracking](https://www.iso.org/standard/78119.html)
- [PupilLabs (오픈소스)](https://pupil-labs.com/)
- [GazePointer (오픈소스)](https://gazepointer.sourceforge.net/)

---

## 철학

**홍익인간 (弘益人間)** - 널리 인간을 이롭게 하라

시선만으로 세상과 소통해야 하는 분들이
어떤 디바이스, 어떤 앱을 사용하든
동일한 경험을 할 수 있도록.
