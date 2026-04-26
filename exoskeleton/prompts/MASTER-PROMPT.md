# WIA Exoskeleton Standard - Master Prompt

## 프로젝트 개요

**WIA Exoskeleton Protocol**은 재활 외골격 로봇의 데이터 및 제어 표준입니다.

### 핵심 문제
- [데이터 수집/분석 표준 없음](https://www.biorxiv.org/content/10.1101/2024.10.02.616295v1.full.pdf)
- 성능 표준 없음 (안전 표준만 있음)
- 고도로 전문화된 시스템 → 상호운용성 0
- 연구 성과 → 상용화 단절

### WIA 솔루션

| Phase | 내용 | 산출물 |
|-------|------|--------|
| 1 | 운동 데이터 표준 | 관절 각도, 토크, 보행 데이터 |
| 2 | 제어 명령 표준 | 모터 제어 인터페이스 |
| 3 | 안전 프로토콜 | 비상 정지, 한계값 |
| 4 | 재활 프로토콜 | 훈련 프로그램, 진척도 |

---

## 핵심 기술

```typescript
interface ExoskeletonData {
  timestamp: number;
  joints: JointState[];
  gait: GaitPhase;
  assistanceLevel: number;     // 0-100%
  userIntent: IntentSignal;
}

interface JointState {
  joint: 'hip' | 'knee' | 'ankle';
  side: 'left' | 'right';
  angle: number;               // degrees
  angularVelocity: number;     // deg/s
  torque: number;              // Nm
  assistTorque: number;        // 외골격이 제공하는 토크
}

enum GaitPhase {
  STANCE = 'stance',
  SWING = 'swing',
  HEEL_STRIKE = 'heel_strike',
  TOE_OFF = 'toe_off',
  DOUBLE_SUPPORT = 'double_support',
}
```

---

## 4-Phase 요약

### Phase 1: 운동 데이터 표준
- 관절 상태 (각도, 속도, 토크)
- 보행 주기 감지
- IMU/Force 센서 데이터

### Phase 2: 제어 명령 표준
- 위치/속도/토크 제어 모드
- 임피던스 제어 파라미터
- 사용자 의도 기반 제어

### Phase 3: 안전 프로토콜
- 비상 정지 (E-Stop)
- ROM 한계값
- 과부하 감지
- 배터리 관리

### Phase 4: 재활 프로토콜
- 훈련 프로그램 정의
- 진척도 측정 메트릭
- 치료사 대시보드

---

## 참조 리소스

- [OpenExo](https://www.biorxiv.org/content/10.1101/2024.10.02.616295v1.full.pdf) - 오픈소스 외골격
- [ASTM F48](https://pmc.ncbi.nlm.nih.gov/articles/PMC6604650/) - 외골격 표준 위원회

---

## 철학

**홍익인간 (弘益人間)** - 널리 인간을 이롭게 하라

다시 걸을 수 있는 기회를 더 많은 사람들에게.
