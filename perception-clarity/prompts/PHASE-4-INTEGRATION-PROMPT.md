# Phase 4: Ecosystem Integration
## Claude Code 작업 프롬프트

---

**Standard**: WIA Perception Clarity
**Phase**: 4 of 4
**목표**: WIA 에코시스템 표준들과 인식 명료도 표준을 통합

---

## 🎯 목표

WIA 피지컬 AI 표준들(robot·auto·drone·lidar-sensor·vision-ai·ai-sensor-fusion)과 인식 명료도(PCI·상태·보고)를 연동합니다. 에이전트의 명료도 상태가 플릿 운영·센서 융합·안전 동작에 흘러들도록 합니다.

---

## 📋 웹서치 키워드

```
sensor contamination, lens soiling autonomous, lidar window cleaning, camera occlusion detection, VDA5050, MTF degradation
```

---

## 🔄 통합 대상

```
4.1 WIA Robot / Auto / Drone 연동
    - 에이전트 상태 머신에 ClarityState 반영
    - blind/obstructed 시 안전 동작 트리거(감속·우회·safe-state)

4.2 WIA LiDAR-Sensor 연동
    - lidar_window PCI 산출 입력(occlusion·유효 인식거리)

4.3 WIA Vision-AI 연동
    - 카메라 MTF/대비 저하를 PCI 축으로 공급
    - 인식 신뢰도와 PCI confidence 정합

4.4 WIA AI-Sensor-Fusion 연동
    - 센서별 PCI를 융합 가중치로 활용(낮은 PCI = 낮은 신뢰 가중)

4.5 Fleet / VDA5050 연동
    - SensorClarityReport를 플릿 메시지 모델로 송출
    - dwell-time SLA 모니터링·세척 작업 디스패치
```

---

## 📦 산출물

```
/perception-clarity/
├── prompts/
│   └── PHASE-4-INTEGRATION-PROMPT.md
├── spec/
│   └── PHASE-4-INTEGRATION.md
├── api/rust/src/
│   ├── integrations/
│   │   ├── mod.rs
│   │   ├── robot.rs            # Robot/Auto/Drone 연동
│   │   ├── lidar_sensor.rs     # LiDAR-Sensor 연동
│   │   ├── vision_ai.rs        # Vision-AI 연동
│   │   ├── sensor_fusion.rs    # AI-Sensor-Fusion 연동
│   │   └── fleet.rs            # Fleet/VDA5050 연동
│   └── cli/
│       ├── mod.rs
│       └── commands.rs         # CLI 명령어
└── examples/
    ├── safe_action_trigger.rs
    ├── fusion_weighting.rs
    └── fleet_reporting.rs
```

---

## 🔄 작업 순서

```
1. PHASE-4-INTEGRATION.md 작성
2. integrations 모듈 구현 (robot·lidar·vision·fusion·fleet)
3. CLI 명령어 구현
4. 테스트 작성
5. 예제 코드 작성
```

---

## ✅ 완료 체크리스트

```
□ 데이터 포맷 (Phase 1)
□ Rust API (Phase 2)
□ 프로토콜 명세 (Phase 3)
□ Robot/Auto/Drone 연동 (Phase 4)
□ LiDAR-Sensor 연동 (Phase 4)
□ Vision-AI 연동 (Phase 4)
□ AI-Sensor-Fusion 연동 (Phase 4)
□ Fleet/VDA5050 연동 (Phase 4)
□ CLI 도구 (Phase 4)
```

---

弘益人間 🤟
