# Phase 3: Communication Protocol
## Claude Code 작업 프롬프트

---

**Standard**: WIA Game
**Phase**: 3 of 4
**Language**: **Rust** (Primary)
**목표**: 적응형 컨트롤러 및 입력 장치 통신 프로토콜

---

## 🎯 목표

게임 접근성 기기와의 통신 프로토콜 정의 및 구현

---

## 📋 웹서치 키워드

```
Xbox Adaptive Controller protocol, HID gamepad protocol, USB HID input, Switch access gaming, Eye tracker gaming API, Tobii eye tracking SDK
```

---

## 🔧 핵심 기능

```
1. HID 프로토콜 레이어
   - USB HID 게임패드 통신
   - Bluetooth HID 지원
   - 커스텀 HID 리포트

2. 적응형 컨트롤러 지원
   - Xbox Adaptive Controller
   - PlayStation Access Controller
   - Switch/Button Arrays
   - Joystick Interfaces

3. 대체 입력 장치
   - Eye Trackers (Tobii, etc.)
   - Head Trackers
   - Sip-and-Puff devices
   - Voice Control

4. 이벤트 시스템
   - 입력 이벤트 표준화
   - 디바운싱/필터링
   - 매크로 실행
```

---

## 📦 프로젝트 구조

```
/api/rust/src/
├── protocol/
│   ├── mod.rs           # 프로토콜 모듈
│   ├── hid.rs           # HID 프로토콜
│   ├── event.rs         # 이벤트 시스템
│   ├── device.rs        # 디바이스 추상화
│   └── adapters/
│       ├── mod.rs
│       ├── xbox.rs      # Xbox Adaptive
│       ├── playstation.rs
│       ├── switch.rs    # Switch Access
│       └── eye_tracker.rs
```

---

## 🔄 작업 순서

```
1. 웹서치로 기술 조사
2. /spec/PHASE-3-PROTOCOL.md 작성
3. protocol 모듈 구현
4. 디바이스 어댑터 구현
5. 이벤트 시스템 구현
6. 테스트 작성
7. README 업데이트
```

---

## 📁 산출물

```
/game/spec/PHASE-3-PROTOCOL.md
/game/api/rust/src/protocol/*.rs
/game/api/rust/tests/protocol_test.rs
```

---

## ✅ 완료 체크리스트

```
□ 웹서치로 기술 조사 완료
□ PHASE-3-PROTOCOL.md 작성
□ HID 프로토콜 구현
□ 이벤트 시스템 구현
□ 디바이스 어댑터 구현
□ 테스트 통과
□ README 업데이트 (Phase 3 완료)
```

---

弘益人間 🤟🎮
