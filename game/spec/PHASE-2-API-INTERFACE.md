# WIA Game - Phase 2: API Interface

## Version
- **Version**: 1.0.0
- **Date**: 2025-12-16
- **Status**: Final

## Overview

본 문서는 WIA Game 접근성 표준의 API 인터페이스를 정의합니다.

## Rust SDK

완전한 Rust 구현이 제공됩니다:

**위치**: `api/rust/`

### 주요 컴포넌트

#### 1. 게임 접근성 제어
- **입력 맵핑**: 커스텀 입력 매핑
- **난이도 조절**: 동적 난이도 조정
- **UI 조정**: 폰트 크기, 색상 대비
- **자막**: 실시간 음성 자막

#### 2. 센서 통합
- **BCI**: 뇌파로 게임 제어
- **Eye Gaze**: 시선 추적 입력
- **Voice**: 음성 명령
- **Haptic**: 촉각 피드백

#### 3. 접근성 프로필
- **시각 장애**: 음성 안내, 오디오 큐
- **청각 장애**: 시각 자막, 진동 알림
- **운동 장애**: 단순화 입력, BCI
- **인지 장애**: 단순화 UI, 자동 진행

#### 4. WIA 통합
- **BCI 게임 제어**: 생각으로 플레이
- **Voice-Sign**: 음성/수어 명령
- **Smart Wheelchair**: 접근 가능한 게임 스테이션
- **AAC**: 다중 의사소통 방식

## API 구조

```rust
// 게임 접근성 제어
pub trait GameAccessibility {
    fn apply_profile(&mut self, profile: AccessibilityProfile) -> Result<()>;
    fn adjust_difficulty(&mut self, level: DifficultyLevel) -> Result<()>;
}

// 센서 입력
pub trait InputAdapter {
    fn read_input(&self) -> Result<GameInput>;
    fn configure(&mut self, config: InputConfig) -> Result<()>;
}

// 접근성 피드백
pub trait FeedbackSystem {
    fn audio_cue(&mut self, cue: AudioCue) -> Result<()>;
    fn haptic_feedback(&mut self, pattern: HapticPattern) -> Result<()>;
    fn visual_indicator(&mut self, indicator: VisualIndicator) -> Result<()>;
}
```

## 통신 프로토콜

### WebSocket
```
ws://game.server/accessibility
- 실시간 접근성 설정 동기화
- 멀티플레이어 접근성 지원
```

### REST API
```
GET    /profiles
GET    /profiles/{userId}
POST   /profiles/{userId}
PUT    /profiles/{userId}/settings
```

### gRPC
```
service GameAccessibility {
  rpc ApplyProfile(ProfileRequest) returns (ProfileResponse);
  rpc AdjustDifficulty(DifficultyRequest) returns (DifficultyResponse);
}
```

## 인증 & 보안

### OAuth 2.0
- 사용자 인증
- 프로필 동기화
- 크로스 플랫폼 지원

### 데이터 보호
- 프로필 암호화
- 로컬 저장
- 클라우드 동기화

## 에러 처리

```rust
pub enum GameAccessibilityError {
    ProfileNotFound,
    DeviceNotAvailable,
    ConfigurationFailed,
    SyncError,
}
```

## 예제

```rust
use wia_game::*;

// 접근성 프로필 적용
let mut game = Game::new();
let profile = AccessibilityProfile::for_visual_impairment();
game.apply_profile(profile)?;

// BCI 입력
let bci = BCIAdapter::new();
let input = bci.read_input()?;
game.process_input(input)?;

// 햅틱 피드백
let haptic = HapticFeedback::new();
haptic.play_pattern(HapticPattern::Success)?;
```

---

**Author**: Yeon Sam-Heum, Ph.D.  
**License**: MIT  
**弘益人間** - Benefit All Humanity
