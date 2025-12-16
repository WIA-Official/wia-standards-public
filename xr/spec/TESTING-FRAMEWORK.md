# WIA XR Accessibility: Testing Framework Specification

## 1. Overview

본 문서는 XR 접근성 시스템의 테스트 프레임워크를 정의합니다.

**Version:** 1.0.0
**Status:** Draft
**Phase:** 3 - Safety & Quality Protocol

---

## 2. Test Architecture

### 2.1 Testing Pyramid

```
                    ┌─────────────┐
                    │   E2E       │  ← 사용자 시나리오
                    │   Tests     │     (10%)
                   ─┴─────────────┴─
                  ┌─────────────────┐
                  │  Integration    │  ← 컴포넌트 통합
                  │     Tests       │     (20%)
                 ─┴─────────────────┴─
                ┌───────────────────────┐
                │      Unit Tests       │  ← 개별 함수/모듈
                │                       │     (70%)
                └───────────────────────┘
```

### 2.2 Test Categories

```typescript
interface XRAccessibilityTestFramework {
  // 단위 테스트
  unitTests: {
    coverage: '≥ 80%';
    focus: [
      'profile_parsing',
      'adaptation_logic',
      'type_validation',
      'error_handling',
      'utility_functions'
    ];
  };

  // 통합 테스트
  integrationTests: {
    coverage: '≥ 70%';
    focus: [
      'engine_to_adapter',
      'profile_to_adaptation',
      'session_management',
      'wia_device_communication',
      'event_propagation'
    ];
  };

  // E2E 테스트
  e2eTests: {
    scenarios: [
      'complete_user_journey',
      'accessibility_feature_activation',
      'emergency_procedures',
      'cross_platform_sync'
    ];
  };

  // 접근성 특화 테스트
  accessibilityTests: {
    automated: AccessibilityAutomatedTests;
    manual: AccessibilityManualTests;
    userTesting: UserAccessibilityTesting;
  };
}
```

---

## 3. Unit Testing

### 3.1 Rust Unit Tests

```rust
// tests/profile_tests.rs

#[cfg(test)]
mod profile_tests {
    use wia_xr_accessibility::*;

    #[test]
    fn test_profile_creation() {
        let profile = ProfileBuilder::new("test-user")
            .with_visual_disability(VisualLevel::LowVision)
            .with_caption_settings(CaptionSettings::default())
            .build();

        assert_eq!(profile.profile_id, "test-user");
        assert!(profile.disabilities.visual.level == VisualLevel::LowVision);
    }

    #[test]
    fn test_profile_validation() {
        let invalid_json = r#"{"invalid": "data"}"#;
        let result = XRAccessibilityProfile::from_json(invalid_json);
        assert!(result.is_err());
    }

    #[test]
    fn test_adaptation_priority() {
        let config = AdaptationConfig::new(AdaptationType::ScreenReader);
        assert_eq!(config.priority, AdaptationPriority::Critical);

        let config = AdaptationConfig::new(AdaptationType::Magnification);
        assert_eq!(config.priority, AdaptationPriority::Medium);
    }

    #[tokio::test]
    async fn test_session_health_monitoring() {
        let session = SessionManager::new(SessionConfig::default());
        session.start().await.unwrap();

        // 30분 시뮬레이션
        session.simulate_duration(Duration::from_secs(1800)).await;

        let health = session.get_health_status().await;
        assert!(health.rest_reminder_due);
    }
}
```

### 3.2 Type Validation Tests

```rust
// tests/type_tests.rs

#[cfg(test)]
mod type_tests {
    use wia_xr_accessibility::types::*;
    use serde_json;

    #[test]
    fn test_visual_disability_serialization() {
        let visual = VisualDisability {
            level: VisualLevel::LegallyBlind,
            color_vision: ColorVisionType::Deuteranopia,
            // ...
        };

        let json = serde_json::to_string(&visual).unwrap();
        let parsed: VisualDisability = serde_json::from_str(&json).unwrap();
        assert_eq!(visual.level, parsed.level);
    }

    #[test]
    fn test_device_capabilities_defaults() {
        let device = XRDeviceCapabilities::default();
        assert!(device.display.refresh_rates.contains(&90.0));
        assert!(device.built_in_accessibility.screen_reader);
    }

    #[test]
    fn test_comfort_settings_range() {
        let comfort = ComfortSettings::default();
        assert!(comfort.motion.vignette_intensity >= 0.0);
        assert!(comfort.motion.vignette_intensity <= 1.0);
    }
}
```

### 3.3 Error Handling Tests

```rust
// tests/error_tests.rs

#[cfg(test)]
mod error_tests {
    use wia_xr_accessibility::error::*;

    #[test]
    fn test_error_display() {
        let error = XRAccessibilityError::ProfileNotFound("test-id".into());
        assert!(error.to_string().contains("test-id"));
    }

    #[test]
    fn test_error_conversion() {
        let io_error = std::io::Error::new(
            std::io::ErrorKind::NotFound,
            "file not found"
        );
        let xr_error: XRAccessibilityError = io_error.into();
        assert!(matches!(xr_error, XRAccessibilityError::Io(_)));
    }

    #[test]
    fn test_validation_error_details() {
        let error = ValidationError::InvalidField {
            field: "brightness".into(),
            reason: "must be between 0 and 1".into(),
        };
        assert!(error.field() == "brightness");
    }
}
```

---

## 4. Integration Testing

### 4.1 Engine Integration Tests

```rust
// tests/integration/engine_tests.rs

#[cfg(test)]
mod engine_integration_tests {
    use wia_xr_accessibility::*;

    #[tokio::test]
    async fn test_full_engine_lifecycle() {
        // 엔진 초기화
        let engine = XRAccessibilityEngine::new();

        // 프로필 로드
        let profile = ProfileBuilder::new("integration-test")
            .with_caption_settings(CaptionSettings {
                enabled: true,
                ..Default::default()
            })
            .build();

        engine.load_profile(profile).await.unwrap();

        // 적응 기능 확인
        let adaptations = engine.get_active_adaptations().await;
        assert!(adaptations.contains_key(&AdaptationType::Captions));

        // 이벤트 핸들러 등록
        let (tx, mut rx) = tokio::sync::mpsc::channel(10);
        engine.add_event_handler(move |event| {
            let tx = tx.clone();
            async move {
                tx.send(event).await.ok();
            }
        }).await;

        // 설정 변경
        let mut new_captions = CaptionSettings::default();
        new_captions.font_size = 24;
        engine.update_caption_settings(new_captions).await.unwrap();

        // 이벤트 수신 확인
        let event = rx.recv().await.unwrap();
        assert!(matches!(event, XREvent::CaptionSettingsChanged { .. }));

        // 정리
        engine.unload_profile().await.unwrap();
    }

    #[tokio::test]
    async fn test_adaptation_manager_integration() {
        let manager = AdaptationManager::new(50.0); // 50% 성능 예산

        // 여러 적응 기능 등록
        let caption = Arc::new(CaptionAdaptation::new(CaptionSettings::default()));
        let audio_desc = Arc::new(AudioDescriptionAdaptation::new(
            AudioDescriptionSettings::default()
        ));

        manager.register(caption, AdaptationConfig::new(AdaptationType::Captions)).await;
        manager.register(audio_desc, AdaptationConfig::new(AdaptationType::AudioDescription)).await;

        // 프로필 기반 추천
        let profile = ProfileBuilder::new("test").build();
        let device = XRDeviceCapabilities::default();
        let recommendations = manager.get_recommendations(&profile, &device).await;

        assert!(!recommendations.is_empty());
    }
}
```

### 4.2 WIA Device Integration Tests

```rust
// tests/integration/wia_tests.rs

#[cfg(test)]
mod wia_integration_tests {
    use wia_xr_accessibility::adapters::wia::*;

    #[tokio::test]
    async fn test_wia_hub_integration() {
        let (mut hub, mut rx) = WIAIntegrationHub::with_events();

        // Mock 어댑터 등록
        let mut exo = MockExoskeletonAdapter::new();
        exo.connect("exo-001").await.unwrap();
        hub.register_exoskeleton(Arc::new(exo)).await;

        let mut vs = MockVoiceSignAdapter::new();
        vs.connect().await.unwrap();
        hub.register_voice_sign(Arc::new(vs)).await;

        // 상태 확인
        let state = hub.get_sync_state().await;
        assert!(state.exoskeleton_connected);
        assert!(state.voice_sign_connected);

        // 프로필 동기화
        let profile = ProfileBuilder::new("wia-test")
            .with_wia_integrations(WIAIntegrations {
                exoskeleton: Some(ExoskeletonIntegration::default()),
                voice_sign: Some(VoiceSignIntegration::default()),
                ..Default::default()
            })
            .build();

        hub.sync_profile(&profile).await.unwrap();

        // 이벤트 확인
        while let Ok(event) = rx.try_recv() {
            if matches!(event, WIAEvent::SyncCompleted) {
                return; // 성공
            }
        }
        panic!("SyncCompleted event not received");
    }

    #[tokio::test]
    async fn test_haptic_command_flow() {
        let (mut hub, _rx) = WIAIntegrationHub::with_events();

        let mut exo = MockExoskeletonAdapter::new();
        exo.connect("exo-001").await.unwrap();
        hub.register_exoskeleton(Arc::new(exo)).await;

        let command = HapticCommand {
            target: HapticTarget::LeftHand,
            intensity: 0.5,
            duration_ms: 100,
        };

        hub.send_haptic(command).await.unwrap();
    }

    #[tokio::test]
    async fn test_voice_sign_translation() {
        let (mut hub, _rx) = WIAIntegrationHub::with_events();

        let mut vs = MockVoiceSignAdapter::new();
        vs.connect().await.unwrap();
        hub.register_voice_sign(Arc::new(vs)).await;

        let result = hub.translate_to_sign("hello world").await.unwrap();
        assert_eq!(result.len(), 2);
    }
}
```

### 4.3 Session Integration Tests

```rust
// tests/integration/session_tests.rs

#[cfg(test)]
mod session_integration_tests {
    use wia_xr_accessibility::core::session::*;

    #[tokio::test]
    async fn test_session_lifecycle() {
        let config = SessionConfig {
            limits: SessionLimits {
                max_duration_minutes: 120,
                break_interval_minutes: 30,
                break_duration_minutes: 5,
                enforce_breaks: false,
            },
            health_monitoring: true,
        };

        let session = SessionManager::new(config);
        let session_id = session.start().await.unwrap();
        assert!(!session_id.is_empty());

        // 건강 상태 확인
        let health = session.get_health_status().await;
        assert!(health.fatigue_level < 0.1);

        // 세션 일시정지
        session.pause().await.unwrap();
        let state = session.get_state().await;
        assert!(matches!(state, SessionState::Paused));

        // 재개
        session.resume().await.unwrap();
        let state = session.get_state().await;
        assert!(matches!(state, SessionState::Active));

        // 종료
        session.end().await.unwrap();
    }

    #[tokio::test]
    async fn test_break_reminder_system() {
        let config = SessionConfig {
            limits: SessionLimits {
                break_interval_minutes: 1,  // 테스트용 1분
                ..Default::default()
            },
            health_monitoring: true,
        };

        let (session, mut rx) = SessionManager::with_events(config);
        session.start().await.unwrap();

        // 1분 후 휴식 알림 대기
        tokio::time::timeout(
            Duration::from_secs(70),
            async {
                while let Some(event) = rx.recv().await {
                    if matches!(event, SessionEvent::RestReminderTriggered { .. }) {
                        return;
                    }
                }
            }
        ).await.expect("Rest reminder should trigger");

        session.end().await.unwrap();
    }
}
```

---

## 5. E2E Testing

### 5.1 E2E Test Scenarios

```yaml
e2e_scenarios:
  user_onboarding:
    name: "신규 사용자 온보딩"
    steps:
      - action: launch_application
        expected: welcome_screen_displayed

      - action: select_accessibility_profile_type
        input: visual_impairment
        expected: visual_options_displayed

      - action: configure_captions
        input:
          enabled: true
          font_size: 24
          background_opacity: 0.8
        expected: caption_preview_shown

      - action: complete_setup
        expected: main_experience_loaded

      - action: verify_captions_active
        expected: captions_displayed_correctly

  emergency_exit:
    name: "비상 탈출 기능"
    steps:
      - action: start_immersive_content
        expected: content_playing

      - action: trigger_panic_exit
        input: double_menu_button
        expected:
          - immediate_passthrough
          - audio_confirmation
          - haptic_feedback

      - action: verify_safe_state
        expected:
          - session_paused
          - user_in_safe_space

  wia_device_sync:
    name: "WIA 기기 동기화"
    steps:
      - action: connect_exoskeleton
        expected: device_recognized

      - action: load_profile_with_wia
        expected: settings_synced_to_device

      - action: test_haptic_feedback
        expected: haptic_received_on_device

      - action: disconnect_device
        expected: graceful_fallback
```

### 5.2 E2E Test Implementation

```typescript
// e2e/tests/accessibility.spec.ts

import { XRTestHarness, AccessibilityProfile } from '@wia/xr-test-harness';

describe('XR Accessibility E2E Tests', () => {
  let harness: XRTestHarness;

  beforeEach(async () => {
    harness = await XRTestHarness.create({
      platform: 'quest',
      mockDevices: true,
    });
  });

  afterEach(async () => {
    await harness.cleanup();
  });

  describe('Caption System', () => {
    test('should display captions with correct styling', async () => {
      // 프로필 로드
      const profile = AccessibilityProfile.withCaptions({
        enabled: true,
        fontSize: 24,
        fontColor: '#FFFFFF',
        backgroundColor: '#000000',
        backgroundOpacity: 0.8,
      });

      await harness.loadProfile(profile);

      // 오디오 재생
      await harness.playAudio('test-dialogue.mp3');

      // 자막 확인
      const caption = await harness.waitForCaption({ timeout: 5000 });
      expect(caption.text).toBeTruthy();
      expect(caption.style.fontSize).toBe(24);
      expect(caption.style.backgroundColor).toContain('rgba(0, 0, 0, 0.8)');

      // 자막 위치 확인
      const position = await harness.getCaptionPosition();
      expect(position.distanceMeters).toBeCloseTo(2.0, 1);
    });

    test('should support multiple speakers', async () => {
      const profile = AccessibilityProfile.withCaptions({
        enabled: true,
        showSpeakerName: true,
      });

      await harness.loadProfile(profile);
      await harness.playMultiSpeakerDialogue();

      const captions = await harness.collectCaptions({ duration: 10000 });
      const speakers = new Set(captions.map(c => c.speaker));
      expect(speakers.size).toBeGreaterThan(1);
    });
  });

  describe('Motion Comfort', () => {
    test('should apply vignette during movement', async () => {
      const profile = AccessibilityProfile.withComfort({
        vignetteEnabled: true,
        vignetteIntensity: 0.5,
        vignetteOnMovement: true,
      });

      await harness.loadProfile(profile);

      // 정지 상태 - 비네트 없음
      let vignette = await harness.getVignetteState();
      expect(vignette.active).toBe(false);

      // 이동 시작
      await harness.simulateMovement({ direction: 'forward', speed: 2.0 });

      // 비네트 활성화 확인
      vignette = await harness.getVignetteState();
      expect(vignette.active).toBe(true);
      expect(vignette.intensity).toBeCloseTo(0.5, 1);

      // 이동 정지
      await harness.stopMovement();
      await harness.wait(1000);

      // 비네트 비활성화 확인
      vignette = await harness.getVignetteState();
      expect(vignette.active).toBe(false);
    });
  });

  describe('Emergency Exit', () => {
    test('should trigger panic exit correctly', async () => {
      await harness.loadProfile(AccessibilityProfile.default());
      await harness.startImmersiveContent();

      // 비상 탈출 트리거
      const exitStart = Date.now();
      await harness.triggerPanicExit();
      const exitDuration = Date.now() - exitStart;

      // 즉시 반응 확인 (500ms 이내)
      expect(exitDuration).toBeLessThan(500);

      // 상태 확인
      const state = await harness.getSessionState();
      expect(state.paused).toBe(true);
      expect(state.inSafeSpace).toBe(true);

      // 피드백 확인
      const feedback = await harness.getLastFeedback();
      expect(feedback.audio).toBe(true);
      expect(feedback.haptic).toBe(true);
    });
  });
});
```

---

## 6. Accessibility-Specific Testing

### 6.1 Screen Reader Testing

```typescript
interface ScreenReaderTestSuite {
  tests: {
    // 기본 호환성
    basicCompatibility: {
      name: 'Screen Reader Basic Compatibility';
      checks: [
        'all_ui_elements_labeled',
        'logical_reading_order',
        'meaningful_link_text',
        'form_labels_associated',
      ];
    };

    // 내비게이션
    navigation: {
      name: 'Screen Reader Navigation';
      checks: [
        'landmark_regions_defined',
        'heading_structure_correct',
        'skip_links_functional',
        'focus_management_correct',
      ];
    };

    // 동적 콘텐츠
    dynamicContent: {
      name: 'Dynamic Content Announcements';
      checks: [
        'status_updates_announced',
        'error_messages_announced',
        'loading_states_announced',
        'live_regions_configured',
      ];
    };

    // XR 특수 요소
    xrSpecific: {
      name: 'XR-Specific Screen Reader Features';
      checks: [
        'spatial_audio_descriptions',
        '3d_object_announcements',
        'distance_information_provided',
        'orientation_cues_available',
      ];
    };
  };

  tools: ['NVDA', 'JAWS', 'VoiceOver', 'TalkBack'];

  passRate: '100%';
}
```

### 6.2 Motor Accessibility Testing

```yaml
motor_accessibility_tests:
  one_handed_operation:
    description: "한 손으로 모든 기능 접근 가능"
    tests:
      - name: menu_access
        action: open_menu_one_handed
        expected: menu_opens_successfully

      - name: content_navigation
        action: navigate_content_one_handed
        expected: all_content_reachable

      - name: settings_adjustment
        action: change_settings_one_handed
        expected: settings_changeable

  dwell_activation:
    description: "시선 응시로 활성화"
    tests:
      - name: dwell_button_activation
        action: gaze_at_button
        duration_ms: 500
        expected: button_activates

      - name: dwell_cancel
        action: look_away_before_timeout
        expected: activation_cancelled

  voice_control:
    description: "음성 명령 제어"
    tests:
      - name: basic_commands
        commands: ['menu', 'back', 'select', 'scroll down']
        expected: all_commands_recognized

      - name: custom_commands
        action: define_custom_command
        expected: custom_command_works

      - name: continuous_dictation
        action: dictate_text
        expected: text_captured_accurately

  switch_access:
    description: "스위치 접근 지원"
    tests:
      - name: scanning_navigation
        action: use_auto_scan
        expected: focus_moves_correctly

      - name: selection
        action: activate_switch
        expected: item_selected
```

### 6.3 Cognitive Accessibility Testing

```yaml
cognitive_accessibility_tests:
  simplified_ui:
    tests:
      - name: reduced_options
        expected: max_5_options_per_screen

      - name: clear_labels
        expected: simple_language_used

      - name: consistent_layout
        expected: same_layout_across_screens

  pause_functionality:
    tests:
      - name: pause_anytime
        action: pause_at_random_point
        expected: content_pauses_immediately

      - name: resume_context
        action: resume_after_pause
        expected: context_preserved

  safe_space:
    tests:
      - name: instant_access
        action: trigger_safe_space
        expected: transition_under_2_seconds

      - name: calming_environment
        expected:
          - no_sudden_sounds
          - no_bright_flashes
          - soothing_colors

  content_warnings:
    tests:
      - name: warning_before_trigger
        expected: warning_displayed_before_content

      - name: skip_option
        expected: option_to_skip_provided
```

---

## 7. Performance Testing

### 7.1 Benchmark Suite

```rust
// benches/accessibility_benchmarks.rs

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use wia_xr_accessibility::*;

fn profile_loading_benchmark(c: &mut Criterion) {
    let profile_json = include_str!("fixtures/complex_profile.json");

    c.bench_function("profile_load_parse", |b| {
        b.iter(|| {
            let profile: XRAccessibilityProfile =
                serde_json::from_str(black_box(profile_json)).unwrap();
            black_box(profile)
        })
    });
}

fn adaptation_application_benchmark(c: &mut Criterion) {
    let rt = tokio::runtime::Runtime::new().unwrap();

    c.bench_function("adaptation_apply", |b| {
        b.to_async(&rt).iter(|| async {
            let manager = AdaptationManager::new(50.0);
            let profile = ProfileBuilder::new("bench").build();
            let device = XRDeviceCapabilities::default();

            let recs = manager.get_recommendations(&profile, &device).await;
            black_box(recs)
        })
    });
}

fn caption_rendering_benchmark(c: &mut Criterion) {
    c.bench_function("caption_style_calculation", |b| {
        b.iter(|| {
            let settings = CaptionSettings {
                font_size: 24,
                font_color: "#FFFFFF".into(),
                background_color: "#000000".into(),
                background_opacity: 0.8,
                ..Default::default()
            };

            let computed = compute_caption_style(black_box(&settings));
            black_box(computed)
        })
    });
}

criterion_group!(
    benches,
    profile_loading_benchmark,
    adaptation_application_benchmark,
    caption_rendering_benchmark
);
criterion_main!(benches);
```

### 7.2 Stress Testing

```typescript
// stress/session_stress.test.ts

describe('Session Stress Tests', () => {
  test('should handle 1000 profile switches', async () => {
    const engine = new XRAccessibilityEngine();

    const profiles = Array.from({ length: 1000 }, (_, i) =>
      ProfileBuilder.new(`stress-test-${i}`).build()
    );

    const startTime = Date.now();

    for (const profile of profiles) {
      await engine.loadProfile(profile);
    }

    const duration = Date.now() - startTime;

    // 평균 로드 시간 < 50ms
    expect(duration / profiles.length).toBeLessThan(50);
  });

  test('should maintain frame rate under load', async () => {
    const harness = await XRTestHarness.create();

    // 모든 접근성 기능 활성화
    const profile = ProfileBuilder.new('stress')
      .withAllAccessibilityFeatures()
      .build();

    await harness.loadProfile(profile);

    // 5분간 프레임 레이트 모니터링
    const frameRates = await harness.monitorFrameRate({ duration: 300000 });

    const avgFrameRate = frameRates.reduce((a, b) => a + b) / frameRates.length;
    const minFrameRate = Math.min(...frameRates);

    expect(avgFrameRate).toBeGreaterThan(72);
    expect(minFrameRate).toBeGreaterThan(60);
  });

  test('should handle concurrent WIA connections', async () => {
    const hub = new WIAIntegrationHub();

    // 3개 기기 동시 연결
    await Promise.all([
      hub.registerExoskeleton(new MockExoskeletonAdapter()),
      hub.registerBionicEye(new MockBionicEyeAdapter()),
      hub.registerVoiceSign(new MockVoiceSignAdapter()),
    ]);

    // 동시 명령 전송
    const results = await Promise.all([
      hub.sendHaptic({ target: 'left_hand', intensity: 0.5 }),
      hub.translateToSign('test message'),
      hub.syncProfile(testProfile),
    ]);

    expect(results.every(r => r.success)).toBe(true);
  });
});
```

---

## 8. Test Automation CI/CD

### 8.1 GitHub Actions Workflow

```yaml
# .github/workflows/accessibility-tests.yml

name: XR Accessibility Tests

on:
  push:
    paths:
      - 'xr/**'
  pull_request:
    paths:
      - 'xr/**'

jobs:
  unit-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Setup Rust
        uses: actions-rs/toolchain@v1
        with:
          toolchain: stable

      - name: Run Unit Tests
        run: |
          cd xr/api/rust
          cargo test --all-features

      - name: Check Coverage
        run: |
          cargo install cargo-tarpaulin
          cargo tarpaulin --out Xml
          # 최소 80% 커버리지 요구

  integration-tests:
    runs-on: ubuntu-latest
    needs: unit-tests
    steps:
      - uses: actions/checkout@v4

      - name: Run Integration Tests
        run: |
          cd xr/api/rust
          cargo test --test '*' --features integration

  accessibility-audit:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Run Accessibility Checks
        run: |
          npm install -g @axe-core/cli
          npx axe --rules wcag2a,wcag2aa xr/demo/index.html

  lint-and-format:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Rust Lint
        run: |
          cd xr/api/rust
          cargo clippy -- -D warnings
          cargo fmt -- --check

  benchmark:
    runs-on: ubuntu-latest
    if: github.event_name == 'push' && github.ref == 'refs/heads/main'
    steps:
      - uses: actions/checkout@v4

      - name: Run Benchmarks
        run: |
          cd xr/api/rust
          cargo bench -- --save-baseline main
```

### 8.2 Test Reporting

```yaml
test_reporting:
  format: junit_xml

  reports:
    - name: unit_test_report
      path: target/test-results/unit-*.xml

    - name: integration_test_report
      path: target/test-results/integration-*.xml

    - name: accessibility_audit_report
      path: target/accessibility-audit.json

  notifications:
    on_failure:
      - slack: '#xr-accessibility'
      - email: accessibility-team@wia.org

    on_success:
      - update_badge: true

  artifacts:
    - coverage_report
    - benchmark_results
    - accessibility_audit
```

---

## 9. Test Data Management

### 9.1 Test Fixtures

```
tests/fixtures/
├── profiles/
│   ├── minimal.json           # 최소 프로필
│   ├── visual_impairment.json # 시각 장애 프로필
│   ├── hearing_impairment.json
│   ├── motor_impairment.json
│   ├── cognitive_impairment.json
│   ├── multiple_disabilities.json
│   └── wia_integration.json   # WIA 기기 통합 프로필
├── devices/
│   ├── quest_3.json
│   ├── vision_pro.json
│   └── generic_vr.json
├── content/
│   ├── test_dialogue.mp3
│   ├── test_scene.json
│   └── test_environment.json
└── expected_outputs/
    ├── captions/
    ├── adaptations/
    └── audio_descriptions/
```

### 9.2 Fixture Generator

```rust
// tests/support/fixtures.rs

pub mod fixtures {
    use wia_xr_accessibility::*;

    pub fn visual_impairment_profile() -> XRAccessibilityProfile {
        ProfileBuilder::new("fixture-visual")
            .with_visual_disability(VisualDisability {
                level: VisualLevel::LowVision,
                color_vision: ColorVisionType::Normal,
                field_of_vision: FieldOfVision::default(),
                ..Default::default()
            })
            .with_caption_settings(CaptionSettings {
                enabled: true,
                font_size: 28,
                ..Default::default()
            })
            .with_screen_reader(ScreenReaderSettings {
                enabled: true,
                verbosity: Verbosity::High,
                ..Default::default()
            })
            .build()
    }

    pub fn motor_impairment_profile() -> XRAccessibilityProfile {
        ProfileBuilder::new("fixture-motor")
            .with_motor_disability(MotorDisability {
                mobility: MobilityLevel::Wheelchair,
                upper_limb: UpperLimbFunction {
                    dominant_hand: DominantHand::Right,
                    grip_strength: GripStrength::Reduced,
                    ..Default::default()
                },
                ..Default::default()
            })
            .with_input_preferences(InputPreferences {
                primary_input: InputMethod::VoiceControl,
                fallback_inputs: vec![InputMethod::EyeTracking],
                ..Default::default()
            })
            .build()
    }

    pub fn quest_device() -> XRDeviceCapabilities {
        XRDeviceCapabilities {
            device_id: "quest-3-fixture".into(),
            device_name: "Meta Quest 3".into(),
            display: DisplayCapabilities {
                resolution_per_eye: Resolution { width: 2064, height: 2208 },
                refresh_rates: vec![72.0, 90.0, 120.0],
                ..Default::default()
            },
            ..Default::default()
        }
    }
}
```

---

## 10. References

- Rust Testing Book
- Jest Documentation
- WCAG-EM (Website Accessibility Conformance Evaluation Methodology)
- XR Accessibility Testing Guidelines (W3C Draft)
- WIA Test Framework v1.0
