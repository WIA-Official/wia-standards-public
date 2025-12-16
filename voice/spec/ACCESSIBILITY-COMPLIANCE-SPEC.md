# WIA Voice-Sign: Accessibility Compliance Specification

## 1. Overview

본 문서는 음성-수화 변환 시스템의 접근성 규정 준수 표준을 정의합니다.

**Version:** 1.0.0
**Status:** Draft

---

## 2. WCAG 2.1 Compliance

### 2.1 Level AAA Requirements

WIA Voice-Sign은 WCAG 2.1 Level AAA를 목표로 합니다.

#### 1.2.6 Sign Language (Prerecorded)

> 사전 녹화된 동기화 미디어의 모든 오디오 콘텐츠에 수화 통역이 제공되어야 합니다.

**구현 요구사항:**

| Requirement | Specification | Priority |
|-------------|---------------|----------|
| 수화 제공 | 모든 오디오에 자동 수화 | MUST |
| 동기화 | 오디오-수화 시간 동기화 | MUST |
| 화질 | 최소 320×240, 권장 720×480 | SHOULD |
| 프레임레이트 | 최소 24fps, 권장 30fps | SHOULD |
| 배경 대비 | 수화자 명확히 구분 | MUST |

### 2.2 Additional WCAG Criteria

#### 1.4.8 Visual Presentation (AAA)

```yaml
text_presentation:
  foreground_background_selectable: true
  max_width_characters: 80
  text_not_justified: true
  line_spacing: 1.5em minimum
  paragraph_spacing: 2.0em minimum
  text_resizable_200: true
```

#### 2.2.4 Interruptions (AAA)

```yaml
interruptions:
  user_can_postpone: true
  user_can_suppress: true
  emergency_exception: true
```

#### 2.3.2 Three Flashes (AAA)

```yaml
flash_prevention:
  max_flash_rate: 3Hz
  red_flash_threshold: 0.0
  general_flash_area_max: 0.006 # steradians
```

---

## 3. Avatar Display Requirements

### 3.1 Minimum Specifications

```typescript
interface AvatarDisplay {
  resolution: {
    minimum: { width: 320, height: 240 };
    recommended: { width: 720, height: 480 };
    optimal: { width: 1280, height: 720 };
  };

  aspectRatio: {
    preferred: '4:3' | '16:9';
    allowCrop: false;
  };

  visibility: {
    minContrastRatio: 4.5;     // 배경 대비
    signSpaceVisible: true;    // 서명 공간 전체 표시
    faceVisible: true;         // 얼굴 표정 가시
    handsVisible: true;        // 양손 항상 가시
  };

  positioning: {
    preferredLocation: 'bottom-right' | 'picture-in-picture';
    resizable: true;
    movable: true;
    canMinimize: true;
    canMaximize: true;
  };
}
```

### 3.2 Signing Space Coverage

```
┌─────────────────────────────────────────┐
│                                         │
│           ┌───────────────┐             │
│           │   HEAD SPACE  │             │
│           │  (emotions)   │             │
│           └───────────────┘             │
│     ┌─────────────────────────────┐     │
│     │      NEUTRAL SPACE          │     │
│     │    (main signing area)      │     │
│     │                             │     │
│     └─────────────────────────────┘     │
│  ┌─────┐                     ┌─────┐    │
│  │ L   │                     │   R │    │
│  │HAND │                     │HAND │    │
│  └─────┘                     └─────┘    │
│                                         │
└─────────────────────────────────────────┘

Coverage Requirements:
- Full neutral space visible
- Both hands visible at maximum extension
- Face clearly visible for non-manual markers
- No cropping of fingerspelling
```

### 3.3 Background Requirements

| Element | Requirement |
|---------|-------------|
| Color | 단색, 무채색 권장 |
| Pattern | 무늬 없음 |
| Movement | 정적 배경 |
| Contrast | 수화자 피부톤과 대비 |
| Lighting | 균일한 조명, 그림자 최소화 |

---

## 4. User Controls

### 4.1 Playback Controls

```typescript
interface PlaybackControls {
  // 필수 컨트롤
  required: {
    play: boolean;
    pause: boolean;
    stop: boolean;
    restart: boolean;
  };

  // 속도 조절
  speed: {
    available: [0.5, 0.75, 1.0, 1.25, 1.5];
    default: 1.0;
    userSelectable: true;
    rememberPreference: true;
  };

  // 탐색
  navigation: {
    seekBar: boolean;
    skipForward: number;   // seconds
    skipBackward: number;  // seconds
    sentenceNavigation: boolean;
    glossNavigation: boolean;
  };

  // 반복
  repeat: {
    loopEnabled: boolean;
    loopSegment: boolean;  // 선택 구간 반복
    autoRepeatLowConfidence: boolean;
  };
}
```

### 4.2 Display Customization

```typescript
interface DisplayCustomization {
  avatar: {
    style: 'realistic' | 'stylized' | 'cartoon';
    skinTone: string[];
    clothing: string[];
    background: string[];
  };

  size: {
    scalable: true;
    minScale: 0.5;
    maxScale: 2.0;
    pinchZoom: boolean;
  };

  position: {
    draggable: boolean;
    snapToCorners: boolean;
    rememberPosition: boolean;
  };

  appearance: {
    highContrast: boolean;
    darkMode: boolean;
    customColors: boolean;
  };
}
```

### 4.3 Keyboard Accessibility

| Key | Action |
|-----|--------|
| Space | Play/Pause |
| Enter | Play/Pause |
| ← | Skip back 5s |
| → | Skip forward 5s |
| ↑ | Volume up |
| ↓ | Volume down |
| M | Mute/Unmute |
| F | Fullscreen |
| Esc | Exit fullscreen |
| 0-9 | Jump to 0-90% |
| < | Slower |
| > | Faster |
| R | Repeat current sign |
| Tab | Next control |

---

## 5. Regional Compliance

### 5.1 EU - EN 301 549

**의무 요구사항:**
- 공공 부문 웹사이트 및 모바일 앱
- 2025년부터 민간 부문 확대

```yaml
en301549_compliance:
  chapter_9:  # Web
    accessible_name: required
    keyboard_accessible: required
    sign_language: recommended
  chapter_11:  # Software
    platform_accessibility: required
    assistive_technology_support: required
```

### 5.2 US - Section 508

**적용 범위:**
- 연방 정부 기관
- 연방 자금 지원 프로그램

```yaml
section508_compliance:
  1194.22:  # Web accessibility
    equivalent_alternatives: required
    synchronized_media: required
  video_content:
    captions: required
    audio_description: required
    sign_language: encouraged
```

### 5.3 Korea - 웹접근성 지침 2.2

**한국정보통신표준(KICS):**

```yaml
korean_accessibility:
  인식의_용이성:
    대체_텍스트: required
    멀티미디어_대체_수단: required  # 수화 포함
    명료성: required
  운용의_용이성:
    키보드_접근성: required
    충분한_시간: required
    광과민성_발작_예방: required
  이해의_용이성:
    가독성: required
    예측_가능성: required
    콘텐츠의_논리성: required
  견고성:
    문법_준수: required
    웹_애플리케이션_접근성: required
```

---

## 6. Deaf User Experience (DUX)

### 6.1 Design Principles

1. **Visual-First**: 시각 정보 우선
2. **Sign Language Native**: 수화가 모국어임을 인식
3. **Cultural Sensitivity**: 농문화 존중
4. **User Empowerment**: 사용자 제어권

### 6.2 Visual Feedback

```typescript
interface VisualFeedback {
  // 상태 표시
  status: {
    listening: 'pulsing_icon';      // 듣는 중
    processing: 'spinning_icon';    // 처리 중
    ready: 'static_icon';           // 준비 완료
    error: 'warning_icon';          // 오류
  };

  // 진행 표시
  progress: {
    transcriptionProgress: boolean;
    translationProgress: boolean;
    renderingProgress: boolean;
    estimatedTime: boolean;
  };

  // 알림
  notifications: {
    visual: true;           // 시각적 알림
    vibration: true;        // 진동 (모바일)
    screenFlash: false;     // 화면 깜빡임 (주의: 광과민성)
    colorChange: true;      // 색상 변화
  };
}
```

### 6.3 Caption Synchronization

```typescript
interface CaptionSync {
  // 자막-수화 동기화
  synchronization: {
    maxLag: 100;            // ms
    highlightCurrentWord: true;
    showGlossSubtitles: boolean;
  };

  // 자막 옵션
  options: {
    fontSize: 'small' | 'medium' | 'large' | 'extra-large';
    fontFamily: string[];
    backgroundColor: string;
    textColor: string;
    position: 'top' | 'bottom';
  };

  // 표시 모드
  displayMode: {
    captionsOnly: boolean;
    signOnly: boolean;
    both: boolean;
    userSelectable: true;
  };
}
```

### 6.4 Sign Language Preferences

```typescript
interface SignLanguagePreferences {
  // 선호 수화
  preferred: {
    primary: SignLanguageCode;
    secondary?: SignLanguageCode;
    dialect?: string;
  };

  // 아바타 선호
  avatarPreference: {
    style: AvatarStyle;
    speed: number;
    expressiveness: 'subtle' | 'normal' | 'expressive';
  };

  // 표시 선호
  displayPreference: {
    glossSubtitles: boolean;
    fingerspellingSpeed: 'slow' | 'normal' | 'fast';
    nonManualMarkers: 'emphasized' | 'normal' | 'subtle';
  };

  // 저장 및 동기화
  storage: {
    savePreferences: boolean;
    syncAcrossDevices: boolean;
    shareWithCommunity: boolean;
  };
}
```

---

## 7. Testing Requirements

### 7.1 Automated Testing

| Tool | Purpose |
|------|---------|
| axe-core | WCAG 자동 검사 |
| WAVE | 웹 접근성 평가 |
| Lighthouse | 접근성 점수 |
| Pa11y | CI/CD 통합 테스트 |

### 7.2 Manual Testing

| Test Type | Frequency | Participants |
|-----------|-----------|--------------|
| Screen reader test | 매 릴리스 | QA 팀 |
| Keyboard-only test | 매 릴리스 | QA 팀 |
| Deaf user test | 분기별 | 농인 사용자 그룹 |
| Expert review | 연간 | 접근성 전문가 |

### 7.3 Conformance Statement

```yaml
conformance_statement:
  standard: "WCAG 2.1 Level AAA"
  scope: "Voice-Sign Translation Interface"
  date: "2025-01-15"
  methods:
    - automated_testing
    - manual_testing
    - user_testing
  contact: "accessibility@wia.org"
  feedback_mechanism: true
```

---

## 8. References

- WCAG 2.1 Guidelines (W3C)
- EN 301 549 v3.2.1 (EU)
- Section 508 Standards (US)
- KICS 웹접근성 지침 2.2 (Korea)
- ISO 9241-171:2008 Ergonomics of human-system interaction
- Deaf Studies Research Guidelines
