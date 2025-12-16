# WIA Smart Wheelchair - Phase 4: Ecosystem Integration Specification

## 1. Overview

이 문서는 WIA 스마트 휠체어의 보조기기 통합 명세를 정의합니다.
Eye Gaze, BCI, 음성 명령, 외골격, 스마트홈 시스템과의 연동을 포함합니다.

### 1.1 Integration Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Interface Manager                             │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐   │
│  │Eye Gaze │ │   BCI   │ │  Voice  │ │   Exo   │ │SmartHome│   │
│  └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘   │
│       │           │           │           │           │         │
│       └───────────┴─────┬─────┴───────────┴───────────┘         │
│                         │                                        │
│                  ┌──────▼──────┐                                │
│                  │  Priority   │                                │
│                  │  Arbiter    │                                │
│                  └──────┬──────┘                                │
│                         │                                        │
└─────────────────────────┼────────────────────────────────────────┘
                          │
                   ┌──────▼──────┐
                   │  cmd_vel    │
                   │  Publisher  │
                   └─────────────┘
```

---

## 2. Eye Gaze Integration

### 2.1 Control Modes

| Mode | Description | Use Case |
|------|-------------|----------|
| DIRECTION | 시선 방향으로 직접 이동 | 실시간 제어 |
| GOAL_SELECT | 화면에서 목적지 선택 | 자율주행 목적지 |
| SWITCH_COMBO | 시선 선택 + 스위치 확인 | 정밀 제어 |

### 2.2 Gaze-to-Velocity Mapping

```python
def gaze_to_velocity(gaze_x: float, gaze_y: float) -> Twist:
    """
    Convert normalized gaze position (0-1) to velocity command.

    Args:
        gaze_x: Horizontal position (0=left, 0.5=center, 1=right)
        gaze_y: Vertical position (0=top, 0.5=center, 1=bottom)

    Returns:
        Twist message with linear.x and angular.z
    """
    # Apply deadzone
    offset_x = gaze_x - 0.5
    offset_y = gaze_y - 0.5

    distance = sqrt(offset_x**2 + offset_y**2)
    if distance < DEADZONE_RADIUS:
        return Twist()  # Zero velocity

    # Map to velocity
    linear = -offset_y * MAX_LINEAR_SPEED * 2   # Up = forward
    angular = -offset_x * MAX_ANGULAR_SPEED * 2  # Left = turn left

    return Twist(linear=linear, angular=angular)
```

### 2.3 Dwell Selection

```yaml
dwell_config:
  enabled: true
  dwell_time: 1.0        # seconds to confirm selection
  dwell_radius: 0.1      # normalized screen radius
  visual_feedback: true  # show progress indicator
```

### 2.4 ROS2 Interface

```
Topics:
  /wia_wheelchair/gaze_point      [geometry_msgs/Point]    # Input
  /wia_wheelchair/gaze_cmd_vel    [geometry_msgs/Twist]    # Output
  /wia_wheelchair/gaze_status     [std_msgs/String]        # Status

Services:
  /wia_wheelchair/gaze/enable     [std_srvs/SetBool]
  /wia_wheelchair/gaze/set_mode   [std_srvs/SetBool]
```

---

## 3. BCI (Brain-Computer Interface) Integration

### 3.1 Intent Classification

| Intent | Pattern | Threshold | Description |
|--------|---------|-----------|-------------|
| FORWARD | Frontal alpha+beta | 0.7 | 전진 의도 |
| STOP | Bilateral theta | 0.6 | 정지 의도 |
| LEFT | Right motor mu | 0.7 | 좌회전 의도 |
| RIGHT | Left motor mu | 0.7 | 우회전 의도 |
| SELECT | Parietal P300 | 0.8 | 선택 의도 |

### 3.2 Control Modes

```typescript
enum BCIControlMode {
  DIRECT = 1,      // Direct BCI to velocity
  DISCRETE = 2,    // Discrete commands (forward/stop/turn)
  HYBRID = 3,      // BCI intent + autonomous execution
  SELECTION = 4    // BCI for menu selection
}
```

### 3.3 Safety Configuration

```yaml
bci_safety:
  max_speed: 0.3                  # m/s (reduced for safety)
  require_confirmation: true
  confirmation_duration: 1.0      # seconds
  auto_stop_timeout: 2.0          # seconds without signal
  obstacle_override: true         # safety override
```

### 3.4 Intent Confirmation Flow

```
Signal Detected → Accumulate → Threshold Check → Confirm Duration → Execute
      │                │              │                  │              │
      └── < 0.7 ──────►│              │                  │              │
                       └── Reset ────►│                  │              │
                                      └── < 1.0s ───────►│              │
                                                         └── Execute ──►│
```

### 3.5 ROS2 Interface

```
Topics:
  /wia_wheelchair/bci_signal      [std_msgs/Float32MultiArray]  # Raw EEG
  /wia_wheelchair/bci_intent      [std_msgs/String]              # Classified
  /wia_wheelchair/bci_cmd_vel     [geometry_msgs/Twist]          # Output
  /wia_wheelchair/bci_status      [std_msgs/String]              # Status

Services:
  /wia_wheelchair/bci/enable      [std_srvs/SetBool]
  /wia_wheelchair/bci/calibrate   [std_srvs/Trigger]
```

---

## 4. Voice Command Integration

### 4.1 Supported Languages

| Language | Code | Wake Words |
|----------|------|------------|
| Korean | ko-KR | "휠체어", "이동" |
| English | en-US | "wheelchair", "move" |

### 4.2 Command Categories

#### 4.2.1 Motion Commands (Korean)

| Command | Action | Velocity |
|---------|--------|----------|
| "앞으로" / "전진" | Forward | linear: 0.5 |
| "뒤로" / "후진" | Backward | linear: -0.3 |
| "왼쪽" / "좌회전" | Turn left | angular: 0.5 |
| "오른쪽" / "우회전" | Turn right | angular: -0.5 |
| "멈춰" / "정지" | Stop | linear: 0, angular: 0 |

#### 4.2.2 Navigation Commands

| Command | Action |
|---------|--------|
| "거실로 가" | Navigate to 거실 |
| "주방으로 이동" | Navigate to 주방 |
| "화장실 가기" | Navigate to 화장실 |
| "충전하러 가" | Navigate to 충전소 |
| "집으로" | Navigate to home |

#### 4.2.3 Control Commands

| Command | Action |
|---------|--------|
| "비상 정지" | Emergency stop |
| "취소" | Cancel current action |
| "천천히" | Reduce speed (0.5x) |
| "빨리" | Increase speed (1.5x) |

### 4.3 AAC Integration

```python
# AAC Symbol to Command Mapping
aac_symbols = {
    'symbol_forward': '앞으로',
    'symbol_stop': '멈춰',
    'symbol_left': '왼쪽',
    'symbol_right': '오른쪽',
    'symbol_home': '집으로',
    'symbol_bathroom': '화장실',
}
```

### 4.4 ROS2 Interface

```
Topics:
  /wia_wheelchair/speech_text     [std_msgs/String]      # Recognized text
  /wia_wheelchair/aac_symbol      [std_msgs/String]      # AAC symbol ID
  /wia_wheelchair/voice_cmd_vel   [geometry_msgs/Twist]  # Output
  /wia_wheelchair/voice_feedback  [std_msgs/String]      # TTS feedback
  /wia_wheelchair/voice_status    [std_msgs/String]      # Status

Services:
  /wia_wheelchair/voice/enable    [std_srvs/SetBool]
```

---

## 5. Exoskeleton Transition Integration

### 5.1 Transition States

```
IDLE → ALIGNING → PREPARING → SAFETY_CHECK → TRANSFERRING → COMPLETING
  │                                                              │
  └─────────────────── COORDINATED_MOVE ◄────────────────────────┘
```

### 5.2 Transition to Exoskeleton Sequence

| Step | Action | Timeout | Description |
|------|--------|---------|-------------|
| 1 | align_with_exo | 30s | 외골격 위치로 정렬 |
| 2 | open_armrests | 5s | 팔걸이 열기 |
| 3 | safety_check | 10s | 안전 점검 |
| 4 | wait_confirmation | 60s | 사용자 확인 대기 |
| 5 | signal_exo | 5s | 외골격 준비 신호 |
| 6 | monitor_transfer | 120s | 환승 모니터링 |

### 5.3 Transition from Exoskeleton Sequence

| Step | Action | Timeout | Description |
|------|--------|---------|-------------|
| 1 | confirm_position | 10s | 휠체어 위치 확인 |
| 2 | prepare_seat | 5s | 좌석 준비 |
| 3 | safety_check | 10s | 안전 점검 |
| 4 | wait_confirmation | 60s | 사용자 확인 대기 |
| 5 | assist_seating | 60s | 착석 보조 |
| 6 | close_armrests | 5s | 팔걸이 닫기 |
| 7 | activate | 5s | 휠체어 활성화 |

### 5.4 Safety Requirements

```yaml
transition_safety:
  wheelchair_stable: true       # 휠체어 안정성
  exo_ready: true              # 외골격 준비 상태
  path_clear: true             # 경로 장애물 없음
  position_aligned: true       # 위치 정렬 완료
  user_confirmed: true         # 사용자 확인 (optional)
```

### 5.5 Coordinated Mobility (Follow Mode)

```yaml
follow_mode:
  enabled: true
  follow_distance: 1.5         # meters
  follow_timeout: 5.0          # seconds to wait if lost
  max_speed: 0.3               # m/s
```

### 5.6 ROS2 Interface

```
Topics:
  /wia_wheelchair/pose              [geometry_msgs/PoseStamped]  # Wheelchair
  /wia_exoskeleton/state            [std_msgs/String]            # Exo state
  /wia_exoskeleton/pose             [geometry_msgs/PoseStamped]  # Exo pose
  /wia_wheelchair/transition_status [std_msgs/String]            # Status
  /wia_exoskeleton/command          [std_msgs/String]            # Commands

Services:
  /wia_wheelchair/transition/to_exo     [std_srvs/Trigger]
  /wia_wheelchair/transition/from_exo   [std_srvs/Trigger]
  /wia_wheelchair/transition/cancel     [std_srvs/Trigger]
  /wia_wheelchair/transition/follow_mode [std_srvs/SetBool]
```

---

## 6. Smart Home Integration

### 6.1 Supported Protocols

| Protocol | Standard | Description |
|----------|----------|-------------|
| Matter | 1.0+ | Primary smart home protocol |
| Thread | 1.3+ | Low-power mesh networking |

### 6.2 Supported Device Types

| Type | Actions | Auto-trigger |
|------|---------|--------------|
| LIGHT | on, off, brightness, color_temp | Path lighting |
| DOOR | open, close | Auto-open on approach |
| LOCK | lock, unlock | - |
| ELEVATOR | call, send | Auto-call |
| BLIND | open, close, set_position | - |

### 6.3 Accessibility Automations

```yaml
accessibility_automation:
  auto_open_doors: true
  door_open_lead_time: 3.0      # seconds before arrival

  auto_call_elevator: true
  elevator_call_distance: 5.0   # meters

  auto_turn_on_lights: true
  light_on_lead_time: 2.0       # seconds

  path_lighting: true           # Light up navigation path
```

### 6.4 Location-Based Triggers

```yaml
triggers:
  - name: bathroom_door
    location: 화장실
    radius: 2.0                 # meters
    actions:
      - device: door_bathroom
        action: open
    cooldown: 30.0              # seconds between triggers

  - name: hallway_light
    location: 복도
    radius: 1.5
    actions:
      - device: light_hallway
        action: on
        params:
          brightness: 100
```

### 6.5 Voice Assistant Integration

```yaml
voice_assistants:
  alexa:
    enabled: false
    skill_id: "amzn1.ask.skill.xxx"

  google_home:
    enabled: false
    project_id: "wia-wheelchair"

  siri:
    enabled: false
    shortcut_name: "휠체어 제어"
```

### 6.6 ROS2 Interface

```
Topics:
  /wia_wheelchair/navigation_goal   [std_msgs/String]   # Nav goal
  /wia_wheelchair/smarthome_status  [std_msgs/String]   # Status
  /wia_wheelchair/door_status       [std_msgs/String]   # Door states

Services:
  /wia_wheelchair/smarthome/discover      [std_srvs/Trigger]
  /wia_wheelchair/smarthome/auto_door     [std_srvs/SetBool]
  /wia_wheelchair/smarthome/call_elevator [std_srvs/Trigger]
```

---

## 7. Interface Manager

### 7.1 Input Priority Levels

| Priority | Level | Description |
|----------|-------|-------------|
| EMERGENCY | 0 | Emergency stop (always override) |
| SAFETY | 1 | Safety systems |
| USER_DIRECT | 2 | Joystick, Gaze, BCI |
| VOICE | 3 | Voice commands |
| AUTONOMOUS | 4 | Autonomous navigation |
| SMARTHOME | 5 | Smart home triggered |

### 7.2 Mode Switching

```yaml
interface_manager:
  input_timeout: 2.0            # seconds to switch to lower priority
  default_mode: MANUAL          # joystick
  auto_switch_on_input: true    # auto-switch to active input
  emergency_stop_all: true      # e-stop overrides everything
```

### 7.3 Arbitration Logic

```python
def select_velocity(inputs: Dict[InputMode, Twist]) -> Twist:
    """
    Select velocity based on priority.

    Priority order:
    1. Emergency stop (if active)
    2. Highest priority active input
    3. Zero velocity (if no active input)
    """
    if emergency_stop:
        return Twist()  # Zero

    # Get active inputs sorted by priority
    active = [
        (mode, twist)
        for mode, twist in inputs.items()
        if is_active(mode) and twist is not None
    ]

    if not active:
        return Twist()  # Zero

    # Return highest priority (lowest number)
    best = min(active, key=lambda x: get_priority(x[0]))
    return best[1]
```

### 7.4 ROS2 Interface

```
Subscribers:
  /wia_wheelchair/joystick_cmd_vel  [geometry_msgs/Twist]
  /wia_wheelchair/gaze_cmd_vel      [geometry_msgs/Twist]
  /wia_wheelchair/bci_cmd_vel       [geometry_msgs/Twist]
  /wia_wheelchair/voice_cmd_vel     [geometry_msgs/Twist]
  /wia_wheelchair/nav_cmd_vel       [geometry_msgs/Twist]
  /wia_wheelchair/emergency_stop    [std_msgs/Bool]

Publishers:
  /wia_wheelchair/cmd_vel           [geometry_msgs/Twist]  # Final output
  /wia_wheelchair/active_mode       [std_msgs/Int32]
  /wia_wheelchair/interface_status  [std_msgs/String]

Services:
  /wia_wheelchair/interface/set_mode      [std_srvs/Trigger]
  /wia_wheelchair/interface/enable_gaze   [std_srvs/SetBool]
  /wia_wheelchair/interface/enable_bci    [std_srvs/SetBool]
  /wia_wheelchair/interface/enable_voice  [std_srvs/SetBool]
```

---

## 8. Configuration Files

### 8.1 interfaces_params.yaml

```yaml
interface_manager:
  ros__parameters:
    input_timeout: 2.0
    default_mode: 0              # MANUAL
    auto_switch: true
    control_rate: 20.0

gaze_controller:
  ros__parameters:
    sensitivity: 1.0
    deadzone: 0.15
    max_linear_speed: 0.5
    max_angular_speed: 1.0
    dwell_time: 1.0
    smoothing: 0.3

bci_controller:
  ros__parameters:
    max_speed: 0.3
    require_confirmation: true
    confirmation_duration: 1.0
    confidence_threshold: 0.7
    autonomous_assist: true

voice_controller:
  ros__parameters:
    language: "ko-KR"
    confidence_threshold: 0.6
    voice_feedback: true
    require_wake_word: false
    default_speed: 0.5

transition_manager:
  ros__parameters:
    alignment_tolerance: 0.05
    require_confirmation: true
    follow_distance: 1.5

matter_bridge:
  ros__parameters:
    auto_open_doors: true
    auto_call_elevator: true
    auto_lights: true
    path_lighting: true
```

---

## 9. Launch Configuration

### 9.1 interfaces.launch.py

```python
# Launch all interface nodes
ros2 launch wia_wheelchair_interfaces interfaces.launch.py \
    enable_gaze:=true \
    enable_bci:=false \
    enable_voice:=true \
    enable_smarthome:=true
```

### 9.2 Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| use_sim_time | false | Use simulation time |
| enable_gaze | false | Enable eye gaze control |
| enable_bci | false | Enable BCI control |
| enable_voice | true | Enable voice control |
| enable_smarthome | true | Enable smart home |

---

## 10. Error Handling

### 10.1 Input Loss Handling

| Input | Timeout | Action |
|-------|---------|--------|
| Gaze | 0.5s | Stop wheelchair |
| BCI | 2.0s | Stop wheelchair |
| Voice | N/A | No action |

### 10.2 Device Disconnection

```yaml
error_handling:
  on_gaze_disconnect: stop_and_notify
  on_bci_disconnect: stop_and_notify
  on_exo_disconnect: abort_transition
  on_smarthome_disconnect: continue_manual
```

### 10.3 Recovery Procedures

1. **Gaze Lost**: Auto-stop, wait for gaze recovery
2. **BCI Signal Lost**: Auto-stop after timeout, switch to manual
3. **Transition Failed**: Abort, return to safe state
4. **Smart Home Offline**: Continue manual operation

---

## 11. Security Considerations

### 11.1 Authentication

- BCI signal validation against calibrated profile
- Voice command speaker verification (optional)
- Smart home device authentication via Matter

### 11.2 Safety Overrides

```yaml
safety_overrides:
  emergency_stop_priority: 0     # Always highest
  obstacle_detection: true       # Override all commands
  cliff_detection: true          # Override all commands
  max_override_speed: 0.1        # m/s during safety events
```

---

## 12. Compliance

이 명세는 다음 표준을 준수합니다:

- **ISO 7176**: Wheelchairs
- **IEC 62443**: Industrial cybersecurity
- **Matter 1.0**: Smart home connectivity
- **ROS2 Humble**: Robot Operating System

---

## 홍익인간

이동의 자유를. 모든 사람에게. 어떤 방식으로든.

---

*WIA Smart Wheelchair Phase 4 Integration Specification v1.0*
*Last Updated: 2025-12-16*
