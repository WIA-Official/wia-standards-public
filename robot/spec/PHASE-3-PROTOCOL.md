# WIA Robot Protocol (WRP) Specification

**Version 1.0.0**

---

## 1. 개요 (Overview)

WIA Robot Protocol (WRP)은 보조 로봇 시스템 간 안전하고 실시간 통신을 위한 표준 프로토콜입니다.

### 목적

- **상호운용성**: 다양한 로봇 유형 간 표준화된 통신
- **안전성**: Safety-first 설계, 긴급 정지 최우선 처리
- **실시간성**: 저지연 제어 명령 전송
- **확장성**: 다양한 전송 계층 지원

### 적용 범위

| 로봇 유형 | 적용 |
|----------|------|
| Exoskeleton (외골격) | ✅ |
| Prosthetics (의수/의족) | ✅ |
| Rehabilitation (재활 로봇) | ✅ |
| Care Robot (돌봄 로봇) | ✅ |
| Surgical (수술 로봇) | ✅ |
| Mobility Aid (이동 보조) | ✅ |

---

## 2. 용어 정의 (Terminology)

| 용어 | 정의 |
|------|------|
| **WRP** | WIA Robot Protocol - 본 프로토콜의 명칭 |
| **Endpoint** | 메시지를 송수신하는 디바이스 또는 시스템 |
| **Message** | WRP 형식에 따른 데이터 단위 |
| **Payload** | 메시지 내 실제 데이터 (Phase 1 형식) |
| **E-Stop** | Emergency Stop, 긴급 정지 |
| **QoS** | Quality of Service, 서비스 품질 |
| **Transport** | 메시지 전송 계층 (WebSocket, MQTT, DDS 등) |
| **Heartbeat** | 연결 상태 확인을 위한 주기적 메시지 |

---

## 3. 메시지 형식 (Message Format)

### 3.1 기본 구조

```json
{
  "protocol": "wia-robot",
  "version": "1.0.0",
  "message_id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-01-15T10:30:00.123Z",
  "sequence": 12345,
  "type": "telemetry",
  "priority": "normal",
  "source": {
    "device_id": "exo-rewalk-001",
    "device_type": "exoskeleton",
    "location": {
      "latitude": 37.5665,
      "longitude": 126.9780,
      "altitude_m": 50.0
    }
  },
  "destination": {
    "device_id": "server-main-001",
    "device_type": "server",
    "location": null
  },
  "safety": {
    "emergency_stop": false,
    "safety_level": "normal",
    "requires_ack": false,
    "ack_timeout_ms": 1000
  },
  "payload": {
    "// Phase 1 Data Format"
  },
  "checksum": "a1b2c3d4"
}
```

### 3.2 필드 설명

| 필드 | 타입 | 필수 | 설명 |
|------|------|------|------|
| `protocol` | string | ✅ | 프로토콜 식별자 ("wia-robot") |
| `version` | string | ✅ | 프로토콜 버전 (SemVer) |
| `message_id` | string | ✅ | UUID v4 메시지 고유 ID |
| `timestamp` | string | ✅ | ISO 8601 UTC 타임스탬프 |
| `sequence` | integer | ✅ | 메시지 시퀀스 번호 |
| `type` | string | ✅ | 메시지 유형 |
| `priority` | string | ✅ | 우선순위 |
| `source` | Endpoint | ✅ | 송신 엔드포인트 |
| `destination` | Endpoint | ✅ | 수신 엔드포인트 |
| `safety` | SafetyInfo | ✅ | 안전 정보 |
| `payload` | object | ✅ | 메시지 데이터 |
| `checksum` | string | | CRC32 체크섬 (hex) |

### 3.3 Endpoint 구조

```json
{
  "device_id": "string (required)",
  "device_type": "string (required)",
  "location": {
    "latitude": "number (optional)",
    "longitude": "number (optional)",
    "altitude_m": "number (optional)"
  }
}
```

### 3.4 SafetyInfo 구조

```json
{
  "emergency_stop": "boolean (required)",
  "safety_level": "normal|warning|caution|critical|emergency",
  "requires_ack": "boolean",
  "ack_timeout_ms": "integer"
}
```

---

## 4. 메시지 유형 (Message Types)

### 4.1 메시지 유형 목록

| Type | 방향 | 설명 | 기본 우선순위 |
|------|------|------|--------------|
| `handshake` | Both | 연결 설정 | high |
| `handshake_ack` | Both | 연결 응답 | high |
| `heartbeat` | Both | 연결 유지 | normal |
| `telemetry` | Device → Server | 센서/상태 데이터 | normal |
| `control` | Server → Device | 제어 명령 | high |
| `control_ack` | Device → Server | 제어 응답 | high |
| `emergency_stop` | Both | 긴급 정지 | emergency |
| `emergency_stop_ack` | Both | 긴급 정지 응답 | emergency |
| `safety_alert` | Both | 안전 경보 | critical |
| `status` | Device → Server | 상태 업데이트 | normal |
| `config` | Server → Device | 설정 변경 | high |
| `config_ack` | Device → Server | 설정 응답 | high |
| `error` | Both | 에러 메시지 | high |
| `log` | Device → Server | 로그 데이터 | low |

### 4.2 Handshake 메시지

```json
{
  "type": "handshake",
  "payload": {
    "protocol_version": "1.0.0",
    "device_info": {
      "id": "exo-001",
      "type": "exoskeleton",
      "name": "ReWalk Personal 6.0",
      "manufacturer": "ReWalk Robotics",
      "firmware_version": "6.2.1",
      "capabilities": ["lower_body", "gait_assist"]
    },
    "supported_message_types": ["telemetry", "control", "safety_alert"],
    "heartbeat_interval_ms": 1000,
    "requested_qos": {
      "reliability": "reliable",
      "durability": "volatile"
    }
  }
}
```

### 4.3 Telemetry 메시지

```json
{
  "type": "telemetry",
  "payload": {
    "// Phase 1 ExoskeletonSpec data"
    "joints": [...],
    "gait": {...},
    "imu": {...}
  }
}
```

### 4.4 Control 메시지

```json
{
  "type": "control",
  "priority": "high",
  "safety": {
    "requires_ack": true,
    "ack_timeout_ms": 500
  },
  "payload": {
    "command": "set_assist_level",
    "parameters": {
      "assist_level": 0.75,
      "transition_time_ms": 1000
    },
    "command_id": "cmd-001"
  }
}
```

### 4.5 Emergency Stop 메시지

```json
{
  "type": "emergency_stop",
  "priority": "emergency",
  "destination": {
    "device_id": "broadcast",
    "device_type": "all"
  },
  "safety": {
    "emergency_stop": true,
    "safety_level": "emergency",
    "requires_ack": true,
    "ack_timeout_ms": 100
  },
  "payload": {
    "reason": "fall_detected",
    "source": "imu_sensor",
    "timestamp": "2025-01-15T10:30:00.123Z",
    "affected_devices": ["exo-001", "control-001"]
  }
}
```

---

## 5. 연결 관리 (Connection Management)

### 5.1 연결 상태 머신

```
┌──────────────┐
│ DISCONNECTED │
└──────┬───────┘
       │ connect()
       ▼
┌──────────────┐
│  CONNECTING  │
└──────┬───────┘
       │ handshake_ack received
       ▼
┌──────────────┐      heartbeat timeout
│    ACTIVE    │─────────────────────────┐
└──────┬───────┘                         │
       │ error/timeout                   │
       ▼                                 ▼
┌──────────────┐                 ┌──────────────┐
│ RECONNECTING │◄────────────────│   INACTIVE   │
└──────┬───────┘ reconnect       └──────────────┘
       │ max_retries exceeded
       │ or emergency_stop
       ▼
┌──────────────┐
│   STOPPED    │
└──────────────┘
```

### 5.2 연결 상태

| 상태 | 설명 |
|------|------|
| DISCONNECTED | 연결 없음 |
| CONNECTING | 연결 시도 중 |
| ACTIVE | 정상 연결 |
| INACTIVE | 하트비트 실패 |
| RECONNECTING | 재연결 시도 중 |
| STOPPED | 연결 종료 (E-Stop 포함) |

### 5.3 Heartbeat 프로토콜

```
Device                          Server
   │                               │
   │──── heartbeat ───────────────►│
   │                               │
   │◄─── heartbeat ────────────────│
   │                               │
   ├─── (interval: 1000ms) ────────┤
   │                               │
   │──── heartbeat ───────────────►│
   │                               │
```

- **기본 간격**: 1000ms
- **타임아웃**: 3 * interval
- **타임아웃 시**: INACTIVE 상태 전환

---

## 6. 안전 프로토콜 (Safety Protocol)

### 6.1 Safety Level

| Level | 값 | 설명 | 동작 |
|-------|---|------|------|
| Normal | 0 | 정상 동작 | 일반 운용 |
| Warning | 1 | 경고 | 모니터링 강화 |
| Caution | 2 | 주의 | 동작 제한 |
| Critical | 3 | 위험 | 최소 동작만 허용 |
| Emergency | 4 | 긴급 | 즉시 정지 |

### 6.2 Emergency Stop Sequence

```
1. E-Stop 트리거 (버튼, 센서, 소프트웨어)
   │
   ▼
2. emergency_stop 메시지 브로드캐스트
   - destination: "broadcast"
   - priority: "emergency"
   - requires_ack: true
   │
   ▼
3. 모든 디바이스 즉시 안전 상태 전환
   - 모터 정지
   - 안전 위치로 이동 (가능한 경우)
   │
   ▼
4. emergency_stop_ack 응답
   - 100ms 이내 응답 필수
   │
   ▼
5. E-Stop 해제 대기
   - 수동 리셋 필요
   - 안전 점검 완료 후 재개
```

### 6.3 E-Stop 소스

| Source | 코드 | 설명 |
|--------|------|------|
| UserButton | 0x01 | 사용자 버튼 |
| Software | 0x02 | 소프트웨어 명령 |
| ImuSensor | 0x03 | IMU 센서 (낙상 감지) |
| ForceSensor | 0x04 | 힘 센서 초과 |
| ThermalSensor | 0x05 | 과열 감지 |
| BatteryLow | 0x06 | 배터리 위험 |
| Communication | 0x07 | 통신 실패 |
| External | 0x08 | 외부 시스템 |
| Watchdog | 0x09 | 와치독 타임아웃 |

### 6.4 Safety Watchdog

```
활성화 조건:
- Safety-critical 디바이스
- Control 메시지 수신 중

동작:
- 마지막 heartbeat 후 3초 내 heartbeat 미수신 시
- 자동으로 safety_level → caution
- 5초 내 미수신 시 → critical
- 10초 내 미수신 시 → emergency (자동 E-Stop)
```

---

## 7. QoS 설정 (Quality of Service)

### 7.1 QoS 매트릭스

| 메시지 유형 | Reliability | Durability | Max Latency |
|------------|-------------|------------|-------------|
| emergency_stop | Reliable | Transient | < 10ms |
| control | Reliable | Volatile | < 50ms |
| telemetry | Best Effort | Volatile | < 100ms |
| status | Reliable | Transient | < 200ms |
| config | Reliable | Persistent | < 1000ms |
| log | Best Effort | Persistent | N/A |
| heartbeat | Best Effort | Volatile | < 100ms |

### 7.2 Reliability

- **Reliable**: 전송 보장, 재전송
- **Best Effort**: 전송 보장 없음

### 7.3 Durability

- **Transient**: 메모리 전용
- **Volatile**: 연결 중에만 유효
- **Persistent**: 영구 저장

---

## 8. 에러 처리 (Error Handling)

### 8.1 에러 코드

| 코드 | 이름 | 설명 |
|------|------|------|
| 1000 | INVALID_MESSAGE | 메시지 형식 오류 |
| 1001 | CHECKSUM_FAILED | 체크섬 검증 실패 |
| 1002 | UNSUPPORTED_VERSION | 지원하지 않는 버전 |
| 1003 | UNKNOWN_MESSAGE_TYPE | 알 수 없는 메시지 유형 |
| 2000 | CONNECTION_FAILED | 연결 실패 |
| 2001 | HANDSHAKE_FAILED | 핸드셰이크 실패 |
| 2002 | HEARTBEAT_TIMEOUT | 하트비트 타임아웃 |
| 2003 | ACK_TIMEOUT | ACK 타임아웃 |
| 3000 | SAFETY_VIOLATION | 안전 위반 |
| 3001 | EMERGENCY_STOP | 긴급 정지 발생 |
| 3002 | SAFETY_LIMIT_EXCEEDED | 안전 한계 초과 |
| 4000 | DEVICE_ERROR | 디바이스 오류 |
| 4001 | DEVICE_OFFLINE | 디바이스 오프라인 |
| 4002 | DEVICE_BUSY | 디바이스 사용 중 |
| 5000 | COMMAND_REJECTED | 명령 거부 |
| 5001 | INVALID_PARAMETER | 잘못된 파라미터 |
| 5002 | COMMAND_TIMEOUT | 명령 타임아웃 |

### 8.2 Error 메시지

```json
{
  "type": "error",
  "priority": "high",
  "payload": {
    "error_code": 1001,
    "error_name": "CHECKSUM_FAILED",
    "message": "Message checksum verification failed",
    "details": {
      "expected": "a1b2c3d4",
      "actual": "e5f6g7h8",
      "message_id": "550e8400-e29b-41d4-a716-446655440000"
    },
    "recoverable": true,
    "suggested_action": "resend_message"
  }
}
```

---

## 9. 전송 계층 (Transport Layer)

### 9.1 지원 전송 방식

| 전송 | 사용 사례 | 특징 |
|------|----------|------|
| **WebSocket** | 웹 인터페이스, 대시보드 | 브라우저 호환, 양방향 |
| **MQTT** | 클라우드 텔레메트리, IoT | 확장성, 경량 |
| **ROS2 DDS** | ROS 로봇, 실시간 제어 | 저지연, QoS |
| **TCP** | 직접 연결, 레거시 | 단순, 신뢰성 |

### 9.2 Transport 추상화

```
┌─────────────────────────────────────┐
│      WRP Application Layer          │
├─────────────────────────────────────┤
│      Transport Interface            │
│  ┌─────┬─────┬─────┬─────┬─────┐   │
│  │ WS  │MQTT │ DDS │ TCP │Mock │   │
│  └─────┴─────┴─────┴─────┴─────┘   │
└─────────────────────────────────────┘
```

### 9.3 WebSocket 설정

```json
{
  "transport": "websocket",
  "config": {
    "url": "wss://robot.example.com/wrp",
    "port": 443,
    "path": "/wrp",
    "subprotocol": "wia-robot-v1",
    "ping_interval_ms": 30000,
    "reconnect_delay_ms": 1000,
    "max_reconnect_attempts": 5
  }
}
```

### 9.4 MQTT 설정

```json
{
  "transport": "mqtt",
  "config": {
    "broker": "mqtt://broker.example.com",
    "port": 8883,
    "client_id": "exo-001",
    "topic_prefix": "wia/robot",
    "qos": 1,
    "use_tls": true,
    "clean_session": false
  }
}
```

### 9.5 ROS2 DDS 토픽 매핑

| WRP 메시지 | ROS2 토픽 |
|-----------|----------|
| telemetry | `/wia/robot/{device_id}/telemetry` |
| control | `/wia/robot/{device_id}/control` |
| emergency_stop | `/wia/robot/emergency_stop` |
| safety_alert | `/wia/robot/{device_id}/safety` |
| status | `/wia/robot/{device_id}/status` |

---

## 10. ROS2 호환성 (ROS2 Compatibility)

### 10.1 메시지 타입 매핑

| WRP 필드 | ROS2 타입 |
|----------|----------|
| Position3D | geometry_msgs/Point |
| Orientation | geometry_msgs/Quaternion |
| Joint | sensor_msgs/JointState |
| ImuData | sensor_msgs/Imu |
| Timestamp | builtin_interfaces/Time |

### 10.2 QoS 매핑

| WRP Priority | DDS QoS Profile |
|--------------|-----------------|
| emergency | RELIABLE, TRANSIENT_LOCAL, deadline=10ms |
| critical | RELIABLE, TRANSIENT_LOCAL, deadline=50ms |
| high | RELIABLE, VOLATILE, deadline=100ms |
| normal | BEST_EFFORT, VOLATILE |
| low | BEST_EFFORT, VOLATILE |

### 10.3 ROS2 브릿지 아키텍처

```
┌─────────────┐         ┌─────────────┐         ┌─────────────┐
│   WRP       │         │    ROS2     │         │   ROS2      │
│   Device    │◄───────►│   Bridge    │◄───────►│   Node      │
└─────────────┘  WRP    └─────────────┘  DDS    └─────────────┘
```

---

## 11. 보안 (Security)

### 11.1 전송 암호화

- **필수**: TLS 1.3 이상
- **인증서**: X.509 디바이스 인증서
- **키 교환**: ECDHE

### 11.2 메시지 인증

- **체크섬**: CRC32 (무결성)
- **서명**: HMAC-SHA256 (인증, 선택)

### 11.3 접근 제어

```json
{
  "access_control": {
    "device_id": "exo-001",
    "allowed_operations": ["telemetry", "status"],
    "denied_operations": ["config", "control"],
    "requires_authentication": true
  }
}
```

---

## 12. 예제 (Examples)

### 12.1 외골격 텔레메트리

```json
{
  "protocol": "wia-robot",
  "version": "1.0.0",
  "message_id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "timestamp": "2025-01-15T10:30:00.123Z",
  "sequence": 100,
  "type": "telemetry",
  "priority": "normal",
  "source": {
    "device_id": "exo-rewalk-001",
    "device_type": "exoskeleton"
  },
  "destination": {
    "device_id": "server-main-001",
    "device_type": "server"
  },
  "safety": {
    "emergency_stop": false,
    "safety_level": "normal",
    "requires_ack": false
  },
  "payload": {
    "joints": [
      {
        "name": "hip_left",
        "angle_deg": 15.5,
        "velocity_deg_s": 2.3,
        "torque_nm": 45.2
      }
    ],
    "gait": {
      "phase": "swing",
      "step_count": 1523,
      "cadence_steps_min": 65
    },
    "imu": {
      "accel_x": 0.1,
      "accel_y": -0.2,
      "accel_z": 9.8,
      "gyro_x": 0.01,
      "gyro_y": -0.02,
      "gyro_z": 0.0
    }
  },
  "checksum": "f3a8b2c1"
}
```

### 12.2 수술 로봇 제어 명령

```json
{
  "protocol": "wia-robot",
  "version": "1.0.0",
  "message_id": "b2c3d4e5-f6a7-8901-bcde-f23456789012",
  "timestamp": "2025-01-15T11:30:00.456Z",
  "sequence": 50001,
  "type": "control",
  "priority": "high",
  "source": {
    "device_id": "console-main-001",
    "device_type": "surgeon_console"
  },
  "destination": {
    "device_id": "surg-davinci-001",
    "device_type": "surgical"
  },
  "safety": {
    "emergency_stop": false,
    "safety_level": "normal",
    "requires_ack": true,
    "ack_timeout_ms": 100
  },
  "payload": {
    "command": "move_instrument",
    "command_id": "cmd-50001",
    "parameters": {
      "arm_id": 1,
      "target_position": {
        "x": 125.5,
        "y": 85.3,
        "z": 200.1
      },
      "velocity_limit_mm_s": 10.0,
      "motion_scaling": 3.0
    }
  },
  "checksum": "d4e5f6a7"
}
```

---

## 13. 참고문헌 (References)

1. [ROS2 DDS-RTPS Specification](https://design.ros2.org/articles/ros_on_dds.html)
2. [MQTT Version 5.0](https://docs.oasis-open.org/mqtt/mqtt/v5.0/mqtt-v5.0.html)
3. [RFC 6455 - WebSocket Protocol](https://datatracker.ietf.org/doc/html/rfc6455)
4. [HL7 FHIR R4](https://www.hl7.org/fhir/)
5. [IEEE 11073 PHD](https://sagroups.ieee.org/11073/phd-wg/)
6. [ISO 13482:2014](https://www.iso.org/standard/53820.html)
7. [UUID RFC 4122](https://datatracker.ietf.org/doc/html/rfc4122)
8. [ISO 8601 Date/Time](https://www.iso.org/iso-8601-date-and-time-format.html)

---

**Document Version**: 1.0.0
**Last Updated**: 2025-01-15
**Status**: Draft

---

<div align="center">

**WIA Robot Protocol (WRP)**

Safe, Real-time, Accessible Communication

弘益人間 - Benefit All Humanity

</div>
