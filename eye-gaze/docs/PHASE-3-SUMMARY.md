# WIA Eye Gaze Standard - Phase 3 Summary

**Phase 3: Real-time Communication Protocol**
**Status**: Complete
**Date**: 2025-01

---

## 1. Phase 3 개요 (Overview)

Phase 3에서는 WIA Eye Gaze 표준의 실시간 통신 프로토콜을 구현했습니다. WebSocket, IPC, 서비스 발견, 바이너리 프로토콜을 통해 저지연 시선 데이터 전송이 가능합니다.

### 1.1 목표

- **WebSocket 스트리밍** - 실시간 시선 데이터 전송
- **IPC (Inter-Process Communication)** - 초저지연 로컬 통신
- **Service Discovery** - mDNS 기반 자동 디바이스 발견
- **Binary Protocol** - 7배 대역폭 감소

### 1.2 철학

**弘益人間 (홍익인간)** - 널리 인간을 이롭게

---

## 2. 구현 현황

### 2.1 프로토콜 명세서

| 문서 | 파일 | 설명 |
|------|------|------|
| **WebSocket Protocol** | `spec/WEBSOCKET-PROTOCOL.md` | WebSocket 메시지 포맷, 제어 명령, 재연결 |
| **IPC Protocol** | `spec/IPC-PROTOCOL.md` | Unix Socket/Named Pipe, 프레임 포맷 |
| **Binary Format** | `spec/BINARY-FORMAT.md` | 21바이트 GazePoint, 32바이트 Event |

### 2.2 Rust 구현

| 모듈 | 파일 | 설명 |
|------|------|------|
| **Binary** | `src/binary.rs` | 바이너리 인코딩/디코딩 |
| **Server** | `src/server.rs` | Axum WebSocket 서버 |
| **IPC** | `src/ipc.rs` | Unix Domain Socket 클라이언트/서버 |
| **Discovery** | `src/discovery.rs` | mDNS 서비스 발견/광고 |

### 2.3 예제 애플리케이션

| 예제 | 파일 | 설명 |
|------|------|------|
| **WebSocket Client** | `examples/websocket-client/index.html` | 브라우저 기반 시각화 클라이언트 |
| **IPC Client** | `examples/ipc-client/main.rs` | Rust IPC 클라이언트 |

---

## 3. 핵심 프로토콜

### 3.1 WebSocket Protocol

```
Endpoint: ws://localhost:8765/wia-eye-gaze/v1/stream
Subprotocol: wia-eye-gaze-v1
```

**메시지 타입:**

| Type | Direction | Description |
|------|-----------|-------------|
| `gaze_data` | Server → Client | 시선 데이터 배치 |
| `gaze_event` | Server → Client | 시선 이벤트 |
| `control` | Client → Server | 제어 명령 |
| `status` | Both | 상태 업데이트 |
| `ping/pong` | Both | Keep-alive |

**제어 명령:**

```typescript
type ControlAction =
  | 'start'           // 스트리밍 시작
  | 'stop'            // 스트리밍 중지
  | 'pause'           // 일시정지
  | 'resume'          // 재개
  | 'calibrate'       // 캘리브레이션
  | 'set_frequency'   // 주파수 변경
  | 'set_format';     // JSON/Binary 전환
```

### 3.2 Binary Format

**GazePointBinary (21 bytes):**

```rust
#[repr(C, packed)]
struct GazePointBinary {
    timestamp_offset: u16,  // 2 bytes
    x: f32,                 // 4 bytes
    y: f32,                 // 4 bytes
    confidence: u16,        // 2 bytes (norm16)
    flags: u8,              // 1 byte
    left_pupil: u16,        // 2 bytes
    right_pupil: u16,       // 2 bytes
    left_openness: u8,      // 1 byte
    right_openness: u8,     // 1 byte
    fixation_id: u16,       // 2 bytes
}
```

**대역폭 비교:**

| Format | Bytes/Point | 60Hz Data Rate |
|--------|-------------|----------------|
| JSON | ~150 bytes | ~9 KB/s |
| Binary | 21 bytes | ~1.3 KB/s |

### 3.3 IPC Protocol

**경로:**

| Platform | Path |
|----------|------|
| Linux/macOS | `/tmp/wia-eye-gaze.sock` |
| Windows | `\\.\pipe\wia-eye-gaze` |

**Frame Header (20 bytes):**

```
┌──────────┬─────────┬──────────┬─────────┬──────────────────┐
│ Magic(2) │ Ver(1)  │ Type(1)  │ Flags   │ PayloadLen(2)    │
├──────────┴─────────┴──────────┴─────────┴──────────────────┤
│ Timestamp (8 bytes)                                        │
├────────────────────────────────────────────────────────────┤
│ Sequence (4 bytes)                                         │
└────────────────────────────────────────────────────────────┘
```

### 3.4 Service Discovery

```
mDNS Service Type: _wia-eye-gaze._tcp

TXT Records:
- version: "1.0.0"
- deviceVendor: "tobii"
- deviceModel: "eye-tracker-5"
- capabilities: "{...}"
```

---

## 4. 지연시간 요구사항

| 구간 | 최대 지연 | 구현 |
|------|----------|------|
| Eye → Tracker | 16ms | 디바이스 의존 |
| Tracker → Server | 5ms | USB/내부 처리 |
| WebSocket 전송 | 3ms | 로컬 네트워크 |
| IPC 전송 | 0.5ms | Unix Socket |
| **총합 (WebSocket)** | **< 26ms** | 50ms 예산 내 |
| **총합 (IPC)** | **< 22ms** | 초저지연 |

---

## 5. 사용 예시

### 5.1 WebSocket 서버 시작

```rust
use wia_eye_gaze::{GazeServer, ServerConfig};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config = ServerConfig {
        port: 8765,
        frequency: 60,
        binary_mode: false,
        ..Default::default()
    };

    let server = GazeServer::with_config(config);
    let state = server.state();

    // 시선 데이터 브로드캐스트
    // state.broadcast_gaze(gaze_point);

    server.start().await?;
    Ok(())
}
```

### 5.2 WebSocket 클라이언트 (TypeScript)

```typescript
const ws = new WebSocket('ws://localhost:8765/wia-eye-gaze/v1/stream', ['wia-eye-gaze-v1']);

ws.onopen = () => {
  // 스트리밍 시작
  ws.send(JSON.stringify({
    type: 'control',
    timestamp: Date.now(),
    action: 'start'
  }));
};

ws.onmessage = (event) => {
  const msg = JSON.parse(event.data);
  if (msg.type === 'gaze_data') {
    for (const point of msg.points) {
      console.log(`Gaze: (${point.x}, ${point.y})`);
    }
  }
};
```

### 5.3 IPC 클라이언트 (Rust)

```rust
use wia_eye_gaze::ipc::{IpcClient, ClientType};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut client = IpcClient::new();

    // 연결
    client.connect_default()?;

    // 핸드셰이크
    client.handshake(ClientType::Consumer, "My App")?;

    // 메시지 수신
    loop {
        match client.recv_message()? {
            IpcMessage::GazePoint(point) => {
                println!("Gaze: ({}, {})", point.x, point.y);
            }
            _ => {}
        }
    }
}
```

### 5.4 서비스 발견

```rust
use wia_eye_gaze::discovery::ServiceDiscovery;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let discovery = ServiceDiscovery::new();

    // 네트워크에서 Eye Tracker 서비스 검색
    let services = discovery.discover().await?;

    for service in services {
        println!("Found: {} at {}", service.name, service.websocket_url());
    }

    Ok(())
}
```

---

## 6. 디렉토리 구조

```
eye-gaze/
├── spec/
│   ├── WEBSOCKET-PROTOCOL.md    # WebSocket 프로토콜 명세
│   ├── IPC-PROTOCOL.md          # IPC 프로토콜 명세
│   └── BINARY-FORMAT.md         # 바이너리 포맷 명세
├── api/rust/
│   └── src/
│       ├── binary.rs            # 바이너리 인코딩
│       ├── server.rs            # WebSocket 서버
│       ├── ipc.rs               # IPC 클라이언트/서버
│       └── discovery.rs         # 서비스 발견
├── examples/
│   ├── websocket-client/        # 브라우저 클라이언트
│   │   └── index.html
│   ├── ipc-client/              # Rust IPC 클라이언트
│   │   └── main.rs
│   └── README.md
└── docs/
    └── PHASE-3-SUMMARY.md       # 이 문서
```

---

## 7. Feature Flags (Rust)

```toml
[features]
default = ["async"]
async = ["tokio"]
server = ["axum", "tokio-tungstenite", "futures-util", "tower", "tower-http"]
discovery = ["mdns-sd"]
msgpack = ["rmp-serde"]
full = ["async", "server", "discovery", "msgpack"]
```

---

## 8. 다음 단계

### Phase 4: Integration

```
목표: WIA 생태계 통합

작업:
├── WIA AAC Standard 연동
├── 데모 애플리케이션
├── 문서 및 튜토리얼
└── 패키지 배포
    ├── npm: @anthropics/wia-eye-gaze
    ├── PyPI: wia-eye-gaze
    └── crates.io: wia-eye-gaze
```

---

## 9. 결론

Phase 3에서 WIA Eye Gaze 표준의 실시간 통신 프로토콜을 완성했습니다.

### 주요 성과

1. **WebSocket Protocol** - 실시간 양방향 통신
2. **IPC Protocol** - 0.5ms 미만 초저지연
3. **Binary Format** - 7배 대역폭 절감
4. **Service Discovery** - 제로 설정 디바이스 발견
5. **Production-Ready** - Axum 기반 서버, 재연결, 오류 처리

### 기대 효과

- **실시간 AAC 앱** 구현 가능
- **다중 앱 동시 연결** 지원
- **네트워크 환경** 유연성
- **성능 최적화** 옵션

---

<div align="center">

**WIA Eye Gaze Standard - Phase 3 Complete**

**弘益人間** - 널리 인간을 이롭게

🤟

**Next: Phase 4 - Integration**

</div>
