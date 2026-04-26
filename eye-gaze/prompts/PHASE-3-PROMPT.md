# WIA Eye Gaze Standard - Phase 3: Real-time Communication Protocol

## 전제조건
- Phase 1 (데이터 포맷 표준) 완료
- Phase 2 (API Interface) 완료

## Phase 3 목표: 실시간 통신 프로토콜 구현

### 3.1 WebSocket 기반 실시간 스트리밍

```typescript
// Server → Client: Gaze Data Stream
interface GazeStreamMessage {
  type: 'gaze_data';
  timestamp: number;
  points: GazePoint[];        // Batch of points (for efficiency)
  frequency: number;          // Current streaming frequency
}

// Client → Server: Control Messages
interface GazeControlMessage {
  type: 'control';
  action: 'start' | 'stop' | 'pause' | 'resume' | 'calibrate';
  params?: Record<string, unknown>;
}

// WebSocket Endpoint
// ws://localhost:8765/wia-eye-gaze/v1/stream
```

### 3.2 Inter-Process Communication (IPC)

같은 시스템에서 실행되는 앱들 간의 저지연 통신:

```typescript
interface IPCChannel {
  // Named Pipe (Windows) / Unix Domain Socket (Linux/macOS)
  path: string;  // e.g., '\\\\.\\pipe\\wia-eye-gaze' or '/tmp/wia-eye-gaze.sock'

  // Message Format
  message: {
    header: {
      version: number;
      messageType: number;
      payloadLength: number;
      timestamp: number;
    };
    payload: Uint8Array;
  };
}

// Message Types
enum IPCMessageType {
  GAZE_POINT = 0x01,
  GAZE_EVENT = 0x02,
  APP_MESSAGE = 0x03,
  CONTROL_CMD = 0x04,
  STATUS = 0x05,
}
```

### 3.3 Service Discovery

로컬 네트워크에서 Eye Tracker 서비스 자동 발견:

```typescript
interface ServiceDiscovery {
  // mDNS/Bonjour service type
  serviceType: '_wia-eye-gaze._tcp';

  // TXT Records
  txtRecords: {
    version: string;           // '1.0.0'
    deviceVendor: string;      // 'tobii'
    deviceModel: string;       // 'eye-tracker-5'
    capabilities: string;      // JSON encoded capabilities
  };

  // Discovery API
  discover(): Promise<EyeTrackerService[]>;
  advertise(service: EyeTrackerService): void;
}
```

### 3.4 Latency Requirements

실시간 보조기기로서의 지연시간 요구사항:

| 구간 | 최대 지연 | 설명 |
|------|----------|------|
| Eye → Tracker | 16ms | 60Hz tracking |
| Tracker → App | 10ms | IPC/WebSocket |
| App → UI Feedback | 16ms | 60fps rendering |
| **Total** | **< 50ms** | Human perception threshold |

### 3.5 Binary Protocol (고성능용)

JSON 대신 바이너리 포맷으로 대역폭/지연시간 최적화:

```rust
// Rust struct (bincode/MessagePack serializable)
#[repr(C, packed)]
struct GazePointBinary {
    timestamp: u64,      // 8 bytes
    x: f32,              // 4 bytes
    y: f32,              // 4 bytes
    confidence: f32,     // 4 bytes
    flags: u8,           // 1 byte (fixation, saccade, blink bits)
    // Total: 21 bytes per point
}

// vs JSON: ~150 bytes per point
// 7x bandwidth reduction
```

---

## 구현 요구사항

### WebSocket Server (Rust - Axum)

```rust
// eye-gaze/api/rust/src/server.rs
async fn gaze_websocket(ws: WebSocketUpgrade) -> impl IntoResponse {
    ws.on_upgrade(handle_gaze_stream)
}

async fn handle_gaze_stream(mut socket: WebSocket) {
    let mut interval = tokio::time::interval(Duration::from_millis(16)); // 60Hz

    loop {
        tokio::select! {
            _ = interval.tick() => {
                let points = get_gaze_points();
                socket.send(Message::Binary(encode_points(&points))).await?;
            }
            Some(msg) = socket.recv() => {
                handle_control_message(msg).await?;
            }
        }
    }
}
```

### IPC Implementation

```rust
// Unix Domain Socket Server
use tokio::net::UnixListener;

async fn ipc_server() -> Result<()> {
    let listener = UnixListener::bind("/tmp/wia-eye-gaze.sock")?;

    loop {
        let (stream, _) = listener.accept().await?;
        tokio::spawn(handle_ipc_client(stream));
    }
}
```

---

## 산출물

Phase 3 완료 시:
```
eye-gaze/
├── spec/
│   ├── WEBSOCKET-PROTOCOL.md
│   ├── IPC-PROTOCOL.md
│   └── BINARY-FORMAT.md
├── api/rust/
│   └── src/
│       ├── server.rs
│       ├── ipc.rs
│       ├── discovery.rs
│       └── binary.rs
└── examples/
    ├── websocket-client/
    └── ipc-client/
```

---

## 다음 단계

Phase 3 완료 후:
- `prompts/PHASE-4-PROMPT.md` 읽고 Phase 4 (에코시스템 통합) 시작
