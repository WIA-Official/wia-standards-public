# WIA Eye Gaze Examples

Example applications demonstrating the WIA Eye Gaze Standard communication protocols.

**弘益人間** - 널리 인간을 이롭게

---

## WebSocket Client

A browser-based client demonstrating real-time gaze data streaming over WebSocket.

### Files

- `websocket-client/index.html` - Single-file HTML/JS client

### Features

- Connect to WIA Eye Gaze WebSocket server
- Start/Stop/Pause streaming
- Real-time gaze visualization
- Binary/JSON mode toggle
- Latency measurement
- Event logging

### Usage

1. Start a WIA Eye Gaze server:
   ```bash
   # Using the Rust server (requires 'server' feature)
   cargo run --features server --example gaze-server
   ```

2. Open `websocket-client/index.html` in a browser

3. Click "Connect" to connect to the server

4. Click "Start Streaming" to begin receiving gaze data

### Protocol

The client uses the WIA Eye Gaze WebSocket Protocol v1:

```
ws://localhost:8765/wia-eye-gaze/v1/stream
Subprotocol: wia-eye-gaze-v1
```

---

## IPC Client

A Rust-based client demonstrating ultra-low-latency communication via Unix Domain Socket.

### Files

- `ipc-client/main.rs` - Rust IPC client

### Features

- Connect to Unix Domain Socket
- Handshake with server
- Receive gaze points and events
- Heartbeat keep-alive

### Usage

1. Start an IPC server:
   ```bash
   cargo run --example gaze-ipc-server
   ```

2. Run the IPC client:
   ```bash
   cargo run --example ipc-client
   ```

### Protocol

The client uses the WIA Eye Gaze IPC Protocol:

```
Socket: /tmp/wia-eye-gaze.sock (Unix)
         \\.\pipe\wia-eye-gaze (Windows)
```

---

## Running Examples

### Prerequisites

- Rust 1.70+ (for Rust examples)
- Modern web browser (for WebSocket client)

### Build

```bash
# Build all examples
cd eye-gaze/api/rust
cargo build --examples --features full

# Run specific example
cargo run --example websocket-client --features server
cargo run --example ipc-client
```

---

## Example Server

To test the clients, you can run a mock gaze server:

```rust
use wia_eye_gaze::{GazeServer, ServerConfig, MockAdapter, EyeTracker};
use std::sync::Arc;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create server
    let config = ServerConfig {
        port: 8765,
        frequency: 60,
        ..Default::default()
    };
    let server = GazeServer::with_config(config);
    let state = server.state();

    // Create mock tracker
    let adapter = MockAdapter::new(60);
    let mut tracker = EyeTracker::new(adapter);

    // Connect and start streaming
    tracker.connect().await?;
    tracker.start_tracking();

    // Broadcast gaze data to WebSocket clients
    tracker.subscribe(move |gaze| {
        state.broadcast_gaze(gaze.clone());
    });

    // Start server
    server.start().await?;
    Ok(())
}
```

---

## Protocol Reference

See the specification documents for detailed protocol information:

- `spec/WEBSOCKET-PROTOCOL.md` - WebSocket protocol specification
- `spec/IPC-PROTOCOL.md` - IPC protocol specification
- `spec/BINARY-FORMAT.md` - Binary encoding specification

---

**WIA Eye Gaze Standard - Phase 3**
