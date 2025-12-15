//! WIA Eye Gaze - IPC Client Example
//!
//! Demonstrates connecting to the WIA Eye Gaze IPC server
//! and receiving gaze data via Unix Domain Socket.
//!
//! Usage:
//!   cargo run --example ipc-client
//!
//! 弘益人間 - 널리 인간을 이롭게

use std::io::{Read, Write};
use std::time::Duration;

#[cfg(unix)]
use std::os::unix::net::UnixStream;

/// IPC socket path
const SOCKET_PATH: &str = "/tmp/wia-eye-gaze.sock";

/// IPC Frame Header size
const HEADER_SIZE: usize = 20;

/// Magic bytes
const MAGIC: [u8; 2] = [0x57, 0x49];

fn main() {
    println!("WIA Eye Gaze - IPC Client Example");
    println!("==================================");
    println!();

    #[cfg(unix)]
    {
        match run_client() {
            Ok(_) => println!("Client finished"),
            Err(e) => eprintln!("Error: {}", e),
        }
    }

    #[cfg(not(unix))]
    {
        println!("IPC example is currently Unix-only.");
        println!("On Windows, Named Pipes would be used instead.");
    }
}

#[cfg(unix)]
fn run_client() -> Result<(), Box<dyn std::error::Error>> {
    println!("Connecting to {}...", SOCKET_PATH);

    let mut stream = UnixStream::connect(SOCKET_PATH)?;
    stream.set_read_timeout(Some(Duration::from_secs(5)))?;

    println!("Connected!");
    println!();

    // Send handshake
    println!("Sending handshake...");
    send_handshake(&mut stream, "IPC Client Example")?;

    // Wait for handshake ack
    let msg = recv_message(&mut stream)?;
    match msg {
        IpcMessage::HandshakeAck { accepted, session_id } => {
            if accepted {
                println!("Handshake accepted! Session ID: {}", session_id);
            } else {
                return Err("Handshake rejected".into());
            }
        }
        _ => return Err("Unexpected message".into()),
    }

    println!();
    println!("Waiting for gaze data (press Ctrl+C to stop)...");
    println!();

    // Receive gaze data
    let mut count = 0;
    loop {
        match recv_message(&mut stream) {
            Ok(msg) => {
                match msg {
                    IpcMessage::GazePoint { x, y, confidence, valid } => {
                        count += 1;
                        if count % 60 == 0 {
                            println!(
                                "Gaze #{}: ({:.3}, {:.3}) confidence: {:.0}% valid: {}",
                                count, x, y, confidence * 100.0, valid
                            );
                        }
                    }
                    IpcMessage::GazeBatch { points } => {
                        count += points.len();
                        if let Some(last) = points.last() {
                            println!(
                                "Batch #{}: {} points, last: ({:.3}, {:.3})",
                                count, points.len(), last.x, last.y
                            );
                        }
                    }
                    IpcMessage::GazeEvent { event_type } => {
                        println!("Event: {}", event_type);
                    }
                    IpcMessage::Status { connected, tracking, calibrated } => {
                        println!(
                            "Status: connected={} tracking={} calibrated={}",
                            connected, tracking, calibrated
                        );
                    }
                    IpcMessage::Error { code, message } => {
                        eprintln!("Error {}: {}", code, message);
                    }
                    _ => {}
                }
            }
            Err(e) => {
                if e.to_string().contains("timed out") {
                    // Send heartbeat
                    send_heartbeat(&mut stream)?;
                } else {
                    return Err(e);
                }
            }
        }
    }
}

/// IPC Message types
#[derive(Debug)]
enum IpcMessage {
    GazePoint {
        x: f64,
        y: f64,
        confidence: f64,
        valid: bool,
    },
    GazeBatch {
        points: Vec<GazePointData>,
    },
    GazeEvent {
        event_type: String,
    },
    Status {
        connected: bool,
        tracking: bool,
        calibrated: bool,
    },
    HandshakeAck {
        accepted: bool,
        session_id: u32,
    },
    Heartbeat,
    Error {
        code: u8,
        message: String,
    },
}

#[derive(Debug)]
struct GazePointData {
    x: f64,
    y: f64,
}

#[cfg(unix)]
fn send_handshake(stream: &mut UnixStream, app_name: &str) -> Result<(), Box<dyn std::error::Error>> {
    // Build handshake payload
    let mut payload = Vec::new();

    // Client type (0x03 = App)
    payload.push(0x03);

    // Client ID (16 bytes UUID)
    let uuid = uuid::Uuid::new_v4();
    payload.extend_from_slice(uuid.as_bytes());

    // Protocol version (0x0100 = v1.0)
    payload.extend_from_slice(&0x0100u16.to_le_bytes());

    // Capabilities (binary gaze + events)
    payload.extend_from_slice(&0x03u32.to_le_bytes());

    // App name
    payload.push(app_name.len() as u8);
    payload.extend_from_slice(app_name.as_bytes());

    // Build header
    let header = build_header(0x10, payload.len() as u16); // 0x10 = Handshake

    stream.write_all(&header)?;
    stream.write_all(&payload)?;
    stream.flush()?;

    Ok(())
}

#[cfg(unix)]
fn send_heartbeat(stream: &mut UnixStream) -> Result<(), Box<dyn std::error::Error>> {
    let header = build_header(0x12, 0); // 0x12 = Heartbeat
    stream.write_all(&header)?;
    stream.flush()?;
    Ok(())
}

fn build_header(msg_type: u8, payload_len: u16) -> [u8; HEADER_SIZE] {
    let mut header = [0u8; HEADER_SIZE];

    // Magic
    header[0] = MAGIC[0];
    header[1] = MAGIC[1];

    // Version
    header[2] = 0x01;

    // Message type
    header[3] = msg_type;

    // Flags
    header[4] = 0;

    // Reserved
    header[5] = 0;

    // Payload length (little-endian)
    header[6..8].copy_from_slice(&payload_len.to_le_bytes());

    // Timestamp (microseconds)
    let timestamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_micros() as u64;
    header[8..16].copy_from_slice(&timestamp.to_le_bytes());

    // Sequence (0 for now)
    header[16..20].copy_from_slice(&0u32.to_le_bytes());

    header
}

#[cfg(unix)]
fn recv_message(stream: &mut UnixStream) -> Result<IpcMessage, Box<dyn std::error::Error>> {
    // Read header
    let mut header = [0u8; HEADER_SIZE];
    stream.read_exact(&mut header)?;

    // Validate magic
    if header[0] != MAGIC[0] || header[1] != MAGIC[1] {
        return Err("Invalid magic number".into());
    }

    let msg_type = header[3];
    let payload_len = u16::from_le_bytes([header[6], header[7]]) as usize;

    // Read payload
    let mut payload = vec![0u8; payload_len];
    if payload_len > 0 {
        stream.read_exact(&mut payload)?;
    }

    // Parse message
    match msg_type {
        0x01 => { // GazePoint
            if payload.len() < 21 {
                return Err("Invalid gaze point payload".into());
            }
            let x = f32::from_le_bytes(payload[2..6].try_into()?) as f64;
            let y = f32::from_le_bytes(payload[6..10].try_into()?) as f64;
            let confidence = u16::from_le_bytes(payload[10..12].try_into()?) as f64 / 65535.0;
            let valid = (payload[12] & 0x01) != 0;

            Ok(IpcMessage::GazePoint { x, y, confidence, valid })
        }
        0x02 => { // GazeBatch
            // Simplified parsing
            Ok(IpcMessage::GazeBatch { points: Vec::new() })
        }
        0x05 => { // Status
            if payload.is_empty() {
                return Err("Invalid status payload".into());
            }
            let flags = payload[0];
            Ok(IpcMessage::Status {
                connected: (flags & 0x01) != 0,
                tracking: (flags & 0x02) != 0,
                calibrated: (flags & 0x04) != 0,
            })
        }
        0x11 => { // HandshakeAck
            if payload.len() < 5 {
                return Err("Invalid handshake ack payload".into());
            }
            Ok(IpcMessage::HandshakeAck {
                accepted: payload[0] != 0,
                session_id: u32::from_le_bytes(payload[1..5].try_into()?),
            })
        }
        0x12 => { // Heartbeat
            Ok(IpcMessage::Heartbeat)
        }
        0x30 => { // Error
            if payload.len() < 2 {
                return Err("Invalid error payload".into());
            }
            let code = payload[0];
            let msg_len = payload[1] as usize;
            let message = if payload.len() >= 2 + msg_len {
                String::from_utf8_lossy(&payload[2..2 + msg_len]).to_string()
            } else {
                "Unknown error".to_string()
            };
            Ok(IpcMessage::Error { code, message })
        }
        _ => Err(format!("Unknown message type: 0x{:02x}", msg_type).into()),
    }
}
