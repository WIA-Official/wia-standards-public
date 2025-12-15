//! WIA Eye Gaze Standard - IPC (Inter-Process Communication)
//!
//! Ultra-low-latency communication using Unix Domain Sockets (Linux/macOS)
//! and Named Pipes (Windows).
//!
//! 弘益人間 - 널리 인간을 이롭게

use std::collections::HashMap;
use std::io::{self, Read, Write};
use std::sync::Arc;

use crate::binary::{
    decode_batch, decode_frame_header, encode_batch, encode_frame, DecodeError, FrameHeader,
    GazeEventBinary, GazePointBinary, MessageType, MAGIC, VERSION,
};
use crate::types::{GazeEvent, GazePoint};

#[cfg(unix)]
use std::os::unix::net::{UnixListener, UnixStream};

/// Default Unix socket path
#[cfg(unix)]
pub const DEFAULT_SOCKET_PATH: &str = "/tmp/wia-eye-gaze.sock";

/// Default Named Pipe path (Windows)
#[cfg(windows)]
pub const DEFAULT_PIPE_PATH: &str = r"\\.\pipe\wia-eye-gaze";

/// IPC Message types
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IpcMessageType {
    GazePoint = 0x01,
    GazeBatch = 0x02,
    GazeEvent = 0x03,
    Control = 0x04,
    Status = 0x05,
    Handshake = 0x10,
    HandshakeAck = 0x11,
    Heartbeat = 0x12,
    AppMessage = 0x20,
    Error = 0x30,
}

impl TryFrom<u8> for IpcMessageType {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x01 => Ok(IpcMessageType::GazePoint),
            0x02 => Ok(IpcMessageType::GazeBatch),
            0x03 => Ok(IpcMessageType::GazeEvent),
            0x04 => Ok(IpcMessageType::Control),
            0x05 => Ok(IpcMessageType::Status),
            0x10 => Ok(IpcMessageType::Handshake),
            0x11 => Ok(IpcMessageType::HandshakeAck),
            0x12 => Ok(IpcMessageType::Heartbeat),
            0x20 => Ok(IpcMessageType::AppMessage),
            0x30 => Ok(IpcMessageType::Error),
            _ => Err(()),
        }
    }
}

/// IPC Frame Header (20 bytes)
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct IpcFrameHeader {
    /// Magic bytes ("WI")
    pub magic: [u8; 2],
    /// Protocol version
    pub version: u8,
    /// Message type
    pub msg_type: u8,
    /// Flags
    pub flags: u8,
    /// Reserved
    pub reserved: u8,
    /// Payload length (little-endian)
    pub payload_len: u16,
    /// Timestamp (microseconds)
    pub timestamp: u64,
    /// Sequence number
    pub sequence: u32,
}

impl IpcFrameHeader {
    /// Size of the IPC header
    pub const SIZE: usize = 20;

    /// Create new IPC frame header
    pub fn new(msg_type: IpcMessageType, payload_len: u16, sequence: u32) -> Self {
        Self {
            magic: MAGIC,
            version: VERSION,
            msg_type: msg_type as u8,
            flags: 0,
            reserved: 0,
            payload_len,
            timestamp: now_micros(),
            sequence,
        }
    }

    /// Serialize to bytes
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut bytes = [0u8; Self::SIZE];
        bytes[0..2].copy_from_slice(&self.magic);
        bytes[2] = self.version;
        bytes[3] = self.msg_type;
        bytes[4] = self.flags;
        bytes[5] = self.reserved;
        bytes[6..8].copy_from_slice(&self.payload_len.to_le_bytes());
        bytes[8..16].copy_from_slice(&self.timestamp.to_le_bytes());
        bytes[16..20].copy_from_slice(&self.sequence.to_le_bytes());
        bytes
    }

    /// Deserialize from bytes
    pub fn from_bytes(bytes: &[u8; Self::SIZE]) -> Result<Self, IpcError> {
        let magic = [bytes[0], bytes[1]];
        if magic != MAGIC {
            return Err(IpcError::InvalidMagic);
        }

        let version = bytes[2];
        if version != VERSION {
            return Err(IpcError::UnsupportedVersion);
        }

        Ok(Self {
            magic,
            version,
            msg_type: bytes[3],
            flags: bytes[4],
            reserved: bytes[5],
            payload_len: u16::from_le_bytes([bytes[6], bytes[7]]),
            timestamp: u64::from_le_bytes(bytes[8..16].try_into().unwrap()),
            sequence: u32::from_le_bytes(bytes[16..20].try_into().unwrap()),
        })
    }
}

/// Handshake message
#[repr(C)]
#[derive(Debug, Clone)]
pub struct Handshake {
    /// Client type
    pub client_type: ClientType,
    /// Client UUID
    pub client_id: [u8; 16],
    /// Protocol version (major.minor)
    pub protocol_version: u16,
    /// Capabilities bitfield
    pub capabilities: u32,
    /// Application name
    pub app_name: String,
}

/// Client types
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClientType {
    /// Consumes gaze data
    Consumer = 0x01,
    /// Produces gaze data (eye tracker adapter)
    Producer = 0x02,
    /// Gaze-aware application
    App = 0x03,
    /// Debugging/monitoring
    Monitor = 0x04,
}

impl TryFrom<u8> for ClientType {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x01 => Ok(ClientType::Consumer),
            0x02 => Ok(ClientType::Producer),
            0x03 => Ok(ClientType::App),
            0x04 => Ok(ClientType::Monitor),
            _ => Err(()),
        }
    }
}

/// Capability flags
pub mod Capabilities {
    pub const BINARY_GAZE: u32 = 0x01;
    pub const EVENTS: u32 = 0x02;
    pub const APP_MESSAGING: u32 = 0x04;
    pub const COMPRESSION: u32 = 0x08;
    pub const ENCRYPTION: u32 = 0x10;
}

/// Control actions
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlAction {
    Start = 0x01,
    Stop = 0x02,
    Pause = 0x03,
    Resume = 0x04,
    Calibrate = 0x05,
    SetFrequency = 0x10,
    SetFormat = 0x11,
    Subscribe = 0x20,
    Unsubscribe = 0x21,
    RegisterApp = 0x30,
    UnregisterApp = 0x31,
}

/// IPC Error types
#[derive(Debug, Clone)]
pub enum IpcError {
    InvalidMagic,
    UnsupportedVersion,
    InvalidMessageType,
    InsufficientData,
    ConnectionClosed,
    Timeout,
    IoError(String),
}

impl std::fmt::Display for IpcError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            IpcError::InvalidMagic => write!(f, "Invalid magic number"),
            IpcError::UnsupportedVersion => write!(f, "Unsupported protocol version"),
            IpcError::InvalidMessageType => write!(f, "Invalid message type"),
            IpcError::InsufficientData => write!(f, "Insufficient data"),
            IpcError::ConnectionClosed => write!(f, "Connection closed"),
            IpcError::Timeout => write!(f, "Operation timed out"),
            IpcError::IoError(e) => write!(f, "IO error: {}", e),
        }
    }
}

impl std::error::Error for IpcError {}

impl From<io::Error> for IpcError {
    fn from(e: io::Error) -> Self {
        IpcError::IoError(e.to_string())
    }
}

/// IPC Message
#[derive(Debug, Clone)]
pub enum IpcMessage {
    GazePoint(GazePoint),
    GazeBatch(Vec<GazePoint>),
    GazeEvent(GazeEvent),
    Control {
        action: ControlAction,
        request_id: u32,
        params: Vec<u8>,
    },
    Status {
        connected: bool,
        tracking: bool,
        calibrated: bool,
    },
    Handshake(Handshake),
    HandshakeAck {
        accepted: bool,
        session_id: u32,
    },
    Heartbeat,
    AppMessage {
        source_app: [u8; 16],
        target_app: [u8; 16],
        msg_type: u8,
        payload: Vec<u8>,
    },
    Error {
        code: u8,
        message: String,
    },
}

/// IPC Client for connecting to the gaze server
pub struct IpcClient {
    #[cfg(unix)]
    stream: Option<UnixStream>,
    sequence: u32,
    session_id: Option<u32>,
    client_id: [u8; 16],
}

impl IpcClient {
    /// Create new IPC client
    pub fn new() -> Self {
        Self {
            #[cfg(unix)]
            stream: None,
            sequence: 0,
            session_id: None,
            client_id: generate_uuid(),
        }
    }

    /// Connect to IPC server
    #[cfg(unix)]
    pub fn connect(&mut self, path: &str) -> Result<(), IpcError> {
        let stream = UnixStream::connect(path)?;
        stream.set_nonblocking(false)?;
        self.stream = Some(stream);
        Ok(())
    }

    /// Connect with default path
    #[cfg(unix)]
    pub fn connect_default(&mut self) -> Result<(), IpcError> {
        self.connect(DEFAULT_SOCKET_PATH)
    }

    /// Send handshake
    pub fn handshake(&mut self, client_type: ClientType, app_name: &str) -> Result<(), IpcError> {
        let handshake = Handshake {
            client_type,
            client_id: self.client_id,
            protocol_version: 0x0100, // v1.0
            capabilities: Capabilities::BINARY_GAZE | Capabilities::EVENTS,
            app_name: app_name.to_string(),
        };

        self.send_message(IpcMessage::Handshake(handshake))?;

        // Wait for ack
        let response = self.recv_message()?;
        match response {
            IpcMessage::HandshakeAck {
                accepted,
                session_id,
            } => {
                if accepted {
                    self.session_id = Some(session_id);
                    Ok(())
                } else {
                    Err(IpcError::ConnectionClosed)
                }
            }
            _ => Err(IpcError::InvalidMessageType),
        }
    }

    /// Send message
    #[cfg(unix)]
    pub fn send_message(&mut self, msg: IpcMessage) -> Result<(), IpcError> {
        let stream = self.stream.as_mut().ok_or(IpcError::ConnectionClosed)?;

        let (msg_type, payload) = encode_ipc_message(&msg);
        let header = IpcFrameHeader::new(msg_type, payload.len() as u16, self.next_sequence());

        stream.write_all(&header.to_bytes())?;
        stream.write_all(&payload)?;
        stream.flush()?;

        Ok(())
    }

    /// Receive message
    #[cfg(unix)]
    pub fn recv_message(&mut self) -> Result<IpcMessage, IpcError> {
        let stream = self.stream.as_mut().ok_or(IpcError::ConnectionClosed)?;

        // Read header
        let mut header_bytes = [0u8; IpcFrameHeader::SIZE];
        stream.read_exact(&mut header_bytes)?;
        let header = IpcFrameHeader::from_bytes(&header_bytes)?;

        // Read payload
        let mut payload = vec![0u8; header.payload_len as usize];
        if !payload.is_empty() {
            stream.read_exact(&mut payload)?;
        }

        decode_ipc_message(header.msg_type, &payload)
    }

    /// Send heartbeat
    pub fn heartbeat(&mut self) -> Result<(), IpcError> {
        self.send_message(IpcMessage::Heartbeat)
    }

    /// Close connection
    #[cfg(unix)]
    pub fn close(&mut self) {
        self.stream = None;
        self.session_id = None;
    }

    fn next_sequence(&mut self) -> u32 {
        self.sequence += 1;
        self.sequence
    }
}

impl Default for IpcClient {
    fn default() -> Self {
        Self::new()
    }
}

/// IPC Server for broadcasting gaze data
#[cfg(unix)]
pub struct IpcServer {
    listener: Option<UnixListener>,
    clients: Vec<UnixStream>,
    sequence: u32,
}

#[cfg(unix)]
impl IpcServer {
    /// Create new IPC server
    pub fn new() -> Self {
        Self {
            listener: None,
            clients: Vec::new(),
            sequence: 0,
        }
    }

    /// Start listening
    pub fn bind(&mut self, path: &str) -> Result<(), IpcError> {
        // Remove stale socket
        let _ = std::fs::remove_file(path);

        let listener = UnixListener::bind(path)?;
        listener.set_nonblocking(true)?;

        // Set permissions (rw-rw----)
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let _ = std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o660));
        }

        self.listener = Some(listener);
        Ok(())
    }

    /// Bind with default path
    pub fn bind_default(&mut self) -> Result<(), IpcError> {
        self.bind(DEFAULT_SOCKET_PATH)
    }

    /// Accept new connections (non-blocking)
    pub fn accept(&mut self) -> Result<bool, IpcError> {
        let listener = self.listener.as_ref().ok_or(IpcError::ConnectionClosed)?;

        match listener.accept() {
            Ok((stream, _)) => {
                stream.set_nonblocking(false)?;
                self.clients.push(stream);
                Ok(true)
            }
            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => Ok(false),
            Err(e) => Err(e.into()),
        }
    }

    /// Broadcast gaze point to all clients
    pub fn broadcast_gaze(&mut self, point: &GazePoint) -> Result<(), IpcError> {
        let msg = IpcMessage::GazePoint(point.clone());
        self.broadcast_message(&msg)
    }

    /// Broadcast gaze batch to all clients
    pub fn broadcast_batch(&mut self, points: &[GazePoint]) -> Result<(), IpcError> {
        let msg = IpcMessage::GazeBatch(points.to_vec());
        self.broadcast_message(&msg)
    }

    /// Broadcast message to all clients
    pub fn broadcast_message(&mut self, msg: &IpcMessage) -> Result<(), IpcError> {
        let (msg_type, payload) = encode_ipc_message(msg);
        let header = IpcFrameHeader::new(msg_type, payload.len() as u16, self.next_sequence());

        let header_bytes = header.to_bytes();

        // Remove disconnected clients
        self.clients.retain_mut(|stream| {
            if stream.write_all(&header_bytes).is_err() {
                return false;
            }
            if stream.write_all(&payload).is_err() {
                return false;
            }
            stream.flush().is_ok()
        });

        Ok(())
    }

    /// Get client count
    pub fn client_count(&self) -> usize {
        self.clients.len()
    }

    fn next_sequence(&mut self) -> u32 {
        self.sequence += 1;
        self.sequence
    }
}

#[cfg(unix)]
impl Default for IpcServer {
    fn default() -> Self {
        Self::new()
    }
}

// Helper functions

fn encode_ipc_message(msg: &IpcMessage) -> (IpcMessageType, Vec<u8>) {
    match msg {
        IpcMessage::GazePoint(point) => {
            let binary = GazePointBinary::from_gaze_point(point, point.timestamp);
            (IpcMessageType::GazePoint, binary.as_bytes().to_vec())
        }
        IpcMessage::GazeBatch(points) => {
            let payload = encode_batch(points);
            (IpcMessageType::GazeBatch, payload)
        }
        IpcMessage::GazeEvent(event) => {
            let binary = GazeEventBinary::from_gaze_event(event);
            (IpcMessageType::GazeEvent, binary.as_bytes().to_vec())
        }
        IpcMessage::Heartbeat => (IpcMessageType::Heartbeat, Vec::new()),
        IpcMessage::Handshake(h) => {
            let mut payload = Vec::new();
            payload.push(h.client_type as u8);
            payload.extend_from_slice(&h.client_id);
            payload.extend_from_slice(&h.protocol_version.to_le_bytes());
            payload.extend_from_slice(&h.capabilities.to_le_bytes());
            payload.push(h.app_name.len() as u8);
            payload.extend_from_slice(h.app_name.as_bytes());
            (IpcMessageType::Handshake, payload)
        }
        IpcMessage::HandshakeAck {
            accepted,
            session_id,
        } => {
            let mut payload = Vec::new();
            payload.push(if *accepted { 1 } else { 0 });
            payload.extend_from_slice(&session_id.to_le_bytes());
            (IpcMessageType::HandshakeAck, payload)
        }
        IpcMessage::Control {
            action,
            request_id,
            params,
        } => {
            let mut payload = Vec::new();
            payload.push(*action as u8);
            payload.extend_from_slice(&request_id.to_le_bytes());
            payload.extend_from_slice(&(params.len() as u16).to_le_bytes());
            payload.extend_from_slice(params);
            (IpcMessageType::Control, payload)
        }
        IpcMessage::Status {
            connected,
            tracking,
            calibrated,
        } => {
            let flags = (*connected as u8) | ((*tracking as u8) << 1) | ((*calibrated as u8) << 2);
            (IpcMessageType::Status, vec![flags])
        }
        IpcMessage::AppMessage {
            source_app,
            target_app,
            msg_type,
            payload,
        } => {
            let mut data = Vec::new();
            data.extend_from_slice(source_app);
            data.extend_from_slice(target_app);
            data.push(*msg_type);
            data.extend_from_slice(&(payload.len() as u16).to_le_bytes());
            data.extend_from_slice(payload);
            (IpcMessageType::AppMessage, data)
        }
        IpcMessage::Error { code, message } => {
            let mut payload = Vec::new();
            payload.push(*code);
            payload.push(message.len() as u8);
            payload.extend_from_slice(message.as_bytes());
            (IpcMessageType::Error, payload)
        }
    }
}

fn decode_ipc_message(msg_type: u8, payload: &[u8]) -> Result<IpcMessage, IpcError> {
    let msg_type = IpcMessageType::try_from(msg_type).map_err(|_| IpcError::InvalidMessageType)?;

    match msg_type {
        IpcMessageType::GazePoint => {
            if payload.len() < GazePointBinary::SIZE {
                return Err(IpcError::InsufficientData);
            }
            let binary: &[u8; 21] = payload[..21].try_into().unwrap();
            let point = GazePointBinary::from_bytes(binary);
            Ok(IpcMessage::GazePoint(point.to_gaze_point(0))) // Base timestamp would come from context
        }
        IpcMessageType::GazeBatch => {
            let points = decode_batch(payload).map_err(|_| IpcError::InsufficientData)?;
            Ok(IpcMessage::GazeBatch(points))
        }
        IpcMessageType::Heartbeat => Ok(IpcMessage::Heartbeat),
        IpcMessageType::HandshakeAck => {
            if payload.len() < 5 {
                return Err(IpcError::InsufficientData);
            }
            Ok(IpcMessage::HandshakeAck {
                accepted: payload[0] != 0,
                session_id: u32::from_le_bytes(payload[1..5].try_into().unwrap()),
            })
        }
        IpcMessageType::Status => {
            if payload.is_empty() {
                return Err(IpcError::InsufficientData);
            }
            let flags = payload[0];
            Ok(IpcMessage::Status {
                connected: (flags & 0x01) != 0,
                tracking: (flags & 0x02) != 0,
                calibrated: (flags & 0x04) != 0,
            })
        }
        _ => Err(IpcError::InvalidMessageType),
    }
}

fn now_micros() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_micros() as u64
}

fn generate_uuid() -> [u8; 16] {
    let uuid = uuid::Uuid::new_v4();
    *uuid.as_bytes()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ipc_frame_header_size() {
        assert_eq!(IpcFrameHeader::SIZE, 20);
    }

    #[test]
    fn test_frame_header_roundtrip() {
        let header = IpcFrameHeader::new(IpcMessageType::GazePoint, 21, 42);
        let bytes = header.to_bytes();
        let decoded = IpcFrameHeader::from_bytes(&bytes).unwrap();

        assert_eq!(decoded.msg_type, IpcMessageType::GazePoint as u8);
        assert_eq!(decoded.payload_len, 21);
        assert_eq!(decoded.sequence, 42);
    }

    #[test]
    fn test_encode_decode_heartbeat() {
        let (msg_type, payload) = encode_ipc_message(&IpcMessage::Heartbeat);
        assert_eq!(msg_type, IpcMessageType::Heartbeat);
        assert!(payload.is_empty());

        let decoded = decode_ipc_message(msg_type as u8, &payload).unwrap();
        assert!(matches!(decoded, IpcMessage::Heartbeat));
    }
}
