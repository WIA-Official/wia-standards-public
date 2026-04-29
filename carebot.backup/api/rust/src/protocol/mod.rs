//! Communication protocol implementation for WIA CareBot
//!
//! This module provides the communication protocols for:
//! - CareBot WebSocket Protocol (CWP)
//! - Emergency Protocol
//! - Video Call Signaling (WebRTC)
//! - Integration with external services

pub mod message;
pub mod emergency;
pub mod websocket;
pub mod security;

pub use message::*;
pub use emergency::*;
pub use websocket::*;
pub use security::*;
