//! Protocol module for WIA Quantum communication
//!
//! This module implements the Phase 3 Communication Protocol for
//! connecting to quantum backends and managing job execution.

mod message;
mod transport;
mod client;
mod error;

pub use message::*;
pub use transport::*;
pub use client::*;
pub use error::*;

/// Protocol version
pub const PROTOCOL_VERSION: &str = "1.0.0";

/// Protocol identifier
pub const PROTOCOL_ID: &str = "wia-quantum";

/// Default WebSocket port
pub const DEFAULT_WS_PORT: u16 = 9500;
