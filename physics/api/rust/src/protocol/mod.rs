//! WIA Physics Protocol (WPP) Implementation
//!
//! This module implements the WIA Physics Protocol for real-time
//! physics data streaming and device control.

mod messages;
mod client;
mod handler;

pub use messages::*;
pub use client::*;
pub use handler::*;

/// Protocol version
pub const WPP_VERSION: &str = "1.0";

/// Default heartbeat interval in milliseconds
pub const DEFAULT_HEARTBEAT_INTERVAL: u64 = 30000;

/// Default command timeout in milliseconds
pub const DEFAULT_COMMAND_TIMEOUT: u64 = 30000;

/// Default WebSocket port
pub const DEFAULT_WS_PORT: u16 = 443;

/// Default TCP port
pub const DEFAULT_TCP_PORT: u16 = 5740;
