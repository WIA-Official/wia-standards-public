//! Transport layer for WIA Space Protocol
//!
//! This module provides transport adapters for different communication channels:
//!
//! - `MockTransport`: In-memory transport for testing
//! - `WebSocketTransport`: WebSocket-based transport (TODO)
//! - `TcpTransport`: TCP-based transport (TODO)
//!
//! ## Latency Simulation
//!
//! The module includes latency simulation for deep space communication:
//!
//! ```rust,ignore
//! use wia_space::transport::*;
//!
//! let config = LatencyConfig::mars_opposition();
//! let simulator = LatencySimulator::new(config);
//! ```

mod base;
mod mock;
mod latency;

pub use base::*;
pub use mock::*;
pub use latency::*;

/// Default WebSocket port
pub const DEFAULT_WS_PORT: u16 = 8080;

/// Default TCP port
pub const DEFAULT_TCP_PORT: u16 = 9090;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_ports() {
        assert_eq!(DEFAULT_WS_PORT, 8080);
        assert_eq!(DEFAULT_TCP_PORT, 9090);
    }
}
