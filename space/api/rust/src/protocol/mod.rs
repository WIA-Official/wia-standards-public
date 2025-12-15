//! WIA Space Protocol (WSP) implementation
//!
//! This module provides the communication protocol for WIA Space Standard.
//!
//! ## Message Types
//!
//! - Connection management: `connect`, `connect_ack`, `disconnect`, `ping`, `pong`
//! - Mission data: `telemetry`, `command`, `command_ack`, `mission_update`
//! - Specification: `spec_update`, `validation_request`, `validation_result`
//! - Errors: `error`, `warning`

mod message;
mod builder;
mod handler;
mod error;

pub use message::*;
pub use builder::*;
pub use handler::*;
pub use error::*;

/// Protocol identifier
pub const PROTOCOL_ID: &str = "wia-space";

/// Protocol version
pub const PROTOCOL_VERSION: &str = "1.0.0";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_protocol_constants() {
        assert_eq!(PROTOCOL_ID, "wia-space");
        assert_eq!(PROTOCOL_VERSION, "1.0.0");
    }
}
