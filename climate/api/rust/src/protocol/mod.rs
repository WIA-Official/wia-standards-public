//! # WIA Climate Protocol Module
//!
//! This module implements the WIA Climate communication protocol (Phase 3).
//! It provides message types, builders, and handlers for climate data communication.

mod message;
mod message_types;
mod builder;
mod handler;

pub use message::*;
pub use message_types::*;
pub use builder::*;
pub use handler::*;

/// Protocol identifier
pub const PROTOCOL_ID: &str = "wia-climate";

/// Protocol version
pub const PROTOCOL_VERSION: &str = "1.0.0";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_protocol_constants() {
        assert_eq!(PROTOCOL_ID, "wia-climate");
        assert_eq!(PROTOCOL_VERSION, "1.0.0");
    }
}
