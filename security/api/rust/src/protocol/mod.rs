//! WIA Security Communication Protocol
//!
//! Implements the WIA Security Communication Protocol for secure message exchange
//! between security components.
//!
//! ## Features
//!
//! - Type-safe message structures
//! - Request/Response/Event/Notification message types
//! - Message signing and verification
//! - Zero Trust access decision protocol
//! - TAXII 2.1 compatible threat intelligence sharing

pub mod authentication;
pub mod message;
pub mod siem;
pub mod streaming;
pub mod taxii;
pub mod transport;
pub mod zero_trust;

pub use authentication::*;
pub use message::*;
pub use siem::*;
pub use streaming::*;
pub use taxii::*;
pub use transport::*;
pub use zero_trust::*;

/// Protocol version
pub const PROTOCOL_VERSION: &str = "1.0.0";

/// Message schema URL
pub const MESSAGE_SCHEMA_URL: &str =
    "https://wia.live/schemas/security/protocol/message/v1.schema.json";
