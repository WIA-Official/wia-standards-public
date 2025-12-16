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

pub mod message;
pub mod transport;
pub mod authentication;
pub mod zero_trust;
pub mod siem;
pub mod taxii;
pub mod streaming;

pub use message::*;
pub use transport::*;
pub use authentication::*;
pub use zero_trust::*;
pub use siem::*;
pub use taxii::*;
pub use streaming::*;

/// Protocol version
pub const PROTOCOL_VERSION: &str = "1.0.0";

/// Message schema URL
pub const MESSAGE_SCHEMA_URL: &str = "https://wia.live/schemas/security/protocol/message/v1.schema.json";
