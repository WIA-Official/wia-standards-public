//! WIA Material Protocol Module
//!
//! This module implements the WIA Material Protocol v1.0.0 for standardized
//! communication between material science data clients and servers.
//!
//! ## Features
//!
//! - Message envelope with protocol versioning
//! - Request/response message types
//! - Query and filter support
//! - Streaming subscriptions
//! - JSON and MessagePack serialization

mod message;
mod filter;

pub use message::*;
pub use filter::*;

/// Protocol identifier
pub const PROTOCOL_ID: &str = "wia-material";

/// Current protocol version
pub const PROTOCOL_VERSION: &str = "1.0.0";
