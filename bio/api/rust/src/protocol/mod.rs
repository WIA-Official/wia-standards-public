//! # WIA Bio Protocol Module
//!
//! Communication protocol implementation for the WIA Biotechnology Standard.
//!
//! This module provides:
//! - Message types for protocol communication
//! - Message building and parsing
//! - Protocol handling logic
//!
//! ## Protocol Overview
//!
//! The WIA Bio protocol uses JSON-based messages for all communication.
//! Messages include:
//! - Connection management (connect, disconnect)
//! - Data streaming (sequence, experiment, structure)
//! - Commands and acknowledgments
//! - Error reporting
//!
//! ## Example
//!
//! ```rust
//! use wia_bio::protocol::*;
//!
//! // Build a connect message
//! let msg = MessageBuilder::connect("client-123", "My Bio App")
//!     .with_capability(ProtocolCapability::SequenceStreaming)
//!     .build();
//!
//! // Serialize to JSON
//! let json = serde_json::to_string(&msg).unwrap();
//! ```

mod message;
mod builder;
mod handler;

pub use message::*;
pub use builder::*;
pub use handler::*;

/// Protocol version
pub const PROTOCOL_VERSION: &str = "1.0.0";

/// Protocol identifier
pub const PROTOCOL_ID: &str = "wia-bio";
