//! # WIA Security SDK
//!
//! Rust implementation of the WIA Security Standard.
//!
//! ## Features
//!
//! - **Core Types**: Security event types (alerts, threats, vulnerabilities, incidents)
//! - **Validation**: JSON schema validation for events
//! - **Builders**: Fluent API for constructing events
//! - **Protocol** (optional): Communication protocol implementation
//!   - Message format and serialization
//!   - Zero Trust access decisions
//!   - SIEM integration
//!   - TAXII 2.1 compatible threat intel sharing
//!   - WebSocket streaming
//!
//! ## Example
//!
//! ```rust
//! use wia_security::{AlertBuilder, Source, SourceType, validate_event};
//!
//! let event = AlertBuilder::new()
//!     .source(Source::new(SourceType::Siem, "Splunk"))
//!     .alert_id("ALERT-001")
//!     .title("Suspicious Activity")
//!     .category("malware")
//!     .status("new")
//!     .priority("high")
//!     .severity(7.0)
//!     .build()
//!     .unwrap();
//!
//! let result = validate_event(&event);
//! assert!(result.is_valid());
//! ```
//!
//! ## Protocol Example
//!
//! ```rust,ignore
//! use wia_security::protocol::{
//!     Component, ComponentType, ProtocolMessage, MessageType,
//!     RequestData, RequestType, create_request,
//! };
//!
//! let sender = Component::new("scanner-001", ComponentType::Scanner);
//! let receiver = Component::new("siem-central", ComponentType::Siem);
//! let request = RequestData::new(RequestType::ScanStart);
//!
//! let msg = create_request(sender, receiver, request);
//! println!("{}", msg.to_json().unwrap());
//! ```

pub mod types;
pub mod validator;
pub mod builder;
pub mod converter;
pub mod client;
pub mod protocol;

pub use types::*;
pub use validator::{validate_event, ValidationResult};
pub use builder::*;
pub use converter::*;
pub use client::{SecurityClient, SecurityClientConfig};

/// SDK version
pub const VERSION: &str = "1.0.0";

/// Schema URL
pub const SCHEMA_URL: &str = "https://wia.live/security/v1/schema.json";
