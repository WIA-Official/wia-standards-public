//! # WIA Security SDK
//!
//! Rust implementation of the WIA Security Standard.
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

pub mod types;
pub mod validator;
pub mod builder;
pub mod converter;
pub mod client;

pub use types::*;
pub use validator::{validate_event, ValidationResult};
pub use builder::*;
pub use converter::*;
pub use client::{SecurityClient, SecurityClientConfig};

/// SDK version
pub const VERSION: &str = "1.0.0";

/// Schema URL
pub const SCHEMA_URL: &str = "https://wia.live/security/v1/schema.json";
