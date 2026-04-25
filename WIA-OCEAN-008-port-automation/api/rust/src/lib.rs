//! Port Automation SDK
//!
//! 弘益人間 (Benefit All Humanity)

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::PortAutomationClient;
pub use error::{Result, PortAutomationError};
pub use types::*;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
