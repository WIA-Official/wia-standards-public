//! WIA Ocean Underwater Communication Standard SDK
//!
//! 弘益人間 (Benefit All Humanity)
//!
//! This SDK provides tools for underwater acoustic communication,
//! data transmission, and network protocols.

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::UnderwaterCommunicationClient;
pub use error::{Result, UnderwaterCommunicationError};
pub use types::*;

/// SDK version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
