//! Organ Shortage SDK
//!
//! 弘益人間 (Benefit All Humanity)

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::OrganShortageClient;
pub use error::{Result, OrganShortageError};
pub use types::*;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
