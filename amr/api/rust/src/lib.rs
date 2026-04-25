//! Amr SDK
//!
//! 弘익人間 (Benefit All Humanity)

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::AmrClient;
pub use error::{Result, AmrError};
pub use types::*;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
