//! Additive Manufacturing SDK
//!
//! 弘익人間 (Benefit All Humanity)

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::AdditiveManufacturingClient;
pub use error::{Result, AdditiveManufacturingError};
pub use types::*;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
