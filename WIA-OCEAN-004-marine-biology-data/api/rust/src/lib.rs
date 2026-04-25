//! WIA Ocean Marine Biology Data Standard SDK
//!
//! 弘益人間 (Benefit All Humanity)

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::MarineBiologyClient;
pub use error::{Result, MarineBiologyError};
pub use types::*;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
