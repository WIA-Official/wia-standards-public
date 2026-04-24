//! Air Quality SDK
//!
//! 弘익人間 (Benefit All Humanity)

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::AirQualityClient;
pub use error::{Result, AirQualityError};
pub use types::*;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
