//! WIA Consciousness Research Standard - Rust SDK
//! 弘益人間 (홍익인간) - Benefit All Humanity

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::Client;
pub use error::{Error, Result};
pub use types::*;

pub const VERSION: &str = "1.0.0";
pub const PHILOSOPHY: &str = "弘益人間 - Benefit All Humanity";
