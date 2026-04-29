//! # WIA Art Authentication Standard - Rust SDK
//!
//! 弘益人間 (홍익人間) - Benefit All Humanity
//!
//! Comprehensive SDK for art authentication, provenance tracking, and digital art verification.

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::Client;
pub use error::{Error, Result};
pub use types::*;
pub use validators::*;
pub use utils::*;

pub const VERSION: &str = "1.0.0";
pub const PHILOSOPHY: &str = "弘益人間 (홍익인간) - Benefit All Humanity";
