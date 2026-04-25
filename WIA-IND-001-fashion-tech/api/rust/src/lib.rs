//! # WIA-IND-001: Fashion Technology Standard
//!
//! 弘益人間 (홍익인간) - Benefit All Humanity through fashion innovation
//!
//! This SDK provides interfaces for virtual try-on, style recommendations,
//! size matching, and fashion analytics.

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::FashionTechClient;
pub use error::{FashionTechError, Result};
pub use types::*;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
pub const PHILOSOPHY: &str = "弘益人間 (홍익인간) - Benefit All Humanity";
