//! # WIA-LANG-009-translation-quality: Translation Quality Assessment
//! 弘益人間 (홍익인간) - Benefit All Humanity

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::Client;
pub use error::{Error, Result};
pub use types::*;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
pub const PHILOSOPHY: &str = "弘益人間 (홍익인간) - Benefit All Humanity";
