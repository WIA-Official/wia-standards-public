//! # WIA-MENTAL-013-youth-mental-health: Youth Mental Health
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
