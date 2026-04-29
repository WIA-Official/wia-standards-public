//! # WIA-MENTAL-006-ptsd-support: PTSD Support Platform
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
