//! Aging In Place SDK
//!
//! 弘益人間 (Benefit All Humanity)

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::AgingInPlaceClient;
pub use error::{Result, AgingInPlaceError};
pub use types::*;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
