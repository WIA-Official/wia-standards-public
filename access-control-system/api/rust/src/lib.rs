//! Access Control System SDK
//!
//! 弘익人間 (Benefit All Humanity)

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::AccessControlSystemClient;
pub use error::{Result, AccessControlSystemError};
pub use types::*;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");
