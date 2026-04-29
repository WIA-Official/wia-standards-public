//! # WIA-CONTACT-009 Cosmic Communication Standard
//!
//! 우주 통신 및 외계 접촉 프로토콜 표준
//!
//! ## 철학: 홍익인간 (弘益人間)
//! "널리 인간을 이롭게 하라" - 우주 생명체와의 소통을 통한 인류 발전
//!
//! ## Features
//! - SETI signal processing and analysis
//! - Universal message encoding/decoding
//! - Cosmic coordinate systems
//! - Signal authenticity verification
//!
//! ## Example
//! ```rust,no_run
//! use wia_contact_009::{CosmicClient, SignalType};
//!
//! #[tokio::main]
//! async fn main() -> Result<(), Box<dyn std::error::Error>> {
//!     let client = CosmicClient::new("your-api-key".to_string());
//!
//!     // Receive cosmic signal
//!     let signal = client.receive_signal().await?;
//!     println!("Received signal: {:?}", signal);
//!
//!     Ok(())
//! }
//! ```

pub mod types;
pub mod client;
pub mod error;
pub mod validators;
pub mod utils;

pub use client::CosmicClient;
pub use error::{Error, Result};
pub use types::*;

/// Library version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        assert!(!VERSION.is_empty());
    }
}
