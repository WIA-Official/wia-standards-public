//! # WIA Bio Transport Module
//!
//! Transport layer implementations for the WIA Biotechnology Standard.
//!
//! This module provides:
//! - Abstract transport interface
//! - WebSocket transport implementation
//! - Mock transport for testing
//! - Transport factory for easy instantiation
//!
//! ## Transport Architecture
//!
//! The transport layer abstracts the underlying communication mechanism,
//! allowing the protocol layer to work with any transport type.
//!
//! ```text
//! ┌─────────────────────────────────────┐
//! │         Application Layer           │
//! ├─────────────────────────────────────┤
//! │          Protocol Layer             │
//! │    (Messages, Handlers, Parser)     │
//! ├─────────────────────────────────────┤
//! │         Transport Interface         │
//! │          (ITransport trait)         │
//! ├──────────┬──────────┬───────────────┤
//! │ WebSocket│   Mock   │   Future...   │
//! │ Transport│ Transport│ (USB, BLE)    │
//! └──────────┴──────────┴───────────────┘
//! ```
//!
//! ## Example
//!
//! ```rust,no_run
//! use wia_bio::transport::*;
//!
//! #[tokio::main]
//! async fn main() {
//!     // Create a WebSocket transport
//!     let transport = TransportFactory::create(TransportType::WebSocket);
//!
//!     // Or use mock for testing
//!     let mock = TransportFactory::create(TransportType::Mock);
//! }
//! ```

mod base;
mod websocket;
mod mock;
mod factory;

pub use base::*;
pub use websocket::*;
pub use mock::*;
pub use factory::*;
