//! Security module for WIA Auto
//!
//! This module provides authentication and cryptographic utilities:
//! - JWT token handling
//! - API key authentication
//! - Message signing (HMAC-SHA256)
//! - Data encryption (AES-256-GCM)

pub mod auth;
pub mod crypto;

pub use auth::*;
pub use crypto::*;
