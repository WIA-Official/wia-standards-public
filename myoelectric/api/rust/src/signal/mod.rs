//! EMG Signal Processing Modules
//!
//! This module contains the core signal processing components:
//!
//! - `filter` - Digital filters for noise removal
//! - `rectify` - Signal rectification methods
//! - `envelope` - Envelope extraction algorithms
//! - `features` - Feature extraction for pattern recognition

pub mod envelope;
pub mod features;
pub mod filter;
pub mod rectify;
