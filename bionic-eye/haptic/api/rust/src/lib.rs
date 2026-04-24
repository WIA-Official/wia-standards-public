//! # WIA Haptic Standard - Rust Low-Level Driver Library
//!
//! This library provides low-level haptic driver implementations for embedded
//! systems and hardware interfaces.
//!
//! ## Features
//!
//! - `std` - Standard library support (default)
//! - `embedded` - Embedded systems support (no_std compatible)
//! - `pwm` - PWM-based haptic driver
//! - `i2c` - I2C haptic controller driver
//! - `spi` - SPI haptic controller driver
//!
//! ## Example
//!
//! ```rust,ignore
//! use wia_haptic::{HapticDriver, PwmHapticDriver, HapticPrimitive};
//!
//! let mut driver = PwmHapticDriver::new(pin, 1000);
//! driver.init()?;
//!
//! let primitive = HapticPrimitive::tick();
//! driver.play(&primitive)?;
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

pub mod types;
pub mod patterns;
pub mod driver;
pub mod drivers;
pub mod spatial;

pub use types::*;
pub use patterns::*;
pub use driver::*;
pub use drivers::*;
pub use spatial::*;
