//! Device Adapters Module
//! 弘益人間 - Gaming for Everyone

pub mod eye_tracker;
pub mod xbox;

pub use eye_tracker::*;
pub use xbox::*;

use super::device::{ConnectedDevice, DeviceId};
use super::event::InputEvent;
use crate::error::Result;
use async_trait::async_trait;

/// Device adapter trait for input devices
#[async_trait]
pub trait DeviceAdapter: Send + Sync + std::fmt::Debug {
    /// Get device info
    fn device_info(&self) -> &ConnectedDevice;

    /// Check if device is connected
    fn is_connected(&self) -> bool;

    /// Poll for events (non-blocking)
    fn poll(&mut self) -> Vec<InputEvent>;

    /// Start event stream
    async fn start(&mut self) -> Result<()>;

    /// Stop event stream
    async fn stop(&mut self) -> Result<()>;

    /// Send rumble/haptic feedback
    async fn rumble(&self, left_motor: f32, right_motor: f32, duration_ms: u32) -> Result<()>;

    /// Set LED color (if supported)
    async fn set_led(&self, r: u8, g: u8, b: u8) -> Result<()>;
}
