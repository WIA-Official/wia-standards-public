//! Platform Adapter Traits
//!
//! Defines traits for XR platform integration.

use async_trait::async_trait;

use crate::error::{DeviceResult, Result};
use crate::types::*;
use crate::core::AdaptationType;

/// Trait for XR platform adapters
#[async_trait]
pub trait XRPlatformAdapter: Send + Sync {
    /// Get the platform name
    fn platform_name(&self) -> &str;

    /// Get device capabilities
    async fn get_device_capabilities(&self) -> DeviceResult<XRDeviceCapabilities>;

    /// Initialize the platform
    async fn initialize(&mut self) -> Result<()>;

    /// Shutdown the platform
    async fn shutdown(&mut self) -> Result<()>;

    /// Check if platform is connected
    fn is_connected(&self) -> bool;

    /// Apply an accessibility adaptation
    async fn apply_adaptation(&mut self, adaptation: &AdaptationType) -> Result<()>;

    /// Remove an accessibility adaptation
    async fn remove_adaptation(&mut self, adaptation: &AdaptationType) -> Result<()>;

    /// Update platform settings
    async fn update_settings(&mut self, settings: PlatformSettings) -> Result<()>;
}

/// Platform-specific settings
#[derive(Debug, Clone, Default)]
pub struct PlatformSettings {
    /// Rendering quality (0.0 - 1.0)
    pub render_quality: f32,
    /// Fixed foveated rendering level
    pub foveated_rendering_level: Option<u32>,
    /// Refresh rate
    pub refresh_rate: Option<f32>,
    /// Passthrough mode
    pub passthrough_enabled: bool,
    /// Guardian/boundary enabled
    pub boundary_enabled: bool,
}

/// Caption display adapter
#[async_trait]
pub trait CaptionAdapter: Send + Sync {
    /// Show caption
    async fn show_caption(&self, text: &str, speaker: Option<&str>, duration_ms: u64) -> Result<()>;

    /// Hide current caption
    async fn hide_caption(&self) -> Result<()>;

    /// Update caption style
    async fn update_style(&mut self, style: &CaptionSettings) -> Result<()>;
}

/// Audio description adapter
#[async_trait]
pub trait AudioDescriptionAdapter: Send + Sync {
    /// Speak description
    async fn speak(&self, text: &str) -> Result<()>;

    /// Stop current description
    async fn stop(&self) -> Result<()>;

    /// Update voice settings
    async fn update_voice(&mut self, settings: &AudioDescriptionSettings) -> Result<()>;

    /// Check if currently speaking
    fn is_speaking(&self) -> bool;
}

/// Haptic feedback adapter
#[async_trait]
pub trait HapticAdapter: Send + Sync {
    /// Play haptic pattern
    async fn play_pattern(&self, pattern: HapticPattern) -> Result<()>;

    /// Play simple vibration
    async fn vibrate(&self, controller: Controller, intensity: f32, duration_ms: u64) -> Result<()>;

    /// Stop all haptics
    async fn stop_all(&self) -> Result<()>;

    /// Check if haptics are available
    fn is_available(&self) -> bool;
}

/// Haptic pattern definitions
#[derive(Debug, Clone)]
pub struct HapticPattern {
    pub name: String,
    pub segments: Vec<HapticSegment>,
}

#[derive(Debug, Clone)]
pub struct HapticSegment {
    pub controller: Controller,
    pub intensity: f32,
    pub duration_ms: u64,
    pub frequency: Option<f32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Controller {
    Left,
    Right,
    Both,
}

impl HapticPattern {
    pub fn notification() -> Self {
        Self {
            name: "notification".into(),
            segments: vec![
                HapticSegment {
                    controller: Controller::Both,
                    intensity: 0.5,
                    duration_ms: 100,
                    frequency: None,
                },
            ],
        }
    }

    pub fn success() -> Self {
        Self {
            name: "success".into(),
            segments: vec![
                HapticSegment {
                    controller: Controller::Both,
                    intensity: 0.3,
                    duration_ms: 50,
                    frequency: None,
                },
                HapticSegment {
                    controller: Controller::Both,
                    intensity: 0.6,
                    duration_ms: 100,
                    frequency: None,
                },
            ],
        }
    }

    pub fn error() -> Self {
        Self {
            name: "error".into(),
            segments: vec![
                HapticSegment {
                    controller: Controller::Both,
                    intensity: 0.8,
                    duration_ms: 200,
                    frequency: Some(150.0),
                },
            ],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_haptic_patterns() {
        let notification = HapticPattern::notification();
        assert_eq!(notification.name, "notification");
        assert!(!notification.segments.is_empty());

        let success = HapticPattern::success();
        assert_eq!(success.segments.len(), 2);
    }
}
