//! XR Accessibility Engine
//!
//! Core engine for managing XR accessibility features and adaptations.

use async_trait::async_trait;
use std::sync::Arc;
use tokio::sync::RwLock;

use crate::error::{Result, XRAccessibilityError, AdaptationError};
use crate::types::*;

/// XR Accessibility Engine configuration
#[derive(Debug, Clone)]
pub struct XREngineConfig {
    /// Maximum number of simultaneous adaptations
    pub max_adaptations: usize,
    /// Enable real-time adaptation updates
    pub real_time_updates: bool,
    /// Performance budget (0.0 - 1.0)
    pub performance_budget: f32,
    /// Enable WIA integrations
    pub enable_wia_integrations: bool,
    /// Fallback behavior when adaptations fail
    pub fallback_behavior: FallbackBehavior,
}

impl Default for XREngineConfig {
    fn default() -> Self {
        Self {
            max_adaptations: 10,
            real_time_updates: true,
            performance_budget: 0.8,
            enable_wia_integrations: true,
            fallback_behavior: FallbackBehavior::GracefulDegrade,
        }
    }
}

/// Fallback behavior when adaptations fail
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FallbackBehavior {
    /// Silently disable failed adaptation
    SilentDisable,
    /// Gracefully degrade to simpler adaptation
    GracefulDegrade,
    /// Notify user of failure
    NotifyUser,
    /// Halt experience until resolved
    HaltExperience,
}

/// Main XR Accessibility Engine
pub struct XRAccessibilityEngine {
    config: XREngineConfig,
    current_profile: Arc<RwLock<Option<XRAccessibilityProfile>>>,
    device_capabilities: Arc<RwLock<Option<XRDeviceCapabilities>>>,
    environment_config: Arc<RwLock<Option<XREnvironmentConfig>>>,
    active_adaptations: Arc<RwLock<Vec<ActiveAdaptation>>>,
    event_handlers: Arc<RwLock<Vec<Box<dyn XREventHandler>>>>,
}

impl XRAccessibilityEngine {
    /// Create a new XR Accessibility Engine with default configuration
    pub fn new() -> Self {
        Self::with_config(XREngineConfig::default())
    }

    /// Create a new XR Accessibility Engine with custom configuration
    pub fn with_config(config: XREngineConfig) -> Self {
        Self {
            config,
            current_profile: Arc::new(RwLock::new(None)),
            device_capabilities: Arc::new(RwLock::new(None)),
            environment_config: Arc::new(RwLock::new(None)),
            active_adaptations: Arc::new(RwLock::new(Vec::new())),
            event_handlers: Arc::new(RwLock::new(Vec::new())),
        }
    }

    /// Load an accessibility profile
    pub async fn load_profile(&self, profile: XRAccessibilityProfile) -> Result<()> {
        // Validate profile
        self.validate_profile(&profile)?;

        // Store profile
        let mut current = self.current_profile.write().await;
        *current = Some(profile.clone());

        // Apply adaptations based on profile
        self.apply_profile_adaptations(&profile).await?;

        // Notify handlers
        self.notify_handlers(XREvent::ProfileLoaded {
            profile_id: profile.profile_id.clone(),
        })
        .await;

        Ok(())
    }

    /// Set device capabilities
    pub async fn set_device_capabilities(&self, capabilities: XRDeviceCapabilities) -> Result<()> {
        let mut device = self.device_capabilities.write().await;
        *device = Some(capabilities.clone());

        // Recalculate adaptations based on new device
        if let Some(profile) = self.current_profile.read().await.as_ref() {
            self.recalculate_adaptations(profile, &capabilities).await?;
        }

        self.notify_handlers(XREvent::DeviceChanged {
            device_id: capabilities.device_id.clone(),
        })
        .await;

        Ok(())
    }

    /// Set environment configuration
    pub async fn set_environment(&self, env_config: XREnvironmentConfig) -> Result<()> {
        // Validate environment compatibility
        if let Some(profile) = self.current_profile.read().await.as_ref() {
            self.check_environment_compatibility(profile, &env_config)?;
        }

        let mut env = self.environment_config.write().await;
        *env = Some(env_config.clone());

        self.notify_handlers(XREvent::EnvironmentChanged {
            environment_id: env_config.environment_id.clone(),
        })
        .await;

        Ok(())
    }

    /// Get current accessibility state
    pub async fn get_accessibility_state(&self) -> AccessibilityState {
        let profile = self.current_profile.read().await;
        let device = self.device_capabilities.read().await;
        let env = self.environment_config.read().await;
        let adaptations = self.active_adaptations.read().await;

        AccessibilityState {
            profile_active: profile.is_some(),
            device_connected: device.is_some(),
            environment_loaded: env.is_some(),
            active_adaptation_count: adaptations.len(),
            adaptations: adaptations.clone(),
        }
    }

    /// Apply a specific adaptation
    pub async fn apply_adaptation(&self, adaptation: AdaptationType) -> Result<()> {
        let mut adaptations = self.active_adaptations.write().await;

        // Check if we've hit the max adaptations limit
        if adaptations.len() >= self.config.max_adaptations {
            return Err(XRAccessibilityError::Adaptation(
                AdaptationError::ResourceUnavailable(
                    "Maximum adaptations reached".into()
                )
            ));
        }

        // Check device support
        if let Some(device) = self.device_capabilities.read().await.as_ref() {
            self.verify_adaptation_support(&adaptation, device)?;
        }

        let active = ActiveAdaptation {
            adaptation_type: adaptation.clone(),
            status: AdaptationStatus::Active,
            applied_at: chrono::Utc::now(),
            performance_impact: self.estimate_performance_impact(&adaptation),
        };

        adaptations.push(active);

        self.notify_handlers(XREvent::AdaptationApplied { adaptation }).await;

        Ok(())
    }

    /// Remove an adaptation
    pub async fn remove_adaptation(&self, adaptation: &AdaptationType) -> Result<()> {
        let mut adaptations = self.active_adaptations.write().await;

        if let Some(pos) = adaptations.iter().position(|a| &a.adaptation_type == adaptation) {
            adaptations.remove(pos);
            self.notify_handlers(XREvent::AdaptationRemoved {
                adaptation: adaptation.clone(),
            })
            .await;
            Ok(())
        } else {
            Err(XRAccessibilityError::Adaptation(
                AdaptationError::NotApplicable("Adaptation not active".into())
            ))
        }
    }

    /// Register an event handler
    pub async fn register_handler(&self, handler: Box<dyn XREventHandler>) {
        let mut handlers = self.event_handlers.write().await;
        handlers.push(handler);
    }

    /// Get comfort settings
    pub async fn get_comfort_settings(&self) -> Option<ComfortSettings> {
        let profile = self.current_profile.read().await;
        profile.as_ref().map(|p| p.comfort.clone())
    }

    /// Update motion comfort settings
    pub async fn update_motion_settings(&self, motion: MotionComfort) -> Result<()> {
        let mut profile = self.current_profile.write().await;

        if let Some(ref mut p) = *profile {
            p.comfort.motion = motion;
            self.notify_handlers(XREvent::ComfortSettingsChanged {
                category: "motion".into(),
            })
            .await;
            Ok(())
        } else {
            Err(XRAccessibilityError::Configuration(
                "No profile loaded".into()
            ))
        }
    }

    /// Activate safe space
    pub async fn activate_safe_space(&self) -> Result<()> {
        let profile = self.current_profile.read().await;

        if let Some(ref p) = *profile {
            if p.comfort.cognitive.safe_space_enabled {
                self.notify_handlers(XREvent::SafeSpaceActivated).await;
                Ok(())
            } else {
                Err(XRAccessibilityError::Configuration(
                    "Safe space not enabled in profile".into()
                ))
            }
        } else {
            Err(XRAccessibilityError::Configuration(
                "No profile loaded".into()
            ))
        }
    }

    /// Trigger rest reminder
    pub async fn trigger_rest_reminder(&self) {
        self.notify_handlers(XREvent::RestReminderTriggered).await;
    }

    // Private helper methods

    fn validate_profile(&self, profile: &XRAccessibilityProfile) -> Result<()> {
        if profile.profile_id.is_empty() {
            return Err(XRAccessibilityError::Validation(
                "Profile ID cannot be empty".into()
            ));
        }

        // Validate version format
        let version_parts: Vec<&str> = profile.version.split('.').collect();
        if version_parts.len() != 3 {
            return Err(XRAccessibilityError::Validation(
                "Invalid version format, expected semver".into()
            ));
        }

        Ok(())
    }

    async fn apply_profile_adaptations(&self, profile: &XRAccessibilityProfile) -> Result<()> {
        let mut adaptations = self.active_adaptations.write().await;
        adaptations.clear();

        // Apply visual adaptations
        if let Some(ref visual) = profile.disabilities.visual {
            if visual.level == VisualLevel::TotallyBlind || visual.level == VisualLevel::LegallyBlind {
                adaptations.push(ActiveAdaptation {
                    adaptation_type: AdaptationType::ScreenReader,
                    status: AdaptationStatus::Active,
                    applied_at: chrono::Utc::now(),
                    performance_impact: 5.0,
                });
                adaptations.push(ActiveAdaptation {
                    adaptation_type: AdaptationType::AudioDescription,
                    status: AdaptationStatus::Active,
                    applied_at: chrono::Utc::now(),
                    performance_impact: 8.0,
                });
            }
            if visual.color_vision != ColorVision::Normal {
                adaptations.push(ActiveAdaptation {
                    adaptation_type: AdaptationType::ColorCorrection,
                    status: AdaptationStatus::Active,
                    applied_at: chrono::Utc::now(),
                    performance_impact: 3.0,
                });
            }
        }

        // Apply auditory adaptations
        if let Some(ref auditory) = profile.disabilities.auditory {
            if auditory.level == AuditoryLevel::Deaf || auditory.level == AuditoryLevel::Deafblind {
                adaptations.push(ActiveAdaptation {
                    adaptation_type: AdaptationType::Captions,
                    status: AdaptationStatus::Active,
                    applied_at: chrono::Utc::now(),
                    performance_impact: 2.0,
                });
                adaptations.push(ActiveAdaptation {
                    adaptation_type: AdaptationType::VisualSoundIndicators,
                    status: AdaptationStatus::Active,
                    applied_at: chrono::Utc::now(),
                    performance_impact: 4.0,
                });
            }
        }

        // Apply motor adaptations
        if let Some(ref motor) = profile.disabilities.motor {
            if motor.level != MotorLevel::None {
                adaptations.push(ActiveAdaptation {
                    adaptation_type: AdaptationType::AdaptiveControls,
                    status: AdaptationStatus::Active,
                    applied_at: chrono::Utc::now(),
                    performance_impact: 1.0,
                });
            }
        }

        // Apply cognitive adaptations
        if let Some(ref cognitive) = profile.disabilities.cognitive {
            if cognitive.level != CognitiveLevel::None {
                adaptations.push(ActiveAdaptation {
                    adaptation_type: AdaptationType::SimplifiedUI,
                    status: AdaptationStatus::Active,
                    applied_at: chrono::Utc::now(),
                    performance_impact: 2.0,
                });
                adaptations.push(ActiveAdaptation {
                    adaptation_type: AdaptationType::ReducedStimuli,
                    status: AdaptationStatus::Active,
                    applied_at: chrono::Utc::now(),
                    performance_impact: 5.0,
                });
            }
        }

        Ok(())
    }

    async fn recalculate_adaptations(
        &self,
        _profile: &XRAccessibilityProfile,
        capabilities: &XRDeviceCapabilities,
    ) -> Result<()> {
        let mut adaptations = self.active_adaptations.write().await;

        // Remove adaptations not supported by device
        adaptations.retain(|a| {
            self.verify_adaptation_support(&a.adaptation_type, capabilities).is_ok()
        });

        Ok(())
    }

    fn check_environment_compatibility(
        &self,
        profile: &XRAccessibilityProfile,
        env: &XREnvironmentConfig,
    ) -> Result<()> {
        // Check for flashing content
        if let Some(ref visual) = profile.disabilities.visual {
            if visual.photosensitivity != PhotosensitivityLevel::None && env.sensory.has_flashing {
                return Err(XRAccessibilityError::Validation(
                    "Environment contains flashing content incompatible with profile".into()
                ));
            }
        }

        Ok(())
    }

    fn verify_adaptation_support(
        &self,
        adaptation: &AdaptationType,
        device: &XRDeviceCapabilities,
    ) -> Result<()> {
        match adaptation {
            AdaptationType::EyeTracking if !device.input.has_eye_tracking => {
                Err(XRAccessibilityError::Adaptation(
                    AdaptationError::RequiresFeature("Eye tracking".into())
                ))
            }
            AdaptationType::HandTracking if !device.input.has_hand_tracking => {
                Err(XRAccessibilityError::Adaptation(
                    AdaptationError::RequiresFeature("Hand tracking".into())
                ))
            }
            AdaptationType::VoiceControl if !device.input.voice_control => {
                Err(XRAccessibilityError::Adaptation(
                    AdaptationError::RequiresFeature("Voice control".into())
                ))
            }
            AdaptationType::HapticFeedback if !device.haptics.controller_haptics => {
                Err(XRAccessibilityError::Adaptation(
                    AdaptationError::RequiresFeature("Haptic feedback".into())
                ))
            }
            _ => Ok(()),
        }
    }

    fn estimate_performance_impact(&self, adaptation: &AdaptationType) -> f32 {
        match adaptation {
            AdaptationType::Captions => 2.0,
            AdaptationType::AudioDescription => 8.0,
            AdaptationType::SignLanguageAvatar => 15.0,
            AdaptationType::ScreenReader => 5.0,
            AdaptationType::ColorCorrection => 3.0,
            AdaptationType::HighContrast => 2.0,
            AdaptationType::Magnification => 4.0,
            AdaptationType::EyeTracking => 10.0,
            AdaptationType::HandTracking => 12.0,
            AdaptationType::VoiceControl => 6.0,
            AdaptationType::AdaptiveControls => 1.0,
            AdaptationType::HapticFeedback => 3.0,
            AdaptationType::SimplifiedUI => 2.0,
            AdaptationType::ReducedStimuli => 5.0,
            AdaptationType::VisualSoundIndicators => 4.0,
            AdaptationType::MotionReduction => 3.0,
            AdaptationType::ComfortVignette => 2.0,
        }
    }

    async fn notify_handlers(&self, event: XREvent) {
        let handlers = self.event_handlers.read().await;
        for handler in handlers.iter() {
            handler.on_event(&event).await;
        }
    }
}

impl Default for XRAccessibilityEngine {
    fn default() -> Self {
        Self::new()
    }
}

/// Types of adaptations that can be applied
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub enum AdaptationType {
    Captions,
    AudioDescription,
    SignLanguageAvatar,
    ScreenReader,
    ColorCorrection,
    HighContrast,
    Magnification,
    EyeTracking,
    HandTracking,
    VoiceControl,
    AdaptiveControls,
    HapticFeedback,
    SimplifiedUI,
    ReducedStimuli,
    VisualSoundIndicators,
    MotionReduction,
    ComfortVignette,
}

/// Status of an active adaptation
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AdaptationStatus {
    Active,
    Paused,
    Degraded,
    Failed,
}

/// Information about an active adaptation
#[derive(Debug, Clone)]
pub struct ActiveAdaptation {
    pub adaptation_type: AdaptationType,
    pub status: AdaptationStatus,
    pub applied_at: chrono::DateTime<chrono::Utc>,
    pub performance_impact: f32,
}

/// Current accessibility state
#[derive(Debug, Clone)]
pub struct AccessibilityState {
    pub profile_active: bool,
    pub device_connected: bool,
    pub environment_loaded: bool,
    pub active_adaptation_count: usize,
    pub adaptations: Vec<ActiveAdaptation>,
}

/// XR accessibility events
#[derive(Debug, Clone)]
pub enum XREvent {
    ProfileLoaded { profile_id: String },
    ProfileUnloaded,
    DeviceChanged { device_id: String },
    EnvironmentChanged { environment_id: String },
    AdaptationApplied { adaptation: AdaptationType },
    AdaptationRemoved { adaptation: AdaptationType },
    AdaptationFailed { adaptation: AdaptationType, reason: String },
    ComfortSettingsChanged { category: String },
    SafeSpaceActivated,
    SafeSpaceDeactivated,
    RestReminderTriggered,
    SessionLimitWarning { minutes_remaining: u32 },
    SessionLimitReached,
    WIAIntegrationConnected { system: String },
    WIAIntegrationDisconnected { system: String },
}

/// Event handler trait for XR accessibility events
#[async_trait]
pub trait XREventHandler: Send + Sync {
    async fn on_event(&self, event: &XREvent);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_profile() -> XRAccessibilityProfile {
        XRAccessibilityProfile::default()
    }

    #[tokio::test]
    async fn test_engine_creation() {
        let engine = XRAccessibilityEngine::new();
        let state = engine.get_accessibility_state().await;
        assert!(!state.profile_active);
        assert!(!state.device_connected);
    }

    #[tokio::test]
    async fn test_load_profile() {
        let engine = XRAccessibilityEngine::new();
        let profile = create_test_profile();

        let result = engine.load_profile(profile).await;
        assert!(result.is_ok());

        let state = engine.get_accessibility_state().await;
        assert!(state.profile_active);
    }

    #[tokio::test]
    async fn test_invalid_profile() {
        let engine = XRAccessibilityEngine::new();
        let mut profile = create_test_profile();
        profile.profile_id = String::new();

        let result = engine.load_profile(profile).await;
        assert!(result.is_err());
    }
}
