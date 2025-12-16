//! Adaptation Engine
//!
//! Manages and applies accessibility adaptations to XR experiences.

use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

use crate::error::{AdaptationError, AdaptationResult};
use crate::types::*;
use super::xr::{AdaptationType, AdaptationStatus};

/// Adaptation priority levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum AdaptationPriority {
    Critical = 4,
    High = 3,
    Medium = 2,
    Low = 1,
}

/// Configuration for an adaptation
#[derive(Debug, Clone)]
pub struct AdaptationConfig {
    pub adaptation_type: AdaptationType,
    pub priority: AdaptationPriority,
    pub auto_enable: bool,
    pub fallback_adaptations: Vec<AdaptationType>,
    pub performance_budget: f32,
}

impl AdaptationConfig {
    pub fn new(adaptation_type: AdaptationType) -> Self {
        let (priority, auto_enable) = match adaptation_type {
            AdaptationType::ScreenReader => (AdaptationPriority::Critical, true),
            AdaptationType::Captions => (AdaptationPriority::High, true),
            AdaptationType::AudioDescription => (AdaptationPriority::High, true),
            AdaptationType::ColorCorrection => (AdaptationPriority::Medium, true),
            AdaptationType::HighContrast => (AdaptationPriority::Medium, false),
            AdaptationType::Magnification => (AdaptationPriority::Medium, false),
            AdaptationType::VoiceControl => (AdaptationPriority::High, false),
            AdaptationType::EyeTracking => (AdaptationPriority::Medium, false),
            AdaptationType::HandTracking => (AdaptationPriority::Medium, false),
            AdaptationType::SimplifiedUI => (AdaptationPriority::Medium, false),
            AdaptationType::ReducedStimuli => (AdaptationPriority::High, false),
            AdaptationType::MotionReduction => (AdaptationPriority::High, true),
            AdaptationType::ComfortVignette => (AdaptationPriority::Medium, true),
            _ => (AdaptationPriority::Low, false),
        };

        Self {
            adaptation_type,
            priority,
            auto_enable,
            fallback_adaptations: Vec::new(),
            performance_budget: 10.0,
        }
    }

    pub fn with_priority(mut self, priority: AdaptationPriority) -> Self {
        self.priority = priority;
        self
    }

    pub fn with_fallback(mut self, fallback: AdaptationType) -> Self {
        self.fallback_adaptations.push(fallback);
        self
    }
}

/// Trait for adaptation implementations
#[async_trait]
pub trait Adaptation: Send + Sync {
    /// Get the adaptation type
    fn adaptation_type(&self) -> AdaptationType;

    /// Check if the adaptation can be applied
    async fn can_apply(&self, profile: &XRAccessibilityProfile, device: &XRDeviceCapabilities) -> bool;

    /// Apply the adaptation
    async fn apply(&self, context: &mut AdaptationContext) -> AdaptationResult<()>;

    /// Remove the adaptation
    async fn remove(&self, context: &mut AdaptationContext) -> AdaptationResult<()>;

    /// Get the estimated performance impact (0-100%)
    fn performance_impact(&self) -> f32;
}

/// Context for adaptation operations
pub struct AdaptationContext {
    pub profile: XRAccessibilityProfile,
    pub device: XRDeviceCapabilities,
    pub environment: Option<XREnvironmentConfig>,
    pub active_adaptations: HashMap<AdaptationType, AdaptationStatus>,
    pub performance_usage: f32,
}

impl AdaptationContext {
    pub fn new(profile: XRAccessibilityProfile, device: XRDeviceCapabilities) -> Self {
        Self {
            profile,
            device,
            environment: None,
            active_adaptations: HashMap::new(),
            performance_usage: 0.0,
        }
    }

    pub fn with_environment(mut self, environment: XREnvironmentConfig) -> Self {
        self.environment = Some(environment);
        self
    }
}

/// Caption adaptation implementation
pub struct CaptionAdaptation {
    settings: CaptionSettings,
}

impl CaptionAdaptation {
    pub fn new(settings: CaptionSettings) -> Self {
        Self { settings }
    }

    pub fn from_profile(profile: &XRAccessibilityProfile) -> Self {
        Self {
            settings: profile.output.captions.clone(),
        }
    }
}

#[async_trait]
impl Adaptation for CaptionAdaptation {
    fn adaptation_type(&self) -> AdaptationType {
        AdaptationType::Captions
    }

    async fn can_apply(&self, _profile: &XRAccessibilityProfile, _device: &XRDeviceCapabilities) -> bool {
        true
    }

    async fn apply(&self, context: &mut AdaptationContext) -> AdaptationResult<()> {
        context.active_adaptations.insert(AdaptationType::Captions, AdaptationStatus::Active);
        context.performance_usage += self.performance_impact();
        Ok(())
    }

    async fn remove(&self, context: &mut AdaptationContext) -> AdaptationResult<()> {
        context.active_adaptations.remove(&AdaptationType::Captions);
        context.performance_usage -= self.performance_impact();
        Ok(())
    }

    fn performance_impact(&self) -> f32 {
        2.0
    }
}

/// Audio description adaptation
pub struct AudioDescriptionAdaptation {
    settings: AudioDescriptionSettings,
}

impl AudioDescriptionAdaptation {
    pub fn new(settings: AudioDescriptionSettings) -> Self {
        Self { settings }
    }
}

#[async_trait]
impl Adaptation for AudioDescriptionAdaptation {
    fn adaptation_type(&self) -> AdaptationType {
        AdaptationType::AudioDescription
    }

    async fn can_apply(&self, _profile: &XRAccessibilityProfile, device: &XRDeviceCapabilities) -> bool {
        device.audio.has_speakers
    }

    async fn apply(&self, context: &mut AdaptationContext) -> AdaptationResult<()> {
        if !context.device.audio.has_speakers {
            return Err(AdaptationError::RequiresFeature("Speakers".into()));
        }
        context.active_adaptations.insert(AdaptationType::AudioDescription, AdaptationStatus::Active);
        context.performance_usage += self.performance_impact();
        Ok(())
    }

    async fn remove(&self, context: &mut AdaptationContext) -> AdaptationResult<()> {
        context.active_adaptations.remove(&AdaptationType::AudioDescription);
        context.performance_usage -= self.performance_impact();
        Ok(())
    }

    fn performance_impact(&self) -> f32 {
        8.0
    }
}

/// Adaptation manager for coordinating multiple adaptations
pub struct AdaptationManager {
    adaptations: Arc<RwLock<HashMap<AdaptationType, Arc<dyn Adaptation>>>>,
    configs: Arc<RwLock<HashMap<AdaptationType, AdaptationConfig>>>,
    performance_budget: f32,
}

impl AdaptationManager {
    pub fn new(performance_budget: f32) -> Self {
        Self {
            adaptations: Arc::new(RwLock::new(HashMap::new())),
            configs: Arc::new(RwLock::new(HashMap::new())),
            performance_budget,
        }
    }

    /// Register an adaptation
    pub async fn register(&self, adaptation: Arc<dyn Adaptation>, config: AdaptationConfig) {
        let mut adaptations = self.adaptations.write().await;
        let mut configs = self.configs.write().await;

        let adaptation_type = adaptation.adaptation_type();
        adaptations.insert(adaptation_type.clone(), adaptation);
        configs.insert(adaptation_type, config);
    }

    /// Get recommended adaptations for a profile
    pub async fn get_recommendations(
        &self,
        profile: &XRAccessibilityProfile,
        device: &XRDeviceCapabilities,
    ) -> Vec<AdaptationType> {
        let mut recommendations = Vec::new();
        let adaptations = self.adaptations.read().await;
        let configs = self.configs.read().await;

        let mut sorted_configs: Vec<_> = configs.iter().collect();
        sorted_configs.sort_by(|a, b| b.1.priority.cmp(&a.1.priority));

        let mut total_impact = 0.0;

        for (adaptation_type, config) in sorted_configs {
            if let Some(adaptation) = adaptations.get(adaptation_type) {
                if adaptation.can_apply(profile, device).await {
                    let impact = adaptation.performance_impact();
                    if total_impact + impact <= self.performance_budget {
                        if config.auto_enable {
                            recommendations.push(adaptation_type.clone());
                            total_impact += impact;
                        }
                    }
                }
            }
        }

        recommendations
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::ProfileBuilder;

    fn create_test_device() -> XRDeviceCapabilities {
        XRDeviceCapabilities {
            device_id: "test-device".into(),
            device_name: "Test Device".into(),
            manufacturer: "Test".into(),
            model: "Test Model".into(),
            firmware_version: "1.0.0".into(),
            display: DisplayCapabilities {
                display_type: DisplayType::Lcd,
                resolution_per_eye: Resolution { width: 1920, height: 1080 },
                refresh_rates: vec![90.0],
                field_of_view: FieldOfViewCaps { horizontal: 100.0, vertical: 90.0 },
                supports_passthrough: true,
                passthrough_color: true,
                supports_foveated_rendering: true,
            },
            audio: AudioCapabilities {
                has_speakers: true,
                speaker_type: Some(SpeakerType::Integrated),
                has_microphone: true,
                supports_spatial_audio: true,
                supports_bone_conduction: false,
            },
            input: InputCapabilities {
                controller_type: ControllerType::SixDof,
                has_eye_tracking: true,
                has_hand_tracking: true,
                has_face_tracking: true,
                has_body_tracking: false,
                voice_control: true,
                controller_features: None,
            },
            haptics: HapticCapabilities {
                controller_haptics: true,
                haptic_fidelity: HapticFidelity::Hd,
                supports_external_haptics: false,
            },
            built_in_accessibility: BuiltInAccessibility {
                screen_reader: true,
                magnification: true,
                color_correction: true,
                caption_support: true,
                voice_control: true,
                one_handed_mode: true,
                seated_mode: true,
            },
            wia_compatibility: WIACompatibility {
                exoskeleton_compatible: true,
                bionic_eye_compatible: true,
                voice_sign_compatible: true,
                protocol_version: "1.0.0".into(),
            },
        }
    }

    #[tokio::test]
    async fn test_caption_adaptation() {
        let profile = ProfileBuilder::new("test").build();
        let device = create_test_device();
        let mut context = AdaptationContext::new(profile.clone(), device.clone());

        let adaptation = CaptionAdaptation::from_profile(&profile);
        assert!(adaptation.can_apply(&profile, &device).await);

        adaptation.apply(&mut context).await.unwrap();
        assert!(context.active_adaptations.contains_key(&AdaptationType::Captions));
    }

    #[tokio::test]
    async fn test_adaptation_manager() {
        let manager = AdaptationManager::new(50.0);

        let profile = ProfileBuilder::new("test").build();
        let device = create_test_device();

        let caption = Arc::new(CaptionAdaptation::from_profile(&profile));
        manager.register(caption, AdaptationConfig::new(AdaptationType::Captions)).await;

        let recommendations = manager.get_recommendations(&profile, &device).await;
        assert!(!recommendations.is_empty());
    }
}
