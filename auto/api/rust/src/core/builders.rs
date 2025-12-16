//! Builder patterns for WIA Auto types

use uuid::Uuid;

use crate::error::{Error, Result};
use crate::types::*;

/// Builder for PassengerProfile
#[derive(Debug, Default)]
pub struct PassengerProfileBuilder {
    name: Option<String>,
    disabilities: Vec<DisabilityType>,
    service_animal: Option<ServiceAnimal>,
    mobility_aid: Option<MobilityAid>,
    communication: Option<CommunicationPrefs>,
    assistance: Option<AssistanceNeeds>,
    preferences: Option<UserPreferences>,
}

impl PassengerProfileBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self::default()
    }

    /// Set passenger name
    pub fn name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Add a disability type
    pub fn add_disability(mut self, disability: DisabilityType) -> Self {
        self.disabilities.push(disability);
        self
    }

    /// Set disabilities
    pub fn disabilities(mut self, disabilities: Vec<DisabilityType>) -> Self {
        self.disabilities = disabilities;
        self
    }

    /// Set service animal
    pub fn service_animal(mut self, animal_type: &str, size: AnimalSize) -> Self {
        self.service_animal = Some(ServiceAnimal {
            has_animal: true,
            animal_type: Some(animal_type.to_string()),
            animal_size: Some(size),
        });
        self
    }

    /// Set mobility aid
    pub fn mobility_aid(
        mut self,
        aid_type: MobilityAidType,
        requires_ramp: bool,
        requires_securement: bool,
    ) -> Self {
        self.mobility_aid = Some(MobilityAid {
            aid_type,
            dimensions: None,
            requires_ramp,
            requires_lift: false,
            requires_securement,
        });
        self
    }

    /// Set mobility aid with dimensions
    pub fn mobility_aid_with_dimensions(
        mut self,
        aid_type: MobilityAidType,
        dimensions: Dimensions,
        requires_ramp: bool,
        requires_lift: bool,
        requires_securement: bool,
    ) -> Self {
        self.mobility_aid = Some(MobilityAid {
            aid_type,
            dimensions: Some(dimensions),
            requires_ramp,
            requires_lift,
            requires_securement,
        });
        self
    }

    /// Set preferred language
    pub fn preferred_language(mut self, lang: &str) -> Self {
        let comm = self.communication.get_or_insert_with(|| CommunicationPrefs {
            preferred_language: None,
            preferred_modalities: vec![],
            aac_device: false,
            sign_language: None,
        });
        comm.preferred_language = Some(lang.to_string());
        self
    }

    /// Set preferred modalities
    pub fn preferred_modalities(mut self, modalities: Vec<InteractionModality>) -> Self {
        let comm = self.communication.get_or_insert_with(|| CommunicationPrefs {
            preferred_language: None,
            preferred_modalities: vec![],
            aac_device: false,
            sign_language: None,
        });
        comm.preferred_modalities = modalities;
        self
    }

    /// Set AAC device usage
    pub fn uses_aac_device(mut self, uses: bool) -> Self {
        let comm = self.communication.get_or_insert_with(|| CommunicationPrefs {
            preferred_language: None,
            preferred_modalities: vec![],
            aac_device: false,
            sign_language: None,
        });
        comm.aac_device = uses;
        self
    }

    /// Set sign language preference
    pub fn sign_language(mut self, language: &str) -> Self {
        let comm = self.communication.get_or_insert_with(|| CommunicationPrefs {
            preferred_language: None,
            preferred_modalities: vec![],
            aac_device: false,
            sign_language: None,
        });
        comm.sign_language = Some(language.to_string());
        self
    }

    /// Set assistance needs
    pub fn needs_boarding_help(mut self, needs: bool) -> Self {
        let assist = self.assistance.get_or_insert_with(|| AssistanceNeeds {
            needs_boarding_help: false,
            needs_securement_help: false,
            companion_present: false,
            emergency_contact: None,
        });
        assist.needs_boarding_help = needs;
        self
    }

    /// Set securement help needs
    pub fn needs_securement_help(mut self, needs: bool) -> Self {
        let assist = self.assistance.get_or_insert_with(|| AssistanceNeeds {
            needs_boarding_help: false,
            needs_securement_help: false,
            companion_present: false,
            emergency_contact: None,
        });
        assist.needs_securement_help = needs;
        self
    }

    /// Set emergency contact
    pub fn emergency_contact(mut self, name: &str, phone: &str, relationship: Option<&str>) -> Self {
        let assist = self.assistance.get_or_insert_with(|| AssistanceNeeds {
            needs_boarding_help: false,
            needs_securement_help: false,
            companion_present: false,
            emergency_contact: None,
        });
        assist.emergency_contact = Some(EmergencyContact {
            name: name.to_string(),
            phone: phone.to_string(),
            relationship: relationship.map(String::from),
        });
        self
    }

    /// Set audio volume preference
    pub fn audio_volume(mut self, volume: u8) -> Self {
        let prefs = self.preferences.get_or_insert_with(|| UserPreferences {
            audio_volume: None,
            screen_brightness: None,
            high_contrast: false,
            large_text: false,
            haptic_intensity: None,
            minimize_walking: false,
        });
        prefs.audio_volume = Some(volume.min(100));
        self
    }

    /// Set high contrast preference
    pub fn high_contrast(mut self, enabled: bool) -> Self {
        let prefs = self.preferences.get_or_insert_with(|| UserPreferences {
            audio_volume: None,
            screen_brightness: None,
            high_contrast: false,
            large_text: false,
            haptic_intensity: None,
            minimize_walking: false,
        });
        prefs.high_contrast = enabled;
        self
    }

    /// Set large text preference
    pub fn large_text(mut self, enabled: bool) -> Self {
        let prefs = self.preferences.get_or_insert_with(|| UserPreferences {
            audio_volume: None,
            screen_brightness: None,
            high_contrast: false,
            large_text: false,
            haptic_intensity: None,
            minimize_walking: false,
        });
        prefs.large_text = enabled;
        self
    }

    /// Set minimize walking preference
    pub fn minimize_walking(mut self, enabled: bool) -> Self {
        let prefs = self.preferences.get_or_insert_with(|| UserPreferences {
            audio_volume: None,
            screen_brightness: None,
            high_contrast: false,
            large_text: false,
            haptic_intensity: None,
            minimize_walking: false,
        });
        prefs.minimize_walking = enabled;
        self
    }

    /// Build the profile
    pub fn build(self) -> Result<PassengerProfile> {
        if self.disabilities.is_empty() {
            return Err(Error::validation("At least one disability type is required"));
        }

        Ok(PassengerProfile {
            profile_id: Uuid::new_v4(),
            version: "1.0.0".to_string(),
            passenger: PassengerInfo {
                name: self.name,
                disabilities: self.disabilities,
                service_animal: self.service_animal,
            },
            mobility_aid: self.mobility_aid,
            communication: self.communication,
            assistance: self.assistance,
            preferences: self.preferences,
        })
    }
}

impl PassengerProfile {
    /// Create a new builder
    pub fn builder() -> PassengerProfileBuilder {
        PassengerProfileBuilder::new()
    }
}

/// Builder for HmiConfig
#[derive(Debug, Default)]
pub struct HmiConfigBuilder {
    visual: Option<VisualConfig>,
    audio: Option<AudioConfig>,
    haptic: Option<HapticConfig>,
    physical: Option<PhysicalConfig>,
}

impl HmiConfigBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self::default()
    }

    /// Configure for visual impairment (blind)
    pub fn for_blind(mut self) -> Self {
        self.visual = Some(VisualConfig {
            enabled: false,
            brightness: 0,
            contrast: ContrastMode::Normal,
            text_size: TextSize::Medium,
            color_scheme: ColorScheme::Default,
            animations: false,
        });
        self.audio = Some(AudioConfig {
            enabled: true,
            volume: 80,
            tts_voice: None,
            tts_speed: TtsSpeed::Normal,
            chimes_enabled: true,
            speech_recognition: true,
            language: "en".to_string(),
        });
        self.haptic = Some(HapticConfig {
            enabled: true,
            intensity: 70,
            patterns: Some(HapticPatterns {
                notification: true,
                navigation: true,
                warning: true,
                confirmation: true,
            }),
        });
        self.physical = Some(PhysicalConfig {
            braille_output: true,
            button_audio_feedback: true,
            button_haptic_feedback: true,
        });
        self
    }

    /// Configure for low vision
    pub fn for_low_vision(mut self) -> Self {
        self.visual = Some(VisualConfig {
            enabled: true,
            brightness: 100,
            contrast: ContrastMode::High,
            text_size: TextSize::ExtraLarge,
            color_scheme: ColorScheme::HighContrast,
            animations: false,
        });
        self.audio = Some(AudioConfig {
            enabled: true,
            volume: 70,
            tts_voice: None,
            tts_speed: TtsSpeed::Normal,
            chimes_enabled: true,
            speech_recognition: true,
            language: "en".to_string(),
        });
        self
    }

    /// Configure for hearing impairment (deaf)
    pub fn for_deaf(mut self) -> Self {
        self.visual = Some(VisualConfig {
            enabled: true,
            brightness: 80,
            contrast: ContrastMode::High,
            text_size: TextSize::Large,
            color_scheme: ColorScheme::Default,
            animations: true,
        });
        self.audio = Some(AudioConfig {
            enabled: false,
            volume: 0,
            tts_voice: None,
            tts_speed: TtsSpeed::Normal,
            chimes_enabled: false,
            speech_recognition: false,
            language: "en".to_string(),
        });
        self.haptic = Some(HapticConfig {
            enabled: true,
            intensity: 90,
            patterns: Some(HapticPatterns {
                notification: true,
                navigation: true,
                warning: true,
                confirmation: true,
            }),
        });
        self
    }

    /// Set visual config
    pub fn visual(mut self, config: VisualConfig) -> Self {
        self.visual = Some(config);
        self
    }

    /// Set audio config
    pub fn audio(mut self, config: AudioConfig) -> Self {
        self.audio = Some(config);
        self
    }

    /// Set haptic config
    pub fn haptic(mut self, config: HapticConfig) -> Self {
        self.haptic = Some(config);
        self
    }

    /// Set physical config
    pub fn physical(mut self, config: PhysicalConfig) -> Self {
        self.physical = Some(config);
        self
    }

    /// Build the config
    pub fn build(self) -> HmiConfig {
        HmiConfig {
            config_id: Uuid::new_v4(),
            version: "1.0.0".to_string(),
            active: true,
            visual: self.visual,
            audio: self.audio,
            haptic: self.haptic,
            physical: self.physical,
        }
    }
}

impl HmiConfig {
    /// Create a new builder
    pub fn builder() -> HmiConfigBuilder {
        HmiConfigBuilder::new()
    }

    /// Create a default config for blind users
    pub fn for_blind() -> Self {
        HmiConfigBuilder::new().for_blind().build()
    }

    /// Create a default config for low vision users
    pub fn for_low_vision() -> Self {
        HmiConfigBuilder::new().for_low_vision().build()
    }

    /// Create a default config for deaf users
    pub fn for_deaf() -> Self {
        HmiConfigBuilder::new().for_deaf().build()
    }
}

/// Builder for AccessibilityRequirements
#[derive(Debug, Default)]
pub struct AccessibilityRequirementsBuilder {
    wheelchair_accessible: bool,
    ramp_required: bool,
    lift_required: bool,
    service_animal_space: bool,
    companion_space: u8,
    preferred_modalities: Vec<InteractionModality>,
}

impl AccessibilityRequirementsBuilder {
    /// Create a new builder
    pub fn new() -> Self {
        Self::default()
    }

    /// Require wheelchair accessibility
    pub fn wheelchair_accessible(mut self) -> Self {
        self.wheelchair_accessible = true;
        self
    }

    /// Require ramp
    pub fn requires_ramp(mut self) -> Self {
        self.ramp_required = true;
        self
    }

    /// Require lift
    pub fn requires_lift(mut self) -> Self {
        self.lift_required = true;
        self
    }

    /// Require service animal space
    pub fn service_animal_space(mut self) -> Self {
        self.service_animal_space = true;
        self
    }

    /// Set companion space
    pub fn companion_space(mut self, count: u8) -> Self {
        self.companion_space = count;
        self
    }

    /// Add preferred modality
    pub fn add_modality(mut self, modality: InteractionModality) -> Self {
        self.preferred_modalities.push(modality);
        self
    }

    /// Set preferred modalities
    pub fn modalities(mut self, modalities: Vec<InteractionModality>) -> Self {
        self.preferred_modalities = modalities;
        self
    }

    /// Build from a passenger profile
    pub fn from_profile(profile: &PassengerProfile) -> Self {
        let mut builder = Self::new();

        // Check mobility aid
        if let Some(ref aid) = profile.mobility_aid {
            if matches!(
                aid.aid_type,
                MobilityAidType::ManualWheelchair
                    | MobilityAidType::PowerWheelchair
                    | MobilityAidType::Scooter
            ) {
                builder.wheelchair_accessible = true;
            }
            builder.ramp_required = aid.requires_ramp;
            builder.lift_required = aid.requires_lift;
        }

        // Check service animal
        if let Some(ref animal) = profile.passenger.service_animal {
            builder.service_animal_space = animal.has_animal;
        }

        // Check communication preferences
        if let Some(ref comm) = profile.communication {
            builder.preferred_modalities = comm.preferred_modalities.clone();
        }

        builder
    }

    /// Build the requirements
    pub fn build(self) -> AccessibilityRequirements {
        AccessibilityRequirements {
            wheelchair_accessible: self.wheelchair_accessible,
            ramp_required: self.ramp_required,
            lift_required: self.lift_required,
            service_animal_space: self.service_animal_space,
            companion_space: self.companion_space,
            preferred_modalities: self.preferred_modalities,
        }
    }
}

impl AccessibilityRequirements {
    /// Create a new builder
    pub fn builder() -> AccessibilityRequirementsBuilder {
        AccessibilityRequirementsBuilder::new()
    }

    /// Create requirements from a profile
    pub fn from_profile(profile: &PassengerProfile) -> Self {
        AccessibilityRequirementsBuilder::from_profile(profile).build()
    }
}
