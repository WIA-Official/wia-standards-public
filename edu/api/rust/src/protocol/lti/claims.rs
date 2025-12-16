//! LTI Accessibility Claims
//! 弘益人間 - Custom claims for accessibility profile transmission

use serde::{Deserialize, Serialize};
use crate::types::{
    DisabilityType, DisplayPreferences, ControlPreferences,
    ContentPreferences, AssessmentAccommodations, LearningStyle,
};

/// WIA Accessibility Claims for LTI 1.3
/// Custom claims namespace: https://wia.live/lti/accessibility
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AccessibilityClaims {
    /// Profile version for compatibility checking
    #[serde(default)]
    pub version: String,

    /// Learner's disability types (if disclosed)
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub disability_types: Vec<DisabilityType>,

    /// Display preferences summary
    #[serde(skip_serializing_if = "Option::is_none")]
    pub display: Option<DisplayClaimsSummary>,

    /// Control/Input preferences summary
    #[serde(skip_serializing_if = "Option::is_none")]
    pub control: Option<ControlClaimsSummary>,

    /// Content preferences summary
    #[serde(skip_serializing_if = "Option::is_none")]
    pub content: Option<ContentClaimsSummary>,

    /// Assessment accommodations summary
    #[serde(skip_serializing_if = "Option::is_none")]
    pub assessment: Option<AssessmentClaimsSummary>,

    /// Learning style preferences (UDL-based)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub learning_style: Option<LearningStyleSummary>,

    /// Full profile URL (for detailed retrieval)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub profile_url: Option<String>,

    /// Profile last modified timestamp
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_modified: Option<String>,
}

impl AccessibilityClaims {
    /// Claim namespace URI
    pub const NAMESPACE: &'static str = "https://wia.live/lti/accessibility";

    /// Create new accessibility claims
    pub fn new() -> Self {
        Self {
            version: "1.0.0".to_string(),
            ..Default::default()
        }
    }

    /// Create claims from full profile
    pub fn from_profile(
        disability_types: Vec<DisabilityType>,
        display: &DisplayPreferences,
        control: &ControlPreferences,
        content: &ContentPreferences,
        assessment: &AssessmentAccommodations,
        learning_style: &LearningStyle,
    ) -> Self {
        Self {
            version: "1.0.0".to_string(),
            disability_types,
            display: Some(DisplayClaimsSummary::from(display)),
            control: Some(ControlClaimsSummary::from(control)),
            content: Some(ContentClaimsSummary::from(content)),
            assessment: Some(AssessmentClaimsSummary::from(assessment)),
            learning_style: Some(LearningStyleSummary::from(learning_style)),
            profile_url: None,
            last_modified: None,
        }
    }

    /// Check if screen reader is needed
    pub fn needs_screen_reader(&self) -> bool {
        self.display
            .as_ref()
            .map(|d| d.screen_reader_enabled)
            .unwrap_or(false)
    }

    /// Check if captions are required
    pub fn needs_captions(&self) -> bool {
        self.content
            .as_ref()
            .map(|c| c.captions_required)
            .unwrap_or(false)
    }

    /// Check if extended time is needed
    pub fn needs_extended_time(&self) -> bool {
        self.assessment
            .as_ref()
            .map(|a| a.extended_time)
            .unwrap_or(false)
    }

    /// Get time multiplier for assessments
    pub fn time_multiplier(&self) -> f32 {
        self.assessment
            .as_ref()
            .map(|a| a.time_multiplier)
            .unwrap_or(1.0)
    }
}

/// Display preferences summary for LTI claims
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DisplayClaimsSummary {
    /// Screen reader enabled
    #[serde(default)]
    pub screen_reader_enabled: bool,
    /// Preferred speech rate (0.5-3.0)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub speech_rate: Option<f32>,
    /// Magnification enabled
    #[serde(default)]
    pub magnification_enabled: bool,
    /// Magnification level (1.0-10.0)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub magnification_level: Option<f32>,
    /// High contrast mode
    #[serde(default)]
    pub high_contrast: bool,
    /// Reduce motion preference
    #[serde(default)]
    pub reduce_motion: bool,
}

impl From<&DisplayPreferences> for DisplayClaimsSummary {
    fn from(prefs: &DisplayPreferences) -> Self {
        Self {
            screen_reader_enabled: prefs.screen_reader.enabled,
            speech_rate: if prefs.screen_reader.enabled {
                Some(prefs.screen_reader.speech_rate)
            } else {
                None
            },
            magnification_enabled: prefs.magnification.enabled,
            magnification_level: if prefs.magnification.enabled {
                Some(prefs.magnification.level)
            } else {
                None
            },
            high_contrast: prefs.color_settings.high_contrast,
            reduce_motion: prefs.reduce_motion,
        }
    }
}

/// Control preferences summary for LTI claims
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ControlClaimsSummary {
    /// Keyboard-only navigation required
    #[serde(default)]
    pub keyboard_only: bool,
    /// Switch access enabled
    #[serde(default)]
    pub switch_access: bool,
    /// Voice control enabled
    #[serde(default)]
    pub voice_control: bool,
    /// Eye gaze enabled
    #[serde(default)]
    pub eye_gaze: bool,
    /// Large click targets needed
    #[serde(default)]
    pub large_targets: bool,
    /// Minimum target size in pixels
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_target_size: Option<u32>,
}

impl From<&ControlPreferences> for ControlClaimsSummary {
    fn from(prefs: &ControlPreferences) -> Self {
        Self {
            keyboard_only: prefs.input_method.keyboard_only,
            switch_access: prefs.input_method.switch_access.enabled,
            voice_control: prefs.input_method.voice_control.enabled,
            eye_gaze: prefs.input_method.eye_gaze.enabled,
            large_targets: prefs.click_settings.large_targets,
            min_target_size: if prefs.click_settings.large_targets {
                Some(prefs.click_settings.min_target_size_px)
            } else {
                None
            },
        }
    }
}

/// Content preferences summary for LTI claims
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ContentClaimsSummary {
    /// Captions required for video
    #[serde(default)]
    pub captions_required: bool,
    /// Preferred caption language
    #[serde(skip_serializing_if = "Option::is_none")]
    pub caption_language: Option<String>,
    /// Audio description required
    #[serde(default)]
    pub audio_description_required: bool,
    /// Sign language preferred
    #[serde(default)]
    pub sign_language_preferred: bool,
    /// Preferred sign language type
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sign_language_type: Option<String>,
    /// Transcripts required
    #[serde(default)]
    pub transcripts_required: bool,
    /// Text-to-speech enabled
    #[serde(default)]
    pub text_to_speech: bool,
}

impl From<&ContentPreferences> for ContentClaimsSummary {
    fn from(prefs: &ContentPreferences) -> Self {
        Self {
            captions_required: prefs.captions.required,
            caption_language: prefs.captions.language.clone(),
            audio_description_required: prefs.audio_description.required,
            sign_language_preferred: prefs.sign_language.preferred,
            sign_language_type: prefs.sign_language.language
                .as_ref()
                .map(|t| format!("{:?}", t).to_lowercase()),
            transcripts_required: prefs.transcripts.required,
            text_to_speech: prefs.text_to_speech.enabled,
        }
    }
}

/// Assessment accommodations summary for LTI claims
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct AssessmentClaimsSummary {
    /// Extended time allowed
    #[serde(default)]
    pub extended_time: bool,
    /// Time multiplier (1.0-5.0)
    #[serde(default = "default_multiplier")]
    pub time_multiplier: f32,
    /// Breaks allowed
    #[serde(default)]
    pub breaks_allowed: bool,
    /// Separate/quiet room needed
    #[serde(default)]
    pub separate_room: bool,
    /// Calculator allowed
    #[serde(default)]
    pub calculator_allowed: bool,
    /// Reader allowed
    #[serde(default)]
    pub reader_allowed: bool,
    /// Scribe allowed
    #[serde(default)]
    pub scribe_allowed: bool,
}

fn default_multiplier() -> f32 {
    1.0
}

impl From<&AssessmentAccommodations> for AssessmentClaimsSummary {
    fn from(acc: &AssessmentAccommodations) -> Self {
        Self {
            extended_time: acc.timing.extended_time,
            time_multiplier: acc.timing.time_multiplier,
            breaks_allowed: acc.timing.breaks_allowed,
            separate_room: acc.environment.separate_room,
            calculator_allowed: acc.assistance.calculator,
            reader_allowed: acc.assistance.reader,
            scribe_allowed: acc.assistance.scribe,
        }
    }
}

/// Learning style summary for LTI claims (UDL-based)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct LearningStyleSummary {
    /// Multiple means of engagement preferences
    #[serde(default)]
    pub engagement: bool,
    /// Multiple means of representation preferences
    #[serde(default)]
    pub representation: bool,
    /// Multiple means of action/expression preferences
    #[serde(default)]
    pub action_expression: bool,
}

impl From<&LearningStyle> for LearningStyleSummary {
    fn from(style: &LearningStyle) -> Self {
        // Check if any UDL preferences are set
        Self {
            engagement: !style.engagement.interest_triggers.is_empty(),
            representation: !style.representation.preferred_modalities.is_empty()
                || style.representation.background_knowledge_support,
            action_expression: !style.action_expression.expression_preferences.is_empty(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create_accessibility_claims() {
        let claims = AccessibilityClaims::new();
        assert_eq!(claims.version, "1.0.0");
        assert!(!claims.needs_screen_reader());
        assert!(!claims.needs_captions());
        assert_eq!(claims.time_multiplier(), 1.0);
    }

    #[test]
    fn test_claims_with_screen_reader() {
        let mut claims = AccessibilityClaims::new();
        claims.display = Some(DisplayClaimsSummary {
            screen_reader_enabled: true,
            speech_rate: Some(1.5),
            ..Default::default()
        });

        assert!(claims.needs_screen_reader());
    }

    #[test]
    fn test_claims_with_extended_time() {
        let mut claims = AccessibilityClaims::new();
        claims.assessment = Some(AssessmentClaimsSummary {
            extended_time: true,
            time_multiplier: 1.5,
            ..Default::default()
        });

        assert!(claims.needs_extended_time());
        assert_eq!(claims.time_multiplier(), 1.5);
    }

    #[test]
    fn test_namespace() {
        assert_eq!(
            AccessibilityClaims::NAMESPACE,
            "https://wia.live/lti/accessibility"
        );
    }
}
