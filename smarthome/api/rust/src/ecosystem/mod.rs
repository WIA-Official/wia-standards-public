//! # WIA Ecosystem Integration
//!
//! Phase 4: Integration with WIA assistive technology ecosystem and external platforms.
//!
//! ## Supported Integrations
//!
//! ### WIA Ecosystem
//! - Eye Gaze: Gaze-based device selection and control
//! - BCI: Brain-computer interface commands
//! - AAC: Augmentative and alternative communication
//! - Smart Wheelchair: Location-based automation
//! - Exoskeleton: Mobility state awareness
//! - Haptic: Tactile feedback
//!
//! ### External Platforms
//! - Amazon Alexa
//! - Google Home
//! - Apple HomeKit

pub mod aac;
pub mod bci;
pub mod eye_gaze;
pub mod exoskeleton;
pub mod external;
pub mod haptic;
pub mod wheelchair;

mod command;

pub use aac::*;
pub use bci::*;
pub use command::*;
pub use eye_gaze::*;
pub use exoskeleton::*;
pub use external::*;
pub use haptic::*;
pub use wheelchair::*;

use crate::error::Result;
use crate::types::{DeviceId, ZoneId};
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use uuid::Uuid;

/// Unified command from any input source
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UnifiedCommand {
    pub id: Uuid,
    pub source: CommandSource,
    pub timestamp: DateTime<Utc>,
    pub target: CommandTarget,
    pub action: DeviceAction,
    pub context: Option<CommandContext>,
    pub priority: CommandPriority,
    pub confirmation: ConfirmationRequirement,
}

impl UnifiedCommand {
    pub fn new(source: CommandSource, target: CommandTarget, action: DeviceAction) -> Self {
        Self {
            id: Uuid::new_v4(),
            source,
            timestamp: Utc::now(),
            target,
            action,
            context: None,
            priority: CommandPriority::Normal,
            confirmation: ConfirmationRequirement::None,
        }
    }

    pub fn with_priority(mut self, priority: CommandPriority) -> Self {
        self.priority = priority;
        self
    }

    pub fn with_confirmation(mut self, confirmation: ConfirmationRequirement) -> Self {
        self.confirmation = confirmation;
        self
    }

    pub fn with_context(mut self, context: CommandContext) -> Self {
        self.context = Some(context);
        self
    }
}

/// Source of the command
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum CommandSource {
    /// Eye gaze input
    EyeGaze {
        gaze_point: GazePoint,
        dwell_ms: u32,
        pattern: GazePattern,
    },
    /// Brain-computer interface
    BCI {
        paradigm: BCIParadigm,
        confidence: f32,
        channel_data: Option<Vec<f32>>,
    },
    /// Augmentative and alternative communication
    AAC {
        symbol: Option<String>,
        voice_text: Option<String>,
        language: Language,
    },
    /// Smart wheelchair location trigger
    Wheelchair {
        zone: ZoneId,
        trigger: WheelchairTrigger,
        position: Option<Position2D>,
    },
    /// Exoskeleton state change
    Exoskeleton {
        state: MobilityState,
        previous_state: Option<MobilityState>,
    },
    /// Haptic gesture input
    Haptic { gesture: HapticGesture },
    /// Amazon Alexa
    Alexa {
        intent: String,
        slots: HashMap<String, String>,
    },
    /// Google Home
    GoogleHome {
        trait_name: String,
        params: serde_json::Value,
    },
    /// Apple HomeKit
    HomeKit {
        characteristic: String,
        value: serde_json::Value,
    },
    /// Manual/direct control
    Manual { user_id: Uuid },
}

/// Target of the command
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum CommandTarget {
    /// Single device
    Device(DeviceId),
    /// Zone (all devices in zone)
    Zone(ZoneId),
    /// Device group
    Group(String),
    /// Scene activation
    Scene(String),
    /// Broadcast to all
    All,
}

/// Device action to perform
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "action")]
pub enum DeviceAction {
    /// Power control
    Power { on: bool },
    /// Brightness control (0-100)
    Brightness { level: u8 },
    /// Color control
    Color { hue: u16, saturation: u8 },
    /// Temperature control
    Temperature { celsius: f32 },
    /// Thermostat mode
    ThermostatMode { mode: ThermostatMode },
    /// Lock control
    Lock { locked: bool },
    /// Door control
    Door { open: bool },
    /// Blind/curtain position (0-100)
    Blind { position: u8 },
    /// Fan speed (0-100)
    FanSpeed { speed: u8 },
    /// Volume control (0-100)
    Volume { level: u8 },
    /// Custom action
    Custom { name: String, params: serde_json::Value },
}

/// Thermostat operating mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ThermostatMode {
    Off,
    Heat,
    Cool,
    Auto,
    Fan,
}

/// Command priority levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum CommandPriority {
    /// Lowest - automation triggers
    Automation = 0,
    /// Normal user commands
    Normal = 1,
    /// Safety-related commands
    Safety = 2,
    /// Emergency override
    Emergency = 3,
}

/// Confirmation requirements
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ConfirmationRequirement {
    /// No confirmation needed
    None,
    /// Single confirmation
    Single,
    /// Double confirmation (security actions)
    Double,
    /// Biometric confirmation
    Biometric,
    /// Voice confirmation
    Voice { phrase: String },
}

/// Additional context for command execution
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandContext {
    pub user_id: Option<Uuid>,
    pub session_id: Option<Uuid>,
    pub accessibility_needs: Vec<AccessibilityNeed>,
    pub timeout_ms: Option<u32>,
    pub retry_count: u8,
}

/// Accessibility needs for command feedback
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AccessibilityNeed {
    VisualFeedback,
    AudioFeedback,
    HapticFeedback,
    LargeFeedback,
    SlowFeedback,
    SimpleFeedback,
}

/// 2D position
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Position2D {
    pub x: f32,
    pub y: f32,
}

/// Supported languages
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Language {
    Korean,
    English,
    Japanese,
    Chinese,
}

impl Default for Language {
    fn default() -> Self {
        Self::Korean
    }
}

/// Command result with feedback
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandResult {
    pub command_id: Uuid,
    pub success: bool,
    pub message: LocalizedMessage,
    pub feedback: Vec<FeedbackItem>,
    pub executed_at: DateTime<Utc>,
}

/// Localized message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LocalizedMessage {
    pub ko: String,
    pub en: String,
}

impl LocalizedMessage {
    pub fn new(ko: impl Into<String>, en: impl Into<String>) -> Self {
        Self {
            ko: ko.into(),
            en: en.into(),
        }
    }

    pub fn get(&self, lang: Language) -> &str {
        match lang {
            Language::Korean => &self.ko,
            Language::English => &self.en,
            _ => &self.en,
        }
    }
}

/// Feedback item for command result
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum FeedbackItem {
    Visual {
        text: String,
        icon: Option<String>,
        duration_ms: u32,
    },
    Audio {
        text: String,
        language: Language,
        speed: f32,
    },
    Haptic {
        pattern: HapticPattern,
    },
    Symbol {
        symbol: String,
    },
}

/// Ecosystem integration hub
pub struct EcosystemHub {
    eye_gaze: Option<EyeGazeAdapter>,
    bci: Option<BCIAdapter>,
    aac: Option<AACAdapter>,
    wheelchair: Option<WheelchairAdapter>,
    exoskeleton: Option<ExoskeletonAdapter>,
    haptic: Option<HapticAdapter>,
    command_processor: CommandProcessor,
}

impl EcosystemHub {
    pub fn new() -> Self {
        Self {
            eye_gaze: None,
            bci: None,
            aac: None,
            wheelchair: None,
            exoskeleton: None,
            haptic: None,
            command_processor: CommandProcessor::new(),
        }
    }

    pub fn with_eye_gaze(mut self, adapter: EyeGazeAdapter) -> Self {
        self.eye_gaze = Some(adapter);
        self
    }

    pub fn with_bci(mut self, adapter: BCIAdapter) -> Self {
        self.bci = Some(adapter);
        self
    }

    pub fn with_aac(mut self, adapter: AACAdapter) -> Self {
        self.aac = Some(adapter);
        self
    }

    pub fn with_wheelchair(mut self, adapter: WheelchairAdapter) -> Self {
        self.wheelchair = Some(adapter);
        self
    }

    pub fn with_exoskeleton(mut self, adapter: ExoskeletonAdapter) -> Self {
        self.exoskeleton = Some(adapter);
        self
    }

    pub fn with_haptic(mut self, adapter: HapticAdapter) -> Self {
        self.haptic = Some(adapter);
        self
    }

    /// Process a unified command
    pub async fn process_command(&self, command: UnifiedCommand) -> Result<CommandResult> {
        self.command_processor.process(command).await
    }

    /// Get feedback for a command result
    pub fn generate_feedback(
        &self,
        result: &CommandResult,
        needs: &[AccessibilityNeed],
    ) -> Vec<FeedbackItem> {
        let mut feedback = Vec::new();

        for need in needs {
            match need {
                AccessibilityNeed::VisualFeedback => {
                    feedback.push(FeedbackItem::Visual {
                        text: result.message.ko.clone(),
                        icon: if result.success {
                            Some("✅".to_string())
                        } else {
                            Some("❌".to_string())
                        },
                        duration_ms: 3000,
                    });
                }
                AccessibilityNeed::AudioFeedback => {
                    feedback.push(FeedbackItem::Audio {
                        text: result.message.ko.clone(),
                        language: Language::Korean,
                        speed: 1.0,
                    });
                }
                AccessibilityNeed::HapticFeedback => {
                    feedback.push(FeedbackItem::Haptic {
                        pattern: if result.success {
                            HapticPattern::Success
                        } else {
                            HapticPattern::Error
                        },
                    });
                }
                _ => {}
            }
        }

        feedback
    }
}

impl Default for EcosystemHub {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unified_command_creation() {
        let command = UnifiedCommand::new(
            CommandSource::Manual {
                user_id: Uuid::new_v4(),
            },
            CommandTarget::Device(Uuid::new_v4()),
            DeviceAction::Power { on: true },
        );

        assert_eq!(command.priority, CommandPriority::Normal);
    }

    #[test]
    fn test_command_priority_ordering() {
        assert!(CommandPriority::Emergency > CommandPriority::Safety);
        assert!(CommandPriority::Safety > CommandPriority::Normal);
        assert!(CommandPriority::Normal > CommandPriority::Automation);
    }

    #[test]
    fn test_localized_message() {
        let msg = LocalizedMessage::new("조명을 켰습니다", "Lights turned on");
        assert_eq!(msg.get(Language::Korean), "조명을 켰습니다");
        assert_eq!(msg.get(Language::English), "Lights turned on");
    }
}
