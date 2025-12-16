//! # AAC (Augmentative and Alternative Communication) Smart Home Integration
//!
//! Enables symbol-based and voice command control of smart home devices.

use super::*;
use crate::error::{Result, SmartHomeError};
use crate::types::DeviceId;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// AAC symbol representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AACSymbol {
    pub id: String,
    pub category: SymbolCategory,
    pub label: LocalizedLabel,
    pub image_path: Option<String>,
    pub emoji: Option<String>,
}

/// Symbol categories for smart home
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SymbolCategory {
    Lighting,
    Climate,
    Security,
    Entertainment,
    Appliance,
    Emergency,
    Navigation,
    Modifier,
}

/// Localized label
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LocalizedLabel {
    pub ko: String,
    pub en: String,
}

impl LocalizedLabel {
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

/// Voice command structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceCommand {
    pub text: String,
    pub language: Language,
    pub confidence: f32,
    pub alternatives: Vec<String>,
}

/// AAC adapter for smart home control
#[derive(Debug)]
pub struct AACAdapter {
    config: AACConfig,
    symbol_mapping: HashMap<String, SymbolMapping>,
    voice_grammar: VoiceGrammar,
    sentence_builder: SentenceBuilder,
}

/// AAC configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AACConfig {
    /// Default language
    pub default_language: Language,
    /// Voice recognition confidence threshold
    pub voice_confidence_threshold: f32,
    /// Enable sentence building
    pub sentence_building_enabled: bool,
    /// Auto-confirm after symbol selection
    pub auto_confirm: bool,
    /// Confirmation timeout (ms)
    pub confirmation_timeout_ms: u32,
    /// TTS speech rate (0.5-2.0)
    pub tts_rate: f32,
}

impl Default for AACConfig {
    fn default() -> Self {
        Self {
            default_language: Language::Korean,
            voice_confidence_threshold: 0.6,
            sentence_building_enabled: true,
            auto_confirm: false,
            confirmation_timeout_ms: 5000,
            tts_rate: 1.0,
        }
    }
}

/// Mapping from symbol to device action
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SymbolMapping {
    pub symbol: AACSymbol,
    pub device_id: Option<DeviceId>,
    pub action: Option<DeviceAction>,
    pub is_modifier: bool,
}

/// Voice grammar rules
#[derive(Debug, Default)]
struct VoiceGrammar {
    custom_commands: HashMap<String, CustomCommand>,
}

/// Parsed command from voice input
#[derive(Debug, Clone)]
struct ParsedCommand {
    location: Option<String>,
    device_type: Option<String>,
    action: Option<String>,
    value: Option<String>,
}

/// Custom voice command
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CustomCommand {
    pub phrases: Vec<String>,
    pub device_id: DeviceId,
    pub action: DeviceAction,
}

/// Sentence builder for symbol sequences
#[derive(Debug, Default)]
struct SentenceBuilder {
    current_sequence: Vec<String>,
    location: Option<String>,
    device: Option<String>,
    action: Option<String>,
}

impl AACAdapter {
    pub fn new() -> Self {
        let mut adapter = Self {
            config: AACConfig::default(),
            symbol_mapping: HashMap::new(),
            voice_grammar: VoiceGrammar::default(),
            sentence_builder: SentenceBuilder::default(),
        };

        adapter.setup_default_symbols();
        adapter.setup_default_grammar();

        adapter
    }

    pub fn with_config(mut self, config: AACConfig) -> Self {
        self.config = config;
        self
    }

    /// Setup default smart home symbols
    fn setup_default_symbols(&mut self) {
        // Lighting symbols
        self.add_symbol_mapping(SymbolMapping {
            symbol: AACSymbol {
                id: "light_on".to_string(),
                category: SymbolCategory::Lighting,
                label: LocalizedLabel::new("불 켜기", "Light On"),
                image_path: None,
                emoji: Some("💡".to_string()),
            },
            device_id: None,
            action: Some(DeviceAction::Power { on: true }),
            is_modifier: false,
        });

        self.add_symbol_mapping(SymbolMapping {
            symbol: AACSymbol {
                id: "light_off".to_string(),
                category: SymbolCategory::Lighting,
                label: LocalizedLabel::new("불 끄기", "Light Off"),
                image_path: None,
                emoji: Some("🌙".to_string()),
            },
            device_id: None,
            action: Some(DeviceAction::Power { on: false }),
            is_modifier: false,
        });

        // Climate symbols
        self.add_symbol_mapping(SymbolMapping {
            symbol: AACSymbol {
                id: "temp_up".to_string(),
                category: SymbolCategory::Climate,
                label: LocalizedLabel::new("온도 올리기", "Temperature Up"),
                image_path: None,
                emoji: Some("🔥".to_string()),
            },
            device_id: None,
            action: None, // Modifier
            is_modifier: true,
        });

        self.add_symbol_mapping(SymbolMapping {
            symbol: AACSymbol {
                id: "temp_down".to_string(),
                category: SymbolCategory::Climate,
                label: LocalizedLabel::new("온도 내리기", "Temperature Down"),
                image_path: None,
                emoji: Some("❄️".to_string()),
            },
            device_id: None,
            action: None,
            is_modifier: true,
        });

        // Security symbols
        self.add_symbol_mapping(SymbolMapping {
            symbol: AACSymbol {
                id: "lock".to_string(),
                category: SymbolCategory::Security,
                label: LocalizedLabel::new("잠금", "Lock"),
                image_path: None,
                emoji: Some("🔒".to_string()),
            },
            device_id: None,
            action: Some(DeviceAction::Lock { locked: true }),
            is_modifier: false,
        });

        self.add_symbol_mapping(SymbolMapping {
            symbol: AACSymbol {
                id: "unlock".to_string(),
                category: SymbolCategory::Security,
                label: LocalizedLabel::new("잠금해제", "Unlock"),
                image_path: None,
                emoji: Some("🔓".to_string()),
            },
            device_id: None,
            action: Some(DeviceAction::Lock { locked: false }),
            is_modifier: false,
        });

        // Emergency symbols
        self.add_symbol_mapping(SymbolMapping {
            symbol: AACSymbol {
                id: "emergency".to_string(),
                category: SymbolCategory::Emergency,
                label: LocalizedLabel::new("긴급", "Emergency"),
                image_path: None,
                emoji: Some("🆘".to_string()),
            },
            device_id: None,
            action: None,
            is_modifier: false,
        });
    }

    /// Setup default voice grammar patterns
    fn setup_default_grammar(&mut self) {
        // Korean patterns are handled by regex in process_voice
        // This is a simplified version
    }

    /// Add symbol mapping
    pub fn add_symbol_mapping(&mut self, mapping: SymbolMapping) {
        self.symbol_mapping
            .insert(mapping.symbol.id.clone(), mapping);
    }

    /// Add custom voice command
    pub fn add_custom_command(&mut self, command: CustomCommand) {
        for phrase in &command.phrases {
            self.voice_grammar
                .custom_commands
                .insert(phrase.to_lowercase(), command.clone());
        }
    }

    /// Process symbol selection
    pub fn process_symbol(&mut self, symbol_id: &str) -> Option<AACResult> {
        let mapping = self.symbol_mapping.get(symbol_id)?;

        if mapping.is_modifier {
            // Add to sentence builder
            self.sentence_builder
                .current_sequence
                .push(symbol_id.to_string());
            return Some(AACResult::SequenceUpdated {
                sequence: self.sentence_builder.current_sequence.clone(),
            });
        }

        // Direct action
        if let (Some(device_id), Some(action)) = (mapping.device_id, mapping.action.clone()) {
            return Some(AACResult::Command {
                device_id,
                action,
                symbol: mapping.symbol.clone(),
            });
        }

        // Action without device - user needs to select device
        if let Some(action) = mapping.action.clone() {
            return Some(AACResult::ActionOnly {
                action,
                symbol: mapping.symbol.clone(),
            });
        }

        None
    }

    /// Process voice command
    pub fn process_voice(&self, command: VoiceCommand) -> Option<AACResult> {
        let text = command.text.to_lowercase();

        // Check custom commands first
        if let Some(custom) = self.voice_grammar.custom_commands.get(&text) {
            return Some(AACResult::Command {
                device_id: custom.device_id,
                action: custom.action.clone(),
                symbol: AACSymbol {
                    id: "voice".to_string(),
                    category: SymbolCategory::Navigation,
                    label: LocalizedLabel::new(&command.text, &command.text),
                    image_path: None,
                    emoji: Some("🎤".to_string()),
                },
            });
        }

        // Parse Korean command patterns
        if command.language == Language::Korean {
            return self.parse_korean_command(&text);
        }

        // Parse English command patterns
        if command.language == Language::English {
            return self.parse_english_command(&text);
        }

        None
    }

    fn parse_korean_command(&self, text: &str) -> Option<AACResult> {
        // Pattern: "{장소}의 {장치}를 {동작}해줘"
        // Example: "거실의 불을 켜줘"

        let action = if text.contains("켜") {
            Some(DeviceAction::Power { on: true })
        } else if text.contains("꺼") {
            Some(DeviceAction::Power { on: false })
        } else if text.contains("잠가") || text.contains("잠궈") {
            Some(DeviceAction::Lock { locked: true })
        } else if text.contains("열어") {
            Some(DeviceAction::Lock { locked: false })
        } else {
            None
        };

        let location = if text.contains("거실") {
            Some("living_room")
        } else if text.contains("침실") {
            Some("bedroom")
        } else if text.contains("주방") || text.contains("부엌") {
            Some("kitchen")
        } else if text.contains("화장실") || text.contains("욕실") {
            Some("bathroom")
        } else if text.contains("현관") {
            Some("entrance")
        } else {
            None
        };

        let device_type = if text.contains("불") || text.contains("조명") {
            Some("light")
        } else if text.contains("에어컨") {
            Some("ac")
        } else if text.contains("문") {
            Some("door")
        } else if text.contains("TV") || text.contains("티비") {
            Some("tv")
        } else {
            None
        };

        if let Some(action) = action {
            return Some(AACResult::ParsedVoice {
                action,
                location: location.map(String::from),
                device_type: device_type.map(String::from),
                original_text: text.to_string(),
            });
        }

        None
    }

    fn parse_english_command(&self, text: &str) -> Option<AACResult> {
        // Pattern: "Turn {action} the {device} in the {location}"
        // Example: "Turn on the lights in the living room"

        let action = if text.contains("turn on") || text.contains("switch on") {
            Some(DeviceAction::Power { on: true })
        } else if text.contains("turn off") || text.contains("switch off") {
            Some(DeviceAction::Power { on: false })
        } else if text.contains("lock") {
            Some(DeviceAction::Lock { locked: true })
        } else if text.contains("unlock") {
            Some(DeviceAction::Lock { locked: false })
        } else {
            None
        };

        let location = if text.contains("living room") {
            Some("living_room")
        } else if text.contains("bedroom") {
            Some("bedroom")
        } else if text.contains("kitchen") {
            Some("kitchen")
        } else if text.contains("bathroom") {
            Some("bathroom")
        } else if text.contains("front door") || text.contains("entrance") {
            Some("entrance")
        } else {
            None
        };

        let device_type = if text.contains("light") {
            Some("light")
        } else if text.contains("ac") || text.contains("air") {
            Some("ac")
        } else if text.contains("door") {
            Some("door")
        } else if text.contains("tv") || text.contains("television") {
            Some("tv")
        } else {
            None
        };

        if let Some(action) = action {
            return Some(AACResult::ParsedVoice {
                action,
                location: location.map(String::from),
                device_type: device_type.map(String::from),
                original_text: text.to_string(),
            });
        }

        None
    }

    /// Clear sentence builder
    pub fn clear_sentence(&mut self) {
        self.sentence_builder = SentenceBuilder::default();
    }

    /// Generate TTS feedback
    pub fn generate_tts_feedback(&self, result: &CommandResult) -> TTSFeedback {
        TTSFeedback {
            text: result.message.get(self.config.default_language).to_string(),
            language: self.config.default_language,
            rate: self.config.tts_rate,
        }
    }

    /// Convert AAC result to unified command
    pub fn result_to_command(&self, result: AACResult, device_id: DeviceId) -> Option<UnifiedCommand> {
        match result {
            AACResult::Command { action, .. } => Some(UnifiedCommand::new(
                CommandSource::AAC {
                    symbol: None,
                    voice_text: None,
                    language: self.config.default_language,
                },
                CommandTarget::Device(device_id),
                action,
            )),
            AACResult::ActionOnly { action, symbol } => Some(UnifiedCommand::new(
                CommandSource::AAC {
                    symbol: Some(symbol.id),
                    voice_text: None,
                    language: self.config.default_language,
                },
                CommandTarget::Device(device_id),
                action,
            )),
            AACResult::ParsedVoice {
                action,
                original_text,
                ..
            } => Some(UnifiedCommand::new(
                CommandSource::AAC {
                    symbol: None,
                    voice_text: Some(original_text),
                    language: self.config.default_language,
                },
                CommandTarget::Device(device_id),
                action,
            )),
            _ => None,
        }
    }

    /// Get all symbols by category
    pub fn get_symbols_by_category(&self, category: SymbolCategory) -> Vec<&AACSymbol> {
        self.symbol_mapping
            .values()
            .filter(|m| m.symbol.category == category)
            .map(|m| &m.symbol)
            .collect()
    }
}

impl Default for AACAdapter {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of AAC processing
#[derive(Debug, Clone)]
pub enum AACResult {
    /// Direct command with device and action
    Command {
        device_id: DeviceId,
        action: DeviceAction,
        symbol: AACSymbol,
    },
    /// Action without device (user needs to select)
    ActionOnly {
        action: DeviceAction,
        symbol: AACSymbol,
    },
    /// Sentence sequence updated
    SequenceUpdated { sequence: Vec<String> },
    /// Parsed voice command
    ParsedVoice {
        action: DeviceAction,
        location: Option<String>,
        device_type: Option<String>,
        original_text: String,
    },
}

/// TTS feedback for AAC
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TTSFeedback {
    pub text: String,
    pub language: Language,
    pub rate: f32,
}

/// Trait for AAC smart home control
pub trait AACSmartHome {
    /// Convert symbol to command
    fn symbol_to_command(&self, symbol: &AACSymbol) -> Option<DeviceAction>;
    /// Process voice command
    fn voice_command(&self, text: &str, lang: Language) -> Result<DeviceAction>;
    /// Generate feedback message
    fn generate_feedback(&self, result: &CommandResult) -> AACMessage;
}

/// AAC message for communication
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AACMessage {
    pub text: String,
    pub symbol: Option<String>,
    pub language: Language,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aac_adapter_creation() {
        let adapter = AACAdapter::new();
        assert!(!adapter.symbol_mapping.is_empty());
    }

    #[test]
    fn test_symbol_processing() {
        let mut adapter = AACAdapter::new();

        let result = adapter.process_symbol("light_on");
        assert!(result.is_some());

        if let Some(AACResult::ActionOnly { action, .. }) = result {
            assert!(matches!(action, DeviceAction::Power { on: true }));
        } else {
            panic!("Expected ActionOnly result");
        }
    }

    #[test]
    fn test_korean_voice_parsing() {
        let adapter = AACAdapter::new();

        let command = VoiceCommand {
            text: "거실의 불을 켜줘".to_string(),
            language: Language::Korean,
            confidence: 0.9,
            alternatives: vec![],
        };

        let result = adapter.process_voice(command);
        assert!(result.is_some());

        if let Some(AACResult::ParsedVoice {
            action,
            location,
            device_type,
            ..
        }) = result
        {
            assert!(matches!(action, DeviceAction::Power { on: true }));
            assert_eq!(location, Some("living_room".to_string()));
            assert_eq!(device_type, Some("light".to_string()));
        }
    }

    #[test]
    fn test_english_voice_parsing() {
        let adapter = AACAdapter::new();

        let command = VoiceCommand {
            text: "turn on the lights in the living room".to_string(),
            language: Language::English,
            confidence: 0.9,
            alternatives: vec![],
        };

        let result = adapter.process_voice(command);
        assert!(result.is_some());

        if let Some(AACResult::ParsedVoice {
            action,
            location,
            device_type,
            ..
        }) = result
        {
            assert!(matches!(action, DeviceAction::Power { on: true }));
            assert_eq!(location, Some("living_room".to_string()));
            assert_eq!(device_type, Some("light".to_string()));
        }
    }

    #[test]
    fn test_get_symbols_by_category() {
        let adapter = AACAdapter::new();

        let lighting_symbols = adapter.get_symbols_by_category(SymbolCategory::Lighting);
        assert!(!lighting_symbols.is_empty());

        let security_symbols = adapter.get_symbols_by_category(SymbolCategory::Security);
        assert!(!security_symbols.is_empty());
    }

    #[test]
    fn test_custom_command() {
        let mut adapter = AACAdapter::new();

        let device_id = Uuid::new_v4();
        adapter.add_custom_command(CustomCommand {
            phrases: vec!["bedtime".to_string(), "good night".to_string()],
            device_id,
            action: DeviceAction::Power { on: false },
        });

        let command = VoiceCommand {
            text: "bedtime".to_string(),
            language: Language::English,
            confidence: 0.9,
            alternatives: vec![],
        };

        let result = adapter.process_voice(command);
        assert!(result.is_some());
    }
}
