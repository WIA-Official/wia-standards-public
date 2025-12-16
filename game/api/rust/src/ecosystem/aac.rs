//! Augmentative and Alternative Communication Gaming Integration
//!
//! Provides voice command and symbol-based game controls.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::time::{Duration, Instant};
use uuid::Uuid;

/// AAC adapter for gaming
#[derive(Debug)]
pub struct AACAdapter {
    config: AACConfig,
    voice_macros: HashMap<String, VoiceMacro>,
    symbol_actions: HashMap<String, SymbolAction>,
    last_command: Option<(String, Instant)>,
    simulated_input: Option<AACInput>,
}

/// AAC configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AACConfig {
    /// Enable voice commands
    pub voice_enabled: bool,
    /// Enable symbol selection
    pub symbols_enabled: bool,
    /// Primary language
    pub language: Language,
    /// Voice recognition settings
    pub voice_settings: VoiceSettings,
    /// Confirmation required for critical actions
    pub require_confirmation: bool,
    /// Cooldown between commands (ms)
    pub command_cooldown_ms: u32,
}

/// Voice recognition settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceSettings {
    /// Minimum confidence for recognition
    pub min_confidence: f32,
    /// Enable wake word
    pub wake_word_enabled: bool,
    /// Wake word phrase
    pub wake_word: String,
    /// Continuous listening
    pub continuous: bool,
    /// Timeout for command (seconds)
    pub timeout_secs: u32,
}

/// Supported languages
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Language {
    English,
    Korean,
    Japanese,
    Spanish,
    German,
    French,
    Chinese,
}

/// Voice macro definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceMacro {
    /// Trigger phrase
    pub trigger_phrase: String,
    /// Alternative phrases
    pub alternate_phrases: Vec<String>,
    /// Language for this macro
    pub language: Language,
    /// Game actions to execute
    pub actions: Vec<MacroAction>,
    /// Cooldown (ms)
    pub cooldown_ms: u32,
    /// Requires confirmation
    pub confirmation_required: bool,
    /// Last used timestamp
    #[serde(skip)]
    pub last_used: Option<Instant>,
}

/// Macro action
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MacroAction {
    Press { button: String },
    Hold { button: String, duration_ms: u32 },
    Release { button: String },
    Wait { duration_ms: u32 },
    Sequence { actions: Vec<MacroAction> },
}

/// Symbol action mapping
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SymbolAction {
    /// Symbol identifier
    pub symbol: String,
    /// Display name
    pub name: String,
    /// Game action
    pub action: String,
    /// Cooldown (ms)
    pub cooldown_ms: u32,
    /// Category
    pub category: SymbolCategory,
}

/// Symbol categories
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SymbolCategory {
    Combat,
    Movement,
    Items,
    Menu,
    Communication,
    Emergency,
}

/// AAC input
#[derive(Debug, Clone)]
pub struct AACInput {
    /// Voice command if recognized
    pub voice_command: Option<VoiceCommand>,
    /// Symbol selected
    pub symbol: Option<String>,
    /// Input type
    pub input_type: AACInputType,
}

/// Voice command data
#[derive(Debug, Clone)]
pub struct VoiceCommand {
    /// Recognized phrase
    pub phrase: String,
    /// Confidence (0.0-1.0)
    pub confidence: f32,
    /// Detected language
    pub language: Language,
    /// Is this a macro trigger
    pub is_macro: bool,
}

/// Input type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AACInputType {
    Voice,
    Symbol,
    Gesture,
}

/// Game action from AAC
#[derive(Debug, Clone)]
pub enum AACGameAction {
    /// Execute voice macro
    ExecuteMacro { macro_id: String, actions: Vec<MacroAction> },
    /// Direct command
    DirectCommand { command: String },
    /// Symbol action
    SymbolAction { symbol: String, action: String },
    /// Requires confirmation
    PendingConfirmation { action: Box<AACGameAction> },
    /// On cooldown
    OnCooldown { remaining_ms: u32 },
}

impl AACAdapter {
    /// Create a new AAC adapter
    pub fn new() -> Self {
        let mut adapter = Self {
            config: AACConfig::default(),
            voice_macros: HashMap::new(),
            symbol_actions: HashMap::new(),
            last_command: None,
            simulated_input: None,
        };

        // Add default voice commands
        adapter.add_default_commands();
        adapter.add_default_symbols();

        adapter
    }

    /// Configure the adapter
    pub fn configure(&mut self, config: AACConfig) {
        self.config = config;
    }

    /// Add default voice commands
    fn add_default_commands(&mut self) {
        let commands = vec![
            ("attack", vec!["fire", "shoot"], MacroAction::Press { button: "RT".to_string() }),
            ("reload", vec!["reloading"], MacroAction::Press { button: "X".to_string() }),
            ("jump", vec!["hop"], MacroAction::Press { button: "A".to_string() }),
            ("crouch", vec!["duck", "down"], MacroAction::Press { button: "B".to_string() }),
            ("interact", vec!["use", "action"], MacroAction::Press { button: "Y".to_string() }),
            ("sprint", vec!["run", "fast"], MacroAction::Press { button: "LS".to_string() }),
            ("pause", vec!["menu", "stop"], MacroAction::Press { button: "Start".to_string() }),
        ];

        for (trigger, alts, action) in commands {
            self.voice_macros.insert(
                trigger.to_string(),
                VoiceMacro {
                    trigger_phrase: trigger.to_string(),
                    alternate_phrases: alts.into_iter().map(String::from).collect(),
                    language: Language::English,
                    actions: vec![action],
                    cooldown_ms: 0,
                    confirmation_required: false,
                    last_used: None,
                },
            );
        }

        // Add Korean commands
        let korean_commands = vec![
            ("공격", vec!["발사", "사격"], MacroAction::Press { button: "RT".to_string() }),
            ("재장전", vec!["장전"], MacroAction::Press { button: "X".to_string() }),
            ("점프", vec!["뛰어"], MacroAction::Press { button: "A".to_string() }),
            ("앉기", vec!["숨어"], MacroAction::Press { button: "B".to_string() }),
            ("상호작용", vec!["사용"], MacroAction::Press { button: "Y".to_string() }),
        ];

        for (trigger, alts, action) in korean_commands {
            self.voice_macros.insert(
                format!("ko_{}", trigger),
                VoiceMacro {
                    trigger_phrase: trigger.to_string(),
                    alternate_phrases: alts.into_iter().map(String::from).collect(),
                    language: Language::Korean,
                    actions: vec![action],
                    cooldown_ms: 0,
                    confirmation_required: false,
                    last_used: None,
                },
            );
        }
    }

    /// Add default symbols
    fn add_default_symbols(&mut self) {
        let symbols = vec![
            ("⚔️", "Attack", "attack", SymbolCategory::Combat),
            ("🛡️", "Defend", "defend", SymbolCategory::Combat),
            ("💊", "Health", "use_health", SymbolCategory::Items),
            ("🗺️", "Map", "toggle_map", SymbolCategory::Menu),
            ("⏸️", "Pause", "pause", SymbolCategory::Menu),
            ("💬", "Chat", "open_chat", SymbolCategory::Communication),
            ("🎯", "Target", "lock_target", SymbolCategory::Combat),
            ("🏃", "Sprint", "sprint", SymbolCategory::Movement),
            ("🆘", "Help", "emergency", SymbolCategory::Emergency),
        ];

        for (symbol, name, action, category) in symbols {
            self.symbol_actions.insert(
                symbol.to_string(),
                SymbolAction {
                    symbol: symbol.to_string(),
                    name: name.to_string(),
                    action: action.to_string(),
                    cooldown_ms: if category == SymbolCategory::Items { 3000 } else { 0 },
                    category,
                },
            );
        }
    }

    /// Add a custom voice macro
    pub fn add_voice_macro(&mut self, id: String, macro_def: VoiceMacro) {
        self.voice_macros.insert(id, macro_def);
    }

    /// Add a custom symbol action
    pub fn add_symbol_action(&mut self, symbol: String, action: SymbolAction) {
        self.symbol_actions.insert(symbol, action);
    }

    /// Set simulated input for testing
    pub fn set_simulated_input(&mut self, input: AACInput) {
        self.simulated_input = Some(input);
    }

    /// Process voice command
    pub fn process_voice(&mut self, phrase: &str, confidence: f32) -> Option<AACGameAction> {
        if confidence < self.config.voice_settings.min_confidence {
            return None;
        }

        let phrase_lower = phrase.to_lowercase();

        // Check cooldown
        if let Some((last_phrase, time)) = &self.last_command {
            if last_phrase == &phrase_lower {
                let elapsed = time.elapsed().as_millis() as u32;
                if elapsed < self.config.command_cooldown_ms {
                    return Some(AACGameAction::OnCooldown {
                        remaining_ms: self.config.command_cooldown_ms - elapsed,
                    });
                }
            }
        }

        // Find matching macro
        for (id, macro_def) in &self.voice_macros {
            if macro_def.trigger_phrase.to_lowercase() == phrase_lower
                || macro_def.alternate_phrases.iter().any(|p| p.to_lowercase() == phrase_lower)
            {
                // Check macro cooldown
                if let Some(last) = macro_def.last_used {
                    let elapsed = last.elapsed().as_millis() as u32;
                    if elapsed < macro_def.cooldown_ms {
                        return Some(AACGameAction::OnCooldown {
                            remaining_ms: macro_def.cooldown_ms - elapsed,
                        });
                    }
                }

                let action = AACGameAction::ExecuteMacro {
                    macro_id: id.clone(),
                    actions: macro_def.actions.clone(),
                };

                if macro_def.confirmation_required {
                    return Some(AACGameAction::PendingConfirmation {
                        action: Box::new(action),
                    });
                }

                self.last_command = Some((phrase_lower, Instant::now()));
                return Some(action);
            }
        }

        // Direct command
        self.last_command = Some((phrase_lower.clone(), Instant::now()));
        Some(AACGameAction::DirectCommand { command: phrase_lower })
    }

    /// Process symbol selection
    pub fn process_symbol(&mut self, symbol: &str) -> Option<AACGameAction> {
        let action = self.symbol_actions.get(symbol)?;

        Some(AACGameAction::SymbolAction {
            symbol: symbol.to_string(),
            action: action.action.clone(),
        })
    }

    /// Poll for input
    pub fn poll(&mut self) -> Option<AACInput> {
        self.simulated_input.take()
    }

    /// Get action from input
    pub fn get_action(&mut self, input: &AACInput) -> Option<AACGameAction> {
        match input.input_type {
            AACInputType::Voice => {
                if let Some(ref cmd) = input.voice_command {
                    self.process_voice(&cmd.phrase, cmd.confidence)
                } else {
                    None
                }
            }
            AACInputType::Symbol => {
                if let Some(ref symbol) = input.symbol {
                    self.process_symbol(symbol)
                } else {
                    None
                }
            }
            AACInputType::Gesture => None, // Future: gesture recognition
        }
    }

    /// Get all available symbols by category
    pub fn get_symbols_by_category(&self, category: SymbolCategory) -> Vec<&SymbolAction> {
        self.symbol_actions
            .values()
            .filter(|a| a.category == category)
            .collect()
    }
}

impl Default for AACAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for AACConfig {
    fn default() -> Self {
        Self {
            voice_enabled: true,
            symbols_enabled: true,
            language: Language::English,
            voice_settings: VoiceSettings {
                min_confidence: 0.7,
                wake_word_enabled: false,
                wake_word: "game".to_string(),
                continuous: true,
                timeout_secs: 5,
            },
            require_confirmation: false,
            command_cooldown_ms: 200,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_aac_adapter_creation() {
        let adapter = AACAdapter::new();
        assert!(!adapter.voice_macros.is_empty());
        assert!(!adapter.symbol_actions.is_empty());
    }

    #[test]
    fn test_voice_command_attack() {
        let mut adapter = AACAdapter::new();
        let action = adapter.process_voice("attack", 0.9);

        assert!(matches!(action, Some(AACGameAction::ExecuteMacro { .. })));
    }

    #[test]
    fn test_voice_command_alternate() {
        let mut adapter = AACAdapter::new();
        let action = adapter.process_voice("fire", 0.9);

        assert!(matches!(action, Some(AACGameAction::ExecuteMacro { .. })));
    }

    #[test]
    fn test_low_confidence_rejected() {
        let mut adapter = AACAdapter::new();
        let action = adapter.process_voice("attack", 0.3);

        assert!(action.is_none());
    }

    #[test]
    fn test_symbol_action() {
        let mut adapter = AACAdapter::new();
        let action = adapter.process_symbol("⚔️");

        assert!(matches!(action, Some(AACGameAction::SymbolAction { .. })));
    }

    #[test]
    fn test_korean_commands() {
        let mut adapter = AACAdapter::new();
        let action = adapter.process_voice("공격", 0.9);

        assert!(matches!(action, Some(AACGameAction::ExecuteMacro { .. })));
    }

    #[test]
    fn test_symbols_by_category() {
        let adapter = AACAdapter::new();
        let combat_symbols = adapter.get_symbols_by_category(SymbolCategory::Combat);

        assert!(!combat_symbols.is_empty());
    }
}
