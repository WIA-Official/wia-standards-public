//! AAC (Augmentative and Alternative Communication) Integration
//!
//! Provides integration with AAC systems, voice commands,
//! and symbol-based communication.

use async_trait::async_trait;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use crate::error::Result;

/// Voice command result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceCommandResult {
    /// Recognized text
    pub text: String,
    /// Confidence score (0-1)
    pub confidence: f32,
    /// Language
    pub language: String,
    /// Matched action (if any)
    pub action: Option<VehicleAction>,
}

/// Vehicle action from voice/AAC
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VehicleAction {
    // Navigation
    ChangeDestination,
    CancelTrip,
    PullOver,
    ResumeTrip,
    NavigateHome,
    FindRestroom,
    FindRestaurant,

    // Comfort
    TemperatureUp,
    TemperatureDown,
    WindowOpen,
    WindowClose,

    // Information
    AnnounceEta,
    AnnounceLocation,
    VehicleStatus,

    // Emergency
    EmergencyStop,
    RequestAssistance,
}

/// Voice command pattern
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VoiceCommandPattern {
    /// Patterns to match (regex)
    pub patterns: Vec<String>,
    /// Action to trigger
    pub action: VehicleAction,
    /// Whether confirmation is required
    pub confirmation_required: bool,
    /// Priority level
    pub priority: CommandPriority,
}

/// Command priority
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommandPriority {
    Low,
    Normal,
    High,
    Critical,
}

/// Default Korean voice command patterns
pub fn default_korean_commands() -> Vec<VoiceCommandPattern> {
    vec![
        // Navigation
        VoiceCommandPattern {
            patterns: vec![
                r"(목적지|행선지)를?\s*(.+)(으로|로)\s*(변경|바꿔)".to_string(),
            ],
            action: VehicleAction::ChangeDestination,
            confirmation_required: true,
            priority: CommandPriority::Normal,
        },
        VoiceCommandPattern {
            patterns: vec![
                r"(여기|지금)\s*(세워|정차|멈춰)".to_string(),
                r"(내려|내릴)".to_string(),
            ],
            action: VehicleAction::PullOver,
            confirmation_required: false,
            priority: CommandPriority::High,
        },
        VoiceCommandPattern {
            patterns: vec![
                r"(이동|여행)\s*(취소|그만)".to_string(),
            ],
            action: VehicleAction::CancelTrip,
            confirmation_required: true,
            priority: CommandPriority::Normal,
        },
        VoiceCommandPattern {
            patterns: vec![
                r"집(으로|에)?\s*(가|데려다)".to_string(),
            ],
            action: VehicleAction::NavigateHome,
            confirmation_required: true,
            priority: CommandPriority::Normal,
        },
        // Comfort
        VoiceCommandPattern {
            patterns: vec![
                r"온도\s*(올려|높여|따뜻하게)".to_string(),
                r"(추워|춥다)".to_string(),
            ],
            action: VehicleAction::TemperatureUp,
            confirmation_required: false,
            priority: CommandPriority::Low,
        },
        VoiceCommandPattern {
            patterns: vec![
                r"온도\s*(내려|낮춰|시원하게)".to_string(),
                r"(더워|덥다)".to_string(),
            ],
            action: VehicleAction::TemperatureDown,
            confirmation_required: false,
            priority: CommandPriority::Low,
        },
        VoiceCommandPattern {
            patterns: vec![
                r"(창문|창)\s*(열어|내려)".to_string(),
            ],
            action: VehicleAction::WindowOpen,
            confirmation_required: false,
            priority: CommandPriority::Low,
        },
        // Information
        VoiceCommandPattern {
            patterns: vec![
                r"(언제|몇\s*시에)\s*(도착|도달)".to_string(),
                r"(얼마나|시간)\s*(남았|걸려)".to_string(),
            ],
            action: VehicleAction::AnnounceEta,
            confirmation_required: false,
            priority: CommandPriority::Low,
        },
        VoiceCommandPattern {
            patterns: vec![
                r"(어디|현재\s*위치)".to_string(),
                r"지금\s*(어디|위치)".to_string(),
            ],
            action: VehicleAction::AnnounceLocation,
            confirmation_required: false,
            priority: CommandPriority::Low,
        },
        // Emergency
        VoiceCommandPattern {
            patterns: vec![
                r"(도움|도와줘|help|긴급)".to_string(),
                r"(비상|응급)".to_string(),
            ],
            action: VehicleAction::RequestAssistance,
            confirmation_required: false,
            priority: CommandPriority::Critical,
        },
    ]
}

/// AAC symbol mapping
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SymbolMapping {
    /// Symbol identifier
    pub symbol_id: String,
    /// Symbol system (PCS, Blissymbols, etc.)
    pub symbol_system: SymbolSystem,
    /// Action to trigger
    pub action: VehicleAction,
    /// Priority
    pub priority: CommandPriority,
}

/// Symbol system types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SymbolSystem {
    /// Picture Communication Symbols
    Pcs,
    /// Blissymbols
    Blissymbols,
    /// SymbolStix
    SymbolStix,
    /// Widgit
    Widgit,
    /// Custom
    Custom,
}

/// Default symbol mappings
pub fn default_symbol_mappings() -> Vec<SymbolMapping> {
    vec![
        SymbolMapping {
            symbol_id: "pcs_stop".to_string(),
            symbol_system: SymbolSystem::Pcs,
            action: VehicleAction::EmergencyStop,
            priority: CommandPriority::Critical,
        },
        SymbolMapping {
            symbol_id: "pcs_go".to_string(),
            symbol_system: SymbolSystem::Pcs,
            action: VehicleAction::ResumeTrip,
            priority: CommandPriority::Normal,
        },
        SymbolMapping {
            symbol_id: "pcs_help".to_string(),
            symbol_system: SymbolSystem::Pcs,
            action: VehicleAction::RequestAssistance,
            priority: CommandPriority::High,
        },
        SymbolMapping {
            symbol_id: "pcs_home".to_string(),
            symbol_system: SymbolSystem::Pcs,
            action: VehicleAction::NavigateHome,
            priority: CommandPriority::Normal,
        },
        SymbolMapping {
            symbol_id: "pcs_hot".to_string(),
            symbol_system: SymbolSystem::Pcs,
            action: VehicleAction::TemperatureDown,
            priority: CommandPriority::Low,
        },
        SymbolMapping {
            symbol_id: "pcs_cold".to_string(),
            symbol_system: SymbolSystem::Pcs,
            action: VehicleAction::TemperatureUp,
            priority: CommandPriority::Low,
        },
        SymbolMapping {
            symbol_id: "pcs_toilet".to_string(),
            symbol_system: SymbolSystem::Pcs,
            action: VehicleAction::FindRestroom,
            priority: CommandPriority::Normal,
        },
    ]
}

/// TTS (Text-to-Speech) configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TtsConfig {
    /// Language code
    pub language: String,
    /// Speech rate (0.5-2.0)
    pub rate: f32,
    /// Pitch (0.5-2.0)
    pub pitch: f32,
    /// Voice ID
    pub voice_id: String,
}

impl Default for TtsConfig {
    fn default() -> Self {
        Self {
            language: "ko-KR".to_string(),
            rate: 0.9,
            pitch: 1.0,
            voice_id: "ko-KR-Standard-A".to_string(),
        }
    }
}

/// TTS announcement types
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AnnouncementType {
    TripStart,
    ApproachingDestination,
    Arrived,
    SecurementReminder,
    EmergencyAlert,
    EtaUpdate,
    LocationUpdate,
    ComfortChange,
}

/// TTS announcement
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TtsAnnouncement {
    /// Announcement type
    pub announcement_type: AnnouncementType,
    /// Text to speak
    pub text: String,
    /// Priority
    pub priority: CommandPriority,
    /// Volume boost
    pub volume_boost: bool,
}

impl TtsAnnouncement {
    /// Create new announcement
    pub fn new(announcement_type: AnnouncementType, text: impl Into<String>) -> Self {
        Self {
            announcement_type,
            text: text.into(),
            priority: CommandPriority::Normal,
            volume_boost: false,
        }
    }

    /// Create emergency announcement
    pub fn emergency(text: impl Into<String>) -> Self {
        Self {
            announcement_type: AnnouncementType::EmergencyAlert,
            text: text.into(),
            priority: CommandPriority::Critical,
            volume_boost: true,
        }
    }
}

/// AAC status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AacStatus {
    /// Voice recognition active
    pub voice_active: bool,
    /// Symbol board active
    pub symbol_board_active: bool,
    /// TTS active
    pub tts_active: bool,
    /// Current language
    pub language: String,
    /// Last command
    pub last_command: Option<VehicleAction>,
}

impl Default for AacStatus {
    fn default() -> Self {
        Self {
            voice_active: false,
            symbol_board_active: false,
            tts_active: true,
            language: "ko-KR".to_string(),
            last_command: None,
        }
    }
}

/// AAC integration handler
#[async_trait]
pub trait AacIntegration: Send + Sync {
    /// Initialize AAC system
    async fn initialize(&mut self) -> Result<()>;

    /// Start voice recognition
    async fn start_voice(&mut self) -> Result<()>;

    /// Stop voice recognition
    async fn stop_voice(&mut self) -> Result<()>;

    /// Process voice input
    async fn process_voice(&self, audio_data: &[u8]) -> Result<Option<VoiceCommandResult>>;

    /// Process symbol input
    async fn process_symbol(&self, symbol_id: &str) -> Result<Option<VehicleAction>>;

    /// Speak text
    async fn speak(&self, announcement: &TtsAnnouncement) -> Result<()>;

    /// Get status
    async fn status(&self) -> Result<AacStatus>;
}

/// Mock AAC integration for testing
pub struct MockAacIntegration {
    status: std::sync::Arc<tokio::sync::RwLock<AacStatus>>,
    symbol_map: HashMap<String, VehicleAction>,
    spoken: std::sync::Arc<tokio::sync::RwLock<Vec<String>>>,
}

impl MockAacIntegration {
    /// Create new mock integration
    pub fn new() -> Self {
        let mut symbol_map = HashMap::new();
        for mapping in default_symbol_mappings() {
            symbol_map.insert(mapping.symbol_id, mapping.action);
        }

        Self {
            status: std::sync::Arc::new(tokio::sync::RwLock::new(AacStatus::default())),
            symbol_map,
            spoken: std::sync::Arc::new(tokio::sync::RwLock::new(Vec::new())),
        }
    }

    /// Get spoken announcements
    pub async fn spoken_texts(&self) -> Vec<String> {
        self.spoken.read().await.clone()
    }
}

impl Default for MockAacIntegration {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl AacIntegration for MockAacIntegration {
    async fn initialize(&mut self) -> Result<()> {
        let mut status = self.status.write().await;
        status.tts_active = true;
        Ok(())
    }

    async fn start_voice(&mut self) -> Result<()> {
        self.status.write().await.voice_active = true;
        Ok(())
    }

    async fn stop_voice(&mut self) -> Result<()> {
        self.status.write().await.voice_active = false;
        Ok(())
    }

    async fn process_voice(&self, _audio_data: &[u8]) -> Result<Option<VoiceCommandResult>> {
        // Mock: return no recognition
        Ok(None)
    }

    async fn process_symbol(&self, symbol_id: &str) -> Result<Option<VehicleAction>> {
        Ok(self.symbol_map.get(symbol_id).cloned())
    }

    async fn speak(&self, announcement: &TtsAnnouncement) -> Result<()> {
        self.spoken.write().await.push(announcement.text.clone());
        Ok(())
    }

    async fn status(&self) -> Result<AacStatus> {
        Ok(self.status.read().await.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_command_priority_ordering() {
        assert!(CommandPriority::Critical > CommandPriority::High);
        assert!(CommandPriority::High > CommandPriority::Normal);
        assert!(CommandPriority::Normal > CommandPriority::Low);
    }

    #[test]
    fn test_tts_announcement() {
        let normal = TtsAnnouncement::new(
            AnnouncementType::TripStart,
            "강남역으로 출발합니다."
        );
        assert!(!normal.volume_boost);

        let emergency = TtsAnnouncement::emergency("긴급 상황입니다.");
        assert!(emergency.volume_boost);
        assert_eq!(emergency.priority, CommandPriority::Critical);
    }

    #[tokio::test]
    async fn test_mock_integration() {
        let mut integration = MockAacIntegration::new();
        integration.initialize().await.unwrap();

        // Test symbol processing
        let action = integration.process_symbol("pcs_stop").await.unwrap();
        assert_eq!(action, Some(VehicleAction::EmergencyStop));

        // Test TTS
        let announcement = TtsAnnouncement::new(
            AnnouncementType::TripStart,
            "출발합니다."
        );
        integration.speak(&announcement).await.unwrap();

        let spoken = integration.spoken_texts().await;
        assert_eq!(spoken.len(), 1);
        assert_eq!(spoken[0], "출발합니다.");
    }
}
