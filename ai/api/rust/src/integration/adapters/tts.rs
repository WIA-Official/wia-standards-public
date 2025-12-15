//! TTS (Text-to-Speech) Output Adapter

use async_trait::async_trait;
use chrono::Utc;
use std::collections::HashMap;

use crate::error::WiaAiError;
use crate::integration::{
    AiOutput, AiOutputType, WiaMessage, WiaMessageType, WiaPayload, WiaStandardType,
    connector::AiOutputAdapter,
};

/// TTS Output Adapter
///
/// Converts AI text output into TTS-ready messages.
pub struct TtsOutputAdapter {
    /// Default voice configuration
    default_voice: VoiceConfig,
}

/// Voice configuration for TTS
#[derive(Debug, Clone)]
pub struct VoiceConfig {
    /// Voice ID
    pub voice_id: String,
    /// Language code (e.g., "en-US", "ko-KR")
    pub language: String,
    /// Speaking rate (0.5 to 2.0, default 1.0)
    pub rate: f32,
    /// Pitch adjustment (-20 to 20, default 0)
    pub pitch: f32,
    /// Volume (0.0 to 1.0, default 1.0)
    pub volume: f32,
}

impl Default for VoiceConfig {
    fn default() -> Self {
        Self {
            voice_id: "default".into(),
            language: "en-US".into(),
            rate: 1.0,
            pitch: 0.0,
            volume: 1.0,
        }
    }
}

impl VoiceConfig {
    /// Create a new voice configuration
    pub fn new(voice_id: impl Into<String>, language: impl Into<String>) -> Self {
        Self {
            voice_id: voice_id.into(),
            language: language.into(),
            ..Default::default()
        }
    }

    /// Set speaking rate
    pub fn with_rate(mut self, rate: f32) -> Self {
        self.rate = rate.clamp(0.5, 2.0);
        self
    }

    /// Set pitch
    pub fn with_pitch(mut self, pitch: f32) -> Self {
        self.pitch = pitch.clamp(-20.0, 20.0);
        self
    }

    /// Set volume
    pub fn with_volume(mut self, volume: f32) -> Self {
        self.volume = volume.clamp(0.0, 1.0);
        self
    }
}

impl TtsOutputAdapter {
    /// Create a new TTS output adapter
    pub fn new() -> Self {
        Self {
            default_voice: VoiceConfig::default(),
        }
    }

    /// Create with a specific voice configuration
    pub fn with_voice(voice: VoiceConfig) -> Self {
        Self {
            default_voice: voice,
        }
    }

    /// Get the current voice configuration
    pub fn voice_config(&self) -> &VoiceConfig {
        &self.default_voice
    }
}

impl Default for TtsOutputAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl AiOutputAdapter for TtsOutputAdapter {
    fn name(&self) -> &str {
        "tts_output"
    }

    fn output_type(&self) -> AiOutputType {
        AiOutputType::Speech
    }

    fn target_standard(&self) -> WiaStandardType {
        WiaStandardType::Tts
    }

    async fn from_ai_output(&self, output: AiOutput) -> Result<WiaMessage, WiaAiError> {
        let mut metadata = HashMap::new();
        metadata.insert("voice_id".into(), serde_json::json!(self.default_voice.voice_id));
        metadata.insert("language".into(), serde_json::json!(self.default_voice.language));
        metadata.insert("rate".into(), serde_json::json!(self.default_voice.rate));
        metadata.insert("pitch".into(), serde_json::json!(self.default_voice.pitch));
        metadata.insert("volume".into(), serde_json::json!(self.default_voice.volume));

        // Merge with output metadata
        for (key, value) in output.metadata {
            metadata.insert(key, value);
        }

        Ok(WiaMessage {
            id: uuid::Uuid::new_v4().to_string(),
            source: WiaStandardType::Ai,
            target: WiaStandardType::Tts,
            message_type: WiaMessageType::Command,
            payload: WiaPayload::Json(serde_json::json!({
                "text": output.text,
                "voice": {
                    "id": self.default_voice.voice_id,
                    "language": self.default_voice.language,
                    "rate": self.default_voice.rate,
                    "pitch": self.default_voice.pitch,
                    "volume": self.default_voice.volume,
                }
            })),
            metadata,
            timestamp: Utc::now(),
        })
    }

    fn supports_streaming(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_tts_output() {
        let adapter = TtsOutputAdapter::new();

        let output = AiOutput::text("Hello, world!");
        let message = adapter.from_ai_output(output).await.unwrap();

        assert_eq!(message.target, WiaStandardType::Tts);
        assert_eq!(message.message_type, WiaMessageType::Command);

        if let WiaPayload::Json(json) = &message.payload {
            assert_eq!(json["text"], "Hello, world!");
        } else {
            panic!("Expected JSON payload");
        }
    }

    #[tokio::test]
    async fn test_tts_with_custom_voice() {
        let voice = VoiceConfig::new("neural-voice-1", "ko-KR")
            .with_rate(1.2)
            .with_pitch(2.0);
        let adapter = TtsOutputAdapter::with_voice(voice);

        let output = AiOutput::text("안녕하세요");
        let message = adapter.from_ai_output(output).await.unwrap();

        if let WiaPayload::Json(json) = &message.payload {
            assert_eq!(json["voice"]["language"], "ko-KR");
            assert!((json["voice"]["rate"].as_f64().unwrap() - 1.2).abs() < 0.01);
        } else {
            panic!("Expected JSON payload");
        }
    }

    #[test]
    fn test_voice_config_clamping() {
        let voice = VoiceConfig::default()
            .with_rate(5.0)  // Should clamp to 2.0
            .with_volume(1.5);  // Should clamp to 1.0

        assert_eq!(voice.rate, 2.0);
        assert_eq!(voice.volume, 1.0);
    }
}
