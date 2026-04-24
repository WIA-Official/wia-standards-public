//! Voice Interface Input Adapter

use async_trait::async_trait;
use std::collections::HashMap;

use crate::error::WiaAiError;
use crate::integration::{
    AiInput, AiInputType, WiaMessage, WiaPayload, WiaStandardType,
    connector::AiInputAdapter,
};

/// Voice Input Adapter
///
/// Converts voice/audio input into AI-processable text.
/// Handles ASR (Automatic Speech Recognition) results.
pub struct VoiceInputAdapter {
    /// Default language
    default_language: String,
}

impl VoiceInputAdapter {
    /// Create a new voice input adapter
    pub fn new() -> Self {
        Self {
            default_language: "en-US".into(),
        }
    }

    /// Create with a specific default language
    pub fn with_language(language: impl Into<String>) -> Self {
        Self {
            default_language: language.into(),
        }
    }

    /// Process voice payload
    fn process_voice(&self, payload: &WiaPayload) -> Result<String, WiaAiError> {
        match payload {
            WiaPayload::Text(text) => Ok(text.clone()),
            WiaPayload::Json(json) => {
                // Extract transcript from ASR result
                if let Some(transcript) = json.get("transcript").and_then(|v| v.as_str()) {
                    Ok(transcript.to_string())
                } else if let Some(text) = json.get("text").and_then(|v| v.as_str()) {
                    Ok(text.to_string())
                } else if let Some(result) = json.get("result").and_then(|v| v.as_str()) {
                    Ok(result.to_string())
                } else {
                    Ok(json.to_string())
                }
            }
            WiaPayload::Binary(data) => {
                // Raw audio data - would need ASR processing
                Ok(format!("[Audio: {} bytes]", data.len()))
            }
            WiaPayload::Signal(_) => {
                Ok("[Audio Signal]".to_string())
            }
        }
    }
}

impl Default for VoiceInputAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl AiInputAdapter for VoiceInputAdapter {
    fn name(&self) -> &str {
        "voice_input"
    }

    fn input_type(&self) -> AiInputType {
        AiInputType::Audio
    }

    fn supported_standards(&self) -> Vec<WiaStandardType> {
        vec![WiaStandardType::Voice]
    }

    async fn to_ai_input(&self, message: WiaMessage) -> Result<AiInput, WiaAiError> {
        let text = self.process_voice(&message.payload)?;

        let mut context = HashMap::new();

        // Extract voice-specific metadata
        if let Some(language) = message.metadata.get("language") {
            context.insert("language".into(), language.clone());
        } else {
            context.insert(
                "language".into(),
                serde_json::json!(self.default_language.clone()),
            );
        }
        if let Some(confidence) = message.metadata.get("confidence") {
            context.insert("confidence".into(), confidence.clone());
        }
        if let Some(speaker) = message.metadata.get("speaker_id") {
            context.insert("speaker_id".into(), speaker.clone());
        }
        if let Some(is_final) = message.metadata.get("is_final") {
            context.insert("is_final".into(), is_final.clone());
        }

        Ok(AiInput {
            input_type: AiInputType::Text,
            text: Some(text),
            data: None,
            source_message: Some(message),
            context,
        })
    }

    fn can_handle(&self, message: &WiaMessage) -> bool {
        message.source == WiaStandardType::Voice
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_voice_transcript_input() {
        let adapter = VoiceInputAdapter::new();

        let message = WiaMessage::text(
            WiaStandardType::Voice,
            WiaStandardType::Ai,
            "Hello, how are you?",
        )
        .with_metadata("language", serde_json::json!("en-US"))
        .with_metadata("confidence", serde_json::json!(0.98));

        let input = adapter.to_ai_input(message).await.unwrap();
        assert_eq!(input.text, Some("Hello, how are you?".into()));
        assert!(input.context.contains_key("language"));
    }

    #[tokio::test]
    async fn test_voice_json_input() {
        let adapter = VoiceInputAdapter::new();

        let message = WiaMessage::command(
            WiaStandardType::Voice,
            WiaStandardType::Ai,
            serde_json::json!({
                "transcript": "Turn on the lights",
                "confidence": 0.95
            }),
        );

        let input = adapter.to_ai_input(message).await.unwrap();
        assert_eq!(input.text, Some("Turn on the lights".into()));
    }

    #[test]
    fn test_can_handle() {
        let adapter = VoiceInputAdapter::new();

        let voice_msg = WiaMessage::text(WiaStandardType::Voice, WiaStandardType::Ai, "test");
        let aac_msg = WiaMessage::text(WiaStandardType::Aac, WiaStandardType::Ai, "test");

        assert!(adapter.can_handle(&voice_msg));
        assert!(!adapter.can_handle(&aac_msg));
    }
}
