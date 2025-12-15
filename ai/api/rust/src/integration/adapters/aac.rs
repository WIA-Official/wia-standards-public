//! AAC (Augmentative and Alternative Communication) Input Adapter

use async_trait::async_trait;
use std::collections::HashMap;

use crate::error::WiaAiError;
use crate::integration::{
    AiInput, AiInputType, WiaMessage, WiaPayload, WiaStandardType,
    connector::AiInputAdapter,
};

/// AAC Input Adapter
///
/// Converts AAC sensor signals and selections into AI-processable input.
/// Supports various AAC input methods:
/// - Eye tracking
/// - EMG (muscle signals)
/// - Switch scanning
/// - Breath control
/// - EEG (brain signals for AAC)
pub struct AacInputAdapter {
    /// Supported sensor types
    supported_sensors: Vec<String>,
}

impl AacInputAdapter {
    /// Create a new AAC input adapter
    pub fn new() -> Self {
        Self {
            supported_sensors: vec![
                "eye_tracker".into(),
                "emg".into(),
                "switch".into(),
                "breath".into(),
                "eeg".into(),
                "touch".into(),
            ],
        }
    }

    /// Create with specific sensor support
    pub fn with_sensors(sensors: Vec<String>) -> Self {
        Self {
            supported_sensors: sensors,
        }
    }

    /// Process AAC signal payload to text
    fn process_signal(&self, payload: &WiaPayload) -> Result<String, WiaAiError> {
        match payload {
            WiaPayload::Text(text) => Ok(text.clone()),
            WiaPayload::Json(json) => {
                // Extract text from JSON payload
                if let Some(text) = json.get("text").and_then(|v| v.as_str()) {
                    Ok(text.to_string())
                } else if let Some(selection) = json.get("selection").and_then(|v| v.as_str()) {
                    Ok(selection.to_string())
                } else if let Some(intent) = json.get("intent").and_then(|v| v.as_str()) {
                    Ok(intent.to_string())
                } else {
                    Ok(json.to_string())
                }
            }
            WiaPayload::Signal(signal) => {
                // For raw signals, we'd normally run classification
                // Here we just indicate signal received
                Ok(format!(
                    "[AAC Signal: {} channels, {} samples]",
                    signal.channels.len(),
                    signal.samples.first().map(|s| s.len()).unwrap_or(0)
                ))
            }
            WiaPayload::Binary(_) => {
                Ok("[AAC Binary Data]".to_string())
            }
        }
    }
}

impl Default for AacInputAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl AiInputAdapter for AacInputAdapter {
    fn name(&self) -> &str {
        "aac_input"
    }

    fn input_type(&self) -> AiInputType {
        AiInputType::Signal
    }

    fn supported_standards(&self) -> Vec<WiaStandardType> {
        vec![WiaStandardType::Aac]
    }

    async fn to_ai_input(&self, message: WiaMessage) -> Result<AiInput, WiaAiError> {
        let text = self.process_signal(&message.payload)?;

        let mut context = HashMap::new();

        // Extract context from metadata
        if let Some(sensor) = message.metadata.get("sensor_type") {
            context.insert("sensor_type".into(), sensor.clone());
        }
        if let Some(confidence) = message.metadata.get("confidence") {
            context.insert("confidence".into(), confidence.clone());
        }
        if let Some(user_id) = message.metadata.get("user_id") {
            context.insert("user_id".into(), user_id.clone());
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
        message.source == WiaStandardType::Aac
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Utc;

    #[tokio::test]
    async fn test_aac_text_input() {
        let adapter = AacInputAdapter::new();

        let message = WiaMessage {
            id: "test".into(),
            source: WiaStandardType::Aac,
            target: WiaStandardType::Ai,
            message_type: crate::integration::WiaMessageType::Text,
            payload: WiaPayload::Text("I need help".into()),
            metadata: HashMap::new(),
            timestamp: Utc::now(),
        };

        let input = adapter.to_ai_input(message).await.unwrap();
        assert_eq!(input.text, Some("I need help".into()));
    }

    #[tokio::test]
    async fn test_aac_json_input() {
        let adapter = AacInputAdapter::new();

        let message = WiaMessage {
            id: "test".into(),
            source: WiaStandardType::Aac,
            target: WiaStandardType::Ai,
            message_type: crate::integration::WiaMessageType::Command,
            payload: WiaPayload::Json(serde_json::json!({
                "selection": "Yes",
                "confidence": 0.95
            })),
            metadata: HashMap::new(),
            timestamp: Utc::now(),
        };

        let input = adapter.to_ai_input(message).await.unwrap();
        assert_eq!(input.text, Some("Yes".into()));
    }

    #[test]
    fn test_can_handle() {
        let adapter = AacInputAdapter::new();

        let aac_msg = WiaMessage::text(WiaStandardType::Aac, WiaStandardType::Ai, "test");
        let bci_msg = WiaMessage::text(WiaStandardType::Bci, WiaStandardType::Ai, "test");

        assert!(adapter.can_handle(&aac_msg));
        assert!(!adapter.can_handle(&bci_msg));
    }
}
