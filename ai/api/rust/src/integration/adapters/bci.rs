//! BCI (Brain-Computer Interface) Input Adapter

use async_trait::async_trait;
use std::collections::HashMap;

use crate::error::WiaAiError;
use crate::integration::{
    AiInput, AiInputType, WiaMessage, WiaPayload, WiaStandardType,
    connector::AiInputAdapter,
};

/// BCI Input Adapter
///
/// Converts brain signal classifications into AI-processable input.
/// Handles various BCI paradigms:
/// - Motor imagery
/// - P300
/// - SSVEP
/// - Mental states
pub struct BciInputAdapter {
    /// Supported paradigms
    supported_paradigms: Vec<String>,
}

impl BciInputAdapter {
    /// Create a new BCI input adapter
    pub fn new() -> Self {
        Self {
            supported_paradigms: vec![
                "motor_imagery".into(),
                "p300".into(),
                "ssvep".into(),
                "mental_state".into(),
            ],
        }
    }

    /// Create with specific paradigm support
    pub fn with_paradigms(paradigms: Vec<String>) -> Self {
        Self {
            supported_paradigms: paradigms,
        }
    }

    /// Process BCI signal to intent
    fn process_bci_signal(&self, payload: &WiaPayload) -> Result<String, WiaAiError> {
        match payload {
            WiaPayload::Text(text) => Ok(text.clone()),
            WiaPayload::Json(json) => {
                // Extract classification result
                if let Some(intent) = json.get("intent").and_then(|v| v.as_str()) {
                    Ok(intent.to_string())
                } else if let Some(class) = json.get("classification").and_then(|v| v.as_str()) {
                    Ok(class.to_string())
                } else if let Some(command) = json.get("command").and_then(|v| v.as_str()) {
                    Ok(command.to_string())
                } else {
                    Ok(json.to_string())
                }
            }
            WiaPayload::Signal(signal) => {
                // Raw EEG signal - would normally run through classifier
                Ok(format!(
                    "[BCI Signal: {} channels at {}Hz]",
                    signal.channels.len(),
                    signal.sample_rate
                ))
            }
            WiaPayload::Binary(_) => {
                Ok("[BCI Binary Data]".to_string())
            }
        }
    }
}

impl Default for BciInputAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl AiInputAdapter for BciInputAdapter {
    fn name(&self) -> &str {
        "bci_input"
    }

    fn input_type(&self) -> AiInputType {
        AiInputType::Signal
    }

    fn supported_standards(&self) -> Vec<WiaStandardType> {
        vec![WiaStandardType::Bci]
    }

    async fn to_ai_input(&self, message: WiaMessage) -> Result<AiInput, WiaAiError> {
        let text = self.process_bci_signal(&message.payload)?;

        let mut context = HashMap::new();

        // Extract BCI-specific metadata
        if let Some(paradigm) = message.metadata.get("paradigm") {
            context.insert("paradigm".into(), paradigm.clone());
        }
        if let Some(confidence) = message.metadata.get("confidence") {
            context.insert("confidence".into(), confidence.clone());
        }
        if let Some(mental_state) = message.metadata.get("mental_state") {
            context.insert("mental_state".into(), mental_state.clone());
        }
        if let Some(attention) = message.metadata.get("attention_level") {
            context.insert("attention_level".into(), attention.clone());
        }

        Ok(AiInput {
            input_type: AiInputType::Signal,
            text: Some(text),
            data: None,
            source_message: Some(message),
            context,
        })
    }

    fn can_handle(&self, message: &WiaMessage) -> bool {
        message.source == WiaStandardType::Bci
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_bci_classification_input() {
        let adapter = BciInputAdapter::new();

        let message = WiaMessage::text(WiaStandardType::Bci, WiaStandardType::Ai, "left_hand")
            .with_metadata("confidence", serde_json::json!(0.87))
            .with_metadata("paradigm", serde_json::json!("motor_imagery"));

        let input = adapter.to_ai_input(message).await.unwrap();
        assert_eq!(input.text, Some("left_hand".into()));
        assert!(input.context.contains_key("confidence"));
    }

    #[test]
    fn test_can_handle() {
        let adapter = BciInputAdapter::new();

        let bci_msg = WiaMessage::text(WiaStandardType::Bci, WiaStandardType::Ai, "test");
        let aac_msg = WiaMessage::text(WiaStandardType::Aac, WiaStandardType::Ai, "test");

        assert!(adapter.can_handle(&bci_msg));
        assert!(!adapter.can_handle(&aac_msg));
    }
}
