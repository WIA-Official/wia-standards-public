//! Mock implementations for testing WIA integration

use async_trait::async_trait;
use chrono::Utc;
use std::collections::HashMap;
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use tokio::sync::Mutex;
use tokio_stream::{Stream, StreamExt};

use crate::error::WiaAiError;
use super::{
    AiInput, AiInputType, AiOutput, AiOutputType, WiaMessage, WiaMessageType,
    WiaPayload, WiaStandardType,
    connector::{AiInputAdapter, AiOutputAdapter, WiaConnector},
};

/// Mock WIA Connector for testing
pub struct MockWiaConnector {
    id: String,
    standard_type: WiaStandardType,
    connected: AtomicBool,
    messages: Arc<Mutex<Vec<WiaMessage>>>,
    sent_messages: Arc<Mutex<Vec<WiaMessage>>>,
}

impl MockWiaConnector {
    /// Create a new mock connector
    pub fn new(id: impl Into<String>, standard_type: WiaStandardType) -> Self {
        Self {
            id: id.into(),
            standard_type,
            connected: AtomicBool::new(false),
            messages: Arc::new(Mutex::new(Vec::new())),
            sent_messages: Arc::new(Mutex::new(Vec::new())),
        }
    }

    /// Queue a message for receiving
    pub async fn queue_message(&self, message: WiaMessage) {
        let mut messages = self.messages.lock().await;
        messages.push(message);
    }

    /// Get sent messages
    pub async fn get_sent_messages(&self) -> Vec<WiaMessage> {
        let messages = self.sent_messages.lock().await;
        messages.clone()
    }

    /// Clear sent messages
    pub async fn clear_sent_messages(&self) {
        let mut messages = self.sent_messages.lock().await;
        messages.clear();
    }
}

#[async_trait]
impl WiaConnector for MockWiaConnector {
    fn id(&self) -> &str {
        &self.id
    }

    fn standard_type(&self) -> WiaStandardType {
        self.standard_type.clone()
    }

    fn is_connected(&self) -> bool {
        self.connected.load(Ordering::SeqCst)
    }

    async fn connect(&mut self) -> Result<(), WiaAiError> {
        self.connected.store(true, Ordering::SeqCst);
        Ok(())
    }

    async fn disconnect(&mut self) -> Result<(), WiaAiError> {
        self.connected.store(false, Ordering::SeqCst);
        Ok(())
    }

    async fn receive(&self) -> Result<WiaMessage, WiaAiError> {
        let mut messages = self.messages.lock().await;
        messages.pop().ok_or_else(|| WiaAiError::NotFound("No messages".into()))
    }

    async fn send(&self, message: WiaMessage) -> Result<(), WiaAiError> {
        let mut sent = self.sent_messages.lock().await;
        sent.push(message);
        Ok(())
    }

    fn receive_stream(&self) -> Pin<Box<dyn Stream<Item = Result<WiaMessage, WiaAiError>> + Send>> {
        let messages = self.messages.clone();
        Box::pin(async_stream::stream! {
            loop {
                let msg = {
                    let mut msgs = messages.lock().await;
                    msgs.pop()
                };
                match msg {
                    Some(m) => yield Ok(m),
                    None => {
                        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
                    }
                }
            }
        })
    }
}

/// Mock Input Adapter for testing
pub struct MockInputAdapter {
    name: String,
    standard_type: WiaStandardType,
}

impl MockInputAdapter {
    /// Create a new mock input adapter
    pub fn new(standard_type: WiaStandardType) -> Self {
        Self {
            name: format!("mock_{}_input", standard_type),
            standard_type,
        }
    }
}

#[async_trait]
impl AiInputAdapter for MockInputAdapter {
    fn name(&self) -> &str {
        &self.name
    }

    fn input_type(&self) -> AiInputType {
        AiInputType::Text
    }

    fn supported_standards(&self) -> Vec<WiaStandardType> {
        vec![self.standard_type.clone()]
    }

    async fn to_ai_input(&self, message: WiaMessage) -> Result<AiInput, WiaAiError> {
        let text = match &message.payload {
            WiaPayload::Text(t) => t.clone(),
            WiaPayload::Json(v) => v.to_string(),
            _ => "[binary data]".to_string(),
        };

        Ok(AiInput {
            input_type: AiInputType::Text,
            text: Some(text),
            data: None,
            source_message: Some(message),
            context: HashMap::new(),
        })
    }

    fn can_handle(&self, message: &WiaMessage) -> bool {
        message.source == self.standard_type
    }
}

/// Mock Output Adapter for testing
pub struct MockOutputAdapter {
    name: String,
    target_standard: WiaStandardType,
}

impl MockOutputAdapter {
    /// Create a new mock output adapter
    pub fn new(target_standard: WiaStandardType) -> Self {
        Self {
            name: format!("mock_{}_output", target_standard),
            target_standard,
        }
    }
}

#[async_trait]
impl AiOutputAdapter for MockOutputAdapter {
    fn name(&self) -> &str {
        &self.name
    }

    fn output_type(&self) -> AiOutputType {
        AiOutputType::Text
    }

    fn target_standard(&self) -> WiaStandardType {
        self.target_standard.clone()
    }

    async fn from_ai_output(&self, output: AiOutput) -> Result<WiaMessage, WiaAiError> {
        Ok(WiaMessage {
            id: uuid::Uuid::new_v4().to_string(),
            source: WiaStandardType::Ai,
            target: self.target_standard.clone(),
            message_type: WiaMessageType::Response,
            payload: WiaPayload::Text(output.text),
            metadata: output.metadata,
            timestamp: Utc::now(),
        })
    }
}

/// Create a test AAC message
pub fn create_test_aac_message(text: &str) -> WiaMessage {
    WiaMessage::text(WiaStandardType::Aac, WiaStandardType::Ai, text)
}

/// Create a test BCI message
pub fn create_test_bci_message(intent: &str) -> WiaMessage {
    WiaMessage::text(WiaStandardType::Bci, WiaStandardType::Ai, intent)
        .with_metadata("confidence", serde_json::json!(0.95))
}

/// Create a test Voice message
pub fn create_test_voice_message(transcript: &str) -> WiaMessage {
    WiaMessage::text(WiaStandardType::Voice, WiaStandardType::Ai, transcript)
        .with_metadata("language", serde_json::json!("en-US"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_mock_connector() {
        let mut connector = MockWiaConnector::new("test", WiaStandardType::Aac);

        assert!(!connector.is_connected());

        connector.connect().await.unwrap();
        assert!(connector.is_connected());

        // Queue and receive a message
        let msg = create_test_aac_message("Hello");
        connector.queue_message(msg.clone()).await;

        let received = connector.receive().await.unwrap();
        assert_eq!(received.source, WiaStandardType::Aac);

        // Send a message
        let response = WiaMessage::text(WiaStandardType::Ai, WiaStandardType::Aac, "Response");
        connector.send(response).await.unwrap();

        let sent = connector.get_sent_messages().await;
        assert_eq!(sent.len(), 1);

        connector.disconnect().await.unwrap();
        assert!(!connector.is_connected());
    }

    #[tokio::test]
    async fn test_mock_input_adapter() {
        let adapter = MockInputAdapter::new(WiaStandardType::Aac);

        assert_eq!(adapter.name(), "mock_aac_input");
        assert!(adapter.supported_standards().contains(&WiaStandardType::Aac));

        let message = create_test_aac_message("Test input");
        assert!(adapter.can_handle(&message));

        let input = adapter.to_ai_input(message).await.unwrap();
        assert_eq!(input.text, Some("Test input".to_string()));
    }

    #[tokio::test]
    async fn test_mock_output_adapter() {
        let adapter = MockOutputAdapter::new(WiaStandardType::Tts);

        assert_eq!(adapter.name(), "mock_tts_output");
        assert_eq!(adapter.target_standard(), WiaStandardType::Tts);

        let output = AiOutput::text("Response");
        let message = adapter.from_ai_output(output).await.unwrap();

        assert_eq!(message.target, WiaStandardType::Tts);
        assert!(matches!(message.payload, WiaPayload::Text(_)));
    }
}
