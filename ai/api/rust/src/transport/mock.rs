//! Mock Transport for Testing
//!
//! This module provides a mock transport implementation for testing purposes.

use async_trait::async_trait;
use std::collections::VecDeque;
use std::sync::Arc;
use tokio::sync::RwLock;

use super::{StreamingTransport, Transport};
use crate::protocol::{ProtocolError, ProtocolMessage, ProtocolResult};

/// Mock transport for testing
pub struct MockTransport {
    /// Queue of responses to return
    responses: Arc<RwLock<VecDeque<ProtocolMessage>>>,

    /// Sent messages (for verification)
    sent_messages: Arc<RwLock<Vec<ProtocolMessage>>>,

    /// Whether the transport is connected
    connected: Arc<RwLock<bool>>,

    /// Error to return on next send (for error testing)
    next_error: Arc<RwLock<Option<ProtocolError>>>,
}

impl MockTransport {
    /// Create a new mock transport
    pub fn new() -> Self {
        Self {
            responses: Arc::new(RwLock::new(VecDeque::new())),
            sent_messages: Arc::new(RwLock::new(Vec::new())),
            connected: Arc::new(RwLock::new(true)),
            next_error: Arc::new(RwLock::new(None)),
        }
    }

    /// Queue a response to be returned
    pub async fn queue_response(&self, message: ProtocolMessage) {
        self.responses.write().await.push_back(message);
    }

    /// Queue multiple responses
    pub async fn queue_responses(&self, messages: Vec<ProtocolMessage>) {
        let mut responses = self.responses.write().await;
        for message in messages {
            responses.push_back(message);
        }
    }

    /// Get all sent messages
    pub async fn sent_messages(&self) -> Vec<ProtocolMessage> {
        self.sent_messages.read().await.clone()
    }

    /// Get the last sent message
    pub async fn last_sent(&self) -> Option<ProtocolMessage> {
        self.sent_messages.read().await.last().cloned()
    }

    /// Clear sent messages
    pub async fn clear_sent(&self) {
        self.sent_messages.write().await.clear();
    }

    /// Set the next error to return
    pub async fn set_next_error(&self, error: ProtocolError) {
        *self.next_error.write().await = Some(error);
    }

    /// Set connection state
    pub async fn set_connected(&self, connected: bool) {
        *self.connected.write().await = connected;
    }
}

impl Default for MockTransport {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl Transport for MockTransport {
    async fn send(&self, message: ProtocolMessage) -> ProtocolResult<ProtocolMessage> {
        // Check for queued error
        if let Some(error) = self.next_error.write().await.take() {
            return Err(error);
        }

        // Check if connected
        if !*self.connected.read().await {
            return Err(ProtocolError::connection_lost("Not connected"));
        }

        // Record the sent message
        self.sent_messages.write().await.push(message);

        // Return queued response or error
        self.responses
            .write()
            .await
            .pop_front()
            .ok_or_else(|| ProtocolError::validation_error("No response queued"))
    }

    async fn send_no_response(&self, message: ProtocolMessage) -> ProtocolResult<()> {
        // Check for queued error
        if let Some(error) = self.next_error.write().await.take() {
            return Err(error);
        }

        // Check if connected
        if !*self.connected.read().await {
            return Err(ProtocolError::connection_lost("Not connected"));
        }

        // Record the sent message
        self.sent_messages.write().await.push(message);

        Ok(())
    }

    async fn is_connected(&self) -> bool {
        *self.connected.read().await
    }

    async fn close(&self) -> ProtocolResult<()> {
        *self.connected.write().await = false;
        Ok(())
    }
}

#[async_trait]
impl StreamingTransport for MockTransport {
    async fn send_streaming(
        &self,
        message: ProtocolMessage,
        callback: Box<dyn Fn(ProtocolMessage) + Send + Sync>,
    ) -> ProtocolResult<()> {
        // Check if connected
        if !*self.connected.read().await {
            return Err(ProtocolError::connection_lost("Not connected"));
        }

        // Record the sent message
        self.sent_messages.write().await.push(message);

        // Send all queued responses through callback
        let responses: Vec<ProtocolMessage> =
            self.responses.write().await.drain(..).collect();

        for response in responses {
            callback(response);
        }

        Ok(())
    }
}

/// Builder for creating mock transport scenarios
pub struct MockScenarioBuilder {
    transport: MockTransport,
}

impl MockScenarioBuilder {
    /// Create a new scenario builder
    pub fn new() -> Self {
        Self {
            transport: MockTransport::new(),
        }
    }

    /// Add a ping-pong exchange
    pub async fn with_ping_pong(self) -> Self {
        self.transport
            .queue_response(ProtocolMessage::pong("mock-ping"))
            .await;
        self
    }

    /// Add a connect-ack exchange
    pub async fn with_connect_ack(self, session_id: &str) -> Self {
        use crate::protocol::{ConnectAckPayload, ProtocolMessageBuilder, MessageType};

        let ack = ProtocolMessageBuilder::new()
            .message_type(MessageType::ConnectAck)
            .typed_payload(&ConnectAckPayload {
                session_id: session_id.to_string(),
                capabilities: Some(vec!["streaming".to_string(), "tools".to_string()]),
                timeout_seconds: Some(3600),
            })
            .unwrap()
            .build()
            .unwrap();

        self.transport.queue_response(ack).await;
        self
    }

    /// Add streaming responses
    pub async fn with_streaming_response(self, text: &str) -> Self {
        use crate::protocol::{
            Delta, DeltaType, MessageType, ProtocolMessageBuilder,
            StreamDeltaPayload, StreamEndPayload, StreamStartPayload, StopReason, Usage,
        };

        // Stream start
        let start = ProtocolMessageBuilder::new()
            .message_type(MessageType::StreamStart)
            .typed_payload(&StreamStartPayload {
                stream_id: "mock-stream".to_string(),
                model: Some("mock-model".to_string()),
                input_tokens: Some(10),
            })
            .unwrap()
            .build()
            .unwrap();

        self.transport.queue_response(start).await;

        // Stream deltas (split text into chunks)
        for (i, chunk) in text.chars().collect::<Vec<_>>().chunks(5).enumerate() {
            let delta = ProtocolMessageBuilder::new()
                .message_type(MessageType::StreamDelta)
                .typed_payload(&StreamDeltaPayload {
                    stream_id: "mock-stream".to_string(),
                    index: Some(i as u32),
                    delta: Delta {
                        delta_type: DeltaType::TextDelta,
                        text: Some(chunk.iter().collect()),
                        partial_json: None,
                    },
                })
                .unwrap()
                .build()
                .unwrap();

            self.transport.queue_response(delta).await;
        }

        // Stream end
        let end = ProtocolMessageBuilder::new()
            .message_type(MessageType::StreamEnd)
            .typed_payload(&StreamEndPayload {
                stream_id: "mock-stream".to_string(),
                stop_reason: Some(StopReason::EndTurn),
                usage: Some(Usage {
                    input_tokens: Some(10),
                    output_tokens: Some(text.len() as u32),
                    total_tokens: Some(10 + text.len() as u32),
                    ..Default::default()
                }),
            })
            .unwrap()
            .build()
            .unwrap();

        self.transport.queue_response(end).await;

        self
    }

    /// Add a tool call response
    pub async fn with_tool_result(self, call_id: &str, result: serde_json::Value) -> Self {
        use crate::protocol::{
            MessageType, ProtocolMessageBuilder, ToolResultPayload, ToolResultStatus,
        };

        let response = ProtocolMessageBuilder::new()
            .message_type(MessageType::ToolResult)
            .typed_payload(&ToolResultPayload {
                call_id: call_id.to_string(),
                status: ToolResultStatus::Success,
                result: Some(result),
                error: None,
                execution_time_ms: Some(100),
            })
            .unwrap()
            .build()
            .unwrap();

        self.transport.queue_response(response).await;
        self
    }

    /// Set transport as disconnected
    pub async fn disconnected(self) -> Self {
        self.transport.set_connected(false).await;
        self
    }

    /// Build the mock transport
    pub fn build(self) -> MockTransport {
        self.transport
    }
}

impl Default for MockScenarioBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::MessageType;

    #[tokio::test]
    async fn test_mock_transport_basic() {
        let transport = MockTransport::new();

        transport
            .queue_response(ProtocolMessage::pong("test"))
            .await;

        let ping = ProtocolMessage::ping();
        let response = transport.send(ping).await.unwrap();

        assert_eq!(response.message_type, MessageType::Pong);

        let sent = transport.sent_messages().await;
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].message_type, MessageType::Ping);
    }

    #[tokio::test]
    async fn test_mock_transport_error() {
        let transport = MockTransport::new();

        transport
            .set_next_error(ProtocolError::agent_not_found("agent-001"))
            .await;

        let result = transport.send(ProtocolMessage::ping()).await;
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_mock_transport_disconnected() {
        let transport = MockTransport::new();
        transport.set_connected(false).await;

        let result = transport.send(ProtocolMessage::ping()).await;
        assert!(result.is_err());
    }

    #[tokio::test]
    async fn test_scenario_builder() {
        let transport = MockScenarioBuilder::new()
            .with_ping_pong()
            .await
            .build();

        let response = transport.send(ProtocolMessage::ping()).await.unwrap();
        assert_eq!(response.message_type, MessageType::Pong);
    }

    #[tokio::test]
    async fn test_streaming_transport() {
        use std::sync::atomic::{AtomicUsize, Ordering};

        let transport = MockScenarioBuilder::new()
            .with_streaming_response("Hello World!")
            .await
            .build();

        let count = Arc::new(AtomicUsize::new(0));
        let count_clone = Arc::clone(&count);

        transport
            .send_streaming(
                ProtocolMessage::ping(),
                Box::new(move |_msg| {
                    count_clone.fetch_add(1, Ordering::SeqCst);
                }),
            )
            .await
            .unwrap();

        // Should have received start + deltas + end
        assert!(count.load(Ordering::SeqCst) >= 3);
    }
}
