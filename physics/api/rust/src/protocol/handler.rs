//! WPP Message Handler
//!
//! Provides message routing and handling for the WIA Physics Protocol.

use std::sync::Arc;
use tokio::sync::mpsc;

use crate::error::{PhysicsError, PhysicsResult};
use super::messages::*;
use super::client::WppClient;

// ============================================================================
// Message Handler Trait
// ============================================================================

/// Trait for handling WPP messages
#[async_trait::async_trait]
pub trait WppMessageHandler: Send + Sync {
    /// Handle connect acknowledgment
    async fn on_connect_ack(&self, payload: ConnectAckPayload) -> PhysicsResult<()> {
        let _ = payload;
        Ok(())
    }

    /// Handle disconnect
    async fn on_disconnect(&self, payload: DisconnectPayload) -> PhysicsResult<()> {
        let _ = payload;
        Ok(())
    }

    /// Handle subscribe acknowledgment
    async fn on_subscribe_ack(&self, payload: SubscribeAckPayload) -> PhysicsResult<()> {
        let _ = payload;
        Ok(())
    }

    /// Handle unsubscribe acknowledgment
    async fn on_unsubscribe_ack(&self, payload: UnsubscribeAckPayload) -> PhysicsResult<()> {
        let _ = payload;
        Ok(())
    }

    /// Handle incoming data
    async fn on_data(&self, payload: DataPayload) -> PhysicsResult<()> {
        let _ = payload;
        Ok(())
    }

    /// Handle command response
    async fn on_response(&self, payload: ResponsePayload) -> PhysicsResult<()> {
        let _ = payload;
        Ok(())
    }

    /// Handle event
    async fn on_event(&self, payload: EventPayload) -> PhysicsResult<()> {
        let _ = payload;
        Ok(())
    }

    /// Handle error
    async fn on_error(&self, payload: ErrorPayload) -> PhysicsResult<()> {
        let _ = payload;
        Ok(())
    }

    /// Handle pong
    async fn on_pong(&self, payload: PongPayload) -> PhysicsResult<()> {
        let _ = payload;
        Ok(())
    }
}

// ============================================================================
// Message Router
// ============================================================================

/// Routes messages to appropriate handlers
pub struct MessageRouter {
    handler: Arc<dyn WppMessageHandler>,
}

impl MessageRouter {
    /// Create a new router with handler
    pub fn new<H: WppMessageHandler + 'static>(handler: H) -> Self {
        Self {
            handler: Arc::new(handler),
        }
    }

    /// Route a message to the appropriate handler
    pub async fn route(&self, message: WppMessage) -> PhysicsResult<()> {
        match message.msg_type {
            MessageType::ConnectAck => {
                if let MessagePayload::ConnectAck(payload) = message.payload {
                    self.handler.on_connect_ack(payload).await
                } else {
                    Err(PhysicsError::protocol("Invalid connect_ack payload"))
                }
            }
            MessageType::Disconnect => {
                if let MessagePayload::Disconnect(payload) = message.payload {
                    self.handler.on_disconnect(payload).await
                } else {
                    Err(PhysicsError::protocol("Invalid disconnect payload"))
                }
            }
            MessageType::SubscribeAck => {
                if let MessagePayload::SubscribeAck(payload) = message.payload {
                    self.handler.on_subscribe_ack(payload).await
                } else {
                    Err(PhysicsError::protocol("Invalid subscribe_ack payload"))
                }
            }
            MessageType::UnsubscribeAck => {
                if let MessagePayload::UnsubscribeAck(payload) = message.payload {
                    self.handler.on_unsubscribe_ack(payload).await
                } else {
                    Err(PhysicsError::protocol("Invalid unsubscribe_ack payload"))
                }
            }
            MessageType::Data => {
                if let MessagePayload::Data(payload) = message.payload {
                    self.handler.on_data(payload).await
                } else {
                    Err(PhysicsError::protocol("Invalid data payload"))
                }
            }
            MessageType::Response => {
                if let MessagePayload::Response(payload) = message.payload {
                    self.handler.on_response(payload).await
                } else {
                    Err(PhysicsError::protocol("Invalid response payload"))
                }
            }
            MessageType::Event => {
                if let MessagePayload::Event(payload) = message.payload {
                    self.handler.on_event(payload).await
                } else {
                    Err(PhysicsError::protocol("Invalid event payload"))
                }
            }
            MessageType::Error => {
                if let MessagePayload::Error(payload) = message.payload {
                    self.handler.on_error(payload).await
                } else {
                    Err(PhysicsError::protocol("Invalid error payload"))
                }
            }
            MessageType::Pong => {
                if let MessagePayload::Pong(payload) = message.payload {
                    self.handler.on_pong(payload).await
                } else {
                    Err(PhysicsError::protocol("Invalid pong payload"))
                }
            }
            // Client-originated messages should not be received by client
            MessageType::Connect
            | MessageType::Subscribe
            | MessageType::Unsubscribe
            | MessageType::Command
            | MessageType::Ping => {
                Err(PhysicsError::protocol("Unexpected message type for client"))
            }
        }
    }
}

// ============================================================================
// Default Handler
// ============================================================================

/// Default message handler that works with WppClient
pub struct DefaultHandler {
    client: Arc<WppClient>,
    data_sender: Option<mpsc::Sender<DataPayload>>,
    event_sender: Option<mpsc::Sender<EventPayload>>,
}

impl DefaultHandler {
    /// Create a new default handler
    pub fn new(client: Arc<WppClient>) -> Self {
        Self {
            client,
            data_sender: None,
            event_sender: None,
        }
    }

    /// Set data sender channel
    pub fn with_data_sender(mut self, sender: mpsc::Sender<DataPayload>) -> Self {
        self.data_sender = Some(sender);
        self
    }

    /// Set event sender channel
    pub fn with_event_sender(mut self, sender: mpsc::Sender<EventPayload>) -> Self {
        self.event_sender = Some(sender);
        self
    }
}

#[async_trait::async_trait]
impl WppMessageHandler for DefaultHandler {
    async fn on_connect_ack(&self, payload: ConnectAckPayload) -> PhysicsResult<()> {
        self.client.handle_connect_ack(payload).await
    }

    async fn on_subscribe_ack(&self, payload: SubscribeAckPayload) -> PhysicsResult<()> {
        self.client.handle_subscribe_ack(payload).await
    }

    async fn on_unsubscribe_ack(&self, payload: UnsubscribeAckPayload) -> PhysicsResult<()> {
        self.client.handle_unsubscribe_ack(payload).await
    }

    async fn on_data(&self, payload: DataPayload) -> PhysicsResult<()> {
        if let Some(sender) = &self.data_sender {
            sender
                .send(payload)
                .await
                .map_err(|_| PhysicsError::protocol("Data channel closed"))?;
        }
        Ok(())
    }

    async fn on_event(&self, payload: EventPayload) -> PhysicsResult<()> {
        if let Some(sender) = &self.event_sender {
            sender
                .send(payload)
                .await
                .map_err(|_| PhysicsError::protocol("Event channel closed"))?;
        }
        Ok(())
    }

    async fn on_error(&self, payload: ErrorPayload) -> PhysicsResult<()> {
        // Log error (in real implementation, use proper logging)
        eprintln!(
            "WPP Error {}: {} - {}",
            payload.code,
            format!("{:?}", payload.category),
            payload.message
        );
        Ok(())
    }
}

// ============================================================================
// Channel Matcher
// ============================================================================

/// Matches channel patterns (with wildcards)
pub struct ChannelMatcher;

impl ChannelMatcher {
    /// Check if a channel matches a pattern
    ///
    /// Supports wildcards:
    /// - `*` matches a single segment
    /// - `**` matches zero or more segments (at end only)
    ///
    /// # Examples
    ///
    /// ```
    /// use wia_physics::protocol::ChannelMatcher;
    ///
    /// assert!(ChannelMatcher::matches("fusion/iter/plasma", "fusion/iter/plasma"));
    /// assert!(ChannelMatcher::matches("fusion/iter/plasma", "fusion/*/plasma"));
    /// assert!(ChannelMatcher::matches("fusion/iter/plasma", "fusion/**"));
    /// assert!(!ChannelMatcher::matches("fusion/iter/plasma", "particle/**"));
    /// ```
    pub fn matches(channel: &str, pattern: &str) -> bool {
        let channel_parts: Vec<&str> = channel.split('/').collect();
        let pattern_parts: Vec<&str> = pattern.split('/').collect();

        Self::match_parts(&channel_parts, &pattern_parts)
    }

    fn match_parts(channel: &[&str], pattern: &[&str]) -> bool {
        if pattern.is_empty() {
            return channel.is_empty();
        }

        let first_pattern = pattern[0];

        if first_pattern == "**" {
            // ** at end matches everything
            if pattern.len() == 1 {
                return true;
            }
            // ** in middle: try matching rest of pattern at each position
            for i in 0..=channel.len() {
                if Self::match_parts(&channel[i..], &pattern[1..]) {
                    return true;
                }
            }
            return false;
        }

        if channel.is_empty() {
            return false;
        }

        let first_channel = channel[0];

        if first_pattern == "*" || first_pattern == first_channel {
            Self::match_parts(&channel[1..], &pattern[1..])
        } else {
            false
        }
    }

    /// Find all channels matching a pattern
    pub fn find_matching(channels: &[String], pattern: &str) -> Vec<String> {
        channels
            .iter()
            .filter(|c| Self::matches(c, pattern))
            .cloned()
            .collect()
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_channel_matcher_exact() {
        assert!(ChannelMatcher::matches("fusion/iter/plasma", "fusion/iter/plasma"));
        assert!(!ChannelMatcher::matches("fusion/iter/plasma", "fusion/iter/magnets"));
    }

    #[test]
    fn test_channel_matcher_single_wildcard() {
        assert!(ChannelMatcher::matches("fusion/iter/plasma", "fusion/*/plasma"));
        assert!(ChannelMatcher::matches("fusion/jet/plasma", "fusion/*/plasma"));
        assert!(!ChannelMatcher::matches("fusion/iter/magnets", "fusion/*/plasma"));
    }

    #[test]
    fn test_channel_matcher_double_wildcard() {
        assert!(ChannelMatcher::matches("fusion/iter/plasma", "fusion/**"));
        assert!(ChannelMatcher::matches("fusion/iter/plasma/temp", "fusion/**"));
        assert!(ChannelMatcher::matches("fusion", "fusion/**"));
        assert!(!ChannelMatcher::matches("particle/lhc/atlas", "fusion/**"));
    }

    #[test]
    fn test_find_matching() {
        let channels = vec![
            "fusion/iter/plasma".to_string(),
            "fusion/jet/plasma".to_string(),
            "fusion/iter/magnets".to_string(),
            "particle/lhc/atlas".to_string(),
        ];

        let matched = ChannelMatcher::find_matching(&channels, "fusion/*/plasma");
        assert_eq!(matched.len(), 2);
        assert!(matched.contains(&"fusion/iter/plasma".to_string()));
        assert!(matched.contains(&"fusion/jet/plasma".to_string()));
    }

    struct TestHandler {
        data_received: std::sync::atomic::AtomicBool,
    }

    impl TestHandler {
        fn new() -> Self {
            Self {
                data_received: std::sync::atomic::AtomicBool::new(false),
            }
        }
    }

    #[async_trait::async_trait]
    impl WppMessageHandler for TestHandler {
        async fn on_data(&self, _payload: DataPayload) -> PhysicsResult<()> {
            self.data_received
                .store(true, std::sync::atomic::Ordering::SeqCst);
            Ok(())
        }
    }

    #[tokio::test]
    async fn test_message_router() {
        let handler = TestHandler::new();
        let router = MessageRouter::new(handler);

        let msg = WppMessage::new(
            MessageType::Data,
            MessagePayload::Data(DataPayload {
                channel: "fusion/iter/plasma".to_string(),
                sequence: Some(1),
                data_type: DataType::Fusion,
                data: Some(serde_json::json!({"test": true})),
                batch: None,
                sequence_start: None,
                count: None,
                items: None,
            }),
        );

        router.route(msg).await.unwrap();
    }
}
