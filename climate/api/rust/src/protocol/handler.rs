//! Message handler traits and implementations

use async_trait::async_trait;

use super::message::ProtocolMessage;
use super::message_types::*;
use crate::ClimateMessage;
use crate::error::Result;

/// Trait for handling incoming protocol messages
///
/// Implement this trait to create custom message handlers for
/// processing different types of protocol messages.
#[async_trait]
pub trait MessageHandler: Send + Sync {
    /// Handle a connect request
    ///
    /// Returns a ConnectAckPayload with success or failure status.
    async fn on_connect(&self, payload: &ConnectPayload) -> Result<ConnectAckPayload>;

    /// Handle a disconnect notification
    async fn on_disconnect(&self, payload: Option<&DisconnectPayload>) -> Result<()>;

    /// Handle incoming climate data
    async fn on_data(&self, message: &ClimateMessage) -> Result<()>;

    /// Handle a command request
    ///
    /// Returns a CommandAckPayload with execution result.
    async fn on_command(&self, payload: &CommandPayload) -> Result<CommandAckPayload>;

    /// Handle a subscribe request
    ///
    /// Returns a SubscribeAckPayload with subscription confirmation.
    async fn on_subscribe(&self, payload: &SubscribePayload) -> Result<SubscribeAckPayload>;

    /// Handle an unsubscribe request
    async fn on_unsubscribe(&self, payload: &UnsubscribePayload) -> Result<()>;

    /// Handle an error message
    async fn on_error(&self, payload: &ErrorPayload) -> Result<()>;

    /// Handle a ping message
    ///
    /// Default implementation returns a pong message.
    async fn on_ping(&self) -> Result<ProtocolMessage> {
        Ok(ProtocolMessage::pong())
    }

    /// Handle a pong message
    ///
    /// Default implementation does nothing.
    async fn on_pong(&self) -> Result<()> {
        Ok(())
    }
}

/// Default no-op message handler
///
/// This handler accepts all messages but performs no action.
/// Useful as a base for selective handler implementations.
pub struct NoOpHandler;

#[async_trait]
impl MessageHandler for NoOpHandler {
    async fn on_connect(&self, _payload: &ConnectPayload) -> Result<ConnectAckPayload> {
        Ok(ConnectAckPayload::success("noop-session".to_string()))
    }

    async fn on_disconnect(&self, _payload: Option<&DisconnectPayload>) -> Result<()> {
        Ok(())
    }

    async fn on_data(&self, _message: &ClimateMessage) -> Result<()> {
        Ok(())
    }

    async fn on_command(&self, payload: &CommandPayload) -> Result<CommandAckPayload> {
        Ok(CommandAckPayload::success(
            payload.target_id.clone(),
            None,
        ))
    }

    async fn on_subscribe(&self, payload: &SubscribePayload) -> Result<SubscribeAckPayload> {
        let topics: Vec<String> = payload.topics.iter().map(|t| t.pattern.clone()).collect();
        Ok(SubscribeAckPayload::success(
            uuid::Uuid::new_v4().to_string(),
            topics,
        ))
    }

    async fn on_unsubscribe(&self, _payload: &UnsubscribePayload) -> Result<()> {
        Ok(())
    }

    async fn on_error(&self, _payload: &ErrorPayload) -> Result<()> {
        Ok(())
    }
}

/// Logging message handler wrapper
///
/// Wraps another handler and logs all incoming messages.
pub struct LoggingHandler<H: MessageHandler> {
    inner: H,
}

impl<H: MessageHandler> LoggingHandler<H> {
    /// Create a new logging handler wrapping the given handler
    pub fn new(inner: H) -> Self {
        Self { inner }
    }
}

#[async_trait]
impl<H: MessageHandler + 'static> MessageHandler for LoggingHandler<H> {
    async fn on_connect(&self, payload: &ConnectPayload) -> Result<ConnectAckPayload> {
        tracing::info!(client_id = %payload.client_id, "Connect request received");
        let result = self.inner.on_connect(payload).await;
        match &result {
            Ok(ack) => tracing::info!(success = ack.success, "Connect response sent"),
            Err(e) => tracing::error!(error = %e, "Connect handler failed"),
        }
        result
    }

    async fn on_disconnect(&self, payload: Option<&DisconnectPayload>) -> Result<()> {
        let reason = payload.and_then(|p| p.reason);
        tracing::info!(reason = ?reason, "Disconnect received");
        self.inner.on_disconnect(payload).await
    }

    async fn on_data(&self, message: &ClimateMessage) -> Result<()> {
        tracing::debug!(data_type = ?message.data_type, "Data received");
        self.inner.on_data(message).await
    }

    async fn on_command(&self, payload: &CommandPayload) -> Result<CommandAckPayload> {
        tracing::info!(
            target_id = %payload.target_id,
            action = %payload.action,
            "Command received"
        );
        let result = self.inner.on_command(payload).await;
        match &result {
            Ok(ack) => tracing::info!(success = ack.success, "Command response sent"),
            Err(e) => tracing::error!(error = %e, "Command handler failed"),
        }
        result
    }

    async fn on_subscribe(&self, payload: &SubscribePayload) -> Result<SubscribeAckPayload> {
        let topic_count = payload.topics.len();
        tracing::info!(topic_count, qos = ?payload.qos, "Subscribe request received");
        self.inner.on_subscribe(payload).await
    }

    async fn on_unsubscribe(&self, payload: &UnsubscribePayload) -> Result<()> {
        tracing::info!(subscription_id = %payload.subscription_id, "Unsubscribe received");
        self.inner.on_unsubscribe(payload).await
    }

    async fn on_error(&self, payload: &ErrorPayload) -> Result<()> {
        tracing::warn!(
            code = %payload.code,
            message = %payload.message,
            "Error received"
        );
        self.inner.on_error(payload).await
    }

    async fn on_ping(&self) -> Result<ProtocolMessage> {
        tracing::trace!("Ping received");
        self.inner.on_ping().await
    }

    async fn on_pong(&self) -> Result<()> {
        tracing::trace!("Pong received");
        self.inner.on_pong().await
    }
}

/// Message dispatcher for routing messages to handlers
pub struct MessageDispatcher<H: MessageHandler> {
    handler: H,
}

impl<H: MessageHandler> MessageDispatcher<H> {
    /// Create a new message dispatcher with the given handler
    pub fn new(handler: H) -> Self {
        Self { handler }
    }

    /// Dispatch a protocol message to the appropriate handler method
    pub async fn dispatch(&self, message: &ProtocolMessage) -> Result<Option<ProtocolMessage>> {
        use super::message::MessageType;

        match message.message_type {
            MessageType::Connect => {
                if let Some(payload) = message.get_payload::<ConnectPayload>() {
                    let payload = payload.map_err(|e| crate::error::ClimateError::SerializationError(e.to_string()))?;
                    let ack = self.handler.on_connect(&payload).await?;
                    let response = ProtocolMessage::connect_ack(ack)
                        .map_err(|e| crate::error::ClimateError::SerializationError(e.to_string()))?;
                    return Ok(Some(response));
                }
            }
            MessageType::Disconnect => {
                let payload = message.get_payload::<DisconnectPayload>()
                    .transpose()
                    .map_err(|e| crate::error::ClimateError::SerializationError(e.to_string()))?;
                self.handler.on_disconnect(payload.as_ref()).await?;
            }
            MessageType::Data => {
                if let Some(payload) = message.get_payload::<ClimateMessage>() {
                    let data = payload.map_err(|e| crate::error::ClimateError::SerializationError(e.to_string()))?;
                    self.handler.on_data(&data).await?;
                }
            }
            MessageType::Command => {
                if let Some(payload) = message.get_payload::<CommandPayload>() {
                    let payload = payload.map_err(|e| crate::error::ClimateError::SerializationError(e.to_string()))?;
                    let ack = self.handler.on_command(&payload).await?;
                    let response = ProtocolMessage::command_ack(ack)
                        .map_err(|e| crate::error::ClimateError::SerializationError(e.to_string()))?;
                    return Ok(Some(response));
                }
            }
            MessageType::Subscribe => {
                if let Some(payload) = message.get_payload::<SubscribePayload>() {
                    let payload = payload.map_err(|e| crate::error::ClimateError::SerializationError(e.to_string()))?;
                    let ack = self.handler.on_subscribe(&payload).await?;
                    let response = ProtocolMessage::subscribe_ack(ack)
                        .map_err(|e| crate::error::ClimateError::SerializationError(e.to_string()))?;
                    return Ok(Some(response));
                }
            }
            MessageType::Unsubscribe => {
                if let Some(payload) = message.get_payload::<UnsubscribePayload>() {
                    let payload = payload.map_err(|e| crate::error::ClimateError::SerializationError(e.to_string()))?;
                    self.handler.on_unsubscribe(&payload).await?;
                }
            }
            MessageType::Error => {
                if let Some(payload) = message.get_payload::<ErrorPayload>() {
                    let payload = payload.map_err(|e| crate::error::ClimateError::SerializationError(e.to_string()))?;
                    self.handler.on_error(&payload).await?;
                }
            }
            MessageType::Ping => {
                let response = self.handler.on_ping().await?;
                return Ok(Some(response));
            }
            MessageType::Pong => {
                self.handler.on_pong().await?;
            }
            MessageType::ConnectAck | MessageType::CommandAck | MessageType::SubscribeAck => {
                // Response messages - typically not dispatched to handlers
                tracing::debug!(msg_type = %message.message_type, "Received response message");
            }
        }

        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_noop_handler() {
        let handler = NoOpHandler;

        let payload = ConnectPayload {
            client_id: "test".to_string(),
            client_type: None,
            capabilities: None,
            auth: None,
            metadata: None,
        };

        let result = handler.on_connect(&payload).await.unwrap();
        assert!(result.success);
    }

    #[tokio::test]
    async fn test_dispatcher_ping_pong() {
        let handler = NoOpHandler;
        let dispatcher = MessageDispatcher::new(handler);

        let ping = ProtocolMessage::ping();
        let response = dispatcher.dispatch(&ping).await.unwrap();

        assert!(response.is_some());
        let pong = response.unwrap();
        assert_eq!(pong.message_type, super::super::message::MessageType::Pong);
    }
}
