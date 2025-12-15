//! Protocol message builder for fluent API construction

use super::message::{MessageMeta, MessageType, ProtocolMessage};
use super::message_types::*;
use super::{PROTOCOL_ID, PROTOCOL_VERSION};
use chrono::Utc;
use uuid::Uuid;
use std::collections::HashMap;

/// Builder for constructing protocol messages with a fluent API
#[derive(Debug, Clone, Default)]
pub struct ProtocolMessageBuilder {
    message_type: Option<MessageType>,
    payload: Option<serde_json::Value>,
    meta: Option<MessageMeta>,
    correlation_id: Option<String>,
}

impl ProtocolMessageBuilder {
    /// Create a new protocol message builder
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the message type
    pub fn message_type(mut self, msg_type: MessageType) -> Self {
        self.message_type = Some(msg_type);
        self
    }

    /// Set the payload from a serializable value
    pub fn payload<T: serde::Serialize>(mut self, payload: T) -> Result<Self, serde_json::Error> {
        self.payload = Some(serde_json::to_value(payload)?);
        Ok(self)
    }

    /// Set the correlation ID for request tracing
    pub fn correlation_id(mut self, id: impl Into<String>) -> Self {
        self.correlation_id = Some(id.into());
        self
    }

    /// Set message metadata
    pub fn meta(mut self, meta: MessageMeta) -> Self {
        self.meta = Some(meta);
        self
    }

    /// Build the protocol message
    pub fn build(self) -> Result<ProtocolMessage, &'static str> {
        let message_type = self.message_type.ok_or("message_type is required")?;

        let mut meta = self.meta;
        if let Some(corr_id) = self.correlation_id {
            let m = meta.get_or_insert_with(MessageMeta::default);
            m.correlation_id = Some(corr_id);
        }

        Ok(ProtocolMessage {
            protocol: PROTOCOL_ID.to_string(),
            version: PROTOCOL_VERSION.to_string(),
            message_id: Uuid::new_v4().to_string(),
            timestamp: Utc::now().timestamp_millis(),
            message_type,
            payload: self.payload,
            meta,
        })
    }
}

/// Builder for connect messages
#[derive(Debug, Clone, Default)]
pub struct ConnectBuilder {
    client_id: Option<String>,
    client_type: Option<ClientType>,
    capabilities: Vec<String>,
    auth: Option<AuthInfo>,
    metadata: HashMap<String, serde_json::Value>,
}

impl ConnectBuilder {
    /// Create a new connect message builder
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the client ID (required)
    pub fn client_id(mut self, id: impl Into<String>) -> Self {
        self.client_id = Some(id.into());
        self
    }

    /// Set the client type
    pub fn client_type(mut self, client_type: ClientType) -> Self {
        self.client_type = Some(client_type);
        self
    }

    /// Add a capability
    pub fn capability(mut self, capability: impl Into<String>) -> Self {
        self.capabilities.push(capability.into());
        self
    }

    /// Set multiple capabilities
    pub fn capabilities<I, S>(mut self, capabilities: I) -> Self
    where
        I: IntoIterator<Item = S>,
        S: Into<String>,
    {
        self.capabilities.extend(capabilities.into_iter().map(Into::into));
        self
    }

    /// Set API key authentication
    pub fn api_key(mut self, key: impl Into<String>) -> Self {
        self.auth = Some(AuthInfo {
            method: AuthMethod::ApiKey,
            token: Some(key.into()),
        });
        self
    }

    /// Set JWT authentication
    pub fn jwt(mut self, token: impl Into<String>) -> Self {
        self.auth = Some(AuthInfo {
            method: AuthMethod::Jwt,
            token: Some(token.into()),
        });
        self
    }

    /// Add metadata
    pub fn metadata(mut self, key: impl Into<String>, value: serde_json::Value) -> Self {
        self.metadata.insert(key.into(), value);
        self
    }

    /// Build the connect message
    pub fn build(self) -> Result<ProtocolMessage, &'static str> {
        let client_id = self.client_id.ok_or("client_id is required")?;

        let payload = ConnectPayload {
            client_id,
            client_type: self.client_type,
            capabilities: if self.capabilities.is_empty() {
                None
            } else {
                Some(self.capabilities)
            },
            auth: self.auth,
            metadata: if self.metadata.is_empty() {
                None
            } else {
                Some(self.metadata)
            },
        };

        ProtocolMessage::connect(payload).map_err(|_| "Failed to serialize payload")
    }
}

/// Builder for command messages
#[derive(Debug, Clone, Default)]
pub struct CommandBuilder {
    target_id: Option<String>,
    action: Option<String>,
    parameters: HashMap<String, serde_json::Value>,
    timeout: Option<u64>,
}

impl CommandBuilder {
    /// Create a new command builder
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the target device/service ID (required)
    pub fn target(mut self, id: impl Into<String>) -> Self {
        self.target_id = Some(id.into());
        self
    }

    /// Set the action name (required)
    pub fn action(mut self, action: impl Into<String>) -> Self {
        self.action = Some(action.into());
        self
    }

    /// Add a parameter
    pub fn param<T: serde::Serialize>(mut self, key: impl Into<String>, value: T) -> Self {
        if let Ok(v) = serde_json::to_value(value) {
            self.parameters.insert(key.into(), v);
        }
        self
    }

    /// Set timeout in milliseconds
    pub fn timeout(mut self, ms: u64) -> Self {
        self.timeout = Some(ms);
        self
    }

    /// Build the command message
    pub fn build(self) -> Result<ProtocolMessage, &'static str> {
        let target_id = self.target_id.ok_or("target_id is required")?;
        let action = self.action.ok_or("action is required")?;

        let payload = CommandPayload {
            target_id,
            action,
            parameters: if self.parameters.is_empty() {
                None
            } else {
                Some(self.parameters)
            },
            timeout: self.timeout,
        };

        ProtocolMessage::command(payload).map_err(|_| "Failed to serialize payload")
    }
}

/// Builder for subscribe messages
#[derive(Debug, Clone, Default)]
pub struct SubscribeBuilder {
    topics: Vec<TopicSubscription>,
    qos: QoS,
}

impl SubscribeBuilder {
    /// Create a new subscribe builder
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a topic to subscribe to
    pub fn topic(mut self, pattern: impl Into<String>) -> Self {
        self.topics.push(TopicSubscription {
            pattern: pattern.into(),
            filter: None,
        });
        self
    }

    /// Add a topic with filter
    pub fn topic_with_filter(
        mut self,
        pattern: impl Into<String>,
        filter: HashMap<String, serde_json::Value>,
    ) -> Self {
        self.topics.push(TopicSubscription {
            pattern: pattern.into(),
            filter: Some(filter),
        });
        self
    }

    /// Set QoS level
    pub fn qos(mut self, qos: QoS) -> Self {
        self.qos = qos;
        self
    }

    /// Build the subscribe message
    pub fn build(self) -> Result<ProtocolMessage, &'static str> {
        if self.topics.is_empty() {
            return Err("At least one topic is required");
        }

        let payload = SubscribePayload {
            topics: self.topics,
            qos: self.qos,
        };

        ProtocolMessage::subscribe(payload).map_err(|_| "Failed to serialize payload")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_connect_builder() {
        let msg = ConnectBuilder::new()
            .client_id("test-client")
            .client_type(ClientType::Sensor)
            .capability("carbon_capture")
            .capability("vertical_farming")
            .api_key("my-api-key")
            .build()
            .unwrap();

        assert_eq!(msg.message_type, MessageType::Connect);

        let payload: ConnectPayload = msg.get_payload().unwrap().unwrap();
        assert_eq!(payload.client_id, "test-client");
        assert_eq!(payload.client_type, Some(ClientType::Sensor));
        assert_eq!(payload.capabilities.unwrap().len(), 2);
    }

    #[test]
    fn test_command_builder() {
        let msg = CommandBuilder::new()
            .target("device-001")
            .action("set_capture_rate")
            .param("rate", 150.0)
            .timeout(5000)
            .build()
            .unwrap();

        assert_eq!(msg.message_type, MessageType::Command);

        let payload: CommandPayload = msg.get_payload().unwrap().unwrap();
        assert_eq!(payload.target_id, "device-001");
        assert_eq!(payload.action, "set_capture_rate");
        assert_eq!(payload.timeout, Some(5000));
    }

    #[test]
    fn test_subscribe_builder() {
        let msg = SubscribeBuilder::new()
            .topic("carbon_capture/*")
            .topic("vertical_farming/sensors/*")
            .qos(QoS::AtLeastOnce)
            .build()
            .unwrap();

        assert_eq!(msg.message_type, MessageType::Subscribe);

        let payload: SubscribePayload = msg.get_payload().unwrap().unwrap();
        assert_eq!(payload.topics.len(), 2);
        assert_eq!(payload.qos, QoS::AtLeastOnce);
    }

    #[test]
    fn test_subscribe_builder_requires_topic() {
        let result = SubscribeBuilder::new().build();
        assert!(result.is_err());
    }
}
