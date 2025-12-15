//! Message builder for WIA Bio Protocol

use crate::types::{Sequence, CrisprExperiment, ProteinStructure, BioPart};
use super::message::*;
use chrono::Utc;
use uuid::Uuid;

/// Builder for constructing protocol messages
pub struct MessageBuilder {
    message_type: MessageType,
    sequence: Option<u64>,
    capabilities: Vec<ProtocolCapability>,
    options: Option<ConnectionOptions>,
    payload_data: PayloadData,
}

/// Internal payload data during building
enum PayloadData {
    Connect {
        client_id: String,
        client_name: String,
        client_version: Option<String>,
    },
    ConnectAck {
        session_id: String,
        status: ConnectionStatus,
        server_version: Option<String>,
        timeout: Option<u64>,
    },
    Disconnect {
        reason_code: u32,
        reason: Option<String>,
        reconnect_allowed: bool,
    },
    Sequence {
        sequence: Sequence,
        source_id: Option<String>,
        batch_id: Option<String>,
        batch_index: Option<u32>,
    },
    Experiment {
        experiment: CrisprExperiment,
        is_update: bool,
    },
    Structure {
        structure: ProteinStructure,
        job_id: Option<String>,
    },
    Part {
        part: BioPart,
        registry: Option<String>,
    },
    Command {
        command: BioCommand,
        params: serde_json::Value,
        correlation_id: Option<String>,
    },
    CommandAck {
        command: BioCommand,
        status: CommandStatus,
        correlation_id: Option<String>,
        result: Option<serde_json::Value>,
        error: Option<String>,
    },
    Error {
        code: u32,
        name: String,
        message: String,
        details: Option<serde_json::Value>,
        recoverable: bool,
    },
    Heartbeat {
        client_timestamp: i64,
        server_timestamp: Option<i64>,
    },
    Subscribe {
        topic: SubscriptionTopic,
        filter: Option<serde_json::Value>,
    },
    Empty,
}

impl MessageBuilder {
    /// Start building a connect message
    pub fn connect(client_id: impl Into<String>, client_name: impl Into<String>) -> Self {
        Self {
            message_type: MessageType::Connect,
            sequence: None,
            capabilities: Vec::new(),
            options: None,
            payload_data: PayloadData::Connect {
                client_id: client_id.into(),
                client_name: client_name.into(),
                client_version: None,
            },
        }
    }

    /// Start building a connect_ack message
    pub fn connect_ack(session_id: impl Into<String>, status: ConnectionStatus) -> Self {
        Self {
            message_type: MessageType::ConnectAck,
            sequence: None,
            capabilities: Vec::new(),
            options: None,
            payload_data: PayloadData::ConnectAck {
                session_id: session_id.into(),
                status,
                server_version: None,
                timeout: None,
            },
        }
    }

    /// Start building a disconnect message
    pub fn disconnect(reason_code: u32) -> Self {
        Self {
            message_type: MessageType::Disconnect,
            sequence: None,
            capabilities: Vec::new(),
            options: None,
            payload_data: PayloadData::Disconnect {
                reason_code,
                reason: None,
                reconnect_allowed: false,
            },
        }
    }

    /// Start building a sequence data message
    pub fn sequence(sequence: Sequence) -> Self {
        Self {
            message_type: MessageType::Sequence,
            sequence: None,
            capabilities: Vec::new(),
            options: None,
            payload_data: PayloadData::Sequence {
                sequence,
                source_id: None,
                batch_id: None,
                batch_index: None,
            },
        }
    }

    /// Start building an experiment data message
    pub fn experiment(experiment: CrisprExperiment) -> Self {
        Self {
            message_type: MessageType::Experiment,
            sequence: None,
            capabilities: Vec::new(),
            options: None,
            payload_data: PayloadData::Experiment {
                experiment,
                is_update: false,
            },
        }
    }

    /// Start building a structure data message
    pub fn structure(structure: ProteinStructure) -> Self {
        Self {
            message_type: MessageType::Structure,
            sequence: None,
            capabilities: Vec::new(),
            options: None,
            payload_data: PayloadData::Structure {
                structure,
                job_id: None,
            },
        }
    }

    /// Start building a part data message
    pub fn part(part: BioPart) -> Self {
        Self {
            message_type: MessageType::Data,
            sequence: None,
            capabilities: Vec::new(),
            options: None,
            payload_data: PayloadData::Part {
                part,
                registry: None,
            },
        }
    }

    /// Start building a command message
    pub fn command(command: BioCommand) -> Self {
        Self {
            message_type: MessageType::Command,
            sequence: None,
            capabilities: Vec::new(),
            options: None,
            payload_data: PayloadData::Command {
                command,
                params: serde_json::Value::Null,
                correlation_id: None,
            },
        }
    }

    /// Start building a command acknowledgment message
    pub fn command_ack(command: BioCommand, status: CommandStatus) -> Self {
        Self {
            message_type: MessageType::CommandAck,
            sequence: None,
            capabilities: Vec::new(),
            options: None,
            payload_data: PayloadData::CommandAck {
                command,
                status,
                correlation_id: None,
                result: None,
                error: None,
            },
        }
    }

    /// Start building an error message
    pub fn error(code: u32, name: impl Into<String>, message: impl Into<String>) -> Self {
        Self {
            message_type: MessageType::Error,
            sequence: None,
            capabilities: Vec::new(),
            options: None,
            payload_data: PayloadData::Error {
                code,
                name: name.into(),
                message: message.into(),
                details: None,
                recoverable: false,
            },
        }
    }

    /// Start building a ping message
    pub fn ping() -> Self {
        Self {
            message_type: MessageType::Ping,
            sequence: None,
            capabilities: Vec::new(),
            options: None,
            payload_data: PayloadData::Heartbeat {
                client_timestamp: Utc::now().timestamp_millis(),
                server_timestamp: None,
            },
        }
    }

    /// Start building a pong message
    pub fn pong(client_timestamp: i64) -> Self {
        Self {
            message_type: MessageType::Pong,
            sequence: None,
            capabilities: Vec::new(),
            options: None,
            payload_data: PayloadData::Heartbeat {
                client_timestamp,
                server_timestamp: Some(Utc::now().timestamp_millis()),
            },
        }
    }

    /// Start building a subscribe message
    pub fn subscribe(topic: SubscriptionTopic) -> Self {
        Self {
            message_type: MessageType::Subscribe,
            sequence: None,
            capabilities: Vec::new(),
            options: None,
            payload_data: PayloadData::Subscribe {
                topic,
                filter: None,
            },
        }
    }

    // Builder methods for additional fields

    /// Add message sequence number
    pub fn with_sequence(mut self, seq: u64) -> Self {
        self.sequence = Some(seq);
        self
    }

    /// Add a capability (for connect/connect_ack)
    pub fn with_capability(mut self, capability: ProtocolCapability) -> Self {
        self.capabilities.push(capability);
        self
    }

    /// Add multiple capabilities
    pub fn with_capabilities(mut self, capabilities: Vec<ProtocolCapability>) -> Self {
        self.capabilities.extend(capabilities);
        self
    }

    /// Add connection options (for connect)
    pub fn with_options(mut self, options: ConnectionOptions) -> Self {
        self.options = Some(options);
        self
    }

    /// Add client version (for connect)
    pub fn with_client_version(mut self, version: impl Into<String>) -> Self {
        if let PayloadData::Connect { ref mut client_version, .. } = self.payload_data {
            *client_version = Some(version.into());
        }
        self
    }

    /// Add server version (for connect_ack)
    pub fn with_server_version(mut self, version: impl Into<String>) -> Self {
        if let PayloadData::ConnectAck { ref mut server_version, .. } = self.payload_data {
            *server_version = Some(version.into());
        }
        self
    }

    /// Add session timeout (for connect_ack)
    pub fn with_timeout(mut self, timeout: u64) -> Self {
        if let PayloadData::ConnectAck { timeout: ref mut t, .. } = self.payload_data {
            *t = Some(timeout);
        }
        self
    }

    /// Add disconnect reason
    pub fn with_reason(mut self, reason: impl Into<String>) -> Self {
        if let PayloadData::Disconnect { reason: ref mut r, .. } = self.payload_data {
            *r = Some(reason.into());
        }
        self
    }

    /// Allow reconnection (for disconnect)
    pub fn allow_reconnect(mut self) -> Self {
        if let PayloadData::Disconnect { ref mut reconnect_allowed, .. } = self.payload_data {
            *reconnect_allowed = true;
        }
        self
    }

    /// Add source ID (for sequence)
    pub fn with_source_id(mut self, source_id: impl Into<String>) -> Self {
        if let PayloadData::Sequence { source_id: ref mut s, .. } = self.payload_data {
            *s = Some(source_id.into());
        }
        self
    }

    /// Add batch info (for sequence)
    pub fn with_batch(mut self, batch_id: impl Into<String>, index: u32) -> Self {
        if let PayloadData::Sequence { batch_id: ref mut b, batch_index: ref mut i, .. } = self.payload_data {
            *b = Some(batch_id.into());
            *i = Some(index);
        }
        self
    }

    /// Mark as update (for experiment)
    pub fn as_update(mut self) -> Self {
        if let PayloadData::Experiment { ref mut is_update, .. } = self.payload_data {
            *is_update = true;
        }
        self
    }

    /// Add job ID (for structure)
    pub fn with_job_id(mut self, job_id: impl Into<String>) -> Self {
        if let PayloadData::Structure { job_id: ref mut j, .. } = self.payload_data {
            *j = Some(job_id.into());
        }
        self
    }

    /// Add registry source (for part)
    pub fn with_registry(mut self, registry: impl Into<String>) -> Self {
        if let PayloadData::Part { registry: ref mut r, .. } = self.payload_data {
            *r = Some(registry.into());
        }
        self
    }

    /// Add command parameters
    pub fn with_params(mut self, params: serde_json::Value) -> Self {
        if let PayloadData::Command { params: ref mut p, .. } = self.payload_data {
            *p = params;
        }
        self
    }

    /// Add correlation ID
    pub fn with_correlation_id(mut self, correlation_id: impl Into<String>) -> Self {
        match &mut self.payload_data {
            PayloadData::Command { correlation_id: ref mut c, .. } => {
                *c = Some(correlation_id.into());
            }
            PayloadData::CommandAck { correlation_id: ref mut c, .. } => {
                *c = Some(correlation_id.into());
            }
            _ => {}
        }
        self
    }

    /// Add command result
    pub fn with_result(mut self, result: serde_json::Value) -> Self {
        if let PayloadData::CommandAck { result: ref mut r, .. } = self.payload_data {
            *r = Some(result);
        }
        self
    }

    /// Add command error
    pub fn with_error_message(mut self, error: impl Into<String>) -> Self {
        if let PayloadData::CommandAck { error: ref mut e, .. } = self.payload_data {
            *e = Some(error.into());
        }
        self
    }

    /// Add error details
    pub fn with_details(mut self, details: serde_json::Value) -> Self {
        if let PayloadData::Error { details: ref mut d, .. } = self.payload_data {
            *d = Some(details);
        }
        self
    }

    /// Mark error as recoverable
    pub fn recoverable(mut self) -> Self {
        if let PayloadData::Error { ref mut recoverable, .. } = self.payload_data {
            *recoverable = true;
        }
        self
    }

    /// Add subscription filter
    pub fn with_filter(mut self, filter: serde_json::Value) -> Self {
        if let PayloadData::Subscribe { filter: ref mut f, .. } = self.payload_data {
            *f = Some(filter);
        }
        self
    }

    /// Build the final message
    pub fn build(self) -> BioMessage {
        let payload = match self.payload_data {
            PayloadData::Connect { client_id, client_name, client_version } => {
                MessagePayload::Connect(ConnectPayload {
                    client_id,
                    client_name,
                    client_version,
                    capabilities: self.capabilities,
                    options: self.options,
                })
            }
            PayloadData::ConnectAck { session_id, status, server_version, timeout } => {
                MessagePayload::ConnectAck(ConnectAckPayload {
                    session_id,
                    status,
                    capabilities: self.capabilities,
                    server_version,
                    timeout,
                })
            }
            PayloadData::Disconnect { reason_code, reason, reconnect_allowed } => {
                MessagePayload::Disconnect(DisconnectPayload {
                    reason_code,
                    reason,
                    reconnect_allowed,
                })
            }
            PayloadData::Sequence { sequence, source_id, batch_id, batch_index } => {
                MessagePayload::Sequence(SequencePayload {
                    sequence,
                    source_id,
                    batch_id,
                    batch_index,
                })
            }
            PayloadData::Experiment { experiment, is_update } => {
                MessagePayload::Experiment(ExperimentPayload {
                    experiment,
                    is_update,
                })
            }
            PayloadData::Structure { structure, job_id } => {
                MessagePayload::Structure(StructurePayload {
                    structure,
                    job_id,
                })
            }
            PayloadData::Part { part, registry } => {
                MessagePayload::Part(PartPayload {
                    part,
                    registry,
                })
            }
            PayloadData::Command { command, params, correlation_id } => {
                MessagePayload::Command(CommandPayload {
                    command,
                    params,
                    correlation_id,
                })
            }
            PayloadData::CommandAck { command, status, correlation_id, result, error } => {
                MessagePayload::CommandAck(CommandAckPayload {
                    command,
                    status,
                    correlation_id,
                    result,
                    error,
                })
            }
            PayloadData::Error { code, name, message, details, recoverable } => {
                MessagePayload::Error(ErrorPayload {
                    code,
                    name,
                    message,
                    details,
                    recoverable,
                })
            }
            PayloadData::Heartbeat { client_timestamp, server_timestamp } => {
                MessagePayload::Heartbeat(HeartbeatPayload {
                    client_timestamp,
                    server_timestamp,
                })
            }
            PayloadData::Subscribe { topic, filter } => {
                MessagePayload::Subscribe(SubscribePayload {
                    topic,
                    filter,
                })
            }
            PayloadData::Empty => {
                MessagePayload::Empty(EmptyPayload {})
            }
        };

        let mut msg = BioMessage::new(self.message_type, payload);
        if let Some(seq) = self.sequence {
            msg = msg.with_sequence(seq);
        }
        msg
    }
}

/// Parse a JSON string into a BioMessage
pub fn parse_message(json: &str) -> Result<BioMessage, ProtocolError> {
    serde_json::from_str(json)
        .map_err(|e| ProtocolError::DeserializationError(e.to_string()))
}

/// Serialize a BioMessage to JSON
pub fn serialize_message(msg: &BioMessage) -> Result<String, ProtocolError> {
    serde_json::to_string(msg)
        .map_err(|e| ProtocolError::SerializationError(e.to_string()))
}

/// Serialize a BioMessage to pretty JSON
pub fn serialize_message_pretty(msg: &BioMessage) -> Result<String, ProtocolError> {
    serde_json::to_string_pretty(msg)
        .map_err(|e| ProtocolError::SerializationError(e.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_connect_builder() {
        let msg = MessageBuilder::connect("client-123", "Test App")
            .with_client_version("1.0.0")
            .with_capability(ProtocolCapability::SequenceStreaming)
            .with_capability(ProtocolCapability::CrisprExperiment)
            .with_options(ConnectionOptions {
                stream_rate: Some(100),
                compression: false,
                buffer_size: None,
                heartbeat_interval: Some(30000),
            })
            .with_sequence(1)
            .build();

        assert_eq!(msg.message_type, MessageType::Connect);
        assert_eq!(msg.sequence, Some(1));

        if let MessagePayload::Connect(payload) = msg.payload {
            assert_eq!(payload.client_id, "client-123");
            assert_eq!(payload.client_name, "Test App");
            assert_eq!(payload.capabilities.len(), 2);
        } else {
            panic!("Expected Connect payload");
        }
    }

    #[test]
    fn test_error_builder() {
        let msg = MessageBuilder::error(
            ErrorCode::INVALID_SEQUENCE,
            "INVALID_SEQUENCE",
            "Invalid DNA sequence detected",
        )
        .with_details(serde_json::json!({
            "position": 42,
            "invalid_char": "X"
        }))
        .recoverable()
        .build();

        assert_eq!(msg.message_type, MessageType::Error);

        if let MessagePayload::Error(payload) = msg.payload {
            assert_eq!(payload.code, 2001);
            assert!(payload.recoverable);
            assert!(payload.details.is_some());
        } else {
            panic!("Expected Error payload");
        }
    }

    #[test]
    fn test_ping_pong() {
        let ping = MessageBuilder::ping().build();
        assert_eq!(ping.message_type, MessageType::Ping);

        if let MessagePayload::Heartbeat(payload) = &ping.payload {
            let pong = MessageBuilder::pong(payload.client_timestamp).build();
            assert_eq!(pong.message_type, MessageType::Pong);

            if let MessagePayload::Heartbeat(pong_payload) = pong.payload {
                assert_eq!(pong_payload.client_timestamp, payload.client_timestamp);
                assert!(pong_payload.server_timestamp.is_some());
            }
        }
    }

    #[test]
    fn test_message_roundtrip() {
        let msg = MessageBuilder::connect("test", "Test")
            .with_capability(ProtocolCapability::SequenceStreaming)
            .build();

        let json = serialize_message(&msg).unwrap();
        let parsed = parse_message(&json).unwrap();

        assert_eq!(parsed.protocol, msg.protocol);
        assert_eq!(parsed.message_type, msg.message_type);
    }
}
