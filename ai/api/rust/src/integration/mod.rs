//! WIA Ecosystem Integration Module
//!
//! This module provides integration between WIA AI and other WIA standards:
//! - AAC (Augmentative and Alternative Communication)
//! - BCI (Brain-Computer Interface)
//! - Voice Interface
//! - TTS (Text-to-Speech)
//! - ISP (International Sign Protocol)
//! - Braille
//!
//! # Architecture
//!
//! ```text
//! WIA Standards ←→ Connectors ←→ Hub ←→ Adapters ←→ AI Agents
//! ```
//!
//! # Example
//!
//! ```rust,no_run
//! use wia_ai::integration::*;
//! use wia_ai::integration::adapters::{AacInputAdapter, TtsOutputAdapter};
//!
//! #[tokio::main]
//! async fn main() {
//!     let hub = WiaAiHub::new();
//!
//!     // Register connectors and adapters
//!     hub.register_input_adapter(Box::new(AacInputAdapter::new())).await;
//!     hub.register_output_adapter(Box::new(TtsOutputAdapter::new())).await;
//! }
//! ```

mod connector;
mod hub;
pub mod adapters;
mod mock;

pub use connector::*;
pub use hub::*;
pub use mock::*;

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::HashMap;

/// WIA Standard types supported for integration
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WiaStandardType {
    /// Augmentative and Alternative Communication
    Aac,
    /// Brain-Computer Interface
    Bci,
    /// Voice Interface
    Voice,
    /// Text-to-Speech
    Tts,
    /// International Sign Protocol
    Isp,
    /// Braille
    Braille,
    /// Extended Reality (VR/AR/MR)
    Xr,
    /// Robotics
    Robot,
    /// Smart Home
    SmartHome,
    /// AI Standard (this standard)
    Ai,
    /// Custom standard
    Custom(String),
}

impl std::fmt::Display for WiaStandardType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            WiaStandardType::Aac => write!(f, "aac"),
            WiaStandardType::Bci => write!(f, "bci"),
            WiaStandardType::Voice => write!(f, "voice"),
            WiaStandardType::Tts => write!(f, "tts"),
            WiaStandardType::Isp => write!(f, "isp"),
            WiaStandardType::Braille => write!(f, "braille"),
            WiaStandardType::Xr => write!(f, "xr"),
            WiaStandardType::Robot => write!(f, "robot"),
            WiaStandardType::SmartHome => write!(f, "smart_home"),
            WiaStandardType::Ai => write!(f, "ai"),
            WiaStandardType::Custom(name) => write!(f, "custom:{}", name),
        }
    }
}

/// Message type for WIA inter-standard communication
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WiaMessageType {
    /// Text message
    Text,
    /// Audio data
    Audio,
    /// Signal data (sensors, EEG, etc.)
    Signal,
    /// Image data
    Image,
    /// Command
    Command,
    /// Response
    Response,
    /// Event notification
    Event,
    /// Error
    Error,
}

/// Payload for WIA messages
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum WiaPayload {
    /// Text payload
    Text(String),
    /// Binary payload (base64 encoded in JSON)
    Binary(Vec<u8>),
    /// JSON payload
    Json(Value),
    /// Signal data
    Signal(SignalData),
}

/// Signal data structure for sensor/BCI/AAC signals
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalData {
    /// Channel names
    pub channels: Vec<String>,
    /// Sample data per channel
    pub samples: Vec<Vec<f64>>,
    /// Sample rate in Hz
    pub sample_rate: f64,
    /// Unit of measurement
    pub unit: String,
}

/// WIA inter-standard message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WiaMessage {
    /// Unique message ID
    pub id: String,
    /// Source standard
    pub source: WiaStandardType,
    /// Target standard
    pub target: WiaStandardType,
    /// Message type
    pub message_type: WiaMessageType,
    /// Payload
    pub payload: WiaPayload,
    /// Metadata
    #[serde(default)]
    pub metadata: HashMap<String, Value>,
    /// Timestamp
    pub timestamp: DateTime<Utc>,
}

impl WiaMessage {
    /// Create a new text message
    pub fn text(
        source: WiaStandardType,
        target: WiaStandardType,
        text: impl Into<String>,
    ) -> Self {
        Self {
            id: uuid::Uuid::new_v4().to_string(),
            source,
            target,
            message_type: WiaMessageType::Text,
            payload: WiaPayload::Text(text.into()),
            metadata: HashMap::new(),
            timestamp: Utc::now(),
        }
    }

    /// Create a new signal message
    pub fn signal(
        source: WiaStandardType,
        target: WiaStandardType,
        signal: SignalData,
    ) -> Self {
        Self {
            id: uuid::Uuid::new_v4().to_string(),
            source,
            target,
            message_type: WiaMessageType::Signal,
            payload: WiaPayload::Signal(signal),
            metadata: HashMap::new(),
            timestamp: Utc::now(),
        }
    }

    /// Create a new command message
    pub fn command(
        source: WiaStandardType,
        target: WiaStandardType,
        command: Value,
    ) -> Self {
        Self {
            id: uuid::Uuid::new_v4().to_string(),
            source,
            target,
            message_type: WiaMessageType::Command,
            payload: WiaPayload::Json(command),
            metadata: HashMap::new(),
            timestamp: Utc::now(),
        }
    }

    /// Add metadata to the message
    pub fn with_metadata(mut self, key: impl Into<String>, value: Value) -> Self {
        self.metadata.insert(key.into(), value);
        self
    }
}

/// AI input type classification
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AiInputType {
    /// Text input
    Text,
    /// Audio input
    Audio,
    /// Image input
    Image,
    /// Signal input (sensors, EEG)
    Signal,
    /// Multimodal input
    Multimodal,
}

/// AI input data structure
#[derive(Debug, Clone)]
pub struct AiInput {
    /// Input type
    pub input_type: AiInputType,
    /// Text content (if applicable)
    pub text: Option<String>,
    /// Binary data (audio, image, etc.)
    pub data: Option<Vec<u8>>,
    /// Original WIA message
    pub source_message: Option<WiaMessage>,
    /// Additional context
    pub context: HashMap<String, Value>,
}

impl AiInput {
    /// Create a text input
    pub fn text(content: impl Into<String>) -> Self {
        Self {
            input_type: AiInputType::Text,
            text: Some(content.into()),
            data: None,
            source_message: None,
            context: HashMap::new(),
        }
    }

    /// Create an audio input
    pub fn audio(data: Vec<u8>) -> Self {
        Self {
            input_type: AiInputType::Audio,
            text: None,
            data: Some(data),
            source_message: None,
            context: HashMap::new(),
        }
    }

    /// Create a signal input
    pub fn signal(text: impl Into<String>, message: WiaMessage) -> Self {
        Self {
            input_type: AiInputType::Signal,
            text: Some(text.into()),
            data: None,
            source_message: Some(message),
            context: HashMap::new(),
        }
    }

    /// Add context
    pub fn with_context(mut self, key: impl Into<String>, value: Value) -> Self {
        self.context.insert(key.into(), value);
        self
    }
}

/// AI output type classification
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AiOutputType {
    /// Text output
    Text,
    /// Speech output
    Speech,
    /// Sign language output
    SignLanguage,
    /// Braille output
    Braille,
    /// Multimodal output
    Multimodal,
}

/// AI output data structure
#[derive(Debug, Clone)]
pub struct AiOutput {
    /// Output type
    pub output_type: AiOutputType,
    /// Text content
    pub text: String,
    /// Binary data (if applicable)
    pub data: Option<Vec<u8>>,
    /// Metadata
    pub metadata: HashMap<String, Value>,
}

impl AiOutput {
    /// Create a text output
    pub fn text(content: impl Into<String>) -> Self {
        Self {
            output_type: AiOutputType::Text,
            text: content.into(),
            data: None,
            metadata: HashMap::new(),
        }
    }

    /// Create a speech output
    pub fn speech(text: impl Into<String>, audio_data: Vec<u8>) -> Self {
        Self {
            output_type: AiOutputType::Speech,
            text: text.into(),
            data: Some(audio_data),
            metadata: HashMap::new(),
        }
    }

    /// Add metadata
    pub fn with_metadata(mut self, key: impl Into<String>, value: Value) -> Self {
        self.metadata.insert(key.into(), value);
        self
    }
}

/// Hub events for monitoring
#[derive(Debug, Clone)]
pub enum HubEvent {
    /// Connector connected
    ConnectorConnected {
        /// Connector ID
        id: String,
        /// Standard type
        standard: WiaStandardType,
    },
    /// Connector disconnected
    ConnectorDisconnected {
        /// Connector ID
        id: String,
    },
    /// Message received
    MessageReceived {
        /// The message
        message: WiaMessage,
    },
    /// Message sent
    MessageSent {
        /// The message
        message: WiaMessage,
    },
    /// Error occurred
    Error {
        /// Error source
        source: String,
        /// Error message
        error: String,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wia_message_creation() {
        let msg = WiaMessage::text(
            WiaStandardType::Aac,
            WiaStandardType::Ai,
            "Hello",
        );

        assert_eq!(msg.source, WiaStandardType::Aac);
        assert_eq!(msg.target, WiaStandardType::Ai);
        assert!(matches!(msg.payload, WiaPayload::Text(_)));
    }

    #[test]
    fn test_ai_input_creation() {
        let input = AiInput::text("Test input")
            .with_context("language", serde_json::json!("en"));

        assert_eq!(input.input_type, AiInputType::Text);
        assert_eq!(input.text, Some("Test input".into()));
        assert!(input.context.contains_key("language"));
    }

    #[test]
    fn test_ai_output_creation() {
        let output = AiOutput::text("Test response")
            .with_metadata("confidence", serde_json::json!(0.95));

        assert_eq!(output.output_type, AiOutputType::Text);
        assert_eq!(output.text, "Test response");
        assert!(output.metadata.contains_key("confidence"));
    }

    #[test]
    fn test_wia_standard_type_display() {
        assert_eq!(WiaStandardType::Aac.to_string(), "aac");
        assert_eq!(WiaStandardType::Bci.to_string(), "bci");
        assert_eq!(
            WiaStandardType::Custom("test".into()).to_string(),
            "custom:test"
        );
    }
}
