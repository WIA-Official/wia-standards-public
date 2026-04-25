//! Stream Handler
//!
//! This module provides handlers for processing streaming responses.

use async_trait::async_trait;
use futures_util::Stream;
use std::pin::Pin;
use std::sync::Arc;
use tokio::sync::{mpsc, RwLock};

use crate::protocol::{
    Delta, DeltaType, ProtocolError, ProtocolMessage, ProtocolResult, StopReason,
    StreamEndPayload, StreamStartPayload, Usage,
};

use super::SseEvent;

/// Callback type for stream events
pub type StreamCallback = Box<dyn Fn(StreamEvent) + Send + Sync>;

/// Stream event types
#[derive(Debug, Clone)]
pub enum StreamEvent {
    /// Stream started
    Start {
        /// Stream ID
        stream_id: String,
        /// Model used
        model: Option<String>,
    },
    /// Text delta received
    TextDelta {
        /// Delta text
        text: String,
        /// Delta index
        index: u32,
    },
    /// Tool use delta received
    ToolUseDelta {
        /// Partial JSON
        partial_json: String,
        /// Delta index
        index: u32,
    },
    /// Stream ended
    End {
        /// Stop reason
        stop_reason: Option<StopReason>,
        /// Usage statistics
        usage: Option<Usage>,
    },
    /// Error occurred
    Error {
        /// Error message
        message: String,
    },
    /// Ping/keepalive
    Ping,
}

/// Trait for handling stream events
#[async_trait]
pub trait StreamEventHandler: Send + Sync {
    /// Handle a stream event
    async fn handle(&self, event: StreamEvent) -> ProtocolResult<()>;

    /// Called when streaming completes
    async fn on_complete(&self, _accumulated: &StreamAccumulator) -> ProtocolResult<()> {
        Ok(())
    }
}

/// Accumulator for stream content
#[derive(Debug, Clone, Default)]
pub struct StreamAccumulator {
    /// Stream ID
    pub stream_id: Option<String>,

    /// Model used
    pub model: Option<String>,

    /// Accumulated text
    pub text: String,

    /// Tool use JSON parts
    pub tool_use_json: String,

    /// Number of deltas received
    pub delta_count: u32,

    /// Stop reason
    pub stop_reason: Option<StopReason>,

    /// Usage statistics
    pub usage: Option<Usage>,

    /// Whether the stream has completed
    pub completed: bool,
}

impl StreamAccumulator {
    /// Create a new stream accumulator
    pub fn new() -> Self {
        Self::default()
    }

    /// Handle a stream start event
    pub fn on_start(&mut self, payload: &StreamStartPayload) {
        self.stream_id = Some(payload.stream_id.clone());
        self.model = payload.model.clone();
    }

    /// Handle a delta event
    pub fn on_delta(&mut self, delta: &Delta, index: u32) {
        self.delta_count = index + 1;

        match delta.delta_type {
            DeltaType::TextDelta => {
                if let Some(ref text) = delta.text {
                    self.text.push_str(text);
                }
            }
            DeltaType::ToolUseDelta => {
                if let Some(ref json) = delta.partial_json {
                    self.tool_use_json.push_str(json);
                }
            }
            DeltaType::ThinkingDelta => {
                // Thinking deltas can be handled separately if needed
            }
        }
    }

    /// Handle a stream end event
    pub fn on_end(&mut self, payload: &StreamEndPayload) {
        self.stop_reason = payload.stop_reason;
        self.usage = payload.usage.clone();
        self.completed = true;
    }

    /// Get the accumulated text
    pub fn text(&self) -> &str {
        &self.text
    }

    /// Check if stream has completed
    pub fn is_completed(&self) -> bool {
        self.completed
    }
}

/// Stream processor for handling protocol messages
pub struct StreamProcessor {
    accumulator: Arc<RwLock<StreamAccumulator>>,
    handlers: Vec<Arc<dyn StreamEventHandler>>,
}

impl StreamProcessor {
    /// Create a new stream processor
    pub fn new() -> Self {
        Self {
            accumulator: Arc::new(RwLock::new(StreamAccumulator::new())),
            handlers: Vec::new(),
        }
    }

    /// Add an event handler
    pub fn add_handler(&mut self, handler: Arc<dyn StreamEventHandler>) {
        self.handlers.push(handler);
    }

    /// Process a protocol message
    pub async fn process(&self, message: &ProtocolMessage) -> ProtocolResult<()> {
        use crate::protocol::MessageType;

        let event = match message.message_type {
            MessageType::StreamStart => {
                if let Some(ref payload) = message.payload {
                    let start: StreamStartPayload = serde_json::from_value(payload.clone())
                        .map_err(|e| ProtocolError::validation_error(e.to_string()))?;

                    self.accumulator.write().await.on_start(&start);

                    Some(StreamEvent::Start {
                        stream_id: start.stream_id,
                        model: start.model,
                    })
                } else {
                    None
                }
            }
            MessageType::StreamDelta => {
                if let Some(ref payload) = message.payload {
                    let delta_payload: crate::protocol::StreamDeltaPayload =
                        serde_json::from_value(payload.clone())
                            .map_err(|e| ProtocolError::validation_error(e.to_string()))?;

                    let index = delta_payload.index.unwrap_or(0);
                    self.accumulator
                        .write()
                        .await
                        .on_delta(&delta_payload.delta, index);

                    match delta_payload.delta.delta_type {
                        DeltaType::TextDelta => Some(StreamEvent::TextDelta {
                            text: delta_payload.delta.text.unwrap_or_default(),
                            index,
                        }),
                        DeltaType::ToolUseDelta => Some(StreamEvent::ToolUseDelta {
                            partial_json: delta_payload.delta.partial_json.unwrap_or_default(),
                            index,
                        }),
                        DeltaType::ThinkingDelta => Some(StreamEvent::TextDelta {
                            text: delta_payload.delta.text.unwrap_or_default(),
                            index,
                        }),
                    }
                } else {
                    None
                }
            }
            MessageType::StreamEnd => {
                if let Some(ref payload) = message.payload {
                    let end: StreamEndPayload = serde_json::from_value(payload.clone())
                        .map_err(|e| ProtocolError::validation_error(e.to_string()))?;

                    self.accumulator.write().await.on_end(&end);

                    Some(StreamEvent::End {
                        stop_reason: end.stop_reason,
                        usage: end.usage,
                    })
                } else {
                    None
                }
            }
            MessageType::Ping => Some(StreamEvent::Ping),
            MessageType::Error => {
                let message = message
                    .payload
                    .as_ref()
                    .and_then(|p| p.get("message"))
                    .and_then(|m| m.as_str())
                    .unwrap_or("Unknown error")
                    .to_string();

                Some(StreamEvent::Error { message })
            }
            _ => None,
        };

        if let Some(event) = event {
            for handler in &self.handlers {
                handler.handle(event.clone()).await?;
            }

            // Call on_complete if stream ended
            if matches!(event, StreamEvent::End { .. }) {
                let acc = self.accumulator.read().await;
                for handler in &self.handlers {
                    handler.on_complete(&acc).await?;
                }
            }
        }

        Ok(())
    }

    /// Process an SSE event
    pub async fn process_sse(&self, sse: &SseEvent) -> ProtocolResult<()> {
        if sse.is_ping() {
            for handler in &self.handlers {
                handler.handle(StreamEvent::Ping).await?;
            }
            return Ok(());
        }

        if sse.is_done() {
            // [DONE] marker, stream complete
            return Ok(());
        }

        if let Some(ref data) = sse.data {
            if let Ok(message) = serde_json::from_str::<ProtocolMessage>(data) {
                return self.process(&message).await;
            }
        }

        Ok(())
    }

    /// Get the accumulated content
    pub async fn accumulator(&self) -> StreamAccumulator {
        self.accumulator.read().await.clone()
    }

    /// Reset the processor
    pub async fn reset(&self) {
        *self.accumulator.write().await = StreamAccumulator::new();
    }
}

impl Default for StreamProcessor {
    fn default() -> Self {
        Self::new()
    }
}

/// Print handler that prints stream events to stdout
pub struct PrintStreamHandler {
    prefix: String,
}

impl PrintStreamHandler {
    /// Create a new print handler
    pub fn new() -> Self {
        Self {
            prefix: String::new(),
        }
    }

    /// Set a prefix for output
    pub fn with_prefix(mut self, prefix: impl Into<String>) -> Self {
        self.prefix = prefix.into();
        self
    }
}

impl Default for PrintStreamHandler {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl StreamEventHandler for PrintStreamHandler {
    async fn handle(&self, event: StreamEvent) -> ProtocolResult<()> {
        match event {
            StreamEvent::Start { stream_id, model } => {
                println!(
                    "{}[Stream Start] ID: {}, Model: {:?}",
                    self.prefix, stream_id, model
                );
            }
            StreamEvent::TextDelta { text, .. } => {
                print!("{}", text);
            }
            StreamEvent::ToolUseDelta { partial_json, .. } => {
                print!("{}", partial_json);
            }
            StreamEvent::End {
                stop_reason,
                usage,
            } => {
                println!();
                println!(
                    "{}[Stream End] Reason: {:?}, Usage: {:?}",
                    self.prefix, stop_reason, usage
                );
            }
            StreamEvent::Error { message } => {
                eprintln!("{}[Error] {}", self.prefix, message);
            }
            StreamEvent::Ping => {
                // Silently handle ping
            }
        }
        Ok(())
    }
}

/// Channel-based stream handler that sends events to a channel
pub struct ChannelStreamHandler {
    sender: mpsc::Sender<StreamEvent>,
}

impl ChannelStreamHandler {
    /// Create a new channel handler with the given sender
    pub fn new(sender: mpsc::Sender<StreamEvent>) -> Self {
        Self { sender }
    }

    /// Create a new channel handler and return the receiver
    pub fn with_channel(buffer: usize) -> (Self, mpsc::Receiver<StreamEvent>) {
        let (sender, receiver) = mpsc::channel(buffer);
        (Self::new(sender), receiver)
    }
}

#[async_trait]
impl StreamEventHandler for ChannelStreamHandler {
    async fn handle(&self, event: StreamEvent) -> ProtocolResult<()> {
        self.sender.send(event).await.map_err(|e| {
            ProtocolError::validation_error(format!("Failed to send event: {}", e))
        })?;
        Ok(())
    }
}

/// Callback-based stream handler
pub struct CallbackStreamHandler {
    callback: StreamCallback,
}

impl CallbackStreamHandler {
    /// Create a new callback handler
    pub fn new(callback: StreamCallback) -> Self {
        Self { callback }
    }
}

#[async_trait]
impl StreamEventHandler for CallbackStreamHandler {
    async fn handle(&self, event: StreamEvent) -> ProtocolResult<()> {
        (self.callback)(event);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{Delta, DeltaType};

    #[test]
    fn test_stream_accumulator() {
        let mut acc = StreamAccumulator::new();

        acc.on_start(&StreamStartPayload {
            stream_id: "stream-001".to_string(),
            model: Some("claude".to_string()),
            input_tokens: Some(100),
        });

        assert_eq!(acc.stream_id, Some("stream-001".to_string()));
        assert_eq!(acc.model, Some("claude".to_string()));

        acc.on_delta(
            &Delta {
                delta_type: DeltaType::TextDelta,
                text: Some("Hello ".to_string()),
                partial_json: None,
            },
            0,
        );

        acc.on_delta(
            &Delta {
                delta_type: DeltaType::TextDelta,
                text: Some("World!".to_string()),
                partial_json: None,
            },
            1,
        );

        assert_eq!(acc.text(), "Hello World!");
        assert_eq!(acc.delta_count, 2);

        acc.on_end(&StreamEndPayload {
            stream_id: "stream-001".to_string(),
            stop_reason: Some(StopReason::EndTurn),
            usage: None,
        });

        assert!(acc.is_completed());
        assert_eq!(acc.stop_reason, Some(StopReason::EndTurn));
    }

    #[tokio::test]
    async fn test_stream_processor() {
        let processor = StreamProcessor::new();

        // Simulate stream start
        let start_msg = crate::protocol::ProtocolMessageBuilder::new()
            .message_type(crate::protocol::MessageType::StreamStart)
            .payload(serde_json::json!({
                "stream_id": "stream-001",
                "model": "claude"
            }))
            .build()
            .unwrap();

        processor.process(&start_msg).await.unwrap();

        let acc = processor.accumulator().await;
        assert_eq!(acc.stream_id, Some("stream-001".to_string()));
    }

    #[tokio::test]
    async fn test_channel_handler() {
        let (handler, mut receiver) = ChannelStreamHandler::with_channel(10);

        handler
            .handle(StreamEvent::TextDelta {
                text: "Hello".to_string(),
                index: 0,
            })
            .await
            .unwrap();

        let event = receiver.recv().await.unwrap();
        match event {
            StreamEvent::TextDelta { text, index } => {
                assert_eq!(text, "Hello");
                assert_eq!(index, 0);
            }
            _ => panic!("Unexpected event type"),
        }
    }
}
