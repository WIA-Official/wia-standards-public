//! Server-Sent Events (SSE) Implementation
//!
//! This module provides SSE parsing and formatting for the WIA AI protocol.

use std::time::Duration;

/// SSE event structure
#[derive(Debug, Clone, Default)]
pub struct SseEvent {
    /// Event type (optional)
    pub event_type: Option<String>,

    /// Event data
    pub data: Option<String>,

    /// Event ID (optional)
    pub id: Option<String>,

    /// Retry interval in milliseconds (optional)
    pub retry: Option<u64>,

    /// Comments (optional)
    pub comments: Vec<String>,
}

impl SseEvent {
    /// Create a new SSE event
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the event type
    pub fn event(mut self, event_type: impl Into<String>) -> Self {
        self.event_type = Some(event_type.into());
        self
    }

    /// Set the event data
    pub fn data(mut self, data: impl Into<String>) -> Self {
        self.data = Some(data.into());
        self
    }

    /// Set the event ID
    pub fn id(mut self, id: impl Into<String>) -> Self {
        self.id = Some(id.into());
        self
    }

    /// Set the retry interval
    pub fn retry(mut self, ms: u64) -> Self {
        self.retry = Some(ms);
        self
    }

    /// Add a comment
    pub fn comment(mut self, comment: impl Into<String>) -> Self {
        self.comments.push(comment.into());
        self
    }

    /// Parse an SSE event from a string
    pub fn parse(input: &str) -> Option<Self> {
        let mut event = Self::new();
        let mut has_content = false;

        for line in input.lines() {
            if line.is_empty() {
                continue;
            }

            if line.starts_with(':') {
                // Comment
                event.comments.push(line[1..].trim().to_string());
                has_content = true;
            } else if let Some(pos) = line.find(':') {
                let field = &line[..pos];
                let value = line[pos + 1..].trim_start();
                has_content = true;

                match field {
                    "event" => event.event_type = Some(value.to_string()),
                    "data" => {
                        if let Some(ref mut existing) = event.data {
                            existing.push('\n');
                            existing.push_str(value);
                        } else {
                            event.data = Some(value.to_string());
                        }
                    }
                    "id" => event.id = Some(value.to_string()),
                    "retry" => {
                        if let Ok(ms) = value.parse() {
                            event.retry = Some(ms);
                        }
                    }
                    _ => {} // Ignore unknown fields
                }
            }
        }

        if has_content {
            Some(event)
        } else {
            None
        }
    }

    /// Format the event as an SSE string
    pub fn format(&self) -> String {
        let mut output = String::new();

        // Comments
        for comment in &self.comments {
            output.push(':');
            output.push_str(comment);
            output.push('\n');
        }

        // Event type
        if let Some(ref event_type) = self.event_type {
            output.push_str("event: ");
            output.push_str(event_type);
            output.push('\n');
        }

        // Data (can be multiline)
        if let Some(ref data) = self.data {
            for line in data.lines() {
                output.push_str("data: ");
                output.push_str(line);
                output.push('\n');
            }
        }

        // ID
        if let Some(ref id) = self.id {
            output.push_str("id: ");
            output.push_str(id);
            output.push('\n');
        }

        // Retry
        if let Some(retry) = self.retry {
            output.push_str("retry: ");
            output.push_str(&retry.to_string());
            output.push('\n');
        }

        // End with blank line
        output.push('\n');

        output
    }

    /// Check if this is a ping/keepalive event
    pub fn is_ping(&self) -> bool {
        self.event_type.as_deref() == Some("ping")
    }

    /// Check if this is a done marker
    pub fn is_done(&self) -> bool {
        self.data.as_deref() == Some("[DONE]")
    }

    /// Parse JSON data
    pub fn parse_json<T: serde::de::DeserializeOwned>(&self) -> Option<T> {
        self.data
            .as_ref()
            .and_then(|data| serde_json::from_str(data).ok())
    }
}

/// SSE event type constants
pub mod event_types {
    /// Stream started
    pub const STREAM_START: &str = "stream_start";
    /// Stream delta (content chunk)
    pub const STREAM_DELTA: &str = "stream_delta";
    /// Stream ended
    pub const STREAM_END: &str = "stream_end";
    /// Ping/keepalive
    pub const PING: &str = "ping";
    /// Error occurred
    pub const ERROR: &str = "error";
    /// Message
    pub const MESSAGE: &str = "message";
}

/// SSE stream parser for parsing a continuous stream of SSE events
pub struct SseParser {
    buffer: String,
}

impl SseParser {
    /// Create a new SSE parser
    pub fn new() -> Self {
        Self {
            buffer: String::new(),
        }
    }

    /// Feed data to the parser and get any complete events
    pub fn feed(&mut self, data: &str) -> Vec<SseEvent> {
        self.buffer.push_str(data);
        self.extract_events()
    }

    /// Extract complete events from the buffer
    fn extract_events(&mut self) -> Vec<SseEvent> {
        let mut events = Vec::new();

        // Events are separated by double newlines
        while let Some(pos) = self.buffer.find("\n\n") {
            let event_str = self.buffer[..pos + 2].to_string();
            self.buffer = self.buffer[pos + 2..].to_string();

            if let Some(event) = SseEvent::parse(&event_str) {
                events.push(event);
            }
        }

        events
    }

    /// Get remaining buffer content
    pub fn remaining(&self) -> &str {
        &self.buffer
    }

    /// Clear the buffer
    pub fn clear(&mut self) {
        self.buffer.clear();
    }
}

impl Default for SseParser {
    fn default() -> Self {
        Self::new()
    }
}

/// SSE connection configuration
#[derive(Debug, Clone)]
pub struct SseConfig {
    /// Reconnect delay after connection loss
    pub reconnect_delay: Duration,

    /// Maximum reconnect attempts
    pub max_reconnect_attempts: u32,

    /// Enable automatic reconnection
    pub auto_reconnect: bool,

    /// Last event ID for resumption
    pub last_event_id: Option<String>,
}

impl Default for SseConfig {
    fn default() -> Self {
        Self {
            reconnect_delay: Duration::from_secs(3),
            max_reconnect_attempts: 5,
            auto_reconnect: true,
            last_event_id: None,
        }
    }
}

impl SseConfig {
    /// Create a new SSE configuration
    pub fn new() -> Self {
        Self::default()
    }

    /// Set reconnect delay
    pub fn with_reconnect_delay(mut self, delay: Duration) -> Self {
        self.reconnect_delay = delay;
        self
    }

    /// Set max reconnect attempts
    pub fn with_max_reconnect_attempts(mut self, attempts: u32) -> Self {
        self.max_reconnect_attempts = attempts;
        self
    }

    /// Disable auto reconnect
    pub fn without_auto_reconnect(mut self) -> Self {
        self.auto_reconnect = false;
        self
    }

    /// Set last event ID
    pub fn with_last_event_id(mut self, id: impl Into<String>) -> Self {
        self.last_event_id = Some(id.into());
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_event() {
        let input = "event: message\ndata: Hello World\n\n";
        let event = SseEvent::parse(input).unwrap();

        assert_eq!(event.event_type, Some("message".to_string()));
        assert_eq!(event.data, Some("Hello World".to_string()));
    }

    #[test]
    fn test_parse_multiline_data() {
        let input = "data: line 1\ndata: line 2\ndata: line 3\n\n";
        let event = SseEvent::parse(input).unwrap();

        assert_eq!(event.data, Some("line 1\nline 2\nline 3".to_string()));
    }

    #[test]
    fn test_parse_with_id_and_retry() {
        let input = "event: update\ndata: {\"value\": 42}\nid: 123\nretry: 5000\n\n";
        let event = SseEvent::parse(input).unwrap();

        assert_eq!(event.event_type, Some("update".to_string()));
        assert_eq!(event.id, Some("123".to_string()));
        assert_eq!(event.retry, Some(5000));
    }

    #[test]
    fn test_parse_comment() {
        let input = ": this is a comment\nevent: ping\n\n";
        let event = SseEvent::parse(input).unwrap();

        assert_eq!(event.comments.len(), 1);
        assert_eq!(event.comments[0], "this is a comment");
        assert_eq!(event.event_type, Some("ping".to_string()));
    }

    #[test]
    fn test_format_event() {
        let event = SseEvent::new()
            .event("stream_delta")
            .data("{\"text\": \"Hello\"}")
            .id("evt-001");

        let formatted = event.format();

        assert!(formatted.contains("event: stream_delta\n"));
        assert!(formatted.contains("data: {\"text\": \"Hello\"}\n"));
        assert!(formatted.contains("id: evt-001\n"));
        assert!(formatted.ends_with("\n\n"));
    }

    #[test]
    fn test_sse_parser() {
        let mut parser = SseParser::new();

        // Partial data
        let events = parser.feed("event: message\n");
        assert!(events.is_empty());

        // Complete the event
        let events = parser.feed("data: test\n\n");
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].event_type, Some("message".to_string()));
    }

    #[test]
    fn test_parse_json() {
        let event = SseEvent::new().data("{\"text\": \"Hello\", \"count\": 42}");

        #[derive(serde::Deserialize, Debug)]
        struct TestData {
            text: String,
            count: i32,
        }

        let data: TestData = event.parse_json().unwrap();
        assert_eq!(data.text, "Hello");
        assert_eq!(data.count, 42);
    }

    #[test]
    fn test_is_done() {
        let done_event = SseEvent::new().data("[DONE]");
        assert!(done_event.is_done());

        let normal_event = SseEvent::new().data("{\"text\": \"Hello\"}");
        assert!(!normal_event.is_done());
    }
}
