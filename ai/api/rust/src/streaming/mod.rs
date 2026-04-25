//! WIA AI Streaming Module
//!
//! This module provides streaming capabilities for the WIA AI protocol,
//! including SSE (Server-Sent Events) support and stream handling.
//!
//! ## Features
//!
//! - SSE event parsing and generation
//! - Stream handler for processing streaming responses
//! - Async stream support with tokio
//!
//! ## Example
//!
//! ```rust,no_run
//! use wia_ai::streaming::*;
//! use futures_util::StreamExt;
//!
//! async fn example() {
//!     let events = vec![
//!         "event: stream_start\ndata: {\"stream_id\": \"s-001\"}\n\n",
//!         "event: stream_delta\ndata: {\"delta\": {\"type\": \"text_delta\", \"text\": \"Hello\"}}\n\n",
//!         "event: stream_end\ndata: {\"stop_reason\": \"end_turn\"}\n\n",
//!     ];
//!
//!     // Parse SSE events
//!     for event_str in events {
//!         let event = SseEvent::parse(event_str).unwrap();
//!         println!("Event: {:?}", event.event_type);
//!     }
//! }
//! ```

mod sse;
mod stream_handler;

pub use sse::*;
pub use stream_handler::*;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sse_parse() {
        let event_str = "event: stream_delta\ndata: {\"text\": \"Hello\"}\n\n";
        let event = SseEvent::parse(event_str).unwrap();

        assert_eq!(event.event_type, Some("stream_delta".to_string()));
        assert_eq!(event.data, Some("{\"text\": \"Hello\"}".to_string()));
    }

    #[test]
    fn test_sse_format() {
        let event = SseEvent::new()
            .event("message")
            .data("{\"content\": \"test\"}");

        let formatted = event.format();
        assert!(formatted.contains("event: message"));
        assert!(formatted.contains("data: {\"content\": \"test\"}"));
    }
}
