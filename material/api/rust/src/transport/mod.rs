//! WIA Material Transport Module
//!
//! This module provides transport layer implementations for the WIA Material Protocol.
//!
//! ## Transport Options
//!
//! - **HTTP/HTTPS**: REST API client for synchronous requests
//! - **WebSocket**: Real-time streaming and bidirectional communication
//!
//! ## Example
//!
//! ```rust,ignore
//! use wia_material::transport::{HttpTransport, TransportConfig};
//!
//! let config = TransportConfig::new("https://api.example.com");
//! let transport = HttpTransport::new(config);
//! ```

mod config;
mod http;
mod websocket;

pub use config::*;
pub use http::*;
pub use websocket::*;

use async_trait::async_trait;

use crate::error::MaterialResult;
use crate::protocol::{
    CreatePayload, CreateResponsePayload, DeletePayload, DeleteResponsePayload, GetPayload,
    GetResponsePayload, Message, QueryPayload, QueryResponsePayload, UpdatePayload,
    UpdateResponsePayload,
};

/// Transport trait for protocol communication
#[async_trait]
pub trait Transport: Send + Sync {
    /// Connect to the server
    async fn connect(&mut self) -> MaterialResult<()>;

    /// Disconnect from the server
    async fn disconnect(&mut self) -> MaterialResult<()>;

    /// Check if connected
    fn is_connected(&self) -> bool;

    /// Send a query request
    async fn query(&self, payload: QueryPayload) -> MaterialResult<QueryResponsePayload>;

    /// Send a get request
    async fn get(&self, payload: GetPayload) -> MaterialResult<GetResponsePayload>;

    /// Send a create request
    async fn create(&self, payload: CreatePayload) -> MaterialResult<CreateResponsePayload>;

    /// Send an update request
    async fn update(&self, payload: UpdatePayload) -> MaterialResult<UpdateResponsePayload>;

    /// Send a delete request
    async fn delete(&self, payload: DeletePayload) -> MaterialResult<DeleteResponsePayload>;
}

/// Streaming transport trait for real-time communication
#[async_trait]
pub trait StreamingTransport: Transport {
    /// Subscribe to a channel
    async fn subscribe(
        &self,
        payload: crate::protocol::SubscribePayload,
    ) -> MaterialResult<crate::protocol::SubscribeAckPayload>;

    /// Unsubscribe from a channel
    async fn unsubscribe(
        &self,
        payload: crate::protocol::UnsubscribePayload,
    ) -> MaterialResult<crate::protocol::UnsubscribeAckPayload>;

    /// Send a message
    async fn send<T: serde::Serialize + Send + Sync>(
        &self,
        message: Message<T>,
    ) -> MaterialResult<()>;

    /// Receive the next message
    async fn receive(&mut self) -> MaterialResult<String>;
}
