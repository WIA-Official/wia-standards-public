//! Communication channel abstractions

use crate::error::{NanoError, NanoResult};
use super::{ProtocolMessage, MessageType};
use async_trait::async_trait;
use tokio::sync::mpsc;

/// Communication channel trait
#[async_trait]
pub trait Channel: Send + Sync {
    /// Send a message
    async fn send(&mut self, message: ProtocolMessage) -> NanoResult<()>;

    /// Receive a message (blocking)
    async fn receive(&mut self) -> NanoResult<ProtocolMessage>;

    /// Try to receive without blocking
    fn try_receive(&mut self) -> Option<ProtocolMessage>;

    /// Check if channel is connected
    fn is_connected(&self) -> bool;

    /// Get channel ID
    fn channel_id(&self) -> &str;

    /// Close the channel
    async fn close(&mut self) -> NanoResult<()>;
}

/// Channel configuration
#[derive(Debug, Clone)]
pub struct ChannelConfig {
    pub buffer_size: usize,
    pub timeout_ms: u64,
    pub retry_attempts: u8,
    pub enable_checksum: bool,
    pub enable_encryption: bool,
}

impl Default for ChannelConfig {
    fn default() -> Self {
        Self {
            buffer_size: 1024,
            timeout_ms: 5000,
            retry_attempts: 3,
            enable_checksum: true,
            enable_encryption: false,
        }
    }
}

/// In-memory channel for simulation
pub struct MemoryChannel {
    id: String,
    tx: mpsc::Sender<ProtocolMessage>,
    rx: mpsc::Receiver<ProtocolMessage>,
    connected: bool,
    config: ChannelConfig,
}

impl MemoryChannel {
    pub fn new(id: impl Into<String>, config: ChannelConfig) -> (Self, mpsc::Sender<ProtocolMessage>) {
        let (tx, rx) = mpsc::channel(config.buffer_size);
        let (return_tx, _) = mpsc::channel(config.buffer_size);

        let channel = Self {
            id: id.into(),
            tx: return_tx,
            rx,
            connected: true,
            config,
        };

        (channel, tx)
    }

    pub fn pair(id1: impl Into<String>, id2: impl Into<String>, config: ChannelConfig) -> (Self, Self) {
        let (tx1, rx1) = mpsc::channel(config.buffer_size);
        let (tx2, rx2) = mpsc::channel(config.buffer_size);

        let ch1 = Self {
            id: id1.into(),
            tx: tx2,
            rx: rx1,
            connected: true,
            config: config.clone(),
        };

        let ch2 = Self {
            id: id2.into(),
            tx: tx1,
            rx: rx2,
            connected: true,
            config,
        };

        (ch1, ch2)
    }
}

#[async_trait]
impl Channel for MemoryChannel {
    async fn send(&mut self, message: ProtocolMessage) -> NanoResult<()> {
        if !self.connected {
            return Err(NanoError::Communication("Channel not connected".into()));
        }

        let msg = if self.config.enable_checksum {
            message.with_checksum()
        } else {
            message
        };

        self.tx
            .send(msg)
            .await
            .map_err(|e| NanoError::Communication(format!("Send failed: {}", e)))
    }

    async fn receive(&mut self) -> NanoResult<ProtocolMessage> {
        if !self.connected {
            return Err(NanoError::Communication("Channel not connected".into()));
        }

        let timeout = tokio::time::Duration::from_millis(self.config.timeout_ms);

        match tokio::time::timeout(timeout, self.rx.recv()).await {
            Ok(Some(msg)) => {
                if self.config.enable_checksum && !msg.verify_checksum() {
                    Err(NanoError::Communication("Checksum verification failed".into()))
                } else {
                    Ok(msg)
                }
            }
            Ok(None) => Err(NanoError::Communication("Channel closed".into())),
            Err(_) => Err(NanoError::Timeout("Receive timeout".into())),
        }
    }

    fn try_receive(&mut self) -> Option<ProtocolMessage> {
        self.rx.try_recv().ok()
    }

    fn is_connected(&self) -> bool {
        self.connected
    }

    fn channel_id(&self) -> &str {
        &self.id
    }

    async fn close(&mut self) -> NanoResult<()> {
        self.connected = false;
        Ok(())
    }
}

/// Broadcast channel for one-to-many communication
pub struct BroadcastChannel {
    id: String,
    subscribers: Vec<mpsc::Sender<ProtocolMessage>>,
    connected: bool,
}

impl BroadcastChannel {
    pub fn new(id: impl Into<String>) -> Self {
        Self {
            id: id.into(),
            subscribers: Vec::new(),
            connected: true,
        }
    }

    pub fn subscribe(&mut self) -> mpsc::Receiver<ProtocolMessage> {
        let (tx, rx) = mpsc::channel(256);
        self.subscribers.push(tx);
        rx
    }

    pub async fn broadcast(&self, message: ProtocolMessage) -> NanoResult<usize> {
        if !self.connected {
            return Err(NanoError::Communication("Broadcast channel not connected".into()));
        }

        let mut success_count = 0;
        for tx in &self.subscribers {
            if tx.send(message.clone()).await.is_ok() {
                success_count += 1;
            }
        }

        Ok(success_count)
    }

    pub fn subscriber_count(&self) -> usize {
        self.subscribers.len()
    }
}

/// Channel statistics
#[derive(Debug, Clone, Default)]
pub struct ChannelStats {
    pub messages_sent: u64,
    pub messages_received: u64,
    pub bytes_sent: u64,
    pub bytes_received: u64,
    pub errors: u64,
    pub retries: u64,
    pub avg_latency_ns: u64,
}

/// Quality of Service levels
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QoS {
    /// Best effort, no guarantees
    BestEffort,
    /// At most once delivery
    AtMostOnce,
    /// At least once delivery (may duplicate)
    AtLeastOnce,
    /// Exactly once delivery
    ExactlyOnce,
}
