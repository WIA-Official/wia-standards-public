//! WIA Eye Gaze Standard - WebSocket Server
//!
//! Real-time gaze data streaming server using Axum.
//!
//! 弘益人間 - 널리 인간을 이롭게

#[cfg(feature = "server")]
use std::collections::HashMap;
#[cfg(feature = "server")]
use std::net::SocketAddr;
#[cfg(feature = "server")]
use std::sync::Arc;
#[cfg(feature = "server")]
use std::time::Duration;

#[cfg(feature = "server")]
use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        State,
    },
    response::IntoResponse,
    routing::get,
    Router,
};
#[cfg(feature = "server")]
use futures_util::{SinkExt, StreamExt};
#[cfg(feature = "server")]
use serde::{Deserialize, Serialize};
#[cfg(feature = "server")]
use tokio::sync::{broadcast, Mutex, RwLock};
#[cfg(feature = "server")]
use tower_http::cors::CorsLayer;

#[cfg(feature = "server")]
use crate::binary::{encode_batch, encode_frame, MessageType};
#[cfg(feature = "server")]
use crate::types::GazePoint;

/// Default WebSocket server port
pub const DEFAULT_PORT: u16 = 8765;

/// Default streaming frequency (Hz)
pub const DEFAULT_FREQUENCY: u32 = 60;

/// Maximum batch size for gaze points
pub const MAX_BATCH_SIZE: usize = 120;

/// WebSocket message types (JSON)
#[cfg(feature = "server")]
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum WsMessage {
    /// Gaze data stream
    GazeData {
        timestamp: u64,
        sequence: u64,
        frequency: u32,
        points: Vec<GazePointJson>,
    },
    /// Gaze event
    GazeEvent {
        timestamp: u64,
        event: GazeEventJson,
    },
    /// Status update
    Status {
        timestamp: u64,
        status: ServerStatus,
    },
    /// Control command (client -> server)
    Control {
        timestamp: u64,
        request_id: Option<String>,
        action: ControlAction,
        params: Option<serde_json::Value>,
    },
    /// Error message
    Error {
        timestamp: u64,
        error: ErrorInfo,
    },
    /// Ping
    Ping { timestamp: u64 },
    /// Pong
    Pong {
        timestamp: u64,
        server_time: u64,
    },
}

#[cfg(feature = "server")]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GazePointJson {
    pub t: u64,
    pub x: f64,
    pub y: f64,
    pub c: f64,
    pub v: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub le: Option<EyeDataJson>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub re: Option<EyeDataJson>,
}

#[cfg(feature = "server")]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EyeDataJson {
    pub x: f64,
    pub y: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub p: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub o: Option<f64>,
}

#[cfg(feature = "server")]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GazeEventJson {
    #[serde(rename = "eventType")]
    pub event_type: String,
    #[serde(rename = "eventId")]
    pub event_id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration: Option<u64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub position: Option<Position>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub target: Option<TargetJson>,
}

#[cfg(feature = "server")]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Position {
    pub x: f64,
    pub y: f64,
}

#[cfg(feature = "server")]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TargetJson {
    #[serde(rename = "elementId")]
    pub element_id: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
}

#[cfg(feature = "server")]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServerStatus {
    pub connected: bool,
    pub tracking: bool,
    pub calibrated: bool,
    pub state: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub device: Option<DeviceInfo>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metrics: Option<StreamMetrics>,
}

#[cfg(feature = "server")]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceInfo {
    pub vendor: String,
    pub model: String,
    pub firmware: String,
}

#[cfg(feature = "server")]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StreamMetrics {
    #[serde(rename = "samplingRate")]
    pub sampling_rate: u32,
    pub latency: f64,
    #[serde(rename = "packetsDropped")]
    pub packets_dropped: u64,
}

#[cfg(feature = "server")]
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlAction {
    Start,
    Stop,
    Pause,
    Resume,
    Calibrate,
    SetFrequency,
    SetFormat,
    Subscribe,
    Unsubscribe,
}

#[cfg(feature = "server")]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ErrorInfo {
    pub code: String,
    pub message: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<serde_json::Value>,
    pub recoverable: bool,
}

/// Server configuration
#[cfg(feature = "server")]
#[derive(Debug, Clone)]
pub struct ServerConfig {
    /// Server port
    pub port: u16,
    /// Streaming frequency (Hz)
    pub frequency: u32,
    /// Use binary format
    pub binary_mode: bool,
    /// Enable CORS
    pub enable_cors: bool,
    /// Maximum clients
    pub max_clients: usize,
}

#[cfg(feature = "server")]
impl Default for ServerConfig {
    fn default() -> Self {
        Self {
            port: DEFAULT_PORT,
            frequency: DEFAULT_FREQUENCY,
            binary_mode: false,
            enable_cors: true,
            max_clients: 100,
        }
    }
}

/// Shared server state
#[cfg(feature = "server")]
pub struct ServerState {
    /// Gaze data broadcast channel
    gaze_tx: broadcast::Sender<GazePoint>,
    /// Active client count
    client_count: Mutex<usize>,
    /// Server configuration
    config: RwLock<ServerConfig>,
    /// Server status
    status: RwLock<ServerStatus>,
    /// Sequence counter
    sequence: Mutex<u64>,
}

#[cfg(feature = "server")]
impl ServerState {
    /// Create new server state
    pub fn new(config: ServerConfig) -> Self {
        let (gaze_tx, _) = broadcast::channel(1024);
        Self {
            gaze_tx,
            client_count: Mutex::new(0),
            config: RwLock::new(config),
            status: RwLock::new(ServerStatus {
                connected: false,
                tracking: false,
                calibrated: false,
                state: "disconnected".to_string(),
                device: None,
                metrics: None,
            }),
            sequence: Mutex::new(0),
        }
    }

    /// Broadcast gaze point to all clients
    pub fn broadcast_gaze(&self, point: GazePoint) {
        let _ = self.gaze_tx.send(point);
    }

    /// Subscribe to gaze data
    pub fn subscribe(&self) -> broadcast::Receiver<GazePoint> {
        self.gaze_tx.subscribe()
    }

    /// Update server status
    pub async fn update_status(&self, status: ServerStatus) {
        let mut s = self.status.write().await;
        *s = status;
    }

    /// Get current status
    pub async fn get_status(&self) -> ServerStatus {
        self.status.read().await.clone()
    }

    /// Get next sequence number
    pub async fn next_sequence(&self) -> u64 {
        let mut seq = self.sequence.lock().await;
        *seq += 1;
        *seq
    }
}

/// WIA Eye Gaze WebSocket Server
#[cfg(feature = "server")]
pub struct GazeServer {
    state: Arc<ServerState>,
    config: ServerConfig,
}

#[cfg(feature = "server")]
impl GazeServer {
    /// Create new server with default config
    pub fn new() -> Self {
        Self::with_config(ServerConfig::default())
    }

    /// Create new server with config
    pub fn with_config(config: ServerConfig) -> Self {
        Self {
            state: Arc::new(ServerState::new(config.clone())),
            config,
        }
    }

    /// Get shared state for broadcasting
    pub fn state(&self) -> Arc<ServerState> {
        Arc::clone(&self.state)
    }

    /// Build the Axum router
    pub fn router(&self) -> Router {
        let mut router = Router::new()
            .route("/wia-eye-gaze/v1/stream", get(ws_handler))
            .route("/wia-eye-gaze/v1/status", get(status_handler))
            .route("/health", get(health_handler))
            .with_state(Arc::clone(&self.state));

        if self.config.enable_cors {
            router = router.layer(CorsLayer::permissive());
        }

        router
    }

    /// Start the server
    pub async fn start(&self) -> Result<(), Box<dyn std::error::Error>> {
        let addr = SocketAddr::from(([0, 0, 0, 0], self.config.port));
        let listener = tokio::net::TcpListener::bind(addr).await?;

        println!(
            "WIA Eye Gaze Server listening on ws://{}:{}/wia-eye-gaze/v1/stream",
            addr.ip(),
            addr.port()
        );

        axum::serve(listener, self.router()).await?;
        Ok(())
    }
}

#[cfg(feature = "server")]
impl Default for GazeServer {
    fn default() -> Self {
        Self::new()
    }
}

/// WebSocket handler
#[cfg(feature = "server")]
async fn ws_handler(
    ws: WebSocketUpgrade,
    State(state): State<Arc<ServerState>>,
) -> impl IntoResponse {
    ws.protocols(["wia-eye-gaze-v1"])
        .on_upgrade(move |socket| handle_socket(socket, state))
}

/// Handle WebSocket connection
#[cfg(feature = "server")]
async fn handle_socket(socket: WebSocket, state: Arc<ServerState>) {
    // Increment client count
    {
        let mut count = state.client_count.lock().await;
        *count += 1;
        println!("Client connected. Total: {}", *count);
    }

    let (mut sender, mut receiver) = socket.split();

    // Subscribe to gaze data
    let mut gaze_rx = state.subscribe();
    let config = state.config.read().await.clone();

    // Send initial status
    let status = state.get_status().await;
    let status_msg = WsMessage::Status {
        timestamp: now_ms(),
        status,
    };
    if let Ok(json) = serde_json::to_string(&status_msg) {
        let _ = sender.send(Message::Text(json)).await;
    }

    // Streaming state
    let mut streaming = false;
    let mut binary_mode = config.binary_mode;
    let mut batch_interval = Duration::from_millis(1000 / config.frequency as u64);
    let mut batch: Vec<GazePoint> = Vec::with_capacity(MAX_BATCH_SIZE);

    loop {
        tokio::select! {
            // Receive gaze data from tracker
            result = gaze_rx.recv() => {
                if let Ok(point) = result {
                    if streaming {
                        batch.push(point);

                        // Send batch when interval elapsed or batch full
                        if batch.len() >= config.frequency as usize / 60 {
                            let sequence = state.next_sequence().await;

                            if binary_mode {
                                let encoded = encode_batch(&batch);
                                let frame = encode_frame(MessageType::GazeBatch, &encoded);
                                if sender.send(Message::Binary(frame)).await.is_err() {
                                    break;
                                }
                            } else {
                                let points_json: Vec<GazePointJson> = batch.iter()
                                    .map(gaze_to_json)
                                    .collect();

                                let msg = WsMessage::GazeData {
                                    timestamp: now_ms(),
                                    sequence,
                                    frequency: config.frequency,
                                    points: points_json,
                                };

                                if let Ok(json) = serde_json::to_string(&msg) {
                                    if sender.send(Message::Text(json)).await.is_err() {
                                        break;
                                    }
                                }
                            }

                            batch.clear();
                        }
                    }
                }
            }

            // Receive control messages from client
            Some(msg) = receiver.next() => {
                match msg {
                    Ok(Message::Text(text)) => {
                        if let Ok(control) = serde_json::from_str::<WsMessage>(&text) {
                            match control {
                                WsMessage::Control { action, params, .. } => {
                                    match action {
                                        ControlAction::Start => {
                                            streaming = true;
                                        }
                                        ControlAction::Stop => {
                                            streaming = false;
                                            batch.clear();
                                        }
                                        ControlAction::Pause => {
                                            streaming = false;
                                        }
                                        ControlAction::Resume => {
                                            streaming = true;
                                        }
                                        ControlAction::SetFormat => {
                                            if let Some(params) = params {
                                                if let Some(format) = params.get("format") {
                                                    binary_mode = format == "binary";
                                                }
                                            }
                                        }
                                        ControlAction::SetFrequency => {
                                            if let Some(params) = params {
                                                if let Some(freq) = params.get("frequency").and_then(|f| f.as_u64()) {
                                                    batch_interval = Duration::from_millis(1000 / freq);
                                                }
                                            }
                                        }
                                        _ => {}
                                    }
                                }
                                WsMessage::Ping { timestamp } => {
                                    let pong = WsMessage::Pong {
                                        timestamp,
                                        server_time: now_ms(),
                                    };
                                    if let Ok(json) = serde_json::to_string(&pong) {
                                        let _ = sender.send(Message::Text(json)).await;
                                    }
                                }
                                _ => {}
                            }
                        }
                    }
                    Ok(Message::Close(_)) => break,
                    Err(_) => break,
                    _ => {}
                }
            }
        }
    }

    // Decrement client count
    {
        let mut count = state.client_count.lock().await;
        *count -= 1;
        println!("Client disconnected. Total: {}", *count);
    }
}

/// Status endpoint handler
#[cfg(feature = "server")]
async fn status_handler(State(state): State<Arc<ServerState>>) -> impl IntoResponse {
    let status = state.get_status().await;
    axum::Json(status)
}

/// Health check handler
#[cfg(feature = "server")]
async fn health_handler() -> &'static str {
    "OK"
}

/// Convert GazePoint to JSON format
#[cfg(feature = "server")]
fn gaze_to_json(point: &GazePoint) -> GazePointJson {
    GazePointJson {
        t: point.timestamp,
        x: point.x,
        y: point.y,
        c: point.confidence,
        v: point.valid,
        le: point.left_eye.as_ref().map(|e| EyeDataJson {
            x: e.gaze.x,
            y: e.gaze.y,
            p: e.pupil_diameter,
            o: e.eye_openness,
        }),
        re: point.right_eye.as_ref().map(|e| EyeDataJson {
            x: e.gaze.x,
            y: e.gaze.y,
            p: e.pupil_diameter,
            o: e.eye_openness,
        }),
    }
}

/// Get current timestamp in milliseconds
#[cfg(feature = "server")]
fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_millis() as u64
}

#[cfg(all(test, feature = "server"))]
mod tests {
    use super::*;

    #[test]
    fn test_server_config_default() {
        let config = ServerConfig::default();
        assert_eq!(config.port, 8765);
        assert_eq!(config.frequency, 60);
    }

    #[tokio::test]
    async fn test_server_state() {
        let state = ServerState::new(ServerConfig::default());

        let seq1 = state.next_sequence().await;
        let seq2 = state.next_sequence().await;

        assert_eq!(seq1, 1);
        assert_eq!(seq2, 2);
    }
}
