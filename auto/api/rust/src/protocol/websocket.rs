//! WebSocket handler for real-time communication

use async_trait::async_trait;
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use tokio::sync::{mpsc, RwLock};
use uuid::Uuid;

use crate::error::{Error, Result};
use crate::types::*;
use super::{
    ClientConfig, HmiCommandPayload, HmiCommandType, SubscribePayload,
    TripStatusPayload, VehicleLocationPayload, WsEventType, WsMessage,
};

/// WebSocket event handler trait
#[async_trait]
pub trait WsEventHandler: Send + Sync {
    /// Handle trip status update
    async fn on_trip_status(&self, payload: TripStatusPayload);

    /// Handle vehicle location update
    async fn on_vehicle_location(&self, payload: VehicleLocationPayload);

    /// Handle securement status change
    async fn on_securement_status(&self, status: SecurementStatus);

    /// Handle emergency alert
    async fn on_emergency(&self, event: EmergencyEvent);

    /// Handle HMI response
    async fn on_hmi_response(&self, success: bool, message: Option<String>);

    /// Handle connection state change
    async fn on_connection_state(&self, connected: bool);

    /// Handle error
    async fn on_error(&self, error: String);
}

/// WebSocket client state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WsState {
    Disconnected,
    Connecting,
    Connected,
    Reconnecting,
}

/// WebSocket client for real-time communication
pub struct WsClient {
    #[allow(dead_code)]
    config: ClientConfig,
    state: Arc<RwLock<WsState>>,
    subscriptions: Arc<RwLock<HashSet<WsEventType>>>,
    handlers: Arc<RwLock<Vec<Arc<dyn WsEventHandler>>>>,
    outgoing: Arc<RwLock<Option<mpsc::Sender<WsMessage>>>>,
}

impl WsClient {
    /// Create a new WebSocket client
    pub fn new(config: ClientConfig) -> Self {
        Self {
            config,
            state: Arc::new(RwLock::new(WsState::Disconnected)),
            subscriptions: Arc::new(RwLock::new(HashSet::new())),
            handlers: Arc::new(RwLock::new(Vec::new())),
            outgoing: Arc::new(RwLock::new(None)),
        }
    }

    /// Get current connection state
    pub async fn state(&self) -> WsState {
        *self.state.read().await
    }

    /// Add an event handler
    pub async fn add_handler(&self, handler: Arc<dyn WsEventHandler>) {
        self.handlers.write().await.push(handler);
    }

    /// Subscribe to events
    pub async fn subscribe(&self, events: Vec<WsEventType>) -> Result<()> {
        // Add to local subscriptions
        {
            let mut subs = self.subscriptions.write().await;
            for event in &events {
                subs.insert(*event);
            }
        }

        // Send subscribe message if connected
        if *self.state.read().await == WsState::Connected {
            let msg = WsMessage::new(WsEventType::Subscribe, SubscribePayload {
                events,
                trip_id: None,
                vehicle_id: None,
            });
            self.send(msg).await?;
        }

        Ok(())
    }

    /// Subscribe to a specific trip
    pub async fn subscribe_trip(&self, trip_id: Uuid) -> Result<()> {
        let events = vec![
            WsEventType::TripStatusUpdated,
            WsEventType::VehicleLocationUpdated,
            WsEventType::VehicleEtaUpdated,
            WsEventType::SecurementStatusChanged,
        ];

        let msg = WsMessage::new(WsEventType::Subscribe, SubscribePayload {
            events,
            trip_id: Some(trip_id),
            vehicle_id: None,
        });

        self.send(msg).await
    }

    /// Unsubscribe from events
    pub async fn unsubscribe(&self, events: Vec<WsEventType>) -> Result<()> {
        // Remove from local subscriptions
        {
            let mut subs = self.subscriptions.write().await;
            for event in &events {
                subs.remove(event);
            }
        }

        // Send unsubscribe message if connected
        if *self.state.read().await == WsState::Connected {
            let msg = WsMessage::new(WsEventType::Unsubscribe, SubscribePayload {
                events,
                trip_id: None,
                vehicle_id: None,
            });
            self.send(msg).await?;
        }

        Ok(())
    }

    /// Send HMI command
    pub async fn send_hmi_command(
        &self,
        vehicle_id: Uuid,
        command: HmiCommandType,
        parameters: HashMap<String, String>,
    ) -> Result<()> {
        let msg = WsMessage::new(WsEventType::HmiCommand, HmiCommandPayload {
            vehicle_id,
            command,
            parameters,
        });

        self.send(msg).await
    }

    /// Send a message
    pub async fn send(&self, message: WsMessage) -> Result<()> {
        if let Some(ref tx) = *self.outgoing.read().await {
            tx.send(message).await.map_err(|e| Error::websocket(e.to_string()))?;
            Ok(())
        } else {
            Err(Error::websocket("Not connected"))
        }
    }

    /// Handle incoming message
    pub async fn handle_message(&self, message: WsMessage) {
        let handlers = self.handlers.read().await;

        match message.event_type {
            WsEventType::TripStatusUpdated => {
                if let Ok(payload) = serde_json::from_value::<TripStatusPayload>(message.payload) {
                    for handler in handlers.iter() {
                        handler.on_trip_status(payload.clone()).await;
                    }
                }
            }
            WsEventType::VehicleLocationUpdated => {
                if let Ok(payload) = serde_json::from_value::<VehicleLocationPayload>(message.payload) {
                    for handler in handlers.iter() {
                        handler.on_vehicle_location(payload.clone()).await;
                    }
                }
            }
            WsEventType::SecurementStatusChanged => {
                if let Ok(status) = serde_json::from_value::<SecurementStatus>(message.payload) {
                    for handler in handlers.iter() {
                        handler.on_securement_status(status.clone()).await;
                    }
                }
            }
            WsEventType::EmergencyAlert => {
                if let Ok(event) = serde_json::from_value::<EmergencyEvent>(message.payload) {
                    for handler in handlers.iter() {
                        handler.on_emergency(event.clone()).await;
                    }
                }
            }
            WsEventType::HmiResponse => {
                let success = message.payload.get("success")
                    .and_then(|v| v.as_bool())
                    .unwrap_or(false);
                let msg = message.payload.get("message")
                    .and_then(|v| v.as_str())
                    .map(String::from);
                for handler in handlers.iter() {
                    handler.on_hmi_response(success, msg.clone()).await;
                }
            }
            WsEventType::Ping => {
                // Respond with pong
                let pong = WsMessage::pong(message.id);
                let _ = self.send(pong).await;
            }
            _ => {}
        }
    }

    /// Notify handlers of connection state change
    #[allow(dead_code)]
    pub async fn notify_connection_state(&self, connected: bool) {
        let handlers = self.handlers.read().await;
        for handler in handlers.iter() {
            handler.on_connection_state(connected).await;
        }
    }

    /// Notify handlers of error
    #[allow(dead_code)]
    pub async fn notify_error(&self, error: String) {
        let handlers = self.handlers.read().await;
        for handler in handlers.iter() {
            handler.on_error(error.clone()).await;
        }
    }
}

/// Mock WebSocket client for testing
pub struct MockWsClient {
    messages: Arc<RwLock<Vec<WsMessage>>>,
    handlers: Arc<RwLock<Vec<Arc<dyn WsEventHandler>>>>,
}

impl MockWsClient {
    /// Create a new mock WebSocket client
    pub fn new() -> Self {
        Self {
            messages: Arc::new(RwLock::new(Vec::new())),
            handlers: Arc::new(RwLock::new(Vec::new())),
        }
    }

    /// Add an event handler
    pub async fn add_handler(&self, handler: Arc<dyn WsEventHandler>) {
        self.handlers.write().await.push(handler);
    }

    /// Get sent messages
    pub async fn sent_messages(&self) -> Vec<WsMessage> {
        self.messages.read().await.clone()
    }

    /// Clear sent messages
    pub async fn clear_messages(&self) {
        self.messages.write().await.clear();
    }

    /// Simulate receiving a message
    pub async fn simulate_receive(&self, message: WsMessage) {
        let handlers = self.handlers.read().await;

        match message.event_type {
            WsEventType::TripStatusUpdated => {
                if let Ok(payload) = serde_json::from_value::<TripStatusPayload>(message.payload) {
                    for handler in handlers.iter() {
                        handler.on_trip_status(payload.clone()).await;
                    }
                }
            }
            WsEventType::VehicleLocationUpdated => {
                if let Ok(payload) = serde_json::from_value::<VehicleLocationPayload>(message.payload) {
                    for handler in handlers.iter() {
                        handler.on_vehicle_location(payload.clone()).await;
                    }
                }
            }
            WsEventType::EmergencyAlert => {
                if let Ok(event) = serde_json::from_value::<EmergencyEvent>(message.payload) {
                    for handler in handlers.iter() {
                        handler.on_emergency(event.clone()).await;
                    }
                }
            }
            _ => {}
        }
    }

    /// Simulate trip status update
    pub async fn simulate_trip_status(
        &self,
        trip_id: Uuid,
        previous: TripStatus,
        new: TripStatus,
        vehicle_id: Option<Uuid>,
    ) {
        let msg = WsMessage::new(WsEventType::TripStatusUpdated, TripStatusPayload {
            trip_id,
            previous_status: previous,
            new_status: new,
            vehicle_id,
            eta_minutes: Some(5.0),
        });
        self.simulate_receive(msg).await;
    }

    /// Simulate vehicle location update
    pub async fn simulate_location(
        &self,
        vehicle_id: Uuid,
        lat: f64,
        lng: f64,
        eta: Option<f32>,
    ) {
        let msg = WsMessage::new(WsEventType::VehicleLocationUpdated, VehicleLocationPayload {
            vehicle_id,
            location: GeoLocation {
                latitude: lat,
                longitude: lng,
                address: None,
            },
            heading_degrees: Some(45.0),
            speed_kmh: Some(30.0),
            eta_minutes: eta,
        });
        self.simulate_receive(msg).await;
    }

    /// Send a message (stores for inspection)
    pub async fn send(&self, message: WsMessage) -> Result<()> {
        self.messages.write().await.push(message);
        Ok(())
    }
}

impl Default for MockWsClient {
    fn default() -> Self {
        Self::new()
    }
}

/// Simple event handler that collects events
pub struct CollectingHandler {
    trip_updates: Arc<RwLock<Vec<TripStatusPayload>>>,
    location_updates: Arc<RwLock<Vec<VehicleLocationPayload>>>,
    emergencies: Arc<RwLock<Vec<EmergencyEvent>>>,
    connected: Arc<RwLock<bool>>,
    errors: Arc<RwLock<Vec<String>>>,
}

impl CollectingHandler {
    pub fn new() -> Self {
        Self {
            trip_updates: Arc::new(RwLock::new(Vec::new())),
            location_updates: Arc::new(RwLock::new(Vec::new())),
            emergencies: Arc::new(RwLock::new(Vec::new())),
            connected: Arc::new(RwLock::new(false)),
            errors: Arc::new(RwLock::new(Vec::new())),
        }
    }

    pub async fn trip_updates(&self) -> Vec<TripStatusPayload> {
        self.trip_updates.read().await.clone()
    }

    pub async fn location_updates(&self) -> Vec<VehicleLocationPayload> {
        self.location_updates.read().await.clone()
    }

    pub async fn emergencies(&self) -> Vec<EmergencyEvent> {
        self.emergencies.read().await.clone()
    }

    pub async fn is_connected(&self) -> bool {
        *self.connected.read().await
    }

    pub async fn errors(&self) -> Vec<String> {
        self.errors.read().await.clone()
    }
}

impl Default for CollectingHandler {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl WsEventHandler for CollectingHandler {
    async fn on_trip_status(&self, payload: TripStatusPayload) {
        self.trip_updates.write().await.push(payload);
    }

    async fn on_vehicle_location(&self, payload: VehicleLocationPayload) {
        self.location_updates.write().await.push(payload);
    }

    async fn on_securement_status(&self, _status: SecurementStatus) {
        // Could store if needed
    }

    async fn on_emergency(&self, event: EmergencyEvent) {
        self.emergencies.write().await.push(event);
    }

    async fn on_hmi_response(&self, _success: bool, _message: Option<String>) {
        // Could store if needed
    }

    async fn on_connection_state(&self, connected: bool) {
        *self.connected.write().await = connected;
    }

    async fn on_error(&self, error: String) {
        self.errors.write().await.push(error);
    }
}
