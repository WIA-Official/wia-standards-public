//! WIA Eye Gaze Standard - Gaze Aware Application
//!
//! 弘益人間 - 널리 인간을 이롭게

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};

use crate::types::GazeTarget;

/// App message types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GazeAppMessageType {
    AppAnnounce,
    AppQuery,
    AppResponse,
    ControlRequest,
    ControlGrant,
    ControlDeny,
    ControlRelease,
    PauseTracking,
    ResumeTracking,
    TargetSync,
}

/// Application capabilities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AppCapabilities {
    pub name: String,
    pub version: String,
    pub supports_control: bool,
    pub supports_dwell: bool,
    pub supports_blink: bool,
    pub priority: i32,
}

/// Inter-app message
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GazeAppMessage {
    pub message_type: GazeAppMessageType,
    pub app_id: String,
    pub timestamp: u64,
    pub payload: Option<serde_json::Value>,
}

/// Control request handler type
pub type ControlRequestHandler = Box<dyn Fn(&str) -> bool + Send + Sync>;

/// Message handler type
pub type MessageHandler = Box<dyn Fn(&GazeAppMessage) + Send + Sync>;

/// Gaze-aware application
pub struct GazeAwareApp {
    app_id: String,
    capabilities: AppCapabilities,
    registered: bool,
    has_control: bool,
    targets: HashMap<String, GazeTarget>,
    control_request_handler: Option<ControlRequestHandler>,
    message_handlers: Vec<MessageHandler>,
    known_apps: HashMap<String, AppCapabilities>,
}

impl GazeAwareApp {
    /// Create new gaze-aware app
    pub fn new(app_id: impl Into<String>, capabilities: AppCapabilities) -> Self {
        Self {
            app_id: app_id.into(),
            capabilities,
            registered: false,
            has_control: false,
            targets: HashMap::new(),
            control_request_handler: None,
            message_handlers: Vec::new(),
            known_apps: HashMap::new(),
        }
    }

    /// Get app ID
    pub fn app_id(&self) -> &str {
        &self.app_id
    }

    /// Register with ecosystem
    pub async fn register(&mut self) -> Result<(), String> {
        if self.registered {
            return Ok(());
        }

        self.registered = true;

        // Announce presence
        self.send_message(GazeAppMessage {
            message_type: GazeAppMessageType::AppAnnounce,
            app_id: self.app_id.clone(),
            timestamp: current_timestamp(),
            payload: Some(serde_json::to_value(&self.capabilities).unwrap()),
        });

        Ok(())
    }

    /// Unregister from ecosystem
    pub async fn unregister(&mut self) {
        if !self.registered {
            return;
        }

        if self.has_control {
            self.release_gaze_control();
        }

        self.registered = false;
    }

    /// Check if registered
    pub fn is_registered(&self) -> bool {
        self.registered
    }

    /// Announce taking gaze control
    pub fn announce_gaze_control(&mut self) {
        self.send_message(GazeAppMessage {
            message_type: GazeAppMessageType::ControlRequest,
            app_id: self.app_id.clone(),
            timestamp: current_timestamp(),
            payload: Some(serde_json::json!({ "priority": self.capabilities.priority })),
        });
    }

    /// Release gaze control
    pub fn release_gaze_control(&mut self) {
        if !self.has_control {
            return;
        }

        self.has_control = false;
        self.send_message(GazeAppMessage {
            message_type: GazeAppMessageType::ControlRelease,
            app_id: self.app_id.clone(),
            timestamp: current_timestamp(),
            payload: None,
        });
    }

    /// Check if has control
    pub fn has_gaze_control(&self) -> bool {
        self.has_control
    }

    /// Register control request handler
    pub fn on_gaze_control_request<F>(&mut self, handler: F)
    where
        F: Fn(&str) -> bool + Send + Sync + 'static,
    {
        self.control_request_handler = Some(Box::new(handler));
    }

    /// Register target
    pub fn register_target(&mut self, target: GazeTarget) {
        self.targets.insert(target.element_id.clone(), target);
    }

    /// Unregister target
    pub fn unregister_target(&mut self, target_id: &str) {
        self.targets.remove(target_id);
    }

    /// Get active targets
    pub fn get_active_targets(&self) -> Vec<GazeTarget> {
        self.targets.values().cloned().collect()
    }

    /// Get known apps
    pub fn get_known_apps(&self) -> &HashMap<String, AppCapabilities> {
        &self.known_apps
    }

    /// Register message handler
    pub fn on_message<F>(&mut self, handler: F)
    where
        F: Fn(&GazeAppMessage) + Send + Sync + 'static,
    {
        self.message_handlers.push(Box::new(handler));
    }

    /// Handle incoming message
    pub fn handle_message(&mut self, message: GazeAppMessage) {
        if message.app_id == self.app_id {
            return;
        }

        match message.message_type {
            GazeAppMessageType::AppAnnounce => {
                if let Some(payload) = &message.payload {
                    if let Ok(caps) = serde_json::from_value::<AppCapabilities>(payload.clone()) {
                        self.known_apps.insert(message.app_id.clone(), caps);
                    }
                }
            }
            GazeAppMessageType::ControlRequest => {
                if self.has_control {
                    let granted = self
                        .control_request_handler
                        .as_ref()
                        .map(|h| h(&message.app_id))
                        .unwrap_or(false);

                    self.send_message(GazeAppMessage {
                        message_type: if granted {
                            GazeAppMessageType::ControlGrant
                        } else {
                            GazeAppMessageType::ControlDeny
                        },
                        app_id: self.app_id.clone(),
                        timestamp: current_timestamp(),
                        payload: Some(serde_json::json!({ "requesting_app": message.app_id })),
                    });

                    if granted {
                        self.has_control = false;
                    }
                }
            }
            GazeAppMessageType::ControlGrant => {
                self.has_control = true;
            }
            _ => {}
        }

        // Forward to handlers
        for handler in &self.message_handlers {
            handler(&message);
        }
    }

    fn send_message(&self, message: GazeAppMessage) {
        if !self.registered {
            return;
        }
        // In real implementation, send via transport
        let _ = message;
    }
}

fn current_timestamp() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_millis() as u64
}
