//! WIA Smart Home Simulator Adapter
//! 弘益人間 - Benefit All Humanity
//!
//! A simulated adapter for testing and development purposes.

use crate::core::{DeviceAdapter, NotificationService};
use crate::error::{Result, SmartHomeError};
use crate::types::*;
use async_trait::async_trait;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use uuid::Uuid;

// ============================================================================
// Simulator Device Adapter
// ============================================================================

/// Simulated device state
#[derive(Debug, Clone)]
pub struct SimulatedDevice {
    pub device_id: Uuid,
    pub status: DeviceStatus,
    pub state: HashMap<String, serde_json::Value>,
}

/// Simulator device adapter for testing
pub struct SimulatorDeviceAdapter {
    devices: Arc<RwLock<HashMap<Uuid, SimulatedDevice>>>,
    response_delay_ms: u64,
}

impl SimulatorDeviceAdapter {
    /// Create new simulator adapter
    pub fn new() -> Self {
        Self {
            devices: Arc::new(RwLock::new(HashMap::new())),
            response_delay_ms: 100,
        }
    }

    /// Set simulated response delay
    pub fn with_delay(mut self, delay_ms: u64) -> Self {
        self.response_delay_ms = delay_ms;
        self
    }

    /// Add simulated device
    pub async fn add_device(&self, device_id: Uuid) {
        let device = SimulatedDevice {
            device_id,
            status: DeviceStatus::Online,
            state: HashMap::new(),
        };
        self.devices.write().await.insert(device_id, device);
    }

    /// Set device status
    pub async fn set_device_status(&self, device_id: Uuid, status: DeviceStatus) -> Result<()> {
        let mut devices = self.devices.write().await;
        if let Some(device) = devices.get_mut(&device_id) {
            device.status = status;
            Ok(())
        } else {
            Err(SmartHomeError::DeviceNotFound(device_id))
        }
    }

    /// Simulate delay
    async fn simulate_delay(&self) {
        if self.response_delay_ms > 0 {
            tokio::time::sleep(tokio::time::Duration::from_millis(self.response_delay_ms)).await;
        }
    }
}

impl Default for SimulatorDeviceAdapter {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl DeviceAdapter for SimulatorDeviceAdapter {
    async fn get_status(&self, device_id: Uuid) -> Result<DeviceStatus> {
        self.simulate_delay().await;
        self.devices
            .read()
            .await
            .get(&device_id)
            .map(|d| d.status)
            .ok_or(SmartHomeError::DeviceNotFound(device_id))
    }

    async fn send_command(
        &self,
        device_id: Uuid,
        command: &str,
        params: &HashMap<String, serde_json::Value>,
    ) -> Result<()> {
        self.simulate_delay().await;

        let mut devices = self.devices.write().await;
        let device = devices
            .get_mut(&device_id)
            .ok_or(SmartHomeError::DeviceNotFound(device_id))?;

        if device.status != DeviceStatus::Online {
            return Err(SmartHomeError::DeviceOffline(device_id));
        }

        // Simulate command execution by updating state
        match command {
            "on" => {
                device
                    .state
                    .insert("power".to_string(), serde_json::json!(true));
            }
            "off" => {
                device
                    .state
                    .insert("power".to_string(), serde_json::json!(false));
            }
            "set_brightness" => {
                if let Some(level) = params.get("level") {
                    device.state.insert("brightness".to_string(), level.clone());
                }
            }
            "set_color" => {
                if let Some(color) = params.get("color") {
                    device.state.insert("color".to_string(), color.clone());
                }
            }
            "set_temperature" => {
                if let Some(temp) = params.get("temperature") {
                    device.state.insert("temperature".to_string(), temp.clone());
                }
            }
            "lock" => {
                device
                    .state
                    .insert("locked".to_string(), serde_json::json!(true));
            }
            "unlock" => {
                device
                    .state
                    .insert("locked".to_string(), serde_json::json!(false));
            }
            _ => {
                // Generic command - store in state
                device
                    .state
                    .insert(format!("last_command_{}", command), serde_json::json!(params));
            }
        }

        Ok(())
    }

    async fn get_state(&self, device_id: Uuid) -> Result<HashMap<String, serde_json::Value>> {
        self.simulate_delay().await;
        self.devices
            .read()
            .await
            .get(&device_id)
            .map(|d| d.state.clone())
            .ok_or(SmartHomeError::DeviceNotFound(device_id))
    }

    async fn set_state(
        &self,
        device_id: Uuid,
        state: HashMap<String, serde_json::Value>,
    ) -> Result<()> {
        self.simulate_delay().await;
        let mut devices = self.devices.write().await;
        let device = devices
            .get_mut(&device_id)
            .ok_or(SmartHomeError::DeviceNotFound(device_id))?;

        if device.status != DeviceStatus::Online {
            return Err(SmartHomeError::DeviceOffline(device_id));
        }

        device.state = state;
        Ok(())
    }
}

// ============================================================================
// Simulator Notification Service
// ============================================================================

/// Notification log entry
#[derive(Debug, Clone)]
pub struct NotificationLogEntry {
    pub notification: Notification,
    pub timestamp: chrono::DateTime<chrono::Utc>,
}

/// Announcement log entry
#[derive(Debug, Clone)]
pub struct AnnouncementLogEntry {
    pub text: String,
    pub zone_ids: Vec<Uuid>,
    pub timestamp: chrono::DateTime<chrono::Utc>,
}

/// Haptic log entry
#[derive(Debug, Clone)]
pub struct HapticLogEntry {
    pub device_ids: Vec<Uuid>,
    pub pattern: HapticPattern,
    pub timestamp: chrono::DateTime<chrono::Utc>,
}

/// Simulator notification service for testing
pub struct SimulatorNotificationService {
    notifications: Arc<RwLock<Vec<NotificationLogEntry>>>,
    announcements: Arc<RwLock<Vec<AnnouncementLogEntry>>>,
    haptics: Arc<RwLock<Vec<HapticLogEntry>>>,
    print_output: bool,
}

impl SimulatorNotificationService {
    /// Create new simulator notification service
    pub fn new() -> Self {
        Self {
            notifications: Arc::new(RwLock::new(Vec::new())),
            announcements: Arc::new(RwLock::new(Vec::new())),
            haptics: Arc::new(RwLock::new(Vec::new())),
            print_output: false,
        }
    }

    /// Enable printing output to console
    pub fn with_print_output(mut self, enabled: bool) -> Self {
        self.print_output = enabled;
        self
    }

    /// Get notification log
    pub async fn get_notifications(&self) -> Vec<NotificationLogEntry> {
        self.notifications.read().await.clone()
    }

    /// Get announcement log
    pub async fn get_announcements(&self) -> Vec<AnnouncementLogEntry> {
        self.announcements.read().await.clone()
    }

    /// Get haptic log
    pub async fn get_haptics(&self) -> Vec<HapticLogEntry> {
        self.haptics.read().await.clone()
    }

    /// Clear all logs
    pub async fn clear_logs(&self) {
        self.notifications.write().await.clear();
        self.announcements.write().await.clear();
        self.haptics.write().await.clear();
    }
}

impl Default for SimulatorNotificationService {
    fn default() -> Self {
        Self::new()
    }
}

#[async_trait]
impl NotificationService for SimulatorNotificationService {
    async fn send(&self, notification: &Notification) -> Result<()> {
        let entry = NotificationLogEntry {
            notification: notification.clone(),
            timestamp: chrono::Utc::now(),
        };

        if self.print_output {
            let msg = notification
                .message
                .get("default")
                .map(|s| s.as_str())
                .unwrap_or("(no message)");
            println!(
                "[NOTIFICATION] {:?} - {:?}: {}",
                notification.notification_type, notification.priority, msg
            );
        }

        self.notifications.write().await.push(entry);
        Ok(())
    }

    async fn announce(&self, text: &str, zone_ids: &[Uuid]) -> Result<()> {
        let entry = AnnouncementLogEntry {
            text: text.to_string(),
            zone_ids: zone_ids.to_vec(),
            timestamp: chrono::Utc::now(),
        };

        if self.print_output {
            println!("[TTS] {}", text);
        }

        self.announcements.write().await.push(entry);
        Ok(())
    }

    async fn haptic(&self, device_ids: &[Uuid], pattern: HapticPattern) -> Result<()> {
        let entry = HapticLogEntry {
            device_ids: device_ids.to_vec(),
            pattern,
            timestamp: chrono::Utc::now(),
        };

        if self.print_output {
            println!("[HAPTIC] {:?} -> {:?}", pattern, device_ids);
        }

        self.haptics.write().await.push(entry);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_simulator_device_adapter() {
        let adapter = SimulatorDeviceAdapter::new().with_delay(0);
        let device_id = Uuid::new_v4();

        // Add device
        adapter.add_device(device_id).await;

        // Check status
        let status = adapter.get_status(device_id).await.unwrap();
        assert_eq!(status, DeviceStatus::Online);

        // Send command
        adapter
            .send_command(device_id, "on", &HashMap::new())
            .await
            .unwrap();

        // Check state
        let state = adapter.get_state(device_id).await.unwrap();
        assert_eq!(state.get("power"), Some(&serde_json::json!(true)));
    }

    #[tokio::test]
    async fn test_simulator_notification_service() {
        let service = SimulatorNotificationService::new();

        // Send announcement
        service.announce("Hello, World!", &[]).await.unwrap();

        // Check log
        let announcements = service.get_announcements().await;
        assert_eq!(announcements.len(), 1);
        assert_eq!(announcements[0].text, "Hello, World!");
    }

    #[tokio::test]
    async fn test_device_offline() {
        let adapter = SimulatorDeviceAdapter::new().with_delay(0);
        let device_id = Uuid::new_v4();

        adapter.add_device(device_id).await;
        adapter
            .set_device_status(device_id, DeviceStatus::Offline)
            .await
            .unwrap();

        // Command should fail when offline
        let result = adapter
            .send_command(device_id, "on", &HashMap::new())
            .await;
        assert!(result.is_err());
    }
}
