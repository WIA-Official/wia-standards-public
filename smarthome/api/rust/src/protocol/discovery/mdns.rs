//! mDNS-based Device Discovery
//! 弘益人間 - Benefit All Humanity

use crate::types::DeviceType;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::net::IpAddr;
use uuid::Uuid;

/// WIA Smart Home service type
pub const WIA_SERVICE_TYPE: &str = "_wia-smarthome._tcp.local";

/// Matter service type
pub const MATTER_SERVICE_TYPE: &str = "_matter._tcp.local";

/// Accessibility features bitmask
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct AccessibilityFeatures {
    bits: u8,
}

impl AccessibilityFeatures {
    pub const VOICE_CONTROL: u8 = 0x01;
    pub const AUDIO_FEEDBACK: u8 = 0x02;
    pub const VISUAL_FEEDBACK: u8 = 0x04;
    pub const HAPTIC_FEEDBACK: u8 = 0x08;
    pub const SWITCH_ACCESS: u8 = 0x10;
    pub const DWELL_CONTROL: u8 = 0x20;
    pub const BCI_SUPPORT: u8 = 0x40;
    pub const KOREAN_TTS: u8 = 0x80;

    pub fn new() -> Self {
        Self { bits: 0 }
    }

    pub fn all() -> Self {
        Self { bits: 0xFF }
    }

    pub fn with(mut self, feature: u8) -> Self {
        self.bits |= feature;
        self
    }

    pub fn has(&self, feature: u8) -> bool {
        self.bits & feature != 0
    }

    pub fn bits(&self) -> u8 {
        self.bits
    }
}

/// Discovered device information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiscoveredDevice {
    /// Device ID
    pub device_id: Uuid,
    /// Device name
    pub name: String,
    /// Device type
    pub device_type: DeviceType,
    /// Vendor ID
    pub vendor_id: u16,
    /// Product ID
    pub product_id: u16,
    /// IP address
    pub ip_address: Option<IpAddr>,
    /// Port
    pub port: u16,
    /// Matter node ID
    pub node_id: Option<u64>,
    /// Accessibility features
    pub accessibility_features: AccessibilityFeatures,
    /// Protocol version
    pub protocol_version: u8,
    /// Additional TXT records
    pub txt_records: HashMap<String, String>,
    /// Discovery timestamp
    pub discovered_at: u64,
}

impl DiscoveredDevice {
    /// Create from TXT records
    pub fn from_txt_records(
        name: String,
        ip: Option<IpAddr>,
        port: u16,
        records: HashMap<String, String>,
    ) -> Option<Self> {
        let device_id = records.get("DI")
            .and_then(|s| Uuid::parse_str(s).ok())
            .unwrap_or_else(Uuid::new_v4);

        let device_type = records.get("DT")
            .and_then(|s| serde_json::from_str(s).ok())
            .unwrap_or(DeviceType::Other);

        let vendor_id = records.get("VD")
            .and_then(|s| u16::from_str_radix(s.trim_start_matches("0x"), 16).ok())
            .unwrap_or(0);

        let product_id = records.get("PD")
            .and_then(|s| u16::from_str_radix(s.trim_start_matches("0x"), 16).ok())
            .unwrap_or(0);

        let accessibility_bits = records.get("AC")
            .and_then(|s| s.parse::<u8>().ok())
            .unwrap_or(0);

        let protocol_version = records.get("V")
            .and_then(|s| s.parse::<u8>().ok())
            .unwrap_or(1);

        use std::time::{SystemTime, UNIX_EPOCH};

        Some(Self {
            device_id,
            name: records.get("DN").cloned().unwrap_or(name),
            device_type,
            vendor_id,
            product_id,
            ip_address: ip,
            port,
            node_id: None,
            accessibility_features: AccessibilityFeatures { bits: accessibility_bits },
            protocol_version,
            txt_records: records,
            discovered_at: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_secs(),
        })
    }

    /// Create TXT records for announcement
    pub fn to_txt_records(&self) -> HashMap<String, String> {
        let mut records = HashMap::new();
        records.insert("DI".to_string(), self.device_id.to_string());
        records.insert("DN".to_string(), self.name.clone());
        records.insert("DT".to_string(), serde_json::to_string(&self.device_type).unwrap_or_default());
        records.insert("VD".to_string(), format!("0x{:04X}", self.vendor_id));
        records.insert("PD".to_string(), format!("0x{:04X}", self.product_id));
        records.insert("AC".to_string(), self.accessibility_features.bits().to_string());
        records.insert("V".to_string(), self.protocol_version.to_string());
        records
    }
}

/// Device discovery state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiscoveryState {
    Idle,
    Scanning,
    Stopped,
}

/// Discovery event
#[derive(Debug, Clone)]
pub enum DiscoveryEvent {
    DeviceFound(DiscoveredDevice),
    DeviceLost(Uuid),
    ScanStarted,
    ScanStopped,
    Error(String),
}

/// Discovery service (simulated implementation)
pub struct DiscoveryService {
    /// Current state
    state: DiscoveryState,
    /// Discovered devices
    devices: HashMap<Uuid, DiscoveredDevice>,
    /// Event handlers
    event_handlers: Vec<Box<dyn Fn(DiscoveryEvent) + Send + Sync>>,
}

impl DiscoveryService {
    /// Create a new discovery service
    pub fn new() -> Self {
        Self {
            state: DiscoveryState::Idle,
            devices: HashMap::new(),
            event_handlers: Vec::new(),
        }
    }

    /// Start scanning for devices
    pub fn start_scan(&mut self) {
        self.state = DiscoveryState::Scanning;
        self.notify(DiscoveryEvent::ScanStarted);
    }

    /// Stop scanning
    pub fn stop_scan(&mut self) {
        self.state = DiscoveryState::Stopped;
        self.notify(DiscoveryEvent::ScanStopped);
    }

    /// Get current state
    pub fn state(&self) -> DiscoveryState {
        self.state
    }

    /// Get discovered devices
    pub fn devices(&self) -> Vec<&DiscoveredDevice> {
        self.devices.values().collect()
    }

    /// Get device by ID
    pub fn get_device(&self, device_id: Uuid) -> Option<&DiscoveredDevice> {
        self.devices.get(&device_id)
    }

    /// Manually add a device (for testing/simulation)
    pub fn add_device(&mut self, device: DiscoveredDevice) {
        let device_id = device.device_id;
        self.devices.insert(device_id, device.clone());
        self.notify(DiscoveryEvent::DeviceFound(device));
    }

    /// Remove a device
    pub fn remove_device(&mut self, device_id: Uuid) {
        if self.devices.remove(&device_id).is_some() {
            self.notify(DiscoveryEvent::DeviceLost(device_id));
        }
    }

    /// Register event handler
    pub fn on_event<F>(&mut self, handler: F)
    where
        F: Fn(DiscoveryEvent) + Send + Sync + 'static,
    {
        self.event_handlers.push(Box::new(handler));
    }

    /// Notify event handlers
    fn notify(&self, event: DiscoveryEvent) {
        for handler in &self.event_handlers {
            handler(event.clone());
        }
    }

    /// Clear all devices
    pub fn clear(&mut self) {
        let device_ids: Vec<_> = self.devices.keys().copied().collect();
        for device_id in device_ids {
            self.remove_device(device_id);
        }
    }
}

impl Default for DiscoveryService {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_accessibility_features() {
        let features = AccessibilityFeatures::new()
            .with(AccessibilityFeatures::VOICE_CONTROL)
            .with(AccessibilityFeatures::KOREAN_TTS);

        assert!(features.has(AccessibilityFeatures::VOICE_CONTROL));
        assert!(features.has(AccessibilityFeatures::KOREAN_TTS));
        assert!(!features.has(AccessibilityFeatures::BCI_SUPPORT));
    }

    #[test]
    fn test_discovered_device() {
        let mut records = HashMap::new();
        records.insert("DI".to_string(), Uuid::new_v4().to_string());
        records.insert("DN".to_string(), "Living Room Light".to_string());
        records.insert("DT".to_string(), "\"light\"".to_string());
        records.insert("VD".to_string(), "0x1234".to_string());
        records.insert("AC".to_string(), "131".to_string()); // Voice + Audio + Korean

        let device = DiscoveredDevice::from_txt_records(
            "test".to_string(),
            Some("192.168.1.100".parse().unwrap()),
            5540,
            records,
        ).unwrap();

        assert_eq!(device.vendor_id, 0x1234);
        assert!(device.accessibility_features.has(AccessibilityFeatures::VOICE_CONTROL));
        assert!(device.accessibility_features.has(AccessibilityFeatures::KOREAN_TTS));
    }

    #[test]
    fn test_discovery_service() {
        let mut service = DiscoveryService::new();

        let device = DiscoveredDevice {
            device_id: Uuid::new_v4(),
            name: "Test Device".to_string(),
            device_type: DeviceType::Light,
            vendor_id: 0,
            product_id: 0,
            ip_address: None,
            port: 5540,
            node_id: None,
            accessibility_features: AccessibilityFeatures::all(),
            protocol_version: 1,
            txt_records: HashMap::new(),
            discovered_at: 0,
        };

        service.add_device(device.clone());

        assert_eq!(service.devices().len(), 1);
        assert!(service.get_device(device.device_id).is_some());
    }
}
