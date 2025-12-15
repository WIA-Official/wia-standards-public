//! WIA Eye Gaze Standard - Service Discovery
//!
//! mDNS/Bonjour-based service discovery for automatic eye tracker detection.
//!
//! 弘益人間 - 널리 인간을 이롭게

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::net::{IpAddr, SocketAddr};
use std::sync::Arc;
use std::time::Duration;

/// mDNS service type for WIA Eye Gaze
pub const SERVICE_TYPE: &str = "_wia-eye-gaze._tcp";

/// Default discovery timeout
pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(5);

/// Discovered eye tracker service
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EyeTrackerService {
    /// Service name
    pub name: String,
    /// Host address
    pub host: String,
    /// Port number
    pub port: u16,
    /// IP addresses
    pub addresses: Vec<IpAddr>,
    /// Service metadata
    pub metadata: ServiceMetadata,
}

impl EyeTrackerService {
    /// Get WebSocket URL for this service
    pub fn websocket_url(&self) -> String {
        let addr = self.addresses.first().map(|a| a.to_string()).unwrap_or_else(|| self.host.clone());
        format!("ws://{}:{}/wia-eye-gaze/v1/stream", addr, self.port)
    }

    /// Get socket address
    pub fn socket_addr(&self) -> Option<SocketAddr> {
        self.addresses.first().map(|addr| SocketAddr::new(*addr, self.port))
    }
}

/// Service metadata from TXT records
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ServiceMetadata {
    /// Protocol version
    pub version: String,
    /// Device vendor
    pub device_vendor: String,
    /// Device model
    pub device_model: String,
    /// Capabilities (JSON encoded)
    pub capabilities: String,
    /// Additional TXT records
    #[serde(flatten)]
    pub extra: HashMap<String, String>,
}

impl ServiceMetadata {
    /// Parse from TXT records
    pub fn from_txt_records(records: &HashMap<String, String>) -> Self {
        Self {
            version: records.get("version").cloned().unwrap_or_default(),
            device_vendor: records.get("deviceVendor").cloned().unwrap_or_default(),
            device_model: records.get("deviceModel").cloned().unwrap_or_default(),
            capabilities: records.get("capabilities").cloned().unwrap_or_default(),
            extra: records.iter()
                .filter(|(k, _)| !["version", "deviceVendor", "deviceModel", "capabilities"].contains(&k.as_str()))
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect(),
        }
    }

    /// Convert to TXT records
    pub fn to_txt_records(&self) -> HashMap<String, String> {
        let mut records = HashMap::new();
        if !self.version.is_empty() {
            records.insert("version".to_string(), self.version.clone());
        }
        if !self.device_vendor.is_empty() {
            records.insert("deviceVendor".to_string(), self.device_vendor.clone());
        }
        if !self.device_model.is_empty() {
            records.insert("deviceModel".to_string(), self.device_model.clone());
        }
        if !self.capabilities.is_empty() {
            records.insert("capabilities".to_string(), self.capabilities.clone());
        }
        records.extend(self.extra.clone());
        records
    }
}

/// Service discovery configuration
#[derive(Debug, Clone)]
pub struct DiscoveryConfig {
    /// Discovery timeout
    pub timeout: Duration,
    /// Service type to discover
    pub service_type: String,
}

impl Default for DiscoveryConfig {
    fn default() -> Self {
        Self {
            timeout: DEFAULT_TIMEOUT,
            service_type: SERVICE_TYPE.to_string(),
        }
    }
}

/// Service advertiser configuration
#[derive(Debug, Clone)]
pub struct AdvertiseConfig {
    /// Service name
    pub name: String,
    /// Port to advertise
    pub port: u16,
    /// Device vendor
    pub device_vendor: String,
    /// Device model
    pub device_model: String,
    /// Additional metadata
    pub metadata: HashMap<String, String>,
}

impl AdvertiseConfig {
    /// Create new advertise config
    pub fn new(name: &str, port: u16) -> Self {
        Self {
            name: name.to_string(),
            port,
            device_vendor: "WIA".to_string(),
            device_model: "Eye Tracker".to_string(),
            metadata: HashMap::new(),
        }
    }

    /// Set device info
    pub fn with_device(mut self, vendor: &str, model: &str) -> Self {
        self.device_vendor = vendor.to_string();
        self.device_model = model.to_string();
        self
    }

    /// Add metadata
    pub fn with_metadata(mut self, key: &str, value: &str) -> Self {
        self.metadata.insert(key.to_string(), value.to_string());
        self
    }
}

/// Service discovery errors
#[derive(Debug, Clone)]
pub enum DiscoveryError {
    /// mDNS initialization failed
    InitFailed(String),
    /// Discovery timed out
    Timeout,
    /// Service registration failed
    RegistrationFailed(String),
    /// Service not found
    NotFound,
}

impl std::fmt::Display for DiscoveryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DiscoveryError::InitFailed(e) => write!(f, "mDNS initialization failed: {}", e),
            DiscoveryError::Timeout => write!(f, "Discovery timed out"),
            DiscoveryError::RegistrationFailed(e) => write!(f, "Service registration failed: {}", e),
            DiscoveryError::NotFound => write!(f, "Service not found"),
        }
    }
}

impl std::error::Error for DiscoveryError {}

/// Service discovery (platform-independent interface)
pub struct ServiceDiscovery {
    config: DiscoveryConfig,
}

impl ServiceDiscovery {
    /// Create new service discovery
    pub fn new() -> Self {
        Self::with_config(DiscoveryConfig::default())
    }

    /// Create with config
    pub fn with_config(config: DiscoveryConfig) -> Self {
        Self { config }
    }

    /// Discover eye tracker services on the network
    #[cfg(feature = "discovery")]
    pub async fn discover(&self) -> Result<Vec<EyeTrackerService>, DiscoveryError> {
        use mdns_sd::{ServiceDaemon, ServiceEvent};
        use tokio::time::timeout;

        let mdns = ServiceDaemon::new()
            .map_err(|e| DiscoveryError::InitFailed(e.to_string()))?;

        let service_type = format!("{}.local.", self.config.service_type);
        let receiver = mdns.browse(&service_type)
            .map_err(|e| DiscoveryError::InitFailed(e.to_string()))?;

        let mut services = Vec::new();
        let deadline = tokio::time::Instant::now() + self.config.timeout;

        loop {
            let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
            if remaining.is_zero() {
                break;
            }

            match timeout(remaining, async {
                receiver.recv_async().await
            }).await {
                Ok(Ok(event)) => {
                    if let ServiceEvent::ServiceResolved(info) = event {
                        let txt_records: HashMap<String, String> = info.get_properties()
                            .iter()
                            .filter_map(|p| {
                                let key = p.key().to_string();
                                p.val_str().map(|v| (key, v.to_string()))
                            })
                            .collect();

                        let service = EyeTrackerService {
                            name: info.get_fullname().to_string(),
                            host: info.get_hostname().to_string(),
                            port: info.get_port(),
                            addresses: info.get_addresses().iter().cloned().collect(),
                            metadata: ServiceMetadata::from_txt_records(&txt_records),
                        };
                        services.push(service);
                    }
                }
                Ok(Err(_)) => break,
                Err(_) => break,
            }
        }

        Ok(services)
    }

    /// Discover without async (blocking)
    #[cfg(not(feature = "discovery"))]
    pub fn discover(&self) -> Result<Vec<EyeTrackerService>, DiscoveryError> {
        // Stub implementation when discovery feature is disabled
        Ok(Vec::new())
    }

    /// Discover first available service
    #[cfg(feature = "discovery")]
    pub async fn discover_first(&self) -> Result<EyeTrackerService, DiscoveryError> {
        let services = self.discover().await?;
        services.into_iter().next().ok_or(DiscoveryError::NotFound)
    }
}

impl Default for ServiceDiscovery {
    fn default() -> Self {
        Self::new()
    }
}

/// Service advertiser for eye tracker servers
pub struct ServiceAdvertiser {
    config: AdvertiseConfig,
    #[cfg(feature = "discovery")]
    daemon: Option<mdns_sd::ServiceDaemon>,
}

impl ServiceAdvertiser {
    /// Create new service advertiser
    pub fn new(config: AdvertiseConfig) -> Self {
        Self {
            config,
            #[cfg(feature = "discovery")]
            daemon: None,
        }
    }

    /// Start advertising the service
    #[cfg(feature = "discovery")]
    pub fn start(&mut self) -> Result<(), DiscoveryError> {
        use mdns_sd::{ServiceDaemon, ServiceInfo};

        let daemon = ServiceDaemon::new()
            .map_err(|e| DiscoveryError::InitFailed(e.to_string()))?;

        let service_type = format!("{}.local.", SERVICE_TYPE);

        let mut properties = vec![
            ("version".to_string(), "1.0.0".to_string()),
            ("deviceVendor".to_string(), self.config.device_vendor.clone()),
            ("deviceModel".to_string(), self.config.device_model.clone()),
        ];

        for (k, v) in &self.config.metadata {
            properties.push((k.clone(), v.clone()));
        }

        let service_info = ServiceInfo::new(
            &service_type,
            &self.config.name,
            &format!("{}.local.", gethostname()),
            "",
            self.config.port,
            properties.iter()
                .map(|(k, v)| (k.as_str(), v.as_str()))
                .collect::<HashMap<_, _>>()
                .as_slice(),
        ).map_err(|e| DiscoveryError::RegistrationFailed(e.to_string()))?;

        daemon.register(service_info)
            .map_err(|e| DiscoveryError::RegistrationFailed(e.to_string()))?;

        self.daemon = Some(daemon);
        Ok(())
    }

    /// Stop advertising
    #[cfg(feature = "discovery")]
    pub fn stop(&mut self) {
        self.daemon = None;
    }

    /// Stub implementations when discovery feature is disabled
    #[cfg(not(feature = "discovery"))]
    pub fn start(&mut self) -> Result<(), DiscoveryError> {
        Ok(())
    }

    #[cfg(not(feature = "discovery"))]
    pub fn stop(&mut self) {}
}

/// Get hostname
fn gethostname() -> String {
    hostname::get()
        .map(|h| h.to_string_lossy().into_owned())
        .unwrap_or_else(|_| "localhost".to_string())
}

/// Callback for discovered services
pub type DiscoveryCallback = Box<dyn Fn(EyeTrackerService) + Send + Sync>;

/// Continuous service discovery listener
pub struct DiscoveryListener {
    config: DiscoveryConfig,
    callbacks: Vec<DiscoveryCallback>,
}

impl DiscoveryListener {
    /// Create new discovery listener
    pub fn new() -> Self {
        Self {
            config: DiscoveryConfig::default(),
            callbacks: Vec::new(),
        }
    }

    /// Register callback for discovered services
    pub fn on_discovered<F>(&mut self, callback: F)
    where
        F: Fn(EyeTrackerService) + Send + Sync + 'static,
    {
        self.callbacks.push(Box::new(callback));
    }

    /// Start listening (runs until stopped)
    #[cfg(feature = "discovery")]
    pub async fn start(&self) -> Result<(), DiscoveryError> {
        use mdns_sd::{ServiceDaemon, ServiceEvent};

        let mdns = ServiceDaemon::new()
            .map_err(|e| DiscoveryError::InitFailed(e.to_string()))?;

        let service_type = format!("{}.local.", self.config.service_type);
        let receiver = mdns.browse(&service_type)
            .map_err(|e| DiscoveryError::InitFailed(e.to_string()))?;

        loop {
            match receiver.recv_async().await {
                Ok(event) => {
                    if let ServiceEvent::ServiceResolved(info) = event {
                        let txt_records: HashMap<String, String> = info.get_properties()
                            .iter()
                            .filter_map(|p| {
                                let key = p.key().to_string();
                                p.val_str().map(|v| (key, v.to_string()))
                            })
                            .collect();

                        let service = EyeTrackerService {
                            name: info.get_fullname().to_string(),
                            host: info.get_hostname().to_string(),
                            port: info.get_port(),
                            addresses: info.get_addresses().iter().cloned().collect(),
                            metadata: ServiceMetadata::from_txt_records(&txt_records),
                        };

                        for callback in &self.callbacks {
                            callback(service.clone());
                        }
                    }
                }
                Err(_) => break,
            }
        }

        Ok(())
    }
}

impl Default for DiscoveryListener {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_service_metadata() {
        let mut records = HashMap::new();
        records.insert("version".to_string(), "1.0.0".to_string());
        records.insert("deviceVendor".to_string(), "Tobii".to_string());
        records.insert("deviceModel".to_string(), "Eye Tracker 5".to_string());

        let metadata = ServiceMetadata::from_txt_records(&records);
        assert_eq!(metadata.version, "1.0.0");
        assert_eq!(metadata.device_vendor, "Tobii");
        assert_eq!(metadata.device_model, "Eye Tracker 5");
    }

    #[test]
    fn test_websocket_url() {
        let service = EyeTrackerService {
            name: "test".to_string(),
            host: "localhost".to_string(),
            port: 8765,
            addresses: vec!["127.0.0.1".parse().unwrap()],
            metadata: ServiceMetadata::default(),
        };

        assert_eq!(
            service.websocket_url(),
            "ws://127.0.0.1:8765/wia-eye-gaze/v1/stream"
        );
    }

    #[test]
    fn test_advertise_config() {
        let config = AdvertiseConfig::new("MyTracker", 8765)
            .with_device("Tobii", "Eye Tracker 5")
            .with_metadata("feature", "3d_gaze");

        assert_eq!(config.name, "MyTracker");
        assert_eq!(config.port, 8765);
        assert_eq!(config.device_vendor, "Tobii");
        assert_eq!(config.metadata.get("feature"), Some(&"3d_gaze".to_string()));
    }
}
