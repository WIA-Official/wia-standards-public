//! Device and service registry for discovery

use crate::error::{NanoError, NanoResult};
use crate::types::{NanoSystemType, Position3D};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;

/// Device registry for nano system discovery
pub struct DeviceRegistry {
    devices: Arc<RwLock<HashMap<String, DeviceEntry>>>,
    services: Arc<RwLock<HashMap<String, ServiceEntry>>>,
    config: RegistryConfig,
}

impl DeviceRegistry {
    /// Create a new registry
    pub fn new(config: RegistryConfig) -> Self {
        Self {
            devices: Arc::new(RwLock::new(HashMap::new())),
            services: Arc::new(RwLock::new(HashMap::new())),
            config,
        }
    }

    /// Create with default config
    pub fn default_registry() -> Self {
        Self::new(RegistryConfig::default())
    }

    /// Register a device
    pub async fn register_device(&self, entry: DeviceEntry) -> NanoResult<String> {
        let mut devices = self.devices.write().await;

        if devices.len() >= self.config.max_devices {
            return Err(NanoError::InvalidParameter("Registry capacity exceeded".into()));
        }

        let id = entry.id.clone();
        devices.insert(id.clone(), entry);
        Ok(id)
    }

    /// Unregister a device
    pub async fn unregister_device(&self, device_id: &str) -> NanoResult<()> {
        let mut devices = self.devices.write().await;
        devices.remove(device_id);
        Ok(())
    }

    /// Get device by ID
    pub async fn get_device(&self, device_id: &str) -> Option<DeviceEntry> {
        let devices = self.devices.read().await;
        devices.get(device_id).cloned()
    }

    /// Find devices by type
    pub async fn find_by_type(&self, system_type: NanoSystemType) -> Vec<DeviceEntry> {
        let devices = self.devices.read().await;
        devices
            .values()
            .filter(|d| d.system_type == system_type)
            .cloned()
            .collect()
    }

    /// Find devices near a position
    pub async fn find_nearby(&self, position: Position3D, radius_nm: f64) -> Vec<DeviceEntry> {
        let devices = self.devices.read().await;
        devices
            .values()
            .filter(|d| {
                d.position
                    .map(|p| p.distance_to(&position) <= radius_nm)
                    .unwrap_or(false)
            })
            .cloned()
            .collect()
    }

    /// Find devices by capability
    pub async fn find_by_capability(&self, capability: &str) -> Vec<DeviceEntry> {
        let devices = self.devices.read().await;
        devices
            .values()
            .filter(|d| d.capabilities.iter().any(|c| c == capability))
            .cloned()
            .collect()
    }

    /// Query devices with custom filter
    pub async fn query(&self, query: DeviceQuery) -> Vec<DeviceEntry> {
        let devices = self.devices.read().await;

        devices
            .values()
            .filter(|d| {
                // Type filter
                if let Some(ref sys_type) = query.system_type {
                    if d.system_type != *sys_type {
                        return false;
                    }
                }

                // Status filter
                if let Some(ref status) = query.status {
                    if d.status != *status {
                        return false;
                    }
                }

                // Capability filter
                if let Some(ref caps) = query.capabilities {
                    for cap in caps {
                        if !d.capabilities.contains(cap) {
                            return false;
                        }
                    }
                }

                // Tag filter
                if let Some(ref tags) = query.tags {
                    for tag in tags {
                        if !d.tags.contains(tag) {
                            return false;
                        }
                    }
                }

                true
            })
            .take(query.limit.unwrap_or(100))
            .cloned()
            .collect()
    }

    /// Update device heartbeat
    pub async fn heartbeat(&self, device_id: &str) -> NanoResult<()> {
        let mut devices = self.devices.write().await;
        if let Some(device) = devices.get_mut(device_id) {
            device.last_seen = chrono::Utc::now().timestamp() as u64;
            device.status = DeviceStatus::Online;
            Ok(())
        } else {
            Err(NanoError::NotFound(format!("Device {} not found", device_id)))
        }
    }

    /// Register a service
    pub async fn register_service(&self, entry: ServiceEntry) -> NanoResult<String> {
        let mut services = self.services.write().await;
        let id = entry.id.clone();
        services.insert(id.clone(), entry);
        Ok(id)
    }

    /// Find services by type
    pub async fn find_services(&self, service_type: &str) -> Vec<ServiceEntry> {
        let services = self.services.read().await;
        services
            .values()
            .filter(|s| s.service_type == service_type)
            .cloned()
            .collect()
    }

    /// Get device count
    pub async fn device_count(&self) -> usize {
        self.devices.read().await.len()
    }

    /// Get all device IDs
    pub async fn list_devices(&self) -> Vec<String> {
        self.devices.read().await.keys().cloned().collect()
    }

    /// Clean up stale entries
    pub async fn cleanup_stale(&self) {
        let now = chrono::Utc::now().timestamp() as u64;
        let timeout = self.config.device_timeout_s;

        let mut devices = self.devices.write().await;
        devices.retain(|_, d| now - d.last_seen < timeout);
    }
}

/// Registry configuration
#[derive(Debug, Clone)]
pub struct RegistryConfig {
    pub max_devices: usize,
    pub device_timeout_s: u64,
    pub cleanup_interval_s: u64,
}

impl Default for RegistryConfig {
    fn default() -> Self {
        Self {
            max_devices: 10000,
            device_timeout_s: 300,
            cleanup_interval_s: 60,
        }
    }
}

/// Device registry entry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceEntry {
    pub id: String,
    pub system_type: NanoSystemType,
    pub name: Option<String>,
    pub status: DeviceStatus,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub position: Option<Position3D>,
    pub capabilities: Vec<String>,
    pub tags: Vec<String>,
    pub last_seen: u64,
    pub registered_at: u64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<serde_json::Value>,
}

impl DeviceEntry {
    pub fn new(id: impl Into<String>, system_type: NanoSystemType) -> Self {
        let now = chrono::Utc::now().timestamp() as u64;
        Self {
            id: id.into(),
            system_type,
            name: None,
            status: DeviceStatus::Online,
            position: None,
            capabilities: Vec::new(),
            tags: Vec::new(),
            last_seen: now,
            registered_at: now,
            metadata: None,
        }
    }

    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    pub fn with_position(mut self, position: Position3D) -> Self {
        self.position = Some(position);
        self
    }

    pub fn with_capabilities(mut self, capabilities: Vec<String>) -> Self {
        self.capabilities = capabilities;
        self
    }

    pub fn with_tags(mut self, tags: Vec<String>) -> Self {
        self.tags = tags;
        self
    }
}

/// Device status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeviceStatus {
    Online,
    Offline,
    Busy,
    Error,
    Maintenance,
}

/// Service registry entry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServiceEntry {
    pub id: String,
    pub service_type: String,
    pub name: String,
    pub endpoint: String,
    pub version: String,
    pub status: ServiceStatus,
    pub capabilities: Vec<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metadata: Option<serde_json::Value>,
}

/// Service status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ServiceStatus {
    Available,
    Unavailable,
    Degraded,
}

/// Device query
#[derive(Debug, Clone, Default)]
pub struct DeviceQuery {
    pub system_type: Option<NanoSystemType>,
    pub status: Option<DeviceStatus>,
    pub capabilities: Option<Vec<String>>,
    pub tags: Option<Vec<String>>,
    pub limit: Option<usize>,
}

impl DeviceQuery {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_type(mut self, system_type: NanoSystemType) -> Self {
        self.system_type = Some(system_type);
        self
    }

    pub fn with_status(mut self, status: DeviceStatus) -> Self {
        self.status = Some(status);
        self
    }

    pub fn with_capability(mut self, capability: impl Into<String>) -> Self {
        self.capabilities
            .get_or_insert_with(Vec::new)
            .push(capability.into());
        self
    }

    pub fn with_tag(mut self, tag: impl Into<String>) -> Self {
        self.tags.get_or_insert_with(Vec::new).push(tag.into());
        self
    }

    pub fn with_limit(mut self, limit: usize) -> Self {
        self.limit = Some(limit);
        self
    }
}
