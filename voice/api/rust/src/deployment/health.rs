//! Health check implementation

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Overall health status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum HealthStatus {
    Healthy,
    Degraded,
    Unhealthy,
}

/// Component health status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComponentHealth {
    /// Component status
    pub status: HealthStatus,

    /// Latency in milliseconds
    pub latency_ms: Option<u64>,

    /// Last check timestamp
    pub last_check: DateTime<Utc>,

    /// Additional details
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<HashMap<String, String>>,
}

/// Health check result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealthCheckResult {
    /// Overall status
    pub status: HealthStatus,

    /// Service version
    pub version: String,

    /// Timestamp
    pub timestamp: DateTime<Utc>,

    /// Uptime in seconds
    pub uptime_seconds: u64,

    /// Component health
    pub components: HashMap<String, ComponentHealth>,

    /// System checks
    pub checks: HashMap<String, SystemCheck>,
}

/// System check result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemCheck {
    /// Check status
    pub status: CheckStatus,

    /// Check value
    #[serde(skip_serializing_if = "Option::is_none")]
    pub value: Option<f64>,

    /// Unit of measurement
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unit: Option<String>,
}

/// Check status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CheckStatus {
    Pass,
    Warn,
    Fail,
}

/// Liveness check result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LivenessResult {
    pub status: SimpleStatus,
}

/// Readiness check result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReadinessResult {
    pub status: ReadinessStatus,
    pub checks: HashMap<String, bool>,
}

/// Simple status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum SimpleStatus {
    Ok,
    Fail,
}

/// Readiness status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReadinessStatus {
    Ready,
    NotReady,
}

/// Health checker trait
#[async_trait::async_trait]
pub trait HealthChecker: Send + Sync {
    /// Check component health
    async fn check(&self) -> ComponentHealth;

    /// Component name
    fn name(&self) -> &str;
}

/// Health check manager
pub struct HealthCheckManager {
    start_time: Instant,
    version: String,
    checkers: Vec<Arc<dyn HealthChecker>>,
}

impl HealthCheckManager {
    /// Create a new health check manager
    pub fn new(version: String) -> Self {
        Self {
            start_time: Instant::now(),
            version,
            checkers: Vec::new(),
        }
    }

    /// Add a health checker
    pub fn add_checker(&mut self, checker: Arc<dyn HealthChecker>) {
        self.checkers.push(checker);
    }

    /// Perform full health check
    pub async fn check(&self) -> HealthCheckResult {
        let mut components = HashMap::new();
        let mut overall_status = HealthStatus::Healthy;

        // Check all components
        for checker in &self.checkers {
            let health = checker.check().await;

            // Update overall status
            match health.status {
                HealthStatus::Unhealthy => overall_status = HealthStatus::Unhealthy,
                HealthStatus::Degraded if overall_status != HealthStatus::Unhealthy => {
                    overall_status = HealthStatus::Degraded;
                }
                _ => {}
            }

            components.insert(checker.name().to_string(), health);
        }

        // System checks
        let checks = self.system_checks();

        // Update overall status based on system checks
        for check in checks.values() {
            if check.status == CheckStatus::Fail {
                overall_status = HealthStatus::Unhealthy;
                break;
            }
        }

        HealthCheckResult {
            status: overall_status,
            version: self.version.clone(),
            timestamp: Utc::now(),
            uptime_seconds: self.start_time.elapsed().as_secs(),
            components,
            checks,
        }
    }

    /// Liveness check
    pub fn liveness(&self) -> LivenessResult {
        LivenessResult {
            status: SimpleStatus::Ok,
        }
    }

    /// Readiness check
    pub async fn readiness(&self) -> ReadinessResult {
        let mut checks = HashMap::new();
        let mut ready = true;

        for checker in &self.checkers {
            let health = checker.check().await;
            let is_healthy = health.status != HealthStatus::Unhealthy;
            checks.insert(checker.name().to_string(), is_healthy);
            if !is_healthy {
                ready = false;
            }
        }

        ReadinessResult {
            status: if ready {
                ReadinessStatus::Ready
            } else {
                ReadinessStatus::NotReady
            },
            checks,
        }
    }

    /// System checks
    fn system_checks(&self) -> HashMap<String, SystemCheck> {
        let mut checks = HashMap::new();

        // Memory check
        checks.insert(
            "memory".to_string(),
            SystemCheck {
                status: CheckStatus::Pass,
                value: Some(self.get_memory_usage()),
                unit: Some("percent".to_string()),
            },
        );

        // CPU check
        checks.insert(
            "cpu".to_string(),
            SystemCheck {
                status: CheckStatus::Pass,
                value: Some(self.get_cpu_usage()),
                unit: Some("percent".to_string()),
            },
        );

        checks
    }

    fn get_memory_usage(&self) -> f64 {
        // Simplified - in production, use system metrics
        50.0
    }

    fn get_cpu_usage(&self) -> f64 {
        // Simplified - in production, use system metrics
        30.0
    }
}

/// Database health checker
pub struct DatabaseHealthChecker {
    name: String,
    connection_string: String,
}

impl DatabaseHealthChecker {
    pub fn new(connection_string: String) -> Self {
        Self {
            name: "database".to_string(),
            connection_string,
        }
    }
}

#[async_trait::async_trait]
impl HealthChecker for DatabaseHealthChecker {
    async fn check(&self) -> ComponentHealth {
        let start = Instant::now();

        // Simulate database check
        // In production, actually ping the database
        let is_healthy = !self.connection_string.is_empty();

        ComponentHealth {
            status: if is_healthy {
                HealthStatus::Healthy
            } else {
                HealthStatus::Unhealthy
            },
            latency_ms: Some(start.elapsed().as_millis() as u64),
            last_check: Utc::now(),
            details: None,
        }
    }

    fn name(&self) -> &str {
        &self.name
    }
}

/// Cache health checker
pub struct CacheHealthChecker {
    name: String,
    url: String,
}

impl CacheHealthChecker {
    pub fn new(url: String) -> Self {
        Self {
            name: "redis".to_string(),
            url,
        }
    }
}

#[async_trait::async_trait]
impl HealthChecker for CacheHealthChecker {
    async fn check(&self) -> ComponentHealth {
        let start = Instant::now();

        // Simulate cache check
        let is_healthy = !self.url.is_empty();

        ComponentHealth {
            status: if is_healthy {
                HealthStatus::Healthy
            } else {
                HealthStatus::Unhealthy
            },
            latency_ms: Some(start.elapsed().as_millis() as u64),
            last_check: Utc::now(),
            details: None,
        }
    }

    fn name(&self) -> &str {
        &self.name
    }
}

/// Model health checker
pub struct ModelHealthChecker {
    name: String,
    model_version: String,
    gpu_available: bool,
}

impl ModelHealthChecker {
    pub fn new(name: String, model_version: String, gpu_available: bool) -> Self {
        Self {
            name,
            model_version,
            gpu_available,
        }
    }
}

#[async_trait::async_trait]
impl HealthChecker for ModelHealthChecker {
    async fn check(&self) -> ComponentHealth {
        let mut details = HashMap::new();
        details.insert("model_version".to_string(), self.model_version.clone());
        details.insert("gpu_available".to_string(), self.gpu_available.to_string());

        ComponentHealth {
            status: HealthStatus::Healthy,
            latency_ms: None,
            last_check: Utc::now(),
            details: Some(details),
        }
    }

    fn name(&self) -> &str {
        &self.name
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_health_check_manager() {
        let mut manager = HealthCheckManager::new("1.0.0".to_string());

        let db_checker = Arc::new(DatabaseHealthChecker::new("postgres://localhost".to_string()));
        manager.add_checker(db_checker);

        let result = manager.check().await;
        assert_eq!(result.status, HealthStatus::Healthy);
        assert!(result.components.contains_key("database"));
    }

    #[test]
    fn test_liveness() {
        let manager = HealthCheckManager::new("1.0.0".to_string());
        let result = manager.liveness();
        assert_eq!(result.status, SimpleStatus::Ok);
    }

    #[tokio::test]
    async fn test_readiness() {
        let manager = HealthCheckManager::new("1.0.0".to_string());
        let result = manager.readiness().await;
        assert_eq!(result.status, ReadinessStatus::Ready);
    }
}
