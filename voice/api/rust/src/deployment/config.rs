//! Deployment configuration types

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Deployment environment
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Environment {
    Development,
    Staging,
    Production,
}

impl Default for Environment {
    fn default() -> Self {
        Self::Development
    }
}

/// Deployment configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeploymentConfig {
    /// Environment
    pub environment: Environment,

    /// Server configuration
    pub server: ServerConfig,

    /// Database configuration
    pub database: DatabaseConfig,

    /// Cache configuration
    pub cache: CacheConfig,

    /// Model configuration
    pub models: ModelConfig,

    /// Feature flags
    pub features: FeatureFlags,

    /// Rate limiting
    pub rate_limiting: RateLimitConfig,

    /// Logging configuration
    pub logging: LoggingConfig,
}

impl Default for DeploymentConfig {
    fn default() -> Self {
        Self {
            environment: Environment::default(),
            server: ServerConfig::default(),
            database: DatabaseConfig::default(),
            cache: CacheConfig::default(),
            models: ModelConfig::default(),
            features: FeatureFlags::default(),
            rate_limiting: RateLimitConfig::default(),
            logging: LoggingConfig::default(),
        }
    }
}

/// Server configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServerConfig {
    /// Host to bind to
    pub host: String,

    /// Port to listen on
    pub port: u16,

    /// Number of worker threads
    pub workers: usize,

    /// Request timeout in seconds
    pub timeout_seconds: u64,

    /// Maximum connections
    pub max_connections: usize,

    /// TLS configuration
    pub tls: Option<TlsConfig>,
}

impl Default for ServerConfig {
    fn default() -> Self {
        Self {
            host: "0.0.0.0".to_string(),
            port: 8080,
            workers: 4, // Default to 4 workers
            timeout_seconds: 30,
            max_connections: 10000,
            tls: None,
        }
    }
}

/// TLS configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TlsConfig {
    /// Path to certificate file
    pub cert_path: String,

    /// Path to private key file
    pub key_path: String,

    /// Minimum TLS version
    pub min_version: String,
}

/// Database configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DatabaseConfig {
    /// Database URL
    pub url: String,

    /// Connection pool size
    pub pool_size: u32,

    /// Connection timeout in seconds
    pub timeout_seconds: u64,

    /// Maximum lifetime of a connection
    pub max_lifetime_seconds: u64,
}

impl Default for DatabaseConfig {
    fn default() -> Self {
        Self {
            url: "postgres://localhost/voicesign".to_string(),
            pool_size: 20,
            timeout_seconds: 30,
            max_lifetime_seconds: 1800,
        }
    }
}

/// Cache configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CacheConfig {
    /// Redis URL
    pub url: String,

    /// Enable caching
    pub enabled: bool,

    /// Default TTL in seconds
    pub ttl_seconds: u64,

    /// Maximum cache entries
    pub max_entries: usize,
}

impl Default for CacheConfig {
    fn default() -> Self {
        Self {
            url: "redis://localhost:6379".to_string(),
            enabled: true,
            ttl_seconds: 3600,
            max_entries: 100000,
        }
    }
}

/// Model configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModelConfig {
    /// Path to models directory
    pub path: String,

    /// ASR model version
    pub asr_version: String,

    /// Translation model version
    pub translation_version: String,

    /// Enable GPU acceleration
    pub gpu_enabled: bool,
}

impl Default for ModelConfig {
    fn default() -> Self {
        Self {
            path: "/models".to_string(),
            asr_version: "whisper-v3".to_string(),
            translation_version: "1.0.0".to_string(),
            gpu_enabled: true,
        }
    }
}

/// Feature flags
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeatureFlags {
    /// Enable streaming mode
    pub streaming_enabled: bool,

    /// Enable batch mode
    pub batch_mode_enabled: bool,

    /// Enable emergency detection
    pub emergency_detection_enabled: bool,

    /// Enable content filtering
    pub content_filter_enabled: bool,

    /// Enable PII redaction
    pub pii_redaction_enabled: bool,

    /// Experimental features
    pub experimental: HashMap<String, bool>,
}

impl Default for FeatureFlags {
    fn default() -> Self {
        Self {
            streaming_enabled: true,
            batch_mode_enabled: true,
            emergency_detection_enabled: true,
            content_filter_enabled: true,
            pii_redaction_enabled: true,
            experimental: HashMap::new(),
        }
    }
}

/// Rate limiting configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RateLimitConfig {
    /// Enable rate limiting
    pub enabled: bool,

    /// Requests per minute
    pub requests_per_minute: u32,

    /// Burst size
    pub burst_size: u32,

    /// Rate limit by
    pub limit_by: RateLimitKey,
}

impl Default for RateLimitConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            requests_per_minute: 100,
            burst_size: 20,
            limit_by: RateLimitKey::ApiKey,
        }
    }
}

/// Rate limit key type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RateLimitKey {
    IpAddress,
    ApiKey,
    UserId,
}

/// Logging configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoggingConfig {
    /// Log level
    pub level: String,

    /// Log format
    pub format: LogFormat,

    /// Include request ID in logs
    pub include_request_id: bool,

    /// Output destination
    pub output: LogOutput,
}

impl Default for LoggingConfig {
    fn default() -> Self {
        Self {
            level: "info".to_string(),
            format: LogFormat::Json,
            include_request_id: true,
            output: LogOutput::Stdout,
        }
    }
}

/// Log format
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum LogFormat {
    Json,
    Text,
}

/// Log output destination
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum LogOutput {
    Stdout,
    Stderr,
    File { path: String },
}

impl DeploymentConfig {
    /// Load configuration from environment variables
    pub fn from_env() -> Result<Self, ConfigError> {
        let mut config = Self::default();

        // Environment
        if let Ok(env) = std::env::var("WIA_ENVIRONMENT") {
            config.environment = match env.to_lowercase().as_str() {
                "development" | "dev" => Environment::Development,
                "staging" | "stage" => Environment::Staging,
                "production" | "prod" => Environment::Production,
                _ => return Err(ConfigError::InvalidValue("WIA_ENVIRONMENT".to_string())),
            };
        }

        // Server
        if let Ok(host) = std::env::var("WIA_HOST") {
            config.server.host = host;
        }
        if let Ok(port) = std::env::var("WIA_PORT") {
            config.server.port = port.parse().map_err(|_| ConfigError::InvalidValue("WIA_PORT".to_string()))?;
        }
        if let Ok(workers) = std::env::var("WIA_WORKERS") {
            config.server.workers = workers.parse().map_err(|_| ConfigError::InvalidValue("WIA_WORKERS".to_string()))?;
        }

        // Database
        if let Ok(url) = std::env::var("WIA_DATABASE_URL") {
            config.database.url = url;
        }
        if let Ok(pool) = std::env::var("WIA_DATABASE_POOL_SIZE") {
            config.database.pool_size = pool.parse().map_err(|_| ConfigError::InvalidValue("WIA_DATABASE_POOL_SIZE".to_string()))?;
        }

        // Cache
        if let Ok(url) = std::env::var("WIA_REDIS_URL") {
            config.cache.url = url;
        }
        if let Ok(ttl) = std::env::var("WIA_CACHE_TTL_SECONDS") {
            config.cache.ttl_seconds = ttl.parse().map_err(|_| ConfigError::InvalidValue("WIA_CACHE_TTL_SECONDS".to_string()))?;
        }

        // Features
        if let Ok(val) = std::env::var("WIA_FEATURE_STREAMING") {
            config.features.streaming_enabled = val.parse().unwrap_or(true);
        }
        if let Ok(val) = std::env::var("WIA_FEATURE_EMERGENCY_PRIORITY") {
            config.features.emergency_detection_enabled = val.parse().unwrap_or(true);
        }

        // Rate limiting
        if let Ok(val) = std::env::var("WIA_RATE_LIMIT_REQUESTS_PER_MINUTE") {
            config.rate_limiting.requests_per_minute = val.parse().map_err(|_| ConfigError::InvalidValue("WIA_RATE_LIMIT_REQUESTS_PER_MINUTE".to_string()))?;
        }

        // Logging
        if let Ok(level) = std::env::var("WIA_LOG_LEVEL") {
            config.logging.level = level;
        }
        if let Ok(format) = std::env::var("WIA_LOG_FORMAT") {
            config.logging.format = match format.to_lowercase().as_str() {
                "json" => LogFormat::Json,
                "text" => LogFormat::Text,
                _ => LogFormat::Json,
            };
        }

        Ok(config)
    }

    /// Validate configuration
    pub fn validate(&self) -> Result<(), ConfigError> {
        // Validate server config
        if self.server.port == 0 {
            return Err(ConfigError::InvalidValue("server.port cannot be 0".to_string()));
        }
        if self.server.workers == 0 {
            return Err(ConfigError::InvalidValue("server.workers cannot be 0".to_string()));
        }

        // Validate database config
        if self.database.url.is_empty() {
            return Err(ConfigError::MissingValue("database.url".to_string()));
        }

        // Validate cache config
        if self.cache.enabled && self.cache.url.is_empty() {
            return Err(ConfigError::MissingValue("cache.url".to_string()));
        }

        Ok(())
    }
}

/// Configuration error
#[derive(Debug, thiserror::Error)]
pub enum ConfigError {
    #[error("Invalid configuration value: {0}")]
    InvalidValue(String),

    #[error("Missing required configuration: {0}")]
    MissingValue(String),

    #[error("Configuration file error: {0}")]
    FileError(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_config() {
        let config = DeploymentConfig::default();
        assert_eq!(config.environment, Environment::Development);
        assert_eq!(config.server.port, 8080);
        assert!(config.features.streaming_enabled);
    }

    #[test]
    fn test_config_validation() {
        let config = DeploymentConfig::default();
        assert!(config.validate().is_ok());

        let mut invalid_config = DeploymentConfig::default();
        invalid_config.server.port = 0;
        assert!(invalid_config.validate().is_err());
    }
}
